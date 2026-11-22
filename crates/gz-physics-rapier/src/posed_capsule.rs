// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A single capsule keeps its collider frame separate from the link and inertia.
use super::*;

pub(super) fn spawn(
    backend: &mut RapierBackend,
    spec: PosedCapsuleBodySpec,
    rotational: RotationSpec,
    inertia: Option<Inertia>,
) -> Result<BodyId, PhysicsError> {
    if !finite(spec.position)
        || !finite(spec.collision_position)
        || !finite(spec.velocity)
        || spec
            .mass
            .is_some_and(|mass| !mass.is_finite() || mass <= 0.0)
        || !finite(rotational.angular_velocity)
        || (spec.mass.is_none()
            && (spec.velocity != [0.0; 3] || rotational.angular_velocity != [0.0; 3]))
    {
        return Err(PhysicsError::InvalidValue);
    }
    let orientation = rotation(rotational.orientation)?;
    let collision_orientation = rotation(spec.collision_orientation)?;
    let mut shape = collider(Shape::Capsule {
        radius: spec.radius,
        half_length: spec.half_length,
    })?
    .mass(spec.mass.unwrap_or(1.0))
    .build();
    // collider() uses capsule_z: its segment is already on the Gazebo Z axis.
    // Apply only the caller's collision frame, preserving the link/inertia frame.
    let local = Pose::from_parts(Vector::from(spec.collision_position), collision_orientation);
    let world = Pose::from_parts(Vector::from(spec.position), orientation) * local;
    if !finite(world.translation.to_array()) || !world.rotation.is_finite() {
        return Err(PhysicsError::InvalidValue);
    }
    shape.set_position(local);
    let geometric_mass = shape.mass_properties();
    let positive = |value: f64| value.is_normal() && value > 0.0;
    if !positive(geometric_mass.inv_mass)
        || !geometric_mass
            .inv_principal_inertia
            .to_array()
            .into_iter()
            .all(positive)
    {
        return Err(PhysicsError::InvalidValue);
    }
    // Local bounds must be representable independently: a world translation
    // cannot rescue an overflowing local extent by cancellation.
    for pose in [local, world] {
        let bounds = shape.shape().compute_aabb(&pose);
        if !finite(bounds.mins.to_array())
            || !finite(bounds.maxs.to_array())
            || !(bounds.maxs - bounds.mins)
                .to_array()
                .into_iter()
                .all(positive)
        {
            return Err(PhysicsError::InvalidValue);
        }
    }
    if spec.mass.is_some() && inertia.is_none() {
        return Err(PhysicsError::Unsupported {
            engine: "rapier",
            capability: "dynamic posed capsule without explicit inertia",
        });
    }
    let properties = inertia
        .map(|value| {
            if !finite(value.center) {
                return Err(PhysicsError::InvalidValue);
            }
            if value.center != [0.0; 3] {
                // Only the newly admitted offset domain needs these extra bounds.
                // The existing `world` is the COLLIDER pose, not the body pose.
                let center = Vector::from(value.center);
                let body_pose = Pose::from_parts(Vector::from(spec.position), orientation);
                let world_center = body_pose * center;
                let sphere = shape
                    .shape()
                    .compute_local_bounding_sphere()
                    .transform_by(shape.position());
                let extent = (sphere.center - center).length() + sphere.radius;
                if !center.length_squared().is_finite()
                    || !finite(world_center.to_array())
                    || !world_center.length_squared().is_finite()
                    || !extent.is_finite()
                {
                    return Err(PhysicsError::InvalidValue);
                }
            }
            inertia_properties(value, spec.mass.unwrap_or(1.0))
        })
        .transpose()?;
    let next = backend
        .next_id
        .checked_add(1)
        .ok_or(PhysicsError::IdOverflow)?;
    let id = BodyId::from_raw(backend.next_id);
    let construction = Construction {
        position: spec.position,
        velocity: spec.velocity,
        mass: spec.mass,
        collider: shape,
    };
    // insert() zeros collider density for explicit inertia, so local collision
    // translation never contributes an inferred COM or parallel-axis tensor.
    let handle = insert(
        &mut backend.world,
        &construction,
        rotational,
        construction.collider.clone(),
        properties,
    );
    backend
        .bodies
        .insert(id, (handle, construction, rotational, inertia, None));
    backend.next_id = next;
    Ok(id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exhausted_posed_capsule_ids_leave_native_and_public_state_unchanged() {
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let existing = backend
            .spawn(BodySpec {
                shape: Shape::Sphere { radius: 0.25 },
                position: [-20.0, 0.0, 0.0],
                velocity: [0.0; 3],
                mass: Some(2.0),
            })
            .unwrap();
        backend
            .add_world_wrench(existing, [2.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
        let before = backend.state(existing);
        backend.next_id = u64::MAX;
        let spec = PosedCapsuleBodySpec {
            radius: 0.5,
            half_length: 1.25,
            collision_position: [1.0, -2.0, 3.0],
            collision_orientation: [0.0, 0.0, 0.0, 1.0],
            position: [0.0; 3],
            velocity: [0.0; 3],
            mass: None,
        };
        assert_eq!(
            backend.spawn_posed_capsule_link(spec, RotationSpec::default(), None),
            Err(PhysicsError::IdOverflow)
        );
        assert_eq!(backend.bodies.len(), 1);
        assert_eq!(backend.world.bodies.len(), 1);
        assert_eq!(backend.world.colliders.len(), 1);
        assert_eq!(backend.pending_wrenches, BTreeSet::from([existing]));
        assert_eq!(backend.state(existing), before);
        assert_eq!(backend.next_id, u64::MAX);
    }
}
