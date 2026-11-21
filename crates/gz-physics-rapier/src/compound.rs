// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A bounded flat primitive union retains one link body and explicit mass frame.
use super::*;
use rapier3d_f64::parry::bounding_volume::{Aabb, BoundingVolume};

fn bounds_representable(bounds: &Aabb) -> bool {
    bounds.mins.is_finite()
        && bounds.maxs.is_finite()
        && bounds.center().is_finite()
        && (bounds.maxs - bounds.mins)
            .to_array()
            .into_iter()
            .all(|value| value.is_normal() && value > 0.0)
}

fn mass_representable(properties: MassProperties) -> bool {
    properties.local_com.is_finite()
        && properties.inv_mass.is_normal()
        && properties.inv_mass > 0.0
        && properties
            .inv_principal_inertia
            .to_array()
            .into_iter()
            .all(|value| value.is_normal() && value > 0.0)
}

pub(super) fn spawn(
    backend: &mut RapierBackend,
    spec: CompoundBodySpec<'_>,
    rotational: RotationSpec,
    inertia: Option<Inertia>,
) -> Result<BodyId, PhysicsError> {
    if !(2..=16).contains(&spec.collisions.len())
        || !finite(spec.position)
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
    let world = Pose::from_parts(
        Vector::from(spec.position),
        rotation(rotational.orientation)?,
    );
    let mut parts = Vec::with_capacity(spec.collisions.len());
    let mut centers: Vec<Vector> = Vec::with_capacity(spec.collisions.len());
    let mut local_union = Aabb::new_invalid();
    let mut world_union = Aabb::new_invalid();
    for collision in spec.collisions {
        if !finite(collision.position) {
            return Err(PhysicsError::InvalidValue);
        }
        let shape = collider(collision.shape)?
            .mass(spec.mass.unwrap_or(1.0))
            .build();
        let caller = Pose::from_parts(
            Vector::from(collision.position),
            rotation(collision.orientation)?,
        );
        // Only the existing Cylinder constructor contributes an axis correction.
        // A child's own rotation never rotates its local translation again.
        let local = caller * *shape.position();
        let child_world = world * local;
        if !local.translation.is_finite()
            || !local.rotation.is_finite()
            || !child_world.translation.is_finite()
            || !child_world.rotation.is_finite()
            || !mass_representable(shape.mass_properties())
        {
            return Err(PhysicsError::InvalidValue);
        }
        let local_bounds = shape.shape().compute_aabb(&local);
        let world_bounds = shape.shape().compute_aabb(&child_world);
        if !bounds_representable(&local_bounds) || !bounds_representable(&world_bounds) {
            return Err(PhysicsError::InvalidValue);
        }
        local_union.merge(&local_bounds);
        world_union.merge(&world_bounds);
        centers.push(local_bounds.center());
        parts.push((local, shape.shared_shape().clone()));
    }
    // Check the union before Parry builds its compound BVH. Separately finite
    // children do not imply a representable aggregate extent or mass tensor.
    if !bounds_representable(&local_union)
        || !bounds_representable(&world_union)
        || !local_union.volume().is_normal()
        || local_union.volume() <= 0.0
        || !(local_union.volume() * spec.collisions.len() as f64).is_finite()
    {
        return Err(PhysicsError::InvalidValue);
    }
    // Parry's binned builder divides by centroid ranges. Check every possible
    // nonzero subset range; coincident centers use its explicit fallback split.
    for (i, a) in centers.iter().enumerate() {
        for b in &centers[i + 1..] {
            for delta in (*a - *b).to_array() {
                if !delta.is_finite()
                    || (delta != 0.0 && !(8.0 * (1.0 - 1.0e-5) / delta).is_finite())
                {
                    return Err(PhysicsError::InvalidValue);
                }
            }
        }
    }
    // Guard the maintained sum's weighted center and shifted tensor before its
    // eigen decomposition. These bounds do not supply the body's physical inertia.
    let span_squared = (local_union.maxs - local_union.mins).length_squared();
    let mut total_mass = 0.0;
    let mut weighted_center_bound = Vector::ZERO;
    let mut tensor_bound = 0.0;
    for (pose, shape) in &parts {
        let properties = shape.mass_properties(1.0).transform_by(pose);
        if !mass_representable(properties) || !span_squared.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        let mass = properties.mass();
        total_mass += mass;
        weighted_center_bound += properties.local_com.abs() * mass;
        let matrix = properties.reconstruct_inertia_matrix();
        tensor_bound += matrix
            .to_cols_array()
            .into_iter()
            .map(f64::abs)
            .sum::<f64>()
            + 2.0 * span_squared * mass;
        if !total_mass.is_finite()
            || !weighted_center_bound.is_finite()
            || !(16.0 * tensor_bound * tensor_bound).is_finite()
        {
            return Err(PhysicsError::InvalidValue);
        }
    }
    let mut geometric = MassProperties::from_compound(1.0, &parts);
    geometric.set_mass(spec.mass.unwrap_or(1.0), true);
    if !mass_representable(geometric) {
        return Err(PhysicsError::InvalidValue);
    }
    let shape = ColliderBuilder::compound(parts)
        .mass_properties(geometric)
        .build();
    if !mass_representable(shape.mass_properties())
        || !bounds_representable(&shape.shape().compute_aabb(&Pose::IDENTITY))
        || !bounds_representable(&shape.shape().compute_aabb(&world))
    {
        return Err(PhysicsError::InvalidValue);
    }
    if spec.mass.is_some() && inertia.is_none() {
        return Err(PhysicsError::Unsupported {
            engine: "rapier",
            capability: "dynamic primitive compound without explicit inertia",
        });
    }
    let properties = inertia
        .map(|value| {
            if !finite(value.center) {
                return Err(PhysicsError::InvalidValue);
            }
            if value.center != [0.0; 3] {
                return Err(PhysicsError::Unsupported {
                    engine: "rapier",
                    capability: "primitive compound offset inertia",
                });
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
    // insert() zeroes density for supplied inertia: no child-derived COM/tensor.
    // A fixed body without inertia retains only geometric collider bookkeeping.
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
    fn exhausted_compound_ids_leave_one_native_collider_and_pending_state_unchanged() {
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let children = [CollisionSpec {
            shape: Shape::Sphere { radius: 0.5 },
            position: [0.0; 3],
            orientation: [0.0, 0.0, 0.0, 1.0],
        }; 2];
        let existing = backend
            .spawn_compound_link(
                CompoundBodySpec {
                    collisions: &children,
                    position: [0.0; 3],
                    velocity: [0.0; 3],
                    mass: Some(2.0),
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [0.0; 3],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    tensor: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
                }),
            )
            .unwrap();
        assert_eq!(backend.world.bodies.len(), 1);
        assert_eq!(backend.world.colliders.len(), 1);
        assert_eq!(
            backend
                .world
                .colliders
                .iter()
                .next()
                .unwrap()
                .1
                .shape()
                .as_compound()
                .unwrap()
                .shapes()
                .len(),
            2
        );
        backend
            .add_world_wrench(existing, [2.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
        let before = backend.state(existing);
        backend.next_id = u64::MAX;
        assert_eq!(
            backend.spawn_compound_link(
                CompoundBodySpec {
                    collisions: &children,
                    position: [0.0; 3],
                    velocity: [0.0; 3],
                    mass: None
                },
                RotationSpec::default(),
                None
            ),
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
