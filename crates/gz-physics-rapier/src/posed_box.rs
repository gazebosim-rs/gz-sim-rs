// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A single box keeps its collider frame separate from the link and inertia.
use super::*;

pub(super) fn spawn(
    backend: &mut RapierBackend,
    spec: PosedBoxBodySpec,
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
    let local = Pose::from_parts(Vector::from(spec.collision_position), collision_orientation);
    let world = Pose::from_parts(Vector::from(spec.position), orientation) * local;
    if !finite(world.translation.to_array()) || !world.rotation.is_finite() {
        return Err(PhysicsError::InvalidValue);
    }
    let shape = collider(Shape::Box {
        half_extents: spec.half_extents,
    })?
    .position(local)
    .mass(spec.mass.unwrap_or(1.0))
    .build();
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
            capability: "dynamic posed box without explicit inertia",
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
    fn exhausted_posed_box_ids_leave_native_and_public_state_unchanged() {
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        backend.next_id = u64::MAX;
        let spec = PosedBoxBodySpec {
            half_extents: [0.5; 3],
            collision_position: [1.0, -2.0, 3.0],
            collision_orientation: [0.0, 0.0, 0.0, 1.0],
            position: [0.0; 3],
            velocity: [0.0; 3],
            mass: None,
        };
        assert_eq!(
            backend.spawn_posed_box_link(spec, RotationSpec::default(), None),
            Err(PhysicsError::IdOverflow)
        );
        assert!(backend.bodies.is_empty());
        assert!(backend.world.bodies.is_empty());
        assert!(backend.world.colliders.is_empty());
        assert!(backend.pending_wrenches.is_empty());
        assert_eq!(backend.next_id, u64::MAX);
    }

    #[test]
    fn offset_posed_primitive_mass_frames_bounds_resets_and_pending_ids_are_native() {
        #[derive(Clone, Copy, Debug)]
        struct Request {
            kind: usize,
            position: [f64; 3],
            local: [f64; 3],
            body_rotation: [f64; 4],
            child_rotation: [f64; 4],
            mass: Option<f64>,
            inertia: Inertia,
        }
        fn request(kind: usize, mass: Option<f64>) -> Request {
            let q = std::f64::consts::FRAC_1_SQRT_2;
            Request {
                kind,
                position: [10., 20., 30.],
                local: [2., 0., 0.],
                body_rotation: [0.5; 4],
                child_rotation: [0., q, 0., q],
                mass,
                inertia: Inertia {
                    center: [0.25, -0.5, 0.75],
                    orientation: [0., 0., 0., 1.],
                    tensor: [2., 2., 2., 0., 0., 0.],
                },
            }
        }
        fn spawn(backend: &mut RapierBackend, r: Request) -> Result<BodyId, PhysicsError> {
            let rotation = RotationSpec {
                orientation: r.body_rotation,
                angular_velocity: [0.; 3],
            };
            match r.kind {
                0 => backend.spawn_posed_box_link(
                    PosedBoxBodySpec {
                        half_extents: [0.75, 0.25, 0.5],
                        collision_position: r.local,
                        collision_orientation: r.child_rotation,
                        position: r.position,
                        velocity: [0.; 3],
                        mass: r.mass,
                    },
                    rotation,
                    Some(r.inertia),
                ),
                1 => backend.spawn_posed_sphere_link(
                    PosedSphereBodySpec {
                        radius: 0.5,
                        collision_position: r.local,
                        collision_orientation: r.child_rotation,
                        position: r.position,
                        velocity: [0.; 3],
                        mass: r.mass,
                    },
                    rotation,
                    Some(r.inertia),
                ),
                2 => backend.spawn_posed_cylinder_link(
                    PosedCylinderBodySpec {
                        radius: 0.5,
                        half_height: 1.,
                        collision_position: r.local,
                        collision_orientation: r.child_rotation,
                        position: r.position,
                        velocity: [0.; 3],
                        mass: r.mass,
                    },
                    rotation,
                    Some(r.inertia),
                ),
                _ => backend.spawn_posed_capsule_link(
                    PosedCapsuleBodySpec {
                        radius: 0.5,
                        half_length: 1.,
                        collision_position: r.local,
                        collision_orientation: r.child_rotation,
                        position: r.position,
                        velocity: [0.; 3],
                        mass: r.mass,
                    },
                    rotation,
                    Some(r.inertia),
                ),
            }
        }
        fn near(a: f64, b: f64) {
            assert!(
                a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
                "{a} != {b}"
            );
        }
        fn vector(a: [f64; 3], b: [f64; 3]) {
            for (a, b) in a.into_iter().zip(b) {
                near(a, b);
            }
        }
        fn properties(backend: &RapierBackend, id: BodyId, r: Request) {
            let entry = &backend.bodies[&id];
            let body = &backend.world.bodies[entry.0];
            let props = body.mass_properties();
            vector(props.local_mprops.local_com.to_array(), r.inertia.center);
            // Independent q_cycle axis permutation. Collision local is NOT part
            // of this mass-frame transform: that would add two meters on Y.
            vector(
                props.world_com.to_array(),
                [
                    r.position[0] + 0.75,
                    r.position[1] + 0.25,
                    r.position[2] - 0.5,
                ],
            );
            vector(
                props.local_mprops.inv_principal_inertia.to_array(),
                [0.5; 3],
            );
            let collider = &backend.world.colliders[body.colliders()[0]];
            assert_eq!(collider.density(), 0.);
            assert_eq!(
                collider
                    .position_wrt_parent()
                    .unwrap()
                    .translation
                    .to_array(),
                r.local
            );
            let radius = [
                (0.75_f64 * 0.75 + 0.25 * 0.25 + 0.5 * 0.5).sqrt(),
                0.5,
                (1.25_f64).sqrt(),
                1.5,
            ][r.kind];
            near(
                props.max_extent(),
                (1.75_f64 * 1.75 + 0.5 * 0.5 + 0.75 * 0.75).sqrt() + radius,
            );
            let saved = entry.3.unwrap();
            assert_eq!(saved.center, r.inertia.center);
            assert_eq!(saved.orientation, r.inertia.orientation);
            assert_eq!(saved.tensor, r.inertia.tensor);
            assert_eq!(entry.1.position, r.position);
            assert_eq!(entry.2.orientation, r.body_rotation);
        }
        let dt = Duration::from_millis(1);
        for kind in 0..4 {
            for mass in [None, Some(2.)] {
                let mut backend = RapierBackend::new([0.; 3]).unwrap();
                let r = request(kind, mass);
                let id = spawn(&mut backend, r).unwrap();
                properties(&backend, id, r);
                let original = backend.state(id).unwrap();
                assert_eq!(original.position, r.position);
                assert_ne!(
                    original.position,
                    backend.world.bodies[backend.bodies[&id].0]
                        .mass_properties()
                        .world_com
                        .to_array()
                );
                let center = [10., 22., 30.];
                near(
                    backend
                        .cast_ray([10.3, 22., 34.], [0., 0., -1.], 5., None)
                        .unwrap()
                        .unwrap(),
                    if kind == 0 { 3.75 } else { 3.6 },
                );
                assert_eq!(
                    backend
                        .cast_ray([10.3, 22., 34.], [0., 0., -1.], 5., Some(id))
                        .unwrap(),
                    None
                );
                assert_eq!(
                    backend.cast_ray(center, [0., 0., 1.], 0.125, None).unwrap(),
                    Some(0.)
                );
                let captured = backend.initial_state();
                if mass.is_some() {
                    let initial_com = backend.world.bodies[backend.bodies[&id].0]
                        .mass_properties()
                        .world_com
                        .to_array();
                    backend.add_world_wrench(id, [0.; 3], [0., 0., 4.]).unwrap();
                    backend.step(Duration::ZERO).unwrap();
                    assert_eq!(backend.state(id), Some(original));
                    backend.step(dt).unwrap();
                    let state = backend.state(id).unwrap();
                    vector(state.velocity, [0.; 3]);
                    vector(state.angular_velocity, [0., 0., 0.002]);
                    vector(
                        backend.world.bodies[backend.bodies[&id].0]
                            .mass_properties()
                            .world_com
                            .to_array(),
                        initial_com,
                    );
                    assert_ne!(state.position, original.position);
                    assert!(backend.pending_wrenches.is_empty());
                    backend.reset().unwrap();
                    properties(&backend, id, r);
                    backend
                        .add_world_wrench(id, [2., -4., 6.], [0.; 3])
                        .unwrap();
                    backend.step(dt).unwrap();
                    let state = backend.state(id).unwrap();
                    vector(state.velocity, [0.001, -0.002, 0.003]);
                    vector(state.angular_velocity, [0.; 3]);
                    let moved = backend.world.bodies[backend.bodies[&id].0]
                        .mass_properties()
                        .world_com
                        .to_array();
                    assert!(
                        moved[0] > initial_com[0]
                            && moved[1] < initial_com[1]
                            && moved[2] > initial_com[2]
                    );
                }
                let changed = BodyState {
                    position: [40., 50., 60.],
                    orientation: [0.5; 4],
                    velocity: if mass.is_some() {
                        [1., 2., 3.]
                    } else {
                        [0.; 3]
                    },
                    angular_velocity: if mass.is_some() {
                        [0., 0., 1.]
                    } else {
                        [0.; 3]
                    },
                };
                backend.validate_body_state(id, changed).unwrap();
                backend.set_body_state(id, changed).unwrap();
                assert_eq!(backend.state(id), Some(changed));
                vector(
                    backend.world.bodies[backend.bodies[&id].0]
                        .mass_properties()
                        .world_com
                        .to_array(),
                    [40.75, 50.25, 59.5],
                );
                backend.reset().unwrap();
                assert_eq!(backend.state(id), Some(original));
                properties(&backend, id, r);
                let prepared = backend.prepare_initial(&captured).unwrap();
                assert!(backend.remove(id));
                let extra = backend
                    .spawn(BodySpec {
                        shape: Shape::Sphere { radius: 0.25 },
                        position: [100.; 3],
                        velocity: [0.; 3],
                        mass: Some(2.),
                    })
                    .unwrap();
                backend.apply_initial(prepared).unwrap();
                assert_eq!(backend.state(extra), None);
                assert_eq!(backend.state(id), Some(original));
                properties(&backend, id, r);
                let next = backend
                    .spawn(BodySpec {
                        shape: Shape::Sphere { radius: 0.25 },
                        position: [200.; 3],
                        velocity: [0.; 3],
                        mass: Some(2.),
                    })
                    .unwrap();
                assert!(next > extra);
                assert!(backend.remove(id));
                assert!(!backend.remove(id));
                assert!(backend.state(next).is_some());
            }
            // Each failure starts with genuine queued native force and torque.
            // The repair is then stepped against an untouched physical twin.
            for row in 0..11 {
                let mut backend = RapierBackend::new([0.; 3]).unwrap();
                let mut twin = RapierBackend::new([0.; 3]).unwrap();
                let spec = BodySpec {
                    shape: Shape::Sphere { radius: 0.25 },
                    position: [-20., 0., 0.],
                    velocity: [0.; 3],
                    mass: Some(2.),
                };
                let id = backend.spawn(spec).unwrap();
                assert_eq!(twin.spawn(spec).unwrap(), id);
                for b in [&mut backend, &mut twin] {
                    b.add_world_wrench(id, [2., 4., 6.], [0., 0., 0.2]).unwrap();
                }
                let mut bad = request(kind, Some(2.));
                match row {
                    0 => bad.inertia.center = [f64::NAN, 0., 0.],
                    1 => bad.inertia.center = [f64::INFINITY, 0., 0.],
                    2 => bad.inertia.center = [f64::MAX, 0., 0.],
                    3 => {
                        bad.inertia.center = [f64::MAX, f64::MAX, 0.];
                        let angle = std::f64::consts::FRAC_PI_8;
                        bad.body_rotation = [0., 0., angle.sin(), angle.cos()];
                    }
                    // These finite extreme COM/child offsets also have an
                    // overflowing relative norm. Earlier collider-bound checks
                    // may reject first; this is not a claim that a later guard ran.
                    4 => {
                        bad.inertia.center = [-1e154, 0., 0.];
                        bad.local = [1e154, 0., 0.];
                    }
                    5 => bad.inertia.orientation = [0.; 4],
                    6 => bad.inertia.tensor = [-1., 1., 1., 0., 0., 0.],
                    7 => bad.inertia.tensor = [f64::MAX; 6],
                    8 => backend.next_id = u64::MAX,
                    9 => bad.mass = Some(f64::NAN),
                    _ => {
                        bad.inertia.center = [1e154, 0., 0.];
                        bad.position = [1e154, 0., 0.];
                        bad.body_rotation = [0., 0., 0., 1.];
                    }
                }
                if row == 2 {
                    assert!(
                        !Vector::from(bad.inertia.center)
                            .length_squared()
                            .is_finite()
                    );
                }
                if row == 3 {
                    let body_pose = Pose::from_parts(
                        Vector::from(bad.position),
                        rotation(bad.body_rotation).unwrap(),
                    );
                    assert!(!finite(
                        (body_pose * Vector::from(bad.inertia.center)).to_array()
                    ));
                }
                if row == 4 {
                    assert!(
                        !(Vector::from(bad.local) - Vector::from(bad.inertia.center))
                            .length()
                            .is_finite()
                    );
                }
                if row == 10 {
                    assert!(
                        Vector::from(bad.inertia.center)
                            .length_squared()
                            .is_finite()
                    );
                    assert!(
                        !(Vector::from(bad.position) + Vector::from(bad.inertia.center))
                            .length_squared()
                            .is_finite()
                    );
                    // The pre-existing collider AABB may reject before this
                    // finite-component/world-norm overflow reaches COM admission.
                }
                let handle = backend.bodies[&id].0;
                let before = backend.state(id);
                let next = backend.next_id;
                let pending = backend.pending_wrenches.clone();
                let force = backend.world.bodies[handle].user_force();
                let torque = backend.world.bodies[handle].user_torque();
                let count = (
                    backend.bodies.len(),
                    backend.world.bodies.len(),
                    backend.world.colliders.len(),
                );
                assert_eq!(
                    spawn(&mut backend, bad),
                    Err(if row == 8 {
                        PhysicsError::IdOverflow
                    } else {
                        PhysicsError::InvalidValue
                    }),
                    "kind={kind}, row={row}"
                );
                assert_eq!(backend.state(id), before);
                assert_eq!(backend.next_id, next);
                assert_eq!(backend.pending_wrenches, pending);
                assert_eq!(backend.world.bodies[handle].user_force(), force);
                assert_eq!(backend.world.bodies[handle].user_torque(), torque);
                assert_eq!(
                    (
                        backend.bodies.len(),
                        backend.world.bodies.len(),
                        backend.world.colliders.len()
                    ),
                    count
                );
                if row == 8 {
                    backend.next_id = twin.next_id;
                }
                let repaired = spawn(&mut backend, request(kind, Some(2.))).unwrap();
                assert_eq!(spawn(&mut twin, request(kind, Some(2.))).unwrap(), repaired);
                assert_eq!(repaired.raw(), 1);
                backend.step(Duration::ZERO).unwrap();
                assert_eq!(backend.state(id), before);
                backend.step(dt).unwrap();
                twin.step(dt).unwrap();
                assert_eq!(backend.state(id), twin.state(id));
                assert_eq!(backend.state(repaired), twin.state(repaired));
                vector(backend.state(id).unwrap().velocity, [0.001, 0.002, 0.003]);
                assert!(backend.state(id).unwrap().angular_velocity[2] > 0.);
            }
        }
    }
}
