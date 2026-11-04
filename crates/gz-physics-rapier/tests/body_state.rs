// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, JointVelocityMotor, MeshBodySpec,
    PhysicsError, RapierBackend, RotationSpec, Shape,
};
use std::time::Duration;

const DT: Duration = Duration::from_millis(10);
const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

fn ball(position: [f64; 3]) -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.5 },
        position,
        velocity: [0.0; 3],
        mass: Some(2.0),
    }
}

fn state(position: [f64; 3]) -> BodyState {
    BodyState {
        position,
        velocity: [0.0; 3],
        orientation: IDENTITY,
        angular_velocity: [0.0; 3],
    }
}

fn hit(backend: &RapierBackend, origin: [f64; 3], expected: f64) {
    let distance = backend
        .cast_ray(origin, [1.0, 0.0, 0.0], 30.0, None)
        .unwrap()
        .unwrap();
    assert!(
        (distance - expected).abs() < 1e-12,
        "{distance} != {expected}"
    );
}

#[test]
fn post_step_spawn_edit_and_remove_keep_existing_motion_and_expose_fresh_shape_frames() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut moving = ball([100.0, 0.0, 0.0]);
    moving.velocity = [1.0, 2.0, 3.0];
    let survivor = backend.spawn(moving).unwrap();
    backend.step(DT).unwrap();
    let before = backend.state(survivor);
    let h = std::f64::consts::FRAC_1_SQRT_2;
    for shape in [
        Shape::Sphere { radius: 0.5 },
        Shape::Box {
            half_extents: [1.0, 0.5, 0.25],
        },
        Shape::Cylinder {
            radius: 0.5,
            half_height: 1.0,
        },
        Shape::Capsule {
            radius: 0.5,
            half_length: 1.0,
        },
    ] {
        let id = backend
            .spawn(BodySpec {
                shape,
                mass: None,
                ..ball([10.0, 0.0, 0.0])
            })
            .unwrap();
        let requested = BodyState {
            orientation: [0.0, 0.0, h, h],
            ..state([5.0, 0.0, 0.0])
        };
        let original = backend.state(id);
        backend.validate_body_state(id, requested).unwrap();
        assert_eq!(backend.state(id), original);
        backend.set_body_state(id, requested).unwrap();
        assert_eq!(backend.state(id), Some(requested));
        hit(&backend, [0.0; 3], 4.5);
        backend.step(Duration::ZERO).unwrap();
        assert_eq!(backend.state(survivor), before);
        assert!(backend.remove(id));
        assert!(!backend.remove(id));
        assert_eq!(
            backend.cast_ray([0.0; 3], [1.0, 0.0, 0.0], 30.0, None),
            Ok(None)
        );
    }
    let mesh = backend
        .spawn_mesh_link(
            MeshBodySpec {
                vertices: &[[0.0, -1.0, -1.0], [0.0, 1.0, -1.0], [0.0, 0.0, 1.0]],
                triangles: &[[0, 1, 2]],
                collision_position: [1.0, 0.0, 0.0],
                collision_orientation: IDENTITY,
                position: [6.0, 10.0, 0.0],
                velocity: [0.0; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    hit(&backend, [0.0, 10.0, 0.0], 7.0);
    backend
        .set_body_state(
            mesh,
            BodyState {
                orientation: [0.0, 0.0, 1.0, 0.0],
                ..state([9.0, 10.0, 0.0])
            },
        )
        .unwrap();
    hit(&backend, [0.0, 10.0, 0.0], 8.0);
    assert_eq!(backend.state(survivor), before);
    assert!(backend.remove(mesh));
    assert_eq!(
        backend.cast_ray([0.0, 10.0, 0.0], [1.0, 0.0, 0.0], 30.0, None),
        Ok(None)
    );
    assert!(backend.spawn(ball([20.0, 0.0, 0.0])).unwrap() > mesh);
    assert_eq!(backend.state(survivor), before);
}

#[test]
fn velocity_is_at_com_and_rotated_inertia_and_pending_wrenches_remain_authoritative() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend
        .spawn_link(
            ball([0.0; 3]),
            RotationSpec::default(),
            Some(Inertia {
                center: [0.1, 0.0, 0.0],
                orientation: IDENTITY,
                tensor: [2.0, 3.0, 4.0, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    backend.step(DT).unwrap();
    backend
        .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let requested = BodyState {
        velocity: [1.0, 2.0, 3.0],
        orientation: [0.0, 0.0, h, h],
        ..state([10.0, 0.0, 0.0])
    };
    backend.set_body_state(id, requested).unwrap();
    assert_eq!(backend.state(id), Some(requested));
    // Rz(π/2) swaps principal X/Y. The native setter must refresh world inertia.
    backend
        .apply_impulse(id, [0.0; 3], [3.0, 2.0, 4.0])
        .unwrap();
    for (actual, expected) in backend
        .state(id)
        .unwrap()
        .angular_velocity
        .into_iter()
        .zip([1.0; 3])
    {
        assert!((actual - expected).abs() < 1e-12);
    }
    backend.step(DT).unwrap();
    // Pose editing did not consume the already queued force: Δv=F/m*dt=.01.
    assert!((backend.state(id).unwrap().velocity[0] - 1.01).abs() < 1e-12);
    backend.step(DT).unwrap();
    assert!((backend.state(id).unwrap().velocity[0] - 1.01).abs() < 1e-12);
}

#[test]
fn invalid_edits_preserve_prior_state_pending_work_ids_and_fixed_velocity_contract() {
    let mut edited = RapierBackend::new([0.0; 3]).unwrap();
    let mut control = RapierBackend::new([0.0; 3]).unwrap();
    let body = edited.spawn(ball([0.0; 3])).unwrap();
    assert_eq!(control.spawn(ball([0.0; 3])).unwrap(), body);
    for backend in [&mut edited, &mut control] {
        backend.step(DT).unwrap();
        backend
            .add_world_wrench(body, [2.0, 4.0, 6.0], [0.1, 0.2, 0.3])
            .unwrap();
    }
    let before = edited.state(body);
    for invalid in [
        BodyState {
            position: [f64::NAN, 0.0, 0.0],
            ..state([0.0; 3])
        },
        BodyState {
            velocity: [f64::INFINITY, 0.0, 0.0],
            ..state([0.0; 3])
        },
        BodyState {
            angular_velocity: [0.0, f64::NEG_INFINITY, 0.0],
            ..state([0.0; 3])
        },
        BodyState {
            orientation: [f64::NAN, 0.0, 0.0, 1.0],
            ..state([0.0; 3])
        },
        BodyState {
            orientation: [0.0; 4],
            ..state([0.0; 3])
        },
        BodyState {
            orientation: [0.0, 0.0, 0.0, 1.0001],
            ..state([0.0; 3])
        },
    ] {
        assert_eq!(
            edited.validate_body_state(body, invalid),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(
            edited.set_body_state(body, invalid),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(edited.state(body), before);
        assert_eq!(
            edited.set_body_state(BodyId::from_raw(999), invalid),
            Err(PhysicsError::MissingBody)
        );
    }
    for backend in [&mut edited, &mut control] {
        backend.step(DT).unwrap();
    }
    assert_eq!(edited.state(body), control.state(body));
    let next = edited
        .spawn(BodySpec {
            mass: None,
            ..ball([10.0, 0.0, 0.0])
        })
        .unwrap();
    assert_eq!(next, BodyId::from_raw(1));
    let fixed_before = edited.state(next);
    for request in [
        BodyState {
            velocity: [1.0, 0.0, 0.0],
            ..state([11.0, 0.0, 0.0])
        },
        BodyState {
            angular_velocity: [0.0, 0.0, 1.0],
            ..state([11.0, 0.0, 0.0])
        },
    ] {
        assert_eq!(
            edited.validate_body_state(next, request),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(
            edited.set_body_state(next, request),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(edited.state(next), fixed_before);
    }
    assert!(edited.remove(next));
    assert_eq!(
        edited.validate_body_state(next, state([0.0; 3])),
        Err(PhysicsError::MissingBody)
    );
    assert_eq!(
        edited.set_body_state(next, state([0.0; 3])),
        Err(PhysicsError::MissingBody)
    );
}

#[test]
fn transformed_mesh_bounds_are_validated_before_any_pose_or_velocity_mutation() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend
        .spawn_mesh_link(
            MeshBodySpec {
                vertices: &[[0.0, -1.0, -1.0], [0.0, 1.0, -1.0], [0.0, 0.0, 1.0]],
                triangles: &[[0, 1, 2]],
                collision_position: [1.0e308, 0.0, 0.0],
                collision_orientation: IDENTITY,
                position: [-1.0e308, 0.0, 0.0],
                velocity: [0.0; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    let before = backend.state(id);
    let overflow = state([1.0e308, 0.0, 0.0]);
    assert_eq!(
        backend.validate_body_state(id, overflow),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        backend.set_body_state(id, overflow),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(backend.state(id), before);
    hit(&backend, [-1.0, 0.0, 0.0], 1.0);
}

#[test]
fn jointed_edits_refuse_both_endpoint_kinds_without_disturbing_native_motors() {
    for reduced in [false, true] {
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let parent = backend.spawn(ball([0.0; 3])).unwrap();
        let child = backend.spawn(ball([0.0, 0.0, 2.0])).unwrap();
        let joint = if reduced {
            backend
                .add_reduced_revolute(Some(parent), child, [0.0; 3], [0.0, 1.0, 0.0])
                .unwrap()
        } else {
            backend
                .add_revolute(parent, child, [0.0; 3], [0.0, 1.0, 0.0])
                .unwrap()
        };
        backend
            .set_joint_velocity_motor(
                joint,
                Some(JointVelocityMotor {
                    target_velocity: 1.0,
                    max_force: 0.1,
                }),
            )
            .unwrap();
        backend.step(DT).unwrap();
        let before = (
            backend.state(parent),
            backend.state(child),
            backend.joint_state(joint),
        );
        for body in [parent, child] {
            let error = PhysicsError::Unsupported {
                engine: "rapier",
                capability: "jointed body-state replacement",
            };
            assert_eq!(
                backend.validate_body_state(body, state([10.0; 3])),
                Err(error.clone())
            );
            assert_eq!(backend.set_body_state(body, state([10.0; 3])), Err(error));
            assert_eq!(
                (
                    backend.state(parent),
                    backend.state(child),
                    backend.joint_state(joint)
                ),
                before
            );
        }
        backend.step(DT).unwrap();
        assert!(backend.joint_state(joint).unwrap().velocity > before.2.unwrap().velocity);
    }
}

#[test]
fn ordinary_and_captured_resets_keep_spawn_records_and_monotonic_runtime_ids() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend.spawn(ball([0.0; 3])).unwrap();
    let original = backend.state(id);
    let initial = backend.initial_state();
    backend.step(DT).unwrap();
    backend
        .set_body_state(
            id,
            BodyState {
                velocity: [1.0; 3],
                angular_velocity: [0.0, 0.0, 2.0],
                ..state([5.0; 3])
            },
        )
        .unwrap();
    let runtime = backend.spawn(ball([10.0, 0.0, 0.0])).unwrap();
    let runtime_original = backend.state(runtime);
    backend
        .set_body_state(runtime, state([20.0, 0.0, 0.0]))
        .unwrap();
    backend.add_world_wrench(id, [20.0; 3], [0.0; 3]).unwrap();
    backend.reset().unwrap();
    assert_eq!(backend.state(id), original);
    assert_eq!(backend.state(runtime), runtime_original);
    backend.step(DT).unwrap();
    assert_eq!(backend.state(id), original);
    assert!(backend.remove(id));
    backend
        .apply_initial(backend.prepare_initial(&initial).unwrap())
        .unwrap();
    assert_eq!(backend.state(id), original);
    assert_eq!(backend.state(runtime), None);
    assert!(backend.spawn(ball([30.0, 0.0, 0.0])).unwrap() > runtime);
}
