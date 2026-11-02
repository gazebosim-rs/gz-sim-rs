// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Backend-only construction and lifecycle controls; no B03 acceptance claim.
use gz_physics::{
    BodyId, BodySpec, BodyState, CollisionSpec, CompoundBodySpec, DynamicBackend, Inertia, JointId,
    PhysicsError, RotationSpec, Shape,
};
use gz_physics_dart::DartBackend;
use gz_physics_rapier::RapierBackend;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use std::time::Duration;

const DT: Duration = Duration::from_millis(1);
const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];
const INERTIA: Inertia = Inertia {
    center: [0.0; 3],
    orientation: IDENTITY,
    tensor: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
};
const SPHERE: CollisionSpec = CollisionSpec {
    shape: Shape::Sphere { radius: 0.2 },
    position: [0.0; 3],
    orientation: IDENTITY,
};

fn rotation(value: [f64; 4]) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(Quaternion::new(value[3], value[0], value[1], value[2]))
}

fn collisions() -> [CollisionSpec; 3] {
    let q = UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.3);
    [
        CollisionSpec {
            shape: Shape::Box {
                half_extents: [0.3, 0.2, 0.4],
            },
            position: [-2.0, 0.0, 0.0],
            orientation: IDENTITY,
        },
        CollisionSpec {
            shape: Shape::Cylinder {
                radius: 0.2,
                half_height: 0.5,
            },
            position: [0.0; 3],
            orientation: [q.i, q.j, q.k, q.w],
        },
        CollisionSpec {
            shape: Shape::Capsule {
                radius: 0.2,
                half_length: 0.4,
            },
            position: [2.0, 0.0, 0.0],
            orientation: [q.i, q.j, q.k, q.w],
        },
    ]
}

fn compound(
    backend: &mut dyn DynamicBackend,
    shapes: &[CollisionSpec],
    position: [f64; 3],
    orientation: [f64; 4],
    fixed: bool,
) -> Result<BodyId, PhysicsError> {
    backend.spawn_compound_link(
        CompoundBodySpec {
            collisions: shapes,
            position,
            velocity: [0.0; 3],
            mass: (!fixed).then_some(1.0),
        },
        RotationSpec {
            orientation,
            ..Default::default()
        },
        INERTIA,
    )
}

fn single(backend: &mut DartBackend, position: [f64; 3]) -> BodyId {
    backend
        .spawn_link(
            BodySpec {
                shape: SPHERE.shape,
                position,
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            Some(INERTIA),
        )
        .unwrap()
}

fn near(actual: BodyState, expected: BodyState) {
    let a: Vec<_> = actual
        .position
        .into_iter()
        .chain(actual.velocity)
        .chain(actual.orientation)
        .chain(actual.angular_velocity)
        .collect();
    let e: Vec<_> = expected
        .position
        .into_iter()
        .chain(expected.velocity)
        .chain(expected.orientation)
        .chain(expected.angular_velocity)
        .collect();
    assert_eq!(a.len(), 13);
    for (a, e) in a.into_iter().zip(e) {
        assert!((a - e).abs() <= 1e-12, "{actual:?} != {expected:?}");
    }
}

#[test]
fn one_centered_collision_preserves_existing_body_and_explicit_inertia_semantics() {
    let mut old = DartBackend::new([0.0; 3]).unwrap();
    let mut new = DartBackend::new([0.0; 3]).unwrap();
    let q = UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
    let r = UnitQuaternion::from_euler_angles(-0.2, 0.3, -0.1);
    let inertia = Inertia {
        orientation: [q.i, q.j, q.k, q.w],
        tensor: [1.0, 1.5, 2.0, 0.0, 0.0, 0.0],
        ..INERTIA
    };
    let rotational = RotationSpec {
        orientation: [r.i, r.j, r.k, r.w],
        ..Default::default()
    };
    let a = old
        .spawn_link(
            BodySpec {
                shape: SPHERE.shape,
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(2.0),
            },
            rotational,
            Some(inertia),
        )
        .unwrap();
    let b = new
        .spawn_compound_link(
            CompoundBodySpec {
                collisions: &[SPHERE],
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(2.0),
            },
            rotational,
            inertia,
        )
        .unwrap();
    for (backend, id) in [(&mut old, a), (&mut new, b)] {
        backend
            .set_world_velocity(id, [0.2, -0.1, 0.3], [0.1, 0.2, -0.3])
            .unwrap();
        backend
            .add_world_wrench(id, [2.0, -1.0, 3.0], [0.3, 0.2, -0.1])
            .unwrap();
    }
    assert_eq!(a, b);
    for _ in 0..10 {
        old.step(DT).unwrap();
        new.step(DT).unwrap();
        near(new.state(b).unwrap(), old.state(a).unwrap());
    }
}

#[test]
fn all_local_collisions_match_separate_fixed_primitives_including_rotation() {
    let shapes = collisions();
    let body_rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3);
    let orientation = [
        body_rotation.i,
        body_rotation.j,
        body_rotation.k,
        body_rotation.w,
    ];
    let origin = Vector3::new(1.0, 2.0, 0.5);
    for selected in 0..3 {
        let mut joined = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
        let mut separate = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
        let body = compound(&mut joined, &shapes, origin.into(), orientation, true).unwrap();
        assert_eq!(body.raw(), 0);
        for shape in shapes {
            let position = origin + body_rotation * Vector3::from(shape.position);
            let q = body_rotation * rotation(shape.orientation);
            separate
                .spawn_link(
                    BodySpec {
                        shape: shape.shape,
                        position: position.into(),
                        velocity: [0.0; 3],
                        mass: None,
                    },
                    RotationSpec {
                        orientation: [q.i, q.j, q.k, q.w],
                        ..Default::default()
                    },
                    Some(INERTIA),
                )
                .unwrap();
        }
        let center = origin + body_rotation * Vector3::from(shapes[selected].position);
        let position = [center.x, center.y, center.z + 1.0];
        let a = single(&mut joined, position);
        let b = single(&mut separate, position);
        // The three primitives share exactly one solver identity.
        assert_eq!(a.raw(), 1);
        assert_eq!(b.raw(), 3);
        let fixed = joined.state(body).unwrap();
        let mut contacted = false;
        for tick in 1..=600 {
            joined.step(DT).unwrap();
            separate.step(DT).unwrap();
            let actual = joined.state(a).unwrap();
            near(actual, separate.state(b).unwrap());
            contacted |= actual.velocity[2] > -9.81 * f64::from(tick) * 0.001 + 0.01;
        }
        assert!(
            contacted,
            "local primitive {selected} never contacted the probe"
        );
        assert_eq!(joined.state(body), Some(fixed));
    }
}

#[test]
fn invalid_final_collision_and_body_values_preserve_ids_state_and_pending_wrench() {
    for case in 0..13 {
        let mut backend = DartBackend::new([0.0; 3]).unwrap();
        let mut control = DartBackend::new([0.0; 3]).unwrap();
        let body = single(&mut backend, [10.0, 0.0, 0.0]);
        let reference = single(&mut control, [10.0, 0.0, 0.0]);
        for value in [&mut backend, &mut control] {
            value
                .add_world_wrench(body, [3.0, -2.0, 1.0], [0.1, 0.2, 0.3])
                .unwrap();
        }
        let before = backend.state(body).unwrap();
        let mut shapes = vec![SPHERE; 3];
        let mut inertia = INERTIA;
        let mut orientation = IDENTITY;
        let mut position = [0.0; 3];
        let mut velocity = [0.0; 3];
        let mut mass = Some(1.0);
        match case {
            0 => shapes.clear(),
            1 => shapes.push(SPHERE),
            2 => shapes[2].position[0] = f64::NAN,
            3 => shapes[2].orientation = [0.0; 4],
            4 => shapes[2].shape = Shape::Sphere { radius: -0.1 },
            5 => shapes[2].position[0] = 5e8,
            6 => shapes[2].position[0] = 1000.0,
            7 => inertia.center[2] = 0.5,
            8 => inertia.tensor[0] = -1.0,
            9 => orientation = [0.0; 4],
            10 => position[0] = f64::INFINITY,
            11 => {
                mass = None;
                velocity[0] = 1.0;
            }
            12 => {
                mass = None;
                position[0] = 1e60;
                for shape in &mut shapes {
                    shape.position[0] = -1e60;
                }
            }
            _ => unreachable!(),
        }
        assert_eq!(
            backend.spawn_compound_link(
                CompoundBodySpec {
                    collisions: &shapes,
                    position,
                    velocity,
                    mass
                },
                RotationSpec {
                    orientation,
                    ..Default::default()
                },
                inertia,
            ),
            Err(PhysicsError::InvalidValue),
            "case {case}"
        );
        assert_eq!(backend.state(body), Some(before));
        let next = compound(&mut backend, &[SPHERE], [-10.0, 0.0, 0.0], IDENTITY, true).unwrap();
        assert_eq!(
            next.raw(),
            1,
            "failed case {case} consumed a public body ID"
        );
        backend.step(DT).unwrap();
        control.step(DT).unwrap();
        near(
            backend.state(body).unwrap(),
            control.state(reference).unwrap(),
        );
    }
}

#[test]
fn rotated_final_shape_changes_the_existing_principal_response_bound() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let mut shapes = [SPHERE; 3];
    shapes[2].shape = Shape::Box {
        half_extents: [10.0, 0.01, 0.01],
    };
    let inertia = Inertia {
        tensor: [0.0001, 1.0, 1.0, 0.0, 0.0, 0.0],
        ..INERTIA
    };
    let accepted = backend
        .spawn_compound_link(
            CompoundBodySpec {
                collisions: &shapes,
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            inertia,
        )
        .unwrap();
    assert!(backend.remove(accepted));
    let q = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
    shapes[2].orientation = [q.i, q.j, q.k, q.w];
    assert_eq!(
        backend.spawn_compound_link(
            CompoundBodySpec {
                collisions: &shapes,
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0)
            },
            RotationSpec::default(),
            inertia
        ),
        Err(PhysicsError::InvalidValue)
    );
}

#[test]
fn world_grid_checks_compose_body_and_last_collision_rotation() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let mut shapes = [SPHERE; 3];
    shapes[2].shape = Shape::Box {
        half_extents: [1.0, 0.1, 0.1],
    };
    let q = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
    let orientation = [q.i, q.j, q.k, q.w];
    let position = [5e8 - 0.5, 0.0, 0.0];
    let accepted = compound(&mut backend, &shapes, position, orientation, true).unwrap();
    assert!(backend.remove(accepted));
    // The second quarter turn moves the long axis back onto world X. Checking
    // only the link rotation would miss the final shape crossing the grid edge.
    shapes[2].orientation = orientation;
    assert_eq!(
        compound(&mut backend, &shapes, position, orientation, true),
        Err(PhysicsError::InvalidValue)
    );
    let next = compound(&mut backend, &[SPHERE], [0.0; 3], IDENTITY, true).unwrap();
    assert_eq!(next.raw(), accepted.raw() + 1);
}

#[test]
fn hinge_reach_includes_the_last_offset_collision_and_preserves_pending_work() {
    let mut shapes = [SPHERE; 3];
    shapes[2].position[0] = 200.0;
    let mut actual = DartBackend::new([0.0; 3]).unwrap();
    let mut expected = DartBackend::new([0.0; 3]).unwrap();
    let body = compound(&mut actual, &shapes, [0.0; 3], IDENTITY, false).unwrap();
    let reference = compound(&mut expected, &shapes, [0.0; 3], IDENTITY, false).unwrap();
    for backend in [&mut actual, &mut expected] {
        backend
            .add_world_wrench(body, [1.0, 2.0, 3.0], [0.1, 0.2, 0.3])
            .unwrap();
    }
    let before = actual.state(body).unwrap();
    assert_eq!(
        actual.add_world_revolute(body, [0.0, 0.0, -200.0], [0.0, 0.0, 1.0]),
        Err(PhysicsError::InvalidJoint)
    );
    assert_eq!(actual.joint_state(JointId::from_raw(0)), None);
    assert_eq!(actual.state(body), Some(before));
    actual.step(DT).unwrap();
    expected.step(DT).unwrap();
    near(
        actual.state(body).unwrap(),
        expected.state(reference).unwrap(),
    );
}

#[test]
fn reset_reconstructs_all_shapes_and_removal_retires_the_whole_link() {
    let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let shapes = collisions();
    let fixed = compound(&mut backend, &shapes, [0.0; 3], IDENTITY, true).unwrap();
    let probe = single(&mut backend, [2.0, 0.0, 1.0]);
    let initial = backend.initial_state();
    let construction = [backend.state(fixed).unwrap(), backend.state(probe).unwrap()];
    for _ in 0..600 {
        backend.step(DT).unwrap();
    }
    let contacted = backend.state(probe).unwrap();
    assert!(contacted.velocity[2] > -5.0);
    let prepared = backend.prepare_initial(&initial).unwrap();
    assert!(backend.remove(fixed));
    assert!(backend.remove(probe));
    let extra = single(&mut backend, [20.0, 0.0, 0.0]);
    backend.apply_initial(prepared).unwrap();
    assert_eq!(backend.state(extra), None);
    assert_eq!(
        [backend.state(fixed).unwrap(), backend.state(probe).unwrap()],
        construction
    );
    for _ in 0..600 {
        backend.step(DT).unwrap();
    }
    near(backend.state(probe).unwrap(), contacted);
    backend.reset().unwrap();
    assert_eq!(
        [backend.state(fixed).unwrap(), backend.state(probe).unwrap()],
        construction
    );
    assert!(backend.remove(fixed));
    assert!(!backend.remove(fixed));
    for _ in 0..600 {
        backend.step(DT).unwrap();
    }
    assert!(backend.state(probe).unwrap().position[2] < -0.5);
    let later = compound(&mut backend, &[SPHERE], [30.0, 0.0, 0.0], IDENTITY, true).unwrap();
    assert!(later.raw() > extra.raw());
}

#[test]
fn unsupported_engine_refuses_before_allocating_a_body() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    assert_eq!(
        compound(&mut backend, &collisions(), [0.0; 3], IDENTITY, false),
        Err(PhysicsError::Unsupported {
            engine: "rapier",
            capability: "compound link collisions",
        })
    );
    let id = backend
        .spawn_link(
            BodySpec {
                shape: SPHERE.shape,
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            Some(INERTIA),
        )
        .unwrap();
    assert_eq!(id.raw(), 0);
}
