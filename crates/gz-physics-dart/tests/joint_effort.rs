// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, JointId, PhysicsError, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use std::time::Duration;

fn hinge() -> (DartBackend, BodyId, JointId) {
    let mut b = DartBackend::new([0.0; 3]).unwrap();
    let body = b
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    let joint = b
        .add_world_revolute(body, [0.0; 3], [0.0, 5.0, 0.0])
        .unwrap();
    (b, body, joint)
}
fn tick(b: &mut DartBackend) {
    b.step(Duration::from_millis(1)).unwrap();
}
#[test]
fn additive_effort_has_analytic_sign_and_consumes_once_after_zero_step() {
    let (mut b, body, joint) = hinge();
    b.add_joint_effort(joint, 2.0).unwrap();
    b.add_joint_effort(joint, -0.5).unwrap();
    assert_eq!(b.joint_state(joint).unwrap().velocity, 0.0);
    b.step(Duration::ZERO).unwrap();
    assert_eq!(b.joint_state(joint).unwrap().position, 0.0);
    tick(&mut b);
    // I=2*m*r^2/5=.016; semi-implicit integration advances with the new rate.
    assert!((b.joint_state(joint).unwrap().velocity - 0.09375).abs() < 1e-12);
    assert!((b.joint_state(joint).unwrap().position - 0.00009375).abs() < 1e-12);
    assert!((b.state(body).unwrap().angular_velocity[1] - 0.09375).abs() < 1e-12);
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.09375).abs() < 1e-12);
    b.add_joint_effort(joint, -3.0).unwrap();
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity + 0.09375).abs() < 1e-12);
}
#[test]
fn invalid_effort_and_accumulation_preserve_admitted_command() {
    let (mut b, body, joint) = hinge();
    b.add_joint_effort(joint, 2.0).unwrap();
    let before = (b.state(body), b.joint_state(joint));
    for torque in [f64::NAN, f64::INFINITY, -f64::INFINITY, 1e61, -1e61] {
        assert_eq!(
            b.add_joint_effort(joint, torque),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!((b.state(body), b.joint_state(joint)), before);
    }
    assert_eq!(
        b.add_joint_effort(JointId::from_raw(u64::MAX), 1.0),
        Err(PhysicsError::MissingJoint)
    );
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.125).abs() < 1e-12);
    b.add_joint_effort(joint, 1e60).unwrap();
    assert_eq!(
        b.add_joint_effort(joint, 1e60),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        b.add_joint_effort(joint, -2e60),
        Err(PhysicsError::InvalidValue)
    );
    b.add_joint_effort(joint, -1e60).unwrap();
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.125).abs() < 1e-12);
}
#[test]
fn both_reset_paths_discard_pending_effort_without_capturing_it() {
    let (mut b, _, joint) = hinge();
    let initial = b.initial_state();
    b.add_joint_effort(joint, 2.0).unwrap();
    b.reset().unwrap();
    tick(&mut b);
    assert_eq!(b.joint_state(joint).unwrap().velocity, 0.0);
    b.add_joint_effort(joint, 2.0).unwrap();
    let prepared = b.prepare_initial(&initial).unwrap();
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.125).abs() < 1e-12);
    b.add_joint_effort(joint, 3.0).unwrap();
    b.apply_initial(prepared).unwrap();
    tick(&mut b);
    assert_eq!(b.joint_state(joint).unwrap().velocity, 0.0);
}
#[test]
fn joint_and_body_retirement_discard_pending_effort_without_reusing_ids() {
    let (mut b, body, joint) = hinge();
    b.add_joint_effort(joint, 2.0).unwrap();
    assert!(b.remove_joint(joint));
    tick(&mut b);
    assert_eq!(b.state(body).unwrap().angular_velocity, [0.0; 3]);
    assert_eq!(
        b.add_joint_effort(joint, 1.0),
        Err(PhysicsError::MissingJoint)
    );
    b.reset().unwrap();
    let next = b
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert_ne!(next, joint);
    tick(&mut b);
    assert_eq!(b.joint_state(next).unwrap().velocity, 0.0);
    b.add_joint_effort(next, 2.0).unwrap();
    assert!(b.remove(body));
    assert_eq!(
        b.add_joint_effort(next, 1.0),
        Err(PhysicsError::MissingJoint)
    );
    tick(&mut b);
}
#[test]
fn native_refusal_preserves_effort_and_unrelated_body_wrench_for_retry() {
    let (mut b, body, joint) = hinge();
    let free = b
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [3.0, 0.0, 0.0],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    b.add_world_wrench(free, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
    b.add_joint_effort(joint, 1e60).unwrap();
    let before = (b.state(body), b.state(free), b.joint_state(joint));
    assert!(matches!(
        b.step(Duration::from_secs(1)),
        Err(PhysicsError::Native { .. })
    ));
    assert_eq!((b.state(body), b.state(free), b.joint_state(joint)), before);
    b.add_joint_effort(joint, -1e60).unwrap();
    b.add_joint_effort(joint, 2.0).unwrap();
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.125).abs() < 1e-12);
    assert!((b.state(free).unwrap().velocity[0] - 0.001).abs() < 1e-12);
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.125).abs() < 1e-12);
    assert!((b.state(free).unwrap().velocity[0] - 0.001).abs() < 1e-12);
}
#[test]
fn rapier_effort_is_deferred_with_precise_input_errors() {
    let mut b = gz_physics_rapier::RapierBackend::new([0.0; 3]).unwrap();
    let body = b
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec {
                orientation: [0.0, 0.0, 0.0, 1.0],
                angular_velocity: [0.0, 10.0, 0.0],
            },
            None,
        )
        .unwrap();
    let joint = b
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    let before = (b.state(body), b.joint_state(joint));
    for id in [joint, JointId::from_raw(u64::MAX)] {
        for value in [0.0, 5.0, f64::NAN] {
            let engine: &mut dyn DynamicBackend = &mut b;
            let result = engine.add_joint_effort(id, value);
            if id == JointId::from_raw(u64::MAX) {
                assert_eq!(result, Err(PhysicsError::MissingJoint));
            } else if value.is_nan() {
                assert_eq!(result, Err(PhysicsError::InvalidValue));
            } else {
                assert_eq!(result, Ok(()));
            }
            assert_eq!((engine.state(body), engine.joint_state(joint)), before);
        }
    }
    b.step(Duration::from_millis(1)).unwrap();
    assert!(b.joint_state(joint).unwrap().position > 0.005);
    assert!((b.joint_state(joint).unwrap().velocity - 10.3125).abs() < 1e-12);
}

#[test]
fn tilted_offset_hinge_effort_matches_work_conjugate_inertia_and_world_twist() {
    use nalgebra::{Unit, UnitQuaternion, Vector3};
    let axis = Unit::new_normalize(Vector3::new(0.3, -0.6, 0.7));
    let position = Vector3::new(0.3, 0.2, -0.1);
    let rotation = UnitQuaternion::from_euler_angles(0.4, 0.2, -0.3);
    let q = rotation.quaternion();
    let mut b = DartBackend::new([0.0; 3]).unwrap();
    let body = b
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: position.into(),
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec {
                orientation: [q.i, q.j, q.k, q.w],
                angular_velocity: [0.0; 3],
            },
            None,
        )
        .unwrap();
    let joint = b
        .add_world_revolute(body, [0.0; 3], axis.into_inner().into())
        .unwrap();
    // A world torque does work only through its projection on the hinge axis.
    b.add_world_wrench(body, [0.0; 3], axis.map(|v| -0.5 * v).into())
        .unwrap();
    b.add_joint_effort(joint, 2.0).unwrap();
    tick(&mut b);
    let moment = 0.016 + position.norm_squared() - axis.dot(&position).powi(2);
    let velocity = 1.5 * 0.001 / moment;
    let angle = velocity * 0.001;
    let state = b.state(body).unwrap();
    let expected_position = UnitQuaternion::from_axis_angle(&axis, angle) * position;
    let expected_angular = axis.into_inner() * velocity;
    assert!((b.joint_state(joint).unwrap().velocity - velocity).abs() < 1e-12);
    assert!((b.joint_state(joint).unwrap().position - angle).abs() < 1e-12);
    assert!((Vector3::from(state.position) - expected_position).norm() < 1e-12);
    assert!((Vector3::from(state.angular_velocity) - expected_angular).norm() < 1e-12);
    assert!(
        (Vector3::from(state.velocity) - expected_angular.cross(&expected_position)).norm() < 1e-12
    );
}

#[test]
fn combined_world_and_joint_torque_overflow_is_atomic_and_retryable() {
    use gz_physics::Inertia;
    let mut b = DartBackend::new([0.0; 3]).unwrap();
    let body = b
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1e60),
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [1e60, 1e60, 1e60, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let joint = b
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    b.add_world_wrench(body, [0.0; 3], [0.0, 1e60, 0.0])
        .unwrap();
    b.add_joint_effort(joint, 1e60).unwrap();
    let before = (b.state(body), b.joint_state(joint));
    assert_eq!(
        b.step(Duration::from_millis(1)),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!((b.state(body), b.joint_state(joint)), before);
    b.add_joint_effort(joint, -1e60).unwrap();
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.001).abs() < 1e-12);
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.001).abs() < 1e-12);
}

#[test]
fn generalized_effort_matches_real_cpp_plugin_force_pulses() {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/joint_effort.json")).unwrap();
    assert_eq!(fixture["dart_version"], "6.13.2");
    let (mut b, _, joint) = hinge();
    let rows = fixture["observations"].as_array().unwrap();
    assert_eq!(rows.len(), 6);
    for row in rows {
        b.add_joint_effort(joint, row["effort"].as_f64().unwrap())
            .unwrap();
        tick(&mut b);
        let actual = b.joint_state(joint).unwrap();
        assert!((actual.position - row["q"].as_f64().unwrap()).abs() < 1e-12);
        assert!((actual.velocity - row["dq"].as_f64().unwrap()).abs() < 1e-12);
    }
}
