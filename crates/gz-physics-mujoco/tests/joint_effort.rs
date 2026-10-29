// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, DynamicBackend, JointId, PhysicsError};
use gz_physics_mujoco::{BodySpec, MujocoBackend, RotationSpec, Shape};
use std::time::Duration;

fn body(b: &mut MujocoBackend, position: [f64; 3]) -> BodyId {
    b.spawn_link(
        BodySpec {
            shape: Shape::Sphere { radius: 0.2 },
            position,
            velocity: [0.0; 3],
            mass: Some(1.0),
        },
        RotationSpec::default(),
        None,
    )
    .unwrap()
}
fn hinge() -> (MujocoBackend, BodyId, JointId) {
    let mut b = MujocoBackend::new([0.0; 3]).unwrap();
    let child = body(&mut b, [0.0; 3]);
    let joint = b
        .add_world_revolute(child, [0.0; 3], [0.0, 5.0, 0.0])
        .unwrap();
    (b, child, joint)
}
fn tick(b: &mut MujocoBackend) {
    b.step(Duration::from_millis(1)).unwrap();
}
#[test]
fn effort_targets_the_hinge_dof_and_combines_with_world_wrench_once() {
    let mut b = MujocoBackend::new([0.0; 3]).unwrap();
    let free = body(&mut b, [3.0, 0.0, 0.0]);
    let child = body(&mut b, [0.0; 3]);
    let joint = b
        .add_world_revolute(child, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    // The free body occupies six coordinates before this hinge's scalar DOF.
    b.add_world_wrench(free, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
    b.add_world_wrench(child, [0.0; 3], [0.0, 0.5, 0.0])
        .unwrap();
    b.add_joint_effort(joint, 2.0).unwrap();
    b.add_joint_effort(joint, -1.0).unwrap();
    b.step(Duration::ZERO).unwrap();
    assert_eq!(b.joint_state(joint).unwrap().velocity, 0.0);
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.09375).abs() < 1e-12);
    assert!((b.joint_state(joint).unwrap().position - 0.00009375).abs() < 1e-12);
    assert!((b.state(free).unwrap().velocity[0] - 0.001).abs() < 1e-12);
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.09375).abs() < 1e-12);
    assert!((b.state(free).unwrap().velocity[0] - 0.001).abs() < 1e-12);
}
#[test]
fn invalid_inputs_and_overflow_preserve_the_admitted_effort() {
    let (mut b, child, joint) = hinge();
    b.add_joint_effort(joint, 2.0).unwrap();
    let before = (b.state(child), b.joint_state(joint));
    for value in [f64::NAN, f64::INFINITY, -f64::INFINITY] {
        assert_eq!(
            b.add_joint_effort(joint, value),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!((b.state(child), b.joint_state(joint)), before);
    }
    assert_eq!(
        b.add_joint_effort(JointId::from_raw(u64::MAX), 1.0),
        Err(PhysicsError::MissingJoint)
    );
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.125).abs() < 1e-12);
    b.add_joint_effort(joint, f64::MAX).unwrap();
    assert_eq!(
        b.add_joint_effort(joint, f64::MAX),
        Err(PhysicsError::InvalidValue)
    );
    b.add_joint_effort(joint, -f64::MAX).unwrap();
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.125).abs() < 1e-12);
}
#[test]
fn joint_body_retirement_and_reset_clear_pending_effort() {
    let (mut b, child, joint) = hinge();
    let initial = b.initial_state();
    b.add_joint_effort(joint, 2.0).unwrap();
    let prepared = b.prepare_initial(&initial).unwrap();
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.125).abs() < 1e-12);
    b.add_joint_effort(joint, 2.0).unwrap();
    b.apply_initial(prepared).unwrap();
    tick(&mut b);
    assert_eq!(b.joint_state(joint).unwrap().velocity, 0.0);
    b.add_joint_effort(joint, 2.0).unwrap();
    b.reset().unwrap();
    tick(&mut b);
    assert_eq!(b.joint_state(joint).unwrap().velocity, 0.0);
    b.add_joint_effort(joint, 2.0).unwrap();
    assert!(b.remove_joint(joint));
    tick(&mut b);
    assert_eq!(b.state(child).unwrap().angular_velocity, [0.0; 3]);
    assert_eq!(
        b.add_joint_effort(joint, 1.0),
        Err(PhysicsError::MissingJoint)
    );
    b.reset().unwrap();
    let next = b
        .add_world_revolute(child, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert_ne!(next, joint);
    b.add_joint_effort(next, 2.0).unwrap();
    assert!(b.remove(child));
    assert_eq!(
        b.add_joint_effort(next, 1.0),
        Err(PhysicsError::MissingJoint)
    );
    tick(&mut b);
}
#[test]
fn native_numeric_fault_keeps_effort_for_an_explicit_retry() {
    let (mut b, child, joint) = hinge();
    b.add_joint_effort(joint, 1e60).unwrap();
    assert!(matches!(
        b.step(Duration::from_millis(1)),
        Err(PhysicsError::Native { .. })
    ));
    // MuJoCo retains its existing fatal-warning reset behavior. Only the queued
    // effort survives; cancelling it must not inject a new negative huge force.
    assert_eq!(b.state(child).unwrap().angular_velocity, [0.0; 3]);
    b.add_joint_effort(joint, -1e60).unwrap();
    b.add_joint_effort(joint, 2.0).unwrap();
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.125).abs() < 1e-12);
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.125).abs() < 1e-12);
}

#[test]
fn effort_survives_unrelated_topology_transfer_and_follows_changed_dof_address() {
    let (mut b, first, a) = hinge();
    let second = body(&mut b, [3.0, 0.0, 0.0]);
    let c = b
        .add_world_revolute(second, [3.0, 0.0, 0.0], [0.0, 1.0, 0.0])
        .unwrap();
    b.add_joint_effort(a, 2.0).unwrap();
    b.add_joint_effort(c, -3.0).unwrap();
    tick(&mut b);
    assert!((b.joint_state(a).unwrap().velocity - 0.125).abs() < 1e-12);
    assert!((b.joint_state(c).unwrap().velocity + 0.1875).abs() < 1e-12);
    b.add_joint_effort(c, 1.0).unwrap();
    assert!(b.remove_joint(a));
    // The newly free first body now occupies six coordinates before the hinge.
    tick(&mut b);
    assert!((b.joint_state(c).unwrap().velocity + 0.125).abs() < 1e-12);
    assert!((b.state(first).unwrap().angular_velocity[1] - 0.125).abs() < 1e-12);
    tick(&mut b);
    assert!((b.joint_state(c).unwrap().velocity + 0.125).abs() < 1e-12);
}

#[test]
fn generalized_effort_is_work_conjugate_for_a_rotated_offset_hinge() {
    let mut b = MujocoBackend::new([0.0; 3]).unwrap();
    let position = [0.3, 0.2, -0.1];
    let axis = [1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0];
    let child = b
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position,
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec {
                orientation: [
                    0.0,
                    0.0,
                    std::f64::consts::FRAC_1_SQRT_2,
                    std::f64::consts::FRAC_1_SQRT_2,
                ],
                angular_velocity: [0.0; 3],
            },
            None,
        )
        .unwrap();
    let joint = b.add_world_revolute(child, [0.0; 3], axis).unwrap();
    b.add_joint_effort(joint, 1.5).unwrap();
    b.add_world_wrench(child, [0.0; 3], axis.map(|v| 0.5 * v))
        .unwrap();
    tick(&mut b);
    let projection: f64 = (0..3).map(|i| axis[i] * position[i]).sum();
    let moment = 0.016 + position.into_iter().map(|v| v * v).sum::<f64>() - projection * projection;
    let expected = 2.0 * 0.001 / moment;
    assert!((b.joint_state(joint).unwrap().velocity - expected).abs() < 1e-12);
    assert!((b.joint_state(joint).unwrap().position - expected * 0.001).abs() < 1e-12);
    for (actual, unit) in b
        .state(child)
        .unwrap()
        .angular_velocity
        .into_iter()
        .zip(axis)
    {
        assert!((actual - unit * expected).abs() < 1e-12);
    }
}

#[test]
fn foreign_reset_errors_preserve_pending_effort() {
    let (mut b, _, joint) = hinge();
    let (other, _, _) = hinge();
    let initial = other.initial_state();
    let prepared = other.prepare_initial(&initial).unwrap();
    b.add_joint_effort(joint, 2.0).unwrap();
    assert!(matches!(
        b.prepare_initial(&initial),
        Err(PhysicsError::ForeignInitialState)
    ));
    assert_eq!(
        b.apply_initial(prepared),
        Err(PhysicsError::ForeignInitialState)
    );
    tick(&mut b);
    assert!((b.joint_state(joint).unwrap().velocity - 0.125).abs() < 1e-12);
}

#[test]
fn effort_pulses_match_real_cpp_sdk_without_actuators() {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/joint_effort.json")).unwrap();
    assert_eq!(fixture["version"], "3.11.0");
    let mut b = MujocoBackend::new([0.0; 3]).unwrap();
    let free = body(&mut b, [3.0, 0.0, 0.0]);
    let child = body(&mut b, [0.0; 3]);
    let joint = b
        .add_world_revolute(child, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    let rows = fixture["observations"].as_array().unwrap();
    assert_eq!(rows.len(), 6);
    for row in rows {
        b.add_joint_effort(joint, row["effort"].as_f64().unwrap())
            .unwrap();
        tick(&mut b);
        let actual = b.joint_state(joint).unwrap();
        assert!((actual.position - row["q"].as_f64().unwrap()).abs() < 1e-12);
        assert!((actual.velocity - row["dq"].as_f64().unwrap()).abs() < 1e-12);
        assert_eq!(b.state(free).unwrap().velocity, [0.0; 3]);
        assert_eq!(b.state(free).unwrap().angular_velocity, [0.0; 3]);
    }
}
