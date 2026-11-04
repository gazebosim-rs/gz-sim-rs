// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    BodyId, BodySpec, DynamicBackend, JointId, JointPositionServo, JointVelocityMotor,
    PhysicsError, RapierBackend, Shape,
};
use std::time::Duration;
const DT: Duration = Duration::from_millis(1);
fn setup(reduced: bool) -> (RapierBackend, BodyId, JointId) {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let body = backend
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.2 },
            position: [0.0; 3],
            velocity: [0.0; 3],
            mass: Some(1.0),
        })
        .unwrap();
    let joint = if reduced {
        backend
            .add_reduced_revolute(None, body, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap()
    } else {
        backend
            .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap()
    };
    (backend, body, joint)
}
fn request(target_position: f64, max_velocity: f64, max_force: f64) -> Option<JointPositionServo> {
    Some(JointPositionServo {
        target_position,
        max_velocity,
        max_force,
    })
}

#[test]
fn damped_reduced_position_servo_tracks_both_turn_directions_without_teleport_or_impulse_aliasing()
{
    let (mut backend, body, joint) = setup(true);
    let before = (backend.state(body), backend.joint_state(joint));
    backend
        .validate_joint_position_servo(joint, request(0.7, 2.0, 3.0), DT)
        .unwrap();
    backend
        .set_joint_position_servo(joint, request(0.7, 2.0, 3.0), DT)
        .unwrap();
    backend.step(Duration::ZERO).unwrap();
    assert_eq!((backend.state(body), backend.joint_state(joint)), before);
    for target in [
        0.7,
        std::f64::consts::TAU + 0.3,
        -std::f64::consts::TAU - 0.2,
    ] {
        for _ in 0..10_000 {
            // The ultimate target belongs to this caller; reissue for each actual dt.
            backend
                .set_joint_position_servo(joint, request(target, 2.0, 3.0), DT)
                .unwrap();
            backend.step(DT).unwrap();
        }
        let actual = backend.joint_state(joint).unwrap();
        assert!(
            (actual.position - target).abs() < 0.01,
            "{actual:?}, target {target}"
        );
        assert!(actual.velocity.abs() < 0.01, "{actual:?}");
        let orientation = backend.state(body).unwrap().orientation;
        let expected = [0.0, (target * 0.5).sin(), 0.0, (target * 0.5).cos()];
        let dot: f64 = orientation
            .into_iter()
            .zip(expected)
            .map(|(a, b)| a * b)
            .sum();
        assert!(
            (dot.abs() - 1.0).abs() < 1e-4,
            "actual {orientation:?}, target {target}"
        );
        assert_eq!(backend.state(body).unwrap().position, [0.0; 3]);
    }
    let mut impulse = setup(false);
    let error = PhysicsError::Unsupported {
        engine: "rapier",
        capability: "continuous position servos on impulse joints",
    };
    assert_eq!(
        impulse.0.validate_joint_position_servo(
            impulse.2,
            request(std::f64::consts::TAU, 2.0, 3.0),
            DT
        ),
        Err(error.clone())
    );
    assert_eq!(
        impulse
            .0
            .set_joint_position_servo(impulse.2, request(std::f64::consts::TAU, 2.0, 3.0), DT),
        Err(error)
    );
    impulse
        .0
        .set_joint_position_servo(impulse.2, None, Duration::ZERO)
        .unwrap();
}

#[test]
fn native_position_torque_cap_disable_and_both_resets_preserve_explicit_runtime_ownership() {
    let (mut backend, body, joint) = setup(true);
    let before = (backend.state(body), backend.joint_state(joint));
    backend
        .set_joint_position_servo(joint, request(100.0, 100.0, 0.08), DT)
        .unwrap();
    let initial = backend.initial_state();
    backend.step(DT).unwrap();
    // Sphere I=.016, so the same .08Nm cap permits at most .005rad/s in1ms.
    let velocity = backend.joint_state(joint).unwrap().velocity;
    assert!(velocity > 0.0 && velocity <= 0.005 + 1e-12, "{velocity}");
    backend
        .set_joint_position_servo(joint, None, Duration::MAX)
        .unwrap();
    for _ in 0..10 {
        backend.step(DT).unwrap();
    }
    assert!((backend.joint_state(joint).unwrap().velocity - velocity).abs() < 1e-12);
    backend.reset().unwrap();
    assert_eq!((backend.state(body), backend.joint_state(joint)), before);
    for _ in 0..10 {
        backend.step(DT).unwrap();
    }
    assert!(backend.joint_state(joint).unwrap().velocity.abs() < 1e-12);
    backend
        .set_joint_position_servo(joint, request(-1.0, 2.0, 3.0), DT)
        .unwrap();
    backend.step(DT).unwrap();
    backend
        .apply_initial(backend.prepare_initial(&initial).unwrap())
        .unwrap();
    assert_eq!((backend.state(body), backend.joint_state(joint)), before);
    backend.step(DT).unwrap();
    assert!(backend.joint_state(joint).unwrap().velocity.abs() < 1e-12);
    backend
        .set_joint_position_servo(joint, request(1.0, 2.0, 3.0), DT)
        .unwrap();
    assert!(backend.remove_joint(joint));
    assert_eq!(
        backend.set_joint_position_servo(joint, None, Duration::ZERO),
        Err(PhysicsError::MissingJoint)
    );
    backend
        .apply_initial(backend.prepare_initial(&initial).unwrap())
        .unwrap();
    backend
        .set_joint_position_servo(joint, request(1.0, 2.0, 3.0), DT)
        .unwrap();
    assert!(backend.remove(body));
    assert_eq!(
        backend.validate_joint_position_servo(joint, None, Duration::ZERO),
        Err(PhysicsError::MissingJoint)
    );
}

#[test]
fn invalid_position_preparation_preserves_prior_velocity_mode_and_pending_effort() {
    let (mut backend, body, joint) = setup(true);
    backend
        .set_joint_velocity_motor(
            joint,
            Some(JointVelocityMotor {
                target_velocity: 0.0,
                max_force: 0.0,
            }),
        )
        .unwrap();
    backend.add_joint_effort(joint, 1.0).unwrap();
    backend
        .add_world_wrench(body, [0.0; 3], [0.0, 2.0, 0.0])
        .unwrap();
    let before = (backend.state(body), backend.joint_state(joint));
    for servo in [
        request(f64::NAN, 2.0, 3.0),
        request(f64::INFINITY, 2.0, 3.0),
        request(1.0, 0.0, 3.0),
        request(1.0, f64::INFINITY, 3.0),
        request(1.0, 2.0, -1.0),
        request(1.0, 2.0, f64::NAN),
    ] {
        assert_eq!(
            backend.validate_joint_position_servo(joint, servo, DT),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(
            backend.set_joint_position_servo(joint, servo, DT),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!((backend.state(body), backend.joint_state(joint)), before);
    }
    assert_eq!(
        backend.set_joint_position_servo(joint, request(1.0, 2.0, 3.0), Duration::ZERO),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        backend.set_joint_position_servo(
            joint,
            request(1.0, 2.0, f64::MAX),
            Duration::from_secs(2)
        ),
        Err(PhysicsError::InvalidValue)
    );
    backend.step(Duration::ZERO).unwrap();
    assert_eq!((backend.state(body), backend.joint_state(joint)), before);
    backend.step(DT).unwrap();
    assert!((backend.joint_state(joint).unwrap().velocity - 0.1875).abs() < 1e-12);
    backend.step(DT).unwrap();
    assert!((backend.joint_state(joint).unwrap().velocity - 0.1875).abs() < 1e-12);
}
