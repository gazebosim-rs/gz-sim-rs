// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    BodyId, BodySpec, DynamicBackend, JointId, JointVelocityMotor, PhysicsError, RapierBackend,
    Shape,
};
use std::time::Duration;

fn sphere(backend: &mut RapierBackend, mass: f64) -> BodyId {
    backend
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.2 },
            position: [0.0; 3],
            velocity: [0.0; 3],
            mass: Some(mass),
        })
        .unwrap()
}
fn hinge(
    backend: &mut RapierBackend,
    reduced: bool,
    parent: Option<BodyId>,
    child: BodyId,
) -> JointId {
    if reduced {
        backend
            .add_reduced_revolute(parent, child, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap()
    } else if let Some(parent) = parent {
        backend
            .add_revolute(parent, child, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap()
    } else {
        backend
            .add_world_revolute(child, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap()
    }
}
fn motor(target_velocity: f64, max_force: f64) -> Option<JointVelocityMotor> {
    Some(JointVelocityMotor {
        target_velocity,
        max_force,
    })
}
fn steps(backend: &mut RapierBackend, count: usize) {
    for _ in 0..count {
        backend.step(Duration::from_millis(1)).unwrap();
    }
}
fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
}

#[test]
fn native_velocity_servo_retains_torque_cap_and_tracks_brakes_reverses_and_coasts() {
    for reduced in [false, true] {
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let body = sphere(&mut backend, 1.0);
        let joint = hinge(&mut backend, reduced, None, body);
        let before = (backend.state(body), backend.joint_state(joint));
        backend
            .validate_joint_velocity_servo(joint, motor(100.0, 0.08))
            .unwrap();
        backend
            .set_joint_velocity_servo(joint, motor(100.0, 0.08))
            .unwrap();
        backend.step(Duration::ZERO).unwrap();
        assert_eq!((backend.state(body), backend.joint_state(joint)), before);
        steps(&mut backend, 1);
        let velocity = backend.joint_state(joint).unwrap().velocity;
        // Sphere I=.4*m*r²=.016. The .08Nm cap permits at most .005rad/s in1ms.
        assert!(velocity > 0.0 && velocity <= 0.005 + 1e-12, "{velocity}");
        backend
            .set_joint_velocity_servo(joint, motor(1.0, 100.0))
            .unwrap();
        steps(&mut backend, 10);
        close(backend.joint_state(joint).unwrap().velocity, 1.0);
        close(backend.state(body).unwrap().angular_velocity[1], 1.0);
        let moving = backend.joint_state(joint).unwrap();
        backend.set_joint_velocity_servo(joint, None).unwrap();
        steps(&mut backend, 50);
        close(
            backend.joint_state(joint).unwrap().velocity,
            moving.velocity,
        );
        assert!(backend.joint_state(joint).unwrap().position > moving.position);
        backend
            .set_joint_velocity_servo(joint, motor(0.0, 100.0))
            .unwrap();
        steps(&mut backend, 1);
        close(backend.joint_state(joint).unwrap().velocity, 0.0);
        backend
            .set_joint_velocity_servo(joint, motor(-1.0, 100.0))
            .unwrap();
        steps(&mut backend, 10);
        close(backend.joint_state(joint).unwrap().velocity, -1.0);
        assert_eq!(backend.state(body).unwrap().position, [0.0; 3]);
    }
}

#[test]
fn rejected_servo_commands_preserve_state_ids_and_pending_work_and_zero_force_is_unpowered() {
    for reduced in [false, true] {
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let body = sphere(&mut backend, 1.0);
        let joint = hinge(&mut backend, reduced, None, body);
        backend
            .set_joint_velocity_servo(joint, motor(0.0, 0.0))
            .unwrap();
        backend.add_joint_effort(joint, 1.0).unwrap();
        backend
            .add_world_wrench(body, [0.0; 3], [0.0, 2.0, 0.0])
            .unwrap();
        let before = (backend.state(body), backend.joint_state(joint));
        for invalid in [
            motor(f64::NAN, 1.0),
            motor(f64::INFINITY, 1.0),
            motor(f64::NEG_INFINITY, 1.0),
            motor(1.0, f64::NAN),
            motor(1.0, f64::INFINITY),
            motor(1.0, -1.0),
        ] {
            assert_eq!(
                backend.validate_joint_velocity_servo(joint, invalid),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!(
                backend.set_joint_velocity_servo(joint, invalid),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!((backend.state(body), backend.joint_state(joint)), before);
        }
        for value in [None, motor(f64::NAN, -1.0)] {
            assert_eq!(
                backend.validate_joint_velocity_servo(JointId::from_raw(99), value),
                Err(PhysicsError::MissingJoint)
            );
            assert_eq!(
                backend.set_joint_velocity_servo(JointId::from_raw(99), value),
                Err(PhysicsError::MissingJoint)
            );
        }
        backend.step(Duration::ZERO).unwrap();
        assert_eq!((backend.state(body), backend.joint_state(joint)), before);
        steps(&mut backend, 1);
        // Both queued torques survive every refusal and are consumed exactly once.
        close(backend.joint_state(joint).unwrap().velocity, 0.1875);
        steps(&mut backend, 1);
        close(backend.joint_state(joint).unwrap().velocity, 0.1875);
        assert_eq!(sphere(&mut backend, 1.0).raw(), 1);
    }
}

#[test]
fn both_resets_and_joint_or_body_removal_retire_the_opt_in_native_servo() {
    for reduced in [false, true] {
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let parent = sphere(&mut backend, 2.0);
        let child = sphere(&mut backend, 1.0);
        let joint = hinge(&mut backend, reduced, Some(parent), child);
        let before = (
            backend.state(parent),
            backend.state(child),
            backend.joint_state(joint),
        );
        backend
            .set_joint_velocity_servo(joint, motor(3.0, 1.0))
            .unwrap();
        let initial = backend.initial_state();
        steps(&mut backend, 100);
        assert!(backend.joint_state(joint).unwrap().velocity > 0.0);
        backend.reset().unwrap();
        assert_eq!(
            (
                backend.state(parent),
                backend.state(child),
                backend.joint_state(joint)
            ),
            before
        );
        steps(&mut backend, 10);
        close(backend.joint_state(joint).unwrap().velocity, 0.0);
        backend
            .set_joint_velocity_servo(joint, motor(-3.0, 1.0))
            .unwrap();
        steps(&mut backend, 100);
        assert!(backend.joint_state(joint).unwrap().velocity < 0.0);
        backend
            .apply_initial(backend.prepare_initial(&initial).unwrap())
            .unwrap();
        assert_eq!(
            (
                backend.state(parent),
                backend.state(child),
                backend.joint_state(joint)
            ),
            before
        );
        steps(&mut backend, 10);
        close(backend.joint_state(joint).unwrap().velocity, 0.0);
        backend
            .set_joint_velocity_servo(joint, motor(3.0, 1.0))
            .unwrap();
        assert!(backend.remove_joint(joint));
        assert_eq!(
            backend.set_joint_velocity_servo(joint, None),
            Err(PhysicsError::MissingJoint)
        );
        assert_eq!(backend.joint_state(joint), None);
        backend
            .apply_initial(backend.prepare_initial(&initial).unwrap())
            .unwrap();
        backend
            .set_joint_velocity_servo(joint, motor(3.0, 1.0))
            .unwrap();
        assert!(backend.remove(child));
        assert_eq!(
            backend.validate_joint_velocity_servo(joint, None),
            Err(PhysicsError::MissingJoint)
        );
        steps(&mut backend, 1);
        assert_eq!(backend.joint_state(joint), None);
        assert!(backend.state(parent).is_some());
    }
}
