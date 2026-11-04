// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn setup(reduced: bool, moving_parent: bool) -> (RapierBackend, Option<BodyId>, BodyId, JointId) {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let spec = BodySpec {
        shape: Shape::Sphere { radius: 0.2 },
        position: [1.0, 2.0, 3.0],
        velocity: [0.0; 3],
        mass: Some(1.0),
    };
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let rotation = RotationSpec {
        orientation: [h, 0.0, 0.0, h],
        ..Default::default()
    };
    let parent = moving_parent.then(|| backend.spawn_link(spec, rotation, None).unwrap());
    let child = backend.spawn_link(spec, rotation, None).unwrap();
    let joint = if reduced {
        backend
            .add_reduced_revolute(parent, child, spec.position, [0.0, 1.0, 0.0])
            .unwrap()
    } else if let Some(parent) = parent {
        backend
            .add_revolute(parent, child, spec.position, [0.0, 1.0, 0.0])
            .unwrap()
    } else {
        backend
            .add_world_revolute(child, spec.position, [0.0, 1.0, 0.0])
            .unwrap()
    };
    (backend, parent, child, joint)
}
fn native(backend: &RapierBackend, joint: JointId) -> &GenericJoint {
    match backend.joints.records[&joint].handle {
        Handle::Impulse(handle) => &backend.world.impulse_joints.get(handle).unwrap().data,
        Handle::Reduced(handle) => {
            let (body, link) = backend.world.multibody_joints.get(handle).unwrap();
            &body.link(link).unwrap().joint.data
        }
    }
}
fn motor(target_velocity: f64, max_force: f64) -> Option<JointVelocityMotor> {
    Some(JointVelocityMotor {
        target_velocity,
        max_force,
    })
}

#[test]
fn explicit_native_zero_compliance_preserves_legacy_mode_and_applies_parent_reaction() {
    for reduced in [false, true] {
        let (mut backend, parent, child, joint) = setup(reduced, true);
        let parent = parent.unwrap();
        backend
            .set_joint_velocity_motor(joint, motor(2.0, 0.1))
            .unwrap();
        let before = (
            backend.state(parent),
            backend.state(child),
            backend.joint_state(joint),
        );
        let work = (
            backend.pending_wrenches.clone(),
            backend.joints.efforts.clone(),
        );
        let ids = (backend.next_id, backend.joints.next);
        backend
            .validate_joint_velocity_servo(joint, motor(2.0, 0.1))
            .unwrap();
        assert_eq!(
            native(&backend, joint)
                .motor(JointAxis::AngX)
                .unwrap()
                .damping,
            1.0
        );
        backend
            .set_joint_velocity_servo(joint, motor(2.0, 0.1))
            .unwrap();
        let data = native(&backend, joint);
        let value = data.motor(JointAxis::AngX).unwrap();
        assert_eq!(
            (
                value.target_vel,
                value.max_force,
                value.stiffness,
                value.damping
            ),
            (2.0, 0.1, 0.0, 0.0)
        );
        assert_eq!(value.model, MotorModel::AccelerationBased);
        assert_eq!(
            value
                .model
                .combine_coefficients(0.001, value.stiffness, value.damping),
            (0.0, 0.0, 0.0)
        );
        assert_eq!(data.motor_axes, JointAxesMask::ANG_X);
        assert_eq!(
            (
                backend.state(parent),
                backend.state(child),
                backend.joint_state(joint)
            ),
            before
        );
        assert_eq!(
            (
                backend.pending_wrenches.clone(),
                backend.joints.efforts.clone()
            ),
            work
        );
        assert_eq!((backend.next_id, backend.joints.next), ids);
        let axis = *backend.world.bodies[backend.bodies[&parent].0].rotation()
            * data.local_frame1.rotation
            * Vector::X;
        assert!((axis - Vector::Y).length() < 1e-12);
        assert!(matches!(
            backend.validate_joint_effort(joint, 1.0),
            Err(PhysicsError::Unsupported {
                capability: "moving-parent joint effort commands",
                ..
            })
        ));
        backend.step(Duration::from_millis(1)).unwrap();
        let parent_velocity = Vector::from(backend.state(parent).unwrap().angular_velocity);
        let child_velocity = Vector::from(backend.state(child).unwrap().angular_velocity);
        assert!(parent_velocity.dot(axis) < 0.0 && child_velocity.dot(axis) > 0.0);
        assert!(
            (backend.joint_state(joint).unwrap().velocity
                - (child_velocity - parent_velocity).dot(axis))
            .abs()
                < 1e-12
        );
        assert!((parent_velocity + child_velocity).length() < 1e-12);
        let after = (
            backend.state(parent),
            backend.state(child),
            backend.joint_state(joint),
        );
        let value = native(&backend, joint).motor(JointAxis::AngX).unwrap();
        let saved = (
            value.target_vel,
            value.max_force,
            value.stiffness,
            value.damping,
            value.impulse,
            value.model,
        );
        for invalid in [motor(f64::NAN, 0.1), motor(2.0, -1.0)] {
            assert_eq!(
                backend.validate_joint_velocity_servo(joint, invalid),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!(
                backend.set_joint_velocity_servo(joint, invalid),
                Err(PhysicsError::InvalidValue)
            );
            let value = native(&backend, joint).motor(JointAxis::AngX).unwrap();
            assert_eq!(
                (
                    value.target_vel,
                    value.max_force,
                    value.stiffness,
                    value.damping,
                    value.impulse,
                    value.model
                ),
                saved
            );
            assert_eq!(
                (
                    backend.state(parent),
                    backend.state(child),
                    backend.joint_state(joint)
                ),
                after
            );
            assert_eq!((backend.next_id, backend.joints.next), ids);
        }
        backend.set_joint_velocity_servo(joint, None).unwrap();
        assert_eq!(native(&backend, joint).motor_axes, JointAxesMask::empty());
        assert_eq!(
            native(&backend, joint).motors[JointAxis::AngX as usize].impulse,
            0.0
        );
        backend
            .set_joint_velocity_motor(joint, motor(-2.0, 0.2))
            .unwrap();
        assert_eq!(
            native(&backend, joint)
                .motor(JointAxis::AngX)
                .unwrap()
                .damping,
            1.0
        );
    }
}

#[test]
fn opt_in_servo_step_overflow_preserves_native_mode_construction_ids_and_pending_work() {
    for reduced in [false, true] {
        let (mut backend, _, child, joint) = setup(reduced, false);
        backend.add_joint_effort(joint, 1.0).unwrap();
        backend
            .add_world_wrench(child, [0.0; 3], [0.0, 2.0, 0.0])
            .unwrap();
        let before = (backend.state(child), backend.joint_state(joint));
        let work = (
            backend.pending_wrenches.clone(),
            backend.joints.efforts.clone(),
        );
        let ids = (backend.next_id, backend.joints.next);
        for command in [motor(f64::MAX, 0.1), motor(0.0, f64::MAX)] {
            backend.set_joint_velocity_servo(joint, command).unwrap();
            backend.step(Duration::ZERO).unwrap();
            assert_eq!(
                backend.step(Duration::from_secs(2)),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!((backend.state(child), backend.joint_state(joint)), before);
            assert_eq!(
                (
                    backend.pending_wrenches.clone(),
                    backend.joints.efforts.clone()
                ),
                work
            );
            assert_eq!((backend.next_id, backend.joints.next), ids);
            assert!(backend.joints.construction_open());
            let value = native(&backend, joint).motor(JointAxis::AngX).unwrap();
            assert_eq!(
                (value.target_vel, value.max_force, value.damping),
                (
                    command.unwrap().target_velocity,
                    command.unwrap().max_force,
                    0.0
                )
            );
            assert_eq!(
                backend.world.bodies[backend.bodies[&child].0].user_torque(),
                Vector::new(0.0, 2.0, 0.0)
            );
        }
        backend.set_joint_velocity_servo(joint, None).unwrap();
        backend.step(Duration::from_millis(1)).unwrap();
        assert!((backend.joint_state(joint).unwrap().velocity - 0.1875).abs() < 1e-12);
        assert!(backend.pending_wrenches.is_empty() && backend.joints.efforts.is_empty());
        backend.step(Duration::from_millis(1)).unwrap();
        assert!((backend.joint_state(joint).unwrap().velocity - 0.1875).abs() < 1e-12);
    }
}
