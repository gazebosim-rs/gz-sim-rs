// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn setup(reduced: bool, parent: bool) -> (RapierBackend, Option<BodyId>, BodyId, JointId) {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let spec = BodySpec {
        shape: Shape::Sphere { radius: 0.2 },
        position: [1.0, 2.0, 3.0],
        velocity: [0.0; 3],
        mass: Some(1.0),
    };
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let orientation = RotationSpec {
        orientation: [h, 0.0, 0.0, h],
        ..Default::default()
    };
    let parent = parent.then(|| backend.spawn_link(spec, orientation, None).unwrap());
    let child = backend.spawn_link(spec, orientation, None).unwrap();
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
fn native(backend: &RapierBackend, id: JointId) -> &GenericJoint {
    data(&backend.world, backend.joints.records[&id].handle)
}
fn command(velocity: f64, force: f64) -> Option<JointVelocityMotor> {
    Some(JointVelocityMotor {
        target_velocity: velocity,
        max_force: force,
    })
}

#[test]
fn both_native_joint_kinds_own_the_motor_in_the_existing_nonidentity_joint_frame() {
    for reduced in [false, true] {
        let (mut b, parent, child, id) = setup(reduced, true);
        let parent = parent.unwrap();
        let before = (b.state(parent), b.state(child), b.joint_state(id));
        let pending_before = (b.pending_wrenches.clone(), b.joints.efforts.clone());
        assert!(native(&b, id).motor(JointAxis::AngX).is_none());
        b.validate_joint_velocity_motor(id, command(2.0, 0.1))
            .unwrap();
        assert!(native(&b, id).motor(JointAxis::AngX).is_none());
        b.set_joint_velocity_motor(id, command(2.0, 0.1)).unwrap();
        assert_eq!((b.state(parent), b.state(child), b.joint_state(id)), before);
        assert_eq!(
            (b.pending_wrenches.clone(), b.joints.efforts.clone()),
            pending_before
        );
        let joint = native(&b, id);
        assert_eq!(joint.motor_axes, JointAxesMask::ANG_X);
        let motor = joint.motor(JointAxis::AngX).unwrap();
        assert_eq!(
            (
                motor.target_vel,
                motor.max_force,
                motor.stiffness,
                motor.damping
            ),
            (2.0, 0.1, 0.0, 1.0)
        );
        assert_eq!(motor.model, MotorModel::AccelerationBased);
        let axis = *b.world.bodies[b.bodies[&parent].0].rotation()
            * joint.local_frame1.rotation
            * Vector::X;
        assert!((axis - Vector::Y).length() < 1e-12);
        for invalid in [command(f64::NAN, 0.1), command(2.0, -1.0)] {
            assert_eq!(
                b.validate_joint_velocity_motor(id, invalid),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!(
                b.set_joint_velocity_motor(id, invalid),
                Err(PhysicsError::InvalidValue)
            );
            let motor = native(&b, id).motor(JointAxis::AngX).unwrap();
            assert_eq!(
                (motor.target_vel, motor.max_force, motor.impulse),
                (2.0, 0.1, 0.0)
            );
            assert_eq!((b.state(parent), b.state(child), b.joint_state(id)), before);
        }
        b.step(Duration::from_millis(1)).unwrap();
        let child_velocity = Vector::from(b.state(child).unwrap().angular_velocity);
        let parent_velocity = Vector::from(b.state(parent).unwrap().angular_velocity);
        let sample = b.joint_state(id).unwrap();
        assert!(sample.velocity > 0.0 && parent_velocity.dot(axis) < 0.0);
        assert!((sample.velocity - (child_velocity - parent_velocity).dot(axis)).abs() < 1e-12);
        b.set_joint_velocity_motor(id, None).unwrap();
        assert_eq!(native(&b, id).motor_axes, JointAxesMask::empty());
        assert_eq!(native(&b, id).motors[JointAxis::AngX as usize].impulse, 0.0);
        b.set_joint_velocity_motor(id, command(-2.0, 0.0)).unwrap();
        assert_eq!(native(&b, id).motor(JointAxis::AngX).unwrap().impulse, 0.0);
        assert_eq!(
            native(&b, id).motor(JointAxis::AngX).unwrap().max_force,
            0.0
        );
        b.reset().unwrap();
        assert!(native(&b, id).motor(JointAxis::AngX).is_none());
    }
}

#[test]
fn unrepresentable_motor_step_is_atomic_and_preserves_pending_world_and_joint_work() {
    for reduced in [false, true] {
        let (mut b, _, child, id) = setup(reduced, false);
        b.add_joint_effort(id, 1.0).unwrap();
        b.add_world_wrench(child, [0.0; 3], [0.0, 2.0, 0.0])
            .unwrap();
        let before = (b.state(child), b.joint_state(id));
        let work = (b.pending_wrenches.clone(), b.joints.efforts.clone());
        let next = (b.next_id, b.joints.next);
        for invalid_step in [command(f64::MAX, 0.1), command(0.0, f64::MAX)] {
            b.set_joint_velocity_motor(id, invalid_step).unwrap();
            b.step(Duration::ZERO).unwrap();
            assert_eq!(
                b.step(Duration::from_secs(2)),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!((b.state(child), b.joint_state(id)), before);
            assert_eq!((b.pending_wrenches.clone(), b.joints.efforts.clone()), work);
            assert_eq!((b.next_id, b.joints.next), next);
            assert!(b.joints.construction_open());
            assert_eq!(
                native(&b, id).motor(JointAxis::AngX).unwrap().target_vel,
                invalid_step.unwrap().target_velocity
            );
            assert_eq!(
                b.world.bodies[b.bodies[&child].0].user_torque(),
                Vector::new(0.0, 2.0, 0.0)
            );
        }
        b.set_joint_velocity_motor(id, None).unwrap();
        b.step(Duration::from_millis(1)).unwrap();
        // Sphere inertia is isotropic .016: accumulated torque3 gives .1875rad/s.
        assert!((b.joint_state(id).unwrap().velocity - 0.1875).abs() < 1e-12);
        assert!(b.pending_wrenches.is_empty() && b.joints.efforts.is_empty());
        b.step(Duration::from_millis(1)).unwrap();
        assert!((b.joint_state(id).unwrap().velocity - 0.1875).abs() < 1e-12);
    }
}
