// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn request(target_position: f64, max_velocity: f64, max_force: f64) -> JointPositionServo {
    JointPositionServo {
        target_position,
        max_velocity,
        max_force,
    }
}
fn native(backend: &RapierBackend, joint: JointId) -> &GenericJoint {
    let Handle::Reduced(handle) = backend.joints.records[&joint].handle else {
        panic!("expected reduced joint");
    };
    let (body, link) = backend.world.multibody_joints.get(handle).unwrap();
    &body.link(link).unwrap().joint.data
}

#[test]
fn represented_waypoints_bound_the_native_substep_erp_limit_and_reject_unrepresentable_inputs() {
    let limit = MotorModel::AccelerationBased
        .combine_coefficients(0.0, STIFFNESS, DAMPING)
        .0;
    assert_eq!(limit, 5.0);
    for dt in [
        Duration::from_nanos(1),
        Duration::from_millis(1),
        Duration::MAX,
    ] {
        let actual = MotorModel::AccelerationBased
            .combine_coefficients(dt.as_secs_f64(), STIFFNESS, DAMPING)
            .0;
        assert!(actual > 0.0 && actual <= limit);
    }
    for (coordinate, desired) in [
        (0.7, 12.8),
        (-0.7, -12.8),
        (0.7, 0.75),
        (-0.7, -0.75),
        (0.7, 0.7),
    ] {
        let prepared = waypoint(
            request(desired, 2.0, 3.0),
            coordinate,
            Duration::from_millis(1),
        )
        .unwrap();
        assert!((prepared - coordinate).abs() * limit <= 2.0);
        assert!(prepared >= coordinate.min(desired) && prepared <= coordinate.max(desired));
        if (desired - coordinate).abs() <= 0.4 {
            assert_eq!(prepared, desired);
        }
    }
    // These binary64 additions round outward. The waypoint must move inward by
    // a representable value rather than allowing an over-bound native request.
    for sign in [-1.0, 1.0] {
        let coordinate = sign * 0.7;
        let naive = coordinate + sign * 0.4;
        assert!(((naive - coordinate) * limit).abs() > 2.0);
        let prepared = waypoint(
            request(sign * 12.8, 2.0, 3.0),
            coordinate,
            Duration::from_millis(1),
        )
        .unwrap();
        assert!((prepared - coordinate).abs() < (naive - coordinate).abs());
        assert!(((prepared - coordinate) * limit).abs() <= 2.0);
    }
    let dt = Duration::from_millis(1);
    for invalid in [
        request(f64::NAN, 2.0, 3.0),
        request(f64::INFINITY, 2.0, 3.0),
        request(1.0, f64::NAN, 3.0),
        request(1.0, f64::INFINITY, 3.0),
        request(1.0, 0.0, 3.0),
        request(1.0, -1.0, 3.0),
        request(1.0, f64::from_bits(1), 3.0),
        request(1.0, 2.0, f64::NAN),
        request(1.0, 2.0, f64::INFINITY),
        request(1.0, 2.0, -1.0),
    ] {
        assert_eq!(waypoint(invalid, 0.0, dt), Err(PhysicsError::InvalidValue));
    }
    for coordinate in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            waypoint(request(1.0, 2.0, 3.0), coordinate, dt),
            Err(PhysicsError::InvalidValue)
        );
    }
    assert_eq!(
        waypoint(request(1.0, 2.0, 3.0), 0.0, Duration::ZERO),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        waypoint(request(f64::MAX, 2.0, 3.0), -f64::MAX, dt),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        waypoint(request(1.0, 2.0, f64::MAX), 0.0, Duration::from_secs(2)),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        waypoint(request(1.0, f64::MAX, 3.0), 0.0, Duration::from_secs(2)),
        Err(PhysicsError::InvalidValue)
    );
    let large = 9_007_199_254_740_992.0;
    assert_eq!(
        waypoint(request(large + 2.0, 1.0, 3.0), large, dt),
        Err(PhysicsError::InvalidValue)
    );
}

#[test]
fn native_position_parameters_parent_reaction_and_invalid_replacement_preserve_owned_work() {
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
    let parent = backend.spawn_link(spec, rotation, None).unwrap();
    let child = backend.spawn_link(spec, rotation, None).unwrap();
    let joint = backend
        .add_reduced_revolute(Some(parent), child, spec.position, [0.0, 1.0, 0.0])
        .unwrap();
    let dt = Duration::from_millis(1);
    let before = (
        backend.state(parent),
        backend.state(child),
        backend.joint_state(joint),
    );
    let ids = (backend.next_id, backend.joints.next);
    backend
        .set_joint_velocity_motor(
            joint,
            Some(JointVelocityMotor {
                target_velocity: 2.0,
                max_force: 0.1,
            }),
        )
        .unwrap();
    backend
        .validate_joint_position_servo(joint, Some(request(1.0, 2.0, 0.1)), dt)
        .unwrap();
    assert_eq!(
        native(&backend, joint)
            .motor(JointAxis::AngX)
            .unwrap()
            .damping,
        1.0
    );
    backend
        .set_joint_position_servo(joint, Some(request(1.0, 2.0, 0.1)), dt)
        .unwrap();
    let data = native(&backend, joint);
    let value = data.motor(JointAxis::AngX).unwrap();
    assert_eq!(
        (
            value.target_vel,
            value.stiffness,
            value.damping,
            value.max_force
        ),
        (0.0, 100.0, 20.0, 0.1)
    );
    assert_eq!(value.model, MotorModel::AccelerationBased);
    assert!(value.target_pos > 0.0 && value.target_pos * 5.0 <= 2.0);
    assert_eq!(
        (
            backend.state(parent),
            backend.state(child),
            backend.joint_state(joint)
        ),
        before
    );
    assert_eq!((backend.next_id, backend.joints.next), ids);
    assert!(matches!(
        backend.validate_joint_effort(joint, 1.0),
        Err(PhysicsError::Unsupported {
            capability: "moving-parent joint effort commands",
            ..
        })
    ));
    backend.step(dt).unwrap();
    let a = Vector::from(backend.state(parent).unwrap().angular_velocity);
    let b = Vector::from(backend.state(child).unwrap().angular_velocity);
    assert!(a.y < 0.0 && b.y > 0.0);
    assert!((a + b).length() < 1e-12);
    assert!((backend.joint_state(joint).unwrap().velocity - (b - a).y).abs() < 1e-12);
    let after = (
        backend.state(parent),
        backend.state(child),
        backend.joint_state(joint),
    );
    let value = native(&backend, joint).motor(JointAxis::AngX).unwrap();
    let saved = (
        value.target_pos,
        value.target_vel,
        value.stiffness,
        value.damping,
        value.max_force,
        value.impulse,
        value.model,
    );
    backend
        .add_world_wrench(child, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    let work = backend.pending_wrenches.clone();
    for invalid in [
        Some(request(f64::NAN, 2.0, 0.1)),
        Some(request(1.0, 2.0, -1.0)),
    ] {
        assert_eq!(
            backend.set_joint_position_servo(joint, invalid, dt),
            Err(PhysicsError::InvalidValue)
        );
        let value = native(&backend, joint).motor(JointAxis::AngX).unwrap();
        assert_eq!(
            (
                value.target_pos,
                value.target_vel,
                value.stiffness,
                value.damping,
                value.max_force,
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
        assert_eq!(backend.pending_wrenches, work);
        assert_eq!((backend.next_id, backend.joints.next), ids);
    }
    backend
        .set_joint_position_servo(joint, None, Duration::ZERO)
        .unwrap();
    assert!(native(&backend, joint).motor(JointAxis::AngX).is_none());
    assert_eq!(
        native(&backend, joint).motors[JointAxis::AngX as usize].impulse,
        0.0
    );
    backend
        .set_joint_velocity_motor(
            joint,
            Some(JointVelocityMotor {
                target_velocity: 1.0,
                max_force: 0.1,
            }),
        )
        .unwrap();
    let value = native(&backend, joint).motor(JointAxis::AngX).unwrap();
    assert_eq!((value.stiffness, value.damping), (0.0, 1.0));
}
