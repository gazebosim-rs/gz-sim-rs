// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    BodyId, BodySpec, DynamicBackend, JointId, JointVelocityMotor, PhysicsError, RapierBackend,
    Shape,
};
use std::time::Duration;

fn sphere(backend: &mut RapierBackend, position: [f64; 3], mass: f64) -> BodyId {
    backend
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.2 },
            position,
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
    anchor: [f64; 3],
) -> JointId {
    if reduced {
        backend
            .add_reduced_revolute(parent, child, anchor, [0.0, 1.0, 0.0])
            .unwrap()
    } else if let Some(parent) = parent {
        backend
            .add_revolute(parent, child, anchor, [0.0, 1.0, 0.0])
            .unwrap()
    } else {
        backend
            .add_world_revolute(child, anchor, [0.0, 1.0, 0.0])
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
fn solver_motor_limits_torque_tracks_signed_continuous_angle_and_brakes_without_teleport() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let body = sphere(&mut b, [0.0; 3], 1.0);
        let id = hinge(&mut b, reduced, None, body, [0.0; 3]);
        let before = (b.state(body), b.joint_state(id));
        b.validate_joint_velocity_motor(id, motor(100.0, 0.08))
            .unwrap();
        b.set_joint_velocity_motor(id, motor(100.0, 0.08)).unwrap();
        b.step(Duration::ZERO).unwrap();
        assert_eq!((b.state(body), b.joint_state(id)), before);
        steps(&mut b, 1);
        let velocity = b.joint_state(id).unwrap().velocity;
        // Sphere I=.4*m*r²=.016; a .08Nm cap over 1ms permits at most .005rad/s.
        assert!(velocity > 0.0 && velocity <= 0.005 + 1e-12, "{velocity}");
        b.set_joint_velocity_motor(id, motor(6.0, 0.2)).unwrap();
        steps(&mut b, 3000);
        let moving = b.joint_state(id).unwrap();
        assert!(moving.position > std::f64::consts::TAU, "{moving:?}");
        assert!(
            moving.velocity > 4.0 && moving.velocity <= 6.0 + 1e-12,
            "{moving:?}"
        );
        close(moving.velocity, b.state(body).unwrap().angular_velocity[1]);
        b.set_joint_velocity_motor(id, None).unwrap();
        steps(&mut b, 50);
        close(b.joint_state(id).unwrap().velocity, moving.velocity);
        assert!(b.joint_state(id).unwrap().position > moving.position);
        b.set_joint_velocity_motor(id, motor(0.0, 0.2)).unwrap();
        steps(&mut b, 2000);
        assert!(b.joint_state(id).unwrap().velocity.abs() < moving.velocity * 0.25);
        b.set_joint_velocity_motor(id, motor(-2.0, 0.2)).unwrap();
        steps(&mut b, 2000);
        assert!(b.joint_state(id).unwrap().velocity < -1.0);
    }
}

#[test]
fn invalid_replacements_preserve_motor_state_ids_pending_effort_and_zero_force_is_unpowered() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let child = sphere(&mut b, [0.0; 3], 1.0);
        let id = hinge(&mut b, reduced, None, child, [0.0; 3]);
        b.set_joint_velocity_motor(id, motor(100.0, 0.0)).unwrap();
        b.add_joint_effort(id, 1.0).unwrap();
        let before = (b.state(child), b.joint_state(id));
        for invalid in [
            motor(f64::NAN, 1.0),
            motor(f64::INFINITY, 1.0),
            motor(f64::NEG_INFINITY, 1.0),
            motor(1.0, f64::NAN),
            motor(1.0, f64::INFINITY),
            motor(1.0, -1.0),
        ] {
            assert_eq!(
                b.validate_joint_velocity_motor(id, invalid),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!(
                b.set_joint_velocity_motor(id, invalid),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!((b.state(child), b.joint_state(id)), before);
        }
        for value in [None, motor(f64::NAN, -1.0)] {
            assert_eq!(
                b.validate_joint_velocity_motor(JointId::from_raw(99), value),
                Err(PhysicsError::MissingJoint)
            );
            assert_eq!(
                b.set_joint_velocity_motor(JointId::from_raw(99), value),
                Err(PhysicsError::MissingJoint)
            );
        }
        b.step(Duration::ZERO).unwrap();
        assert_eq!((b.state(child), b.joint_state(id)), before);
        steps(&mut b, 1);
        close(b.joint_state(id).unwrap().velocity, 0.0625);
        steps(&mut b, 1);
        close(b.joint_state(id).unwrap().velocity, 0.0625);
        assert_eq!(sphere(&mut b, [10.0, 0.0, 0.0], 1.0).raw(), 1);
    }
}

#[test]
fn both_resets_clear_even_captured_active_motors_and_retirement_is_explicit() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let parent = sphere(&mut b, [0.0; 3], 2.0);
        let child = sphere(&mut b, [0.0; 3], 1.0);
        let id = hinge(&mut b, reduced, Some(parent), child, [0.0; 3]);
        let start = (b.state(parent), b.state(child), b.joint_state(id));
        b.set_joint_velocity_motor(id, motor(3.0, 1.0)).unwrap();
        let initial = b.initial_state();
        steps(&mut b, 100);
        assert!(b.joint_state(id).unwrap().velocity > 0.0);
        b.reset().unwrap();
        assert_eq!((b.state(parent), b.state(child), b.joint_state(id)), start);
        steps(&mut b, 100);
        close(b.joint_state(id).unwrap().velocity, 0.0);
        b.set_joint_velocity_motor(id, motor(-3.0, 1.0)).unwrap();
        steps(&mut b, 100);
        assert!(b.joint_state(id).unwrap().velocity < 0.0);
        let prepared = b.prepare_initial(&initial).unwrap();
        b.apply_initial(prepared).unwrap();
        assert_eq!((b.state(parent), b.state(child), b.joint_state(id)), start);
        steps(&mut b, 100);
        close(b.joint_state(id).unwrap().velocity, 0.0);
        b.set_joint_velocity_motor(id, motor(3.0, 1.0)).unwrap();
        assert!(b.remove_joint(id));
        assert_eq!(
            b.set_joint_velocity_motor(id, None),
            Err(PhysicsError::MissingJoint)
        );
        assert_eq!(b.joint_state(id), None);
        b.apply_initial(b.prepare_initial(&initial).unwrap())
            .unwrap();
        assert_eq!(b.joint_child(id), Some(child));
        b.set_joint_velocity_motor(id, motor(3.0, 1.0)).unwrap();
        assert!(b.remove(child));
        assert_eq!(
            b.validate_joint_velocity_motor(id, None),
            Err(PhysicsError::MissingJoint)
        );
        steps(&mut b, 1);
        assert_eq!(b.joint_state(id), None);
        assert!(b.state(parent).is_some());
    }
}

fn rolling_pair(reduced: bool, floor: bool) -> (RapierBackend, BodyId, [BodyId; 2], [JointId; 2]) {
    let mut b = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    if floor {
        b.spawn(BodySpec {
            shape: Shape::Box {
                half_extents: [10.0, 10.0, 0.05],
            },
            position: [0.0, 0.0, -0.05],
            velocity: [0.0; 3],
            mass: None,
        })
        .unwrap();
    }
    let parent = b
        .spawn(BodySpec {
            shape: Shape::Box {
                half_extents: [0.25, 0.2, 0.1],
            },
            position: [0.0, 0.0, 0.2],
            velocity: [0.0; 3],
            mass: Some(5.0),
        })
        .unwrap();
    let children = [-0.35, 0.35].map(|y| sphere(&mut b, [0.0, y, 0.2], 1.0));
    let ids = children.map(|child| {
        let anchor = b.state(child).unwrap().position;
        hinge(&mut b, reduced, Some(parent), child, anchor)
    });
    for id in ids {
        assert!(matches!(
            b.validate_joint_effort(id, 1.0),
            Err(PhysicsError::Unsupported {
                capability: "moving-parent joint effort commands",
                ..
            })
        ));
        b.set_joint_velocity_motor(id, motor(4.0, 1.0)).unwrap();
    }
    (b, parent, children, ids)
}

#[test]
fn moving_parent_motors_drive_real_contacts_instead_of_prescribing_body_motion() {
    for reduced in [false, true] {
        let (mut ground, chassis, wheels, joints) = rolling_pair(reduced, true);
        let (mut air, free_chassis, _, _) = rolling_pair(reduced, false);
        let before = ground.state(chassis).unwrap();
        assert_eq!(before.position, [0.0, 0.0, 0.2]);
        steps(&mut ground, 2000);
        steps(&mut air, 2000);
        let driven = ground.state(chassis).unwrap();
        assert!(driven.position[0] > 0.02, "{driven:?}");
        assert!(driven.position[2] > 0.05 && driven.position[2] < 0.5);
        assert!(air.state(free_chassis).unwrap().position[0].abs() < 1e-10);
        assert!(air.state(free_chassis).unwrap().position[2] < -1.0);
        for (wheel, joint) in wheels.into_iter().zip(joints) {
            let body = ground.state(wheel).unwrap();
            assert!(
                body.position
                    .into_iter()
                    .chain(body.velocity)
                    .chain(body.orientation)
                    .chain(body.angular_velocity)
                    .all(f64::is_finite)
            );
            assert!(body.position[2] > 0.15, "{body:?}");
            assert!(ground.joint_state(joint).unwrap().velocity > 0.0);
        }
    }
}
