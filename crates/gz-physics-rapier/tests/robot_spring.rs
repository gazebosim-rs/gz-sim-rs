// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{
    BodyId, BodySpec, DynamicBackend, JointState, JointVelocityMotor, PhysicsError, Shape,
};
use gz_physics_rapier::{RapierBackend, RobotJoint, RobotSpring};
use std::time::Duration;

fn sphere(b: &mut RapierBackend, xyz: [f64; 3]) -> BodyId {
    b.spawn(BodySpec {
        shape: Shape::Sphere { radius: 0.1 },
        position: xyz,
        velocity: [0.; 3],
        mass: Some(1.),
    })
    .unwrap()
}
fn step(b: &mut RapierBackend, count: usize) {
    for _ in 0..count {
        b.step(Duration::from_millis(1)).unwrap();
    }
}
const SLIDE: RobotJoint = RobotJoint::Prismatic {
    axis: [1., 0., 0.],
    limits: Some([-1., 1.]),
};
const SPRING: RobotSpring = RobotSpring {
    reference: 0.1,
    stiffness: 100.,
    damping: 12.,
};

#[test]
fn passive_slider_reaches_reference_after_active_motor_is_removed_and_repeats_after_reset() {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let child = sphere(&mut b, [0.; 3]);
    let j = b
        .add_robot_joint_with_spring(None, child, [0.; 3], SLIDE, SPRING)
        .unwrap();
    for i in 0..2 {
        step(&mut b, 1);
        let s = b.joint_state(j).unwrap();
        assert!(
            (s.velocity - 0.01).abs() < 0.0002,
            "first spring impulse {s:?}"
        );
        assert_eq!(
            SPRING.effort(JointState {
                position: 0.05,
                velocity: 0.25
            }),
            2.
        );
        b.set_joint_velocity_motor(
            j,
            Some(JointVelocityMotor {
                target_velocity: -100.,
                max_force: 40.,
            }),
        )
        .unwrap();
        step(&mut b, 100);
        assert!(b.joint_state(j).unwrap().position < -0.045);
        b.set_joint_velocity_motor(j, None).unwrap();
        step(&mut b, 1850);
        let s = b.joint_state(j).unwrap();
        assert!(
            (s.position - 0.1).abs() < 0.0001 && s.velocity.abs() < 0.0001,
            "{s:?}"
        );
        if i == 0 {
            b.reset().unwrap();
        }
    }
    assert!(b.remove_joint(j));
    let s = b.state(child).unwrap();
    step(&mut b, 100);
    assert!((b.state(child).unwrap().velocity[0] - s.velocity[0]).abs() < 1e-10);
}
#[test]
fn opposed_hinge_spring_conserves_free_parent_momentum_and_respects_stop() {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let parent = sphere(&mut b, [0.; 3]);
    let c = sphere(&mut b, [0., 0., 0.21]);
    let j = b
        .add_robot_joint_with_spring(
            Some(parent),
            c,
            [0., 0., 0.105],
            RobotJoint::Revolute {
                axis: [0., 0., 1.],
                limits: Some([-0.2, 0.2]),
            },
            RobotSpring {
                reference: 0.3,
                damping: 0.1,
                stiffness: 2.,
            },
        )
        .unwrap();
    step(&mut b, 50);
    let a = b.state(parent).unwrap().angular_velocity[2];
    let d = b.state(c).unwrap().angular_velocity[2];
    assert!(a < 0. && d > 0. && (a + d).abs() < 1e-8, "{a} {d}");
    step(&mut b, 450);
    assert!((b.joint_state(j).unwrap().position - 0.2).abs() < 0.001);
}
#[test]
fn malformed_passive_laws_and_wrapped_rotational_springs_fail_before_insertion() {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let c = sphere(&mut b, [0.; 3]);
    let unbounded = RobotJoint::Revolute {
        axis: [0., 0., 1.],
        limits: None,
    };
    let hinge = RobotJoint::Revolute {
        axis: [0., 0., 1.],
        limits: Some([-0.5, 1.]),
    };
    for (joint, s) in [
        (RobotJoint::Fixed, SPRING),
        (unbounded, SPRING),
        (
            RobotJoint::Revolute {
                axis: [0., 0., 1.],
                limits: Some([-3., 3.]),
            },
            RobotSpring {
                reference: 2.9,
                ..SPRING
            },
        ),
        (
            hinge,
            RobotSpring {
                reference: 4.,
                ..SPRING
            },
        ),
        (
            SLIDE,
            RobotSpring {
                reference: f64::NAN,
                ..SPRING
            },
        ),
        (
            SLIDE,
            RobotSpring {
                damping: f64::INFINITY,
                ..SPRING
            },
        ),
        (
            SLIDE,
            RobotSpring {
                stiffness: f64::INFINITY,
                ..SPRING
            },
        ),
        (
            SLIDE,
            RobotSpring {
                damping: -1.,
                ..SPRING
            },
        ),
        (
            SLIDE,
            RobotSpring {
                stiffness: -1.,
                ..SPRING
            },
        ),
        (
            SLIDE,
            RobotSpring {
                damping: 0.,
                stiffness: 0.,
                ..SPRING
            },
        ),
    ] {
        assert_eq!(
            b.add_robot_joint_with_spring(None, c, [0.; 3], joint, s),
            Err(PhysicsError::InvalidValue)
        );
    }
    assert!(
        b.add_robot_joint_with_spring(
            None,
            c,
            [0.; 3],
            RobotJoint::Prismatic {
                axis: [0.; 3],
                limits: None
            },
            SPRING
        )
        .is_err()
    );
    let j = b
        .add_robot_joint_with_spring(
            None,
            c,
            [0.; 3],
            unbounded,
            RobotSpring {
                stiffness: 0.,
                reference: 0.,
                ..SPRING
            },
        )
        .unwrap();
    b.add_joint_effort(j, 0.2).unwrap();
    step(&mut b, 1);
    let before = b.joint_state(j).unwrap().velocity;
    step(&mut b, 2);
    assert!(b.joint_state(j).unwrap().velocity.abs() < before.abs());
}

#[test]
fn constraint_refinement_validates_the_root_budget_and_construction_phase() {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let root = sphere(&mut b, [0.; 3]);
    let other = sphere(&mut b, [1., 0., 0.]);
    assert_eq!(
        b.set_robot_root_iterations(root, 41),
        Err(PhysicsError::InvalidJoint)
    );
    let j = b
        .add_robot_joint(
            None,
            root,
            [0.; 3],
            RobotJoint::Revolute {
                axis: [0., 0., 1.],
                limits: None,
            },
        )
        .unwrap();
    assert_eq!(
        b.set_robot_root_iterations(root, 129),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        b.set_robot_root_iterations(other, 10),
        Err(PhysicsError::InvalidJoint)
    );
    b.set_robot_root_iterations(root, 128).unwrap();
    b.set_robot_root_iterations(root, 0).unwrap();
    b.set_robot_root_iterations(root, 41).unwrap();
    step(&mut b, 1);
    assert_eq!(
        b.set_robot_root_iterations(root, 7),
        Err(PhysicsError::JointConstructionClosed)
    );
    assert!(b.remove_joint(j));
    step(&mut b, 1);
    b.reset().unwrap();
}

#[test]
fn extra_passes_on_one_mounted_arm_leave_a_separate_arms_impulse_response_unchanged() {
    fn arms(settings: &[u8]) -> Vec<f64> {
        let mut b = RapierBackend::new([0.; 3]).unwrap();
        let mut joints = vec![];
        for (i, &passes) in settings.iter().enumerate() {
            let y = i as f64 * 4.;
            let rotor = sphere(&mut b, [0.25, y, 0.]);
            let distal = sphere(&mut b, [0.75, y, 0.]);
            let shoulder = b
                .add_robot_joint(
                    None,
                    rotor,
                    [0., y, 0.],
                    RobotJoint::Revolute {
                        axis: [0., 0., 1.],
                        limits: None,
                    },
                )
                .unwrap();
            b.add_robot_joint(Some(rotor), distal, [0.5, y, 0.], SLIDE)
                .unwrap();
            b.set_robot_root_iterations(rotor, passes).unwrap();
            joints.push(shoulder);
        }
        for &j in &joints {
            b.add_joint_effort(j, 1.).unwrap();
        }
        step(&mut b, 1);
        joints
            .into_iter()
            .map(|j| b.joint_state(j).unwrap().velocity)
            .collect()
    }
    let reference = 0.001 / (0.25_f64.powi(2) + 0.75_f64.powi(2) + 2. * 0.4 * 0.1_f64.powi(2));
    let coarse = arms(&[0])[0];
    let fine = arms(&[100])[0];
    assert!(
        (fine - reference).abs() < 0.005 * reference,
        "{coarse} {fine} {reference}"
    );
    assert!((coarse - reference).abs() > 0.1 * reference);
    for (got, wanted) in [
        (&arms(&[0, 100])[..], [coarse, fine]),
        (&arms(&[100, 0])[..], [fine, coarse]),
    ] {
        for (&v, w) in got.iter().zip(wanted) {
            assert!((v - w).abs() < 1e-10, "{got:?} != {wanted:?}");
        }
    }
}
