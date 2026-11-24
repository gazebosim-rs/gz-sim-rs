// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    ArticulationTier::{Dynamic, Prescribed},
    BodyId, BodySpec, DynamicBackend, JointId, JointVelocityMotor, PhysicsError, RapierBackend,
    RobotJoint, RobotSpring, Shape,
};
use std::time::Duration;
const DT: Duration = Duration::from_millis(1);
fn body(b: &mut RapierBackend, x: f64) -> BodyId {
    b.spawn(BodySpec {
        shape: Shape::Sphere { radius: 0.05 },
        position: [x, 0., 0.],
        velocity: [0.; 3],
        mass: Some(1.),
    })
    .unwrap()
}
fn fixture(
    g: [f64; 3],
    axis: [f64; 3],
    limits: Option<[f64; 2]>,
    spring: Option<RobotSpring>,
) -> (RapierBackend, BodyId, BodyId, JointId) {
    let mut b = RapierBackend::new(g).unwrap();
    let base = body(&mut b, 0.);
    let tool = body(&mut b, 0.3);
    b.add_robot_joint(None, base, [0.; 3], RobotJoint::Fixed)
        .unwrap();
    let kind = RobotJoint::Prismatic { axis, limits };
    let id = if let Some(s) = spring {
        b.add_robot_joint_with_spring(Some(base), tool, [0.3, 0., 0.], kind, s)
            .unwrap()
    } else {
        b.add_robot_joint(Some(base), tool, [0.3, 0., 0.], kind)
            .unwrap()
    };
    (b, base, tool, id)
}
fn refused(b: &mut RapierBackend, root: BodyId, tool: BodyId, reason: &str) {
    let before = b.state(tool);
    let Err(PhysicsError::TransitionDeferred { reason: actual }) =
        b.switch_articulation(root, Prescribed, DT)
    else {
        panic!("expected {reason}")
    };
    assert!(actual.contains(reason), "{actual}");
    assert_eq!(b.state(tool), before);
    assert_eq!(b.articulation_tier(root).unwrap(), Dynamic);
}
#[test]
fn release_requires_equilibrium_and_only_the_correct_side_of_a_stop_can_supply_it() {
    for (g, limit, eligible) in [
        (-9.81, [0., 0.1], true),
        (9.81, [-0.1, 0.], true),
        (-9.81, [-0.1, 0.], false),
        (9.81, [0., 0.1], false),
    ] {
        let mut b = RapierBackend::new([0., 0., g]).unwrap();
        let base = body(&mut b, 0.);
        let tool = base;
        let id = b
            .add_robot_joint(
                None,
                base,
                [0.; 3],
                RobotJoint::Prismatic {
                    axis: [0., 0., 1.],
                    limits: Some(limit),
                },
            )
            .unwrap();
        if eligible {
            for _ in 0..500 {
                b.step(DT).unwrap();
            }
            eprintln!("signed stop {g} m/s², q={:?}", b.joint_state(id));
            b.switch_articulation(base, Prescribed, DT).unwrap();
            b.switch_articulation(base, Prescribed, DT).unwrap();
            b.step(DT).unwrap();
            assert_eq!(b.articulation_tier(base).unwrap(), Prescribed);
            b.reset().unwrap();
            assert_eq!(b.articulation_tier(base).unwrap(), Dynamic);
        } else {
            refused(&mut b, base, tool, "gravity");
        }
    }
    for rest in [-0.03, 0.03] {
        let (mut b, root, tip, _) = fixture(
            [0.; 3],
            [1., 0., 0.],
            Some([-0.1, 0.1]),
            Some(RobotSpring {
                stiffness: 10.,
                reference: rest,
                damping: 0.3,
            }),
        );
        refused(&mut b, root, tip, "gravity");
    }
}
#[test]
fn stationary_mixed_members_preserve_pose_and_wake_on_the_first_commanded_solve() {
    let (mut b, r, t, j) = fixture([0.; 3], [1., 0., 0.], None, None);
    let other = body(&mut b, 5.);
    refused(&mut b, r, t, "initial committed contact solve");
    assert_eq!(b.articulation_link_count(r).unwrap(), 2);
    assert!(b.articulation_contains(r, t));
    assert!(!b.articulation_contains(r, other));
    assert!(b.articulation_tier(other).is_err());
    b.step(DT).unwrap();
    assert!(matches!(
        b.validate_articulation_demotion(r, Duration::ZERO),
        Err(PhysicsError::InvalidValue)
    ));
    b.set_joint_velocity_motor(
        j,
        Some(JointVelocityMotor {
            target_velocity: 0.4,
            max_force: 5.,
        }),
    )
    .unwrap();
    refused(&mut b, r, t, "motor");
    b.set_joint_velocity_motor(j, None).unwrap();
    b.add_joint_effort(j, 1.).unwrap();
    refused(&mut b, r, t, "input");
    b.reset().unwrap();
    b.add_world_wrench(t, [1., 0., 0.], [0.; 3]).unwrap();
    refused(&mut b, r, t, "wrench");
    b.reset().unwrap();
    b.step(DT).unwrap();
    b.switch_articulation(r, Prescribed, DT).unwrap();
    for _ in 0..25 {
        b.step(DT).unwrap();
    }
    assert_eq!(b.articulation_tier(r).unwrap(), Prescribed);
    let before = b.state(t).unwrap();
    b.switch_articulation(r, Dynamic, DT).unwrap();
    assert_eq!(b.state(t).unwrap(), before);
    b.switch_articulation(r, Prescribed, DT).unwrap();
    b.add_joint_effort(j, 5.).unwrap();
    b.step(Duration::ZERO).unwrap();
    assert_eq!(b.articulation_tier(r).unwrap(), Prescribed);
    b.step(DT).unwrap();
    assert_eq!(b.articulation_tier(r).unwrap(), Dynamic);
    assert!((b.state(t).unwrap().velocity[0] - 0.005).abs() < 1e-7);
    refused(&mut b, r, t, "motion");
    b.reset().unwrap();
    b.step(DT).unwrap();
    b.switch_articulation(r, Prescribed, DT).unwrap();
    b.set_joint_velocity_motor(
        j,
        Some(JointVelocityMotor {
            target_velocity: 0.4,
            max_force: 5.,
        }),
    )
    .unwrap();
    b.step(DT).unwrap();
    assert_eq!(b.articulation_tier(r).unwrap(), Dynamic);
    assert!(b.state(t).unwrap().velocity[0] > 0.);
}
#[test]
fn incoming_free_body_wakes_the_parked_robot_and_preserves_first_impact_impulse() {
    let run = |park: bool| {
        let (mut b, r, t, _) = fixture([0.; 3], [1., 0., 0.], None, None);
        let load = body(&mut b, 0.55);
        b.apply_impulse(load, [-1., 0., 0.], [0.; 3]).unwrap();
        b.step(DT).unwrap();
        if park {
            b.switch_articulation(r, Prescribed, DT).unwrap();
        }
        for tick in 2..=220 {
            b.step(DT).unwrap();
            if let Some(c) = b
                .rigid_contact_observations()
                .iter()
                .find(|c| [c.a, c.b] == [t, load] || [c.b, c.a] == [t, load])
                .filter(|c| c.normal_impulse > 0.)
            {
                let impulse = c.normal_impulse;
                assert_eq!(b.articulation_tier(r).unwrap(), Dynamic);
                return (tick, impulse, b.state(t).unwrap(), b.state(load).unwrap());
            }
        }
        panic!("missing impact");
    };
    let (a, b) = (run(false), run(true));
    assert_eq!(a.0, b.0);
    assert!((a.1 - b.1).abs() < 1e-8);
    assert_eq!((a.2, a.3), (b.2, b.3));
}

#[test]
fn a_loaded_or_just_touching_external_body_defers_even_without_a_drive() {
    for obstacle_first in [false, true] {
        let mut b = RapierBackend::new([0.; 3]).unwrap();
        let obstacle_spec = BodySpec {
            shape: Shape::Sphere { radius: 0.05 },
            position: [0.399, 0., 0.],
            velocity: [0.; 3],
            mass: None,
        };
        if obstacle_first {
            b.spawn(obstacle_spec).unwrap();
        }
        let r = body(&mut b, 0.);
        let t = body(&mut b, 0.3);
        b.add_robot_joint(
            None,
            r,
            [0.; 3],
            RobotJoint::Revolute {
                axis: [0., 0., 1.],
                limits: None,
            },
        )
        .unwrap();
        b.add_robot_joint(
            Some(r),
            t,
            [0.3, 0., 0.],
            RobotJoint::Prismatic {
                axis: [0., 0., 1.],
                limits: None,
            },
        )
        .unwrap();
        if !obstacle_first {
            b.spawn(obstacle_spec).unwrap();
        }
        b.step(DT).unwrap();
        refused(&mut b, r, t, "external contact");
    }
}
#[test]
fn mounted_parking_rejects_unmodelled_coordinates_and_any_attached_foreign_constraint() {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let r = body(&mut b, 0.);
    let t = body(&mut b, 0.3);
    b.add_robot_joint(None, r, [0.; 3], RobotJoint::Fixed)
        .unwrap();
    b.add_robot_joint(
        Some(r),
        t,
        [0.3, 0., 0.],
        RobotJoint::Ball {
            orientation: [0., 0., 0., 1.],
        },
    )
    .unwrap();
    refused(&mut b, r, t, "unsupported mounted");
    let (mut b, r, t, _) = fixture([0.; 3], [1., 0., 0.], None, None);
    let extra = body(&mut b, 0.6);
    b.add_revolute(t, extra, [0.6, 0., 0.], [0., 0., 1.])
        .unwrap();
    refused(&mut b, r, t, "external or duplicate joint");
}
#[test]
fn removed_mount_releases_gravity_before_the_first_advancing_solve() {
    let mut b = RapierBackend::new([0., 0., -9.81]).unwrap();
    let r = body(&mut b, 0.);
    let t = body(&mut b, 0.3);
    let mount = b
        .add_robot_joint(
            None,
            r,
            [0.; 3],
            RobotJoint::Revolute {
                axis: [0., 0., 1.],
                limits: None,
            },
        )
        .unwrap();
    b.add_robot_joint(
        Some(r),
        t,
        [0.3, 0., 0.],
        RobotJoint::Prismatic {
            axis: [1., 0., 0.],
            limits: None,
        },
    )
    .unwrap();
    for _ in 0..250 {
        b.step(DT).unwrap();
    }
    b.switch_articulation(r, Prescribed, DT).unwrap();
    b.step(DT).unwrap();
    assert_eq!(b.articulation_tier(r).unwrap(), Prescribed);
    assert!(b.remove_joint(mount));
    assert!(b.articulation_tier(r).is_err());
    b.step(DT).unwrap();
    let rebound: f64 = [r, t]
        .iter()
        .map(|id| b.state(*id).unwrap().velocity[2])
        .sum();
    assert!((rebound + 2. * 9.81 * 0.001).abs() < 1e-6, "{rebound}");
}

#[test]
fn screw_parking_uses_signed_pitch_and_the_physical_stop() {
    for pitch in [-0.2, 0.2] {
        for at_lower in [false, true] {
            let mut b = RapierBackend::new([0., 0., -9.81]).unwrap();
            let mount = body(&mut b, 0.);
            b.add_robot_joint(
                None,
                mount,
                [0.; 3],
                RobotJoint::Screw {
                    axis: [0., 0., 1.],
                    pitch,
                    effort_limit: 4.,
                    limits: Some(if at_lower { [0., 1.] } else { [-1., 0.] }),
                },
            )
            .unwrap();
            if at_lower == (pitch > 0.) {
                b.step(DT).unwrap();
                b.switch_articulation(mount, Prescribed, DT).unwrap();
            } else {
                refused(&mut b, mount, mount, "gravity");
            }
        }
    }
}

#[test]
fn each_wrist_axis_needs_balance_and_active_inputs_keep_the_robot_dynamic() {
    let wrist = |limits| RobotJoint::Universal {
        axes: [[1., 0., 0.], [0., 1., 0.]],
        limits,
        effort_limits: [2.; 2],
    };
    for (at, stops) in [
        ([0., 0.2, 0.], [Some([-1., 0.]), None]),
        ([0.2, 0., 0.], [None, Some([0., 1.])]),
    ] {
        let mut b = RapierBackend::new([0., 0., -9.81]).unwrap();
        let r = b
            .spawn(BodySpec {
                position: at,
                velocity: [0.; 3],
                mass: Some(1.),
                shape: Shape::Sphere { radius: 0.05 },
            })
            .unwrap();
        b.add_robot_joint(None, r, [0.; 3], wrist(stops)).unwrap();
        refused(&mut b, r, r, "gravity");
    }
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let r = body(&mut b, 0.);
    let j = b
        .add_robot_joint(None, r, [0.; 3], wrist([None; 2]))
        .unwrap();
    b.step(DT).unwrap();
    for axis in 0..2 {
        b.set_universal_motor(
            j,
            axis,
            Some(JointVelocityMotor {
                target_velocity: 0.1,
                max_force: 1.,
            }),
        )
        .unwrap();
        refused(&mut b, r, r, "motor");
        b.set_universal_motor(j, axis, None).unwrap();
    }
    b.add_universal_effort(j, [0., 0.15]).unwrap();
    refused(&mut b, r, r, "input");
}

#[test]
fn moving_speed_references_prevent_partial_sleep_but_a_fixed_speed_stop_can_park() {
    for cap in [0., 0.8] {
        let mut b = RapierBackend::new([0.; 3]).unwrap();
        let r = body(&mut b, 0.);
        let j = b
            .add_robot_joint(
                None,
                r,
                [0.; 3],
                RobotJoint::Screw {
                    axis: [0., 0., 1.],
                    pitch: 0.2,
                    effort_limit: 2.,
                    limits: None,
                },
            )
            .unwrap();
        b.set_world_joint_speed_limits(j, [Some(cap), None])
            .unwrap();
        b.step(DT).unwrap();
        if cap > 0. {
            refused(&mut b, r, r, "moving speed-limit references");
        } else {
            b.switch_articulation(r, Prescribed, DT).unwrap();
            b.step(DT).unwrap();
            assert_eq!(b.articulation_tier(r).unwrap(), Prescribed);
        }
    }
}
