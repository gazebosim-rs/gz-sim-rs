// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    ArticulationTier::{Dynamic, Prescribed},
    BodyId, BodySpec, DynamicBackend, JointId, PhysicsError, RapierBackend, RobotJoint, Shape,
};
use std::time::Duration;

const DT: Duration = Duration::from_millis(1);

fn body(engine: &mut RapierBackend, position: [f64; 3], mass: f64) -> BodyId {
    engine
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.02 },
            mass: Some(mass),
            position,
            velocity: [0.; 3],
        })
        .unwrap()
}

fn wrist(gravity: [f64; 3], tool_position: [f64; 3]) -> (RapierBackend, BodyId, BodyId, JointId) {
    let mut engine = RapierBackend::new(gravity).unwrap();
    let mount = body(&mut engine, [0., 0., 0.5], 1.);
    let tool = body(&mut engine, tool_position, 2.);
    engine
        .add_robot_joint(None, mount, [0., 0., 0.5], RobotJoint::Fixed)
        .unwrap();
    let orientation = rapier3d_f64::prelude::Rotation::from_rotation_z(0.6);
    let joint = engine
        .add_robot_joint(
            Some(mount),
            tool,
            [0., 0., 0.5],
            RobotJoint::Ball {
                orientation: orientation.to_array(),
            },
        )
        .unwrap();
    (engine, mount, tool, joint)
}

#[test]
fn a_hanging_tool_parks_and_a_three_axis_torque_reaches_the_first_resumed_solve() {
    let run = |park: bool| {
        let (mut engine, mount, tool, joint) = wrist([0., 0., -9.81], [0., 0., 0.25]);
        engine.step(DT).unwrap();
        if park {
            engine.switch_articulation(mount, Prescribed, DT).unwrap();
        }
        for _ in 0..250 {
            engine.step(DT).unwrap();
        }
        if park {
            assert_eq!(engine.articulation_tier(mount).unwrap(), Prescribed);
        }
        engine.add_ball_effort(joint, [0.01, -0.02, 0.03]).unwrap();
        assert_eq!(engine.pending_universal_effort(joint), None);
        assert_eq!(engine.pending_ball_effort(joint), Some([0.01, -0.02, 0.03]));
        assert_eq!(
            engine.pending_ball_effort(JointId::from_raw(u64::MAX)),
            None
        );
        let mut states = Vec::new();
        for _ in 0..25 {
            engine.step(DT).unwrap();
            assert_eq!(engine.articulation_tier(mount).unwrap(), Dynamic);
            states.push((
                engine.state(tool).unwrap(),
                engine.ball_joint_state(joint).unwrap(),
            ));
        }
        assert!(states[0].1.angular_velocity.iter().all(|v| v.abs() > 1e-6));
        states
    };
    let (parked, continuous) = (run(true), run(false));
    for ((p, j), (q, k)) in parked.into_iter().zip(continuous) {
        for (a, b) in p
            .position
            .into_iter()
            .chain(p.orientation)
            .chain(p.velocity)
            .chain(p.angular_velocity)
            .zip(
                q.position
                    .into_iter()
                    .chain(q.orientation)
                    .chain(q.velocity)
                    .chain(q.angular_velocity),
            )
        {
            assert!((a - b).abs() < 1e-5, "{a} {b}");
        }
        for (a, b) in j.angular_velocity.into_iter().zip(k.angular_velocity) {
            assert!((a - b).abs() < 1e-5);
        }
    }
}

#[test]
fn an_inverted_or_side_loaded_wrist_must_continue_solving() {
    for tool_position in [[0., 0., 0.75], [0.03, 0., 0.25], [0., -0.03, 0.25]] {
        let (mut engine, mount, tool, _) = wrist([0., 0., -0.001], tool_position);
        engine.step(DT).unwrap();
        let before = engine.state(tool).unwrap();
        assert!(
            matches!(engine.switch_articulation(mount,Prescribed,DT),Err(PhysicsError::TransitionDeferred{reason}) if reason.contains("gravity"))
        );
        assert_eq!(engine.state(tool).unwrap(), before);
        assert_eq!(engine.articulation_tier(mount).unwrap(), Dynamic);
    }
}

#[test]
fn a_welded_off_axis_tool_can_torque_an_otherwise_balanced_bearing() {
    let (mut engine, mount, tool, _) = wrist([0., 0., -0.001], [0., 0., 0.35]);
    let payload = body(&mut engine, [0., 0.1, 0.35], 1.);
    engine
        .add_robot_joint(Some(tool), payload, [0., 0.1, 0.35], RobotJoint::Fixed)
        .unwrap();
    engine.step(DT).unwrap();
    assert!(
        matches!(engine.switch_articulation(mount,Prescribed,DT),Err(PhysicsError::TransitionDeferred{reason}) if reason.contains("gravity"))
    );
}

#[test]
fn a_side_impact_wakes_the_parked_wrist_on_the_first_loaded_tick() {
    let run = |park: bool| {
        let (mut engine, mount, tool, joint) = wrist([0.; 3], [0., 0., 0.25]);
        let projectile = body(&mut engine, [0., -0.12, 0.25], 0.5);
        engine.step(DT).unwrap();
        if park {
            engine.switch_articulation(mount, Prescribed, DT).unwrap();
        }
        for tick in 2..=180 {
            if tick == 22 {
                engine
                    .apply_impulse(projectile, [0., 0.5, 0.], [0.; 3])
                    .unwrap();
            }
            engine.step(DT).unwrap();
            let impulse: f64 = engine
                .rigid_contact_observations()
                .iter()
                .map(|p| p.normal_impulse)
                .sum();
            if impulse > 0. {
                assert_eq!(engine.articulation_tier(mount).unwrap(), Dynamic);
                return (
                    tick,
                    impulse,
                    engine.state(tool).unwrap(),
                    engine.ball_joint_state(joint).unwrap(),
                );
            }
            if park && tick < 22 {
                assert_eq!(engine.articulation_tier(mount).unwrap(), Prescribed);
            }
        }
        panic!("the projectile never loaded the wrist")
    };
    let (a, b) = (run(true), run(false));
    assert!(a.1 > 0.01);
    assert_eq!(a, b);
}
