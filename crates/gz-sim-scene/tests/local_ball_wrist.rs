// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![allow(deprecated)]
use glam::{DQuat, DVec3, EulerRot};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BallJointSample, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    queue_ball_joint_effort, reset_loaded_world,
};

const XML: &str = include_str!("../../../examples/worlds/local_ball_wrist.sdf");

fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "wrist.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn id(sim: &Simulation, link: &str) -> u64 {
    let scoped = format!("default::wrist::{link}");
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == scoped))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

#[test]
fn torque_rotates_with_its_free_carrier_and_conserves_angular_momentum() {
    let mut sim = load(XML);
    let bearing = id(&sim, "bearing");
    let initial_frame = DQuat::from_euler(EulerRot::ZYX, 0.63, -0.42, 0.31);
    let mut previous_run = Vec::new();
    for pass in 0..2 {
        let [carrier, tool, joint] =
            ["carrier", "tool", "bearing"].map(|n| entity_by_id(sim.world(), id(&sim, n)).unwrap());

        for tick in 0..650 {
            let local = if tick < 250 {
                DVec3::new(0.14, -0.18, 0.28)
            } else if tick < 500 {
                DVec3::new(-0.10, 0.24, 0.06)
            } else {
                DVec3::ZERO
            };
            let rotation =
                DQuat::from_array(sim.world().get::<WorldPose>(carrier).unwrap().rotation);
            let angular = |s: &Simulation, e| {
                DVec3::from_array(s.world().get::<Velocity>(e).unwrap().angular)
            };
            let before = angular(&sim, tool);
            if tick < 500 {
                for _ in 0..2 {
                    queue_ball_joint_effort(&mut sim, bearing, (local * 0.5).into()).unwrap();
                }
                assert!(queue_ball_joint_effort(&mut sim, bearing, [0., f64::NAN, 1.]).is_err());
            }
            if tick == 0 {
                sim.set_paused(true);
                sim.tick().unwrap();
                sim.set_paused(false);
                assert_eq!(angular(&sim, tool), before);
            }
            sim.step_exact(1).unwrap();
            let child = angular(&sim, tool);
            let parent = angular(&sim, carrier);
            let expected = rotation * initial_frame * local * (0.001 / 0.02);
            assert!(
                (child - before).distance(expected) < 1e-9,
                "tick {tick}, observed {:?} expected {expected:?}",
                child - before
            );
            assert!((0.02 * child + 0.04 * parent).length() < 1e-10);
            let state = sim.world().get::<BallJointSample>(joint).unwrap().state;
            let frame = DQuat::from_array(sim.world().get::<WorldPose>(carrier).unwrap().rotation)
                * initial_frame;
            assert!(
                (frame * DVec3::from_array(state.angular_velocity)).distance(child - parent)
                    < 1e-10
            );
            if pass == 0 {
                previous_run.push(state);
            } else {
                assert_eq!(previous_run[tick], state);
            }
        }
        queue_ball_joint_effort(&mut sim, bearing, [1., 2., 3.]).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
    }
}

#[test]
fn a_world_mounted_ball_consumes_accumulated_input_once_and_rejects_overflow() {
    use gz_physics::{JointId, PhysicsBackend, PhysicsError};
    use gz_physics_multifidelity::MultiFidelityBackend;
    let mut xml = XML.replace("<parent>carrier</parent>", "<parent>world</parent>");
    let first = xml.find("<link name=\"carrier\">").unwrap();
    let last = first + xml[first..].find("</link>").unwrap() + "</link>".len();
    xml.replace_range(first..last, "");
    let mut sim = load(&xml);
    let bearing = id(&sim, "bearing");
    let tool = id(&sim, "tool");
    assert!(queue_ball_joint_effort(&mut sim, tool, [1.; 3]).is_err());
    assert!(queue_ball_joint_effort(&mut sim, u64::MAX, [1.; 3]).is_err());
    assert!(queue_ball_joint_effort(&mut sim, bearing, [f64::INFINITY; 3]).is_err());
    let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
    let missing = backend
        .downcast_mut::<MultiFidelityBackend>()
        .unwrap()
        .add_ball_effort(JointId::from_raw(u64::MAX), [0.; 3]);
    assert!(matches!(missing, Err(PhysicsError::MissingJoint)));
    queue_ball_joint_effort(&mut sim, bearing, [0., 0., f64::MAX]).unwrap();
    assert!(queue_ball_joint_effort(&mut sim, bearing, [0., 1., f64::MAX]).is_err());
    queue_ball_joint_effort(&mut sim, bearing, [0., 0., -f64::MAX]).unwrap();
    queue_ball_joint_effort(&mut sim, bearing, [1., -2., 0.5]).unwrap();
    sim.step_exact(50).unwrap();
    let joint = entity_by_id(sim.world(), bearing).unwrap();
    let expected = DVec3::new(1., -2., 0.5) * (0.001 / 0.02);
    assert!(
        DVec3::from_array(
            sim.world()
                .get::<BallJointSample>(joint)
                .unwrap()
                .state
                .angular_velocity
        )
        .distance(expected)
            < 1e-10
    );
    let mut sim = load(&XML.replace("type=\"ball\"", "type=\"fixed\""));
    let fixed = id(&sim, "bearing");
    assert!(queue_ball_joint_effort(&mut sim, fixed, [1.; 3]).is_err());
}
