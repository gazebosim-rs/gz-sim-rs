// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    JointControllerConfiguration, JointSample, Name, held_joint_controller_target,
    load_local_replay_world, reset_loaded_world, set_joint_controller_command,
};

const WORLD: &str = include_str!("../../../examples/worlds/upstream_joint_position.sdf");
const JOINT: &str = "default::joint_position_controller_test::j1";

fn load(xml: &str) -> Simulation {
    load_local_replay_world(xml, "stock.sdf", "default", [0; 32]).unwrap()
}
fn binding(sim: &mut Simulation) -> u64 {
    let mut q = sim.world_mut().query::<(&Name, &EntityId)>();
    q.iter(sim.world())
        .find(|(n, _)| n.0 == JOINT)
        .unwrap()
        .1
        .get()
}
fn state(sim: &Simulation, id: u64) -> (f64, f64) {
    let s = sim
        .world()
        .get::<JointSample>(entity_by_id(sim.world(), id).unwrap())
        .unwrap()
        .state;
    (s.position, s.velocity)
}

#[test]
fn stock_pid_reversal_matches_dart_and_reset_repeats_every_encoder_sample() {
    let mut sim = load(WORLD);
    let id = binding(&mut sim);
    let entity = entity_by_id(sim.world(), id).unwrap();
    assert_eq!(
        sim.world()
            .get::<JointControllerConfiguration>(entity)
            .unwrap()
            .topic,
        "/model/joint_position_controller_test/joint/j1/0/cmd_pos"
    );
    let mut histories = Vec::new();
    for pass in 0..3 {
        if pass == 1 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        if pass == 2 {
            sim.reset().unwrap();
        }
        assert_eq!(held_joint_controller_target(&sim, id), Some(0.));
        sim.step_exact(50).unwrap();
        let mut trace = Vec::new();
        for (target, first_dart_speed) in [(1., 91.6675), (-0.5, -137.50125), (0.25, 68.75065)] {
            set_joint_controller_command(&mut sim, id, target).unwrap();
            if pass == 1 {
                sim.set_paused(true);
                let before = state(&sim, id);
                for _ in 0..3 {
                    sim.tick().unwrap();
                }
                assert_eq!(state(&sim, id), before);
                sim.set_paused(false);
            }
            for tick in 1..=400 {
                sim.step_exact(1).unwrap();
                let current = state(&sim, id);
                if tick == 1 {
                    assert!((current.1 - first_dart_speed).abs() < 0.01);
                }
                trace.push(current);
            }
            assert!((state(&sim, id).0 - target).abs() < 0.00001);
        }
        histories.push(trace);
    }
    assert_eq!(histories[0], histories[1]);
    assert_eq!(histories[0], histories[2]);
}

#[test]
fn force_pid_respects_output_and_axis_effort_caps_and_rejects_an_overflowing_target() {
    for (parameter, from, to) in [
        (
            "controller clamp",
            "<cmd_max>1000</cmd_max>",
            "<cmd_max>0.012</cmd_max>",
        ),
        (
            "joint effort limit",
            "<xyz>0 0 1</xyz>",
            "<xyz>0 0 1</xyz><limit><effort>0.012</effort></limit>",
        ),
    ] {
        let mut sim = load(&WORLD.replace(from, to));
        let id = binding(&mut sim);
        set_joint_controller_command(&mut sim, id, 1.).unwrap();
        assert!(set_joint_controller_command(&mut sim, id, f64::MAX).is_err());
        assert_eq!(held_joint_controller_target(&sim, id), Some(1.));
        sim.step_exact(1).unwrap();
        assert!(
            (state(&sim, id).1 - 0.1).abs() < 1e-5,
            "{parameter}: {:?}",
            state(&sim, id)
        );
    }
}

#[test]
fn pid_and_broadcaster_configuration_reject_unknown_content_and_invalid_values() {
    let extension = |body: &str| {
        WORLD.replace(
            "<joint_name>j1</joint_name>",
            &format!("<joint_name>j1</joint_name>{body}"),
        )
    };
    for body in [
        "<sub_topic>bad</sub_topic>",
        "<joint_index>1</joint_index>",
        "<use_velocity_commands>true</use_velocity_commands>",
        "<initial_position>NaN</initial_position>",
        "<cmd_offset>inf</cmd_offset>",
    ] {
        assert!(
            load_local_replay_world(&extension(body), "bad", "default", [0; 32]).is_err(),
            "{body}"
        );
    }
    let plugin = "<plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>";
    for source in [
        WORLD.replace("</world>", &format!("{plugin}</world>")),
        WORLD.replace(
            "systems::SceneBroadcaster\">",
            "systems::SceneBroadcaster\"><unknown/>",
        ),
        WORLD.replace(
            "<xyz>0 0 1</xyz>",
            "<xyz>0 0 1</xyz><limit><velocity>2</velocity></limit>",
        ),
    ] {
        assert!(load_local_replay_world(&source, "bad", "default", [0; 32]).is_err());
    }
}

#[test]
fn authored_pid_fields_and_omitted_defaults_produce_the_expected_first_effort() {
    let start = WORLD.find("<p_gain>").unwrap();
    let end = WORLD.find("</cmd_min>").unwrap() + "</cmd_min>".len();
    let source = |fields: &str| {
        let mut xml = WORLD.to_owned();
        xml.replace_range(start..end, fields);
        xml
    };
    let trajectory = |xml: &str, target: f64| {
        let mut sim = load(xml);
        let id = binding(&mut sim);
        set_joint_controller_command(&mut sim, id, target).unwrap();
        (0..20)
            .map(|_| {
                sim.step_exact(1).unwrap();
                state(&sim, id)
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(trajectory(WORLD, 1.), trajectory(&source(""), 1.));
    let fields = "<p_gain>0.04</p_gain><i_gain>4</i_gain><d_gain>0.00002</d_gain>
        <i_min>-0.0004</i_min><i_max>0.0001</i_max>
        <cmd_min>-0.004</cmd_min><cmd_max>0.025</cmd_max><cmd_offset>-0.001</cmd_offset>
        <initial_position>0.25</initial_position><use_velocity_commands>false</use_velocity_commands>";
    let custom = source(fields);
    // On the first positive step, 0.01 P + 0.0004 clamped I + 0.005 D - 0.001 offset.
    // The negative offset and the independently asymmetric command clamp test the signs.
    for (target, expected_effort) in [(0.25, 0.0144), (-0.25, -0.004)] {
        let first = trajectory(&custom, target)[0];
        assert!((first.1 - expected_effort * 0.001 / 0.00012).abs() < 1e-5);
    }
    let mut sim = load(&custom);
    let id = binding(&mut sim);
    assert_eq!(held_joint_controller_target(&sim, id), Some(0.25));
    sim.step_exact(1).unwrap();
    assert!((state(&sim, id).1 - 0.12).abs() < 1e-5);
}
