// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![allow(deprecated)]

use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    JointControllerConfiguration, JointSample, Name, PhysicsEngine, UniversalJointSample,
    load_world_with_sdf_step, reset_loaded_world, set_joint_controller_command,
};
use sdformat::NoResources;
const SCREW: &str = include_str!("../../../examples/worlds/local_robot_screw_controller.sdf");
const WRIST: &str = include_str!("../../../examples/worlds/local_robot_wrist_controller.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "local_controllers.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn scalar(sim: &Simulation, joint: u64) -> [f64; 2] {
    let s = sim
        .world()
        .get::<JointSample>(entity_by_id(sim.world(), joint).unwrap())
        .unwrap()
        .state;
    [s.position, s.velocity]
}
fn wrist(sim: &Simulation, joint: u64) -> ([f64; 2], [f64; 2]) {
    let s = sim
        .world()
        .get::<UniversalJointSample>(entity_by_id(sim.world(), joint).unwrap())
        .unwrap()
        .state;
    (s.position, s.velocity)
}

#[test]
fn screw_position_controller_lifts_a_mass_through_one_turn_then_reverses_and_resets() {
    let mut sim = load(SCREW);
    let j = id(&sim, "default::screw_drive::spindle");
    let mut replay = None;
    for _ in 0..2 {
        let mut peak = 0_f64;
        for _ in 0..5000 {
            sim.step_exact(1).unwrap();
            peak = peak.max(scalar(&sim, j)[1].abs());
        }
        let [q, speed] = scalar(&sim, j);
        eprintln!("controlled screw q={q} final speed {speed}, peak {peak}");
        assert!((q - 7.1).abs() < 0.002 && speed.abs() < 0.002 && peak < 1.71);
        set_joint_controller_command(&mut sim, j, 4.5).unwrap();
        assert!(set_joint_controller_command(&mut sim, j, f64::NAN).is_err());
        sim.step_exact(2200).unwrap();
        let end = scalar(&sim, j);
        eprintln!("screw reverse {:?}", end);
        assert!((end[0] - 4.5).abs() < 0.002);
        if let Some(prior) = replay {
            assert_eq!(end, prior);
        }
        replay = Some(end);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
    }
}

#[test]
fn selected_child_axis_tracks_position_through_a_folded_sprung_wrist() {
    let mut sim = load(WRIST);
    let j = id(&sim, "default::robot::wrist");
    let configuration = sim
        .world()
        .get::<JointControllerConfiguration>(entity_by_id(sim.world(), j).unwrap())
        .unwrap();
    assert_eq!(configuration.topic, "/model/robot/joint/wrist/1/cmd_pos");
    for _ in 0..2 {
        sim.step_exact(3500).unwrap();
        let (q, rate) = wrist(&sim, j);
        eprintln!("controlled wrist {q:?} {rate:?}");
        assert!((q[1] + 0.82).abs() < 0.002 && rate[1].abs() < 0.002 && q[0] > 0.5);
        set_joint_controller_command(&mut sim, j, 0.41).unwrap();
        sim.step_exact(4500).unwrap();
        let (q, rate) = wrist(&sim, j);
        eprintln!("controlled wrist reverse {q:?} {rate:?}");
        assert!((q[1] - 0.41).abs() < 0.002 && rate[1].abs() < 0.002);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
    }
}

fn velocity_plugin(xml: &str, joint: &str, target: f64) -> String {
    let mut xml = xml.to_owned();
    let start = xml.find("<plugin ").unwrap();
    let end = xml.find("</plugin>").unwrap() + "</plugin>".len();
    xml.replace_range(start..end,&format!("<plugin filename='gz-sim-joint-controller-system' name='gz::sim::systems::JointController'><joint_name>{joint}</joint_name><initial_velocity>{target}</initial_velocity></plugin>"));
    xml
}

#[test]
fn screw_velocity_plugin_obeys_effort_at_start_and_saturates_held_commands() {
    let mut sim = load(&velocity_plugin(SCREW, "spindle", 0.7));
    let joint = id(&sim, "default::screw_drive::spindle");
    sim.step_exact(1).unwrap();
    let first = scalar(&sim, joint)[1];
    let lead = 0.4 / std::f64::consts::TAU;
    let expected = 0.001 * (2. - 9.81 * lead) / (0.02 + lead * lead);
    eprintln!("loaded screw first tick rate {first}, torque-limited oracle {expected}");
    assert!((first - expected).abs() < 0.001);
    sim.step_exact(120).unwrap();
    assert!((scalar(&sim, joint)[1] - 0.7).abs() < 0.001);
    for target in [100., -100.] {
        set_joint_controller_command(&mut sim, joint, target).unwrap();
        sim.step_exact(300).unwrap();
        let actual = scalar(&sim, joint)[1];
        eprintln!("speed demand {target} gave {actual}");
        assert!((actual - target.signum() * 2.).abs() < 0.001);
    }
}

#[test]
fn plain_wrist_velocity_plugin_drives_the_first_axis_and_leaves_the_second_sprung() {
    let xml = velocity_plugin(WRIST, "wrist", 0.7).replace(
        "<upper>2.5</upper>",
        "<upper>2.5</upper><velocity>1.1</velocity><effort>9</effort>",
    );
    let mut sim = load(&xml);
    let joint = id(&sim, "default::robot::wrist");
    sim.step_exact(300).unwrap();
    let (q, rate) = wrist(&sim, joint);
    eprintln!("velocity wrist {q:?} {rate:?}");
    assert!((rate[0] - 0.7).abs() < 0.002 && q[1] < -0.1);
    set_joint_controller_command(&mut sim, joint, -8.).unwrap();
    sim.step_exact(300).unwrap();
    assert!((wrist(&sim, joint).1[0] + 1.1).abs() < 0.002);
}

#[test]
fn bad_authored_axis_and_missing_actuator_limits_are_rejected() {
    for xml in [
        WRIST.replace("<joint_index>1", "<joint_index>2"),
        WRIST.replace("<joint_index>1", "<joint_index>bogus"),
        SCREW.replace(
            "<joint_name>spindle</joint_name>",
            "<joint_name>spindle</joint_name><joint_index>1</joint_index>",
        ),
        WRIST.replace("<effort>8</effort>", ""),
        WRIST.replace("<velocity>1.3</velocity>", ""),
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml,
                "bad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}
