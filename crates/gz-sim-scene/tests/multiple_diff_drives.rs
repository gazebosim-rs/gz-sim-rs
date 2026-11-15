// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Real admitted M8 graphs; no original-System or no-slip trajectory oracle.
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    DiffDriveCommandError, DiffDriveSample, EntityKind, Name, PhysicsEngine, SceneResetError,
    WorldPose, diff_drive_binding_matches, initial_diff_drive_configurations,
    initial_diff_drive_configurations_with_identity, load_world_with_sdf_step, remove_model,
    reset_loaded_world, set_diff_drive_command,
};
use std::{collections::BTreeMap, time::Duration};
const XML: &str = include_str!("../../../examples/m8_diff_drive.sdf");
fn pair_xml() -> String {
    let begin = XML.find("<model name=\"robot\"").unwrap();
    let end = begin + XML[begin..].find("</model>").unwrap() + "</model>".len();
    let robot = &XML[begin..end];
    let owner = |side: &str, y: i32| {
        robot
            .replace("name=\"robot\"", &format!("name=\"robot_{side}\""))
            .replace(
                "<pose>0 0 0.2 0 0 0</pose>",
                &format!("<pose>0 {y} 0.2 0 0 0</pose>"),
            )
            .replace("/model/robot/cmd_vel", &format!("/m59/{side}/cmd_vel"))
            .replace("/model/robot/odometry", &format!("/m59/{side}/odom"))
            .replace("robot/odom", &format!("m59_{side}_odom"))
            .replace("robot/base_link", &format!("m59_{side}_base"))
    };
    XML.replacen(robot, &(owner("left", -2) + &owner("right", 2)), 1)
}
fn load() -> Simulation {
    load_world_with_sdf_step(
        &pair_xml(),
        "m59-two-drives.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn identities(sim: &Simulation) -> BTreeMap<String, u64> {
    initial_diff_drive_configurations_with_identity(sim)
        .unwrap()
        .map(|row| {
            assert_ne!(row.id, 0);
            assert_eq!(row.kind, EntityKind::Model);
            let entity = entity_by_id(sim.world(), row.id).unwrap();
            assert_eq!(sim.world().get::<EntityId>(entity).unwrap().get(), row.id);
            assert_eq!(sim.world().get::<Name>(entity).unwrap().0, row.name);
            assert!(row.parent.is_some());
            (row.configuration.model_name.clone(), row.id)
        })
        .collect()
}
#[test]
fn two_real_drive_models_receive_independent_commands_and_measured_encoder_motion() {
    assert!(matches!(
        initial_diff_drive_configurations_with_identity(
            &Simulation::new(Duration::from_millis(1)).unwrap()
        ),
        Err(SceneResetError::MissingInitialState)
    ));
    let mut sim = load();
    let ids = identities(&sim);
    assert_eq!(ids.len(), 2);
    assert_eq!(initial_diff_drive_configurations(&sim).unwrap().count(), 2);
    let left = ids["robot_left"];
    let right = ids["robot_right"];
    assert!(diff_drive_binding_matches(&sim, left) && diff_drive_binding_matches(&sim, right));
    set_diff_drive_command(&mut sim, left, 0.4, 0.0).unwrap();
    set_diff_drive_command(&mut sim, right, -0.4, 0.0).unwrap();
    sim.step_exact(1000).unwrap();
    for (id, sign) in [(left, 1.0), (right, -1.0)] {
        let entity = entity_by_id(sim.world(), id).unwrap();
        let sample = sim.world().get::<DiffDriveSample>(entity).unwrap();
        assert_eq!((sample.time, sample.epoch), (Duration::from_secs(1), 0));
        assert!(
            sample
                .pose
                .into_iter()
                .chain(sample.twist)
                .all(f64::is_finite)
        );
        assert!(sample.pose[0] * sign > 0.01);
        assert!(sim.world().get::<WorldPose>(entity).unwrap().translation[0] * sign > 0.01);
    }
    let before = sim.elapsed();
    sim.set_paused(true);
    for id in [left, right] {
        assert!(matches!(
            set_diff_drive_command(&mut sim, id, f64::NAN, 0.0),
            Err(DiffDriveCommandError::InvalidCommand)
        ));
    }
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(sim.elapsed(), before);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(identities(&sim), ids);
    assert_eq!(sim.epoch(), 1);
    for id in [left, right] {
        let entity = entity_by_id(sim.world(), id).unwrap();
        assert!(diff_drive_binding_matches(&sim, id));
        assert!(sim.world().get::<DiffDriveSample>(entity).is_none());
    }
}

#[test]
fn removing_one_real_drive_retires_only_it_and_loaded_reset_restores_both_original_ids() {
    let mut sim = load();
    let ids = identities(&sim);
    let left = ids["robot_left"];
    let right = ids["robot_right"];
    assert!(remove_model(&mut sim, left).unwrap().contains(&left));
    assert!(entity_by_id(sim.world(), left).is_none());
    assert!(!diff_drive_binding_matches(&sim, left));
    assert!(diff_drive_binding_matches(&sim, right));
    assert!(matches!(
        set_diff_drive_command(&mut sim, left, 0.4, 0.0),
        Err(DiffDriveCommandError::MissingDrive(_))
    ));
    set_diff_drive_command(&mut sim, right, -0.4, 0.0).unwrap();
    sim.step_exact(1000).unwrap();
    let entity = entity_by_id(sim.world(), right).unwrap();
    assert!(sim.world().get::<DiffDriveSample>(entity).unwrap().pose[0] < -0.01);
    assert_eq!(
        initial_diff_drive_configurations_with_identity(&sim)
            .unwrap()
            .count(),
        2
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(identities(&sim), ids);
    assert!(diff_drive_binding_matches(&sim, left) && diff_drive_binding_matches(&sim, right));
}
