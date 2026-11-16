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

fn fleet_xml(count: usize) -> String {
    let begin = XML.find("<model name=\"robot\"").unwrap();
    let end = begin + XML[begin..].find("</model>").unwrap() + "</model>".len();
    let robot = &XML[begin..end];
    let models = (0..count)
        .map(|index| {
            let x = if index.is_multiple_of(2) { -1.25 } else { 1.25 };
            let y = -2.25 + (index / 2) as f64 * 1.5;
            robot
                .replace("name=\"robot\"", &format!("name=\"fleet_{index}\""))
                .replace(
                    "<pose>0 0 0.2 0 0 0</pose>",
                    &format!("<pose>{x} {y} 0.2 0 0 0</pose>"),
                )
                .replace("/model/robot/cmd_vel", &format!("/fleet/{index}/cmd_vel"))
                .replace("/model/robot/odometry", &format!("/fleet/{index}/odom"))
                .replace("robot/odom", &format!("fleet_{index}_odom"))
                .replace("robot/base_link", &format!("fleet_{index}_base"))
        })
        .collect::<Vec<_>>()
        .join("");
    XML.replacen(robot, &models, 1)
}

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

#[test]
fn fleet_counts_and_ninth_refusal_preserve_loaded_identity_domain() {
    for count in [0, 1, 2, 3, 7, 8] {
        let sim = load_world_with_sdf_step(
            &fleet_xml(count),
            "m61-fleet.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let ids = identities(&sim);
        assert_eq!(ids.len(), count);
        let rows: Vec<_> = initial_diff_drive_configurations(&sim).unwrap().collect();
        assert_eq!(rows.len(), count);
        for index in 0..count {
            let name = format!("fleet_{index}");
            assert!(diff_drive_binding_matches(&sim, ids[&name]));
            let row = rows.iter().find(|row| row.model_name == name).unwrap();
            assert_eq!(row.topic, format!("/fleet/{index}/cmd_vel"));
            assert_eq!(row.odom_topic, format!("/fleet/{index}/odom"));
            assert_eq!(row.frame_id, format!("fleet_{index}_odom"));
            assert_eq!(row.child_frame_id, format!("fleet_{index}_base"));
        }
    }
    assert!(matches!(
        load_world_with_sdf_step(
            &fleet_xml(9),
            "m61-ninth.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        ),
        Err(gz_sim_scene::LoadError::Unsupported {
            feature: "multiple differential-drive systems",
            ..
        })
    ));
}

#[test]
fn eight_loaded_owners_keep_real_motion_pause_reset_and_retirement_independent() {
    let mut sim = load_world_with_sdf_step(
        &fleet_xml(8),
        "m61-fleet-motion.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let ids = identities(&sim);
    let origins: BTreeMap<_, _> = ids
        .iter()
        .map(|(name, id)| {
            let entity = entity_by_id(sim.world(), *id).unwrap();
            (
                name.clone(),
                sim.world().get::<WorldPose>(entity).unwrap().translation,
            )
        })
        .collect();
    for index in 0_usize..8 {
        let id = ids[&format!("fleet_{index}")];
        let sign = if index.is_multiple_of(2) { -1.0 } else { 1.0 };
        set_diff_drive_command(&mut sim, id, sign * 0.4, 0.0).unwrap();
    }
    sim.step_exact(3000).unwrap();
    for index in 0_usize..8 {
        let name = format!("fleet_{index}");
        let entity = entity_by_id(sim.world(), ids[&name]).unwrap();
        let sign = if index.is_multiple_of(2) { -1.0 } else { 1.0 };
        let sample = sim.world().get::<DiffDriveSample>(entity).unwrap();
        assert_eq!((sample.time, sample.epoch), (Duration::from_secs(3), 0));
        assert!(
            sample
                .pose
                .into_iter()
                .chain(sample.twist)
                .all(f64::is_finite)
        );
        assert!(sample.pose[0] * sign > 0.01);
        let actual = sim.world().get::<WorldPose>(entity).unwrap().translation;
        assert!((actual[0] - origins[&name][0]) * sign > 0.05);
    }
    let elapsed = sim.elapsed();
    sim.set_paused(true);
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(sim.elapsed(), elapsed);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(identities(&sim), ids);
    for (name, id) in &ids {
        let entity = entity_by_id(sim.world(), *id).unwrap();
        assert!(diff_drive_binding_matches(&sim, *id));
        assert!(sim.world().get::<DiffDriveSample>(entity).is_none());
        let pose = sim.world().get::<WorldPose>(entity).unwrap().translation;
        assert_eq!(pose[0].to_bits(), origins[name][0].to_bits());
        assert_eq!(pose[1].to_bits(), origins[name][1].to_bits());
    }
    let retired = ids["fleet_3"];
    remove_model(&mut sim, retired).unwrap();
    assert!(entity_by_id(sim.world(), retired).is_none());
    assert!(!diff_drive_binding_matches(&sim, retired));
    assert!(matches!(
        set_diff_drive_command(&mut sim, retired, 0.4, 0.0),
        Err(DiffDriveCommandError::MissingDrive(_))
    ));
    for id in ids.values().copied().filter(|id| *id != retired) {
        assert!(diff_drive_binding_matches(&sim, id));
    }
    assert_eq!(
        initial_diff_drive_configurations_with_identity(&sim)
            .unwrap()
            .count(),
        8
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(identities(&sim), ids);
    for id in ids.values() {
        remove_model(&mut sim, *id).unwrap();
    }
    for id in ids.values() {
        assert!(entity_by_id(sim.world(), *id).is_none());
    }
}
