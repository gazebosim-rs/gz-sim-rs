// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{JointState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    EntityKind, JointControllerCommandError, JointControllerMode, JointHandle, JointSample, Name,
    PhysicsEngine, WorldPose, initial_joint_controller_configurations,
    initial_joint_controller_configurations_with_identity, load_world_with_sdf_step, remove_model,
    reset_loaded_world, set_joint_controller_command,
};
use std::time::Duration;

const XML: &str = include_str!("../../../examples/m58_multiple_joint_controllers.sdf");
#[derive(Clone, Debug)]
struct OwnerJoint {
    id: u64,
    name: String,
    model: String,
    mode: JointControllerMode,
}
fn load() -> Simulation {
    load_world_with_sdf_step(
        XML,
        "m58-multiple-controllers.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn joints(sim: &Simulation) -> Vec<OwnerJoint> {
    let mut result: Vec<_> = initial_joint_controller_configurations_with_identity(sim)
        .unwrap()
        .map(|row| {
            assert_eq!(row.kind, EntityKind::Joint);
            assert!(row.parent.is_some());
            OwnerJoint {
                id: row.id,
                name: row.name.into(),
                model: row.configuration.model_name.clone(),
                mode: row.configuration.mode,
            }
        })
        .collect();
    result.sort_by(|left, right| left.name.cmp(&right.name));
    result
}
fn state(sim: &Simulation, joint: &OwnerJoint) -> JointState {
    let entity = entity_by_id(sim.world(), joint.id).unwrap();
    let handle = sim.world().get::<JointHandle>(entity).unwrap();
    sim.world()
        .resource::<PhysicsBackend>()
        .joint_state(handle.id())
        .unwrap()
}
fn model(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|entity| {
            entity.get::<EntityKind>() == Some(&EntityKind::Model)
                && entity
                    .get::<Name>()
                    .is_some_and(|actual| actual.0 == format!("default::{name}"))
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

#[test]
fn independent_real_chains_track_opposite_continuous_targets_and_body_geometry() {
    let mut sim = load();
    let values = joints(&sim);
    assert_eq!(values.len(), 4);
    assert_eq!(
        initial_joint_controller_configurations(&sim)
            .unwrap()
            .count(),
        4
    );
    assert_eq!(
        values.iter().filter(|row| row.model == "arm_left").count(),
        2
    );
    assert_eq!(
        values.iter().filter(|row| row.model == "arm_right").count(),
        2
    );
    let before: Vec<_> = values.iter().map(|joint| state(&sim, joint)).collect();
    let left = 4.0 * std::f64::consts::PI + 0.3;
    let right = -2.0 * std::f64::consts::PI - 0.2;
    for joint in &values {
        let target = match (joint.model.as_str(), joint.mode) {
            ("arm_left", JointControllerMode::Position) => left,
            ("arm_left", JointControllerMode::Velocity) => 0.5,
            ("arm_right", JointControllerMode::Position) => right,
            ("arm_right", JointControllerMode::Velocity) => -0.75,
            _ => panic!("unexpected loaded owner"),
        };
        set_joint_controller_command(&mut sim, joint.id, target).unwrap();
    }
    assert_eq!(
        values
            .iter()
            .map(|joint| state(&sim, joint))
            .collect::<Vec<_>>(),
        before,
        "command receipt must not teleport either chain"
    );
    sim.step_exact(30_000).unwrap();
    for joint in &values {
        let actual = state(&sim, joint);
        let expected = match (joint.model.as_str(), joint.mode) {
            ("arm_left", JointControllerMode::Position) => left,
            ("arm_right", JointControllerMode::Position) => right,
            ("arm_left", JointControllerMode::Velocity) => 0.5,
            ("arm_right", JointControllerMode::Velocity) => -0.75,
            _ => unreachable!(),
        };
        let measured = if joint.mode == JointControllerMode::Position {
            actual.position
        } else {
            actual.velocity
        };
        assert!(
            (measured - expected).abs() < 0.01,
            "{}: {actual:?}",
            joint.name
        );
        let entity = entity_by_id(sim.world(), joint.id).unwrap();
        let sample = sim.world().get::<JointSample>(entity).unwrap();
        assert_eq!(sample.time, sim.elapsed());
        assert_eq!(sample.epoch, 0);
        assert_eq!(sample.state, actual);
        if joint.mode == JointControllerMode::Position {
            let base = sim
                .world()
                .iter_entities()
                .find(|entity| {
                    entity
                        .get::<Name>()
                        .is_some_and(|name| name.0 == format!("default::{}::base", joint.model))
                })
                .unwrap();
            let forward = DQuat::from_array(base.get::<WorldPose>().unwrap().rotation) * DVec3::X;
            assert!(
                (forward.x - expected.cos()).abs() < 0.02
                    && (forward.y - expected.sin()).abs() < 0.02,
                "body orientation must follow its own measured joint motion"
            );
        }
    }
}

#[test]
fn pause_invalid_targets_resets_and_first_last_owner_removal_preserve_peer_physics() {
    let mut sim = load();
    let values = joints(&sim);
    let original_ids: Vec<_> = values.iter().map(|joint| joint.id).collect();
    let before: Vec<_> = values.iter().map(|joint| state(&sim, joint)).collect();
    sim.set_paused(true);
    for joint in &values {
        set_joint_controller_command(
            &mut sim,
            joint.id,
            if joint.model == "arm_left" { 0.5 } else { -0.5 },
        )
        .unwrap();
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(matches!(
                set_joint_controller_command(&mut sim, joint.id, invalid),
                Err(JointControllerCommandError::InvalidCommand)
            ));
        }
    }
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(
        values
            .iter()
            .map(|joint| state(&sim, joint))
            .collect::<Vec<_>>(),
        before
    );
    sim.set_paused(false);
    sim.step_exact(1000).unwrap();
    assert!(
        values
            .iter()
            .all(|joint| state(&sim, joint).position.abs() > 0.05)
    );
    sim.reset().unwrap();
    assert!(!sim.is_paused());
    assert!(values.iter().all(|joint| {
        let actual = state(&sim, joint);
        actual.position == 0.0 && actual.velocity == 0.0
    }));
    sim.step_exact(100).unwrap();
    assert!(
        values
            .iter()
            .all(|joint| state(&sim, joint).position.abs() < 1e-12),
        "configured initial targets are zero"
    );
    let first = model(&sim, "arm_left");
    let removed = remove_model(&mut sim, first).unwrap();
    assert!(
        values
            .iter()
            .filter(|joint| joint.model == "arm_left")
            .all(|joint| removed.contains(&joint.id))
    );
    let peer: Vec<_> = values
        .iter()
        .filter(|joint| joint.model == "arm_right")
        .collect();
    for joint in &peer {
        set_joint_controller_command(&mut sim, joint.id, -0.6).unwrap();
    }
    sim.step_exact(1000).unwrap();
    assert!(peer.iter().all(|joint| state(&sim, joint).position < -0.05));
    let last = model(&sim, "arm_right");
    remove_model(&mut sim, last).unwrap();
    for joint in &values {
        assert!(matches!(
            set_joint_controller_command(&mut sim, joint.id, 0.0),
            Err(JointControllerCommandError::MissingController(_))
        ));
    }
    assert_eq!(
        initial_joint_controller_configurations(&sim)
            .unwrap()
            .count(),
        4
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = joints(&sim);
    assert_eq!(
        restored.iter().map(|joint| joint.id).collect::<Vec<_>>(),
        original_ids
    );
    assert!(restored.iter().all(|joint| {
        let actual = state(&sim, joint);
        actual.position == 0.0 && actual.velocity == 0.0
    }));
    sim.step_exact(100).unwrap();
    assert!(
        restored
            .iter()
            .all(|joint| state(&sim, joint).position.abs() < 1e-12)
    );
}

const FLEET_ORIGINAL: &str = include_str!("../../../examples/m9_joint_controllers.sdf");
fn controller_fleet_xml(count: usize, single: Option<bool>) -> String {
    let begin = FLEET_ORIGINAL.find("  <model name=\"arm\"").unwrap();
    let end = FLEET_ORIGINAL.rfind("</model>").unwrap() + "</model>".len();
    let mut result = FLEET_ORIGINAL[..begin].to_owned();
    for owner in 0..count {
        let x = if owner.is_multiple_of(2) { -1.5 } else { 1.5 };
        let y = -4.5 + (owner / 2) as f64 * 3.0;
        let mut model = FLEET_ORIGINAL[begin..end]
            .replacen("name=\"arm\"", &format!("name=\"chain_{owner}\""), 1)
            .replacen(
                "<pose>0 0 1 0 0 0</pose>",
                &format!("<pose>{x} {y} 1 0 0 0</pose>"),
                1,
            );
        if single.unwrap_or_else(|| owner.is_multiple_of(2)) {
            let first = model
                .find("<plugin filename=\"gz-sim-joint-position-controller-system\"")
                .unwrap();
            let last = first + model[first..].find("</plugin>").unwrap() + "</plugin>".len();
            model.replace_range(first..last, "");
            model = model.replacen(
                "<limit><effort>3</effort><velocity>4</velocity></limit>",
                "",
                1,
            );
        }
        result.push_str(&model);
    }
    result.push_str(&FLEET_ORIGINAL[end..]);
    result
}
fn load_controller_fleet(count: usize, single: Option<bool>) -> Simulation {
    load_world_with_sdf_step(
        &controller_fleet_xml(count, single),
        "m62-scene-fleet.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}

#[test]
fn loaded_controller_fleet_counts_preserve_blueprints_and_reject_ninth_owner() {
    for count in [0, 1, 2, 3, 7, 8] {
        for single in [Some(false), Some(true), None] {
            let sim = load_controller_fleet(count, single);
            let values = joints(&sim);
            let expected = match single {
                Some(false) => count * 2,
                Some(true) => count,
                None => count * 2 - count.div_ceil(2),
            };
            assert_eq!(values.len(), expected);
            let ids: std::collections::BTreeSet<_> = values.iter().map(|row| row.id).collect();
            assert_eq!(ids.len(), expected);
            let mut owners = std::collections::BTreeMap::new();
            let mut topics = std::collections::BTreeSet::new();
            for row in initial_joint_controller_configurations_with_identity(&sim).unwrap() {
                assert_eq!(row.kind, EntityKind::Joint);
                assert!(row.parent.is_some());
                assert_ne!(row.id, 0);
                *owners
                    .entry(row.configuration.model_name.clone())
                    .or_insert(0) += 1;
                assert!(topics.insert(row.configuration.topic.clone()));
                let entity = entity_by_id(sim.world(), row.id).unwrap();
                assert_eq!(sim.world().get::<EntityId>(entity).unwrap().get(), row.id);
                assert_eq!(sim.world().get::<Name>(entity).unwrap().0, row.name);
            }
            assert_eq!(owners.len(), count);
            assert!(owners.values().all(|value| (1..=2).contains(value)));
        }
    }
    assert!(matches!(
        load_world_with_sdf_step(
            &controller_fleet_xml(9, Some(true)),
            "m62-ninth-owner.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        ),
        Err(gz_sim_scene::LoadError::Unsupported {
            feature: "multiple joint-controller chains",
            ..
        })
    ));
}

#[test]
fn eight_controller_chains_keep_physical_response_and_loaded_reset() {
    let mut sim = load_controller_fleet(8, Some(false));
    let values = joints(&sim);
    assert_eq!(values.len(), 16);
    let original_ids: Vec<_> = values.iter().map(|row| row.id).collect();
    let before: Vec<_> = values.iter().map(|joint| state(&sim, joint)).collect();
    for joint in &values {
        let owner: usize = joint.model.strip_prefix("chain_").unwrap().parse().unwrap();
        let sign = if owner.is_multiple_of(2) { 1.0 } else { -1.0 };
        set_joint_controller_command(
            &mut sim,
            joint.id,
            sign * if joint.mode == JointControllerMode::Position {
                0.4
            } else {
                0.5
            },
        )
        .unwrap();
    }
    assert_eq!(
        values
            .iter()
            .map(|joint| state(&sim, joint))
            .collect::<Vec<_>>(),
        before,
        "receipt may only update pending targets"
    );
    sim.step_exact(1000).unwrap();
    for joint in &values {
        let owner: usize = joint.model.strip_prefix("chain_").unwrap().parse().unwrap();
        let sign = if owner.is_multiple_of(2) { 1.0 } else { -1.0 };
        let measured = state(&sim, joint);
        assert!(measured.position.is_finite() && measured.velocity.is_finite());
        assert!(measured.position * sign > 0.05);
        if joint.mode == JointControllerMode::Position {
            assert!((measured.position - sign * 0.4).abs() < 0.1);
            let base = sim
                .world()
                .iter_entities()
                .find(|row| {
                    row.get::<Name>()
                        .is_some_and(|name| name.0 == format!("default::{}::base", joint.model))
                })
                .unwrap();
            let forward = DQuat::from_array(base.get::<WorldPose>().unwrap().rotation) * DVec3::X;
            assert!(
                (forward.x - measured.position.cos()).abs() < 0.02
                    && (forward.y - measured.position.sin()).abs() < 0.02
            );
        } else {
            // Match the accepted one-second canonical control; the original
            // thirty-second exact-target control above remains unchanged.
            assert!(
                measured.position * (sign * 0.5) > 0.03 && measured.velocity * (sign * 0.5) > 0.03
            );
        }
        let entity = entity_by_id(sim.world(), joint.id).unwrap();
        let sample = sim.world().get::<JointSample>(entity).unwrap();
        assert_eq!(sample.state, measured);
        assert_eq!((sample.time, sample.epoch), (Duration::from_secs(1), 0));
    }
    sim.set_paused(true);
    let held: Vec<_> = values.iter().map(|joint| state(&sim, joint)).collect();
    let elapsed = sim.elapsed();
    for joint in &values {
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(matches!(
                set_joint_controller_command(&mut sim, joint.id, invalid),
                Err(JointControllerCommandError::InvalidCommand)
            ));
        }
    }
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(sim.elapsed(), elapsed);
    assert_eq!(
        values
            .iter()
            .map(|joint| state(&sim, joint))
            .collect::<Vec<_>>(),
        held
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = joints(&sim);
    assert_eq!(
        restored.iter().map(|row| row.id).collect::<Vec<_>>(),
        original_ids
    );
    assert_eq!(sim.epoch(), 1);
    assert!(restored.iter().all(|joint| {
        let state = state(&sim, joint);
        state.position == 0.0 && state.velocity == 0.0
    }));
    sim.set_paused(false);
    sim.step_exact(100).unwrap();
    assert!(
        restored
            .iter()
            .all(|joint| state(&sim, joint).position.abs() < 1e-12)
    );
    let removed = model(&sim, "chain_7");
    remove_model(&mut sim, removed).unwrap();
    for joint in &restored {
        if joint.model == "chain_7" {
            assert!(entity_by_id(sim.world(), joint.id).is_none());
            assert!(matches!(
                set_joint_controller_command(&mut sim, joint.id, 0.0),
                Err(JointControllerCommandError::MissingController(_))
            ));
        } else {
            set_joint_controller_command(&mut sim, joint.id, 0.4).unwrap();
        }
    }
    sim.step_exact(1000).unwrap();
    assert!(
        restored
            .iter()
            .filter(|joint| joint.model != "chain_7")
            .all(|joint| state(&sim, joint).position > 0.05)
    );
    for owner in 0..7 {
        let id = model(&sim, &format!("chain_{owner}"));
        remove_model(&mut sim, id).unwrap();
    }
    assert!(
        restored
            .iter()
            .all(|joint| entity_by_id(sim.world(), joint.id).is_none())
    );
    assert_eq!(
        initial_joint_controller_configurations(&sim)
            .unwrap()
            .count(),
        16
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        joints(&sim).iter().map(|row| row.id).collect::<Vec<_>>(),
        original_ids
    );
}
