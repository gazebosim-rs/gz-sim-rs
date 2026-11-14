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
