// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_distributed::SceneBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, ModelState, Name, PhysicsEngine, RemoveError, WorldLoadOptions, WorldPose,
    WorldRuntimeOptions, load_world_with_runtime, model_state, prepare_model_template,
    remove_model, reset_loaded_world, set_model_state, spawn_model_template,
};
use sdformat::NoResources;
use std::path::Path;
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const XML: &str = include_str!("../../../examples/distributed_four_lanes.sdf");
fn name_id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn body_id(sim: &Simulation, name: &str) -> BodyId {
    let e = entity_by_id(sim.world(), name_id(sim, name)).unwrap();
    sim.world().get::<BodyHandle>(e).unwrap().id()
}
fn lose(sim: &mut Simulation, worker: u8) -> Result<()> {
    Ok(sim
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<SceneBackend>()
        .unwrap()
        .drop_next_acknowledgement(worker)?)
}

#[test]
fn sdf_template_birth_retirement_and_loaded_reset_keep_ecs_physics_and_visual_ids_consistent()
-> Result<()> {
    let parsed = sdformat::load_str(XML, "four.sdf", &NoResources)?;
    let cargo = parsed.worlds[0]
        .models
        .iter()
        .find(|m| m.name == "cargo_0")
        .unwrap();
    for width in [3, 4, 8] {
        let mut sim = load_world_with_runtime(
            XML,
            "four.sdf",
            "default",
            &NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::DistributedPool(width),
                step: None,
                noise_seed: [0; 32],
                distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
            },
            WorldRuntimeOptions {
                distributed_packed_handoffs: true,
                ..Default::default()
            },
        )?;
        let template = prepare_model_template(&sim, &parsed, cargo)?;
        let original = name_id(&sim, "default::cargo_0");
        let link = name_id(&sim, "default::cargo_0::body");
        let visual = name_id(&sim, "default::cargo_0::body::paint");
        let original_handle = body_id(&sim, "default::cargo_0::body");
        let floor = name_id(&sim, "default::floor");
        let original_state = model_state(&sim, original)?;
        let initial = backend(&sim).pool_frame().unwrap().clone();
        assert_eq!(
            remove_model(&mut sim, floor),
            Err(RemoveError::ImmutableModel)
        );
        sim.step_exact(1)?;
        let current = model_state(&sim, original)?;
        let first = next_entity_id(sim.world())?;
        let overlap = WorldPose {
            translation: current.translation,
            rotation: current.rotation,
        };
        assert!(spawn_model_template(&mut sim, &template, overlap).is_err());
        assert_eq!(next_entity_id(sim.world())?, first);
        let pose = WorldPose {
            translation: [15., -10., 4.],
            rotation: [0., 0., 0., 1.],
        };
        let before_pids = backend(&sim).local_worker_pids().unwrap();
        let owner = backend(&sim)
            .pool_frame()
            .unwrap()
            .workers
            .iter()
            .enumerate()
            .min_by_key(|(_, o)| o.generations.len())
            .unwrap()
            .0;
        lose(&mut sim, owner as u8)?;
        let runtime = spawn_model_template(&mut sim, &template, pose)?;
        assert_eq!(runtime, first);
        assert_eq!(next_entity_id(sim.world())?, runtime + 4);
        assert_ne!(
            backend(&sim).local_worker_pids().unwrap()[owner],
            before_pids[owner]
        );
        let handle = body_id(&sim, &format!("default::runtime_{runtime}::body"));
        assert!(handle > original_handle);
        assert_eq!(
            backend(&sim)
                .pool_frame()
                .unwrap()
                .body(handle.raw())
                .unwrap()
                .0,
            owner as u8
        );
        set_model_state(
            &mut sim,
            runtime,
            ModelState {
                linear_velocity: [0.3, 0., 0.],
                ..model_state_at(pose)
            },
        )?;
        sim.step_exact(10)?;
        let motion = model_state(&sim, runtime)?;
        assert!((motion.translation[0] - 15.015).abs() < 1e-12);
        let runtime_visual = entity_by_id(sim.world(), runtime + 3).unwrap();
        assert_eq!(
            sim.world()
                .get::<WorldPose>(runtime_visual)
                .unwrap()
                .translation,
            motion.translation
        );
        // The other authorities have already seen this deletion when the final
        // acknowledgement is lost. The single public ECS deletion happens after retry.
        lose(&mut sim, width - 1)?;
        let removed = remove_model(&mut sim, original)?;
        assert!(
            removed.contains(&original) && removed.contains(&link) && removed.contains(&visual)
        );
        assert!(
            removed
                .iter()
                .all(|&id| entity_by_id(sim.world(), id).is_none())
        );
        assert!(
            backend(&sim)
                .pool_frame()
                .unwrap()
                .body(original_handle.raw())
                .is_none()
        );
        assert_eq!(model_state(&sim, runtime)?, motion);
        reset_loaded_world(&mut sim, |_| ())?;
        assert_eq!(backend(&sim).pool_frame(), Some(&initial));
        assert_eq!(
            backend(&sim).packed_handoff_workers(),
            Some(usize::from(width))
        );
        assert_eq!(name_id(&sim, "default::cargo_0"), original);
        assert_eq!(name_id(&sim, "default::cargo_0::body::paint"), visual);
        assert_eq!(body_id(&sim, "default::cargo_0::body"), original_handle);
        assert_eq!(model_state(&sim, original)?, original_state);
        for id in runtime..runtime + 4 {
            assert!(entity_by_id(sim.world(), id).is_none());
        }
        assert!(
            backend(&sim)
                .pool_frame()
                .unwrap()
                .body(handle.raw())
                .is_none()
        );
        let later = spawn_model_template(&mut sim, &template, pose)?;
        assert!(later >= runtime + 4);
        let later_handle = body_id(&sim, &format!("default::runtime_{later}::body"));
        assert!(later_handle > handle);
        assert_eq!(
            remove_model(&mut sim, later)?,
            (later..later + 4).collect::<Vec<_>>()
        );
        sim.step_exact(1)?;
    }
    Ok(())
}
fn model_state_at(pose: WorldPose) -> ModelState {
    ModelState {
        translation: pose.translation,
        rotation: pose.rotation,
        linear_velocity: [0.; 3],
        angular_velocity: [0.; 3],
    }
}
