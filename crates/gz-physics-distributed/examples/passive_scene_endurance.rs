// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A repeatedly restored editable Bevy pool versus an independent ordinary SDF load.
#[path = "../tests/private/passive_pose_proof.rs"]
mod passive_pose_proof;
use clap::Parser;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::SceneBackend;
use gz_sim_core::{Simulation, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, ModelState, PhysicsEngine, WorldLoadOptions, WorldPose, WorldRuntimeOptions,
    load_world_with_runtime,
    passive_checkpoint::{PassivePoolScene, RestoredTemplates, save_paused_runtime},
    prepare_model_template, queue_link_world_wrench, remove_model, reset_loaded_world,
    set_model_state, spawn_model_template,
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const SDF: &str = include_str!("../../../examples/distributed_stack.sdf");
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    reference_worker: PathBuf,
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u8).range(3..=8))]
    width: u8,
    #[arg(long,default_value_t=128,value_parser=clap::value_parser!(u16).range(1..))]
    cycles: u16,
    output: PathBuf,
}
#[derive(Serialize, Default)]
struct Audit {
    width: u8,
    cycles: u16,
    ticks: u64,
    compared_public_native_states: u64,
    births: u64,
    retirements: u64,
    loaded_resets: u64,
    queued_impulses_restored: u64,
    held_wrenches_restored: u64,
    coordinator_restarts: u64,
    pair_receipts: u64,
    owner_changes: u64,
    next_public: u64,
    next_native: u64,
    largest_archive_bytes: u64,
    trajectory_sha256: String,
}
fn options(worker: &Path, width: u8) -> WorldLoadOptions<'_> {
    WorldLoadOptions {
        engine: PhysicsEngine::DistributedIslands(width),
        distributed_worker: Some(worker),
        step: None,
        noise_seed: [0; 32],
    }
}
fn pose(x: f64, y: f64, z: f64) -> WorldPose {
    WorldPose {
        translation: [x, y, z],
        rotation: [0., 0., 0., 1.],
    }
}
fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn impulse(sim: &mut Simulation, link: u64) -> Result<()> {
    let body = sim
        .world()
        .get::<BodyHandle>(entity_by_id(sim.world(), link).ok_or("missing impulse link")?)
        .ok_or("missing native handle")?
        .id();
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<SceneBackend>()
        .unwrap()
        .queue_linear_impulse(body, [0.025, 0., 0.0125])?;
    Ok(())
}
struct Comparison {
    audit: Audit,
    digest: Sha256,
    owners: BTreeMap<u64, usize>,
}
impl Comparison {
    fn check(
        &mut self,
        candidate: &Simulation,
        reference: &Simulation,
        advance: bool,
    ) -> Result<()> {
        let state = passive_pose_proof::proof(candidate);
        if state != passive_pose_proof::proof(reference)
            || next_entity_id(candidate.world())? != next_entity_id(reference.world())?
            || backend(candidate).next_body_handle() != backend(reference).next_body_handle()
            || backend(candidate).last_island_refined_bodies()
                != backend(reference).last_island_refined_bodies()
        {
            return Err(format!(
                "public/native divergence epoch {} time {:?} after {} committed checks",
                candidate.epoch(),
                candidate.elapsed(),
                self.audit.compared_public_native_states
            )
            .into());
        }
        let mut owners = BTreeMap::new();
        for (index, owner) in backend(candidate)
            .pool_frame()
            .ok_or("native pool disappeared")?
            .workers
            .iter()
            .enumerate()
        {
            for body in &owner.snapshot.bodies {
                if self
                    .owners
                    .get(&body.id)
                    .is_some_and(|previous| *previous != index)
                {
                    self.audit.owner_changes += 1;
                }
                owners.insert(body.id, index);
            }
            if advance {
                self.audit.pair_receipts += owner.snapshot.impulses.len() as u64;
            }
        }
        self.owners = owners;
        self.digest.update(state);
        self.audit.compared_public_native_states += 1;
        self.audit.ticks += u64::from(advance);
        Ok(())
    }
    fn advance(&mut self, mut worlds: [&mut Simulation; 2], steps: usize) -> Result<()> {
        for _ in 0..steps {
            for sim in &mut worlds {
                sim.step_exact(1).map_err(|e| {
                    format!(
                        "stepping epoch {} time {:?}: {e} {:?}",
                        sim.epoch(),
                        sim.elapsed(),
                        sim.fault()
                    )
                })?;
            }
            self.check(worlds[0], worlds[1], true)?;
        }
        Ok(())
    }
}
fn main() -> Result<()> {
    let a = Args::parse();
    let runtime = WorldRuntimeOptions {
        distributed_packed_handoffs: true,
        distributed_local_helper_limit: Some(0),
        ..Default::default()
    };
    let mut tested = PassivePoolScene::load(
        SDF,
        "stack.sdf",
        "default",
        options(&a.worker, a.width),
        runtime,
    )?
    .into_checkpointable_runtime();
    let mut ordinary = load_world_with_runtime(
        SDF,
        "stack.sdf",
        "default",
        &sdformat::NoResources,
        options(&a.reference_worker, a.width),
        runtime,
    )?;
    let dom = sdformat::load_str(SDF, "stack.sdf", &sdformat::NoResources)?;
    let ordinary_sources = [
        prepare_model_template(&ordinary, &dom, &dom.worlds[0].models[1])?,
        prepare_model_template(&ordinary, &dom, &dom.worlds[0].models[2])?,
    ];
    let mut comparison = Comparison {
        audit: Audit {
            width: a.width,
            cycles: a.cycles,
            ..Default::default()
        },
        digest: Sha256::new(),
        owners: BTreeMap::new(),
    };
    let scratch = tempfile::tempdir()?;
    for cycle in 0..a.cycles {
        if cycle % 4 == 0 {
            for sim in [&mut tested, &mut ordinary] {
                reset_loaded_world(sim, |_| ())?;
            }
            comparison.owners.clear();
            comparison.audit.loaded_resets += 1;
        }
        let choice = usize::from(cycle % 2);
        let name = ["default::first", "default::second"][choice];
        let source = tested
            .world()
            .resource::<RestoredTemplates>()
            .by_name(name)
            .ok_or("original template missing")?;
        let pos = pose(0., 1.25, 1.6);
        let model = spawn_model_template(&mut tested, &source, pos)?;
        if model != spawn_model_template(&mut ordinary, &ordinary_sources[choice], pos)? {
            return Err("different new public ID".into());
        }
        comparison.audit.births += 1;
        let airborne = pose(3. + f64::from(cycle % 2), 1., 3.5);
        let other = spawn_model_template(&mut tested, &source, airborne)?;
        if other != spawn_model_template(&mut ordinary, &ordinary_sources[choice], airborne)? {
            return Err("different second new public ID".into());
        }
        comparison.audit.births += 1;
        let state = ModelState {
            translation: airborne.translation,
            rotation: airborne.rotation,
            linear_velocity: [0.35, 0., 0.25],
            angular_velocity: [0., 0.125, 0.],
        };
        for sim in [&mut tested, &mut ordinary] {
            set_model_state(sim, other, state)?;
            sim.set_paused(true);
            sim.tick()?;
        }
        comparison.check(&tested, &ordinary, false)?;
        comparison.advance([&mut tested, &mut ordinary], 53 + usize::from(cycle % 3))?;
        if cycle % 4 == 1 {
            for sim in [&mut tested, &mut ordinary] {
                remove_model(sim, 10)?;
            }
            comparison.audit.retirements += 1;
        }
        for sim in [&mut tested, &mut ordinary] {
            impulse(sim, other + 1)?;
            sim.tick()?;
        }
        let file = tempfile::tempdir_in(scratch.path())?;
        let archive = file.path().join("committed");
        save_paused_runtime(&mut tested, &archive)?;
        comparison.audit.largest_archive_bytes = comparison
            .audit
            .largest_archive_bytes
            .max(fs::metadata(&archive)?.len());
        comparison.check(&tested, &ordinary, false)?;
        drop(tested);
        tested = PassivePoolScene::resume(
            SDF,
            "stack.sdf",
            "default",
            options(&a.worker, a.width),
            runtime,
            &archive,
        )?
        .into_checkpointable_runtime();
        comparison.audit.coordinator_restarts += 1;
        comparison.audit.queued_impulses_restored += 1;
        comparison.check(&tested, &ordinary, false)?;
        comparison.advance([&mut tested, &mut ordinary], 7)?;
        for sim in [&mut tested, &mut ordinary] {
            queue_link_world_wrench(sim, other + 1, [0., 0., 0.1875], [0.; 3])?;
        }
        let pending = file.path().join("pending");
        save_paused_runtime(&mut tested, &pending)?;
        comparison.audit.largest_archive_bytes = comparison
            .audit
            .largest_archive_bytes
            .max(fs::metadata(&pending)?.len());
        drop(tested);
        tested = PassivePoolScene::resume(
            SDF,
            "stack.sdf",
            "default",
            options(&a.worker, a.width),
            runtime,
            &pending,
        )?
        .into_checkpointable_runtime();
        comparison.audit.coordinator_restarts += 1;
        if passive_pose_proof::proof(&tested) != passive_pose_proof::proof(&ordinary)
            || backend(&tested).held_scene_wrenches() != backend(&ordinary).held_scene_wrenches()
        {
            return Err("restored one-tick field or committed scene diverged".into());
        }
        comparison.audit.held_wrenches_restored += 1;
        comparison.advance([&mut tested, &mut ordinary], 3)?;
        for sim in [&mut tested, &mut ordinary] {
            remove_model(sim, model)?;
            remove_model(sim, other)?;
        }
        comparison.audit.retirements += 2;
        comparison.check(&tested, &ordinary, false)?;
    }
    if comparison.audit.pair_receipts == 0 || comparison.audit.owner_changes == 0 {
        return Err(
            "campaign did not exercise loaded contacts and worker ownership changes".into(),
        );
    }
    comparison.audit.next_public = next_entity_id(tested.world())?;
    comparison.audit.next_native = backend(&tested).next_body_handle();
    comparison.audit.trajectory_sha256 = format!("{:x}", comparison.digest.finalize());
    let json = serde_json::to_string_pretty(&comparison.audit)?;
    fs::write(a.output, format!("{json}\n"))?;
    println!("{json}");
    Ok(())
}
