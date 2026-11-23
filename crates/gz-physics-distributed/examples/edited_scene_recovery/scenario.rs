// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_physics_distributed::SceneBackend;
use gz_sim_core::{EntityId, next_entity_id};
use gz_sim_scene::{
    Name, PhysicsEngine, WorldLoadOptions, WorldPose, WorldRuntimeOptions,
    passive_checkpoint::PassivePoolScene,
};
use serde::Serialize;
use std::path::Path;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
pub const SDF: &str = include_str!("../../../../examples/distributed_stack.sdf");
pub fn options(worker: &Path, width: u8) -> WorldLoadOptions<'_> {
    WorldLoadOptions {
        engine: PhysicsEngine::DistributedIslands(width),
        step: None,
        noise_seed: [0; 32],
        distributed_worker: Some(worker),
    }
}
pub fn id(s: &PassivePoolScene, name: &str) -> u64 {
    s.simulation()
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
#[derive(Serialize, Debug, PartialEq, Eq)]
pub struct Receipts {
    pub template_root: u64,
    pub prior_epoch: [u64; 2],
    pub retired_runtime: u64,
    pub airborne_clone: u64,
    pub newborn_clone: u64,
    pub next_public: u64,
    pub next_native: u64,
    pub checkpoint_tick: u64,
    pub epoch: u64,
}
pub fn pose(x: f64, y: f64, z: f64) -> WorldPose {
    WorldPose {
        translation: [x, y, z],
        rotation: [0., 0., 0., 1.],
    }
}
pub fn build(worker: &Path, width: u8) -> Result<(PassivePoolScene, Receipts)> {
    let mut s = PassivePoolScene::load(
        SDF,
        "stack.sdf",
        "default",
        options(worker, width),
        WorldRuntimeOptions::default(),
    )?;
    let template_root = id(&s, "default::first");
    let second = id(&s, "default::second");
    let prior = [
        s.spawn(template_root, pose(-3., 0., 2.))?,
        s.spawn(second, pose(3., 0., 2.))?,
    ];
    s.step_exact(4)?;
    s.retire(prior[0])?;
    s.reset_loaded()?;
    s.step_exact(55)?;
    let retired_runtime = s.spawn(template_root, pose(2., -2., 4.))?;
    s.retire(retired_runtime)?;
    let airborne = s.spawn(template_root, pose(0., 0., 4.5))?;
    s.step_exact(30)?;
    s.retire(template_root)?;
    s.step_exact(25)?;
    let mut birth = pose(1.7, 1.2, 2.25);
    let half = 0.175_f64;
    birth.rotation = [0., 0., half.sin(), half.cos()];
    let newest = s.spawn(template_root, birth)?;
    s.queue_impulse(airborne + 1, [0., 0.0125, 0.])?;
    let sim = s.simulation();
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap();
    let receipt = Receipts {
        template_root,
        prior_epoch: prior,
        retired_runtime,
        airborne_clone: airborne,
        newborn_clone: newest,
        next_public: next_entity_id(sim.world())?,
        next_native: native.next_body_handle(),
        checkpoint_tick: native.pool_frame().unwrap().tick,
        epoch: sim.epoch(),
    };
    Ok((s, receipt))
}
