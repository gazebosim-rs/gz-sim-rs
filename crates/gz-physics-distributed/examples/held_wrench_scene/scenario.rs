// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, passive_checkpoint::PassivePoolScene};
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

pub fn load(worker: &Path, width: u8) -> Result<PassivePoolScene> {
    Ok(PassivePoolScene::load(
        SDF,
        "distributed_stack.sdf",
        "default",
        options(worker, width),
        Default::default(),
    )?)
}
pub fn queue(scene: &mut PassivePoolScene) -> Result<()> {
    // The cyan body has mass 2 kg and isotropic inertia 0.12 kg m².
    // Its COM receives +0.25 N s immediately before the next 10 ms interval.
    // The resident amber body exercises a separate owner in the same field.
    scene.queue_wrench(11, [160., 0., 0.], [0., 8., 0.])?;
    scene.queue_wrench(7, [0., 0., -0.125], [0.; 3])?;
    scene.queue_impulse(11, [0.25, 0., 0.])?;
    scene.queue_wrench(11, [240., 0., 0.], [0., 16., 0.])?;
    Ok(())
}
