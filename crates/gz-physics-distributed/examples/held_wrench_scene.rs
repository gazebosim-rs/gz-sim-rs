// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Seal or serve the uninterrupted source for the one-tick force and torque recovery comparison.
#[path = "held_wrench_scene/scenario.rs"]
mod scenario;
use clap::Parser;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::SceneBackend;
use serde_json::json;
use std::path::PathBuf;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t=4, value_parser=clap::value_parser!(u8).range(3..=8))]
    width: u8,
    /// Continue serving this independently loaded source paused at 1.10 s.
    #[arg(long, value_parser=clap::value_parser!(u16).range(1..))]
    reference_port: Option<u16>,
    /// New immutable passive SDF archive.
    output: PathBuf,
}
fn main() -> scenario::Result<()> {
    let a = Args::parse();
    let mut scene = scenario::load(&a.worker, a.width)?;
    scene.step_exact(110)?;
    scenario::queue(&mut scene)?;
    scene.save(&a.output)?;
    let backend = scene
        .simulation()
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap();
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "epoch":scene.simulation().epoch(), "time_ns":scene.simulation().elapsed().as_nanos() as u64,
            "target_public_model":10, "target_public_link":11, "target_public_visual":13,
            "held_wrenches":backend.held_scene_wrenches(), "queued_cyan_com_impulse_ns":[0.25,0.,0.],
            "worker_pids":backend.local_worker_pids(), "archive":a.output
        }))?
    );
    if let Some(port) = a.reference_port {
        let sdf = sdformat::load_str(
            scenario::SDF,
            "distributed_stack.sdf",
            &sdformat::NoResources,
        )?;
        gz_sim_cli::live::run(
            scene.into_simulation(),
            "default",
            false,
            0,
            None,
            gz_sim_cli::live::ViewerOptions {
                port: Some(port),
                lifecycle_sdf: Some(sdf),
                recording_path: None,
            },
        )?;
    }
    Ok(())
}
