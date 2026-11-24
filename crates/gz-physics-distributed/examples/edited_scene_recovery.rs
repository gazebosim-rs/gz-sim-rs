// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Create an edited, paused real Bevy scene for the ordinary gz-sim-rs live viewer.
#[path = "edited_scene_recovery/scenario.rs"]
mod scenario;
use clap::Parser;
use std::path::PathBuf;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t=4,value_parser=clap::value_parser!(u8).range(3..=8))]
    width: u8,
    /// Serve the original still-live scene and its template catalog after sealing.
    #[arg(long, value_parser=clap::value_parser!(u16).range(1..))]
    reference_port: Option<u16>,
    /// New checkpoint path.
    output: PathBuf,
}
fn main() -> scenario::Result<()> {
    let a = Args::parse();
    let (mut scene, receipts) = scenario::build(&a.worker, a.width)?;
    scene.save(&a.output)?;
    println!("{}", serde_json::to_string_pretty(&receipts)?);
    if let Some(port) = a.reference_port {
        let sdf = sdformat::load_str(scenario::SDF, "stack.sdf", &sdformat::NoResources)?;
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
