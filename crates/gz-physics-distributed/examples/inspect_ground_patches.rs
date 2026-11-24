// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read shared-plane receipts and native points from an immutable archive without starting workers.
use clap::Parser;
use gz_physics_distributed::pool::PoolNativeRecovery;
use serde_json::json;
use std::{error::Error, path::PathBuf};

#[derive(Parser)]
struct Args {
    /// Read the nested native barrier from a public Bevy scene checkpoint.
    #[arg(long)]
    scene: bool,
    path: PathBuf,
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let image = if args.scene {
        PoolNativeRecovery::open_scene_manifest(args.path)?.2
    } else {
        PoolNativeRecovery::open(args.path)?
    };
    let rows = image.ground_contact_patches()?;
    let frame = image.frame();
    let supports: Vec<_> = frame.workers.iter().map(|w| &w.snapshot.supports).collect();
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "tick":frame.tick, "ground": frame.workers.first().and_then(|w| w.snapshot.ground),
            "public_interval_supports_by_owner":supports, "last_native_substep_by_owner":rows,
        }))?
    );
    Ok(())
}
