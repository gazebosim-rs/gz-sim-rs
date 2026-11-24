// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Inspect actual native contact geometry without starting any worker process.
use clap::Parser;
use gz_physics_distributed::pool::NativeContactSnapshot;
use serde_json::json;
use std::{error::Error, path::PathBuf};

#[derive(Parser)]
struct Args {
    /// Require a public Bevy scene envelope. By default either file type is accepted.
    #[arg(long)]
    scene: bool,
    /// Include ground points and complete public interval receipts on each owner.
    #[arg(long)]
    all: bool,
    /// Immutable native pool or public Bevy scene checkpoint.
    path: PathBuf,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let cut = NativeContactSnapshot::open(args.path)?;
    if args.scene && cut.population.is_none() {
        return Err("expected a public scene archive".into());
    }
    let output = if args.all {
        let owners: Vec<_> = cut
            .frame
            .workers
            .iter()
            .zip(cut.patches)
            .map(|(worker, patches)| {
                json!({
                    "last_native_substep":patches,
                    "public_interval_finite":worker.snapshot.impulses,
                    "public_interval_ground":worker.snapshot.supports,
                })
            })
            .collect();
        json!({"tick":cut.frame.tick,"owners":owners})
    } else {
        let finite: Vec<_> = cut.patches.into_iter().map(|p| p.finite).collect();
        json!({"tick":cut.frame.tick,"last_native_substep_by_owner":finite})
    };
    println!("{}", serde_json::to_string_pretty(&output)?);
    Ok(())
}
