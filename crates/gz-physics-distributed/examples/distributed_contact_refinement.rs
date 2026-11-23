// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare four adaptive worker processes with an independent uninterrupted solver.
#[path = "support/distributed_encounter.rs"]
mod distributed_encounter;
#[path = "support/seeded.rs"]
mod seeded;
#[path = "support/train_fixture.rs"]
mod train_fixture;

use clap::Parser;
use distributed_encounter::Result;
use std::{fs, path::PathBuf};

#[derive(Parser)]
struct Args {
    /// New output directory for the checkpoint, traces, and numerical evidence.
    output: PathBuf,
    #[arg(long)]
    worker: PathBuf,
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.output)?;
    let mut results = Vec::new();
    for (seed, displaced) in [
        (20261161424, false),
        (20261184268, false),
        (20261199941, false),
        (20261161424, true),
        (20261186018, false),
        (20261122691, false),
    ] {
        let faults = results.is_empty();
        let result = distributed_encounter::run(
            &args.worker,
            seed,
            displaced,
            faults,
            faults.then_some(args.output.as_path()),
        )?;
        if matches!(seed, 20261161424 | 20261184268 | 20261199941)
            && !displaced
            && (result.max_cross_impulse_ns <= 0.01
                || result.peak_left_momentum_exchange_ns <= 0.01
                || result.coarse_max_cross_impulse_ns != 0.)
        {
            return Err(format!("seed {seed} failed the rotating-box impact control").into());
        }
        if matches!(seed, 20261186018 | 20261122691)
            && (result.max_cross_impulse_ns != 0.
                || result.peak_left_momentum_exchange_ns > 1e-10
                || result.minimum_public_gap_m <= 0.)
        {
            return Err(format!("seed {seed} failed its finite-separation control").into());
        }
        println!("{}", serde_json::to_string(&result)?);
        results.push(result);
    }
    fs::write(
        args.output.join("qualification.json"),
        serde_json::to_string_pretty(&results)?,
    )?;
    Ok(())
}
