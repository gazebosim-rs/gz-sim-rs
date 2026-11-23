// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run with either a clean prior build or the current build for matched comparisons.
#[path = "support/dense_separation.rs"]
mod dense;
use clap::Parser;
use serde::Serialize;
use std::{
    fs,
    path::{Path, PathBuf},
    time::Instant,
};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// Create this new directory for a qualified run, including its trace and archive.
    #[arg(
        long,
        required_unless_present_any = ["benchmark", "verify_checkpoint"],
        conflicts_with_all = ["benchmark", "verify_checkpoint"]
    )]
    out: Option<PathBuf>,
    /// Time just the four-worker stepping API. No injected faults or reference solvers.
    #[arg(long)]
    benchmark: bool,
    /// Reopen this prior qualification directory and compare its remaining trajectory.
    #[arg(long, conflicts_with = "benchmark")]
    verify_checkpoint: Option<PathBuf>,
}

#[derive(Serialize)]
struct Timing {
    ticks: u64,
    bodies: usize,
    planned_body_solves: usize,
    elapsed_ms: f64,
    tick_p50_us: f64,
    tick_p95_us: f64,
    tick_p99_us: f64,
    integrated_pair_impulse_ns: f64,
}
fn measure(worker: &Path) -> dense::Result<Timing> {
    let scene = dense::Scene::new();
    let mut pool = scene.spawn(worker)?;
    let mut times = Vec::new();
    let mut work = 0;
    let mut load = 0.;
    for _ in 0..dense::END {
        let start = Instant::now();
        pool.advance_islands_in_field(dense::GRAVITY, &scene.loads)?;
        times.push(start.elapsed().as_secs_f64() * 1e6);
        work += scene.bodies.len() + 9 * pool.last_island_refined_bodies().unwrap().len();
        for obs in &pool.frame().workers {
            load += obs
                .snapshot
                .impulses
                .iter()
                .map(|p| p.normal_impulse)
                .sum::<f64>();
        }
    }
    let elapsed_ms = times.iter().sum::<f64>() * 0.001;
    times.sort_unstable_by(f64::total_cmp);
    let p = |q: usize| times[(times.len() * q).div_ceil(100) - 1];
    Ok(Timing {
        ticks: dense::END,
        bodies: scene.bodies.len(),
        planned_body_solves: work,
        elapsed_ms,
        tick_p50_us: p(50),
        tick_p95_us: p(95),
        tick_p99_us: p(99),
        integrated_pair_impulse_ns: load,
    })
}
fn main() -> dense::Result<()> {
    let args = Args::parse();
    if args.benchmark {
        println!("{}", serde_json::to_string(&measure(&args.worker)?)?);
    } else if let Some(path) = args.verify_checkpoint {
        let (tick, resumed) = dense::verify_previous_checkpoint(&args.worker, &path)?;
        println!("Restored barrier {tick} and matched {resumed} subsequent frames byte for byte.");
    } else if let Some(path) = args.out {
        fs::create_dir(&path)?;
        let report = dense::qualify(&args.worker, &path, true)?;
        let json = serde_json::to_string_pretty(&report)?;
        fs::write(path.join("qualification.json"), &json)?;
        println!("{json}");
    }
    Ok(())
}
