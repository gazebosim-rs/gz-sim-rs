// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify a mixed four-worker run or benchmark both public stepping policies.
#[path = "support/mixed_islands.rs"]
mod mixed;
use clap::Parser;
use serde::Serialize;
use std::{
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::Instant,
};

#[derive(Parser)]
struct Args {
    /// New output directory for either the qualification or the timing series.
    output: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    /// Run matched pairs instead of injecting failures.
    #[arg(long)]
    benchmark: bool,
    /// Number of noninteracting dynamic bodies accompanying the three contact bodies.
    #[arg(long, default_value_t = 64)]
    free_bodies: usize,
    #[arg(long, default_value_t = 8)]
    rounds: usize,
}

#[derive(Serialize)]
struct Timing {
    round: usize,
    policy: &'static str,
    quiet_bodies: usize,
    ticks: u64,
    planned_body_solves: u64,
    elapsed_ms: f64,
    tick_p50_us: f64,
    tick_p95_us: f64,
    tick_p99_us: f64,
    quiet_end_z_m: f64,
    pair_load_ns: f64,
    ground_load_ns: f64,
}
fn measure(s: &mixed::Scene, worker: &Path, round: usize, local: bool) -> mixed::Result<Timing> {
    let mut pool = s.spawn(worker)?;
    let mut durations = Vec::new();
    let mut solves = 0;
    let mut pair = 0.;
    let mut ground = 0.;
    for _ in 1..=mixed::END {
        let start = Instant::now();
        if local {
            pool.advance_islands_in_field(mixed::GRAVITY, &s.loads)?;
        } else {
            pool.advance_adaptive_in_field(mixed::GRAVITY, &s.loads)?;
        }
        durations.push(start.elapsed().as_secs_f64());
        solves += if local {
            s.bodies.len() + 9 * pool.last_island_refined_bodies().unwrap().len()
        } else {
            s.bodies.len() * usize::from(pool.last_substeps().unwrap())
        };
        for w in &pool.frame().workers {
            pair += w
                .snapshot
                .impulses
                .iter()
                .map(|p| p.normal_impulse)
                .sum::<f64>();
            ground += w
                .snapshot
                .supports
                .iter()
                .map(|p| p.normal_impulse)
                .sum::<f64>();
        }
    }
    assert!(pair > 1. && ground > 1.);
    let elapsed_ms = durations.iter().sum::<f64>() * 1000.;
    durations.sort_unstable_by(f64::total_cmp);
    let p = |q: usize| durations[(durations.len() * q).div_ceil(100) - 1] * 1e6;
    Ok(Timing {
        round,
        policy: if local { "island" } else { "scene" },
        quiet_bodies: s.bodies.len() - 3,
        ticks: mixed::END,
        planned_body_solves: solves as u64,
        elapsed_ms,
        tick_p50_us: p(50),
        tick_p95_us: p(95),
        tick_p99_us: p(99),
        quiet_end_z_m: pool.frame().body(4).unwrap().1.position[2],
        pair_load_ns: pair,
        ground_load_ns: ground,
    })
}
fn main() -> mixed::Result<()> {
    let a = Args::parse();
    if a.benchmark
        && (a.rounds < 2 || !a.rounds.is_multiple_of(2) || !(1..=512).contains(&a.free_bodies))
    {
        return Err(
            "benchmark requires an even number of rounds >=2 and 1..=512 free bodies".into(),
        );
    }
    fs::create_dir(&a.output)?;
    if a.benchmark {
        let s = mixed::Scene::new(a.free_bodies)?;
        let mut out = BufWriter::new(File::create_new(a.output.join("timings.jsonl"))?);
        for round in 0..a.rounds {
            for local in [round % 2 == 0, round % 2 != 0] {
                let result = measure(&s, &a.worker, round, local)?;
                serde_json::to_writer(&mut out, &result)?;
                writeln!(out)?;
                out.flush()?;
                println!("{}", serde_json::to_string(&result)?);
            }
        }
    } else {
        let r = mixed::qualify(&a.worker, &a.output, true)?;
        let json = serde_json::to_string_pretty(&r)?;
        fs::write(a.output.join("qualification.json"), &json)?;
        println!("{json}");
    }
    Ok(())
}
