// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure durable advance calls from a moving-hinge cut against an ordinary Bevy trace.
use clap::Parser;
use gz_physics_distributed::moving::{MovingObservation, MovingPool};
use serde::Deserialize;
use serde_json::json;
use std::{
    error::Error,
    fs::{self, File},
    io::{self, BufRead, BufReader},
    path::PathBuf,
    time::Instant,
};

#[derive(Parser)]
struct Args {
    /// Source barrier.gzmoving, copied to private storage for the benchmark.
    archive: PathBuf,
    /// Directory on the filesystem to measure.
    filesystem: PathBuf,
    /// states.jsonl recorded by the moving_hinge example.
    reference: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    /// Include the recorded nonzero command for every interval in the timed work.
    #[arg(long)]
    continuous: bool,
    #[arg(long, default_value_t = 1600)]
    through: u64,
}

#[derive(Deserialize)]
struct ReferenceRow {
    tick: u64,
    direct: MovingObservation,
    #[serde(default)]
    queued_effort_nm: Option<f64>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let dir = tempfile::tempdir_in(&args.filesystem)?;
    fs::copy(args.archive, dir.path().join("barrier.gzmoving"))?;
    let mut pool = MovingPool::resume(&args.worker, dir.path())?;
    let start = pool.observation().hinge.tick;
    if args.through <= start {
        return Err("end tick must follow the initial cut".into());
    }
    let rows = BufReader::new(File::open(args.reference)?)
        .lines()
        .map(|line| -> Result<ReferenceRow, Box<dyn Error>> { Ok(serde_json::from_str(&line?)?) })
        .collect::<Result<Vec<_>, _>>()?;
    let rows = rows
        .get(usize::try_from(start)?..=usize::try_from(args.through)?)
        .ok_or("reference does not cover the requested interval")?;
    if rows[0].tick != start || rows[0].direct != *pool.observation() {
        return Err("initial cut differs from independent reference".into());
    }
    let mut samples = Vec::new();
    let mut joined_steps = 0;
    let mut transitions = Vec::new();
    let mut owners = pool.owners();
    let mut pending = if args.continuous {
        let image = pool
            .image()
            .ok_or("continuous benchmark requires a warm archive")?;
        gz_physics_multifidelity::MultiFidelityBackend::moving_checkpoint_effort(image)?
    } else {
        None
    };
    let joint = pool.spec().hinge.joint_id;
    let mut commands = 0;
    for interval in rows.windows(2) {
        let row = &interval[1];
        let effort = interval[0].queued_effort_nm;
        let begin = Instant::now();
        if args.continuous {
            let effort = effort
                .filter(|value| value.is_finite() && *value != 0.)
                .ok_or("missing nonzero torque for a continuously driven interval")?;
            match pending.take() {
                Some(saved) if saved == effort => (),
                Some(_) => {
                    return Err("initial archive command differs from the recorded drive".into());
                }
                None => {
                    pool.queue_effort(joint, effort)?;
                    commands += 1;
                }
            }
        }
        pool.advance()?;
        let ns = u64::try_from(begin.elapsed().as_nanos())?;
        if row.tick != pool.observation().hinge.tick || row.direct != *pool.observation() {
            return Err(format!("step {} differs from independent reference", row.tick).into());
        }
        let current = pool.owners();
        if current != owners {
            transitions.push(json!({"tick":row.tick,"owners":current}));
        }
        owners = current;
        joined_steps += usize::from(owners[0] == owners[1]);
        samples.push(ns);
    }
    let mut sorted = samples.clone();
    sorted.sort_unstable();
    let rank = |n: usize| sorted[(sorted.len() * n).div_ceil(100) - 1];
    serde_json::to_writer(
        io::stdout().lock(),
        &json!({
            "start_tick":start,"through_tick":args.through,"workers":pool.width(),
            "continuous":args.continuous,"commands_queued_in_window":commands,
            "joined_steps":joined_steps,"steps":samples.len(),"transitions":transitions,
            "total_ns":samples.iter().sum::<u64>(),"p50_ns":rank(50),"p95_ns":rank(95),
            "samples_ns":samples
        }),
    )?;
    println!();
    Ok(())
}
