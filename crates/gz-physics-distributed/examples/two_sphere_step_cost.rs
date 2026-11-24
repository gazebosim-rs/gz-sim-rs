// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure the public queue-and-advance path for each two-sphere island layout.
use clap::Parser;
use gz_physics_distributed::multi_sphere::{MultiSphereObservation, MultiSpherePool};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use serde::Deserialize;
use serde_json::json;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::PathBuf,
    time::Instant,
};

#[derive(Parser)]
struct Args {
    sdf: PathBuf,
    /// Observations from two_sphere_hinge.
    states: PathBuf,
    /// Directory on the filesystem to measure. The private session is removed on exit.
    filesystem: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    /// Write individual timed intervals with their actual owner slots.
    #[arg(long)]
    samples: Option<PathBuf>,
    /// Apply torque and advance as one durable operation.
    #[arg(long)]
    atomic: bool,
    /// Use binary RPC over loopback Zenoh between each managed worker and the pool.
    #[arg(long)]
    zenoh: bool,
    #[arg(long, default_value_t = 4)]
    workers: u8,
    /// Configure fresh-worker attempts after rollback exhaustion.
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u8).range(0..=8))]
    reopen_attempts: u8,
}

#[derive(Deserialize)]
struct Row {
    tick: u64,
    direct: MultiSphereObservation,
    queued_effort_nm: f64,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let spec = distributed_multi_sphere_spec(
        &fs::read_to_string(&args.sdf)?,
        &args.sdf.to_string_lossy(),
        "default",
    )?;
    let joint = spec.hinge.joint_id;
    let session = tempfile::tempdir_in(args.filesystem)?;
    let transport = if args.zenoh {
        gz_physics_distributed::moving::WorkerTransport::LoopbackZenoh
    } else {
        gz_physics_distributed::moving::WorkerTransport::Pipe
    };
    let mut pool = MultiSpherePool::create_with_transport(
        &args.worker,
        session.path(),
        args.workers,
        spec,
        transport,
    )?;
    pool.set_reopen_attempts(args.reopen_attempts)?;
    let rows = BufReader::new(File::open(args.states)?)
        .lines()
        .map(|line| -> Result<Row, Box<dyn Error>> { Ok(serde_json::from_str(&line?)?) })
        .collect::<Result<Vec<_>, _>>()?;
    let mut samples = BTreeMap::<&str, Vec<u64>>::new();
    if rows.len() < 2
        || rows[0].tick != 0
        || bincode::serialize(pool.observation())? != bincode::serialize(&rows[0].direct)?
    {
        return Err("invalid initial reference state".into());
    }
    let mut intervals = Vec::new();
    for w in rows.windows(2) {
        let before = pool.owners();
        let start = Instant::now();
        if args.atomic {
            pool.advance_with_effort(joint, w[0].queued_effort_nm)?;
        } else {
            pool.queue_effort(joint, w[0].queued_effort_nm)?;
            pool.advance()?;
        }
        let ns = u64::try_from(start.elapsed().as_nanos())?;
        if w[1].tick != w[0].tick + 1
            || bincode::serialize(pool.observation())? != bincode::serialize(&w[1].direct)?
        {
            return Err(format!("reference differs at tick {}", w[1].tick).into());
        }
        let after = pool.owners();
        let key = if before != after {
            "ownership_change"
        } else {
            match [after[0] == after[1], after[0] == after[2]] {
                [false, false] if after[1] == after[2] => "sphere_pair",
                [false, false] => "separate",
                [true, true] => "all_together",
                _ => "one_sphere_with_hinge",
            }
        };
        if args.samples.is_some() {
            intervals.push((w[1].tick, before, after, ns));
        }
        samples.entry(key).or_default().push(ns);
    }
    if pool.reopen_attempts_used() != 0 {
        return Err("healthy benchmark encountered a full-pool reopen".into());
    }
    if let Some(path) = args.samples {
        serde_json::to_writer(File::create_new(path)?, &intervals)?;
    }
    let mut totals = BTreeMap::new();
    for (name, samples) in samples.iter_mut() {
        samples.sort_unstable();
        let rank = |p: usize| samples[(samples.len() * p).div_ceil(100) - 1];
        totals.insert(name,json!({"steps":samples.len(),"p50_ns":rank(50),"p95_ns":rank(95),"total_ns":samples.iter().sum::<u64>()}));
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"workers":args.workers,"steps":rows.len()-1,"layouts":totals})
        )?
    );
    Ok(())
}
