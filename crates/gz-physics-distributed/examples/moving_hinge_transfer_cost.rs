// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time ownership changes at a fixed warm moving-body contact.
use clap::Parser;
use gz_physics_distributed::moving::MovingPool;
use std::{error::Error, fs, io, path::PathBuf, time::Instant};

#[derive(Parser)]
struct Args {
    /// A warm barrier.gzmoving with a positive cached and solved contact impulse.
    archive: PathBuf,
    /// The filesystem where the measured pool should publish its durable barriers.
    filesystem: PathBuf,
    /// An independently owned cut after this same encounter, for advancing-call timings.
    #[arg(long)]
    split_archive: Option<PathBuf>,
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t = 64, value_parser = clap::value_parser!(u16).range(1..=4096))]
    samples: u16,
}

fn timings(samples: Vec<u64>) -> serde_json::Value {
    let mut sorted = samples.clone();
    sorted.sort_unstable();
    let rank = |p: usize| sorted[(p * sorted.len()).div_ceil(100) - 1];
    serde_json::json!({"samples_ns":samples, "p50_ns":rank(50), "p95_ns":rank(95), "p99_ns":rank(99)})
}

fn time_advances(
    args: &Args,
    archive: &std::path::Path,
    joined: bool,
) -> Result<serde_json::Value, Box<dyn Error>> {
    let dir = tempfile::tempdir_in(&args.filesystem)?;
    let path = dir.path().join("barrier.gzmoving");
    fs::copy(archive, &path)?;
    let mut reference = MovingPool::resume(&args.worker, dir.path())?;
    let n = usize::from(args.samples) + 2;
    let mut expected = Vec::with_capacity(n);
    for _ in 0..n {
        reference.advance()?;
        expected.push((
            *reference.observation(),
            reference.image().map(<[u8]>::to_vec),
        ));
    }
    drop(reference);
    fs::copy(archive, &path)?;
    let mut pool = MovingPool::resume(&args.worker, dir.path())?;
    let owners = pool.owners();
    let tick = pool.observation().hinge.tick;
    if (owners[0] == owners[1]) != joined {
        return Err("incorrect ownership for advancing benchmark".into());
    }
    let mut samples = Vec::new();
    for (i, (state, image)) in expected.into_iter().enumerate() {
        let start = Instant::now();
        pool.advance()?;
        let elapsed = u64::try_from(start.elapsed().as_nanos())?;
        if state != *pool.observation()
            || image.as_deref() != pool.image()
            || pool.owners() != owners
        {
            return Err(
                "timed advancing interval changed placement or disagreed with replay".into(),
            );
        }
        if i >= 2 {
            samples.push(elapsed);
        }
    }
    let mut report = timings(samples);
    report["owners"] = serde_json::json!(owners);
    report["start_tick"] = serde_json::json!(tick);
    Ok(report)
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let dir = tempfile::tempdir_in(&args.filesystem)?;
    let control = tempfile::tempdir_in(&args.filesystem)?;
    let file = "barrier.gzmoving";
    let archive_bytes = fs::copy(&args.archive, dir.path().join(file))?;
    fs::copy(&args.archive, control.path().join(file))?;
    let mut pool = MovingPool::resume(&args.worker, dir.path())?;
    let mut reference = MovingPool::resume(&args.worker, control.path())?;
    let state = *pool.observation();
    if !state
        .reactions
        .iter()
        .any(|r| r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
    {
        return Err("a loaded contact cut with a retained impulse is required".into());
    }
    let image = pool.image().ok_or("requires a joined world")?.to_owned();
    let pids: Vec<_> = (0..pool.width()).map(|i| pool.worker_pid(i)).collect();
    let epoch = pool.epoch();
    let mut samples = Vec::new();
    for i in 0..usize::from(args.samples) + 2 {
        let generation = pool.generation();
        let destination = (pool.owners()[0] + 1) % pool.width();
        let start = Instant::now();
        pool.transfer(destination)?;
        let elapsed = u64::try_from(start.elapsed().as_nanos())?;
        if pool.generation() != generation + 1
            || pool.epoch() != epoch
            || pool.owners() != [destination; 2]
            || pool.image() != Some(image.as_slice())
            || pool.observation() != &state
            || (0..pool.width()).any(|slot| pool.worker_pid(slot) != pids[usize::from(slot)])
        {
            return Err("handoff changed the contact cut or replaced a worker".into());
        }
        if i >= 2 {
            samples.push(elapsed);
        }
    }
    for _ in 0..16 {
        reference.advance()?;
        pool.advance()?;
        if reference.observation() != pool.observation() || reference.image() != pool.image() {
            return Err("post-handoff trajectory differs from the stationary-owner control".into());
        }
    }
    drop(pool);
    drop(reference);
    let mut report = serde_json::json!({
        "filesystem":args.filesystem,"archive":args.archive,"tick":state.hinge.tick,
        "archive_bytes":archive_bytes,"image_bytes":image.len(),"workers":pids.len(),
        "excluded_warmups":2,"handoff":timings(samples),
        "reactions":state.reactions,"exact_subsequent_steps":16
    });
    if let Some(split) = &args.split_archive {
        report["advance_joined"] = time_advances(&args, &args.archive, true)?;
        report["advance_split"] = time_advances(&args, split, false)?;
    }
    serde_json::to_writer_pretty(io::stdout().lock(), &report)?;
    println!();
    Ok(())
}
