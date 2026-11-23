// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure full transfers at one fixed loaded contact, including durable publication.
use clap::Parser;
use gz_physics_distributed::articulation::{ArticulationPool, NativeArticulation};
use sha2::{Digest, Sha256};
use std::{error::Error, fs, io, path::PathBuf, time::Instant};

#[derive(Parser)]
struct Args {
    /// A committed barrier.gzhinge with positive ground reaction.
    archive: PathBuf,
    /// Existing directory on the filesystem to measure. A temporary session is created within it.
    filesystem: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t = 64, value_parser = clap::value_parser!(u16).range(1..=4096))]
    samples: u16,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let dir = tempfile::tempdir_in(&args.filesystem)?;
    let archive_bytes = fs::copy(&args.archive, dir.path().join("barrier.gzhinge"))?;
    let mut pool = ArticulationPool::resume(&args.worker, dir.path())?;
    let state = *pool.observation();
    let reactions = state.ground_reactions.ok_or("missing ground")?;
    if !reactions.iter().any(|r| r.active && r.normal_impulse > 0.) {
        return Err("a positive loaded ground contact is required".into());
    }
    let mut reference = NativeArticulation::restore(pool.spec(), state.tick, pool.image())?;
    let image = pool.image().to_vec();
    let pids: Vec<_> = (0..pool.width()).map(|i| pool.worker_pid(i)).collect();
    let epoch = pool.epoch();
    let mut samples = Vec::new();
    for i in 0..usize::from(args.samples) + 2 {
        let generation = pool.generation();
        let destination = (pool.owner() + 1) % pool.width();
        let start = Instant::now();
        pool.transfer(destination)?;
        let wall_ns = u64::try_from(start.elapsed().as_nanos())?;
        if pool.generation() != generation + 1
            || pool.epoch() != epoch
            || pool.owner() != destination
            || pool.image() != image
            || pool.observation() != &state
            || (0..pool.width()).any(|slot| pool.worker_pid(slot) != pids[usize::from(slot)])
        {
            return Err(
                "ownership, worker continuity or contact state differed after transfer".into(),
            );
        }
        if i >= 2 {
            samples.push(wall_ns);
        }
    }
    for _ in 0..16 {
        reference.advance()?;
        pool.advance()?;
        if reference.observe() != *pool.observation() || reference.capture()? != pool.image() {
            return Err("continuation after measured handoffs differs".into());
        }
    }
    let mut sorted = samples.clone();
    sorted.sort_unstable();
    let rank = |p: usize| sorted[(p * sorted.len()).div_ceil(100) - 1];
    let report = serde_json::json!({
        "filesystem":args.filesystem,"archive":args.archive,"tick":state.tick,
        "archive_bytes":archive_bytes,"image_bytes":image.len(),"image_sha256":format!("{:x}",Sha256::digest(&image)),
        "workers":pool.width(),"excluded_warmups":2,"samples_ns":samples,"p50_ns":rank(50),
        "p95_ns":rank(95),"p99_ns":rank(99),"ground_reactions":reactions,"exact_subsequent_steps":16
    });
    serde_json::to_writer_pretty(io::stdout().lock(), &report)?;
    println!();
    Ok(())
}
