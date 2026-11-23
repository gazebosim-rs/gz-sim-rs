// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure end-to-end restart of a two-worker run at increasingly late barriers.
use clap::Parser;
use gz_physics_distributed::{
    coordinator::{Coordinator, NativeRecoveryPlan, RecoveryPlan},
    physics::fixture,
    transport::WorkerProcess,
};
use std::{
    error::Error,
    fs, io,
    path::{Path, PathBuf},
    time::Instant,
};

#[derive(Parser)]
struct Args {
    /// Keep both immutable archive formats in this directory.
    directory: PathBuf,
    #[arg(long)]
    worker: Option<PathBuf>,
    #[arg(long, value_delimiter = ',', default_value = "150,1500,6000")]
    ticks: Vec<usize>,
    #[arg(long, default_value_t = 5)]
    repetitions: usize,
}

fn measure(
    worker: &Path,
    directory: &Path,
    ticks: usize,
    repetitions: usize,
) -> Result<(), Box<dyn Error>> {
    let run = 2026111900;
    let seeds = fixture();
    let peers = [
        WorkerProcess::spawn(worker, run, 0, &seeds[..1])?,
        WorkerProcess::spawn(worker, run, 1, &seeds[1..])?,
    ];
    let mut c = Coordinator::new_recorded(run, peers, ticks + 32)?;
    let mut migrated = false;
    for _ in 0..ticks {
        if c.frame().tick == 10 {
            c.transfer(1, true)?;
        }
        if !migrated && c.frame().workers[1].snapshot.contacts > 0 {
            c.transfer_group(1)?;
            migrated = true;
        }
        c.advance_overlapped()?;
    }
    if !migrated {
        return Err(io::Error::other("checkpoint must follow the warm handoff").into());
    }
    c.queue_impulse(1, [0.25, 0., 0.])?;
    let native = directory.join(format!("tick-{ticks}.gznative"));
    let seed = directory.join(format!("tick-{ticks}.gzrec"));
    let before = Instant::now();
    c.save_recovery(&seed)?;
    let seed_save = before.elapsed();
    let before = Instant::now();
    c.save_native_recovery(&native)?;
    let native_save = before.elapsed();
    let baseline = c.frame().clone();
    let mut expected = Vec::new();
    for _ in 0..8 {
        expected.push(c.advance()?.clone());
    }
    drop(c);
    for repetition in 0..repetitions {
        let order = if repetition % 2 == 0 {
            [false, true]
        } else {
            [true, false]
        };
        for native_mode in order {
            let (path, mode, save) = if native_mode {
                (&native, "native", native_save)
            } else {
                (&seed, "seed", seed_save)
            };
            let before = Instant::now();
            let mut restored = if native_mode {
                NativeRecoveryPlan::open(path)?.resume(worker)?
            } else {
                RecoveryPlan::open(path)?
                    .resume(|run, id, boxes| WorkerProcess::spawn(worker, run, id, boxes))?
            };
            let latency = before.elapsed();
            if restored.frame() != &baseline {
                return Err(io::Error::other("restored barrier differs").into());
            }
            for frame in &expected {
                if restored.advance()? != frame {
                    return Err(io::Error::other("future trajectory differs").into());
                }
            }
            println!(
                "{ticks},{},{mode},{},{:.3},{:.3}",
                repetition + 1,
                fs::metadata(path)?.len(),
                save.as_secs_f64() * 1000.,
                latency.as_secs_f64() * 1000.
            );
        }
    }
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    if args.repetitions == 0 || args.ticks.iter().any(|&t| !(150..=999_950).contains(&t)) {
        return Err(io::Error::other(
            "choose at least one repetition and 150 to 999950 ticks per recovery point",
        )
        .into());
    }
    let worker = match args.worker {
        Some(path) => path,
        None => std::env::current_exe()?
            .ancestors()
            .nth(2)
            .ok_or_else(|| io::Error::other("pass --worker"))?
            .join(format!("physics_worker{}", std::env::consts::EXE_SUFFIX)),
    };
    fs::create_dir_all(&args.directory)?;
    println!("tick,repetition,method,file_bytes,save_ms,restore_ms");
    for ticks in args.ticks {
        measure(&worker, &args.directory, ticks, args.repetitions)?;
    }
    Ok(())
}
