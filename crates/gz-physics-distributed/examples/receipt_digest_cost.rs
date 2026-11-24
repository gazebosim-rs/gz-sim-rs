// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isolate canonical JSON-to-SHA receipt work on complete native exports.
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod scene;

use clap::Parser;
use gz_physics_distributed::{
    physics::{GroundPlane, SimulationStep, Snapshot, StepField},
    protocol::{Command, Outcome, Request, WarmHandoff, WarmTicket, Worker},
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    io::{self, BufWriter, Write},
    path::PathBuf,
    time::Instant,
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const CAPACITIES: [usize; 4] = [0, 512, 8192, 32768];
const RUN: u64 = 0x72656365697074;

#[derive(Parser)]
struct Args {
    #[arg(long,default_value_t=256,value_parser=clap::value_parser!(u16).range(1..))]
    rounds: u16,
    output: PathBuf,
}

struct Sink {
    hash: Sha256,
    remaining: usize,
    writes: u64,
}
impl Write for Sink {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.remaining = self
            .remaining
            .checked_sub(buf.len())
            .ok_or(io::ErrorKind::InvalidData)?;
        self.hash.update(buf);
        self.writes += 1;
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

fn export(bodies: usize, tick: u64) -> Result<Box<WarmHandoff>> {
    let groups = scene::seeds(scene::Layout::Adjacent);
    let group = groups
        .iter()
        .find(|g| g.len() == bodies)
        .ok_or("missing native group")?;
    let ids: Vec<_> = group.iter().map(|b| b.id).collect();
    let mut worker = Worker::new(
        RUN,
        0,
        Snapshot {
            step: SimulationStep::Ms10,
            bodies: group.clone(),
            ground: Some(GroundPlane::new(0.)?),
            ..Default::default()
        },
    )?;
    for t in 1..=tick {
        let wrenches = scene::wrenches(t, &groups)
            .into_iter()
            .filter(|w| ids.contains(&w.id))
            .collect();
        worker.handle(Request {
            run_id: RUN,
            worker_id: 0,
            sequence: t,
            command: Command::StepRefined {
                tick: t - 1,
                field: StepField {
                    gravity: scene::GRAVITY,
                    wrenches,
                },
            },
        })?;
    }
    let ticket = WarmTicket {
        run_id: RUN,
        serial: 1,
        source: 0,
        destination: 1,
        tick,
        generations: vec![1; ids.len()],
        ids,
    };
    let reply = worker.handle(Request {
        run_id: RUN,
        worker_id: 0,
        sequence: tick + 1,
        command: Command::ExportSolver { ticket },
    })?;
    let Outcome::WarmExported(h) = reply.outcome else {
        return Err("missing native export".into());
    };
    Ok(h)
}

fn digest(h: &WarmHandoff, capacity: usize) -> Result<(u64, usize)> {
    let limit = 4 * gz_physics_multifidelity::PARTITION_CHECKPOINT_LIMIT + 256 * 1024 - 16 * 1024;
    let mut sink = Sink {
        hash: Sha256::new(),
        remaining: limit,
        writes: 0,
    };
    sink.hash
        .update(b"gz-physics-distributed/solver-handoff/v1\0");
    let payload = (&h.receipt.ticket, &h.checkpoint);
    if capacity == 0 {
        serde_json::to_writer(&mut sink, &payload)?;
    } else {
        let mut buf = BufWriter::with_capacity(capacity, &mut sink);
        serde_json::to_writer(&mut buf, &payload)?;
        buf.flush()?;
    }
    let hash: [u8; 32] = sink.hash.finalize().into();
    if hash != h.receipt.digest {
        return Err("receipt digest differs".into());
    }
    Ok((sink.writes, limit - sink.remaining))
}

#[derive(Serialize)]
struct Measurement {
    calls: u64,
    body_bytes: usize,
    native_bytes: usize,
    samples: usize,
    median_us: f64,
    p95_us: f64,
    us: Vec<f64>,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut cases = BTreeMap::new();
    for (name, bodies, tick) in [("stack", 2, 75), ("loaded", 25, 75), ("released", 25, 100)] {
        let export = export(bodies, tick)?;
        let mut samples: BTreeMap<usize, Vec<f64>> = BTreeMap::new();
        for round in 0..=args.rounds {
            for n in 0..CAPACITIES.len() {
                let capacity = CAPACITIES[(usize::from(round) + n) % CAPACITIES.len()];
                let start = Instant::now();
                let _ = digest(std::hint::black_box(&export), capacity)?;
                let elapsed = start.elapsed().as_secs_f64() * 1e6;
                if round > 0 {
                    samples.entry(capacity).or_default().push(elapsed)
                }
            }
        }
        let mut result = BTreeMap::new();
        for (capacity, us) in samples {
            let mut sorted = us.clone();
            sorted.sort_by(f64::total_cmp);
            let n = us.len();
            let (calls, body_bytes) = digest(&export, capacity)?;
            result.insert(
                capacity,
                Measurement {
                    calls,
                    body_bytes,
                    native_bytes: export.checkpoint.native_bytes(),
                    samples: n,
                    median_us: (sorted[(n - 1) / 2] + sorted[n / 2]) / 2.,
                    p95_us: sorted[(95 * n).div_ceil(100) - 1],
                    us,
                },
            );
        }
        cases.insert(name, result);
    }
    fs::write(
        args.output,
        format!("{}\n", serde_json::to_string_pretty(&cases)?),
    )?;
    Ok(())
}
