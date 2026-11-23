// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure canonical journal hashing on real native observations and warm exports.
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod fixture;
use clap::Parser;
use gz_physics_distributed::{
    physics::{GroundPlane, SimulationStep, Snapshot, StepField},
    protocol::{Command, Outcome, Reply, Request, WarmTicket, Worker},
};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    io::{self, BufWriter, Write},
    path::PathBuf,
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const RUN: u64 = 492318;
const CAPACITIES: [usize; 5] = [0, 256, 1024, 8192, 32768];
#[derive(Parser)]
struct Args {
    #[arg(long,default_value_t=256,value_parser=clap::value_parser!(u16).range(1..))]
    rounds: u16,
    out: PathBuf,
}
struct CountingSink {
    hash: Sha256,
    writes: usize,
    bytes: usize,
}
impl Write for CountingSink {
    fn write(&mut self, b: &[u8]) -> io::Result<usize> {
        self.writes += 1;
        self.bytes += b.len();
        self.hash.update(b);
        Ok(b.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}
fn digest(reply: &Reply, capacity: usize) -> Result<([u8; 32], usize, usize)> {
    let mut out = CountingSink {
        hash: Sha256::new(),
        writes: 0,
        bytes: 0,
    };
    if capacity == 0 {
        bincode::serialize_into(&mut out, reply)?;
    } else {
        let mut buffer = BufWriter::with_capacity(capacity, &mut out);
        bincode::serialize_into(&mut buffer, reply)?;
        buffer.flush()?;
    }
    Ok((out.hash.finalize().into(), out.writes, out.bytes))
}
fn replies() -> Result<BTreeMap<String, Reply>> {
    let groups = fixture::seeds(fixture::Layout::Adjacent);
    let mut corpus = BTreeMap::new();
    for (name, groups) in [
        ("empty", vec![]),
        ("two", vec![groups.iter().find(|g| g.len() == 2).unwrap()]),
        ("pad", vec![groups.iter().find(|g| g.len() == 25).unwrap()]),
        ("all", groups.iter().collect()),
    ] {
        let bodies: Vec<_> = groups.iter().flat_map(|g| g.iter().cloned()).collect();
        let mut worker = Worker::new(
            RUN,
            0,
            Snapshot {
                ground: Some(GroundPlane::new(0.)?),
                step: SimulationStep::Ms10,
                bodies,
                ..Default::default()
            },
        )?;
        let mut sequence = 0;
        for t in 1..=75 {
            sequence += 1;
            let reply = worker.handle(Request {
                run_id: RUN,
                worker_id: 0,
                sequence,
                command: Command::StepRefined {
                    tick: t - 1,
                    field: StepField {
                        gravity: fixture::GRAVITY,
                        wrenches: vec![],
                    },
                },
            })?;
            if t == 75 {
                corpus.insert(name.to_string(), reply);
            }
        }
        if name == "pad" {
            let ids: Vec<_> = groups[0].iter().map(|b| b.id).collect();
            let ticket = WarmTicket {
                run_id: RUN,
                serial: 1,
                source: 0,
                destination: 1,
                tick: 75,
                generations: vec![1; ids.len()],
                ids,
            };
            let reply = worker.handle(Request {
                run_id: RUN,
                worker_id: 0,
                sequence: sequence + 1,
                command: Command::ExportSolver { ticket },
            })?;
            if !matches!(reply.outcome, Outcome::WarmExported(_)) {
                return Err("missing export".into());
            }
            corpus.insert("warm_export".to_string(), reply);
        }
    }
    Ok(corpus)
}
fn main() -> Result<()> {
    let a = Args::parse();
    fs::create_dir(&a.out)?;
    let corpus = replies()?;
    let mut results = BTreeMap::new();
    for (name, reply) in corpus {
        // Off-path independent full-buffer SHA attests canonical historical bytes.
        let canonical = bincode::serialize(&reply)?;
        let expected: [u8; 32] = Sha256::digest(&canonical).into();
        let mut times = CAPACITIES.map(|_| Vec::new());
        for round in 0..=usize::from(a.rounds) {
            for i in 0..CAPACITIES.len() {
                let n = (round + i) % CAPACITIES.len();
                let at = Instant::now();
                let (hash, _, size) = digest(std::hint::black_box(&reply), CAPACITIES[n])?;
                let elapsed = at.elapsed().as_secs_f64() * 1e6;
                if hash != expected || size != canonical.len() {
                    return Err("canonical receipt changed".into());
                }
                if round != 0 {
                    times[n].push(elapsed);
                }
            }
        }
        let mut capacities = BTreeMap::new();
        for (cap, mut group) in CAPACITIES.into_iter().zip(times) {
            group.sort_by(f64::total_cmp);
            let n = group.len();
            capacities.insert(
                cap,
                json!({"writes":digest(&reply,cap)?.1,"median_us":(group[n/2]+group[(n-1)/2])/2.,
        "p95_us":group[(95*n).div_ceil(100)-1]}),
            );
        }
        results.insert(name,json!({"bytes":canonical.len(),"sha256":format!("{:x}",Sha256::digest(&canonical)),"capacities":capacities}));
    }
    fs::write(
        a.out.join("summary.json"),
        serde_json::to_vec_pretty(&results)?,
    )?;
    Ok(())
}
