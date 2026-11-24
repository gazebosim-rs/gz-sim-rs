// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record four simultaneous 64-member native graphs and a partial worker evacuation.
#[path = "support/resident_capacity.rs"]
mod capacity;
#[path = "support/native_contact_control.rs"]
mod native;
use clap::Parser;
use gz_physics_distributed::pool::WorkerPool;
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// Fresh directory for observed frames and verified native events.
    out: PathBuf,
}
type Result<T> = std::result::Result<T, Box<dyn Error>>;

struct Recording {
    out: BufWriter<File>,
    hash: Sha256,
    body_states: usize,
    pair_receipts: usize,
    floor_receipts: usize,
}
impl Recording {
    fn sample(&mut self, pool: &WorkerPool) -> Result<()> {
        let frame = pool.frame();
        let mut rows = vec![];
        let mut pairs = vec![];
        let mut floors = vec![];
        for (owner, obs) in frame.workers.iter().enumerate() {
            for body in &obs.snapshot.bodies {
                rows.push((body, owner, obs.generations[&body.id]));
            }
            pairs.extend(&obs.snapshot.impulses);
            floors.extend(&obs.snapshot.supports);
        }
        rows.sort_by_key(|(body, _, _)| body.id);
        pairs.sort_by_key(|p: &&gz_physics_distributed::physics::ContactImpulse| (p.a, p.b));
        floors.sort_by_key(|p: &&gz_physics_distributed::physics::GroundSupport| p.body);
        let bodies: Vec<_> = rows.iter().map(|r| r.0).collect();
        let owners: Vec<_> = rows.iter().map(|r| r.1).collect();
        let generations: Vec<_> = rows.iter().map(|r| r.2).collect();
        let state = bincode::serialize(&(frame.tick, &bodies, &pairs, &floors))?;
        self.hash.update(&state);
        self.body_states += bodies.len();
        self.pair_receipts += pairs.len();
        self.floor_receipts += floors.len();
        let components: Vec<_> = frame.workers.iter().map(|w| &w.components).collect();
        serde_json::to_writer(
            &mut self.out,
            &json!({
                "tick":frame.tick, "bodies":bodies, "owners":owners, "generations":generations,
                "pairs":pairs,"floors":floors,"components":components,"substeps":pool.last_substeps(),
                "trajectory_sha256_sample":format!("{:x}",Sha256::digest(&state))
            }),
        )?;
        writeln!(self.out)?;
        Ok(())
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.out)?;
    let scratch = tempfile::tempdir_in(&args.out)?;
    let mut recording = Recording {
        out: BufWriter::new(File::create_new(args.out.join("frames.jsonl"))?),
        hash: Sha256::new(),
        body_states: 0,
        pair_receipts: 0,
        floor_receipts: 0,
    };
    let stats = capacity::exercise(&args.worker, true, scratch.path(), 20, |pool| {
        recording.sample(pool)
    })?;
    recording.out.flush()?;
    let groups: Vec<_> = (0..4)
        .map(|s| {
            json!({
                "index":s,"ids":[64*s+1,64*s+64],"center_x_m":15*s,
                "initial_ring":"4x4x4, x fastest, y next, z layers of 16"
            })
        })
        .collect();
    fs::write(
        args.out.join("summary.json"),
        serde_json::to_vec_pretty(&json!({
            "first_tick":0,"last_tick":20,"native_step_seconds":0.01,"exact_native_frames":21,
            "body_states":recording.body_states,"pair_receipts":recording.pair_receipts,"floor_receipts":recording.floor_receipts,
            "trajectory_sha256":format!("{:x}",recording.hash.finalize()),"groups":groups,"sphere_radius_m":0.2,"ground_z":0,
            "control":"one uninterrupted 256-body native world and four independent 64-body native worlds with identical physical inputs at every resolved native substep",
            "events":stats.events,"peak_resident_pair_receipts":stats.peak_pairs,"peak_resident_floor_receipts":stats.peak_floor,
            "peak_worker_barrier_reply_bytes":stats.peak_worker_barrier_reply_bytes,"full_archive_bytes":stats.full_archive_bytes,"native_solver_bytes":stats.native_bytes
        }))?,
    )?;
    Ok(())
}
