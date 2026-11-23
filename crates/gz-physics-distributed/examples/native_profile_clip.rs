// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify and record a dense 64-sphere warm migration against a continuous native world.
#[path = "support/native_profile_grid.rs"]
mod grid;
#[path = "support/native_contact_control.rs"]
mod native;
use clap::Parser;
use gz_physics_distributed::{
    physics::{GroundPlane, NativeSolverProfile, PendingImpulse, SimulationStep},
    pool::{PoolNativeRecovery, WorkerPool},
};
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
    /// Fresh output directory.
    out: PathBuf,
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    fs::create_dir(&args.out)?;
    let ground = Some(GroundPlane::new(0.)?);
    let g = [0., 0., -9.81];
    let seeds = grid::grid(64, true);
    let mut native = native::Control::new(&seeds, ground, g)?;
    let mut pool = WorkerPool::spawn_with_profile(
        &args.worker,
        0x4064,
        vec![seeds, vec![], vec![]],
        SimulationStep::Ms10,
        ground,
        128,
        NativeSolverProfile::Extended64,
    )?;
    pool.set_packed_handoffs(true);
    let checkpoint = tempfile::tempdir_in(&args.out)?;
    let mut out = BufWriter::new(File::create_new(args.out.join("frames.jsonl"))?);
    let mut digest = Sha256::new();
    let mut events = vec![];
    let mut owner = 0u8;
    let mut pair_count = 0;
    let mut floor_count = 0;
    for tick in 0..=120 {
        let (pairs, floors) = if tick == 0 {
            (vec![], vec![])
        } else {
            let mut impulses = vec![];
            if matches!(tick, 20 | 45 | 98) {
                let id = if tick == 45 { 64 } else { 1 };
                let impulse = if tick == 45 {
                    [-0.4, -0.1, 0.15]
                } else {
                    [0.4, 0.1, 0.15]
                };
                pool.queue_impulse(id, impulse)?;
                impulses.push(PendingImpulse { id, impulse });
                events
                    .push(json!({"tick":tick,"operation":"COM impulse","id":id,"impulse":impulse}));
            }
            if tick == 27 {
                let before = pool.frame().clone();
                let pid = pool.worker_pid(owner);
                pool.drop_next_acknowledgement(owner)?;
                if pool.advance_adaptive_in_field(g, &[]).is_ok() || pool.frame() != &before {
                    return Err("ambiguous acknowledgment published a partial step".into());
                }
                pool.advance_adaptive_in_field(g, &[])?;
                if pool.worker_pid(owner) != pid {
                    return Err("retry restarted a healthy worker".into());
                }
                events.push(json!({"tick":tick,"operation":"lost acknowledgment and duplicate reply","owner":owner}));
            } else {
                pool.advance_adaptive_in_field(g, &[])?;
            }
            native.advance(pool.last_substeps().unwrap(), &[], &impulses)?
        };
        if matches!(tick, 4 | 90) {
            let destination = if tick == 4 { 1 } else { 2 };
            let old = pool.frame().workers[usize::from(owner)].snapshot.clone();
            pool.transfer_component(1, destination)?;
            if old != pool.frame().workers[usize::from(destination)].snapshot {
                return Err("transfer changed physical state".into());
            }
            events.push(json!({"tick":tick,"operation":"warm transfer","source":owner,"destination":destination,"members":old.bodies.len(),"pairs":old.impulses.len(),"floors":old.supports.len()}));
            owner = destination;
        }
        if tick == 52 {
            let old = pool.frame().clone();
            let pid = pool.worker_pid(owner);
            pool.terminate_worker(owner)?;
            pool.recover(owner)?;
            if old != *pool.frame() || pool.worker_pid(owner) == pid {
                return Err("replacement diverged".into());
            }
            events.push(
                json!({"tick":tick,"operation":"terminated and recovered worker","owner":owner}),
            );
        }
        if tick == 76 {
            let old = pool.frame().clone();
            let path = checkpoint.path().join("complete.gzpool");
            pool.save_native_recovery(&path)?;
            drop(pool);
            pool = PoolNativeRecovery::open(path)?.resume(&args.worker)?;
            pool.set_packed_handoffs(true);
            if old != *pool.frame()
                || pool.native_solver_profile() != NativeSolverProfile::Extended64
            {
                return Err("native reload diverged".into());
            }
            events.push(json!({"tick":tick,"operation":"GZPOOL03 persisted and resumed"}));
        }
        let w = &pool.frame().workers[usize::from(owner)];
        if w.snapshot.bodies != native.bodies
            || w.snapshot.impulses != pairs
            || w.snapshot.supports != floors
        {
            return Err(format!("independent native control differs at tick {tick}").into());
        }
        pair_count += pairs.len();
        floor_count += floors.len();
        let state = bincode::serialize(&(tick as u64, &native.bodies, &pairs, &floors))?;
        digest.update(&state);
        serde_json::to_writer(
            &mut out,
            &json!({"tick":tick,"bodies":w.snapshot.bodies,"owners":vec![owner;64],
            "generations":w.generations.values().collect::<Vec<_>>(),"pairs":pairs,"floors":floors,
            "substeps":pool.last_substeps(),"trajectory_sha256_sample":format!("{:x}",Sha256::digest(&state))}),
        )?;
        writeln!(out)?;
    }
    out.flush()?;
    fs::write(
        args.out.join("summary.json"),
        serde_json::to_vec_pretty(&json!({
            "native_step_seconds":0.01,"first_tick":0,"last_tick":120,"sphere_ids":[1,64],"ground_z":0,
            "control":"one independent 64-body native world stepped at the resolved scene-wide native rate",
            "exact_native_frames":121,"body_states":121*64,"pair_receipts":pair_count,"floor_receipts":floor_count,
            "trajectory_sha256":format!("{:x}",digest.finalize()),"events":events,
        }))?,
    )?;
    Ok(())
}
