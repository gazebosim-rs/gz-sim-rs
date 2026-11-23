// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify fresh-session and current-session returns to live measured placement.
#[allow(dead_code)]
#[path = "support/changing_oracle.rs"]
mod control;
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod fixture;
use clap::Parser;
use control::Oracle;
use gz_physics_distributed::{
    physics::{GroundPlane, SimulationStep},
    pool::{PlacementRecord, PoolNativeRecovery, WorkerPool},
};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    path::PathBuf,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    out: PathBuf,
}
fn configure(p: &mut WorkerPool) -> Result<()> {
    p.set_periodic_placement(Some("4,8,64,0".parse()?))?;
    p.set_local_helper_limit(Some(0));
    Ok(())
}
fn run(a: &Args, reference: &Oracle, width: usize, fresh: bool) -> Result<Value> {
    let mut seeds = fixture::Placement::Single.worker_seeds(&reference.groups);
    seeds.resize_with(width, Vec::new);
    let mut p = WorkerPool::spawn(
        &a.worker,
        81991,
        seeds,
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        64,
    )?;
    configure(&mut p)?;
    let mut owners: BTreeMap<u64, (u8, u64)> = reference
        .groups
        .iter()
        .flatten()
        .map(|b| (b.id, (0, 0)))
        .collect();
    let mut moves = vec![];
    let mut observations = vec![];
    let mut old_status = None;
    let mut birth_pids = None;
    for t in 1..=33 {
        let inputs = fixture::wrenches(t, &reference.groups);
        if (2..=8).contains(&t) {
            p.advance_islands_with_placement(
                fixture::GRAVITY,
                &inputs,
                &PlacementRecord {
                    target_tick: t,
                    optional_move: None,
                },
            )?;
        } else {
            p.advance_islands_in_field(fixture::GRAVITY, &inputs)?;
        }
        let r = p.last_island_placement().ok_or("missing receipt")?;
        if r.target_tick != t {
            return Err("incorrect committed placement clock".into());
        }
        if let Some(m) = &r.optional_move {
            if t < 13 {
                return Err("overlapping live window survived a recorded barrier".into());
            }
            for b in &m.members {
                let owner = owners.get_mut(&b.id).ok_or("unknown member")?;
                if *owner != (m.source, b.generation) {
                    return Err("stale measured move fence".into());
                }
                *owner = (m.destination, b.generation + 1);
            }
            moves.push(r.clone());
        }
        reference.verify(p.frame(), &owners)?;
        let s = p.placement_status().unwrap();
        if [1, 2, 4, 8, 9, 12, 13, 17, 21, 25, 29, 33].contains(&t) {
            observations.push(
                json!({"tick":t,"started":s.windows_started,"completed":s.windows_completed,
                "interrupted":s.windows_interrupted,"committed_moves":s.committed_moves}),
            );
        }
        if t == 1 && s.windows_started != 1 {
            return Err("initial live window failed to start; requires multiple CPU lanes".into());
        }
        if t == 2 && (s.windows_interrupted != 1 || s.windows_completed != 0) {
            return Err("stale live window was retained".into());
        }
        if t == 8 && s.windows_started != u64::from(!fresh) {
            return Err("replay started a timing window".into());
        }
        if t == 9 && s.windows_started != 1 + u64::from(!fresh) {
            return Err("ordinary live placement failed to resume at tick 8".into());
        }
        if fresh && t == 4 {
            old_status = Some(
                json!({"started":s.windows_started,"interrupted":s.windows_interrupted,"completed":s.windows_completed}),
            );
            let dir = tempfile::tempdir_in(&a.out)?;
            let path = dir.path().join("mid-replay.gzpool");
            let old: Vec<_> = (0..width).map(|w| p.worker_pid(w as u8)).collect();
            let f = p.frame().clone();
            p.save_native_recovery(&path)?;
            drop(p);
            p = PoolNativeRecovery::open(path)?.resume(&a.worker)?;
            let new: Vec<_> = (0..width).map(|w| p.worker_pid(w as u8)).collect();
            if p.frame() != &f
                || p.placement_status().is_some()
                || p.last_island_placement().is_some()
                || old.iter().zip(&new).any(|(a, b)| a == b)
            {
                return Err("invalid fresh pool".into());
            }
            birth_pids = Some(json!({"before":old,"after":new}));
            configure(&mut p)?;
        }
    }
    if moves.is_empty() || p.placement_status().unwrap().windows_completed == 0 {
        return Err("resumed sampling never selected a real loaded move".into());
    }
    Ok(
        json!({"width":width,"fresh_at_tick_4":fresh,"retired_sampler":old_status,"restart_pids":birth_pids,
      "observations":observations,"moves":moves,"native_verified_ticks":33}),
    )
}
fn main() -> Result<()> {
    let a = Args::parse();
    fs::create_dir_all(&a.out)?;
    let mut rows = vec![];
    for layout in [fixture::Layout::Adjacent, fixture::Layout::Separated] {
        let oracle = Oracle::new(layout)?;
        for width in [3, 4, 8] {
            for fresh in [false, true] {
                rows.push(json!({"layout":layout,"trial":run(&a,&oracle,width,fresh)?}));
            }
        }
    }
    serde_json::to_writer_pretty(
        File::create_new(a.out.join("sampler-transitions.json"))?,
        &rows,
    )?;
    Ok(())
}
