// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Capture actual owners and warm receipts around the cross-worker checkpoint cost fixture.
#[path = "support/changing_oracle.rs"]
mod control;
#[path = "support/changing_worlds.rs"]
#[allow(dead_code, unused_imports)]
mod fixture;
use clap::{Parser, ValueEnum};
use control::Oracle;
use gz_physics_distributed::{
    physics::{GroundPlane, SimulationStep},
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, value_enum)]
    layout: fixture::Layout,
    #[arg(long,default_value_t=8,value_parser=clap::value_parser!(u8).range(3..=8))]
    width: u8,
    directory: PathBuf,
}
fn observed(frame: &PoolFrame) -> Result<Vec<u8>> {
    (1..=62)
        .map(|id| {
            frame
                .body(id)
                .map(|(w, _)| w)
                .ok_or_else(|| format!("missing observed body {id}").into())
        })
        .collect()
}
fn receipts(frame: &PoolFrame) -> Value {
    let pairs: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.impulses)
        .collect();
    let floors: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.supports)
        .collect();
    json!({"pairs":pairs.len(),"positive_pairs":pairs.iter().filter(|p|p.normal_impulse>0.).count(),"floors":floors.len(),"positive_floors":floors.iter().filter(|p|p.normal_impulse>0.).count()})
}
fn main() -> Result<()> {
    let a = Args::parse();
    let oracle = Oracle::new(a.layout)?;
    let mut seeds = vec![vec![]; a.width as usize];
    let mut owners = BTreeMap::new();
    for (i, g) in oracle.groups.iter().enumerate() {
        let w = i % a.width as usize;
        seeds[w].extend_from_slice(g);
        owners.extend(g.iter().map(|b| (b.id, (w as u8, 0))));
    }
    let group = oracle
        .groups
        .iter()
        .find(|g| g.len() == 25)
        .ok_or("missing loaded pad")?;
    let members: Vec<_> = group.iter().map(|b| b.id).collect();
    let mut pool = WorkerPool::spawn(
        &a.worker,
        0x62617365,
        seeds,
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        256,
    )?;
    fs::create_dir(&a.directory)?;
    let file = a.directory.join("tick-75.gzpool");
    let first = observed(pool.frame())?;
    let mut last = first.clone();
    let mut map = vec![json!({"tick":0,"owners":first})];
    let mut barrier = Value::Null;
    let mut counts = vec![];
    for t in 1..=fixture::END {
        pool.advance_islands_in_field(fixture::GRAVITY, &fixture::wrenches(t, &oracle.groups))?;
        if t == 70 {
            let next = (owners[&group[0].id].0 + 1) % a.width;
            pool.transfer_component(group[0].id, next)?;
            for b in group {
                owners.insert(b.id, (next, 1));
            }
        }
        oracle.verify(pool.frame(), &owners)?;
        let current = observed(pool.frame())?;
        if current != last {
            if t != 70 {
                return Err("unexpected ownership change".into());
            }
            map.push(json!({"tick":t,"owners":current}));
            last = current;
        }
        if (69..=77).contains(&t) {
            counts.push(json!({"tick":t,"contacts":receipts(pool.frame())}));
        }
        if t == 75 {
            let before = pool.frame().clone();
            let counts = |pool: &WorkerPool| {
                pool.traffic().iter().fold([0u64; 3], |s, p| {
                    [
                        s[0] + p.requests,
                        s[1] + p.request_bytes,
                        s[2] + p.reply_bytes,
                    ]
                })
            };
            let old = counts(&pool);
            pool.save_native_recovery(&file)?;
            let now = counts(&pool);
            pool.compact()?;
            if pool.frame() != &before {
                return Err("compact changed the transfer barrier".into());
            }
            drop(pool);
            pool = PoolNativeRecovery::open(&file)?.resume(&a.worker)?;
            if pool.frame() != &before {
                return Err(
                    "fresh resume changed identities, native caches or contact history".into(),
                );
            }
            oracle.verify(pool.frame(), &owners)?;
            let bytes = fs::read(&file)?;
            barrier = json!({"tick":75,"archive_bytes":bytes.len(),"archive_sha256":format!("{:x}",Sha256::digest(&bytes)),"requests":now[0]-old[0],"request_bytes":now[1]-old[1],"reply_bytes":now[2]-old[2],"before":receipts(&before),"after":receipts(pool.frame())});
        }
    }
    let layout = a.layout.to_possible_value().unwrap();
    let report = json!({"first_body_id":1,"last_tick":fixture::END,"width":a.width,"layout":layout.get_name(),"placement":"checkpoint","trajectory_sha256":oracle.digest,"owner_changes":map,"member_ids":members,"barrier":barrier,"contacts":counts,"native_pair_receipts":oracle.pairs,"native_floor_receipts":oracle.floors});
    fs::write(
        a.directory.join("owners.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    println!(
        "Verified 350 native barriers, warm handoff at 70 and byte-exact post-restore frame at 75. Native archive {} bytes.",
        barrier["archive_bytes"]
    );
    Ok(())
}
