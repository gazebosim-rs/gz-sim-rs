// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure acknowledged native-component migration separately from advancing the scene.
#[path = "support/pad_control.rs"]
mod control;
#[path = "support/changing_worlds.rs"]
mod scene;
use control::{local_digest, physical_bytes};

use clap::{Parser, ValueEnum};
use gz_physics_distributed::{
    physics::{Body, GroundPlane, SimulationStep},
    pool::WorkerPool,
};
use scene::{END, GRAVITY, Layout, Placement, SETTLED};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::Instant,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// changing_world_profile binary for independently verified serial-native trajectory hashes.
    #[arg(long)]
    profile: PathBuf,
    #[arg(long, default_value_t=8, value_parser=clap::value_parser!(u16).range(1..))]
    rounds: u16,
    /// Compare both encodings using the same worker and coordinator in alternating order.
    #[arg(long,value_enum,default_value_t=Wire::Json)]
    wire: Wire,
    /// A new JSONL file. Round zero warms up each case and is excluded.
    output: PathBuf,
}

#[derive(Clone, Copy, ValueEnum)]
enum Wire {
    Json,
    Packed,
    Paired,
}

#[derive(Clone, Copy, Serialize)]
#[serde(rename_all = "snake_case")]
enum Case {
    LoadedToActive,
    AirborneToActive,
    LoadedToIdle,
    StackToActive,
    SingleToSplit,
}

const CASES: [Case; 5] = [
    Case::LoadedToActive,
    Case::AirborneToActive,
    Case::LoadedToIdle,
    Case::StackToActive,
    Case::SingleToSplit,
];

#[derive(Serialize)]
struct Movement {
    from: u8,
    to: u8,
    component: u64,
    bodies: usize,
    positive_pairs: usize,
    positive_supports: usize,
    worlds_before: Vec<usize>,
    elapsed_us: f64,
    request_bytes: u64,
    reply_bytes: u64,
    requests: u64,
    replies: u64,
}
#[derive(Serialize)]
struct Trial {
    round: u16,
    layout: Layout,
    case: Case,
    packed_requested: bool,
    packed_peers: usize,
    barrier: u64,
    movements: Vec<Movement>,
    transfer_total_us: f64,
    following_step_us: f64,
    measured_physics_steps: usize,
    physics_total_ms: f64,
    physics_sha256: String,
}
fn move_group(
    pool: &mut WorkerPool,
    group: &[Body],
    to: u8,
    expected_pairs: usize,
) -> Result<Movement> {
    let member = group[0].id;
    let before = pool.frame();
    let from = before.body(member).ok_or("missing component member")?.0;
    let source = &before.workers[usize::from(from)];
    let component = source
        .components
        .iter()
        .find(|c| c.ids.first() == Some(&member))
        .ok_or("missing native component")?;
    if !group.iter().map(|b| b.id).eq(component.ids.iter().copied()) {
        return Err("transfer would select the wrong native island".into());
    }
    let inside = |id| component.ids.binary_search(&id).is_ok();
    let positive_pairs = source
        .snapshot
        .impulses
        .iter()
        .filter(|p| inside(p.a) && inside(p.b) && p.normal_impulse > 0.)
        .count();
    let positive_supports = source
        .snapshot
        .supports
        .iter()
        .filter(|p| inside(p.body) && p.normal_impulse > 0.)
        .count();
    if positive_pairs != expected_pairs || positive_supports != 1 {
        return Err("unexpected contact load before migration".into());
    }
    let worlds_before = before.workers.iter().map(|w| w.components.len()).collect();
    let expected = physical_bytes(before)?;
    let expected_ownership: Vec<_> = before
        .workers
        .iter()
        .enumerate()
        .flat_map(|(i, o)| {
            o.generations.iter().map(move |(&id, &generation)| {
                let migrating = inside(id);
                (
                    id,
                    if migrating { to } else { i as u8 },
                    generation + u64::from(migrating),
                )
            })
        })
        .collect();
    let traffic = pool.traffic();
    let start = Instant::now();
    pool.transfer_component(member, to)?;
    let elapsed_us = start.elapsed().as_secs_f64() * 1e6;
    let mut request_bytes = 0;
    let mut reply_bytes = 0;
    let mut requests = 0;
    let mut replies = 0;
    for (before, after) in traffic.iter().zip(pool.traffic()) {
        request_bytes += after.request_bytes - before.request_bytes;
        reply_bytes += after.reply_bytes - before.reply_bytes;
        requests += after.requests - before.requests;
        replies += after.replies - before.replies;
    }
    if expected != physical_bytes(pool.frame())?
        || expected_ownership.iter().any(|&(id, owner, generation)| {
            pool.frame().workers[usize::from(owner)]
                .generations
                .get(&id)
                != Some(&generation)
        })
    {
        return Err("migration changed public physics or ownership generation".into());
    }
    Ok(Movement {
        from,
        to,
        component: member,
        bodies: group.len(),
        positive_pairs,
        positive_supports,
        worlds_before,
        elapsed_us,
        request_bytes,
        reply_bytes,
        requests,
        replies,
    })
}

fn measure(
    worker: &Path,
    layout: Layout,
    case: Case,
    round: u16,
    packed_requested: bool,
    expected_hash: &str,
) -> Result<Trial> {
    let groups = scene::seeds(layout);
    let initial = if matches!(case, Case::SingleToSplit) {
        Placement::Single
    } else {
        Placement::Split
    };
    let mut pool = WorkerPool::spawn(
        worker,
        0x68616e646f6666,
        initial.worker_seeds(&groups),
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        1024,
    )?;
    let packed_peers = pool.set_packed_handoffs(packed_requested);
    let owners = Placement::Split.owners(&groups);
    let first_pad = groups
        .iter()
        .position(|g| g.len() == 25)
        .ok_or("missing pad")?;
    let barrier = if matches!(case, Case::AirborneToActive) {
        100
    } else {
        75
    };
    let mut movements = Vec::new();
    let mut hash = Sha256::new();
    let mut physics_total_ms = 0.;
    let mut following_step_us = 0.;
    for tick in 1..=END {
        let field = scene::wrenches(tick, &groups);
        let start = Instant::now();
        pool.advance_islands_in_field(GRAVITY, &field)?;
        let step_us = start.elapsed().as_secs_f64() * 1e6;
        if tick > SETTLED {
            physics_total_ms += step_us / 1e3;
        }
        if tick == barrier + 1 {
            following_step_us = step_us;
        }
        if tick == barrier {
            if matches!(case, Case::SingleToSplit) {
                for (group, owner) in groups.iter().zip(&owners) {
                    if *owner == 1 {
                        movements.push(move_group(&mut pool, group, 1, group.len() - 1)?);
                    }
                }
            } else {
                let group = if matches!(case, Case::StackToActive) {
                    groups
                        .iter()
                        .zip(&owners)
                        .find(|(g, w)| g.len() == 2 && **w == owners[first_pad])
                        .ok_or("missing stack")?
                        .0
                } else {
                    &groups[first_pad]
                };
                let to = if matches!(case, Case::LoadedToIdle) {
                    2
                } else {
                    (1 - owners[first_pad]) as u8
                };
                let pairs = if matches!(case, Case::AirborneToActive) {
                    0
                } else {
                    group.len() - 1
                };
                movements.push(move_group(&mut pool, group, to, pairs)?);
            }
        }
        hash.update(physical_bytes(pool.frame())?);
    }
    let physics_sha256 = format!("{:x}", hash.finalize());
    if physics_sha256 != expected_hash {
        return Err(
            format!("physics diverged from independent native control on round {round}").into(),
        );
    }
    Ok(Trial {
        round,
        layout,
        case,
        packed_peers,
        packed_requested,
        barrier,
        transfer_total_us: movements.iter().map(|m| m.elapsed_us).sum(),
        movements,
        following_step_us,
        measured_physics_steps: (END - SETTLED) as usize,
        physics_total_ms,
        physics_sha256,
    })
}

fn main() -> Result<()> {
    let args = Args::parse();
    let layouts = [Layout::Adjacent, Layout::Separated];
    let checks = [
        local_digest(&args.profile, layouts[0])?,
        local_digest(&args.profile, layouts[1])?,
    ];
    let mut out = BufWriter::new(File::create_new(&args.output)?);
    for round in 0..=args.rounds {
        for j in 0..2 {
            let index = (usize::from(round) + j) % 2;
            for k in 0..CASES.len() {
                let case_index = (usize::from(round) + index + k) % CASES.len();
                let case = CASES[case_index];
                let arms: &[bool] = match args.wire {
                    Wire::Json => &[false],
                    Wire::Packed => &[true],
                    Wire::Paired if (usize::from(round) + index + case_index).is_multiple_of(2) => {
                        &[false, true]
                    }
                    Wire::Paired => &[true, false],
                };
                for &enabled in arms {
                    let trial = measure(
                        &args.worker,
                        layouts[index],
                        case,
                        round,
                        enabled,
                        &checks[index],
                    )?;
                    if round > 0 {
                        serde_json::to_writer(&mut out, &trial)?;
                        writeln!(out)?;
                        out.flush()?;
                    }
                }
            }
        }
    }
    Ok(())
}
