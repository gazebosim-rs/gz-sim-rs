// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a single atomic ownership transaction against four acknowledged singleton calls.
#[path = "support/pad_control.rs"]
mod control;
#[path = "support/changing_worlds.rs"]
mod scene;
use clap::{Parser, Subcommand};
use control::{local_digest, physical_bytes};
use gz_physics_distributed::{
    physics::{GroundPlane, SimulationStep},
    pool::WorkerPool,
};
use scene::{Layout, Placement};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::{BufRead, BufReader, BufWriter, Write},
    path::{Path, PathBuf},
    time::Instant,
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    command: Action,
}

#[derive(Subcommand)]
enum Action {
    Run {
        #[arg(long)]
        worker: PathBuf,
        #[arg(long)]
        profile: PathBuf,
        #[arg(long, default_value_t=16,value_parser=clap::value_parser!(u16).range(1..))]
        rounds: u16,
        #[arg(long)]
        packed: bool,
        output: PathBuf,
    },
    Report {
        #[arg(long)]
        output: PathBuf,
        #[arg(required = true)]
        campaigns: Vec<PathBuf>,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct Cost {
    elapsed_us: f64,
    requests: u64,
    replies: u64,
    request_bytes: u64,
    reply_bytes: u64,
    physics_sha256: String,
}

#[derive(Serialize, Deserialize)]
struct Pair {
    round: u16,
    layout: Layout,
    packed: bool,
    separate: Cost,
    atomic: Cost,
}

fn arm(
    executable: &Path,
    layout: Layout,
    packed: bool,
    atomic: bool,
    expected_digest: &str,
) -> Result<Cost> {
    let groups = scene::seeds(layout);
    let mut pool = WorkerPool::spawn(
        executable,
        0x636f686f7274,
        Placement::Single.worker_seeds(&groups),
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        1024,
    )?;
    if pool.set_packed_handoffs(packed) != if packed { 4 } else { 0 } {
        return Err("unexpected worker capabilities".into());
    }
    let mut hash = Sha256::new();
    let mut cost = None;
    for tick in 1..=scene::END {
        pool.advance_islands_in_field(scene::GRAVITY, &scene::wrenches(tick, &groups))?;
        if tick == scene::SETTLED + 5 {
            let mut moves = Vec::new();
            let mut ids = BTreeSet::new();
            for (g, owner) in groups.iter().zip(Placement::Split.owners(&groups)) {
                if owner == 1 {
                    moves.push((g[0].id, 1));
                    ids.extend(g.iter().map(|b| b.id));
                }
            }
            moves.sort_unstable();
            let source = &pool.frame().workers[0].snapshot;
            let pair_count = source
                .impulses
                .iter()
                .filter(|p| ids.contains(&p.a) && ids.contains(&p.b) && p.normal_impulse > 0.)
                .count();
            let supports = source
                .supports
                .iter()
                .filter(|s| ids.contains(&s.body) && s.normal_impulse > 0.)
                .count();
            if (moves.len(), ids.len(), pair_count, supports) != (4, 31, 27, 4) {
                return Err("unexpected pre-transfer contact topology".into());
            }
            let before = physical_bytes(pool.frame())?;
            let initial = pool.traffic();
            let start = Instant::now();
            if atomic {
                pool.transfer_components(&moves)?;
            } else {
                for &(id, dest) in &moves {
                    pool.transfer_component(id, dest)?;
                }
            }
            let elapsed_us = start.elapsed().as_secs_f64() * 1e6;
            let mut result = Cost {
                elapsed_us,
                requests: 0,
                replies: 0,
                request_bytes: 0,
                reply_bytes: 0,
                physics_sha256: String::new(),
            };
            for (a, b) in initial.iter().zip(pool.traffic()) {
                result.requests += b.requests - a.requests;
                result.replies += b.replies - a.replies;
                result.request_bytes += b.request_bytes - a.request_bytes;
                result.reply_bytes += b.reply_bytes - a.reply_bytes;
            }
            let expected = if atomic { 17 } else { 20 };
            if (result.requests, result.replies) != (expected, expected)
                || physical_bytes(pool.frame())? != before
            {
                return Err("unverified transfer or changed public physics".into());
            }
            for group in &groups {
                for b in group {
                    let owner = usize::from(ids.contains(&b.id));
                    if pool.frame().workers[owner].generations.get(&b.id) != Some(&(owner as u64)) {
                        return Err("unexpected published owner or generation".into());
                    }
                }
            }
            cost = Some(result);
        }
        hash.update(physical_bytes(pool.frame())?);
    }
    let mut cost = cost.ok_or("transfer barrier missing")?;
    cost.physics_sha256 = format!("{:x}", hash.finalize());
    if cost.physics_sha256 != expected_digest {
        return Err("trajectory differs from independent serial reference".into());
    }
    Ok(cost)
}

fn run(
    worker: PathBuf,
    profile: PathBuf,
    rounds: u16,
    packed: bool,
    output: PathBuf,
) -> Result<()> {
    let layouts = [Layout::Adjacent, Layout::Separated];
    let hashes = [
        local_digest(&profile, layouts[0])?,
        local_digest(&profile, layouts[1])?,
    ];
    let mut out = BufWriter::new(File::create_new(output)?);
    for round in 0..=rounds {
        for offset in 0..2 {
            let idx = (usize::from(round) + offset) % 2;
            let layout = layouts[idx];
            let atomic_first = (usize::from(round) + idx).is_multiple_of(2);
            let a = arm(&worker, layout, packed, atomic_first, &hashes[idx])?;
            let b = arm(&worker, layout, packed, !atomic_first, &hashes[idx])?;
            let (separate, atomic) = if atomic_first { (b, a) } else { (a, b) };
            if round != 0 {
                serde_json::to_writer(
                    &mut out,
                    &Pair {
                        round,
                        layout,
                        packed,
                        separate,
                        atomic,
                    },
                )?;
                writeln!(out)?;
                out.flush()?;
            }
        }
    }
    Ok(())
}

#[derive(Serialize)]
struct Stats {
    pairs: usize,
    atomic_faster: usize,
    separate_median_us: f64,
    atomic_median_us: f64,
    separate_p95_us: f64,
    atomic_p95_us: f64,
    median_paired_change_pct: f64,
    separate_bytes: u64,
    atomic_bytes: u64,
    physics_sha256: String,
}
fn compare(pairs: &[Pair]) -> Result<Stats> {
    let n = pairs.len();
    let first = pairs.first().ok_or("empty campaign")?;
    let bytes = |c: &Cost| c.request_bytes + c.reply_bytes;
    let separate_bytes = bytes(&first.separate);
    let atomic_bytes = bytes(&first.atomic);
    for p in pairs {
        if p.separate.physics_sha256 != first.separate.physics_sha256
            || p.atomic.physics_sha256 != p.separate.physics_sha256
            || bytes(&p.separate) != separate_bytes
            || bytes(&p.atomic) != atomic_bytes
            || (
                p.separate.requests,
                p.separate.replies,
                p.atomic.requests,
                p.atomic.replies,
            ) != (20, 20, 17, 17)
            || ![p.separate.elapsed_us, p.atomic.elapsed_us]
                .iter()
                .all(|v| v.is_finite() && *v > 0.)
        {
            return Err("unmatched measurements".into());
        }
    }
    let sorted = |f: fn(&Pair) -> f64| {
        let mut a: Vec<_> = pairs.iter().map(f).collect();
        a.sort_by(f64::total_cmp);
        a
    };
    let median = |v: &[f64]| (v[(n - 1) / 2] + v[n / 2]) / 2.;
    let s = sorted(|p| p.separate.elapsed_us);
    let a = sorted(|p| p.atomic.elapsed_us);
    let rank = (n * 95).div_ceil(100) - 1;
    Ok(Stats {
        pairs: n,
        atomic_faster: pairs
            .iter()
            .filter(|p| p.atomic.elapsed_us < p.separate.elapsed_us)
            .count(),
        separate_median_us: median(&s),
        atomic_median_us: median(&a),
        separate_p95_us: s[rank],
        atomic_p95_us: a[rank],
        median_paired_change_pct: median(&sorted(|p| {
            (p.atomic.elapsed_us / p.separate.elapsed_us - 1.) * 100.
        })),
        separate_bytes,
        atomic_bytes,
        physics_sha256: first.separate.physics_sha256.clone(),
    })
}

fn report(campaigns: Vec<PathBuf>, output: PathBuf) -> Result<()> {
    let mut bins: BTreeMap<String, BTreeMap<String, Vec<Pair>>> = BTreeMap::new();
    for path in campaigns {
        let mut seen = BTreeSet::new();
        for line in BufReader::new(File::open(path)?).lines() {
            let p: Pair = serde_json::from_str(&line?)?;
            let layout = match p.layout {
                Layout::Adjacent => "adjacent",
                Layout::Separated => "separated",
            };
            if !seen.insert((layout, p.round, p.packed)) {
                return Err("duplicate campaign round".into());
            }
            bins.entry(if p.packed { "packed" } else { "json" }.into())
                .or_default()
                .entry(layout.into())
                .or_default()
                .push(p);
        }
        if seen.is_empty() {
            return Err("empty campaign".into());
        }
    }
    let mut summary = BTreeMap::new();
    for (wire, layouts) in bins {
        let mut stats = BTreeMap::new();
        for (layout, pairs) in layouts {
            stats.insert(layout, compare(&pairs)?);
        }
        summary.insert(wire, stats);
    }
    std::fs::write(
        output,
        format!("{}\n", serde_json::to_string_pretty(&summary)?),
    )?;
    Ok(())
}

fn main() -> Result<()> {
    match Args::parse().command {
        Action::Run {
            worker,
            profile,
            rounds,
            packed,
            output,
        } => run(worker, profile, rounds, packed, output),
        Action::Report { output, campaigns } => report(campaigns, output),
    }
}
