// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare two frozen coordinator/worker builds on identical handoff fixtures and CPUs.
use clap::Parser;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{BufRead, BufReader, BufWriter, Write},
    path::{Path, PathBuf},
    process::Command,
};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[derive(Parser)]
struct Args {
    /// Directory containing the original pad_handoff_cost and physics_worker binaries.
    #[arg(long)]
    baseline: PathBuf,
    /// Directory with the corresponding changed binaries.
    #[arg(long)]
    candidate: PathBuf,
    #[arg(long)]
    profile: PathBuf,
    #[arg(long,default_value_t=16,value_parser=clap::value_parser!(u16).range(1..))]
    rounds: u16,
    /// New campaign directory.
    output: PathBuf,
}

#[derive(Deserialize)]
struct Trial {
    round: u16,
    case: String,
    layout: String,
    packed_requested: bool,
    packed_peers: usize,
    barrier: u64,
    movements: Vec<Value>,
    transfer_total_us: f64,
    physics_sha256: String,
}

type Key = (String, String, bool);
fn run(bin: &Path, profile: &Path, path: &Path) -> Result<BTreeMap<Key, Trial>> {
    let status = Command::new(bin.join("pad_handoff_cost"))
        .arg("--worker")
        .arg(bin.join("physics_worker"))
        .arg("--profile")
        .arg(profile)
        .args(["--rounds", "1", "--wire", "paired"])
        .arg(path)
        .status()?;
    if !status.success() {
        return Err(format!("failed benchmark {bin:?}: {status}").into());
    }
    let mut trials = BTreeMap::new();
    for line in BufReader::new(File::open(path)?).lines() {
        let mut t: Trial = serde_json::from_str(&line?)?;
        if t.round != 1
            || t.packed_peers != if t.packed_requested { 4 } else { 0 }
            || !t.transfer_total_us.is_finite()
            || t.transfer_total_us <= 0.
            || t.movements.is_empty()
            || t.physics_sha256.len() != 64
        {
            return Err("invalid measurement".into());
        }
        for m in &mut t.movements {
            m.as_object_mut()
                .ok_or("invalid movement")?
                .remove("elapsed_us")
                .ok_or("missing timing")?;
        }
        let key = (t.layout.clone(), t.case.clone(), t.packed_requested);
        if trials.insert(key, t).is_some() {
            return Err("duplicate measurement".into());
        }
    }
    if trials.len() != 20 {
        return Err("expected ten fixtures, each with two wire encodings".into());
    }
    Ok(trials)
}

#[derive(Serialize, Clone)]
struct Pair {
    round: u16,
    wire: String,
    layout: String,
    case: String,
    baseline_first: bool,
    baseline_us: f64,
    candidate_us: f64,
    barrier: u64,
    movements: Vec<Value>,
    physics_sha256: String,
}
#[derive(Serialize)]
struct Stats {
    pairs: usize,
    candidate_faster: usize,
    baseline_median_us: f64,
    candidate_median_us: f64,
    baseline_p95_us: f64,
    candidate_p95_us: f64,
    median_paired_change_pct: f64,
}
fn stats(p: &[Pair]) -> Stats {
    let n = p.len();
    let sorted = |select: fn(&Pair) -> f64| {
        let mut v: Vec<_> = p.iter().map(select).collect();
        v.sort_by(f64::total_cmp);
        v
    };
    let median = |v: &[f64]| (v[(n - 1) / 2] + v[n / 2]) / 2.;
    let a = sorted(|p| p.baseline_us);
    let b = sorted(|p| p.candidate_us);
    let rank = (n * 95).div_ceil(100) - 1;
    Stats {
        pairs: n,
        candidate_faster: p.iter().filter(|p| p.candidate_us < p.baseline_us).count(),
        baseline_median_us: median(&a),
        candidate_median_us: median(&b),
        baseline_p95_us: a[rank],
        candidate_p95_us: b[rank],
        median_paired_change_pct: median(&sorted(|p| (p.candidate_us / p.baseline_us - 1.) * 100.)),
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.output)?;
    let mut out = BufWriter::new(File::create_new(args.output.join("pairs.jsonl"))?);
    let mut all = BTreeMap::<Key, Vec<Pair>>::new();
    for round in 1..=args.rounds {
        let dir = tempfile::tempdir_in(&args.output)?;
        let baseline_first = round % 2 == 1;
        let (a, b) = if baseline_first {
            (&args.baseline, &args.candidate)
        } else {
            (&args.candidate, &args.baseline)
        };
        let x = run(a, &args.profile, &dir.path().join("a.jsonl"))?;
        let y = run(b, &args.profile, &dir.path().join("b.jsonl"))?;
        let (baseline, mut candidate) = if baseline_first { (x, y) } else { (y, x) };
        for (key, a) in baseline {
            let b = candidate.remove(&key).ok_or("missing comparison arm")?;
            if a.barrier != b.barrier
                || a.movements != b.movements
                || a.physics_sha256 != b.physics_sha256
            {
                return Err("differing physics, protocol bytes or transfer controls".into());
            }
            let pair = Pair {
                round,
                wire: if a.packed_requested { "packed" } else { "json" }.into(),
                layout: a.layout,
                case: a.case,
                baseline_first,
                baseline_us: a.transfer_total_us,
                candidate_us: b.transfer_total_us,
                barrier: a.barrier,
                movements: a.movements,
                physics_sha256: a.physics_sha256,
            };
            serde_json::to_writer(&mut out, &pair)?;
            writeln!(out)?;
            all.entry(key).or_default().push(pair);
        }
        out.flush()?;
    }
    let mut summary: BTreeMap<_, BTreeMap<String, BTreeMap<String, Stats>>> = BTreeMap::new();
    for ((layout, case, packed), pairs) in all {
        summary
            .entry(if packed { "packed" } else { "json" })
            .or_default()
            .entry(layout)
            .or_default()
            .insert(case, stats(&pairs));
    }
    fs::write(
        args.output.join("summary.json"),
        format!("{}\n", serde_json::to_string_pretty(&summary)?),
    )?;
    Ok(())
}
