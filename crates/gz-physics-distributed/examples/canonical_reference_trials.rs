// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Paired timings for a single canonical native scene and ordinary split placement.
use clap::Parser;
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::{
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
    process::Command,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    profile: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    /// Complete paired repetitions per ID layout, excluding warmup.
    #[arg(long,default_value_t=8,value_parser=clap::value_parser!(u16).range(1..))]
    rounds: u16,
    /// Initial epoch and final epoch are reported separately. At 43, both layouts are still exact.
    #[arg(long,default_value_t=43,value_parser=clap::value_parser!(u32).range(1..=43))]
    cycles: u32,
    out: PathBuf,
}
#[derive(Deserialize)]
struct Cycle {
    cycle: u32,
    spawn_us: f64,
    transfer_us: f64,
    transfer_wire_bytes: u64,
    transfer_members: usize,
    regular_samples_us: Vec<f64>,
    canonical_reference: Option<bool>,
    native_solver_capacity: usize,
    trajectory_sha256: String,
    resident_component_sizes: Vec<Vec<usize>>,
}
#[derive(Serialize)]
struct Distribution {
    count: usize,
    transfer_us: f64,
    transfer_wire_bytes: u64,
    transfer_members: usize,
    mean_us: f64,
    median_us: f64,
    p95_us: f64,
    max_us: f64,
    resident_component_sizes: Vec<Vec<usize>>,
}
impl Distribution {
    fn new(row: &Cycle) -> Result<Self> {
        let v = &row.regular_samples_us;
        if v.len() != 349 || v.iter().any(|x| !x.is_finite() || *x <= 0.) || !v.is_sorted() {
            return Err("invalid ordinary barrier timing series".into());
        }
        Ok(Self {
            count: v.len(),
            transfer_us: row.transfer_us,
            transfer_wire_bytes: row.transfer_wire_bytes,
            transfer_members: row.transfer_members,
            mean_us: v.iter().sum::<f64>() / v.len() as f64,
            median_us: v[v.len() / 2],
            p95_us: v[(v.len() * 95).div_ceil(100) - 1],
            max_us: *v.last().unwrap(),
            resident_component_sizes: row.resident_component_sizes.clone(),
        })
    }
}
#[derive(Serialize)]
struct Run {
    spawn_us: f64,
    first: Distribution,
    last: Distribution,
    trajectory_sha256: String,
}
fn measure(args: &Args, layout: &str, canonical: bool, cycles: u32) -> Result<Run> {
    let scratch = tempfile::tempdir_in(&args.out)?;
    let dir = scratch.path().join("run");
    let mut command = Command::new(&args.profile);
    command
        .arg("--worker")
        .arg(&args.worker)
        .args([
            "--layout",
            layout,
            "--native-64",
            "--scene-wide",
            "--contact-transitions",
            "--placement-timings",
            "--cycles",
        ])
        .arg(cycles.to_string());
    if canonical {
        command.arg("--canonical-reference");
    }
    let output = command.arg(&dir).output()?;
    if !output.status.success() {
        return Err(String::from_utf8_lossy(&output.stderr).into_owned().into());
    }
    let mut first = None;
    let mut last = None;
    for (i, line) in BufReader::new(File::open(dir.join("cycles.jsonl"))?)
        .lines()
        .enumerate()
    {
        let row: Cycle = serde_json::from_str(&line?)?;
        if row.cycle as usize != i + 1
            || row.canonical_reference.unwrap_or(false) != canonical
            || row.native_solver_capacity != 64
        {
            return Err("trial did not report requested capacity, layout and barrier epoch".into());
        }
        if i == 0 {
            first = Some(Distribution::new(&row)?);
        }
        last = Some(row);
    }
    let last = last.ok_or("no completed epochs")?;
    if last.cycle != cycles {
        return Err("incomplete timing run".into());
    }
    Ok(Run {
        spawn_us: last.spawn_us,
        first: first.unwrap(),
        last: Distribution::new(&last)?,
        trajectory_sha256: last.trajectory_sha256,
    })
}
fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    (v[(v.len() - 1) / 2] + v[v.len() / 2]) / 2.
}
#[derive(Serialize)]
struct Pair {
    round: u16,
    layout: &'static str,
    canonical_first: bool,
    split: Run,
    canonical: Run,
}
fn stats(rows: &[Pair], layout: &str, f: fn(&Run) -> f64) -> serde_json::Value {
    let rows = rows
        .iter()
        .filter(|p| p.layout == layout)
        .collect::<Vec<_>>();
    let split = median(&mut rows.iter().map(|p| f(&p.split)).collect::<Vec<_>>());
    let canonical = median(&mut rows.iter().map(|p| f(&p.canonical)).collect::<Vec<_>>());
    let mut paired = rows
        .iter()
        .map(|p| 100. * (f(&p.canonical) / f(&p.split) - 1.))
        .collect::<Vec<_>>();
    let paired_median = median(&mut paired);
    json!({"split_us":split,"canonical_us":canonical,"paired_median_change_percent":paired_median,
        "paired_min_percent":paired[0],"paired_max_percent":paired.last(),"canonical_faster_pairs":paired.iter().filter(|&&x|x<0.).count()})
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.out)?;
    let layouts = ["adjacent", "separated"];
    for canonical in [false, true] {
        measure(&args, layouts[0], canonical, 1)?;
    }
    let mut pairs = vec![];
    for round in 0..args.rounds {
        for offset in 0..2 {
            let index = (round as usize + offset) % 2;
            let layout = layouts[index];
            let canonical_first = (round as usize + index).is_multiple_of(2);
            let a = measure(&args, layout, canonical_first, args.cycles)?;
            let b = measure(&args, layout, !canonical_first, args.cycles)?;
            let (canonical, split) = if canonical_first { (a, b) } else { (b, a) };
            if canonical.trajectory_sha256 != split.trajectory_sha256 {
                return Err(
                    format!("unmatched physical trajectories in {layout} round {round}").into(),
                );
            }
            pairs.push(Pair {
                round,
                layout,
                canonical_first,
                split,
                canonical,
            });
        }
        fs::write(
            args.out.join("pairs.json"),
            serde_json::to_vec_pretty(&pairs)?,
        )?;
    }
    let mut result = vec![];
    for layout in layouts {
        result.push(json!({"layout":layout,"spawn":stats(&pairs,layout,|r|r.spawn_us),
            "first_epoch_mean":stats(&pairs,layout,|r|r.first.mean_us),"first_epoch_p95":stats(&pairs,layout,|r|r.first.p95_us),
            "initial_epoch_transfer":stats(&pairs,layout,|r|r.first.transfer_us),
            "final_epoch_transfer":stats(&pairs,layout,|r|r.last.transfer_us),
            "final_epoch_mean":stats(&pairs,layout,|r|r.last.mean_us),"final_epoch_p95":stats(&pairs,layout,|r|r.last.p95_us)}));
    }
    fs::write(
        Path::new(&args.out).join("summary.json"),
        serde_json::to_vec_pretty(&json!({
            "rounds_per_layout":args.rounds,"cycles_per_run":args.cycles,"cpus":std::thread::available_parallelism()?.get(),
            "samples_per_epoch":349,"excluded":"one deliberately interrupted barrier per epoch",
            "summary":result
        }))?,
    )?;
    Ok(())
}
