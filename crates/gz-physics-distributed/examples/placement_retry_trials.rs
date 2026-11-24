// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run paired reference and candidate endurance coordinators against one worker executable.
use clap::Parser;
use serde::{Deserialize, Serialize};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader, BufWriter, Write},
    path::{Path, PathBuf},
    process::Command,
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    reference: PathBuf,
    #[arg(long)]
    candidate: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    /// Each arm restarts from the same seeds after this many continuous cycles.
    #[arg(long, default_value_t = 4, value_parser=clap::value_parser!(u32).range(1..=16))]
    cycles: u32,
    #[arg(long, default_value_t = 0)]
    pace_ms: u16,
    #[arg(long, default_value_t = 1, value_parser=clap::value_parser!(u32).range(1..))]
    pairs: u32,
    /// Finish the current pair once this many seconds have elapsed.
    #[arg(long, conflicts_with="pairs", value_parser=clap::value_parser!(u32).range(1..))]
    seconds: Option<u32>,
    out: PathBuf,
}
#[derive(Serialize, Deserialize)]
struct Cycle {
    cycle: u32,
    counts: serde_json::Value,
    trajectory_sha256: String,
    retry_us: f64,
    retry_restarted_workers: usize,
    regular_p95_us: f64,
    regular_total_us: f64,
}
fn arm(args: &Args, bin: &Path, layout: &str) -> Result<Vec<Cycle>> {
    let scratch = tempfile::tempdir_in(&args.out)?;
    let out = scratch.path().join("trial");
    let status = Command::new(bin)
        .arg("--worker")
        .arg(&args.worker)
        .arg("--layout")
        .arg(layout)
        .arg("--cycles")
        .arg(args.cycles.to_string())
        .arg("--pace-ms")
        .arg(args.pace_ms.to_string())
        .arg(&out)
        .status()?;
    if !status.success() {
        let evidence = scratch.keep();
        return Err(format!(
            "{} failed ({status}), evidence at {}",
            bin.display(),
            evidence.display()
        )
        .into());
    }
    let rows = BufReader::new(File::open(out.join("cycles.jsonl"))?)
        .lines()
        .map(|line| Ok(serde_json::from_str(&line?)?))
        .collect::<Result<Vec<_>>>()?;
    if rows.len() != args.cycles as usize {
        return Err("incomplete trial".into());
    }
    Ok(rows)
}
fn median(samples: &mut [f64]) -> f64 {
    samples.sort_by(f64::total_cmp);
    (samples[samples.len() / 2] + samples[(samples.len() - 1) / 2]) / 2.
}
#[derive(Default)]
struct Times {
    retry: [Vec<f64>; 2],
    restarts: [usize; 2],
    regular_p95_change: Vec<f64>,
    regular_total_change: Vec<f64>,
    ticks: u64,
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.out)?;
    let mut log = BufWriter::new(File::create_new(args.out.join("pairs.jsonl"))?);
    let begin = Instant::now();
    let mut layouts = [Times::default(), Times::default()];
    for pair in 0..u32::MAX {
        let layout = ["adjacent", "separated"][(pair % 2) as usize];
        let reference_first = matches!(pair % 4, 0 | 3);
        let (a, b) = if reference_first {
            (&args.reference, &args.candidate)
        } else {
            (&args.candidate, &args.reference)
        };
        let first = arm(&args, a, layout)?;
        let second = arm(&args, b, layout)?;
        let (reference, candidate) = if reference_first {
            (first, second)
        } else {
            (second, first)
        };
        let times = &mut layouts[(pair % 2) as usize];
        for (i, (r, c)) in reference.iter().zip(&candidate).enumerate() {
            if c.cycle != i as u32 + 1
                || r.cycle != c.cycle
                || r.counts != c.counts
                || r.trajectory_sha256 != c.trajectory_sha256
            {
                return Err(
                    format!("divergent {layout} pair {} cycle {}", pair + 1, c.cycle).into(),
                );
            }
            for (arm, row) in [r, c].into_iter().enumerate() {
                times.retry[arm].push(row.retry_us);
                times.restarts[arm] += row.retry_restarted_workers;
            }
            times
                .regular_p95_change
                .push(100. * (c.regular_p95_us / r.regular_p95_us - 1.));
            times
                .regular_total_change
                .push(100. * (c.regular_total_us / r.regular_total_us - 1.));
        }
        times.ticks += candidate.last().unwrap().counts["ticks"]
            .as_u64()
            .ok_or("missing tick count")?;
        writeln!(
            log,
            "{}",
            serde_json::json!({"pair":pair+1,"layout":layout,"reference_first":reference_first,"elapsed_s":begin.elapsed().as_secs_f64(),"reference":reference,"candidate":candidate})
        )?;
        log.flush()?;
        let mut stats = serde_json::Map::new();
        for (layout, t) in ["adjacent", "separated"].into_iter().zip(&mut layouts) {
            if !t.retry[0].is_empty() {
                stats.insert(layout.to_owned(),serde_json::json!({"matched_cycles":t.retry[0].len(),"ticks_per_arm":t.ticks,"restarted_workers":t.restarts,
                    "reference_retry_median_us":median(&mut t.retry[0]),"candidate_retry_median_us":median(&mut t.retry[1]),
                    "median_regular_p95_change_pct":median(&mut t.regular_p95_change),"median_regular_total_change_pct":median(&mut t.regular_total_change)}));
            }
        }
        let tmp = args.out.join("progress.tmp");
        fs::write(&tmp, serde_json::to_vec_pretty(&stats)?)?;
        fs::rename(tmp, args.out.join("progress.json"))?;
        if args.seconds.map_or(pair + 1 >= args.pairs, |s| {
            begin.elapsed() >= Duration::from_secs(u64::from(s))
        }) {
            fs::write(
                args.out.join("result.json"),
                serde_json::to_vec_pretty(&stats)?,
            )?;
            break;
        }
    }
    Ok(())
}
