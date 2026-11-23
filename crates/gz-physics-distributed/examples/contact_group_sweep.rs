// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded, reproducible, real-process contact-group qualification.
//! Build this example and physics_worker with the same release features and lockfile.
//! Run `contact_group_sweep --worker ./physics_worker --out ./results --cases 8`.
//! Replay an individual case with the recorded seed and `--cases 1 --capture-first`.
#[path = "support/replay.rs"]
mod replay;
#[path = "support/sweep_report.rs"]
mod report;
#[path = "support/sweep_case.rs"]
mod sweep_case;
use clap::Parser;
use report::{Case, Summary};
use std::{
    error::Error,
    fs, io,
    io::Write,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// New output directory, or an existing directory without a summary file.
    #[arg(long)]
    out: PathBuf,
    #[arg(long, default_value_t = 1)]
    start_seed: u64,
    #[arg(long,default_value_t=8,value_parser=clap::value_parser!(u64).range(1..=1_000_000))]
    cases: u64,
    /// Absolute Unix time after which the current case is interrupted and recorded separately.
    #[arg(long)]
    deadline_unix: Option<u64>,
    #[arg(long,default_value_t=60,value_parser=clap::value_parser!(u64).range(1..=3600))]
    summary_seconds: u64,
    #[arg(long,default_value_t=75,value_parser=clap::value_parser!(u64).range(0..=60000))]
    idle_ms: u64,
    /// Save up to ~50 real frames from the first seeded case to out/replay.
    #[arg(long)]
    capture_first: bool,
}
fn save(dir: &Path, summary: &Summary) -> Result<(), Box<dyn Error>> {
    let tmp = dir.join("summary.tmp");
    fs::write(&tmp, serde_json::to_vec_pretty(summary)?)?;
    fs::rename(tmp, dir.join("summary.json"))?;
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    if !args.worker.is_file() {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            "--worker must name the matching, built physics_worker",
        )
        .into());
    }
    fs::create_dir_all(&args.out)?;
    if args.out.join("summary.json").try_exists()? {
        return Err("output already contains a sweep summary".into());
    }
    let mut series = fs::File::create_new(args.out.join("series.csv"))?;
    writeln!(
        series,
        "elapsed_s,cases,exact,steps,migrations,replacements,peak_monolithic_position_m,peak_same_partition_position_m"
    )?;
    let mut samples = 0;
    let mut summary = Summary {
        start_seed: args.start_seed,
        requested_cases: args.cases,
        stop_unix: args.deadline_unix,
        ..Default::default()
    };
    let started = Instant::now();
    let mut last = Instant::now();
    save(&args.out, &summary)?;
    for i in 0..args.cases {
        if args.deadline_unix.is_some_and(|d| sweep_case::unix() >= d) {
            break;
        }
        let seed = args.start_seed.checked_add(i).ok_or("seed overflow")?;
        let capture = (args.capture_first && i == 0).then(|| args.out.join("replay"));
        let mut case = Case {
            seed,
            stage: "setup",
            ..Default::default()
        };
        if let Err(e) = sweep_case::run(
            &mut case,
            &args.worker,
            capture.as_deref(),
            args.deadline_unix,
        ) {
            case.error = Some(e.to_string());
        }
        if case.stage == "deadline" {
            summary.interrupted_seed = Some(seed);
            break;
        }
        let failures = summary.first_by_category.len();
        summary.add(&case);
        if i == 0
            || summary.first_by_category.len() != failures
            || last.elapsed() >= Duration::from_secs(args.summary_seconds)
        {
            last = Instant::now();
            summary.elapsed_s = started.elapsed().as_secs_f64();
            save(&args.out, &summary)?;
            if samples < 512 {
                writeln!(
                    series,
                    "{:.3},{},{},{},{},{},{},{}",
                    summary.elapsed_s,
                    summary.completed,
                    summary.exact,
                    summary.steps,
                    summary.migrations,
                    summary.worker_replacements,
                    summary.peak_monolithic.position_m,
                    summary.peak_same_partition.position_m
                )?;
                samples += 1;
            }
            println!(
                "seed {seed}, completed {}, exact {}, steps {}, transfers {}, replacements {}, categories {:?}",
                summary.completed,
                summary.exact,
                summary.steps,
                summary.migrations,
                summary.worker_replacements,
                summary.category_counts
            );
        }
        std::thread::sleep(Duration::from_millis(args.idle_ms));
    }
    summary.elapsed_s = started.elapsed().as_secs_f64();
    summary.finished = true;
    save(&args.out, &summary)?;
    println!(
        "finished {} cases and {} steps; {} exact; categories {:?}",
        summary.completed, summary.steps, summary.exact, summary.category_counts
    );
    if summary.completed != summary.exact {
        return Err(
            "see summary.json for each numerical or incomplete case category and first reproducer"
                .into(),
        );
    }
    Ok(())
}
