// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Seeded native controls for two local workers in uniform gravity.
use clap::Parser;
#[path = "support/field_trial.rs"]
mod field_trial;
use field_trial::{Trial, trial};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
    time::{SystemTime, UNIX_EPOCH},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// New result directory, created once. Each line describes one complete trial.
    #[arg(long)]
    output: PathBuf,
    #[arg(long, default_value_t = 20261119001)]
    seed: u64,
    #[arg(long, default_value_t = 100)]
    cases: u64,
    /// Stop between cases when the UTC Unix timestamp is reached.
    #[arg(long)]
    stop_unix: Option<u64>,
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.output)?;
    let mut rows = BufWriter::new(File::create_new(args.output.join("trials.jsonl"))?);
    let (mut cases, mut exact, mut within, mut faults, mut complete, mut ticks) =
        (0, 0, 0, 0, 0, 0);
    for i in 0..args.cases {
        if args.stop_unix.is_some_and(|t| {
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs()
                >= t
        }) {
            break;
        }
        let mut t = Trial {
            seed: args.seed.checked_add(i).ok_or("seed overflow")?,
            bitwise: true,
            exact_contact_sets: true,
            ..Default::default()
        };
        if let Err(e) = trial(&mut t, &args.worker) {
            t.error = Some(e.to_string());
        }
        ticks += t.ticks;
        cases += 1;
        let finished = t.error.is_none()
            && t.ticks == 216
            && t.replacement
            && t.transfers > 0
            && t.impacts.iter().all(Option::is_some);
        complete += u64::from(finished);
        exact += u64::from(finished && t.bitwise);
        within += u64::from(
            finished
                && t.exact_contact_sets
                && t.position_m < 1e-7
                && t.velocity_m_s < 1e-6
                && t.spin_rad_s < 1e-5
                && t.orientation_rad < 1e-6
                && t.impulse_n_s < 1e-5,
        );
        faults += u64::from(t.error.is_some());
        serde_json::to_writer(&mut rows, &t)?;
        writeln!(rows)?;
        rows.flush()?;
        if i % 100 == 0 || i + 1 == args.cases || t.error.is_some() && faults <= 3 {
            println!(
                "cases={cases} complete={complete} ticks={ticks} bitwise={exact} within_budgets={within} errors={faults}"
            );
        }
    }
    println!(
        "final cases={cases} complete={complete} ticks={ticks} bitwise={exact} within_budgets={within} errors={faults}"
    );
    Ok(())
}
