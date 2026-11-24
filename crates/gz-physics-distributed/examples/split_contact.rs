// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Release an isolated body from a formerly connected three-box solver.
#[path = "support/replay.rs"]
mod replay;
#[path = "support/split_run.rs"]
mod split_run;
use clap::Parser;
use std::{error::Error, path::PathBuf};
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: Option<PathBuf>,
    #[arg(long)]
    capture: Option<PathBuf>,
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let worker = match args.worker {
        Some(path) => path,
        None => std::env::current_exe()?
            .ancestors()
            .nth(2)
            .ok_or("pass --worker")?
            .join(format!("physics_worker{}", std::env::consts::EXE_SUFFIX)),
    };
    split_run::run(&worker, args.capture.as_deref())
}
