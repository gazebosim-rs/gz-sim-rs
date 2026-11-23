// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise rolling native journal baselines under a real warm handoff with queued input.
#[path = "support/compact_run.rs"]
mod run;
use clap::Parser;
use std::path::PathBuf;
#[derive(Parser)]
struct Args {
    /// New output directory for qualification measurements and a native gzweb replay.
    directory: PathBuf,
    /// Defaults to physics_worker adjacent to the examples directory.
    #[arg(long)]
    worker: Option<PathBuf>,
}
fn main() -> run::Result<()> {
    let args = Args::parse();
    let exe = std::env::current_exe()?;
    let worker = args.worker.unwrap_or_else(|| {
        exe.ancestors()
            .nth(2)
            .expect("Cargo profile directory")
            .join(format!("physics_worker{}", std::env::consts::EXE_SUFFIX))
    });
    run::run(&worker, &args.directory)?;
    Ok(())
}
