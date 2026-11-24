// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Kill and recover a real Gazebo coordinator with a live contact and queued impulse.
#[path = "support/durable_run.rs"]
mod run;
use clap::{Parser, ValueEnum};
use std::{path::PathBuf, process::Command};

#[derive(Clone, Copy, ValueEnum)]
enum Mode {
    Demonstrate,
    Original,
    Replacement,
}
#[derive(Parser)]
struct Args {
    /// New directory for the durable checkpoint, comparison evidence and gzweb replay.
    directory: PathBuf,
    /// Defaults to physics_worker in the Cargo build profile directory.
    #[arg(long)]
    worker: Option<PathBuf>,
    #[arg(long, value_enum, default_value = "demonstrate")]
    mode: Mode,
}
fn main() -> run::Result<()> {
    let args = Args::parse();
    let exe = std::env::current_exe()?;
    let worker = args.worker.unwrap_or_else(|| {
        exe.parent()
            .unwrap()
            .parent()
            .unwrap()
            .join(format!("physics_worker{}", std::env::consts::EXE_SUFFIX))
    });
    match args.mode {
        Mode::Original => run::original(&worker, &args.directory),
        Mode::Replacement => run::replacement(&worker, &args.directory),
        Mode::Demonstrate => {
            let command = |mode: &str| {
                let mut c = Command::new(&exe);
                c.arg(&args.directory)
                    .arg("--worker")
                    .arg(&worker)
                    .arg("--mode")
                    .arg(mode);
                c
            };
            run::supervise(command("original"), command("replacement"), &args.directory)
        }
    }
}
