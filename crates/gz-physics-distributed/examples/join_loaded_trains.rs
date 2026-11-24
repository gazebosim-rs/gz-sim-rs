// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bring two loaded contact groups into one native solver before they collide.
#[path = "support/replay.rs"]
mod replay;
#[path = "support/train_run.rs"]
mod train_run;
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
        Some(w) => w,
        None => std::env::current_exe()?
            .ancestors()
            .nth(2)
            .ok_or("pass --worker")?
            .join(format!("physics_worker{}", std::env::consts::EXE_SUFFIX)),
    };
    let mut frames = Vec::new();
    train_run::run(&worker, |f| {
        if args.capture.is_some() {
            frames.push(f.clone())
        }
    })?;
    if let Some(path) = args.capture {
        replay::write(&path, &frames)?;
    }
    Ok(())
}
