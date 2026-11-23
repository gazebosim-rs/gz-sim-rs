// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify a crossing and collision on two independent physics worker processes.
#[path = "support/qualification.rs"]
mod qualification;
use clap::Parser;
use std::{error::Error, io, path::PathBuf};

#[derive(Parser)]
struct Args {
    /// Worker executable. Defaults to physics_worker beside the Cargo examples directory.
    #[arg(long)]
    worker: Option<PathBuf>,
    /// Write a gzweb replay archive into a new directory.
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
            .ok_or_else(|| io::Error::other("cannot locate worker; supply --worker"))?
            .join(format!("physics_worker{}", std::env::consts::EXE_SUFFIX)),
    };
    if !worker.is_file() {
        return Err(io::Error::other(format!(
            "build physics_worker first, or supply --worker; missing {}",
            worker.display()
        ))
        .into());
    }
    qualification::run(&worker, args.capture.as_deref())
}
