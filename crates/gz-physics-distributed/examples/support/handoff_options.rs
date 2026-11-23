// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use clap::Parser;
use std::{
    error::Error,
    io,
    path::{Path, PathBuf},
};

#[derive(Parser)]
struct Args {
    /// Worker executable; defaults to physics_worker in the Cargo profile directory.
    #[arg(long)]
    worker: Option<PathBuf>,
    /// Export a native gzweb replay to a new directory.
    #[arg(long)]
    capture: Option<PathBuf>,
    /// Deliver the activation reply on the first try.
    #[arg(long)]
    reliable_transport: bool,
}

pub fn execute(
    run: impl FnOnce(&Path, Option<&Path>, bool) -> Result<u64, Box<dyn Error>>,
) -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let worker = match args.worker {
        Some(path) => path,
        None => std::env::current_exe()?
            .ancestors()
            .nth(2)
            .ok_or_else(|| io::Error::other("cannot locate physics_worker; pass --worker"))?
            .join(format!("physics_worker{}", std::env::consts::EXE_SUFFIX)),
    };
    run(&worker, args.capture.as_deref(), !args.reliable_transport)?;
    Ok(())
}
