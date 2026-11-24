// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Capture one validated published pair checkpoint without pausing or acquiring its writer lease.
use clap::Parser;
#[path = "support/published_pair.rs"]
mod published_pair;
use std::{
    error::Error,
    fs,
    io::Write,
    path::PathBuf,
    thread,
    time::{Duration, Instant},
};
#[derive(Parser)]
struct Args {
    /// Published barrier.gzmultisphere from a running or stopped session.
    archive: PathBuf,
    /// New directory for a recoverable checkpoint and its public receipt.
    output: PathBuf,
    #[arg(long)]
    tick: Option<u64>,
    #[arg(long)]
    min_epoch: Option<u64>,
    #[arg(long)]
    even_epoch: bool,
    #[arg(long)]
    pair_loaded: bool,
    #[arg(long, default_value_t = 30)]
    seconds: u64,
}
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    let start = Instant::now();
    loop {
        let (cut, bytes) = published_pair::snapshot(&a.archive)?;
        if a.tick.is_none_or(|t| t == cut.observation.hinge.tick)
            && a.min_epoch.is_none_or(|e| cut.epoch >= e)
            && (!a.even_epoch || cut.epoch % 2 == 0)
            && (!a.pair_loaded || cut.observation.sphere_reaction.normal_impulse > 0.)
        {
            let destination = a
                .output
                .parent()
                .filter(|p| !p.as_os_str().is_empty())
                .unwrap_or(std::path::Path::new("."));
            let dir = tempfile::tempdir_in(destination)?;
            fs::write(dir.path().join("barrier.gzmultisphere"), &bytes)?;
            let receipt = serde_json::to_vec_pretty(&cut)?;
            fs::write(dir.path().join("receipt.json"), &receipt)?;
            // Keep a prior sample intact if the operator reuses a nonempty path.
            fs::rename(dir.path(), &a.output)?;
            std::io::stdout().lock().write_all(&receipt)?;
            return Ok(());
        }
        if start.elapsed() >= Duration::from_secs(a.seconds) {
            return Err("no published cut matched before the deadline".into());
        }
        thread::sleep(Duration::from_millis(1));
    }
}
