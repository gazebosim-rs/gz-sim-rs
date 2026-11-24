// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record curved contact and ground support across four workers and checkpoint recovery.
#[path = "support/curved_ground.rs"]
mod curved_ground;
use clap::Parser;
use std::{fs, path::PathBuf};
#[derive(Parser)]
struct Args {
    /// New output directory for the physical traces and recovery checkpoint.
    output: PathBuf,
    #[arg(long)]
    worker: PathBuf,
}
fn main() -> curved_ground::Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.output)?;
    let result = curved_ground::run(&args.worker, &args.output, true)?;
    let json = serde_json::to_string_pretty(&result)?;
    fs::write(args.output.join("qualification.json"), &json)?;
    println!("{json}");
    Ok(())
}
