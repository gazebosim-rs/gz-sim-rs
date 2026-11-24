// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "support/worker_pool.rs"]
mod demo;
use clap::Parser;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: std::path::PathBuf,
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!(
        "{}",
        serde_json::to_string_pretty(&demo::qualify(&Args::parse().worker)?)?
    );
    Ok(())
}
