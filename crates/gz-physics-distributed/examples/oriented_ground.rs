// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "support/oriented_ground.rs"]
mod ground;
use clap::Parser;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: std::path::PathBuf,
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    ground::run(&Args::parse().worker)
}
