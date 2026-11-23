// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Present independent Bevy/Rapier sliding-payload runs on a shared relative launch clock.
#[path = "sliding_payload_view/grazing.rs"]
mod grazing;
#[path = "sliding_payload_view/plots.rs"]
mod plots;
#[path = "sliding_payload_view/replay.rs"]
mod replay;
mod soft_replay;
use clap::{Parser, Subcommand};
use serde::{Deserialize, de::DeserializeOwned};
use std::{
    error::Error,
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    command: Command,
}
#[derive(Subcommand)]
enum Command {
    /// Export a native viewer archive and print its frame index.
    Replay {
        measurements: PathBuf,
        output: PathBuf,
    },
    /// Export the previous, updated and always-dynamic grazing strike and its response plots.
    Grazing {
        measurements: PathBuf,
        output: PathBuf,
    },
    /// Compare the two measured offset/timestep matrices and draw the measured responses.
    Plot {
        measurements: PathBuf,
        baseline: PathBuf,
        output: PathBuf,
    },
}
#[derive(Deserialize)]
struct Trace {
    dt_s: f64,
    settle_s: f64,
    samples: Vec<Sample>,
}
#[derive(Deserialize)]
struct Sample {
    tick: u64,
    tier: String,
    near_miss_tier: String,
    automatic: [Body; 3],
    reference: [Body; 3],
    near_miss: [Body; 3],
    automatic_impulse_ns: f64,
    automatic_floor_impulse_ns: f64,
    reference_floor_impulse_ns: f64,
    reference_impulse_ns: f64,
    hinge_rate_rad_s: f64,
    reference_hinge_rate_rad_s: f64,
    near_miss_hinge_rate_rad_s: f64,
}
#[derive(Deserialize)]
struct Body {
    name: String,
    position: [f64; 3],
    orientation: [f64; 4],
    velocity: [f64; 3],
}
fn read<T: DeserializeOwned>(p: &Path) -> Result<T> {
    Ok(serde_json::from_reader(BufReader::new(File::open(p)?))?)
}
fn main() -> Result<()> {
    match Args::parse().command {
        Command::Replay {
            measurements,
            output,
        } => replay::export(&read(&measurements.join("trajectory.json"))?, &output),
        Command::Grazing {
            measurements,
            output,
        } => grazing::export(&measurements, &output),
        Command::Plot {
            measurements,
            baseline,
            output,
        } => plots::render(&measurements, &baseline, &output),
    }
}
