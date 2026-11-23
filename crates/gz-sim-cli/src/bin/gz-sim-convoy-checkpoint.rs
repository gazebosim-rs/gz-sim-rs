// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Capture or review immutable, worker-free restart inputs for a convoy supervisor.
use clap::{Parser, Subcommand};
use gz_sim_cli::live::{ConvoyCapsule, JointTorqueDrive};
use serde_json::json;
use std::{error::Error, fs, io::Read, path::PathBuf};

#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    command: Command,
}
#[derive(Subcommand)]
enum Command {
    /// Select an existing physics archive and explicitly author its scene and continuing drive.
    Create {
        #[arg(long)]
        session: PathBuf,
        #[arg(long)]
        sdf: PathBuf,
        #[arg(long, default_value = "default")]
        world: String,
        #[arg(
            long,
            conflicts_with = "torque_nm",
            required_unless_present = "torque_nm"
        )]
        drive_program: Option<PathBuf>,
        #[arg(long, allow_hyphen_values = true)]
        torque_nm: Option<f64>,
        #[arg(long)]
        output: PathBuf,
    },
    /// Capture a new committed cut of the same session, inheriting a reviewed source's SDF and drive.
    CaptureLatest {
        #[arg(long)]
        from: PathBuf,
        #[arg(long)]
        package_sha256: String,
        #[arg(long)]
        session: PathBuf,
        #[arg(long)]
        output: PathBuf,
    },
    /// Verify all package contents offline and print restart metadata.
    Inspect {
        package: PathBuf,
        #[arg(long)]
        package_sha256: Option<String>,
    },
}

fn main() -> Result<(), Box<dyn Error>> {
    let capsule = match Args::parse().command {
        Command::Create {
            session,
            sdf,
            world,
            drive_program,
            torque_nm,
            output,
        } => {
            let mut xml = String::new();
            fs::File::open(sdf)?
                .take(1024 * 1024 + 1)
                .read_to_string(&mut xml)?;
            let drive = match drive_program {
                Some(file) => JointTorqueDrive::read(fs::File::open(file)?)?,
                None => JointTorqueDrive::constant(torque_nm.ok_or("missing selected drive")?)?,
            };
            let cap = ConvoyCapsule::capture(
                &session.join("barrier.gzmultisphere"),
                &xml,
                &world,
                &drive,
            )?;
            cap.publish(&output)?;
            cap
        }
        Command::CaptureLatest {
            from,
            package_sha256,
            session,
            output,
        } => {
            let previous = ConvoyCapsule::open(&from, &package_sha256)?;
            let next = ConvoyCapsule::capture(
                &session.join("barrier.gzmultisphere"),
                previous.sdf(),
                previous.world(),
                previous.drive(),
            )?;
            if next.origin() != previous.origin() {
                return Err("checkpoint belongs to another session or writer marker".into());
            }
            let before = &previous.cut().cut;
            let after = &next.cut().cut;
            if (after.epoch, after.observation.hinge.tick)
                < (before.epoch, before.observation.hinge.tick)
            {
                return Err("current checkpoint precedes the selected source".into());
            }
            if (after.epoch, after.observation.hinge.tick)
                == (before.epoch, before.observation.hinge.tick)
                && next.cut().sha256 != previous.cut().sha256
            {
                return Err("different archive at the same clock; capture directly from the active supervisor".into());
            }
            next.publish(&output)?;
            next
        }
        Command::Inspect {
            package,
            package_sha256,
        } => match package_sha256 {
            Some(digest) => ConvoyCapsule::open(&package, &digest)?,
            None => ConvoyCapsule::inspect(&package)?,
        },
    };
    let cut = &capsule.cut().cut;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "package_sha256":capsule.sha256(),"archive_sha256":capsule.cut().sha256,"world":capsule.world(),
            "epoch":cut.epoch,"tick":cut.observation.hinge.tick,"width":cut.width,"owners":cut.owners,
            "pending_torque_nm":cut.pending_effort,"next_torque_nm":capsule.drive().interval_effort(cut.observation.hinge.tick),
            "drive":capsule.drive(),"origin":capsule.origin(),"normal_impulse_ns":cut.observation.sphere_reaction.normal_impulse,
        }))?
    );
    Ok(())
}
