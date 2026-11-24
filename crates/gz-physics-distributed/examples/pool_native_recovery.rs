// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Kill and recover a four-worker coordinator with two loaded native contacts and interleaved impulses.
#[path = "support/pool_crash.rs"]
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
    /// Use adaptive cuboid contacts in both coordinators and the uninterrupted control.
    #[arg(long)]
    adaptive_contacts: bool,
    /// Run the adaptive crash trial with sphere pairs under load on the shared floor.
    #[arg(long, conflicts_with = "adaptive_contacts")]
    curved_ground: bool,
    /// Add a coarse airborne body to the loaded curved crash trial.
    #[arg(long, conflicts_with_all = ["adaptive_contacts","curved_ground"])]
    island_refinement: bool,
    /// Use diagonal grounded lanes that overlap under the earlier per-axis reach bound.
    #[arg(long, conflicts_with_all = ["adaptive_contacts", "curved_ground", "island_refinement"])]
    diagonal_lanes: bool,
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
    let profile = if args.diagonal_lanes {
        run::Profile::DiagonalLanes
    } else if args.island_refinement {
        run::Profile::Islands
    } else if args.curved_ground {
        run::Profile::CurvedGround
    } else if args.adaptive_contacts {
        run::Profile::Adaptive
    } else {
        run::Profile::Coarse
    };
    match args.mode {
        Mode::Original => run::original(&worker, &args.directory, profile),
        Mode::Replacement => run::replacement(&worker, &args.directory, profile),
        Mode::Demonstrate => {
            let command = |mode: &str| {
                let mut c = Command::new(&exe);
                if args.diagonal_lanes {
                    c.arg("--diagonal-lanes");
                }
                if args.island_refinement {
                    c.arg("--island-refinement");
                }
                if args.curved_ground {
                    c.arg("--curved-ground");
                }
                if args.adaptive_contacts {
                    c.arg("--adaptive-contacts");
                }
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
