// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record component states and load attribution from two live Bevy simulations.
#[allow(dead_code)] // The benchmark also runs open and particle-only controls.
#[path = "../../gz-sim-scene/examples/support/capsule_grasp.rs"]
mod grasp;
#[path = "soft_replay/capsule_grasp.rs"]
mod grasp_view;
mod soft_replay;
use clap::Parser;
use glam::DVec3;
use grasp::*;
use grasp_view::sample;
use serde_json::json;
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};
#[derive(Parser)]
struct Args {
    /// New gz-web-rs wire archive path.
    output: PathBuf,
    /// Compare two nonzero squeeze forces, both at friction coefficient 0.65.
    #[arg(long)]
    force_threshold: bool,
    /// All raw Bevy ticks, actual commands, body states and contact wrenches.
    #[arg(long)]
    measurements: Option<PathBuf>,
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let (control, force) = if args.force_threshold {
        (Mode::Closed, 0.012)
    } else {
        (Mode::NoFriction, SQUEEZE)
    };
    let mut runs = [
        Grasp::new(control, WORLD.into(), force)?,
        Grasp::new(Mode::Closed, WORLD.into(), SQUEEZE)?,
    ];
    let mut raw = args
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let mut frames = vec![];
    let ticks = (END / runs[0].sim.step_duration().as_secs_f64()).round() as usize;
    for tick in 0..=ticks {
        let mut f = Frame::default();
        let mut row = json!({"tick":tick,"time_ns":tick as u64*1_000_000});
        for ((name, dy), run) in [("control", 0.63), ("loaded", -0.63)]
            .into_iter()
            .zip(runs.iter_mut())
        {
            let cmd = if tick == 0 {
                [(DVec3::ZERO, DVec3::ZERO); 2]
            } else {
                run.step()?
            };
            row[name] = sample(run, name, dy, &mut f, cmd);
        }
        if tick % 5 == 0 {
            frames.push(f.finish(runs[0].sim.elapsed(), frames.len() + 1));
        }
        if let Some(w) = &mut raw {
            serde_json::to_writer(&mut *w, &row)?;
            writeln!(w)?;
        }
    }
    if let Some(w) = &mut raw {
        w.flush()?;
    }
    println!(
        "Recorded {} native gz-web-rs frames; control final COM {:?}, closed final COM {:?}.",
        frames.len(),
        runs[0].center(),
        runs[1].center()
    );
    gz_sim_cli::export_archive(&args.output, BTreeMap::new(), frames)?;
    Ok(())
}
