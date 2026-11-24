// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record real cable/cloth interior impacts on both fingertips of ordinary Bevy grippers.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/articulated_mobility.rs"]
mod fixture;
mod soft_replay;
#[path = "soft_replay/articulated_material.rs"]
mod view;
use clap::Parser;
use fixture::*;
use serde_json::json;
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};
use view::record;

#[derive(Parser)]
struct Args {
    output: PathBuf,
    /// Compare the same cloth and centered gripper, first floating, then hinged to the world.
    #[arg(long)]
    grounded: bool,
    /// Compare a slipping cloth and a cloth that sticks instantaneously at impact.
    #[arg(long, conflicts_with = "grounded")]
    friction: bool,
    #[arg(long)]
    measurements: Option<PathBuf>,
}
const CLOTH: &str = include_str!("../../../examples/articulated_material_cloth_impact.sdf");
const CABLE: &str = include_str!("../../../examples/articulated_material_cable_impact.sdf");
const FRICTION: &str = include_str!("../../../examples/articulated_friction_impact.sdf");
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    let mut g = if a.grounded {
        [
            Gripper::new(&centered_xml(CLOTH, false))?,
            Gripper::new(&centered_xml(CLOTH, true))?,
        ]
    } else if a.friction {
        [
            Gripper::new(&FRICTION.replace("friction=\"0.9\"", "friction=\"0.09\""))?,
            Gripper::new(FRICTION)?,
        ]
    } else {
        [Gripper::new(CLOTH)?, Gripper::new(CABLE)?]
    };
    let mut raw = a
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let mut frames = Vec::new();
    for k in 0..=240 {
        let mut f = Frame::default();
        let mut row =
            json!({"tick":k,"grounded_comparison":a.grounded,"friction_comparison":a.friction});
        for (i, g) in g.iter_mut().enumerate() {
            if k > 0 {
                g.sim.step_exact(1)?;
            }
            let name = if a.grounded {
                ["free", "grounded"][i]
            } else if a.friction {
                ["sliding", "sticking"][i]
            } else {
                ["cloth", "cable"][i]
            };
            row[name] = record(g, name, [0.85, -0.85][i], a.grounded && i == 1, &mut f);
        }
        if k % 2 == 0 {
            frames.push(f.finish(g[0].sim.elapsed(), frames.len() + 1));
        }
        if let Some(w) = &mut raw {
            serde_json::to_writer(&mut *w, &row)?;
            writeln!(w)?;
        }
    }
    if let Some(w) = &mut raw {
        w.flush()?;
    }
    let count = frames.len();
    gz_sim_cli::export_archive(&a.output, BTreeMap::new(), frames)?;
    println!(
        "Recorded {count} physical frames, both simultaneous two-fingertip impacts at 100 ms."
    );
    Ok(())
}
