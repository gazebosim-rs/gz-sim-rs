// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record continuous material impact against a native articulation between public ticks.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/articulated_mobility.rs"]
mod fixture;
mod soft_replay;
#[path = "soft_replay/articulated_material.rs"]
mod view;
use clap::Parser;
use fixture::*;
use gz_sim_scene::deformable::{self, articulated_impact};
use serde_json::json;
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};
const CLOTH: &str = include_str!("../../../examples/articulated_first_impact.sdf");
const CABLE: &str = include_str!("../../../examples/articulated_transient_cable.sdf");
#[derive(Parser)]
struct Args {
    output: PathBuf,
    /// Compare swept contact with querying only at public endpoints.
    #[arg(long)]
    transient: bool,
    #[arg(long)]
    measurements: Option<PathBuf>,
}
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    let mut g = if a.transient {
        let start = CABLE.find("      <articulated_impact").unwrap();
        let end = CABLE.find("</articulated_impact>").unwrap() + "</articulated_impact>".len();
        let mut off = CABLE.to_owned();
        off.replace_range(start..end, "");
        [Gripper::new(CABLE)?, Gripper::new(&off)?]
    } else {
        [
            Gripper::new(&CLOTH.replace("0.001</max_step_size>", "0.002</max_step_size>"))?,
            Gripper::new(&CLOTH.replace("0.001</max_step_size>", "0.0005</max_step_size>"))?,
        ]
    };
    let mut raw = a
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let frames = if a.transient { 31 } else { 121 };
    let mut archive = vec![];
    for frame in 0..frames {
        let mut f = Frame::default();
        let mut row = json!({"ordinal":frame+1,"transient":a.transient});
        for (lane, g) in g.iter_mut().enumerate() {
            let previous = g.sim.elapsed();
            if frame > 0 {
                g.sim
                    .step_exact(if !a.transient && lane == 1 { 4 } else { 1 })?;
                if a.transient && lane == 1 {
                    let ids = [g.ids[1].0, g.ids[2].0];
                    if articulated_impact::resolve(&mut g.sim, "material", &ids, 0.55)?.is_some() {
                        return Err(
                            "endpoint control unexpectedly saw the between-tick collision".into(),
                        );
                    }
                }
            }
            let lane_name = if a.transient {
                ["swept", "endpoint"][lane]
            } else {
                ["two_ms", "half_ms"][lane]
            };
            let mut state = view::record(g, lane_name, [0.85, -0.85][lane], false, &mut f);
            let sample = deformable::bodies(&g.sim).next().unwrap().1;
            state["event"] = json!(
                sample
                    .latest_articulated_impact()
                    .filter(|r| frame > 0 && r.time > previous && r.time <= g.sim.elapsed())
                    .map(view::event)
            );
            state["tick_ns"] = json!(g.sim.step_duration().as_nanos() as u64);
            row[lane_name] = state;
            row["time_ns"] = json!(g.sim.elapsed().as_nanos() as u64);
        }
        archive.push(f.finish(g[0].sim.elapsed(), frame + 1));
        if let Some(w) = &mut raw {
            serde_json::to_writer(&mut *w, &row)?;
            writeln!(w)?;
        }
    }
    if let Some(w) = &mut raw {
        w.flush()?;
    }
    println!(
        "Recorded {frames} actual public state comparisons over {:?}, with geometric impulses preserved at the internal native and material event.",
        g[0].sim.elapsed()
    );
    gz_sim_cli::export_archive(&a.output, BTreeMap::new(), archive)?;
    Ok(())
}
