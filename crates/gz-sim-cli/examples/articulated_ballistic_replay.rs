// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record gravity-driven material impacts and their independently checked controls.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/articulated_mobility.rs"]
mod fixture;
mod soft_replay;
#[path = "soft_replay/articulated_material.rs"]
mod view;
use clap::Parser;
use fixture::*;
use glam::DVec3;
use gz_sim_scene::deformable;
use serde_json::json;
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};
const CLOTH: &str = include_str!("../../../examples/articulated_ballistic_cloth.sdf");
const APEX: &str = include_str!("../../../examples/articulated_ballistic_apex.sdf");
#[derive(Parser)]
struct Args {
    output: PathBuf,
    /// Show a thrown cable and the missed-contact straight-chord control.
    #[arg(long)]
    apex: bool,
    /// Extend the single-event cloth beyond its return to the fingertip.
    #[arg(long, conflicts_with = "apex")]
    diagnostic_tail: bool,
    #[arg(long)]
    measurements: Option<PathBuf>,
    /// Show recurrent reaction alongside the identical-fidelity single-event continuation.
    #[arg(long, conflicts_with_all=["apex","diagnostic_tail"])]
    recurrent: bool,
}

// For the authored transverse cable the swept straight chord is the complete
// vertical rectangle spanning both cable endpoints. Check the exact point-to-
// rectangle distance at both native fingertips before admitting each coarse step.
fn chord_gap(g: &Gripper) -> Result<f64, Box<dyn Error>> {
    let nodes = deformable::bodies(&g.sim).next().unwrap().1.particles();
    let (a, b) = (
        DVec3::from(nodes[0].position),
        DVec3::from(nodes[1].position),
    );
    let v = DVec3::from(nodes[0].velocity);
    let end = a.z + 0.1 * v.z - 0.5 * 9.81 * 0.1 * 0.1;
    if a.x != b.x || a.z != b.z || v.x != 0. || v.y != 0. || nodes[0].velocity != nodes[1].velocity
    {
        return Err("straight-chord control requires this vertical ballistic source".into());
    }
    let gap = g.states()[1..]
        .iter()
        .map(|s| {
            let p = DVec3::from(s.position);
            let y = p.y.clamp(a.y.min(b.y), a.y.max(b.y));
            let z = p.z.clamp(a.z.min(end), a.z.max(end));
            p.distance(DVec3::new(a.x, y, z)) - 0.005
        })
        .fold(f64::INFINITY, f64::min);
    if gap <= 0. {
        return Err("straight chord intersected a pad and requires an actual impulse".into());
    }
    Ok(gap)
}
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    let (source, control, names, frames) = if a.recurrent {
        let first = CLOTH.replace("0.001</max_step_size>", "0.000004</max_step_size>");
        let active = include_str!("../../../examples/articulated_recurrent_cloth.sdf")
            .replace("0.001</max_step_size>", "0.000004</max_step_size>");
        (active, first, ["recurrent", "first_only"], 161)
    } else if a.apex {
        let fine = APEX.replace("0.1</max_step_size>", "0.001</max_step_size>");
        // The independently queried coarse chord stays clear. Advance the
        // unimpulsed control with the same ordinary exact free-flight owner.
        // A distant palm enrolls the owner without selecting either fingertip.
        let control = fine
            .replace(
                "<sphere link=\"default::gripper::left_finger\"/>",
                "<sphere link=\"default::gripper::palm\"/>",
            )
            .replace("<sphere link=\"default::gripper::right_finger\"/>", "");
        (fine, control, ["ballistic", "chord"], 151)
    } else {
        (
            CLOTH.to_owned(),
            CLOTH.replace("<gravity>0 0 -9.81</gravity>", "<gravity>0 0 0</gravity>"),
            ["gravity", "zero_g"],
            if a.diagnostic_tail { 161 } else { 131 },
        )
    };
    let mut pair = [Gripper::new(&source)?, Gripper::new(&control)?];
    let mut info = a
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let mut archive = vec![];
    let mut min_gap = f64::INFINITY;
    for idx in 0..frames {
        let mut visuals = Frame::default();
        let mut row = json!({"ordinal":idx+1,"apex":a.apex});
        for (lane, g) in pair.iter_mut().enumerate() {
            let prev = g.sim.elapsed();
            if idx > 0 {
                g.sim.step_exact(if a.recurrent { 500 } else { 2 })?;
            }
            let name = names[lane];
            let mut raw = if a.recurrent {
                view::record_impulses(g, name, [0.85, -0.85][lane], true, &mut visuals, true)
            } else {
                view::record(g, name, [0.85, -0.85][lane], true, &mut visuals)
            };
            let latest = deformable::bodies(&g.sim)
                .next()
                .unwrap()
                .1
                .latest_articulated_impact();
            if !a.recurrent && lane == 1 && latest.is_some() {
                return Err("control accumulated an unexpected contact".into());
            }
            raw["event"] = json!(
                latest
                    .filter(|r| idx > 0 && r.time > prev && r.time <= g.sim.elapsed())
                    .map(view::event)
            );
            if a.recurrent {
                raw["count"] = json!(
                    deformable::bodies(&g.sim)
                        .next()
                        .unwrap()
                        .1
                        .articulated_impact_count()
                );
            }
            row[name] = raw;
            row["time_ns"] = json!(g.sim.elapsed().as_nanos() as u64);
            if a.apex && lane == 1 && idx % 50 == 0 && idx + 1 < frames {
                let gap = chord_gap(g)?;
                row["chord_gap_m"] = json!(gap);
                min_gap = min_gap.min(gap);
            }
        }
        archive.push(visuals.finish(pair[0].sim.elapsed(), idx + 1));
        if let Some(w) = &mut info {
            serde_json::to_writer(&mut *w, &row)?;
            writeln!(w)?;
        }
    }
    if let Some(w) = &mut info {
        w.flush()?;
    }
    println!(
        "Recorded {frames} actual two-world states over {:?}.",
        pair[0].sim.elapsed()
    );
    if a.apex {
        println!(
            "Three separately admitted 100 ms chord-only intervals, minimum entire-chord clearance {min_gap:.12} m. Actual ballistic states sampled at 2 ms display intervals."
        );
    }
    gz_sim_cli::export_archive(&a.output, BTreeMap::new(), archive)?;
    Ok(())
}
