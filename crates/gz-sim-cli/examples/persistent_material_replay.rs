// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two ordinary SDF worlds with actual normal support forces and release.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/articulated_mobility.rs"]
mod fixture;
#[allow(dead_code)]
#[path = "soft_replay/articulated_material.rs"]
mod geometry;
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/supported_cable.rs"]
mod mechanical;
mod soft_replay;
use clap::Parser;
use fixture::*;
use glam::{DQuat, DVec3};
use gz_sim_scene::deformable::{self, articulated_impact::SupportApplication};
use serde_json::{Value, json};
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
    output: PathBuf,
    #[arg(long)]
    release: bool,
    #[arg(long)]
    measurements: Option<PathBuf>,
}
fn impulse(a: Option<&SupportApplication>) -> Value {
    a.map_or(Value::Null,|v|{
        json!({"native_change_j":v.native.kinetic_energy_change,"loads":v.material.iter().map(|l|json!({"sphere":l.site.sphere,"at":l.site.point,"normal":l.site.normal,"clearance":l.site.clearance,"vertices":l.site.feature.vertices(),"weights":l.site.weights,"on_link":l.on_rigid,"on_particles":l.on_particles})).collect::<Vec<_>>()})
    })
}
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    let (sources, names) = if a.release {
        (
            [mechanical::RELEASE.to_owned(), mechanical::launched(1.24)],
            ["sliding", "immediate_release"],
        )
    } else {
        (
            [
                mechanical::REST.to_owned(),
                include_str!("../../../examples/articulated_supported_cloth.sdf").to_owned(),
            ],
            ["cable", "cloth"],
        )
    };
    let mut worlds = [Gripper::new(&sources[0])?, Gripper::new(&sources[1])?];
    let mut archive = Vec::new();
    let mut output = a
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    for frame in 0..=200 {
        let mut visuals = Frame::default();
        let mut row = json!({"ordinal":frame+1,"release":a.release});
        for (lane, world) in worlds.iter_mut().enumerate() {
            let mut every = vec![];
            let mut begin = vec![];
            let mut total = [DVec3::ZERO; 2];
            let mut centers = [None; 2];
            if frame > 0 {
                for _ in 0..2 {
                    world.sim.step_exact(1)?;
                    let sample = deformable::bodies(&world.sim).next().unwrap().1;
                    begin.extend(
                        sample
                            .impacts_this_step()
                            .iter()
                            .map(|e| geometry::event(e)),
                    );
                    for s in sample.normal_support_this_step() {
                        every.push(json!({"start_ns":s.start.as_nanos() as u64,"dt_ns":s.duration.as_nanos() as u64,"head":impulse(s.beginning.as_ref()),"tail":impulse(s.end.as_ref()),"end_gaps":s.endpoint_clearances,"trials":s.trajectory_trials}));
                        for application in s.beginning.iter().chain(&s.end) {
                            for l in &application.material {
                                let j = DVec3::from(l.on_rigid);
                                if j.length_squared() > 0. {
                                    let i = l.site.sphere;
                                    total[i] += j;
                                    centers[i] = Some(DVec3::from(l.site.point));
                                }
                            }
                        }
                    }
                }
            }
            let shift = [0.85, -0.85][lane];
            let name = names[lane];
            let mut raw = geometry::record_bare(world, name, shift, true, &mut visuals);
            let force = total.map(|x| x / 0.002);
            let q = DQuat::from_rotation_z(-0.72);
            for site in 0..2 {
                let at =
                    centers[site].unwrap_or_else(|| DVec3::from(world.states()[site + 1].position));
                for (side, sign, color) in [
                    ("support_on_link", 1., [0.99, 0.31, 0.16, 1.]),
                    ("support_on_material", -1., [1., 0.78, 0.18, 1.]),
                ] {
                    for k in 1..=3 {
                        let p = q * (at + sign * (0.105 * k as f64 / 3.) * force[site])
                            + shift * DVec3::Y;
                        visuals.add(
                            &format!("{name}_{side}_{site}_{k}"),
                            p,
                            DQuat::IDENTITY,
                            json!({"kind":"sphere","radius":if k==3{0.019}else{0.012}}),
                            color,
                        );
                    }
                }
            }
            raw["forces_n"] = json!(force.map(|v| v.to_array()));
            raw["origins"] = json!(centers.map(|v| v.map(|x| x.to_array())));
            raw["support"] = json!(every);
            raw["events"] = json!(begin);
            row[name] = raw;
            row["time_ns"] = json!(world.sim.elapsed().as_nanos() as u64);
        }
        archive.push(visuals.finish(worlds[0].sim.elapsed(), frame + 1));
        if let Some(w) = &mut output {
            serde_json::to_writer(&mut *w, &row)?;
            writeln!(w)?;
        }
    }
    if let Some(w) = &mut output {
        w.flush()?;
    }
    gz_sim_cli::export_archive(&a.output, BTreeMap::new(), archive)?;
    println!(
        "Recorded 201 physical states from both SDF worlds. Each force glyph measures all normal impulses accumulated over the preceding 2 ms."
    );
    Ok(())
}
