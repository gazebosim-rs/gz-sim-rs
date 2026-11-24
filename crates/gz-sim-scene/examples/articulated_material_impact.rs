// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Simulate the public SDF-authored event and print actual native and material impulses.
#[allow(dead_code)]
#[path = "support/articulated_mobility.rs"]
mod fixture;
use fixture::*;
use glam::DVec3;
use gz_sim_scene::{deformable, reset_loaded_world};
use std::{error::Error, time::Instant};
const CLOTH: &str = include_str!("../../../examples/articulated_material_cloth_impact.sdf");
const CABLE: &str = include_str!("../../../examples/articulated_material_cable_impact.sdf");
fn main() -> Result<(), Box<dyn Error>> {
    let bench = std::env::args().any(|s| s == "--bench");
    for (xml, kind) in [(CLOTH, "cloth"), (CABLE, "cable")] {
        let mut g = Gripper::new(xml)?;
        g.sim.step_exact(100)?;
        let material = deformable::bodies(&g.sim).next().unwrap().1;
        let hit = material
            .latest_articulated_impact()
            .ok_or("missing physical impact")?;
        println!(
            "{kind} at {:.3} seconds, {} native coordinates, dissipated {:.9} J",
            hit.time.as_secs_f64(),
            hit.native.before.dofs,
            hit.material.dissipated_energy
        );
        for l in &hit.material.loads {
            let n = DVec3::from(l.site.normal);
            println!(
                "{}: {:?} with weights {:?}, signed world impulse ON the rigid link {:?} N s, {:.6} to {:.6} m/s material-minus-rigid normal velocity, normal scalar on material {:.9} N s",
                NAMES[1 + l.site.sphere],
                l.site.feature,
                l.site.weights,
                l.on_rigid,
                l.initial_normal_velocity,
                l.final_normal_velocity,
                -n.dot(l.on_rigid.into())
            );
        }
        if bench {
            let mut clean = vec![];
            let mut contact = vec![];
            for trial in 0..240 {
                reset_loaded_world(&mut g.sim, |_| ())?;
                g.sim.step_exact(98)?;
                let start = Instant::now();
                g.sim.step_exact(1)?;
                let gap_cost = start.elapsed().as_nanos();
                let start = Instant::now();
                g.sim.step_exact(1)?;
                let hit_cost = start.elapsed().as_nanos();
                if trial >= 40 {
                    clean.push(gap_cost);
                    contact.push(hit_cost);
                }
            }
            for (label, sample) in [
                ("clear adjacent tick", clean),
                ("simultaneous impact tick", contact),
            ] {
                let mut ns = sample;
                ns.sort_unstable();
                println!(
                    "{kind}, {label}, 200 post-warmup ordinary public ticks, p50/p95/p99 {} / {} / {} ns",
                    ns[99], ns[189], ns[197]
                );
            }
        }
    }
    Ok(())
}
