// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Trace every signed momentum exchange and independently measured clearance.
#[allow(dead_code)]
#[path = "support/articulated_mobility.rs"]
mod fixture;
use fixture::*;
use glam::DVec3;
use gz_sim_scene::deformable;
use parry3d_f64::{query::PointQuery, shape::Triangle};
use std::error::Error;
const SOURCE: &str = include_str!("../../../examples/articulated_recurrent_cloth.sdf");
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let mut s = SOURCE.to_owned();
    if let Some(w) = args.first() {
        s = s.replace("4000", w);
    }
    if let Some(t) = args.get(1) {
        s = s.replace("0.001</max_step_size>", &format!("{t}</max_step_size>"));
    }
    if let Some(e) = args.get(2) {
        s = s.replace("restitution=\"0.55\"", &format!("restitution=\"{e}\""));
    }
    let mut g = Gripper::new(&s)?;
    let mut minimum = f64::INFINITY;
    while g.sim.elapsed().as_secs_f64() < 0.32 {
        if let Err(e) = g.sim.step_exact(1) {
            eprintln!(
                "stopped after {:?}, minimum gap {minimum:e}",
                g.sim.elapsed()
            );
            return Err(e.into());
        }
        let soft = deformable::bodies(&g.sim).next().unwrap().1;
        let p: Vec<_> = soft
            .particles()
            .iter()
            .map(|n| DVec3::from(n.position))
            .collect();
        for tip in [1, 2] {
            let c = DVec3::from(g.states()[tip].position);
            for f in [[0, 1, 3], [0, 3, 2]] {
                let tri = Triangle::new(p[f[0]], p[f[1]], p[f[2]]);
                minimum = minimum.min(c.distance(tri.project_local_point(c, false).point) - 0.112);
            }
        }
        for r in soft.impacts_this_step() {
            let t = r.detection.unwrap();
            println!(
                "{:.12} s, {} ns, dissipation {:.9} J",
                t.public_start.as_secs_f64() + t.seconds_after_start,
                r.time.as_nanos(),
                r.material.dissipated_energy
            );
            for l in &r.material.loads {
                println!(
                    "  fingertip {} {:?}, {:.9} -> {:.9} m/s, on rigid {:?} N s, clearance {:.4e} m",
                    l.site.sphere,
                    l.site.feature,
                    l.initial_normal_velocity,
                    l.final_normal_velocity,
                    l.on_rigid,
                    l.site.clearance
                );
            }
        }
    }
    println!(
        "through {:?}, {} contact events, minimum independently sampled gap {:.9e} m",
        g.sim.elapsed(),
        deformable::bodies(&g.sim)
            .next()
            .unwrap()
            .1
            .articulated_impact_count(),
        minimum
    );
    Ok(())
}
