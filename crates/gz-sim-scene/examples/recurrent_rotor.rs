// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "support/rotating_pad.rs"]
mod source;
use glam::DVec3;
use parry3d_f64::{query::PointQuery, shape::Triangle};
use source::*;
use std::error::Error;
const SHEET: &str = include_str!("../../../examples/rotating_recurrent_cloth.sdf");
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let mut s = SHEET.to_owned();
    if let Some(w) = args.first() {
        s = s.replace("4000", w);
    }
    if let Some(t) = args.get(1) {
        s = s.replace("0.001</max_step_size>", &format!("{t}</max_step_size>"));
    }
    if let Some(e) = args.get(2) {
        s = s.replace("restitution=\"0.55\"", &format!("restitution=\"{e}\""));
    }
    let mut g = Rotor::new(&s)?;
    let mut minimum = f64::INFINITY;
    while g.sim.elapsed().as_secs_f64() < 0.32 {
        g.advance(1)?;
        let soft = g.material();
        let p: Vec<_> = soft
            .particles()
            .iter()
            .map(|n| DVec3::from(n.position))
            .collect();
        let c = DVec3::from(g.state().position);
        for f in [[0, 1, 3], [0, 3, 2]] {
            let tri = Triangle::new(p[f[0]], p[f[1]], p[f[2]]);
            minimum = minimum.min(c.distance(tri.project_local_point(c, false).point) - 0.05);
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
                    "  pad {} {:?}, {:.9} -> {:.9} m/s, on rigid {:?} N s, clearance {:.4e} m, rotor speed {:.9} rad/s",
                    l.site.sphere,
                    l.site.feature,
                    l.initial_normal_velocity,
                    l.final_normal_velocity,
                    l.on_rigid,
                    l.site.clearance,
                    g.state().angular_velocity[2]
                );
            }
        }
    }
    println!(
        "through {:?}, {} contact events, minimum independently sampled gap {:.9e} m",
        g.sim.elapsed(),
        g.material().articulated_impact_count(),
        minimum
    );
    Ok(())
}
