// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently observe the return of an unimpulsed, deforming sheet.
#[allow(dead_code)]
#[path = "support/articulated_mobility.rs"]
mod fixture;
use fixture::*;
use glam::DVec3;
use gz_sim_scene::deformable;
use parry3d_f64::{query::PointQuery, shape::Triangle};
use std::error::Error;
const SOURCE: &str = include_str!("../../../examples/articulated_ballistic_cloth.sdf");
fn main() -> Result<(), Box<dyn Error>> {
    for dt in [
        "0.001",
        "0.0005",
        "0.00025",
        "0.000125",
        "0.0000625",
        "0.00003125",
    ] {
        let mut g = Gripper::new(
            &SOURCE.replace("0.001</max_step_size>", &format!("{dt}</max_step_size>")),
        )?;
        let public = g.sim.step_duration().as_secs_f64();
        let mut prior = [0.; 2];
        let mut start = [None; 2];
        let mut minimum = [1.0_f64; 2];
        while g.sim.elapsed().as_secs_f64() < 0.32 {
            g.sim.step_exact(1)?;
            let time = g.sim.elapsed().as_secs_f64();
            let soft = deformable::bodies(&g.sim).next().unwrap().1;
            if soft.latest_articulated_impact().is_none() {
                continue;
            }
            let p: Vec<_> = soft
                .particles()
                .iter()
                .map(|n| DVec3::from(n.position))
                .collect();
            for site in 0..2 {
                let center = DVec3::from(g.states()[site + 1].position);
                let gap = [[0, 1, 3], [0, 3, 2]]
                    .map(|i| {
                        center.distance(
                            Triangle::new(p[i[0]], p[i[1]], p[i[2]])
                                .project_local_point(center, false)
                                .point,
                        ) - 0.112
                    })
                    .into_iter()
                    .fold(f64::INFINITY, f64::min);
                minimum[site] = minimum[site].min(gap);
                if prior[site] > 0. && gap < 0. && start[site].is_none() {
                    let estimate = time - public + public * prior[site] / (prior[site] - gap);
                    start[site] = Some(estimate);
                    println!(
                        "native/XPBD step {dt} s, fingertip {site} first return between {:.9} and {time:.9} s, independent signed-gap secant {estimate:.12} s, bracket {:.9e}/{gap:.9e} m",
                        time - public,
                        prior[site]
                    );
                }
                prior[site] = gap;
            }
        }
        assert!(start[0].is_some());
        println!(
            "native/XPBD step {dt} s, minimum signed clearances through 320 ms: {minimum:?} m"
        );
    }
    Ok(())
}
