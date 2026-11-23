// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "support/articulated_mobility.rs"]
mod fixture;
use fixture::*;
use glam::DVec3;
use gz_sim_scene::articulated_contact;
use std::{error::Error, hint::black_box, time::Instant};

fn main() -> Result<(), Box<dyn Error>> {
    let mut g = Gripper::new(WORLD)?;
    let j = [DVec3::ZERO, DVec3::new(0.07, -0.15, 0.10), DVec3::ZERO];
    let receipt = g.apply(j)?;
    println!(
        "{} native links, {} free DoFs, source/target order {:?}; applied {:?} N s",
        receipt.before.links, receipt.before.dofs, NAMES, j[1]
    );
    for (i, name) in NAMES.iter().enumerate() {
        let v =
            DVec3::from(receipt.after_velocities[i]) - DVec3::from(receipt.before.velocities[i]);
        println!(
            "{name}: point velocity change {v:?} m/s, mobility from left finger {:?}",
            receipt.before.mobility(i, 1).unwrap()
        );
    }
    println!(
        "Whole-tree impact work {:.9} J",
        receipt.kinetic_energy_change
    );
    if std::env::args().any(|s| s == "--bench") {
        let base = g.sites();
        for count in [1, 2, 3, 8, 16] {
            let sites = (0..count)
                .map(|i| {
                    let mut s = base[i % 3];
                    s.local_point[0] += 0.003 * (i / 3) as f64;
                    s
                })
                .collect::<Vec<_>>();
            let mut elapsed = vec![];
            for run in 0..2300 {
                let want = Instant::now();
                let sample = articulated_contact::sample(&mut g.sim, black_box(&sites))?;
                black_box(sample.mobility(count - 1, 0).unwrap());
                let ns = want.elapsed().as_nanos();
                if run >= 300 {
                    elapsed.push(ns);
                }
            }
            elapsed.sort_unstable();
            println!(
                "{count} simultaneous sites, {} full 3x3 blocks, {} native cut queries: p50/p95/p99 {} / {} / {} ns",
                count * count,
                elapsed.len(),
                elapsed[999],
                elapsed[1899],
                elapsed[1979]
            );
        }
    }
    Ok(())
}
