// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Per-tick latency of the two finite receiver examples through the ordinary Bevy loop.
use gz_sim_scene::{PhysicsEngine, deformable::bodies, load_world_with_sdf_step};
use std::{hint::black_box, time::Instant};
fn main() {
    println!("body,mode,substeps,window,repeats,ticks,median_us,p95_us,p99_us");
    for (name, xml, hit) in [
        (
            "cable",
            include_str!("../../../examples/finite_material_cable.sdf"),
            67,
        ),
        (
            "cloth",
            include_str!("../../../examples/finite_material_cloth.sdf"),
            56,
        ),
    ] {
        for n in [1, 4] {
            for material in [false, true] {
                let xml = xml
                    .replace("substeps=\"4\"", &format!("substeps=\"{n}\""))
                    .replace(
                        "capsule_contact=\"material\"",
                        &format!(
                            "capsule_contact=\"{}\"",
                            if material { "material" } else { "particles" }
                        ),
                    );
                let mut windows: [Vec<f64>; 3] = Default::default();
                for repeat in 0..13 {
                    let mut sim = load_world_with_sdf_step(
                        &xml,
                        "finite_material.sdf",
                        "default",
                        &sdformat::NoResources,
                        PhysicsEngine::MultiFidelity,
                    )
                    .unwrap();
                    for tick in 1..=350 {
                        let begin = Instant::now();
                        sim.step_exact(1).unwrap();
                        let us = begin.elapsed().as_secs_f64() * 1e6;
                        black_box(bodies(&sim).next().unwrap().1.particles());
                        if repeat >= 2 {
                            let key = if tick < hit - 2 {
                                0
                            } else if tick <= hit + 2 {
                                1
                            } else {
                                2
                            };
                            windows[key].push(us);
                        }
                    }
                }
                for (label, mut samples) in ["approach", "landing", "following"]
                    .into_iter()
                    .zip(windows)
                {
                    samples.sort_by(f64::total_cmp);
                    let len = samples.len();
                    let q = |p: usize| samples[(p * len).div_ceil(100) - 1];
                    println!(
                        "{name},{},{n},{label},11,{len},{:.5},{:.5},{:.5}",
                        if material { "material" } else { "particles" },
                        q(50),
                        q(95),
                        q(99)
                    );
                }
            }
        }
    }
}
