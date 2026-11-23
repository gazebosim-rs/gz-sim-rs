// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cost of one identical soft-body tick near or far from four finite guides.
use glam::DVec3;
use gz_physics_xpbd::{Boundary, Particle, SoftBody, SolverConfig};
use std::{
    hint::black_box,
    time::{Duration, Instant},
};
fn case(cloth: bool, landing: bool, material: bool) -> (SoftBody, Vec<Boundary>) {
    let config = SolverConfig {
        substeps: 1,
        iterations: 4,
        gravity: [0.; 3],
        self_contact: false,
        particle_radius: 0.004,
        friction: 0.,
    };
    let cols = if cloth { 11 } else { 81 };
    let rows = if cloth { 11 } else { 1 };
    let z = if landing { 0.0188 } else { 0.3 };
    let ps = (0..cols * rows)
        .map(|i| Particle {
            position: [(i % cols) as f64 * 0.14, (i / cols) as f64 * 0.14, z],
            velocity: [0., 0., -1.],
            inverse_mass: 20.,
        })
        .collect();
    let mut b = if cloth {
        SoftBody::cloth(ps, cols, 1e-5, 1e-5, 1e-5, config).unwrap()
    } else {
        SoftBody::cable(ps, 1e-5, 1e-5, config).unwrap()
    };
    if material {
        b = b.with_capsule_material_contact().unwrap();
    }
    let guides = if cloth {
        [(1., 2.), (6., 1.), (3., 6.), (8., 7.)].map(|(i, j)| {
            let x = 0.14 * (i + 2. / 3.);
            let y = 0.14 * (j + 1. / 3.);
            (DVec3::new(x, y, -0.05), DVec3::Z)
        })
    } else {
        [3., 18., 42., 68.].map(|i| (DVec3::new(0.14 * (i + 0.5), 0., 0.), DVec3::Y))
    };
    let g = guides
        .into_iter()
        .map(|(c, a)| Boundary::Capsule {
            center: c.to_array(),
            axis: a.to_array(),
            radius: 0.014,
            half_length: 0.05,
            velocity: [0.; 3],
            angular_speed: 0.,
        })
        .collect();
    (b, g)
}
fn main() {
    println!("body,state,mode,bodies_per_sample,samples,median_us,p95_us,active_guides");
    for cloth in [false, true] {
        for landing in [false, true] {
            for material in [false, true] {
                let (base, g) = case(cloth, landing, material);
                let mut samples = vec![];
                let mut last = 0;
                for rep in 0..33 {
                    let mut bodies = vec![base.clone(); 12];
                    let timer = Instant::now();
                    for b in &mut bodies {
                        b.step(Duration::from_millis(2), &g).unwrap();
                    }
                    let us = timer.elapsed().as_secs_f64() * 1e6 / bodies.len() as f64;
                    let one = black_box(&bodies[0]);
                    last = one
                        .last_boundary_impulses()
                        .iter()
                        .filter(|r| r.linear != [0.; 3])
                        .count();
                    if rep >= 2 {
                        samples.push(us);
                    }
                }
                assert_eq!(last, if landing && material { 4 } else { 0 });
                samples.sort_by(f64::total_cmp);
                println!(
                    "{},{},{},12,31,{:.4},{:.4},{last}",
                    if cloth {
                        "11x11-cloth"
                    } else {
                        "81-node-cable"
                    },
                    if landing { "landing" } else { "clear" },
                    if material { "material" } else { "particles" },
                    samples[15],
                    samples[29]
                );
            }
        }
    }
}
