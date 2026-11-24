// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Single-threaded matched-state cost of a structural formulation.
use glam::DVec3;
use gz_physics_xpbd::{Particle, SoftBody, SolverConfig};
use std::{
    error::Error,
    hint::black_box,
    time::{Duration, Instant},
};
fn specimen(columns: usize, compliance: f64) -> SoftBody {
    let config = SolverConfig {
        gravity: [0.; 3],
        substeps: 1,
        iterations: 12,
        self_contact: false,
        friction: 0.,
        ..Default::default()
    };
    if columns == 0 {
        let ps = [-0.35, 0.35].map(|y| Particle {
            position: [0., y, 0.8],
            velocity: [0., 0., 6. * y],
            inverse_mass: 20.,
        });
        return SoftBody::cable(ps.to_vec(), 0., 0., config).unwrap();
    }
    let mut ps = Vec::new();
    for r in 0..columns {
        for c in 0..columns {
            let y = 0.04 * (c as f64 - 0.5 * (columns - 1) as f64);
            let z = 0.04 * (r as f64 - 0.5 * (columns - 1) as f64);
            ps.push(Particle {
                position: [0., y, 0.8 + z],
                velocity: [
                    if compliance > 0. {
                        0.2 * (r as f64 - c as f64).sin()
                    } else {
                        0.
                    },
                    -6. * z,
                    6. * y,
                ],
                inverse_mass: 100.,
            });
        }
    }
    SoftBody::cloth(ps, columns, compliance, compliance, compliance, config).unwrap()
}
fn main() -> Result<(), Box<dyn Error>> {
    let dt = Duration::from_millis(2);
    println!("scene,mode,samples,p50_us,p95_us,particle_ticks_per_second");
    for (name, cols, compliance) in [
        ("cable", 0, 0.),
        ("hard3x3", 3, 0.),
        ("hard11x11", 11, 0.),
        ("compliant11x11", 11, 0.0002),
    ] {
        let initial = specimen(cols, compliance);
        let reference = initial.clone().with_angular_momentum_conservation();
        let mut times = [Vec::new(), Vec::new()];
        for rep in 0..13 {
            let order = if rep % 2 == 0 { [0, 1] } else { [1, 0] };
            for mode in order {
                let mut body = if mode == 0 {
                    initial.clone()
                } else {
                    reference.clone()
                };
                let start = Instant::now();
                for _ in 0..24 {
                    body.step(dt, &[])?;
                }
                let duration = start.elapsed().as_secs_f64() * 1e6 / 24.;
                black_box(DVec3::from(body.particles()[0].position));
                if rep > 1 {
                    times[mode].push(duration);
                }
            }
        }
        for (mode, ticks) in times.iter_mut().enumerate() {
            ticks.sort_by(f64::total_cmp);
            let n = ticks.len();
            let median = ticks[n / 2];
            let p95 = ticks[(n * 95).div_ceil(100) - 1];
            println!(
                "{name},{},{n},{median:.5},{p95:.5},{:.0}",
                if mode == 0 { "ordinary" } else { "conserving" },
                initial.particles().len() as f64 * 1e6 / median
            );
        }
    }
    Ok(())
}
