// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Release-build cost probe. Timing is per contact query, averaged in blocks
//! to reduce clock-call overhead. No state is advanced between samples.
use glam::{DQuat, DVec3};
use gz_physics::sole_contact::{SoleBody, SoleDeck, SoleMaterial, SoleState};
use std::{hint::black_box, time::Instant};

fn main() {
    let b = SoleBody {
        half_extents: DVec3::new(0.1, 0.06, 0.04),
        foot_offset: DVec3::new(0.02, 0., -0.05),
        mass: 2.,
        inertia: DVec3::splat(0.05),
    };
    let d = SoleDeck {
        center: DVec3::ZERO,
        orientation: DQuat::IDENTITY,
        velocity: DVec3::ZERO,
        half_extents: DVec3::new(0.4, 0.3, 0.05),
    };
    let m = SoleMaterial::new(2e8, 2e8, 0.8, 0.5, 0.005).unwrap();
    let s = SoleState {
        center: DVec3::new(0.01, 0.01, 0.1398),
        orientation: DQuat::IDENTITY,
        velocity: DVec3::new(0.2, 0.1, -0.1),
        spin: DVec3::ZERO,
    };
    let rim = SoleState {
        center: DVec3::new(0.40, 0.30, 0.1398),
        orientation: DQuat::from_rotation_y(0.02) * DQuat::from_rotation_x(-0.01),
        ..s
    };
    for (name, state) in [
        ("whole flat sole", s),
        ("inclined at corner", rim),
        (
            "twisting at corner",
            SoleState {
                spin: DVec3::new(20., -15., 2.),
                ..rim
            },
        ),
    ] {
        for depth in [0, 1, 2] {
            let query = || black_box(b.contact(black_box(&state), &d, &m, depth).unwrap());
            for _ in 0..100 {
                query();
            }
            let mut samples = Vec::with_capacity(101);
            for _ in 0..101 {
                let start = Instant::now();
                for _ in 0..32 {
                    query();
                }
                samples.push(start.elapsed().as_secs_f64() * 1e9 / 32.);
            }
            samples.sort_by(f64::total_cmp);
            println!(
                "{name:20} depth {depth} median {:.0} ns, p95 {:.0} ns",
                samples[50], samples[95]
            );
        }
    }
}
