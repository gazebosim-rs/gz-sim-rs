// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Deterministic contact geometry and interval torques shared by the worker and mechanics checks.
const BASE: &str = include_str!("../../../../examples/distributed_sphere_pair_island.sdf");

pub(super) fn draw(seed: u64, stream: u64) -> u64 {
    let mut x = seed.wrapping_add(stream.wrapping_mul(0x9e3779b97f4a7c15));
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
    x ^ (x >> 31)
}

pub(super) fn geometry(seed: u64) -> String {
    // Use only the admitted shapes. Vary longitudinal separation, glancing
    // angle and incoming speed so each seed exercises a different contact graph.
    let ay = (draw(seed, 1) % 181) as f64 / 1000. - 0.090;
    let by = ay + (draw(seed, 2) % 51) as f64 / 1000. - 0.025;
    let bx = 1.09 + 0.32 + (draw(seed, 3) % 181) as f64 / 1000.;
    let av = -(0.65 + (draw(seed, 4) % 501) as f64 / 1000.);
    let bv = av - 0.50 - (draw(seed, 5) % 1001) as f64 / 1000.;
    BASE.replace("1.090 0.060 0.5", &format!("1.090 {ay:.6} 0.5"))
        .replace("1.53975 0.063 0.5", &format!("{bx:.6} {by:.6} 0.5"))
        .replace(">-1.0 0 0<", &format!(">{av:.6} 0 0<"))
        .replace(">-2.0 0 0<", &format!(">{bv:.6} 0 0<"))
}

pub(super) fn drive(seed: u64, tick: u64) -> f64 {
    ((draw(seed, 20 + tick / 32) % 65) as f64 - 32.) / 2048.
}

pub(super) fn extra(seed: u64, tick: u64) -> Option<f64> {
    (tick % 97 == draw(seed, 8) % 97).then_some(if tick.is_multiple_of(2) {
        3. / 1024.
    } else {
        -3. / 1024.
    })
}
