// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reproducible free-space trains with independently seeded dimensions and loads.
use crate::seeded::Rng;
use gz_physics_distributed::physics::{Body, BodyWrench, fixture};

pub(crate) fn train_fixture(seed: u64) -> (Vec<Body>, [BodyWrench; 2]) {
    let mut rng = Rng(seed);
    let yaw = rng.number(-0.38, 0.38);
    let tilt = rng.number(-0.21, 0.21);
    // Rz(yaw) Ry(tilt), quaternion xyzw.
    let (sy, cy) = (yaw * 0.5).sin_cos();
    let (sp, cp) = (tilt * 0.5).sin_cos();
    let q = [-sy * sp, cy * sp, sy * cp, cy * cp];
    let rotate = |x: f64, y: f64, z: f64| {
        let (s, c) = tilt.sin_cos();
        let (v, w) = yaw.sin_cos();
        let u = c * x + s * z;
        [w * u - v * y, v * u + w * y, -s * x + c * z]
    };
    let gap = rng.number(0.24, 0.40);
    let center = rng.number(7.6, 8.9);
    let speed = rng.number(1.1, 1.9);
    let mut bodies = Vec::new();
    for i in 0..4 {
        let sign = if i < 2 { -1. } else { 1. };
        let outer = i == 0 || i == 3;
        let x = sign * (center + if outer { 0.32 + gap } else { -0.32 });
        let shape = [
            rng.number(0.47, 0.62),
            rng.number(0.32, 0.55),
            rng.number(0.35, 0.60),
        ];
        bodies.push(Body {
            id: (i + 1) as u64,
            mass: rng.number(0.8, 1.9),
            size: shape,
            position: rotate(x, rng.number(-0.012, 0.012), rng.number(-0.015, 0.015)),
            linear: rotate(-sign * speed, 0., 0.),
            angular: [0.; 3],
            orientation: q,
            ..fixture()[0].promoted()
        });
    }
    let force = rng.number(3.5, 6.0);
    let inputs = [(1, force), (4, -force)].map(|(id, f)| BodyWrench {
        id,
        force: rotate(f, 0., 0.),
        torque: [0.; 3],
    });
    (bodies, inputs)
}
