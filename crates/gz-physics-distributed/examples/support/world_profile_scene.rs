// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::physics::{Body, Primitive, Tier};

pub fn sphere(id: u64, position: [f64; 3], diameter: f64, linear: [f64; 3]) -> Body {
    Body {
        id,
        position,
        mass: 1.,
        tier: Tier::Dynamic,
        primitive: Primitive::Sphere,
        friction: 0.,
        size: [diameter; 3],
        orientation: [0., 0., 0., 1.],
        linear,
        angular: [0.; 3],
    }
}

pub fn stacks() -> Vec<Vec<Body>> {
    [
        (-6.5, 0., 1.8),
        (0., 6.5, 2.5),
        (6.5, 0., 3.2),
        (0., -6.5, 3.9),
    ]
    .into_iter()
    .enumerate()
    .map(|(i, (x, y, h))| {
        vec![
            sphere(2 * i as u64 + 1, [x, y, 0.455], 0.9, [0.; 3]),
            sphere(2 * i as u64 + 2, [x, y, h], 0.9, [0.; 3]),
        ]
    })
    .collect()
}
