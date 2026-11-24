// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::physics::{Body, Primitive, fixture};

pub fn grid(count: usize, inward: bool) -> Vec<Body> {
    (0..count)
        .map(|i| {
            let x = (i % 8) as f64 * 0.44 - 1.54;
            let y = (i / 8) as f64 * 0.44 - 1.54;
            Body {
                id: 1 + i as u64,
                primitive: Primitive::Sphere,
                size: [0.4; 3],
                position: [x, y, 0.25],
                linear: if inward {
                    [-4. * x, -4. * y, -2.]
                } else {
                    [0.; 3]
                },
                ..fixture()[1].clone()
            }
        })
        .collect()
}
