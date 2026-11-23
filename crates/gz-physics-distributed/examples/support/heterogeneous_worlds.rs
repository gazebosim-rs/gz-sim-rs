// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Loaded worlds separated by 13 m, with public IDs chosen to vary canonical solver order.
use clap::ValueEnum;
use gz_physics_distributed::physics::{Body, Primitive, Tier};
use serde::Serialize;

pub const GRAVITY: [f64; 3] = [0., 0., -9.81];
pub const WARMUP: usize = 110;
pub const TICKS: usize = 200;

#[derive(Debug, Clone, Copy, ValueEnum, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Layout {
    First,
    Middle,
    Last,
    Twins,
    Uniform,
}

impl Layout {
    fn heavy_index(self) -> Option<usize> {
        match self {
            Self::First | Self::Twins => Some(0),
            Self::Middle => Some(4),
            Self::Last => Some(7),
            Self::Uniform => None,
        }
    }

    pub fn is_heavy(self, group: usize) -> bool {
        self.heavy_index() == Some(group) || (matches!(self, Self::Twins) && group == 1)
    }
}

pub fn seeds(layout: Layout) -> Vec<Vec<Body>> {
    let mut locations: Vec<_> = (0..8).collect();
    if let Some(at) = layout.heavy_index() {
        locations.swap(0, at);
    }
    // Two large slabs on adjacent physical sites would share a conservative
    // reach envelope. Keep their IDs adjacent and put the slabs diagonally apart.
    if matches!(layout, Layout::Twins) {
        locations.swap(1, 5);
    }
    let mut id = 0;
    let mut body = |position, size, primitive, mass| {
        id += 1;
        Body {
            id,
            position,
            size,
            primitive,
            mass,
            tier: Tier::Dynamic,
            friction: 0.,
            linear: [0.; 3],
            angular: [0.; 3],
            orientation: [0., 0., 0., 1.],
        }
    };
    locations
        .into_iter()
        .enumerate()
        .map(|(group, spot)| {
            let x = (spot % 4) as f64 * 13.0 - 19.5;
            let y = (spot / 4) as f64 * 13.0 - 6.5;
            if layout.is_heavy(group) {
                // A single floor-supported slab rigidly couples the normal loads of all cargo.
                let mut members = vec![body([x, y, 0.205], [4., 3., 0.4], Primitive::Box, 24.)];
                for iy in 0..4 {
                    for ix in 0..6 {
                        let pos = [
                            x + 0.55 * (ix as f64 - 2.5),
                            y + 0.55 * (iy as f64 - 1.5),
                            0.665,
                        ];
                        members.push(body(pos, [0.4; 3], Primitive::Sphere, 1.));
                    }
                }
                members
            } else {
                vec![
                    body([x, y, 0.305], [0.6; 3], Primitive::Sphere, 1.),
                    body([x, y, 0.97], [0.6; 3], Primitive::Sphere, 1.),
                ]
            }
        })
        .collect()
}
