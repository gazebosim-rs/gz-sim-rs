// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::physics::{Body, Primitive, Tier};
pub(super) fn seeds(square: bool, moving: bool, smooth: bool) -> [Body; 2] {
    let table = Body {
        id: 1,
        primitive: Primitive::Box,
        tier: Tier::Prescribed,
        size: [8., 3., 0.4],
        mass: 10.,
        friction: 0.65,
        position: [0., 0., -0.2],
        orientation: [0., 0., 0., 1.],
        linear: [moving as u8 as f64, 0., 0.],
        angular: [0.; 3],
    };
    let block = Body {
        id: 2,
        tier: Tier::Dynamic,
        size: if square {
            [0.6, 0.6, 0.2]
        } else {
            [0.8, 0.4, 0.2]
        },
        mass: 2.,
        friction: if smooth { 0. } else { 0.35 },
        position: [0., 0., 0.15],
        ..table.clone()
    };
    [table, block]
}
