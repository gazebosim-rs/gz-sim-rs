// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::{CarryGripperLane, ParallelGripperLane};
pub fn fixture(mass: f64, mu: f64, shelf_x: f64) -> CarryGripperLane {
    let hand = ParallelGripperLane {
        position: [0.03, -0.05, 0.05],
        speed: [0.; 3],
        mass: [0.38, 0.12, 0.12],
        half_size: [[0.006, 0.011]; 2],
        ball: [0., 0.022],
        velocity: [0.; 2],
        angle: 0.,
        spin: 0.,
        radius: 0.022,
        ball_mass: mass,
        gravity: 9.81,
        ground: 0.,
        target: [0.03, -0.022, 0.022],
        stiffness: [270., 1100., 1100.],
        damping: [23., 16., 16.],
        force_limit: [25., 9., 9.],
        friction: [mu, mu, 0.3],
        normal_impulse: [0.; 3],
        tangent_impulse: [0.; 3],
        support_impulse: [0.; 2],
        lower: [0.027, -0.07, 0.015],
        upper: [0.22, -0.015, 0.07],
    };
    CarryGripperLane {
        hand,
        track: 0.,
        track_speed: 0.,
        track_mass: 0.30,
        track_origin: 0.,
        track_target: 0.,
        track_stiffness: 175.,
        track_damping: 23.,
        track_effort: 16.,
        track_lower: -0.05,
        track_upper: 0.28,
        shelf_center: [shelf_x, 0.007],
        shelf_half: [0.044, 0.007],
        shelf_friction: 0.4,
        shelf_normal: 0.,
        shelf_tangent: 0.,
        shelf_up: 0.,
        com_offset: [0.; 2],
        angular_inertia: 0.4 * mass * hand.radius.powi(2),
    }
}
/// Each action depends on the last publicly committed receiving surface load.
pub fn targets(tick: usize, released: bool, wrong_floor: bool) -> [f64; 4] {
    let track = if (571..2001).contains(&tick) || tick >= 2001 && !released {
        0.18
    } else {
        0.
    };
    let lift = if (201..1201).contains(&tick) || tick >= 1751 && released {
        0.13
    } else if tick >= 1201 {
        if wrong_floor { 0.0275 } else { 0.044 }
    } else {
        0.03
    };
    let jaw = if released { 0.05 } else { 0.022 };
    [track, lift, -jaw, jaw]
}
