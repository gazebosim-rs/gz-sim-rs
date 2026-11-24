// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::reference::turn;
use gz_physics_gpu::{PlanarArmLane, PlanarToolLane};

pub fn reference_fixture() -> PlanarToolLane {
    PlanarToolLane {
        arm: PlanarArmLane {
            angle: [turn(0.48), turn(-0.62)],
            speed: [0.; 2],
            length: [0.30, 0.21],
            center: [0.15, 0.12],
            mass: [0.8, 0.4],
            inertia: [0.0062, 0.0034],
            tip_mass: 0.,
            gravity: 9.81,
            target: [turn(-0.36), turn(-0.50)],
            stiffness: [16., 5.],
            damping: [2.3, 0.41],
            effort_limit: [8., 2.8],
            effort: [0.; 2],
        },
        ground_z: 0.65 - 0.60,
        radius: 0.018,
        friction: 0.30,
        normal_impulse: 0.,
        tangent_impulse: 0.,
        first_contact_offset: -1.,
        first_normal_impulse: 0.,
        first_tangent_impulse: 0.,
        turns: [0.; 2],
        velocity_mode: [0.; 2],
        velocity_target: [0.; 2],
        stops: Default::default(),
        stop_impulse: [0.; 2],
    }
}
/// Drive targets are updated once per millisecond in the server's joint coordinates.
pub fn drive_velocity(command: [f64; 2], angles: [f64; 2]) -> [f64; 2] {
    let origin = [0.48, -0.62];
    let limits = [2.4, 3.2];
    std::array::from_fn(|i| {
        ((command[i] - (angles[i] - origin[i])) / 0.001).clamp(-limits[i], limits[i])
    })
}
