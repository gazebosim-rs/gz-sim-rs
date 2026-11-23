// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
/// Capped lateral center-of-pressure control, evaluated from measured contact loads.
pub fn roll_torques(
    com: glam::DVec3,
    velocity: glam::DVec3,
    reference_y: f64,
    ankle_y: [f64; 2],
    support_n: [f64; 2],
    mass: f64,
    gravity: f64,
) -> [Option<f64>; 2] {
    let load = support_n.iter().sum::<f64>();
    if load < 0.1 * mass * gravity {
        return [None; 2];
    }
    let center = (ankle_y[0] * support_n[0] + ankle_y[1] * support_n[1]) / load;
    // Inverted-pendulum planar acceleration: (y - center_of_pressure) * g / height.
    let acceleration = 15. * (reference_y - com.y) - 7. * velocity.y;
    let cop = com.y - com.z / gravity * acceleration;
    let shift = (cop - center).clamp(-0.049, 0.049);
    support_n.map(|f| (f > 3.).then_some(-mass * gravity * (f / load) * shift))
}
