// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    ArticulationLinkSpec, FloatingTreeJointLimit, FloatingTreeJointSpec, FloatingTreeSpec,
    PayloadTreeSpec,
};

/// A locked polygonal cage makes simultaneous reactions measurable on many independent contacts.
pub(super) fn cage(
    n: usize,
    payload_first: bool,
    mass: f64,
    velocity: [f64; 3],
) -> PayloadTreeSpec {
    let sphere = |id, radius: f64, mass: f64, position, velocity| {
        let inertia = 0.4 * mass * radius * radius;
        ArticulationLinkSpec {
            id,
            radius,
            mass,
            position,
            orientation: [0., 0., 0., 1.],
            velocity,
            angular_velocity: [0.; 3],
            inertia_center: [0.; 3],
            inertia_orientation: [0., 0., 0., 1.],
            inertia_tensor: [inertia, inertia, inertia, 0., 0., 0.],
        }
    };
    let mut links: Vec<ArticulationLinkSpec> = Vec::new();
    let mut joints = Vec::new();
    let mut limits = Vec::new();
    for i in 0..n {
        let angle = std::f64::consts::TAU * (i as f64) / (n as f64);
        let position = [0.238 * angle.cos(), 0.238 * angle.sin(), 0.7];
        let id = 70 + i as u64;
        if let Some(prev) = links.last() {
            let joint_id = 400 + i as u64;
            joints.push(FloatingTreeJointSpec {
                joint_id,
                parent_id: prev.id,
                child_id: id,
                anchor: std::array::from_fn(|axis| 0.5 * (prev.position[axis] + position[axis])),
                axis: [0., 0., 1.],
            });
            limits.push(FloatingTreeJointLimit {
                joint_id,
                lower: -0.0,
                upper: 0.0,
            });
        }
        links.push(sphere(id, 0.06, 1.1, position, [0.; 3]));
    }
    PayloadTreeSpec {
        tree: FloatingTreeSpec {
            model_id: 32,
            links,
            joints,
        },
        limits,
        payload_can_sleep: true,
        exact_revolute_locks: false,
        payload: sphere(
            if payload_first { 4 } else { 900 },
            0.18,
            mass,
            [0., 0., 0.7],
            velocity,
        ),
    }
}
