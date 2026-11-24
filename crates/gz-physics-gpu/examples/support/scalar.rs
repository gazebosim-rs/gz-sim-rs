// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Benchmark-only serial CPU implementation of the specialized vertical equation.
//! The independent three-dimensional Rapier/XPBD path validates both implementations.
use gz_physics_gpu::Lane;

fn side(z: f64) -> f64 {
    if z < 0.0 { -1.0 } else { 1.0 }
}

pub fn step(cells: &mut [Lane]) {
    const H: f64 = 0.001;
    const DV: f64 = super::reference::GRAVITY * H;
    for lane in cells {
        let anchor = lane.anchor_z + H * lane.anchor_v;
        let bob_v = lane.bob_v + DV;
        let predicted_bob = lane.bob_z + H * bob_v;
        let rigid_v = lane.rigid_v + DV;
        // Mean speed of the pinned Rapier predictor's four gravity microsteps.
        let predicted_rigid = lane.rigid_z + H * (lane.rigid_v + (5.0 / 8.0) * DV);
        let chord_v = (predicted_rigid - lane.rigid_z) / H;

        let wb = lane.bob_inverse_mass;
        let wr = lane.rigid_inverse_mass;
        let length = predicted_bob - anchor;
        let lambda = (lane.rest_length - length.abs()) / (wb + lane.compliance / (H * H));
        let dx = side(length) * (wb * lambda);
        let bob = predicted_bob + dx;
        let bob_v = bob_v + dx / H;

        let start_gap = lane.rigid_z - lane.bob_z;
        let end_gap = predicted_rigid - bob;
        // Outside starts retain the first face struck, even for swept crossings.
        // Inside starts use the end normal, including a coincident-end fallback.
        let normal = if start_gap.abs() >= lane.combined_radius || end_gap == 0.0 {
            side(start_gap)
        } else {
            side(end_gap)
        };
        let depth = lane.combined_radius - normal * end_gap;
        let inverse_mass = wb + wr;
        let position_impulse = normal * (depth.max(0.0) / inverse_mass / H);
        let bob_dz = (H * wb) * position_impulse;
        let rigid_dv = wr * position_impulse;
        let bob_v = bob_v - bob_dz / H;
        let relative_v = normal * ((chord_v + rigid_dv) - bob_v);
        let initial_overlap_v = (lane.combined_radius - start_gap.abs()).max(0.0) / H;
        let velocity_impulse = if depth > 0.0 {
            normal * ((-relative_v).max(-initial_overlap_v) / inverse_mass)
        } else {
            0.0
        };

        lane.anchor_z = anchor;
        lane.bob_z = bob - bob_dz;
        lane.bob_v = bob_v - wb * velocity_impulse;
        lane.rigid_z = predicted_rigid + H * rigid_dv;
        lane.rigid_v = rigid_v + (rigid_dv + wr * velocity_impulse);
        lane.last_receiver_impulse_z = position_impulse + velocity_impulse;
    }
}
