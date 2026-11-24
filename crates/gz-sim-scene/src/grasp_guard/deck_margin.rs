// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use glam::{DQuat, DVec3};

/// Erosion of the top face of a horizontal finite cuboid used by a release controller.
#[derive(Clone, Copy, Debug)]
pub struct FiniteDeckMargin {
    half: DVec3,
    inset_m: f64,
    deck_from_world: DQuat,
}

impl FiniteDeckMargin {
    pub fn new(half_size_m: [f64; 3], inset_m: f64, world_rotation_xyzw: [f64; 4]) -> Option<Self> {
        let half = DVec3::from(half_size_m);
        let rotation = DQuat::from_array(world_rotation_xyzw);
        (rotation.is_finite()
            && (rotation.length_squared() - 1.).abs() < 1e-12
            && (rotation * DVec3::Z).distance(DVec3::Z) < 1e-12
            && half.is_finite()
            && half.min_element() > 0.
            && inset_m.is_finite()
            && inset_m > 0.
            && inset_m < half.x.min(half.y))
        .then_some(Self {
            half,
            inset_m,
            deck_from_world: rotation.conjugate(),
        })
    }

    /// The impulse on the payload and its moment about the collision center at
    /// the start of the interval, expressed in world axes (+Z upward). Displacement
    /// is the end center minus the start center in the same world axes. Require
    /// the resultant to intersect both endpoint footprints of the translating box.
    pub fn accepts(
        self,
        impulse_ns: [f64; 3],
        moment_nms: [f64; 3],
        displacement_m: [f64; 3],
    ) -> bool {
        let j = self.deck_from_world * DVec3::from(impulse_ns);
        let k = self.deck_from_world * DVec3::from(moment_nms);
        let d = self.deck_from_world * DVec3::from(displacement_m);
        if !j.is_finite() || !k.is_finite() || !d.is_finite() || j.z <= 0. {
            return false;
        }
        [k, k - d.cross(j)].into_iter().all(|moment| {
            // Evaluate the intersection at z=half.z in homogeneous coordinates,
            // avoiding division by a very small measured normal impulse.
            let x = self.half.z * j.x - moment.y;
            let y = self.half.z * j.y + moment.x;
            let x_bound = (self.half.x - self.inset_m) * j.z;
            let y_bound = (self.half.y - self.inset_m) * j.z;
            x_bound.is_finite() && y_bound.is_finite() && x.abs() <= x_bound && y.abs() <= y_bound
        })
    }
}

#[cfg(test)]
#[path = "../../tests/private/finite_deck_margin.rs"]
mod tests;
