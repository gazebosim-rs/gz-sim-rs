// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reproduce the native pose and broad-phase leaf when admitting a dormant
//! translating deck. A few ulps or a different broad-phase refit phase can change
//! a glancing speculative contact. Shared coordinates avoid replaying step history.
use super::*;
use rapier3d_f64::parry::bounding_volume::{Aabb, BoundingVolume};
use std::collections::HashMap;

#[derive(Default)]
pub(super) struct Registration {
    axes: [HashMap<u64, u32>; 3],
    leaves: HashMap<([u32; 3], [u64; 3]), u32>,
}
#[derive(Debug, Default, Clone, Copy)]
pub(super) struct Keys {
    axes: [u32; 3],
    leaf: u32,
}
struct Leaf {
    axes: [u32; 3],
    half: Vector,
    center: Vector,
    expanded: bool,
}
pub(super) struct Coordinates {
    axes: [Vec<(f64, f64)>; 3],
    leaves: Vec<Leaf>,
    prediction: f64,
    skin: f64,
    drift: f64,
}
impl Coordinates {
    pub(super) fn new(params: &IntegrationParameters) -> Self {
        Self {
            axes: Default::default(),
            leaves: Vec::new(),
            prediction: params.prediction_distance() / 2.,
            skin: 0.04 * params.length_unit,
            drift: 0.,
        }
    }
    // Both retained leaf bounds can extend by twice the skin from the
    // current AABBs before a refit. Each actual collider uses half prediction.
    pub(super) fn contact_margin(&self) -> f64 {
        2. * self.prediction + 4. * self.skin + self.drift
    }
    pub(super) fn register(
        &mut self,
        lookup: &mut Registration,
        position: Vector,
        half: Vector,
        velocity: Vector,
    ) -> Keys {
        let axes = std::array::from_fn(|axis| {
            let p = position[axis];
            *lookup.axes[axis].entry(p.to_bits()).or_insert_with(|| {
                let values = &mut self.axes[axis];
                let index = values.len() as u32;
                values.push((p, p));
                index
            })
        });
        // With a shared fixed velocity and no rotation, fixed axes can never
        // force a refit. Decks on different rows of a grid share the x phase.
        let moving = std::array::from_fn(|i| if velocity[i] == 0. { 0 } else { axes[i] });
        let extent = std::array::from_fn(|i| {
            if velocity[i] == 0. {
                0
            } else {
                half[i].to_bits()
            }
        });
        let leaf = *lookup.leaves.entry((moving, extent)).or_insert_with(|| {
            let id = self.leaves.len() as u32;
            self.leaves.push(Leaf {
                axes,
                half,
                center: position,
                expanded: false,
            });
            id
        });
        Keys { axes, leaf }
    }
    pub(super) fn position(&self, keys: Keys) -> Vector {
        Vector::from_array(std::array::from_fn(|i| {
            self.axes[i][keys.axes[i] as usize].1
        }))
    }
    pub(super) fn leaf(
        &self,
        keys: Keys,
        velocity: Vector,
        collider: &Collider,
        pose: &Pose,
    ) -> Aabb {
        let leaf = &self.leaves[keys.leaf as usize];
        let mut anchor = *pose;
        for i in 0..3 {
            if velocity[i] != 0. {
                anchor.translation[i] = leaf.center[i];
            }
        }
        let aabb = collider
            .shape()
            .compute_aabb(&anchor)
            .loosened(self.prediction);
        if leaf.expanded {
            aabb.loosened(self.skin)
        } else {
            aabb
        }
    }
    pub(super) fn advance(&mut self, velocity: Vector, dt: f64, elapsed: f64, steps: usize) {
        self.drift = 0.;
        for (axis, values) in self.axes.iter_mut().enumerate() {
            let v = velocity[axis];
            if v == 0. {
                continue;
            }
            let increment = v * (dt / steps as f64);
            let offset = v * elapsed;
            for (start, value) in values {
                for _ in 0..steps {
                    *value += increment;
                }
                self.drift = self.drift.max((*value - (*start + offset)).abs());
            }
        }
        for leaf in &mut self.leaves {
            let current = Vector::from_array(std::array::from_fn(|i| {
                self.axes[i][leaf.axes[i] as usize].1
            }));
            let bounds =
                |center| Aabb::from_half_extents(center, leaf.half).loosened(self.prediction);
            let old = bounds(leaf.center);
            // Parry inserts a raw leaf; on its first motion outside that raw AABB
            // it refits to the new AABB expanded by the change-detection skin.
            let old = if leaf.expanded {
                old.loosened(self.skin)
            } else {
                old
            };
            if !old.contains(&bounds(current)) {
                leaf.center = current;
                leaf.expanded = true;
            }
        }
    }
}
