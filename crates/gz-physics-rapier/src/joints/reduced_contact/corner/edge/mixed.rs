// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Simultaneous cold edge impact with identifiable generalized motion and
//! potentially indeterminate longitudinal site impulses.
use super::*;
use gz_physics::{EdgeImpulseSet, EdgeStickMode};

/// The native generalized jump and all physically identifiable site reactions.
/// World-space vectors use the simulation world frame. Contact-frame impulses
/// use (edge_direction, normal × edge_direction, normal), directed onto foot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MixedEdgeImpactReceipt {
    pub elapsed: Duration,
    pub foot: BodyId,
    pub support: BodyId,
    pub points: [[f64; 3]; 2],
    pub gaps: [f64; 2],
    pub normal: [f64; 3],
    pub edge_direction: [f64; 3],
    pub mode: EdgeStickMode,
    pub impulses: EdgeImpulseSet,
    pub friction: f64,
    pub before_relative_velocity: [[f64; 3]; 2],
    pub after_relative_velocity: [[f64; 3]; 2],
    pub kinetic_energy_change: f64,
    pub dissipated_energy: f64,
    pub uniqueness_bound: f64,
}
impl MixedEdgeImpactReceipt {
    /// Identifiable world resultant of the simultaneous impulse on the foot.
    pub fn total_impulse_on_foot(&self) -> [f64; 3] {
        let edge = Vector::from_array(self.edge_direction);
        let n = Vector::from_array(self.normal);
        let set = self.impulses;
        (edge * set.longitudinal_total
            + n.cross(edge) * (set.transverse[0] + set.transverse[1])
            + n * (set.normal[0] + set.normal[1]))
            .to_array()
    }
    /// One feasible assignment of world impulses. Each admissible assignment
    /// delivers the same generalized jump and both scalar normal loads.
    pub fn split_world(&self, first_longitudinal: f64) -> Option<[[f64; 3]; 2]> {
        let edge = Vector::from_array(self.edge_direction);
        let n = Vector::from_array(self.normal);
        self.impulses
            .split(first_longitudinal)
            .map(|pair| pair.map(|j| (edge * j[0] + n.cross(edge) * j[1] + n * j[2]).to_array()))
    }
}
