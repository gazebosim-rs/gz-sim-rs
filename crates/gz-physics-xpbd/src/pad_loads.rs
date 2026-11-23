// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{SoftBody, XpbdError};
use glam::DVec3;
use std::collections::BTreeMap;

/// Signed world impulses (N·s) on a compliant cuboid from one material particle.
/// Each vector sums the impulses actually applied across all iterations and
/// substeps of the last committed interval, including unloading corrections.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParticlePadImpulse {
    pub boundary: usize,
    pub particle: usize,
    pub normal: [f64; 3],
    pub tangent: [f64; 3],
}

#[derive(Default, Clone)]
pub(crate) struct PadLoads {
    pending: BTreeMap<(usize, usize), ParticlePadImpulse>,
    committed: Vec<ParticlePadImpulse>,
}

impl PadLoads {
    pub fn react_at(
        &mut self,
        boundary: usize,
        index: usize,
        sample: Option<crate::PadSample>,
        normal: DVec3,
        tangent: DVec3,
    ) -> Result<(), XpbdError> {
        if let Some(sample) = sample {
            for (node, w) in sample.particles.into_iter().zip(sample.weights) {
                self.react(boundary, node, w * normal, w * tangent)?;
            }
            Ok(())
        } else {
            self.react(boundary, index, normal, tangent)
        }
    }

    pub fn begin(&mut self) {
        self.pending.clear();
    }

    pub fn react(
        &mut self,
        boundary: usize,
        particle: usize,
        normal_on_particle: DVec3,
        tangent_on_particle: DVec3,
    ) -> Result<(), XpbdError> {
        let load = self
            .pending
            .entry((boundary, particle))
            .or_insert(ParticlePadImpulse {
                boundary,
                particle,
                normal: [0.; 3],
                tangent: [0.; 3],
            });
        let normal = DVec3::from(load.normal) - normal_on_particle;
        let tangent = DVec3::from(load.tangent) - tangent_on_particle;
        if !normal.is_finite() || !tangent.is_finite() {
            return Err(XpbdError::Numerical);
        }
        load.normal = normal.to_array();
        load.tangent = tangent.to_array();
        Ok(())
    }

    pub fn publish(&mut self) {
        self.committed.clear();
        self.committed.extend(
            self.pending
                .values()
                .copied()
                .filter(|v| v.normal != [0.; 3] || v.tangent != [0.; 3]),
        );
    }
}

impl SoftBody {
    /// Enable per-particle readings for cuboids with positive normal compliance.
    pub fn with_pad_loads(mut self) -> Self {
        self.contacts.pad_loads.get_or_insert_with(Default::default);
        self
    }

    /// Entries are ordered by boundary index, then construction particle index.
    /// An absent pair contributed zero net normal and tangential impulse.
    pub fn last_pad_loads(&self) -> &[ParticlePadImpulse] {
        if self.reactions.is_empty() {
            return &[];
        }
        self.contacts
            .pad_loads
            .as_ref()
            .map_or(&[], |v| v.committed.as_slice())
    }
}

#[cfg(test)]
#[path = "../tests/private/pad_loads.rs"]
mod tests;
