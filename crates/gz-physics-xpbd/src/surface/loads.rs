// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Hit, Surface};
use crate::{SoftBody, XpbdError};
use glam::DVec3;
use std::collections::BTreeMap;

/// Normal load at a pair of geometric witnesses over one accepted public step.
/// The first side occupies the first one (vertex) or two (edge) particle slots.
/// These barycentric shares locate the normal load on the skin. Torque transfer
/// through the surrounding triangles can additionally load neighboring nodes.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SelfContactLoad {
    pub particles: [usize; 4],
    pub first_side_len: usize,
    /// Compressive magnitude on either side, in N s.
    pub normal_ns: f64,
    /// Unsigned shares in N s. The shares on each side sum to normal_ns.
    pub particle_normal_ns: [f64; 4],
    /// Signed world-space impulse shares in N s, equal and opposite across sides.
    pub particle_impulse_ns: [[f64; 3]; 4],
}

#[derive(Clone, Default)]
pub(super) struct Loads {
    pending: BTreeMap<([usize; 4], usize), SelfContactLoad>,
    committed: Vec<SelfContactLoad>,
}
impl Surface {
    pub(super) fn record_load(&mut self, hit: &Hit, impulse: f64) {
        let Some(loads) = &mut self.loads else { return };
        if impulse == 0. {
            return;
        }
        let particles = hit.stencil.ids;
        let first_side_len = if hit.stencil.vf { 1 } else { 2 };
        let entry = loads
            .pending
            .entry((particles, first_side_len))
            .or_insert(SelfContactLoad {
                particles,
                first_side_len,
                ..Default::default()
            });
        entry.normal_ns += impulse;
        for (i, weight) in hit.weights.iter().copied().enumerate() {
            let signed = weight * impulse;
            entry.particle_normal_ns[i] += signed.abs();
            entry.particle_impulse_ns[i] =
                (DVec3::from(entry.particle_impulse_ns[i]) + signed * hit.normal).to_array();
        }
    }
    pub(crate) fn begin_loads(&mut self) {
        if let Some(loads) = &mut self.loads {
            loads.pending.clear();
        }
    }
    pub(crate) fn publish_loads(&mut self) {
        if let Some(loads) = &mut self.loads {
            loads.committed.clear();
            loads.committed.extend(loads.pending.values().copied());
        }
    }
}
impl SoftBody {
    /// Record where the swept material skins exchange normal impulse.
    pub fn with_self_contact_loads(mut self) -> Result<Self, XpbdError> {
        let surface = self
            .surface
            .as_mut()
            .filter(|s| s.friction.is_some())
            .ok_or(XpbdError::InvalidConfig)?;
        surface.loads.get_or_insert_with(Default::default);
        Ok(self)
    }

    pub fn last_self_contact_loads(&self) -> &[SelfContactLoad] {
        self.surface
            .as_ref()
            .and_then(|s| s.loads.as_ref())
            .map_or(&[], |l| l.committed.as_slice())
    }
}
