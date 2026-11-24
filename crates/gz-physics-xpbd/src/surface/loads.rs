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

/// Signed normal and friction impulses applied by shell self-contact to one
/// material node, including the surrounding triangle's torque contribution.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SelfContactParticleImpulse {
    pub particle: usize,
    /// World-space impulse in N·s.
    pub normal: [f64; 3],
    /// World-space impulse in N·s.
    pub tangent: [f64; 3],
}

/// World-space impulses (N·s) from distance and dihedral projections on one shell node.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ShellConstraintImpulse {
    pub distance: DVec3,
    pub bending: DVec3,
    /// Viscous dihedral impulse in N·s.
    pub bend_damping: DVec3,
}

#[derive(Clone, Default)]
pub(super) struct Loads {
    pending: BTreeMap<([usize; 4], usize), SelfContactLoad>,
    committed: Vec<SelfContactLoad>,
    nodes: BTreeMap<usize, SelfContactParticleImpulse>,
    committed_nodes: Vec<SelfContactParticleImpulse>,
    constraints: Vec<ShellConstraintImpulse>,
    committed_constraints: Vec<ShellConstraintImpulse>,
}
pub(super) enum NodeImpulse {
    Normal(DVec3),
    Tangent(DVec3),
}
impl Loads {
    pub(super) fn node(&mut self, id: usize, applied: NodeImpulse) -> Result<(), XpbdError> {
        let impulse = match applied {
            NodeImpulse::Normal(v) | NodeImpulse::Tangent(v) => v,
        };
        if impulse == DVec3::ZERO {
            return Ok(());
        }
        let node = self.nodes.entry(id).or_insert(SelfContactParticleImpulse {
            particle: id,
            ..Default::default()
        });
        let vector = match applied {
            NodeImpulse::Normal(_) => &mut node.normal,
            NodeImpulse::Tangent(_) => &mut node.tangent,
        };
        let total = DVec3::from(*vector) + impulse;
        if !total.is_finite() {
            return Err(XpbdError::Numerical);
        }
        *vector = total.to_array();
        Ok(())
    }
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
    pub(crate) fn constraint_loads(&mut self) -> Option<&mut [ShellConstraintImpulse]> {
        self.loads.as_mut().map(|l| l.constraints.as_mut_slice())
    }
    pub(crate) fn begin_loads(&mut self) {
        if let Some(loads) = &mut self.loads {
            loads.pending.clear();
            loads.nodes.clear();
            loads.constraints.fill(Default::default());
        }
    }
    pub(crate) fn publish_loads(&mut self) {
        if let Some(loads) = &mut self.loads {
            loads.committed.clear();
            loads.committed.extend(loads.pending.values().copied());
            loads.committed_constraints.clone_from(&loads.constraints);
            loads.committed_nodes.clear();
            loads.committed_nodes.extend(loads.nodes.values().copied());
        }
    }
}
impl SoftBody {
    /// Record geometric contact loads, delivered nodal impulses and structural
    /// constraint impulses for balancing forces across the loaded shell.
    pub fn with_self_contact_loads(mut self) -> Result<Self, XpbdError> {
        let surface = self
            .surface
            .as_mut()
            .filter(|s| s.friction.is_some())
            .ok_or(XpbdError::InvalidConfig)?;
        let loads = surface.loads.get_or_insert_with(Default::default);
        loads
            .constraints
            .resize(self.particles.len(), ShellConstraintImpulse::default());
        Ok(self)
    }

    /// Nodal impulses from accepted intervals, including reactions on prescribed nodes.
    /// Uses the same opt-in as the geometric self-contact load observations.
    pub fn last_self_contact_particle_impulses(&self) -> &[SelfContactParticleImpulse] {
        self.surface
            .as_ref()
            .and_then(|s| s.loads.as_ref())
            .map_or(&[], |l| l.committed_nodes.as_slice())
    }

    /// Distance and hinge projection impulses in construction-node order.
    /// An empty slice indicates that no load buffer has been published.
    pub fn last_shell_constraint_impulses(&self) -> &[ShellConstraintImpulse] {
        self.surface
            .as_ref()
            .and_then(|s| s.loads.as_ref())
            .map_or(&[], |l| &l.committed_constraints)
    }

    pub fn last_self_contact_loads(&self) -> &[SelfContactLoad] {
        self.surface
            .as_ref()
            .and_then(|s| s.loads.as_ref())
            .map_or(&[], |l| l.committed.as_slice())
    }
}
