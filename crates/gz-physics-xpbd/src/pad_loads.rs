// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{SoftBody, XpbdError};
use glam::DVec3;
use std::collections::BTreeMap;

/// Signed world impulses (N·s) on a prescribed boundary from one material particle.
/// Each vector sums the impulses actually applied across all iterations and
/// substeps of the last committed interval, including unloading corrections.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParticleBoundaryImpulse {
    pub boundary: usize,
    pub particle: usize,
    pub normal: [f64; 3],
    pub tangent: [f64; 3],
}

/// Per-particle impulse on a prescribed cuboid pad.
pub type ParticlePadImpulse = ParticleBoundaryImpulse;

#[derive(Default, Clone)]
pub(crate) struct PadLoads {
    pending: BTreeMap<(usize, usize), ParticlePadImpulse>,
    committed: Vec<ParticlePadImpulse>,
    incomplete_probe: bool,
    committed_incomplete_probe: bool,
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
        self.incomplete_probe = false;
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

    // A diagnostic overflow must never alter native physics or its commit.
    pub(crate) fn observe_plane(
        &mut self,
        boundary: usize,
        node: usize,
        normal: DVec3,
        tangent: DVec3,
    ) {
        self.incomplete_probe |= self.react(boundary, node, normal, tangent).is_err();
    }

    pub fn publish(&mut self) {
        self.committed_incomplete_probe = self.incomplete_probe;
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
    /// Enable per-particle impulse readings for prescribed rigid and compliant cuboid pads.
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

/// Observes the normal and tangential impulses on prescribed planes during one
/// ordinary material advance. The probe owns its readings and storage. It can
/// be attached to a different material or to a restored material at the next
/// step without changing the material's equations or checkpoints.
#[derive(Default)]
pub struct PlaneImpulseProbe {
    pub(crate) loads: Option<Box<PadLoads>>,
}

impl PlaneImpulseProbe {
    /// Last successful step, in boundary and particle order. Missing pairs had zero net impulse.
    pub fn readings(&self) -> Result<&[ParticleBoundaryImpulse], XpbdError> {
        match &self.loads {
            Some(p) if p.committed_incomplete_probe => Err(XpbdError::Numerical),
            Some(p) => Ok(&p.committed),
            None => Ok(&[]),
        }
    }

    /// Advance one material using the usual coupled solver, including rigid or
    /// damped pads, and accumulate actual plane impulses across accepted substeps.
    pub fn step(
        &mut self,
        material: &mut SoftBody,
        dt: std::time::Duration,
        boundaries: &[crate::Boundary],
        mobility: &[Option<crate::RigidMobility>],
        damping: &[(usize, f64)],
    ) -> Result<Vec<Option<crate::ReceiverMotion>>, XpbdError> {
        Self::step_group(
            &mut [material],
            &mut [Some(self)],
            dt,
            boundaries,
            mobility,
            damping,
        )
    }

    /// Advance the complete coupled group once. A probe slot corresponds to the
    /// same material slot, including explicitly unobserved materials.
    pub fn step_group(
        materials: &mut [&mut SoftBody],
        probes: &mut [Option<&mut Self>],
        dt: std::time::Duration,
        boundaries: &[crate::Boundary],
        mobility: &[Option<crate::RigidMobility>],
        damping: &[(usize, f64)],
    ) -> Result<Vec<Option<crate::ReceiverMotion>>, XpbdError> {
        if materials.len() != probes.len() {
            return Err(XpbdError::InvalidConfig);
        }
        for (material, probe) in materials.iter_mut().zip(probes.iter_mut()) {
            if let Some(probe) = probe {
                probe.attach(material);
            }
        }
        let result = crate::step_coupled_damped_pads(materials, dt, boundaries, mobility, damping);
        for (material, probe) in materials.iter_mut().zip(probes.iter_mut()) {
            if let Some(probe) = probe {
                probe.detach(material);
            }
        }
        result
    }

    /// Observe an ordinary one-way advance with its original curved boundary
    /// paths and subdivision tolerance.
    pub fn step_with_boundary_paths(
        &mut self,
        material: &mut SoftBody,
        dt: std::time::Duration,
        boundaries: &[crate::Boundary],
        paths: &[(usize, &dyn crate::BoundaryPath)],
        tolerance: f64,
    ) -> Result<(), XpbdError> {
        self.attach(material);
        let result = material.step_with_boundary_paths(dt, boundaries, paths, tolerance);
        self.detach(material);
        result
    }

    fn attach(&mut self, material: &mut SoftBody) {
        material.contacts.plane_loads = Some(self.loads.take().unwrap_or_default());
    }

    fn detach(&mut self, material: &mut SoftBody) {
        self.loads = material.contacts.plane_loads.take();
    }
}

#[cfg(test)]
#[path = "../tests/private/pad_loads.rs"]
mod tests;
