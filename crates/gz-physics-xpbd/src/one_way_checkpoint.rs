// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A cold construction and committed publication for externally prescribed boundary owners.
use super::*;
use serde::{Deserialize, Serialize};
pub const MAX_ONE_WAY_PARTICLES: usize = 4096;
const MAX_ROWS: usize = 32768;
#[derive(Clone, Serialize, Deserialize)]
pub struct OneWayConstruction {
    particles: Vec<Particle>,
    constraints: Vec<DistanceConstraint>,
    bending: Vec<DihedralConstraint>,
    config: SolverConfig,
    topology: MaterialTopology,
    conserve_momentum: bool,
}
#[derive(Clone, Serialize, Deserialize, PartialEq, Debug)]
pub struct OneWayState {
    pub particles: Vec<([f64; 3], [f64; 3])>,
    pub reactions: Vec<BoundaryImpulse>,
    pub path_substeps: u32,
}
impl SoftBody {
    fn admits_one_way_cut(&self) -> bool {
        self.particles.len() <= MAX_ONE_WAY_PARTICLES
            && self.constraints.len() <= MAX_ROWS
            && self.bending.len() <= MAX_ROWS
            && self.self_contact.is_none()
            && self.surface.is_none()
            && self.pads.is_none()
            && self.capsule_material.is_none()
    }
    /// Digest only the immutable equations of motion. Public particle coordinates,
    /// velocities and reactions can change at every cut. A cached value is shared by
    /// sequential steps and is invalidated by a change of structural formulation.
    pub fn one_way_identity(&self) -> Result<[u8; 32], XpbdError> {
        use sha2::{Digest, Sha256};
        if !self.admits_one_way_cut() {
            return Err(XpbdError::MaterialCheckpoint);
        }
        Ok(*self.one_way_identity.get_or_init(|| {
            let mut h = Sha256::new();
            h.update(b"gz-one-way-material-law-01");
            h.update([u8::from(self.momentum_rows.is_some())]);
            let topology = match self.topology {
                MaterialTopology::Particles => 0,
                MaterialTopology::Cable => 1,
                MaterialTopology::Cloth(n) => 2 + n as u64,
            };
            h.update(topology.to_le_bytes());
            let cfg = self.config;
            h.update(cfg.iterations.to_le_bytes());
            h.update(cfg.substeps.to_le_bytes());
            for value in cfg
                .gravity
                .into_iter()
                .chain([cfg.friction, cfg.particle_radius])
            {
                h.update(value.to_le_bytes());
            }
            h.update((self.particles.len() as u64).to_le_bytes());
            for p in &self.particles {
                h.update(p.inverse_mass.to_le_bytes());
            }
            h.update((self.constraints.len() as u64).to_le_bytes());
            for (c, d) in self.constraints.iter().zip(&self.rest_directions) {
                h.update((c.a as u64).to_le_bytes());
                h.update((c.b as u64).to_le_bytes());
                for v in [c.compliance, c.rest_length]
                    .into_iter()
                    .chain(d.to_array())
                {
                    h.update(v.to_le_bytes());
                }
            }
            h.update((self.bending.len() as u64).to_le_bytes());
            for b in &self.bending {
                for i in b.vertices {
                    h.update((i as u64).to_le_bytes());
                }
                h.update(b.rest_angle.to_le_bytes());
                h.update(b.compliance.to_le_bytes());
            }
            h.finalize().into()
        }))
    }
    /// Capture reset particles and structural rest data from a seed with no published reactions.
    pub fn one_way_construction(&self) -> Result<OneWayConstruction, XpbdError> {
        if !self.admits_one_way_cut()
            || !self.reactions.is_empty()
            || self.sweep_refinements != 0
            || self.path_substeps != 0
        {
            return Err(XpbdError::MaterialCheckpoint);
        }
        // Rest directions are derived from the construction positions even for inextensible rows.
        let seed = OneWayConstruction {
            particles: self.particles.clone(),
            constraints: self.constraints.clone(),
            bending: self.bending.clone(),
            config: self.config,
            topology: self.topology,
            conserve_momentum: self.momentum_rows.is_some(),
        };
        let rebuilt = Self::from_one_way_construction(&seed)?;
        if rebuilt.rest_directions != self.rest_directions {
            return Err(XpbdError::MaterialCheckpoint);
        }
        Ok(seed)
    }
    pub fn from_one_way_construction(seed: &OneWayConstruction) -> Result<Self, XpbdError> {
        let n = seed.particles.len();
        if n > MAX_ONE_WAY_PARTICLES
            || seed.config.self_contact
            || seed.constraints.len() > MAX_ROWS
            || seed.bending.len() > MAX_ROWS
        {
            return Err(XpbdError::MaterialCheckpoint);
        }
        if let MaterialTopology::Cloth(width) = seed.topology
            && (width < 2 || !n.is_multiple_of(width) || n / width < 2)
        {
            return Err(XpbdError::MaterialCheckpoint);
        }
        let mut body = Self::new(
            seed.particles.clone(),
            seed.constraints.clone(),
            seed.config,
        )?;
        body.topology = seed.topology;
        for b in &seed.bending {
            if b.vertices.iter().any(|&i| i >= n)
                || b.vertices
                    .iter()
                    .enumerate()
                    .any(|(i, v)| b.vertices[..i].contains(v))
                || !b.rest_angle.is_finite()
                || !b.compliance.is_finite()
                || b.compliance < 0.
            {
                return Err(XpbdError::MaterialCheckpoint);
            }
        }
        body.bending.clone_from(&seed.bending);
        body.bending_lambdas.resize(body.bending.len(), 0.);
        if seed.conserve_momentum {
            body = body.with_angular_momentum_conservation();
        }
        Ok(body)
    }
    /// The predictor rebuilds gradients and clears every contact and structural multiplier on the
    /// next substep. Velocities, positions and public interval reactions are the continuation.
    pub fn one_way_state(&self) -> Result<OneWayState, XpbdError> {
        if !self.admits_one_way_cut()
            || self.sweep_refinements != 0
            || self.self_impulse != Default::default()
        {
            return Err(XpbdError::MaterialCheckpoint);
        }
        let state = OneWayState {
            particles: self
                .particles
                .iter()
                .map(|p| (p.position, p.velocity))
                .collect(),
            reactions: self.reactions.clone(),
            path_substeps: self.path_substeps,
        };
        self.validate_one_way_state(&state)?;
        Ok(state)
    }
    pub fn validate_one_way_state(&self, s: &OneWayState) -> Result<(), XpbdError> {
        if !self.admits_one_way_cut()
            || s.particles.len() != self.particles.len()
            || s.reactions.len() > 4096
            || s.path_substeps > 16384
            || s.particles
                .iter()
                .any(|(x, v)| !DVec3::from(*x).is_finite() || !DVec3::from(*v).is_finite())
            || s.reactions.iter().enumerate().any(|(i, r)| {
                r.boundary != i
                    || !DVec3::from(r.linear).is_finite()
                    || !DVec3::from(r.angular).is_finite()
            })
        {
            return Err(XpbdError::MaterialCheckpoint);
        }
        Ok(())
    }
    pub fn install_one_way_state(&mut self, s: OneWayState) -> Result<(), XpbdError> {
        self.validate_one_way_state(&s)?;
        for (p, (x, v)) in self.particles.iter_mut().zip(s.particles) {
            p.position = x;
            p.velocity = v;
        }
        // A one-way publication contains no articulated-pad impulses.
        if let Some(loads) = &mut self.contacts.pad_loads {
            loads.begin();
            loads.publish();
        }
        self.reactions = s.reactions;
        self.path_substeps = s.path_substeps;
        Ok(())
    }
}
