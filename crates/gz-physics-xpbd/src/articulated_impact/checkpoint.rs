// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Portable warm continuation for the isolated frictionless material profile.
use super::*;

const MAGIC: &[u8; 8] = b"GZMAT\0\0\x01";
pub const MAX_MATERIAL_CHECKPOINT_NODES: usize = 4096;

impl SoftBody {
    fn material_recipe(&self) -> Result<Vec<u8>, XpbdError> {
        if !self.supports_articulated_impact()
            || self.config.friction != 0.
            || self.particles.len() > MAX_MATERIAL_CHECKPOINT_NODES
            || self.capsule_material.is_some()
            || self.surface.is_some()
            || self.pads.is_some()
            || self.self_contact.is_some()
            || !self.reactions.is_empty()
            || self.sweep_refinements != 0
            || self.self_impulse != Default::default()
        {
            return Err(XpbdError::MaterialCheckpoint);
        }
        let mut out = MAGIC.to_vec();
        let mut word = |x: u64| out.extend_from_slice(&x.to_le_bytes());
        let columns = match self.topology {
            MaterialTopology::Cable => 0,
            MaterialTopology::Cloth(columns) => columns,
            MaterialTopology::Particles => return Err(XpbdError::MaterialCheckpoint),
        };
        for x in [
            columns as u64,
            self.particles.len() as u64,
            self.constraints.len() as u64,
            self.bending.len() as u64,
            u64::from(self.config.substeps),
            u64::from(self.config.iterations),
            u64::from(self.momentum_rows.is_some()),
        ] {
            word(x);
        }
        for x in self
            .config
            .gravity
            .into_iter()
            .chain([self.config.particle_radius, self.config.friction])
        {
            word(x.to_bits());
        }
        for p in &self.particles {
            word(p.inverse_mass.to_bits());
        }
        for (c, r) in self.constraints.iter().zip(&self.rest_directions) {
            word(c.a as u64);
            word(c.b as u64);
            word(c.rest_length.to_bits());
            word(c.compliance.to_bits());
            for x in r.to_array() {
                word(x.to_bits());
            }
        }
        for b in &self.bending {
            for i in b.vertices {
                word(i as u64);
            }
            word(b.rest_angle.to_bits());
            word(b.compliance.to_bits());
        }
        Ok(out)
    }

    /// Exact portable solver state at a committed isolated material barrier.
    /// Every future-used immutable input is included for exact recipe comparison.
    /// The owning session binds time, reset seeds, identity, receipt and integrity.
    pub fn checkpoint_articulated_material(&self) -> Result<Vec<u8>, XpbdError> {
        let mut bytes = self.material_recipe()?;
        bytes.reserve(48 * self.particles.len());
        for p in &self.particles {
            for x in p.position.into_iter().chain(p.velocity) {
                if !x.is_finite() {
                    return Err(XpbdError::MaterialCheckpoint);
                }
                bytes.extend_from_slice(&x.to_le_bytes());
            }
        }
        Ok(bytes)
    }

    /// Rehydrate into the same construction, validating all bytes before any write.
    /// Scratch coordinates are copied from committed particles on each step.
    /// The predictor rebuilds momentum gradients and clears structural multipliers.
    pub fn restore_articulated_material(&mut self, bytes: &[u8]) -> Result<(), XpbdError> {
        let recipe = self.material_recipe()?;
        let data = bytes
            .strip_prefix(recipe.as_slice())
            .ok_or(XpbdError::MaterialCheckpoint)?;
        if data.len() != 48 * self.particles.len() {
            return Err(XpbdError::MaterialCheckpoint);
        }
        let read = |b: &[u8]| f64::from_le_bytes(b.try_into().expect("eight-byte word"));
        if data.chunks_exact(8).any(|b| !read(b).is_finite()) {
            return Err(XpbdError::MaterialCheckpoint);
        }
        for (p, record) in self.particles.iter_mut().zip(data.chunks_exact(48)) {
            for (dest, b) in p
                .position
                .iter_mut()
                .chain(&mut p.velocity)
                .zip(record.chunks_exact(8))
            {
                *dest = read(b);
            }
        }
        Ok(())
    }
}
