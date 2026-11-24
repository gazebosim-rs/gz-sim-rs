// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{SoftBody, XpbdError, bending};
use glam::DVec3;

/// Mechanical energy of the material in joules. Gravity uses the world origin as its datum.
/// Kinetic and gravitational energy include particles with finite mass. Distance and
/// dihedral terms include only springs with positive compliance. Boundary energy is external.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MaterialEnergy {
    pub kinetic: f64,
    pub gravitational: f64,
    pub distance: f64,
    pub bending: f64,
}

impl MaterialEnergy {
    pub fn total(&self) -> f64 {
        self.kinetic + self.gravitational + self.distance + self.bending
    }
}

/// Elastic restoring forces in newtons at a material node, in world coordinates.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MaterialElasticForce {
    /// Structural and shear springs at their committed extension.
    pub distance: DVec3,
    /// Dihedral springs at their committed deflection.
    pub bending: DVec3,
}

impl SoftBody {
    /// Negative gradient of the stored spring energies at the committed positions.
    /// Ideal constraints have no spring energy. A collapsed, pre-stretched bond
    /// or a dihedral at its angle branch cut has no unique restoring direction.
    pub fn elastic_node_forces(&self) -> Result<Vec<MaterialElasticForce>, XpbdError> {
        let mut forces = vec![MaterialElasticForce::default(); self.particles.len()];
        let position = |index: usize| DVec3::from(self.particles[index].position);
        for c in self.constraints.iter().filter(|c| c.compliance > 0.) {
            let delta = position(c.a) - position(c.b);
            let length = delta.length();
            if length == 0. {
                return Err(XpbdError::Numerical);
            }
            let force = (delta / length) * ((c.rest_length - length) / c.compliance);
            forces[c.a].distance += force;
            forces[c.b].distance -= force;
        }
        for c in self.bending.iter().filter(|c| c.compliance > 0.) {
            let (angle, gradients) =
                bending::geometry(c.vertices.map(position)).ok_or(XpbdError::Numerical)?;
            let deflection = bending::angle_difference(angle, c.rest_angle);
            if deflection.abs() == std::f64::consts::PI {
                return Err(XpbdError::Numerical);
            }
            for (id, gradient) in c.vertices.into_iter().zip(gradients) {
                forces[id].bending -= gradient * (deflection / c.compliance);
            }
        }
        if forces
            .iter()
            .any(|f| !f.distance.is_finite() || !f.bending.is_finite())
        {
            return Err(XpbdError::Numerical);
        }
        Ok(forces)
    }

    /// Evaluate the last committed particle configuration on demand, using the
    /// solver's own rest lengths, signed rest angles and constraint compliances.
    pub fn mechanical_energy(&self) -> Result<MaterialEnergy, XpbdError> {
        let mut energy = MaterialEnergy::default();
        let gravity = DVec3::from(self.config.gravity);
        for p in self.particles.iter().filter(|p| p.inverse_mass > 0.) {
            energy.kinetic += 0.5 * DVec3::from(p.velocity).length_squared() / p.inverse_mass;
            energy.gravitational -= gravity.dot(p.position.into()) / p.inverse_mass;
        }
        let position = |index: usize| DVec3::from(self.particles[index].position);
        for c in self.constraints.iter().filter(|c| c.compliance > 0.) {
            let extension = position(c.a).distance(position(c.b)) - c.rest_length;
            energy.distance += 0.5 * extension.powi(2) / c.compliance;
        }
        for c in self.bending.iter().filter(|c| c.compliance > 0.) {
            let (angle, _) =
                bending::geometry(c.vertices.map(position)).ok_or(XpbdError::Numerical)?;
            let deflection = bending::angle_difference(angle, c.rest_angle);
            energy.bending += 0.5 * deflection.powi(2) / c.compliance;
        }
        if !energy.total().is_finite() {
            return Err(XpbdError::Numerical);
        }
        Ok(energy)
    }
}

#[cfg(test)]
#[path = "../tests/private/material_energy.rs"]
mod tests;
