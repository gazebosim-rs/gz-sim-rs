// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Kelvin–Voigt damping on the rate of the signed cloth dihedral angle.
use super::{MaterialTopology, SoftBody, XpbdError, bending};
use glam::DVec3;

impl SoftBody {
    /// Specify the viscous moment in N·m·s/rad for each cloth hinge.
    pub fn with_bending_damping(mut self, coefficient: f64) -> Result<Self, XpbdError> {
        if !matches!(self.topology, MaterialTopology::Cloth(_))
            || self.bending.is_empty()
            || !coefficient.is_finite()
            || coefficient < 0.
        {
            return Err(XpbdError::InvalidConfig);
        }
        self.bending_damping = coefficient;
        #[cfg(feature = "one-way-checkpoint")]
        self.one_way_identity.take();
        Ok(self)
    }

    /// Apply once per accepted native material substep. Each hinge uses its own
    /// current effective inertia, so its implicit impulse cannot reverse its angular rate.
    pub(super) fn damp_bending(&mut self, h: f64) -> Result<(), XpbdError> {
        let inverse_drag = (h * self.bending_damping).recip();
        let mut loads = self
            .surface
            .as_mut()
            .and_then(super::surface::Surface::constraint_loads);
        for hinge in &self.bending {
            let ids = hinge.vertices;
            let (_, gradient) = bending::geometry(ids.map(|id| self.scratch[id].position.into()))
                .ok_or(XpbdError::Numerical)?;
            let reference = DVec3::from(self.scratch[ids[0]].velocity);
            let mut inverse_inertia = 0.;
            let mut rate = 0.;
            for (id, g) in ids.into_iter().zip(gradient) {
                let p = self.scratch[id];
                inverse_inertia += p.inverse_mass * g.length_squared();
                rate += g.dot(DVec3::from(p.velocity) - reference);
            }
            if !rate.is_finite() || !inverse_inertia.is_finite() {
                return Err(XpbdError::Numerical);
            }
            if rate == 0. {
                continue;
            }
            let angular_impulse = -rate / (inverse_drag + inverse_inertia);
            for (id, g) in ids.into_iter().zip(gradient) {
                let impulse = angular_impulse * g;
                let p = &mut self.scratch[id];
                let velocity = DVec3::from(p.velocity) + p.inverse_mass * impulse;
                if !velocity.is_finite() {
                    return Err(XpbdError::Numerical);
                }
                p.velocity = velocity.to_array();
                if let Some(rows) = loads.as_deref_mut() {
                    rows[id].bend_damping += impulse;
                    if !rows[id].bend_damping.is_finite() {
                        return Err(XpbdError::Numerical);
                    }
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../tests/private/bending_damping.rs"]
mod tests;
