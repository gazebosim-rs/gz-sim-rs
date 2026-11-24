// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum MaterialForceError {
    #[error("the simulation is faulted")]
    Faulted,
    #[error("unknown deformable material")]
    UnknownMaterial,
    #[error("the force and resulting acceleration must be finite")]
    InvalidForce,
    #[error("the material must have finite positive movable mass")]
    InvalidMass,
    #[error("the material uses a dedicated impact or worker solver")]
    UnsupportedOwner,
}

/// Set a total world-frame force in newtons for the next advancing local material tick.
/// Movable particles share the force in proportion to their masses throughout each
/// substep. Pinned particles receive none. A newer command replaces the previous one,
/// zero cancels it, and resetting the world clears it. A paused tick preserves it.
pub fn set_material_force(
    sim: &mut Simulation,
    name: &str,
    force_n: [f64; 3],
) -> Result<(), MaterialForceError> {
    use MaterialForceError::*;
    if sim.is_faulted() {
        return Err(Faulted);
    }
    let mut soft = sim
        .world_mut()
        .get_resource_mut::<Deformables>()
        .ok_or(UnknownMaterial)?;
    let sample = soft.samples.get(name).ok_or(UnknownMaterial)?;
    #[cfg(feature = "distributed-physics")]
    if soft.remote_material {
        return Err(UnsupportedOwner);
    }
    if soft.automatic_impact.is_some() || !soft.impact_barriers.is_empty() {
        return Err(UnsupportedOwner);
    }
    let force = DVec3::from(force_n);
    if force == DVec3::ZERO {
        soft.pending_accelerations.remove(name);
        return Ok(());
    }
    let mass = sample.movable_mass_kg;
    if mass <= 0. || !mass.is_finite() {
        return Err(InvalidMass);
    }
    let acceleration = DVec3::from(soft.initial[name].config().gravity) + force / mass;
    if !acceleration.is_finite() {
        return Err(InvalidForce);
    }
    soft.pending_accelerations
        .insert(name.to_owned(), acceleration.to_array());
    Ok(())
}

impl Deformables {
    pub(super) fn advance_with_forces(
        &mut self,
        physics: &mut PhysicsBackend,
        dt: Duration,
    ) -> Result<(), CouplingError> {
        self.with_forces(|soft| soft.advance(physics, dt))
    }
    pub(super) fn with_forces<T>(
        &mut self,
        solve: impl FnOnce(&mut Self) -> Result<T, CouplingError>,
    ) -> Result<T, CouplingError> {
        for (name, acceleration) in &self.pending_accelerations {
            self.current
                .get_mut(name)
                .expect("registered material")
                .set_gravity(*acceleration)
                .expect("validated acceleration");
        }
        let step = solve(self);
        // Restore the authored field even if integration fails. Potential-energy queries,
        // future solver steps and checkpoints always see the ordinary gravitational field.
        for name in self.pending_accelerations.keys() {
            self.current
                .get_mut(name)
                .expect("registered material")
                .set_gravity(self.initial[name].config().gravity)
                .expect("authored gravity");
        }
        self.pending_accelerations.clear();
        step
    }
}
