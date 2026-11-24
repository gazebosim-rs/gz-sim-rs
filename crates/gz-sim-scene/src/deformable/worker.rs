// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Enroll unidirectional material and its original surface identities in a common process lease.
use super::*;
use gz_physics_distributed::{
    articulation::ArticulationError,
    trajectory::material::{MaterialRecipe, Surface as WorkerSurface},
};
use gz_physics_xpbd::OneWayState;

impl Deformables {
    pub(crate) fn managed_recipe(&self) -> Result<MaterialRecipe, ArticulationError> {
        if !self.plane_meters.is_empty() {
            return Err(ArticulationError::Invalid(
                "per-particle plane impulses require local material ownership",
            ));
        }
        if self.automatic_impact.is_some() || !self.impact_barriers.is_empty() {
            return Err(ArticulationError::Invalid(
                "curve material workers require one-way boundaries",
            ));
        }
        let shapes = self
            .surfaces
            .iter()
            .map(|s| {
                Ok(match *s {
                    Surface::Plane { point, normal } => WorkerSurface::Plane { point, normal },
                    Surface::Rigid {
                        body,
                        shape: ReceiverShape::Sphere(radius),
                        mobility: None,
                        ..
                    } => WorkerSurface::Sphere {
                        body: body.raw(),
                        radius,
                    },
                    Surface::Rigid {
                        body,
                        shape:
                            ReceiverShape::Capsule {
                                radius,
                                half_length,
                            },
                        mobility: None,
                        ..
                    } => WorkerSurface::Capsule {
                        body: body.raw(),
                        radius,
                        half_length,
                    },
                    _ => {
                        return Err(ArticulationError::Invalid(
                            "unsupported material worker surface",
                        ));
                    }
                })
            })
            .collect::<Result<_, ArticulationError>>()?;
        let mut seeds = Vec::new();
        for (name, body) in &self.initial {
            seeds.push((name.clone(), body.one_way_construction()?));
        }
        MaterialRecipe::new(shapes, seeds)
    }
    pub(crate) fn enroll(&mut self) {
        self.remote_material = true;
        self.active_slots = (0..self.surfaces.len()).collect();
    }
    pub(crate) fn managed_bodies(&mut self) -> Vec<&mut SoftBody> {
        self.current.values_mut().collect()
    }
    pub(crate) fn saved_material(&self) -> Result<Vec<(String, OneWayState)>, ArticulationError> {
        if !self.remote_material {
            return Err(ArticulationError::Invalid(
                "material has no managed continuation",
            ));
        }
        self.current
            .iter()
            .map(|(n, b)| Ok((n.clone(), b.one_way_state()?)))
            .collect()
    }
    pub(crate) fn restore_material(
        &mut self,
        rows: Vec<(String, OneWayState)>,
        time: Duration,
        epoch: u64,
    ) -> Result<(), ArticulationError> {
        if !self.remote_material || self.current.len() != rows.len() {
            return Err(ArticulationError::Invalid(
                "recovered material population differs",
            ));
        }
        for ((name, body), (n, state)) in self.current.iter().zip(&rows) {
            if name != n
                || state.reactions.len()
                    != if time.is_zero() {
                        0
                    } else {
                        self.surfaces.len()
                    }
            {
                return Err(ArticulationError::Invalid(
                    "recovered material identity or surface ordering differs",
                ));
            }
            body.validate_one_way_state(state)?;
        }
        for (body, (_, state)) in self.current.values_mut().zip(rows) {
            body.install_one_way_state(state)?;
        }
        self.publish(time, epoch);
        Ok(())
    }
}
pub(crate) fn sync_reopened(sim: &mut Simulation) {
    if sim
        .world()
        .get_resource::<Deformables>()
        .is_some_and(|s| !s.visuals.is_empty())
    {
        let mut p = Schedule::default();
        p.add_systems(particle_visual::sync);
        p.run(sim.world_mut());
    }
}
