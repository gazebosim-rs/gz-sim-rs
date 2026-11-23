// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_distributed::multi_sphere::{ColdSelection, InspectedCut, PublishedCut};

/// Restore a cold operator selection after checking session and archive identity
/// under the original writer lease. No advancing interval is executed.
pub fn resume_selected_hinge_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    options: HingeWorkerOptions,
    selection: &ColdSelection,
) -> Result<Simulation, FencedHingeError> {
    let (spec, sim) = selected_recipe(xml, source, world, selection.cut())?;
    crate::require(
        options.reopen_attempts <= MultiSpherePool::MAX_REOPEN_ATTEMPTS,
        world,
        "worker reopen attempts exceed the multi-sphere limit",
    )?;
    let pool = MultiSpherePool::resume_scene_selected(
        worker,
        directory,
        options.width,
        spec,
        options.transport,
        selection,
    )?;
    Ok(install_convoy(
        xml,
        source,
        world,
        sim,
        pool,
        options.reopen_attempts,
    )?)
}

/// Project a selected checkpoint into the same ECS hierarchy as a worker
/// restart, including local visuals, without opening a worker transport.
pub fn preview_selected_hinge_world(
    xml: &str,
    source: &str,
    world: &str,
    inspected: &InspectedCut,
) -> Result<Simulation, LoadError> {
    let (spec, initial) = selected_recipe(xml, source, world, inspected)?;
    let cut = &inspected.cut;
    let clock = (cut.observation.hinge.tick, cut.epoch);
    let sim = at_session_clock(xml, source, world, initial, clock)?;
    let spheres = spec.spheres.map(|s| s.id);
    install(sim, spec.hinge, &spheres, Preview(*cut), clock)
}

fn selected_recipe(
    xml: &str,
    source: &str,
    world: &str,
    inspected: &InspectedCut,
) -> Result<(super::super::MultiSphereSpec, Simulation), LoadError> {
    let (recipe, sim) = spec_and_sim(xml, source, world)?;
    let HingeRecipe::MultiSphere(spec) = recipe else {
        return Err(LoadError::Unsupported {
            entity: world.into(),
            feature: "cold attachment requires a two-sphere convoy",
        });
    };
    crate::require(
        spec.same_recipe(&inspected.cut.spec)
            .map_err(physics_error)?,
        world,
        "cold checkpoint does not match the authored convoy recipe",
    )?;
    Ok((*spec, sim))
}

struct Preview(PublishedCut);
const READ_ONLY: ArticulationError = ArticulationError::Invalid("checkpoint preview is read only");

impl SessionBackend for Preview {
    fn hinge_sample(&self) -> &ArticulationObservation {
        &self.0.observation.hinge
    }
    fn sphere_sample_at(&self, index: usize) -> Option<&ArticulationLinkState> {
        self.0.observation.spheres.get(index)
    }
    fn pending_joint_effort(&self) -> Option<f64> {
        self.0.pending_effort
    }
    fn queue_effort(&mut self, _: u64, _: f64) -> Result<(), ArticulationError> {
        Err(READ_ONLY)
    }
    fn advance(&mut self) -> Result<(), ArticulationError> {
        Err(READ_ONLY)
    }
    fn reset(&mut self) -> Result<(), ArticulationError> {
        Err(READ_ONLY)
    }
    fn transfer(&mut self, _: u8) -> Result<(), ArticulationError> {
        Err(READ_ONLY)
    }
    fn replace_worker(&mut self, _: u8) -> Result<(), ArticulationError> {
        Err(READ_ONLY)
    }
}
