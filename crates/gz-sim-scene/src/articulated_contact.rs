// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One ordinary Bevy scene barrier can sample the full native effective
//! mobility of multiple points and apply simultaneous impulses on their links.
use crate::{BodyHandle, EntityKind, InputError, PhysicsBackend, initial};
use gz_physics::PhysicsError;
pub use gz_physics_multifidelity::{ArticulatedContactSample, ArticulatedImpulseReport};
use gz_physics_multifidelity::{
    ArticulatedContactSite, ArticulatedContactTransaction, MultiFidelityBackend,
};
use gz_sim_core::{Simulation, entity_by_id};

/// A stable loaded SDF link entity and material point, in the local link frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkContactSite {
    pub link: u64,
    pub local_point: [f64; 3],
}

pub(crate) fn loaded_link(
    world: &bevy_ecs::world::World,
    link: u64,
) -> Result<gz_physics::BodyId, InputError> {
    let e = entity_by_id(world, link).ok_or(InputError::MissingEntity(link))?;
    let body = world
        .get::<BodyHandle>(e)
        .ok_or(InputError::InvalidPhysics)?
        .id();
    if world.get::<EntityKind>(e) != Some(&EntityKind::Link)
        || !initial::owns_loaded_body(world, link, body)
    {
        return Err(InputError::InvalidPhysics);
    }
    Ok(body)
}

fn with_transaction<T>(
    sim: &mut Simulation,
    sites: &[LinkContactSite],
    f: impl FnOnce(ArticulatedContactTransaction<'_>) -> Result<T, PhysicsError>,
) -> Result<T, InputError> {
    sim.validate_steps(0)?;
    if sites.is_empty() || sites.len() > gz_physics_multifidelity::MAX_ARTICULATED_CONTACT_SITES {
        return Err(InputError::InvalidValue);
    }
    if !initial::matches_physics_owner(sim.world()) {
        return Err(InputError::InvalidPhysics);
    }
    let native = sites
        .iter()
        .map(|s| {
            Ok(ArticulatedContactSite {
                body: loaded_link(sim.world(), s.link)?,
                local_point: s.local_point,
            })
        })
        .collect::<Result<Vec<_>, InputError>>()?;
    let mut physics = sim
        .world_mut()
        .get_resource_mut::<PhysicsBackend>()
        .ok_or(InputError::InvalidPhysics)?;
    let backend = physics
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or(InputError::Unsupported(
            "articulated point mobility requires the multi-fidelity backend",
        ))?;
    Ok(f(backend.prepare_articulated_contacts(&native)?)?)
}

/// Read the current physical point velocities and all source/target response
/// blocks without mutating the native state, consuming commands or advancing time.
pub fn sample(
    sim: &mut Simulation,
    sites: &[LinkContactSite],
) -> Result<ArticulatedContactSample, InputError> {
    with_transaction(sim, sites, |t| Ok(t.sample().clone()))
}

/// Atomically apply the callback's world impulses to the sampled sites, in
/// N·s on the articulated links. The exclusive scene borrow holds the same
/// configuration and the native tree carries the resulting generalized
/// velocity into the next ordinary physics tick. Poses remain at this barrier.
pub fn apply(
    sim: &mut Simulation,
    sites: &[LinkContactSite],
    solve: impl FnOnce(&ArticulatedContactSample) -> Result<Vec<[f64; 3]>, PhysicsError>,
) -> Result<ArticulatedImpulseReport, InputError> {
    with_transaction(sim, sites, |t| {
        let impulses = solve(t.sample())?;
        t.commit(&impulses)
    })
}
