// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resume dynamics when a prescribed articulation loses a loaded support or its parked support starts moving.
use crate::articulation::{self, ArticulationTier, FidelityError};
use bevy_ecs::{prelude::*, schedule::ScheduleError};
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
pub use gz_physics_multifidelity::SupportLossReason;
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo, entity_by_id};
use std::{collections::BTreeMap, time::Duration};

/// Runs after scene inputs, before predictive contact promotion and the native solve.
/// Order a PhysicsPrepare observer after this set to see the tier used by the solver.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BeforeSolve;

/// A promotion on an advancing step. The model uses its public ECS ID. The link
/// and support use their native body IDs so a retired support remains identifiable.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Promotion {
    pub model: u64,
    pub link: BodyId,
    pub support: BodyId,
    pub reason: SupportLossReason,
    pub epoch: u64,
    pub iteration: u64,
    pub start_time: Duration,
    pub sim_time: Duration,
}

struct Policy {
    root: BodyId,
    enabled: bool,
    last: Option<Promotion>,
}

#[derive(Resource, Default)]
struct Policies(BTreeMap<u64, Policy>);

pub(super) fn bind(sim: &mut Simulation, model: u64, root: BodyId) {
    if !sim.world().contains_resource::<Policies>() {
        sim.add_systems(
            Phase::PhysicsPrepare,
            evaluate.in_set(BeforeSolve).after(crate::inputs::apply),
        );
        sim.world_mut().init_resource::<Policies>();
    }
    sim.world_mut()
        .resource_mut::<Policies>()
        .0
        .entry(model)
        .or_insert(Policy {
            root,
            enabled: true,
            last: None,
        });
}

/// Enabled on load. A change takes effect on the next advancing step and
/// preserves the most recent promotion in the current epoch.
pub fn set_enabled(sim: &mut Simulation, model: u64, enabled: bool) -> Result<(), FidelityError> {
    sim.validate_steps(0)?;
    articulation::binding(sim.world(), model)?;
    let mut policies = sim
        .world_mut()
        .get_resource_mut::<Policies>()
        .ok_or(FidelityError::InvalidBinding)?;
    policies
        .0
        .get_mut(&model)
        .ok_or(FidelityError::InvalidBinding)?
        .enabled = enabled;
    Ok(())
}

/// The most recent committed promotion of this model in the current epoch.
pub fn last_promotion(sim: &Simulation, model: u64) -> Option<Promotion> {
    let event = sim
        .world()
        .get_resource::<Policies>()?
        .0
        .get(&model)?
        .last?;
    (!sim.is_faulted() && event.epoch == sim.epoch() && event.sim_time <= sim.elapsed())
        .then_some(event)
}

pub(super) fn retire_if_unused(sim: &mut Simulation) -> Result<(), ScheduleError> {
    if sim.world().contains_resource::<Policies>() {
        let empty = sim
            .world_mut()
            .resource_scope(|world, mut policies: Mut<Policies>| {
                policies
                    .0
                    .retain(|model, _| entity_by_id(world, *model).is_some());
                policies.0.is_empty()
            });
        if empty {
            sim.remove_systems_in_set(Phase::PhysicsPrepare, evaluate)?;
            sim.world_mut().remove_resource::<Policies>();
        }
    }
    Ok(())
}

fn native(world: &World) -> Option<&MultiFidelityBackend> {
    world.get_resource::<PhysicsBackend>()?.downcast_ref()
}

fn evaluate(world: &mut World) {
    let time = *world.resource::<UpdateInfo>();
    if time.dt.is_zero()
        || !native(world).is_some_and(MultiFidelityBackend::has_pending_support_loss)
    {
        return;
    }
    world.resource_scope(|world, mut policies: Mut<Policies>| {
        for (&model, policy) in &mut policies.0 {
            if !policy.enabled {
                continue;
            }
            if let Some(loss) = native(world).and_then(|b| b.pending_support_loss(policy.root)) {
                if let Err(err) =
                    articulation::switch(world, model, ArticulationTier::Dynamic, time.dt)
                {
                    world.resource::<StepFault>().report(err);
                    break;
                }
                policy.last = Some(Promotion {
                    model,
                    link: loss.link,
                    support: loss.support,
                    reason: loss.reason,
                    epoch: time.epoch,
                    iteration: time.iteration,
                    start_time: time.start_time,
                    sim_time: time.sim_time,
                });
            }
        }
    });
}
