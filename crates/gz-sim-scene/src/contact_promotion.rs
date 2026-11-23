// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Opt-in promotion of a prescribed articulation before its next rigid contact solve.
use crate::{
    articulation::{self, ArticulationTier, FidelityError},
    inputs,
};
use bevy_ecs::{prelude::*, schedule::ScheduleError};
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo, entity_by_id};
use std::{collections::BTreeMap, time::Duration};

#[derive(Resource, Default)]
pub(super) struct Policies(pub(super) BTreeMap<u64, Policy>);

pub(super) struct Policy {
    targets: Vec<u64>,
    last: Option<ContactPromotion>,
}

impl From<Vec<u64>> for Policy {
    fn from(targets: Vec<u64>) -> Self {
        Self {
            targets,
            last: None,
        }
    }
}

/// A completed automatic tier change. Times bracket its native solver step.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ContactPromotion {
    pub model: u64,
    /// Lowest public link ID predicting contact on this step.
    pub obstacle: u64,
    pub epoch: u64,
    pub iteration: u64,
    pub start_time: Duration,
    pub sim_time: Duration,
}

/// Most recent committed promotion under the model's current policy in this epoch.
/// Replacing the policy or removing its triggering link clears the observation. Paused ticks preserve it.
pub fn last_promotion(sim: &Simulation, model: u64) -> Option<ContactPromotion> {
    let event = sim
        .world()
        .get_resource::<Policies>()?
        .0
        .get(&model)?
        .last?;
    (!sim.is_faulted() && event.epoch == sim.epoch() && event.sim_time <= sim.elapsed())
        .then_some(event)
}

#[derive(Debug, thiserror::Error)]
pub enum PredictionError {
    #[error("expected a nonempty list of distinct public obstacle link IDs")]
    InvalidObstacles,
    #[error(transparent)]
    Model(#[from] FidelityError),
    #[error(transparent)]
    Schedule(#[from] ScheduleError),
}

/// Select free dynamic, prescribed or fixed obstacle links by public ID. A
/// prescribed model with stationary internal joints promotes on the first
/// advancing tick whose predicted sweep reaches a selected shape's contact
/// margin. The sweep includes rotation, gravity and forces staged for this tick.
/// A single loaded, fixed planar face can keep the discrete solver's ordinary
/// prediction margin. Edges, fresh loading and potential secondary strikes
/// can promote within the payload's one-step speed limit. List replacement is atomic.
pub fn configure(
    sim: &mut Simulation,
    model: u64,
    mut obstacles: Vec<u64>,
) -> Result<(), PredictionError> {
    obstacles.sort_unstable();
    if obstacles.is_empty() || obstacles.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(PredictionError::InvalidObstacles);
    }
    articulation::binding(sim.world(), model)?.first_contact(
        sim.world(),
        &obstacles,
        sim.step_duration(),
    )?;
    if !sim.world().contains_resource::<Policies>() {
        sim.add_systems(
            Phase::PhysicsPrepare,
            evaluate
                .after(inputs::apply)
                .after(crate::support_loss_promotion::BeforeSolve),
        );
        sim.world_mut().init_resource::<Policies>();
    }
    sim.world_mut()
        .resource_mut::<Policies>()
        .0
        .insert(model, obstacles.into());
    Ok(())
}

pub(super) fn enabled(world: &World, model: u64) -> bool {
    targets(world, model).is_some()
}

pub(super) fn targets(world: &World, model: u64) -> Option<&[u64]> {
    Some(&world.get_resource::<Policies>()?.0.get(&model)?.targets)
}

/// Remove this model's promotion policy. Its current tier is preserved.
pub fn clear(sim: &mut Simulation, model: u64) -> Result<(), PredictionError> {
    if let Some(mut policies) = sim.world_mut().get_resource_mut::<Policies>() {
        policies.0.remove(&model);
    }
    retire_if_unused(sim)?;
    Ok(())
}

pub(super) fn retire_if_unused(sim: &mut Simulation) -> Result<(), ScheduleError> {
    if !sim.world().contains_resource::<Policies>() {
        return Ok(());
    }
    let empty = sim
        .world_mut()
        .resource_scope(|world, mut policies: Mut<Policies>| {
            policies.0.retain(|model, policy| {
                policy
                    .targets
                    .retain(|id| entity_by_id(world, *id).is_some());
                if policy
                    .last
                    .is_some_and(|last| policy.targets.binary_search(&last.obstacle).is_err())
                {
                    policy.last = None;
                }
                !policy.targets.is_empty() && entity_by_id(world, *model).is_some()
            });
            policies.0.is_empty()
        });
    if empty
        && !sim
            .world()
            .contains_resource::<crate::contact_promotion_sdf::Authored>()
    {
        sim.remove_systems_in_set(Phase::PhysicsPrepare, evaluate)?;
        sim.world_mut().remove_resource::<Policies>();
    }
    Ok(())
}

fn evaluate(world: &mut World) {
    let time = *world.resource::<UpdateInfo>();
    let dt = time.dt;
    if dt.is_zero() {
        return;
    }
    world.resource_scope(|world, mut policies: Mut<Policies>| {
        for (&model, policy) in &mut policies.0 {
            let result = (|| -> Result<(), FidelityError> {
                let bound = articulation::binding(world, model)?;
                if bound.tier()? == ArticulationTier::Prescribed
                    && let Some(obstacle) = bound.first_contact(world, &policy.targets, dt)?
                {
                    articulation::switch(world, model, ArticulationTier::Dynamic, dt)?;
                    policy.last = Some(ContactPromotion {
                        model,
                        obstacle,
                        epoch: time.epoch,
                        iteration: time.iteration,
                        start_time: time.start_time,
                        sim_time: time.sim_time,
                    });
                }
                Ok(())
            })();
            if let Err(error) = result {
                world.resource::<StepFault>().report(error);
                break;
            }
        }
    });
}
