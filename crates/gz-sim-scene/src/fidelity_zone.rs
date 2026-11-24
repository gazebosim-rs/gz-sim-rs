// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Automatic assembly tier selection at advancing tick boundaries.
use crate::articulation::{self, FidelityError};
use bevy_ecs::{prelude::*, schedule::ScheduleError};
use gz_physics_multifidelity::ArticulationTier;
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo, entity_by_id};
use std::{collections::BTreeMap, time::Duration};

/// A spherical high-fidelity region in world metres. The two radii form
/// a deadband around its boundary.
#[derive(Clone, Copy, Debug)]
pub struct FidelityZone {
    pub center: [f64; 3],
    pub promote_radius: f64,
    pub demote_radius: f64,
}

impl FidelityZone {
    fn valid(self) -> bool {
        self.center.into_iter().all(f64::is_finite)
            && self.promote_radius.is_finite()
            && self.promote_radius >= 0.0
            && self.demote_radius.is_finite()
            && self.demote_radius > self.promote_radius
    }

    fn distance(self, point: [f64; 3]) -> f64 {
        let [x, y, z] = std::array::from_fn(|i| point[i] - self.center[i]);
        x.hypot(y).hypot(z)
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ZoneError {
    #[error("expected at least one finite zone with 0 <= promote radius < demote radius")]
    InvalidZone,
    #[error(transparent)]
    Model(#[from] FidelityError),
    #[error(transparent)]
    Schedule(#[from] ScheduleError),
}

struct Policy {
    zones: Vec<FidelityZone>,
    cooldown: Duration,
    outside_since: Option<Duration>,
    eligibility_dwell: Duration,
    eligible_since: Option<Duration>,
    epoch: u64,
}

impl Policy {
    fn target(&mut self, point: [f64; 3], time: UpdateInfo) -> Option<ArticulationTier> {
        if self.epoch != time.epoch {
            self.epoch = time.epoch;
            self.eligible_since = None;
            self.outside_since = None;
        }
        let mut outside = true;
        for zone in &self.zones {
            let distance = zone.distance(point);
            if distance <= zone.promote_radius {
                self.outside_since = None;
                return Some(ArticulationTier::Dynamic);
            }
            outside &= distance > zone.demote_radius;
        }
        if !outside {
            self.outside_since = None;
            return None;
        }
        let since = self.outside_since.get_or_insert(time.start_time);
        (time.start_time.saturating_sub(*since) >= self.cooldown)
            .then_some(ArticulationTier::Prescribed)
    }
}

#[derive(Resource, Default)]
struct Policies<const AFTER_INPUTS: bool>(BTreeMap<u64, Policy>);

/// Install or replace one model's policy. Evaluate the committed root position
/// at the beginning of advancing ticks. Re-entering an outer radius cancels
/// demotion dwell. Reset starts a fresh dwell interval.
pub fn configure(
    sim: &mut Simulation,
    model: u64,
    zones: Vec<FidelityZone>,
    cooldown: Duration,
) -> Result<(), ZoneError> {
    configure_with_eligibility_dwell(sim, model, zones, cooldown, Duration::ZERO)
}

/// Require the model to pass the ordinary demotion checks for an uninterrupted
/// interval of simulation time after the zone cooldown. A deferral, zone
/// re-entry, policy replacement or epoch change starts a fresh interval.
/// A positive dwell is evaluated after queued link inputs and before support-loss
/// promotion and the solve. A contact predicted by the configured obstacle policy
/// on the next solve restarts that interval. Zero uses the original lifecycle policy.
pub fn configure_with_eligibility_dwell(
    sim: &mut Simulation,
    model: u64,
    zones: Vec<FidelityZone>,
    cooldown: Duration,
    eligibility_dwell: Duration,
) -> Result<(), ZoneError> {
    if zones.is_empty() || zones.iter().any(|zone| !zone.valid()) {
        return Err(ZoneError::InvalidZone);
    }
    crate::model_fidelity(sim, model)?;
    let epoch = sim.epoch();
    let policy = Policy {
        zones,
        cooldown,
        outside_since: None,
        eligibility_dwell,
        eligible_since: None,
        epoch,
    };
    if eligibility_dwell.is_zero() {
        remove_policy::<true>(sim, model)?;
        if !sim.world().contains_resource::<Policies<false>>() {
            sim.add_systems(Phase::Lifecycle, evaluate::<false>);
            sim.world_mut().init_resource::<Policies<false>>();
        }
        sim.world_mut()
            .resource_mut::<Policies<false>>()
            .0
            .insert(model, policy);
    } else {
        remove_policy::<false>(sim, model)?;
        if !sim.world().contains_resource::<Policies<true>>() {
            // Admitting before queued inputs would allow a demotion and a support-loss
            // promotion at the very same boundary, perturbing a fully dynamic solve.
            sim.add_systems(
                Phase::PhysicsPrepare,
                evaluate::<true>
                    .after(crate::inputs::apply)
                    .before(crate::support_loss_promotion::BeforeSolve),
            );
            sim.world_mut().init_resource::<Policies<true>>();
        }
        sim.world_mut()
            .resource_mut::<Policies<true>>()
            .0
            .insert(model, policy);
    }
    Ok(())
}

/// Remove the policy without changing the current tier.
pub fn clear(sim: &mut Simulation, model: u64) -> Result<(), ZoneError> {
    remove_policy::<false>(sim, model)?;
    remove_policy::<true>(sim, model)?;
    Ok(())
}

fn remove_policy<const AFTER_INPUTS: bool>(
    sim: &mut Simulation,
    model: u64,
) -> Result<(), ScheduleError> {
    if let Some(mut policies) = sim.world_mut().get_resource_mut::<Policies<AFTER_INPUTS>>() {
        policies.0.remove(&model);
        retire_empty::<AFTER_INPUTS>(sim)?;
    }
    Ok(())
}

pub(super) fn retire_if_unused(sim: &mut Simulation) -> Result<(), ScheduleError> {
    retire_empty::<false>(sim)?;
    retire_empty::<true>(sim)
}

fn retire_empty<const AFTER_INPUTS: bool>(sim: &mut Simulation) -> Result<(), ScheduleError> {
    if !sim.world().contains_resource::<Policies<AFTER_INPUTS>>() {
        return Ok(());
    }
    let empty =
        sim.world_mut()
            .resource_scope(|world, mut policies: Mut<Policies<AFTER_INPUTS>>| {
                policies
                    .0
                    .retain(|id, _| entity_by_id(world, *id).is_some());
                policies.0.is_empty()
            });
    if empty {
        let phase = if AFTER_INPUTS {
            Phase::PhysicsPrepare
        } else {
            Phase::Lifecycle
        };
        sim.remove_systems_in_set(phase, evaluate::<AFTER_INPUTS>)?;
        sim.world_mut().remove_resource::<Policies<AFTER_INPUTS>>();
    }
    Ok(())
}

/// A cohort can share its live neighborhood check while every candidate is
/// outside its zone and can only remain dynamic or become stationary.
fn evaluate_parked<const AFTER_INPUTS: bool>(
    world: &mut World,
    policies: &mut Policies<AFTER_INPUTS>,
    time: UpdateInfo,
) -> bool {
    use gz_physics::PhysicsBackend;
    use gz_physics_multifidelity::MultiFidelityBackend;
    if !AFTER_INPUTS
        || policies.0.len() < 2
        || world
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .is_none_or(|s| s.measured_parked_trees() < policies.0.len())
    {
        return false;
    }
    let mut requests = Vec::with_capacity(policies.0.len());
    let mut decision = Vec::with_capacity(policies.0.len());
    for (&id, p) in &mut policies.0 {
        let Ok(Some(root)) = articulation::parked_zone_root(world, id) else {
            return false;
        };
        let Ok(pose) = articulation::root_pose(world, id) else {
            return false;
        };
        if p.target(pose.translation, time) != Some(ArticulationTier::Prescribed) {
            return false;
        }
        let since = p.eligible_since.unwrap_or(time.start_time);
        let apply = time.start_time.saturating_sub(since) >= p.eligibility_dwell;
        requests.push((root, apply));
        decision.push((id, (!apply).then_some(since)));
    }
    let Ok(true) = world
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<MultiFidelityBackend>()
        .expect("checked local engine")
        .apply_parked_demotions(&requests, time.dt)
    else {
        return false;
    };
    for (id, since) in decision {
        policies
            .0
            .get_mut(&id)
            .expect("existing policy")
            .eligible_since = since;
    }
    true
}

fn evaluate<const AFTER_INPUTS: bool>(world: &mut World) {
    let time = *world.resource::<UpdateInfo>();
    if time.dt.is_zero() || world.resource::<StepFault>().error().is_some() {
        return;
    }
    world.resource_scope(|world, mut policies: Mut<Policies<AFTER_INPUTS>>| {
        if evaluate_parked(world, &mut policies, time) {
            return;
        }
        for (&model, policy) in &mut policies.0 {
            let result = (|| {
                let pose = articulation::root_pose(world, model)?;
                let Some(target) = policy.target(pose.translation, time) else {
                    policy.eligible_since = None;
                    return Ok(());
                };
                if articulation::tier(world, model)? == target {
                    policy.eligible_since = None;
                    return Ok(());
                }
                if AFTER_INPUTS && target == ArticulationTier::Prescribed {
                    let start = *policy.eligible_since.get_or_insert(time.start_time);
                    let waiting = time.start_time.saturating_sub(start) < policy.eligibility_dwell;
                    if waiting || crate::contact_promotion::enabled(world, model) {
                        articulation::validate_demotion(world, model, time.dt)?;
                    }
                    if waiting {
                        return Ok(());
                    }
                }
                policy.eligible_since = None;
                articulation::switch(world, model, target, time.dt)?;
                Ok(())
            })();
            match result {
                Ok(()) => {}
                Err(FidelityError::PendingCommand)
                | Err(FidelityError::Physics(gz_physics::PhysicsError::TransitionDeferred {
                    ..
                })) => policy.eligible_since = None,
                Err(error) => {
                    world.resource::<StepFault>().report(error);
                    break;
                }
            }
        }
    });
}
