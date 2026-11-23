// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optional measured execution placement for a homogeneous-owner, fixed SDF population.
use super::{
    BatchBoundary, ExecutionPlacement, GpuSceneBindings, MixedBatch, MixedBatchOwner, Simulation,
    batch_boundary,
};
use bevy_ecs::prelude::*;
use gz_physics_gpu::simulation::GpuReset;
use gz_sim_core::{Phase, UpdateInfo};
use std::{collections::VecDeque, time::Instant};
mod policy;
use policy::Policy;
pub use policy::{Evidence, SamplingPhase};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AutoPlacementConfig {
    /// Settled advancing ticks per baseline, alternative, or confirmation window.
    pub window_ticks: u16,
    /// Required stable workload duration and maximum acceptable amortization horizon.
    pub min_residency_ticks: u64,
    /// Minimum interval from a completed comparison to the next measurement.
    pub reprobe_ticks: u64,
    /// Required relative margin between the conservative timing quartiles.
    pub hysteresis_percent: u8,
}
impl Default for AutoPlacementConfig {
    fn default() -> Self {
        Self {
            window_ticks: 32,
            min_residency_ticks: 128,
            reprobe_ticks: 2048,
            hysteresis_percent: 8,
        }
    }
}
impl AutoPlacementConfig {
    pub(super) fn valid(self) -> bool {
        (8..=256).contains(&self.window_ticks)
            && self.min_residency_ticks >= self.window_ticks as u64
            && self.reprobe_ticks >= self.min_residency_ticks
            && (1..=50).contains(&self.hysteresis_percent)
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlacementReason {
    Probe,
    ConfirmBaseline,
    Commit,
    UnstableWindow,
    InsufficientGain,
    InsufficientResidence,
    WorkloadChanged,
    ManualOverride,
    Resumed,
    OwnershipChanged,
}
#[derive(Clone, Copy, Debug)]
pub struct PlacementEvent {
    pub at: BatchBoundary,
    pub workload: u64,
    /// None denotes mixed ownership after a manual override.
    pub owner: Option<ExecutionPlacement>,
    pub reason: PlacementReason,
    pub evidence: Evidence,
}
/// Bounded observations for the caller. Timestamps belong to Gazebo's committed
/// clock; wall time influences placement decisions only when explicitly enabled.
#[derive(Resource)]
pub struct AutomaticPlacement {
    owner: MixedBatchOwner,
    ids: Vec<u64>,
    policy: Policy,
    initial: ExecutionPlacement,
    actual: ExecutionPlacement,
    workload: u64,
    start: Option<Instant>,
    events: VecDeque<PlacementEvent>,
    probes: u64,
}
impl AutomaticPlacement {
    pub fn events(&self) -> &VecDeque<PlacementEvent> {
        &self.events
    }
    pub fn phase(&self) -> SamplingPhase {
        self.policy.phase
    }
    pub fn workload(&self) -> u64 {
        self.workload
    }
    pub fn probes(&self) -> u64 {
        self.probes
    }
    fn record(&mut self, at: BatchBoundary, reason: PlacementReason, evidence: Evidence) {
        if self.events.len() == 64 {
            self.events.pop_front();
        }
        self.events.push_back(PlacementEvent {
            at,
            workload: self.workload,
            owner: Some(self.actual),
            reason,
            evidence,
        });
    }
    pub(super) fn manual(&mut self, at: BatchBoundary, observed: Option<ExecutionPlacement>) {
        if self.policy.phase == SamplingPhase::Manual
            && self.events.back().is_some_and(|e| e.owner == observed)
        {
            return;
        }
        self.start = None;
        self.policy.phase = SamplingPhase::Manual;
        self.record(at, PlacementReason::ManualOverride, Evidence::default());
        self.events.back_mut().unwrap().owner = observed;
    }
    fn move_to(
        &mut self,
        batch: &mut MixedBatch,
        at: BatchBoundary,
        target: ExecutionPlacement,
    ) -> Result<(), gz_physics_gpu::GpuError> {
        if self.actual != target {
            let before = Instant::now();
            batch.set_placement(at.tick, &self.ids, target)?;
            self.policy.management_cost(before.elapsed());
            self.actual = target;
        }
        Ok(())
    }
}
#[derive(thiserror::Error, Debug)]
pub enum AutoPlacementError {
    #[error("invalid automatic placement configuration")]
    Configuration,
    #[error(
        "automatic placement requires a loaded GPU scene with a uniform starting execution owner"
    )]
    Population,
    #[error("automatic placement is already installed")]
    AlreadyInstalled,
    #[error(
        "automatic placement requires the current committed boundary and its original scene owner"
    )]
    Boundary,
    #[error("automatic placement was suspended by an explicit manual placement")]
    Manual,
    #[error(transparent)]
    Physics(#[from] gz_physics_gpu::GpuError),
}
/// Opt in at a committed boundary. All environments keep advancing during probes.
/// Both execution paths are timed through the ordinary Bevy physics, sensor, and publication stages.
pub fn install_auto_placement(
    sim: &mut Simulation,
    config: AutoPlacementConfig,
) -> Result<(), AutoPlacementError> {
    if !config.valid() {
        return Err(AutoPlacementError::Configuration);
    }
    if sim.world().contains_resource::<AutomaticPlacement>() {
        return Err(AutoPlacementError::AlreadyInstalled);
    }
    let batch = sim
        .world()
        .get_resource::<MixedBatch>()
        .ok_or(AutoPlacementError::Population)?;
    let bindings = sim
        .world()
        .get_resource::<GpuSceneBindings>()
        .ok_or(AutoPlacementError::Population)?;
    if sim.is_faulted()
        || batch.owner() != bindings.owner
        || batch.tick() != batch_boundary(sim).tick
    {
        return Err(AutoPlacementError::Boundary);
    }
    if batch.has_guarded_native_pairs() {
        return Err(AutoPlacementError::Population);
    }
    let owner = uniform(batch).ok_or(AutoPlacementError::Population)?;
    let start_owner = if batch.initial_cpu_environments() == 0 {
        ExecutionPlacement::Gpu
    } else if batch.initial_cpu_environments() == batch.environments().len() {
        ExecutionPlacement::Cpu
    } else {
        return Err(AutoPlacementError::Population);
    };
    let state = AutomaticPlacement {
        owner: batch.owner(),
        ids: batch.environments().iter().map(|e| e.id).collect(),
        policy: Policy::new(config, owner, batch.tick()),
        initial: start_owner,
        actual: owner,
        workload: 0,
        start: None,
        events: VecDeque::with_capacity(64),
        probes: 0,
    };
    if !sim.register_extension::<AutomaticPlacement>() {
        return Err(AutoPlacementError::AlreadyInstalled);
    }
    sim.world_mut().insert_resource(state);
    sim.add_systems(Phase::Ingress, start);
    sim.add_systems(Phase::Finalize, finish);
    sim.add_systems(Phase::Reset, reset.after(GpuReset));
    Ok(())
}
fn uniform(batch: &MixedBatch) -> Option<ExecutionPlacement> {
    let n = batch.cpu_environments();
    if n == 0 {
        Some(ExecutionPlacement::Gpu)
    } else if n == batch.environments().len() {
        Some(ExecutionPlacement::Cpu)
    } else {
        None
    }
}
/// Declare a change in the workload that affects whole-step cost, such as a
/// host acquisition or learner phase. Repeated keys at the same workload are free.
/// During exploration the previous incumbent resumes immediately at this same barrier.
pub fn set_placement_workload(
    sim: &mut Simulation,
    at: BatchBoundary,
    key: u64,
) -> Result<(), AutoPlacementError> {
    if sim.is_faulted() || at != batch_boundary(sim) {
        return Err(AutoPlacementError::Boundary);
    }
    let world = sim.world_mut();
    if !world.contains_resource::<AutomaticPlacement>() {
        return Err(AutoPlacementError::Boundary);
    }
    world.resource_scope(|w, mut a: Mut<AutomaticPlacement>| {
        if !w
            .get_resource::<GpuSceneBindings>()
            .is_some_and(|binding| binding.owner == a.owner)
        {
            return Err(AutoPlacementError::Boundary);
        }
        let mut b = w
            .get_resource_mut::<MixedBatch>()
            .ok_or(AutoPlacementError::Boundary)?;
        if b.owner() != a.owner || b.tick() != at.tick {
            return Err(AutoPlacementError::Boundary);
        }
        if a.policy.phase == SamplingPhase::Manual {
            return Err(AutoPlacementError::Manual);
        }
        if uniform(&b) != Some(a.actual) {
            return Err(AutoPlacementError::Boundary);
        }
        if a.workload != key {
            let incumbent = a.policy.incumbent;
            a.move_to(&mut b, at, incumbent)?;
            a.workload = key;
            a.policy.rearm(at.tick, incumbent);
            a.start = None;
            a.record(at, PlacementReason::WorkloadChanged, Evidence::default());
        }
        Ok(())
    })
}
/// Resume profiling after a manual override once the population again has one
/// execution owner. The caller's current authoritative physical frame is preserved.
pub fn resume_auto_placement(
    sim: &mut Simulation,
    at: BatchBoundary,
) -> Result<(), AutoPlacementError> {
    if sim.is_faulted() || batch_boundary(sim) != at {
        return Err(AutoPlacementError::Boundary);
    }
    let w = sim.world_mut();
    let batch = w
        .get_resource::<MixedBatch>()
        .ok_or(AutoPlacementError::Boundary)?;
    let scene = w
        .get_resource::<GpuSceneBindings>()
        .ok_or(AutoPlacementError::Boundary)?;
    if scene.owner != batch.owner() || batch.tick() != at.tick {
        return Err(AutoPlacementError::Boundary);
    }
    let owner = uniform(batch).ok_or(AutoPlacementError::Population)?;
    let binding = batch.owner();
    let mut a = w
        .get_resource_mut::<AutomaticPlacement>()
        .ok_or(AutoPlacementError::Boundary)?;
    if a.owner != binding {
        return Err(AutoPlacementError::Boundary);
    }
    if a.policy.phase == SamplingPhase::Manual {
        a.policy.rearm(at.tick, owner);
        a.actual = owner;
        a.start = None;
        a.record(at, PlacementReason::Resumed, Evidence::default());
    }
    Ok(())
}
fn start(info: Res<UpdateInfo>, mut a: ResMut<AutomaticPlacement>) {
    a.start = (!info.dt.is_zero() && a.policy.phase != SamplingPhase::Manual).then(Instant::now);
}
fn finish(info: Res<UpdateInfo>, mut a: ResMut<AutomaticPlacement>, mut batch: ResMut<MixedBatch>) {
    let Some(start) = a.start.take() else {
        return;
    };
    let at = BatchBoundary {
        epoch: info.epoch,
        tick: info.iteration,
    };
    if batch.owner() != a.owner || batch.tick() != at.tick || uniform(&batch) != Some(a.actual) {
        a.policy.phase = SamplingPhase::Manual;
        a.record(at, PlacementReason::OwnershipChanged, Evidence::default());
        a.events.back_mut().unwrap().owner = uniform(&batch);
        return;
    }
    let Some(d) = a.policy.sample(at.tick, start.elapsed()) else {
        return;
    };
    if a.move_to(&mut batch, at, d.target).is_err() {
        a.policy.phase = SamplingPhase::Manual;
        a.record(at, PlacementReason::OwnershipChanged, d.evidence);
        return;
    }
    if d.reason == PlacementReason::Probe {
        a.probes = a.probes.saturating_add(1);
    }
    a.record(at, d.reason, d.evidence);
}
fn reset(mut a: ResMut<AutomaticPlacement>) {
    let initial = a.initial;
    a.policy.rearm(0, initial);
    a.actual = initial;
    a.workload = 0;
    a.start = None;
    a.probes = 0;
    a.events.clear();
}
