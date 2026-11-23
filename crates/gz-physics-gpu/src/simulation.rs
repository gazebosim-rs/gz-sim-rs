// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A standalone batch world on Gazebo's ordinary Bevy simulation clock.

use super::{
    GpuBatch, GpuError, Lane, ResidentBatch, STEP, SpherePlaneBatch, SpherePlaneLane, StepMetrics,
};
use bevy_ecs::prelude::*;
use bytemuck::Pod;
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo};
use std::io;

/// Measurements for the most recent committed GPU step of the current epoch.
/// Empty on initialization and reset. A paused tick preserves the last result.
#[derive(Resource, Debug, Default)]
pub struct BatchTiming {
    pub last: Option<StepMetrics>,
}

/// Create a paused simulation with one independent vertical physics environment
/// per lane. The selected GPU profile uses a fixed one-millisecond public tick.
pub fn batch_simulation(adapter_name: &str, initial: Vec<Lane>) -> Result<Simulation, GpuError> {
    Ok(install(GpuBatch::new(adapter_name, initial)?))
}

/// Create independent fixed-base, two-revolute-joint arms on the ordinary 1 ms clock.
pub fn planar_arm_simulation(
    adapter_name: &str,
    initial: Vec<super::PlanarArmLane>,
) -> Result<Simulation, GpuError> {
    Ok(install(super::PlanarArmBatch::new(adapter_name, initial)?))
}

/// Create independent two-joint arms with a round tool and a stationary floor.
pub fn planar_tool_simulation(
    adapter_name: &str,
    initial: Vec<super::PlanarToolLane>,
) -> Result<Simulation, GpuError> {
    Ok(install(super::PlanarToolBatch::new(adapter_name, initial)?))
}

/// Create a paused Bevy world of independent free sphere/plane fixtures.
pub fn sphere_plane_simulation(
    adapter_name: &str,
    initial: Vec<SpherePlaneLane>,
) -> Result<Simulation, GpuError> {
    Ok(install(SpherePlaneBatch::new(adapter_name, initial)?))
}

/// Create a paused Bevy world with two finite spinning spheres per independent
/// environment. The ordinary published cadence is one millisecond.
pub fn sphere_pair_simulation(
    adapter_name: &str,
    initial: Vec<super::SpherePairLane>,
) -> Result<Simulation, GpuError> {
    sphere_pair_simulation_with_rotation(adapter_name, initial, super::SphereRotation::Midpoint)
}
/// Create a 1 ms public simulation with an explicit sphere attitude integrator.
pub fn sphere_pair_simulation_with_rotation(
    adapter_name: &str,
    initial: Vec<super::SpherePairLane>,
    rotation: super::SphereRotation,
) -> Result<Simulation, GpuError> {
    Ok(install(super::SpherePairBatch::with_rotation(
        adapter_name,
        initial,
        rotation,
    )?))
}

/// Register several physical profiles in a single Bevy world. The complete
/// ID-ordered frame is available in the MixedBatch resource after PhysicsStep.
pub fn mixed_batch_simulation(
    adapter_name: &str,
    initial: Vec<super::Environment>,
) -> Result<Simulation, GpuError> {
    Ok(install(super::MixedBatch::new(adapter_name, initial)?))
}

/// Attach a prepared homogeneous articulated population to a fresh one-millisecond world.
pub fn install_planar_tool(
    sim: &mut Simulation,
    batch: super::PlanarToolBatch,
) -> Result<(), GpuError> {
    if sim.step_duration() != STEP
        || !sim.elapsed().is_zero()
        || sim.is_faulted()
        || batch.tick() != 0
        || sim.world().contains_resource::<BatchTiming>()
    {
        return Err(GpuError::StaleBoundary);
    }
    attach(sim, batch);
    Ok(())
}

trait ScheduledBatch: Resource<Mutability = bevy_ecs::component::Mutable> {
    fn time(&self) -> std::time::Duration;
    fn advance_one(&mut self) -> Result<StepMetrics, GpuError>;
    fn restore(&mut self);
}
impl<T: Pod + Send + Sync> ScheduledBatch for ResidentBatch<T> {
    fn time(&self) -> std::time::Duration {
        self.sim_time()
    }
    fn advance_one(&mut self) -> Result<StepMetrics, GpuError> {
        self.step()
    }
    fn restore(&mut self) {
        self.reset();
    }
}
impl ScheduledBatch for super::MixedBatch {
    fn time(&self) -> std::time::Duration {
        self.sim_time()
    }
    fn advance_one(&mut self) -> Result<StepMetrics, GpuError> {
        self.step()
    }
    fn restore(&mut self) {
        self.reset();
    }
}

fn install<B: ScheduledBatch>(batch: B) -> Simulation {
    install_at(
        Simulation::new(STEP).expect("positive fixed timestep"),
        batch,
    )
}
fn install_at<B: ScheduledBatch>(mut simulation: Simulation, batch: B) -> Simulation {
    attach(&mut simulation, batch);
    simulation
}
/// Order downstream reset publication after the GPU's fixed construction frame.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GpuReset;
/// Attach a prepared mixed physical population to a fresh ordinary Gazebo world.
pub fn install_mixed(sim: &mut Simulation, batch: super::MixedBatch) -> Result<(), GpuError> {
    if sim.step_duration() != STEP
        || !sim.elapsed().is_zero()
        || sim.is_faulted()
        || batch.tick() != 0
        || sim.world().contains_resource::<BatchTiming>()
    {
        return Err(GpuError::StaleBoundary);
    }
    attach(sim, batch);
    Ok(())
}
fn attach<B: ScheduledBatch>(simulation: &mut Simulation, batch: B) {
    simulation.world_mut().insert_resource(batch);
    simulation.world_mut().init_resource::<BatchTiming>();
    simulation.add_systems(Phase::PhysicsStep, advance::<B>);
    simulation.add_systems(Phase::Reset, reset::<B>.in_set(GpuReset));
}

fn advance<B: ScheduledBatch>(
    info: Res<UpdateInfo>,
    mut batch: ResMut<B>,
    mut timing: ResMut<BatchTiming>,
    fault: Res<StepFault>,
) {
    if info.paused {
        return;
    }
    if batch.time() != info.start_time || info.dt != STEP {
        fault.report(io::Error::other(
            "GPU batch and Gazebo step clocks disagree",
        ));
        return;
    }
    match batch.advance_one() {
        Ok(metrics) => timing.last = Some(metrics),
        Err(error) => fault.report(error),
    }
}

fn reset<B: ScheduledBatch>(mut batch: ResMut<B>, mut timing: ResMut<BatchTiming>) {
    batch.restore();
    timing.last = None;
}

/// Create a Bevy simulation whose observation barrier spans a fixed number of
/// 1 ms device ticks. Systems run at the published barrier cadence. Use the
/// resident batch directly to choose a different block length on each call.
pub fn sphere_plane_block_simulation(
    adapter_name: &str,
    initial: Vec<SpherePlaneLane>,
    ticks: u32,
) -> Result<Simulation, GpuError> {
    install_block(ticks, || {
        super::SpherePlaneBlockBatch::new(adapter_name, initial)
    })
}

/// Observe independent finite sphere pairs at a fixed multiple of the device timestep.
/// Every observation includes the impulse and contact times within that interval.
pub fn sphere_pair_block_simulation(
    adapter_name: &str,
    initial: Vec<super::SpherePairLane>,
    ticks: u32,
) -> Result<Simulation, GpuError> {
    sphere_pair_block_simulation_with_rotation(
        adapter_name,
        initial,
        ticks,
        super::SphereRotation::Midpoint,
    )
}
/// Select the angular integration accuracy independently of the observation cadence.
pub fn sphere_pair_block_simulation_with_rotation(
    adapter_name: &str,
    initial: Vec<super::SpherePairLane>,
    ticks: u32,
    rotation: super::SphereRotation,
) -> Result<Simulation, GpuError> {
    install_block(ticks, || {
        super::SpherePairBlockBatch::with_rotation(adapter_name, initial, rotation)
    })
}

fn install_block<T: Pod + Send + Sync>(
    ticks: u32,
    prepare: impl FnOnce() -> Result<ResidentBatch<T>, GpuError>,
) -> Result<Simulation, GpuError> {
    if !(1..=super::MAX_BLOCK_TICKS).contains(&ticks) {
        return Err(GpuError::BlockLength(ticks));
    }
    let sim = Simulation::new(STEP * ticks).expect("bounded positive observation duration");
    Ok(attach_observations(sim, prepare()?, ticks))
}
pub(crate) fn attach_observations<T: Pod + Send + Sync>(
    mut sim: Simulation,
    batch: ResidentBatch<T>,
    ticks: u32,
) -> Simulation {
    schedule_observations(&mut sim, batch, ticks);
    sim
}
/// Attach independent finite sphere pairs to a freshly loaded world at its authored observation cadence.
pub fn install_sphere_pair_block(
    sim: &mut Simulation,
    batch: super::SpherePairBlockBatch,
    ticks: u32,
) -> Result<(), GpuError> {
    check_pair_attachment(sim, &batch, ticks)?;
    schedule_observations(sim, batch, ticks);
    Ok(())
}
pub(crate) fn check_pair_attachment(
    sim: &Simulation,
    batch: &super::SpherePairBlockBatch,
    ticks: u32,
) -> Result<(), GpuError> {
    if !(1..=super::MAX_BLOCK_TICKS).contains(&ticks) {
        return Err(GpuError::BlockLength(ticks));
    }
    if sim.step_duration() != STEP * ticks
        || !sim.elapsed().is_zero()
        || sim.is_faulted()
        || batch.tick() != 0
        || sim.world().contains_resource::<BatchTiming>()
    {
        return Err(GpuError::StaleBoundary);
    }
    Ok(())
}
fn schedule_observations<T: Pod + Send + Sync>(
    sim: &mut Simulation,
    batch: ResidentBatch<T>,
    ticks: u32,
) {
    let dt = STEP * ticks;
    sim.world_mut().insert_resource(batch);
    sim.world_mut().init_resource::<BatchTiming>();
    sim.add_systems(
        Phase::PhysicsStep,
        move |info: Res<UpdateInfo>,
              mut batch: ResMut<ResidentBatch<T>>,
              mut timing: ResMut<BatchTiming>,
              fault: Res<StepFault>| {
            if info.paused {
                return;
            }
            if info.start_time != batch.sim_time() || info.dt != dt {
                fault.report(io::Error::other(
                    "resident block and Gazebo observation clocks disagree",
                ));
                return;
            }
            match batch.advance(ticks) {
                Ok(metrics) => timing.last = Some(metrics),
                Err(e) => fault.report(e),
            }
        },
    );
    sim.add_systems(Phase::Reset, reset::<ResidentBatch<T>>.in_set(GpuReset));
}

/// Begin a paused 1 ms Bevy world of independently co-supported sphere pairs.
pub fn supported_sphere_pair_simulation(
    adapter_name: &str,
    initial: Vec<super::SupportedSpherePairLane>,
) -> Result<Simulation, GpuError> {
    Ok(install(super::SupportedSpherePairBatch::new(
        adapter_name,
        initial,
    )?))
}

/// The model-independent stamp required for edits through a Bevy world.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatchBoundary {
    pub epoch: u64,
    pub tick: u64,
}

pub fn batch_boundary(sim: &Simulation) -> BatchBoundary {
    let info = sim.world().resource::<UpdateInfo>();
    BatchBoundary {
        epoch: info.epoch,
        tick: info.iteration,
    }
}
fn edited_batch(
    sim: &mut Simulation,
    at: BatchBoundary,
) -> Result<Mut<'_, super::MixedBatch>, GpuError> {
    if sim.is_faulted() || batch_boundary(sim) != at {
        return Err(GpuError::StaleBoundary);
    }
    sim.world_mut()
        .get_resource_mut::<super::MixedBatch>()
        .ok_or_else(|| GpuError::Checkpoint("world does not contain a mixed batch".into()))
}
/// Move complete independent contact environments between scalar CPU and f64 GPU
/// execution at this public Gazebo epoch and iteration. No physics interval elapses.
pub fn set_environment_placement(
    sim: &mut Simulation,
    at: BatchBoundary,
    ids: &[u64],
    target: super::ExecutionPlacement,
) -> Result<super::PlacementChange, GpuError> {
    edited_batch(sim, at)?.set_placement(at.tick, ids, target)
}
/// A selective new episode commits at the current barrier without moving Gazebo time.
pub fn reset_environments(
    sim: &mut Simulation,
    at: BatchBoundary,
    replacements: &[super::Environment],
) -> Result<(), GpuError> {
    edited_batch(sim, at)?.reset_selected(at.tick, replacements)
}
/// Queue one center impulse per selected environment for its next advancing tick.
pub fn queue_environment_impulses(
    sim: &mut Simulation,
    at: BatchBoundary,
    actions: &[super::CentralImpulse],
) -> Result<(), GpuError> {
    edited_batch(sim, at)?.queue_impulses(at.tick, actions)
}
/// Capture the last committed frame, including inputs admitted for the next tick.
/// A failed advance leaves this frame and pending commands available for recovery.
pub fn checkpoint_mixed(sim: &Simulation) -> Result<super::MixedCheckpoint, GpuError> {
    let info = sim
        .committed_update_info()
        .map_err(|e| GpuError::Checkpoint(e.to_string()))?;
    let batch = sim
        .world()
        .get_resource::<super::MixedBatch>()
        .ok_or_else(|| GpuError::Checkpoint("world does not contain a mixed batch".into()))?;
    super::MixedCheckpoint::capture(batch, info, sim.is_paused())
}
/// Restore a complete physics world in a new Bevy scheduler and device session.
pub fn restore_mixed_simulation(
    adapter_name: &str,
    cp: &super::MixedCheckpoint,
) -> Result<Simulation, GpuError> {
    let (info, paused) = cp.info();
    let sim = Simulation::from_committed_clock(STEP, info, paused)
        .map_err(|e| GpuError::Checkpoint(e.to_string()))?;
    Ok(install_at(sim, cp.restore(adapter_name)?))
}
