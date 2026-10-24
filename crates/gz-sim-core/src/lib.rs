// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicit simulation steps over a Bevy ECS world, independent of rendering.

use bevy_ecs::{prelude::*, schedule::ScheduleConfigs, system::ScheduleSystem};
use std::{any::TypeId, collections::HashSet, time::Duration};

/// Ordered phases of one simulation tick. Systems within a phase may run in parallel.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Phase {
    Ingress,
    Lifecycle,
    PreStep,
    PhysicsPrepare,
    PhysicsStep,
    PhysicsCommit,
    PostStep,
    Publish,
    Finalize,
    Reset,
}

/// Time visible to systems. `sim_time` is the end of the step, as in gz-sim.
#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
pub struct UpdateInfo {
    pub start_time: Duration,
    pub sim_time: Duration,
    pub dt: Duration,
    pub iteration: u64,
    pub epoch: u64,
    pub paused: bool,
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum StepError {
    #[error("step duration must be positive")]
    ZeroDuration,
    #[error("simulation time, iteration, or epoch overflow")]
    Overflow,
}

/// Owns the ECS and advances only when explicitly requested.
///
/// `tick` respects pause; `step_exact` advances a requested number of steps even
/// while paused. Reset handlers restore domain state; the core resets only time.
/// A panicking system invalidates that run: mutations are not rolled back.
pub struct Simulation {
    world: World,
    schedule: Schedule,
    reset_schedule: Schedule,
    step_duration: Duration,
    elapsed: Duration,
    iteration: u64,
    epoch: u64,
    paused: bool,
    extensions: HashSet<TypeId>,
}

impl Simulation {
    pub fn new(step_duration: Duration) -> Result<Self, StepError> {
        if step_duration.is_zero() {
            return Err(StepError::ZeroDuration);
        }
        let mut world = World::new();
        world.insert_resource(UpdateInfo {
            start_time: Duration::ZERO,
            sim_time: Duration::ZERO,
            dt: Duration::ZERO,
            iteration: 0,
            epoch: 0,
            paused: true,
        });
        let mut schedule = Schedule::default();
        schedule.configure_sets(
            (
                Phase::Ingress,
                Phase::Lifecycle,
                Phase::PreStep,
                Phase::PhysicsPrepare,
                Phase::PhysicsStep,
                Phase::PhysicsCommit,
                Phase::PostStep,
                Phase::Publish,
                Phase::Finalize,
            )
                .chain(),
        );
        Ok(Self {
            world,
            schedule,
            reset_schedule: Schedule::default(),
            step_duration,
            elapsed: Duration::ZERO,
            iteration: 0,
            epoch: 0,
            paused: true,
            extensions: HashSet::new(),
        })
    }

    /// Claims a system extension once, independently of mutable ECS resources.
    pub fn register_extension<T: 'static>(&mut self) -> bool {
        self.extensions.insert(TypeId::of::<T>())
    }

    /// Registers systems using Bevy's own scheduling configuration.
    pub fn add_systems<M>(
        &mut self,
        phase: Phase,
        systems: impl IntoScheduleConfigs<ScheduleSystem, M>,
    ) {
        let configured: ScheduleConfigs<ScheduleSystem> = systems.in_set(phase);
        if phase == Phase::Reset {
            self.reset_schedule.add_systems(configured);
        } else {
            self.schedule.add_systems(configured);
        }
    }

    pub fn world(&self) -> &World {
        &self.world
    }

    /// Direct ECS access. UpdateInfo is refreshed before every tick and reset.
    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }

    pub fn set_paused(&mut self, paused: bool) {
        self.paused = paused;
    }

    pub fn is_paused(&self) -> bool {
        self.paused
    }

    pub fn elapsed(&self) -> Duration {
        self.elapsed
    }

    /// Runs all phases once. Paused ticks have zero dt and do not advance time.
    pub fn tick(&mut self) -> Result<UpdateInfo, StepError> {
        self.run_tick(self.paused)
    }

    /// Advances exactly `count` steps, retaining the current pause setting.
    /// Overflow is checked for the whole request before any system runs.
    pub fn step_exact(&mut self, count: u64) -> Result<(), StepError> {
        let nanos = self
            .step_duration
            .as_nanos()
            .checked_mul(u128::from(count))
            .ok_or(StepError::Overflow)?;
        let available = (Duration::MAX - self.elapsed).as_nanos();
        if nanos > available || self.iteration.checked_add(count).is_none() {
            return Err(StepError::Overflow);
        }
        for _ in 0..count {
            self.run_tick(false)?;
        }
        Ok(())
    }

    fn run_tick(&mut self, paused: bool) -> Result<UpdateInfo, StepError> {
        let dt = if paused {
            Duration::ZERO
        } else {
            self.step_duration
        };
        let end = self.elapsed.checked_add(dt).ok_or(StepError::Overflow)?;
        let iteration = self
            .iteration
            .checked_add(u64::from(!paused))
            .ok_or(StepError::Overflow)?;
        let info = UpdateInfo {
            start_time: self.elapsed,
            sim_time: end,
            dt,
            iteration,
            epoch: self.epoch,
            paused,
        };
        self.world.insert_resource(info);
        self.schedule.run(&mut self.world);
        self.elapsed = end;
        self.iteration = iteration;
        Ok(info)
    }

    /// Starts a new epoch and invokes reset handlers without advancing physics.
    pub fn reset(&mut self) -> Result<(), StepError> {
        let epoch = self.epoch.checked_add(1).ok_or(StepError::Overflow)?;
        self.elapsed = Duration::ZERO;
        self.iteration = 0;
        self.epoch = epoch;
        self.world.insert_resource(UpdateInfo {
            start_time: Duration::ZERO,
            sim_time: Duration::ZERO,
            dt: Duration::ZERO,
            iteration: 0,
            epoch,
            paused: self.paused,
        });
        self.reset_schedule.run(&mut self.world);
        Ok(())
    }
}

#[cfg(test)]
#[path = "../tests/private/overflow.rs"]
mod overflow_tests;
