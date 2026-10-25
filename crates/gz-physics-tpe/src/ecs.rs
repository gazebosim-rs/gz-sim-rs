// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{Body, MotionError};
use bevy_ecs::prelude::*;
use gz_physics::{BodyState, PhysicsInstallation};
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo};

/// Bevy-owned constant-velocity body and its immutable constructor state.
/// No static flag, hierarchy, collider or link-command policy is implied.
#[derive(Component, Clone, Debug)]
pub struct Motion {
    body: Body,
    initial: Body,
}

impl Motion {
    pub fn new(state: BodyState) -> Result<Self, MotionError> {
        let body = Body::try_from(state)?;
        Ok(Self {
            initial: body.clone(),
            body,
        })
    }

    pub fn state(&self) -> &BodyState {
        self.body.state()
    }
}

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MotionReset;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum InstallError {
    #[error("a physics backend is already installed in this world")]
    AlreadyInstalled,
}

/// Context for a failed body step. Earlier body advances are not rolled back.
#[derive(Debug, thiserror::Error)]
#[error("TPE motion failed for Bevy entity {entity}: {source}")]
pub struct StepFailure {
    pub entity: Entity,
    #[source]
    pub source: MotionError,
}

/// Installs TPE motion/reset systems once, exclusively with other physics installers.
/// Bevy owns body lifetimes. Reset restores each surviving Motion's constructor
/// state; removed bodies are not recreated by this low-level adapter.
pub fn install(sim: &mut Simulation) -> Result<(), InstallError> {
    if !sim.register_extension::<PhysicsInstallation>() {
        return Err(InstallError::AlreadyInstalled);
    }
    sim.add_systems(Phase::PhysicsStep, advance);
    sim.add_systems(Phase::Reset, reset.in_set(MotionReset));
    crate::contacts::install(sim);
    Ok(())
}

fn advance(time: Res<UpdateInfo>, fault: Res<StepFault>, mut bodies: Query<(Entity, &mut Motion)>) {
    if time.dt.is_zero() {
        return;
    }
    for (entity, mut motion) in &mut bodies {
        if let Err(source) = motion.body.advance(time.dt) {
            fault.report(StepFailure { entity, source });
            break;
        }
    }
}

fn reset(mut bodies: Query<&mut Motion>) {
    for mut motion in &mut bodies {
        motion.body = motion.initial.clone();
    }
}
