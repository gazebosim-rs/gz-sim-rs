// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{Body, ModelBounds, MotionError};
use bevy_ecs::prelude::*;
use gz_physics::{BodyState, PhysicsInstallation};
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo};

/// Bevy-owned constant-velocity body and its immutable constructor state.
/// Fixed motion or static ModelBounds holds pose and clears velocities on step/reset.
/// Hierarchy and link-command policy belong to the simulator adapter.
#[derive(Component, Clone, Debug)]
pub struct Motion {
    body: Body,
    initial: Body,
    fixed: bool,
}

impl Motion {
    pub fn new(state: BodyState) -> Result<Self, MotionError> {
        let body = Body::try_from(state)?;
        Ok(Self {
            initial: body.clone(),
            body,
            fixed: false,
        })
    }

    /// Constructs immobile motion independently of collision geometry.
    /// Stored velocities are cleared on the next physics phase or reset.
    pub fn new_fixed(state: BodyState) -> Result<Self, MotionError> {
        Ok(Self {
            fixed: true,
            ..Self::new(state)?
        })
    }

    pub fn state(&self) -> &BodyState {
        self.body.state()
    }

    /// Replaces finite world velocities without changing the state restored by reset.
    /// Fixed motion clears these values on the next physics phase; scene admission
    /// rejects fixed-body commands before this low-level setter is called.
    pub fn set_world_velocity(
        &mut self,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), MotionError> {
        self.body = Body::try_from(BodyState {
            velocity: linear,
            angular_velocity: angular,
            ..*self.body.state()
        })?;
        Ok(())
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

fn freeze(motion: &mut Motion) {
    motion.body.state.velocity = [0.0; 3];
    motion.body.state.angular_velocity = [0.0; 3];
}

fn advance(
    time: Res<UpdateInfo>,
    fault: Res<StepFault>,
    mut bodies: Query<(Entity, &mut Motion, Option<&ModelBounds>)>,
) {
    for (entity, mut motion, bounds) in &mut bodies {
        if motion.fixed || bounds.is_some_and(ModelBounds::is_static) {
            freeze(&mut motion);
            continue;
        }
        if time.dt.is_zero() {
            continue;
        }
        if let Err(source) = motion.body.advance(time.dt) {
            fault.report(StepFailure { entity, source });
            break;
        }
    }
}

fn reset(mut bodies: Query<(&mut Motion, Option<&ModelBounds>)>) {
    for (mut motion, bounds) in &mut bodies {
        motion.body = motion.initial.clone();
        if motion.fixed || bounds.is_some_and(ModelBounds::is_static) {
            freeze(&mut motion);
        }
    }
}
