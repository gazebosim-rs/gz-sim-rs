// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A capped two-finger grasp under a calibrated transverse load.
use bevy_ecs::prelude::*;
use clap::ValueEnum;
use gz_physics::{BodyId, JointId, JointVelocityMotor, PhysicsBackend, PhysicsError};
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo, entity_by_id};
use gz_sim_scene::{
    BodyHandle,
    distributed_hinge::{DistributedGraspTreeBackend, DistributedSplitGraspBackend},
};
use std::error::Error;

#[derive(Clone, Copy, Debug, PartialEq, ValueEnum)]
pub enum Grip {
    Strong,
    Weak,
    Open,
    /// Release, drive the workpiece away and back, then reclose the fingers.
    Return,
}

#[derive(Clone, Resource)]
pub struct GraspDrive {
    mode: Grip,
    fingers: [(u64, JointId, f64); 2],
    payload: BodyId,
}
impl GraspDrive {
    pub fn resolve(sim: &Simulation, mode: Grip, payload: u64) -> Result<Self, Box<dyn Error>> {
        Self::resolve_named(sim, mode, payload, "")
    }
    pub fn resolve_named(
        sim: &Simulation,
        mode: Grip,
        payload: u64,
        model: &str,
    ) -> Result<Self, Box<dyn Error>> {
        let name = |joint: &str| {
            if model.is_empty() {
                joint.to_owned()
            } else {
                format!("{model}::{joint}")
            }
        };
        let fingers = super::floating_tree_drive::resolve(
            sim,
            &[(name("right_close"), 1.), (name("left_close"), -1.)],
            None,
        )?
        .0;
        let entity = entity_by_id(sim.world(), payload).ok_or("missing payload entity")?;
        Ok(Self {
            fingers: fingers
                .try_into()
                .map_err(|_| "expected two named gripper fingers")?,
            mode,
            payload: sim
                .world()
                .get::<BodyHandle>(entity)
                .ok_or("missing free payload body")?
                .id(),
        })
    }
    pub fn fingers(&self) -> &[(u64, JointId, f64); 2] {
        &self.fingers
    }
    pub fn payload(&self) -> BodyId {
        self.payload
    }
    pub fn first_joint(&self) -> u64 {
        self.fingers[0].0
    }
    fn motor(&self, tick: u64, sign: f64) -> JointVelocityMotor {
        let open = match self.mode {
            Grip::Open => true,
            Grip::Return => (600..1450).contains(&tick),
            _ => tick >= 1100,
        };
        let speed = if open { -6. } else { 6. };
        let cap = if tick >= 650 && self.mode == Grip::Weak {
            0.08
        } else {
            4.
        };
        JointVelocityMotor {
            target_velocity: sign * speed,
            max_force: cap,
        }
    }
    pub fn apply(&self, backend: &mut PhysicsBackend, tick: u64) -> Result<(), PhysicsError> {
        let new_motors = if self.mode == Grip::Return {
            matches!(tick, 0 | 600 | 1450)
        } else {
            matches!(tick, 0 | 650 | 1100)
        };
        if new_motors {
            for &(_, joint, sign) in &self.fingers {
                backend.validate_joint_velocity_motor(joint, Some(self.motor(tick, sign)))?;
            }
            for &(_, joint, sign) in &self.fingers {
                backend.set_joint_velocity_motor(joint, Some(self.motor(tick, sign)))?;
            }
        }
        let force = match (self.mode, tick) {
            (Grip::Return, 650..850 | 1250..1450) => 1.2,
            (Grip::Return, 850..1250) => -1.2,
            (Grip::Return, _) => 0.,
            (_, 650..1500) => 1.2,
            _ => 0.,
        };
        if force != 0. {
            backend.add_world_wrench(self.payload, [0., 0., force], [0.; 3])?;
        }
        Ok(())
    }
    /// Resolve all persisted inputs through the saved physics cut before replay starts.
    pub fn verify_motors(
        &self,
        tick: u64,
        read: impl Fn(u64) -> Option<JointVelocityMotor>,
    ) -> Result<(), Box<dyn Error>> {
        for &(joint, _, sign) in &self.fingers {
            let saved = read(joint);
            if tick > 0 && saved != Some(self.motor(tick - 1, sign))
                || tick == 0 && saved.is_some_and(|m| m != self.motor(0, sign))
            {
                return Err(format!(
                    "saved velocity motor on joint {joint} differs from the selected grasp"
                )
                .into());
            }
        }
        Ok(())
    }
    pub fn install(sim: &mut Simulation, mode: Grip) -> Result<Self, Box<dyn Error>> {
        let be = sim.world().resource::<PhysicsBackend>();
        let (payload, tick) = if let Some(split) = be.downcast_ref::<DistributedSplitGraspBackend>()
        {
            (split.pool().payload_state().id, split.pool().tick())
        } else {
            let g = be
                .downcast_ref::<DistributedGraspTreeBackend>()
                .ok_or("expected a motorized payload session")?
                .pool();
            (g.payload_state().id, g.observation().tick)
        };
        let drive = Self::resolve(sim, mode, payload)?;
        drive.verify_motors(tick, |joint| {
            if let Some(split) = be.downcast_ref::<DistributedSplitGraspBackend>() {
                split.pool().velocity_motor(joint)
            } else {
                be.downcast_ref::<DistributedGraspTreeBackend>()
                    .unwrap()
                    .pool()
                    .velocity_motor(joint)
            }
        })?;
        sim.world_mut().insert_resource(drive.clone());
        sim.add_systems(Phase::PreStep, command);
        Ok(drive)
    }
}
fn command(
    clock: Res<UpdateInfo>,
    drive: Res<GraspDrive>,
    mut be: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
) {
    if !clock.dt.is_zero()
        && let Err(e) = drive.apply(&mut be, clock.iteration - 1)
    {
        fault.report(e);
    }
}
