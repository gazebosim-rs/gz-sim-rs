// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::*;
use gz_physics::PhysicsBackend;
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo};
use gz_sim_scene::{
    JointHandle,
    distributed_hinge::{
        ArticulationPool, DistributedHingeBackend, DistributedMovingHingeBackend,
        DistributedMultiSphereBackend, MovingPool, MultiSpherePool, SessionBackend,
    },
};
use std::{error::Error, io};

use gz_sim_cli::live::JointTorqueDrive;

fn command<P: SessionBackend>(
    time: Res<UpdateInfo>,
    drive: Res<JointTorqueDrive>,
    joints: Query<&JointHandle>,
    mut backend: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
) {
    if time.dt.is_zero() {
        return;
    }
    let Ok(joint) = joints.single() else {
        fault.report(io::Error::other("expected one driven joint"));
        return;
    };
    let Some(host) = backend.downcast_mut::<DistributedHingeBackend<P>>() else {
        fault.report(io::Error::other("driven hinge backend was replaced"));
        return;
    };
    // A restored durable command is already part of the next advancing step.
    let torque = drive.interval_effort(time.iteration - 1);
    match host.pool().pending_joint_effort() {
        Some(pending) if pending.to_bits() == torque.to_bits() => (),
        Some(_) => fault.report(io::Error::other(
            "pending hinge torque differs from the drive",
        )),
        None => {
            if let Err(error) = host.stage_step_effort(joint.id(), torque) {
                fault.report(error);
            }
        }
    }
}

fn install<P: SessionBackend>(
    sim: &mut Simulation,
    drive: gz_sim_cli::live::JointTorqueDrive,
) -> Result<(), Box<dyn Error>> {
    let torque = drive.interval_effort(sim.committed_update_info()?.iteration);
    let pending = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<DistributedHingeBackend<P>>()
        .ok_or("expected a distributed hinge backend")?
        .pool()
        .pending_joint_effort();
    if pending.is_some_and(|value| value.to_bits() != torque.to_bits()) {
        return Err("saved hinge torque differs from the selected drive".into());
    }
    sim.world_mut().insert_resource(drive);
    sim.add_systems(Phase::PreStep, command::<P>);
    Ok(())
}

pub fn install_drive(sim: &mut Simulation, torque: f64) -> Result<(), Box<dyn Error>> {
    install_program(sim, gz_sim_cli::live::JointTorqueDrive::constant(torque)?)
}

pub fn install_program(
    sim: &mut Simulation,
    drive: gz_sim_cli::live::JointTorqueDrive,
) -> Result<(), Box<dyn Error>> {
    let backend = sim.world().resource::<PhysicsBackend>();
    if backend
        .downcast_ref::<DistributedMultiSphereBackend>()
        .is_some()
    {
        install::<MultiSpherePool>(sim, drive)
    } else if backend
        .downcast_ref::<DistributedMovingHingeBackend>()
        .is_some()
    {
        install::<MovingPool>(sim, drive)
    } else {
        install::<ArticulationPool>(sim, drive)
    }
}
