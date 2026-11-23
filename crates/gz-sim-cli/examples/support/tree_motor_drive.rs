// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A small ordinary Bevy controller changes native motor settings only at command boundaries.
use super::floating_tree_drive;
use bevy_ecs::prelude::*;
use gz_physics::{JointId, JointVelocityMotor, PhysicsBackend};
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo};
use gz_sim_scene::distributed_hinge::DistributedMotorTreeBackend;
use std::{collections::BTreeSet, error::Error, num::NonZeroU64};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Clone, Resource)]
pub struct MotorDrive {
    pub inputs: Vec<(u64, JointId, JointVelocityMotor)>,
    reverse_every: Option<NonZeroU64>,
}
pub fn parse(s: &str) -> std::result::Result<(String, JointVelocityMotor), String> {
    let (name, numbers) = s
        .split_once('=')
        .ok_or("expected JOINT=RAD_PER_SEC,MAX_NM")?;
    let (v, f) = numbers
        .split_once(',')
        .ok_or("expected velocity,maximum torque")?;
    let target_velocity = v.parse::<f64>().map_err(|e| e.to_string())?;
    let max_force = f.parse::<f64>().map_err(|e| e.to_string())?;
    if name.is_empty() || !target_velocity.is_finite() || !max_force.is_finite() || max_force < 0. {
        return Err("expected a joint, finite speed and nonnegative finite maximum torque".into());
    }
    Ok((
        name.into(),
        JointVelocityMotor {
            target_velocity,
            max_force,
        },
    ))
}
impl MotorDrive {
    pub fn resolve(
        sim: &Simulation,
        entries: &[(String, JointVelocityMotor)],
        reverse_every: Option<NonZeroU64>,
    ) -> Result<Self> {
        let mut inputs = Vec::new();
        if entries.is_empty() {
            for (i, (public, id, _)) in floating_tree_drive::resolve(sim, &[], None)?
                .0
                .into_iter()
                .enumerate()
            {
                let speed = 1.2 + 0.3 * i as f64;
                inputs.push((
                    public,
                    id,
                    JointVelocityMotor {
                        target_velocity: if i % 2 == 0 { speed } else { -speed },
                        max_force: 0.8 + i as f64 * 0.13,
                    },
                ));
            }
        } else {
            let mut used = BTreeSet::new();
            for (name, m) in entries {
                let (public, id, _) =
                    floating_tree_drive::resolve(sim, &[(name.clone(), m.target_velocity)], None)?
                        .0[0];
                if !used.insert(public) {
                    return Err(format!("repeated motor on joint {public}").into());
                }
                inputs.push((public, id, *m));
            }
            inputs.sort_by_key(|r| r.0);
        }
        Ok(Self {
            inputs,
            reverse_every,
        })
    }
    fn target(&self, tick: u64, mut m: JointVelocityMotor) -> JointVelocityMotor {
        if self
            .reverse_every
            .is_some_and(|p| (tick / p.get()) % 2 == 1)
        {
            m.target_velocity = -m.target_velocity;
        }
        m
    }
    pub fn configure(
        &self,
        backend: &mut PhysicsBackend,
        tick: u64,
    ) -> std::result::Result<(), gz_physics::PhysicsError> {
        if tick == 0
            || self
                .reverse_every
                .is_some_and(|p| tick.is_multiple_of(p.get()))
        {
            for &(_, id, m) in &self.inputs {
                backend.validate_joint_velocity_motor(id, Some(self.target(tick, m)))?;
            }
            for &(_, id, m) in &self.inputs {
                backend.set_joint_velocity_motor(id, Some(self.target(tick, m)))?;
            }
        }
        Ok(())
    }
    pub fn install(self, sim: &mut Simulation) -> Result<()> {
        let host = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<DistributedMotorTreeBackend>()
            .ok_or("expected motor-enabled worker")?;
        let pool = host.pool();
        let tick = pool.observation().tick;
        for &(public, _, m) in &self.inputs {
            let accepted = pool.velocity_motor(public);
            if tick > 0 && accepted != Some(self.target(tick - 1, m))
                || tick == 0 && accepted.is_some_and(|v| v != m)
            {
                return Err(format!(
                    "saved motor on joint {public} differs from the requested drive"
                )
                .into());
            }
        }
        sim.world_mut().insert_resource(self);
        sim.add_systems(Phase::PreStep, command);
        Ok(())
    }
}
fn command(
    time: Res<UpdateInfo>,
    drive: Res<MotorDrive>,
    mut be: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
) {
    if !time.dt.is_zero()
        && let Err(e) = drive.configure(&mut be, time.iteration - 1)
    {
        fault.report(e);
    }
}
