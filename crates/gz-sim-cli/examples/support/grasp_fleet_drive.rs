// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two independently driven grippers sharing a single simulation clock.
use super::grasp_drive::{GraspDrive, Grip};
use bevy_ecs::prelude::*;
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo, entity_by_id};
use gz_sim_scene::{
    Name,
    distributed_hinge::{DistributedGraspFleetBackend, GraspTreeSpec},
};
use std::error::Error;

#[derive(Clone, Copy, clap::ValueEnum, PartialEq, Eq, Debug)]
pub enum FleetSchedule {
    /// Each group closes immediately, receives a distinct signed load and releases at its own clock.
    Bilateral,
    /// Delay the north gripper by 350 ms and drive the southern part away and back.
    Staggered,
    /// Accelerate two initially clear parts toward one another, then reverse the force.
    Ballistic,
    /// Add a transverse load during the approach to produce sliding friction at impact.
    Oblique,
    /// Maintain both capped grasps and bring the loaded fingers into physical contact.
    LoadedContact,
}
#[derive(Resource, Clone)]
pub struct FleetDrive(Vec<MemberDrive>, FleetSchedule);
#[derive(Clone)]
struct MemberDrive {
    start: u64,
    drive: GraspDrive,
    inward: f64,
    first_joint: gz_physics::JointId,
}
impl FleetDrive {
    pub fn resolve(
        sim: &Simulation,
        specs: &[GraspTreeSpec],
        schedule: FleetSchedule,
    ) -> Result<Self, Box<dyn Error>> {
        let mut plans = Vec::new();
        for (index, s) in specs.iter().enumerate() {
            let id = s.payload.tree.model_id;
            let e = entity_by_id(sim.world(), id).ok_or("missing fleet model")?;
            let name = &sim.world().get::<Name>(e).ok_or("missing gripper name")?.0;
            // Stagger the alternating release/regrip and sustained-load exercises.
            let (mode, offset) = if index % 2 == 0 {
                (Grip::Return, 0)
            } else {
                (Grip::Strong, 350)
            };
            let drive = GraspDrive::resolve_named(sim, mode, s.payload.payload.id, name)?;
            let first = s
                .payload
                .tree
                .joints
                .first()
                .ok_or("gripper has no joints")?
                .joint_id;
            let first_joint = drive
                .fingers()
                .iter()
                .find(|f| f.0 == first)
                .ok_or("first solver joint must be a named finger")?
                .1;
            plans.push(MemberDrive {
                start: offset,
                drive,
                first_joint,
                inward: -s.payload.payload.position[1].signum(),
            });
        }
        Ok(Self(plans, schedule))
    }
    pub fn apply(
        &self,
        be: &mut PhysicsBackend,
        tick: u64,
    ) -> Result<(), gz_physics::PhysicsError> {
        for (idx, member) in self.0.iter().enumerate() {
            let MemberDrive {
                start,
                drive,
                first_joint,
                inward,
            } = member;
            if matches!(self.1, FleetSchedule::Ballistic | FleetSchedule::Oblique) {
                if idx == 0 {
                    be.add_joint_effort(*first_joint, 0.0002)?;
                }
                be.add_world_wrench(
                    drive.payload(),
                    [
                        0.,
                        if tick < 900 {
                            4. * inward
                        } else {
                            -4. * inward
                        },
                        if self.1 == FleetSchedule::Oblique && tick < 900 {
                            0.03 * inward
                        } else {
                            0.
                        },
                    ],
                    [0.; 3],
                )?;
                continue;
            }
            if self.1 == FleetSchedule::LoadedContact {
                if tick == 0 {
                    for &(_, joint, sign) in drive.fingers() {
                        be.set_joint_velocity_motor(
                            joint,
                            Some(JointVelocityMotor {
                                target_velocity: 6. * sign,
                                max_force: 4.,
                            }),
                        )?;
                    }
                }
                if tick >= 650 {
                    be.add_world_wrench(
                        drive.payload(),
                        [
                            0.,
                            if tick < 1980 {
                                3. * inward
                            } else {
                                -3. * inward
                            },
                            0.,
                        ],
                        [0.; 3],
                    )?;
                }
                if tick.is_multiple_of(107) {
                    be.add_joint_effort(*first_joint, 0.0002 * inward)?;
                }
                continue;
            }
            if self.1 == FleetSchedule::Staggered {
                if let Some(local) = tick.checked_sub(*start) {
                    drive.apply(be, local)?;
                }
                continue;
            }
            let (open, close) = Self::switches(idx);
            if tick == 0 || tick == open || tick == close {
                for &(_, j, sign) in drive.fingers() {
                    be.set_joint_velocity_motor(j, Some(Self::motor(idx, tick, sign)))?;
                }
            }
            if (600..1400).contains(&tick) {
                let z = if idx % 2 == 0 { 1.2 } else { -0.82 };
                be.add_world_wrench(drive.payload(), [0., 0., z], [0.; 3])?;
            }
            if tick.is_multiple_of(67) {
                be.add_joint_effort(
                    drive.fingers()[((tick / 67) % 2) as usize].1,
                    if idx % 2 == 0 { 0.001 } else { -0.002 },
                )?;
            }
            if idx == 0 && tick == 510 {
                be.add_joint_effort(drive.fingers()[0].1, -0.017)?;
            }
        }
        Ok(())
    }
    fn switches(idx: usize) -> (u64, u64) {
        if idx.is_multiple_of(2) {
            (830, 1260)
        } else {
            (1120, 1350)
        }
    }
    fn motor(idx: usize, tick: u64, sign: f64) -> JointVelocityMotor {
        let (open, close) = Self::switches(idx);
        JointVelocityMotor {
            target_velocity: sign
                * if (open..close).contains(&tick) {
                    -6.
                } else {
                    6.
                },
            max_force: 4.,
        }
    }
    pub fn install(sim: &mut Simulation, schedule: FleetSchedule) -> Result<(), Box<dyn Error>> {
        let pool = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<DistributedGraspFleetBackend>()
            .ok_or("expected a worker-owned grasp fleet")?
            .pool();
        let plan = Self::resolve(sim, &pool.spec().assemblies, schedule)?;
        for (idx, member) in plan.0.iter().enumerate() {
            let MemberDrive { start, drive, .. } = member;
            if schedule == FleetSchedule::Staggered {
                drive.verify_motors(pool.tick().saturating_sub(*start), |id| {
                    pool.velocity_motor(id)
                })?;
            } else if pool.tick() > 0 {
                for &(id, _, sign) in drive.fingers() {
                    let expected = match schedule {
                        FleetSchedule::Ballistic | FleetSchedule::Oblique => None,
                        FleetSchedule::LoadedContact => Some(JointVelocityMotor {
                            target_velocity: 6. * sign,
                            max_force: 4.,
                        }),
                        _ => Some(Self::motor(idx, pool.tick() - 1, sign)),
                    };
                    if pool.velocity_motor(id) != expected {
                        return Err(format!(
                            "saved fleet motor {id} differs from selected schedule"
                        )
                        .into());
                    }
                }
            }
        }
        sim.world_mut().insert_resource(plan);
        sim.add_systems(Phase::PreStep, command);
        Ok(())
    }
}
fn command(
    clock: Res<UpdateInfo>,
    plan: Res<FleetDrive>,
    mut backend: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
) {
    if !clock.dt.is_zero()
        && let Err(e) = plan.apply(&mut backend, clock.iteration - 1)
    {
        fault.report(e);
    }
}
