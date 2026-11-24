// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Torque-actuated flat-ground single support and slow alternating foot placements.
use super::{JointPostures, PostureActuation, PostureController};
use crate::{BodyHandle, JointHandle, LoadError, Name};
use bevy_ecs::prelude::*;
use glam::{DVec2, DVec3};
use gz_physics::{BodyId, JointId, PhysicsBackend, PhysicsError};
use gz_sim_core::{EntityLookup, Phase, Simulation, StepFault, UpdateInfo};
use std::{collections::BTreeMap, time::Duration};

mod heading;
mod inverse;
mod recovery;
pub mod trajectory;
use inverse::SupportControl;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FootstepMotion {
    WeightShift,
    SingleSupport,
    Step,
    Walk,
}
#[derive(Clone, Copy, Debug)]
pub struct FootstepPlan {
    pub motion: FootstepMotion,
    /// Distance from the initial centerline toward the planned stance foot, in meters.
    pub lateral_m: f64,
    /// Distance between successive planned touchdown centers along world X, in meters.
    pub length_m: f64,
    pub count: u16,
    /// Recenter the walking plan on the measured supporting foot and correct swing placement.
    pub placement_feedback: bool,
    /// Use the acquired footholds for planted-leg geometry between swings.
    pub support_relative: bool,
    /// Maximum lateral correction toward the authored lane per unloaded placement.
    /// At zero the swinging foot keeps its authored width relative to the measured stance foot.
    pub lateral_correction_limit_m: Option<f64>,
    /// Bounded hip-yaw feedback from measured pelvis and foot headings during walking.
    pub heading_gain: Option<f64>,
    /// Lower the swing foot early when the measured capture point approaches the inner sole edge.
    pub recovery_landing: bool,
}
impl FootstepPlan {
    pub(crate) fn valid(self) -> bool {
        (!self.support_relative
            || self.motion == FootstepMotion::Walk && self.placement_feedback
            || self.motion == FootstepMotion::Step && self.recovery_landing)
            && self.heading_gain.is_none_or(|gain| {
                self.motion == FootstepMotion::Walk
                    && self.support_relative
                    && (0.0..=1.).contains(&gain)
            })
            && self.lateral_correction_limit_m.is_none_or(|limit| {
                self.support_relative
                    && self.motion == FootstepMotion::Walk
                    && limit.is_finite()
                    && limit >= 0.
            })
            && self.lateral_m.is_finite()
            && (0.0..=0.17).contains(&self.lateral_m)
            && self.length_m.is_finite()
            && (0.0..=0.20).contains(&self.length_m)
            && (1..=128).contains(&self.count)
    }
}
#[derive(Clone)]
pub struct FootstepController {
    plan: FootstepPlan,
    support: SupportControl,
    joints: Vec<(JointId, &'static str)>,
    gravity: f64,
    initial_feet: [DVec2; 2],
    foot_ids: [BodyId; 2],
    floor: BodyId,
    sole_half_width: [f64; 2],
    level_sole_roll_limits: [[f64; 2]; 2],
    heading: Option<heading::HeadingControl>,
}
impl FootstepController {
    /// Bind the planar 30-axis human profile to its original SDF frames and native identities.
    pub fn bind(
        sdf: &sdformat::Sdf,
        world: &sdformat::World,
        model: &sdformat::Model,
        sim: &Simulation,
        plan: FootstepPlan,
    ) -> Result<Self, LoadError> {
        let invalid = |cause: &str| LoadError::JointPosture {
            entity: model.scoped_name.clone(),
            reason: cause.into(),
        };
        if !plan.valid()
            || sim.step_duration() != Duration::from_millis(1)
            || world.gravity[0] != 0.
            || world.gravity[1] != 0.
            || world.gravity[2] >= 0.
            || model.links.len() != 31
            || model.joints.len() != 30
            || model.self_collide
            || model
                .joints
                .iter()
                .any(|j| j.kind != sdformat::JointKind::Revolute)
        {
            return Err(invalid(
                "footsteps require the 30-axis floating humanoid, vertical gravity, a 1 ms interval and a bounded stride",
            ));
        }
        let mut widths = [0.; 2];
        for (side_index, side) in ["left", "right"].into_iter().enumerate() {
            let joint = |suffix| format!("{}::{side}_{suffix}", model.scoped_name);
            let center = |suffix| {
                sdf.pose_of(&joint(suffix), &world.name)
                    .map(|p| DVec3::from_array(p.translation))
            };
            let hip = center("hip_pitch")?;
            let knee = center("knee_pitch")?;
            let ankle = center("ankle_pitch")?;
            if (hip - knee - DVec3::Z * 0.377).length() > 1.0e-8
                || (knee - ankle - DVec3::Z * 0.371).length() > 1.0e-8
            {
                return Err(invalid(
                    "footstep profile requires upright 0.377 m thighs and 0.371 m shanks",
                ));
            }
            let foot = model
                .links
                .iter()
                .find(|l| l.name == format!("{side}_foot"))
                .ok_or_else(|| invalid("missing foot"))?;
            let [collision] = foot.collisions.as_slice() else {
                return Err(invalid("expected one support box per sole"));
            };
            let sdformat::Geometry::Box { size } = collision.geometry else {
                return Err(invalid("footstep profile requires box soles"));
            };
            let frame = sdf.resolve_pose(
                &collision.pose,
                &model.scoped_name,
                &foot.scoped_name,
                &foot.scoped_name,
            )?;
            if DVec3::from_array(frame.translation).length() > 1e-9
                || glam::DQuat::from_array(frame.rotation).angle_between(glam::DQuat::IDENTITY)
                    > 1e-9
            {
                return Err(invalid(
                    "expected centered sole collisions in the link frame",
                ));
            }
            widths[side_index] = size[1] * 0.5;
            if size[0] < 0.22 || size[1] < 0.104 {
                return Err(invalid(
                    "sole is too small for the planned center-of-pressure range",
                ));
            }
        }
        let mut floor_count = 0;
        let mut floor_name = None;
        for candidate in &world.models {
            if !candidate.is_static {
                continue;
            }
            for link in &candidate.links {
                for c in &link.collisions {
                    if let sdformat::Geometry::Plane { normal, .. } = c.geometry {
                        let p = sdf.resolve_pose(
                            &c.pose,
                            &candidate.scoped_name,
                            &link.scoped_name,
                            &world.name,
                        )?;
                        let up = glam::DQuat::from_array(p.rotation)
                            * DVec3::from_array(normal).normalize_or_zero();
                        if (up - DVec3::Z).length() > 1e-8 || p.translation[2].abs() > 1e-8 {
                            return Err(invalid(
                                "expected a horizontal ground plane at world height zero",
                            ));
                        }
                        floor_count += 1;
                        floor_name = Some(link.scoped_name.as_str());
                    }
                }
            }
        }
        if floor_count != 1 {
            return Err(invalid("footsteps require one physical horizontal floor"));
        }
        let mut joints = Vec::new();
        let mut level_sole_roll_limits = [[f64::NEG_INFINITY, f64::INFINITY]; 2];
        for &(name, _) in trajectory::AXES {
            let scoped = format!("{}::{name}", model.scoped_name);
            let j = model
                .joints
                .iter()
                .find(|j| j.name == name)
                .ok_or_else(|| invalid("missing selected gait joint"))?;
            let direction = DVec3::from_array(
                sdf.resolve_axis(
                    j.axis
                        .as_ref()
                        .ok_or_else(|| invalid("missing gait axis"))?,
                    &scoped,
                    &world.name,
                )?,
            );
            let expected = if name.ends_with("_roll") {
                DVec3::X
            } else {
                DVec3::Y
            };
            if (direction - expected).length() > 1e-8 {
                return Err(invalid(
                    "expected world-forward roll and leftward pitch axes in the initial stance",
                ));
            }
            if name.ends_with("_hip_roll") || name.ends_with("_ankle_roll") {
                let side = usize::from(name.starts_with("right_"));
                let axis = j
                    .axis
                    .as_ref()
                    .ok_or_else(|| invalid("missing gait roll axis"))?;
                let limit = if name.ends_with("_ankle_roll") {
                    [-axis.upper, -axis.lower]
                } else {
                    [axis.lower, axis.upper]
                };
                let bounds = &mut level_sole_roll_limits[side];
                bounds[0] = bounds[0].max(limit[0]);
                bounds[1] = bounds[1].min(limit[1]);
                if bounds[0] > 0. || bounds[1] < 0. {
                    return Err(invalid(
                        "hip and ankle roll stops must admit the initial level sole",
                    ));
                }
            }
            let h = sim
                .world()
                .iter_entities()
                .find_map(|e| {
                    (e.get::<Name>()?.0 == scoped)
                        .then(|| e.get::<JointHandle>().map(|j| j.id()))
                        .flatten()
                })
                .ok_or_else(|| invalid("missing native gait actuator"))?;
            joints.push((h, name));
        }
        let foot = |s| {
            sdf.pose_of(&format!("{}::{s}_foot", model.scoped_name), &world.name)
                .map(|pose| DVec3::from_array(pose.translation).truncate())
        };
        let body = |name: &str| {
            sim.world()
                .iter_entities()
                .find_map(|e| {
                    (e.get::<Name>()?.0 == name)
                        .then(|| e.get::<BodyHandle>().map(|b| b.id()))
                        .flatten()
                })
                .ok_or_else(|| invalid("missing gait support body"))
        };
        let foot_ids = [
            body(&format!("{}::left_foot", model.scoped_name))?,
            body(&format!("{}::right_foot", model.scoped_name))?,
        ];
        let heading = plan
            .heading_gain
            .filter(|gain| *gain > 0.)
            .map(|gain| {
                let pelvis = body(&format!("{}::pelvis", model.scoped_name))?;
                heading::HeadingControl::bind(sdf, model, world, sim, pelvis, foot_ids, gain)
            })
            .transpose()?;
        Ok(Self {
            foot_ids,
            heading,
            floor: body(floor_name.ok_or_else(|| invalid("missing physical ground"))?)?,
            sole_half_width: widths,
            level_sole_roll_limits,
            initial_feet: [foot("left")?, foot("right")?],
            plan,
            support: SupportControl::bind(sdf, model, sim)?,
            joints,
            gravity: -world.gravity[2],
        })
    }
    /// Set bounded targets and feedforward terms from the committed native pose at this input interval.
    pub fn drive(
        &self,
        ctl: &mut PostureController,
        be: &PhysicsBackend,
        tick: u64,
    ) -> Result<(), PhysicsError> {
        let mut target = trajectory::target(
            self.plan.motion,
            tick,
            self.plan.lateral_m,
            self.plan.length_m,
            self.plan.count,
        )?;
        if self.plan.placement_feedback && self.plan.motion == FootstepMotion::Walk && tick >= 7000
        {
            let feet = self.support.foot_centers(be)?;
            if self.plan.support_relative {
                target.support_relative(
                    feet,
                    self.initial_feet,
                    self.plan.lateral_correction_limit_m,
                );
            } else {
                target.track_feet(feet, self.initial_feet);
            }
        }
        if self.plan.recovery_landing && self.plan.motion != FootstepMotion::WeightShift {
            self.recover(&mut target, ctl, be)?;
        }
        target.limit_level_sole_roll(self.level_sole_roll_limits);
        if let Some(heading) = &self.heading {
            heading.apply(ctl, be, &target)?;
        }
        ctl.set_balance_offset(target.balance)?;
        let reference =
            ctl.reference_center_of_mass().truncate() + DVec2::from_array(target.balance);
        self.support.apply(
            ctl,
            be,
            reference,
            self.gravity,
            target.lifted.then_some(target.swing_foot),
        )?;
        for &(joint, name) in &self.joints {
            let reference = target.joint(name)?;
            ctl.set_target(joint, reference)?;
            if name.contains("_ankle_") {
                let swing = usize::from(name.starts_with("right_")) == target.swing_foot;
                let q = be.joint_state(joint).ok_or(PhysicsError::MissingJoint)?;
                ctl.set_direct_effort(
                    joint,
                    (target.lifted && swing)
                        .then_some(65. * (reference - q.position) - 1.6 * q.velocity),
                )?;
            }
        }
        Ok(())
    }
    pub(super) fn validate_actuators(&self, controller: &PostureController) -> bool {
        controller.axes().len() == 30
            && self
                .joints
                .iter()
                .all(|(joint, _)| controller.axes().iter().any(|axis| axis.joint == *joint))
    }
}
#[derive(Resource)]
struct Footsteps(BTreeMap<u64, FootstepController>);
pub(super) fn install(sim: &mut Simulation, controllers: BTreeMap<u64, FootstepController>) {
    if controllers.is_empty() {
        return;
    }
    sim.world_mut().insert_resource(Footsteps(controllers));
    sim.add_systems(
        Phase::PreStep,
        command
            .after(crate::scheduled_link_force::ScheduledForces)
            .before(PostureActuation),
    );
}
fn command(
    clock: Res<UpdateInfo>,
    programs: Res<Footsteps>,
    models: EntityLookup,
    mut states: ResMut<JointPostures>,
    backend: Res<PhysicsBackend>,
    fault: Res<StepFault>,
) {
    if clock.dt.is_zero() {
        return;
    }
    for (&id, rule) in &programs.0 {
        if models.get(id).is_none() {
            continue;
        }
        if let Some(controller) = states.get_mut(id)
            && let Err(error) = rule.drive(controller, &backend, clock.iteration - 1)
        {
            fault.report(error);
            break;
        }
    }
}
