// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Authored torque-limited joint posture and horizontal center-of-mass feedback.
use crate::{BodyHandle, JointHandle, LoadError, Name, PhysicsEngine};
use bevy_ecs::prelude::*;
use glam::DVec3;
pub use gz_physics::posture::PostureMode;
use gz_physics::posture::{
    MassCenterFeedback, MassCenterSum, PostureAxisLaw, PostureFeedback as Feedback, PostureLaw,
    PostureSetpoint, PostureTargets,
};
use gz_physics::{BodyId, JointId, PhysicsBackend, PhysicsError};
use gz_sim_core::{EntityId, Phase, Simulation, StepFault, UpdateInfo, entity_by_id};
use std::collections::BTreeMap;

mod config;
pub mod footsteps;
use config::{AxisConfig, Plan};
pub(super) use config::{is_plugin, parse};

#[derive(Clone, Copy, Debug)]
pub struct PostureAxis {
    pub entity_id: u64,
    pub joint: JointId,
    pub max_effort: f64,
    stiffness: f64,
    damping: f64,
    goal: f64,
    initial_goal: f64,
    effort_override: Option<f64>,
    feedforward_nm: f64,
    limits: [f64; 2],
    feedback: Feedback,
}
impl PostureAxis {
    fn law(&self) -> PostureAxisLaw {
        PostureAxisLaw {
            joint_id: self.entity_id,
            reference_rad: self.initial_goal,
            stiffness: self.stiffness,
            damping: self.damping,
            max_effort_nm: self.max_effort,
            limits: self.limits,
            feedback: self.feedback,
        }
    }
    fn setting(&self) -> PostureSetpoint {
        PostureSetpoint {
            joint_id: self.entity_id,
            target_rad: self.goal,
            feedforward_nm: self.feedforward_nm,
            direct_effort_nm: self.effort_override,
        }
    }
}
#[derive(Clone, Copy)]
struct Segment {
    public: u64,
    body: BodyId,
    mass: f64,
    local_com: DVec3,
}
/// Command admitted for the previous native interval. Torque is in N·m.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PostureCommand {
    pub joint: JointId,
    pub target_rad: f64,
    pub effort_nm: f64,
}
/// A stateless feedback law bound to the original model identities and local inertias.
#[derive(Clone)]
pub struct PostureController {
    links: Vec<Segment>,
    axes: Vec<PostureAxis>,
    mass: f64,
    reference_com: DVec3,
    balance_offset: [f64; 2],
    gains: [f64; 2],
    max_correction: [f64; 2],
    mode: PostureMode,
    last: Vec<PostureCommand>,
}
impl PostureController {
    fn balance_law(&self) -> MassCenterFeedback {
        MassCenterFeedback {
            reference: self.reference_com.to_array(),
            gains: self.gains,
            max_correction: self.max_correction,
        }
    }
    /// Portable SDF law for an identically constructed worker with its own native handles.
    pub fn law(&self) -> PostureLaw {
        PostureLaw {
            link_order: self.links.iter().map(|l| l.public).collect(),
            axes: self.axes.iter().map(PostureAxis::law).collect(),
            balance: self.balance_law(),
        }
    }
    /// Current operator settings, including all changed references and independent feedforward.
    pub fn targets(&self) -> PostureTargets {
        PostureTargets {
            mode: self.mode,
            balance_offset: self.balance_offset,
            axes: self
                .axes
                .iter()
                .filter(|a| {
                    a.goal.to_bits() != a.initial_goal.to_bits()
                        || a.effort_override.is_some()
                        || a.feedforward_nm.to_bits() != 0
                })
                .map(PostureAxis::setting)
                .collect(),
        }
    }
    pub fn axes(&self) -> &[PostureAxis] {
        &self.axes
    }
    pub fn last_commands(&self) -> &[PostureCommand] {
        &self.last
    }
    pub fn mass(&self) -> f64 {
        self.mass
    }
    pub fn reference_center_of_mass(&self) -> DVec3 {
        self.reference_com
    }
    pub fn set_mode(&mut self, mode: PostureMode) {
        self.mode = mode;
    }
    /// Request a horizontal COM offset, in world meters relative to the authored standing pose.
    pub fn set_balance_offset(&mut self, offset: [f64; 2]) -> Result<(), PhysicsError> {
        if !offset.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        self.balance_offset = offset;
        Ok(())
    }
    /// Add an inverse-dynamics torque to the feedback command before the physical effort cap.
    pub fn set_feedforward(&mut self, joint: JointId, effort: f64) -> Result<(), PhysicsError> {
        if !effort.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        self.axes
            .iter_mut()
            .find(|a| a.joint == joint)
            .ok_or(PhysicsError::MissingJoint)?
            .feedforward_nm = effort;
        Ok(())
    }
    /// Select an explicit torque on one controlled axis. The authored physical cap still applies.
    /// Passing None resumes its position/velocity feedback at the current reference.
    pub fn set_direct_effort(
        &mut self,
        joint: JointId,
        command: Option<f64>,
    ) -> Result<(), PhysicsError> {
        let axis = self
            .axes
            .iter_mut()
            .find(|axis| axis.joint == joint)
            .ok_or(PhysicsError::MissingJoint)?;
        if command.is_some_and(|v| !v.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        axis.effort_override = command;
        Ok(())
    }
    /// Set an angle in radians on an authored actuator. Limits are checked before mutation.
    pub fn set_target(&mut self, joint: JointId, value: f64) -> Result<(), PhysicsError> {
        let axis = self
            .axes
            .iter_mut()
            .find(|a| a.joint == joint)
            .ok_or(PhysicsError::MissingJoint)?;
        if !value.is_finite() || value < axis.limits[0] || value > axis.limits[1] {
            return Err(PhysicsError::InvalidValue);
        }
        axis.goal = value;
        Ok(())
    }
    /// Inertial mass-weighted world position and COM velocity. Body origins may differ from their COMs.
    pub fn center_of_mass(&self, be: &PhysicsBackend) -> Result<(DVec3, DVec3), PhysicsError> {
        let mut sum = MassCenterSum::default();
        for l in &self.links {
            sum.add(
                l.mass,
                l.local_com.to_array(),
                be.state(l.body).ok_or(PhysicsError::MissingBody)?,
            );
        }
        let (p, v) = sum.finish(self.mass);
        Ok((p.into(), v.into()))
    }
    /// Evaluate all torques against one committed cut before any motor command is queued.
    pub fn commands(
        &self,
        be: &PhysicsBackend,
        out: &mut Vec<PostureCommand>,
    ) -> Result<(), PhysicsError> {
        out.clear();
        if self.mode == PostureMode::Passive {
            return Ok(());
        }
        let correction = if self.mode == PostureMode::Balance {
            let (p, v) = self.center_of_mass(be)?;
            self.balance_law()
                .correction(p.to_array(), v.to_array(), self.balance_offset)
        } else {
            [0.; 2]
        };
        for a in &self.axes {
            let q = be.joint_state(a.joint).ok_or(PhysicsError::MissingJoint)?;
            let [target, effort] = a.law().command(q, correction, Some(&a.setting()));
            be.validate_joint_effort(a.joint, effort)?;
            out.push(PostureCommand {
                joint: a.joint,
                target_rad: target,
                effort_nm: effort,
            });
        }
        Ok(())
    }
    /// Advance no clock. Queue one set of signed efforts and retain its readback.
    pub fn apply(&mut self, be: &mut PhysicsBackend) -> Result<(), PhysicsError> {
        let mut commands = std::mem::take(&mut self.last);
        let result = self.commands(be, &mut commands).and_then(|()| {
            #[cfg(feature = "distributed-physics")]
            if let Some(robot) =
                be.downcast_mut::<crate::distributed_hinge::DistributedGroundedRobot>()
            {
                return robot.stage_posture(&commands);
            }

            if let Some(native) =
                be.downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
            {
                for c in &commands {
                    if !(native.pending_joint_effort(c.joint).unwrap_or_default() + c.effort_nm)
                        .is_finite()
                    {
                        return Err(PhysicsError::InvalidValue);
                    }
                }
            }
            for c in &commands {
                be.add_joint_effort(c.joint, c.effort_nm)?;
            }
            Ok(())
        });
        if result.is_err() {
            commands.clear();
        }
        self.last = commands;
        result
    }
}
/// Scheduled before the physics step. A command producer may order an update before this set.
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct PostureActuation;
#[derive(Resource, Default)]
pub struct JointPostures(BTreeMap<u64, PostureController>);
impl JointPostures {
    pub fn get(&self, model: u64) -> Option<&PostureController> {
        self.0.get(&model)
    }
    pub fn get_mut(&mut self, model: u64) -> Option<&mut PostureController> {
        self.0.get_mut(&model)
    }
}
pub(super) fn install(sim: &mut Simulation, plans: Vec<Plan>,sdf:&sdformat::Sdf,world:&sdformat::World) -> Result<(), LoadError> {
    if plans.is_empty() {
        return Ok(());
    }
    let names: BTreeMap<_, _> = sim
        .world()
        .iter_entities()
        .filter_map(|e| {
            Some((
                e.get::<Name>()?.0.clone(),
                (e.get::<EntityId>()?.get(), e.id()),
            ))
        })
        .collect();
    let mut controllers = JointPostures::default();
    let mut walking=BTreeMap::new();
    for plan in plans {
        let error = |reason: &str| LoadError::JointPosture {
            entity: plan.model.clone(),
            reason: reason.into(),
        };
        let find = |name: &str| {
            names
                .get(name)
                .copied()
                .ok_or_else(|| error("missing constructed entity"))
        };
        let be = sim.world().resource::<PhysicsBackend>();
        let mut links = Vec::new();
        for (name, mass, local_com) in plan.links {
            let (public, entity) = find(&name)?;
            let h = sim
                .world()
                .get::<BodyHandle>(entity)
                .ok_or_else(|| error("missing link body"))?;
            links.push(Segment {
                public,
                body: h.id(),
                mass,
                local_com,
            });
        }
        let mut axes = Vec::new();
        for AxisConfig {
            name,
            goal,
            stiffness,
            damping,
            max_effort,
            feedback,
            limits,
        } in plan.joints
        {
            let (pub_id, e) = find(&name)?;
            let joint = sim
                .world()
                .get::<JointHandle>(e)
                .ok_or_else(|| error("missing native revolute"))?
                .id();
            be.validate_joint_effort(joint, 0.)?;
            axes.push(PostureAxis {
                entity_id: pub_id,
                joint,
                goal,
                initial_goal: goal,
                effort_override: None,
                feedforward_nm: 0.,
                stiffness,
                damping,
                max_effort,
                feedback,
                limits,
            });
        }
        let mass = links.iter().map(|s| s.mass).sum::<f64>();
        if !mass.is_finite() || mass <= 0. {
            return Err(error("total mass must be finite and positive"));
        }
        let mut controller = PostureController {
            links,
            axes,
            mass,
            reference_com: DVec3::ZERO,
            balance_offset: [0.; 2],
            gains: plan.gains,
            max_correction: plan.max_correction,
            mode: PostureMode::Balance,
            last: Vec::new(),
        };
        controller.reference_com = controller.center_of_mass(be)?.0;
        let owner=find(&plan.model)?.0;
        if let Some(program)=plan.footsteps {
            let model=world.models.iter().find(|m|m.scoped_name==plan.model).ok_or_else(||error("missing gait model"))?;
            let rule=footsteps::FootstepController::bind(sdf,world,model,sim,program)?;
            if !rule.validate_actuators(&controller){return Err(error("footstep profile needs all thirty named joint actuators"));}
            controller.set_mode(PostureMode::Posture);
            walking.insert(owner,rule);
        }
        controllers.0.insert(owner, controller);
    }
    sim.world_mut().insert_resource(controllers);
    footsteps::install(sim,walking);
    sim.add_systems(Phase::PreStep, act.in_set(PostureActuation));
    sim.add_systems(Phase::Reset, clear);
    Ok(())
}
fn act(world: &mut World) {
    if world.resource::<UpdateInfo>().dt.is_zero() {
        return;
    }
    world.resource_scope(|world, mut postures: Mut<JointPostures>| {
        for (&id, p) in &mut postures.0 {
            if entity_by_id(world, id).is_none() {
                continue;
            }
            if let Err(e) = p.apply(&mut world.resource_mut::<PhysicsBackend>()) {
                world.resource::<StepFault>().report(e);
                break;
            }
        }
    });
}
fn clear(mut postures: ResMut<JointPostures>) {
    for p in postures.0.values_mut() {
        p.last.clear();
        p.balance_offset = [0.; 2];
        for a in &mut p.axes {
            a.goal = a.initial_goal;
            a.effort_override = None;
            a.feedforward_nm = 0.;
        }
    }
}
