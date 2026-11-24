// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Closed-loop standing on two physical feet with a floating 30-DOF humanoid.
#[path = "support/humanoid_actuation.rs"]
mod humanoid_actuation;
#[path = "support/humanoid_gait.rs"]
mod humanoid_gait;
use gz_sim_scene::joint_posture::footsteps::{
    FootstepController, FootstepMotion, FootstepPlan, trajectory,
};
#[path = "support/stand_support.rs"]
mod stand_support;
#[path = "support/tree_visuals.rs"]
mod tree_visuals;
use bevy_ecs::prelude::*;
use clap::{Parser, ValueEnum};
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, JointId, PhysicsBackend, PhysicsError};
use gz_physics_multifidelity::{GroundReaction, MultiFidelityBackend, NativeGroundPatch};
use gz_sim_cli::live::{self, ViewerOptions};
use gz_sim_core::{EntityId, Phase, Simulation, StepFault, UpdateInfo};
use gz_sim_scene::distributed_hinge::{
    DistributedGroundedRobot, LocalGroundedRobotSession, checkpoint_returned_grounded_robot,
    handoff_grounded_robot_to_workers, load_distributed_grounded_robot_world,
    load_returned_grounded_robot_world, rejoin_grounded_robot_workers, return_grounded_robot_to_local,
};
use gz_sim_scene::joint_posture::{
    JointPostures, PostureActuation, PostureAxis, PostureController, PostureMode,
};
use gz_sim_scene::{BodyHandle, JointHandle, Name, PhysicsEngine};
use serde::Serialize;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
    time::{Duration, Instant},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/humanoid_stand.sdf");
const DT: Duration = Duration::from_millis(1);
#[derive(Debug, Clone, Copy, PartialEq, ValueEnum, Serialize)]
enum Control {
    Balance,
    Posture,
    Passive,
}
#[derive(Debug, Clone, Copy, PartialEq, ValueEnum, Serialize)]
enum Motion {
    Stand,
    Reach,
    Squat,
    WeightShift,
    SingleSupport,
    Step,
    Walk,
}
impl Motion {
    fn footstep(self) -> Option<FootstepMotion> {
        match self {
            Self::WeightShift => Some(FootstepMotion::WeightShift),
            Self::SingleSupport => Some(FootstepMotion::SingleSupport),
            Self::Step => Some(FootstepMotion::Step),
            Self::Walk => Some(FootstepMotion::Walk),
            _ => None,
        }
    }
}
#[derive(Debug, Clone, Copy, PartialEq, ValueEnum, Serialize)]
enum GaitControl {
    JointTargets,
    InverseStatics,
}
#[derive(Parser)]
struct Args {
    /// Write a standalone SDF with this authored footstep program and finish.
    #[arg(long, conflicts_with_all=["worker","session","web_viewer","trace","frames","replay_actuation","record_actuation"])]
    write_footstep_sdf: Option<PathBuf>,
    /// Run the ordinary SDF footstep system in this experiment, with its normal Bevy scheduling.
    #[arg(long,conflicts_with_all=["replay_actuation","write_footstep_sdf"])]
    authored_footsteps: bool,
    /// Include the chest-COM force in the ordinary SDF as a ScheduledLinkForce system.
    #[arg(long)]
    author_disturbance: bool,
    /// Use a pool of real CPU worker processes. The session stores the last accepted physical cut.
    #[arg(long, requires = "session")]
    worker: Option<PathBuf>,
    #[arg(long, requires = "worker")]
    session: Option<PathBuf>,
    #[arg(long,requires="worker",value_parser=clap::value_parser!(u8).range(2..=8))]
    workers: Option<u8>,
    /// Transfer this whole loaded graph immediately after the given absolute simulation tick.
    #[arg(long, requires = "worker")]
    migrate_at: Option<u64>,
    /// Start in the local solver, then adopt the accepted cut in a fresh worker session.
    #[arg(long, requires = "worker")]
    handoff_local_at: Option<u64>,
    /// Return the whole loaded graph to the ordinary local solver at this absolute tick.
    #[arg(long, requires = "worker")]
    return_local_at: Option<u64>,
    /// Resume the last durable local-owner cut without spawning a worker.
    #[arg(long, conflicts_with_all = ["worker","session","write_footstep_sdf"])]
    returned_session: Option<PathBuf>,
    /// Persist an accepted ordinary tick after returning from a worker.
    #[arg(long)]
    checkpoint_local_at: Option<u64>,
    /// Rejoin the same managed session at this explicitly checkpointed local cut.
    #[arg(long, requires = "checkpoint_local_at")]
    rejoin_workers_at: Option<u64>,
    /// Trusted physical worker for rejoining a session reopened with --returned-session.
    #[arg(long, requires = "rejoin_workers_at")]
    rejoin_worker: Option<PathBuf>,

    /// Hold the nominal pose, reach outward, or bend both knees then return.
    #[arg(long,value_enum,default_value_t=Motion::Stand)]
    motion: Motion,
    #[arg(long,value_enum,default_value_t=GaitControl::InverseStatics)]
    gait_control: GaitControl,
    /// Use the nominal foot centers throughout walking for a matched placement-feedback control.
    #[arg(long)]
    fixed_placement_reference: bool,
    /// Use measured footholds when computing planted-leg geometry during an alternating walk.
    #[arg(long,conflicts_with="fixed_placement_reference")]
    support_relative: bool,
    /// Maximum correction to the original lateral lane in one support-relative walking swing.
    #[arg(long, requires = "support_relative")]
    lateral_correction_limit_m: Option<f64>,
    /// Gain from measured heading error to bounded hip yaw during a support-relative walk.
    #[arg(long, requires = "support_relative")]
    heading_gain: Option<f64>,
    /// Land the swing foot early when the measured capture point approaches the inner stance edge.
    #[arg(long)]
    recovery_landing: bool,
    /// Number of alternating planned foot placements in walking mode.
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u16).range(1..=128))]
    steps: u16,
    /// Leftward COM reference displacement in weight-shift and stepping motions (m).
    #[arg(long, default_value_t = 0.100)]
    shift_m: f64,
    /// Forward displacement requested for the right foot in the single step (m).
    #[arg(long, default_value_t = 0.095)]
    step_m: f64,
    #[arg(long,value_enum,default_value_t=Control::Balance)]
    control: Control,
    /// Horizontal push at the chest COM, in newtons along world X.
    #[arg(long, default_value_t = 60., allow_hyphen_values = true)]
    push_x: f64,
    /// Horizontal push at the chest COM, in newtons along world Y.
    #[arg(long, default_value_t = 0., allow_hyphen_values = true)]
    push_y: f64,
    #[arg(long, default_value_t = 1000)]
    push_at: u64,
    #[arg(long, default_value_t = 80)]
    push_ticks: u64,
    #[arg(long, default_value_t = 5000)]
    ticks: u64,
    /// Record the exact bounded joint torques of each accepted feedback interval.
    #[arg(long, conflicts_with = "replay_actuation")]
    record_actuation: Option<PathBuf>,
    /// Apply a frozen joint torque sequence, independent of measured position and support.
    /// Interactive playback coasts with zero actuator effort outside the recorded interval.
    #[arg(long)]
    replay_actuation: Option<PathBuf>,
    /// Write physical measurements at this many intervals when --trace is given.
    #[arg(long,default_value_t=10,value_parser=clap::value_parser!(u64).range(1..))]
    trace_every: u64,
    /// Write a JSON line for the initial pose and each selected interval.
    #[arg(long)]
    trace: Option<PathBuf>,
    /// Record all WebSocket visual identities and poses per tick for external verification.
    #[arg(long)]
    frames: Option<PathBuf>,
    /// Start visual transform recording at this absolute physical tick.
    #[arg(long, default_value_t = 0)]
    frames_from: u64,
    /// Serve the interactive gz-web-rs view; --ticks chooses its starting simulation time.
    #[arg(long)]
    web_viewer: Option<u16>,
}
enum Support<'a> {
    Native(&'a MultiFidelityBackend),
    Worker(&'a DistributedGroundedRobot),
}
impl<'a> Support<'a> {
    fn from(be: &'a PhysicsBackend) -> std::result::Result<Self, PhysicsError> {
        if let Some(solver) = be.downcast_ref::<MultiFidelityBackend>() {
            return Ok(Self::Native(solver));
        }
        be.downcast_ref::<DistributedGroundedRobot>()
            .map(Self::Worker)
            .ok_or(PhysicsError::InvalidValue)
    }
    fn ground_reaction(
        &self,
        link: BodyId,
        floor: BodyId,
    ) -> std::result::Result<GroundReaction, PhysicsError> {
        match self {
            Self::Native(s) => s.ground_reaction(link, floor),
            Self::Worker(s) => s.ground_reaction(link, floor),
        }
    }
    fn fixed_ground_patches(
        &self,
        floor: BodyId,
    ) -> std::result::Result<Vec<NativeGroundPatch>, PhysicsError> {
        match self {
            Self::Native(s) => s.fixed_ground_patches(floor),
            Self::Worker(s) => s.fixed_ground_patches(floor),
        }
    }
}
#[derive(Clone, Resource)]
struct Stand {
    replay: Option<humanoid_actuation::Script>,
    interactive: bool,
    authored_footsteps: bool,
    authored_force: bool,
    support_control: Option<FootstepController>,
    model: u64,
    motion: Motion,
    steps: u16,
    shift_m: f64,
    step_m: f64,
    goals: Vec<(JointId, f64, &'static str)>,
    push: DVec3,
    at: u64,
    end: u64,
    segments: Vec<BodyId>,
    axes: Vec<PostureAxis>,
    mass: f64,
    gravity: DVec3,
    feet: [BodyId; 2],
    chest: BodyId,
    pelvis: BodyId,
    floor: BodyId,
    reference: DVec3,
}
impl Stand {
    fn resolve(sim: &mut Simulation, args: &Args) -> Result<Self> {
        let parsed = sdformat::load_str(SDF, "humanoid_stand.sdf", &sdformat::NoResources)?;
        let model = parsed.worlds[0]
            .models
            .iter()
            .find(|m| m.name == "humanoid")
            .ok_or("missing humanoid model")?;
        let name = |s: &str| format!("default::humanoid::{s}");
        let body = |full: &str| {
            sim.world()
                .iter_entities()
                .find_map(|e| {
                    if e.get::<Name>()?.0 == full {
                        Some(e.get::<BodyHandle>()?.id())
                    } else {
                        None
                    }
                })
                .ok_or_else(|| format!("missing body {full}"))
        };
        let segments = model
            .links
            .iter()
            .map(|l| body(&name(&l.name)))
            .collect::<std::result::Result<Vec<_>, _>>()?;
        let model_id = sim
            .world()
            .iter_entities()
            .find_map(|e| {
                (e.get::<Name>()?.0 == model.scoped_name)
                    .then(|| e.get::<EntityId>().map(|id| id.get()))
                    .flatten()
            })
            .ok_or("missing humanoid model")?;
        let controller = sim
            .world()
            .resource::<JointPostures>()
            .get(model_id)
            .ok_or("missing authored posture")?;
        let axes = controller.axes().to_vec();
        if axes.len() != 30 || segments.len() != 31 {
            return Err("expected 30 revolutes and 31 dynamic links".into());
        }
        let mass = controller.mass();
        let reference = controller.reference_center_of_mass();
        let poses: &[(&'static str, f64)] = match args.motion {
            Motion::Stand => &[],
            Motion::WeightShift | Motion::SingleSupport | Motion::Step | Motion::Walk => {
                trajectory::AXES
            }
            Motion::Reach => &[
                ("left_shoulder_roll", 1.15),
                ("right_shoulder_roll", -1.15),
                ("left_elbow_pitch", -0.38),
                ("right_elbow_pitch", -0.38),
                ("neck_yaw", 0.32),
            ],
            Motion::Squat => &[
                ("left_hip_pitch", -0.24),
                ("right_hip_pitch", -0.24),
                ("left_knee_pitch", 0.48),
                ("right_knee_pitch", 0.48),
                ("left_ankle_pitch", -0.24),
                ("right_ankle_pitch", -0.24),
                ("left_shoulder_pitch", -0.30),
                ("right_shoulder_pitch", -0.30),
            ],
        };
        let mut goals = Vec::new();
        for &(joint, target) in poses {
            let scoped = name(joint);
            let id = sim
                .world()
                .iter_entities()
                .find_map(|e| {
                    if e.get::<Name>()?.0 == scoped {
                        Some(e.get::<JointHandle>()?.id())
                    } else {
                        None
                    }
                })
                .ok_or("missing motion joint")?;
            goals.push((id, target, joint));
        }
        let inverse = matches!(
            args.motion,
            Motion::WeightShift | Motion::SingleSupport | Motion::Step | Motion::Walk
        ) && args.gait_control == GaitControl::InverseStatics;
        if (args.replay_actuation.is_some() || args.record_actuation.is_some())
            && args.control == Control::Passive
        {
            return Err("torque recording and replay require enabled actuators".into());
        }
        let stand = Self {
            interactive: args.web_viewer.is_some(),
            authored_footsteps: args.authored_footsteps,
            authored_force: args.author_disturbance,
            replay: args
                .replay_actuation
                .as_ref()
                .map(|p| humanoid_actuation::Script::read(p, model_id, &axes))
                .transpose()?,
            support_control: (inverse && !args.authored_footsteps)
                .then(|| {
                    FootstepController::bind(
                        &parsed,
                        &parsed.worlds[0],
                        model,
                        sim,
                        FootstepPlan {
                            placement_feedback: !args.fixed_placement_reference,
                            support_relative: args.support_relative,
                            lateral_correction_limit_m: args.lateral_correction_limit_m,
                            heading_gain: args.heading_gain,
                            recovery_landing: args.recovery_landing,
                            motion: args.motion.footstep().unwrap(),
                            lateral_m: args.shift_m,
                            length_m: args.step_m,
                            count: args.steps,
                        },
                    )
                })
                .transpose()?,
            model: model_id,
            motion: args.motion,
            shift_m: args.shift_m,
            steps: args.steps,
            step_m: args.step_m,
            goals,
            push: DVec3::new(args.push_x, args.push_y, 0.),
            at: args.push_at,
            end: args
                .push_at
                .checked_add(args.push_ticks)
                .ok_or("push time overflow")?,
            segments,
            axes,
            mass,
            gravity: DVec3::from_array(parsed.worlds[0].gravity),
            feet: [body(&name("left_foot"))?, body(&name("right_foot"))?],
            pelvis: body(&name("pelvis"))?,
            chest: body(&name("chest"))?,
            floor: body("default::floor::ground")?,
            reference,
        };
        if !stand.shift_m.is_finite()
            || !(0. ..=0.17).contains(&stand.shift_m)
            || !stand.step_m.is_finite()
            || !(0. ..=0.20).contains(&stand.step_m)
        {
            return Err("the leftward shift must be 0–0.17 m and the forward step 0–0.20 m".into());
        }
        if !stand.push.is_finite() {
            return Err("push must be finite".into());
        }
        let mode = match args.control {
            Control::Balance if inverse => PostureMode::Posture,
            Control::Balance => PostureMode::Balance,
            Control::Posture => PostureMode::Posture,
            Control::Passive => PostureMode::Passive,
        };
        sim.world_mut()
            .resource_mut::<JointPostures>()
            .get_mut(model_id)
            .unwrap()
            .set_mode(mode);
        Ok(stand)
    }
    fn pose(
        &self,
        ctl: &mut PostureController,
        be: &PhysicsBackend,
        t: u64,
    ) -> std::result::Result<(), PhysicsError> {
        if let Some(script) = &self.replay {
            if self.interactive && !script.contains(t) {
                for axis in &self.axes {
                    ctl.set_direct_effort(axis.joint, Some(0.))?;
                }
                return Ok(());
            }
            return script.apply(ctl, t, &self.axes);
        }
        if self.goals.is_empty() || self.authored_footsteps {
            return Ok(());
        }
        if matches!(
            self.motion,
            Motion::WeightShift | Motion::SingleSupport | Motion::Step | Motion::Walk
        ) {
            if let Some(rule) = &self.support_control {
                return rule.drive(ctl, be, t);
            }
            let target = trajectory::target(
                self.motion.footstep().expect("selected foot motion"),
                t,
                self.shift_m,
                self.step_m,
                self.steps,
            )?;
            ctl.set_balance_offset(target.balance)?;
            let native = Support::from(be)?;
            let (com, velocity) = ctl.center_of_mass(be)?;
            let mut ankle_y = [0.; 2];
            let mut loads = [0.; 2];
            for (side, &id) in self.feet.iter().enumerate() {
                let foot = be.state(id).ok_or(PhysicsError::MissingBody)?;
                let anchor = DVec3::from_array(foot.position)
                    + DQuat::from_array(foot.orientation) * DVec3::new(-0.025, 0., 0.047);
                ankle_y[side] = anchor.y;
                loads[side] =
                    native.ground_reaction(id, self.floor)?.normal_impulse / DT.as_secs_f64();
            }
            let efforts = humanoid_gait::roll_torques(
                com,
                velocity,
                ankle_y[1 - target.swing_foot],
                ankle_y,
                loads,
                self.mass,
                -self.gravity.z,
            );
            for &(joint, _, name) in &self.goals {
                ctl.set_target(joint, target.joint(name)?)?;
                if name.ends_with("ankle_roll") {
                    let side = usize::from(name.starts_with("right_"));
                    let single = (2350..5050).contains(&target.local_tick)
                        && self.motion != Motion::WeightShift
                        && loads[target.swing_foot] < 75.;
                    let torque = if !single {
                        None
                    } else if side != target.swing_foot {
                        efforts[side]
                    } else {
                        let q = be.joint_state(joint).ok_or(PhysicsError::MissingJoint)?;
                        Some(
                            (65. * (target.joint(name)? - q.position) - 1.6 * q.velocity)
                                .clamp(-12., 12.),
                        )
                    };
                    ctl.set_direct_effort(joint, torque)?;
                }
            }
            return Ok(());
        }
        let gain = match t {
            0..400 => 0.,
            400..1600 => 0.5 * (1. - (std::f64::consts::PI * (t - 400) as f64 / 1200.).cos()),
            1600..2500 => 1.,
            2500..3700 => 0.5 * (1. + (std::f64::consts::PI * (t - 2500) as f64 / 1200.).cos()),
            _ => 0.,
        };
        for &(joint, target, _) in &self.goals {
            ctl.set_target(joint, gain * target)?;
        }
        Ok(())
    }
    fn disturb(&self, be: &mut PhysicsBackend, t: u64) -> std::result::Result<(), PhysicsError> {
        if !self.authored_force && self.push != DVec3::ZERO && (self.at..self.end).contains(&t) {
            be.add_world_wrench(self.chest, self.push.to_array(), [0.; 3])?;
        }
        Ok(())
    }
    fn controller<'a>(&self, sim: &'a Simulation) -> &'a PostureController {
        sim.world()
            .resource::<JointPostures>()
            .get(self.model)
            .expect("bound model")
    }
}
fn control(
    clock: Res<UpdateInfo>,
    stand: Res<Stand>,
    mut be: ResMut<PhysicsBackend>,
    mut controllers: ResMut<JointPostures>,
    fault: Res<StepFault>,
) {
    if clock.dt.is_zero() {
        return;
    }
    let t = clock.iteration - 1;
    let ctl = controllers.get_mut(stand.model).expect("bound posture");
    if let Err(e) = stand
        .pose(ctl, &be, t)
        .and_then(|()| stand.disturb(&mut be, t))
    {
        fault.report(e);
    }
}
#[derive(Serialize)]
struct JointTracking {
    name: &'static str,
    target_rad: f64,
    actual_rad: f64,
    rate_rad_s: f64,
    effort_nm: f64,
}
#[derive(Serialize)]
struct Frame {
    tick: u64,
    com: [f64; 3],
    velocity: [f64; 3],
    pelvis_height: f64,
    tilt_rad: f64,
    feet_force_n: [f64; 2],
    other_force_n: f64,
    total_floor_impulse_ns: [f64; 3],
    max_abs_joint_rad: f64,
    foot_bottom_m: [f64; 2],
    foot_centers_m: [[f64; 3]; 2],
    capture_margin_m: Option<f64>,
    foot_center_of_pressure_m: Option<[f64; 2]>,
    foot_support_margin_m: Option<f64>,
    loaded_sole_points: [usize; 2],
    joint_error_rms_rad: f64,
    joint_error_max_rad: f64,
    max_command_fraction: f64,
    max_command_nm: f64,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    motion_joints: Vec<JointTracking>,
}
fn frame(sim: &Simulation, stand: &Stand, tick: u64) -> Result<Frame> {
    let be = sim.world().resource::<PhysicsBackend>();
    let native = Support::from(be)?;
    let controller = stand.controller(sim);
    let (p, v) = controller.center_of_mass(be)?;
    let pelvis = be.state(stand.pelvis).ok_or("missing pelvis")?;
    let chest = be.state(stand.chest).ok_or("missing chest")?;
    let up = DQuat::from_array(chest.orientation) * DVec3::Z;
    let mut forces = [0.; 2];
    let mut bottom = [0.; 2];
    let mut centers = [[0.; 3]; 2];
    for (i, &id) in stand.feet.iter().enumerate() {
        forces[i] = native.ground_reaction(id, stand.floor)?.normal_impulse / DT.as_secs_f64();
        let s = be.state(id).ok_or("missing foot")?;
        centers[i] = s.position;
        let q = DQuat::from_array(s.orientation);
        let mut lo = f64::INFINITY;
        for x in [-0.1275, 0.1275] {
            for y in [-0.058, 0.058] {
                lo = lo.min(s.position[2] + (q * DVec3::new(x, y, -0.05)).z);
            }
        }
        bottom[i] = lo;
    }
    let mut qmax = 0f64;
    let mut error_max = 0f64;
    let mut error_sq = 0.;
    let mut command_max = 0f64;
    let mut command_ratio = 0f64;
    let targets = controller.last_commands();
    for (i, a) in stand.axes.iter().enumerate() {
        let q = be.joint_state(a.joint).ok_or("missing joint")?.position;
        qmax = qmax.max(q.abs());
        let (target, torque) = targets
            .get(i)
            .map(|c| (c.target_rad, c.effort_nm))
            .unwrap_or_default();
        error_max = error_max.max((q - target).abs());
        error_sq += (q - target).powi(2);
        command_max = command_max.max(torque.abs());
        command_ratio = command_ratio.max(torque.abs() / a.max_effort);
    }
    let mut weight = 0.;
    let mut weighted = glam::DVec2::ZERO;
    let mut contacts = Vec::new();
    let mut counts = [0; 2];
    let mut other = 0.;
    let mut impulse = DVec3::ZERO;
    let mut last_link = None;
    for patch in native.fixed_ground_patches(stand.floor)? {
        if last_link != Some(patch.body) {
            let reaction = native.ground_reaction(patch.body, stand.floor)?;
            impulse +=
                DVec3::from_array(reaction.tangent_impulse) + DVec3::Z * reaction.normal_impulse;
            last_link = Some(patch.body);
        }
        let foot = stand.feet.iter().position(|id| *id == patch.body);
        for point in patch.points {
            if let Some(side) = foot {
                if point.normal_impulse > 0. {
                    let location = DVec3::from_array(point.plane_witness).truncate();
                    counts[side] += 1;
                    contacts.push(location);
                    weighted += point.normal_impulse * location;
                    weight += point.normal_impulse;
                }
            } else {
                other += point.normal_impulse / DT.as_secs_f64();
            }
        }
    }
    let mut motion_joints = Vec::with_capacity(stand.goals.len());
    for &(id, _, name) in &stand.goals {
        let joint = be.joint_state(id).ok_or("missing motion joint")?;
        let command = targets.iter().find(|c| c.joint == id);
        motion_joints.push(JointTracking {
            name,
            target_rad: command.map_or(0., |c| c.target_rad),
            actual_rad: joint.position,
            rate_rad_s: joint.velocity,
            effort_nm: command.map_or(0., |c| c.effort_nm),
        });
    }
    Ok(Frame {
        tick,
        com: p.to_array(),
        velocity: v.to_array(),
        pelvis_height: pelvis.position[2],
        tilt_rad: up.z.clamp(-1., 1.).acos(),
        feet_force_n: forces,
        other_force_n: other,
        total_floor_impulse_ns: impulse.to_array(),
        max_abs_joint_rad: qmax,
        foot_bottom_m: bottom,
        foot_centers_m: centers,
        capture_margin_m: stand_support::margin(
            contacts.clone(),
            p.truncate() + v.truncate() * (p.z.max(0.) / -stand.gravity.z).sqrt(),
        ),
        foot_center_of_pressure_m: (weight > 0.).then(|| (weighted / weight).to_array()),
        foot_support_margin_m: stand_support::margin(contacts, p.truncate()),
        loaded_sole_points: counts,
        joint_error_rms_rad: (error_sq / stand.axes.len() as f64).sqrt(),
        joint_error_max_rad: error_max,
        max_command_fraction: command_ratio,
        max_command_nm: command_max,
        motion_joints,
    })
}
#[derive(Serialize)]
struct Report {
    axes: usize,
    links: usize,
    mass_kg: f64,
    control: Control,
    motion: Motion,
    push_n: [f64; 3],
    push_at_tick: u64,
    push_ticks: u64,
    gait_control: GaitControl,
    open_loop_replay: bool,
    support_relative: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    lateral_correction_limit_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    heading_gain: Option<f64>,
    recovery_landing: bool,
    authored_footsteps: bool,
    lateral_reference_m: f64,
    requested_step_m: f64,
    first_fall: Option<u64>,
    min_foot_bottom_m: f64,
    max_torso_tilt_rad: f64,
    max_joint_angle_rad: f64,
    maximum_tracking_error_rad: f64,
    maximum_command_nm: f64,
    maximum_torque_cap_fraction: f64,
    joints_with_nonzero_torque: usize,
    minimum_loaded_foot_support_margin_m_after_100ms: Option<f64>,
    bilateral_ticks: u64,
    nonfoot_support_ticks: u64,
    last_half_second_mean_foot_force_n: [f64; 2],
    max_step_momentum_residual_ns: f64,
    rms_step_momentum_residual_ns: f64,
    max_cumulative_momentum_residual_ns: f64,
    cumulative_momentum_residual_ns: [f64; 3],
    worker: Option<serde_json::Value>,
    local_authority: Option<serde_json::Value>,
    ownership_changes: Vec<serde_json::Value>,
    physics_wall_ns: u64,
    p50_ns: Option<u64>,
    p95_ns: Option<u64>,
    p99_ns: Option<u64>,
    initial_com: [f64; 3],
    last: Frame,
}
fn footstep_sdf(args: &Args) -> Result<String> {
    let mode = match args.motion {
        Motion::WeightShift => "weight-shift",
        Motion::SingleSupport => "single-support",
        Motion::Step => "step",
        Motion::Walk => "walk",
        _ => {
            return Err(
                "choose weight-shift, single-support, step or walk for a footstep SDF".into(),
            );
        }
    };
    if args.gait_control != GaitControl::InverseStatics || args.control != Control::Balance {
        return Err("a footstep SDF uses the bounded physical feedback controller".into());
    }
    let count = if args.motion == Motion::Walk {
        args.steps
    } else {
        1
    };
    let heading = args.heading_gain.map(|v|format!(" heading_gain=\"{v}\"")).unwrap_or_default();
    let lane = args.lateral_correction_limit_m
        .map(|v| format!(" lateral_correction_limit_m=\"{v}\""))
        .unwrap_or_default();
    let command = format!(
        "      <footsteps motion=\"{mode}\" lateral_m=\"{}\" length_m=\"{}\" count=\"{count}\" placement_feedback=\"{}\" recovery_landing=\"{}\"{}{lane}{heading}/>\n",
        args.shift_m, args.step_m, !args.fixed_placement_reference, args.recovery_landing,
        if args.support_relative { " support_relative=\"true\"" } else { "" }
    );
    let mut xml = SDF.replacen("      <balance ", &(command + "      <balance "), 1);
    if args.author_disturbance {
        let begin = args.push_at;
        let end = begin
            .checked_add(args.push_ticks)
            .filter(|&v| v > begin)
            .ok_or("invalid authored force interval")?;
        if !args.push_x.is_finite() || !args.push_y.is_finite() {
            return Err("nonfinite authored chest force".into());
        }
        let force = format!(
            "    <plugin filename=\"gz-sim-scheduled-link-force-system\" name=\"gz::sim::systems::ScheduledLinkForce\">\n      <force link=\"default::humanoid::chest\" begin_tick=\"{begin}\" end_tick=\"{end}\" xyz=\"{} {} 0\"/>\n    </plugin>\n",
            args.push_x, args.push_y
        );
        xml = xml.replacen("    <plugin ", &(force + "    <plugin "), 1);
    }
    Ok(xml)
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.author_disturbance && !args.authored_footsteps && args.write_footstep_sdf.is_none() {
        return Err(
            "--author-disturbance requires --authored-footsteps or --write-footstep-sdf".into(),
        );
    }
    if let Some(path) = &args.write_footstep_sdf {
        let source = footstep_sdf(&args)?;
        gz_sim_scene::load_world_with_sdf_step(
            &source,
            "humanoid_walk.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        std::fs::write(path, source)?;
        println!("Wrote {}", path.display());
        return Ok(());
    }
    let authored = args
        .authored_footsteps
        .then(|| footstep_sdf(&args))
        .transpose()?;
    let xml = authored.as_deref().unwrap_or(SDF);
    let mut sim = match (&args.worker, &args.session) {
        (Some(_), Some(_)) if args.handoff_local_at.is_some() =>
            gz_sim_scene::load_world_with_sdf_step(xml, "humanoid_stand.sdf", "default", &sdformat::NoResources, PhysicsEngine::MultiFidelity)?,
        (Some(worker), Some(session)) => load_distributed_grounded_robot_world(
            xml,
            "humanoid_stand.sdf",
            "default",
            worker,
            session,
            args.workers,
        )?,
        (None,None) if args.returned_session.is_some() => load_returned_grounded_robot_world(
            xml, "humanoid_stand.sdf", "default", args.returned_session.as_deref().expect("matched returned session"),
        )?,
        (None, None) => gz_sim_scene::load_world_with_sdf_step(
            xml,
            "humanoid_stand.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?,
        _ => return Err("provide both the worker executable and the session directory".into()),
    };
    let start_tick = sim.world().resource::<UpdateInfo>().iteration;
    let end_tick = start_tick
        .checked_add(args.ticks)
        .ok_or("simulation time overflow")?;
    if args.handoff_local_at.is_some_and(|t| t == 0 || t > end_tick || args.migrate_at.is_some_and(|m| m < t)) {
        return Err("local adoption requires a positive input cut before any worker migration and within the run".into());
    }
    if args
        .migrate_at
        .is_some_and(|t| t <= start_tick || t > end_tick)
    {
        return Err("handoff must fall within the requested advancing intervals".into());
    }
    if args.return_local_at.is_some_and(|t| t <= start_tick || t > end_tick
        || args.handoff_local_at.is_some_and(|handoff| handoff >= t)
        || args.migrate_at.is_some_and(|migration| migration >= t))
    {
        return Err("local return must follow worker adoption and migration and fall within the run".into());
    }
    if args.checkpoint_local_at.is_some_and(|t| t <= start_tick || t > end_tick
        || args.returned_session.is_none() && args.return_local_at.is_none_or(|release| t <= release))
    {
        return Err("checkpoint requires a later accepted interval under local ownership".into());
    }
    if args.rejoin_workers_at.is_some_and(|t| Some(t) != args.checkpoint_local_at
        || args.rejoin_worker.is_none() && args.worker.is_none()) {
        return Err("rejoin requires the same explicit checkpoint tick and a trusted worker executable".into());
    }
    let mut ownership_changes = Vec::new();
    let stand = Stand::resolve(&mut sim, &args)?;
    sim.world_mut().insert_resource(stand.clone());
    sim.add_systems(Phase::PreStep, control.before(PostureActuation));
    if stand
        .replay
        .as_ref()
        .is_some_and(|s| !s.covers(start_tick, end_tick))
    {
        return Err("the recording does not cover every requested interval".into());
    }
    let mut recorder = args
        .record_actuation
        .as_ref()
        .map(|p| humanoid_actuation::Recorder::create(p, stand.model, &stand.axes))
        .transpose()?;
    if args.frames_from > 0 && (args.frames.is_none() || args.frames_from > end_tick) {
        return Err("--frames-from needs a visual recording with a cut inside the run".into());
    }
    let mut frames = args
        .frames
        .as_ref()
        .map(File::create)
        .transpose()?
        .map(BufWriter::new);
    let mut trace = args
        .trace
        .as_ref()
        .map(File::create)
        .transpose()?
        .map(BufWriter::new);
    let mut max_tilt = 0f64;
    let mut peak_error = 0f64;
    let mut lo = f64::INFINITY;
    let mut max_torque = 0f64;
    let mut max_fraction = 0f64;
    let mut tracking_max = 0f64;
    let mut stance_margin: Option<f64> = None;
    let mut coupled = 0u64;
    let mut nonfoot = 0u64;
    let mut fall = None;
    let mut load = [0.; 2];
    let mut hold = 0u64;
    let mut costs = Vec::new();
    let mut active_joints = std::collections::BTreeSet::new();
    let mut f = frame(&sim, &stand, start_tick)?;
    let mut residual = DVec3::ZERO;
    let mut residual_sq = 0.;
    let mut residual_max = 0f64;
    let mut cumulative_max = 0f64;
    let mut preceding = DVec3::from_array(f.velocity);
    for t in start_tick..=end_tick {
        if t > start_tick {
            let start = Instant::now();
            sim.step_exact(1)
                .map_err(|e| gz_sim_cli::with_fault(&sim, e))?;
            costs.push(start.elapsed().as_nanos() as u64);
            if let Some(out) = &mut recorder {
                out.append(t - 1, stand.controller(&sim).last_commands())?;
            }
            if Some(t) == args.handoff_local_at {
                let start = Instant::now();
                handoff_grounded_robot_to_workers(
                    &mut sim, xml, "humanoid_stand.sdf", "default",
                    args.worker.as_deref().expect("worker required"),
                    args.session.as_deref().expect("session required"), args.workers.unwrap_or(2),
                )?;
                ownership_changes.push(serde_json::json!({"tick":t,"operation":"to_workers","elapsed_ns":start.elapsed().as_nanos()}));
            }
            if Some(t) == args.migrate_at {
                let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
                let robot = backend
                    .downcast_mut::<DistributedGroundedRobot>()
                    .ok_or("missing managed robot")?;
                let next = (robot.pool().owner() + 1) % robot.pool().width();
                robot.transfer(next)?;
            }
            if Some(t) == args.return_local_at {
                let start = Instant::now();
                return_grounded_robot_to_local(&mut sim, xml, "humanoid_stand.sdf", "default")?;
                ownership_changes.push(serde_json::json!({"tick":t,"operation":"to_local","elapsed_ns":start.elapsed().as_nanos()}));
            }
            if Some(t) == args.checkpoint_local_at {
                let start = Instant::now();
                checkpoint_returned_grounded_robot(&mut sim)?;
                ownership_changes.push(serde_json::json!({"tick":t,"operation":"checkpoint_local","elapsed_ns":start.elapsed().as_nanos()}));
            }
            if Some(t) == args.rejoin_workers_at {
                let start = Instant::now();
                let exe = args.rejoin_worker.as_deref().or(args.worker.as_deref()).expect("validated worker");
                rejoin_grounded_robot_workers(&mut sim,exe)?;
                ownership_changes.push(serde_json::json!({"tick":t,"operation":"rejoin_workers","elapsed_ns":start.elapsed().as_nanos()}));
            }
            f = frame(&sim, &stand, t)?;
            let v = DVec3::from_array(f.velocity);
            let applied = if (stand.at..stand.end).contains(&(t - 1)) {
                stand.push
            } else {
                DVec3::ZERO
            };
            let external = (stand.mass * stand.gravity + applied) * DT.as_secs_f64()
                + DVec3::from_array(f.total_floor_impulse_ns);
            let error = stand.mass * (v - preceding) - external;
            residual += error;
            residual_sq += error.length_squared();
            residual_max = residual_max.max(error.length());
            cumulative_max = cumulative_max.max(residual.length());
            preceding = v;
        }
        if t >= args.frames_from && let Some(out) = &mut frames {
            let visuals = tree_visuals::poses(&sim)?.into_values().collect::<Vec<_>>();
            serde_json::to_writer(&mut *out, &serde_json::json!({"tick":t,"visuals":visuals}))?;
            writeln!(out)?;
        }
        for c in stand
            .controller(&sim)
            .last_commands()
            .iter()
            .filter(|c| c.effort_nm != 0.)
        {
            active_joints.insert(c.joint);
        }
        tracking_max = tracking_max.max(f.joint_error_max_rad);
        max_torque = max_torque.max(f.max_command_nm);
        max_fraction = max_fraction.max(f.max_command_fraction);
        if t >= 100
            && let Some(margin) = f.foot_support_margin_m
        {
            stance_margin = Some(stance_margin.map_or(margin, |v| v.min(margin)));
        }
        max_tilt = max_tilt.max(f.tilt_rad);
        peak_error = peak_error.max(f.max_abs_joint_rad);
        lo = lo.min(f.foot_bottom_m[0].min(f.foot_bottom_m[1]));
        if f.feet_force_n.iter().all(|v| *v > 1.) {
            coupled += 1;
        }
        if f.other_force_n > 1. {
            nonfoot += 1;
        }
        if fall.is_none() && (f.tilt_rad > 0.75 || f.pelvis_height < 0.65) {
            fall = Some(t);
        }
        if t >= end_tick.saturating_sub(499) && t > start_tick {
            for i in 0..2 {
                load[i] += f.feet_force_n[i];
            }
            hold += 1;
        }
        if let Some(out) = &mut trace
            && (t == start_tick || t % args.trace_every == 0 || t == end_tick)
        {
            serde_json::to_writer(&mut *out, &f)?;
            writeln!(out)?;
        }
    }
    if let Some(out) = &mut recorder {
        out.flush()?;
    }
    costs.sort_unstable();
    let cost = |p: usize| costs.get(costs.len().saturating_sub(1) * p / 100).copied();
    if let Some(out) = &mut trace {
        out.flush()?;
    }
    if let Some(out) = &mut frames {
        out.flush()?;
    }
    let record = Report {
        axes: stand.axes.len(),
        links: stand.segments.len(),
        mass_kg: stand.mass,
        control: args.control,
        motion: args.motion,
        push_n: stand.push.to_array(),
        push_at_tick: stand.at,
        push_ticks: args.push_ticks,
        gait_control: args.gait_control,
        open_loop_replay: stand.replay.is_some(),
        support_relative: args.support_relative,
        lateral_correction_limit_m: args.lateral_correction_limit_m,
        heading_gain: args.heading_gain,
        recovery_landing:args.recovery_landing,
        authored_footsteps:args.authored_footsteps,
        lateral_reference_m: args.shift_m,
        requested_step_m: args.step_m,
        first_fall: fall,
        min_foot_bottom_m: lo,
        max_torso_tilt_rad: max_tilt,
        max_joint_angle_rad: peak_error,
        maximum_tracking_error_rad: tracking_max,
        maximum_command_nm: max_torque,
        maximum_torque_cap_fraction: max_fraction,
        joints_with_nonzero_torque: active_joints.len(),
        minimum_loaded_foot_support_margin_m_after_100ms: stance_margin,
        bilateral_ticks: coupled,
        nonfoot_support_ticks: nonfoot,
        last_half_second_mean_foot_force_n: load.map(|x| x / hold.max(1) as f64),
        max_step_momentum_residual_ns: residual_max,
        rms_step_momentum_residual_ns: (residual_sq / args.ticks.max(1) as f64).sqrt(),
        max_cumulative_momentum_residual_ns: cumulative_max,
        cumulative_momentum_residual_ns: residual.to_array(),
        worker:sim.world().resource::<PhysicsBackend>().downcast_ref::<DistributedGroundedRobot>().map(|backend|{
            let p=backend.pool();
            serde_json::json!({"tick":p.tick(),"owner":p.owner(),"width":p.width(),"generation":p.generation(),"epoch":p.epoch()})
        }),
        local_authority:sim.world().get_resource::<LocalGroundedRobotSession>().map(|a| {
            serde_json::json!({"durable_tick":a.durable_tick(),"durable_epoch":a.durable_epoch(),"generation":a.generation()})
        }),
        ownership_changes,
        physics_wall_ns: costs.iter().sum(),
        p50_ns: cost(50),
        p95_ns: cost(95),
        p99_ns: cost(99),
        initial_com: stand.reference.to_array(),
        last: f,
    };
    println!("{}", serde_json::to_string_pretty(&record)?);
    if let Some(port) = args.web_viewer {
        live::run(
            sim,
            "default",
            false,
            0,
            None,
            ViewerOptions {
                port: Some(port),
                lifecycle_sdf: None,
                recording_path: None,
            },
        )?;
    }
    Ok(())
}

#[cfg(all(test, unix))]
#[path = "support/humanoid_adoption_tests.rs"]
mod adoption_tests;
#[cfg(all(test, unix))]
#[path = "support/humanoid_return_tests.rs"]
mod return_tests;

#[cfg(test)]
mod tests {
    use super::*;

    fn load() -> Result<Simulation> {
        Ok(gz_sim_scene::load_world_with_sdf_step(
            SDF,
            "humanoid_stand.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?)
    }
    pub(super) fn native(s: &Simulation) -> &MultiFidelityBackend {
        s.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
    }

    #[test]
    #[ignore = "requires a built production physics_worker via GZ_MULTI_TEST_WORKER"]
    fn bevy_posture_and_chest_load_use_one_durable_robot_interval() -> Result<()> {
        let exe = PathBuf::from(
            std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing GZ_MULTI_TEST_WORKER")?,
        );
        let dir = tempfile::tempdir()?;
        let args = Args::try_parse_from(["humanoid_stand", "--push-x", "140"])?;
        let setup = |mut sim: Simulation| -> Result<(Simulation, Stand)> {
            let stand = Stand::resolve(&mut sim, &args)?;
            sim.world_mut().insert_resource(stand.clone());
            sim.add_systems(Phase::PreStep, control.before(PostureActuation));
            Ok((sim, stand))
        };
        let open = || {
            load_distributed_grounded_robot_world(
                SDF,
                "humanoid_stand.sdf",
                "default",
                &exe,
                dir.path(),
                Some(2),
            )
        };
        let (mut physical, stand) = setup(load()?)?;
        let (mut worker, _) = setup(open()?)?;
        let mut loaded = 0;
        for tick in 0..1190 {
            if [0, 1049].contains(&tick) {
                for s in [&mut physical, &mut worker] {
                    s.world_mut()
                        .resource_mut::<PhysicsBackend>()
                        .add_joint_effort(stand.axes[5].joint, -0.125)?;
                }
                drop(worker);
                (worker, _) = setup(open()?)?;
                assert_eq!(worker.world().resource::<UpdateInfo>().iteration, tick);
            }
            if [680, 1051].contains(&tick) {
                let mut be = worker.world_mut().resource_mut::<PhysicsBackend>();
                let owned = be.downcast_mut::<DistributedGroundedRobot>().unwrap();
                owned.transfer(1 - owned.pool().owner())?;
            }
            physical
                .step_exact(1)
                .map_err(|e| gz_sim_cli::with_fault(&physical, e))?;
            worker
                .step_exact(1)
                .map_err(|e| gz_sim_cli::with_fault(&worker, e))?;
            assert_eq!(
                stand.controller(&physical).last_commands(),
                stand.controller(&worker).last_commands(),
                "controller {tick}"
            );
            let a = physical.world().resource::<PhysicsBackend>();
            let b = worker.world().resource::<PhysicsBackend>();
            for link in &stand.segments {
                assert_eq!(a.state(*link), b.state(*link), "link {link:?} {tick}");
            }
            for axis in &stand.axes {
                assert_eq!(
                    a.joint_state(axis.joint),
                    b.joint_state(axis.joint),
                    "axis {tick}"
                );
            }
            let owned = b.downcast_ref::<DistributedGroundedRobot>().unwrap();
            assert_eq!(owned.pool().tick(), tick + 1);
            let mut from_worker = owned.fixed_ground_patches(stand.floor)?;
            let mut from_native = native(&physical).fixed_ground_patches(stand.floor)?;
            from_worker.sort_by_key(|p| p.body.raw());
            from_native.sort_by_key(|p| p.body.raw());
            assert_eq!(from_worker, from_native, "sole witnesses at {tick}");
            if stand.feet.iter().all(|&foot| {
                owned
                    .ground_reaction(foot, stand.floor)
                    .unwrap()
                    .normal_impulse
                    > 0.01
            }) {
                loaded += 1;
            }
            if tick % 53 == 0 || (1048..1053).contains(&tick) {
                assert_eq!(
                    native(&physical)
                        .checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?
                        .as_slice(),
                    owned.pool().native_image().unwrap()
                );
            }
        }
        assert!(loaded > 1150);
        println!(
            "1190 bit-exact Bevy worker/ordinary intervals; {loaded} bilateral loads; 2 warm transfers and 2 held-effort process reopens; all body/joint commands and contact points checked."
        );
        Ok(())
    }

    #[test]
    fn ordinary_authored_walking_profile_runs_resets_and_rejects_unsuitable_robots() -> Result<()> {
        let args = Args::try_parse_from([
            "humanoid_stand",
            "--motion",
            "walk",
            "--steps",
            "2",
            "--shift-m",
            ".1",
            "--push-x",
            "0",
        ])?;
        let xml = footstep_sdf(&args)?;
        let load_authored = |xml: &str| {
            gz_sim_scene::load_world_with_sdf_step(
                xml,
                "humanoid_walk.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity,
            )
        };
        for input in [
            xml.replace("count=\"2\"", "count=\"129\""),
            xml.replace("lateral_m=\"0.1\"", "lateral_m=\"NaN\""),
            xml.replace(
                "<gravity>0 0 -9.81</gravity>",
                "<gravity>0 -1 -9.81</gravity>",
            ),
            xml.replace(
                "0.00000 0.00000 0.18500 0 0 0",
                "0.00000 0.00000 0.18000 0 0 0",
            ),
            xml.replace(
                "0.00000 0.00000 1.00000</xyz>",
                "0.00000 1.00000 0.00000</xyz>",
            )
            .replace(
                "1.00000 0.00000 0.00000</xyz>",
                "0.00000 1.00000 0.00000</xyz>",
            ),
            xml.replace("left_knee_pitch", "unexpected_knee"),
        ] {
            assert_ne!(input, xml);
            assert!(
                load_authored(&input).is_err(),
                "unsuitable footstep profile was accepted"
            );
        }
        let mut authored = load_authored(&xml)?;
        let mut controlled = load()?;
        let stand = Stand::resolve(&mut controlled, &args)?;
        controlled.world_mut().insert_resource(stand.clone());
        controlled.add_systems(Phase::PreStep, control.before(PostureActuation));
        for tick in 1..=12350 {
            authored.step_exact(1)?;
            controlled.step_exact(1)?;
            for b in &stand.segments {
                assert_eq!(
                    authored.world().resource::<PhysicsBackend>().state(*b),
                    controlled.world().resource::<PhysicsBackend>().state(*b),
                    "pose {tick}"
                );
            }
            assert_eq!(
                stand.controller(&authored).last_commands(),
                stand.controller(&controlled).last_commands(),
                "command {tick}"
            );
            if [1000, 2442, 3800, 4741, 7000, 9442, 10800, 11740, 12350].contains(&tick) {
                assert_eq!(
                    native(&authored).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
                    native(&controlled)
                        .checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
                    "native at {tick}"
                );
            }
        }
        gz_sim_scene::reset_loaded_world(&mut authored, |_| ())?;
        let mut fresh = load_authored(&xml)?;
        for _ in 0..120 {
            authored.step_exact(1)?;
            fresh.step_exact(1)?;
        }
        assert_eq!(
            native(&authored).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
            native(&fresh).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?
        );
        println!(
            "12,350 complete ordinary authored/manual Bevy intervals, both swing legs, full native cuts, cold reset, and 6 unsuitable input controls."
        );
        Ok(())
    }

    #[test]
    fn scheduled_sdf_push_and_early_foot_landing_match_the_physical_driver() -> Result<()> {
        let plain = [
            "humanoid_stand",
            "--motion",
            "step",
            "--recovery-landing",
            "--push-x",
            "0",
            "--push-y=-60",
            "--push-at",
            "3380",
            "--push-ticks",
            "80",
        ];
        let args = Args::try_parse_from(plain)?;
        let ordinary_args = Args::try_parse_from(
            plain
                .into_iter()
                .chain(["--author-disturbance", "--authored-footsteps"]),
        )?;
        let xml = footstep_sdf(&ordinary_args)?;
        let mut authored = gz_sim_scene::load_world_with_sdf_step(
            &xml,
            "recovery.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let mut manual = load()?;
        let stand = Stand::resolve(&mut manual, &args)?;
        manual.world_mut().insert_resource(stand.clone());
        manual.add_systems(Phase::PreStep, control.before(PostureActuation));
        for t in 1..=7000 {
            manual.step_exact(1)?;
            authored.step_exact(1)?;
            assert_eq!(
                stand.controller(&manual).last_commands(),
                stand.controller(&authored).last_commands(),
                "authored control at {t}"
            );
            for foot in stand.feet {
                assert_eq!(
                    native(&manual).ground_reaction(foot, stand.floor)?,
                    native(&authored).ground_reaction(foot, stand.floor)?,
                    "foot load at {t}"
                );
            }
            if [2442, 3380, 3460, 3600, 3810, 4950, 6200, 7000].contains(&t) {
                assert_eq!(
                    native(&manual).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
                    native(&authored).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
                    "whole graph at {t}"
                );
            }
        }
        assert!(frame(&authored, &stand, 7000)?.tilt_rad < 0.1);
        gz_sim_scene::reset_loaded_world(&mut authored, |_| ())?;
        gz_sim_scene::reset_loaded_world(&mut manual, |_| ())?;
        for t in 1..=3850 {
            authored.step_exact(1)?;
            manual.step_exact(1)?;
            assert_eq!(stand.controller(&authored).last_commands(), stand.controller(&manual).last_commands(), "restored program {t}");
        }
        assert_eq!(native(&authored).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
            native(&manual).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?, "replayed scheduled disturbance after entity reconstruction");
        println!(
            "7000 ordinary SDF/manual intervals with identical joint torques and sole reactions; eight full native images, before, inside and after the authored push and early landing."
        );
        Ok(())
    }
    #[test]
    fn frozen_torque_recording_reproduces_native_step_and_rejects_invalid_input() -> Result<()> {
        use std::collections::BTreeMap;
        let dir = tempfile::tempdir()?;
        let file = dir.path().join("efforts.jsonl");
        let movement = [
            "humanoid_stand",
            "--motion",
            "step",
            "--shift-m",
            ".1",
            "--push-x",
            "0",
        ];
        let args = Args::try_parse_from(movement)?;
        let setup = |args: &Args| -> Result<(Simulation, Stand)> {
            let mut sim = load()?;
            let stand = Stand::resolve(&mut sim, args)?;
            sim.world_mut().insert_resource(stand.clone());
            sim.add_systems(Phase::PreStep, control.before(PostureActuation));
            Ok((sim, stand))
        };
        let mut images = BTreeMap::new();
        let (mut sim, stand) = setup(&args)?;
        let mut record = humanoid_actuation::Recorder::create(&file, stand.model, &stand.axes)?;
        for t in 0..4900 {
            sim.step_exact(1)?;
            record.append(t, stand.controller(&sim).last_commands())?;
            if [2441, 3199, 3800, 4740, 4899].contains(&t) {
                images.insert(
                    t,
                    native(&sim).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
                );
            }
        }
        record.flush()?;
        let replay_args = Args::try_parse_from(
            movement
                .into_iter()
                .chain(["--replay-actuation", file.to_str().unwrap()]),
        )?;
        let (mut replay, stand) = setup(&replay_args)?;
        for t in 0..4900 {
            replay.step_exact(1)?;
            if let Some(reference) = images.get(&t) {
                assert_eq!(
                    &native(&replay).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
                    reference,
                    "actual full graph at {t}"
                );
            }
        }
        let source = std::fs::read_to_string(&file)?;
        let mut rows = source.lines();
        let header = rows.next().unwrap();
        let first = rows.next().unwrap();
        let corrupt = dir.path().join("bad.jsonl");
        for bad in [
            format!("{header}\n{first}\n{first}\n"),
            format!("{header}\n{{\"tick\":0,\"effort_bits\":[]}}\n"),
        ] {
            std::fs::write(&corrupt, bad)?;
            assert!(humanoid_actuation::Script::read(&corrupt, stand.model, &stand.axes).is_err());
        }
        println!(
            "4900 frozen vector intervals, five complete native graph comparisons across liftoff, swing and first landing."
        );
        Ok(())
    }

    #[cfg(unix)]
    #[test]
    #[ignore = "requires the production CPU worker via GZ_MULTI_TEST_WORKER"]
    fn physical_swing_and_touchdown_survive_owner_death_and_reopen() -> Result<()> {
        check_loaded_walking_worker(false)
    }
    #[cfg(unix)]
    #[test]
    #[ignore = "requires the production CPU worker via GZ_MULTI_TEST_WORKER"]
    fn early_recovery_landing_matches_the_ordinary_world_across_real_owner_deaths() -> Result<()> {
        check_loaded_walking_worker(true)
    }
    #[cfg(unix)]
    fn check_loaded_walking_worker(recovery: bool) -> Result<()> {
        use std::{fs, process::Command};
        let exe = PathBuf::from(
            std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing GZ_MULTI_TEST_WORKER")?,
        );
        let evidence = std::env::var_os(if recovery {
            "GZ_RECOVERY_WITNESS_DIR"
        } else {
            "GZ_GAIT_WITNESS_DIR"
        })
        .map(PathBuf::from);
        if let Some(dir) = &evidence {
            fs::create_dir_all(dir)?;
        }
        let dir = tempfile::tempdir()?;
        let mut fields = vec![
            "humanoid_stand",
            "--push-x",
            "0",
            "--motion",
            "step",
            "--shift-m",
            ".100",
        ];
        if recovery {
            fields.extend([
                "--recovery-landing",
                "--authored-footsteps",
                "--author-disturbance",
                "--push-at",
                "3380",
                "--push-y=-60",
                "--push-ticks",
                "80",
            ]);
        }
        let args = Args::try_parse_from(fields)?;
        let authored = recovery.then(|| footstep_sdf(&args)).transpose()?;
        let xml = authored.as_deref().unwrap_or(SDF);
        let bind = |mut sim: Simulation| -> Result<(Simulation, Stand)> {
            let stand = Stand::resolve(&mut sim, &args)?;
            sim.world_mut().insert_resource(stand.clone());
            sim.add_systems(Phase::PreStep, control.before(PostureActuation));
            Ok((sim, stand))
        };
        let open = || {
            load_distributed_grounded_robot_world(
                xml,
                "humanoid_stand.sdf",
                "default",
                &exe,
                dir.path(),
                Some(3),
            )
        };
        let (mut physical, stand) = bind(gz_sim_scene::load_world_with_sdf_step(
            xml,
            "humanoid_stand.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?)?;
        let (mut worker, _) = bind(open()?)?;
        let mut events = Vec::new();
        let mut bilateral = 0;
        let mut airborne = 0;
        for tick in 1..=7000 {
            physical
                .step_exact(1)
                .map_err(|e| gz_sim_cli::with_fault(&physical, e))?;
            worker
                .step_exact(1)
                .map_err(|e| gz_sim_cli::with_fault(&worker, e))?;
            let native = native(&physical);
            let ordinary = physical.world().resource::<PhysicsBackend>();
            let observed = worker.world().resource::<PhysicsBackend>();
            let robot = observed.downcast_ref::<DistributedGroundedRobot>().unwrap();
            assert_eq!(robot.pool().tick(), tick);
            for link in &stand.segments {
                assert_eq!(
                    ordinary.state(*link),
                    observed.state(*link),
                    "body at t{tick}"
                );
                assert_eq!(
                    native.ground_reaction(*link, stand.floor)?,
                    robot.ground_reaction(*link, stand.floor)?,
                    "reaction at t{tick}"
                );
            }
            for axis in &stand.axes {
                assert_eq!(
                    ordinary.joint_state(axis.joint),
                    observed.joint_state(axis.joint)
                );
            }
            assert_eq!(
                stand.controller(&physical).last_commands(),
                stand.controller(&worker).last_commands(),
                "torque at t{tick}"
            );
            let mut local = native.fixed_ground_patches(stand.floor)?;
            let mut managed = robot.fixed_ground_patches(stand.floor)?;
            local.sort_by_key(|p| p.body.raw());
            managed.sort_by_key(|p| p.body.raw());
            assert_eq!(local, managed, "warm sole patches at t{tick}");
            let left = robot
                .ground_reaction(stand.feet[0], stand.floor)?
                .normal_impulse;
            let right = robot
                .ground_reaction(stand.feet[1], stand.floor)?
                .normal_impulse;
            bilateral += usize::from(left > 0.001 && right > 0.001);
            airborne += usize::from(tick > 2000 && left > 0.1 && right == 0.);
            let name = match events.len() {
                0 if tick > 2350 && left > 0.35 && right == 0. => Some("release"),
                1 if recovery && tick == 3440 => Some("push"),
                1 if !recovery && tick == 3800 => Some("swing"),
                2 if tick > if recovery { 3460 } else { 4200 } && right > 0.001 => {
                    Some("touchdown")
                }
                3 if tick == 6200 => Some("recovered"),
                _ => None,
            };
            if name.is_some() || tick % 101 == 0 {
                assert_eq!(
                    native
                        .checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?
                        .as_slice(),
                    robot.pool().native_image().unwrap(),
                    "exact native graph t{tick}"
                );
            }
            if let Some(name) = name {
                let pool = robot.pool();
                let old = pool.owner();
                let pid = pool.worker_pid(old).ok_or("missing owner PID")?;
                if let Some(dst) = &evidence {
                    fs::copy(
                        dir.path().join("barrier.gzgroundedrobot"),
                        dst.join(format!("{name}.gzgroundedrobot")),
                    )?;
                    fs::write(
                        dst.join(format!("{name}.json")),
                        serde_json::to_vec_pretty(
                            &serde_json::json!({"tick":tick,"epoch":pool.epoch(),"generation":pool.generation(),"owner":old,"left_impulse_ns":left,"right_impulse_ns":right}),
                        )?,
                    )?;
                }
                if !Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
                {
                    return Err("could not kill the real CPU owner".into());
                }
                if events.len() % 2 == 0 {
                    drop(worker);
                    (worker, _) = bind(open()?)?;
                }
                let mut new = worker.world_mut().resource_mut::<PhysicsBackend>();
                let robot = new.downcast_mut::<DistributedGroundedRobot>().unwrap();
                robot.transfer((old + 1) % robot.pool().width())?;
                assert_eq!(robot.pool().tick(), tick);
                println!(
                    "{name} at {tick}: killed PID {pid}; owner {old} -> {}; left={left} right={right} N·s",
                    robot.pool().owner()
                );
                events.push((name, tick));
            }
        }
        assert_eq!(events.len(), 4);
        if recovery {
            assert!(
                events[2].1 < 4400,
                "recovery contact did not precede nominal landing"
            );
            assert!(
                frame(&physical, &stand, 7000)?.tilt_rad < 0.10,
                "failed to return upright"
            );
        }
        assert!(
            airborne > if recovery { 900 } else { 1900 } && bilateral > 3800,
            "airborne={airborne}, bilateral={bilateral}"
        );
        println!(
            "7000 complete ordinary / managed intervals, every public state, all joint efforts, all 31 reactions, each warm sole point; {airborne} airborne and {bilateral} bilateral, four real owner deaths/handoffs."
        );
        Ok(())
    }

    #[test]
    fn replay_under_foot_load_reopens_true_contact_caches_and_held_signed_torque() -> Result<()> {
        let mut sim = load()?;
        let args = Args::try_parse_from(["humanoid_stand", "--push-x", "140"])?;
        let stand = Stand::resolve(&mut sim, &args)?;
        sim.world_mut().insert_resource(stand.clone());
        sim.add_systems(Phase::PreStep, control.before(PostureActuation));
        let mut copy: Option<PhysicsBackend> = None;
        let mut controller = stand.controller(&sim).clone();
        let mut biggest = 0usize;
        let mut loaded = 0;
        for t in 0..1800 {
            if [5usize, 680, 1050, 1550].contains(&t) {
                let j = stand.axes[4].joint;
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .add_joint_effort(j, -0.125)?;
                let graph =
                    native(&sim).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?;
                assert_eq!(&graph[..8], b"GZHRBT01");
                assert!(graph.len() <= gz_physics_multifidelity::GROUNDED_ROBOT_IMAGE_LIMIT);
                biggest = biggest.max(graph.len());
                let mut fresh = load()?;
                let mut be = fresh
                    .world_mut()
                    .remove_resource::<PhysicsBackend>()
                    .unwrap();
                let target = be.downcast_mut::<MultiFidelityBackend>().unwrap();
                assert!(
                    target
                        .restore_grounded_robot_tree(stand.chest, stand.floor, &graph)
                        .is_err()
                );
                assert!(
                    target
                        .restore_grounded_robot_tree(
                            stand.pelvis,
                            stand.floor,
                            &graph[..graph.len() - 1]
                        )
                        .is_err()
                );
                let mut corrupt = graph.clone();
                corrupt[105] ^= 64;
                use sha2::{Digest, Sha256};
                let digest = Sha256::digest(&corrupt[40..]);
                corrupt[8..40].copy_from_slice(&digest);
                assert!(
                    target
                        .restore_grounded_robot_tree(stand.pelvis, stand.floor, &corrupt)
                        .is_err()
                );
                target.restore_grounded_robot_tree(stand.pelvis, stand.floor, &graph)?;
                assert_eq!(
                    target.checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
                    graph
                );
                assert_eq!(target.pending_joint_effort(j), Some(-0.125));
                println!(
                    "t={t} full graph={} bytes, sole corner counts={:?}",
                    graph.len(),
                    frame(&sim, &stand, t as u64)?.loaded_sole_points
                );
                copy = Some(be);
            }
            if let Some(be) = &mut copy {
                stand.disturb(be, t as u64)?;
                controller.apply(be)?;
                be.step(DT)?;
            }
            sim.step_exact(1)
                .map_err(|e| gz_sim_cli::with_fault(&sim, e))?;
            if let Some(be) = &copy {
                let ordinary = sim.world().resource::<PhysicsBackend>();
                for body in &stand.segments {
                    assert_eq!(ordinary.state(*body), be.state(*body));
                }
                for axis in &stand.axes {
                    assert_eq!(ordinary.joint_state(axis.joint), be.joint_state(axis.joint));
                }
                let before = native(&sim);
                let after = be.downcast_ref::<MultiFidelityBackend>().unwrap();
                assert_eq!(
                    before.fixed_ground_patches(stand.floor)?,
                    after.fixed_ground_patches(stand.floor)?
                );
                for link in &stand.segments {
                    assert_eq!(
                        before.ground_reaction(*link, stand.floor)?,
                        after.ground_reaction(*link, stand.floor)?
                    );
                }
                if t % 43 == 0 {
                    assert_eq!(
                        before.checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
                        after.checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?
                    );
                }
                let f = frame(&sim, &stand, t as u64 + 1)?;
                if f.loaded_sole_points.iter().all(|&n| n >= 3) {
                    loaded += 1;
                }
                if t > 25 {
                    assert_eq!(f.other_force_n, 0.);
                }
            }
        }
        println!(
            "loaded frames with at least three real contact points per sole={loaded}; max measured image={biggest}"
        );
        assert!(loaded > 1550);
        Ok(())
    }
}
