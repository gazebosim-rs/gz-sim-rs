// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    ArticulationError, ArticulationPool, ArticulationSpec, HingeRecipe, MovingPool,
    MultiSpherePool, spec_and_sim,
};
use crate::{
    BodyHandle, JointHandle, LoadError, PhysicsEngine, WorldLoadOptions, initial, joints,
    load_world_at_clock, sync_attached_poses, sync_bodies, sync_local_poses, sync_models,
};
use bevy_ecs::prelude::*;
use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, InitialPhysicsState, JointId, JointState,
    PhysicsBackend, PhysicsError, PreparedPhysicsReset, RotationSpec,
};
use gz_physics_distributed::articulation::{ArticulationLinkState, ArticulationObservation};
use gz_sim_core::{Simulation, UpdateInfo, entity_by_id};
use std::{any::Any, path::Path, sync::Arc, time::Duration};

const ENGINE: &str = "distributed-hinge";
const STEP: Duration = Duration::from_millis(1);

fn physics_error(error: ArticulationError) -> PhysicsError {
    PhysicsError::Native {
        engine: ENGINE,
        message: error.to_string(),
    }
}

fn unsupported(capability: &'static str) -> PhysicsError {
    PhysicsError::Unsupported {
        engine: ENGINE,
        capability,
    }
}

/// Open a worker-owned SDF hinge in an ordinary paused Bevy simulation.
/// After reopening, simulation time and epoch match the durable barrier. Its
/// update duration is one millisecond at positive ticks and zero at tick zero.
/// Paused updates and pause settings are local to a running simulation.
/// Omitted width adopts the recorded width or starts with the minimum for that profile.
pub fn load_distributed_hinge_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
) -> Result<Simulation, LoadError> {
    load_distributed_hinge_world_with_transport(
        xml,
        source,
        world,
        worker,
        directory,
        width,
        super::WorkerTransport::Pipe,
    )
}

/// Select the managed RPC transport for the one- and two-companion profiles.
/// The floating and grounded two-link worker lanes already use loopback Zenoh.
pub fn load_distributed_hinge_world_with_transport(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
    transport: super::WorkerTransport,
) -> Result<Simulation, LoadError> {
    load_distributed_hinge_world_with_options(
        xml,
        source,
        world,
        worker,
        directory,
        HingeWorkerOptions {
            width,
            transport,
            ..Default::default()
        },
    )
}

/// Worker settings for opening or resuming a hinge session. These apply to the
/// current coordinator and are never stored in a physics checkpoint.
#[derive(Clone, Copy, Debug, Default)]
pub struct HingeWorkerOptions {
    /// Omit to reuse the saved worker count or choose the profile minimum.
    pub width: Option<u8>,
    /// Managed worker transport for the one- and two-companion profiles.
    pub transport: super::WorkerTransport,
    /// Additional reopen attempts per advancing interval after ordinary worker
    /// recovery is exhausted. Supported on the two-companion profile. Zero disables it.
    pub reopen_attempts: u8,
}

/// Open a worker-owned SDF hinge with an explicit coordinator recovery policy.
pub fn load_distributed_hinge_world_with_options(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    options: HingeWorkerOptions,
) -> Result<Simulation, LoadError> {
    let HingeWorkerOptions {
        width,
        transport,
        reopen_attempts,
    } = options;
    crate::require(
        reopen_attempts <= MultiSpherePool::MAX_REOPEN_ATTEMPTS,
        world,
        "worker reopen attempts exceed the multi-sphere limit",
    )?;
    let (recipe, initial_sim) = spec_and_sim(xml, source, world)?;
    crate::require(
        reopen_attempts == 0 || matches!(recipe, HingeRecipe::MultiSphere(_)),
        world,
        "automatic worker reopen requires a hinge and two free spheres",
    )?;
    match recipe {
        HingeRecipe::Tree(spec) => {
            let spec = *spec;
            let pool = ArticulationPool::open_scene(worker, directory, width, spec)
                .map_err(physics_error)?;
            let clock = (pool.observation().tick, pool.epoch());
            let sim = at_session_clock(xml, source, world, initial_sim, clock)?;
            install(sim, spec, &[], pool, clock)
        }
        HingeRecipe::Moving(spec) => {
            let hinge = spec.hinge;
            let sphere = spec.sphere.id;
            let pool =
                MovingPool::open_scene_with_transport(worker, directory, width, *spec, transport)
                    .map_err(physics_error)?;
            let clock = (pool.observation().hinge.tick, pool.epoch());
            let sim = at_session_clock(xml, source, world, initial_sim, clock)?;
            install(sim, hinge, &[sphere], pool, clock)
        }
        HingeRecipe::MultiSphere(spec) => {
            let pool = MultiSpherePool::open_scene_with_transport(
                worker, directory, width, *spec, transport,
            )
            .map_err(physics_error)?;
            install_convoy(xml, source, world, initial_sim, pool, reopen_attempts)
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum FencedHingeError {
    #[error(transparent)]
    Scene(#[from] LoadError),
    #[error(transparent)]
    Worker(#[from] ArticulationError),
}

impl FencedHingeError {
    /// Only ownership contention and worker connection loss allow another
    /// attempt against the same verified checkpoint.
    pub fn retryable(&self) -> bool {
        use std::io::ErrorKind::*;
        matches!(self, Self::Worker(ArticulationError::Io(e)) if matches!(e.kind(),
            WouldBlock | TimedOut | BrokenPipe | UnexpectedEof | ConnectionReset |
            ConnectionAborted | ConnectionRefused | NotConnected))
    }
}

mod cold;
pub use cold::{preview_selected_hinge_world, resume_selected_hinge_world};

/// Restore the exact authorized archive under its writer lease, and rehydrate
/// the ordinary Bevy world at the same epoch and tick.
pub fn resume_fenced_hinge_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    options: HingeWorkerOptions,
    receipt: &super::FenceReceipt,
) -> Result<Simulation, FencedHingeError> {
    let (recipe, sim) = spec_and_sim(xml, source, world)?;
    let HingeRecipe::MultiSphere(spec) = recipe else {
        return Err(LoadError::Unsupported {
            entity: world.into(),
            feature: "operator recovery requires a two-sphere convoy",
        }
        .into());
    };
    crate::require(
        options.reopen_attempts <= MultiSpherePool::MAX_REOPEN_ATTEMPTS,
        world,
        "worker reopen attempts exceed the multi-sphere limit",
    )?;
    let pool = MultiSpherePool::resume_scene_guarded(
        worker,
        directory,
        options.width,
        *spec,
        options.transport,
        receipt,
    )?;
    Ok(install_convoy(
        xml,
        source,
        world,
        sim,
        pool,
        options.reopen_attempts,
    )?)
}

fn install_convoy(
    xml: &str,
    source: &str,
    world: &str,
    initial: Simulation,
    mut pool: MultiSpherePool,
    attempts: u8,
) -> Result<Simulation, LoadError> {
    pool.set_reopen_attempts(attempts).map_err(physics_error)?;
    let clock = (pool.observation().hinge.tick, pool.epoch());
    let sim = at_session_clock(xml, source, world, initial, clock)?;
    let spheres = pool.spec().spheres.map(|s| s.id);
    install(sim, pool.spec().hinge, &spheres, pool, clock)
}

fn at_session_clock(
    xml: &str,
    source: &str,
    world: &str,
    initial_sim: Simulation,
    (tick, epoch): (u64, u64),
) -> Result<Simulation, LoadError> {
    let clock = UpdateInfo {
        iteration: tick,
        epoch,
        sim_time: Duration::from_millis(tick),
        start_time: Duration::from_millis(tick.saturating_sub(1)),
        dt: if tick == 0 { Duration::ZERO } else { STEP },
        paused: tick == 0,
    };
    let at_construction = tick == 0 && clock.epoch == 0;
    if at_construction {
        Ok(initial_sim)
    } else {
        drop(initial_sim);
        load_world_at_clock(
            xml,
            source,
            world,
            &sdformat::NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::MultiFidelity,
                step: None,
                noise_seed: [0; 32],
                distributed_worker: None,
            },
            Default::default(),
            Some((clock, true)),
        )
    }
}

fn install<P: SessionBackend>(
    mut sim: Simulation,
    spec: ArticulationSpec,
    sphere_ids: &[u64],
    pool: P,
    clock: (u64, u64),
) -> Result<Simulation, LoadError> {
    let body = |id| {
        entity_by_id(sim.world(), id)
            .and_then(|entity| sim.world().get::<BodyHandle>(entity))
            .map(BodyHandle::id)
            .ok_or(PhysicsError::MissingBody)
    };
    let bodies = [body(spec.links[0].id)?, body(spec.links[1].id)?];
    let spheres = sphere_ids
        .iter()
        .map(|&id| body(id))
        .collect::<Result<Vec<_>, _>>()?;
    let ground = spec
        .ground
        .map(|floor| -> Result<_, PhysicsError> {
            let id = body(floor.link_id)?;
            let state = sim
                .world()
                .resource::<PhysicsBackend>()
                .state(id)
                .ok_or(PhysicsError::MissingBody)?;
            Ok((id, state))
        })
        .transpose()?;
    let joint = entity_by_id(sim.world(), spec.joint_id)
        .and_then(|entity| sim.world().get::<JointHandle>(entity))
        .ok_or(PhysicsError::MissingJoint)?
        .id();
    sim.world_mut()
        .insert_resource(PhysicsBackend::from(DistributedHingeBackend {
            pool,
            bodies,
            ground,
            spheres,
            joint,
            step_effort: None,
            identity: Arc::new(()),
        }));
    finish_install(sim, clock)
}

fn finish_install(mut sim: Simulation, clock: (u64, u64)) -> Result<Simulation, LoadError> {
    // Capture construction poses under the new engine's reset identity before
    // publishing the worker's current poses and joint sample to the ECS.
    initial::capture(&mut sim);
    if clock == (0, 0) {
        // The loader already initialized exact authored poses and joint sample.
        // A zero-time pose roundtrip would perturb rotated local frames.
        return Ok(sim);
    }
    sync_session_poses(&mut sim);
    Ok(sim)
}

fn sync_session_poses(sim: &mut Simulation) {
    let mut publish = Schedule::default();
    publish.add_systems(
        (
            sync_bodies,
            sync_models,
            sync_local_poses,
            sync_attached_poses,
            joints::sample,
        )
            .chain(),
    );
    publish.run(sim.world_mut());
}

/// Bevy physics backend for one durably owned articulation. Stepping and
/// actuation use the normal scene APIs. Placement operations preserve the clock.
pub struct DistributedHingeBackend<P = ArticulationPool> {
    pool: P,
    bodies: [BodyId; 2],
    ground: Option<(BodyId, BodyState)>,
    spheres: Vec<BodyId>,
    joint: JointId,
    step_effort: Option<StepEffort>,
    identity: Arc<()>,
}

#[derive(Clone, Copy)]
struct StepEffort {
    torque: f64,
    expected: Option<(u64, u64)>,
}

/// Bevy host for a hinge and an independently owned sphere.
pub type DistributedMovingHingeBackend = DistributedHingeBackend<MovingPool>;

/// Bevy host for a floating hinge and two independently owned spheres.
pub type DistributedMultiSphereBackend = DistributedHingeBackend<MultiSpherePool>;

impl<P> DistributedHingeBackend<P> {
    pub fn pool(&self) -> &P {
        &self.pool
    }
}

/// Shared operations for the local hinge worker profiles.
pub trait SessionBackend: Send + Sync + 'static {
    fn hinge_sample(&self) -> &ArticulationObservation;
    fn sphere_sample(&self) -> Option<&ArticulationLinkState> {
        None
    }
    fn sphere_sample_at(&self, index: usize) -> Option<&ArticulationLinkState> {
        if index == 0 {
            self.sphere_sample()
        } else {
            None
        }
    }
    fn pending_joint_effort(&self) -> Option<f64> {
        None
    }
    fn queue_effort(&mut self, joint: u64, value: f64) -> Result<(), ArticulationError>;
    fn advance(&mut self) -> Result<(), ArticulationError>;
    /// Publish one advancing tick with the supplied torque and any already queued effort.
    fn advance_with_effort(&mut self, _joint: u64, _value: f64) -> Result<(), ArticulationError> {
        Err(ArticulationError::Invalid(
            "this profile has no atomic effort step",
        ))
    }
    fn check_interval(&self, _epoch: u64, _tick: u64) -> Result<(), ArticulationError> {
        Err(ArticulationError::Invalid(
            "this profile has no guarded step",
        ))
    }
    fn advance_with_effort_at(
        &mut self,
        epoch: u64,
        tick: u64,
        joint: u64,
        value: f64,
    ) -> Result<(), ArticulationError> {
        self.check_interval(epoch, tick)?;
        self.advance_with_effort(joint, value)
    }
    fn reset(&mut self) -> Result<(), ArticulationError>;
    fn transfer(&mut self, destination: u8) -> Result<(), ArticulationError>;
    /// Migrate an independently owned contact between the two free spheres.
    fn transfer_pair(&mut self, _destination: u8) -> Result<(), ArticulationError> {
        Err(ArticulationError::Invalid(
            "this profile has no independent sphere pair",
        ))
    }
    fn replace_worker(&mut self, slot: u8) -> Result<(), ArticulationError>;
}

macro_rules! session_controls {
    () => {
        fn pending_joint_effort(&self) -> Option<f64> {
            Self::pending_joint_effort(self)
        }
        fn advance_with_effort(&mut self, joint: u64, value: f64) -> Result<(), ArticulationError> {
            Self::advance_with_effort(self, joint, value)
        }
        fn queue_effort(&mut self, joint: u64, value: f64) -> Result<(), ArticulationError> {
            Self::queue_effort(self, joint, value)
        }
        fn advance(&mut self) -> Result<(), ArticulationError> {
            Self::advance(self)
        }
        fn check_interval(&self, epoch: u64, tick: u64) -> Result<(), ArticulationError> {
            Self::check_interval(self, epoch, tick)
        }
        fn advance_with_effort_at(
            &mut self,
            epoch: u64,
            tick: u64,
            joint: u64,
            value: f64,
        ) -> Result<(), ArticulationError> {
            Self::advance_with_effort_at(self, epoch, tick, joint, value)
        }
        fn reset(&mut self) -> Result<(), ArticulationError> {
            Self::reset(self)
        }
        fn transfer(&mut self, destination: u8) -> Result<(), ArticulationError> {
            Self::transfer(self, destination)
        }
        fn replace_worker(&mut self, slot: u8) -> Result<(), ArticulationError> {
            Self::replace_worker(self, slot)
        }
    };
}
impl SessionBackend for ArticulationPool {
    fn hinge_sample(&self) -> &ArticulationObservation {
        self.observation()
    }
    session_controls!();
}
impl SessionBackend for MovingPool {
    fn hinge_sample(&self) -> &ArticulationObservation {
        &self.observation().hinge
    }
    fn sphere_sample(&self) -> Option<&ArticulationLinkState> {
        Some(&self.observation().sphere)
    }
    session_controls!();
}
impl SessionBackend for MultiSpherePool {
    fn hinge_sample(&self) -> &ArticulationObservation {
        &self.observation().hinge
    }
    fn sphere_sample_at(&self, index: usize) -> Option<&ArticulationLinkState> {
        self.observation().spheres.get(index)
    }
    fn transfer_pair(&mut self, destination: u8) -> Result<(), ArticulationError> {
        Self::transfer_pair(self, destination)
    }
    session_controls!();
}
impl DistributedMultiSphereBackend {
    /// Observe solver recovery independently of the Bevy simulation thread.
    pub fn subscribe_recovery(&mut self) -> super::RecoveryMonitor {
        self.pool.subscribe_recovery()
    }

    /// Migrate the independently owned sphere pair at the current barrier.
    pub fn transfer_pair(&mut self, destination: u8) -> Result<(), ArticulationError> {
        self.pool.transfer_pair(destination)
    }
}
impl<P: SessionBackend> DistributedHingeBackend<P> {
    /// Supply one torque contribution for the next advancing Bevy physics step.
    /// The input is local until it is committed together with that step. Any
    /// effort already queued durably on the pool is accumulated first. Paused
    /// updates retain the staged input; reset or closing the host discards it.
    pub fn stage_step_effort(&mut self, joint: JointId, torque: f64) -> Result<(), PhysicsError> {
        self.stage_effort(joint, torque, None)
    }
    /// Supply one contribution for a particular durable interval. A reset creates
    /// a new epoch, so a delayed request cannot target the same tick after reset.
    pub fn stage_step_effort_at(
        &mut self,
        expected_epoch: u64,
        expected_tick: u64,
        joint: JointId,
        torque: f64,
    ) -> Result<(), PhysicsError> {
        self.pool
            .check_interval(expected_epoch, expected_tick)
            .map_err(physics_error)?;
        self.stage_effort(joint, torque, Some((expected_epoch, expected_tick)))
    }
    fn stage_effort(
        &mut self,
        joint: JointId,
        torque: f64,
        expected: Option<(u64, u64)>,
    ) -> Result<(), PhysicsError> {
        self.validate_joint_effort(joint, torque)?;
        if !(self.pool.pending_joint_effort().unwrap_or(0.) + torque).is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        if self.step_effort.is_some() {
            return Err(unsupported(
                "multiple staged contributions to one atomic effort step",
            ));
        }
        self.step_effort = Some(StepEffort { torque, expected });
        Ok(())
    }

    /// Migrate the complete coupled island at its current committed barrier.
    pub fn transfer(&mut self, destination: u8) -> Result<(), ArticulationError> {
        self.pool.transfer(destination)
    }
    pub fn replace_worker(&mut self, worker: u8) -> Result<(), ArticulationError> {
        self.pool.replace_worker(worker)
    }
}

// Both distributed articulation backends retain the same immutable authored graph
// and use the same reset identity protocol.
macro_rules! fixed_backend_methods {
    () => {
        fn initial_state(&self) -> InitialPhysicsState {
            InitialPhysicsState::new(&self.identity, ())
        }
        fn owns_initial_state(&self, initial: &InitialPhysicsState) -> bool {
            initial.is_owned_by(&self.identity)
        }
        fn prepare_initial(
            &self,
            initial: &InitialPhysicsState,
        ) -> Result<PreparedPhysicsReset, PhysicsError> {
            initial.data::<()>(&self.identity)?;
            Ok(PreparedPhysicsReset::new(&self.identity, ()))
        }
        fn apply_initial(&mut self, prepared: PreparedPhysicsReset) -> Result<(), PhysicsError> {
            prepared.into_data::<()>(&self.identity)?;
            self.reset()
        }

        fn spawn_link(
            &mut self,
            _: BodySpec,
            _: RotationSpec,
            _: Option<Inertia>,
        ) -> Result<BodyId, PhysicsError> {
            Err(unsupported("runtime link construction"))
        }
        fn remove(&mut self, _: BodyId) -> bool {
            false
        }
        fn remove_joint(&mut self, _: JointId) -> bool {
            false
        }
        fn set_world_velocity(
            &mut self,
            _: BodyId,
            _: [f64; 3],
            _: [f64; 3],
        ) -> Result<(), PhysicsError> {
            Err(unsupported("direct articulated velocity edits"))
        }
        fn apply_impulse(
            &mut self,
            _: BodyId,
            _: [f64; 3],
            _: [f64; 3],
        ) -> Result<(), PhysicsError> {
            Err(unsupported("link impulses"))
        }
        fn add_world_revolute(
            &mut self,
            _: BodyId,
            _: [f64; 3],
            _: [f64; 3],
        ) -> Result<JointId, PhysicsError> {
            Err(unsupported("runtime joint construction"))
        }
        fn add_revolute(
            &mut self,
            _: BodyId,
            _: BodyId,
            _: [f64; 3],
            _: [f64; 3],
        ) -> Result<JointId, PhysicsError> {
            Err(unsupported("runtime joint construction"))
        }
        fn add_reduced_revolute(
            &mut self,
            _: Option<BodyId>,
            _: BodyId,
            _: [f64; 3],
            _: [f64; 3],
        ) -> Result<JointId, PhysicsError> {
            Err(unsupported("runtime joint construction"))
        }
        fn set_self_collision_group(
            &mut self,
            _: BodyId,
            _: Option<u64>,
        ) -> Result<(), PhysicsError> {
            Err(unsupported("runtime collision filtering"))
        }
        fn as_any(&self) -> &dyn Any {
            self
        }
        fn as_any_mut(&mut self) -> &mut dyn Any {
            self
        }
    };
}
mod grasp_fleet;
pub use grasp_fleet::{
    DistributedGraspFleetBackend, GraspFleetLayout, ManagedGraspFleet,
    load_distributed_grasp_fleet_world,
};
mod grounded_robot;
pub(crate) use grounded_robot::remember_robot_program;
pub use grounded_robot::{
    DistributedGroundedRobot, LocalGroundedRobotSession, checkpoint_returned_grounded_robot,
    handoff_grounded_robot_to_workers, load_distributed_grounded_robot_world,
    load_returned_grounded_robot_world, rejoin_grounded_robot_workers,
    return_grounded_robot_to_local,
};
mod tree;
pub use tree::{
    DistributedFloatingTreeBackend, DistributedGraspTreeBackend, DistributedGroundedTreeBackend,
    DistributedMotorTreeBackend, DistributedPayloadTreeBackend, DistributedSplitGraspBackend,
    DistributedStoppedTreeBackend, DistributedTreeBackend, TreeSceneSession, WorkerTreeBackend,
    load_distributed_articulation_world, load_distributed_articulation_world_with_step,
    load_distributed_floating_tree_world, load_distributed_grasp_tree_world,
    load_distributed_grounded_tree_world, load_distributed_motor_tree_world,
    load_distributed_payload_tree_world, load_distributed_split_grasp_world,
    load_distributed_stopped_tree_world, load_distributed_tree_world,
};

impl<P: SessionBackend> DynamicBackend for DistributedHingeBackend<P> {
    fn add_world_wrench(
        &mut self,
        _: BodyId,
        _: [f64; 3],
        _: [f64; 3],
    ) -> Result<(), PhysicsError> {
        Err(unsupported("link wrenches"))
    }

    fn engine(&self) -> &'static str {
        ENGINE
    }

    fn state(&self, id: BodyId) -> Option<BodyState> {
        if let Some((ground, state)) = self.ground
            && id == ground
        {
            return Some(state);
        }
        let state = if let Some(index) = self.spheres.iter().position(|&body| body == id) {
            self.pool.sphere_sample_at(index)?
        } else {
            &self.pool.hinge_sample().links[self.bodies.iter().position(|&body| body == id)?]
        };
        Some(BodyState {
            position: state.position,
            velocity: state.velocity,
            orientation: state.orientation,
            angular_velocity: state.angular_velocity,
        })
    }

    fn joint_state(&self, id: JointId) -> Option<JointState> {
        let sample = self.pool.hinge_sample();
        (id == self.joint).then_some(JointState {
            position: sample.q,
            velocity: sample.qdot,
        })
    }

    fn is_dynamic(&self, id: BodyId) -> Option<bool> {
        if self.ground.is_some_and(|(ground, _)| ground == id) {
            Some(false)
        } else {
            (self.bodies.contains(&id) || self.spheres.contains(&id)).then_some(true)
        }
    }
    fn is_jointed(&self, id: BodyId) -> Option<bool> {
        self.is_dynamic(id).map(|_| self.bodies.contains(&id))
    }
    fn joint_child(&self, id: JointId) -> Option<BodyId> {
        (id == self.joint).then_some(self.bodies[1])
    }

    fn validate_joint_effort(&self, id: JointId, torque: f64) -> Result<(), PhysicsError> {
        if id != self.joint {
            return Err(PhysicsError::MissingJoint);
        }
        if !torque.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }

    fn add_joint_effort(&mut self, id: JointId, torque: f64) -> Result<(), PhysicsError> {
        self.validate_joint_effort(id, torque)?;
        if let Some(staged) = self.step_effort {
            let pending = self.pool.pending_joint_effort().unwrap_or(0.);
            if !((pending + torque) + staged.torque).is_finite() {
                return Err(PhysicsError::InvalidValue);
            }
        }
        self.pool
            .queue_effort(self.pool.hinge_sample().joint_id, torque)
            .map_err(physics_error)
    }

    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        if dt.is_zero() {
            return Ok(());
        }
        if dt != STEP {
            return Err(unsupported("hinge timestep other than 1 ms"));
        }
        if let Some(effort) = self.step_effort {
            let joint = self.pool.hinge_sample().joint_id;
            if let Some((epoch, tick)) = effort.expected {
                self.pool
                    .advance_with_effort_at(epoch, tick, joint, effort.torque)
            } else {
                self.pool.advance_with_effort(joint, effort.torque)
            }
            .map_err(physics_error)?;
            self.step_effort = None;
            Ok(())
        } else {
            self.pool.advance().map_err(physics_error)
        }
    }

    fn reset(&mut self) -> Result<(), PhysicsError> {
        self.pool.reset().map_err(physics_error)?;
        self.step_effort = None;
        Ok(())
    }

    fixed_backend_methods!();
}
