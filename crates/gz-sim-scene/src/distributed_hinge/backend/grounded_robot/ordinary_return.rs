// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Live ownership transfer into the ordinary MultiFidelityBackend.
use super::*;
#[path = "rejoin.rs"]
mod rejoin;
use gz_physics_distributed::grounded_robot::{ReturnedGroundedRobot, RobotLocalInputs};
pub use rejoin::rejoin_grounded_robot_workers;
use sha2::{Digest, Sha256};

/// Keeps this process's exclusive ownership of a returned robot session. The durable
/// tick is the last explicit release or checkpoint, independently of ordinary in-memory steps.
#[derive(Resource)]
pub struct LocalGroundedRobotSession(ReturnedGroundedRobot);

impl LocalGroundedRobotSession {
    /// Last acknowledged durable tick. Check publication_certain after a disk error.
    pub fn durable_tick(&self) -> u64 {
        self.0.tick()
    }
    pub fn durable_epoch(&self) -> u64 {
        self.0.epoch()
    }
    pub fn publication_certain(&self) -> bool {
        self.0.publication_certain()
    }
    pub fn generation(&self) -> u64 {
        self.0.generation()
    }
}

fn program_digest(xml: &str, world: &str) -> [u8; 32] {
    let mut hash = Sha256::new();
    hash.update(b"GZRB-SDF-1");
    hash.update((world.len() as u64).to_le_bytes());
    hash.update(world.as_bytes());
    hash.update(xml.as_bytes());
    hash.finalize().into()
}

#[derive(Resource)]
struct RobotProgramIdentity([u8; 32]);

pub(crate) fn remember_robot_program(sim: &mut Simulation, xml: &str, world: &sdformat::World) {
    if world.models.len() == 2
        && world.models.iter().any(|m| {
            !m.is_static && (3..=32).contains(&m.links.len()) && m.joints.len() + 1 == m.links.len()
        })
    {
        sim.world_mut()
            .insert_resource(RobotProgramIdentity(program_digest(xml, &world.name)));
    }
}

pub(super) fn matches_robot_program(sim: &Simulation, xml: &str, world: &str) -> bool {
    sim.world()
        .get_resource::<RobotProgramIdentity>()
        .is_some_and(|identity| identity.0 == program_digest(xml, world))
}

fn apply_to_ordinary(
    be: &mut PhysicsBackend,
    binding: &RobotBindings,
    spec: &GroundedRobotSpec,
    image: &[u8],
    inputs: &RobotLocalInputs,
) -> Result<(), PhysicsError> {
    let native = be
        .downcast_mut::<gz_physics_multifidelity::MultiFidelityBackend>()
        .ok_or(PhysicsError::ForeignInitialState)?;
    native.restore_grounded_robot_tree(binding.root, binding.floor, image)?;
    if native.checkpoint_grounded_robot_tree(binding.root, binding.floor)? != image {
        return Err(PhysicsError::InvalidValue);
    }
    let mut sorted: Vec<_> = spec.joints.iter().map(|j| j.joint_id).collect();
    sorted.sort_unstable();
    for &(id, effort) in &inputs.efforts {
        let row = sorted
            .binary_search(&id)
            .map_err(|_| PhysicsError::MissingJoint)?;
        let joint = binding
            .joints
            .iter()
            .find_map(|(&j, &(r, _))| (r == row).then_some(j))
            .ok_or(PhysicsError::MissingJoint)?;
        be.add_joint_effort(joint, effort)?;
    }
    for &(id, force) in &inputs.forces {
        if force != [0.; 3] {
            let body = binding.handles.get(&id).ok_or(PhysicsError::MissingBody)?;
            be.add_world_wrench(*body, force, [0.; 3])?;
        }
    }
    Ok(())
}

/// Reconstruct the complete loaded graph locally, fence its worker and atomically release
/// its durable authority. Existing Bevy entity identities, controller state, reset poses
/// and clock remain attached to the same running scene.
pub fn return_grounded_robot_to_local(
    sim: &mut Simulation,
    xml: &str,
    source: &str,
    world: &str,
) -> Result<(), LoadError> {
    sim.validate_steps(0)?;
    let clock = sim.committed_update_info()?;
    if sim.step_duration() != STEP
        || clock.iteration == 0
        || clock.sim_time != Duration::from_millis(clock.iteration)
        || !matches_robot_program(sim, xml, world)
        || !initial::matches_physics_owner(sim.world())
        || sim.world().contains_resource::<LocalGroundedRobotSession>()
    {
        return Err(PhysicsError::InvalidValue.into());
    }
    let (spec, mut cold) = grounded_robot_and_sim(xml, source, world)?;
    validate_robot_identities(sim, &cold)?;
    let bind = RobotBindings::collect(&cold, &spec)?;
    let mut local = cold
        .world_mut()
        .remove_resource::<PhysicsBackend>()
        .ok_or(PhysicsError::ForeignInitialState)?;
    let reset = local.initial_state();
    let managed = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<DistributedGroundedRobot>()
        .ok_or(PhysicsError::ForeignInitialState)?;
    if managed.pool.spec() != &spec {
        return Err(PhysicsError::ForeignInitialState.into());
    }
    let inputs = RobotLocalInputs {
        efforts: managed.posture.clone(),
        forces: managed
            .pending_forces
            .iter()
            .map(|(&id, &f)| (id, f))
            .collect(),
    };
    let image = managed
        .pool
        .native_image()
        .ok_or(PhysicsError::InvalidValue)?;
    apply_to_ordinary(&mut local, &bind, &spec, image, &inputs)?;
    if sim
        .world_mut()
        .resource_mut::<initial::InitialScene>()
        .rebindable_physics()
        .is_none()
    {
        return Err(PhysicsError::ForeignInitialState.into());
    }
    let result = sim
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<DistributedGroundedRobot>()
        .expect("same live managed backend")
        .pool
        .release_to_local_at(
            clock.epoch,
            clock.iteration,
            program_digest(xml, world),
            inputs,
        );
    let lease = ownership_result(sim, result)?;
    *sim.world_mut()
        .resource_mut::<initial::InitialScene>()
        .rebindable_physics()
        .expect("uniquely held reset blueprint checked before publication") = reset;
    sim.world_mut().insert_resource(local);
    sim.world_mut()
        .insert_resource(LocalGroundedRobotSession(lease));
    Ok(())
}

/// Reopen the most recent physical return or local checkpoint. The same authored program
/// reconstructs the Bevy systems. Runtime-only custom ECS resources belong to the caller.
pub fn load_returned_grounded_robot_world(
    xml: &str,
    source: &str,
    world: &str,
    directory: &Path,
) -> Result<Simulation, LoadError> {
    let lease = ReturnedGroundedRobot::resume(directory).map_err(physics_error)?;
    let (spec, cold) = grounded_robot_and_sim(xml, source, world)?;
    if &spec != lease.spec() || lease.source_digest() != program_digest(xml, world) {
        return Err(PhysicsError::ForeignInitialState.into());
    }
    let clock = (lease.tick(), lease.epoch());
    let mut sim = at_session_clock(xml, source, world, cold, clock)?;
    let bind = RobotBindings::collect(&sim, &spec)?;
    apply_to_ordinary(
        &mut sim.world_mut().resource_mut::<PhysicsBackend>(),
        &bind,
        &spec,
        lease.native_image(),
        lease.inputs(),
    )?;
    sync_session_poses(&mut sim);
    sim.world_mut()
        .insert_resource(LocalGroundedRobotSession(lease));
    Ok(sim)
}

/// Durably checkpoint an accepted ordinary interval while retaining local ownership.
/// The caller chooses the interval; intervening ordinary steps perform no disk I/O.
pub fn checkpoint_returned_grounded_robot(sim: &mut Simulation) -> Result<(), LoadError> {
    sim.validate_steps(0)?;
    let clock = sim.committed_update_info()?;
    let lease = sim
        .world()
        .get_resource::<LocalGroundedRobotSession>()
        .ok_or(PhysicsError::ForeignInitialState)?;
    let binding = RobotBindings::collect(sim, lease.0.spec())?;
    if !initial::matches_physics_owner(sim.world())
        || sim.step_duration() != STEP
        || clock.sim_time != Duration::from_millis(clock.iteration)
    {
        return Err(PhysicsError::ForeignInitialState.into());
    }
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
        .ok_or(PhysicsError::ForeignInitialState)?;
    let image = native.checkpoint_grounded_robot_tree(binding.root, binding.floor)?;
    let result = sim
        .world_mut()
        .resource_mut::<LocalGroundedRobotSession>()
        .0
        .checkpoint_native(clock.epoch, clock.iteration, &image);
    ownership_result(sim, result)
}
