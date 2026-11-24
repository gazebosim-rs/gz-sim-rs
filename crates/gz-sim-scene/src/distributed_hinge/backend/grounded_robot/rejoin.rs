// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// Rejoin the original worker session from an explicitly checkpointed ordinary interval.
/// The physical image and clock must match the last local checkpoint exactly.
pub fn rejoin_grounded_robot_workers(sim: &mut Simulation, exe: &Path) -> Result<(), LoadError> {
    sim.validate_steps(0)?;
    let clock = sim.committed_update_info()?;
    let returned = sim
        .world()
        .get_resource::<LocalGroundedRobotSession>()
        .ok_or(PhysicsError::ForeignInitialState)?;
    let binding = RobotBindings::collect(sim, returned.0.spec())?;
    if sim.step_duration() != STEP
        || clock.sim_time != Duration::from_millis(clock.iteration)
        || !initial::matches_physics_owner(sim.world())
    {
        return Err(PhysicsError::ForeignInitialState.into());
    }
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
        .ok_or(PhysicsError::ForeignInitialState)?;
    let image = native.checkpoint_grounded_robot_tree(binding.root, binding.floor)?;
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
        .resource_mut::<LocalGroundedRobotSession>()
        .0
        .rejoin_workers_at(exe, clock.epoch, clock.iteration, &image);
    let pool = ownership_result(sim, result)?;
    let managed = binding.backend(pool);
    *sim.world_mut()
        .resource_mut::<initial::InitialScene>()
        .rebindable_physics()
        .expect("validated unique local reset blueprint") = managed.initial_state();
    sim.world_mut().insert_resource(managed);
    sim.world_mut()
        .remove_resource::<LocalGroundedRobotSession>();
    Ok(())
}
