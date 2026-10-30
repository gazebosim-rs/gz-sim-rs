// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Resource)]
pub(super) struct Installed;

pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<JointHandle>())
    {
        return;
    }
    sim.add_systems(
        Phase::PhysicsPrepare,
        joints::preflight.before(inputs::apply_joint_forces),
    );
    sim.add_systems(
        Phase::PhysicsCommit,
        joints::sample.after(sync_attached_poses),
    );
    sim.add_systems(Phase::Reset, joints::sample.after(sync_attached_poses));
    sim.world_mut().insert_resource(Installed);
}

pub(super) fn retire_if_unused(
    sim: &mut Simulation,
) -> Result<(), bevy_ecs::schedule::ScheduleError> {
    if !sim.world().contains_resource::<Installed>()
        || sim
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<JointHandle>())
    {
        return Ok(());
    }
    sim.initialize_schedules()?;
    sim.remove_systems_in_set(Phase::PhysicsPrepare, joints::preflight)?;
    sim.remove_systems_in_set(Phase::PhysicsCommit, joints::sample)?;
    sim.remove_systems_in_set(Phase::Reset, joints::sample)?;
    sim.world_mut().remove_resource::<Installed>();
    Ok(())
}

#[cfg(test)]
#[path = "../tests/private/joint_lifecycle.rs"]
mod tests;
