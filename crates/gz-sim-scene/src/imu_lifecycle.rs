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
            .any(|entity| entity.contains::<Imu>())
    {
        return;
    }
    sim.add_systems(Phase::PostStep, sample_imus);
    sim.add_systems(Phase::Reset, reset_imus.after(sync_attached_poses));
    sim.world_mut().insert_resource(Installed);
}

pub(super) fn retire(sim: &mut Simulation) -> Result<(), bevy_ecs::schedule::ScheduleError> {
    if !sim.world().contains_resource::<Installed>() {
        return Ok(());
    }
    // Validate both schedules before changing either. These private callback
    // sets remain known even if a caller already retired a whole phase.
    sim.initialize_schedules()?;
    sim.remove_systems_in_set(Phase::PostStep, sample_imus)?;
    sim.remove_systems_in_set(Phase::Reset, reset_imus)?;
    sim.world_mut().remove_resource::<Installed>();
    sim.world_mut().remove_resource::<ImuInterest>();
    Ok(())
}

pub(super) fn retire_if_unused(
    sim: &mut Simulation,
) -> Result<(), bevy_ecs::schedule::ScheduleError> {
    if sim.world().contains_resource::<Installed>()
        && !sim
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<Imu>())
    {
        retire(sim)?;
    }
    Ok(())
}

#[cfg(test)]
#[path = "../tests/private/imu_lifecycle.rs"]
mod tests;
