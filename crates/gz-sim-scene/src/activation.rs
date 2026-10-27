// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::Imu;
use gz_sim_core::{Simulation, StepError, entity_by_id};

#[derive(Debug, thiserror::Error)]
pub enum ImuActivationError {
    #[error("public IMU entity {0} does not exist")]
    MissingImu(u64),
    #[error(transparent)]
    Clock(#[from] StepError),
}

/// Reads acquisition state without advancing the sensor or simulation.
pub fn imu_active(sim: &Simulation, id: u64) -> Result<bool, ImuActivationError> {
    entity_by_id(sim.world(), id)
        .and_then(|entity| sim.world().get::<Imu>(entity))
        .map(|sensor| sensor.active)
        .ok_or(ImuActivationError::MissingImu(id))
}

/// Changes ordinary acquisition without resetting its deadline, history or noise.
/// Loaded-world reconstruction restores the initial state. This is a local API;
/// it neither changes transport registration nor creates a wire endpoint.
pub fn set_imu_active(
    sim: &mut Simulation,
    id: u64,
    active: bool,
) -> Result<(), ImuActivationError> {
    sim.validate_steps(0)?;
    let entity = entity_by_id(sim.world(), id).ok_or(ImuActivationError::MissingImu(id))?;
    let mut sensor = sim
        .world_mut()
        .get_mut::<Imu>(entity)
        .ok_or(ImuActivationError::MissingImu(id))?;
    sensor.active = active;
    Ok(())
}
