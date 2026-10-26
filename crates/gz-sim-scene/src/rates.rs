// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::Imu;
use gz_sim_core::{Simulation, StepError, entity_by_id};

#[derive(Debug, Clone, Copy)]
pub struct ImuRateInfo {
    /// Raw SDF rate; native values below 1e-6 disable the service ceiling.
    pub sdf_max_hz: f64,
    /// Effective runtime value, including native NaN/infinity controls.
    pub rate_hz: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RateChange {
    Applied,
    Ignored,
}

#[derive(Debug, thiserror::Error)]
pub enum ImuRateError {
    #[error("public IMU entity {0} does not exist")]
    MissingImu(u64),
    #[error("rate has no representable native sampling period")]
    InvalidPeriod,
    #[error(transparent)]
    Clock(#[from] StepError),
}

pub fn imu_rate(sim: &Simulation, id: u64) -> Result<ImuRateInfo, ImuRateError> {
    let sensor = entity_by_id(sim.world(), id)
        .and_then(|entity| sim.world().get::<Imu>(entity))
        .ok_or(ImuRateError::MissingImu(id))?;
    Ok(ImuRateInfo {
        sdf_max_hz: sensor.sdf_rate_hz,
        rate_hz: sensor.rate_hz,
    })
}

/// Applies the native capped rate-service policy without changing the pending
/// deadline, acquisition history or noise state. Ignored bounds keep all state.
/// Unrepresentable finite periods return an error before mutation. Unlike SDF
/// loading, native runtime control accepts NaN and positive infinity at no limit.
pub fn set_imu_rate(
    sim: &mut Simulation,
    id: u64,
    requested_hz: f64,
) -> Result<RateChange, ImuRateError> {
    sim.validate_steps(0)?;
    let entity = entity_by_id(sim.world(), id).ok_or(ImuRateError::MissingImu(id))?;
    let sensor = sim
        .world()
        .get::<Imu>(entity)
        .ok_or(ImuRateError::MissingImu(id))?;
    let rate = if requested_hz < 0.0 {
        0.0
    } else {
        requested_hz
    };
    // Preserve native lessOrNearEqual's strict epsilon boundary, including NaN.
    let near = |a: f64, b: f64| a < b + 1e-6;
    if !near(sensor.sdf_rate_hz, 0.0) && (near(rate, 0.0) || !near(rate, sensor.sdf_rate_hz)) {
        return Ok(RateChange::Ignored);
    }
    let mut schedule = sensor.schedule.clone();
    schedule
        .change_runtime_rate(rate)
        .map_err(|_| ImuRateError::InvalidPeriod)?;
    let mut sensor = sim.world_mut().get_mut::<Imu>(entity).expect("checked IMU");
    sensor.schedule = schedule;
    sensor.rate_hz = rate;
    Ok(RateChange::Applied)
}
