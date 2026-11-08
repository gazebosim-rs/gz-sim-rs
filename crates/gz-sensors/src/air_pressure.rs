// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ideal default-atmosphere pressure in pascals at absolute sensor world Z.
// Constants/formula from gz-sensors src/AirPressureSensor.cc at
// 8ddf4c17be8a434c51b51d5be7c866fb69aaa54f (OSRF, Apache-2.0), which
// attributes its atmospheric model to ethz-asl/rotors_simulator's
// rotors_gazebo_plugins/{include/rotors_gazebo_plugins/gazebo_pressure_plugin.h,
// src/gazebo_pressure_plugin.cpp}. No atmosphere layers or noise are added here.

const EARTH_RADIUS: f64 = 6_356_766.0;
const SEA_LEVEL_TEMPERATURE: f64 = 288.15;
const TEMPERATURE_LAPSE: f64 = 0.0065;
const ONE_ATMOSPHERE: f64 = 101_325.0;
const EXPONENT: f64 = 9.80665 * 28.9644 / (8314.32 * -TEMPERATURE_LAPSE);

#[derive(Debug, Clone, Copy)]
pub struct AirPressureInput {
    pub world_z: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirPressureReading {
    pub pressure: f64,
    /// Canonical FluidPressure interprets zero as unknown variance.
    pub variance: f64,
}

#[derive(Debug, Clone, Copy, thiserror::Error, PartialEq, Eq)]
#[error("air-pressure height and derived atmosphere must be finite and physical")]
pub struct InvalidAirPressureInput;

/// Preserves the pinned native geopotential formula with reference altitude zero.
/// No initial-height subtraction, clamp, atmospheric layer or runtime reference.
pub fn air_pressure(
    input: AirPressureInput,
) -> Result<AirPressureReading, InvalidAirPressureInput> {
    let denominator = EARTH_RADIUS + input.world_z;
    let numerator = EARTH_RADIUS * input.world_z;
    if !input.world_z.is_finite()
        || !denominator.is_finite()
        || denominator <= 0.0
        || !numerator.is_finite()
    {
        return Err(InvalidAirPressureInput);
    }
    let geopotential_height = numerator / denominator;
    let temperature = SEA_LEVEL_TEMPERATURE - TEMPERATURE_LAPSE * geopotential_height;
    if !geopotential_height.is_finite() || !temperature.is_finite() || temperature <= 0.0 {
        return Err(InvalidAirPressureInput);
    }
    let pressure = ONE_ATMOSPHERE * (EXPONENT * (SEA_LEVEL_TEMPERATURE / temperature).ln()).exp();
    if !pressure.is_finite() || pressure <= 0.0 {
        return Err(InvalidAirPressureInput);
    }
    Ok(AirPressureReading {
        pressure,
        variance: 0.0,
    })
}
