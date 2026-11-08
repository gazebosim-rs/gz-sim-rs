// Copyright (C) 2023, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ideal AirSpeed numeric policy from gz-sensors AirSpeedSensor.cc at
//! 8ddf4c17be8a434c51b51d5be7c866fb69aaa54f (OSRF, Apache-2.0).
//! Native publishes a Pa-scale number after multiplying its hPa intermediate by
//! 100, despite the canonical protobuf comment saying hPa. No conversion is added.

#[derive(Debug, Clone, Copy)]
pub struct AirSpeedInput {
    /// Absolute sensor world height, not initial-height displacement.
    pub world_z: f64,
    /// Already in the composite sensor frame, at the actual sensor point.
    pub velocity_sensor: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirSpeedReading {
    /// Native published number: signed Pa scale, including valid reverse flow.
    pub diff_pressure: f64,
    pub temperature: f64,
}

#[derive(Debug, Clone, Copy, thiserror::Error, PartialEq, Eq)]
#[error("air-speed inputs and derived float atmosphere/pressure must be finite and physical")]
pub struct InvalidAirSpeedInput;

/// Zero-wind/no-noise native formula. Preserve its float atmosphere, mixed
/// float/double velocity product, float assignment and final float *100.
pub fn air_speed(input: AirSpeedInput) -> Result<AirSpeedReading, InvalidAirSpeedInput> {
    let height = input.world_z as f32;
    if !input.world_z.is_finite()
        || !height.is_finite()
        || !input.velocity_sensor.iter().all(|v| v.is_finite())
    {
        return Err(InvalidAirSpeedInput);
    }
    let temperature = 288.15_f32 - 0.0065_f32 * height;
    if !temperature.is_finite() || temperature <= 0.0 {
        return Err(InvalidAirSpeedInput);
    }
    let ratio = (288.15_f32 / temperature).powf(4.256_f32);
    let density = 1.225_f32 / ratio;
    if !ratio.is_finite() || ratio <= 0.0 || !density.is_finite() || density <= 0.0 {
        return Err(InvalidAirSpeedInput);
    }
    let x = input.velocity_sensor[0];
    let sign = if x > 0.0 {
        1.0_f32
    } else if x < 0.0 {
        -1.0_f32
    } else {
        0.0_f32
    };
    let coefficient = sign * 0.005_f32 * density;
    let first_product = f64::from(coefficient) * x;
    let second_product = first_product * x;
    let pressure = second_product as f32;
    let published = pressure * 100.0_f32;
    if !coefficient.is_finite()
        || !first_product.is_finite()
        || !second_product.is_finite()
        || !pressure.is_finite()
        || !published.is_finite()
    {
        return Err(InvalidAirSpeedInput);
    }
    Ok(AirSpeedReading {
        diff_pressure: f64::from(published),
        temperature: f64::from(temperature),
    })
}
