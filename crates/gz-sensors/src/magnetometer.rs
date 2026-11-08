// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ideal constant-field magnetometer, matching gz-sensors8ddf4c17
//! src/MagnetometerSensor.cc: inverse sensor world rotation times world field.
use glam::{DQuat, DVec3};

/// SI inputs at acquisition; sensor-to-world quaternion uses [x, y, z, w].
#[derive(Debug, Clone, Copy)]
pub struct MagnetometerInput {
    pub world_orientation: [f64; 4],
    pub field_world: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagnetometerReading {
    pub field_tesla: [f64; 3],
}

#[derive(Debug, Clone, Copy, thiserror::Error, PartialEq, Eq)]
#[error("magnetometer inputs and rotated field must be finite, with a unit quaternion")]
pub struct InvalidMagnetometerInput;

/// A spatially uniform world vector expressed in the sensor frame, in Tesla.
/// There is no geographic lookup, translation term, reference subtraction or noise.
pub fn magnetometer(
    input: MagnetometerInput,
) -> Result<MagnetometerReading, InvalidMagnetometerInput> {
    let rotation = DQuat::from_array(input.world_orientation);
    let field = DVec3::from_array(input.field_world);
    if !rotation.is_finite()
        || (rotation.length_squared() - 1.0).abs() > 1e-12
        || !field.is_finite()
    {
        return Err(InvalidMagnetometerInput);
    }
    let local = rotation.inverse() * field;
    if !local.is_finite() {
        return Err(InvalidMagnetometerInput);
    }
    Ok(MagnetometerReading {
        field_tesla: local.to_array(),
    })
}
