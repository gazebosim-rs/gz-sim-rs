// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ideal world-Z altimeter. Frame transforms, cadence and transport belong to the caller.

#[derive(Debug, Clone, Copy)]
pub struct AltimeterInput {
    pub world_z: f64,
    pub world_velocity_z: f64,
    pub vertical_reference: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltimeterReading {
    pub vertical_position: f64,
    pub vertical_velocity: f64,
    pub vertical_reference: f64,
}

#[derive(Debug, Clone, Copy, thiserror::Error, PartialEq, Eq)]
#[error("altimeter inputs and derived relative position must be finite")]
pub struct InvalidAltimeterInput;

/// Native ideal convention: position is relative to a fixed world-Z reference;
/// velocity is the sensor point's world-Z velocity, independent of sensor rotation.
pub fn altimeter(input: AltimeterInput) -> Result<AltimeterReading, InvalidAltimeterInput> {
    let relative = input.world_z - input.vertical_reference;
    if !input.world_z.is_finite()
        || !input.world_velocity_z.is_finite()
        || !input.vertical_reference.is_finite()
        || !relative.is_finite()
    {
        return Err(InvalidAltimeterInput);
    }
    Ok(AltimeterReading {
        vertical_position: relative,
        vertical_velocity: input.world_velocity_z,
        vertical_reference: input.vertical_reference,
    })
}
