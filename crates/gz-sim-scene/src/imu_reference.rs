// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Immutable IMU reference selection with validated world heading.
use super::{DQuat, LoadError};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Frame {
    Custom,
    CustomWorld([u64; 3]),
    Enu,
    Ned,
    Nwu,
    EnuHeading(u64),
    NedHeading(u64),
    NwuHeading(u64),
    CustomWorldHeading([u64; 3], u64),
}

pub(super) fn validate(
    config: &sdformat::Imu,
    spherical: Option<&sdformat::SphericalCoordinates>,
    entity: &str,
) -> Result<Frame, LoadError> {
    let bad = || LoadError::Unsupported {
        entity: entity.to_owned(),
        feature: "IMU orientation configuration",
    };
    let frame = if config.localization == "CUSTOM" && config.custom_rpy_parent_frame == "world" {
        if config
            .custom_rpy
            .iter()
            .any(|angle| !angle.is_finite() || angle.is_subnormal())
        {
            return Err(bad());
        }
        Frame::CustomWorld(config.custom_rpy.map(f64::to_bits))
    } else {
        if config.custom_rpy != [0.0; 3] || !config.custom_rpy_parent_frame.is_empty() {
            return Err(bad());
        }
        match config.localization.as_str() {
            // Legacy initial-reference admission never consumed spherical metadata.
            "CUSTOM" => return Ok(Frame::Custom),
            "ENU" => Frame::Enu,
            "NED" => Frame::Ned,
            "NWU" => Frame::Nwu,
            _ => return Err(bad()),
        }
    };
    let Some(world) = spherical else {
        return Ok(frame);
    };
    if world.surface != sdformat::SphericalSurface::EarthWgs84
        || world.orientation != sdformat::WorldFrameOrientation::EastNorthUp
        || [
            world.latitude_radians,
            world.longitude_radians,
            world.elevation,
        ]
        .iter()
        .any(|value| !value.is_finite() || *value != 0.0)
        || !world.heading_radians.is_finite()
        || world.heading_radians.is_subnormal()
        || world.heading_radians.abs() > std::f64::consts::PI
    {
        return Err(bad());
    }
    let heading = world.heading_radians.to_bits();
    Ok(match frame {
        Frame::Enu => Frame::EnuHeading(heading),
        Frame::Ned => Frame::NedHeading(heading),
        Frame::Nwu => Frame::NwuHeading(heading),
        Frame::CustomWorld(angles) => Frame::CustomWorldHeading(angles, heading),
        // All other variants are constructed only by this validated handoff.
        _ => return Err(bad()),
    })
}

// Fixed ENU-source table entries and world-heading yaw from ImuSensor.cc.
// Preserve Quaternion::SetFromEuler's scalar order and normalization. These
// constants have unit norm, so its near-zero fallback is outside this domain.
fn table_rotation(roll: f64, yaw: f64) -> [f64; 4] {
    let phi = roll / 2.0;
    let the = 0.0_f64 / 2.0;
    let psi = yaw / 2.0;
    let w = phi.cos() * the.cos() * psi.cos() + phi.sin() * the.sin() * psi.sin();
    let x = phi.sin() * the.cos() * psi.cos() - phi.cos() * the.sin() * psi.sin();
    let y = phi.cos() * the.sin() * psi.cos() + phi.sin() * the.cos() * psi.sin();
    let z = phi.cos() * the.cos() * psi.sin() - phi.sin() * the.sin() * psi.cos();
    let length = (w * w + x * x + y * y + z * z).sqrt();
    [x / length, y / length, z / length, w / length]
}

// Sensors constructs Quaternion(custom_rpy) before the world-reference product.
// Keep the existing named table operation path independent of this new input.
fn custom_rotation(angles: [u64; 3]) -> [f64; 4] {
    let [roll, pitch, yaw] = angles.map(f64::from_bits);
    let phi = roll / 2.0;
    let the = pitch / 2.0;
    let psi = yaw / 2.0;
    let w = phi.cos() * the.cos() * psi.cos() + phi.sin() * the.sin() * psi.sin();
    let x = phi.sin() * the.cos() * psi.cos() - phi.cos() * the.sin() * psi.sin();
    let y = phi.cos() * the.sin() * psi.cos() + phi.sin() * the.cos() * psi.sin();
    let z = phi.cos() * the.cos() * psi.sin() - phi.sin() * the.sin() * psi.cos();
    let length = (w * w + x * x + y * y + z * z).sqrt();
    [x / length, y / length, z / length, w / length]
}

pub(super) fn reference(frame: Frame, initial: DQuat) -> DQuat {
    let (q, heading) = match frame {
        Frame::CustomWorld(angles) => (custom_rotation(angles), 0.0),
        Frame::Enu => (table_rotation(0.0, 0.0), 0.0),
        Frame::Ned => (
            table_rotation(std::f64::consts::PI, std::f64::consts::FRAC_PI_2),
            0.0,
        ),
        Frame::Nwu => (table_rotation(0.0, std::f64::consts::FRAC_PI_2), 0.0),
        Frame::EnuHeading(bits) => (table_rotation(0.0, 0.0), f64::from_bits(bits)),
        Frame::NedHeading(bits) => (
            table_rotation(std::f64::consts::PI, std::f64::consts::FRAC_PI_2),
            f64::from_bits(bits),
        ),
        Frame::NwuHeading(bits) => (
            table_rotation(0.0, std::f64::consts::FRAC_PI_2),
            f64::from_bits(bits),
        ),
        Frame::CustomWorldHeading(angles, bits) => (custom_rotation(angles), f64::from_bits(bits)),
        // Return the exact pre-existing initial-sensor product.
        Frame::Custom => return initial,
    };
    // Sim supplies Quaternion(0,0,heading) in ENU; absent metadata keeps +0.
    // Sensors then evaluates worldRelativeOrientation * transformTable entry.
    let [x, y, z, w] = q;
    let [ax, ay, az, aw] = table_rotation(0.0, heading);
    DQuat::from_xyzw(
        aw * x + ax * w + ay * z - az * y,
        aw * y - ax * z + ay * w + az * x,
        aw * z + ax * y - ay * x + az * w,
        aw * w - ax * x - ay * y - az * z,
    )
}

#[cfg(test)]
#[path = "../tests/private/imu_reference.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/imu_reference_native.rs"]
mod native_tests;

#[cfg(test)]
#[path = "../tests/private/imu_custom_world_native.rs"]
mod custom_world_native_tests;

#[cfg(test)]
#[path = "../tests/private/imu_world_heading_native.rs"]
mod world_heading_native_tests;
