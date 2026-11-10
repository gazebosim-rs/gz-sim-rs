// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Immutable, validated IMU reference selection in the zero-heading domain.
use super::{DQuat, LoadError};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Frame {
    Custom,
    Enu,
    Ned,
    Nwu,
}

pub(super) fn validate(
    config: &sdformat::Imu,
    world_has_spherical: bool,
    entity: &str,
) -> Result<Frame, LoadError> {
    let bad = || LoadError::Unsupported {
        entity: entity.to_owned(),
        feature: "IMU orientation configuration",
    };
    if config.custom_rpy != [0.0; 3] || !config.custom_rpy_parent_frame.is_empty() {
        return Err(bad());
    }
    let frame = match config.localization.as_str() {
        "CUSTOM" => Frame::Custom,
        "ENU" => Frame::Enu,
        "NED" => Frame::Ned,
        "NWU" => Frame::Nwu,
        _ => return Err(bad()),
    };
    if frame != Frame::Custom && world_has_spherical {
        return Err(bad());
    }
    Ok(frame)
}

// Only the three fixed ENU-source table entries from ImuSensor.cc are needed.
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

pub(super) fn reference(frame: Frame, initial: DQuat) -> DQuat {
    let q = match frame {
        Frame::Enu => table_rotation(0.0, 0.0),
        Frame::Ned => table_rotation(std::f64::consts::PI, std::f64::consts::FRAC_PI_2),
        Frame::Nwu => table_rotation(0.0, std::f64::consts::FRAC_PI_2),
        // Return the exact pre-existing initial-sensor product.
        Frame::Custom => return initial,
    };
    // Sim supplies Quaternion(0,0,heading) in ENU, with heading fixed at +0.
    // Sensors then evaluates worldRelativeOrientation * transformTable entry.
    let [x, y, z, w] = q;
    let [ax, ay, az, aw] = table_rotation(0.0, 0.0);
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
