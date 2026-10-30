// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Conversion from authoritative Gazebo world poses to a Bevy-compatible frame.
//!
//! Gazebo uses a right-handed Z-up frame; Bevy uses a right-handed Y-up frame.
//! The conversion maps Gazebo +X to Bevy +X, +Y to -Z, and +Z to +Y.
//! Subtract the origin in f64 before narrowing to f32. The caller must use the
//! same origin for all objects and cameras in a frame.

pub mod camera;

use glam::{DQuat, DVec3, Quat, Vec3};

/// A world-space f64 pose. Quaternion components are in x, y, z, w order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldPose {
    pub translation: DVec3,
    pub rotation: DQuat,
}

/// Derived f32 position and orientation in the render frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RenderPose {
    pub translation: Vec3,
    pub rotation: Quat,
}

/// A pose cannot be represented safely in the render frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum ConversionError {
    #[error("origin and pose must be finite")]
    NonFinite,
    #[error("rotation must be a unit quaternion within 1e-12 squared-norm error")]
    NonUnitRotation,
    #[error("relative position is outside the finite f32 range")]
    PositionOutOfRange,
}

/// Convert a pose relative to a world-space origin, both in metres.
///
/// Rotation maps local Gazebo-frame vectors to world-frame vectors; the result
/// maps the corresponding local Bevy-frame vectors to the render frame.
/// Inputs must be finite and the quaternion squared norm must be within 1e-12
/// of one. Relative positions that overflow f64 or f32 are rejected.
pub fn to_render_pose(pose: WorldPose, origin: DVec3) -> Result<RenderPose, ConversionError> {
    if !origin.is_finite() || !pose.translation.is_finite() || !pose.rotation.is_finite() {
        return Err(ConversionError::NonFinite);
    }
    if (pose.rotation.length_squared() - 1.0).abs() > 1e-12 {
        return Err(ConversionError::NonUnitRotation);
    }
    let relative = pose.translation - origin;
    let translation = Vec3::new(relative.x as f32, relative.z as f32, -relative.y as f32);
    if !translation.is_finite() {
        return Err(ConversionError::PositionOutOfRange);
    }
    // Conjugating by the -90 degree X basis rotation maps both frames.
    let q = pose.rotation;
    let rotation = Quat::from_xyzw(q.x as f32, q.z as f32, -q.y as f32, q.w as f32).normalize();
    Ok(RenderPose {
        translation,
        rotation,
    })
}
