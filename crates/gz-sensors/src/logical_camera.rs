// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded model-origin LogicalCamera, from gz-math Frustum.cc / Plane.hh at
//! 1316ed334f4d5bda7be59aae07cd6e8ff31cfe13 and gz-sensors8ddf4c17.
//! This is the native point predicate, not AABB visibility or occlusion.
use glam::{DQuat, DVec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LogicalCameraConfig {
    pub near: f64,
    pub far: f64,
    pub horizontal_fov: f64,
    pub aspect_ratio: f64,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LogicalCameraPose {
    pub translation: [f64; 3],
    /// Quaternion order [x,y,z,w].
    pub rotation: [f64; 4],
}
#[derive(Debug, Clone, PartialEq)]
pub struct LogicalCameraModel {
    pub name: String,
    pub pose: LogicalCameraPose,
}
#[derive(Debug, Clone, PartialEq)]
pub struct LogicalCameraReading {
    pub pose: LogicalCameraPose,
    pub models: Vec<LogicalCameraModel>,
}
#[derive(Debug, Clone, Copy, thiserror::Error, PartialEq, Eq)]
#[error(
    "logical-camera configuration, poses, names and derived planes must be finite, bounded and nondegenerate"
)]
pub struct InvalidLogicalCameraInput;

pub fn valid_pose(pose: LogicalCameraPose) -> bool {
    pose.translation
        .iter()
        .chain(pose.rotation.iter())
        .all(|v| v.is_finite())
        && (pose.rotation.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() <= 1e-12
}
pub fn valid_name(name: &str) -> bool {
    !name.is_empty()
        && name.len() <= 65535
        && !name.contains("::")
        && !name
            .bytes()
            .any(|b| b.is_ascii_control() || b.is_ascii_whitespace())
}
impl LogicalCameraConfig {
    pub fn validate(self) -> Result<(), InvalidLogicalCameraInput> {
        planes(
            self,
            LogicalCameraPose {
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        )
        .map(|_| ())
    }
}
fn planes(
    config: LogicalCameraConfig,
    pose: LogicalCameraPose,
) -> Result<[(DVec3, f64); 6], InvalidLogicalCameraInput> {
    let c = config;
    if ![c.near, c.far, c.horizontal_fov, c.aspect_ratio]
        .iter()
        .all(|v| v.is_finite())
        || !(0.0 < c.near
            && c.near < c.far
            && 0.0 < c.horizontal_fov
            && c.horizontal_fov < std::f64::consts::PI
            && c.aspect_ratio > 0.0)
        || !valid_pose(pose)
    {
        return Err(InvalidLogicalCameraInput);
    }
    let tangent = (c.horizontal_fov * 0.5).tan();
    let near_width = 2.0 * tangent * c.near;
    let near_height = near_width / c.aspect_ratio;
    let far_width = 2.0 * tangent * c.far;
    let far_height = far_width / c.aspect_ratio;
    if ![near_width, near_height, far_width, far_height]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
    {
        return Err(InvalidLogicalCameraInput);
    }
    let q = DQuat::from_array(pose.rotation);
    let p = DVec3::from_array(pose.translation);
    let forward = q * DVec3::X;
    let up = q * DVec3::Z;
    let right = q * (-DVec3::Y);
    let nc = p + forward * c.near;
    let fc = p + forward * c.far;
    let un = up * (near_height * 0.5);
    let rn = right * (near_width * 0.5);
    let uf = up * (far_height * 0.5);
    let rf = right * (far_width * 0.5);
    let ntl = nc + un - rn;
    let ntr = nc + un + rn;
    let nbl = nc - un - rn;
    let nbr = nc - un + rn;
    let ftl = fc + uf - rf;
    let ftr = fc + uf + rf;
    let fbl = fc - uf - rf;
    let fbr = fc - uf + rf;
    if ![ntl, ntr, nbl, nbr, ftl, ftr, fbl, fbr]
        .iter()
        .all(|v| v.is_finite())
    {
        return Err(InvalidLogicalCameraInput);
    }
    let rows = [
        (ntl, ntr, nbl, nc),
        (ftr, ftl, fbl, fc),
        (ftl, ntl, nbl, (ftl + ntl + fbl + nbl) / 4.0),
        (ntr, ftr, fbr, (ftr + ntr + fbr + nbr) / 4.0),
        (ntl, ftl, ntr, (ftr + ntr + ftl + ntl) / 4.0),
        (nbl, nbr, fbr, (fbr + nbr + fbl + nbl) / 4.0),
    ];
    let mut result = [(DVec3::ZERO, 0.0); 6];
    for (out, (a, b, c, center)) in result.iter_mut().zip(rows) {
        let mut normal = (b - a).cross(c - a);
        let length = normal.length();
        if !length.is_finite() || length == 0.0 || !center.is_finite() {
            return Err(InvalidLogicalCameraInput);
        }
        // Native Vector3::Normalize leaves lengths equal-to-zero at its 1e-6
        // normalization threshold unscaled. There is NO visibility epsilon.
        if length > 1e-6 {
            normal /= length;
        }
        let offset = center.dot(normal);
        if !normal.is_finite() || !offset.is_finite() {
            return Err(InvalidLogicalCameraInput);
        }
        *out = (normal, offset);
    }
    Ok(result)
}

/// Sample every model origin, including the owner if visible; return lexical
/// bare-name order. Validate the WHOLE input before returning any partial frame.
pub fn logical_camera(
    config: LogicalCameraConfig,
    pose: LogicalCameraPose,
    models: &[LogicalCameraModel],
) -> Result<LogicalCameraReading, InvalidLogicalCameraInput> {
    let clip = planes(config, pose)?;
    if models.len() > 128 {
        return Err(InvalidLogicalCameraInput);
    }
    let mut sorted: Vec<_> = models.iter().collect();
    sorted.sort_by(|a, b| a.name.cmp(&b.name));
    if sorted
        .iter()
        .any(|m| !valid_name(&m.name) || !valid_pose(m.pose))
        || sorted.windows(2).any(|m| m[0].name == m[1].name)
    {
        return Err(InvalidLogicalCameraInput);
    }
    let inverse = DQuat::from_array(pose.rotation).inverse();
    let origin = DVec3::from_array(pose.translation);
    let mut selected = Vec::new();
    for model in sorted {
        let world = DVec3::from_array(model.pose.translation);
        let mut inside = true;
        for (normal, offset) in clip {
            let distance = normal.dot(world) - offset;
            if !distance.is_finite() {
                return Err(InvalidLogicalCameraInput);
            }
            if distance < 0.0 {
                inside = false;
            }
        }
        if inside {
            let relative = LogicalCameraPose {
                translation: (inverse * (world - origin)).to_array(),
                rotation: (inverse * DQuat::from_array(model.pose.rotation)).to_array(),
            };
            if !valid_pose(relative) {
                return Err(InvalidLogicalCameraInput);
            }
            selected.push(LogicalCameraModel {
                name: model.name.clone(),
                pose: relative,
            });
        }
    }
    Ok(LogicalCameraReading {
        pose,
        models: selected,
    })
}
