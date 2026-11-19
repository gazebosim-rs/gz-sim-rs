// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded pinhole calibration for a future demand-driven RGB renderer.
//!
//! Pixels have their origin at the upper left. Optical coordinates are +X
//! right, +Y down, +Z forward; a Gazebo camera uses +X forward, +Y left, +Z up.
//! This module does not create a camera, schedule sampling, or render an image.

use crate::{ConversionError, RenderPose, WorldPose, to_render_pose};
use glam::{DMat4, DVec3, DVec4, Mat4, Quat};

const MAX_DIMENSION: u32 = 4096;
const MAX_PIXELS: u64 = 4_194_304;

/// Explicit pinhole parameters in pixels and clip distances in metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraConfig {
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub skew: f64,
    pub tx: f64,
    pub ty: f64,
    pub near: f64,
    pub far: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum CameraError {
    #[error("image dimensions exceed the bounded RGB camera domain")]
    InvalidDimensions,
    #[error("intrinsics must be finite with positive focal lengths")]
    InvalidIntrinsics,
    #[error("skew must be finite and projection translation is not supported")]
    UnsupportedProjection,
    #[error("clip planes must be finite, positive and ordered")]
    InvalidClip,
    #[error("projection is not representable in the render frame")]
    ProjectionOutOfRange,
    #[error("optical point must be finite and in front of the camera")]
    InvalidPoint,
}

/// Validated camera configuration and its finite reverse-Z projection.
#[derive(Debug, Clone, Copy)]
pub struct Calibration {
    config: CameraConfig,
    projection: DMat4,
}

fn frustum_corners(config: CameraConfig, z_near: f64, z_far: f64) -> [DVec3; 8] {
    let plane = |z: f64| {
        let distance = z.abs();
        let left = -config.cx * distance / config.fx;
        let right = (f64::from(config.width) - config.cx) * distance / config.fx;
        let bottom = -(f64::from(config.height) - config.cy) * distance / config.fy;
        let top = config.cy * distance / config.fy;
        let corners = [
            DVec3::new(right, bottom, z),
            DVec3::new(right, top, z),
            DVec3::new(left, top, z),
            DVec3::new(left, bottom, z),
        ];
        if config.skew == 0.0 {
            corners
        } else {
            corners.map(|point| {
                DVec3::new(
                    point.x - (config.skew / config.fx) * point.y,
                    point.y,
                    point.z,
                )
            })
        }
    };
    let near = plane(z_near);
    let far = plane(z_far);
    [
        near[0], near[1], near[2], near[3], far[0], far[1], far[2], far[3],
    ]
}

impl Calibration {
    /// Validate finite signed skew and the bounded frustum before narrowing.
    /// Stereo projection translation remains unsupported.
    pub fn new(config: CameraConfig) -> Result<Self, CameraError> {
        if config.width == 0
            || config.height == 0
            || config.width > MAX_DIMENSION
            || config.height > MAX_DIMENSION
            || u64::from(config.width) * u64::from(config.height) > MAX_PIXELS
        {
            return Err(CameraError::InvalidDimensions);
        }
        if ![config.fx, config.fy, config.cx, config.cy]
            .iter()
            .all(|v| v.is_finite())
            || config.fx <= 0.0
            || config.fy <= 0.0
        {
            return Err(CameraError::InvalidIntrinsics);
        }
        if !config.skew.is_finite() || config.tx != 0.0 || config.ty != 0.0 {
            return Err(CameraError::UnsupportedProjection);
        }
        if !config.near.is_finite()
            || !config.far.is_finite()
            || config.near <= 0.0
            || config.far <= config.near
        {
            return Err(CameraError::InvalidClip);
        }
        let width = f64::from(config.width);
        let height = f64::from(config.height);
        let left = -config.cx * config.near / config.fx;
        let right = (width - config.cx) * config.near / config.fx;
        let bottom = -(height - config.cy) * config.near / config.fy;
        let top = config.cy * config.near / config.fy;
        if ![left, right, bottom, top].iter().all(|v| v.is_finite())
            || left >= right
            || bottom >= top
        {
            return Err(CameraError::ProjectionOutOfRange);
        }
        // Bevy's clip space is Y-up with depth 1 at near and 0 at far. Glam
        // supplies the asymmetric finite frustum; this changes only depth.
        let forward = glam::dcamera::rh::proj::directx::frustum(
            left,
            right,
            bottom,
            top,
            config.near,
            config.far,
        );
        let reverse = DMat4::from_cols(
            DVec4::X,
            DVec4::Y,
            DVec4::new(0.0, 0.0, -1.0, 0.0),
            DVec4::new(0.0, 0.0, 1.0, 1.0),
        );
        let mut projection = reverse * forward;
        if config.skew != 0.0 {
            projection.y_axis.x = 2.0 * config.skew / width;
        }
        let narrowed = projection.as_mat4();
        let far_corners =
            [left, right, bottom, top].map(|edge| (edge * (config.far / config.near)) as f32);
        if !projection.is_finite()
            || !narrowed.is_finite()
            || [
                narrowed.x_axis.x,
                narrowed.y_axis.y,
                narrowed.z_axis.z,
                narrowed.w_axis.z,
            ]
            .iter()
            .any(|value| !value.is_normal() || *value <= 0.0)
            || !far_corners.iter().all(|edge| edge.is_finite())
            || !((config.near as f32).is_normal()
                && config.near as f32 > 0.0
                && (config.far as f32).is_finite()
                && config.far as f32 > config.near as f32)
        {
            return Err(CameraError::ProjectionOutOfRange);
        }
        if config.skew != 0.0 {
            let corners = frustum_corners(config, -config.near, -config.far);
            let narrowed_corners = frustum_corners(
                config,
                f64::from(-(config.near as f32)),
                f64::from(-(config.far as f32)),
            );
            let determinant = narrowed.determinant();
            let precise_determinant = projection.determinant();
            if !narrowed.y_axis.x.is_normal()
                || !corners
                    .iter()
                    .chain(&narrowed_corners)
                    .all(|point| point.is_finite() && point.as_vec3().is_finite())
                || !determinant.is_finite()
                || determinant == 0.0
                || !precise_determinant.is_finite()
                || precise_determinant == 0.0
            {
                return Err(CameraError::ProjectionOutOfRange);
            }
            if !projection.inverse().is_finite()
                || !narrowed.inverse().is_finite()
                || !narrowed.as_dmat4().inverse().is_finite()
            {
                return Err(CameraError::ProjectionOutOfRange);
            }
        }
        Ok(Self { config, projection })
    }

    /// Derive square-pixel focal lengths from horizontal FOV, as in the native
    /// camera fixture. Other parameters are explicit, including clip planes.
    pub fn from_horizontal_fov(
        width: u32,
        height: u32,
        fov: f64,
        near: f64,
        far: f64,
    ) -> Result<Self, CameraError> {
        if !fov.is_finite() || fov <= 0.0 || fov >= std::f64::consts::PI {
            return Err(CameraError::InvalidIntrinsics);
        }
        let focal = f64::from(width) / (2.0 * (fov / 2.0).tan());
        Self::new(CameraConfig {
            width,
            height,
            fx: focal,
            fy: focal,
            cx: f64::from(width) / 2.0,
            cy: f64::from(height) / 2.0,
            skew: 0.0,
            tx: 0.0,
            ty: 0.0,
            near,
            far,
        })
    }

    pub fn config(self) -> CameraConfig {
        self.config
    }
    /// Bevy-compatible Y-up, finite reverse-Z projection in f64.
    pub fn projection(self) -> DMat4 {
        self.projection
    }
    /// Narrow only after admission has verified the matrix is representable.
    pub fn render_projection(self) -> Mat4 {
        self.projection.as_mat4()
    }

    /// View-space corners in right-bottom, right-top, left-top, left-bottom
    /// order at each supplied plane. Distance uses abs(z), preserving the z sign.
    pub fn frustum_corners(self, z_near: f64, z_far: f64) -> [DVec3; 8] {
        frustum_corners(self.config, z_near, z_far)
    }

    /// Project an optical-frame point without clipping it to image bounds.
    pub fn project_optical(self, point: DVec3) -> Result<[f64; 2], CameraError> {
        if !point.is_finite() || point.z <= 0.0 {
            return Err(CameraError::InvalidPoint);
        }
        let mut pixel = [
            self.config.fx * point.x / point.z + self.config.cx,
            self.config.fy * point.y / point.z + self.config.cy,
        ];
        if self.config.skew != 0.0 {
            pixel[0] -= self.config.skew * point.y / point.z;
        }
        if !pixel.iter().all(|value| value.is_finite()) {
            return Err(CameraError::ProjectionOutOfRange);
        }
        Ok(pixel)
    }
}

/// Convert a Gazebo camera pose to Bevy's camera basis: Gazebo +X forward,
/// +Y left, +Z up becomes Bevy camera -Z forward, +X right, +Y up.
pub fn to_render_camera_pose(
    pose: WorldPose,
    origin: DVec3,
) -> Result<RenderPose, ConversionError> {
    let mut converted = to_render_pose(pose, origin)?;
    converted.rotation *= Quat::from_rotation_y(-std::f32::consts::FRAC_PI_2);
    Ok(converted)
}
