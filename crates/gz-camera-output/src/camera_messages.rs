// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! CPU-only messages for the bounded 64×64 RGB camera domain.
//!
//! These functions do not schedule sensors, create publishers or start a renderer.

use gz_msgs::msgs::{
    CameraInfo, Header, Image, PixelFormatType, Time,
    camera_info::{Distortion, Intrinsics, Projection, distortion::DistortionModelType},
    header::Map,
};
use std::time::Duration;

const WIDTH: u32 = 64;
const HEIGHT: u32 = 64;
const RGBA_BYTES: usize = 64 * 64 * 4;
const RGB_BYTES: usize = 64 * 64 * 3;

/// Validated by the caller's camera projection before use for rendering.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntrinsicValues {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum CameraMessageError {
    #[error("expected exactly one 64×64 RGBA frame")]
    InvalidFrame,
    #[error("camera intrinsics must be finite with positive focal lengths")]
    InvalidIntrinsics,
    #[error("acquisition timestamp exceeds protobuf seconds range")]
    TimeRange,
}

fn stamp(time: Duration) -> Result<Time, CameraMessageError> {
    Ok(Time {
        sec: i64::try_from(time.as_secs()).map_err(|_| CameraMessageError::TimeRange)?,
        nsec: time.subsec_nanos() as i32,
    })
}

fn header(stamp: Time, frame_id: &str, sequence: Option<u64>) -> Header {
    let mut data = vec![Map {
        key: "frame_id".into(),
        value: vec![frame_id.into()],
    }];
    if let Some(sequence) = sequence {
        data.push(Map {
            key: "seq".into(),
            value: vec![sequence.to_string()],
        });
    }
    Header {
        stamp: Some(stamp),
        data,
    }
}

/// Convert an acquired RGBA frame to RGB. The caller owns sequence advancement
/// and supplies the acquisition time, independently of readback completion.
pub fn image_packet(
    rgba: &[u8],
    acquisition: Duration,
    frame_id: &str,
    sequence: u64,
) -> Result<Image, CameraMessageError> {
    if rgba.len() != RGBA_BYTES {
        return Err(CameraMessageError::InvalidFrame);
    }
    let stamp = stamp(acquisition)?;
    let mut data = Vec::with_capacity(RGB_BYTES);
    for pixel in rgba.chunks_exact(4) {
        data.extend_from_slice(&pixel[..3]);
    }
    Ok(Image {
        header: Some(header(stamp, frame_id, Some(sequence))),
        width: WIDTH,
        height: HEIGHT,
        step: WIDTH * 3,
        data,
        pixel_format_type: PixelFormatType::RgbInt8 as i32,
    })
}

/// Produce native-shaped monocular camera information for the admitted
/// zero-distortion, absent-explicit-K/P domain. No topic is advertised.
pub fn camera_info_packet(
    acquisition: Duration,
    frame_id: &str,
    intrinsics: IntrinsicValues,
) -> Result<CameraInfo, CameraMessageError> {
    let IntrinsicValues { fx, fy, cx, cy } = intrinsics;
    if ![fx, fy, cx, cy].iter().all(|value| value.is_finite()) || fx <= 0.0 || fy <= 0.0 {
        return Err(CameraMessageError::InvalidIntrinsics);
    }
    let stamp = stamp(acquisition)?;
    Ok(CameraInfo {
        header: Some(header(stamp, frame_id, None)),
        width: WIDTH,
        height: HEIGHT,
        distortion: Some(Distortion {
            model: DistortionModelType::PlumbBob as i32,
            k: vec![0.0; 5],
        }),
        intrinsics: Some(Intrinsics {
            k: vec![fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        }),
        projection: Some(Projection {
            p: vec![fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        }),
        rectification_matrix: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    })
}

#[cfg(test)]
#[path = "../tests/private/camera_messages.rs"]
mod tests;
