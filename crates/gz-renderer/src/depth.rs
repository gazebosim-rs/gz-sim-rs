// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Demand-only copy of Bevy's stored, single-sample camera depth attachment.
use super::*;
use bevy::render::render_resource::{BufferDescriptor, Texture};
use glam::{DMat4, DVec3};

/// Optical-forward axial depth in metres, row-major from the upper-left pixel.
/// Background and far-clipped pixels are positive infinity. Near-plane
/// intersections are refused at admission, rather than reported as native -INF.
#[derive(Debug)]
pub struct DepthFrame {
    pub epoch: u64,
    pub time: Duration,
    pub metres: Vec<f32>,
}

pub(super) struct DepthPending {
    pub pending: Pending,
    inverse_projection: DMat4,
}

// This is a geometric admission restriction, not an analytic depth renderer.
// Use every oriented box corner to bound its camera-forward minimum in f64.
pub(super) fn validate_near(
    snapshot: &Snapshot,
    calibration: Calibration,
    camera: WorldPose,
) -> Result<(), RenderError> {
    to_render_camera_pose(camera, snapshot.origin)?;
    for visual in &snapshot.visuals {
        to_render_pose(visual.pose, snapshot.origin)?;
        let size = visual.size.as_vec3();
        if !visual.size.is_finite() || !size.is_finite() || size.min_element() <= 0.0 {
            return Err(RenderError::InvalidVisual);
        }
        let offset = camera.rotation.inverse() * (visual.pose.translation - camera.translation);
        let rotation = camera.rotation.inverse() * visual.pose.rotation;
        for x in [-0.5, 0.5] {
            for y in [-0.5, 0.5] {
                for z in [-0.5, 0.5] {
                    let point = offset + rotation * (visual.size * DVec3::new(x, y, z));
                    // A Gazebo camera looks along its local +X axis.
                    if !point.is_finite() || point.x <= calibration.config().near {
                        return Err(RenderError::DepthNearClip);
                    }
                }
            }
        }
    }
    Ok(())
}

pub(super) fn decode(bytes: &[u8], inverse: DMat4) -> Result<Vec<f32>, RenderError> {
    if bytes.len() != (WIDTH * HEIGHT * 4) as usize || !inverse.is_finite() {
        return Err(RenderError::ReadbackFailed);
    }
    let mut metres = Vec::with_capacity((WIDTH * HEIGHT) as usize);
    for (index, bytes) in bytes.chunks_exact(4).enumerate() {
        let depth = f32::from_le_bytes(bytes.try_into().expect("four-byte chunk"));
        if !depth.is_finite() || !(0.0..=1.0).contains(&depth) {
            return Err(RenderError::ReadbackFailed);
        }
        if depth == 0.0 {
            metres.push(f32::INFINITY);
            continue;
        }
        let x = (index % WIDTH as usize) as f64 + 0.5;
        let y = (index / WIDTH as usize) as f64 + 0.5;
        let point = inverse.project_point3(DVec3::new(
            2.0 * x / f64::from(WIDTH) - 1.0,
            1.0 - 2.0 * y / f64::from(HEIGHT),
            f64::from(depth),
        ));
        let axial = -point.z as f32;
        if !point.is_finite() || !axial.is_finite() || axial <= 0.0 {
            return Err(RenderError::ReadbackFailed);
        }
        metres.push(axial);
    }
    Ok(metres)
}

pub(super) fn request(
    device: &RenderDevice,
    queue: &RenderQueue,
    texture: &Texture,
    stamp: (u64, Duration),
    calibration: Calibration,
) -> Result<DepthPending, RenderError> {
    if texture.format() != TextureFormat::Depth32Float
        || texture.sample_count() != 1
        || texture.width() != WIDTH
        || texture.height() != HEIGHT
        || !texture.usage().contains(wgpu::TextureUsages::COPY_SRC)
    {
        return Err(RenderError::NotReady);
    }
    // Width64 * sizeof(f32) is exactly wgpu's 256-byte row alignment.
    let buffer = device.create_buffer(&BufferDescriptor {
        label: Some("metric_depth_readback"),
        size: u64::from(WIDTH) * u64::from(HEIGHT) * 4,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("metric_depth_copy"),
    });
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::DepthOnly,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(WIDTH * 4),
                rows_per_image: Some(HEIGHT),
            },
        },
        wgpu::Extent3d {
            width: WIDTH,
            height: HEIGHT,
            depth_or_array_layers: 1,
        },
    );
    queue.submit([encoder.finish()]);
    let pixels = Arc::new(Mutex::new(None));
    let result = pixels.clone();
    let mapped = buffer.clone();
    buffer
        .slice(..)
        .map_async(wgpu::MapMode::Read, move |status| {
            let bytes = if status.is_ok() {
                let bytes = mapped
                    .slice(..)
                    .get_mapped_range()
                    .map(|view| view.to_vec())
                    .ok();
                mapped.unmap();
                bytes
            } else {
                None
            };
            if let Ok(mut slot) = result.lock() {
                *slot = Some(bytes);
            }
        });
    Ok(DepthPending {
        pending: Pending {
            epoch: stamp.0,
            time: stamp.1,
            valid: true,
            pixels,
        },
        // Use the exact narrowed matrix sent to Bevy, not a separate ideal camera.
        inverse_projection: calibration.render_projection().as_dmat4().inverse(),
    })
}

impl DepthPending {
    pub(super) fn completed_bytes(&self) -> Result<Option<Option<Vec<u8>>>, RenderError> {
        Ok(self
            .pending
            .pixels
            .lock()
            .map_err(|_| RenderError::ReadbackPoisoned)?
            .take())
    }

    pub(super) fn finish(self, bytes: Option<Vec<u8>>) -> Result<Option<DepthFrame>, RenderError> {
        if !self.pending.valid {
            return Ok(None);
        }
        let metres = decode(
            &bytes.ok_or(RenderError::ReadbackFailed)?,
            self.inverse_projection,
        )?;
        Ok(Some(DepthFrame {
            epoch: self.pending.epoch,
            time: self.pending.time,
            metres,
        }))
    }
}
