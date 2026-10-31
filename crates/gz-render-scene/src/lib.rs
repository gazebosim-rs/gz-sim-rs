// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! On-demand extraction of a bounded box scene for the optional renderer.
//!
//! This is a derived snapshot, not an SDF material or sensor implementation.
//! Supported materials supply an opaque diffuse RGB color; other shading and
//! shadow behavior is outside this renderer slice.

use glam::{DQuat, DVec3};
use gz_render_frame::camera::Calibration;
use gz_render_frame::{WorldPose as RenderWorldPose, to_render_pose};
use gz_render_protocol::{Request, Stamp, Visual};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    CameraSensor, EntityKind, VisualGeometry, VisualMaterial, VisualProperties, WorldPose,
    supports_camera_sensor,
};

const MAX_VISUALS: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum SnapshotError {
    #[error("simulation is faulted")]
    Faulted,
    #[error("request ID must be nonzero and origin must be finite")]
    InvalidRequest,
    #[error("visual entity has no valid owned identity")]
    InvalidIdentity,
    #[error("visual {0} is missing required pose, geometry, properties, or material")]
    MissingVisualData(u64),
    #[error("visual {0} has unsupported geometry")]
    UnsupportedGeometry(u64),
    #[error("visual {0} has unsupported material or visual properties")]
    UnsupportedMaterial(u64),
    #[error("visual {0} has an invalid or unrepresentable pose or size")]
    InvalidVisual(u64),
    #[error("snapshot exceeds the renderer visual limit")]
    TooManyVisuals,
    #[error("camera sensor {0} is missing or has invalid identity or pose")]
    MissingCamera(u64),
    #[error("camera sensor {0} has unsupported or invalid calibration")]
    InvalidCamera(u64),
}

/// Extract an explicit calibrated request from one canonical camera sensor.
/// This creates no renderer and does not schedule or publish a sensor sample.
pub fn extract_camera(
    sim: &Simulation,
    sensor_id: u64,
    request_id: u64,
    origin: [f64; 3],
) -> Result<Request, SnapshotError> {
    let request = extract(sim, request_id, origin)?;
    let world = sim.world();
    let entity = entity_by_id(world, sensor_id).ok_or(SnapshotError::MissingCamera(sensor_id))?;
    let entity = world.entity(entity);
    if entity.get::<EntityKind>() != Some(&EntityKind::Sensor) {
        return Err(SnapshotError::MissingCamera(sensor_id));
    }
    let sensor = entity
        .get::<CameraSensor>()
        .ok_or(SnapshotError::MissingCamera(sensor_id))?;
    if !supports_camera_sensor(&sensor.0) {
        return Err(SnapshotError::InvalidCamera(sensor_id));
    }
    let camera = sensor
        .0
        .camera()
        .expect("supported camera sensor has camera metadata");
    let pose = entity
        .get::<WorldPose>()
        .ok_or(SnapshotError::MissingCamera(sensor_id))?;
    let invalid = || SnapshotError::InvalidCamera(sensor_id);
    let config = Calibration::from_horizontal_fov(
        camera.image_width,
        camera.image_height,
        camera.horizontal_fov,
        camera.near_clip,
        camera.far_clip,
    )
    .map_err(|_| invalid())?
    .config();
    gz_render_frame::camera::to_render_camera_pose(
        RenderWorldPose {
            translation: DVec3::from_array(pose.translation),
            rotation: DQuat::from_array(pose.rotation),
        },
        DVec3::from_array(origin),
    )
    .map_err(|_| invalid())?;
    let Request::Snapshot {
        request_id,
        stamp,
        origin,
        visuals,
    } = request
    else {
        unreachable!()
    };
    Ok(Request::CalibratedSnapshot {
        request_id,
        stamp,
        origin,
        visuals,
        camera: Box::new(gz_render_protocol::Camera {
            width: config.width,
            height: config.height,
            fx: config.fx,
            fy: config.fy,
            cx: config.cx,
            cy: config.cy,
            skew: config.skew,
            tx: config.tx,
            ty: config.ty,
            near: config.near,
            far: config.far,
            translation: pose.translation,
            rotation: pose.rotation,
        }),
    })
}

/// Build one immutable request from committed simulation state. This function
/// registers no systems and starts no renderer, worker, or background task.
pub fn extract(
    sim: &Simulation,
    request_id: u64,
    origin: [f64; 3],
) -> Result<Request, SnapshotError> {
    if sim.is_faulted() {
        return Err(SnapshotError::Faulted);
    }
    if request_id == 0 || !origin.iter().all(|value| value.is_finite()) {
        return Err(SnapshotError::InvalidRequest);
    }
    let world = sim.world();
    let mut visuals = Vec::new();
    for entity in world.iter_entities() {
        if entity.get::<EntityKind>() != Some(&EntityKind::Visual) {
            continue;
        }
        let id = entity
            .get::<EntityId>()
            .filter(|id| entity_by_id(world, id.get()) == Some(entity.id()))
            .ok_or(SnapshotError::InvalidIdentity)?
            .get();
        let pose = entity
            .get::<WorldPose>()
            .ok_or(SnapshotError::MissingVisualData(id))?;
        let geometry = entity
            .get::<VisualGeometry>()
            .ok_or(SnapshotError::MissingVisualData(id))?;
        let properties = entity
            .get::<VisualProperties>()
            .ok_or(SnapshotError::MissingVisualData(id))?;
        let material = entity
            .get::<VisualMaterial>()
            .ok_or(SnapshotError::MissingVisualData(id))?;
        let sdformat::Geometry::Box { size } = geometry.0.as_ref() else {
            return Err(SnapshotError::UnsupportedGeometry(id));
        };
        let material = &material.0;
        let black = [0.0, 0.0, 0.0, 1.0];
        if properties.0.transparency != 0.0
            || !properties.0.cast_shadows
            || properties.0.visibility_flags != u32::MAX
            || material.ambient != black
            || material.specular != black
            || material.emissive != black
            || material.shininess != 0.0
            || material.render_order != 0.0
            || !material.lighting
            || material.double_sided
            || material.shader != sdformat::ShaderType::Pixel
            || !material.normal_map.is_empty()
            || material.script.is_some()
            || material.pbr.is_some()
            || material.diffuse[3] != 1.0
            || material.diffuse[..3]
                .iter()
                .any(|value| !value.is_finite() || !(0.0..=1.0).contains(value))
        {
            return Err(SnapshotError::UnsupportedMaterial(id));
        }
        if size
            .iter()
            .any(|value| !value.is_finite() || !(*value as f32).is_finite() || *value as f32 <= 0.0)
            || to_render_pose(
                RenderWorldPose {
                    translation: DVec3::from_array(pose.translation),
                    rotation: DQuat::from_array(pose.rotation),
                },
                DVec3::from_array(origin),
            )
            .is_err()
        {
            return Err(SnapshotError::InvalidVisual(id));
        }
        if visuals.len() == MAX_VISUALS {
            return Err(SnapshotError::TooManyVisuals);
        }
        visuals.push(Visual {
            id,
            translation: pose.translation,
            rotation: pose.rotation,
            size: *size,
            color: [
                material.diffuse[0],
                material.diffuse[1],
                material.diffuse[2],
            ],
        });
    }
    visuals.sort_unstable_by_key(|visual| visual.id);
    let elapsed = sim.elapsed();
    let request = Request::Snapshot {
        request_id,
        stamp: Stamp {
            epoch: sim.epoch(),
            seconds: elapsed.as_secs(),
            nanoseconds: elapsed.subsec_nanos(),
        },
        origin,
        visuals,
    };
    debug_assert!(request.wire_valid());
    Ok(request)
}
