// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! On-demand bounded box snapshots and calibrated RGB/depth primitive snapshots.
//!
//! This is a derived snapshot, not an SDF material or sensor implementation.
//! Supported materials supply an opaque diffuse RGB color; other shading and
//! shadow behavior is outside this renderer slice.

use glam::{DQuat, DVec3};
use gz_render_frame::camera::{Calibration, CameraConfig};
use gz_render_frame::{WorldPose as RenderWorldPose, to_render_pose};
use gz_render_protocol::{Geometry, Request, ShapeVisual, Stamp, Visual};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    CameraSensor, EntityKind, VisualGeometry, VisualMaterial, VisualProperties, WorldLight,
    WorldPose, supports_camera_sensor, supports_camera_visibility_bits,
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
    #[error("world lighting is not supported by the renderer")]
    UnsupportedLighting,
}

/// Separately selected CameraInfo intrinsics and effective render projection.
#[derive(Debug, Clone, Copy)]
pub struct CameraCalibrations {
    pub intrinsics: Calibration,
    pub projection: Calibration,
    /// Native CameraInfo translations; they do not alter rendering.
    pub projection_offsets: [f64; 2],
}

/// Select and validate both matrices without IO or simulation mutation.
/// Explicit P replaces the render matrix while leaving K unchanged. Absent
/// fields are ignored; absent P uses the selected K with zero translation.
pub fn camera_calibrations(
    sensor: &sdformat::Sensor,
    sensor_id: u64,
) -> Result<CameraCalibrations, SnapshotError> {
    let invalid = || SnapshotError::InvalidCamera(sensor_id);
    if !supports_camera_sensor(sensor) {
        return Err(invalid());
    }
    let camera = sensor.camera().expect("supported camera metadata");
    let intrinsics = if camera.lens.has_intrinsics {
        let k = &camera.lens.intrinsics;
        Calibration::new(CameraConfig {
            width: camera.image_width,
            height: camera.image_height,
            fx: k.fx,
            fy: k.fy,
            cx: k.cx,
            cy: k.cy,
            skew: k.skew,
            tx: 0.0,
            ty: 0.0,
            near: camera.near_clip,
            far: camera.far_clip,
        })
    } else {
        Calibration::from_horizontal_fov(
            camera.image_width,
            camera.image_height,
            camera.horizontal_fov,
            camera.near_clip,
            camera.far_clip,
        )
    }
    .map_err(|_| invalid())?;
    let projection = if camera.lens.has_projection {
        let p = &camera.lens.projection;
        Calibration::new(CameraConfig {
            fx: p.fx,
            fy: p.fy,
            cx: p.cx,
            cy: p.cy,
            skew: 0.0,
            tx: 0.0,
            ty: 0.0,
            ..intrinsics.config()
        })
        .map_err(|_| invalid())?
    } else {
        intrinsics
    };
    Ok(CameraCalibrations {
        intrinsics,
        projection,
        projection_offsets: if camera.lens.has_projection {
            [camera.lens.projection.tx, camera.lens.projection.ty]
        } else {
            [0.0, 0.0]
        },
    })
}

/// Extract an explicit calibrated request from one canonical camera sensor.
/// This creates no renderer and does not schedule or publish a sensor sample.
pub fn extract_camera(
    sim: &Simulation,
    sensor_id: u64,
    request_id: u64,
    origin: [f64; 3],
) -> Result<Request, SnapshotError> {
    let world = sim.world();
    // Only an actual calibrated RGB/depth camera admits curved primitives.
    // Missing cameras retain visual validation's original error ordering;
    // generic extract() below remains box-only.
    let calibrated_shapes = entity_by_id(world, sensor_id)
        .and_then(|entity| world.get::<CameraSensor>(entity))
        .is_some_and(|sensor| {
            matches!(
                sensor.0.effective_type(),
                Some(sdformat::SensorType::Camera | sdformat::SensorType::DepthCamera)
            )
        });
    let visibility_mask = if calibrated_shapes {
        entity_by_id(world, sensor_id)
            .filter(|entity| world.get::<EntityKind>(*entity) == Some(&EntityKind::Sensor))
            .and_then(|entity| world.get::<CameraSensor>(entity))
            .and_then(|sensor| sensor.0.camera())
            .map(|camera| camera.visibility_mask)
    } else {
        None
    };
    let (stamp, visuals) =
        extract_visuals(sim, request_id, origin, calibrated_shapes, visibility_mask)?;
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
    let pose = entity
        .get::<WorldPose>()
        .ok_or(SnapshotError::MissingCamera(sensor_id))?;
    let invalid = || SnapshotError::InvalidCamera(sensor_id);
    let calibrations = camera_calibrations(&sensor.0, sensor_id)?;
    let config = calibrations.projection.config();
    gz_render_frame::camera::to_render_camera_pose(
        RenderWorldPose {
            translation: DVec3::from_array(pose.translation),
            rotation: DQuat::from_array(pose.rotation),
        },
        DVec3::from_array(origin),
    )
    .map_err(|_| invalid())?;
    let camera = Box::new(gz_render_protocol::Camera {
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
    });
    let curved = visuals
        .iter()
        .any(|visual| !matches!(visual.geometry, Geometry::Box { .. }));
    let request = if sensor.0.effective_type() == Some(sdformat::SensorType::DepthCamera) {
        if curved {
            Request::CalibratedDepthShapeSnapshot {
                request_id,
                stamp,
                origin,
                visuals,
                camera,
            }
        } else {
            Request::CalibratedDepthSnapshot {
                request_id,
                stamp,
                origin,
                visuals: box_visuals(visuals),
                camera,
            }
        }
    } else if curved {
        Request::CalibratedShapeSnapshot {
            request_id,
            stamp,
            origin,
            visuals,
            camera,
        }
    } else {
        Request::CalibratedSnapshot {
            request_id,
            stamp,
            origin,
            visuals: box_visuals(visuals),
            camera,
        }
    };
    debug_assert!(request.wire_valid());
    Ok(request)
}

/// Build one immutable request from committed simulation state. This function
/// registers no systems and starts no renderer, worker, or background task.
pub fn extract(
    sim: &Simulation,
    request_id: u64,
    origin: [f64; 3],
) -> Result<Request, SnapshotError> {
    let (stamp, visuals) = extract_visuals(sim, request_id, origin, false, None)?;
    let request = Request::Snapshot {
        request_id,
        stamp,
        origin,
        visuals: box_visuals(visuals),
    };
    debug_assert!(request.wire_valid());
    Ok(request)
}

fn box_visuals(visuals: Vec<ShapeVisual>) -> Vec<Visual> {
    visuals
        .into_iter()
        .map(|visual| {
            let Geometry::Box { size } = visual.geometry else {
                unreachable!("box-only collector")
            };
            Visual {
                id: visual.id,
                translation: visual.translation,
                rotation: visual.rotation,
                size,
                color: visual.color,
            }
        })
        .collect()
}

fn extract_visuals(
    sim: &Simulation,
    request_id: u64,
    origin: [f64; 3],
    calibrated_shapes: bool,
    visibility_mask: Option<u32>,
) -> Result<(Stamp, Vec<ShapeVisual>), SnapshotError> {
    if sim.is_faulted() {
        return Err(SnapshotError::Faulted);
    }
    if request_id == 0 || !origin.iter().all(|value| value.is_finite()) {
        return Err(SnapshotError::InvalidRequest);
    }
    let world = sim.world();
    let mut visuals = Vec::new();
    for entity in world.iter_entities() {
        if entity.contains::<WorldLight>()
            || matches!(
                entity.get::<EntityKind>(),
                Some(EntityKind::Light | EntityKind::LightVisual)
            )
        {
            return Err(SnapshotError::UnsupportedLighting);
        }
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
        let (geometry, dimensions) = match geometry.0.as_ref() {
            sdformat::Geometry::Box { size } => (Geometry::Box { size: *size }, *size),
            sdformat::Geometry::Cylinder { radius, length } if calibrated_shapes => {
                let diameter = 2.0 * radius;
                if [*radius, *length, diameter].iter().any(|value| {
                    !value.is_finite() || !(*value as f32).is_finite() || *value as f32 <= 0.0
                }) {
                    return Err(SnapshotError::InvalidVisual(id));
                }
                (
                    Geometry::Cylinder {
                        radius: *radius,
                        length: *length,
                    },
                    [diameter, *length, diameter],
                )
            }
            sdformat::Geometry::Capsule { radius, length } if calibrated_shapes => {
                let diameter = 2.0 * radius;
                let full_length = length + diameter;
                if [*radius, *length, diameter, full_length]
                    .iter()
                    .any(|value| {
                        !value.is_finite() || !(*value as f32).is_finite() || *value as f32 <= 0.0
                    })
                {
                    return Err(SnapshotError::InvalidVisual(id));
                }
                (
                    Geometry::Capsule {
                        radius: *radius,
                        length: *length,
                    },
                    [diameter, diameter, full_length],
                )
            }
            sdformat::Geometry::Sphere { radius } if calibrated_shapes => {
                let diameter = 2.0 * radius;
                if [*radius, diameter].iter().any(|value| {
                    !value.is_finite() || !(*value as f32).is_finite() || *value as f32 <= 0.0
                }) {
                    return Err(SnapshotError::InvalidVisual(id));
                }
                (Geometry::Sphere { radius: *radius }, [diameter; 3])
            }
            sdformat::Geometry::Ellipsoid { radii } if calibrated_shapes => {
                let dimensions = radii.map(|radius| 2.0 * radius);
                if radii.iter().chain(&dimensions).any(|value| {
                    !value.is_finite() || !(*value as f32).is_finite() || *value as f32 <= 0.0
                }) {
                    return Err(SnapshotError::InvalidVisual(id));
                }
                (Geometry::Ellipsoid { radii: *radii }, dimensions)
            }
            _ => return Err(SnapshotError::UnsupportedGeometry(id)),
        };
        let material = &material.0;
        let black = [0.0, 0.0, 0.0, 1.0];
        let unsupported_visibility = match visibility_mask {
            Some(_) => !supports_camera_visibility_bits(properties.0.visibility_flags),
            None => properties.0.visibility_flags != u32::MAX,
        };
        if properties.0.transparency != 0.0
            || !properties.0.cast_shadows
            || unsupported_visibility
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
        if dimensions
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
        visuals.push((
            ShapeVisual {
                id,
                translation: pose.translation,
                rotation: pose.rotation,
                geometry,
                color: [
                    material.diffuse[0],
                    material.diffuse[1],
                    material.diffuse[2],
                ],
            },
            properties.0.visibility_flags,
        ));
    }
    // The complete world has passed validation and the global bound before
    // camera-specific selection; hidden data cannot bypass either admission.
    let mut visuals: Vec<_> = visuals
        .into_iter()
        .filter(|(_, flags)| visibility_mask.is_none_or(|mask| *flags & mask != 0))
        .map(|(visual, _)| visual)
        .collect();
    visuals.sort_unstable_by_key(|visual| visual.id);
    let elapsed = sim.elapsed();
    Ok((
        Stamp {
            epoch: sim.epoch(),
            seconds: elapsed.as_secs(),
            nanoseconds: elapsed.subsec_nanos(),
        },
        visuals,
    ))
}
