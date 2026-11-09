// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One acquisition, two canonical channels. Validate the entire pair before IO.
use super::*;
use crate::logical_camera_io::LogicalCameraChannel;
use gz_sim_proto::msgs::{
    Header, LogicalCameraImage, LogicalCameraSensor, Pose, header::Map, logical_camera_image,
};
use gz_sim_scene::{
    EntityKind, LogicalCameraPublication, LogicalCameraSample, PendingResetRemoval,
};
use std::collections::BTreeSet;
pub(super) struct Stream {
    entity: Entity,
    identity: u64,
    pub(super) metadata: LogicalCameraPublication,
    pub(super) image_endpoint: String,
    pub(super) frustum_endpoint: String,
    pub(super) image: Arc<world_io::LogicalCameraOutput>,
    pub(super) frustum: Arc<world_io::LogicalCameraOutput>,
    sequence: u64,
    last_sample: Option<(u64, Duration)>,
}
fn valid_metadata(m: &LogicalCameraPublication) -> bool {
    !m.entity_name.is_empty()
        && m.entity_name.len() <= 65535
        && !m.frame_id.is_empty()
        && m.frame_id.len() <= 65535
        && [&m.topic, &m.frustum_topic].into_iter().all(|s| {
            s.starts_with('/')
                && s.len() > 1
                && s.len() <= 65535
                && !s
                    .bytes()
                    .any(|b| b.is_ascii_whitespace() || b.is_ascii_control())
                && !["~", "//", "@", ":="].iter().any(|v| s.contains(v))
        })
        && m.frustum_topic == format!("{}/frustum", m.topic)
        && m.config.validate().is_ok()
}
fn valid_pose(translation: [f64; 3], rotation: [f64; 4]) -> bool {
    translation
        .iter()
        .chain(rotation.iter())
        .all(|v| v.is_finite())
        && (rotation.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() <= 1e-12
}
fn valid_name(name: &str) -> bool {
    !name.is_empty()
        && name.len() <= 65535
        && !name.contains("::")
        && !name
            .bytes()
            .any(|b| b.is_ascii_whitespace() || b.is_ascii_control())
}
fn pose(translation: [f64; 3], rotation: [f64; 4]) -> Pose {
    Pose {
        header: None,
        name: String::new(),
        id: 0,
        position: Some(vector3(translation)),
        orientation: Some(quaternion(rotation)),
    }
}
fn packets(
    m: &LogicalCameraPublication,
    s: &LogicalCameraSample,
    sequence: u64,
) -> Option<(LogicalCameraImage, LogicalCameraSensor)> {
    let value = s.reading.as_ref().ok()?;
    if !valid_metadata(m)
        || !valid_pose(value.pose.translation, value.pose.rotation)
        || value.models.len() > 128
        || value
            .models
            .iter()
            .any(|v| !valid_name(&v.name) || !valid_pose(v.pose.translation, v.pose.rotation))
        || value.models.windows(2).any(|p| p[0].name >= p[1].name)
    {
        return None;
    }
    let stamp = Some(time(s.time.as_nanos() as i128).ok()?);
    let frame = Map {
        key: "frame_id".into(),
        value: vec![m.frame_id.clone()],
    };
    Some((
        LogicalCameraImage {
            header: Some(Header {
                stamp,
                data: vec![
                    frame.clone(),
                    Map {
                        key: "seq".into(),
                        value: vec![sequence.to_string()],
                    },
                ],
            }),
            pose: Some(pose(value.pose.translation, value.pose.rotation)),
            model: value
                .models
                .iter()
                .map(|v| logical_camera_image::Model {
                    name: v.name.clone(),
                    pose: Some(pose(v.pose.translation, v.pose.rotation)),
                })
                .collect(),
        },
        LogicalCameraSensor {
            header: Some(Header {
                stamp,
                data: vec![frame],
            }),
            near_clip: m.config.near,
            far_clip: m.config.far,
            horizontal_fov: m.config.horizontal_fov,
            aspect_ratio: m.config.aspect_ratio,
        },
    ))
}
type Outputs = (
    Arc<world_io::LogicalCameraOutput>,
    Arc<world_io::LogicalCameraOutput>,
);
pub(super) type Prepared = BTreeMap<String, (LogicalCameraPublication, String, Outputs)>;
fn endpoints(
    m: &LogicalCameraPublication,
    options: &ServerOptions,
    occupied: &BTreeSet<String>,
) -> Result<(String, String), Error> {
    if !valid_metadata(m) {
        return Err(Error::LogicalCameraIdentity);
    }
    let a = options.topic_key(&options::as_valid_topic(&m.topic))?;
    let b = options.topic_key(&options::as_valid_topic(&m.frustum_topic))?;
    if a == b || occupied.contains(&a) || occupied.contains(&b) {
        return Err(Error::LogicalCameraIdentity);
    }
    Ok((a, b))
}
pub(super) fn prepare_reset(
    server: &Server,
    occupied: &BTreeSet<String>,
) -> Result<Prepared, Error> {
    let metadata: Vec<_> = gz_sim_scene::initial_logical_camera_publications(&server.simulation)?
        .cloned()
        .collect();
    if metadata.len() > 1 {
        return Err(Error::LogicalCameraIdentity);
    }
    // All metadata and both remaps precede either declaration.
    let selected: Vec<_> = metadata
        .into_iter()
        .map(|m| endpoints(&m, &server.network_config, occupied).map(|(a, b)| (m, a, b)))
        .collect::<Result<_, _>>()?;
    let mut result = BTreeMap::new();
    for (m, a, b) in selected {
        let outputs = match server
            .logical_cameras
            .iter()
            .find(|s| s.image_endpoint == a && s.frustum_endpoint == b)
        {
            Some(s) => (Arc::clone(&s.image), Arc::clone(&s.frustum)),
            None => (
                server
                    .world_io
                    .logical_camera_output(a.clone(), LogicalCameraChannel::Image)
                    .map_err(|e| Error::WorldNetwork(e.to_string()))?,
                server
                    .world_io
                    .logical_camera_output(b.clone(), LogicalCameraChannel::Frustum)
                    .map_err(|e| Error::WorldNetwork(e.to_string()))?,
            ),
        };
        result.insert(a, (m, b, outputs));
    }
    Ok(result)
}
pub(super) fn streams(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    options: &ServerOptions,
    occupied: &BTreeSet<String>,
    prepared: Option<&Prepared>,
) -> Result<Vec<Stream>, Error> {
    let candidates: Vec<_> = simulation
        .world()
        .iter_entities()
        .filter(|e| {
            !e.contains::<PendingResetRemoval>() && e.contains::<LogicalCameraPublication>()
        })
        .collect();
    if candidates.len() > 1 {
        return Err(Error::LogicalCameraIdentity);
    }
    let mut selected = Vec::new();
    for e in candidates {
        let m = e
            .get::<LogicalCameraPublication>()
            .expect("candidate metadata");
        let id = e
            .get::<EntityId>()
            .ok_or(Error::LogicalCameraIdentity)?
            .get();
        if e.get::<EntityKind>() != Some(&EntityKind::Sensor)
            || entity_by_id(simulation.world(), id) != Some(e.id())
        {
            return Err(Error::LogicalCameraIdentity);
        }
        let (a, b) = endpoints(m, options, occupied)?;
        if let Some(p) = prepared {
            let (old, old_b, _) = p.get(&a).ok_or(Error::ResetLogicalCameras)?;
            if old != m || old_b != &b {
                return Err(Error::ResetLogicalCameras);
            }
        }
        selected.push((e.id(), id, m.clone(), a, b));
    }
    if prepared.is_some_and(|p| p.len() != selected.len()) {
        return Err(Error::ResetLogicalCameras);
    }
    let mut result = Vec::new();
    for (entity, identity, metadata, image_endpoint, frustum_endpoint) in selected {
        let (image, frustum) = match prepared {
            Some(p) => {
                let (_, _, (a, b)) = p.get(&image_endpoint).expect("prevalidated pair");
                (Arc::clone(a), Arc::clone(b))
            }
            None => (
                io.logical_camera_output(image_endpoint.clone(), LogicalCameraChannel::Image)
                    .map_err(|e| Error::WorldNetwork(e.to_string()))?,
                io.logical_camera_output(frustum_endpoint.clone(), LogicalCameraChannel::Frustum)
                    .map_err(|e| Error::WorldNetwork(e.to_string()))?,
            ),
        };
        result.push(Stream {
            entity,
            identity,
            metadata,
            image_endpoint,
            frustum_endpoint,
            image,
            frustum,
            sequence: 0,
            last_sample: None,
        });
    }
    Ok(result)
}
impl Stream {
    fn live(&self, sim: &Simulation) -> bool {
        let w = sim.world();
        w.get::<EntityKind>(self.entity) == Some(&EntityKind::Sensor)
            && w.get::<LogicalCameraPublication>(self.entity) == Some(&self.metadata)
            && w.get::<EntityId>(self.entity).map(EntityId::get) == Some(self.identity)
            && entity_by_id(w, self.identity) == Some(self.entity)
    }
}
impl Server {
    pub(super) fn refresh_logical_camera_interest(&mut self) {
        if self.interest_policy == InterestPolicy::Always {
            gz_sim_scene::clear_logical_camera_interest(&mut self.simulation);
            return;
        }
        let counters = &mut self.counters;
        gz_sim_scene::set_logical_camera_interest(
            &mut self.simulation,
            self.logical_cameras.iter().filter_map(|s| {
                let status = if self.world_io.session.is_closed() {
                    Err("world session closed")
                } else {
                    s.image
                        .matching()
                        .and_then(|a| s.frustum.matching().map(|b| a || b))
                };
                match status {
                    Ok(true) => Some(s.entity),
                    Ok(false) => None,
                    Err(_) => {
                        counters.interest_failures += 1;
                        None
                    }
                }
            }),
        );
    }
    pub(super) fn publish_logical_cameras(&mut self) {
        for s in &mut self.logical_cameras {
            if !s.live(&self.simulation) {
                continue;
            }
            let Some(sample) = self.simulation.world().get::<LogicalCameraSample>(s.entity) else {
                continue;
            };
            let key = (sample.epoch, sample.time);
            if s.last_sample == Some(key) {
                continue;
            }
            s.last_sample = Some(key);
            let pair = (sample.epoch == self.simulation.epoch()
                && sample.time <= self.simulation.elapsed())
            .then(|| packets(&s.metadata, sample, s.sequence))
            .flatten();
            let Some((image, frustum)) = pair else {
                self.counters.invalid_logical_camera_samples += 1;
                continue;
            };
            s.sequence = s.sequence.wrapping_add(1);
            if self
                .world_io
                .publish_logical_camera(&s.image, image.encode_to_vec())
                .is_ok()
            {
                self.counters.logical_camera_images_queued += 1;
            } else {
                self.counters.logical_camera_image_failures += 1;
            }
            if self
                .world_io
                .publish_logical_camera(&s.frustum, frustum.encode_to_vec())
                .is_ok()
            {
                self.counters.logical_camera_frustums_queued += 1;
            } else {
                self.counters.logical_camera_frustum_failures += 1;
            }
        }
    }
    pub(super) fn retire_logical_cameras(&mut self) {
        self.logical_cameras.retain(|s| s.live(&self.simulation));
        if self.logical_cameras.is_empty() {
            self.logical_cameras = Vec::new();
        }
    }
}
#[cfg(test)]
#[path = "../tests/private/logical_camera.rs"]
mod tests;
