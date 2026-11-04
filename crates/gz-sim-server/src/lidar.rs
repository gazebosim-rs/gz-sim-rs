// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical planar scans and CPU publication handoff; never casts physics rays.
use super::*;
use gz_sim_proto::msgs::{Header, LaserScan, Pose, header::Map};
use gz_sim_scene::{EntityKind, LidarPublication, LidarSample, PendingResetRemoval};

pub(super) struct Stream {
    entity: Entity,
    identity: u64,
    pub(super) metadata: LidarPublication,
    pub(super) endpoint: String,
    pub(super) publisher: Arc<world_io::LidarOutput>,
    sequence: u64,
    last_sample: Option<(u64, Duration)>,
}

fn valid_metadata(metadata: &LidarPublication) -> bool {
    let c = metadata.config;
    let span = c.angle_max - c.angle_min;
    let valid = (2..=128).contains(&c.samples)
        && !metadata.entity_name.is_empty()
        && !metadata.frame_id.is_empty()
        && [
            c.angle_min,
            c.angle_max,
            c.range_min,
            c.range_max,
            metadata.angle_step,
        ]
        .into_iter()
        .all(f64::is_finite)
        && span.is_finite()
        && span > 0.0
        && span <= std::f64::consts::TAU
        && c.range_min >= 0.0
        && c.range_max > c.range_min
        && metadata.angle_step > 0.0
        && metadata.angle_step == span / (c.samples - 1) as f64;
    if !valid {
        return false;
    }
    let mut previous = None;
    for index in 0..c.samples {
        let angle = if index + 1 == c.samples {
            c.angle_max
        } else {
            c.angle_min + index as f64 * metadata.angle_step
        };
        if !angle.is_finite() || previous.is_some_and(|value| angle <= value) {
            return false;
        }
        previous = Some(angle);
    }
    true
}

fn packet(metadata: &LidarPublication, sample: &LidarSample, sequence: u64) -> Option<LaserScan> {
    let config = metadata.config;
    let pose = sample.world_pose;
    let norm: f64 = pose.rotation.into_iter().map(|x| x * x).sum();
    if !valid_metadata(metadata)
        || sample.ranges.len() != config.samples
        || !sample.ranges.iter().all(|&range| {
            range == f64::INFINITY
                || range.is_finite() && (config.range_min..=config.range_max).contains(&range)
        })
        || !pose
            .translation
            .into_iter()
            .chain(pose.rotation)
            .all(f64::is_finite)
        || (norm - 1.0).abs() > 1e-12
    {
        return None;
    }
    Some(LaserScan {
        header: Some(Header {
            stamp: Some(time(sample.time.as_nanos() as i128).ok()?),
            data: vec![
                Map {
                    key: "frame_id".into(),
                    value: vec![metadata.frame_id.clone()],
                },
                Map {
                    key: "seq".into(),
                    value: vec![sequence.to_string()],
                },
            ],
        }),
        frame: metadata.frame_id.clone(),
        world_pose: Some(Pose {
            position: Some(vector3(pose.translation)),
            orientation: Some(quaternion(pose.rotation)),
            ..Default::default()
        }),
        angle_min: config.angle_min,
        angle_max: config.angle_max,
        angle_step: metadata.angle_step,
        range_min: config.range_min,
        range_max: config.range_max,
        count: config.samples as u32,
        vertical_angle_min: 0.0,
        vertical_angle_max: 0.0,
        vertical_angle_step: 0.0,
        vertical_count: 1,
        ranges: sample.ranges.clone(),
        intensities: vec![0.0; config.samples],
    })
}

type Prepared = BTreeMap<String, (LidarPublication, Arc<world_io::LidarOutput>)>;

pub(super) fn prepare_reset(server: &Server) -> Result<Prepared, Error> {
    let mut prepared = BTreeMap::new();
    for metadata in gz_sim_scene::initial_lidar_publications(&server.simulation)? {
        if !prepared.is_empty() || !valid_metadata(metadata) {
            return Err(Error::LidarIdentity);
        }
        let topic = options::as_valid_topic(&metadata.topic);
        let key = server.network_config.topic_key(&topic)?;
        if server.imus.iter().any(|stream| stream.endpoint == key) {
            return Err(Error::LidarIdentity);
        }
        let publisher = match server.lidars.iter().find(|stream| stream.endpoint == key) {
            Some(stream) => Arc::clone(&stream.publisher),
            None => server
                .world_io
                .lidar_output(key.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        prepared.insert(key, (metadata.clone(), publisher));
    }
    Ok(prepared)
}

pub(super) fn streams(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    options: &ServerOptions,
    imus: &[ImuStream],
    prepared: Option<&Prepared>,
) -> Result<Vec<Stream>, Error> {
    let mut result = Vec::new();
    for entity in simulation.world().iter_entities() {
        if entity.contains::<PendingResetRemoval>() {
            continue;
        }
        let Some(metadata) = entity.get::<LidarPublication>() else {
            continue;
        };
        let id = entity.get::<EntityId>().ok_or(Error::LidarIdentity)?.get();
        if !result.is_empty()
            || !valid_metadata(metadata)
            || entity.get::<EntityKind>() != Some(&EntityKind::Sensor)
            || entity_by_id(simulation.world(), id) != Some(entity.id())
        {
            return Err(Error::LidarIdentity);
        }
        let topic = crate::options::as_valid_topic(&metadata.topic);
        let endpoint = options.topic_key(&topic)?;
        if imus.iter().any(|stream| stream.endpoint == endpoint) {
            return Err(Error::LidarIdentity);
        }
        let publisher = if let Some(prepared) = prepared {
            let (original, publisher) = prepared.get(&endpoint).ok_or(Error::ResetLidars)?;
            if original != metadata {
                return Err(Error::ResetLidars);
            }
            Arc::clone(publisher)
        } else {
            io.lidar_output(endpoint.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?
        };
        result.push(Stream {
            entity: entity.id(),
            identity: id,
            metadata: metadata.clone(),
            endpoint,
            publisher,
            sequence: 0,
            last_sample: None,
        });
    }
    if prepared.is_some_and(|prepared| prepared.len() != result.len()) {
        return Err(Error::ResetLidars);
    }
    Ok(result)
}

impl Stream {
    fn live(&self, simulation: &Simulation) -> bool {
        let world = simulation.world();
        world.get::<EntityKind>(self.entity) == Some(&EntityKind::Sensor)
            && world.get::<LidarPublication>(self.entity) == Some(&self.metadata)
            && world.get::<EntityId>(self.entity).map(EntityId::get) == Some(self.identity)
            && entity_by_id(world, self.identity) == Some(self.entity)
    }
}

impl Server {
    /// Advisory local viewer demand, unioned with canonical Zenoh interest.
    /// False selects no acquisition when neither consumer is present. Explicit
    /// `InterestPolicy::Always` still forces local acquisition as documented.
    pub fn set_lidar_viewer_interest(&mut self, interested: bool) {
        self.lidar_viewer_interest = interested;
    }

    pub(super) fn refresh_lidar_interest(&mut self) {
        if self.interest_policy == InterestPolicy::Always {
            gz_sim_scene::clear_lidar_interest(&mut self.simulation);
            return;
        }
        let viewer = self.lidar_viewer_interest;
        let counters = &mut self.counters;
        gz_sim_scene::set_lidar_interest(
            &mut self.simulation,
            self.lidars.iter().filter_map(|stream| {
                if viewer {
                    return Some(stream.entity);
                }
                match if self.world_io.session.is_closed() {
                    Err("world session is closed")
                } else {
                    stream.publisher.matching()
                } {
                    Ok(true) => Some(stream.entity),
                    Ok(false) => None,
                    Err(_) => {
                        counters.interest_failures += 1;
                        None
                    }
                }
            }),
        );
    }

    pub(super) fn publish_lidars(&mut self) {
        for stream in &mut self.lidars {
            if !stream.live(&self.simulation) {
                continue;
            }
            let Some(sample) = self.simulation.world().get::<LidarSample>(stream.entity) else {
                continue;
            };
            let key = (sample.epoch, sample.time);
            if stream.last_sample == Some(key) {
                continue;
            }
            stream.last_sample = Some(key);
            let message = (sample.epoch == self.simulation.epoch()
                && sample.time <= self.simulation.elapsed())
            .then(|| packet(&stream.metadata, sample, stream.sequence))
            .flatten();
            let Some(message) = message else {
                self.counters.invalid_lidar_samples += 1;
                continue;
            };
            stream.sequence = stream.sequence.wrapping_add(1);
            if self
                .world_io
                .publish_lidar(&stream.publisher, message.encode_to_vec())
                .is_ok()
            {
                self.counters.lidars_queued += 1;
            } else {
                self.counters.lidar_failures += 1;
            }
        }
    }

    pub(super) fn retire_lidars(&mut self) {
        self.lidars.retain(|stream| stream.live(&self.simulation));
        if self.lidars.is_empty() {
            self.lidars = Vec::new();
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/lidar.rs"]
mod tests;
