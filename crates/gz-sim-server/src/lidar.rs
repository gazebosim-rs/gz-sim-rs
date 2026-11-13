// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical planar scans and CPU publication handoff; never casts physics rays.
use super::*;
use gz_sim_proto::msgs::{Header, LaserScan, Pose, header::Map};
use gz_sim_scene::{
    EntityKind, LidarPublication, LidarSample, Name as SceneName, PendingResetRemoval,
};

pub(super) struct Stream {
    entity: Entity,
    identity: u64,
    scoped_name: String,
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

// Keep the metadata/output prefix used by existing reset controls.
type Prepared = BTreeMap<String, (LidarPublication, Arc<world_io::LidarOutput>, u64, String)>;
const MAX_LIDARS: usize = 8;
type Original = (u64, String, LidarPublication, String);

fn world_name(simulation: &Simulation) -> Result<&str, Error> {
    let mut worlds = simulation
        .world()
        .iter_entities()
        .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::World));
    let owner = worlds.next().ok_or(Error::LidarIdentity)?;
    if worlds.next().is_some() {
        return Err(Error::LidarIdentity);
    }
    owner
        .get::<SceneName>()
        .map(|name| name.0.as_str())
        .filter(|name| !name.is_empty())
        .ok_or(Error::LidarIdentity)
}

fn valid_owner(
    world: &str,
    id: u64,
    kind: EntityKind,
    name: &str,
    metadata: &LidarPublication,
) -> bool {
    id != 0
        && kind == EntityKind::Sensor
        && valid_metadata(metadata)
        && name
            .strip_prefix(world)
            .and_then(|tail| tail.strip_prefix("::"))
            == Some(metadata.entity_name.as_str())
}

// Validate even removed originals: a currently vacant endpoint is restored by reset.
fn originals(
    simulation: &Simulation,
    options: &ServerOptions,
    occupied: &BTreeSet<String>,
) -> Result<Vec<Original>, Error> {
    let rows: Vec<_> = gz_sim_scene::initial_lidar_publications_with_identity(simulation)?
        .map(|(id, kind, name, metadata)| (id, kind, name.to_owned(), metadata.clone()))
        .collect();
    if rows.len() > MAX_LIDARS {
        return Err(Error::LidarIdentity);
    }
    if rows.is_empty() {
        return Ok(Vec::new());
    }
    let world = world_name(simulation)?;
    let mut ids = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut keys = occupied.clone();
    let mut result = Vec::new();
    for (id, kind, name, metadata) in rows {
        if !valid_owner(world, id, kind, &name, &metadata)
            || !ids.insert(id)
            || !names.insert(name.clone())
        {
            return Err(Error::LidarIdentity);
        }
        let endpoint = options.topic_key(&options::as_valid_topic(&metadata.topic))?;
        if !keys.insert(endpoint.clone()) {
            return Err(Error::LidarIdentity);
        }
        result.push((id, name, metadata, endpoint));
    }
    Ok(result)
}

// Descriptor validation is pure: do not clone or look up a publisher until the
// constructor/reset boundary has checked every own and peer endpoint.
fn reset_plan(server: &Server, occupied: &BTreeSet<String>) -> Result<Vec<Original>, Error> {
    let selected = originals(&server.simulation, &server.network_config, occupied)?;
    for (id, name, metadata, endpoint) in &selected {
        if let Some(stream) = server
            .lidars
            .iter()
            .find(|stream| &stream.endpoint == endpoint)
            && (stream.identity != *id
                || &stream.scoped_name != name
                || &stream.metadata != metadata)
        {
            return Err(Error::ResetLidars);
        }
    }
    Ok(selected)
}

pub(super) fn prepare_reset(server: &Server) -> Result<Prepared, Error> {
    let occupied =
        lidar_reserved_keys(&server.simulation, &server.world_io, &server.network_config)?;
    let selected = reset_plan(server, &occupied)?;
    let mut prepared = BTreeMap::new();
    for (id, name, metadata, key) in selected {
        let publisher = match server.lidars.iter().find(|stream| stream.endpoint == key) {
            Some(stream) => Arc::clone(&stream.publisher),
            None => server
                .world_io
                .lidar_output(key.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        prepared.insert(key, (metadata, publisher, id, name));
    }
    Ok(prepared)
}

type Selection = (Entity, u64, String, LidarPublication, String);

fn admission_plan(
    simulation: &Simulation,
    options: &ServerOptions,
    reserved: impl FnOnce() -> Result<BTreeSet<String>, Error>,
    prepared: Option<&Prepared>,
) -> Result<Vec<Selection>, Error> {
    let candidates: Vec<_> = simulation
        .world()
        .iter_entities()
        .filter(|e| !e.contains::<PendingResetRemoval>() && e.contains::<LidarPublication>())
        .map(|e| e.id())
        .collect();
    if candidates.len() > MAX_LIDARS {
        return Err(Error::LidarIdentity);
    }
    if candidates.is_empty() {
        return if prepared.is_some_and(|p| !p.is_empty()) {
            Err(Error::ResetLidars)
        } else {
            Ok(Vec::new())
        };
    }
    // Reject malformed public ownership before requiring a loaded blueprint.
    for &entity in &candidates {
        let node = simulation.world().entity(entity);
        let id = node.get::<EntityId>().ok_or(Error::LidarIdentity)?.get();
        if node.get::<EntityKind>() != Some(&EntityKind::Sensor)
            || entity_by_id(simulation.world(), id) != Some(entity)
            || !valid_metadata(node.get::<LidarPublication>().expect("selected metadata"))
        {
            return Err(Error::LidarIdentity);
        }
    }
    let occupied = reserved()?;
    let original = originals(simulation, options, &occupied)?;
    let world = world_name(simulation)?;
    let mut ids = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut keys = occupied;
    let mut selected = Vec::new();
    for entity in candidates {
        let node = simulation.world().entity(entity);
        let metadata = node.get::<LidarPublication>().expect("selected metadata");
        let id = node.get::<EntityId>().expect("validated ID").get();
        let name = node.get::<SceneName>().ok_or(Error::LidarIdentity)?;
        if !valid_owner(world, id, EntityKind::Sensor, &name.0, metadata)
            || !ids.insert(id)
            || !names.insert(name.0.clone())
        {
            return Err(Error::LidarIdentity);
        }
        // Empty/invalid topics retain their existing typed Setup error layer.
        let endpoint = options.topic_key(&options::as_valid_topic(&metadata.topic))?;
        if !keys.insert(endpoint.clone()) {
            return Err(Error::LidarIdentity);
        }
        let initial = original
            .iter()
            .find(|(initial_id, initial_name, _, _)| *initial_id == id && initial_name == &name.0)
            .ok_or(Error::LidarIdentity)?;
        if &initial.2 != metadata {
            return Err(if prepared.is_some() {
                Error::ResetLidars
            } else {
                Error::LidarIdentity
            });
        }
        if let Some(prepared) = prepared {
            let (initial_metadata, _, initial_id, initial_name) =
                prepared.get(&endpoint).ok_or(Error::ResetLidars)?;
            if initial_metadata != metadata || *initial_id != id || initial_name != &name.0 {
                return Err(Error::ResetLidars);
            }
        }
        selected.push((entity, id, name.0.clone(), metadata.clone(), endpoint));
    }
    if prepared.is_some_and(|p| p.len() != selected.len()) {
        return Err(Error::ResetLidars);
    }
    Ok(selected)
}

// Own-family malformed identities, duplicate keys, IMU/service aliases and reset
// descriptors take their original Lidar/setup errors before legacy peer errors.
// Returning only strings keeps the boundary independent of Arc/output lookup.
pub(super) fn boundary_keys(
    simulation: &Simulation,
    options: &ServerOptions,
    io: &world_io::WorldIo,
    reset: Option<&Server>,
    own_priority: bool,
) -> Result<BTreeSet<String>, Error> {
    let reserved = || {
        if own_priority {
            lidar_priority_reserved_keys(simulation, io, options)
        } else {
            lidar_reserved_keys(simulation, io, options)
        }
    };
    if let Some(server) = reset {
        let occupied = reserved()?;
        return Ok(reset_plan(server, &occupied)?
            .into_iter()
            .map(|row| row.3)
            .collect());
    }
    let current = admission_plan(simulation, options, reserved, None)?;
    let mut keys: BTreeSet<_> = current.into_iter().map(|row| row.4).collect();
    // Empty manually constructed families keep their existing no-blueprint path.
    match gz_sim_scene::initial_lidar_publications(simulation) {
        Ok(_) => {
            let occupied = reserved()?;
            keys.extend(
                originals(simulation, options, &occupied)?
                    .into_iter()
                    .map(|row| row.3),
            );
        }
        Err(gz_sim_scene::SceneResetError::MissingInitialState) if keys.is_empty() => {}
        Err(error) => return Err(error.into()),
    }
    Ok(keys)
}

pub(super) fn streams(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    options: &ServerOptions,
    imus: &[ImuStream],
    prepared: Option<&Prepared>,
) -> Result<Vec<Stream>, Error> {
    let selected = admission_plan(
        simulation,
        options,
        || {
            let mut occupied = lidar_reserved_keys(simulation, io, options)?;
            occupied.extend(imus.iter().map(|stream| stream.endpoint.clone()));
            Ok(occupied)
        },
        prepared,
    )?;
    let mut result = Vec::new();
    for (entity, identity, scoped_name, metadata, endpoint) in selected {
        let publisher = match prepared {
            Some(p) => Arc::clone(&p.get(&endpoint).expect("entire prepared set validated").1),
            None => io
                .lidar_output(endpoint.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        result.push(Stream {
            entity,
            identity,
            scoped_name,
            metadata,
            endpoint,
            publisher,
            sequence: 0,
            last_sample: None,
        });
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
#[path = "../tests/private/multiple_lidars_native.rs"]
mod multiple_lidars_native;
#[cfg(test)]
#[path = "../tests/private/lidar.rs"]
mod tests;
