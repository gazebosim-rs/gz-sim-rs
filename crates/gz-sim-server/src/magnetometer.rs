// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical ideal magnetometer stream; physics and acquisition remain in Scene.
use super::*;
use gz_sim_proto::msgs::{Header, Magnetometer, header::Map};
use gz_sim_scene::{
    EntityKind, MagnetometerPublication, MagnetometerSample, Name, PendingResetRemoval,
};
use std::collections::BTreeSet;

pub(super) struct Stream {
    entity: Entity,
    identity: u64,
    scoped_name: String,
    pub(super) metadata: MagnetometerPublication,
    pub(super) endpoint: String,
    pub(super) publisher: Arc<world_io::MagnetometerOutput>,
    sequence: u64,
    last_sample: Option<(u64, Duration)>,
}

fn valid_metadata(metadata: &MagnetometerPublication) -> bool {
    !metadata.entity_name.is_empty() && !metadata.frame_id.is_empty() && !metadata.topic.is_empty()
}

fn packet(
    metadata: &MagnetometerPublication,
    sample: &MagnetometerSample,
    sequence: u64,
) -> Option<Magnetometer> {
    let reading = sample.reading.as_ref().ok()?;
    if !valid_metadata(metadata) || !reading.field_tesla.iter().all(|value| value.is_finite()) {
        return None;
    }
    Some(Magnetometer {
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
        field_tesla: Some(vector3(reading.field_tesla)),
    })
}

// Reset metadata retains the immutable loaded identity, not only its endpoint.
// Keep the legacy metadata/lease tuple prefix for existing private controls.
pub(super) type Prepared = BTreeMap<
    String,
    (
        MagnetometerPublication,
        Arc<world_io::MagnetometerOutput>,
        u64,
        String,
    ),
>;
const MAX_MAGNETOMETERS: usize = 8;

fn world_name(simulation: &Simulation) -> Result<&str, Error> {
    let mut owners = simulation
        .world()
        .iter_entities()
        .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::World));
    let owner = owners.next().ok_or(Error::MagnetometerIdentity)?;
    if owners.next().is_some() {
        return Err(Error::MagnetometerIdentity);
    }
    owner
        .get::<Name>()
        .map(|name| name.0.as_str())
        .filter(|name| !name.is_empty())
        .ok_or(Error::MagnetometerIdentity)
}

fn valid_owner(
    world: &str,
    id: u64,
    kind: EntityKind,
    name: &str,
    metadata: &MagnetometerPublication,
) -> bool {
    id != 0
        && kind == EntityKind::Sensor
        && valid_metadata(metadata)
        && name
            .strip_prefix(world)
            .and_then(|tail| tail.strip_prefix("::"))
            == Some(metadata.entity_name.as_str())
}

/// Validate the entire immutable reset set before even looking up a CPU lease.
pub(super) fn prepare_reset(
    server: &Server,
    occupied: &BTreeSet<String>,
) -> Result<Prepared, Error> {
    let originals: Vec<_> =
        gz_sim_scene::initial_magnetometer_publications_with_identity(&server.simulation)?
            .map(|(id, kind, name, metadata)| (id, kind, name.to_owned(), metadata.clone()))
            .collect();
    if originals.len() > MAX_MAGNETOMETERS {
        return Err(Error::MagnetometerIdentity);
    }
    if originals.is_empty() {
        return Ok(BTreeMap::new());
    }
    let world = world_name(&server.simulation)?;
    let mut ids = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut keys = occupied.clone();
    let mut selected = Vec::new();
    for (id, kind, name, metadata) in originals {
        if !valid_owner(world, id, kind, &name, &metadata)
            || !ids.insert(id)
            || !names.insert(name.clone())
        {
            return Err(Error::MagnetometerIdentity);
        }
        let endpoint = server
            .network_config
            .topic_key(&options::as_valid_topic(&metadata.topic))?;
        if !keys.insert(endpoint.clone()) {
            return Err(Error::MagnetometerIdentity);
        }
        selected.push((id, name, metadata, endpoint));
    }
    // Check every reusable binding before the first possible declaration. A
    // later bad reset entry cannot follow an earlier publisher side effect.
    for (id, name, metadata, endpoint) in &selected {
        if let Some(stream) = server
            .magnetometers
            .iter()
            .find(|s| &s.endpoint == endpoint)
            && (stream.identity != *id
                || &stream.scoped_name != name
                || &stream.metadata != metadata)
        {
            return Err(Error::ResetMagnetometers);
        }
    }
    let mut prepared = BTreeMap::new();
    for (id, name, metadata, endpoint) in selected {
        let publisher = match server.magnetometers.iter().find(|s| s.endpoint == endpoint) {
            Some(stream) => Arc::clone(&stream.publisher),
            None => server
                .world_io
                .magnetometer_output(endpoint.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        prepared.insert(endpoint, (metadata, publisher, id, name));
    }
    Ok(prepared)
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
        .filter(|e| !e.contains::<PendingResetRemoval>() && e.contains::<MagnetometerPublication>())
        .map(|e| e.id())
        .collect();
    if candidates.len() > MAX_MAGNETOMETERS {
        return Err(Error::MagnetometerIdentity);
    }
    if candidates.is_empty() {
        return if prepared.is_some_and(|p| !p.is_empty()) {
            Err(Error::ResetMagnetometers)
        } else {
            Ok(Vec::new())
        };
    }
    let world = world_name(simulation)?;
    let originals: Vec<_> =
        gz_sim_scene::initial_magnetometer_publications_with_identity(simulation)?
            .map(|(id, kind, name, metadata)| (id, kind, name.to_owned(), metadata.clone()))
            .collect();
    let mut ids = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut keys = occupied.clone();
    let mut selected = Vec::new();
    for entity in candidates {
        let owner = simulation.world().entity(entity);
        let metadata = owner
            .get::<MagnetometerPublication>()
            .expect("selected metadata");
        let id = owner
            .get::<EntityId>()
            .ok_or(Error::MagnetometerIdentity)?
            .get();
        let name = owner.get::<Name>().ok_or(Error::MagnetometerIdentity)?;
        let kind = owner
            .get::<EntityKind>()
            .ok_or(Error::MagnetometerIdentity)?;
        let original = originals
            .iter()
            .find(|(initial_id, initial_kind, initial_name, _)| {
                *initial_id == id && initial_kind == kind && initial_name == &name.0
            });
        if !valid_owner(world, id, *kind, &name.0, metadata)
            || entity_by_id(simulation.world(), id) != Some(entity)
            || original.is_none()
            || !ids.insert(id)
            || !names.insert(name.0.clone())
        {
            return Err(Error::MagnetometerIdentity);
        }
        // A valid but changed topic or frame still violates the immutable
        // loaded blueprint. Preserve the existing prepared-reset error layer.
        if &original.expect("validated original owner").3 != metadata {
            return Err(if prepared.is_some() {
                Error::ResetMagnetometers
            } else {
                Error::MagnetometerIdentity
            });
        }
        let endpoint = options.topic_key(&options::as_valid_topic(&metadata.topic))?;
        if !keys.insert(endpoint.clone()) {
            return Err(Error::MagnetometerIdentity);
        }
        if let Some(prepared) = prepared {
            let (original, _, initial_id, initial_name) =
                prepared.get(&endpoint).ok_or(Error::ResetMagnetometers)?;
            if *initial_id != id || initial_name != &name.0 || original != metadata {
                return Err(Error::ResetMagnetometers);
            }
        }
        selected.push((entity, id, name.0.clone(), metadata.clone(), endpoint));
    }
    if prepared.is_some_and(|p| p.len() != selected.len()) {
        return Err(Error::ResetMagnetometers);
    }
    let mut result = Vec::new();
    for (entity, identity, scoped_name, metadata, endpoint) in selected {
        let publisher = match prepared {
            Some(p) => Arc::clone(&p.get(&endpoint).expect("entire prepared set validated").1),
            None => io
                .magnetometer_output(endpoint.clone())
                .map_err(|e| Error::WorldNetwork(e.to_string()))?,
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
            && world
                .get::<Name>(self.entity)
                .is_some_and(|name| name.0 == self.scoped_name)
            && world.get::<MagnetometerPublication>(self.entity) == Some(&self.metadata)
            && world.get::<EntityId>(self.entity).map(EntityId::get) == Some(self.identity)
            && entity_by_id(world, self.identity) == Some(self.entity)
    }
}

impl Server {
    pub(super) fn refresh_magnetometer_interest(&mut self) {
        if self.interest_policy == InterestPolicy::Always {
            gz_sim_scene::clear_magnetometer_interest(&mut self.simulation);
            return;
        }
        let counters = &mut self.counters;
        gz_sim_scene::set_magnetometer_interest(
            &mut self.simulation,
            self.magnetometers.iter().filter_map(|stream| {
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

    pub(super) fn publish_magnetometers(&mut self) {
        for stream in &mut self.magnetometers {
            if !stream.live(&self.simulation) {
                continue;
            }
            let Some(sample) = self
                .simulation
                .world()
                .get::<MagnetometerSample>(stream.entity)
            else {
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
                self.counters.invalid_magnetometer_samples += 1;
                continue;
            };
            stream.sequence = stream.sequence.wrapping_add(1);
            if self
                .world_io
                .publish_magnetometer(&stream.publisher, message.encode_to_vec())
                .is_ok()
            {
                self.counters.magnetometers_queued += 1;
            } else {
                self.counters.magnetometer_failures += 1;
            }
        }
    }

    pub(super) fn retire_magnetometers(&mut self) {
        self.magnetometers
            .retain(|stream| stream.live(&self.simulation));
        if self.magnetometers.is_empty() {
            self.magnetometers = Vec::new();
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/magnetometer.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/multiple_magnetometers_native.rs"]
mod multiple_magnetometers_native;
