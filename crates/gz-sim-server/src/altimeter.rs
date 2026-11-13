// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent canonical altimeter streams; physics and acquisition remain in Scene.
use super::*;
use gz_sim_proto::msgs::{Altimeter, Header, header::Map};
use gz_sim_scene::{AltimeterPublication, AltimeterSample, EntityKind, Name, PendingResetRemoval};
use std::collections::BTreeSet;

pub(super) struct Stream {
    entity: Entity,
    identity: u64,
    scoped_name: String,
    pub(super) metadata: AltimeterPublication,
    pub(super) endpoint: String,
    pub(super) publisher: Arc<world_io::AltimeterOutput>,
    sequence: u64,
    last_sample: Option<(u64, Duration)>,
}

fn valid_metadata(metadata: &AltimeterPublication) -> bool {
    !metadata.entity_name.is_empty()
        && !metadata.frame_id.is_empty()
        && !metadata.topic.is_empty()
        && metadata.vertical_reference.is_finite()
}

fn packet(
    metadata: &AltimeterPublication,
    sample: &AltimeterSample,
    sequence: u64,
) -> Option<Altimeter> {
    let reading = sample.reading.as_ref().ok()?;
    if !valid_metadata(metadata)
        || ![
            reading.vertical_position,
            reading.vertical_velocity,
            reading.vertical_reference,
        ]
        .into_iter()
        .all(f64::is_finite)
        || reading.vertical_reference != metadata.vertical_reference
    {
        return None;
    }
    Some(Altimeter {
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
        vertical_position: reading.vertical_position,
        vertical_velocity: reading.vertical_velocity,
        vertical_reference: reading.vertical_reference,
    })
}

// Reset metadata retains the immutable loaded identity, not only its endpoint.
pub(super) type Prepared = BTreeMap<
    String,
    (
        u64,
        String,
        AltimeterPublication,
        Arc<world_io::AltimeterOutput>,
    ),
>;
const MAX_ALTIMETERS: usize = 8;

fn world_name(simulation: &Simulation) -> Result<&str, Error> {
    let mut owners = simulation
        .world()
        .iter_entities()
        .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::World));
    let owner = owners.next().ok_or(Error::AltimeterIdentity)?;
    if owners.next().is_some() {
        return Err(Error::AltimeterIdentity);
    }
    owner
        .get::<Name>()
        .map(|name| name.0.as_str())
        .filter(|name| !name.is_empty())
        .ok_or(Error::AltimeterIdentity)
}

fn valid_owner(
    world: &str,
    id: u64,
    kind: EntityKind,
    name: &str,
    metadata: &AltimeterPublication,
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
        gz_sim_scene::initial_altimeter_publications_with_identity(&server.simulation)?
            .map(|(id, kind, name, metadata)| (id, kind, name.to_owned(), metadata.clone()))
            .collect();
    if originals.len() > MAX_ALTIMETERS {
        return Err(Error::AltimeterIdentity);
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
            return Err(Error::AltimeterIdentity);
        }
        let endpoint = server
            .network_config
            .topic_key(&options::as_valid_topic(&metadata.topic))?;
        if !keys.insert(endpoint.clone()) {
            return Err(Error::AltimeterIdentity);
        }
        selected.push((id, name, metadata, endpoint));
    }
    // Check every reusable binding before the first possible declaration. A
    // later bad reset entry cannot follow an earlier publisher side effect.
    for (id, name, metadata, endpoint) in &selected {
        if let Some(stream) = server.altimeters.iter().find(|s| &s.endpoint == endpoint)
            && (stream.identity != *id
                || &stream.scoped_name != name
                || &stream.metadata != metadata)
        {
            return Err(Error::ResetAltimeters);
        }
    }
    let mut prepared = BTreeMap::new();
    for (id, name, metadata, endpoint) in selected {
        let publisher = match server.altimeters.iter().find(|s| s.endpoint == endpoint) {
            Some(stream) => Arc::clone(&stream.publisher),
            None => server
                .world_io
                .altimeter_output(endpoint.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        prepared.insert(endpoint, (id, name, metadata, publisher));
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
        .filter(|e| !e.contains::<PendingResetRemoval>() && e.contains::<AltimeterPublication>())
        .map(|e| e.id())
        .collect();
    if candidates.len() > MAX_ALTIMETERS {
        return Err(Error::AltimeterIdentity);
    }
    if candidates.is_empty() {
        return if prepared.is_some_and(|p| !p.is_empty()) {
            Err(Error::ResetAltimeters)
        } else {
            Ok(Vec::new())
        };
    }
    let world = world_name(simulation)?;
    let originals: Vec<_> = gz_sim_scene::initial_altimeter_publications_with_identity(simulation)?
        .map(|(id, kind, name, _)| (id, kind, name.to_owned()))
        .collect();
    let mut ids = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut keys = occupied.clone();
    let mut selected = Vec::new();
    for entity in candidates {
        let owner = simulation.world().entity(entity);
        let metadata = owner
            .get::<AltimeterPublication>()
            .expect("selected metadata");
        let id = owner
            .get::<EntityId>()
            .ok_or(Error::AltimeterIdentity)?
            .get();
        let name = owner.get::<Name>().ok_or(Error::AltimeterIdentity)?;
        let kind = owner.get::<EntityKind>().ok_or(Error::AltimeterIdentity)?;
        if !valid_owner(world, id, *kind, &name.0, metadata)
            || entity_by_id(simulation.world(), id) != Some(entity)
            || !originals
                .iter()
                .any(|(initial_id, initial_kind, initial_name)| {
                    *initial_id == id && initial_kind == kind && initial_name == &name.0
                })
            || !ids.insert(id)
            || !names.insert(name.0.clone())
        {
            return Err(Error::AltimeterIdentity);
        }
        let endpoint = options.topic_key(&options::as_valid_topic(&metadata.topic))?;
        if !keys.insert(endpoint.clone()) {
            return Err(Error::AltimeterIdentity);
        }
        if let Some(prepared) = prepared {
            let (initial_id, initial_name, original, _) =
                prepared.get(&endpoint).ok_or(Error::ResetAltimeters)?;
            if *initial_id != id || initial_name != &name.0 || original != metadata {
                return Err(Error::ResetAltimeters);
            }
        }
        selected.push((entity, id, name.0.clone(), metadata.clone(), endpoint));
    }
    if prepared.is_some_and(|p| p.len() != selected.len()) {
        return Err(Error::ResetAltimeters);
    }
    let mut result = Vec::new();
    for (entity, identity, scoped_name, metadata, endpoint) in selected {
        let publisher = match prepared {
            Some(p) => Arc::clone(&p.get(&endpoint).expect("entire prepared set validated").3),
            None => io
                .altimeter_output(endpoint.clone())
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
            && world.get::<AltimeterPublication>(self.entity) == Some(&self.metadata)
            && world.get::<EntityId>(self.entity).map(EntityId::get) == Some(self.identity)
            && entity_by_id(world, self.identity) == Some(self.entity)
    }
}

impl Server {
    pub(super) fn refresh_altimeter_interest(&mut self) {
        if self.interest_policy == InterestPolicy::Always {
            gz_sim_scene::clear_altimeter_interest(&mut self.simulation);
            return;
        }
        let counters = &mut self.counters;
        gz_sim_scene::set_altimeter_interest(
            &mut self.simulation,
            self.altimeters.iter().filter_map(|stream| {
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

    pub(super) fn publish_altimeters(&mut self) {
        for stream in &mut self.altimeters {
            if !stream.live(&self.simulation) {
                continue;
            }
            let Some(sample) = self
                .simulation
                .world()
                .get::<AltimeterSample>(stream.entity)
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
                self.counters.invalid_altimeter_samples += 1;
                continue;
            };
            stream.sequence = stream.sequence.wrapping_add(1);
            if self
                .world_io
                .publish_altimeter(&stream.publisher, message.encode_to_vec())
                .is_ok()
            {
                self.counters.altimeters_queued += 1;
            } else {
                self.counters.altimeter_failures += 1;
            }
        }
    }

    pub(super) fn retire_altimeters(&mut self) {
        self.altimeters
            .retain(|stream| stream.live(&self.simulation));
        if self.altimeters.is_empty() {
            self.altimeters = Vec::new();
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/altimeter.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/multiple_altimeters_native.rs"]
mod multiple_altimeters_native;
