// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical ideal magnetometer stream; physics and acquisition remain in Scene.
use super::*;
use gz_sim_proto::msgs::{Header, Magnetometer, header::Map};
use gz_sim_scene::{EntityKind, MagnetometerPublication, MagnetometerSample, PendingResetRemoval};
use std::collections::BTreeSet;

pub(super) struct Stream {
    entity: Entity,
    identity: u64,
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

pub(super) type Prepared =
    BTreeMap<String, (MagnetometerPublication, Arc<world_io::MagnetometerOutput>)>;

/// `occupied` contains every non-magnetometer reserved/publication/input/service key,
/// already resolved using these ServerOptions. Check all metadata before any IO.
pub(super) fn prepare_reset(
    server: &Server,
    occupied: &BTreeSet<String>,
) -> Result<Prepared, Error> {
    let metadata: Vec<_> = gz_sim_scene::initial_magnetometer_publications(&server.simulation)?
        .cloned()
        .collect();
    if metadata.len() > 1 || metadata.iter().any(|m| !valid_metadata(m)) {
        return Err(Error::MagnetometerIdentity);
    }
    let mut prepared = BTreeMap::new();
    for metadata in metadata {
        let key = server
            .network_config
            .topic_key(&options::as_valid_topic(&metadata.topic))?;
        if occupied.contains(&key) {
            return Err(Error::MagnetometerIdentity);
        }
        let publisher = match server
            .magnetometers
            .iter()
            .find(|stream| stream.endpoint == key)
        {
            Some(stream) => Arc::clone(&stream.publisher),
            None => server
                .world_io
                .magnetometer_output(key.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        prepared.insert(key, (metadata, publisher));
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
        .filter(|entity| !entity.contains::<PendingResetRemoval>())
        .filter(|entity| entity.contains::<MagnetometerPublication>())
        .map(|entity| entity.id())
        .collect();
    if candidates.len() > 1 {
        return Err(Error::MagnetometerIdentity);
    }
    // No declaration before count, ownership, metadata and reset-binding validation.
    let mut selected = Vec::new();
    for entity in candidates {
        let entity = simulation.world().entity(entity);
        let metadata = entity
            .get::<MagnetometerPublication>()
            .expect("candidate owns metadata");
        let id = entity
            .get::<EntityId>()
            .ok_or(Error::MagnetometerIdentity)?
            .get();
        if !valid_metadata(metadata)
            || entity.get::<EntityKind>() != Some(&EntityKind::Sensor)
            || entity_by_id(simulation.world(), id) != Some(entity.id())
        {
            return Err(Error::MagnetometerIdentity);
        }
        let endpoint = options.topic_key(&crate::options::as_valid_topic(&metadata.topic))?;
        if occupied.contains(&endpoint) {
            return Err(Error::MagnetometerIdentity);
        }
        if let Some(prepared) = prepared {
            let (original, _) = prepared.get(&endpoint).ok_or(Error::ResetMagnetometers)?;
            if original != metadata {
                return Err(Error::ResetMagnetometers);
            }
        }
        selected.push((entity.id(), id, metadata.clone(), endpoint));
    }
    if prepared.is_some_and(|prepared| prepared.len() != selected.len()) {
        return Err(Error::ResetMagnetometers);
    }
    let mut result = Vec::new();
    for (entity, identity, metadata, endpoint) in selected {
        let publisher = match prepared {
            Some(prepared) => Arc::clone(
                &prepared
                    .get(&endpoint)
                    .expect("prepared endpoint validated before IO")
                    .1,
            ),
            None => io
                .magnetometer_output(endpoint.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        result.push(Stream {
            entity,
            identity,
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
