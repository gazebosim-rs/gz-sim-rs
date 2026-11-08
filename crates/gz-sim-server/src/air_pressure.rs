// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical air-pressure stream; physics and acquisition remain in Scene.
use super::*;
use gz_sim_proto::msgs::{FluidPressure, Header, header::Map};
use gz_sim_scene::{AirPressurePublication, AirPressureSample, EntityKind, PendingResetRemoval};
use std::collections::BTreeSet;

pub(super) struct Stream {
    entity: Entity,
    identity: u64,
    pub(super) metadata: AirPressurePublication,
    pub(super) endpoint: String,
    pub(super) publisher: Arc<world_io::AirPressureOutput>,
    sequence: u64,
    last_sample: Option<(u64, Duration)>,
}

fn valid_metadata(metadata: &AirPressurePublication) -> bool {
    !metadata.entity_name.is_empty() && !metadata.frame_id.is_empty() && !metadata.topic.is_empty()
}

fn packet(
    metadata: &AirPressurePublication,
    sample: &AirPressureSample,
    sequence: u64,
) -> Option<FluidPressure> {
    let reading = sample.reading.as_ref().ok()?;
    if !valid_metadata(metadata)
        || !reading.pressure.is_finite()
        || reading.pressure <= 0.0
        || !reading.variance.is_finite()
        || reading.variance < 0.0
    {
        return None;
    }
    Some(FluidPressure {
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
        pressure: reading.pressure,
        // The canonical schema defines zero as unknown variance.
        variance: reading.variance,
    })
}

pub(super) type Prepared =
    BTreeMap<String, (AirPressurePublication, Arc<world_io::AirPressureOutput>)>;

/// `occupied` contains every non-pressure reserved/publication/input/service key,
/// already resolved using these ServerOptions. Check all metadata before any IO.
pub(super) fn prepare_reset(
    server: &Server,
    occupied: &BTreeSet<String>,
) -> Result<Prepared, Error> {
    let metadata: Vec<_> = gz_sim_scene::initial_air_pressure_publications(&server.simulation)?
        .cloned()
        .collect();
    if metadata.len() > 1 || metadata.iter().any(|m| !valid_metadata(m)) {
        return Err(Error::AirPressureIdentity);
    }
    let mut prepared = BTreeMap::new();
    for metadata in metadata {
        let key = server
            .network_config
            .topic_key(&options::as_valid_topic(&metadata.topic))?;
        if occupied.contains(&key) {
            return Err(Error::AirPressureIdentity);
        }
        let publisher = match server
            .air_pressures
            .iter()
            .find(|stream| stream.endpoint == key)
        {
            Some(stream) => Arc::clone(&stream.publisher),
            None => server
                .world_io
                .air_pressure_output(key.clone())
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
        .filter(|entity| entity.contains::<AirPressurePublication>())
        .map(|entity| entity.id())
        .collect();
    if candidates.len() > 1 {
        return Err(Error::AirPressureIdentity);
    }
    // No declaration before count, ownership, metadata and reset-binding validation.
    let mut selected = Vec::new();
    for entity in candidates {
        let entity = simulation.world().entity(entity);
        let metadata = entity
            .get::<AirPressurePublication>()
            .expect("candidate owns metadata");
        let id = entity
            .get::<EntityId>()
            .ok_or(Error::AirPressureIdentity)?
            .get();
        if !valid_metadata(metadata)
            || entity.get::<EntityKind>() != Some(&EntityKind::Sensor)
            || entity_by_id(simulation.world(), id) != Some(entity.id())
        {
            return Err(Error::AirPressureIdentity);
        }
        let endpoint = options.topic_key(&crate::options::as_valid_topic(&metadata.topic))?;
        if occupied.contains(&endpoint) {
            return Err(Error::AirPressureIdentity);
        }
        if let Some(prepared) = prepared {
            let (original, _) = prepared.get(&endpoint).ok_or(Error::ResetAirPressures)?;
            if original != metadata {
                return Err(Error::ResetAirPressures);
            }
        }
        selected.push((entity.id(), id, metadata.clone(), endpoint));
    }
    if prepared.is_some_and(|prepared| prepared.len() != selected.len()) {
        return Err(Error::ResetAirPressures);
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
                .air_pressure_output(endpoint.clone())
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
            && world.get::<AirPressurePublication>(self.entity) == Some(&self.metadata)
            && world.get::<EntityId>(self.entity).map(EntityId::get) == Some(self.identity)
            && entity_by_id(world, self.identity) == Some(self.entity)
    }
}

impl Server {
    pub(super) fn refresh_air_pressure_interest(&mut self) {
        if self.interest_policy == InterestPolicy::Always {
            gz_sim_scene::clear_air_pressure_interest(&mut self.simulation);
            return;
        }
        let counters = &mut self.counters;
        gz_sim_scene::set_air_pressure_interest(
            &mut self.simulation,
            self.air_pressures.iter().filter_map(|stream| {
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

    pub(super) fn publish_air_pressures(&mut self) {
        for stream in &mut self.air_pressures {
            if !stream.live(&self.simulation) {
                continue;
            }
            let Some(sample) = self
                .simulation
                .world()
                .get::<AirPressureSample>(stream.entity)
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
                self.counters.invalid_air_pressure_samples += 1;
                continue;
            };
            stream.sequence = stream.sequence.wrapping_add(1);
            if self
                .world_io
                .publish_air_pressure(&stream.publisher, message.encode_to_vec())
                .is_ok()
            {
                self.counters.air_pressures_queued += 1;
            } else {
                self.counters.air_pressure_failures += 1;
            }
        }
    }

    pub(super) fn retire_air_pressures(&mut self) {
        self.air_pressures
            .retain(|stream| stream.live(&self.simulation));
        if self.air_pressures.is_empty() {
            self.air_pressures = Vec::new();
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/air_pressure.rs"]
mod tests;
