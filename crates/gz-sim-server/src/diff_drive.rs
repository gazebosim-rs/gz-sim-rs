// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Up to eight original model owners; canonical command and encoder packet arithmetic is unchanged.
use super::*;
use crate::diff_drive_io::{Endpoint, Installed, Output, Setup};
use bevy_ecs::prelude::ChildOf;
use gz_sim_proto::msgs::{Header, Odometry, Pose, Twist, header::Map};
use gz_sim_scene::{
    DiffDriveConfiguration, DiffDriveSample, EntityKind, Name, PendingResetRemoval,
};
use std::{collections::BTreeSet, sync::mpsc};

pub(super) type Metadata = BTreeMap<u64, DiffDriveConfiguration>;
pub(super) const MAX_OWNERS: usize = 8;

fn valid(metadata: &DiffDriveConfiguration) -> bool {
    !metadata.model_name.is_empty()
        && !metadata.frame_id.is_empty()
        && !metadata.child_frame_id.is_empty()
        && [&metadata.topic, &metadata.odom_topic]
            .into_iter()
            .all(|topic| topic.starts_with('/') && options::as_valid_topic(topic) == *topic)
        && metadata.topic != metadata.odom_topic
}
fn packet(
    metadata: &DiffDriveConfiguration,
    sample: &DiffDriveSample,
    sequence: u64,
) -> Option<Odometry> {
    if !valid(metadata)
        || !sample
            .pose
            .into_iter()
            .chain(sample.twist)
            .all(f64::is_finite)
    {
        return None;
    }
    let (sin, cos) = (sample.pose[2] * 0.5).sin_cos();
    Some(Odometry {
        header: Some(Header {
            stamp: Some(time(sample.time.as_nanos() as i128).ok()?),
            data: vec![
                Map {
                    key: "frame_id".into(),
                    value: vec![metadata.frame_id.clone()],
                },
                Map {
                    key: "child_frame_id".into(),
                    value: vec![metadata.child_frame_id.clone()],
                },
                Map {
                    key: "seq".into(),
                    value: vec![sequence.to_string()],
                },
            ],
        }),
        pose: Some(Pose {
            position: Some(vector3([sample.pose[0], sample.pose[1], 0.0])),
            orientation: Some(quaternion([0.0, 0.0, sin, cos])),
            ..Default::default()
        }),
        twist: Some(Twist {
            linear: Some(vector3([sample.twist[0], 0.0, 0.0])),
            angular: Some(vector3([0.0, 0.0, sample.twist[1]])),
            ..Default::default()
        }),
    })
}

struct Original {
    name: String,
    parent: u64,
    metadata: DiffDriveConfiguration,
}
struct Candidate {
    entity: Entity,
    identity: u64,
    name: Option<String>,
    parent: Option<u64>,
    metadata: DiffDriveConfiguration,
}
struct Admission {
    owners: BTreeMap<u64, Candidate>,
    metadata: Metadata,
    keys: BTreeSet<String>,
}

fn originals(sim: &Simulation) -> Result<Option<BTreeMap<u64, Original>>, Error> {
    let rows = match gz_sim_scene::initial_diff_drive_configurations_with_identity(sim) {
        Ok(rows) => rows.take(MAX_OWNERS + 1).collect::<Vec<_>>(),
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => return Ok(None),
        Err(error) => return Err(error.into()),
    };
    if rows.len() > MAX_OWNERS {
        return Err(Error::DiffDriveIdentity);
    }
    let mut result = BTreeMap::new();
    if rows.is_empty() {
        return Ok(Some(result));
    }
    let mut worlds = sim
        .world()
        .iter_entities()
        .filter(|node| node.get::<EntityKind>() == Some(&EntityKind::World));
    let world = worlds.next().ok_or(Error::DiffDriveIdentity)?;
    let world_id = world
        .get::<EntityId>()
        .ok_or(Error::DiffDriveIdentity)?
        .get();
    let world_name = &world.get::<Name>().ok_or(Error::DiffDriveIdentity)?.0;
    if world_id == 0 || world_name.is_empty() || worlds.next().is_some() {
        return Err(Error::DiffDriveIdentity);
    }
    let mut names = BTreeSet::new();
    for row in rows {
        if row.id == 0
            || row.id == world_id
            || row.kind != EntityKind::Model
            || row.parent != Some(world_id)
            || !valid(row.configuration)
            || row.name != format!("{world_name}::{}", row.configuration.model_name)
            || !names.insert(row.name.to_owned())
            || result.contains_key(&row.id)
        {
            return Err(Error::DiffDriveIdentity);
        }
        result.insert(
            row.id,
            Original {
                name: row.name.to_owned(),
                parent: world_id,
                metadata: row.configuration.clone(),
            },
        );
    }
    Ok(Some(result))
}

// Keep this vector API for the existing reservation/priority callers.
pub(super) fn initial(sim: &Simulation) -> Result<Vec<DiffDriveConfiguration>, Error> {
    let values: Vec<_> = match gz_sim_scene::initial_diff_drive_configurations(sim) {
        Ok(values) => values.take(MAX_OWNERS + 1).cloned().collect(),
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => Vec::new(),
        Err(error) => return Err(error.into()),
    };
    if values.len() > MAX_OWNERS || values.iter().any(|value| !valid(value)) {
        return Err(Error::DiffDriveIdentity);
    }
    Ok(values)
}

// Discovery is also the real absent-system / expected-set boundary. It never
// admits a private owner: constructor/live checks still examine every original.
pub(super) fn declared(
    sim: &Simulation,
    expected: Option<&Metadata>,
) -> Result<Option<Metadata>, Error> {
    let loaded = match gz_sim_scene::initial_diff_drive_configurations(sim) {
        Ok(_) => true,
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => false,
        Err(error) => return Err(error.into()),
    };
    let mut rows = BTreeMap::new();
    for node in sim.world().iter_entities().filter(|node| {
        node.contains::<DiffDriveConfiguration>() && !node.contains::<PendingResetRemoval>()
    }) {
        if rows.len() == if loaded { MAX_OWNERS } else { 1 } {
            return Err(Error::DiffDriveIdentity);
        }
        let metadata = node
            .get::<DiffDriveConfiguration>()
            .expect("selected configured model");
        let identity = node
            .get::<EntityId>()
            .ok_or(Error::DiffDriveIdentity)?
            .get();
        if identity == 0
            || !valid(metadata)
            || node.get::<EntityKind>() != Some(&EntityKind::Model)
            || entity_by_id(sim.world(), identity) != Some(node.id())
            || rows.contains_key(&identity)
        {
            return Err(Error::DiffDriveIdentity);
        }
        rows.insert(identity, metadata.clone());
    }
    if expected.is_some_and(|expected| expected != &rows) {
        return Err(Error::ResetDiffDrive);
    }
    Ok((!rows.is_empty()).then_some(rows))
}

fn candidates(
    sim: &Simulation,
    saved: &Option<BTreeMap<u64, Original>>,
    restoring: bool,
) -> Result<BTreeMap<u64, Candidate>, Error> {
    let mut owners = BTreeMap::new();
    for (identity, metadata) in declared(sim, None)?.unwrap_or_default() {
        let entity = entity_by_id(sim.world(), identity).ok_or(Error::DiffDriveIdentity)?;
        let node = sim.world().entity(entity);
        let (name, parent) = if let Some(saved) = saved {
            let original = saved.get(&identity).ok_or(Error::DiffDriveIdentity)?;
            let parent = node.get::<ChildOf>().and_then(|parent| {
                sim.world()
                    .get::<EntityId>(parent.parent())
                    .map(EntityId::get)
            });
            if node.get::<Name>().map(|name| &name.0) != Some(&original.name)
                || parent != Some(original.parent)
                || metadata.model_name != original.metadata.model_name
                || if restoring {
                    !gz_sim_scene::diff_drive_binding_matches_configuration(
                        sim,
                        identity,
                        &original.metadata,
                    )
                } else {
                    metadata != original.metadata
                        || !gz_sim_scene::diff_drive_binding_matches(sim, identity)
                }
            {
                return Err(Error::DiffDriveIdentity);
            }
            (Some(original.name.clone()), Some(original.parent))
        } else {
            // The inherited manual singleton has no original blueprint to invent.
            (None, None)
        };
        owners.insert(
            identity,
            Candidate {
                entity,
                identity,
                name,
                parent,
                metadata,
            },
        );
    }
    Ok(owners)
}

pub(super) struct Stream {
    entity: Entity,
    identity: u64,
    name: Option<String>,
    parent: Option<u64>,
    metadata: DiffDriveConfiguration,
    command_key: String,
    odometry_key: String,
    lease_id: uuid::Uuid,
    output: Arc<Output>,
    receiver: mpsc::Receiver<crate::diff_drive_io::Command>,
    sequence: u64,
    last_sample: Option<(u64, Duration)>,
}
impl Stream {
    fn removed(&self, sim: &Simulation) -> bool {
        sim.world().get_entity(self.entity).is_err()
            || sim
                .world()
                .entity(self.entity)
                .contains::<PendingResetRemoval>()
    }
    fn live(&self, sim: &Simulation) -> bool {
        let world = sim.world();
        world.get::<EntityKind>(self.entity) == Some(&EntityKind::Model)
            && world.get::<DiffDriveConfiguration>(self.entity) == Some(&self.metadata)
            && world.get::<EntityId>(self.entity).map(EntityId::get) == Some(self.identity)
            && entity_by_id(world, self.identity) == Some(self.entity)
            && !world.entity(self.entity).contains::<PendingResetRemoval>()
            && self.name.as_ref().is_none_or(|name| {
                world.get::<Name>(self.entity).map(|current| &current.0) == Some(name)
                    && world.get::<ChildOf>(self.entity).and_then(|parent| {
                        world.get::<EntityId>(parent.parent()).map(EntityId::get)
                    }) == self.parent
                    && gz_sim_scene::diff_drive_binding_matches(sim, self.identity)
            })
    }
}

pub(super) struct Drives {
    streams: BTreeMap<u64, Stream>,
    next_poll: usize,
}

fn retained(
    sim: &Simulation,
    drives: &Drives,
    saved: &Option<BTreeMap<u64, Original>>,
    active: bool,
) -> Result<(), Error> {
    if drives.streams.len() > if saved.is_some() { MAX_OWNERS } else { 1 } {
        return Err(Error::DiffDriveIdentity);
    }
    let mut keys = BTreeSet::new();
    let mut leases = BTreeSet::new();
    for (id, stream) in &drives.streams {
        if *id != stream.identity
            || !leases.insert(stream.lease_id)
            || !keys.insert(stream.command_key.as_str())
            || !keys.insert(stream.odometry_key.as_str())
            || drives
                .streams
                .range(..*id)
                .any(|(_, earlier)| Arc::ptr_eq(&earlier.output, &stream.output))
            || if active {
                !stream.output.matches_lease(
                    stream.lease_id,
                    &stream.command_key,
                    &stream.odometry_key,
                )
            } else {
                !stream.output.matches_identity(
                    stream.lease_id,
                    &stream.command_key,
                    &stream.odometry_key,
                )
            }
        {
            return Err(Error::DiffDriveIdentity);
        }
        if let Some(saved) = saved {
            let original = saved.get(id).ok_or(Error::DiffDriveIdentity)?;
            if stream.name.as_ref() != Some(&original.name)
                || stream.parent != Some(original.parent)
                || stream.metadata != original.metadata
            {
                return Err(Error::DiffDriveIdentity);
            }
        }
        if !stream.removed(sim) && !stream.live(sim) {
            return Err(Error::DiffDriveIdentity);
        }
    }
    Ok(())
}

fn own_keys(metadata: &Metadata, config: &ServerOptions) -> Result<BTreeSet<String>, Error> {
    if metadata.len() > MAX_OWNERS || metadata.contains_key(&0) {
        return Err(Error::DiffDriveIdentity);
    }
    let mut keys = BTreeSet::new();
    for row in metadata.values() {
        if !valid(row)
            || !keys.insert(config.topic_key(&row.topic)?)
            || !keys.insert(config.topic_key(&row.odom_topic)?)
        {
            return Err(Error::DiffDriveIdentity);
        }
    }
    Ok(keys)
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Channel {
    Command,
    Odometry,
}
type OriginalKeys = BTreeMap<String, (u64, Channel)>;

fn original_keys(
    saved: &Option<BTreeMap<u64, Original>>,
    config: &ServerOptions,
) -> Result<OriginalKeys, Error> {
    let mut keys = BTreeMap::new();
    for (id, row) in saved.iter().flat_map(|rows| rows.iter()) {
        for (topic, channel) in [
            (&row.metadata.topic, Channel::Command),
            (&row.metadata.odom_topic, Channel::Odometry),
        ] {
            if keys
                .insert(config.topic_key(topic)?, (*id, channel))
                .is_some()
            {
                return Err(Error::DiffDriveIdentity);
            }
        }
    }
    Ok(keys)
}

fn original_claims(
    owners: &BTreeMap<u64, Candidate>,
    keys: &OriginalKeys,
    config: &ServerOptions,
) -> Result<(), Error> {
    for (id, owner) in owners {
        for (topic, channel) in [
            (&owner.metadata.topic, Channel::Command),
            (&owner.metadata.odom_topic, Channel::Odometry),
        ] {
            if keys
                .get(&config.topic_key(topic)?)
                .is_some_and(|original| *original != (*id, channel))
            {
                return Err(Error::DiffDriveIdentity);
            }
        }
    }
    Ok(())
}

fn own_set(
    sim: &Simulation,
    config: &ServerOptions,
    reset: Option<&Server>,
) -> Result<Admission, Error> {
    let saved = originals(sim)?;
    // The retained OLD declaration witness is checked before any NEW remap plan.
    if let Some(server) = reset
        && let Some(drives) = &server.diff_drive
    {
        retained(sim, drives, &saved, true)?;
    }
    let owners = live_owners(sim, &saved, false)?;
    original_claims(&owners, &original_keys(&saved, config)?, config)?;
    let metadata: Metadata = if reset.is_some() {
        saved
            .as_ref()
            .map(|rows| {
                rows.iter()
                    .map(|(id, row)| (*id, row.metadata.clone()))
                    .collect()
            })
            .unwrap_or_default()
    } else {
        owners
            .iter()
            .map(|(id, owner)| (*id, owner.metadata.clone()))
            .collect()
    };
    let keys = own_keys(&metadata, config)?;
    Ok(Admission {
        owners,
        metadata,
        keys,
    })
}

pub(super) fn boundary_keys(
    sim: &Simulation,
    config: &ServerOptions,
    reset: Option<&Server>,
) -> Result<BTreeSet<String>, Error> {
    own_set(sim, config, reset).map(|admission| admission.keys)
}

pub(super) fn preflight(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<Metadata, Error> {
    let admission = own_set(sim, config, None)?;
    if !admission
        .keys
        .is_disjoint(&diff_drive_reserved_keys(sim, io, config)?)
    {
        return Err(Error::DiffDriveIdentity);
    }
    Ok(admission.metadata)
}

pub(super) fn prepare_reset(server: &Server) -> Result<Metadata, Error> {
    let admission = own_set(&server.simulation, &server.network_config, Some(server))?;
    if !admission.keys.is_disjoint(&diff_drive_reserved_keys(
        &server.simulation,
        &server.world_io,
        &server.network_config,
    )?) {
        return Err(Error::DiffDriveIdentity);
    }
    Ok(admission.metadata)
}

fn prepared(
    owners: &BTreeMap<u64, Candidate>,
    installed: &Installed,
    config: &ServerOptions,
) -> Result<(), Error> {
    if !owners.keys().eq(installed.outputs.keys()) || !owners.keys().eq(installed.lease_ids.keys())
    {
        return Err(Error::DiffDriveIdentity);
    }
    let mut ids = BTreeSet::new();
    for (id, owner) in owners {
        let expected = installed.lease_ids[id];
        let output = &installed.outputs[id];
        if !ids.insert(expected)
            || !output.matches_lease(
                expected,
                &config.topic_key(&owner.metadata.topic)?,
                &config.topic_key(&owner.metadata.odom_topic)?,
            )
            || installed
                .outputs
                .range(..*id)
                .any(|(_, earlier)| Arc::ptr_eq(earlier, output))
        {
            return Err(Error::DiffDriveIdentity);
        }
    }
    Ok(())
}

pub(super) fn prepare(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
    expected: &Metadata,
) -> Result<Option<Drives>, Error> {
    build(sim, io, config, expected, false)
}

// Only the post-reset factory can classify valid public metadata drift as Reset.
// Root first validates the retained OLD active leases through prepare_reset.
pub(super) fn prepare_restored(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
    expected: &Metadata,
) -> Result<Option<Drives>, Error> {
    build(sim, io, config, expected, true)
}

fn build(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
    expected: &Metadata,
    restoring: bool,
) -> Result<Option<Drives>, Error> {
    let admission = if restoring {
        let saved = originals(sim)?;
        let owners = live_owners(sim, &saved, true)?;
        original_claims(&owners, &original_keys(&saved, config)?, config)?;
        let metadata = owners
            .iter()
            .map(|(id, row)| (*id, row.metadata.clone()))
            .collect();
        let keys = own_keys(&metadata, config)?;
        Admission {
            owners,
            metadata,
            keys,
        }
    } else {
        own_set(sim, config, None)?
    };
    let reserved = diff_drive_reserved_keys(sim, io, config)?;
    if !admission.keys.is_disjoint(&reserved) || !own_keys(expected, config)?.is_disjoint(&reserved)
    {
        return Err(Error::DiffDriveIdentity);
    }
    // Shape/native/lease and both actual/planned peer-key checks precede this
    // real production expected-set comparison. No Identity error is remapped.
    declared(sim, Some(expected))?;
    preflight(sim, io, config)?;
    if admission.owners.is_empty() {
        return Ok(None);
    }
    let mut endpoints = Vec::new();
    let mut receivers = BTreeMap::new();
    for (id, owner) in &admission.owners {
        let (input, receiver) = mpsc::sync_channel(config.capacity);
        endpoints.push(Endpoint {
            owner: *id,
            command_key: config.topic_key(&owner.metadata.topic)?,
            odometry_key: config.topic_key(&owner.metadata.odom_topic)?,
            input,
        });
        receivers.insert(*id, receiver);
    }
    let installed = io.diff_drive_outputs(Setup {
        endpoints,
        max_bytes: config.max_message_bytes,
        drops: Arc::clone(&io.ingress_drops),
        epoch: Arc::clone(&io.epoch),
    })?;
    prepared(&admission.owners, &installed, config)?;
    let Installed { outputs, lease_ids } = installed;
    let mut streams = BTreeMap::new();
    for (id, output) in outputs {
        let owner = admission
            .owners
            .get(&id)
            .expect("validated installed owner");
        streams.insert(
            id,
            Stream {
                entity: owner.entity,
                identity: owner.identity,
                name: owner.name.clone(),
                parent: owner.parent,
                metadata: owner.metadata.clone(),
                command_key: config.topic_key(&owner.metadata.topic)?,
                odometry_key: config.topic_key(&owner.metadata.odom_topic)?,
                lease_id: lease_ids[&id],
                output,
                receiver: receivers.remove(&id).expect("prepared owner receiver"),
                sequence: 0,
                last_sample: None,
            },
        );
    }
    Ok(Some(Drives {
        streams,
        next_poll: 0,
    }))
}

fn live_owners(
    sim: &Simulation,
    saved: &Option<BTreeMap<u64, Original>>,
    restoring: bool,
) -> Result<BTreeMap<u64, Candidate>, Error> {
    let owners = candidates(sim, saved, restoring)?;
    if let Some(saved) = saved {
        for (id, row) in saved {
            let present = sim.world().iter_entities().any(|node| {
                !node.contains::<PendingResetRemoval>()
                    && (node.get::<EntityId>().map(EntityId::get) == Some(*id)
                        || node.get::<Name>().map(|name| &name.0) == Some(&row.name))
            });
            if present && !owners.contains_key(id) {
                return Err(Error::DiffDriveIdentity);
            }
        }
    }
    Ok(owners)
}

impl Drives {
    fn validate_live(&self, sim: &Simulation, active: bool) -> Result<(), Error> {
        let saved = originals(sim)?;
        retained(sim, self, &saved, active)?;
        let owners = live_owners(sim, &saved, false)?;
        for (id, owner) in &owners {
            let stream = self.streams.get(id).ok_or(Error::DiffDriveIdentity)?;
            if stream.entity != owner.entity || stream.metadata != owner.metadata {
                return Err(Error::DiffDriveIdentity);
            }
        }
        Ok(())
    }
}

impl Server {
    pub(super) fn retire_diff_drive(&mut self) {
        if let Some(drives) = &mut self.diff_drive {
            // This direct retirement operation preserves its old no-output contract.
            drives
                .streams
                .retain(|_, stream| stream.live(&self.simulation));
            if drives.streams.is_empty() {
                self.diff_drive = None;
            }
        }
    }
    pub(super) fn reconcile_diff_drive(&mut self) -> Result<(), Error> {
        let valid = if let Some(drives) = &self.diff_drive {
            drives.validate_live(&self.simulation, true)
        } else {
            originals(&self.simulation).and_then(|saved| {
                if live_owners(&self.simulation, &saved, false)?.is_empty() {
                    Ok(())
                } else {
                    Err(Error::DiffDriveIdentity)
                }
            })
        };
        if let Err(error) = valid {
            self.diff_drive = None;
            self.reset_failed = true;
            return Err(error);
        }
        self.retire_diff_drive();
        Ok(())
    }
    pub(super) fn poll_diff_drive(&mut self) -> Result<(), Error> {
        self.reconcile_diff_drive()?;
        let Some(drives) = &mut self.diff_drive else {
            return Ok(());
        };
        let owners: Vec<_> = drives.streams.keys().copied().collect();
        let mut idle = 0;
        for _ in 0..self.command_budget.get() {
            let message = loop {
                if idle == owners.len() {
                    return Ok(());
                }
                let index = drives.next_poll % owners.len();
                drives.next_poll = (index + 1) % owners.len();
                let stream = &drives.streams[&owners[index]];
                match stream.receiver.try_recv() {
                    Ok(message) => {
                        idle = 0;
                        break (stream, message);
                    }
                    Err(mpsc::TryRecvError::Empty) => idle += 1,
                    Err(mpsc::TryRecvError::Disconnected) => return Err(Error::WorldClosed),
                }
            };
            let (stream, message) = message;
            self.counters.diff_drive_messages_received += 1;
            let accepted = message.0 == self.simulation.epoch()
                && stream.live(&self.simulation)
                && message.1.is_some_and(|[linear, angular]| {
                    gz_sim_scene::set_diff_drive_command(
                        &mut self.simulation,
                        stream.identity,
                        linear,
                        angular,
                    )
                    .is_ok()
                });
            if accepted {
                self.counters.diff_drive_commands_applied += 1;
            } else {
                self.counters.diff_drive_messages_rejected += 1;
            }
        }
        Ok(())
    }
    pub(super) fn publish_odometry(&mut self) {
        let Some(drives) = &mut self.diff_drive else {
            return;
        };
        if drives.validate_live(&self.simulation, false).is_err() {
            return;
        }
        for stream in drives.streams.values_mut() {
            if stream.removed(&self.simulation) {
                continue;
            }
            let Some(sample) = self
                .simulation
                .world()
                .get::<DiffDriveSample>(stream.entity)
            else {
                continue;
            };
            let key = (sample.epoch, sample.time);
            if stream.last_sample == Some(key) {
                continue;
            }
            stream.last_sample = Some(key);
            let Some(message) = (sample.epoch == self.simulation.epoch()
                && sample.time <= self.simulation.elapsed())
            .then(|| packet(&stream.metadata, sample, stream.sequence))
            .flatten() else {
                self.counters.invalid_odometry_samples += 1;
                continue;
            };
            let interested = if self.interest_policy == InterestPolicy::Always {
                true
            } else {
                match stream.output.matching() {
                    Ok(value) => value,
                    Err(_) => {
                        self.counters.interest_failures += 1;
                        continue;
                    }
                }
            };
            if !interested {
                continue;
            }
            stream.sequence = stream.sequence.wrapping_add(1);
            if self
                .world_io
                .publish_odometry(&stream.output, message.encode_to_vec())
                .is_ok()
            {
                self.counters.odometries_queued += 1;
            } else {
                self.counters.odometry_failures += 1;
            }
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/diff_drive.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/multiple_diff_drives.rs"]
mod multiple_diff_drives;

#[cfg(test)]
#[path = "../tests/private/multiple_diff_drive_boundaries.rs"]
mod multiple_diff_drive_boundaries;
