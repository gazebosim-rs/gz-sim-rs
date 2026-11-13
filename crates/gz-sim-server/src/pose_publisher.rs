// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical model-local PosePublisher, independent of viewer/world pose streaming.
use super::*;
use gz_sim_proto::msgs::{Header, Pose, PoseV, header::Map};
use gz_sim_scene::{
    EntityKind, Name, PendingResetRemoval, PosePublication, PoseSample, StaticPoseSample,
};

pub(super) struct Stream {
    entity: Entity,
    identity: u64,
    frames: Vec<(u64, Entity)>,
    pub(super) metadata: PosePublication,
    pub(super) endpoint: String,
    pub(super) publisher: Arc<world_io::PosePublisherOutput>,
    last_sample: Option<(u64, u64)>,
    static_channel: bool,
}
fn channels(metadata: &PosePublication) -> impl Iterator<Item = (bool, &str)> {
    std::iter::once((false, metadata.topic.as_str())).chain(
        metadata
            .static_publication
            .iter()
            .map(|s| (true, s.topic.as_str())),
    )
}
fn channel_metadata(metadata: &PosePublication, fixed: bool) -> PosePublication {
    let mut result = metadata.clone();
    if fixed {
        let channel = metadata
            .static_publication
            .as_ref()
            .expect("selected static channel");
        result.topic = channel.topic.clone();
        result.poses = channel.poses.clone();
    }
    result.static_publication = None;
    result
}
fn valid_metadata(metadata: &PosePublication, owner: u64) -> bool {
    if let Some(fixed) = &metadata.static_publication {
        let mut all = metadata.clone();
        all.static_publication = None;
        all.poses.extend(fixed.poses.iter().cloned());
        all.poses
            .sort_by(|a, b| a.child_frame_id.cmp(&b.child_frame_id));
        let mut static_topic = all.clone();
        static_topic.topic = fixed.topic.clone();
        return metadata.topic != fixed.topic
            && metadata
                .poses
                .windows(2)
                .all(|p| p[0].child_frame_id < p[1].child_frame_id)
            && fixed
                .poses
                .windows(2)
                .all(|p| p[0].child_frame_id < p[1].child_frame_id)
            && valid_metadata(&all, owner)
            && valid_metadata(&static_topic, owner);
    }

    owner != 0
        && !metadata.model_name.is_empty()
        && !metadata.model_name.contains("::")
        && metadata.topic.starts_with('/')
        && metadata.topic.len() > 1
        && metadata.topic.len() <= 65535
        && !metadata
            .topic
            .bytes()
            .any(|b| b.is_ascii_control() || b.is_ascii_whitespace())
        && !["~", "//", "@", ":="]
            .iter()
            .any(|p| metadata.topic.contains(p))
        && (1..=128).contains(&metadata.poses.len())
        && metadata
            .poses
            .windows(2)
            .all(|pair| pair[0].child_frame_id < pair[1].child_frame_id)
        && metadata.poses.iter().enumerate().all(|(index, frame)| {
            frame.identity != 0
                && !frame.frame_id.is_empty()
                && metadata.poses[..index]
                    .iter()
                    .all(|prior| prior.identity != frame.identity)
                && if frame.identity == owner {
                    frame.child_frame_id == metadata.model_name && !frame.frame_id.contains("::")
                } else {
                    frame.frame_id == metadata.model_name
                        && frame
                            .child_frame_id
                            .strip_prefix(&(metadata.model_name.clone() + "::"))
                            .is_some_and(|name| !name.is_empty() && !name.contains("::"))
                }
        })
}
fn packet(metadata: &PosePublication, owner: u64, sample: &PoseSample) -> Option<PoseV> {
    if !valid_metadata(metadata, owner) || metadata.poses.len() != sample.poses.len() {
        return None;
    }
    let mut poses = Vec::with_capacity(sample.poses.len());
    for (frame, value) in metadata.poses.iter().zip(&sample.poses) {
        if frame.identity != value.identity
            || !value
                .pose
                .translation
                .iter()
                .chain(value.pose.rotation.iter())
                .all(|v| v.is_finite())
            || (value.pose.rotation.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() > 1e-12
        {
            return None;
        }
        poses.push(Pose {
            header: Some(Header {
                stamp: Some(time(sample.time.as_nanos() as i128).ok()?),
                data: vec![
                    Map {
                        key: "frame_id".into(),
                        value: vec![frame.frame_id.clone()],
                    },
                    Map {
                        key: "child_frame_id".into(),
                        value: vec![frame.child_frame_id.clone()],
                    },
                ],
            }),
            name: frame.child_frame_id.clone(),
            id: 0,
            position: Some(vector3(value.pose.translation)),
            orientation: Some(quaternion(value.pose.rotation)),
        });
    }
    Some(PoseV {
        header: None,
        pose: poses,
    })
}
// Preserve owner/metadata/output prefix used by existing loaded-reset controls.
pub(super) type Prepared = BTreeMap<
    String,
    (
        u64,
        PosePublication,
        Arc<world_io::PosePublisherOutput>,
        String,
    ),
>;
const MAX_POSE_PUBLISHERS: usize = 8;

struct Original {
    identity: u64,
    scoped_name: String,
    metadata: PosePublication,
    endpoints: Vec<(bool, String)>,
}

fn world_scope(sim: &Simulation) -> Result<&str, Error> {
    let mut worlds = sim
        .world()
        .iter_entities()
        .filter(|entity| entity.get::<EntityKind>() == Some(&EntityKind::World));
    let world = worlds.next().ok_or(Error::PosePublisherIdentity)?;
    if worlds.next().is_some() {
        return Err(Error::PosePublisherIdentity);
    }
    world
        .get::<Name>()
        .map(|name| name.0.as_str())
        .filter(|name| !name.is_empty())
        .ok_or(Error::PosePublisherIdentity)
}

fn valid_owner(
    world: &str,
    id: u64,
    kind: EntityKind,
    name: &str,
    metadata: &PosePublication,
) -> bool {
    kind == EntityKind::Model
        && valid_metadata(metadata, id)
        && name
            .strip_prefix(world)
            .and_then(|tail| tail.strip_prefix("::"))
            == Some(metadata.model_name.as_str())
}

fn originals(
    sim: &Simulation,
    options: &ServerOptions,
    occupied: &BTreeSet<String>,
) -> Result<Vec<Original>, Error> {
    let rows: Vec<_> = gz_sim_scene::initial_pose_publications_with_identity(sim)?
        .map(|(id, kind, name, metadata)| (id, kind, name.to_owned(), metadata.clone()))
        .collect();
    if rows.len() > MAX_POSE_PUBLISHERS {
        return Err(Error::PosePublisherIdentity);
    }
    if rows.is_empty() {
        return Ok(Vec::new());
    }
    let world = world_scope(sim)?;
    let mut identities = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut frame_owners = BTreeMap::new();
    let mut keys = occupied.clone();
    let mut selected = Vec::new();
    for (identity, kind, scoped_name, metadata) in rows {
        if !valid_owner(world, identity, kind, &scoped_name, &metadata)
            || !identities.insert(identity)
            || !names.insert(scoped_name.clone())
        {
            return Err(Error::PosePublisherIdentity);
        }
        for id in std::iter::once(identity).chain(
            metadata
                .poses
                .iter()
                .chain(
                    metadata
                        .static_publication
                        .iter()
                        .flat_map(|channel| &channel.poses),
                )
                .map(|frame| frame.identity),
        ) {
            if frame_owners
                .insert(id, identity)
                .is_some_and(|owner| owner != identity)
            {
                return Err(Error::PosePublisherIdentity);
            }
        }
        let mut endpoints = Vec::new();
        for (fixed, topic) in channels(&metadata) {
            let key = options.topic_key(&options::as_valid_topic(topic))?;
            if !keys.insert(key.clone()) {
                return Err(Error::PosePublisherIdentity);
            }
            endpoints.push((fixed, key));
        }
        selected.push(Original {
            identity,
            scoped_name,
            metadata,
            endpoints,
        });
    }
    Ok(selected)
}

pub(super) fn prepare_reset(
    server: &Server,
    occupied: &BTreeSet<String>,
) -> Result<Prepared, Error> {
    let mut keys =
        pose_publisher_reserved_keys(&server.simulation, &server.world_io, &server.network_config)?;
    keys.extend(occupied.iter().cloned());
    let original = originals(&server.simulation, &server.network_config, &keys)?;
    let candidates: Vec<_> = server
        .simulation
        .world()
        .iter_entities()
        .filter(|entity| {
            !entity.contains::<PendingResetRemoval>() && entity.contains::<PosePublication>()
        })
        .map(|entity| entity.id())
        .collect();
    if candidates.len() > MAX_POSE_PUBLISHERS {
        return Err(Error::PosePublisherIdentity);
    }
    let mut current_ids = BTreeSet::new();
    let mut current_names = BTreeSet::new();
    let mut current_keys = keys.clone();
    for entity in candidates {
        let node = server.simulation.world().entity(entity);
        let id = node
            .get::<EntityId>()
            .ok_or(Error::PosePublisherIdentity)?
            .get();
        let name = &node.get::<Name>().ok_or(Error::PosePublisherIdentity)?.0;
        let metadata = node.get::<PosePublication>().expect("candidate metadata");
        if node.get::<EntityKind>() != Some(&EntityKind::Model)
            || entity_by_id(server.simulation.world(), id) != Some(entity)
            || !valid_owner(
                world_scope(&server.simulation)?,
                id,
                EntityKind::Model,
                name,
                metadata,
            )
            || !current_ids.insert(id)
            || !current_names.insert(name.as_str())
        {
            return Err(Error::PosePublisherIdentity);
        }
        for (_, topic) in channels(metadata) {
            let key = server
                .network_config
                .topic_key(&options::as_valid_topic(topic))?;
            if !current_keys.insert(key) {
                return Err(Error::PosePublisherIdentity);
            }
        }
        let initial = original
            .iter()
            .find(|owner| owner.identity == id && &owner.scoped_name == name)
            .ok_or(Error::PosePublisherIdentity)?;
        if &initial.metadata != metadata {
            return Err(Error::ResetPosePublishers);
        }
        // Missing original selected links may be restored by loaded reset;
        // immutable original IDs/metadata still bind their output ownership.
    }
    // Validate the complete retained lease set before looking up or cloning an Arc.
    let mut leases = BTreeSet::new();
    let mut owner_channels = BTreeSet::new();
    for (index, stream) in server.pose_publishers.iter().enumerate() {
        // A valid network remap may change both endpoint keys. Bind each old
        // lease to its immutable owner/channel before considering the new keys.
        let owner = original
            .iter()
            .find(|owner| owner.identity == stream.identity)
            .ok_or(Error::ResetPosePublishers)?;
        let owner_binding_valid = match entity_by_id(server.simulation.world(), stream.identity) {
            Some(entity) => {
                entity == stream.entity
                    && server.simulation.world().get::<EntityKind>(entity)
                        == Some(&EntityKind::Model)
                    && server
                        .simulation
                        .world()
                        .get::<Name>(entity)
                        .map(|name| name.0.as_str())
                        == Some(owner.scoped_name.as_str())
            }
            None => server.simulation.world().get_entity(stream.entity).is_err(),
        };
        if !owner_binding_valid
            || stream.metadata != owner.metadata
            || !stream.publisher.matches_endpoint(&stream.endpoint)
            || !owner
                .endpoints
                .iter()
                .any(|(fixed, _)| *fixed == stream.static_channel)
            || !leases.insert(stream.endpoint.as_str())
            || !owner_channels.insert((stream.identity, stream.static_channel))
            || server.pose_publishers[..index]
                .iter()
                .any(|prior| Arc::ptr_eq(&prior.publisher, &stream.publisher))
        {
            return Err(Error::ResetPosePublishers);
        }
    }
    let mut prepared = BTreeMap::new();
    for owner in original {
        for (_, key) in owner.endpoints {
            let publisher = match server
                .pose_publishers
                .iter()
                .find(|stream| stream.endpoint == key)
            {
                Some(stream) => Arc::clone(&stream.publisher),
                None => server
                    .world_io
                    .pose_publisher_output(key.clone())
                    .map_err(|error| Error::WorldNetwork(error.to_string()))?,
            };
            prepared.insert(
                key,
                (
                    owner.identity,
                    owner.metadata.clone(),
                    publisher,
                    owner.scoped_name.clone(),
                ),
            );
        }
    }
    Ok(prepared)
}
fn bindings(
    sim: &Simulation,
    entity: Entity,
    identity: u64,
    metadata: &PosePublication,
) -> Option<Vec<(u64, Entity)>> {
    let world = sim.world();
    if !valid_metadata(metadata, identity)
        || world.get::<EntityKind>(entity) != Some(&EntityKind::Model)
        || world.get::<EntityId>(entity)?.get() != identity
        || entity_by_id(world, identity) != Some(entity)
    {
        return None;
    }
    let (world_name, model_name) = world.get::<Name>(entity)?.0.split_once("::")?;
    if model_name != metadata.model_name {
        return None;
    }
    let mut frames = Vec::with_capacity(metadata.poses.len());
    for frame in metadata
        .poses
        .iter()
        .chain(metadata.static_publication.iter().flat_map(|s| &s.poses))
    {
        let id = entity_by_id(world, frame.identity)?;
        if frame.identity == identity {
            if id != entity || frame.frame_id != world_name {
                return None;
            }
        } else if world.get::<EntityKind>(id) != Some(&EntityKind::Link)
            || world.get::<bevy_ecs::prelude::ChildOf>(id)?.parent() != entity
            || world.get::<Name>(id)?.0 != format!("{world_name}::{}", frame.child_frame_id)
        {
            return None;
        }
        frames.push((frame.identity, id));
    }
    Some(frames)
}
pub(super) fn streams(
    sim: &Simulation,
    io: &world_io::WorldIo,
    options: &ServerOptions,
    occupied: &BTreeSet<String>,
    prepared: Option<&Prepared>,
) -> Result<Vec<Stream>, Error> {
    let candidates: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|entity| {
            !entity.contains::<PendingResetRemoval>() && entity.contains::<PosePublication>()
        })
        .map(|entity| entity.id())
        .collect();
    if candidates.len() > MAX_POSE_PUBLISHERS {
        return Err(Error::PosePublisherIdentity);
    }
    if candidates.is_empty() {
        return if prepared.is_some_and(|entries| !entries.is_empty()) {
            Err(Error::ResetPosePublishers)
        } else {
            Ok(Vec::new())
        };
    }
    // Preserve the malformed manual/missing-ID refusal before blueprint lookup.
    for &entity in &candidates {
        let identity = sim
            .world()
            .get::<EntityId>(entity)
            .ok_or(Error::PosePublisherIdentity)?
            .get();
        let metadata = sim
            .world()
            .get::<PosePublication>(entity)
            .expect("candidate metadata");
        bindings(sim, entity, identity, metadata).ok_or(Error::PosePublisherIdentity)?;
    }
    let mut keys = pose_publisher_reserved_keys(sim, io, options)?;
    keys.extend(occupied.iter().cloned());
    let original = originals(sim, options, &keys)?;
    let world = world_scope(sim)?;
    let mut identities = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut selected = Vec::new();
    for entity in candidates {
        let identity = sim
            .world()
            .get::<EntityId>(entity)
            .expect("validated owner ID")
            .get();
        let metadata = sim
            .world()
            .get::<PosePublication>(entity)
            .expect("candidate metadata");
        let name = &sim
            .world()
            .get::<Name>(entity)
            .ok_or(Error::PosePublisherIdentity)?
            .0;
        if !valid_owner(world, identity, EntityKind::Model, name, metadata)
            || !identities.insert(identity)
            || !names.insert(name.as_str())
        {
            return Err(Error::PosePublisherIdentity);
        }
        let frames =
            bindings(sim, entity, identity, metadata).ok_or(Error::PosePublisherIdentity)?;
        let owner = original
            .iter()
            .find(|owner| owner.identity == identity && &owner.scoped_name == name)
            .ok_or(Error::PosePublisherIdentity)?;
        for (static_channel, topic) in channels(metadata) {
            let endpoint = options.topic_key(&options::as_valid_topic(topic))?;
            if !keys.insert(endpoint.clone()) {
                return Err(Error::PosePublisherIdentity);
            }
            if &owner.metadata != metadata {
                return Err(if prepared.is_some() {
                    Error::ResetPosePublishers
                } else {
                    Error::PosePublisherIdentity
                });
            }
            if let Some(prepared) = prepared {
                let (original, config, publisher, original_name) =
                    prepared.get(&endpoint).ok_or(Error::ResetPosePublishers)?;
                if *original != identity
                    || config != metadata
                    || original_name != name
                    || !publisher.matches_endpoint(&endpoint)
                {
                    return Err(Error::ResetPosePublishers);
                }
            }
            selected.push((
                entity,
                identity,
                frames.clone(),
                metadata.clone(),
                endpoint,
                static_channel,
            ));
        }
    }
    if prepared.is_some_and(|entries| entries.len() != selected.len()) {
        return Err(Error::ResetPosePublishers);
    }
    let mut result = Vec::new();
    for (entity, identity, frames, metadata, endpoint, static_channel) in selected {
        let publisher = match prepared {
            Some(entries) => Arc::clone(
                &entries
                    .get(&endpoint)
                    .expect("all-set preflight before IO")
                    .2,
            ),
            None => io
                .pose_publisher_output(endpoint.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        result.push(Stream {
            entity,
            identity,
            frames,
            metadata,
            endpoint,
            publisher,
            last_sample: None,
            static_channel,
        });
    }
    result.sort_by_key(|stream| !stream.static_channel);
    Ok(result)
}
impl Stream {
    fn live(&self, sim: &Simulation) -> bool {
        let world = sim.world();
        world.get::<PosePublication>(self.entity) == Some(&self.metadata)
            && bindings(sim, self.entity, self.identity, &self.metadata)
                .is_some_and(|frames| frames == self.frames)
    }
}
impl Server {
    pub(super) fn publish_pose_publishers(&mut self) {
        for stream in &mut self.pose_publishers {
            if !stream.live(&self.simulation) {
                continue;
            }
            let sample = if stream.static_channel {
                self.simulation
                    .world()
                    .get::<StaticPoseSample>(stream.entity)
                    .map(|s| &s.0)
            } else {
                self.simulation.world().get::<PoseSample>(stream.entity)
            };
            let Some(sample) = sample else {
                continue;
            };
            let key = (sample.epoch, sample.revision);
            if stream.last_sample == Some(key) {
                continue;
            }
            stream.last_sample = Some(key);
            let frame = (sample.epoch == self.simulation.epoch()
                && sample.time <= self.simulation.elapsed())
            .then(|| {
                packet(
                    &channel_metadata(&stream.metadata, stream.static_channel),
                    stream.identity,
                    sample,
                )
            })
            .flatten();
            let Some(frame) = frame else {
                self.counters.invalid_pose_publication_samples += 1;
                continue;
            };
            if self
                .world_io
                .publish_pose_publisher(&stream.publisher, frame.encode_to_vec())
                .is_ok()
            {
                self.counters.pose_publications_queued += 1;
            } else {
                self.counters.pose_publisher_failures += 1;
            }
        }
    }
    pub(super) fn retire_pose_publishers(&mut self) {
        self.pose_publishers.retain(|s| s.live(&self.simulation));
        if self.pose_publishers.is_empty() {
            self.pose_publishers = Vec::new();
        }
    }
}
#[cfg(test)]
#[path = "../tests/private/pose_publisher.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/multiple_pose_publishers_native.rs"]
mod multiple_pose_publishers_native;
