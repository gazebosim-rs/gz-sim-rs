// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical loaded joint-state publication; acquisition and cadence belong to Scene.
use super::*;
use gz_sim_proto::msgs::{Axis, Header, Joint, Model, Pose};
use gz_sim_scene::{
    EntityKind, JointStatePublication, JointStateSample, Name, PendingResetRemoval,
};
use std::collections::BTreeSet;

pub(super) struct Stream {
    entity: Entity,
    identity: u64,
    joints: Vec<(u64, Entity)>,
    pub(super) metadata: JointStatePublication,
    pub(super) endpoint: String,
    pub(super) publisher: Arc<world_io::JointStateOutput>,
    last_sample: Option<(u64, u64)>,
}

fn wire_id(id: u64) -> Option<u32> {
    u32::try_from(id).ok().filter(|id| *id != 0)
}
fn valid_pose(translation: [f64; 3], rotation: [f64; 4]) -> bool {
    translation
        .iter()
        .chain(rotation.iter())
        .all(|value| value.is_finite())
        && (rotation.iter().map(|value| value * value).sum::<f64>() - 1.0).abs() <= 1e-12
}
fn pose(translation: [f64; 3], rotation: [f64; 4]) -> Option<Pose> {
    valid_pose(translation, rotation).then(|| Pose {
        position: Some(vector3(translation)),
        orientation: Some(quaternion(rotation)),
        ..Default::default()
    })
}
fn valid_metadata(metadata: &JointStatePublication) -> bool {
    !metadata.model_name.is_empty()
        && !metadata.topic.is_empty()
        && !metadata.joints.is_empty()
        && metadata.joints.len() <= 128
        && metadata
            .joints
            .windows(2)
            .all(|pair| pair[0].id < pair[1].id)
        && metadata.joints.iter().all(|joint| {
            wire_id(joint.id).is_some() && !joint.name.is_empty()
                && !joint.parent.is_empty() && !joint.child.is_empty()
                && valid_pose(joint.pose.translation, joint.pose.rotation)
                && joint.axis.iter().all(|value| value.is_finite())
                && joint.axis.iter().map(|value| value * value).sum::<f64>().is_finite()
                && joint.axis.iter().any(|value| *value != 0.0)
                // Native unrestricted joints carry -/+ infinity, not invented bounds.
                && !joint.lower.is_nan() && joint.lower != f64::INFINITY
                && !joint.upper.is_nan() && joint.upper != f64::NEG_INFINITY
                && joint.lower <= joint.upper
                && joint.damping.is_finite() && joint.damping >= 0.0
        })
        && metadata.joints.iter().enumerate().all(|(index, joint)| {
            metadata.joints[..index]
                .iter()
                .all(|previous| previous.name != joint.name)
        })
}

fn packet(
    metadata: &JointStatePublication,
    identity: u64,
    sample: &JointStateSample,
) -> Option<Model> {
    if !valid_metadata(metadata) || sample.joints.len() != metadata.joints.len() {
        return None;
    }
    let mut joints = Vec::with_capacity(metadata.joints.len());
    for (joint, value) in metadata.joints.iter().zip(&sample.joints) {
        if joint.id != value.id || !value.position.is_finite() || !value.velocity.is_finite() {
            return None;
        }
        let kind = match joint.kind {
            gz_sim_scene::JointStateKind::Revolute => gz_sim_proto::msgs::joint::Type::Revolute,
            gz_sim_scene::JointStateKind::Prismatic => gz_sim_proto::msgs::joint::Type::Prismatic,
        };
        joints.push(Joint {
            r#type: kind as i32,
            id: wire_id(joint.id)?,
            name: joint.name.clone(),
            parent: joint.parent.clone(),
            child: joint.child.clone(),
            pose: Some(pose(joint.pose.translation, joint.pose.rotation)?),
            axis1: Some(Axis {
                xyz: Some(vector3(joint.axis)),
                limit_lower: joint.lower,
                limit_upper: joint.upper,
                damping: joint.damping,
                position: value.position,
                velocity: value.velocity,
                // No force measurement is available. All unsupported fields stay unset/default.
                ..Default::default()
            }),
            ..Default::default()
        });
    }
    Some(Model {
        header: Some(Header {
            stamp: Some(time(sample.time.as_nanos() as i128).ok()?),
            data: Vec::new(),
        }),
        id: wire_id(identity)?,
        name: metadata.model_name.clone(),
        pose: Some(pose(sample.pose.translation, sample.pose.rotation)?),
        joint: joints,
        ..Default::default()
    })
}

// Preserve the original identity/metadata/output prefix used by reset controls.
pub(super) type Prepared = BTreeMap<
    String,
    (
        u64,
        JointStatePublication,
        Arc<world_io::JointStateOutput>,
        String,
    ),
>;
const MAX_JOINT_STATE_PUBLISHERS: usize = 8;
struct Original {
    identity: u64,
    name: String,
    metadata: JointStatePublication,
    endpoint: String,
}
struct Candidate {
    entity: Entity,
    identity: u64,
    name: String,
    metadata: JointStatePublication,
    joints: Vec<(u64, Entity)>,
    endpoint: String,
}
fn original_owners(
    sim: &Simulation,
    options: &ServerOptions,
    occupied: &BTreeSet<String>,
) -> Result<Option<Vec<Original>>, Error> {
    let rows = match gz_sim_scene::initial_joint_state_publications_with_identity(sim) {
        Ok(rows) => rows
            .map(|(id, kind, name, metadata)| (id, kind, name.to_owned(), metadata.clone()))
            .collect::<Vec<_>>(),
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => return Ok(None),
        Err(error) => return Err(error.into()),
    };
    if rows.len() > MAX_JOINT_STATE_PUBLISHERS {
        return Err(Error::JointStateIdentity);
    }
    let mut keys = occupied.clone();
    let mut identities = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut selected_ids = BTreeSet::new();
    let mut result = Vec::new();
    for (identity, kind, name, metadata) in rows {
        let mut worlds = sim
            .world()
            .iter_entities()
            .filter(|node| node.get::<EntityKind>() == Some(&EntityKind::World));
        let world = worlds.next().and_then(|node| node.get::<Name>());
        if kind != EntityKind::Model
            || wire_id(identity).is_none()
            || !valid_metadata(&metadata)
            || worlds.next().is_some()
            || !world.is_some_and(|world| {
                !world.0.is_empty() && name == format!("{}::{}", world.0, metadata.model_name)
            })
            || !identities.insert(identity)
            || !names.insert(name.clone())
        {
            return Err(Error::JointStateIdentity);
        }
        for joint in &metadata.joints {
            if joint.id == identity || !selected_ids.insert(joint.id) {
                return Err(Error::JointStateIdentity);
            }
        }
        let endpoint = options.topic_key(&options::as_valid_topic(&metadata.topic))?;
        if !keys.insert(endpoint.clone()) {
            return Err(Error::JointStateIdentity);
        }
        result.push(Original {
            identity,
            name,
            metadata,
            endpoint,
        });
    }
    if identities.iter().any(|id| selected_ids.contains(id)) {
        return Err(Error::JointStateIdentity);
    }
    Ok(Some(result))
}
fn candidate_owners(
    sim: &Simulation,
    options: &ServerOptions,
    occupied: &BTreeSet<String>,
    original: Option<&[Original]>,
    restoring: bool,
    prepared: bool,
) -> Result<Vec<Candidate>, Error> {
    let nodes = sim
        .world()
        .iter_entities()
        .filter(|node| {
            !node.contains::<PendingResetRemoval>() && node.contains::<JointStatePublication>()
        })
        .map(|node| node.id())
        .collect::<Vec<_>>();
    if nodes.len() > MAX_JOINT_STATE_PUBLISHERS || original.is_none() && nodes.len() > 1 {
        return Err(Error::JointStateIdentity);
    }
    let mut identities = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut joint_ids = BTreeSet::new();
    let mut keys = occupied.clone();
    let mut result = Vec::new();
    for entity in nodes {
        let node = sim.world().entity(entity);
        let identity = node
            .get::<EntityId>()
            .ok_or(Error::JointStateIdentity)?
            .get();
        let metadata = node
            .get::<JointStatePublication>()
            .expect("selected metadata");
        if wire_id(identity).is_none()
            || !valid_metadata(metadata)
            || node.get::<EntityKind>() != Some(&EntityKind::Model)
            || entity_by_id(sim.world(), identity) != Some(entity)
            || !identities.insert(identity)
        {
            return Err(Error::JointStateIdentity);
        }
        let name = node
            .get::<Name>()
            .map(|name| name.0.clone())
            .unwrap_or_default();
        if let Some(original) = original {
            let owner = original
                .iter()
                .find(|owner| owner.identity == identity && owner.name == name)
                .ok_or(Error::JointStateIdentity)?;
            if &owner.metadata != metadata {
                return Err(if prepared || restoring {
                    Error::ResetJointStates
                } else {
                    Error::JointStateIdentity
                });
            }
            if !names.insert(name.clone()) {
                return Err(Error::JointStateIdentity);
            }
        }
        let mut joints = Vec::with_capacity(metadata.joints.len());
        for joint in &metadata.joints {
            if joint.id == identity || !joint_ids.insert(joint.id) {
                return Err(Error::JointStateIdentity);
            }
            if !restoring {
                let id = entity_by_id(sim.world(), joint.id).ok_or(Error::JointStateIdentity)?;
                let child = sim.world().entity(id);
                if child.get::<EntityKind>() != Some(&EntityKind::Joint)
                    || child
                        .get::<bevy_ecs::prelude::ChildOf>()
                        .map(bevy_ecs::prelude::ChildOf::parent)
                        != Some(entity)
                {
                    return Err(Error::JointStateIdentity);
                }
                joints.push((joint.id, id));
            }
        }
        let endpoint = options.topic_key(&options::as_valid_topic(&metadata.topic))?;
        if !keys.insert(endpoint.clone())
            || original.is_some_and(|rows| {
                rows.iter()
                    .any(|owner| owner.identity != identity && owner.endpoint == endpoint)
            })
        {
            return Err(Error::JointStateIdentity);
        }
        result.push(Candidate {
            entity,
            identity,
            name,
            metadata: metadata.clone(),
            joints,
            endpoint,
        });
    }
    if identities.iter().any(|id| joint_ids.contains(id)) {
        return Err(Error::JointStateIdentity);
    }
    Ok(result)
}
pub(super) fn prepare_reset(
    server: &Server,
    occupied: &BTreeSet<String>,
) -> Result<Prepared, Error> {
    let mut keys =
        joint_state_reserved_keys(&server.simulation, &server.world_io, &server.network_config)?;
    keys.extend(occupied.iter().cloned());
    let original = original_owners(&server.simulation, &server.network_config, &keys)?
        .ok_or(gz_sim_scene::SceneResetError::MissingInitialState)?;
    candidate_owners(
        &server.simulation,
        &server.network_config,
        &keys,
        Some(&original),
        true,
        false,
    )?;
    let mut retained_keys = BTreeSet::new();
    let mut retained_ids = BTreeSet::new();
    for (index, stream) in server.joint_states.iter().enumerate() {
        let owner = original
            .iter()
            .find(|owner| owner.identity == stream.identity)
            .ok_or(Error::ResetJointStates)?;
        let owner_binding = match entity_by_id(server.simulation.world(), stream.identity) {
            Some(entity) => {
                entity == stream.entity
                    && server.simulation.world().get::<EntityKind>(entity)
                        == Some(&EntityKind::Model)
                    && server
                        .simulation
                        .world()
                        .get::<Name>(entity)
                        .map(|name| name.0.as_str())
                        == Some(owner.name.as_str())
            }
            None => server.simulation.world().get_entity(stream.entity).is_err(),
        };
        // Removed selected joints are restored from the initial snapshot, not
        // required to exist during reset preparation. Their immutable IDs bind.
        if !owner_binding
            || stream.metadata != owner.metadata
            || stream.joints.iter().map(|(id, _)| *id).ne(owner
                .metadata
                .joints
                .iter()
                .map(|joint| joint.id))
            || stream.joints.iter().any(|(id, old_entity)| {
                match entity_by_id(server.simulation.world(), *id) {
                    Some(entity) => {
                        entity != *old_entity
                            || server.simulation.world().get::<EntityKind>(entity)
                                != Some(&EntityKind::Joint)
                            || server
                                .simulation
                                .world()
                                .get::<bevy_ecs::prelude::ChildOf>(entity)
                                .map(bevy_ecs::prelude::ChildOf::parent)
                                != Some(stream.entity)
                    }
                    None => server.simulation.world().get_entity(*old_entity).is_ok(),
                }
            })
            || !stream.publisher.matches_endpoint(&stream.endpoint)
            || !retained_keys.insert(stream.endpoint.as_str())
            || !retained_ids.insert(stream.identity)
            || server.joint_states[..index]
                .iter()
                .any(|prior| Arc::ptr_eq(&prior.publisher, &stream.publisher))
        {
            return Err(Error::ResetJointStates);
        }
    }
    // All current, original and retained bindings precede lookup/Arc reuse/IO.
    let mut prepared = BTreeMap::new();
    for owner in original {
        let publisher = match server
            .joint_states
            .iter()
            .find(|stream| stream.endpoint == owner.endpoint)
        {
            Some(stream) => Arc::clone(&stream.publisher),
            None => server
                .world_io
                .joint_state_output(owner.endpoint.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        prepared.insert(
            owner.endpoint,
            (owner.identity, owner.metadata, publisher, owner.name),
        );
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
    // Keep malformed manual identity/metadata errors before any initial lookup.
    let nodes = simulation
        .world()
        .iter_entities()
        .filter(|node| {
            !node.contains::<PendingResetRemoval>() && node.contains::<JointStatePublication>()
        })
        .collect::<Vec<_>>();
    if nodes.len() > MAX_JOINT_STATE_PUBLISHERS
        || nodes.iter().any(|node| {
            let Some(id) = node.get::<EntityId>() else {
                return true;
            };
            wire_id(id.get()).is_none()
                || node.get::<EntityKind>() != Some(&EntityKind::Model)
                || entity_by_id(simulation.world(), id.get()) != Some(node.id())
                || !valid_metadata(
                    node.get::<JointStatePublication>()
                        .expect("selected metadata"),
                )
        })
    {
        return Err(Error::JointStateIdentity);
    }
    let mut keys = joint_state_reserved_keys(simulation, io, options)?;
    keys.extend(occupied.iter().cloned());
    let original = original_owners(simulation, options, &keys)?;
    let selected = candidate_owners(
        simulation,
        options,
        &keys,
        original.as_deref(),
        false,
        prepared.is_some(),
    )?;
    if let Some(entries) = prepared {
        if entries.len() != selected.len() {
            return Err(Error::ResetJointStates);
        }
        for owner in &selected {
            let (identity, metadata, output, name) = entries
                .get(&owner.endpoint)
                .ok_or(Error::ResetJointStates)?;
            if *identity != owner.identity
                || metadata != &owner.metadata
                || name != &owner.name
                || !output.matches_endpoint(&owner.endpoint)
            {
                return Err(Error::ResetJointStates);
            }
        }
        let rows = entries.values().collect::<Vec<_>>();
        if rows.iter().enumerate().any(|(index, row)| {
            rows[..index]
                .iter()
                .any(|prior| Arc::ptr_eq(&prior.2, &row.2))
        }) {
            return Err(Error::ResetJointStates);
        }
    }
    let mut result = Vec::new();
    for owner in selected {
        let publisher = match prepared {
            Some(entries) => Arc::clone(
                &entries
                    .get(&owner.endpoint)
                    .expect("all entries preflighted")
                    .2,
            ),
            None => io
                .joint_state_output(owner.endpoint.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        result.push(Stream {
            entity: owner.entity,
            identity: owner.identity,
            joints: owner.joints,
            metadata: owner.metadata,
            endpoint: owner.endpoint,
            publisher,
            last_sample: None,
        });
    }
    Ok(result)
}
impl Stream {
    fn live(&self, simulation: &Simulation) -> bool {
        let world = simulation.world();
        world.get::<EntityKind>(self.entity) == Some(&EntityKind::Model)
            && world.get::<JointStatePublication>(self.entity) == Some(&self.metadata)
            && world.get::<EntityId>(self.entity).map(EntityId::get) == Some(self.identity)
            && entity_by_id(world, self.identity) == Some(self.entity)
            && self.joints.iter().all(|(identity, entity)| {
                world.get::<EntityKind>(*entity) == Some(&EntityKind::Joint)
                    && world.get::<EntityId>(*entity).map(EntityId::get) == Some(*identity)
                    && entity_by_id(world, *identity) == Some(*entity)
                    && world
                        .get::<bevy_ecs::prelude::ChildOf>(*entity)
                        .map(bevy_ecs::prelude::ChildOf::parent)
                        == Some(self.entity)
            })
    }
}
impl Server {
    pub(super) fn publish_joint_states(&mut self) {
        for stream in &mut self.joint_states {
            if !stream.live(&self.simulation) {
                continue;
            }
            let Some(sample) = self
                .simulation
                .world()
                .get::<JointStateSample>(stream.entity)
            else {
                continue;
            };
            let key = (sample.epoch, sample.revision);
            if stream.last_sample == Some(key) {
                continue;
            }
            stream.last_sample = Some(key);
            let message = (sample.epoch == self.simulation.epoch()
                && sample.time <= self.simulation.elapsed())
            .then(|| packet(&stream.metadata, stream.identity, sample))
            .flatten();
            let Some(message) = message else {
                self.counters.invalid_joint_state_samples += 1;
                continue;
            };
            if self
                .world_io
                .publish_joint_state(&stream.publisher, message.encode_to_vec())
                .is_ok()
            {
                self.counters.joint_states_queued += 1;
            } else {
                self.counters.joint_state_failures += 1;
            }
        }
    }
    pub(super) fn retire_joint_states(&mut self) {
        self.joint_states
            .retain(|stream| stream.live(&self.simulation));
        if self.joint_states.is_empty() {
            self.joint_states = Vec::new();
        }
    }
}
#[cfg(test)]
#[path = "../tests/private/joint_state.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/multiple_joint_states.rs"]
mod multiple_joint_states;

#[cfg(test)]
#[path = "../tests/private/multiple_joint_states_native.rs"]
mod multiple_joint_states_native;
