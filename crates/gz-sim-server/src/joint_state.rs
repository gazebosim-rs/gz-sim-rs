// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical loaded joint-state publication; acquisition and cadence belong to Scene.
use super::*;
use gz_sim_proto::msgs::{Axis, Header, Joint, Model, Pose};
use gz_sim_scene::{EntityKind, JointStatePublication, JointStateSample, PendingResetRemoval};
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
        joints.push(Joint {
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

pub(super) type Prepared =
    BTreeMap<String, (u64, JointStatePublication, Arc<world_io::JointStateOutput>)>;
pub(super) fn prepare_reset(
    server: &Server,
    occupied: &BTreeSet<String>,
) -> Result<Prepared, Error> {
    let metadata: Vec<_> = gz_sim_scene::initial_joint_state_publications(&server.simulation)?
        .map(|(identity, metadata)| (identity, metadata.clone()))
        .collect();
    if metadata.len() > 1
        || metadata
            .iter()
            .any(|(identity, metadata)| wire_id(*identity).is_none() || !valid_metadata(metadata))
    {
        return Err(Error::JointStateIdentity);
    }
    let mut prepared = BTreeMap::new();
    for (identity, metadata) in metadata {
        let key = server
            .network_config
            .topic_key(&options::as_valid_topic(&metadata.topic))?;
        if occupied.contains(&key) {
            return Err(Error::JointStateIdentity);
        }
        let publisher = match server
            .joint_states
            .iter()
            .find(|stream| stream.endpoint == key)
        {
            Some(stream) => Arc::clone(&stream.publisher),
            None => server
                .world_io
                .joint_state_output(key.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        prepared.insert(key, (identity, metadata, publisher));
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
        .filter(|entity| {
            !entity.contains::<PendingResetRemoval>() && entity.contains::<JointStatePublication>()
        })
        .map(|entity| entity.id())
        .collect();
    if candidates.len() > 1 {
        return Err(Error::JointStateIdentity);
    }
    let mut selected = Vec::new();
    for entity in candidates {
        let node = simulation.world().entity(entity);
        let metadata = node
            .get::<JointStatePublication>()
            .expect("candidate owns metadata");
        let identity = node
            .get::<EntityId>()
            .ok_or(Error::JointStateIdentity)?
            .get();
        if wire_id(identity).is_none()
            || !valid_metadata(metadata)
            || node.get::<EntityKind>() != Some(&EntityKind::Model)
            || entity_by_id(simulation.world(), identity) != Some(entity)
        {
            return Err(Error::JointStateIdentity);
        }
        let mut joints = Vec::with_capacity(metadata.joints.len());
        for joint in &metadata.joints {
            let id = entity_by_id(simulation.world(), joint.id).ok_or(Error::JointStateIdentity)?;
            let child = simulation.world().entity(id);
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
        let endpoint = options.topic_key(&options::as_valid_topic(&metadata.topic))?;
        if occupied.contains(&endpoint) {
            return Err(Error::JointStateIdentity);
        }
        if let Some(prepared) = prepared {
            let (original_identity, original_metadata, _) =
                prepared.get(&endpoint).ok_or(Error::ResetJointStates)?;
            if *original_identity != identity || original_metadata != metadata {
                return Err(Error::ResetJointStates);
            }
        }
        selected.push((entity, identity, joints, metadata.clone(), endpoint));
    }
    if prepared.is_some_and(|prepared| prepared.len() != selected.len()) {
        return Err(Error::ResetJointStates);
    }
    let mut result = Vec::new();
    for (entity, identity, joints, metadata, endpoint) in selected {
        let publisher = match prepared {
            Some(prepared) => Arc::clone(&prepared.get(&endpoint).expect("validated before IO").2),
            None => io
                .joint_state_output(endpoint.clone())
                .map_err(|error| Error::WorldNetwork(error.to_string()))?,
        };
        result.push(Stream {
            entity,
            identity,
            joints,
            metadata,
            endpoint,
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
