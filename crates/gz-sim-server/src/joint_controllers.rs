// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Sixteen bounded Double inputs on at most eight admitted loaded controller chains.
use super::*;
use crate::joint_controller_io::{Endpoint, Lease, Setup};
use bevy_ecs::prelude::ChildOf;
use gz_sim_scene::{
    BodyHandle, EntityKind, JointControllerCommandError, JointControllerConfiguration,
    JointDefinition, JointHandle, Name, PendingResetRemoval,
};
use std::{collections::BTreeSet, sync::mpsc};

type Metadata = BTreeMap<u64, JointControllerConfiguration>;
const MAX_OWNERS: usize = 8;
pub(super) const MAX_INPUTS: usize = 16;

fn valid(metadata: &JointControllerConfiguration) -> bool {
    !metadata.model_name.is_empty()
        && !metadata.joint_name.is_empty()
        && metadata.topic.starts_with('/')
        && options::as_valid_topic(&metadata.topic) == metadata.topic
}
#[derive(Clone, Copy, PartialEq, Eq)]
struct PhysicalJoint {
    native: Option<gz_physics::JointId>,
    child: u64,
}
impl PhysicalJoint {
    fn resolve(sim: &Simulation, entity: Entity, _id: u64) -> Option<Self> {
        let world = sim.world();
        if let Some(joint) = world.get::<JointHandle>(entity) {
            let body = world.get::<BodyHandle>(entity_by_id(world, joint.child())?)?;
            let backend = world.get_resource::<gz_physics::PhysicsBackend>()?;
            // The scene binding witness validates the coordinate of the controlled axis.
            return (backend.joint_child(joint.id()) == Some(body.id())).then_some(Self {
                native: Some(joint.id()),
                child: joint.child(),
            });
        }
        #[cfg(feature = "gpu-batch")]
        if let Some(child) = gz_sim_scene::gpu_scene::planar_tool::controlled_child(sim, _id) {
            return Some(Self {
                native: None,
                child,
            });
        }
        None
    }
    fn conflicts(self, other: Self) -> bool {
        self.child == other.child || self.native.is_some() && self.native == other.native
    }
}
struct Original {
    identity: u64,
    name: String,
    parent: u64,
    metadata: JointControllerConfiguration,
}
struct Candidate {
    entity: Entity,
    identity: u64,
    name: String,
    parent: u64,
    model: Entity,
    metadata: JointControllerConfiguration,
    joint: PhysicalJoint,
    endpoint: String,
}
fn group(
    owners: &mut BTreeMap<u64, (String, usize)>,
    parent: u64,
    metadata: &JointControllerConfiguration,
) -> Result<(), Error> {
    if !owners.contains_key(&parent)
        && owners
            .values()
            .any(|(name, _)| name == &metadata.model_name)
    {
        return Err(Error::JointControllerIdentity);
    }
    let owner = owners
        .entry(parent)
        .or_insert_with(|| (metadata.model_name.clone(), 0));
    if owner.0 != metadata.model_name || owner.1 == 2 {
        return Err(Error::JointControllerIdentity);
    }
    owner.1 += 1;
    if owners.len() > MAX_OWNERS {
        return Err(Error::JointControllerIdentity);
    }
    Ok(())
}
fn world_identity(sim: &Simulation) -> Result<(Entity, String), Error> {
    let mut worlds = sim
        .world()
        .iter_entities()
        .filter(|node| node.get::<EntityKind>() == Some(&EntityKind::World));
    let world = worlds.next().ok_or(Error::JointControllerIdentity)?;
    let name = world.get::<Name>().ok_or(Error::JointControllerIdentity)?;
    if name.0.is_empty() || worlds.next().is_some() {
        return Err(Error::JointControllerIdentity);
    }
    Ok((world.id(), name.0.clone()))
}
fn originals(sim: &Simulation) -> Result<Option<Vec<Original>>, Error> {
    let rows = match gz_sim_scene::initial_joint_controller_configurations_with_identity(sim) {
        Ok(rows) => rows
            .take(MAX_INPUTS + 1)
            .map(|row| {
                (
                    row.id,
                    row.kind,
                    row.name.to_owned(),
                    row.parent,
                    row.configuration.clone(),
                )
            })
            .collect::<Vec<_>>(),
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => return Ok(None),
        Err(error) => return Err(error.into()),
    };
    if rows.len() > MAX_INPUTS {
        return Err(Error::JointControllerIdentity);
    }
    if rows.is_empty() {
        return Ok(Some(Vec::new()));
    }
    let (_, world_name) = world_identity(sim)?;
    let mut ids = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut owners = BTreeMap::new();
    let mut result = Vec::new();
    for (identity, kind, name, parent, metadata) in rows {
        let parent = parent.ok_or(Error::JointControllerIdentity)?;
        if identity == 0
            || parent == 0
            || identity == parent
            || kind != EntityKind::Joint
            || !valid(&metadata)
            || name
                != format!(
                    "{world_name}::{}::{}",
                    metadata.model_name, metadata.joint_name
                )
            || !ids.insert(identity)
            || !names.insert(name.clone())
        {
            return Err(Error::JointControllerIdentity);
        }
        group(&mut owners, parent, &metadata)?;
        result.push(Original {
            identity,
            name,
            parent,
            metadata,
        });
    }
    if owners.keys().any(|id| ids.contains(id)) {
        return Err(Error::JointControllerIdentity);
    }
    Ok(Some(result))
}
pub(super) fn initial(sim: &Simulation) -> Result<Metadata, Error> {
    Ok(originals(sim)?
        .unwrap_or_default()
        .into_iter()
        .map(|row| (row.identity, row.metadata))
        .collect())
}
fn candidates(sim: &Simulation, config: &ServerOptions) -> Result<Vec<Candidate>, Error> {
    candidates_for(sim, config, None)
}
fn candidates_for(
    sim: &Simulation,
    config: &ServerOptions,
    trusted: Option<&Metadata>,
) -> Result<Vec<Candidate>, Error> {
    let nodes = sim
        .world()
        .iter_entities()
        .filter(|node| {
            !node.contains::<PendingResetRemoval>()
                && node.contains::<JointControllerConfiguration>()
        })
        .take(MAX_INPUTS + 1)
        .map(|node| node.id())
        .collect::<Vec<_>>();
    if nodes.len() > MAX_INPUTS {
        return Err(Error::JointControllerIdentity);
    }
    if nodes.is_empty() {
        return Ok(Vec::new());
    }
    let (world, world_name) = world_identity(sim)?;
    let mut ids = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut keys = BTreeSet::new();
    let mut owners = BTreeMap::new();
    let mut result: Vec<Candidate> = Vec::new();
    for entity in nodes {
        let node = sim.world().entity(entity);
        let metadata = node
            .get::<JointControllerConfiguration>()
            .ok_or(Error::JointControllerIdentity)?;
        let identity = node
            .get::<EntityId>()
            .ok_or(Error::JointControllerIdentity)?
            .get();
        let name = node.get::<Name>().ok_or(Error::JointControllerIdentity)?;
        let model = node
            .get::<ChildOf>()
            .ok_or(Error::JointControllerIdentity)?
            .parent();
        let parent_node = sim
            .world()
            .get_entity(model)
            .map_err(|_| Error::JointControllerIdentity)?;
        let parent = parent_node
            .get::<EntityId>()
            .ok_or(Error::JointControllerIdentity)?
            .get();
        let parent_name = parent_node
            .get::<Name>()
            .ok_or(Error::JointControllerIdentity)?;
        let joint =
            PhysicalJoint::resolve(sim, entity, identity).ok_or(Error::JointControllerIdentity)?;
        let definition = node
            .get::<JointDefinition>()
            .ok_or(Error::JointControllerIdentity)?;
        let child = entity_by_id(sim.world(), joint.child).ok_or(Error::JointControllerIdentity)?;
        let child_node = sim.world().entity(child);
        let binding_matches = if let Some(original) = trusted {
            original.get(&identity).is_some_and(|configuration| {
                gz_sim_scene::joint_controller_binding_matches_configuration(
                    sim,
                    identity,
                    configuration,
                )
            })
        } else {
            gz_sim_scene::joint_controller_binding_matches(sim, identity)
        };
        if !binding_matches
            || !valid(metadata)
            || node.get::<EntityKind>() != Some(&EntityKind::Joint)
            || entity_by_id(sim.world(), identity) != Some(entity)
            || parent_node.get::<EntityKind>() != Some(&EntityKind::Model)
            || entity_by_id(sim.world(), parent) != Some(model)
            || parent_node.get::<ChildOf>().map(ChildOf::parent) != Some(world)
            || parent_name.0 != format!("{world_name}::{}", metadata.model_name)
            || name.0 != format!("{}::{}", parent_name.0, metadata.joint_name)
            || definition.0.name != metadata.joint_name
            || child_node.get::<EntityKind>() != Some(&EntityKind::Link)
            || child_node.get::<ChildOf>().map(ChildOf::parent) != Some(model)
            || !child_node.get::<Name>().is_some_and(|child_name| {
                child_name.0 == format!("{}::{}", parent_name.0, definition.0.child)
            })
            || !ids.insert(identity)
            || !names.insert(name.0.clone())
            || result.iter().any(|prior| prior.joint.conflicts(joint))
        {
            return Err(Error::JointControllerIdentity);
        }
        group(&mut owners, parent, metadata)?;
        let endpoint = config.topic_key(&metadata.topic)?;
        if !keys.insert(endpoint.clone()) {
            return Err(Error::JointControllerIdentity);
        }
        result.push(Candidate {
            entity,
            identity,
            name: name.0.clone(),
            parent,
            model,
            metadata: metadata.clone(),
            joint,
            endpoint,
        });
    }
    if owners.keys().any(|id| ids.contains(id)) {
        return Err(Error::JointControllerIdentity);
    }
    Ok(result)
}
struct Stream {
    entity: Entity,
    identity: u64,
    metadata: JointControllerConfiguration,
    joint: PhysicalJoint,
    _lease: Arc<Lease>,
    receiver: mpsc::Receiver<crate::joint_controller_io::Command>,
    name: String,
    parent: u64,
    model: Entity,
    endpoint: String,
}
impl Stream {
    fn removed(&self, sim: &Simulation) -> bool {
        sim.world().get_entity(self.entity).is_err()
            || sim
                .world()
                .get::<PendingResetRemoval>(self.entity)
                .is_some()
    }
    fn live(&self, sim: &Simulation) -> bool {
        let world = sim.world();
        gz_sim_scene::joint_controller_binding_matches(sim, self.identity)
            && world.get::<EntityKind>(self.entity) == Some(&EntityKind::Joint)
            && world.get::<JointControllerConfiguration>(self.entity) == Some(&self.metadata)
            && world.get::<EntityId>(self.entity).map(EntityId::get) == Some(self.identity)
            && entity_by_id(world, self.identity) == Some(self.entity)
            && world
                .get::<Name>(self.entity)
                .is_some_and(|name| name.0 == self.name)
            && world.get::<ChildOf>(self.entity).map(ChildOf::parent) == Some(self.model)
            && entity_by_id(world, self.parent) == Some(self.model)
            && world.get::<EntityKind>(self.model) == Some(&EntityKind::Model)
            && PhysicalJoint::resolve(sim, self.entity, self.identity) == Some(self.joint)
            && world.get::<Name>(self.model).is_some_and(|name| {
                self.name == format!("{}::{}", name.0, self.metadata.joint_name)
            })
            && world.get::<ChildOf>(self.model).is_some_and(|parent| {
                world.get::<EntityKind>(parent.parent()) == Some(&EntityKind::World)
                    && world.get::<Name>(parent.parent()).is_some_and(|name| {
                        world.get::<Name>(self.model).is_some_and(|model| {
                            model.0 == format!("{}::{}", name.0, self.metadata.model_name)
                        })
                    })
            })
            && world
                .get::<JointDefinition>(self.entity)
                .is_some_and(|definition| {
                    definition.0.name == self.metadata.joint_name
                        && entity_by_id(world, self.joint.child).is_some_and(|child| {
                            world.get::<EntityKind>(child) == Some(&EntityKind::Link)
                                && world.get::<ChildOf>(child).map(ChildOf::parent)
                                    == Some(self.model)
                                && world.get::<Name>(child).is_some_and(|name| {
                                    world.get::<Name>(self.model).is_some_and(|model| {
                                        name.0 == format!("{}::{}", model.0, definition.0.child)
                                    })
                                })
                        })
                })
            && self._lease.matches_endpoint(&self.endpoint)
    }
}
pub(super) struct Controllers {
    streams: Vec<Stream>,
}
fn retained(
    sim: &Simulation,
    controllers: &Controllers,
    original: &[Original],
) -> Result<(), Error> {
    if controllers.streams.len() > MAX_INPUTS {
        return Err(Error::JointControllerIdentity);
    }
    for (index, stream) in controllers.streams.iter().enumerate() {
        let row = original
            .iter()
            .find(|row| row.identity == stream.identity)
            .ok_or(Error::JointControllerIdentity)?;
        if stream.metadata != row.metadata
            || stream.name != row.name
            || stream.parent != row.parent
            || !stream._lease.matches_endpoint(&stream.endpoint)
            || controllers.streams[..index].iter().any(|prior| {
                prior.identity == stream.identity
                    || prior.entity == stream.entity
                    || prior.endpoint == stream.endpoint
                    || prior.joint.conflicts(stream.joint)
                    || Arc::ptr_eq(&prior._lease, &stream._lease)
            })
        {
            return Err(Error::JointControllerIdentity);
        }
        if let Some(entity) = entity_by_id(sim.world(), stream.identity)
            && !sim.world().entity(entity).contains::<PendingResetRemoval>()
            && (entity != stream.entity || !stream.live(sim))
        {
            return Err(Error::JointControllerIdentity);
        }
        if !stream.removed(sim) && !stream.live(sim) {
            return Err(Error::JointControllerIdentity);
        }
    }
    Ok(())
}
struct Admission {
    metadata: Metadata,
    owners: Vec<Candidate>,
    keys: BTreeSet<String>,
}
fn own_set(
    sim: &Simulation,
    config: &ServerOptions,
    reset: Option<&Server>,
) -> Result<Admission, Error> {
    own_set_for(sim, config, reset, false)
}
fn own_set_for(
    sim: &Simulation,
    config: &ServerOptions,
    reset: Option<&Server>,
    restoring: bool,
) -> Result<Admission, Error> {
    let saved = originals(sim)?;
    let expected: Metadata = saved
        .as_deref()
        .unwrap_or_default()
        .iter()
        .map(|row| (row.identity, row.metadata.clone()))
        .collect();
    let mut keys = BTreeSet::new();
    let mut original_keys = BTreeMap::new();
    for row in saved.as_deref().unwrap_or_default() {
        let key = config.topic_key(&row.metadata.topic)?;
        if !keys.insert(key.clone()) {
            return Err(Error::JointControllerIdentity);
        }
        original_keys.insert(key, row.identity);
    }
    if restoring {
        for row in saved.as_deref().unwrap_or_default() {
            if let Some(entity) = entity_by_id(sim.world(), row.identity)
                && !sim.world().entity(entity).contains::<PendingResetRemoval>()
                && (sim
                    .world()
                    .get::<JointControllerConfiguration>(entity)
                    .is_none()
                    || !gz_sim_scene::joint_controller_binding_matches_configuration(
                        sim,
                        row.identity,
                        &row.metadata,
                    ))
            {
                return Err(Error::JointControllerIdentity);
            }
        }
    }
    let current = if restoring {
        candidates_for(sim, config, Some(&expected))?
    } else {
        candidates(sim, config)?
    };
    let actual: Metadata = current
        .iter()
        .map(|row| (row.identity, row.metadata.clone()))
        .collect();
    if let Some(server) = reset {
        if let Some(controllers) = &server.joint_controllers {
            retained(sim, controllers, saved.as_deref().unwrap_or_default())?;
        }
        if actual
            .iter()
            .any(|(id, value)| expected.get(id) != Some(value))
        {
            return Err(Error::ResetJointControllers);
        }
        for row in saved.as_deref().unwrap_or_default() {
            if let Some(entity) = entity_by_id(sim.world(), row.identity)
                && !sim.world().entity(entity).contains::<PendingResetRemoval>()
                && !current.iter().any(|candidate| candidate.entity == entity)
            {
                return Err(Error::ResetJointControllers);
            }
        }
    } else if !restoring && actual != expected {
        return Err(Error::ResetJointControllers);
    }
    for candidate in &current {
        let row = saved
            .as_deref()
            .unwrap_or_default()
            .iter()
            .find(|row| row.identity == candidate.identity)
            .ok_or(Error::ResetJointControllers)?;
        if row.name != candidate.name || row.parent != candidate.parent {
            return Err(Error::JointControllerIdentity);
        }
        if restoring
            && original_keys
                .get(&candidate.endpoint)
                .is_some_and(|owner| *owner != candidate.identity)
        {
            return Err(Error::JointControllerIdentity);
        }
        keys.insert(candidate.endpoint.clone());
    }
    Ok(Admission {
        metadata: expected,
        owners: current,
        keys,
    })
}
pub(super) fn legacy_occupied(
    sim: &Simulation,
    config: &ServerOptions,
    restoring: bool,
) -> Result<BTreeSet<String>, Error> {
    let mut keys = BTreeSet::from([config.topic_key("/clock")?]);
    for entity in sim.world().iter_entities() {
        if entity.contains::<PendingResetRemoval>() {
            continue;
        }
        if let Some(metadata) = entity.get::<ImuPublication>() {
            let topic = options::as_valid_topic(&metadata.topic);
            keys.insert(config.topic_key(&topic)?);
            keys.insert(config.topic_key(&format!("{topic}/set_rate"))?);
        }
        if let Some(metadata) = entity.get::<gz_sim_scene::LidarPublication>() {
            keys.insert(config.topic_key(&metadata.topic)?);
        }
        if let Some(metadata) = entity.get::<gz_sim_scene::DiffDriveConfiguration>() {
            keys.insert(config.topic_key(&metadata.topic)?);
            keys.insert(config.topic_key(&metadata.odom_topic)?);
        }
        if let Some(metadata) = entity.get::<gz_sim_scene::JointForceConfiguration>() {
            keys.insert(config.topic_key(&options::as_valid_topic(&format!(
                "/model/{}/joint/{}/cmd_force",
                metadata.model_name(),
                metadata.joint_name()
            )))?);
        }
    }
    if restoring {
        match gz_sim_scene::initial_imu_publications(sim) {
            Ok(values) => {
                for metadata in values {
                    let topic = options::as_valid_topic(&metadata.topic);
                    keys.insert(config.topic_key(&topic)?);
                    keys.insert(config.topic_key(&format!("{topic}/set_rate"))?);
                }
                for metadata in gz_sim_scene::initial_lidar_publications(sim)? {
                    keys.insert(config.topic_key(&metadata.topic)?);
                }
                for metadata in gz_sim_scene::initial_diff_drive_configurations(sim)? {
                    keys.insert(config.topic_key(&metadata.topic)?);
                    keys.insert(config.topic_key(&metadata.odom_topic)?);
                }
                for (_, metadata) in gz_sim_scene::initial_joint_force_configurations(sim)? {
                    keys.insert(config.topic_key(&options::as_valid_topic(&format!(
                        "/model/{}/joint/{}/cmd_force",
                        metadata.model_name(),
                        metadata.joint_name()
                    )))?);
                }
            }
            Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
            Err(error) => return Err(error.into()),
        }
    }
    Ok(keys)
}

/// Pure own-family admission for the shared legacy error-priority selector.
pub(super) fn boundary_keys(
    sim: &Simulation,
    config: &ServerOptions,
    reset: Option<&Server>,
) -> Result<BTreeSet<String>, Error> {
    own_set(sim, config, reset).map(|admission| admission.keys)
}
/// Complete constructor admission before any input setup or queue allocation.
pub(super) fn preflight(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<Metadata, Error> {
    let admission = own_set(sim, config, None)?;
    let occupied = joint_controller_reserved_keys(sim, io, config)?;
    if !admission.keys.is_disjoint(&occupied) {
        return Err(Error::JointControllerIdentity);
    }
    Ok(admission.metadata)
}
/// Borrow old leases against their actual declaration keys, then plan new keys.
pub(super) fn prepare_reset(server: &Server) -> Result<Metadata, Error> {
    let admission = own_set(&server.simulation, &server.network_config, Some(server))?;
    let occupied = joint_controller_reserved_keys(
        &server.simulation,
        &server.world_io,
        &server.network_config,
    )?;
    if !admission.keys.is_disjoint(&occupied) {
        return Err(Error::JointControllerIdentity);
    }
    Ok(admission.metadata)
}
fn prepared(owners: &[Candidate], leases: &[Arc<Lease>]) -> Result<(), Error> {
    if leases.len() != owners.len() || owners.len() > MAX_INPUTS {
        return Err(Error::JointControllerIdentity);
    }
    for (index, (owner, lease)) in owners.iter().zip(leases).enumerate() {
        if !lease.matches_endpoint(&owner.endpoint)
            || leases[..index]
                .iter()
                .any(|previous| Arc::ptr_eq(previous, lease))
        {
            return Err(Error::JointControllerIdentity);
        }
    }
    Ok(())
}
pub(super) fn prepare(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
    expected: &Metadata,
) -> Result<Option<Controllers>, Error> {
    prepare_for(sim, io, config, expected, false)
}
/// Reconstructed metadata drift has a reset error, without relaxing live admission.
pub(super) fn prepare_restored(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
    expected: &Metadata,
) -> Result<Option<Controllers>, Error> {
    prepare_for(sim, io, config, expected, true)
}
fn prepare_for(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
    expected: &Metadata,
    restoring: bool,
) -> Result<Option<Controllers>, Error> {
    let admission = own_set_for(sim, config, None, restoring)?;
    let occupied = joint_controller_reserved_keys(sim, io, config)?;
    if !admission.keys.is_disjoint(&occupied) {
        return Err(Error::JointControllerIdentity);
    }
    let actual: Metadata = admission
        .owners
        .iter()
        .map(|owner| (owner.identity, owner.metadata.clone()))
        .collect();
    // Original/private/native identity is already proven. The full witness can
    // now differ only in the reconstructed public configuration.
    let full_public = !restoring
        || admission
            .owners
            .iter()
            .all(|owner| gz_sim_scene::joint_controller_binding_matches(sim, owner.identity));
    if actual != admission.metadata || &admission.metadata != expected || !full_public {
        return Err(Error::ResetJointControllers);
    }
    let owners = admission.owners;
    if owners.is_empty() {
        return Ok(None);
    }
    let mut endpoints = Vec::new();
    let mut receivers = Vec::new();
    for owner in &owners {
        let (input, receiver) = mpsc::sync_channel(config.capacity);
        endpoints.push(Endpoint {
            key: owner.endpoint.clone(),
            input,
        });
        receivers.push(receiver);
    }
    let leases = io.joint_controller_inputs(Setup {
        endpoints,
        max_bytes: config.max_message_bytes,
        drops: Arc::clone(&io.ingress_drops),
        epoch: Arc::clone(&io.epoch),
    })?;
    prepared(&owners, &leases)?;
    let streams = owners
        .into_iter()
        .zip(receivers)
        .zip(leases)
        .map(|((owner, receiver), lease)| Stream {
            entity: owner.entity,
            identity: owner.identity,
            metadata: owner.metadata,
            joint: owner.joint,
            _lease: lease,
            receiver,
            name: owner.name,
            parent: owner.parent,
            model: owner.model,
            endpoint: owner.endpoint,
        })
        .collect();
    Ok(Some(Controllers { streams }))
}
impl Controllers {
    fn reconcile(&mut self, sim: &Simulation) -> Result<(), Error> {
        let saved = originals(sim)?.unwrap_or_default();
        retained(sim, self, &saved)?;
        let live: BTreeSet<_> = self
            .streams
            .iter()
            .filter(|stream| !stream.removed(sim))
            .map(|stream| stream.identity)
            .collect();
        let mut current = BTreeSet::new();
        for node in sim.world().iter_entities().filter(|node| {
            !node.contains::<PendingResetRemoval>()
                && node.contains::<JointControllerConfiguration>()
        }) {
            let id = node
                .get::<EntityId>()
                .ok_or(Error::JointControllerIdentity)?
                .get();
            if current.len() == MAX_INPUTS
                || !current.insert(id)
                || !self
                    .streams
                    .iter()
                    .any(|stream| stream.entity == node.id() && stream.identity == id)
            {
                return Err(Error::JointControllerIdentity);
            }
        }
        if current != live {
            return Err(Error::JointControllerIdentity);
        }
        self.streams.retain(|stream| !stream.removed(sim));
        if self.streams.iter().any(|stream| !stream.live(sim)) {
            return Err(Error::JointControllerIdentity);
        }
        Ok(())
    }
    fn poll(
        &mut self,
        sim: &mut Simulation,
        budget: NonZeroUsize,
        counters: &mut Counters,
    ) -> Result<(), Error> {
        for stream in &self.streams {
            for _ in 0..budget.get() {
                let (epoch, value) = match stream.receiver.try_recv() {
                    Ok(command) => command,
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => return Err(Error::WorldClosed),
                };
                counters.joint_controller_messages_received += 1;
                let Some(value) = value.filter(|_| epoch == sim.epoch()) else {
                    counters.joint_controller_messages_rejected += 1;
                    continue;
                };
                match gz_sim_scene::set_joint_controller_command(sim, stream.identity, value) {
                    Ok(()) => counters.joint_controller_commands_applied += 1,
                    Err(JointControllerCommandError::InvalidCommand) => {
                        counters.joint_controller_messages_rejected += 1
                    }
                    Err(_) => return Err(Error::JointControllerIdentity),
                }
            }
        }
        Ok(())
    }
}
impl Server {
    pub(super) fn reconcile_joint_controllers(&mut self) -> Result<(), Error> {
        if let Some(controllers) = &mut self.joint_controllers {
            if let Err(error) = controllers.reconcile(&self.simulation) {
                self.joint_controllers = None;
                self.reset_failed = true;
                return Err(error);
            }
            if controllers.streams.is_empty() {
                self.joint_controllers = None;
            }
        }
        Ok(())
    }
    pub(super) fn poll_joint_controllers(&mut self) -> Result<(), Error> {
        self.reconcile_joint_controllers()?;
        if let Some(controllers) = &mut self.joint_controllers
            && let Err(error) = controllers.poll(
                &mut self.simulation,
                self.command_budget,
                &mut self.counters,
            )
        {
            self.joint_controllers = None;
            self.reset_failed = true;
            return Err(error);
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../tests/private/joint_controllers.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/multiple_joint_controllers.rs"]
mod multiple_joint_controllers;

#[cfg(test)]
#[path = "../tests/private/multiple_joint_controllers_native.rs"]
mod multiple_joint_controllers_native;
