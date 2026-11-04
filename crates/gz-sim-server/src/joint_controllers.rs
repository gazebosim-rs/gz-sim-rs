// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! At most two canonical joint command streams; all actuation belongs to Scene.
use super::*;
use crate::joint_controller_io::{Endpoint, Lease, Setup};
use gz_sim_scene::{
    EntityKind, JointControllerCommandError, JointControllerConfiguration, JointHandle,
    PendingResetRemoval,
};
use std::{collections::BTreeSet, sync::mpsc};

type Metadata = BTreeMap<u64, JointControllerConfiguration>;

fn valid(metadata: &JointControllerConfiguration) -> bool {
    !metadata.model_name.is_empty()
        && !metadata.joint_name.is_empty()
        && metadata.topic.starts_with('/')
        && options::as_valid_topic(&metadata.topic) == metadata.topic
}
pub(super) fn initial(sim: &Simulation) -> Result<Metadata, Error> {
    let mut result = Metadata::new();
    match gz_sim_scene::initial_joint_controller_configurations(sim) {
        Ok(values) => {
            for (id, configuration) in values {
                if !valid(configuration) || result.insert(id, configuration.clone()).is_some() {
                    return Err(Error::JointControllerIdentity);
                }
            }
        }
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
        Err(error) => return Err(error.into()),
    }
    if result.len() > 2 {
        return Err(Error::JointControllerIdentity);
    }
    Ok(result)
}
struct Stream {
    entity: Entity,
    identity: u64,
    metadata: JointControllerConfiguration,
    joint: JointHandle,
    _lease: Arc<Lease>,
    receiver: mpsc::Receiver<crate::joint_controller_io::Command>,
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
        world.get::<EntityKind>(self.entity) == Some(&EntityKind::Joint)
            && world.get::<JointControllerConfiguration>(self.entity) == Some(&self.metadata)
            && world.get::<EntityId>(self.entity).map(EntityId::get) == Some(self.identity)
            && entity_by_id(world, self.identity) == Some(self.entity)
            && world.get::<JointHandle>(self.entity).is_some_and(|joint| {
                joint.id() == self.joint.id() && joint.child() == self.joint.child()
            })
    }
}
pub(super) struct Controllers {
    streams: Vec<Stream>,
}

fn occupied(sim: &Simulation, config: &ServerOptions) -> Result<BTreeSet<String>, Error> {
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
    Ok(keys)
}
pub(super) fn prepare(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
    expected: &Metadata,
) -> Result<Option<Controllers>, Error> {
    let mut actual = Metadata::new();
    let mut owners = Vec::new();
    let mut keys = occupied(sim, config)?;
    for entity in sim.world().iter_entities() {
        if entity.contains::<PendingResetRemoval>() {
            continue;
        }
        let Some(metadata) = entity.get::<JointControllerConfiguration>() else {
            continue;
        };
        let id = entity
            .get::<EntityId>()
            .ok_or(Error::JointControllerIdentity)?
            .get();
        let joint = entity
            .get::<JointHandle>()
            .ok_or(Error::JointControllerIdentity)?;
        if !valid(metadata)
            || entity.get::<EntityKind>() != Some(&EntityKind::Joint)
            || entity_by_id(sim.world(), id) != Some(entity.id())
            || actual.insert(id, metadata.clone()).is_some()
            || owners.len() == 2
            || !keys.insert(config.topic_key(&metadata.topic)?)
        {
            return Err(Error::JointControllerIdentity);
        }
        owners.push((entity.id(), id, metadata.clone(), *joint));
    }
    if &actual != expected {
        return Err(Error::ResetJointControllers);
    }
    if owners.is_empty() {
        return Ok(None);
    }
    let mut endpoints = Vec::new();
    let mut receivers = Vec::new();
    for (_, _, metadata, _) in &owners {
        let (input, receiver) = mpsc::sync_channel(config.capacity);
        endpoints.push(Endpoint {
            key: config.topic_key(&metadata.topic)?,
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
    if leases.len() != owners.len() {
        return Err(Error::JointControllerIdentity);
    }
    let streams = owners
        .into_iter()
        .zip(receivers)
        .zip(leases)
        .map(
            |(((entity, identity, metadata, joint), receiver), lease)| Stream {
                entity,
                identity,
                metadata,
                joint,
                receiver,
                _lease: lease,
            },
        )
        .collect();
    Ok(Some(Controllers { streams }))
}
impl Controllers {
    fn reconcile(&mut self, sim: &Simulation) -> Result<(), Error> {
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
