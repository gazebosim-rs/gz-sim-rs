// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Original loaded identities and bounded, fair model-frame command ingress.
const MAX_OWNERS: usize = 8;
use super::*;
use bevy_ecs::prelude::ChildOf;
use gz_sim_scene::{EntityKind, Name, PendingResetRemoval, VelocityControlConfiguration};
use std::sync::mpsc;

#[derive(Clone)]
struct Original {
    id: u64,
    parent: u64,
    name: String,
    metadata: VelocityControlConfiguration,
}
fn valid(metadata: &VelocityControlConfiguration) -> bool {
    metadata.link != 0
        && !metadata.model_name.is_empty()
        && metadata.topic.starts_with('/')
        && options::as_valid_topic(&metadata.topic) == metadata.topic
        && metadata
            .initial_linear
            .into_iter()
            .chain(metadata.initial_angular)
            .all(f64::is_finite)
}
fn originals(sim: &Simulation) -> Result<Vec<Original>, Error> {
    let rows = match gz_sim_scene::initial_velocity_control_configurations_with_identity(sim) {
        Ok(rows) => rows.take(MAX_OWNERS + 1).collect::<Vec<_>>(),
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => {
            if sim
                .world()
                .iter_entities()
                .any(|e| e.contains::<VelocityControlConfiguration>())
            {
                return Err(Error::VelocityControlIdentity);
            }
            return Ok(Vec::new());
        }
        Err(error) => return Err(error.into()),
    };
    if rows.is_empty() {
        return Ok(Vec::new());
    }
    if rows.len() > MAX_OWNERS {
        return Err(Error::VelocityControlIdentity);
    }
    let mut worlds = sim
        .world()
        .iter_entities()
        .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::World));
    let world = worlds.next().ok_or(Error::VelocityControlIdentity)?;
    let parent = world
        .get::<EntityId>()
        .ok_or(Error::VelocityControlIdentity)?
        .get();
    let world_name = &world.get::<Name>().ok_or(Error::VelocityControlIdentity)?.0;
    if worlds.next().is_some() || parent == 0 {
        return Err(Error::VelocityControlIdentity);
    }
    let mut ids = BTreeSet::new();
    let mut topics = BTreeSet::new();
    let mut result = Vec::with_capacity(rows.len());
    for row in rows {
        if row.id == 0
            || row.id == parent
            || !ids.insert(row.id)
            || row.parent != Some(parent)
            || row.kind != EntityKind::Model
            || row.name != row.configuration.model_name
            || !row.name.starts_with(&format!("{world_name}::"))
            || !valid(row.configuration)
            || !topics.insert(row.configuration.topic.clone())
        {
            return Err(Error::VelocityControlIdentity);
        }
        result.push(Original {
            id: row.id,
            parent,
            name: row.name.to_owned(),
            metadata: row.configuration.clone(),
        });
    }
    result.sort_by_key(|row| row.id);
    Ok(result)
}
#[cfg(test)]
fn original(sim: &Simulation) -> Result<Option<Original>, Error> {
    Ok(originals(sim)?.into_iter().next())
}
fn current(sim: &Simulation, saved: &[Original]) -> Result<BTreeMap<u64, Entity>, Error> {
    let mut found = BTreeMap::new();
    for node in sim.world().iter_entities().filter(|e| {
        e.contains::<VelocityControlConfiguration>() && !e.contains::<PendingResetRemoval>()
    }) {
        let id = node
            .get::<EntityId>()
            .ok_or(Error::VelocityControlIdentity)?
            .get();
        let saved = saved
            .iter()
            .find(|s| s.id == id)
            .ok_or(Error::VelocityControlIdentity)?;
        let parent = node
            .get::<ChildOf>()
            .and_then(|p| sim.world().get::<EntityId>(p.parent()).map(EntityId::get));
        if found.insert(id, node.id()).is_some()
            || node.get::<EntityKind>() != Some(&EntityKind::Model)
            || entity_by_id(sim.world(), id) != Some(node.id())
            || node.get::<Name>().map(|n| &n.0) != Some(&saved.name)
            || parent != Some(saved.parent)
            || node.get::<VelocityControlConfiguration>() != Some(&saved.metadata)
            || !gz_sim_scene::velocity_control_binding_matches_configuration(
                sim,
                id,
                &saved.metadata,
            )
            || !gz_sim_scene::velocity_control_binding_matches(sim, id)
        {
            return Err(Error::VelocityControlIdentity);
        }
    }
    for saved in saved {
        for node in sim
            .world()
            .iter_entities()
            .filter(|e| !e.contains::<PendingResetRemoval>())
        {
            if (node.get::<EntityId>().map(EntityId::get) == Some(saved.id)
                || node.get::<Name>().map(|n| &n.0) == Some(&saved.name))
                && found.get(&saved.id) != Some(&node.id())
            {
                return Err(Error::VelocityControlIdentity);
            }
        }
    }
    Ok(found)
}
pub(super) fn reserved_keys(
    sim: &Simulation,
    config: &ServerOptions,
) -> Result<BTreeSet<String>, Error> {
    let saved = originals(sim)?;
    current(sim, &saved)?;
    let mut keys = BTreeSet::new();
    for owner in saved {
        if !keys.insert(config.topic_key(&owner.metadata.topic)?) {
            return Err(Error::VelocityControlIdentity);
        }
    }
    Ok(keys)
}
// External camera writers are owned outside Server; reserve their actual
// native image/CameraInfo topic rules without constructing a writer or renderer.
// Source basis: crates/gz-camera-output/src/application.rs prepare_selected_camera typed
// ancestry, image default and camera_info sibling derivation.
fn camera_keys(sim: &Simulation, config: &ServerOptions) -> Result<BTreeSet<String>, Error> {
    let mut keys = BTreeSet::new();
    for sensor in sim.world().iter_entities().filter(|e| {
        e.contains::<gz_sim_scene::CameraSensor>() && !e.contains::<PendingResetRemoval>()
    }) {
        let metadata = sensor
            .get::<gz_sim_scene::CameraSensor>()
            .expect("selected camera");
        let image = if metadata.0.topic.is_empty() {
            let mut parts = Vec::new();
            let mut node = sensor.id();
            loop {
                let entity = sim.world().entity(node);
                let kind = match entity.get::<EntityKind>() {
                    Some(EntityKind::World) => "world",
                    Some(EntityKind::Model) => "model",
                    Some(EntityKind::Link) => "link",
                    Some(EntityKind::Sensor) => "sensor",
                    _ => return Err(Error::VelocityControlIdentity),
                };
                let local = entity
                    .get::<gz_sim_scene::LocalName>()
                    .ok_or(Error::VelocityControlIdentity)?;
                parts.push(format!("/{kind}/{}", local.0));
                if kind == "world" {
                    break;
                }
                if parts.len() > sim.world().entities().len() as usize {
                    return Err(Error::VelocityControlIdentity);
                }
                node = entity
                    .get::<ChildOf>()
                    .ok_or(Error::VelocityControlIdentity)?
                    .parent();
            }
            options::as_valid_topic(&(parts.into_iter().rev().collect::<String>() + "/image"))
        } else {
            options::as_valid_topic(&metadata.0.topic)
        };
        let camera = metadata.0.camera().ok_or(Error::VelocityControlIdentity)?;
        let mut info = options::as_valid_topic(&camera.camera_info_topic);
        if info.is_empty() {
            info = image
                .rsplit_once('/')
                .map_or(String::new(), |(parent, _)| parent.to_owned());
            info.push_str("/camera_info");
        }
        keys.insert(config.topic_key(&image)?);
        keys.insert(config.topic_key(&info)?);
    }
    Ok(keys)
}
fn check_keys(
    sim: &Simulation,
    config: &ServerOptions,
    mut occupied: BTreeSet<String>,
) -> Result<(), Error> {
    if !originals(sim)?.is_empty() {
        occupied.extend(camera_keys(sim, config)?);
    }
    if !reserved_keys(sim, config)?.is_disjoint(&occupied) {
        return Err(Error::VelocityControlIdentity);
    }
    Ok(())
}
pub(super) fn before_network(
    sim: &Simulation,
    world: &str,
    config: &ServerOptions,
) -> Result<(), Error> {
    if originals(sim)?.is_empty() {
        current(sim, &[])?;
        return Ok(());
    }
    let mut base = BTreeSet::new();
    for topic in [
        format!("/world/{world}/clock"),
        format!("/world/{world}/control"),
        format!("/world/{world}/pose/info"),
        format!("/world/{world}/remove"),
        format!("/world/{world}/remove/blocking"),
    ] {
        base.insert(config.topic_key(&topic)?);
    }
    check_keys(
        sim,
        config,
        typed_reserved_keys_from_base(sim, config, PublicationFamily::VelocityControl, base)?,
    )
}
pub(super) fn preflight(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<(), Error> {
    check_keys(
        sim,
        config,
        typed_reserved_keys(sim, io, config, PublicationFamily::VelocityControl)?,
    )
}
pub(super) struct Controller {
    entity: Entity,
    original: Original,
    key: String,
    lease_id: uuid::Uuid,
    lease: Arc<crate::velocity_control_io::Lease>,
    receiver: mpsc::Receiver<crate::velocity_control_io::Command>,
    others: Vec<Controller>,
    next_poll: usize,
}
impl Drop for Controller {
    fn drop(&mut self) {
        self.lease.retire();
    }
}
pub(super) fn prepare(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<Option<Controller>, Error> {
    let saved = originals(sim)?;
    let live = current(sim, &saved)?;
    preflight(sim, io, config)?;
    let mut controllers = Vec::with_capacity(live.len());
    for saved in saved {
        let Some(&entity) = live.get(&saved.id) else {
            continue;
        };
        let key = config.topic_key(&saved.metadata.topic)?;
        let (input, receiver) = mpsc::sync_channel(config.capacity);
        let lease = io.velocity_control_input(crate::velocity_control_io::Setup {
            key: key.clone(),
            input,
            max_bytes: config.max_message_bytes,
            drops: Arc::clone(&io.ingress_drops),
            epoch: Arc::clone(&io.epoch),
        })?;
        let lease_id = lease.identity();
        if !lease.matches(lease_id, &key) {
            return Err(Error::VelocityControlIdentity);
        }
        controllers.push(Controller {
            entity,
            original: saved,
            key,
            lease_id,
            lease,
            receiver,
            others: Vec::new(),
            next_poll: 0,
        });
    }
    if controllers.is_empty() {
        return Ok(None);
    }
    let mut first = controllers.remove(0);
    first.others = controllers;
    Ok(Some(first))
}

impl Server {
    pub(super) fn retire_velocity_control(&mut self) {
        let removed = |c: &Controller| {
            self.simulation.world().get_entity(c.entity).is_err()
                || self
                    .simulation
                    .world()
                    .entity(c.entity)
                    .contains::<PendingResetRemoval>()
        };
        let Some(mut first) = self.velocity_control.take() else {
            return;
        };
        first.others.retain(|c| !removed(c));
        if removed(&first) {
            if first.others.is_empty() {
                return;
            }
            let mut replacement = first.others.remove(0);
            replacement.others = std::mem::take(&mut first.others);
            replacement.next_poll = first.next_poll;
            self.velocity_control = Some(replacement);
        } else {
            self.velocity_control = Some(first);
        }
    }
    pub(super) fn poll_velocity_control(&mut self) -> Result<(), Error> {
        self.retire_velocity_control();
        let validation = (|| -> Result<(), Error> {
            let saved = originals(&self.simulation)?;
            let live = current(&self.simulation, &saved)?;
            let Some(first) = self.velocity_control.as_ref() else {
                return if live.is_empty() {
                    Ok(())
                } else {
                    Err(Error::VelocityControlIdentity)
                };
            };
            if live.len() != first.others.len() + 1 {
                return Err(Error::VelocityControlIdentity);
            }
            for c in std::iter::once(first).chain(first.others.iter()) {
                if live.get(&c.original.id) != Some(&c.entity)
                    || !saved
                        .iter()
                        .any(|s| s.id == c.original.id && s.metadata == c.original.metadata)
                    || !c.lease.matches(c.lease_id, &c.key)
                {
                    return Err(Error::VelocityControlIdentity);
                }
            }
            Ok(())
        })();
        if let Err(error) = validation {
            self.velocity_control = None;
            self.reset_failed = true;
            return Err(error);
        }
        let Some(first) = self.velocity_control.as_mut() else {
            return Ok(());
        };
        let count = first.others.len() + 1;
        let mut idle = 0;
        for _ in 0..self.command_budget.get() {
            let (id, message) = loop {
                if idle == count {
                    return Ok(());
                }
                let index = first.next_poll % count;
                first.next_poll = (index + 1) % count;
                let c = if index == 0 {
                    &*first
                } else {
                    &first.others[index - 1]
                };
                match c.receiver.try_recv() {
                    Ok(message) => {
                        idle = 0;
                        break (c.original.id, message);
                    }
                    Err(mpsc::TryRecvError::Empty) => idle += 1,
                    Err(mpsc::TryRecvError::Disconnected) => return Err(Error::WorldClosed),
                }
            };
            self.counters.velocity_control_messages_received += 1;
            let accepted = message.0 == self.simulation.epoch()
                && message.1.is_some_and(|(linear, angular)| {
                    gz_sim_scene::set_velocity_control_command(
                        &mut self.simulation,
                        id,
                        linear,
                        angular,
                    )
                    .is_ok()
                });
            if accepted {
                self.counters.velocity_control_commands_pending += 1;
            } else {
                self.counters.velocity_control_messages_rejected += 1;
            }
        }
        Ok(())
    }
}
#[cfg(test)]
#[path = "../tests/private/velocity_control.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/m105_velocity_control_fleet.rs"]
mod fleet_tests;
