// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical input admission; Scene owns force state and Physics owns integration.
use crate::{
    Counters, Error, ServerOptions,
    apply_link_wrench_io::{Channel, Command, Envelope, WrenchEndpoint, WrenchSetup},
};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_proto::msgs::{Entity, EntityWrench, entity::Type};
use gz_sim_scene::{
    ApplyLinkWrenchConfiguration, ApplyLinkWrenchMode, EntityKind, Name, apply_link_wrench_active,
    clear_apply_link_wrench, queue_apply_link_wrench,
};
use std::{
    collections::BTreeSet,
    num::NonZeroUsize,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
};

#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct Prepared {
    pub(super) world_id: u64,
    pub(super) configuration: ApplyLinkWrenchConfiguration,
    pub(super) keys: [String; 3],
}
pub(super) struct ApplyLinkWrenches {
    prepared: Prepared,
    inputs: [mpsc::Receiver<Envelope>; 3],
    active: Arc<AtomicBool>,
    epoch: u64,
    next: usize,
}
fn current(sim: &Simulation) -> Result<Option<(u64, ApplyLinkWrenchConfiguration)>, Error> {
    let mut selected = None;
    for entity in sim.world().iter_entities() {
        let Some(configuration) = entity.get::<ApplyLinkWrenchConfiguration>() else {
            continue;
        };
        let id = entity
            .get::<EntityId>()
            .ok_or(Error::ApplyLinkWrenchIdentity)?
            .get();
        if selected.is_some()
            || entity.get::<EntityKind>() != Some(&EntityKind::World)
            || entity_by_id(sim.world(), id) != Some(entity.id())
        {
            return Err(Error::ApplyLinkWrenchIdentity);
        }
        selected = Some((id, configuration.clone()));
    }
    Ok(selected)
}
fn prepared(
    value: (u64, ApplyLinkWrenchConfiguration),
    options: &ServerOptions,
    occupied: &BTreeSet<String>,
) -> Result<Prepared, Error> {
    options.validate()?;
    let (world_id, configuration) = value;
    let raw = [
        &configuration.topic,
        &configuration.persistent_topic,
        &configuration.clear_topic,
    ];
    if raw
        .iter()
        .any(|topic| crate::as_valid_topic(topic) != **topic)
    {
        return Err(Error::ApplyLinkWrenchIdentity);
    }
    let keys = [
        options.topic_key(raw[0])?,
        options.topic_key(raw[1])?,
        options.topic_key(raw[2])?,
    ];
    if keys.iter().collect::<BTreeSet<_>>().len() != 3
        || keys.iter().any(|key| occupied.contains(key))
    {
        return Err(Error::ApplyLinkWrenchIdentity);
    }
    Ok(Prepared {
        world_id,
        configuration,
        keys,
    })
}
pub(super) fn prepare_reset(
    sim: &Simulation,
    options: &ServerOptions,
    occupied: &BTreeSet<String>,
) -> Result<Option<Prepared>, Error> {
    gz_sim_scene::initial_apply_link_wrench_configuration(sim)?
        .map(|(id, configuration)| prepared((id, configuration.clone()), options, occupied))
        .transpose()
}
fn resolve(sim: &Simulation, entity: &Entity) -> Option<u64> {
    if entity.r#type != Type::Link as i32 {
        return None;
    }
    if entity.id != 0 {
        return Some(entity.id);
    }
    if entity.name.is_empty() {
        return None;
    }
    let mut matches = sim.world().iter_entities().filter(|e| {
        e.get::<EntityKind>() == Some(&EntityKind::Link)
            && e.get::<Name>().is_some_and(|name| name.0 == entity.name)
    });
    let id = matches.next()?.get::<EntityId>()?.get();
    matches.next().is_none().then_some(id)
}
fn values(message: &EntityWrench) -> Option<(Entity, [f64; 3], [f64; 3])> {
    let entity = message.entity.as_ref()?.clone();
    let wrench = message.wrench.as_ref()?;
    let vector = |v: Option<&gz_sim_proto::msgs::Vector3d>| v.map_or([0.0; 3], |v| [v.x, v.y, v.z]);
    let force = vector(wrench.force.as_ref());
    let torque = vector(wrench.torque.as_ref());
    let offset = vector(wrench.force_offset.as_ref());
    (force.into_iter().chain(torque).all(f64::is_finite)
        && offset.into_iter().all(|v| v.is_finite() && v == 0.0))
    .then_some((entity, force, torque))
}
impl ApplyLinkWrenches {
    pub(super) fn prepare(
        sim: &Simulation,
        options: &ServerOptions,
        occupied: &BTreeSet<String>,
        drops: &Arc<AtomicU64>,
    ) -> Result<Option<(Self, WrenchSetup)>, Error> {
        let Some(value) = current(sim)? else {
            return Ok(None);
        };
        let prepared = prepared(value, options, occupied)?;
        if !apply_link_wrench_active(sim) {
            return Ok(None);
        }
        let active = Arc::new(AtomicBool::new(true));
        let mut receivers = Vec::new();
        let mut endpoints = Vec::new();
        for channel in [Channel::Once, Channel::Persistent, Channel::Clear] {
            let (input, receiver) = mpsc::sync_channel(options.capacity);
            receivers.push(receiver);
            endpoints.push(WrenchEndpoint {
                key: prepared.keys[channel.index()].clone(),
                channel,
                input,
                active: Arc::clone(&active),
                epoch: sim.epoch(),
            });
        }
        let inputs = receivers
            .try_into()
            .map_err(|_| Error::ApplyLinkWrenchIdentity)?;
        Ok(Some((
            Self {
                prepared,
                inputs,
                active,
                epoch: sim.epoch(),
                next: 0,
            },
            WrenchSetup {
                endpoints,
                max_bytes: options.max_message_bytes,
                ingress_drops: Arc::clone(drops),
            },
        )))
    }
    pub(super) fn bind(&mut self, sim: &Simulation, expected: &Prepared) -> Result<(), Error> {
        if &self.prepared != expected
            || current(sim)? != Some((expected.world_id, expected.configuration.clone()))
            || self.epoch != sim.epoch()
        {
            return Err(Error::ApplyLinkWrenchIdentity);
        }
        Ok(())
    }
    pub(super) fn poll(
        &mut self,
        sim: &mut Simulation,
        budget: NonZeroUsize,
        counters: &mut Counters,
    ) -> Result<(), Error> {
        if self.is_empty() {
            return Ok(());
        }
        if self.epoch != sim.epoch()
            || current(sim)? != Some((self.prepared.world_id, self.prepared.configuration.clone()))
        {
            return Err(Error::ApplyLinkWrenchIdentity);
        }
        for _ in 0..budget.get() {
            let mut received = None;
            for _ in 0..3 {
                let index = self.next;
                self.next = (self.next + 1) % 3;
                match self.inputs[index].try_recv() {
                    Ok(message) => {
                        received = Some((index, message));
                        break;
                    }
                    Err(mpsc::TryRecvError::Empty) => {}
                    Err(mpsc::TryRecvError::Disconnected) => return Err(Error::WorldClosed),
                }
            }
            let Some((index, message)) = received else {
                break;
            };
            counters.wrench_messages_received += 1;
            let accepted = if message.epoch != self.epoch {
                false
            } else {
                match message.command {
                    Some(Command::Clear(entity)) if index == Channel::Clear.index() => {
                        resolve(sim, &entity).is_some_and(|id| {
                            let accepted = clear_apply_link_wrench(sim, id).is_ok();
                            if accepted {
                                counters.link_wrench_clears_queued += 1;
                            }
                            accepted
                        })
                    }
                    Some(Command::Wrench(message)) if index != Channel::Clear.index() => {
                        values(&message).is_some_and(|(entity, force, torque)| {
                            resolve(sim, &entity).is_some_and(|id| {
                                let mode = if index == Channel::Once.index() {
                                    ApplyLinkWrenchMode::Once
                                } else {
                                    ApplyLinkWrenchMode::Persistent
                                };
                                let accepted =
                                    queue_apply_link_wrench(sim, id, force, torque, mode).is_ok();
                                if accepted {
                                    counters.link_wrenches_queued += 1;
                                }
                                accepted
                            })
                        })
                    }
                    _ => false,
                }
            };
            counters.wrench_messages_rejected += u64::from(!accepted);
        }
        Ok(())
    }
    pub(super) fn retire(&mut self, sim: &Simulation) {
        if !apply_link_wrench_active(sim) {
            self.shutdown();
        }
    }
    pub(super) fn shutdown(&mut self) {
        self.active.store(false, Ordering::Release);
    }
    pub(super) fn is_empty(&self) -> bool {
        !self.active.load(Ordering::Acquire)
    }
}
impl Drop for ApplyLinkWrenches {
    fn drop(&mut self) {
        self.shutdown();
    }
}
