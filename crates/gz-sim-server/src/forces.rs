// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{
    Counters, Error,
    force_io::{ForceEndpoint, ForceSetup},
};
use crate::{ServerOptions, SetupError};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    EntityKind, JointForceCommand, JointForceConfiguration, JointHandle, set_joint_force,
};
use std::{
    collections::BTreeMap,
    num::NonZeroUsize,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
};

struct Stream {
    ids: Vec<u64>,
    receiver: mpsc::Receiver<Option<f64>>,
    active: Arc<AtomicBool>,
}
impl Drop for Stream {
    fn drop(&mut self) {
        self.active.store(false, Ordering::Release);
    }
}

pub(crate) struct Forces {
    streams: Vec<Stream>,
    expected: BTreeMap<u64, JointForceConfiguration>,
    next: usize,
}

impl Forces {
    pub(crate) fn prepare(
        config: &ServerOptions,
        configurations: impl IntoIterator<Item = (u64, JointForceConfiguration)>,
        drops: &Arc<AtomicU64>,
    ) -> Result<(Self, ForceSetup), Error> {
        if config.capacity == 0
            || config.capacity > i32::MAX as usize
            || config.max_message_bytes == 0
            || config.max_message_bytes > i64::MAX as usize
        {
            return Err(SetupError::InvalidConfig.into());
        }
        let mut groups: BTreeMap<String, Vec<u64>> = BTreeMap::new();
        let mut expected = BTreeMap::new();
        for (id, configuration) in configurations {
            let topic = crate::options::as_valid_topic(&format!(
                "/model/{}/joint/{}/cmd_force",
                configuration.model_name(),
                configuration.joint_name()
            ));
            if topic.is_empty() {
                return Err(SetupError::InvalidName.into());
            }
            let remapped = config.topic_remap(&topic).unwrap_or(&topic);
            let key =
                crate::options::qualified_name(&config.partition, &config.namespace, remapped)?;
            groups.entry(key).or_default().push(id);
            if expected.insert(id, configuration).is_some() {
                return Err(Error::JointForceIdentity);
            }
        }
        let mut result = Self {
            streams: Vec::new(),
            expected,
            next: 0,
        };
        let mut setup = ForceSetup {
            endpoints: Vec::new(),
            max_bytes: config.max_message_bytes,
            ingress_drops: Arc::clone(drops),
        };
        for (key, ids) in groups {
            let (input, receiver) = mpsc::sync_channel(config.capacity);
            let active = Arc::new(AtomicBool::new(true));
            result.streams.push(Stream {
                ids,
                receiver,
                active: Arc::clone(&active),
            });
            setup.endpoints.push(ForceEndpoint { key, input, active });
        }
        Ok((result, setup))
    }

    pub(crate) fn bind(&mut self, sim: &Simulation) -> Result<(), Error> {
        for entity in sim
            .world()
            .iter_entities()
            .filter(|e| e.contains::<JointForceConfiguration>())
        {
            let id = entity
                .get::<EntityId>()
                .ok_or(Error::JointForceIdentity)?
                .get();
            if entity_by_id(sim.world(), id) != Some(entity.id())
                || entity.get::<EntityKind>() != Some(&EntityKind::Joint)
                || !entity.contains::<JointHandle>()
                || !entity.contains::<JointForceCommand>()
                || self.expected.get(&id) != entity.get::<JointForceConfiguration>()
            {
                return Err(Error::JointForceIdentity);
            }
        }
        self.retire(sim);
        Ok(())
    }

    pub(crate) fn retire(&mut self, sim: &Simulation) {
        for stream in &mut self.streams {
            stream.ids.retain(|id| {
                entity_by_id(sim.world(), *id).is_some_and(|entity| {
                    sim.world().get::<JointForceConfiguration>(entity) == self.expected.get(id)
                        && sim.world().get::<JointForceCommand>(entity).is_some()
                })
            });
        }
        self.streams.retain(|stream| !stream.ids.is_empty());
        self.expected.retain(|id, configuration| {
            entity_by_id(sim.world(), *id).is_some_and(|entity| {
                sim.world().get::<JointForceConfiguration>(entity) == Some(configuration)
                    && sim.world().get::<JointForceCommand>(entity).is_some()
            })
        });
        if self.streams.is_empty() {
            self.streams = Vec::new();
            self.expected = BTreeMap::new();
            self.next = 0;
        }
    }

    pub(crate) fn restart(
        config: &ServerOptions,
        sim: &Simulation,
        drops: &Arc<AtomicU64>,
    ) -> Result<(Self, ForceSetup), Error> {
        // Loaded Scene metadata, not retained endpoint templates, owns reset demand.
        let configurations: Vec<_> = match gz_sim_scene::initial_joint_force_configurations(sim) {
            Ok(values) => values
                .filter(|(id, _)| {
                    entity_by_id(sim.world(), *id).is_some_and(|entity| {
                        sim.world().get::<JointForceConfiguration>(entity).is_some()
                    })
                })
                .map(|(id, configuration)| (id, configuration.clone()))
                .collect(),
            Err(gz_sim_scene::SceneResetError::MissingInitialState) => Vec::new(),
            Err(error) => return Err(error.into()),
        };
        let (mut replacement, setup) = Self::prepare(config, configurations, drops)?;
        replacement.bind(sim)?;
        Ok((replacement, setup))
    }

    pub(crate) fn clear(&mut self) {
        self.streams = Vec::new();
        self.expected = BTreeMap::new();
        self.next = 0;
    }
    pub(crate) fn shutdown(&mut self) {
        self.clear();
    }
    pub(crate) fn is_empty(&self) -> bool {
        self.streams.is_empty()
    }

    pub(crate) fn into_active(self) -> Option<Self> {
        if self.is_empty() { None } else { Some(self) }
    }

    pub(crate) fn poll(
        &mut self,
        sim: &mut Simulation,
        budget: NonZeroUsize,
        counters: &mut Counters,
    ) -> Result<(), Error> {
        for _ in 0..budget.get() {
            let mut received = None;
            for _ in 0..self.streams.len() {
                self.next %= self.streams.len();
                let index = self.next;
                self.next += 1;
                match self.streams[index].receiver.try_recv() {
                    Ok(message) => {
                        received = Some((index, message));
                        break;
                    }
                    Err(mpsc::TryRecvError::Empty) => {}
                    Err(mpsc::TryRecvError::Disconnected) => {
                        return Err(Error::WorldClosed);
                    }
                }
            }
            let Some((index, message)) = received else {
                break;
            };
            counters.force_messages_received += 1;
            let Some(value) = message else {
                counters.force_messages_rejected += 1;
                continue;
            };
            let mut rejected = false;
            for id in &self.streams[index].ids {
                if set_joint_force(sim, *id, value).is_ok() {
                    counters.joint_forces_applied += 1;
                } else {
                    rejected = true;
                }
            }
            counters.force_messages_rejected += u64::from(rejected);
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../tests/private/forces.rs"]
mod tests;
