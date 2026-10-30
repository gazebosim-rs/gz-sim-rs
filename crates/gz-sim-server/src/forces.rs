// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{Counters, Error, add_transport_statistics};
use gz_msgs::{msgs::Double, prost::Message};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    EntityKind, JointForceCommand, JointForceConfiguration, JointHandle, set_joint_force,
};
use gz_transport::{Config, Error as TransportError, Node, Statistics, Subscription};
use std::{collections::BTreeMap, num::NonZeroUsize, time::Duration};

struct Template {
    worker: usize,
    topic: String,
    ids: Vec<u64>,
}
struct Stream {
    ids: Vec<u64>,
    subscription: Subscription,
}

pub(crate) struct Forces {
    // Retire subscriptions before their prepared worker Nodes.
    streams: Vec<Stream>,
    workers: Vec<Node>,
    templates: Vec<Template>,
    expected: BTreeMap<u64, JointForceConfiguration>,
    next: usize,
}
impl Forces {
    pub(crate) fn prepare(
        config: &Config,
        configurations: impl IntoIterator<Item = (u64, JointForceConfiguration)>,
    ) -> Result<Self, Error> {
        if config.capacity == 0 {
            return Err(TransportError::InvalidConfig.into());
        }
        let mut groups: BTreeMap<String, (String, Vec<u64>)> = BTreeMap::new();
        let mut expected = BTreeMap::new();
        for (id, configuration) in configurations {
            let topic = gz_transport::as_valid_topic(&format!(
                "/model/{}/joint/{}/cmd_force",
                configuration.model_name(),
                configuration.joint_name()
            ));
            if topic.is_empty() {
                return Err(TransportError::InvalidName.into());
            }
            let remapped = config.topic_remap(&topic).unwrap_or(&topic);
            let key = gz_transport::qualified_name(&config.partition, &config.namespace, remapped)?;
            groups
                .entry(key)
                .or_insert_with(|| (topic, Vec::new()))
                .1
                .push(id);
            if expected.insert(id, configuration).is_some() {
                return Err(Error::JointForceIdentity);
            }
        }
        let mut result = Self {
            streams: Vec::new(),
            workers: Vec::new(),
            templates: Vec::new(),
            expected,
            next: 0,
        };
        for (index, (topic, ids)) in groups.into_values().enumerate() {
            let worker = index / config.capacity;
            if worker == result.workers.len() {
                result.workers.push(Node::new(config.clone())?);
            }
            let subscription = result.workers[worker].subscribe(&topic, "gz.msgs.Double")?;
            result.streams.push(Stream {
                ids: ids.clone(),
                subscription,
            });
            result.templates.push(Template { worker, topic, ids });
        }
        Ok(result)
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
    }

    pub(crate) fn restart(&mut self, sim: &Simulation) -> Result<(), Error> {
        self.clear();
        // Reuse workers prepared outside stepping; drop old queued epoch inputs.
        for template in &self.templates {
            let subscription =
                match self.workers[template.worker].subscribe(&template.topic, "gz.msgs.Double") {
                    Ok(subscription) => subscription,
                    Err(error) => {
                        self.clear();
                        return Err(error.into());
                    }
                };
            self.streams.push(Stream {
                ids: template.ids.clone(),
                subscription,
            });
        }
        if let Err(error) = self.bind(sim) {
            self.clear();
            return Err(error);
        }
        Ok(())
    }

    pub(crate) fn clear(&mut self) {
        self.streams.clear();
        self.next = 0;
    }
    pub(crate) fn shutdown(&mut self) {
        self.clear();
        for worker in &mut self.workers {
            worker.shutdown();
        }
    }
    pub(crate) fn add_statistics(&self, total: &mut Statistics) {
        for worker in &self.workers {
            add_transport_statistics(total, worker.statistics());
        }
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
                match self.streams[index]
                    .subscription
                    .recv_timeout(Duration::ZERO)
                {
                    Ok(message) => {
                        received = Some((index, message));
                        break;
                    }
                    Err(TransportError::Timeout) => {}
                    Err(error) => return Err(error.into()),
                }
            }
            let Some((index, message)) = received else {
                break;
            };
            counters.force_messages_received += 1;
            let Ok(value) = Double::decode(message.data.as_slice()) else {
                counters.force_messages_rejected += 1;
                continue;
            };
            let mut rejected = false;
            for id in &self.streams[index].ids {
                if set_joint_force(sim, *id, value.data).is_ok() {
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
