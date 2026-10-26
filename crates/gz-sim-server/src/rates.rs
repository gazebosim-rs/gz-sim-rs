// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{Counters, Error};
use gz_msgs::{msgs::Double, prost::Message};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{ImuPublication, RateChange, set_imu_rate};
use gz_transport::{Config, Error as TransportError, Node, Service, Statistics};
use std::{collections::BTreeMap, num::NonZeroUsize, time::Duration};

#[derive(Clone)]
struct ProviderSpec {
    topic: String,
    name: String,
}

fn plan<'a>(
    config: &Config,
    sensors: impl IntoIterator<Item = &'a ImuPublication>,
) -> Result<Vec<Vec<ProviderSpec>>, TransportError> {
    if config.capacity == 0 {
        return Err(TransportError::InvalidConfig);
    }
    let mut groups: BTreeMap<String, Vec<ProviderSpec>> = BTreeMap::new();
    for sensor in sensors {
        let topic = format!("{}/set_rate", gz_transport::as_valid_topic(&sensor.topic));
        let remapped = config.topic_remap(&topic).unwrap_or(&topic);
        let endpoint =
            gz_transport::qualified_name(&config.partition, &config.namespace, remapped)?;
        groups.entry(endpoint).or_default().push(ProviderSpec {
            topic,
            name: sensor.entity_name.clone(),
        });
    }
    let total: usize = groups.values().map(Vec::len).sum();
    let count = groups
        .values()
        .map(Vec::len)
        .max()
        .unwrap_or(0)
        .max(total.div_ceil(config.capacity));
    let mut workers: Vec<Vec<ProviderSpec>> = (0..count).map(|_| Vec::new()).collect();
    for group in groups.into_values() {
        // Distinct workers per endpoint; least-loaded placement minimizes IO threads.
        let mut slots: Vec<_> = (0..count).collect();
        slots.sort_by_key(|index| workers[*index].len());
        for (provider, index) in group.into_iter().zip(slots) {
            workers[index].push(provider);
        }
    }
    Ok(workers)
}

pub(crate) struct Provider {
    name: String,
    id: Option<u64>,
    service: Service,
}

struct Template {
    worker: usize,
    spec: ProviderSpec,
}

pub(crate) struct Rates {
    // Drop leases before their worker Nodes.
    providers: Vec<Provider>,
    workers: BTreeMap<usize, Node>,
    templates: Vec<Template>,
    next: usize,
}

impl Rates {
    pub(crate) fn prepare<'a>(
        config: &Config,
        sensors: impl IntoIterator<Item = &'a ImuPublication>,
    ) -> Result<Self, Error> {
        let mut rates = Self {
            providers: Vec::new(),
            workers: BTreeMap::new(),
            templates: Vec::new(),
            next: 0,
        };
        for (index, providers) in plan(config, sensors)?.into_iter().enumerate() {
            let node = Node::new(config.clone())?;
            for provider in providers {
                let service = node.serve(&provider.topic, "gz.msgs.Double", "gz.msgs.Empty")?;
                rates.templates.push(Template {
                    worker: index,
                    spec: provider.clone(),
                });
                rates.providers.push(Provider {
                    name: provider.name,
                    id: None,
                    service,
                });
            }
            rates.workers.insert(index, node);
        }
        Ok(rates)
    }

    pub(crate) fn bind(&mut self, sim: &Simulation) -> Result<(), Error> {
        let mut identities = BTreeMap::new();
        for entity in sim.world().iter_entities() {
            if entity.contains::<gz_sim_scene::PendingResetRemoval>() {
                continue;
            }
            if let Some(sensor) = entity.get::<ImuPublication>() {
                let id = entity.get::<EntityId>().ok_or(Error::ImuIdentity)?.get();
                if identities.insert(sensor.entity_name.as_str(), id).is_some() {
                    return Err(Error::ImuIdentity);
                }
            }
        }
        for provider in &mut self.providers {
            provider.id = identities.get(provider.name.as_str()).copied();
        }
        self.retire(sim);
        Ok(())
    }

    pub(crate) fn retire(&mut self, sim: &Simulation) {
        self.providers.retain(|provider| {
            provider
                .id
                .and_then(|id| entity_by_id(sim.world(), id))
                .and_then(|entity| sim.world().get::<ImuPublication>(entity))
                .is_some_and(|sensor| sensor.entity_name == provider.name)
        });
    }

    pub(crate) fn restart(&mut self, sim: &Simulation) -> Result<(), Error> {
        // Native plugin reload retires handlers and queued controls from the old epoch.
        self.clear();
        self.providers = self
            .templates
            .iter()
            .map(|template| {
                let service = self.workers[&template.worker].serve(
                    &template.spec.topic,
                    "gz.msgs.Double",
                    "gz.msgs.Empty",
                )?;
                Ok(Provider {
                    name: template.spec.name.clone(),
                    id: None,
                    service,
                })
            })
            .collect::<Result<_, Error>>()?;
        if let Err(error) = self.bind(sim) {
            self.clear();
            return Err(error);
        }
        Ok(())
    }

    pub(crate) fn add_statistics(&self, total: &mut Statistics) {
        for worker in self.workers.values() {
            let stats = worker.statistics();
            total.queue_drops = total.queue_drops.saturating_add(stats.queue_drops);
            total.malformed_frames = total
                .malformed_frames
                .saturating_add(stats.malformed_frames);
            total.send_failures = total.send_failures.saturating_add(stats.send_failures);
            total.reply_timeouts = total.reply_timeouts.saturating_add(stats.reply_timeouts);
            total.routing_errors = total.routing_errors.saturating_add(stats.routing_errors);
        }
    }

    pub(crate) fn clear(&mut self) {
        self.providers.clear();
    }

    pub(crate) fn shutdown(&mut self) {
        self.clear();
        for node in self.workers.values_mut() {
            node.shutdown();
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
            for _ in 0..self.providers.len() {
                self.next %= self.providers.len();
                let provider = &self.providers[self.next];
                self.next += 1;
                match provider.service.recv_timeout(Duration::ZERO) {
                    Ok(request) => {
                        received = Some((provider.id, request));
                        break;
                    }
                    Err(TransportError::Timeout) => {}
                    Err(error) => return Err(error.into()),
                }
            }
            let Some((id, request)) = received else { break };
            let change = Double::decode(request.data.as_slice())
                .ok()
                .and_then(|message| id.and_then(|id| set_imu_rate(sim, id, message.data).ok()));
            match change {
                Some(RateChange::Applied) => counters.imu_rates_applied += 1,
                Some(RateChange::Ignored) => counters.imu_rates_ignored += 1,
                None => counters.imu_rates_rejected += 1,
            }
            // Empty response services are one-way; handler outcomes stay local.
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../tests/private/rates.rs"]
mod tests;
