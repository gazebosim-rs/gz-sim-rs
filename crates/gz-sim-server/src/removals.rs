// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{Counters, Error};
use bevy_ecs::hierarchy::ChildOf;
use gz_msgs::{
    msgs::{Boolean, Entity, entity::Type},
    prost::Message,
};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{EntityKind, LocalName, remove_model};
use gz_transport::{Error as TransportError, Node, Request, Service};
use std::{collections::BTreeMap, num::NonZeroUsize, time::Duration};

struct Pending {
    entity: Entity,
    reply: Option<Request>,
}

pub(crate) struct Removals {
    services: [Service; 2],
    pending: Vec<Pending>,
    next: usize,
}

impl Removals {
    pub(crate) fn new(node: &Node, world: &str) -> Result<Self, Error> {
        let topic = format!("/world/{world}/remove");
        Ok(Self {
            services: [
                node.serve(&topic, "gz.msgs.Entity", "gz.msgs.Boolean")?,
                node.serve(
                    &format!("{topic}/blocking"),
                    "gz.msgs.Entity",
                    "gz.msgs.Boolean",
                )?,
            ],
            pending: Vec::new(),
            next: 0,
        })
    }

    pub(crate) fn poll(
        &mut self,
        budget: NonZeroUsize,
        counters: &mut Counters,
    ) -> Result<(), Error> {
        for _ in 0..budget.get() {
            if self.pending.len() == budget.get() {
                break;
            }
            let mut received = None;
            for _ in 0..2 {
                let index = self.next;
                self.next ^= 1;
                match self.services[index].recv_timeout(Duration::ZERO) {
                    Ok(request) => {
                        received = Some((index, request));
                        break;
                    }
                    Err(TransportError::Timeout) => {}
                    Err(error) => return Err(error.into()),
                }
            }
            let Some((index, request)) = received else {
                break;
            };
            match Entity::decode(request.data.as_slice()) {
                Ok(entity) => {
                    counters.removals_queued += 1;
                    let reply = if index == 1 {
                        Some(request)
                    } else {
                        respond(request, true, counters);
                        None
                    };
                    self.pending.push(Pending { entity, reply });
                }
                Err(_) => {
                    counters.removals_rejected += 1;
                    respond(request, false, counters);
                }
            }
        }
        Ok(())
    }

    pub(crate) fn execute(&mut self, sim: &mut Simulation, counters: &mut Counters) {
        // Native commands resolve before deferred deletion; duplicate targets succeed together.
        let targets: Vec<_> = self
            .pending
            .iter()
            .map(|pending| target(sim, &pending.entity))
            .collect();
        let mut results = BTreeMap::new();
        for (pending, id) in self.pending.drain(..).zip(targets) {
            let removed = id.is_some_and(|id| {
                *results
                    .entry(id)
                    .or_insert_with(|| remove_model(sim, id).is_ok())
            });
            if removed {
                counters.removals_completed += 1;
            } else {
                counters.removals_failed += 1;
            }
            if let Some(reply) = pending.reply {
                respond(reply, removed, counters);
            }
        }
    }
}

fn target(sim: &Simulation, request: &Entity) -> Option<u64> {
    if request.id != 0 {
        return Some(request.id);
    }
    if request.r#type != Type::Model as i32 || request.name.is_empty() {
        return None;
    }
    let mut matches = sim.world().iter_entities().filter_map(|entity| {
        if entity.get::<EntityKind>() != Some(&EntityKind::Model) {
            return None;
        }
        let mut ancestor = Some(entity.id());
        for part in request.name.rsplit("::") {
            let current = ancestor?;
            if sim.world().get::<LocalName>(current)?.0 != part {
                return None;
            }
            ancestor = sim.world().get::<ChildOf>(current).map(ChildOf::parent);
        }
        let id = entity.get::<EntityId>()?.get();
        (entity_by_id(sim.world(), id) == Some(entity.id())).then_some(id)
    });
    let id = matches.next()?;
    // Native lookup uses an unordered set. Ambiguity has no stable target contract.
    matches.next().is_none().then_some(id)
}

#[cfg(test)]
#[path = "../tests/private/removals.rs"]
mod tests;

fn respond(request: Request, accepted: bool, counters: &mut Counters) {
    if request
        .respond(
            Boolean {
                data: accepted,
                ..Default::default()
            }
            .encode_to_vec(),
            true,
        )
        .is_err()
    {
        counters.response_failures += 1;
    }
}
