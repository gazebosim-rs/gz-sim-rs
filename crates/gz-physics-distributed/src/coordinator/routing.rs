// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plan cold-body handoffs before a predicted cross-worker interaction.
use super::*;
use crate::physics::{
    ComponentInfo, ForceMotion, MAX_PENDING_IMPULSES, Snapshot, forced_contact_groups,
    forced_groups, validate_impulse,
};

/// One body changing ownership at the same published tick. Members of a warm
/// component travel together in a single native checkpoint transaction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyRoute {
    pub body: u64,
    pub destination: u8,
}

struct Choice {
    routes: Vec<BodyRoute>,
    new_ids: [usize; 2],
    incoming_inputs: [usize; 2],
}

pub(super) struct RoutingScene {
    pub(super) snapshot: Snapshot,
    pub(super) components: Vec<ComponentInfo>,
    pub(super) owners: Vec<usize>,
}

impl RoutingScene {
    pub(super) fn from_frame(frame: &Frame) -> Self {
        let mut scene = Self {
            snapshot: Snapshot {
                tick: frame.tick,
                step: frame.step(),
                ground: frame.ground(),
                ..Snapshot::default()
            },
            components: Vec::new(),
            owners: Vec::new(),
        };
        for (owner, worker) in frame.workers.iter().enumerate() {
            scene
                .snapshot
                .bodies
                .extend_from_slice(&worker.snapshot.bodies);
            scene
                .snapshot
                .impulses
                .extend_from_slice(&worker.snapshot.impulses);
            scene
                .snapshot
                .supports
                .extend_from_slice(&worker.snapshot.supports);
            scene.snapshot.contacts += worker.snapshot.contacts;
            scene.snapshot.contact_history |= worker.snapshot.contact_history;
            scene.components.extend_from_slice(&worker.components);
            scene
                .owners
                .extend(std::iter::repeat_n(owner, worker.components.len()));
        }
        scene.snapshot.supports.sort_by_key(|p| p.body);
        scene.snapshot.bodies.sort_by_key(|b| b.id);
        scene.snapshot.impulses.sort_by_key(|p| (p.a, p.b));
        scene
    }

    // Project each source cold body as the singleton solver that arrives at
    // prepare/activate. A resident warm component retains its native solver.
    fn choice(
        &self,
        indices: &[usize],
        destination: usize,
        frame: &Frame,
        accepted: &[PendingImpulse],
        projected_inputs: &[PendingImpulse],
        forced: &[ForceMotion],
    ) -> Option<Choice> {
        let mut routes = Vec::new();
        let mut components = Vec::new();
        let mut ids = BTreeSet::new();
        let mut new_ids = [0; 2];
        let mut incoming_inputs = [0; 2];
        for &i in indices {
            let c = &self.components[i];
            ids.extend(c.ids.iter().copied());
            let owner = self.owners[i];
            if owner == destination {
                components.push(c.clone());
            } else {
                let commands = accepted
                    .iter()
                    .filter(|p| c.ids.binary_search(&p.id).is_ok())
                    .count();
                // An established native contact owns the destination cache. A
                // free arrival with accepted commands must bring its complete
                // native image, including the ordered command ledger.
                let native_image = c.contact_history || commands != 0;
                if native_image
                    && !indices.iter().any(|&j| {
                        self.owners[j] == destination && self.components[j].contact_history
                    })
                {
                    return None;
                }
                incoming_inputs[destination] += commands;
                for &id in &c.ids {
                    new_ids[destination] += usize::from(frame.workers[owner].generations[&id] == 0);
                    routes.push(BodyRoute {
                        body: id,
                        destination: destination as u8,
                    });
                    if !native_image {
                        components.push(ComponentInfo {
                            ids: vec![id],
                            contact_history: false,
                        });
                    }
                }
                if native_image {
                    components.push(c.clone());
                }
            }
        }
        // Every active contact belongs to one resident solver. Keeping the
        // projected contact sample alongside the bodies also preserves history.
        let impulses: Vec<_> = self
            .snapshot
            .impulses
            .iter()
            .filter(|p| ids.contains(&p.a))
            .cloned()
            .collect();
        let projected = Snapshot {
            tick: self.snapshot.tick,
            step: self.snapshot.step,
            ground: self.snapshot.ground,
            supports: self
                .snapshot
                .supports
                .iter()
                .filter(|p| ids.contains(&p.body))
                .cloned()
                .collect(),
            bodies: self
                .snapshot
                .bodies
                .iter()
                .filter(|b| ids.contains(&b.id))
                .cloned()
                .collect(),
            contacts: impulses.len(),
            contact_history: components.iter().any(|c| c.contact_history),
            impulses,
        };
        if forced_groups(&projected, &components, projected_inputs, forced).is_err() {
            return None;
        }
        routes.sort();
        Some(Choice {
            routes,
            new_ids,
            incoming_inputs,
        })
    }
}

fn better(candidate: &[BodyRoute], current: &[BodyRoute]) -> bool {
    (candidate.len(), candidate) < (current.len(), current)
}

impl<P: Peer> Coordinator<P> {
    /// Plan same-tick transfers for predicted interactions. Local contact
    /// caches travel as complete native solvers when independently warm groups
    /// approach one another. Native groups carry their accepted inputs. A cold
    /// body with a queued input remains on its owner.
    /// Among feasible placements, minimize the total number of moved bodies.
    /// Stable-ID order breaks ties. This performs no network exchange.
    pub fn contact_routes(&self) -> Result<Vec<BodyRoute>, Error> {
        self.plan_routes(&self.queued, None, &[])
    }

    pub(super) fn plan_routes(
        &self,
        projected: &[PendingImpulse],
        next_input: Option<u64>,
        forced: &[ForceMotion],
    ) -> Result<Vec<BodyRoute>, Error> {
        self.plan_routes_in(&self.frame, projected, next_input, forced)
    }

    pub(super) fn plan_routes_in(
        &self,
        frame: &Frame,
        projected: &[PendingImpulse],
        next_input: Option<u64>,
        forced: &[ForceMotion],
    ) -> Result<Vec<BodyRoute>, Error> {
        let scene = RoutingScene::from_frame(frame);
        let groups = forced_contact_groups(&scene.snapshot, &scene.components, projected, forced)
            .map_err(|_| Error::Colocation)?;
        self.place_groups(frame, &scene, groups, projected, next_input, forced)
    }

    fn place_groups(
        &self,
        frame: &Frame,
        scene: &RoutingScene,
        groups: Vec<Vec<usize>>,
        projected: &[PendingImpulse],
        next_input: Option<u64>,
        forced: &[ForceMotion],
    ) -> Result<Vec<BodyRoute>, Error> {
        // Reserve arrivals against current free space, so every intermediate
        // transaction fits even if an unrelated departure happens later.
        let mut capacity = [
            MAX_BODY_IDS - frame.known_ids(0),
            MAX_BODY_IDS - frame.known_ids(1),
            MAX_PENDING_IMPULSES - self.input_count_in(frame, 0),
            MAX_PENDING_IMPULSES - self.input_count_in(frame, 1),
        ];
        // An already-admitted native domain with no prospective neighbors needs
        // no ownership planning. Reserve any new input on that domain up front.
        if let Some(id) = next_input
            && let Some(&index) = groups.iter().find_map(|g| match g.as_slice() {
                [i] if scene.components[*i].ids.binary_search(&id).is_ok() => Some(i),
                _ => None,
            })
        {
            let slot = 2 + scene.owners[index];
            capacity[slot] = capacity[slot].checked_sub(1).ok_or(Error::Capacity)?;
        }
        let mut plans = BTreeMap::from([([0usize; 4], Vec::<BodyRoute>::new())]);
        for indices in groups.into_iter().filter(|g| g.len() > 1) {
            let owner = scene.owners[indices[0]];
            let local = indices.iter().all(|&i| scene.owners[i] == owner);
            let new_input = next_input.is_some_and(|id| {
                indices
                    .iter()
                    .any(|&i| scene.components[i].ids.binary_search(&id).is_ok())
            });
            let options = [0, 1].map(|d| {
                if local && d != owner {
                    return None;
                }
                let mut choice =
                    scene.choice(&indices, d, frame, &self.queued, projected, forced)?;
                choice.incoming_inputs[d] += usize::from(new_input);
                Some(choice)
            });
            if options.iter().all(Option::is_none) {
                return Err(Error::Colocation);
            }
            let mut next = BTreeMap::new();
            for (used, current) in plans {
                for option in options.iter().flatten() {
                    let costs = [
                        option.new_ids[0],
                        option.new_ids[1],
                        option.incoming_inputs[0],
                        option.incoming_inputs[1],
                    ];
                    let slots = std::array::from_fn::<_, 4, _>(|w| used[w] + costs[w]);
                    if (0..4).any(|w| slots[w] > capacity[w]) {
                        continue;
                    }
                    let mut routes = current.clone();
                    routes.extend_from_slice(&option.routes);
                    routes.sort();
                    match next.entry(slots) {
                        std::collections::btree_map::Entry::Vacant(e) => {
                            e.insert(routes);
                        }
                        std::collections::btree_map::Entry::Occupied(mut e)
                            if better(&routes, e.get()) =>
                        {
                            e.insert(routes);
                        }
                        _ => {}
                    }
                }
            }
            if next.is_empty() {
                return Err(Error::Capacity);
            }
            plans = next;
        }
        plans
            .into_values()
            .min_by(|a, b| (a.len(), a).cmp(&(b.len(), b)))
            .ok_or(Error::Capacity)
    }

    pub(super) fn island_routes(
        &self,
        gravity: [f64; 3],
        inputs: &[crate::physics::BodyWrench],
    ) -> Result<(Vec<BodyRoute>, Vec<u64>), Error> {
        let scene = RoutingScene::from_frame(&self.frame);
        let motion = crate::physics::field_motion(
            scene.snapshot.step,
            &scene.snapshot.bodies,
            gravity,
            inputs,
        )
        .map_err(|_| Error::Invalid("invalid native step wrenches"))?;
        let groups =
            forced_contact_groups(&scene.snapshot, &scene.components, &self.queued, &motion)
                .map_err(|_| Error::Colocation)?;
        let ids = crate::adaptive::group_refinements(
            &scene.snapshot,
            &scene.components,
            &groups,
            &self.queued,
            &motion,
        )?;
        let routes = self.place_groups(&self.frame, &scene, groups, &self.queued, None, &motion)?;
        Ok((routes, ids))
    }

    /// Route predicted collisions, then advance one tick. Each acknowledged
    /// handoff publishes at the old tick. On interruption, retry this method
    /// or abort the incomplete individual operation at its last published frame.
    pub fn advance_routed(&mut self) -> Result<&Frame, Error> {
        if self
            .pending
            .as_ref()
            .is_some_and(|p| p.action == Action::Advance)
        {
            return self.advance();
        }
        self.execute_routes(self.contact_routes()?, [1, 1])?;
        self.advance()
    }

    /// Co-locate all current prospective interactions at the same public tick.
    /// Each completed transfer is individually visible and recoverable.
    pub fn colocate_contacts(&mut self) -> Result<&Frame, Error> {
        self.execute_routes(self.contact_routes()?, [0, 0])?;
        Ok(&self.frame)
    }

    fn input_count_in(&self, frame: &Frame, worker: usize) -> usize {
        self.queued
            .iter()
            .filter(|p| frame.workers[worker].generations.contains_key(&p.id))
            .count()
    }

    /// Place the bodies for the proposed next-tick impulse, then queue it on
    /// its final owner. Cold arrivals carrying earlier commands use native images when joining a loaded solver. A lost
    /// input receipt can be retried with the exact same impulse.
    pub fn queue_impulse_routed(&mut self, id: u64, impulse: [f64; 3]) -> Result<&Frame, Error> {
        self.queue_impulse_routed_forced(id, impulse, &[])
    }

    pub(crate) fn queue_impulse_routed_forced(
        &mut self,
        id: u64,
        impulse: [f64; 3],
        forced: &[ForceMotion],
    ) -> Result<&Frame, Error> {
        if self
            .pending
            .as_ref()
            .is_some_and(|p| matches!(p.action, Action::Queue(_)))
        {
            return self.queue_impulse(id, impulse);
        }
        let (original, body) = self
            .frame
            .body(id)
            .ok_or(Error::Invalid("unknown impulse body"))?;
        let prior: Vec<_> = self.queued.iter().filter(|p| p.id == id).cloned().collect();
        validate_impulse(std::slice::from_ref(body), &prior, id, impulse)
            .map_err(|_| Error::Invalid("impulse exceeds the admitted input bounds"))?;
        let mut projected = self.queued.clone();
        projected.push(PendingImpulse { id, impulse });
        let routes = self.plan_routes(&projected, Some(id), forced)?;
        let owner = usize::from(
            routes
                .iter()
                .find(|r| r.body == id)
                .map_or(original, |r| r.destination),
        );
        if self.input_count_in(&self.frame, owner) >= MAX_PENDING_IMPULSES {
            return Err(Error::Capacity);
        }
        let mut counts = [0; 2];
        counts[owner] = 1;
        self.execute_routes(routes, counts)?;
        self.queue_impulse(id, impulse)
    }

    pub(super) fn execute_routes(
        &mut self,
        routes: Vec<BodyRoute>,
        mut required: [usize; 2],
    ) -> Result<(), Error> {
        if self.aborting
            || self.pending.as_ref().is_some_and(|p| {
                !matches!(p.action, Action::Transfer(_) | Action::TransferSolver(_))
            })
        {
            return Err(Error::Pending);
        }
        if let Some(worker) = self.offline.iter().position(|&x| x) {
            return Err(Error::Offline(worker as u8));
        }
        // Reserve the whole route and the advancing barrier before any transfer.
        // During a retry, capacity for the current partial operation was
        // reserved relative to its original journal start.
        let final_action = usize::from(required != [0, 0]);
        let mut transactions = Vec::with_capacity(routes.len());
        for r in &routes {
            let d = usize::from(r.destination);
            let component = self.frame.workers[1 - d]
                .components
                .iter()
                .find(|c| c.ids.binary_search(&r.body).is_ok())
                .ok_or(Error::Invalid("route body has no source component"))?;
            let native_image = component.contact_history
                || self
                    .queued
                    .iter()
                    .any(|p| component.ids.binary_search(&p.id).is_ok());
            if native_image {
                if component.ids[0] != r.body {
                    continue;
                }
                if !component.ids.iter().all(|&id| {
                    routes
                        .binary_search(&BodyRoute {
                            body: id,
                            destination: r.destination,
                        })
                        .is_ok()
                }) {
                    return Err(Error::Invalid("route would split a native solver"));
                }
            }
            required[d] += 2;
            required[1 - d] += 3;
            transactions.push((*r, native_image));
        }
        if self.recording.as_ref().is_some_and(|a| {
            transactions.len() + final_action > recovery::MAX_ACTIONS.saturating_sub(a.len())
        }) {
            return Err(Error::Capacity);
        }
        self.reserve_journal(required)?;
        if let Some(pending) = &self.pending {
            let (body, destination, warm) = match &pending.action {
                Action::Transfer(t) if !t.promote => (t.body_id, t.destination, false),
                Action::TransferSolver(t) => (t.ids[0], t.destination, true),
                _ => return Err(Error::Pending),
            };
            if transactions.first() != Some(&(BodyRoute { body, destination }, warm)) {
                return Err(Error::Pending);
            }
        }
        for (r, warm) in transactions {
            if warm {
                self.transfer_component(r.body)?;
            } else {
                self.transfer(r.body, false)?;
            }
        }
        Ok(())
    }
}

impl<P: OverlappingPeers> Coordinator<P> {
    /// Route predicted contacts at their original tick, then dispatch both
    /// independent worker steps concurrently. Retries resume either barrier.
    pub fn advance_routed_overlapped(&mut self) -> Result<&Frame, Error> {
        if self
            .pending
            .as_ref()
            .is_some_and(|p| p.action == Action::Advance)
        {
            return self.advance_overlapped();
        }
        self.execute_routes(self.contact_routes()?, [1, 1])?;
        self.advance_overlapped()
    }
}
