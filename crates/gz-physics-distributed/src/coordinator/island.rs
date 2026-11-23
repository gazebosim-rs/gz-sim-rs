// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::physics::{
    ComponentInfo, MAX_PENDING_IMPULSES, Snapshot, Tier, active_contact_group, validate_impulse,
};

// Form the two expected receipts from the last published observations. Only
// the selected IDs, their component history and their contacts can change
// owners. Every unrelated observation must survive the handoff bit for bit.
fn projected_handoff(
    workers: &[Observation; 2],
    ticket: &WarmTicket,
) -> (Snapshot, [Observation; 2]) {
    let (s, d) = (usize::from(ticket.source), usize::from(ticket.destination));
    let (selected, mut expected) = project_component(&workers[s], &workers[d], ticket);
    if s == 1 {
        expected.swap(0, 1);
    }
    (selected, expected)
}

pub(crate) fn project_component(
    source: &Observation,
    destination: &Observation,
    ticket: &WarmTicket,
) -> (Snapshot, [Observation; 2]) {
    let mut expected = [source.clone(), destination.clone()];
    let (s, d) = (0, 1);
    let ids = &ticket.ids;
    let mut selected = Snapshot {
        tick: ticket.tick,
        step: source.snapshot.step,
        ground: source.snapshot.ground,
        ..Default::default()
    };
    expected[s].snapshot.bodies.retain(|body| {
        if ids.contains(&body.id) {
            selected.bodies.push(body.clone());
            false
        } else {
            true
        }
    });
    expected[s].snapshot.impulses.retain(|pair| {
        if ids.contains(&pair.a) {
            selected.impulses.push(pair.clone());
            false
        } else {
            true
        }
    });
    expected[s].snapshot.supports.retain(|p| {
        if ids.contains(&p.body) {
            selected.supports.push(p.clone());
            false
        } else {
            true
        }
    });
    expected[d]
        .snapshot
        .supports
        .extend_from_slice(&selected.supports);
    expected[d].snapshot.supports.sort_by_key(|p| p.body);
    expected[s].components.retain(|c| {
        if ids.contains(&c.ids[0]) {
            selected.contact_history |= c.contact_history;
            false
        } else {
            true
        }
    });
    selected.contacts = selected.impulses.len();
    expected[d]
        .snapshot
        .bodies
        .extend_from_slice(&selected.bodies);
    expected[d].snapshot.bodies.sort_unstable_by_key(|b| b.id);
    expected[d]
        .snapshot
        .impulses
        .extend_from_slice(&selected.impulses);
    expected[d]
        .snapshot
        .impulses
        .sort_unstable_by_key(|p| (p.a, p.b));
    expected[d].components.push(ComponentInfo {
        ids: ids.to_vec(),
        contact_history: selected.contact_history,
    });
    expected[d].components.sort_unstable_by_key(|c| c.ids[0]);
    for (&id, &generation) in ids.iter().zip(&ticket.generations) {
        expected[s].generations.remove(&id);
        expected[d].generations.insert(id, generation);
    }
    for w in &mut expected {
        w.snapshot.contacts = w.snapshot.impulses.len();
        w.snapshot.contact_history = w.components.iter().any(|c| c.contact_history);
    }
    (selected, expected)
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Selection {
    ActiveGroup,
    FreePair,
    CompleteSolver,
}

impl<P: Peer> Coordinator<P> {
    /// Queue one world-frame central impulse for a dynamic body. Each successful
    /// call queues one input. An interrupted call can be retried with identical
    /// arguments or rolled back. The next advance consumes inputs in order.
    pub fn queue_impulse(&mut self, id: u64, impulse: [f64; 3]) -> Result<&Frame, Error> {
        let (owner, _) = self
            .frame
            .body(id)
            .ok_or(Error::Invalid("unknown impulse body"))?;
        let source = usize::from(owner);
        let state = &self.frame.workers[source];
        let local: Vec<_> = self
            .queued
            .iter()
            .filter(|p| state.generations.contains_key(&p.id))
            .cloned()
            .collect();
        validate_impulse(&state.snapshot.bodies, &local, id, impulse)
            .map_err(|_| Error::Invalid("impulse exceeds the admitted input bounds"))?;
        let input = PendingImpulse { id, impulse };
        let mut proposed = self.queued.clone();
        proposed.push(input.clone());
        self.frame.check_sweeps(&proposed)?;
        let tick = self.frame.tick;
        let mut counts = [0; 2];
        counts[source] = 1;
        let mut cursor = self.begin(Action::Queue(input.clone()), counts)?;
        let ack = observation(self.call(
            &mut cursor,
            source,
            Command::QueueImpulse { tick, id, impulse },
        )?)?;
        if ack != self.frame.workers[source] {
            return Err(Error::Invalid("queueing input changed committed state"));
        }
        self.queued.push(input);
        self.commit_action();
        Ok(&self.frame)
    }

    /// Move a whole connected dynamic worker to its peer. A whole pair can
    /// also transfer in free flight when the destination is empty.
    pub fn transfer_island(&mut self, source: u8) -> Result<&Frame, Error> {
        let worker = self
            .frame
            .workers
            .get(usize::from(source))
            .ok_or(Error::Invalid("unknown source worker"))?;
        let ids: Vec<_> = worker.snapshot.bodies.iter().map(|b| b.id).collect();
        let legacy_free_pair = ids.len() == 2
            && self.frame.workers[1 - usize::from(source)]
                .snapshot
                .bodies
                .is_empty();
        self.transfer_members(
            source,
            ids,
            if legacy_free_pair {
                Selection::FreePair
            } else {
                Selection::ActiveGroup
            },
        )
    }

    /// Transfer the complete native solver containing `member` if its active
    /// contacts connect all its dynamic boxes at this barrier.
    pub fn transfer_group(&mut self, member: u64) -> Result<&Frame, Error> {
        self.transfer_containing(member, Selection::ActiveGroup)
    }

    /// Move exactly one complete native solver. This includes singletons,
    /// prescribed members and separated bodies with previous contact history.
    /// The receiving worker must remain physically admissible with the arrival.
    pub fn transfer_component(&mut self, member: u64) -> Result<&Frame, Error> {
        self.transfer_containing(member, Selection::CompleteSolver)
    }

    fn transfer_containing(&mut self, member: u64, selection: Selection) -> Result<&Frame, Error> {
        let (source, _) = self
            .frame
            .body(member)
            .ok_or(Error::Invalid("unknown group member"))?;
        let worker = &self.frame.workers[usize::from(source)];
        let component = worker
            .components
            .iter()
            .find(|c| c.ids.binary_search(&member).is_ok())
            .ok_or(Error::Invalid("missing component"))?;
        self.transfer_members(source, component.ids.clone(), selection)
    }

    fn transfer_members(
        &mut self,
        source: u8,
        ids: Vec<u64>,
        selection: Selection,
    ) -> Result<&Frame, Error> {
        let s = usize::from(source);
        let d = 1 - s;
        let old = &self.frame.workers[s];
        if selection == Selection::FreePair {
            if old.snapshot.bodies.iter().any(|b| b.tier != Tier::Dynamic) {
                return Err(Error::Invalid("warm handoff requires dynamic boxes"));
            }
        } else if selection == Selection::ActiveGroup && !active_contact_group(&old.snapshot, &ids)
        {
            return Err(Error::Invalid(
                "selective handoff requires a connected group of dynamic boxes",
            ));
        }
        let generations = ids
            .iter()
            .map(|id| old.generations[id].checked_add(1).ok_or(Error::Capacity))
            .collect::<Result<Vec<_>, _>>()?;
        let ticket = WarmTicket {
            run_id: self.run,
            serial: self.serial.checked_add(1).ok_or(Error::Capacity)?,
            source,
            destination: d as u8,
            tick: self.frame.tick,
            ids,
            generations,
        };
        let (selected, expected) = projected_handoff(&self.frame.workers, &ticket);
        let inputs: Vec<_> = self
            .queued
            .iter()
            .filter(|p| ticket.ids.contains(&p.id))
            .cloned()
            .collect();
        let dest = &expected[d];
        if !self.frame.can_receive(d, &ticket.ids)
            || self
                .queued
                .iter()
                .filter(|p| dest.generations.contains_key(&p.id))
                .count()
                > MAX_PENDING_IMPULSES
        {
            return Err(Error::Capacity);
        }
        if !partition_can_step(&dest.snapshot, &dest.components, &self.queued) {
            return Err(Error::Colocation);
        }
        let mut counts = [2; 2];
        counts[s] = 3;
        let complete = selection == Selection::CompleteSolver;
        let (action, export) = if complete {
            (
                Action::TransferSolver(ticket.clone()),
                Command::ExportSolver {
                    ticket: ticket.clone(),
                },
            )
        } else {
            (
                Action::TransferIsland(ticket.clone()),
                Command::ExportWarm {
                    ticket: ticket.clone(),
                },
            )
        };
        let mut cursor = self.begin(action, counts)?;
        let Outcome::WarmExported(handoff) = self.call(&mut cursor, s, export)? else {
            return Err(Error::Invalid("warm export acknowledgement"));
        };
        if handoff.receipt.ticket != ticket
            || handoff.checkpoint.snapshot != selected
            || handoff.checkpoint.pending != inputs
            || handoff.validate_scope(complete).is_err()
        {
            return Err(Error::Invalid(
                "warm export changed identity, state, or queued inputs",
            ));
        }
        let receipt = handoff.receipt.clone();
        // The source and destination each preserved their local FIFO. Convey the
        // original interleaving only when native append order would differ.
        let mut incoming_slots = 0u32;
        let mut interleaved = false;
        for (slot, p) in self
            .queued
            .iter()
            .filter(|p| dest.generations.contains_key(&p.id))
            .enumerate()
        {
            if ticket.ids.binary_search(&p.id).is_ok() {
                incoming_slots |= 1u32 << slot;
            } else if incoming_slots != 0 {
                interleaved = true;
            }
        }
        let prepare = if interleaved {
            Command::PrepareOrdered {
                handoff,
                complete,
                incoming_slots,
            }
        } else if complete {
            Command::PrepareSolver { handoff }
        } else {
            Command::PrepareWarm { handoff }
        };
        if self.call(&mut cursor, d, prepare)? != Outcome::WarmPrepared(receipt.prepared()) {
            return Err(Error::Invalid("warm prepare acknowledgement"));
        }
        if self.call(
            &mut cursor,
            s,
            Command::ReleaseWarm {
                receipt: receipt.prepared(),
            },
        )? != Outcome::WarmReleased(receipt.released())
        {
            return Err(Error::Invalid("warm release acknowledgement"));
        }
        let Outcome::WarmActivated(active) = self.call(
            &mut cursor,
            d,
            Command::ActivateWarm {
                receipt: receipt.released(),
            },
        )?
        else {
            return Err(Error::Invalid("warm activate acknowledgement"));
        };
        let remainder = observation(self.call(&mut cursor, s, Command::Observe)?)?;
        if remainder != expected[s] || active != expected[d] {
            return Err(Error::Invalid(
                "warm activation changed contacts, state or ownership",
            ));
        }
        self.frame = Frame::new(ticket.tick, expected)?;
        self.serial = ticket.serial;
        self.commit_action();
        Ok(&self.frame)
    }
}
