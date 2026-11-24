// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::time::Duration;

impl WorkerPool {
    /// Confirm an unavailable migration acknowledgement once at its original
    /// sequence. If the worker is gone or still unavailable, rebuild that
    /// participant from its verified transaction prefix. Direct pool callers
    /// default to returning the first failure. Supervised Bevy scenes enable this.
    pub fn set_handoff_confirmation(&mut self, enabled: bool) {
        self.handoff_confirmation = enabled.then_some(Duration::from_millis(150));
    }
    /// Set the grace for the second attempt on a still-live migration peer.
    /// The first operation and known-dead restart keep their ordinary deadlines.
    /// None returns the first error to the caller. The maximum grace is one second.
    pub fn set_handoff_confirmation_window(
        &mut self,
        window: Option<Duration>,
    ) -> Result<(), Error> {
        if window.is_some_and(|d| d.is_zero() || d > Duration::from_secs(1)) {
            return Err(Error::Invalid(
                "handoff confirmation grace must be positive and at most one second",
            ));
        }
        self.handoff_confirmation = window;
        Ok(())
    }

    fn exchange_handoff(
        &mut self,
        stage: &mut Stage,
        w: usize,
        command: Command,
    ) -> Result<Outcome, Error> {
        self.exchange_in_stage(stage, w, command, self.handoff_confirmation)
    }

    pub(super) fn confirm_handoff_request(
        &mut self,
        stage: &Stage,
        w: usize,
        request: &Request,
        grace: Duration,
    ) -> Result<Reply, Error> {
        use crate::{protocol::ProtocolError, transport::TransportError};
        if !self.peers[w].has_exited()? {
            match self.peers[w].request_with_timeout(request, grace) {
                Ok(reply) => return Ok(reply),
                Err(
                    TransportError::Unavailable(_)
                    | TransportError::Protocol(ProtocolError::Sequence { .. }),
                ) => {}
                Err(error) => return Err(error.into()),
            }
        }
        self.recover_stage_prefix(w, stage)?;
        Ok(self.peers[w].request(request)?)
    }

    pub(super) fn validate_transfer_geometry(
        &self,
        destination: &Observation,
        pending: &[PendingImpulse],
    ) -> Result<(), Error> {
        crate::physics::forced_groups_with_profile(
            &destination.snapshot, &destination.components, pending, &[], self.profile,
        ).map(|_| ()).map_err(|_| Error::Colocation)
    }

    /// Move the entire native solver containing this member at the current tick.
    pub fn transfer_component(
        &mut self,
        member: u64,
        destination: u8,
    ) -> Result<&PoolFrame, Error> {
        let source = self
            .frame
            .body(member)
            .ok_or(Error::Invalid("unknown pool member"))?
            .0;
        let mut counts = vec![0; self.peers.len()];
        if destination as usize >= counts.len() || source == destination {
            return Err(Error::Invalid("invalid pool destination"));
        }
        counts[source as usize] = 3;
        counts[destination as usize] = 2;
        self.reserve(&counts)?;
        let mut stage = self.stage()?;
        self.transfer(&mut stage, member, usize::from(destination))?;
        self.commit(stage)
    }
    pub(super) fn transfer(
        &mut self,
        stage: &mut Stage,
        member: u64,
        d: usize,
    ) -> Result<(), Error> {
        let source = self.stage_transfer(stage, member, d)?;
        self.verify_source(stage, source)
    }

    fn verify_source(&mut self, stage: &mut Stage, source: usize) -> Result<(), Error> {
        let reply = self.exchange_handoff(stage, source, Command::Observe)?;
        if matches!(reply, Outcome::State(actual) if actual == stage.frame.workers[source]) {
            return Ok(());
        }
        Err(Error::Invalid("pool handoff changed native state"))
    }

    /// Export, prepare, release and activate. Verification of the final source frame
    /// remains the caller's responsibility before committing this stage.
    fn stage_transfer(&mut self, stage: &mut Stage, member: u64, d: usize) -> Result<usize, Error> {
        let (owner, _) = stage
            .frame
            .body(member)
            .ok_or(Error::Invalid("unknown pool member"))?;
        let s = usize::from(owner);
        if d >= self.peers.len() || s == d {
            return Err(Error::Invalid("invalid pool destination"));
        }
        let source = &stage.frame.workers[s];
        let ids = source
            .components
            .iter()
            .find(|c| c.ids.binary_search(&member).is_ok())
            .ok_or(Error::Invalid("missing native component"))?
            .ids
            .clone();
        if stage.frame.workers[d].generations.len()
            + stage.retired[d].len()
            + ids
                .iter()
                .filter(|id| !stage.retired[d].contains_key(id))
                .count()
            > MAX_BODY_IDS
        {
            return Err(Error::Capacity);
        }
        let generations = ids
            .iter()
            .map(|id| source.generations[id].checked_add(1).ok_or(Error::Capacity))
            .collect::<Result<Vec<_>, _>>()?;
        let serial = stage.serial.checked_add(1).ok_or(Error::Capacity)?;
        let ticket = WarmTicket {
            run_id: self.run,
            serial,
            tick: stage.frame.tick,
            source: owner,
            destination: d as u8,
            ids,
            generations,
        };
        let (selected, [remainder, dest]) =
            project_component(source, &stage.frame.workers[d], &ticket);
        self.validate_transfer_geometry(&dest, &stage.queued)?;
        let arriving: Vec<_> = stage
            .queued
            .iter()
            .filter(|p| ticket.ids.binary_search(&p.id).is_ok())
            .cloned()
            .collect();
        let mut incoming_slots = 0u32;
        let mut interleaved = false;
        for (slot, input) in stage
            .queued
            .iter()
            .filter(|p| dest.generations.contains_key(&p.id))
            .enumerate()
        {
            if slot >= MAX_PENDING_IMPULSES {
                return Err(Error::Capacity);
            }
            if ticket.ids.binary_search(&input.id).is_ok() {
                incoming_slots |= 1u32 << slot;
            } else if incoming_slots != 0 {
                interleaved = true;
            }
        }
        let Outcome::WarmExported(handoff) = self.exchange_handoff(
            stage,
            s,
            Command::ExportSolver {
                ticket: ticket.clone(),
            },
        )?
        else {
            return Err(Error::Invalid("expected pool solver export"));
        };
        if handoff.receipt.ticket != ticket
            || handoff.checkpoint.snapshot != selected
            || handoff.checkpoint.pending != arriving
            || handoff
                .validate_scope_in(self.peers.len() as u8, true)
                .is_err()
        {
            return Err(Error::Invalid("pool solver export disagrees with frame"));
        }
        let receipt = handoff.receipt.clone();
        let prepare = if interleaved {
            Command::PrepareOrdered {
                handoff,
                complete: true,
                incoming_slots,
            }
        } else {
            Command::PrepareSolver { handoff }
        };
        if self.exchange_handoff(stage, d, prepare)? != Outcome::WarmPrepared(receipt.prepared()) {
            return Err(Error::Invalid("pool prepare receipt"));
        }
        if self.exchange_handoff(
            stage,
            s,
            Command::ReleaseWarm {
                receipt: receipt.prepared(),
            },
        )? != Outcome::WarmReleased(receipt.released())
        {
            return Err(Error::Invalid("pool release receipt"));
        }
        if self.exchange_handoff(
            stage,
            d,
            Command::ActivateWarm {
                receipt: receipt.released(),
            },
        )? != Outcome::WarmActivated(dest.clone())
        {
            return Err(Error::Invalid("pool handoff changed native state"));
        }
        for (&id, &generation) in ticket.ids.iter().zip(&ticket.generations) {
            stage.retired[s].insert(id, generation);
            stage.retired[d].remove(&id);
        }
        stage.frame.workers[s] = remainder;
        stage.frame.workers[d] = dest;
        stage.serial = serial;
        stage.serials[s] = serial;
        stage.serials[d] = serial;
        Ok(s)
    }
}

impl WorkerPool {
    /// Move distinct native islands at the current tick and publish their ownership together.
    ///
    /// Each pair names any member and its destination. Order is canonical by the
    /// smallest body ID of each island. Duplicate islands, unknown IDs and current
    /// owners as destinations are invalid. An empty plan leaves the frame untouched.
    /// If a worker fails, subsequent mutations fence and replay affected endpoints
    /// from the last published barrier before using them again.
    pub fn transfer_components(&mut self, moves: &[(u64, u8)]) -> Result<&PoolFrame, Error> {
        if moves.is_empty() {
            return Ok(&self.frame);
        }
        let count: usize = self.frame.workers.iter().map(|w| w.components.len()).sum();
        if moves.len() > count {
            return Err(Error::Invalid(
                "more requested moves than native components",
            ));
        }
        let mut plan = BTreeMap::new();
        let mut sources = BTreeSet::new();
        let mut counts = vec![0; self.peers.len()];
        for &(member, destination) in moves {
            let (owner, _) = self
                .frame
                .body(member)
                .ok_or(Error::Invalid("unknown pool member"))?;
            let source = usize::from(owner);
            let dest = usize::from(destination);
            if source == dest || dest >= counts.len() {
                return Err(Error::Invalid("invalid pool destination"));
            }
            let component = self.frame.workers[source]
                .components
                .iter()
                .find(|c| c.ids.binary_search(&member).is_ok())
                .ok_or(Error::Invalid("missing native component"))?;
            if plan.insert(component.ids[0], dest).is_some() {
                return Err(Error::Invalid("duplicate component in handoff plan"));
            }
            counts[source] += 2;
            counts[dest] += 2;
            if sources.insert(source) {
                counts[source] += 1;
            }
        }
        self.reserve(&counts)?;
        let mut stage = self.stage()?;
        for (member, destination) in plan {
            self.stage_transfer(&mut stage, member, destination)?;
        }
        for source in sources {
            self.verify_source(&mut stage, source)?;
        }
        self.commit(stage)
    }
}
