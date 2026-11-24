// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Per-barrier recording and deterministic replay of optional whole-island placement.
use super::*;
use serde::{Deserialize, Serialize};

/// The optional placement published with one island-resolution step. Required
/// collision ownership is always recomputed from the actual field and inputs.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlacementRecord {
    /// Committed clock after the transfer and solve publish together.
    pub target_tick: u64,
    /// None explicitly suppresses an optional handoff for this barrier.
    // An omitted JSON field must not silently turn a damaged record into "stay".
    #[serde(deserialize_with = "Deserialize::deserialize")]
    pub optional_move: Option<PlacementMove>,
}
/// A whole native solver in its exact pre-transfer ownership incarnation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlacementMove {
    pub source: u8,
    pub destination: u8,
    /// All current members in ascending public ID order.
    #[serde(deserialize_with = "decode_members")]
    pub members: Vec<PlacementMember>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlacementMember {
    pub id: u64,
    pub generation: u64,
}

fn decode_members<'de, D: serde::Deserializer<'de>>(
    d: D,
) -> Result<Vec<PlacementMember>, D::Error> {
    use serde::de::{Error, SeqAccess, Visitor};
    struct Members;
    impl<'de> Visitor<'de> for Members {
        type Value = Vec<PlacementMember>;
        fn expecting(&self, fmt: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            fmt.write_str("one complete native solver in strictly increasing body-ID order")
        }
        fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Self::Value, A::Error> {
            let cap = NativeSolverProfile::Extended64.max_bodies();
            if seq.size_hint().is_some_and(|n| n == 0 || n > cap) {
                return Err(A::Error::custom("invalid placement member count"));
            }
            let mut members = Vec::new();
            while let Some(member) = seq.next_element::<PlacementMember>()? {
                if members.len() == cap
                    || members
                        .last()
                        .is_some_and(|p: &PlacementMember| p.id >= member.id)
                {
                    return Err(A::Error::custom(
                        "oversized or unordered placement membership",
                    ));
                }
                members.push(member);
            }
            if members.is_empty() {
                return Err(A::Error::invalid_length(0, &self));
            }
            Ok(members)
        }
    }
    d.deserialize_seq(Members)
}

/// Only present while executing or retrying an already staged replay step.
/// It cannot become an input queued for a future published barrier.
pub(super) struct ReplayAttempt {
    pub(super) record: PlacementRecord,
    field: Vec<u8>,
}
#[derive(Default)]
pub(super) struct PlacementReplay {
    pub(super) committed: Option<PlacementRecord>,
    pub(super) attempt: Option<Box<ReplayAttempt>>,
}
impl WorkerPool {
    /// Receipt for the latest committed island step in this coordinator.
    /// A fresh or restored coordinator and other step policies return None.
    pub fn last_island_placement(&self) -> Option<&PlacementRecord> {
        self.placement_replay.committed.as_ref()
    }
    /// Replay exactly this barrier's optional ownership choice. Required contact
    /// routing and island resolution still use the actual gravity, queued inputs
    /// and ordered wrenches. No timing query participates in the decision.
    pub fn advance_islands_with_placement(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
        record: &PlacementRecord,
    ) -> Result<&PoolFrame, Error> {
        self.advance_placed(
            gravity,
            inputs,
            Resolution::Islands(Vec::new()),
            Some(record),
        )
    }
    /// Abandon a failed replay step. All speculative endpoints must return to
    /// the published journals before other commands become admissible.
    pub fn rollback_interrupted_placement(&mut self) -> Result<bool, Error> {
        if self.placement_replay.attempt.is_none() {
            return Ok(false);
        }
        self.pending_step = None;
        for w in 0..self.peers.len() {
            if self.dirty[w] {
                self.recover(w as u8)?;
            }
        }
        self.placement_replay.attempt = None;
        Ok(true)
    }
    pub(super) fn check_attempt(
        &self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
        record: Option<&PlacementRecord>,
    ) -> Result<(), Error> {
        if let Some(attempt) = &self.placement_replay.attempt {
            if record != Some(&attempt.record)
                || attempt.field != retry::input_bytes(gravity, inputs)?
            {
                return Err(Error::Pending);
            }
        // An ambiguous timing-selected step must first be completed with its own policy.
        } else if self.pending_step.is_some() && record.is_some() {
            return Err(Error::Pending);
        }
        Ok(())
    }
    pub(super) fn begin_replay_attempt(
        &mut self,
        record: &PlacementRecord,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<(), Error> {
        if self.placement_replay.attempt.is_none() {
            self.placement_replay.attempt = Some(Box::new(ReplayAttempt {
                record: record.clone(),
                field: retry::input_bytes(gravity, inputs)?,
            }));
        }
        Ok(())
    }
    fn validate_replay_move(
        &self,
        record: &PlacementRecord,
        next: u64,
        mandatory: bool,
        plan: &routing::ProspectiveGroups,
    ) -> Result<Option<(u64, usize)>, Error> {
        if record.target_tick != next {
            return Err(Error::Invalid("placement record has the wrong target tick"));
        }
        let Some(m) = &record.optional_move else {
            return Ok(None);
        };
        if mandatory {
            return Err(Error::Colocation);
        }
        let source = usize::from(m.source);
        let dst = usize::from(m.destination);
        if source >= self.peers.len() || dst >= self.peers.len() || source == dst {
            return Err(Error::Invalid("invalid recorded placement endpoints"));
        }
        let first = m
            .members
            .first()
            .ok_or(Error::Invalid("recorded placement has no members"))?
            .id;
        let i = plan
            .components
            .iter()
            .position(|c| c.ids.first() == Some(&first))
            .ok_or(Error::Invalid("recorded native membership has changed"))?;
        let ids = &plan.components[i].ids;
        if plan.owners[i] != source
            || ids.len() != m.members.len()
            || ids.iter().zip(&m.members).any(|(id, member)| {
                *id != member.id
                    || self.frame.workers[source].generations.get(id) != Some(&member.generation)
            })
        {
            return Err(Error::Invalid(
                "recorded member ownership or generation has changed",
            ));
        }
        if !plan.groups.iter().any(|g| g.as_slice() == [i]) {
            return Err(Error::Colocation);
        }
        if !self.placement_destination_fits(ids, dst) {
            return Err(Error::Capacity);
        }
        Ok(Some((first, dst)))
    }

    /// Simulate the native admission of all mandatory or recorded arrivals in
    /// order, including the receiver's neutral-field geometry checks. This
    /// runs only on replay barriers containing a transfer.
    pub(super) fn validate_replay_routes(
        &self,
        record: &PlacementRecord,
        next: u64,
        moves: &mut Vec<(u64, usize)>,
        plan: &routing::ProspectiveGroups,
    ) -> Result<(), Error> {
        if let Some(m) = self.validate_replay_move(record, next, !moves.is_empty(), plan)? {
            moves.push(m);
        }
        if moves.is_empty() {
            return Ok(());
        }
        let mut projected = self.frame.clone();
        let mut serial = self.serial;
        for &(member, destination) in moves.iter() {
            let (source, _) = projected
                .body(member)
                .ok_or(Error::Invalid("missing routed member"))?;
            let owner = &projected.workers[usize::from(source)];
            let component = owner
                .components
                .iter()
                .find(|c| c.ids.binary_search(&member).is_ok())
                .ok_or(Error::Invalid("missing routed native solver"))?;
            serial = serial.checked_add(1).ok_or(Error::Capacity)?;
            let ticket = WarmTicket {
                run_id: self.run,
                serial,
                tick: projected.tick,
                source,
                destination: destination as u8,
                ids: component.ids.clone(),
                generations: component
                    .ids
                    .iter()
                    .map(|id| owner.generations[id].checked_add(1).ok_or(Error::Capacity))
                    .collect::<Result<_, _>>()?,
            };
            let (_, [remaining, incoming]) =
                project_component(owner, &projected.workers[destination], &ticket);
            self.validate_transfer_geometry(&incoming, &self.queued)?;
            projected.workers[usize::from(source)] = remaining;
            projected.workers[destination] = incoming;
        }
        Ok(())
    }

    /// Validate against the published barrier, current queued impulses and this
    /// ordered field without consuming a timing sample or issuing any request.
    pub fn validate_island_placement(
        &self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
        record: &PlacementRecord,
    ) -> Result<(), Error> {
        self.check_attempt(gravity, inputs, Some(record))?;
        let next = self.frame.tick.checked_add(1).ok_or(Error::Capacity)?;
        let plan = routing::ProspectiveGroups::new(
            &self.frame,
            gravity,
            &self.queued,
            inputs,
            self.profile,
        )?;
        let (mut moves, _) = self.route_plan(&self.frame, &self.queued, true, &plan)?;
        self.validate_replay_routes(record, next, &mut moves, &plan)?;
        self.validate_step_counts(&self.command_counts(&moves))
    }
    pub(super) fn placement_destination_fits(&self, ids: &[u64], dst: usize) -> bool {
        let target = &self.frame.workers[dst];
        let fresh = ids
            .iter()
            .filter(|id| !self.retired[dst].contains_key(id))
            .count();
        target.generations.len() + self.retired[dst].len() + fresh <= MAX_BODY_IDS
            && self
                .queued
                .iter()
                .filter(|p| {
                    target.generations.contains_key(&p.id) || ids.binary_search(&p.id).is_ok()
                })
                .count()
                <= MAX_PENDING_IMPULSES
    }
    pub(super) fn measured_placement_record(
        &self,
        next: u64,
        decision: Option<&PoolBalanceDecision>,
    ) -> PlacementRecord {
        PlacementRecord {
            target_tick: next,
            optional_move: decision.map(|m| PlacementMove {
                source: m.source,
                destination: m.destination,
                members: m
                    .members
                    .iter()
                    .map(|&id| PlacementMember {
                        id,
                        generation: self.frame.workers[m.source as usize].generations[&id],
                    })
                    .collect(),
            }),
        }
    }
}
