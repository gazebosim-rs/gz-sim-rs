// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local worker ownership protocol, driven by one trusted coordinator. The
//! coordinator waits for every step acknowledgement and completes every transfer
//! before issuing the next group step. Each worker accepts one ordered request
//! at a time and caches its last successful response for loss/retry recovery.
//! Rebuild a failed worker from its seed or native baseline and committed suffix.

use crate::physics::{
    Body, ComponentInfo, Island, NativeSeedLayout, NativeSolverProfile, Partition, Snapshot,
    SolveSchedule, WorldError,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

// A worker can host several native solvers. This bounds its live IDs and tombstones.
// Encoded observations and warm transfers each have a separate wire byte ceiling.
pub(crate) const MAX_BODY_IDS: usize = crate::physics::MAX_PARTITION_BODIES;

mod image;
pub use image::WorkerImage;
mod warm;
pub use warm::{WarmHandoff, WarmReceipt, WarmTicket};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Request {
    pub run_id: u64,
    pub worker_id: u8,
    pub sequence: u64,
    pub command: Command,
}

impl PartialEq for Request {
    fn eq(&self, other: &Self) -> bool {
        if self.run_id != other.run_id
            || self.worker_id != other.worker_id
            || self.sequence != other.sequence
        {
            return false;
        }
        match (&self.command, &other.command) {
            (
                Command::QueueImpulse { tick, id, impulse },
                Command::QueueImpulse {
                    tick: t,
                    id: i,
                    impulse: v,
                },
            ) => tick == t && id == i && impulse.map(f64::to_bits) == v.map(f64::to_bits),
            (a, b) => a == b,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Reply {
    pub run_id: u64,
    pub worker_id: u8,
    pub sequence: u64,
    pub outcome: Outcome,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum Command {
    Observe,
    QueueImpulse {
        tick: u64,
        id: u64,
        impulse: [f64; 3],
    },
    ExportWarm {
        ticket: WarmTicket,
    },
    PrepareWarm {
        handoff: Box<WarmHandoff>,
    },
    ReleaseWarm {
        receipt: WarmReceipt,
    },
    ActivateWarm {
        receipt: WarmReceipt,
    },
    /// Require this current committed tick, then advance once.
    Step {
        tick: u64,
    },
    Export {
        ticket: Ticket,
    },
    Prepare {
        handoff: Handoff,
    },
    Release {
        prepared: Prepared,
    },
    Activate {
        released: Released,
    },
    SetVelocity {
        tick: u64,
        edit: VelocityEdit,
    },
    RetireBody {
        tick: u64,
        id: u64,
        owner: u8,
        generation: u64,
    },
    StepWithWrenches {
        tick: u64,
        wrenches: Vec<crate::physics::BodyWrench>,
    },
    ImpulseNow {
        tick: u64,
        impulse: SpatialImpulse,
    },
    ExportSolver {
        ticket: WarmTicket,
    },
    PrepareSolver {
        handoff: Box<WarmHandoff>,
    },
    SplitComponent {
        tick: u64,
        member: u64,
    },
    StepInField {
        tick: u64,
        field: crate::physics::StepField,
    },
    InsertBody {
        tick: u64,
        body: Body,
    },
    ReplaceBody {
        tick: u64,
        body: Body,
    },
    /// Same native receipt plus the stable merge of resident and arriving input FIFOs.
    PrepareOrdered {
        handoff: Box<WarmHandoff>,
        complete: bool,
        incoming_slots: u32,
    },
    /// Advance one 10 ms public barrier using ten native 1 ms solves.
    StepRefined {
        tick: u64,
        field: crate::physics::StepField,
    },
    /// Advance only the declared native islands in ten 1 ms solves. Every other
    /// island runs one 10 ms solve. IDs describe complete islands after routing.
    StepIslands {
        tick: u64,
        field: crate::physics::StepField,
        refined: Vec<u64>,
    },
}

/// A world linear impulse (N s) at the COM and angular impulse (N m s).
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SpatialImpulse {
    pub id: u64,
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}
impl PartialEq for SpatialImpulse {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
            && self.linear.map(f64::to_bits) == other.linear.map(f64::to_bits)
            && self.angular.map(f64::to_bits) == other.angular.map(f64::to_bits)
    }
}

/// A same-tick world-frame twist replacement for one centered rigid box.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VelocityEdit {
    pub id: u64,
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}
impl PartialEq for VelocityEdit {
    fn eq(&self, b: &Self) -> bool {
        self.id == b.id
            && self.linear.map(f64::to_bits) == b.linear.map(f64::to_bits)
            && self.angular.map(f64::to_bits) == b.angular.map(f64::to_bits)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum Outcome {
    State(Observation),
    Exported(Handoff),
    Prepared(Prepared),
    Released(Released),
    Activated(Observation),
    WarmExported(Box<WarmHandoff>),
    WarmPrepared(WarmReceipt),
    WarmReleased(WarmReceipt),
    WarmActivated(Observation),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Observation {
    pub snapshot: Snapshot,
    /// Native solver memberships sorted by their first global body ID.
    pub components: Vec<ComponentInfo>,
    /// Only authoritative bodies appear in this map and in the snapshot.
    pub generations: BTreeMap<u64, u64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ticket {
    pub run_id: u64,
    /// A new serial strictly exceeds the last accepted transfer on either worker.
    pub serial: u64,
    pub source: u8,
    pub destination: u8,
    pub body_id: u64,
    /// Next owner generation. Bootstrap owners begin at zero.
    pub generation: u64,
    pub tick: u64,
    /// Preserve the source snapshot and activate its dynamic representation.
    pub promote: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Handoff {
    pub ticket: Ticket,
    /// The exact source representation, before any requested promotion.
    pub body: Body,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Prepared {
    pub handoff: Handoff,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Released {
    pub handoff: Handoff,
}

#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum ProtocolError {
    #[error("request or transfer addresses the wrong run or worker")]
    Address,
    #[error("expected request sequence {expected}, received {received}")]
    Sequence { expected: u64, received: u64 },
    #[error("request sequence was reused with a conflicting command")]
    ConflictingRetry,
    #[error("expected committed tick {expected}, received {received}")]
    Tick { expected: u64, received: u64 },
    #[error("worker already has a pending transfer")]
    Pending,
    #[error("invalid or stale ownership ticket")]
    Ticket,
    #[error("body ownership or next generation does not match")]
    Ownership,
    #[error("transfer phase or payload disagrees with the pending handoff")]
    Receipt,
    #[error("worker body-ID capacity exceeded")]
    Capacity,
    #[error("protocol counter exhausted")]
    Exhausted,
    #[error("physics admission refused: {0}")]
    Physics(String),
    #[error("worker physics failed; recover from the committed log: {0}")]
    Terminal(String),
}

enum Pending {
    Frozen(Handoff),
    Staged {
        handoff: Handoff,
        prior: Box<Observation>,
    },
    WarmFrozen {
        receipt: WarmReceipt,
        complete: bool,
    },
    WarmStaged {
        receipt: WarmReceipt,
        restored: Box<Island>,
        order: Option<Vec<crate::physics::PendingImpulse>>,
    },
}

pub struct Worker {
    width: u8,
    run_id: u64,
    worker_id: u8,
    partition: Partition,
    tick: u64,
    generations: BTreeMap<u64, u64>,
    // A former owner remembers the generation to which it released this body.
    // Returning from the peer must advance again, even after a delayed packet.
    retired: BTreeMap<u64, u64>,
    serial: u64,
    pending: Option<Pending>,
    baseline_sequence: u64,
    last: Option<(Request, Reply)>,
    terminal: Option<String>,
}

impl Worker {
    pub fn new(run_id: u64, worker_id: u8, initial: Snapshot) -> Result<Self, ProtocolError> {
        Self::with_width(run_id, worker_id, initial, 2)
    }

    pub(crate) fn with_width(
        run_id: u64,
        worker_id: u8,
        initial: Snapshot,
        width: u8,
    ) -> Result<Self, ProtocolError> {
        Self::with_seed_layout(
            run_id,
            worker_id,
            initial,
            width,
            NativeSeedLayout::Standard32,
        )
    }
    pub(crate) fn with_seed_layout(
        run_id: u64,
        worker_id: u8,
        initial: Snapshot,
        width: u8,
        layout: NativeSeedLayout,
    ) -> Result<Self, ProtocolError> {
        let profile = layout.profile();
        if !(2..=8).contains(&width) || worker_id >= width || (width == 2 && !profile.is_standard())
        {
            return Err(ProtocolError::Address);
        }
        if initial.bodies.len() > MAX_BODY_IDS {
            return Err(ProtocolError::Capacity);
        }
        let partition = Partition::restore_with_layout(&initial, layout)
            .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        let generations = initial.bodies.iter().map(|b| (b.id, 0)).collect();
        Ok(Self {
            width,
            run_id,
            worker_id,
            partition,
            tick: initial.tick,
            generations,
            retired: BTreeMap::new(),
            serial: 0,
            pending: None,
            baseline_sequence: 0,
            last: None,
            terminal: None,
        })
    }

    pub(crate) fn native_solver_profile(&self) -> NativeSolverProfile {
        self.partition.profile
    }

    fn check_read_epoch(&self, sequence: u64, tick: u64) -> Result<(), ProtocolError> {
        self.check_tick(tick)?;
        if let Some(error) = &self.terminal {
            return Err(ProtocolError::Terminal(error.clone()));
        }
        if self.pending.is_some() {
            return Err(ProtocolError::Pending);
        }
        let current = self
            .last
            .as_ref()
            .map_or(self.baseline_sequence, |(r, _)| r.sequence);
        if sequence != current {
            return Err(ProtocolError::Sequence {
                expected: current,
                received: sequence,
            });
        }
        Ok(())
    }

    /// The resident solver retains its precise world-space inverse inertia.
    /// Reading its prospective twist consumes neither time nor a sequence.
    pub(crate) fn preview_impulse(
        &self,
        sequence: u64,
        tick: u64,
        j: &SpatialImpulse,
    ) -> Result<VelocityEdit, ProtocolError> {
        self.check_read_epoch(sequence, tick)?;
        let (linear, angular) = self
            .partition
            .preview_impulse(j.id, j.linear, j.angular)
            .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        Ok(VelocityEdit {
            id: j.id,
            linear,
            angular,
        })
    }

    pub(crate) fn set_local_helper(&mut self, enabled: bool) {
        self.partition.helper_allowed = enabled;
    }

    pub fn handle(&mut self, request: Request) -> Result<Reply, ProtocolError> {
        if request.run_id != self.run_id || request.worker_id != self.worker_id {
            return Err(ProtocolError::Address);
        }
        if let Some(error) = &self.terminal {
            return Err(ProtocolError::Terminal(error.clone()));
        }
        let previous = self
            .last
            .as_ref()
            .map_or(self.baseline_sequence, |(request, _)| request.sequence);
        if let Some((cached, reply)) = &self.last
            && request.sequence == previous
        {
            return if &request == cached {
                Ok(reply.clone())
            } else {
                Err(ProtocolError::ConflictingRetry)
            };
        }
        let expected = previous.checked_add(1).ok_or(ProtocolError::Exhausted)?;
        if request.sequence != expected {
            return Err(ProtocolError::Sequence {
                expected,
                received: request.sequence,
            });
        }
        let outcome = match &request.command {
            Command::Observe => Outcome::State(self.observation()),
            Command::QueueImpulse { tick, id, impulse } => {
                self.queue_impulse(*tick, *id, *impulse)?
            }
            Command::ExportWarm { ticket } => self.export_warm(ticket, false)?,
            Command::PrepareWarm { handoff } => self.prepare_warm(handoff, false, None)?,
            Command::ExportSolver { ticket } => self.export_warm(ticket, true)?,
            Command::PrepareSolver { handoff } => self.prepare_warm(handoff, true, None)?,
            Command::PrepareOrdered {
                handoff,
                complete,
                incoming_slots,
            } => self.prepare_warm(handoff, *complete, Some(*incoming_slots))?,
            Command::ReleaseWarm { receipt } => self.release_warm(receipt)?,
            Command::ActivateWarm { receipt } => self.activate_warm(receipt)?,
            Command::Step { tick } => self.step(*tick, [0.; 3], &[], SolveSchedule::Uniform(1))?,
            Command::StepWithWrenches { tick, wrenches } => {
                self.step(*tick, [0.; 3], wrenches, SolveSchedule::Uniform(1))?
            }
            Command::StepInField { tick, field } => self.step(
                *tick,
                field.gravity,
                &field.wrenches,
                SolveSchedule::Uniform(1),
            )?,
            Command::StepRefined { tick, field } => self.step(
                *tick,
                field.gravity,
                &field.wrenches,
                SolveSchedule::Uniform(10),
            )?,
            Command::StepIslands {
                tick,
                field,
                refined,
            } => self.step(
                *tick,
                field.gravity,
                &field.wrenches,
                SolveSchedule::Islands(refined),
            )?,
            Command::ReplaceBody { tick, body } => {
                self.check_body_edit(*tick, body.id)?;
                self.partition
                    .replace_state(body)
                    .map_err(|e| ProtocolError::Physics(e.to_string()))?;
                Outcome::State(self.observation())
            }
            Command::InsertBody { tick, body } => {
                self.check_unfrozen_tick(*tick)?;
                if self.generations.contains_key(&body.id) || self.retired.contains_key(&body.id) {
                    return Err(ProtocolError::Ownership);
                }
                if self.generations.len() + self.retired.len() >= MAX_BODY_IDS {
                    return Err(ProtocolError::Capacity);
                }
                self.partition
                    .import(body.clone())
                    .map_err(|e| ProtocolError::Physics(e.to_string()))?;
                self.generations.insert(body.id, 0);
                Outcome::State(self.observation())
            }
            Command::SetVelocity { tick, edit } => {
                self.check_body_edit(*tick, edit.id)?;
                self.partition
                    .set_velocity(edit.id, edit.linear, edit.angular)
                    .map_err(|e| ProtocolError::Physics(e.to_string()))?;
                Outcome::State(self.observation())
            }
            Command::ImpulseNow { tick, impulse } => {
                self.check_unfrozen_tick(*tick)?;
                self.partition
                    .impulse_now(impulse.id, impulse.linear, impulse.angular)
                    .map_err(|e| ProtocolError::Physics(e.to_string()))?;
                Outcome::State(self.observation())
            }
            Command::SplitComponent { tick, member } => {
                self.check_unfrozen_tick(*tick)?;
                self.partition
                    .split_component(*member)
                    .map_err(|e| ProtocolError::Physics(e.to_string()))?;
                Outcome::State(self.observation())
            }
            Command::RetireBody {
                tick,
                id,
                owner,
                generation,
            } => {
                self.check_unfrozen_tick(*tick)?;
                if *owner >= self.width {
                    return Err(ProtocolError::Address);
                }
                if *owner == self.worker_id {
                    if self.generations.get(id) != Some(generation) {
                        return Err(ProtocolError::Ownership);
                    }
                    self.partition
                        .retire(*id)
                        .map_err(|e| ProtocolError::Physics(e.to_string()))?;
                    self.generations.remove(id);
                } else {
                    if self.generations.contains_key(id)
                        || self.retired.get(id).copied().unwrap_or(0) != *generation
                    {
                        return Err(ProtocolError::Ownership);
                    }
                    self.retired.remove(id);
                }
                Outcome::State(self.observation())
            }
            Command::Export { ticket } => self.export(*ticket)?,
            Command::Prepare { handoff } => self.prepare(handoff)?,
            Command::Release { prepared } => self.release(prepared)?,
            Command::Activate { released } => self.activate(released)?,
        };
        let reply = Reply {
            run_id: self.run_id,
            worker_id: self.worker_id,
            sequence: request.sequence,
            outcome,
        };
        self.last = Some((request, reply.clone()));
        Ok(reply)
    }

    pub(crate) fn start_probe(&mut self, tick: u64, span: u16) -> Result<(), ProtocolError> {
        self.probe_admission(tick)?;
        self.partition
            .start_probe(tick, span)
            .map_err(|e| ProtocolError::Physics(e.to_string()))
    }

    fn probe_admission(&self, tick: u64) -> Result<(), ProtocolError> {
        self.check_unfrozen_tick(tick)?;
        if let Some(reason) = &self.terminal {
            return Err(ProtocolError::Terminal(reason.clone()));
        }
        Ok(())
    }

    pub(crate) fn start_scheduled_probe(
        &mut self,
        tick: u64,
        span: u16,
    ) -> Result<(), ProtocolError> {
        self.probe_admission(tick)?;
        self.partition
            .start_probe_in_mode(tick, span, true)
            .map_err(|e| ProtocolError::Physics(e.to_string()))
    }

    pub(crate) fn read_probe(
        &self,
        start: u64,
        span: u16,
    ) -> Option<Vec<crate::physics::ComponentWork>> {
        if self.pending.is_some() || self.terminal.is_some() {
            return None;
        }
        self.partition.read_probe(start, span)
    }

    pub(crate) fn observation(&self) -> Observation {
        // Cold preparation can merge free bodies into a new native solver.
        // Publish the prior topology and all committed samples until activation.
        if let Some(Pending::Staged { prior, .. }) = &self.pending {
            return (**prior).clone();
        }
        Observation {
            snapshot: self.partition.snapshot(),
            components: self.partition.topology(),
            generations: self.generations.clone(),
        }
    }

    fn check_body_edit(&self, tick: u64, id: u64) -> Result<(), ProtocolError> {
        self.check_unfrozen_tick(tick)?;
        if self.width > 2 {
            self.partition
                .admit_free_edit(id)
                .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        }
        Ok(())
    }

    fn check_unfrozen_tick(&self, tick: u64) -> Result<(), ProtocolError> {
        self.check_tick(tick)?;
        if self.pending.is_some() {
            return Err(ProtocolError::Pending);
        }
        Ok(())
    }

    fn check_tick(&self, tick: u64) -> Result<(), ProtocolError> {
        if self.tick != tick {
            return Err(ProtocolError::Tick {
                expected: self.tick,
                received: tick,
            });
        }
        Ok(())
    }

    fn step(
        &mut self,
        tick: u64,
        gravity: [f64; 3],
        wrenches: &[crate::physics::BodyWrench],
        schedule: SolveSchedule<'_>,
    ) -> Result<Outcome, ProtocolError> {
        self.check_unfrozen_tick(tick)?;
        let next = self.tick.checked_add(1).ok_or(ProtocolError::Exhausted)?;
        // The partition preflights all prospective solver merges before it mutates
        // any resident. A failed native commit is explicitly reported as Fault.
        self.partition
            .step_scheduled(gravity, wrenches, schedule)
            .map_err(|error| match error {
                WorldError::Fault(reason) => {
                    self.terminal = Some(reason.clone());
                    ProtocolError::Terminal(reason)
                }
                admission => ProtocolError::Physics(admission.to_string()),
            })?;
        self.tick = next;
        Ok(Outcome::State(self.observation()))
    }

    fn check_ticket(&self, t: Ticket, role: u8) -> Result<(), ProtocolError> {
        // Historical cold receipts contain no pool scope. Wider worlds always
        // use a width-bound native checkpoint, including for free singletons.
        if self.width != 2 {
            return Err(ProtocolError::Address);
        }
        self.check_phase(t.run_id, t.serial, t.source, t.destination, t.tick, role)
    }

    fn check_phase(
        &self,
        run: u64,
        serial: u64,
        source: u8,
        destination: u8,
        tick: u64,
        role: u8,
    ) -> Result<(), ProtocolError> {
        if run != self.run_id
            || source >= self.width
            || destination >= self.width
            || source == destination
            || self.worker_id != role
        {
            return Err(ProtocolError::Address);
        }
        self.check_unfrozen_tick(tick)?;
        if serial <= self.serial {
            return Err(ProtocolError::Ticket);
        }
        Ok(())
    }

    // In a pair, every intermediate owner is observed. A pool worker may
    // miss generations on other shards. The authenticated source checks the
    // exact successor, and each destination independently enforces its fence.
    fn accepts_generation(&self, id: u64, next: u64) -> bool {
        let known = self.retired.get(&id).copied().unwrap_or(0);
        if self.width == 2 {
            known.checked_add(1) == Some(next)
        } else {
            next > known
        }
    }

    fn export(&mut self, ticket: Ticket) -> Result<Outcome, ProtocolError> {
        self.check_ticket(ticket, ticket.source)?;
        let current = self
            .generations
            .get(&ticket.body_id)
            .ok_or(ProtocolError::Ownership)?;
        if current.checked_add(1) != Some(ticket.generation) {
            return Err(ProtocolError::Ownership);
        }
        let body = self
            .partition
            .export(ticket.body_id)
            .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        if ticket.promote && body.tier == crate::physics::Tier::Fixed {
            return Err(ProtocolError::Ticket);
        }
        let handoff = Handoff { ticket, body };
        self.pending = Some(Pending::Frozen(handoff.clone()));
        self.serial = ticket.serial;
        Ok(Outcome::Exported(handoff))
    }

    fn prepare(&mut self, handoff: &Handoff) -> Result<Outcome, ProtocolError> {
        let t = handoff.ticket;
        self.check_ticket(t, t.destination)?;
        if t.promote && handoff.body.tier == crate::physics::Tier::Fixed {
            return Err(ProtocolError::Ticket);
        }
        if t.body_id != handoff.body.id
            || self.generations.contains_key(&t.body_id)
            || !self.accepts_generation(t.body_id, t.generation)
        {
            return Err(ProtocolError::Ownership);
        }
        if !self.retired.contains_key(&t.body_id)
            && self.generations.len() + self.retired.len() >= MAX_BODY_IDS
        {
            return Err(ProtocolError::Capacity);
        }
        let body = if t.promote {
            handoff.body.promoted()
        } else {
            handoff.body.clone()
        };
        let prior = Box::new(self.observation());
        self.partition
            .import(body)
            .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        self.pending = Some(Pending::Staged {
            handoff: handoff.clone(),
            prior,
        });
        self.serial = t.serial;
        Ok(Outcome::Prepared(Prepared {
            handoff: handoff.clone(),
        }))
    }

    fn release(&mut self, prepared: &Prepared) -> Result<Outcome, ProtocolError> {
        let Some(Pending::Frozen(frozen)) = &self.pending else {
            return Err(ProtocolError::Receipt);
        };
        if frozen != &prepared.handoff {
            return Err(ProtocolError::Receipt);
        }
        let t = frozen.ticket;
        self.partition
            .remove(t.body_id)
            .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        self.generations.remove(&t.body_id);
        self.retired.insert(t.body_id, t.generation);
        self.pending = None;
        Ok(Outcome::Released(Released {
            handoff: prepared.handoff.clone(),
        }))
    }

    fn activate(&mut self, released: &Released) -> Result<Outcome, ProtocolError> {
        let Some(Pending::Staged {
            handoff: staged, ..
        }) = &self.pending
        else {
            return Err(ProtocolError::Receipt);
        };
        if staged != &released.handoff {
            return Err(ProtocolError::Receipt);
        }
        let t = staged.ticket;
        self.retired.remove(&t.body_id);
        self.generations.insert(t.body_id, t.generation);
        self.pending = None;
        Ok(Outcome::Activated(self.observation()))
    }
}

#[cfg(test)]
#[path = "../tests/private/protocol.rs"]
mod tests;
