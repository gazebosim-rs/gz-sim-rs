// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same-build transfer of a complete dynamic solver component between workers.
use super::{MAX_BODY_IDS, Outcome, Pending, ProtocolError, Worker};
use crate::physics::{Island, Tier, WarmCheckpoint};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::io::{self, BufWriter, Write};

// Leave room for the command/reply fields around the hashed ticket and checkpoint.
// This streaming check happens before source freeze.
const MAX_WARM_ENCODING: usize = crate::transport::MAX_CHECKPOINT_PAYLOAD - 16 * 1024;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WarmTicket {
    pub run_id: u64,
    pub serial: u64,
    pub source: u8,
    pub destination: u8,
    pub tick: u64,
    /// Strictly increasing IDs of every transferred member.
    pub ids: Vec<u64>,
    /// Next owner generation, aligned with each member in ids.
    pub generations: Vec<u64>,
}

/// Identifies one checkpoint and transaction. Its digest binds the bytes
/// forwarded by the trusted coordinator; peer authentication is external.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WarmReceipt {
    pub ticket: WarmTicket,
    pub digest: [u8; 32],
    phase: WarmPhase,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum WarmPhase {
    Exported,
    Prepared,
    Released,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WarmHandoff {
    pub receipt: WarmReceipt,
    pub checkpoint: WarmCheckpoint,
}

struct DigestWriter {
    hash: Sha256,
    remaining: usize,
}

impl Write for DigestWriter {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.remaining = self.remaining.checked_sub(bytes.len()).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidData, "checkpoint exceeds wire limit")
        })?;
        self.hash.update(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl DigestWriter {
    fn write_json(&mut self, payload: &impl Serialize) -> Result<(), ProtocolError> {
        // A native byte array produces a separate JSON write for every integer and
        // comma. Hash complete chunks while retaining the exact legacy encoding.
        let mut buffered = BufWriter::with_capacity(8192, self);
        serde_json::to_writer(&mut buffered, payload).map_err(|_| ProtocolError::Capacity)?;
        buffered.flush().map_err(|_| ProtocolError::Capacity)
    }
}

impl WarmReceipt {
    pub fn prepared(&self) -> Self {
        Self {
            phase: WarmPhase::Prepared,
            ..self.clone()
        }
    }

    pub fn released(&self) -> Self {
        Self {
            phase: WarmPhase::Released,
            ..self.clone()
        }
    }

    fn is_acknowledged_by(&self, ack: &Self, phase: WarmPhase) -> bool {
        ack.phase == phase && self.ticket == ack.ticket && self.digest == ack.digest
    }

    fn new(
        ticket: &WarmTicket,
        checkpoint: &WarmCheckpoint,
        complete: bool,
        width: u8,
    ) -> Result<Self, ProtocolError> {
        if !(2..=8).contains(&width) {
            return Err(ProtocolError::Address);
        }
        let mut writer = DigestWriter {
            hash: Sha256::new(),
            remaining: MAX_WARM_ENCODING,
        };
        let domain: &[u8] = if complete {
            b"gz-physics-distributed/solver-handoff/v1\0"
        } else {
            b"gz-physics-distributed/warm-handoff/v1\0"
        };
        if width != 2 {
            writer
                .hash
                .update(b"gz-physics-distributed/pool/handoff/v1\0");
            writer.hash.update([width]);
        }
        writer.hash.update(domain);
        writer.write_json(&(ticket, checkpoint))?;
        Ok(Self {
            ticket: ticket.clone(),
            digest: writer.hash.finalize().into(),
            phase: WarmPhase::Exported,
        })
    }
}

impl WarmHandoff {
    /// Verify a historical pair handoff in its original digest domain.
    pub fn validate_digest(&self) -> Result<(), ProtocolError> {
        self.validate_digest_for_width(2)
    }

    /// Verify the handoff for its expected bootstrap width and scope.
    pub fn validate_digest_for_width(&self, width: u8) -> Result<(), ProtocolError> {
        match self.validate_scope_in(width, false) {
            Err(ProtocolError::Receipt) => self.validate_scope_in(width, true),
            result => result,
        }
    }
    pub(crate) fn validate_scope(&self, complete: bool) -> Result<(), ProtocolError> {
        self.validate_scope_in(2, complete)
    }
    pub(crate) fn validate_scope_in(&self, width: u8, complete: bool) -> Result<(), ProtocolError> {
        if WarmReceipt::new(&self.receipt.ticket, &self.checkpoint, complete, width)?
            != self.receipt
        {
            return Err(ProtocolError::Receipt);
        }
        Ok(())
    }
}

impl Worker {
    pub(super) fn queue_impulse(
        &mut self,
        tick: u64,
        id: u64,
        impulse: [f64; 3],
    ) -> Result<Outcome, ProtocolError> {
        self.check_unfrozen_tick(tick)?;
        let committed = self.observation();
        if !committed
            .snapshot
            .bodies
            .iter()
            .any(|b| b.id == id && b.tier == Tier::Dynamic)
        {
            return Err(ProtocolError::Ownership);
        }
        self.partition
            .queue_impulse(id, impulse)
            .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        Ok(Outcome::State(committed))
    }

    fn check_warm_ticket(
        &self,
        t: &WarmTicket,
        role: u8,
        complete: bool,
    ) -> Result<(), ProtocolError> {
        self.check_phase(t.run_id, t.serial, t.source, t.destination, t.tick, role)?;
        let minimum = if complete { 1 } else { 2 };
        if !(minimum..=self.partition.profile.max_bodies()).contains(&t.ids.len())
            || t.generations.len() != t.ids.len()
            || !t.ids.windows(2).all(|w| w[0] < w[1])
        {
            return Err(ProtocolError::Ticket);
        }
        Ok(())
    }

    pub(super) fn export_warm(
        &mut self,
        ticket: &WarmTicket,
        complete: bool,
    ) -> Result<Outcome, ProtocolError> {
        self.check_warm_ticket(ticket, ticket.source, complete)?;
        if ticket
            .ids
            .iter()
            .zip(&ticket.generations)
            .any(|(id, next)| {
                self.generations.get(id).and_then(|v| v.checked_add(1)) != Some(*next)
            })
        {
            return Err(ProtocolError::Ownership);
        }
        let checkpoint = if complete {
            self.partition.checkpoint_component(&ticket.ids)
        } else {
            self.partition.checkpoint_group(&ticket.ids)
        }
        .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        let receipt = WarmReceipt::new(ticket, &checkpoint, complete, self.width)?;
        self.pending = Some(Pending::WarmFrozen {
            receipt: receipt.clone(),
            complete,
        });
        self.serial = ticket.serial;
        Ok(Outcome::WarmExported(Box::new(WarmHandoff {
            receipt,
            checkpoint,
        })))
    }

    pub(super) fn prepare_warm(
        &mut self,
        handoff: &WarmHandoff,
        complete: bool,
        incoming_slots: Option<u32>,
    ) -> Result<Outcome, ProtocolError> {
        let receipt = &handoff.receipt;
        let t = &receipt.ticket;
        self.check_warm_ticket(t, t.destination, complete)?;
        if !handoff.checkpoint.matches_profile(self.partition.profile) {
            return Err(ProtocolError::Receipt);
        }
        let snapshot = &handoff.checkpoint.snapshot;
        if t.ids.iter().any(|id| self.generations.contains_key(id))
            || (!complete && snapshot.bodies.iter().any(|b| b.tier != Tier::Dynamic))
            || snapshot.tick != t.tick
            || snapshot
                .bodies
                .iter()
                .map(|b| b.id)
                .ne(t.ids.iter().copied())
            || t.ids
                .iter()
                .zip(&t.generations)
                .any(|(id, next)| !self.accepts_generation(*id, *next))
        {
            return Err(ProtocolError::Ownership);
        }
        let new_ids = t
            .ids
            .iter()
            .filter(|id| !self.retired.contains_key(id))
            .count();
        if self.generations.len() + self.retired.len() + new_ids > MAX_BODY_IDS {
            return Err(ProtocolError::Capacity);
        }
        handoff.validate_scope_in(self.width, complete)?;
        let restored = if complete {
            Island::restore_solver_with_profile(&handoff.checkpoint, self.partition.profile)
        } else {
            Island::restore_warm_with_profile(&handoff.checkpoint, self.partition.profile)
        }
        .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        if complete {
            self.partition.check_insert_component(&restored)
        } else {
            self.partition.check_insert(&restored)
        }
        .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        let order = incoming_slots
            .map(|mask| self.partition.ordered_arrival(&restored, mask))
            .transpose()
            .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        // Keep resident solvers and their inputs untouched until source release.
        self.pending = Some(Pending::WarmStaged {
            receipt: receipt.clone(),
            restored: Box::new(restored),
            order,
        });
        self.serial = t.serial;
        Ok(Outcome::WarmPrepared(receipt.prepared()))
    }

    pub(super) fn release_warm(&mut self, ack: &WarmReceipt) -> Result<Outcome, ProtocolError> {
        let Some(Pending::WarmFrozen { receipt, complete }) = &self.pending else {
            return Err(ProtocolError::Receipt);
        };
        if !receipt.is_acknowledged_by(ack, WarmPhase::Prepared) {
            return Err(ProtocolError::Receipt);
        }
        if *complete {
            self.partition.release_component(&ack.ticket.ids)
        } else {
            self.partition.release_group(&ack.ticket.ids)
        }
        .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        for (&id, &generation) in ack.ticket.ids.iter().zip(&ack.ticket.generations) {
            self.generations.remove(&id);
            self.retired.insert(id, generation);
        }
        self.pending = None;
        Ok(Outcome::WarmReleased(ack.released()))
    }

    pub(super) fn activate_warm(&mut self, ack: &WarmReceipt) -> Result<Outcome, ProtocolError> {
        let Some(Pending::WarmStaged { restored, order, .. }) = self.pending.take_if(
            |phase| matches!(phase, Pending::WarmStaged {receipt,..} if receipt.is_acknowledged_by(ack, WarmPhase::Released)),
        ) else {
            return Err(ProtocolError::Receipt);
        };
        self.partition.insert_ordered(*restored, order);
        for (&id, &generation) in ack.ticket.ids.iter().zip(&ack.ticket.generations) {
            self.retired.remove(&id);
            self.generations.insert(id, generation);
        }
        Ok(Outcome::WarmActivated(self.observation()))
    }
}

#[cfg(test)]
mod receipt_bytes_tests {
    use super::*;
    use crate::{
        physics::{PendingImpulse, Snapshot, fixture},
        protocol::{Command, Request},
    };

    #[test]
    fn buffered_hash_preserves_canonical_width_scope_native_values_and_pending_input_order() {
        let b = fixture()[1].clone();
        let mut worker = Worker::new(
            23,
            0,
            Snapshot {
                bodies: vec![b.clone()],
                ..Default::default()
            },
        )
        .unwrap();
        let ticket = WarmTicket {
            run_id: 23,
            serial: 1,
            source: 0,
            destination: 1,
            tick: 0,
            ids: vec![b.id],
            generations: vec![1],
        };
        let Outcome::WarmExported(mut handoff) = worker
            .handle(Request {
                run_id: 23,
                worker_id: 0,
                sequence: 1,
                command: Command::ExportSolver {
                    ticket: ticket.clone(),
                },
            })
            .unwrap()
            .outcome
        else {
            unreachable!()
        };
        handoff.checkpoint.pending = vec![
            PendingImpulse {
                id: b.id,
                impulse: [0.25, -0., 0.],
            },
            PendingImpulse {
                id: b.id,
                impulse: [-0., 0.1, -0.3],
            },
        ];
        for len in [0, 1, 255, 256, 8191, 8192, 8193, 32768] {
            let native = handoff.checkpoint.native_wire_payload();
            native.clear();
            native.extend((0..len).map(|i| i as u8));
            handoff.checkpoint.pending.reverse();
            let canonical = serde_json::to_vec(&(&ticket, &handoff.checkpoint)).unwrap();
            for width in [2, 3, 4, 8] {
                for complete in [false, true] {
                    let mut reference = Sha256::new();
                    if width != 2 {
                        reference.update(b"gz-physics-distributed/pool/handoff/v1\0");
                        reference.update([width]);
                    }
                    if complete {
                        reference.update(b"gz-physics-distributed/solver-handoff/v1\0")
                    } else {
                        reference.update(b"gz-physics-distributed/warm-handoff/v1\0")
                    }
                    reference.update(&canonical);
                    let expected: [u8; 32] = reference.finalize().into();
                    assert_eq!(
                        WarmReceipt::new(&ticket, &handoff.checkpoint, complete, width)
                            .unwrap()
                            .digest,
                        expected
                    );
                }
            }
        }
    }

    #[test]
    fn buffered_hash_enforces_the_exact_encoded_limit_including_the_final_flush() {
        for limit in [0, 1, 8191, 8192, 8193, MAX_WARM_ENCODING] {
            for content in [
                limit.saturating_sub(3),
                limit.saturating_sub(2),
                limit.saturating_sub(1),
            ] {
                let input = "x".repeat(content);
                let mut writer = DigestWriter {
                    hash: Sha256::new(),
                    remaining: limit,
                };
                let result = writer.write_json(&input);
                if content + 2 <= limit {
                    assert_eq!(result, Ok(()));
                    assert_eq!(writer.remaining, limit - content - 2);
                    let expected = Sha256::digest(serde_json::to_vec(&input).unwrap());
                    assert_eq!(writer.hash.finalize(), expected);
                } else {
                    assert_eq!(result, Err(ProtocolError::Capacity));
                }
            }
        }
    }
}

#[cfg(test)]
mod model_admission {
    use super::*;
    use crate::{
        physics::{NativeFrictionModel, NativeSeedLayout, NativeSolverProfile, Snapshot, fixture},
        protocol::{Command, Request},
    };

    #[test]
    fn genuine_warm_handoff_from_another_friction_law_cannot_stage_or_advance() {
        let request = |worker_id, command| Request {
            run_id: 71,
            worker_id,
            sequence: 1,
            command,
        };
        for capacity in [
            NativeSolverProfile::Standard32,
            NativeSolverProfile::Extended64,
        ] {
            let point = capacity.with_friction(NativeFrictionModel::PerPoint);
            for (from, to) in [(capacity, point), (point, capacity)] {
                let b = fixture()[0].promoted();
                let mut source = Worker::with_seed_layout(
                    71,
                    0,
                    Snapshot {
                        bodies: vec![b.clone()],
                        ..Default::default()
                    },
                    3,
                    NativeSeedLayout::from(from),
                )
                .unwrap();
                let mut target = Worker::with_seed_layout(
                    71,
                    1,
                    Snapshot::default(),
                    3,
                    NativeSeedLayout::from(to),
                )
                .unwrap();
                let prior = target.recovery_image(0, 0).unwrap();
                let ticket = WarmTicket {
                    run_id: 71,
                    serial: 1,
                    source: 0,
                    destination: 1,
                    tick: 0,
                    ids: vec![b.id],
                    generations: vec![1],
                };
                let Outcome::WarmExported(handoff) = source
                    .handle(request(0, Command::ExportSolver { ticket }))
                    .unwrap()
                    .outcome
                else {
                    panic!("expected native export")
                };
                assert_eq!(
                    target.handle(request(1, Command::PrepareSolver { handoff })),
                    Err(ProtocolError::Receipt)
                );
                assert_eq!(target.recovery_image(0, 0).unwrap(), prior);
                assert!(target.handle(request(1, Command::Observe)).is_ok());
            }
        }
    }
}
