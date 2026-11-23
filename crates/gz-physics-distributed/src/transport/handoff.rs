// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Versioned packed native transfer: bounded JSON metadata followed by the original solver image.
use super::{Operation, Reply, Request, TransportError, WorkerCommand, encode_limited, wire};
use crate::{physics::WarmCheckpoint, protocol::Outcome};
use gz_physics_multifidelity::PARTITION_CHECKPOINT_LIMIT;
use serde::Serialize;

pub(super) const PARAMETER: &str = "gz_handoff_v1";
pub(super) const SELECTOR: &str = "gz_handoff_v1=1";
const MAGIC: &[u8; 5] = b"GZHW\x01";
const REQUEST: u8 = 0;
const REPLY: u8 = 1;
const PREFIX_BYTES: usize = 14;

pub(super) fn is_packed(bytes: &[u8]) -> bool {
    bytes.starts_with(b"GZHW")
}

pub(super) fn use_selector(op: &Operation<&Request>) -> bool {
    matches!(op, Operation::Request(r) | Operation::DropReply(r) if matches!(
        r.command, WorkerCommand::PrepareWarm {..} | WorkerCommand::PrepareSolver {..}
            | WorkerCommand::PrepareOrdered {..} | WorkerCommand::ExportWarm {..}
            | WorkerCommand::ExportSolver {..}
    ))
}

pub(super) fn expects_export(op: &Operation<&Request>) -> bool {
    matches!(op, Operation::Request(r) if matches!(r.command,
        WorkerCommand::ExportWarm {..} | WorkerCommand::ExportSolver {..}))
}

fn checkpoint(request: &mut Request) -> Option<&mut WarmCheckpoint> {
    match &mut request.command {
        WorkerCommand::PrepareWarm { handoff }
        | WorkerCommand::PrepareSolver { handoff }
        | WorkerCommand::PrepareOrdered { handoff, .. } => Some(&mut handoff.checkpoint),
        _ => None,
    }
}

fn encode(
    kind: u8,
    metadata: &impl Serialize,
    native: &[u8],
) -> Result<Option<Vec<u8>>, TransportError> {
    if native.is_empty() || native.len() > PARTITION_CHECKPOINT_LIMIT {
        return Err(wire("invalid packed solver-image size"));
    }
    // Retain the original JSON path for exceptional legal metadata larger than this codec's limit.
    let Ok(metadata) = encode_limited(metadata, super::MAX_PAYLOAD) else {
        return Ok(None);
    };
    let mut bytes = Vec::with_capacity(PREFIX_BYTES + metadata.len() + native.len());
    bytes.extend_from_slice(MAGIC);
    bytes.push(kind);
    bytes.extend_from_slice(&(metadata.len() as u32).to_le_bytes());
    bytes.extend_from_slice(&(native.len() as u32).to_le_bytes());
    bytes.extend_from_slice(&metadata);
    bytes.extend_from_slice(native);
    Ok(Some(bytes))
}

fn sections(bytes: &[u8], kind: u8) -> Result<(&[u8], &[u8]), TransportError> {
    let (prefix, body) = bytes
        .split_first_chunk::<PREFIX_BYTES>()
        .ok_or_else(|| wire("incomplete packed handoff envelope"))?;
    if &prefix[..5] != MAGIC || prefix[5] != kind {
        return Err(wire("unknown packed handoff version or direction"));
    }
    let header = u32::from_le_bytes(prefix[6..10].try_into().unwrap()) as usize;
    let native = u32::from_le_bytes(prefix[10..14].try_into().unwrap()) as usize;
    if !(1..=super::MAX_PAYLOAD).contains(&header)
        || !(1..=PARTITION_CHECKPOINT_LIMIT).contains(&native)
        || header.checked_add(native) != Some(body.len())
    {
        return Err(wire("invalid packed handoff section lengths"));
    }
    Ok(body.split_at(header))
}

pub(super) fn encode_request(op: &Operation<&Request>) -> Result<Option<Vec<u8>>, TransportError> {
    let (request, drop_reply) = match op {
        Operation::Request(r) => (*r, false),
        Operation::DropReply(r) => (*r, true),
        _ => return Ok(None),
    };
    if !matches!(
        request.command,
        WorkerCommand::PrepareWarm { .. }
            | WorkerCommand::PrepareSolver { .. }
            | WorkerCommand::PrepareOrdered { .. }
    ) {
        return Ok(None);
    }
    // The coordinator retains its complete request for the journal and replay.
    let mut header = request.clone();
    let native = std::mem::take(checkpoint(&mut header).unwrap().native_wire_payload());
    let op = if drop_reply {
        Operation::DropReply(&header)
    } else {
        Operation::Request(&header)
    };
    encode(REQUEST, &op, &native)
}

pub(super) fn encode_reply(reply: &mut Reply) -> Result<Option<Vec<u8>>, TransportError> {
    let Outcome::WarmExported(handoff) = &mut reply.outcome else {
        return Ok(None);
    };
    let native = std::mem::take(handoff.checkpoint.native_wire_payload());
    let bytes = encode(REPLY, reply, &native);
    // JSON remains possible when the header alone exceeds this format's limit.
    if let Outcome::WarmExported(handoff) = &mut reply.outcome {
        *handoff.checkpoint.native_wire_payload() = native;
    }
    bytes
}

fn attach(checkpoint: &mut WarmCheckpoint, native: &[u8]) -> Result<(), TransportError> {
    let slot = checkpoint.native_wire_payload();
    if !slot.is_empty() {
        return Err(wire("duplicate packed solver image"));
    }
    slot.extend_from_slice(native);
    Ok(())
}

pub(super) fn decode_request(bytes: &[u8]) -> Result<Operation<Request>, TransportError> {
    let (header, native) = sections(bytes, REQUEST)?;
    let mut op: Operation<Request> = serde_json::from_slice(header).map_err(wire)?;
    let target = match &mut op {
        Operation::Request(r) | Operation::DropReply(r) => checkpoint(r),
        _ => None,
    }
    .ok_or_else(|| wire("expected packed prepare request"))?;
    attach(target, native)?;
    Ok(op)
}

pub(super) fn decode_reply(bytes: &[u8]) -> Result<Reply, TransportError> {
    let (header, native) = sections(bytes, REPLY)?;
    let mut reply: Reply = serde_json::from_slice(header).map_err(wire)?;
    let Outcome::WarmExported(handoff) = &mut reply.outcome else {
        return Err(wire("expected packed export reply"));
    };
    attach(&mut handoff.checkpoint, native)?;
    Ok(reply)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        physics::{Snapshot, fixture},
        protocol::{WarmTicket, Worker},
    };

    fn export() -> Reply {
        let body = fixture()[1].clone();
        let ticket = WarmTicket {
            run_id: 74,
            serial: 1,
            source: 0,
            destination: 1,
            tick: 0,
            ids: vec![body.id],
            generations: vec![1],
        };
        let mut worker = Worker::new(
            74,
            0,
            Snapshot {
                bodies: vec![body],
                ..Default::default()
            },
        )
        .unwrap();
        worker
            .handle(Request {
                run_id: 74,
                worker_id: 0,
                sequence: 1,
                command: WorkerCommand::ExportSolver { ticket },
            })
            .unwrap()
    }

    #[test]
    fn complete_native_bytes_and_receipts_round_trip_in_either_direction() {
        let mut reply = export();
        let expected = serde_json::to_vec(&reply).unwrap();
        let payload = encode_reply(&mut reply).unwrap().unwrap();
        assert_eq!(serde_json::to_vec(&reply).unwrap(), expected);
        assert_eq!(
            serde_json::to_vec(&decode_reply(&payload).unwrap()).unwrap(),
            expected
        );
        let Outcome::WarmExported(handoff) = reply.outcome else {
            unreachable!()
        };
        handoff.validate_digest().unwrap();
        for command in [
            WorkerCommand::PrepareWarm {
                handoff: handoff.clone(),
            },
            WorkerCommand::PrepareSolver {
                handoff: handoff.clone(),
            },
            WorkerCommand::PrepareOrdered {
                handoff: handoff.clone(),
                complete: true,
                incoming_slots: 5,
            },
        ] {
            let request = Request {
                run_id: 74,
                worker_id: 1,
                sequence: 1,
                command,
            };
            for op in [Operation::Request(&request), Operation::DropReply(&request)] {
                let packed = encode_request(&op).unwrap().unwrap();
                let header = decode_request(&packed).unwrap();
                assert_eq!(
                    serde_json::to_vec(&op).unwrap(),
                    serde_json::to_vec(&header).unwrap()
                );
                assert!(decode_reply(&packed).is_err());
            }
        }
        let mut changed = payload;
        *changed.last_mut().unwrap() ^= 1;
        let Outcome::WarmExported(h) = decode_reply(&changed).unwrap().outcome else {
            unreachable!()
        };
        assert!(h.validate_digest().is_err());
    }

    #[test]
    fn envelope_checks_direction_bounds_and_exact_consumption_before_decoding_metadata() {
        let mut reply = export();
        let payload = encode_reply(&mut reply).unwrap().unwrap();
        for length in 0..=PREFIX_BYTES {
            assert!(decode_reply(&payload[..length]).is_err());
        }
        assert!(decode_reply(&payload[..payload.len() - 1]).is_err());
        let mut extra = payload.clone();
        extra.push(0);
        assert!(decode_reply(&extra).is_err());
        for (idx, value) in [(4, 2), (5, 2), (6, 255), (13, 255)] {
            let mut corrupt = payload.clone();
            corrupt[idx] = value;
            assert!(decode_reply(&corrupt).is_err());
        }
        assert!(decode_request(&payload).is_err());
        let bad = encode(REPLY, &reply, &[1]).unwrap().unwrap();
        assert!(decode_reply(&bad).is_err()); // Header and trailing slice both carry an image.
        let wrong_command = encode(REQUEST, &Operation::<Request>::Shutdown, &[1])
            .unwrap()
            .unwrap();
        assert!(decode_request(&wrong_command).is_err());
        let native = vec![5; PARTITION_CHECKPOINT_LIMIT];
        let at_limit = encode(REPLY, &"header", &native).unwrap().unwrap();
        assert_eq!(sections(&at_limit, REPLY).unwrap().1, native);
        assert!(encode(REPLY, &"header", &[]).is_err());
        assert!(encode(REQUEST, &"header", &vec![0; PARTITION_CHECKPOINT_LIMIT + 1]).is_err());
        assert!(
            encode(REQUEST, &"x".repeat(crate::transport::MAX_PAYLOAD), &[1])
                .unwrap()
                .is_none()
        );
    }
}
