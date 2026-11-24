// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A requested pool capture can carry opaque native solver bytes separately from typed JSON.
use super::{
    MAX_WORKER_IMAGE_PAYLOAD, Response, TransportError, WorkerImage, encode_limited, wire,
};
use crate::{physics::MAX_PARTITION_BODIES, protocol::ProtocolError};
use gz_physics_multifidelity::PARTITION_CHECKPOINT_LIMIT;

const MAGIC: &[u8; 5] = b"GZIM\x01";
const HEADER: usize = 11;
const MAX_METADATA: usize = 1024 * 1024;
pub(super) const SELECTOR: &str = "gz_worker_image_v1=1";
pub(super) const PARAMETER: &str = "gz_worker_image_v1";

pub(super) fn is_packed(bytes: &[u8]) -> bool {
    bytes.starts_with(MAGIC)
}

// Leave enough room for every native byte expanded as an ordinary JSON u8 and
// comma, plus the historical response/seed envelope. Larger legal images use
// the existing JSON path and its exact bound.
fn fits_legacy_image(metadata: usize, native: usize) -> bool {
    native
        .checked_mul(4)
        .and_then(|n| n.checked_add(metadata))
        .and_then(|n| n.checked_add(64))
        .is_some_and(|n| n <= MAX_WORKER_IMAGE_PAYLOAD)
}
fn pack(image: &WorkerImage, payloads: &[Vec<u8>]) -> Result<Option<Vec<u8>>, TransportError> {
    if payloads.len() > MAX_PARTITION_BODIES
        || payloads
            .iter()
            .any(|p| p.is_empty() || p.len() > PARTITION_CHECKPOINT_LIMIT)
    {
        return Err(wire("invalid native checkpoint component lengths"));
    }
    if payloads.is_empty() {
        return Ok(None);
    }
    let Ok(metadata) = encode_limited(image, MAX_METADATA) else {
        return Ok(None);
    };
    let native = payloads.iter().map(Vec::len).sum();
    if !fits_legacy_image(metadata.len(), native) {
        return Ok(None);
    }
    let mut out = Vec::with_capacity(HEADER + 4 * payloads.len() + metadata.len() + native);
    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&(metadata.len() as u32).to_le_bytes());
    out.extend_from_slice(&(payloads.len() as u16).to_le_bytes());
    for p in payloads {
        out.extend_from_slice(&(p.len() as u32).to_le_bytes());
    }
    out.extend_from_slice(&metadata);
    for p in payloads {
        out.extend_from_slice(p);
    }
    Ok(Some(out))
}
pub(super) fn encode(
    result: Result<WorkerImage, ProtocolError>,
    packed: bool,
) -> Result<Vec<u8>, TransportError> {
    let mut result = result;
    if packed && let Ok(image) = &mut result {
        let payloads = image
            .native_wire_payloads()
            .map(std::mem::take)
            .collect::<Vec<_>>();
        if let Some(bytes) = pack(image, &payloads)? {
            return Ok(bytes);
        }
        for (slot, bytes) in image.native_wire_payloads().zip(payloads) {
            *slot = bytes;
        }
    }
    encode_limited(
        &Response::Saved(result.map(Box::new)),
        MAX_WORKER_IMAGE_PAYLOAD,
    )
}
pub(super) fn decode(bytes: &[u8]) -> Result<WorkerImage, TransportError> {
    let (prefix, rest) = bytes
        .split_first_chunk::<HEADER>()
        .ok_or_else(|| wire("incomplete packed worker image"))?;
    if &prefix[..5] != MAGIC || bytes.len() > MAX_WORKER_IMAGE_PAYLOAD {
        return Err(wire("invalid worker image version or size"));
    }
    let meta = u32::from_le_bytes(prefix[5..9].try_into().unwrap()) as usize;
    let count = u16::from_le_bytes(prefix[9..11].try_into().unwrap()) as usize;
    if !(1..=MAX_METADATA).contains(&meta) || !(1..=MAX_PARTITION_BODIES).contains(&count) {
        return Err(wire("invalid worker image metadata or component count"));
    }
    let (table, rest) = rest
        .split_at_checked(count * 4)
        .ok_or_else(|| wire("truncated worker image length table"))?;
    let (metadata, mut payload) = rest
        .split_at_checked(meta)
        .ok_or_else(|| wire("truncated worker image metadata"))?;
    let lengths = || {
        table
            .chunks_exact(4)
            .map(|s| u32::from_le_bytes(s.try_into().unwrap()) as usize)
    };
    // Consume the complete length table against the bounded actual message
    // before parsing metadata or allocating any native buffers.
    let left = lengths().try_fold(payload.len(), |n, len| {
        if (1..=PARTITION_CHECKPOINT_LIMIT).contains(&len) {
            n.checked_sub(len)
        } else {
            None
        }
    });
    if left != Some(0) || !fits_legacy_image(meta, payload.len()) {
        return Err(wire("invalid worker image native lengths"));
    }
    let mut image: WorkerImage = serde_json::from_slice(metadata).map_err(wire)?;
    if image.native_wire_payloads().len() != count
        || image.native_wire_payloads().any(|v| !v.is_empty())
    {
        return Err(wire("worker image metadata disagrees with native payloads"));
    }
    for (slot, len) in image.native_wire_payloads().zip(lengths()) {
        let (part, rest) = payload.split_at(len);
        slot.extend_from_slice(part);
        payload = rest;
    }
    Ok(image)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        physics::{Snapshot, fixture},
        protocol::Worker,
    };
    #[test]
    fn packed_images_restore_exact_bytes_and_reject_inconsistent_lengths_before_hydration() {
        let mut bodies = fixture();
        bodies[0].position[0] = -20.;
        bodies[1].position[0] = 20.;
        let peer = Worker::new(
            4,
            0,
            Snapshot {
                bodies,
                ..Default::default()
            },
        )
        .unwrap();
        let image = peer.recovery_image(0, 0).unwrap();
        let bytes = encode(Ok(image.clone()), true).unwrap();
        assert!(is_packed(&bytes));
        assert_eq!(decode(&bytes).unwrap(), image);
        Worker::from_trusted_image(4, 0, &decode(&bytes).unwrap()).unwrap();
        assert!(bytes.len() < encode(Ok(image.clone()), false).unwrap().len());
        for cut in [0, 5, HEADER - 1, HEADER, HEADER + 4, bytes.len() - 1] {
            assert!(decode(&bytes[..cut]).is_err());
        }
        let mut trailing = bytes.clone();
        trailing.push(0);
        assert!(decode(&trailing).is_err());
        for (offset, val) in [(4, 2), (9, 0), (9, 3), (10, 255), (8, 255), (14, 255)] {
            let mut bad = bytes.clone();
            bad[offset] = val;
            assert!(decode(&bad).is_err());
        }
        let mut bad = bytes.clone();
        *bad.last_mut().unwrap() ^= 1;
        assert!(Worker::from_trusted_image(4, 0, &decode(&bad).unwrap()).is_err());
        assert!(!fits_legacy_image(1, usize::MAX));
        let empty = Worker::new(4, 1, Snapshot::default())
            .unwrap()
            .recovery_image(0, 0)
            .unwrap();
        assert!(!is_packed(&encode(Ok(empty), true).unwrap()));
    }
    #[test]
    fn conservative_binary_budget_falls_back_to_the_original_legal_json() {
        let template = fixture()[1].clone();
        let bodies = (1..=3)
            .map(|id| crate::physics::Body {
                id,
                position: [20. * id as f64, 0., 0.],
                ..template.clone()
            })
            .collect();
        let peer = Worker::new(
            4,
            0,
            Snapshot {
                bodies,
                ..Default::default()
            },
        )
        .unwrap();
        let mut image = peer.recovery_image(0, 0).unwrap();
        assert_eq!(image.native_wire_payloads().len(), 3);
        // The conservative four-character decimal bound crosses 24 MiB, while
        // these one-character integer values still fit the legacy response.
        for (i, p) in image.native_wire_payloads().enumerate() {
            p.resize(PARTITION_CHECKPOINT_LIMIT, i as u8);
        }
        let bytes = encode(Ok(image.clone()), true).unwrap();
        assert!(!is_packed(&bytes));
        assert_eq!(bytes, encode(Ok(image.clone()), false).unwrap());
        let Response::Saved(Ok(decoded)) = serde_json::from_slice(&bytes).unwrap() else {
            panic!("unexpected fallback")
        };
        assert_eq!(*decoded, image);
        assert!(Worker::from_trusted_image(4, 0, &decoded).is_err());
        assert!(!fits_legacy_image(usize::MAX, 0));
    }
    #[test]
    fn packed_metadata_requires_the_right_number_of_empty_native_slots() {
        let mut b = fixture()[1].clone();
        b.position[0] = 20.;
        let peer = Worker::new(
            79,
            0,
            Snapshot {
                bodies: vec![b],
                ..Default::default()
            },
        )
        .unwrap();
        let mut image = peer.recovery_image(0, 0).unwrap();
        let payloads = image
            .native_wire_payloads()
            .map(std::mem::take)
            .collect::<Vec<_>>();
        let mut extra = payloads.clone();
        extra.push(vec![1]);
        assert!(decode(&pack(&image, &extra).unwrap().unwrap()).is_err());
        image.native_wire_payloads().next().unwrap().push(42);
        assert!(decode(&pack(&image, &payloads).unwrap().unwrap()).is_err());
    }
}
