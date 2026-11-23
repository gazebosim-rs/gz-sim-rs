// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A complete local recovery point, including the generation fences on released IDs.
use super::*;
use crate::physics::{PartitionImage, PendingImpulse};
use sha2::{Digest, Sha256};
use std::io::{BufWriter, Write};

/// Pinned native state for one complete trusted worker. Each image includes an
/// integrity digest; provenance and exclusive authority are the caller's responsibility.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorkerImage {
    pub(crate) run: u64,
    pub(crate) worker: u8,
    pub(crate) sequence: u64,
    pub(crate) serial: u64,
    pub(crate) generations: BTreeMap<u64, u64>,
    pub(crate) retired: BTreeMap<u64, u64>,
    partition: PartitionImage,
    digest: [u8; 32],
}

impl WorkerImage {
    pub(crate) fn native_wire_payloads(&mut self) -> impl ExactSizeIterator<Item = &mut Vec<u8>> {
        self.partition.native_wire_payloads()
    }

    pub fn native_solver_profile(&self) -> NativeSolverProfile {
        self.partition.profile
    }
    pub fn sequence(&self) -> u64 {
        self.sequence
    }
    pub fn tick(&self) -> u64 {
        self.partition.tick
    }
    pub fn digest(&self) -> [u8; 32] {
        self.digest
    }
    pub(crate) fn pending(&self) -> &[PendingImpulse] {
        &self.partition.pending
    }
    fn checksum(&self, width: u8) -> Result<[u8; 32], ProtocolError> {
        let mut hash = Sha256::new();
        if width == 2 {
            hash.update(b"gz-physics-distributed/worker-baseline/v1");
        } else {
            hash.update(b"gz-physics-distributed/pool-worker-baseline/v1");
            hash.update([width]);
        }
        let mut sink = BufWriter::new(&mut hash);
        bincode::serialize_into(
            &mut sink,
            &(
                self.run,
                self.worker,
                self.sequence,
                self.serial,
                &self.generations,
                &self.retired,
                &self.partition,
            ),
        )
        .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        sink.flush()
            .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        drop(sink);
        Ok(hash.finalize().into())
    }
}

impl Worker {
    /// Inspect last-native-substep shared-plane points using stable distributed IDs.
    pub fn ground_contact_patches(&self) -> Vec<crate::physics::GroundContactPatch> {
        self.partition.ground_contact_patches()
    }
    /// Read the last native finite manifold points with stable distributed body IDs.
    pub fn finite_contact_patches(&self) -> Vec<crate::physics::FiniteContactPatch> {
        self.partition.finite_contact_patches()
    }

    /// Capture a healthy worker at its exact acknowledged sequence and tick.
    /// This read-only operation is also available when the request journal is full.
    pub fn recovery_image(&self, sequence: u64, tick: u64) -> Result<WorkerImage, ProtocolError> {
        self.check_read_epoch(sequence, tick)?;
        let mut image = WorkerImage {
            run: self.run_id,
            worker: self.worker_id,
            sequence,
            serial: self.serial,
            generations: self.generations.clone(),
            retired: self.retired.clone(),
            partition: self
                .partition
                .image()
                .map_err(|e| ProtocolError::Physics(e.to_string()))?,
            digest: [0; 32],
        };
        image.digest = image.checksum(self.width)?;
        Ok(image)
    }

    /// Start a fenced replacement using a verified image from this worker.
    /// The first new sequenced request follows the captured sequence.
    pub fn from_trusted_image(
        run: u64,
        id: u8,
        image: &WorkerImage,
    ) -> Result<Self, ProtocolError> {
        Self::from_scoped_image(run, id, image, 2)
    }

    pub(crate) fn from_scoped_image(
        run: u64,
        id: u8,
        image: &WorkerImage,
        width: u8,
    ) -> Result<Self, ProtocolError> {
        if !(2..=8).contains(&width)
            || id >= width
            || (width == 2 && !image.native_solver_profile().is_standard())
            || image.run != run
            || image.worker != id
        {
            return Err(ProtocolError::Address);
        }
        if image.generations.len() + image.retired.len() > MAX_BODY_IDS
            || image
                .generations
                .keys()
                .any(|id| image.retired.contains_key(id))
            || image.retired.values().any(|&g| g == 0)
            || image.checksum(width)? != image.digest
        {
            return Err(ProtocolError::Receipt);
        }
        let partition = Partition::from_image(&image.partition)
            .map_err(|e| ProtocolError::Physics(e.to_string()))?;
        if partition
            .snapshot()
            .bodies
            .iter()
            .map(|b| &b.id)
            .ne(image.generations.keys())
        {
            return Err(ProtocolError::Ownership);
        }
        Ok(Self {
            width,
            run_id: run,
            worker_id: id,
            partition,
            tick: image.tick(),
            generations: image.generations.clone(),
            retired: image.retired.clone(),
            serial: image.serial,
            pending: None,
            baseline_sequence: image.sequence,
            last: None,
            terminal: None,
        })
    }

    pub(crate) fn restored_observation(image: &WorkerImage) -> Result<Observation, ProtocolError> {
        Ok(Self::from_trusted_image(image.run, image.worker, image)?.observation())
    }
}

#[cfg(test)]
#[path = "../../tests/private/worker_image.rs"]
mod tests;
