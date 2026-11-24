// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Trusted same-build coordinator restart from a published native barrier.
use super::*;
use crate::protocol::{Worker, WorkerImage};
use serde::{Deserialize, Serialize};
use std::path::Path;

const MAGIC: &[u8; 8] = b"GZNATP01";

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct NativeBarrier {
    limit: usize,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    last_refined: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    last_island_ids: Option<Vec<u64>>,
    // Inputs on different workers may interleave prior to a later handoff.
    queued: Vec<PendingImpulse>,
    images: [WorkerImage; 2],
    #[serde(default)]
    issued: insert::IssuedIds,
}

/// A complete native recovery point. Only open files from a trusted directory
/// created by the same native build. The caller fences the previous owner.
pub struct NativeRecoveryPlan {
    barrier: NativeBarrier,
    frame: Frame,
}

impl NativeRecoveryPlan {
    /// Decode the complete file and reproduce both worker observations before
    /// opening endpoints or accepting authority for the run.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, RecoveryError> {
        let (_, payload) = recovery::read_verified(path.as_ref(), &[MAGIC])?;
        let barrier: NativeBarrier = serde_json::from_slice(&payload)?;
        let [a, b] = &barrier.images;
        if barrier.limit == 0
            || a.worker != 0
            || b.worker != 1
            || a.run != b.run
            || a.serial != b.serial
            || [a, b].iter().any(|i| i.sequence == 0)
        {
            return Err(RecoveryError::Format(
                "worker identities or journal capacity",
            ));
        }
        let restore = |image| {
            Worker::restored_observation(image)
                .map_err(|e| Error::Transport(TransportError::Protocol(e)))
        };
        let frame = Frame::new(a.tick(), [restore(a)?, restore(b)?])?;
        if !crate::adaptive::valid_last_interval(
            frame.tick,
            frame.step(),
            barrier.last_refined,
            barrier.last_island_ids.as_deref(),
            frame.workers.len(),
        ) {
            return Err(RecoveryError::Format("invalid adaptive interval"));
        }
        // A body from this interval can already have been retired at the same
        // tick. Such IDs must still fit below the serialized issuance ceiling.
        if barrier
            .last_island_ids
            .as_ref()
            .and_then(|ids| ids.last())
            .is_some_and(|id| match barrier.issued {
                insert::IssuedIds::Empty => true,
                insert::IssuedIds::Through(high) => *id > high,
                insert::IssuedIds::Untracked => false,
            })
        {
            return Err(RecoveryError::Format("unissued body in island interval"));
        }
        if !barrier.issued.consistent(&frame) {
            return Err(RecoveryError::Format(
                "body ID allocator behind worker state",
            ));
        }
        if barrier.queued.iter().any(|p| frame.body(p.id).is_none()) {
            return Err(RecoveryError::Format("unknown input owner"));
        }
        for image in &barrier.images {
            compact::check_authority(image, &frame, &barrier.queued)?;
        }
        Ok(Self { barrier, frame })
    }

    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Start two new local worker processes directly at the saved barrier.
    /// Read back their native images before allowing publication. There are no
    /// seed actions to replay and the new journals start empty.
    pub fn resume(
        self,
        executable: impl AsRef<Path>,
    ) -> Result<Coordinator<WorkerProcess>, RecoveryError> {
        let NativeBarrier {
            limit,
            last_refined,
            last_island_ids,
            queued,
            images,
            issued,
        } = self.barrier;
        let [a, b] = &images;
        let path = executable.as_ref();
        let peers = [
            WorkerProcess::spawn_from_image(path, a)?,
            WorkerProcess::spawn_from_image(path, b)?,
        ];
        let mut coordinator = Coordinator {
            run: a.run,
            last_refined,
            last_island_ids,
            peers,
            journals: Default::default(),
            baseline_sequences: images.each_ref().map(WorkerImage::sequence),
            offline: [false; 2],
            limit,
            rolling_journals: false,
            serial: a.serial,
            frame: self.frame,
            issued,
            pending: None,
            aborting: false,
            queued,
            recording: None,
        };
        let booted = coordinator.checked_images()?;
        if !booted
            .iter()
            .zip(&images)
            .all(|(actual, expected)| actual.digest() == expected.digest())
        {
            return Err(RecoveryError::Divergence);
        }
        Ok(coordinator)
    }
}

impl<P: Peer> Coordinator<P> {
    /// Atomically publish a complete immutable native barrier to a trusted
    /// same-build file. This requires a healthy, idle coordinator and leaves
    /// the live worker journals untouched.
    pub fn save_native_recovery(&mut self, path: impl AsRef<Path>) -> Result<(), RecoveryError> {
        let barrier = NativeBarrier {
            images: self.checked_images()?,
            issued: self.issued,
            queued: self.queued.clone(),
            limit: self.limit,
            last_refined: self.last_refined,
            last_island_ids: self.last_island_ids.clone(),
        };
        recovery::write_immutable(path.as_ref(), MAGIC, |out| {
            serde_json::to_writer(out, &barrier)?;
            Ok(())
        })
    }
}
