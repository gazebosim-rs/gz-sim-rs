// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Rebase bounded recovery journals on complete, verified worker images.
use super::*;
use crate::protocol::{Worker, WorkerImage};

/// A peer whose replacement can boot an entire trusted same-build solver
/// image. Retention changes only the *next* replacement's private bootstrap.
pub trait CheckpointPeer {
    fn capture_committed(
        &mut self,
        sequence: u64,
        tick: u64,
    ) -> Result<WorkerImage, TransportError>;
    fn retain_restart_image(&mut self, encoded: Vec<u8>);
}
impl CheckpointPeer for WorkerProcess {
    fn capture_committed(
        &mut self,
        sequence: u64,
        tick: u64,
    ) -> Result<WorkerImage, TransportError> {
        self.fetch_image(sequence, tick)
    }
    fn retain_restart_image(&mut self, encoded: Vec<u8>) {
        self.retain_restart_seed(encoded);
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Compaction {
    pub tick: u64,
    pub sequences: [u64; 2],
    pub released_entries: [usize; 2],
    /// Combined JSON seed payloads retained by the two worker launchers.
    pub bootstrap_bytes: usize,
}

impl Coordinator<WorkerProcess> {
    /// Rebase committed worker journals on demand when the next operation would
    /// exceed their remaining space. A resumed coordinator can select this again.
    pub fn enable_rolling_journals(&mut self) {
        self.rolling_journals = true;
    }
}

impl<P: Peer> Coordinator<P> {
    pub(super) fn reserve_journal(&mut self, counts: [usize; 2]) -> Result<(), Error> {
        let start = self
            .pending
            .as_ref()
            .map_or_else(|| self.retained_entries(), |p| p.start);
        if (0..2).all(|i| counts[i] <= self.limit.saturating_sub(start[i])) {
            return Ok(());
        }
        if !self.rolling_journals
            || self.pending.is_some()
            || counts.iter().any(|n| *n > self.limit)
        {
            return Err(Error::Capacity);
        }
        self.compact()?;
        Ok(())
    }

    pub(super) fn require_committed(&self) -> Result<(), Error> {
        if self.pending.is_some() || self.aborting {
            return Err(Error::Pending);
        }
        if let Some(w) = self.offline.iter().position(|&x| x) {
            return Err(Error::Offline(w as u8));
        }
        Ok(())
    }

    pub(super) fn checked_images(&mut self) -> Result<[WorkerImage; 2], Error> {
        self.require_committed()?;
        let seq = [0, 1].map(|w| self.baseline_sequences[w] + self.journals[w].len() as u64);
        let [a, b] = &mut self.peers;
        let a = a
            .checkpoint_peer()
            .ok_or(Error::Invalid("peer lacks native checkpoint support"))?;
        let b = b
            .checkpoint_peer()
            .ok_or(Error::Invalid("peer lacks native checkpoint support"))?;
        let tick = self.frame.tick;
        let images = [
            a.capture_committed(seq[0], tick)?,
            b.capture_committed(seq[1], tick)?,
        ];
        self.verify_images(&images, seq)?;
        Ok(images)
    }

    fn verify_images(&self, images: &[WorkerImage; 2], seq: [u64; 2]) -> Result<(), Error> {
        for (w, image) in images.iter().enumerate() {
            let obs = &self.frame.workers[w];
            check_authority(image, &self.frame, &self.queued)?;
            if (
                image.run,
                image.worker,
                image.sequence,
                image.tick(),
                image.serial,
            ) != (self.run, w as u8, seq[w], self.frame.tick, self.serial)
                || Worker::restored_observation(image).map_err(TransportError::Protocol)? != *obs
            {
                return Err(Error::Invalid(
                    "checkpoint does not reproduce published worker",
                ));
            }
        }
        Ok(())
    }

    /// Replace both launchers' recovery seeds and recycle their acknowledged
    /// journal entries. This does not advance or restart either live solver.
    /// Fenced recovery then replays only entries since the last compaction.
    pub fn compact(&mut self) -> Result<Compaction, Error> {
        self.require_committed()?;
        let released = self.journals.each_ref().map(Vec::len);
        if released == [0, 0] {
            return Ok(Compaction {
                tick: self.frame.tick,
                sequences: self.baseline_sequences,
                released_entries: released,
                bootstrap_bytes: 0,
            });
        }
        if self
            .recording
            .as_ref()
            .is_some_and(|r| r.len() >= recovery::MAX_ACTIONS)
        {
            return Err(Error::Capacity);
        }
        let images = self.checked_images()?;
        let sequences = images.each_ref().map(WorkerImage::sequence);
        let encoded = [
            WorkerProcess::encode_restart_image(&images[0])?,
            WorkerProcess::encode_restart_image(&images[1])?,
        ];
        let bootstrap_bytes = encoded.iter().map(Vec::len).sum();
        if let Some(recorder) = &mut self.recording {
            recorder
                .compact(&self.journals)
                .map_err(|_| Error::Invalid("compaction journal encoding"))?;
        }
        // Both encodings and both real native restores have succeeded. Every
        // remaining operation is a local, infallible replacement.
        for (peer, seed) in self.peers.iter_mut().zip(encoded) {
            peer.checkpoint_peer()
                .expect("checkpoint capability is stable")
                .retain_restart_image(seed);
        }
        self.baseline_sequences = sequences;
        self.journals = [Vec::new(), Vec::new()];
        Ok(Compaction {
            tick: self.frame.tick,
            sequences,
            released_entries: released,
            bootstrap_bytes,
        })
    }
}

pub(super) fn check_authority(
    image: &WorkerImage,
    frame: &Frame,
    queued: &[PendingImpulse],
) -> Result<(), Error> {
    let w = usize::from(image.worker);
    let obs = frame
        .workers
        .get(w)
        .ok_or(Error::Invalid("checkpoint worker ID"))?;
    let retired: BTreeMap<_, _> = frame.workers[1 - w]
        .generations
        .iter()
        .filter(|(_, g)| **g > 0)
        .map(|(&id, &g)| (id, g))
        .collect();
    if image.generations != obs.generations
        || image.retired != retired
        || image.pending().iter().ne(queued
            .iter()
            .filter(|p| obs.generations.contains_key(&p.id)))
    {
        return Err(Error::Invalid(
            "checkpoint ownership fences or accepted inputs",
        ));
    }
    Ok(())
}
