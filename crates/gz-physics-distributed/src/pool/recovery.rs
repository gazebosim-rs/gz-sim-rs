// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::protocol::{Worker, WorkerImage};
use crate::transport::MAX_CHECKPOINT_PAYLOAD;
impl WorkerPool {
    pub(super) fn command_counts(&self, moves: &[(u64, usize)]) -> Vec<usize> {
        self.command_counts_in(&self.frame, moves)
    }
    pub(super) fn command_counts_in(
        &self,
        frame: &PoolFrame,
        moves: &[(u64, usize)],
    ) -> Vec<usize> {
        let mut counts = vec![1; self.peers.len()];
        for &(id, d) in moves {
            let (s, _) = frame.body(id).expect("route from projected frame");
            counts[s as usize] += 3;
            counts[d] += 2;
        }
        counts
    }
    /// Check prospective interactions, per-shard ownership capacity and the
    /// command budget for one complete advancing barrier without any RPC.
    pub fn validate_next(&self, gravity: [f64; 3]) -> Result<(), Error> {
        self.validate_field(gravity, &[])
    }

    /// Check a prospective held field and all accepted impulses against the
    /// complete pool without dispatching a command or consuming a journal slot.
    pub fn validate_field(&self, gravity: [f64; 3], inputs: &[BodyWrench]) -> Result<(), Error> {
        let counts = self.command_counts(&self.routes(gravity, &self.queued, inputs)?);
        if counts.iter().any(|&c| c > self.limit) {
            return Err(Error::Capacity);
        }
        Ok(())
    }
    pub(super) fn reserve(&mut self, counts: &[usize]) -> Result<(), Error> {
        if self.placement_replay.attempt.is_some() {
            return Err(Error::Pending);
        }
        self.reserve_step(counts)
    }
    pub(super) fn validate_step_counts(&self, counts: &[usize]) -> Result<(), Error> {
        for (w, &count) in counts.iter().enumerate() {
            let through = self.history[w]
                .entries
                .len()
                .checked_add(count)
                .ok_or(Error::Capacity)?;
            if count > self.limit || self.baseline[w].checked_add(through as u64).is_none() {
                return Err(Error::Capacity);
            }
        }
        Ok(())
    }
    pub(super) fn reserve_step(&mut self, counts: &[usize]) -> Result<(), Error> {
        self.validate_step_counts(counts)?;
        // Keep space for one maximal incoming image. Exact byte limits are
        // also checked before each native exchange in a multi-transfer barrier.
        if self.history.iter().zip(counts).any(|(h, &c)| {
            h.entries.len() + c > self.limit
                || h.bytes + MAX_CHECKPOINT_PAYLOAD as u64 > MAX_JOURNAL_BYTES
        }) {
            self.compact()?;
        }
        Ok(())
    }
    /// Capture all native caches and ownership fences at one committed tick.
    /// Restart seeds and replay cursors change together after every image agrees.
    pub fn compact(&mut self) -> Result<(), Error> {
        if self.placement_replay.attempt.is_some() {
            return Err(Error::Pending);
        }
        let width = self.peers.len() as u8;
        let encoded = self
            .checked_images()?
            .iter()
            .map(|image| {
                Ok((
                    image.sequence(),
                    WorkerProcess::encode_pool_image(width, image)?,
                ))
            })
            .collect::<Result<Vec<_>, Error>>()?;
        for (w, (sequence, image)) in encoded.into_iter().enumerate() {
            self.peers[w].retain_restart_seed(image);
            self.baseline[w] = sequence;
            self.history[w] = Journal::default();
        }
        Ok(())
    }
    pub(super) fn checked_images(&mut self) -> Result<Vec<WorkerImage>, Error> {
        let width = self.peers.len();
        let mut cursors = Vec::with_capacity(width);
        for w in 0..width {
            if self.dirty[w] {
                self.recover(w as u8)?;
            }
            cursors.push(
                self.baseline[w]
                    .checked_add(self.history[w].entries.len() as u64)
                    .ok_or(Error::Capacity)?,
            );
        }
        // Up to two 24 MiB image replies can be queued at once. Complete and
        // validate each pair before asking additional workers for native state.
        const CAPTURE_WIDTH: usize = 2;
        let mut images = Vec::with_capacity(width);
        for start in (0..width).step_by(CAPTURE_WIDTH) {
            let end = (start + CAPTURE_WIDTH).min(width);
            let rows = WorkerProcess::fetch_pool_images(
                &mut self.peers[start..end],
                &cursors[start..end],
                self.frame.tick,
            );
            for (dirty, row) in self.dirty[start..end].iter_mut().zip(&rows) {
                *dirty = row.is_err();
            }
            for (i, row) in rows.into_iter().enumerate() {
                let w = start + i;
                let image = row?;
                self.verify_captured_image(w, cursors[w], &image)?;
                images.push(image);
            }
        }
        Ok(images)
    }

    fn verify_captured_image(
        &self,
        w: usize,
        sequence: u64,
        image: &WorkerImage,
    ) -> Result<(), Error> {
        let worker = Worker::from_scoped_image(self.run, w as u8, image, self.peers.len() as u8)
            .map_err(crate::transport::TransportError::from)?;
        if image.native_solver_profile() != self.profile
            || image.sequence() != sequence
            || image.tick() != self.frame.tick
            || image.serial != self.serials[w]
            || image.pending().iter().ne(self
                .queued
                .iter()
                .filter(|p| self.frame.workers[w].generations.contains_key(&p.id)))
            || image.retired != self.retired[w]
            || worker.observation() != self.frame.workers[w]
        {
            return Err(Error::Invalid(
                "pool baseline disagrees with committed ownership or state",
            ));
        }
        Ok(())
    }

    pub fn retained_entries(&self) -> Vec<usize> {
        self.history.iter().map(|h| h.entries.len()).collect()
    }
}
