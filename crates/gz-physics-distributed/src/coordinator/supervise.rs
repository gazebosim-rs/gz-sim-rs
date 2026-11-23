// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local process supervision for normal scene operations.
use super::*;

impl Coordinator<WorkerProcess> {
    pub(crate) fn set_packed_observations(&mut self, enabled: bool) {
        for peer in &mut self.peers {
            peer.set_packed_observations(enabled);
        }
    }
    pub(crate) fn set_packed_handoffs(&mut self, enabled: bool) {
        for peer in &mut self.peers {
            peer.set_packed_handoffs(enabled);
        }
    }
    pub(crate) fn packed_handoff_workers(&self) -> usize {
        self.peers
            .iter()
            .filter(|p| p.uses_packed_handoffs())
            .count()
    }
    pub(crate) fn traffic(&self) -> [crate::transport::Traffic; 2] {
        self.peers.each_ref().map(WorkerProcess::traffic)
    }
    pub(crate) fn drop_next_acknowledgement(&mut self, worker: u8) -> Result<(), Error> {
        self.peers
            .get_mut(usize::from(worker))
            .ok_or(Error::Invalid("unknown worker"))?
            .drop_acknowledgement_after(0);
        Ok(())
    }
    /// Operating-system process IDs in partition order at the current barrier.
    pub fn worker_pids(&self) -> [u32; 2] {
        self.peers.each_ref().map(WorkerProcess::pid)
    }

    // A final step has an immutable action, sequence and acknowledged prefix.
    // Confirm its missing reply once before paying to restart a live solver.
    // Handoffs and edits retain their existing fence-and-replay path.
    pub(crate) fn supervised(
        &mut self,
        mut operation: impl FnMut(&mut Self) -> Result<(), Error>,
    ) -> Result<(), Error> {
        for p in &mut self.peers {
            p.unavailable = false;
        }
        let mut result = operation(self);
        if matches!(
            result,
            Err(Error::Transport(TransportError::Unavailable(_)))
        ) && self.pending.as_ref().is_some_and(|p| {
            matches!(
                p.action,
                Action::Advance
                    | Action::Forced(_)
                    | Action::Field(_)
                    | Action::Refined(_)
                    | Action::Islands(_, _)
                    | Action::SceneWithSeparation(_, _)
            )
        }) {
            result = operation(self);
        }
        let failed = match result {
            Ok(()) => return Ok(()),
            Err(Error::Offline(w)) => [w == 0, w == 1],
            Err(Error::Transport(TransportError::Unavailable(_))) => {
                self.peers.each_ref().map(|p| p.unavailable)
            }
            Err(e) => {
                self.abort_pending()?;
                return Err(e);
            }
        };
        for (i, replace) in failed.into_iter().enumerate() {
            if replace {
                self.recover(i as u8)?;
            }
        }
        if let Err(e) = operation(self) {
            self.abort_pending()?;
            return Err(e);
        }
        Ok(())
    }
}
