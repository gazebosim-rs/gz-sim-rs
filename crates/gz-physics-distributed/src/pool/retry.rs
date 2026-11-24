// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recover an ambiguous final step reply using the worker's sequence-and-payload cache.
use super::*;
use crate::transport::TransportError;

pub(super) struct StepBatch {
    pub(super) sampled_move: Option<PoolBalanceDecision>,
    pub(super) stage: Stage,
    pub(super) requests: Vec<Request>,
    pub(super) helper_mask: u8,
    pub(super) resolution: Resolution,
}
pub(super) struct PendingStep {
    batch: StepBatch,
    input: Vec<u8>,
    seen: Vec<Option<Reply>>,
}

pub(super) fn input_bytes(gravity: [f64; 3], field: &[BodyWrench]) -> Result<Vec<u8>, Error> {
    // Signed zero and caller input order are part of the protocol identity.
    bincode::serialize(&(gravity, field)).map_err(|_| Error::Invalid("unserializable retry field"))
}

impl WorkerPool {
    /// Apply the local supervisor's policies at both fresh construction and native restore.
    pub(crate) fn enable_scene_supervision(&mut self) {
        self.set_handoff_confirmation(true);
        self.step_confirmation = std::time::Duration::from_millis(150);
    }

    /// Bound the second request for an ambiguous final step. The initial solve
    /// and a reconstructed worker both keep the full physical request deadline.
    /// Raw pools start at one second and reset this local policy on disk reload.
    pub fn set_step_confirmation_window(
        &mut self,
        grace: std::time::Duration,
    ) -> Result<(), Error> {
        if grace.is_zero() || grace > crate::transport::REQUEST_TIMEOUT {
            return Err(Error::Invalid(
                "step confirmation grace must be positive and at most one second",
            ));
        }
        self.step_confirmation = grace;
        Ok(())
    }

    pub(super) fn remember_interrupted_step(
        &mut self,
        batch: StepBatch,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
        replies: Vec<Result<Reply, TransportError>>,
    ) -> Result<&PoolFrame, Error> {
        let retryable = replies
            .iter()
            .all(|r| matches!(r, Ok(_) | Err(TransportError::Unavailable(_))));
        let mut error = None;
        let seen = replies
            .into_iter()
            .map(|r| match r {
                Ok(reply) => Some(reply),
                Err(e) => {
                    error.get_or_insert(e);
                    None
                }
            })
            .collect();
        if retryable {
            self.pending_step = Some(Box::new(PendingStep {
                batch,
                input: input_bytes(gravity, inputs)?,
                seen,
            }));
        }
        Err(error.expect("at least one incomplete reply").into())
    }

    /// The committed frame and journals still precede the incomplete dispatch.
    /// Other mutations and explicit fencing discard this optimization in stage()/recover().
    pub(super) fn retry_step(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
        resolution: &Resolution,
    ) -> Result<bool, Error> {
        let Some(mut pending) = self.pending_step.take() else {
            return Ok(false);
        };
        if &pending.batch.resolution != resolution || pending.input != input_bytes(gravity, inputs)?
        {
            return Ok(false);
        }
        // A previously acknowledged worker can die before the barrier is published.
        // Verify its reconstructed final observation against the original reply.
        let mut expected = Vec::new();
        for (w, (peer, seen)) in self.peers.iter_mut().zip(&mut pending.seen).enumerate() {
            if let Some(reply) = seen.as_ref()
                && peer.has_exited()?
            {
                expected.push((w, reply_digest(reply)?));
                *seen = None;
            }
        }
        let seen = std::mem::take(&mut pending.seen);
        let mut replies = crate::measured("step_confirm_start", "step_confirm_end", || {
            WorkerProcess::confirm_pool_steps(
                &mut self.peers,
                &pending.batch.requests,
                pending.batch.helper_mask,
                seen,
                self.step_confirmation,
            )
        });
        if replies.iter().any(Result::is_err) {
            replies = crate::measured("step_rebuild_start", "step_rebuild_end", || {
                self.reconstruct_unconfirmed(&pending.batch, replies)
            })?;
        }
        for (w, digest) in expected {
            if let Ok(reply) = &replies[w]
                && reply_digest(reply)? != digest
            {
                return Err(Error::Diverged(w as u8, pending.batch.requests[w].sequence));
            }
        }
        let next = self.frame.tick.checked_add(1).ok_or(Error::Capacity)?;
        self.finish_step(pending.batch, next, replies)?;
        Ok(true)
    }

    /// Return a single failed peer to the exact point preceding the next staged request.
    pub(super) fn recover_stage_prefix(&mut self, w: usize, stage: &Stage) -> Result<(), Error> {
        self.recover(w as u8)?;
        self.dirty[w] = true;
        for entry in &stage.logs[w].entries {
            let actual = self.peers[w].request(&entry.request)?;
            entry.verify(&actual)?;
        }
        Ok(())
    }

    fn reconstruct_unconfirmed(
        &mut self,
        batch: &StepBatch,
        replies: Vec<Result<Reply, TransportError>>,
    ) -> Result<Vec<Result<Reply, TransportError>>, Error> {
        let mut confirmed = Vec::with_capacity(replies.len());
        for (w, result) in replies.into_iter().enumerate() {
            match result {
                Ok(reply) => confirmed.push(Some(reply)),
                Err(
                    TransportError::Unavailable(_)
                    | TransportError::Protocol(crate::protocol::ProtocolError::Sequence { .. }),
                ) => {
                    self.recover_stage_prefix(w, &batch.stage)?;
                    confirmed.push(None);
                }
                Err(error) => return Err(error.into()),
            }
        }
        // Rebuilt workers execute the original step with its exact sequence. Other
        // workers have already delivered validated replies at the intended barrier.
        let timeout = self.native_work_timeout();
        Ok(WorkerProcess::confirm_pool_steps(
            &mut self.peers,
            &batch.requests,
            batch.helper_mask,
            confirmed,
            timeout,
        ))
    }
}
