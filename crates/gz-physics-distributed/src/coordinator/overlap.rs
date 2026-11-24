// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Overlap the network waits of two independently admitted native solves.
use super::*;

/// A transport that can dispatch both independent steps before waiting on a
/// reply. Return an outcome for each worker, including on partial failure.
pub trait OverlappingPeers: Peer + Sized {
    fn exchange_steps(
        peers: &mut [Self; 2],
        requests: &[Request; 2],
    ) -> [Result<Reply, TransportError>; 2];
}

impl OverlappingPeers for WorkerProcess {
    fn exchange_steps(
        peers: &mut [Self; 2],
        requests: &[Request; 2],
    ) -> [Result<Reply, TransportError>; 2] {
        Self::simultaneous_steps(peers, requests)
    }
}

impl<P: OverlappingPeers> Coordinator<P> {
    /// Dispatch both workers' steps before awaiting either reply. Fencing,
    /// journal capacity and publication use the same barriers as `advance`.
    /// Retry or roll back a partially delivered step through either API.
    pub fn advance_overlapped(&mut self) -> Result<&Frame, Error> {
        let (end, cursor) = self.begin_step()?;
        self.dispatch_steps(end, cursor, self.step_commands())
    }

    /// Route the same field and held forces as the sequential API, then issue
    /// the independent native solves before collecting either reply.
    pub fn advance_in_field_overlapped(
        &mut self,
        gravity: [f64; 3],
        inputs: &[crate::physics::BodyWrench],
    ) -> Result<&Frame, Error> {
        if gravity == [0.; 3] && inputs.is_empty() {
            return self.advance_routed_overlapped();
        }
        let (end, cursor, commands) = self.begin_field(gravity, inputs)?;
        self.dispatch_steps(end, cursor, commands)
    }

    /// Select refinement from the committed global scene before routing, and
    /// dispatch a common barrier width to both workers. Retries replay the same
    /// sequenced native solve including its held inputs.
    pub fn advance_adaptive_in_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[crate::physics::BodyWrench],
    ) -> Result<&Frame, Error> {
        if self.adaptive_substeps(gravity, inputs)? == 1 {
            return self.advance_in_field_overlapped(gravity, inputs);
        }
        let (end, cursor, commands) = self.begin_selected_field(gravity, inputs, true)?;
        self.dispatch_steps(end, cursor, commands)
    }

    /// Apply the selected scene-wide rate while allowing safe native separation at commit.
    pub fn advance_adaptive_with_separation(
        &mut self,
        gravity: [f64; 3],
        inputs: &[crate::physics::BodyWrench],
    ) -> Result<&Frame, Error> {
        let fine = self.adaptive_substeps(gravity, inputs)? == 10;
        let (end, cursor, commands) = self.begin_uniform_field(
            gravity,
            inputs,
            crate::adaptive::Resolution::SceneWithSeparation { fine },
        )?;
        self.dispatch_steps(end, cursor, commands)
    }

    /// Advance independent native solver groups at their selected resolutions.
    pub fn advance_islands_in_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[crate::physics::BodyWrench],
    ) -> Result<&Frame, Error> {
        let (end, cursor, commands) = self.begin_island_field(gravity, inputs)?;
        self.dispatch_steps(end, cursor, commands)
    }

    fn dispatch_steps(
        &mut self,
        end: u64,
        cursor: [usize; 2],
        commands: [Command; 2],
    ) -> Result<&Frame, Error> {
        if self.journals.each_ref().map(Vec::len) == cursor {
            let requests = [0, 1].map(|w| Request {
                run_id: self.run,
                worker_id: w as u8,
                sequence: self.baseline_sequences[w] + cursor[w] as u64 + 1,
                command: commands[w].clone(),
            });
            let replies = P::exchange_steps(&mut self.peers, &requests);
            let mut first_error = None;
            for ((journal, request), result) in self.journals.iter_mut().zip(requests).zip(replies)
            {
                let checked = result.map_err(Error::from).and_then(|reply| {
                    validate_envelope(&request, &reply)?;
                    Ok(reply)
                });
                match checked {
                    Ok(reply) => journal.push((request, reply)),
                    Err(error) => {
                        first_error.get_or_insert(error);
                    }
                }
            }
            if let Some(error) = first_error {
                return Err(error);
            }
        }
        self.complete_step(end, cursor, commands)
    }
}
