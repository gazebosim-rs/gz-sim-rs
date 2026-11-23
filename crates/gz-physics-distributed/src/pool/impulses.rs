// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::physics::validate_impulse;

impl WorkerPool {
    /// Queue a world-space linear impulse at the center of mass for the next
    /// committed step. Each acknowledged call represents a distinct input.
    pub fn queue_impulse(&mut self, id: u64, impulse: [f64; 3]) -> Result<&PoolFrame, Error> {
        self.queue_impulse_in_gravity(id, impulse, [0.; 3])
    }
    pub fn queue_impulse_in_gravity(
        &mut self,
        id: u64,
        impulse: [f64; 3],
        gravity: [f64; 3],
    ) -> Result<&PoolFrame, Error> {
        self.queue_impulse_in_field(id, impulse, gravity, &[])
    }
    /// Admit an impulse alongside the field already intended for the next
    /// barrier. The fields themselves are held by the caller until advance.
    pub fn queue_impulse_in_field(
        &mut self,
        id: u64,
        impulse: [f64; 3],
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<&PoolFrame, Error> {
        let (owner, _) = self
            .frame
            .body(id)
            .ok_or(Error::Invalid("unknown impulse body"))?;
        let w = owner as usize;
        let resident = &self.frame.workers[w];
        let local: Vec<_> = self
            .queued
            .iter()
            .filter(|p| resident.generations.contains_key(&p.id))
            .cloned()
            .collect();
        validate_impulse(&resident.snapshot.bodies, &local, id, impulse)
            .map_err(|_| Error::Invalid("impulse exceeds admitted input bounds"))?;
        let mut proposed = self.queued.clone();
        proposed.push(PendingImpulse { id, impulse });
        let predicted = self.routes(gravity, &proposed, inputs)?;
        if self
            .command_counts(&predicted)
            .iter()
            .any(|&n| n > self.limit)
        {
            return Err(Error::Capacity);
        }
        let mut budget = vec![0; self.peers.len()];
        budget[w] = 1;
        self.reserve(&budget)?;
        let mut stage = self.stage()?;
        let tick = stage.frame.tick;
        let ack = self.exchange(&mut stage, w, Command::QueueImpulse { tick, id, impulse })?;
        if ack != Outcome::State(stage.frame.workers[w].clone()) {
            return Err(Error::Invalid("queuing impulse changed public state"));
        }
        stage.queued = proposed;
        self.commit(stage)
    }
    /// Immediately update a dynamic body's world twist using its resident
    /// native inverse mass and rotated inverse inertia.
    pub fn apply_impulse(
        &mut self,
        id: u64,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<&PoolFrame, Error> {
        self.apply_impulse_in_field(id, linear, angular, [0.; 3], &[])
    }
    pub fn apply_impulse_in_field(
        &mut self,
        id: u64,
        linear: [f64; 3],
        angular: [f64; 3],
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<&PoolFrame, Error> {
        let (owner, body) = self
            .frame
            .body(id)
            .ok_or(Error::Invalid("unknown impulse body"))?;
        if body.tier != crate::physics::Tier::Dynamic
            || !linear.into_iter().chain(angular).all(f64::is_finite)
        {
            return Err(Error::Invalid("invalid spatial impulse"));
        }
        let w = usize::from(owner);
        let mut stage = self.stage()?;
        let tick = stage.frame.tick;
        let impulse = crate::protocol::SpatialImpulse {
            id,
            linear,
            angular,
        };
        let seq = self.baseline[w]
            .checked_add(self.history[w].entries.len() as u64)
            .ok_or(Error::Capacity)?;
        let preview = match self.peers[w].preview_impulse(seq, tick, impulse.clone()) {
            Ok(value) => value,
            Err(error) => {
                if !matches!(&error, crate::transport::TransportError::Protocol(_)) {
                    self.dirty[w] = true;
                }
                return Err(error.into());
            }
        };
        if preview.id != id
            || !preview
                .linear
                .into_iter()
                .chain(preview.angular)
                .all(f64::is_finite)
        {
            return Err(Error::Invalid("invalid resident impulse preview"));
        }
        let body = stage.frame.workers[w]
            .snapshot
            .bodies
            .iter_mut()
            .find(|b| b.id == id)
            .unwrap();
        body.linear = preview.linear;
        body.angular = preview.angular;
        crate::physics::validate_queued_for(body, &self.queued)
            .map_err(|_| Error::Invalid("spatial impulse invalidates pending input"))?;
        let moves = self.routes_in(&stage.frame, gravity, &self.queued, inputs)?;
        if self
            .command_counts(&moves)
            .iter()
            .any(|&count| count > self.limit)
        {
            return Err(Error::Capacity);
        }
        let mut counts = vec![0; self.peers.len()];
        counts[w] = 1;
        self.reserve(&counts)?;
        let ack = self.exchange(&mut stage, w, Command::ImpulseNow { tick, impulse })?;
        if ack != Outcome::State(stage.frame.workers[w].clone()) {
            return Err(Error::Invalid(
                "spatial impulse disagrees with native prediction",
            ));
        }
        self.commit(stage)
    }
}
