// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same-tick edits of free, independently owned native bodies.
use super::*;
use crate::{
    physics::{WorldError, admit_edit},
    protocol::VelocityEdit,
};
use gz_physics::{BodyState, PhysicsError};

impl WorkerPool {
    /// Validate both the destination pose and the next barrier without a worker
    /// exchange. The caller supplies the gravity and unconsumed held field.
    pub fn validate_state_in_field(
        &self,
        id: u64,
        state: BodyState,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<(), Error> {
        self.project_edit(id, state, gravity, inputs).map(|_| ())
    }

    /// Replace the current world pose and twist while retaining ownership,
    /// pending impulses and the initial reset seed. This is an instantaneous
    /// pose edit; the new twist determines the subsequent swept motion.
    pub fn replace_state_in_field(
        &mut self,
        id: u64,
        state: BodyState,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<&PoolFrame, Error> {
        let (owner, body) = self.project_edit(id, state, gravity, inputs)?;
        let command = Command::ReplaceBody {
            tick: self.frame.tick,
            body: body.clone(),
        };
        self.publish_edit(owner, body, command)
    }

    /// Replace only the world-space linear and angular velocity of a free
    /// dynamic or prescribed body at the current published tick.
    pub fn set_velocity_in_field(
        &mut self,
        id: u64,
        linear: [f64; 3],
        angular: [f64; 3],
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<&PoolFrame, Error> {
        let (_, current) = self
            .frame
            .body(id)
            .ok_or(Error::Invalid("unknown edit body"))?;
        let state = BodyState {
            velocity: linear,
            angular_velocity: angular,
            ..current.state()
        };
        let (owner, body) = self.project_edit(id, state, gravity, inputs)?;
        let command = Command::SetVelocity {
            tick: self.frame.tick,
            edit: VelocityEdit {
                id,
                linear,
                angular,
            },
        };
        self.publish_edit(owner, body, command)
    }

    fn project_edit(
        &self,
        id: u64,
        state: BodyState,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<(usize, Body), Error> {
        let (owner, old) = self
            .frame
            .body(id)
            .ok_or(Error::Invalid("unknown edit body"))?;
        let owner = usize::from(owner);
        let component = self.frame.workers[owner]
            .components
            .iter()
            .find(|c| c.ids.binary_search(&id).is_ok())
            .ok_or(Error::Invalid("edit body missing from native topology"))?;
        component.admit_free_edit(old.tier).map_err(|e| match e {
            PhysicsError::TransitionDeferred { reason } => Error::EditDeferred(reason),
            _ => Error::Invalid("pool edits require a moving body"),
        })?;
        let body = old.with_state(state);
        admit_edit(
            old,
            &body,
            self.frame.workers.iter().flat_map(|w| &w.snapshot.bodies),
            &self.queued,
            self.frame.step().duration(),
            self.frame.ground(),
        )
        .map_err(|error| match error {
            WorldError::Physics(PhysicsError::TransitionDeferred { reason }) => {
                Error::EditDeferred(reason)
            }
            _ => Error::Invalid("invalid body-state edit"),
        })?;
        let mut projected = self.frame.clone();
        let residents = &mut projected.workers[owner].snapshot.bodies;
        let index = residents
            .binary_search_by_key(&id, |b| b.id)
            .expect("body found in committed frame");
        residents[index] = body.clone();
        let moves = self.routes_in(&projected, gravity, &self.queued, inputs)?;
        if self.command_counts(&moves).iter().any(|&n| n > self.limit) {
            return Err(Error::Capacity);
        }
        Ok((owner, body))
    }

    fn publish_edit(
        &mut self,
        owner: usize,
        body: Body,
        command: Command,
    ) -> Result<&PoolFrame, Error> {
        let mut counts = vec![0; self.peers.len()];
        counts[owner] = 1;
        self.reserve(&counts)?;
        let mut stage = self.stage()?;
        let reply = self.exchange(&mut stage, owner, command)?;
        stage.frame.accept_edit(owner, body, reply)?;
        self.commit(stage)
    }
}

impl PoolFrame {
    pub(super) fn accept_edit(
        &mut self,
        owner: usize,
        mut body: Body,
        reply: Outcome,
    ) -> Result<(), Error> {
        let Outcome::State(actual) = reply else {
            return Err(Error::Invalid(
                "state-edit acknowledgement missing observation",
            ));
        };
        let expected = &mut self.workers[owner];
        let index = expected
            .snapshot
            .bodies
            .binary_search_by_key(&body.id, |b| b.id)
            .expect("edit retains owner");
        let old = &expected.snapshot.bodies[index];
        let observed = actual.snapshot.bodies.get(index).ok_or(Error::Invalid(
            "state-edit acknowledgement missing edited body",
        ))?;
        // Rapier's setters retain the entire existing pose or velocity vector
        // when the requested value compares equal (including signed zero).
        // set_position may also update its pending pose. Accept either exact
        // whole pose in that case, while still checking every unrelated bit.
        if old.position == body.position
            && old.orientation == body.orientation
            && observed.position.map(f64::to_bits) == old.position.map(f64::to_bits)
            && observed.orientation.map(f64::to_bits) == old.orientation.map(f64::to_bits)
        {
            body.position = old.position;
            body.orientation = old.orientation;
        }
        if old.linear == body.linear {
            body.linear = old.linear;
        }
        if old.angular == body.angular {
            body.angular = old.angular;
        }
        let previous = std::mem::replace(&mut expected.snapshot.bodies[index], body);
        if actual != *expected {
            expected.snapshot.bodies[index] = previous;
            return Err(Error::Invalid(
                "state-edit acknowledgement differs from proposed world state",
            ));
        }
        Ok(())
    }
}
