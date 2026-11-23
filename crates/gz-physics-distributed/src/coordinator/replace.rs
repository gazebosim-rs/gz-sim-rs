// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::physics::admit_edit;
use gz_physics::BodyState;

impl<P: Peer> Coordinator<P> {
    pub(crate) fn validate_replacement(&self, body: &Body) -> Result<(), Error> {
        let (_, old) = self
            .frame
            .body(body.id)
            .ok_or(Error::Invalid("unknown state-edit body"))?;
        admit_edit(
            old,
            body,
            self.frame.workers.iter().flat_map(|w| &w.snapshot.bodies),
            &self.queued,
            self.frame.step().duration(),
            self.frame.ground(),
        )
        .map_err(|_| Error::Invalid("invalid or intersecting body-state edit"))
    }

    /// Replace the world pose and twist on the current authoritative body.
    /// Moving into another box's prediction bounds is refused before mutation.
    pub fn replace_state(&mut self, id: u64, state: BodyState) -> Result<&Frame, Error> {
        let (w, old) = self
            .frame
            .body(id)
            .ok_or(Error::Invalid("unknown state-edit body"))?;
        let w = usize::from(w);
        let body = old.with_state(state);
        self.validate_replacement(&body)?;
        let mut expected = self.frame.workers[w].clone();
        *expected
            .snapshot
            .bodies
            .iter_mut()
            .find(|b| b.id == id)
            .unwrap() = body.clone();
        let mut counts = [0; 2];
        counts[w] = 1;
        let mut cursor = self.begin(Action::Replace(body.clone()), counts)?;
        let obs = observation(self.call(
            &mut cursor,
            w,
            Command::ReplaceBody {
                tick: self.frame.tick,
                body,
            },
        )?)?;
        if obs != expected {
            return Err(Error::Invalid(
                "state edit changed unrelated state or contact history",
            ));
        }
        self.frame.workers[w] = obs;
        self.commit_action();
        Ok(&self.frame)
    }
}
