// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ordered creation of a separated box at a published ownership barrier.
use super::*;
use crate::physics::{ComponentInfo, Island, NativeSolverProfile};
use serde::{Deserialize, Serialize};

/// Native files written before runtime insertion did not track erased IDs.
/// Seed replay reconstructs the allocation history from the original owners.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub(super) enum IssuedIds {
    #[default]
    Untracked,
    Empty,
    Through(u64),
}

impl IssuedIds {
    pub(super) fn from_frame(frame: &Frame) -> Self {
        frame
            .workers
            .iter()
            .flat_map(|w| w.generations.keys())
            .max()
            .map_or(Self::Empty, |&id| Self::Through(id))
    }

    pub(super) fn consistent(self, frame: &Frame) -> bool {
        match (self, Self::from_frame(frame)) {
            (Self::Untracked, _) | (_, Self::Empty) => true,
            (Self::Through(a), Self::Through(b)) => a >= b,
            _ => false,
        }
    }
}

/// Check the actual primitive geometry and cross-owner clearance at this barrier.
pub(crate) fn validate_separated_birth<'a>(
    body: &Body,
    at: &crate::physics::Snapshot,
    residents: impl Iterator<Item = &'a Body>,
    profile: NativeSolverProfile,
) -> Result<(), Error> {
    let singleton = crate::physics::Snapshot {
        tick: at.tick,
        step: at.step,
        ground: at.ground,
        bodies: vec![body.clone()],
        ..Default::default()
    };
    Island::restore_with_profile(&singleton, profile).map_err(|_| Error::Invalid("invalid primitive construction"))?;
    let bounds = |b: &Body| b.bounds().map_err(|_| Error::Invalid("invalid primitive bounds"));
    let incoming = bounds(body)?;
    for b in residents {
        if incoming.predicts_overlap(&bounds(b)?) {
            return Err(Error::Invalid(
                "inserted primitive intersects resident contact prediction bounds",
            ));
        }
    }
    Ok(())
}

impl<P: Peer> Coordinator<P> {
    /// Create a new global ID at the current tick. IDs increase across retirement
    /// and durable recovery. Both partitions are checked for initial overlap.
    pub fn insert_body(&mut self, body: Body) -> Result<&Frame, Error> {
        self.insert_checked(body, None)
    }

    pub(crate) fn insert_checked(
        &mut self,
        body: Body,
        field: Option<([f64; 3], &[crate::physics::BodyWrench])>,
    ) -> Result<&Frame, Error> {
        match self.issued {
            IssuedIds::Untracked => {
                return Err(Error::Invalid("native baseline predates the ID allocator"));
            }
            IssuedIds::Through(last) if body.id <= last => {
                return Err(Error::Invalid(
                    "new body ID must exceed all previously issued IDs",
                ));
            }
            _ => {}
        }
        validate_separated_birth(
            &body,
            &self.frame.workers[0].snapshot,
            self.frame.workers.iter().flat_map(|w| &w.snapshot.bodies),
            NativeSolverProfile::Standard32,
        )?;
        let c = ComponentInfo {
            ids: vec![body.id],
            contact_history: false,
        };
        let mut destinations = [0, 1];
        destinations.sort_by_key(|&i| self.frame.workers[i].generations.len());
        let mut room = false;
        for w in destinations {
            if self.frame.known_ids(w) >= MAX_BODY_IDS {
                continue;
            }
            room = true;
            let mut expected = self.frame.workers[w].clone();
            expected.snapshot.bodies.push(body.clone());
            expected.components.push(c.clone());
            expected.generations.insert(body.id, 0);
            if !partition_can_step(&expected.snapshot, &expected.components, &self.queued) {
                continue;
            }
            if let Some((gravity, inputs)) = field {
                self.validate_edited_owner(w, expected.clone(), gravity, inputs)?;
            }
            let mut counts = [0; 2];
            counts[w] = 1;
            let mut cursor = self.begin(Action::Insert(body.clone()), counts)?;
            let obs = observation(self.call(
                &mut cursor,
                w,
                Command::InsertBody {
                    tick: self.frame.tick,
                    body: body.clone(),
                },
            )?)?;
            if obs != expected {
                return Err(Error::Invalid("insertion changed committed state"));
            }
            self.frame.workers[w] = obs;
            self.issued = IssuedIds::Through(body.id);
            self.commit_action();
            return Ok(&self.frame);
        }
        Err(if room {
            Error::Colocation
        } else {
            Error::Capacity
        })
    }
}
