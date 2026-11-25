// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded subdivision of an unresolved supported shell on its prescribed clock.
use super::{BoundaryImpulse, PreparedBoundary, SoftBody, XpbdError, substep_interval};
use std::ops::Range;

pub(super) struct ShellRetry {
    pub remaining_splits: u32,
    pub deepest: u32,
}

impl ShellRetry {
    pub fn advance(
        &mut self,
        body: &mut SoftBody,
        boundaries: &mut [PreparedBoundary],
        reactions: &mut [Vec<BoundaryImpulse>],
        interval: Range<u128>,
        iterations: u32,
        depth: u32,
    ) -> Result<(), XpbdError> {
        // A failed substep may already have accumulated jaw reactions, updated
        // pad loads or projected one panel. Both children need the same clock
        // and the first child must see the exact input of its parent.
        let saved = (body.clone(), boundaries.to_vec(), reactions.to_vec());
        let width = interval.end - interval.start;
        body.surface.as_mut().unwrap().subdivide_on_budget = false;
        let error = match substep_interval(
            &mut [body],
            boundaries,
            reactions,
            interval.start,
            width,
            iterations,
            false,
        ) {
            Ok(()) => {
                self.deepest = self.deepest.max(depth);
                return Ok(());
            }
            Err(error) => error,
        };
        let splittable = error == XpbdError::UnresolvedSurface
            || (error == XpbdError::SurfaceBudget
                && body.surface.as_ref().unwrap().subdivide_on_budget);
        if !splittable || width < 2 || self.remaining_splits == 0 {
            return Err(error);
        }
        self.remaining_splits -= 1;
        *body = saved.0;
        boundaries.clone_from_slice(&saved.1);
        reactions.clone_from_slice(&saved.2);
        let middle = interval.start + width.div_ceil(2);
        self.advance(
            body,
            boundaries,
            reactions,
            interval.start..middle,
            iterations,
            depth + 1,
        )?;
        self.advance(
            body,
            boundaries,
            reactions,
            middle..interval.end,
            iterations,
            depth + 1,
        )
    }
}
