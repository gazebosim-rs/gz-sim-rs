// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded subdivision of an unresolved supported shell on its prescribed clock.
use super::{
    Boundary, BoundaryImpulse, PreparedBoundary, ReceiverMotion, RigidMobility, SoftBody,
    XpbdError, solve_at_resolution, substep_interval,
};
use crate::surface::RowOrder;
use std::ops::Range;
use std::time::Duration;

pub(super) struct ShellRetry {
    pub remaining_splits: u32,
    pub deepest: u32,
    pub accepted: u32,
}

fn splittable(error: &XpbdError, body: &SoftBody) -> bool {
    *error == XpbdError::UnresolvedSurface
        || (*error == XpbdError::SurfaceBudget
            && body.surface.as_ref().unwrap().subdivide_on_budget)
}

pub(super) fn solve<M: Copy + Into<RigidMobility>>(
    bodies: &mut [&mut SoftBody],
    dt: Duration,
    boundaries: &[Boundary],
    mobility: &[Option<M>],
    count: u32,
    maximum: u32,
) -> Result<Vec<Option<ReceiverMotion>>, XpbdError> {
    let surface = bodies[0].surface.as_mut().unwrap();
    surface.retry_plane = true;
    surface.row_order = RowOrder::CertifiedSpeed;
    surface.subdivide_on_budget = false;
    // First try changing only the order of the certified velocity constraints.
    let mut result = solve_at_resolution(bodies, dt, boundaries, mobility, count.into(), None);
    let mut depth = 0;
    let mut accepted = None;
    // If a finer base cannot resolve the tick within its budget, retain the
    // original coarse-grid subdivision and terminal-only ordering as a fallback.
    for (base, order) in [
        (
            count.saturating_mul(2).min(maximum),
            RowOrder::CertifiedSpeed,
        ),
        (count, RowOrder::TerminalSpeed),
    ] {
        if !result.as_ref().is_err_and(|e| splittable(e, bodies[0])) {
            break;
        }
        bodies[0].surface.as_mut().unwrap().row_order = order;
        let mut retry = ShellRetry {
            remaining_splits: maximum - base,
            deepest: 0,
            accepted: 0,
        };
        result = solve_at_resolution(
            bodies,
            dt,
            boundaries,
            mobility,
            base.into(),
            Some(&mut retry),
        );
        accepted = Some(retry.accepted);
        depth = retry.deepest + u32::from(base > count);
    }
    let surface = bodies[0].surface.as_mut().unwrap();
    surface.retry_plane = false;
    surface.row_order = RowOrder::Time;
    if result.is_ok() {
        surface.local_intervals = accepted;
        bodies[0].sweep_refinements = depth;
    }
    result
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
                self.accepted += 1;
                self.deepest = self.deepest.max(depth);
                return Ok(());
            }
            Err(error) => error,
        };
        if !splittable(&error, body) || width < 2 || self.remaining_splits == 0 {
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

#[cfg(test)]
#[path = "../../tests/private/fold_slide_cut.rs"]
mod fold_slide_cut;
