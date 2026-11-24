// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_multifidelity::{ContactSegment, ContactSolverIterations};
use std::time::Duration;

#[derive(Clone, Copy)]
pub(super) struct EarlyFace {
    pub step_ns: u32,
    pub max_steps: u32,
}

impl EarlyFace {
    /// Advance the incoming state with the ordinary solver until one full original grid
    /// interval beyond the predicted crossing cell. This guards against small changes in
    /// approach speed as the mechanism advances. All later contacts still use the native
    /// solver. A late arrival, or a prefix that would coarsen the clock, retains the old grid.
    pub(super) fn schedule(
        self,
        dt: Duration,
        arrival: Duration,
        calls: u32,
        impact: ContactSolverIterations,
        ordinary: ContactSolverIterations,
    ) -> Option<[ContactSegment; 2]> {
        let tick = dt.as_nanos();
        let count = u128::from(calls);
        let prefix_cells = (arrival.as_nanos() * count).div_ceil(tick) + 1;
        if prefix_cells >= count {
            return None;
        }
        let prefix_ns = prefix_cells * tick / count;
        let fine_calls = prefix_ns.div_ceil(u128::from(self.step_ns));
        let tail_calls = calls - prefix_cells as u32;
        if fine_calls <= prefix_cells
            || fine_calls > u128::from(self.max_steps.min(u32::MAX - tail_calls))
        {
            return None;
        }
        let prefix = Duration::new(
            (prefix_ns / 1_000_000_000) as u64,
            (prefix_ns % 1_000_000_000) as u32,
        );
        Some([
            ContactSegment {
                duration: prefix,
                native_steps: fine_calls as u32,
                iterations: ordinary,
            },
            ContactSegment {
                duration: dt - prefix,
                native_steps: tail_calls,
                iterations: impact,
            },
        ])
    }
}
