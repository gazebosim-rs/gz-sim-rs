// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_multifidelity::{ContactSegment, ContactSolverIterations};
use std::time::Duration;

#[derive(Clone, Copy)]
pub(super) struct EarlyFace {
    pub step_ns: u32,
    pub max_steps: u32,
    pub tail: Option<(u32, u32)>,
}

impl EarlyFace {
    /// Keep the fine prefix anchored to the incoming state. A separate native
    /// check at this boundary decides whether to spend fewer calls afterwards.
    pub(super) fn after_release(
        self,
        original: [ContactSegment; 2],
        arrival: Duration,
    ) -> Option<([ContactSegment; 3], u32)> {
        let (post, guard) = self.tail?;
        let step = u128::from(self.step_ns);
        let [prefix, rest] = original;
        let post = u128::from(post).min(rest.duration.as_nanos() / u128::from(rest.native_steps));
        let total = prefix.duration.as_nanos();
        // Only split exact fine-grid intervals so the pre-impact trajectory and
        // the fallback preserve all the original native times and solver settings.
        if total != u128::from(prefix.native_steps) * step {
            return None;
        }
        let cut = (arrival.as_nanos() + u128::from(guard)).div_ceil(step) * step;
        if cut >= total {
            return None;
        }
        let suffix = total - cut;
        let coarse = suffix.div_ceil(post) as u32;
        let fallback = (suffix / step) as u32;
        if coarse >= fallback {
            return None;
        }
        let duration = Duration::new((cut / 1_000_000_000) as u64, (cut % 1_000_000_000) as u32);
        Some((
            [
                ContactSegment {
                    duration,
                    native_steps: (cut / step) as u32,
                    ..prefix
                },
                ContactSegment {
                    duration: prefix.duration - duration,
                    native_steps: coarse,
                    ..prefix
                },
                rest,
            ],
            fallback,
        ))
    }

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
