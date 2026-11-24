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

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn release_window_preserves_original_boundaries_and_step_limits() {
        let ns = Duration::from_nanos;
        let window = EarlyFace {
            step_ns: 25,
            max_steps: 100,
            tail: Some((200, 13)),
        };
        let prefix = ContactSegment {
            duration: ns(400),
            native_steps: 16,
            iterations: 4.into(),
        };
        let rest = ContactSegment {
            duration: ns(400),
            native_steps: 4,
            iterations: 8.into(),
        };
        let (s, fallback) = window.after_release([prefix, rest], ns(45)).unwrap();
        assert_eq!(s[0].duration, ns(75));
        assert_eq!((s[0].native_steps, s[1].native_steps, fallback), (3, 4, 13));
        assert_eq!(s[1].iterations, prefix.iterations);
        assert_eq!(s[2], rest);
        for bad in [
            ContactSegment {
                duration: ns(399),
                ..prefix
            },
            prefix,
        ] {
            let arrival = if bad == prefix { ns(375) } else { ns(0) };
            assert!(window.after_release([bad, rest], arrival).is_none());
        }
        assert!(window.after_release([prefix, rest], ns(350)).is_none());
        assert!(
            EarlyFace {
                tail: None,
                ..window
            }
            .after_release([prefix, rest], ns(45))
            .is_none()
        );
    }
}
