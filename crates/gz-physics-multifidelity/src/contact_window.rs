// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Contact timing for a translating sole and a flat support with constant relative normal acceleration.
use super::{ContactSegment, ContactSolverIterations};
use gz_physics::PhysicsError;
use std::time::Duration;

/// Enlarge each predicted touch by these margins and coalesce intersecting windows.
#[derive(Clone, Copy, Debug)]
pub struct ContactWindow {
    pub before: Duration,
    pub after: Duration,
    /// Upper bound on the duration of a native solve inside a window.
    pub max_native_dt: Duration,
    pub ordinary: ContactSolverIterations,
    pub refined: ContactSolverIterations,
}
impl ContactWindow {
    /// Each ordinary gap receives one native solve. Native time adds to the
    /// original public interval exactly, even for overlapping windows.
    pub fn schedule(
        self,
        dt: Duration,
        events: &[Duration],
    ) -> Result<Vec<ContactSegment>, PhysicsError> {
        if events.iter().any(|&e| e > dt) {
            return Err(PhysicsError::InvalidValue);
        }
        self.schedule_at(Duration::ZERO, dt, events)
    }

    /// Plan one actuator tick on an absolute simulation clock. Known event
    /// windows can overlap either boundary. Retain their timestamps while the
    /// same approaching pair and commands remain valid.
    pub fn schedule_at(
        self,
        start: Duration,
        dt: Duration,
        events: &[Duration],
    ) -> Result<Vec<ContactSegment>, PhysicsError> {
        let end = start.checked_add(dt).ok_or(PhysicsError::InvalidValue)?;
        if self.max_native_dt.is_zero() || (self.before.is_zero() && self.after.is_zero()) {
            return Err(PhysicsError::InvalidValue);
        }
        let mut windows: Vec<_> = events
            .iter()
            .filter_map(|&t| {
                let lo = t.saturating_sub(self.before).max(start);
                let hi = t.saturating_add(self.after).min(end);
                (hi > lo).then(|| (lo - start, hi - start))
            })
            .collect();
        windows.sort_unstable();
        let mut union: Vec<(Duration, Duration)> = Vec::new();
        for (start, end) in windows {
            if start == end {
                continue;
            }
            if let Some(last) = union.last_mut()
                && start <= last.1
            {
                last.1 = last.1.max(end);
            } else {
                union.push((start, end));
            }
        }
        let mut plan = Vec::new();
        let mut cursor = Duration::ZERO;
        for (start, end) in union {
            if start > cursor {
                plan.push(ContactSegment {
                    duration: start - cursor,
                    native_steps: 1,
                    iterations: self.ordinary,
                });
            }
            let duration = end - start;
            let calls = duration.as_nanos().div_ceil(self.max_native_dt.as_nanos());
            plan.push(ContactSegment {
                duration,
                native_steps: u32::try_from(calls).map_err(|_| PhysicsError::InvalidValue)?,
                iterations: self.refined,
            });
            cursor = end;
        }
        if dt > cursor {
            plan.push(ContactSegment {
                duration: dt - cursor,
                native_steps: 1,
                iterations: self.ordinary,
            });
        }
        Ok(plan)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gz_physics::NormalApproach;
    #[test]
    fn absolute_window_spans_ticks_without_restarting_or_advancing_the_actuator() {
        let n = Duration::from_nanos;
        let w = ContactWindow {
            before: n(4),
            after: n(8),
            max_native_dt: n(3),
            ordinary: 1.into(),
            refined: 4.into(),
        };
        let events = [n(9), n(15), n(18)];
        let run = |s| {
            w.schedule_at(n(s), n(10), &events)
                .unwrap()
                .into_iter()
                .map(|q| (q.duration.as_nanos(), q.native_steps, q.iterations.temporal))
                .collect::<Vec<_>>()
        };
        assert_eq!(run(0), [(5, 1, 1), (5, 2, 4)]);
        assert_eq!(run(10), [(10, 4, 4)]);
        assert_eq!(run(20), [(6, 2, 4), (4, 1, 1)]);
        assert_eq!(run(30), [(10, 1, 1)]);
        assert!(
            w.schedule_at(n(10), Duration::ZERO, &events)
                .unwrap()
                .is_empty()
        );
        assert!(w.schedule_at(Duration::MAX, n(1), &[]).is_err());
    }
    #[test]
    fn predicts_first_approaching_root_with_acceleration_or_deceleration() {
        for (c, a, t) in [
            (1., 0., 0.25),
            (1., -1., 0.25),
            (-1., 4., 1.),
            (1e4, 1e-12, 0.01),
            (0., 4., 0.5),
        ] {
            let n = NormalApproach {
                gap: c * t + 0.5 * a * t * t,
                closing_speed: c,
                closing_acceleration: a,
            };
            let dt = Duration::from_secs(2);
            assert_eq!(n.touchdown(dt).unwrap(), Some(Duration::from_secs_f64(t)));
            assert_eq!(
                n.touchdown(Duration::from_secs_f64(t * 0.99)).unwrap(),
                None
            );
        }
        let miss = NormalApproach {
            gap: 2.,
            closing_speed: 1.,
            closing_acceleration: -2.,
        };
        assert_eq!(miss.touchdown(Duration::from_secs(10)).unwrap(), None);
        assert!(
            NormalApproach {
                gap: f64::NAN,
                ..miss
            }
            .touchdown(Duration::ZERO)
            .is_err()
        );
    }
    #[test]
    fn overlapping_unsorted_clipped_windows_partition_the_public_clock_exactly() {
        let n = Duration::from_nanos;
        let p = ContactWindow {
            before: n(6),
            after: n(7),
            max_native_dt: n(4),
            ordinary: 2.into(),
            refined: 8.into(),
        };
        for (events, expected) in [
            (
                vec![30, 0, 10, 99, 33],
                vec![(17, 5, 8), (7, 1, 2), (16, 4, 8), (53, 1, 2), (7, 2, 8)],
            ),
            (vec![], vec![(100, 1, 2)]),
            (vec![100], vec![(94, 1, 2), (6, 2, 8)]),
        ] {
            let events = events.into_iter().map(n).collect::<Vec<_>>();
            let r = p.schedule(n(100), &events).unwrap();
            assert_eq!(
                r.iter()
                    .map(|s| (s.duration.as_nanos(), s.native_steps, s.iterations.temporal))
                    .collect::<Vec<_>>(),
                expected
            );
            assert_eq!(r.iter().map(|s| s.duration).sum::<Duration>(), n(100));
        }
        assert!(p.schedule(n(100), &[n(101)]).is_err());
        assert!(p.schedule(Duration::ZERO, &[]).unwrap().is_empty());
    }
}
