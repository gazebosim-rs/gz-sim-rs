// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Consecutive native intervals with zero measured compressive load.
use serde::Serialize;

#[derive(Debug, Default, Serialize)]
pub(crate) struct ContactWindows {
    /// Left, right and simultaneous loss on both fingers.
    pub(super) unloaded: [Unloaded; 3],
    #[serde(skip)]
    previous: Option<[f64; 2]>,
}

#[derive(Debug, Default, Serialize)]
pub(super) struct Unloaded {
    episodes: u32,
    total_ns: u64,
    longest_ns: u64,
    first: Option<Opening>,
    #[serde(skip)]
    current_ns: u64,
}

#[derive(Debug, Serialize)]
struct Opening {
    start_ns: u64,
    end_ns: u64,
    previous_normal_n: Option<[f64; 2]>,
    end_normal_n: [f64; 2],
    /// Minimum signed sphere-to-pad distance on its facing panel, measured at the native endpoint.
    end_gap_m: [f64; 2],
}

impl ContactWindows {
    pub(crate) fn observe(&mut self, end_ns: u64, dt_ns: u64, normal: [f64; 2], gap: [f64; 2]) {
        let [left, right] = normal.map(|n| n == 0.);
        for (lost, record) in [left, right, left && right]
            .into_iter()
            .zip(&mut self.unloaded)
        {
            if lost {
                if record.current_ns == 0 {
                    record.episodes += 1;
                    record.first.get_or_insert(Opening {
                        start_ns: end_ns - dt_ns,
                        end_ns,
                        previous_normal_n: self.previous,
                        end_normal_n: normal,
                        end_gap_m: gap,
                    });
                }
                record.total_ns += dt_ns;
                record.current_ns += dt_ns;
                record.longest_ns = record.longest_ns.max(record.current_ns);
            } else {
                record.current_ns = 0;
            }
        }
        self.previous = Some(normal);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn individual_and_overlapping_openings_use_native_time() {
        let mut w = ContactWindows::default();
        for (i, n) in [[0., 1.], [0., 0.], [0.01, 0.], [0., 0.], [1., 1.]]
            .into_iter()
            .enumerate()
        {
            w.observe(100 + i as u64 * 4, 4, n, [0.001, -0.002]);
        }
        let [left, right, both] = &w.unloaded;
        assert_eq!((left.episodes, left.total_ns, left.longest_ns), (2, 12, 8));
        assert_eq!(
            (right.episodes, right.total_ns, right.longest_ns),
            (1, 12, 12)
        );
        assert_eq!((both.episodes, both.total_ns, both.longest_ns), (2, 8, 4));
        assert!(left.first.as_ref().unwrap().previous_normal_n.is_none());
        let first = both.first.as_ref().unwrap();
        assert_eq!((first.start_ns, first.end_ns), (100, 104));
        assert_eq!(first.previous_normal_n, Some([0., 1.]));
        assert_eq!(first.end_normal_n, [0., 0.]);
        assert_eq!(first.end_gap_m, [0.001, -0.002]);
    }
}
