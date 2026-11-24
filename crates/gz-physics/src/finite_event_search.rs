// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Locate a previously certified, strictly closing margin on the same dyadic
//! time grid as the reference search. Geometry and force still decide both ends.
use super::*;

/// Recreate a time from the rounded midpoints of the reference bisection.
fn grid_time(h: f64, depth: u32, index: u64) -> f64 {
    if index == 0 {
        return 0.;
    }
    if index == 1 << depth {
        return h;
    }
    let (mut lo, mut hi) = (0., h);
    // A dyadic fraction ends at the last nonzero bit of its numerator.
    for bit in (index.trailing_zeros()..depth).rev() {
        let mid = (lo + hi) * 0.5;
        if bit == index.trailing_zeros() {
            return mid;
        }
        if index & (1 << bit) == 0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    unreachable!()
}

impl FiniteFeatureArc {
    pub(super) fn monotone_bracket(
        self,
        y: State,
        h: f64,
        width: f64,
        limit: FeatureLimit,
        last: State,
    ) -> Option<(f64, f64, State)> {
        let distance = |state| {
            self.frame(state)?
                .1
                .margins()
                .iter()
                .find(|m| m.limit == limit)
                .map(|m| (m.distance, m.speed))
        };
        let mut depth = 0;
        let mut cell = h;
        while cell >= width && depth < 50 {
            cell *= 0.5;
            depth += 1;
        }
        if depth == 0 || cell >= width {
            return None;
        }
        let cells = 1_u64 << depth;
        let (mut lo, mut hi) = (0., h);
        let (mut a, mut b) = (distance(y)?, distance(last)?);
        if a.0 <= 0. || b.0 > 0. {
            return None;
        }
        let mut t = h * (a.0 / (a.0 - b.0));
        for _ in 0..8 {
            if !t.is_finite() || t <= lo || t >= hi {
                t = (lo + hi) * 0.5;
            }
            let (d, rate) = distance(self.step(y, t)?)?;
            if rate >= 0. || !rate.is_finite() {
                return None;
            }
            let extrapolated = t - d / rate;
            if (extrapolated - t).abs() < 4. * cell {
                let i = ((extrapolated / cell).floor() as u64).min(cells - 1);
                // A signed distance near zero may round in either direction.
                for j in [
                    Some(i),
                    i.checked_sub(1),
                    i.checked_add(1).filter(|x| *x < cells),
                ]
                .into_iter()
                .flatten()
                {
                    let l = grid_time(h, depth, j);
                    let r = grid_time(h, depth, j + 1);
                    // On a rounded time grid, the parent width can be just
                    // below tolerance even when twice the ideal cell is not.
                    // The ordinary search would already have returned there.
                    let parent = j / 2;
                    let parent_width =
                        grid_time(h, depth - 1, parent + 1) - grid_time(h, depth - 1, parent);
                    if r - l >= width || parent_width < width {
                        continue;
                    }
                    let admitted = self.step(y, l)?;
                    if self.boundary(admitted) == FiniteArcEnd::Horizon
                        && self.boundary(self.step(y, r)?) == FiniteArcEnd::Feature(limit)
                    {
                        return Some((l, r, admitted));
                    }
                }
                return None;
            }
            if d > 0. {
                lo = t;
                a = (d, rate);
            } else {
                hi = t;
                b = (d, rate);
            }
            t = if extrapolated > lo && extrapolated < hi {
                extrapolated
            } else {
                lo + (hi - lo) * (a.0 / (a.0 - b.0))
            };
        }
        None
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn dyadic_grid_reconstructs_every_rounded_reference_midpoint() {
        for h in [1.0e-7, 0.000031416, 0.010009871] {
            let mut old = vec![0., h];
            for depth in 1..=10 {
                let mut new = vec![0.];
                for w in old.windows(2) {
                    new.push((w[0] + w[1]) * 0.5);
                    new.push(w[1]);
                }
                for (i, &value) in new.iter().enumerate() {
                    assert_eq!(grid_time(h, depth, i as u64), value);
                }
                old = new;
            }
        }
    }
}
