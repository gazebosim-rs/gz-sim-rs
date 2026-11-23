// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Candidate discovery for distributed ownership. The native solver still owns contact geometry.
use super::{Motion, Tier, partition_prediction_distance};

struct Envelope {
    lo: [f64; 3],
    hi: [f64; 3],
}
impl Envelope {
    /// Enclose both free flight and the full reachable domain after a potential
    /// secondary deflection. The candidate set stays valid as deflections propagate.
    fn of(m: &Motion<'_>) -> Self {
        let reach = if m.body.tier == Tier::Dynamic {
            m.max_travel
        } else {
            0.
        };
        let mut out = Self {
            lo: [0.; 3],
            hi: [0.; 3],
        };
        for axis in 0..3 {
            let x = m.body.position[axis];
            let dx = m.displacement[axis];
            let a = m.acceleration_travel[axis];
            let radius = if m.body.tier == Tier::Dynamic {
                m.radius
            } else {
                m.free_axes[axis]
            } + partition_prediction_distance();
            // This per-body allowance dominates the pairwise roundoff budget in overlaps().
            let slack = 128. * f64::EPSILON * (1. + x.abs() + dx.abs() + a.abs() + radius + reach);
            let lo = x + (dx.min(0.) + a.min(0.)).min(-reach) - radius - slack;
            let hi = x + (dx.max(0.) + a.max(0.)).max(reach) + radius + slack;
            if lo.is_finite() && hi.is_finite() {
                out.lo[axis] = lo;
                out.hi[axis] = hi;
            } else {
                out.lo[axis] = f64::NEG_INFINITY;
                out.hi[axis] = f64::INFINITY;
            }
        }
        out
    }
    fn intersects(&self, b: &Self) -> bool {
        (0..3).all(|k| self.lo[k] <= b.hi[k] && b.lo[k] <= self.hi[k])
    }
}

/// Keep pair visitation in input order for identical fixed-point propagation.
/// Dense scenes use direct traversal with constant extra memory.
pub(super) enum Candidates {
    All(usize),
    Sparse(Vec<(usize, usize)>),
}

impl Candidates {
    pub(super) fn new(entries: &[Motion<'_>]) -> Self {
        let n = entries.len();
        if n <= 32 {
            return Self::All(n);
        }
        let boxes: Vec<_> = entries.iter().map(Envelope::of).collect();
        let score = |axis: usize| {
            let (mut lo, mut hi, mut widths) = (f64::INFINITY, f64::NEG_INFINITY, 0.);
            for b in &boxes {
                lo = lo.min(b.lo[axis]);
                hi = hi.max(b.hi[axis]);
                widths += b.hi[axis] - b.lo[axis];
            }
            if widths.is_finite() && widths > 0. {
                (hi - lo) / widths
            } else {
                0.
            }
        };
        // Choose the axis covering the most scene span per total interval width.
        let axis = (0..3)
            .map(|a| (a, score(a)))
            .max_by(|(i, a), (j, b)| a.total_cmp(b).then(j.cmp(i)))
            .unwrap()
            .0;
        let mut order: Vec<_> = (0..n).collect();
        order.sort_unstable_by(|&a, &b| {
            boxes[a].lo[axis]
                .total_cmp(&boxes[b].lo[axis])
                .then(a.cmp(&b))
        });
        let mut pairs = Vec::new();
        let budget = n.saturating_mul(12);
        for (rank, &a) in order.iter().enumerate() {
            for &b in &order[rank + 1..] {
                if boxes[b].lo[axis] > boxes[a].hi[axis] {
                    break;
                }
                if boxes[a].intersects(&boxes[b]) {
                    pairs.push((a.min(b), a.max(b)));
                    if pairs.len() > budget {
                        return Self::All(n);
                    }
                }
            }
        }
        pairs.sort_unstable();
        Self::Sparse(pairs)
    }

    pub(super) fn any(&self, mut f: impl FnMut(usize, usize) -> bool) -> bool {
        match self {
            Self::All(n) => (0..*n).any(|a| (a + 1..*n).any(|b| f(a, b))),
            Self::Sparse(pairs) => pairs.iter().any(|&(a, b)| f(a, b)),
        }
    }

    pub(super) fn visit(&self, mut f: impl FnMut(usize, usize)) {
        self.any(|a, b| {
            f(a, b);
            false
        });
    }
}
