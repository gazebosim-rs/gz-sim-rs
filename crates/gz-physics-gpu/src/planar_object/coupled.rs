// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Eliminate the sphere-floor normal and tangent before solving the tool-sphere contact.
use super::*;

pub(super) struct PairBlock<T> {
    pub floor: [T; 2],
    pub pair: [[T; 2]; 2],
    /// Pair rows, floor columns.
    pub cross: [[T; 2]; 2],
}
// Sliding on the floor makes the condensed pair response nonsymmetric.
fn contact<T: Scalar>(v: [T; 2], k: [[T; 2]; 2], mu: T, z: T, one: T) -> [T; 2] {
    let [[a, b], [c, d]] = k;
    let det = a * d - b * c;
    let div = det.greater(z).select(det, one);
    let normal = (b * v[1] - d * v[0]) / div;
    let tangent = (c * v[0] - a * v[1]) / div;
    let stick = det
        .greater(z)
        .and(normal.at_least(z))
        .and(tangent.at_least(z - mu * normal))
        .and(tangent.at_most(mu * normal));
    let slip = a * v[1] - c * v[0];
    let direction = slip.greater(z).select(z - mu, slip.less(z).select(mu, z));
    let mobility = a + b * direction;
    let slide = (z - v[0]) / mobility.greater(z).select(mobility, one);
    let closed = v[0].less(z).and(a.greater(z));
    [
        closed.select(stick.select(normal, slide), z),
        closed.select(stick.select(tangent, direction * slide), z),
    ]
}
impl<T: Scalar> PairBlock<T> {
    pub fn solve(&self, gaps: [T; 2], free: [[T; 2]; 2], mu: [T; 2], z: T, one: T) -> [[T; 2]; 2] {
        let [vf, vp] = free;
        let [kn, kt] = self.floor;
        let cross = self.cross;
        let mut chosen = [[z; 2]; 2];
        for state in 0..4 {
            // The four possible floor states are open, forward sliding, reverse sliding, sticking.
            let direction = if state == 1 { mu[0] } else { z - mu[0] };
            let floor = |v: [T; 2]| {
                let normal = if state == 0 { z } else { (z - v[0]) / kn };
                [
                    normal,
                    if state == 3 {
                        (z - v[1]) / kt
                    } else {
                        normal * direction
                    },
                ]
            };
            let base = floor(vf);
            let effective_v = std::array::from_fn(|i| vp[i] + dot(cross[i], base));
            let k = std::array::from_fn(|i| {
                std::array::from_fn(|j| self.pair[i][j] + dot(cross[i], floor(cross[j])))
            });
            let pair =
                contact(effective_v, k, mu[1], z, one).map(|p| gaps[1].at_most(z).select(p, z));
            let after_pair =
                std::array::from_fn(|i| vf[i] + cross[0][i] * pair[0] + cross[1][i] * pair[1]);
            let f = floor(after_pair);
            let slip = after_pair[1] + kt * f[1];
            let valid = if state == 0 {
                gaps[0].greater(z).or(after_pair[0].at_least(z))
            } else {
                let friction = if state == 3 {
                    f[1].at_least(z - mu[0] * f[0])
                        .and(f[1].at_most(mu[0] * f[0]))
                } else if state == 1 {
                    slip.at_most(z)
                } else {
                    slip.at_least(z)
                };
                gaps[0].at_most(z).and(f[0].at_least(z)).and(friction)
            };
            for i in 0..2 {
                chosen[0][i] = valid.select(f[i], chosen[0][i]);
                chosen[1][i] = valid.select(pair[i], chosen[1][i]);
            }
        }
        chosen
    }
}
