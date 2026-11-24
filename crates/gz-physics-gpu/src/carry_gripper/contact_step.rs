// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[derive(Clone, Copy)]
pub(super) struct Point<T> {
    pub(super) support: usize,
    pub(super) normal: [T; 2],
    pub(super) gap: T,
    pub(super) spin_normal: T,
    pub(super) spin_tangent: T,
}
pub(super) struct Loads<T> {
    pub(super) normal: [T; 4],
    pub(super) tangent: [T; 4],
    pub(super) up: [T; 4],
    pub(super) points: [[T; 2]; 4],
}
pub(super) struct Step<T> {
    pub(super) limits: [[T; 2]; 4],
    pub(super) inv: [T; 7],
    pub(super) speed: [T; 7],
    pub(super) mu: [T; 4],
    pub(super) zero: T,
    pub(super) one: T,
}
impl<T: Scalar> Step<T> {
    pub(super) fn solve<const N: usize>(
        &self,
        points: [Point<T>; N],
        sum: &mut Loads<T>,
    ) -> ([T; 7], [T; 7]) {
        let z = self.zero;
        let rows = points.map(|p| {
            let row = |x: T, h: T, spin: T| {
                [
                    if p.support < 2 { z - h } else { z },
                    if p.support == 0 { z - x } else { z },
                    if p.support == 1 { z - x } else { z },
                    x,
                    h,
                    spin,
                    z,
                ]
            };
            let [x, h] = p.normal;
            [row(x, h, p.spin_normal), row(h, z - x, p.spin_tangent)]
        });
        let mu = points.map(|p| self.mu[p.support]);
        let contacts = Contacts::new(rows, self.inv, points.map(|p| p.gap));
        let (next, imp) = contacts.project::<false>(
            self.speed,
            mu,
            z,
            self.one,
            stops::<false, T>(self.limits, self.inv, z),
        );
        let (delta, _) = contacts.project::<true>(
            [z; 7],
            mu,
            z,
            self.one,
            stops::<true, T>(self.limits, self.inv, z),
        );
        for (j, p) in points.iter().enumerate() {
            let i = p.support;
            sum.normal[i] = sum.normal[i] + imp[j][0];
            sum.tangent[i] = sum.tangent[i] + imp[j][1];
            sum.up[i] = sum.up[i] + p.normal[1] * imp[j][0] - p.normal[0] * imp[j][1];
            let end = if N == 8 { j % 2 } else { 0 };
            sum.points[i][end] = sum.points[i][end] + imp[j][0];
        }
        (next, delta)
    }
}
