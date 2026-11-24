// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Coupled Coulomb contact in generalized coordinates [shoulder, elbow, x, z, spin].
use super::*;
type Row<T> = [T; 5];

fn product<T: Scalar>(a: Row<T>, b: Row<T>) -> T {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] + a[4] * b[4]
}
pub(super) struct Jacobian<T> {
    pub rows: [[Row<T>; 2]; 3],
    pub mobility: [[Row<T>; 2]; 3],
}
impl<T: Scalar> Jacobian<T> {
    /// Solve the normal and tangent at each contact together. Each contact's previous
    /// iterate is removed before reprojection, so lift-off also clears its friction.
    pub fn solve(
        &self,
        gap: [T; 3],
        mu: [T; 3],
        incoming: Row<T>,
        zero: T,
        one: T,
        half: T,
    ) -> ([[T; 2]; 3], Row<T>, [T; 4]) {
        let k: [[T; 3]; 3] = std::array::from_fn(|i| {
            let [n, t] = self.rows[i];
            let [a, b] = self.mobility[i];
            [product(n, a), product(n, b), product(t, b)]
        });
        let mut v = incoming;
        let mut impulse = [[zero; 2]; 3];
        let cross = std::array::from_fn(|i| {
            std::array::from_fn(|j| product(self.rows[2][i], self.mobility[1][j]))
        });
        let pair = super::coupled::PairBlock {
            floor: [k[1][0], k[1][2]],
            pair: [[k[2][0], k[2][1]], [k[2][1], k[2][2]]],
            cross,
        };
        // Project the tool-floor row against the coupled sphere contacts. Their
        // four floor friction modes are solved exactly inside each iteration.
        for _ in 0..4 {
            let [pn, pt] = impulse[0];
            let [nn, nt, tt] = k[0];
            let free = [
                product(self.rows[0][0], v) - nn * pn - nt * pt,
                product(self.rows[0][1], v) - nt * pn - tt * pt,
            ];
            let floor = crate::planar_tool::impact(free, k[0], mu[0], zero, one)
                .map(|p| gap[0].at_most(zero).select(p, zero));
            let base: Row<T> = std::array::from_fn(|j| {
                incoming[j] + self.mobility[0][0][j] * floor[0] + self.mobility[0][1][j] * floor[1]
            });
            let free = std::array::from_fn(|i| self.rows[i + 1].map(|r| product(r, base)));
            let [ball, tool] = pair.solve([gap[1], gap[2]], free, [mu[1], mu[2]], zero, one);
            impulse = [floor, ball, tool];
            v = std::array::from_fn(|j| {
                base[j]
                    + self.mobility[1][0][j] * ball[0]
                    + self.mobility[1][1][j] * ball[1]
                    + self.mobility[2][0][j] * tool[0]
                    + self.mobility[2][1][j] * tool[1]
            });
        }
        let midpoint = std::array::from_fn(|i| half * (incoming[i] + v[i]));
        let tangent_work =
            std::array::from_fn::<_, 3, _>(|i| impulse[i][1] * product(self.rows[i][1], midpoint));
        let mut total = tangent_work[0] + tangent_work[1] + tangent_work[2];
        for (i, imp) in impulse.iter().enumerate() {
            total = total + imp[0] * product(self.rows[i][0], midpoint);
        }
        (
            impulse,
            v,
            [tangent_work[0], tangent_work[1], tangent_work[2], total],
        )
    }
}
