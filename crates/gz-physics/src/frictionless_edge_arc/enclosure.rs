// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Enclose the smooth forces and every other vertex's height on one local
//! two-bearing segment. In body coordinates only the roll angle changes the
//! support normal, independently of the magnitude or angle of yaw.
use super::*;
use crate::interval::Interval as I;
type VecI = [I; 3];
type MatI = [VecI; 3];
fn vec(p: DVec3) -> VecI {
    p.to_array().map(I::point)
}
fn dot(a: VecI, b: VecI) -> I {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: VecI, b: VecI) -> VecI {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn mul(a: MatI, x: VecI) -> VecI {
    a.map(|r| dot(r, x))
}
fn plus(a: VecI, b: VecI) -> VecI {
    std::array::from_fn(|k| a[k] + b[k])
}
fn scale(a: VecI, v: I) -> VecI {
    a.map(|k| k * v)
}
fn inverse(m: MatI) -> Option<MatI> {
    let cols = [cross(m[1], m[2]), cross(m[2], m[0]), cross(m[0], m[1])];
    let det = dot(cols[0], m[0]);
    let inv = I::point(1.).over(det)?;
    Some(std::array::from_fn(|i| {
        std::array::from_fn(|j| cols[j][i] * inv)
    }))
}
/// Outward range of a pair's signed normal reactions and relative point
/// heights on a continuous two-bearing candidate. A missing envelope or a
/// nonpositive lower bound leaves the unilateral work to the caller.
#[derive(Clone, Copy, Debug)]
pub struct EdgeSupportEnvelope {
    pub force_bounds: [[f64; 2]; 2],
    pub angular_speed_bound: f64,
    pub center_speed_bound: f64,
    normal_body: VecI,
    initial_normal: VecI,
    edge: VecI,
    roll: I,
    rate: I,
    midpoint: DVec3,
}
impl EdgeSupportEnvelope {
    /// Bound the gap directly as a function of roll. The scalar height
    /// satisfies g''(θ) = −A cos θ −B sin θ. Its Taylor remainder is bounded
    /// by one half of the projected lever length times θ².
    pub fn directed_height(self, from_com: DVec3) -> [f64; 2] {
        let d = std::array::from_fn(|k| I::point(from_com[k]) - I::point(self.midpoint[k]));
        let initial = dot(self.initial_normal, d);
        let slope = dot(self.initial_normal, cross(self.edge, d));
        let lever =
            (from_com - self.midpoint).length() * (1. + 128. * f64::EPSILON) + 128. * f64::EPSILON;
        let rem =
            (I::point(0.5 * lever) * I::point(self.roll.maxabs()) * I::point(self.roll.maxabs()))
                .hi;
        let gap = initial + slope * self.roll + I::around(0., rem);
        [gap.lo, gap.hi]
    }
    /// Signed derivative of that same vertex's gap (m/s).
    pub fn normal_speed(self, from_com: DVec3) -> [f64; 2] {
        let d = std::array::from_fn(|k| I::point(from_com[k]) - I::point(self.midpoint[k]));
        let value = dot(self.normal_body, cross(self.edge, d)) * self.rate;
        [value.lo, value.hi]
    }
    /// Gap range to the same fixed support plane, in metres.
    pub fn height(self, from_com: DVec3) -> [f64; 2] {
        let v = dot(
            self.normal_body,
            std::array::from_fn(|k| I::point(from_com[k]) - I::point(self.midpoint[k])),
        );
        [v.lo, v.hi]
    }
}
impl BoundFrictionlessEdge {
    /// Bound smooth quantities on [0, seconds] from this binding. A first-exit
    /// argument encloses roll and roll rate. Signed forces follow from the
    /// acceleration constraints using the full body-space Delassus matrix.
    pub fn enclose(self, seconds: f64) -> Result<Option<EdgeSupportEnvelope>, PhysicsError> {
        if !seconds.is_finite() || seconds < 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let b = self.body;
        let n = b.normal;
        let s = self.start;
        let i = b
            .inertia_body
            .transpose()
            .to_cols_array_2d()
            .map(|r| r.map(I::point));
        let Some(inv) = inverse(i) else {
            return Ok(None);
        };
        let d = self
            .edge
            .to_array()
            .map(|v| I::around(v, 64. * f64::EPSILON));
        let initial_normal = s.orientation.conjugate() * n;
        let gn = dot(vec(b.gravity), vec(n));
        let Some(inv_mass) = I::point(1.).over(I::point(b.mass)) else {
            return Ok(None);
        };
        let jn = self.momentum;
        let momentum = I::around(
            jn,
            128. * f64::EPSILON
                * (1.
                    + jn.abs()
                    + b.inertia_body
                        .to_cols_array()
                        .into_iter()
                        .map(f64::abs)
                        .sum::<f64>()
                        * s.angular_velocity.length()),
        );
        let mut radius = 256. * f64::EPSILON * (1. + self.initial_roll_speed.abs())
            + seconds * (1. + self.stage(self.initial())?.rates.rate.abs()) * 1.25;
        for _ in 0..5 {
            // A unit vector rotated by angle θ changes each component by at
            // most |θ|. Include the rounding in the initial body-frame rotation.
            let u = I::around(self.initial_roll_speed, radius);
            let theta = (I::point(seconds) * I::point(u.maxabs())).hi;
            if theta > 0.3 || !u.finite() {
                return Ok(None);
            }
            let nbody = initial_normal
                .to_array()
                .map(|v| I::around(v, theta + 256. * f64::EPSILON));
            let a = dot(nbody, mul(i, nbody));
            let cross_inertia = dot(nbody, mul(i, d));
            let Some(yaw) = (momentum - cross_inertia * u).over(a) else {
                return Ok(None);
            };
            let w = plus(scale(d, u), scale(nbody, yaw));
            let gyro = mul(inv, scale(cross(w, mul(i, w)), I::point(-1.)));
            let arms = b.sites_body.map(vec);
            let torque = arms.map(|r| cross(r, nbody));
            let free = arms.map(|r| gn + dot(nbody, plus(cross(gyro, r), cross(w, cross(w, r)))));
            let w11 = inv_mass + dot(torque[0], mul(inv, torque[0]));
            let w22 = inv_mass + dot(torque[1], mul(inv, torque[1]));
            let w12 = inv_mass + dot(torque[0], mul(inv, torque[1]));
            let det = w11 * w22 - w12 * w12;
            let Some(f1) = (w12 * free[1] - w22 * free[0]).over(det) else {
                return Ok(None);
            };
            let Some(f2) = (w12 * free[0] - w11 * free[1]).over(det) else {
                return Ok(None);
            };
            let alpha = plus(
                gyro,
                mul(inv, plus(scale(torque[0], f1), scale(torque[1], f2))),
            );
            let required = (I::point(seconds) * I::point(dot(d, alpha).maxabs())).hi;
            if required <= radius {
                let spin = (I::point(u.maxabs()) + I::point(yaw.maxabs())).hi;
                let horizontal =
                    (s.velocity - n * n.dot(s.velocity)).length() + seconds * b.gravity.length();
                let velocity = horizontal + spin * self.midpoint.length();
                if !f1.finite() || !f2.finite() || !velocity.is_finite() || !spin.is_finite() {
                    return Ok(None);
                }
                return Ok(Some(EdgeSupportEnvelope {
                    force_bounds: [[f1.lo, f1.hi], [f2.lo, f2.hi]],
                    angular_speed_bound: spin.next_up(),
                    center_speed_bound: (velocity * (1. + 128. * f64::EPSILON)
                        + 128. * f64::EPSILON)
                        .next_up(),
                    normal_body: nbody,
                    initial_normal: initial_normal
                        .to_array()
                        .map(|v| I::around(v, 256. * f64::EPSILON)),
                    edge: d,
                    roll: I::range(0., seconds) * u,
                    rate: u,
                    midpoint: self.midpoint,
                }));
            }
            radius = 1.25 * required + 256. * f64::EPSILON * (1. + self.initial_roll_speed.abs());
        }
        Ok(None)
    }
}
