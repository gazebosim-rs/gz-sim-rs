// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Necessary conditions for simultaneous normal closure anywhere on a rigid line.
use crate::{
    EdgeMobility, PhysicsError,
    edge_impact::{ScaledEdge, positive_semidefinite},
    interval::Interval as I,
};
use glam::{DMat3, DVec3};
use std::ops::{Add, Mul, Sub};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineClosure {
    /// Every pressure box violates a necessary condition (including work).
    Excluded,
    /// The bounded search retained a possible simultaneous pressure field.
    Undetermined,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LineClosureSearch {
    pub status: LineClosure,
    pub visited: u32,
}
#[derive(Clone, Copy)]
struct Affine([I; 4]);
impl Affine {
    fn eval(self, z: [I; 3]) -> I {
        self.0[0] + self.0[1] * z[0] + self.0[2] * z[1] + self.0[3] * z[2]
    }
}
impl Add for Affine {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self(std::array::from_fn(|i| self.0[i] + b.0[i]))
    }
}
impl Sub for Affine {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        Self(std::array::from_fn(|i| self.0[i] - b.0[i]))
    }
}
impl Mul<I> for Affine {
    type Output = Self;
    fn mul(self, b: I) -> Self {
        Self(self.0.map(|x| x * b))
    }
}

struct Closure {
    // For a total positive normal impulse N, the three coordinates are
    // longitudinal/N, transverse/N and endpoint-one equivalent transverse/N.
    // The linear normal closure equations then fix endpoint-one normal/N
    // and 1/N. Every local slip divided by N is also affine in these variables.
    fraction: Affine,
    reciprocal: Affine,
    slip: [[Affine; 2]; 2],
    mu: f64,
}
impl Closure {
    fn reduce(w: [[DMat3; 2]; 2], v: [DVec3; 2], mu: f64) -> Option<Self> {
        let tol = 1024. * f64::EPSILON;
        if (v[0].x - v[1].x).abs() > tol
            || w.iter().any(|r| r[0].x_axis.distance(r[1].x_axis) > tol)
        {
            return None;
        }
        let value = |x| I::around(x, tol);
        let terms = |i: usize, axis: usize| {
            let (a, b) = (w[i][0].row(axis), w[i][1].row(axis));
            // Include any rounding in representing an unobservable internal
            // longitudinal split by a single resultant on endpoint zero.
            let extra = (b.x - a.x).abs() * mu;
            let base = Affine([
                I::around(a.z, tol + extra),
                value(a.x),
                value(a.y),
                value(b.y) - value(a.y),
            ]);
            (base, value(b.z) - value(a.z), value(v[i][axis]))
        };
        let (b0, f0, l0) = terms(0, 2);
        let (b1, f1, l1) = terms(1, 2);
        let det = f0 * l1 - f1 * l0;
        let inv = if det.lo > 0. {
            I::point(1.).over(det)?
        } else if det.hi < 0. {
            -I::point(1.).over(-det)?
        } else {
            return None;
        };
        let fraction = (b1 * l0 - b0 * l1) * inv;
        let reciprocal = (b0 * f1 - b1 * f0) * inv;
        let slip = std::array::from_fn(|i| {
            std::array::from_fn(|axis| {
                let (b, f, l) = terms(i, axis);
                b + fraction * f + reciprocal * l
            })
        });
        Some(Self {
            fraction,
            reciprocal,
            slip,
            mu,
        })
    }
    fn excludes(&self, z: &mut [I; 3]) -> bool {
        fn minabs(v: I) -> f64 {
            0.0_f64.max(v.lo).max(-v.hi)
        }
        fn min_len(a: I, b: I) -> f64 {
            let a = I::point(minabs(a));
            let b = I::point(minabs(b));
            (a * a + b * b).lo.max(0.).sqrt().next_down()
        }
        let [a, b, _] = *z;
        let mu = I::point(self.mu);
        let qq = self.fraction.eval(*z);
        if !qq.finite() {
            return false;
        }
        if qq.hi < 0. || qq.lo > 1. || self.reciprocal.eval(*z).hi <= 0. || min_len(a, b) > self.mu
        {
            return true;
        }
        let q = I {
            lo: qq.lo.max(0.),
            hi: qq.hi.min(1.),
        };
        let left = I::point(1.) - q;
        let cap0 = mu * left;
        let cap1 = mu * q;
        // Endpoint transverse impulse moments satisfy |t|≤μq and
        // |b−t|≤μ(1−q), even for a continuous pressure distribution.
        // Intersect their implications before evaluating slip and choosing
        // the next subdivision. All arithmetic expands the enclosing interval.
        let symmetric = |x: f64| I { lo: -x, hi: x };
        fn intersect(a: &mut I, b: I) -> bool {
            a.lo = a.lo.max(b.lo);
            a.hi = a.hi.min(b.hi);
            a.lo <= a.hi
        }
        if !intersect(&mut z[2], symmetric(cap1.hi))
            || !intersect(&mut z[2], b + symmetric(cap0.hi))
        {
            return true;
        }
        let t = z[2];
        if !intersect(&mut z[1], t + symmetric(cap0.hi)) {
            return true;
        }
        let b = z[1];
        let mut t0 = b - t;
        if !intersect(&mut t0, symmetric(cap0.hi)) {
            return true;
        }
        // Use both circular disks, including their common unknown axial split.
        let available = |cap: I, y: I| {
            let r = I::point(cap.hi);
            let c = I::point(minabs(y));
            ((r - c) * (r + c)).hi.max(0.).sqrt().next_up()
        };
        let axial_cap = (I::point(available(cap0, t0)) + I::point(available(cap1, t))).hi;
        if !intersect(&mut z[0], symmetric(axial_cap)) {
            return true;
        }
        let a = z[0];
        let s0 = self.slip[0][0].eval(*z);
        let s1 = self.slip[1][0].eval(*z);
        // The exact axial post slip is shared along the entire contact line.
        let sx = I {
            lo: s0.lo.max(s1.lo),
            hi: s0.hi.min(s1.hi),
        };
        if !sx.finite() {
            return false;
        }
        let v0 = self.slip[0][1].eval(*z);
        let v1 = self.slip[1][1].eval(*z);
        if (a * sx).lo > 0.
            || (v0.lo.min(v1.lo) > 0. && (t0.lo > 0. || t.lo > 0.))
            || (v0.hi.max(v1.hi) < 0. && (t0.hi < 0. || t.hi < 0.))
        {
            return true;
        }
        // Dissipated post-slip power must lie between the barycentre and
        // endpoint evaluations of the convex norm, for every pressure measure.
        let diss = -(a * sx + t0 * v0 + t * v1);
        let lower = mu * I::point(min_len(sx, left * v0 + q * v1));
        let max_len = |a: I, b: I| {
            let a = I::point(a.maxabs());
            let b = I::point(b.maxabs());
            (a * a + b * b).hi.sqrt().next_up()
        };
        let upper = mu * (left * I::point(max_len(sx, v0)) + q * I::point(max_len(sx, v1)));
        diss.hi < lower.lo || diss.lo > upper.hi
    }
}
/// Test all continuous, nonnegative Coulomb pressure distributions, including
/// atomic endpoints, for a closing rigid segment with uniform friction and no
/// restitution. Use a common orthonormal contact frame: X along the segment,
/// Z along its normal. A retained box may still be infeasible. The search
/// budget bounds cost and never converts uncertainty to exclusion.
pub fn exclude_closed_line(
    mobility: EdgeMobility,
    velocity: [[f64; 3]; 2],
    friction: f64,
    max_boxes: u32,
) -> Result<LineClosureSearch, PhysicsError> {
    let g = ScaledEdge::new(mobility, velocity, [friction; 2])?;
    if !positive_semidefinite(g.w) {
        return Err(PhysicsError::InvalidValue);
    }
    let mut answer = LineClosureSearch {
        status: LineClosure::Undetermined,
        visited: 0,
    };
    if max_boxes == 0 || g.vs == 0. || g.v.iter().all(|v| v.z >= 0.) {
        return Ok(answer);
    }
    let Some(model) = Closure::reduce(g.w, g.v, friction) else {
        return Ok(answer);
    };
    let mut work = vec![([I::range(-friction, friction); 3], 0)];
    while let Some((mut z, depth)) = work.pop() {
        if answer.visited == max_boxes {
            return Ok(answer);
        }
        answer.visited += 1;
        if model.excludes(&mut z) {
            continue;
        }
        if depth >= 48 {
            return Ok(answer);
        }
        let mut i = 0;
        for j in 1..3 {
            if z[j].hi - z[j].lo > z[i].hi - z[i].lo {
                i = j
            }
        }
        let mid = 0.5 * z[i].lo + 0.5 * z[i].hi;
        if mid <= z[i].lo || mid >= z[i].hi {
            return Ok(answer);
        }
        let mut a = z;
        let mut b = z;
        a[i].hi = mid;
        b[i].lo = mid;
        work.push((a, depth + 1));
        work.push((b, depth + 1));
    }
    answer.status = LineClosure::Excluded;
    Ok(answer)
}
#[cfg(test)]
mod tests {
    use super::*;
    fn matrices(r: [DVec3; 2], inv: DMat3) -> EdgeMobility {
        let basis = [DVec3::X, DVec3::Y, DVec3::Z];
        std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                std::array::from_fn(|a| {
                    std::array::from_fn(|b| {
                        basis[a].dot(0.5 * basis[b] + (inv * r[j].cross(basis[b])).cross(r[i]))
                    })
                })
            })
        })
    }
    #[test]
    fn distributed_fields_with_genuine_simultaneous_slip_are_never_excluded() {
        let axis = DMat3::from_quat(glam::DQuat::from_rotation_y(0.41));
        let inv = axis * DMat3::from_diagonal(DVec3::new(13., 21., 18.)) * axis.transpose();
        let mut attempted = 0;
        for m in [0., 0.45, 0.7] {
            for shift in [-0.31, -0.13, 0.02, 0.19] {
                let r = [
                    DVec3::new(shift, 0.064, -0.11),
                    DVec3::new(shift + 0.35, 0.064, -0.11),
                ];
                let w = matrices(r, inv);
                for yaw in [-4., 0.08, 1.3] {
                    let final_w = DVec3::new(0.6, 0., yaw);
                    let final_v = DVec3::new(0.24, -0.16, -0.6 * r[0].y);
                    let mut j = DVec3::ZERO;
                    let mut moment = DVec3::ZERO;
                    for (s, t) in [(0.05, 0.24), (0.43, 0.31), (0.79, 0.45)] {
                        let arm = r[0].lerp(r[1], s);
                        let slip = final_v + final_w.cross(arm);
                        assert!(slip.z.abs() < 1e-14);
                        let f = t * (DVec3::Z - m * slip.normalize());
                        j += f;
                        moment += arm.cross(f);
                    }
                    let v = final_v - 0.5 * j;
                    let omega = final_w - inv * moment;
                    let pre = r.map(|p| (v + omega.cross(p)).to_array());
                    let ans = exclude_closed_line(w, pre, m, 1024).unwrap();
                    assert_eq!(ans.status, LineClosure::Undetermined);
                    attempted += usize::from(ans.visited > 0);
                }
            }
        }
        assert!(attempted >= 32);
    }
    #[test]
    fn a_closing_short_edge_with_compressive_cross_mobility_forces_one_end_to_open() {
        let r = [DVec3::new(-0.03, 0., -0.12), DVec3::new(0.03, 0., -0.12)];
        let w = matrices(r, DMat3::from_diagonal(DVec3::splat(10.)));
        let u = r.map(|p| DVec3::Y.cross(p - r[0]).to_array());
        assert_eq!(
            exclude_closed_line(w, u, 0.1, 128).unwrap().status,
            LineClosure::Excluded
        );
        assert_eq!(
            exclude_closed_line(w, u, 0.1, 0).unwrap().status,
            LineClosure::Undetermined
        );
    }
}
#[cfg(test)]
#[path = "line_closure_controls.rs"]
mod controls;
