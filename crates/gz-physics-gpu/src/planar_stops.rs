// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Angular travel bounds and the two-coordinate unilateral impulse solve.
use crate::arithmetic::{Condition, Scalar};
use crate::planar_arm::ArmInertia;
use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
struct Bound {
    c: f64,
    s: f64,
    turns: f64,
    enabled: f64,
}
impl Bound {
    fn at(angle: f64) -> Self {
        let (s, c) = angle.sin_cos();
        let phase = (if s == 0. { 0. } else { s }).atan2(c);
        Self {
            c,
            s,
            turns: ((angle - phase) / std::f64::consts::TAU).round(),
            enabled: 1.,
        }
    }
}
/// One SDF hinge's limits relative to its initial configuration. Default is free.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct PlanarJointStops {
    lower: Bound,
    upper: Bound,
    locked: f64,
}
impl PlanarJointStops {
    pub fn from_relative(origin: f64, lower: f64, upper: f64) -> Option<Self> {
        if !origin.is_finite() || !(lower <= 0. && upper >= 0.) {
            return None;
        }
        let bound = |q: f64| {
            if q.is_infinite() {
                return Some(Bound::default());
            }
            let b = Bound::at(q + origin);
            // The device winding counter must distinguish adjacent revolutions.
            (b.turns.abs() + 1. > b.turns.abs()).then_some(b)
        };
        Some(Self {
            lower: bound(lower)?,
            upper: bound(upper)?,
            locked: f64::from(lower == upper),
        })
    }
    pub(super) fn admits(self, phase: [f64; 2]) -> bool {
        let q = [phase[0], phase[1], 0.];
        let valid = |b: Bound| {
            b.enabled == 0.
                || (b.enabled == 1.
                    && b.c.is_finite()
                    && b.s.is_finite()
                    && ((b.c * b.c + b.s * b.s) - 1.).abs() <= 1e-12
                    && b.turns.fract() == 0.
                    && b.turns.abs() + 1. > b.turns.abs())
        };
        valid(self.lower)
            && valid(self.upper)
            && (self.lower.enabled == 0.
                || before([self.lower.c, self.lower.s, self.lower.turns], q, 0.))
            && (self.upper.enabled == 0.
                || before(q, [self.upper.c, self.upper.s, self.upper.turns], 0.))
            && self.locked
                == f64::from(
                    self.lower.enabled == 1.
                        && self.upper.enabled == 1.
                        && (self.lower.c, self.lower.s, self.lower.turns)
                            == (self.upper.c, self.upper.s, self.upper.turns),
                )
    }
    pub(super) fn enabled(self) -> bool {
        self.lower.enabled != 0. || self.upper.enabled != 0.
    }
}
fn before<T: Scalar>([c, s, t]: [T; 3], [d, u, v]: [T; 3], z: T) -> T::Condition {
    // Order by complete turns, then the two signed half-circles. The cross
    // product resolves small angles even next to zero and pi. At antipodes
    // the cosine order disambiguates the two ends of the closed positive arc.
    let same_half = s
        .less(z)
        .and(u.less(z))
        .or(s.at_least(z).and(u.at_least(z)));
    let cross = c * u - s * d;
    let lexical = s.less(z).and(u.at_least(z)).or(same_half.and(
        cross.greater(z).or(cross.at_least(z).and(
            s.less(z)
                .and(c.at_most(d))
                .or(s.at_least(z).and(c.at_least(d))),
        )),
    ));
    t.less(v).or(t.at_most(v).and(lexical))
}
/// Snap each crossed endpoint and return the inward direction. Locked joints
/// accept either sign of reaction in the velocity solve.
pub(super) fn snap<T: Scalar>(v: &mut [T], z: T, one: T) -> [T; 2] {
    std::array::from_fn(|i| {
        let b = 42 + 9 * i;
        let phase = [v[2 * i], v[2 * i + 1], v[36 + i]];
        let lower = [v[b], v[b + 1], v[b + 2]];
        let upper = [v[b + 4], v[b + 5], v[b + 6]];
        let lo = v[b + 3].greater(z).and(before(phase, lower, z));
        let hi = v[b + 7].greater(z).and(before(upper, phase, z));
        for (k, slot) in [2 * i, 2 * i + 1, 36 + i].into_iter().enumerate() {
            v[slot] = lo.select(lower[k], hi.select(upper[k], phase[k]));
        }
        lo.select(one, hi.select(z - one, z))
    })
}
/// A bound reached first moves the free coordinate along the same inverse
/// inertia column as the constraint impulse. This avoids lifting a tool that
/// was resting on the floor solely by projecting the driven hinge backward.
pub(super) fn project<T: Scalar>(
    v: &mut [T],
    mass: &ArmInertia<T>,
    z: T,
    one: T,
    half: T,
) -> [T; 2] {
    let old = [[v[0], v[1]], [v[2], v[3]]];
    let side = snap(v, z, one);
    for i in 0..2 {
        let j = 1 - i;
        let active = side[i].greater(z).or(side[i].less(z));
        let free = side[j].at_least(z).and(side[j].at_most(z));
        let [c, s] = old[i];
        let delta = c * v[i * 2 + 1] - s * v[i * 2];
        let e = if i == 0 { [one, z] } else { [z, one] };
        let col = mass.inverse(v[2], v[3], e);
        let h = active.and(free).select(half * delta * col[j] / col[i], z);
        let co = (one - h * h) / (one + h * h);
        let si = (h + h) / (one + h * h);
        let [c, s] = [v[j * 2], v[j * 2 + 1]];
        let rotated = [c * co - s * si, s * co + c * si];
        v[j * 2] = rotated[0];
        v[j * 2 + 1] = rotated[1];
        v[36 + j] = v[36 + j] + super::planar_tool::crossing([c, s], rotated, z, one);
    }
    snap(v, z, one)
}
/// Solve at the predicted configuration. A single active row projects with the
/// full inverse inertia. Two independent active rows stop both DOFs. In that
/// case enumerate admissible Coulomb tractions and solve the two momentum
/// balances directly. Prefer zero tangential force when sticking is redundant.
#[allow(clippy::too_many_arguments)]
pub(super) fn solve<T: Scalar>(
    mass: &ArmInertia<T>,
    c: T,
    s: T,
    speed: [T; 2],
    side: [T; 2],
    locked: [T; 2],
    floor: T::Condition,
    jn: [T; 2],
    jt: [T; 2],
    mu: T,
    free_floor: [T; 2],
    z: T,
    one: T,
) -> ([T; 2], [T; 2], [T; 2]) {
    let inv = [mass.inverse(c, s, [one, z]), mass.inverse(c, s, [z, one])];
    let h = mass.momentum(c, speed).map(|v| z - v);
    let enabled = |i: usize| side[i].less(z).or(side[i].greater(z));
    let reaction = |i: usize, p: T| locked[i].greater(z).or((p * side[i]).at_least(z));
    let feasible = |i: usize, w: T| {
        // A locked coordinate has no feasible nonzero speed.
        let signed = side[i] * w;
        locked[i]
            .greater(z)
            .select(z - w.maximum(z - w), signed)
            .at_least(z)
    };
    let mut result = (speed, [z; 2], [z; 2]);
    let mut select = |yes: T::Condition, w: [T; 2], f: [T; 2], stop: [T; 2]| {
        for i in 0..2 {
            result.0[i] = yes.select(w[i], result.0[i]);
            result.1[i] = yes.select(f[i], result.1[i]);
            result.2[i] = yes.select(stop[i], result.2[i]);
        }
    };
    // Redundant area contacts can admit more than one distribution. Active
    // floor + one stop takes precedence over two stops alone.
    let both = enabled(0)
        .and(enabled(1))
        .and(reaction(0, h[0]))
        .and(reaction(1, h[1]));
    select(both, [z; 2], [z; 2], h);
    for i in 0..2 {
        let k = 1 - i;
        for ratio in [mu, z - mu, z] {
            let row = [jn[0] + ratio * jt[0], jn[1] + ratio * jt[1]];
            let regular = row[k].greater(z).or(row[k].less(z));
            let n = h[k] / regular.select(row[k], one);
            let p = h[i] - row[i] * n;
            let valid = floor
                .and(enabled(i))
                .and(regular)
                .and(n.at_least(z))
                .and(reaction(i, p));
            let mut stop = [z; 2];
            stop[i] = p;
            select(valid, [z; 2], [n, ratio * n], stop);
        }
    }
    for i in 0..2 {
        let k = 1 - i;
        let p = (z - speed[i]) / inv[i][i];
        let mut w = speed;
        w[i] = z;
        w[k] = w[k] + inv[i][k] * p;
        let vn = jn[0] * w[0] + jn[1] * w[1];
        let floor_ok = floor.select(vn, z).at_least(z);
        let valid = enabled(i)
            .and(reaction(i, p))
            .and(feasible(k, w[k]))
            .and(floor_ok);
        let mut stop = [z; 2];
        stop[i] = p;
        select(valid, w, [z; 2], stop);
    }
    let [n, t] = free_floor;
    let an = mass.inverse(c, s, jn);
    let at = mass.inverse(c, s, jt);
    let w = std::array::from_fn(|i| speed[i] + an[i] * n + at[i] * t);
    select(feasible(0, w[0]).and(feasible(1, w[1])), w, [n, t], [z; 2]);
    result
}
/// Remove outward directions from a floor position correction. When one joint
/// is blocked the other coordinate alone must provide the geometric correction.
pub(super) fn floor_correction<T: Scalar>(
    q: [T; 2],
    j: [T; 2],
    depth: T,
    side: [T; 2],
    locked: [T; 2],
    z: T,
    one: T,
) -> [T; 2] {
    let blocked: [T::Condition; 2] =
        std::array::from_fn(|i| (q[i] * side[i]).less(z).or(locked[i].greater(z)));
    std::array::from_fn(|i| {
        let regular = j[i].greater(z).or(j[i].less(z));
        let alone = regular.select(depth / regular.select(j[i], one), z);
        blocked[i].select(z, blocked[1 - i].select(alone, q[i]))
    })
}
