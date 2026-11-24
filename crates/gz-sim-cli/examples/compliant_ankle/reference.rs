// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent planar Lagrangian with two distinct link inertias and an ankle constraint.
use glam::{DMat4, DVec2, DVec4};

#[derive(Clone, Copy, Default)]
pub struct State {
    /// Foot COM x/z and absolute foot/shank pitch angles.
    pub q: DVec4,
    pub dq: DVec4,
    /// Integrated ground impulse (x,z) and signed frictional work.
    pub impulse: DVec2,
    pub work: f64,
}
impl State {
    fn plus(self, rate: Self, dt: f64) -> Self {
        Self {
            q: self.q + dt * rate.q,
            dq: self.dq + dt * rate.dq,
            impulse: self.impulse + dt * rate.impulse,
            work: self.work + dt * rate.work,
        }
    }
    pub fn shank_velocity(self) -> DVec2 {
        DVec2::new(self.dq.x, self.dq.y)
            + 0.045 * self.dq.z * tangent(self.q.z)
            + 0.20 * self.dq.w * tangent(self.q.w)
    }
    pub fn shank_center(self) -> DVec2 {
        DVec2::new(self.q.x, self.q.y) + 0.045 * up(self.q.z) + 0.20 * up(self.q.w)
    }
}
fn up(a: f64) -> DVec2 {
    DVec2::new(a.sin(), a.cos())
}
fn tangent(a: f64) -> DVec2 {
    DVec2::new(a.cos(), -a.sin())
}

fn plantar(s: State) -> (DVec2, f64, f64) {
    let up = up(s.q.z);
    let along = tangent(s.q.z);
    let base = -0.025 * up;
    // The two unilateral pressure factors are affine in the material coordinate.
    let mut lo = -0.18_f64;
    let mut hi = 0.18_f64;
    for (offset, slope) in [
        (-s.q.y - base.y, -along.y),
        (
            1. - 2.5 * (s.dq.y - s.dq.z * base.x),
            2.5 * s.dq.z * along.x,
        ),
    ] {
        if slope > 0. {
            lo = lo.max(-offset / slope);
        } else if slope < 0. {
            hi = hi.min(-offset / slope);
        } else if offset <= 0. {
            return (Default::default(), 0., 0.);
        }
    }
    if lo >= hi {
        return (Default::default(), 0., 0.);
    }
    let (mut force, mut torque, mut work) = (DVec2::ZERO, 0., 0.);
    // Symmetric Gauss-Legendre rule on the physical loaded interval across a 20 cm width.
    let nodes = [
        (0.1834346424956498, 0.362683783378362),
        (0.525532409916329, 0.3137066458778873),
        (0.7966664774136267, 0.2223810344533745),
        (0.9602898564975363, 0.1012285362903763),
    ];
    let du = 0.5 * (hi - lo);
    for (u, weight) in nodes {
        for sign in [-1., 1.] {
            let r = base + along * (lo + du * (1. + sign * u));
            let slip = s.dq.x + s.dq.z * r.y;
            let vz = s.dq.y - s.dq.z * r.x;
            let n = 1_200_000. * (-s.q.y - r.y) * (1. - 2.5 * vz) * du * 0.20 * weight;
            let f = DVec2::new(-0.32 * n * slip / slip.hypot(0.015), n);
            force += f;
            torque += r.y * f.x - r.x * f.y;
            work += f.x * slip;
        }
    }
    (force, torque, work)
}
fn rhs(s: State, locked: bool, applied: f64) -> State {
    let (floor, moment, work) = plantar(s);
    let a = 0.045 * tangent(s.q.z);
    let b = 0.20 * tangent(s.q.w);
    let cols = [
        DVec2::X,
        DVec2::Y,
        if locked { a + b } else { a },
        if locked { DVec2::ZERO } else { b },
    ];
    let inertias = DVec4::new(
        1.,
        1.,
        if locked { 0.016 + 0.011 } else { 0.016 },
        if locked { 1. } else { 0.011 },
    );
    let matrix = DMat4::from_cols_array_2d(&std::array::from_fn(|i| {
        std::array::from_fn(|j| cols[i].dot(cols[j]) + if i == j { inertias[i] } else { 0. })
    }));
    // Bias acceleration of the shank COM from the two rotating offsets.
    let effective = DVec2::new(0., -9.81)
        + 0.045 * up(s.q.z) * s.dq.z.powi(2)
        + 0.20 * up(s.q.w) * s.dq.w.powi(2);
    let generalized = DVec4::from_array(cols.map(|c| c.dot(effective)))
        + DVec4::new(
            floor.x,
            floor.y - 9.81,
            moment + if locked { applied } else { 0. },
            if locked { 0. } else { applied },
        );
    let mut acc = matrix.inverse() * generalized;
    if locked {
        acc.w = acc.z;
    }
    State {
        q: s.dq,
        dq: acc,
        impulse: floor,
        work,
    }
}
pub fn torque(tick: usize) -> f64 {
    if (81..=200).contains(&tick) { 0.55 } else { 0. }
}
pub fn trajectory(
    locked: bool,
    substeps: usize,
    ticks: usize,
    dt: f64,
    driven: bool,
) -> Vec<State> {
    let mut s = State {
        q: DVec4::new(0., 0.02477291666666667, 0., 0.),
        dq: DVec4::new(0.18, 0., 0., 0.),
        ..Default::default()
    };
    let mut out = vec![s];
    let h = dt / substeps as f64;
    for tick in 1..=ticks {
        let m = if driven { torque(tick) } else { 0. };
        for _ in 0..substeps {
            let a = rhs(s, locked, m);
            let b = rhs(s.plus(a, h * 0.5), locked, m);
            let c = rhs(s.plus(b, h * 0.5), locked, m);
            let d = rhs(s.plus(c, h), locked, m);
            s = s
                .plus(a, h / 6.)
                .plus(b, h / 3.)
                .plus(c, h / 3.)
                .plus(d, h / 6.);
        }
        out.push(s);
    }
    out
}
