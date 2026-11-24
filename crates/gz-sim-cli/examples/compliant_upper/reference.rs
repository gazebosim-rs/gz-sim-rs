// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent Cartesian RK4 integration in world linear and angular momentum.
//! The lower pressure integral uses strips in material coordinates. The production
//! kernel clips and triangulates a world-space polygon and splits the upper impulse.
use glam::{DQuat, DVec3, DVec4, EulerRot};

pub const MASS: f64 = 2.;
pub const INERTIA: DVec3 = DVec3::new(0.0518, 0.0558, 0.050);
pub const G: DVec3 = DVec3::new(0., 0., -9.81);

#[derive(Clone, Copy, Default)]
pub struct State {
    pub center: DVec3,
    pub quat: DVec4,
    pub p: DVec3,
    pub l: DVec3,
    pub upper_ns: f64,
    pub lower_ns: f64,
}
impl State {
    pub fn initial() -> Self {
        Self {
            center: DVec3::new(0., 0., 0.02477291666666667 + 0.07),
            quat: DVec4::W,
            p: MASS * DVec3::new(0.65, 0.2, 0.),
            ..Default::default()
        }
    }
    fn plus(self, b: Self, h: f64) -> Self {
        Self {
            center: self.center + h * b.center,
            quat: self.quat + h * b.quat,
            p: self.p + h * b.p,
            l: self.l + h * b.l,
            upper_ns: self.upper_ns + h * b.upper_ns,
            lower_ns: self.lower_ns + h * b.lower_ns,
        }
    }
    pub fn rotation(self) -> DQuat {
        DQuat::from_array(self.quat.normalize().to_array())
    }
}

// Roots of depth and positive pressure-rate cuts split the rectangle into
// strips. In each strip, depth and rate are affine, and the moment integrand
// is cubic. Three-point Gauss quadrature integrates the varying strip width.
fn floor(s: State, q: DQuat, w: DVec3) -> (f64, DVec3) {
    let ex = q * DVec3::X;
    let ey = q * DVec3::Y;
    let r0 = q * (DVec3::NEG_Z * 0.095);
    for x in [-0.18, 0.18] {
        for y in [-0.1, 0.1] {
            let p = s.center + r0 + x * ex + y * ey;
            assert!((p.x + 0.085).abs() < 0.375 && p.y.abs() < 0.425);
        }
    }
    let cuts = [
        [-(s.center + r0).z, -ex.z, -ey.z],
        [
            1. - 2.5 * (s.p / MASS + w.cross(r0)).z,
            -2.5 * w.cross(ex).z,
            -2.5 * w.cross(ey).z,
        ],
    ];
    let mut spans = vec![-0.1, 0.1];
    let mut split = |y: f64| {
        if y.abs() < 0.1 {
            spans.push(y);
        }
    };
    for [c, a, b] in cuts {
        for x in [-0.18, 0.18] {
            split(-(c + a * x) / b);
        }
    }
    let [[c, a, b], [d, e, f]] = cuts;
    split((a * d - e * c) / (e * b - a * f));
    spans.sort_by(f64::total_cmp);
    let nodes = [
        (-0.7745966692414834, 5. / 9.),
        (0., 8. / 9.),
        (0.7745966692414834, 5. / 9.),
    ];
    let (mut force, mut moment) = (0., DVec3::ZERO);
    for pair in spans.windows(2) {
        let half = (pair[1] - pair[0]) * 0.5;
        for (v, wy) in nodes {
            let y = pair[0] + half * (1. + v);
            let (mut lo, mut hi) = (-0.18_f64, 0.18_f64);
            for [c, a, b] in cuts {
                let intercept = c + b * y;
                if a > 0. {
                    lo = lo.max(-intercept / a);
                } else if a < 0. {
                    hi = hi.min(-intercept / a);
                } else if intercept <= 0. {
                    hi = lo;
                }
            }
            if hi <= lo {
                continue;
            }
            let dx = 0.5 * (hi - lo);
            for (u, wx) in nodes {
                let x = lo + dx * (1. + u);
                let r = r0 + x * ex + y * ey;
                let depth = -(s.center + r).z;
                let rate = 1. - 2.5 * (s.p / MASS + w.cross(r)).z;
                let n = 1_200_000. * depth * rate * wx * wy * dx * half;
                force += n;
                moment += r.cross(n * DVec3::Z);
            }
        }
    }
    (force, moment)
}
fn rhs(s: State, miss: bool) -> State {
    let q = s.rotation();
    let omega = q * ((q.conjugate() * s.l) / INERTIA);
    let (lower, torque) = floor(s, q, omega);
    let mut side = 0.;
    let mut arm = DVec3::ZERO;
    let geometry = DQuat::from_euler(EulerRot::ZYX, 0.23, 0.15, 0.);
    let y_center = if miss { 0.65 } else { 0. };
    for x in [-0.28, 0.28] {
        for y in [-0.07, 0.07] {
            for z in [-0.04, 0.04] {
                let r = q * (0.07 * DVec3::Z + geometry * DVec3::new(x, y, z));
                let p = s.center + r;
                if p.x > 0.30 && (p.y - y_center).abs() < 0.225 && (p.z - 0.16).abs() < 0.16 {
                    assert_eq!(
                        side, 0.,
                        "reference fixture no longer has one engaged corner"
                    );
                    side = 40_000. * (p.x - 0.30);
                    arm = r;
                }
            }
        }
    }
    let force = -side * DVec3::X;
    let qdot = DQuat::from_xyzw(omega.x, omega.y, omega.z, 0.) * q;
    State {
        center: s.p / MASS,
        quat: 0.5 * DVec4::from_array(qdot.to_array()),
        p: force + lower * DVec3::Z + MASS * G,
        l: torque + arm.cross(force),
        upper_ns: side,
        lower_ns: lower,
    }
}
pub fn trajectory(grids: usize, ticks: usize, dt: f64, miss: bool) -> Vec<State> {
    let mut s = State::initial();
    let h = dt / grids as f64;
    let mut frames = vec![s];
    for _ in 0..ticks {
        for _ in 0..grids {
            let a = rhs(s, miss);
            let b = rhs(s.plus(a, h * 0.5), miss);
            let c = rhs(s.plus(b, h * 0.5), miss);
            let d = rhs(s.plus(c, h), miss);
            s = s
                .plus(a, h / 6.)
                .plus(b, h / 3.)
                .plus(c, h / 3.)
                .plus(d, h / 6.);
            s.quat = s.quat.normalize();
        }
        frames.push(s);
    }
    frames
}
