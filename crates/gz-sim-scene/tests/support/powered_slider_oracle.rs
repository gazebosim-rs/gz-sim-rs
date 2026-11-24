// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Planar rigid-body oracle with internal slider force. All inputs use coordinates
//! pointing toward the upper stop. Reflect X and angle together for the other fixture.
use glam::DVec2;

pub(super) const L: f64 = 0.10;
pub(super) const D: f64 = 0.02;
pub(super) const M: f64 = 0.08;
pub(super) const MP: f64 = 0.20;
pub(super) const I: f64 = 0.0004266666666666667 + 0.00003208666666666667;
pub(super) const SPEED: f64 = 0.75;
const FACE_AND_RADIUS: f64 = 0.009 + 0.015;
const GAP: f64 = 50e-6;
pub(super) type State = [f64; 4]; // θ, s, angular velocity, slider velocity.

#[derive(Clone, Copy, Debug)]
pub(super) struct Case {
    pub name: &'static str,
    pub upper: f64,
    pub approach: f64,
    pub force: f64,
}
pub(super) const CASES: [Case; 6] = [
    Case {
        name: "seated",
        upper: 0.,
        approach: 1.,
        force: 2.,
    },
    Case {
        name: "away",
        upper: 0.,
        approach: -1.,
        force: 2.,
    },
    Case {
        name: "interior",
        upper: 0.015,
        approach: 1.,
        force: 2.,
    },
    Case {
        name: "near_before",
        upper: 0.05e-6,
        approach: 1.,
        force: 2.,
    },
    Case {
        name: "near_after",
        upper: 2e-6,
        approach: 1.,
        force: 2.,
    },
    Case {
        name: "force_away",
        upper: 0.,
        approach: 1.,
        force: -2.,
    },
];

fn mass_theta(s: f64) -> f64 {
    I + M * (L * L + s * s)
}
fn rhs([_, s, w, u]: State, f: f64) -> State {
    let a = (L * f + M * s * w * (L * w - 2. * u)) / (I + M * s * s);
    [w, u, a, L * a + s * w * w + f / M]
}
fn add(q: State, k: State, h: f64) -> State {
    std::array::from_fn(|i| q[i] + k[i] * h)
}
fn integrate(mut q: State, f: f64, t: f64) -> State {
    if t <= 0. {
        return q;
    }
    let n = (t / 2e-6).ceil() as usize;
    let h = t / n as f64;
    for _ in 0..n {
        let a = rhs(q, f);
        let b = rhs(add(q, a, h / 2.), f);
        let c = rhs(add(q, b, h / 2.), f);
        let d = rhs(add(q, c, h), f);
        q = std::array::from_fn(|i| q[i] + h * (a[i] + 2. * b[i] + 2. * c[i] + d[i]) / 6.);
    }
    q
}
fn first_crossing(mut hi: f64, past: impl Fn(f64) -> bool) -> f64 {
    let mut lo = 0.;
    for _ in 0..48 {
        let middle = (hi + lo) / 2.;
        if past(middle) {
            hi = middle;
        } else {
            lo = middle;
        }
    }
    hi
}
/// Valid up to the first rebound or reversal; all selected incoming and first
/// loaded intervals have monotone travel or are initially pressed against the stop.
fn advance(q: State, case: Case, t: f64) -> (State, Option<f64>) {
    let free = integrate(q, case.force, t);
    if free[1] < case.upper || free[3] < 0. {
        return (free, None);
    }
    let time = if q[1] == case.upper {
        0.
    } else {
        first_crossing(t, |t| integrate(q, case.force, t)[1] >= case.upper)
    };
    let mut hit = integrate(q, case.force, time);
    // Stop impulse supplies no generalized hinge torque.
    hit[2] -= M * L * hit[3] / mass_theta(case.upper);
    hit[3] = 0.;
    hit[1] = case.upper;
    if case.force + M * case.upper * hit[2] * hit[2] < 0. {
        // A stop can arrest a fast slider moving against its drive, but cannot pull
        // it back against the stop after the incident velocity has been removed.
        return (integrate(hit, case.force, t - time), Some(time));
    }
    hit[0] += hit[2] * (t - time);
    (hit, Some(time))
}
fn sphere(t: f64, approach: f64) -> DVec2 {
    DVec2::new(approach * (SPEED * t - FACE_AND_RADIUS - GAP), L + D)
}
pub(super) struct Hit {
    pub time: f64,
    pub pre: State,
    pub post: State,
    pub normal: DVec2,
    pub approach_velocity: f64,
    pub impulse: f64,
    pub stop_impulse: f64,
    pub stop_time: Option<f64>,
    pub blocked: bool,
}
impl Hit {
    pub fn solve(c: Case, restitution: f64) -> Self {
        let time = first_crossing(150e-6, |t| {
            let (q, _) = advance([0.; 4], c, t);
            let x = DVec2::from_angle(q[0]);
            -c.approach * (sphere(t, c.approach).dot(x) - q[1]) <= FACE_AND_RADIUS
        });
        let (pre, stop_time) = advance([0.; 4], c, time);
        let x = DVec2::from_angle(pre[0]);
        let point_height = sphere(time, c.approach).dot(x.perp());
        let d = point_height - L;
        assert!(d.abs() < 0.0335);
        let normal = c.approach * x;
        let approach_velocity =
            c.approach * SPEED * normal.x - c.approach * (pre[3] - point_height * pre[2]);
        let free_mass = I + M * pre[1] * pre[1];
        let free_j =
            (1. + restitution) * approach_velocity / (1. / MP + 1. / M + d * d / free_mass);
        let free_w = pre[2] - c.approach * d * free_j / free_mass;
        let free_u = pre[3] + c.approach * free_j / M + L * (free_w - pre[2]);
        let blocked = stop_time.is_some() && free_u > 0.;
        let (impulse, w, u, stop_impulse) = if blocked {
            let mass = mass_theta(pre[1]);
            let j = (1. + restitution) * approach_velocity
                / (1. / MP + point_height * point_height / mass);
            let w = pre[2] - c.approach * point_height * j / mass;
            (j, w, 0., -M * L * (w - pre[2]) - c.approach * j)
        } else {
            (free_j, free_w, free_u, 0.)
        };
        Hit {
            time,
            pre,
            post: [pre[0], pre[1], w, u],
            normal,
            approach_velocity,
            impulse,
            stop_impulse,
            stop_time,
            blocked,
        }
    }
    pub fn before(&self, c: Case, t: f64) -> State {
        advance([0.; 4], c, t).0
    }
    pub fn next_stop(&self, c: Case) -> Option<f64> {
        if self.blocked {
            None
        } else {
            advance(self.post, c, 50e-6).1.map(|t| self.time + t)
        }
    }
    pub fn at_end(&self, c: Case, t: f64) -> State {
        if self.blocked {
            let mut q = self.post;
            q[0] += q[2] * (t - self.time);
            q
        } else {
            advance(self.post, c, t - self.time).0
        }
    }
}
