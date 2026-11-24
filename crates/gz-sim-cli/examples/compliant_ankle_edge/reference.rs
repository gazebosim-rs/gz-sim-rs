// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Event-time inelastic contact followed by a unilateral acceleration-level reaction.
use super::planar::{State, derivative, mass, tangent, up};
use glam::{DVec2, DVec4};

#[derive(Clone, Copy)]
pub struct Fixture {
    pub right: f64,
    pub shank_half: f64,
}
impl Fixture {
    fn toe(self, s: State) -> DVec2 {
        // During these trials the lowest shank feature is the distal edge.
        assert!(s.q.w.sin() < 0. && s.q.w.cos() < 0.);
        (0.20 + self.shank_half) * up(s.q.w) - 0.04 * tangent(s.q.w)
    }
    fn gap(self, s: State) -> f64 {
        let toe = self.toe(s);
        let p = DVec2::new(s.q.x, s.q.y) + 0.16 * up(s.q.z) + toe;
        assert!(p.x > self.right - 1.20 && p.x < self.right);
        p.y
    }
    fn gradient(self, s: State) -> DVec4 {
        DVec4::new(0., 1., -0.16 * s.q.z.sin(), -self.toe(s).x)
    }
    fn rhs(self, s: State, moment: f64, supported: bool) -> (State, f64) {
        let mut d = derivative(
            s,
            false,
            moment,
            0.16,
            Some([self.right - 1.20, self.right]),
        );
        if !supported {
            return (d, 0.);
        }
        let g = self.gradient(s);
        let inv = mass(s, false, 0.16).0.inverse() * g;
        let bias = -0.16 * s.q.z.cos() * s.dq.z.powi(2) - self.toe(s).y * s.dq.w.powi(2);
        let normal = (-(g.dot(d.dq) + bias) / g.dot(inv)).max(0.);
        d.dq += inv * normal;
        (d, normal)
    }
    fn step(self, s: State, h: f64, m: f64, loaded: bool) -> (State, f64) {
        let (a, fa) = self.rhs(s, m, loaded);
        let (b, fb) = self.rhs(s.plus(a, h / 2.), m, loaded);
        let (c, fc) = self.rhs(s.plus(b, h / 2.), m, loaded);
        let (d, fd) = self.rhs(s.plus(c, h), m, loaded);
        (
            s.plus(a, h / 6.)
                .plus(b, h / 3.)
                .plus(c, h / 3.)
                .plus(d, h / 6.),
            h * (fa + 2. * fb + 2. * fc + fd) / 6.,
        )
    }
}
#[derive(Clone, Copy)]
pub struct Sample {
    pub state: State,
    pub hard: f64,
    pub onset: Option<f64>,
    pub impact: f64,
}
pub fn moment(tick: usize) -> f64 {
    if tick >= 33 { -0.45 } else { 0. }
}
pub fn solve(f: Fixture, ticks: usize, dt: f64, grid: usize) -> Vec<Sample> {
    let mut s = State {
        q: DVec4::new(0., 0.02477291666666667, 0., -1.96),
        dq: DVec4::new(0.75, 0., 0., 0.),
        ..Default::default()
    };
    let h = dt / grid as f64;
    let mut hard = 0.;
    let mut onset = None;
    let mut impact = 0.;
    let mut out = vec![Sample {
        state: s,
        hard,
        onset,
        impact,
    }];
    for tick in 1..=ticks {
        for sub in 0..grid {
            let m = moment(tick);
            let loaded = f.gap(s) <= 1e-10 && f.gradient(s).dot(s.dq).abs() < 1e-8;
            let (next, j) = f.step(s, h, m, loaded);
            if !loaded && f.gap(next) < 0. {
                let (mut low, mut high) = (0., h);
                for _ in 0..42 {
                    let t = 0.5 * (low + high);
                    if f.gap(f.step(s, t, m, false).0) > 0. {
                        low = t;
                    } else {
                        high = t;
                    }
                }
                s = f.step(s, high, m, false).0;
                let g = f.gradient(s);
                let inv = mass(s, false, 0.16).0.inverse() * g;
                let pulse = (-g.dot(s.dq) / g.dot(inv)).max(0.);
                s.dq += pulse * inv;
                onset.get_or_insert((tick - 1) as f64 * dt + sub as f64 * h + high);
                impact += pulse;
                let rest = f.step(s, h - high, m, true);
                s = rest.0;
                hard += pulse + rest.1;
            } else {
                s = next;
                hard += j;
            }
        }
        out.push(Sample {
            state: s,
            hard,
            onset,
            impact,
        });
    }
    out
}
