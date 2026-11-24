// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent spatial momentum construction. Try stopping normal motion of
//! the entire face; if its pressure center lies outside the sole, test only
//! supporting edges and individual vertices using the actual world lever arms.
use super::*;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Mode {
    Face,
    Edge(usize, usize),
    Vertex(usize),
    Free,
}
#[derive(Clone, Copy)]
pub(super) struct Law {
    pub mode: Mode,
    pub after: State,
    pub total: f64,
    pub center: Option<DVec2>,
    pub post: [f64; 4],
    pub loss: f64,
}
#[derive(Clone, Copy)]
pub(super) struct Sole {
    pub shape: Shape,
    pub incoming: State,
}
impl Sole {
    pub fn arms(self) -> [DVec3; 4] {
        std::array::from_fn(|k| self.incoming.motion.q * self.shape.vertices[k])
    }
    pub fn incoming_normal(self) -> [f64; 4] {
        std::array::from_fn(|k| self.shape.point(k, self.incoming).1.z)
    }
    pub fn iw(self) -> DMat3 {
        let q = DMat3::from_quat(self.incoming.motion.q);
        q * self.shape.inertia * q.transpose()
    }
    pub fn mobility(self) -> [[f64; 4]; 4] {
        let t = self.arms().map(|r| r.cross(DVec3::Z));
        let inverse = self.iw().inverse();
        t.map(|r| t.map(|a| 1. / self.shape.mass + r.dot(inverse * a)))
    }
    fn attempt(self, mode: Mode, j: f64, cop: DVec3) -> Option<Law> {
        let body = self.shape;
        let s = self.incoming;
        if j < 0. || !j.is_finite() {
            return None;
        }
        let local = s.motion.q.conjugate() * cop;
        let x = (local.x - self.shape.vertices[0].x)
            / (self.shape.vertices[2].x - self.shape.vertices[0].x);
        let y = (local.y - self.shape.vertices[0].y)
            / (self.shape.vertices[1].y - self.shape.vertices[0].y);
        let center = (j > 0.).then_some(DVec2::new(2. * x - 1., 2. * y - 1.));
        if j > 0. && (x < -1e-10 || x > 1. + 1e-10 || y < -1e-10 || y > 1. + 1e-10) {
            return None;
        }
        let after = State {
            v: s.v + (j / body.mass) * DVec3::Z,
            motion: Motion {
                q: s.motion.q,
                w: s.motion.w + self.iw().inverse() * cop.cross(j * DVec3::Z),
            },
            c: s.c,
        };
        let post = std::array::from_fn(|k| body.point(k, after).1.z);
        if post.iter().any(|v| *v < -2e-10) {
            return None;
        }
        let sum_v = s.v + s.motion.w.cross(cop) + after.v + after.motion.w.cross(cop);
        let loss = -0.5 * j * sum_v.z;
        if loss < -1e-10 || (body.energy(s) - body.energy(after) - loss).abs() > 2e-10 {
            return None;
        }
        Some(Law {
            mode,
            after,
            total: j,
            center,
            post,
            loss,
        })
    }
    pub fn solve(self) -> Result<Law> {
        let s = self.incoming;
        let arms = self.arms();
        let n = DVec3::Z;
        if arms
            .iter()
            .map(|p| (p.z - arms[0].z).abs())
            .fold(0., f64::max)
            > 1e-10
        {
            return Err("surface is not level with support".into());
        }
        let u = self.incoming_normal();
        if u.iter().all(|x| *x >= 0.) {
            return self
                .attempt(Mode::Free, 0., DVec3::ZERO)
                .ok_or_else(|| "unresolved free state".into());
        }
        let iw = self.iw();
        let inv = iw.inverse();
        let momentum = iw * s.motion.w;
        // No normal pressure produces torque around the supporting plane normal.
        // A fully stopped face therefore retains this unique yaw rate.
        let final_spin = n * (momentum.z / n.dot(iw * n));
        let delta_h = iw * final_spin - momentum;
        let j = -self.shape.mass * s.v.z;
        if j > 0. {
            let cp = n.cross(delta_h) / j + n * arms[0].z;
            if let Some(l) = self.attempt(Mode::Face, j, cp) {
                return Ok(l);
            }
        }
        let t = arms.map(|r| r.cross(n));
        let w = |a: usize, b: usize| 1. / self.shape.mass + t[a].dot(inv * t[b]);
        for (a, b) in [(0, 1), (0, 2), (1, 3), (2, 3)] {
            let m = DMat2::from_cols(DVec2::new(w(a, a), w(b, a)), DVec2::new(w(a, b), w(b, b)));
            let pair = -(m.inverse() * DVec2::new(u[a], u[b]));
            if pair.min_element() > 1e-12 {
                let total = pair.x + pair.y;
                let cp = (pair.x * arms[a] + pair.y * arms[b]) / total;
                if let Some(l) = self.attempt(Mode::Edge(a, b), total, cp) {
                    return Ok(l);
                }
            }
        }
        for k in 0..4 {
            let j = -u[k] / w(k, k);
            if j > 0.
                && let Some(l) = self.attempt(Mode::Vertex(k), j, arms[k])
            {
                return Ok(l);
            }
        }
        Err("unresolved physical normal pressure".into())
    }
}
