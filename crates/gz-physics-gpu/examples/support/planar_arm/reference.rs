// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cartesian particle Jacobians and their centripetal accelerations; no GPU mass coefficients.
use gz_physics_gpu::PlanarArmLane as Lane;

pub fn turn(a: f64) -> [f64; 2] {
    [a.cos(), a.sin()]
}
pub fn fixture(i: usize) -> Lane {
    let mix = |stride, offset| ((i * stride + offset) % 2048usize) as f64 / 2047.;
    let length = [0.28 + 0.08 * mix(131, 97), 0.18 + 0.08 * mix(911, 13)];
    let mass = [0.65 + 0.25 * mix(775, 51), 0.22 + 0.18 * mix(31, 39)];
    let q = [-0.35 + 0.55 * mix(517, 13), 0.25 + 0.35 * mix(71, 997)];
    let goal = [0.75 + 0.25 * mix(895, 421), -0.72 + 0.4 * mix(73, 211)];
    Lane {
        angle: q.map(turn),
        speed: [0.; 2],
        length,
        center: length.map(|l| 0.5 * l),
        mass,
        inertia: std::array::from_fn(|k| mass[k] * length[k].powi(2) / 12.),
        tip_mass: [0., 0.1, 0.25][i % 3],
        gravity: 9.81,
        target: goal.map(turn),
        stiffness: [16., 5.],
        damping: [2.3, 0.41],
        effort_limit: [8., 2.8],
        effort: [0.; 2],
    }
}
pub fn angles(v: &Lane) -> [f64; 2] {
    v.angle.map(|p| p[1].atan2(p[0]))
}
pub fn servo(v: &Lane, q: [f64; 2], w: [f64; 2]) -> [f64; 2] {
    std::array::from_fn(|i| {
        (v.stiffness[i] * (v.target[i][1].atan2(v.target[i][0]) - q[i]).sin() - v.damping[i] * w[i])
            .clamp(-v.effort_limit[i], v.effort_limit[i])
    })
}
#[derive(Clone, Copy)]
pub struct State {
    pub q: [f64; 2],
    pub w: [f64; 2],
}
impl State {
    pub fn new(l: &Lane) -> Self {
        Self {
            q: angles(l),
            w: l.speed,
        }
    }
    pub fn points(self, l: &Lane) -> [[f64; 2]; 2] {
        let a = turn(self.q[0]).map(|v| l.length[0] * v);
        let b = turn(self.q[0] + self.q[1]).map(|v| l.length[1] * v);
        [a, [a[0] + b[0], a[1] + b[1]]]
    }
    fn derivative(self, l: &Lane) -> Self {
        let u = turn(self.q[0]);
        let v = turn(self.q[0] + self.q[1]);
        let mut a = l.inertia[0] + l.inertia[1];
        let mut b = l.inertia[1];
        let mut d = l.inertia[1];
        let mut f = servo(l, self.q, self.w);
        for (m, x, y) in [
            (l.mass[0], l.center[0], 0.),
            (l.mass[1], l.length[0], l.center[1]),
            (l.tip_mass, l.length[0], l.length[1]),
        ] {
            let j2 = [-y * v[1], y * v[0]];
            let j1 = [-x * u[1] + j2[0], x * u[0] + j2[1]];
            let acc = std::array::from_fn(|i| {
                x * u[i] * self.w[0].powi(2) + y * v[i] * (self.w[0] + self.w[1]).powi(2)
            });
            let dot = |a: [f64; 2], b: [f64; 2]| a[0] * b[0] + a[1] * b[1];
            a += m * dot(j1, j1);
            b += m * dot(j1, j2);
            d += m * dot(j2, j2);
            f[0] += m * (dot(j1, acc) - j1[1] * l.gravity);
            f[1] += m * (dot(j2, acc) - j2[1] * l.gravity);
        }
        let det = a * d - b * b;
        Self {
            q: self.w,
            w: [(d * f[0] - b * f[1]) / det, (a * f[1] - b * f[0]) / det],
        }
    }
    pub fn discrete(&mut self, l: &Lane) {
        let start = self.derivative(l);
        let mut mid = self.offset(start, 0.0005);
        for i in 0..2 {
            mid.q[i] = self.q[i] + 2. * (0.00025 * self.w[i]).atan();
        }
        let dw = mid.derivative(l).w;
        for (i, acc) in dw.into_iter().enumerate() {
            self.q[i] += 2. * (0.0005 * mid.w[i]).atan();
            self.w[i] += 0.001 * acc;
        }
    }
    fn offset(self, by: Self, h: f64) -> Self {
        Self {
            q: std::array::from_fn(|i| self.q[i] + h * by.q[i]),
            w: std::array::from_fn(|i| self.w[i] + h * by.w[i]),
        }
    }
    pub fn fine(&mut self, l: &Lane) {
        for _ in 0..8 {
            self.rk4(l, 0.001 / 8.);
        }
    }
    pub fn rk4(&mut self, l: &Lane, h: f64) {
        let a = self.derivative(l);
        let b = self.offset(a, h * 0.5).derivative(l);
        let c = self.offset(b, h * 0.5).derivative(l);
        let d = self.offset(c, h).derivative(l);
        *self = self
            .offset(a, h / 6.)
            .offset(b, h / 3.)
            .offset(c, h / 3.)
            .offset(d, h / 6.);
    }
}
