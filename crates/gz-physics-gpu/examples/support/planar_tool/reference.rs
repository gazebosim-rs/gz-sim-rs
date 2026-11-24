// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::reference::{State, fixture as free_arm, turn};
use gz_physics_gpu::PlanarToolLane as Lane;

pub fn fixture(i: usize) -> Lane {
    let mut arm = free_arm(i);
    let high = 0.48 + 0.015 * (i % 5) as f64;
    let elbow = -0.62 + 0.025 * (i % 7) as f64;
    arm.angle = [turn(high), turn(elbow)];
    arm.target = [turn(-0.36), turn(-0.50)];
    let radius = 0.018;
    let ground_z = arm.points()[1][1] - radius - 0.035 - 0.002 * (i % 4) as f64;
    Lane {
        arm,
        ground_z,
        radius,
        friction: [0., 0.2, 0.45][(i / 3) % 3],
        normal_impulse: 0.,
        tangent_impulse: 0.,
        first_contact_offset: -1.,
        first_normal_impulse: 0.,
        first_tangent_impulse: 0.,
        turns: [0.; 2],
        velocity_mode: [0.; 2],
        velocity_target: [0.; 2],
        stops: Default::default(),
        stop_impulse: [0.; 2],
    }
}
pub fn lift(i: usize) -> [f64; 2] {
    [0.82 + (i % 4) as f64 * 0.025, -0.15]
}

/// All data describes reactions on the tool; the first offset is seconds since tick start.
#[derive(Default, Clone, Copy)]
pub struct Observation {
    pub n: f64,
    pub t: f64,
    pub first: Option<f64>,
    pub first_n: f64,
    pub first_t: f64,
    pub tip: [f64; 2],
}
impl Observation {
    pub fn add(&mut self, n: f64, t: f64, time: f64) {
        if n > 0. && self.first.is_none() {
            self.first_n = n;
            self.first_t = t;
            self.first.get_or_insert(time);
        }
        self.n += n;
        self.t += t;
    }
}
fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
/// Build M by summing Cartesian point-mass and rigid-link rotational kinetic energies.
fn mobility(s: &State, l: &Lane, j: [f64; 2]) -> [f64; 2] {
    let a = &l.arm;
    let u = turn(s.q[0]);
    let v = turn(s.q[0] + s.q[1]);
    let mut m11 = a.inertia[0] + a.inertia[1];
    let mut m22 = a.inertia[1];
    let mut m12 = m22;
    for (m, x, y) in [
        (a.mass[0], a.center[0], 0.),
        (a.mass[1], a.length[0], a.center[1]),
        (a.tip_mass, a.length[0], a.length[1]),
    ] {
        let j2 = [-y * v[1], y * v[0]];
        let j1 = [-x * u[1] + j2[0], x * u[0] + j2[1]];
        m11 += m * dot(j1, j1);
        m22 += m * dot(j2, j2);
        m12 += m * dot(j1, j2);
    }
    let d = m11 * m22 - m12 * m12;
    [(m22 * j[0] - m12 * j[1]) / d, (m11 * j[1] - m12 * j[0]) / d]
}
/// Solve normal complementarity and Coulomb friction independently using a
/// scalar reduction that eliminates the normal equality before computing friction.
fn impulse(vn: f64, vt: f64, nn: f64, nt: f64, tt: f64, mu: f64) -> [f64; 2] {
    if vn >= 0. || nn <= 0. {
        return [0.; 2];
    }
    let normal = |t: f64| (-vn - nt * t) / nn;
    let t_free = vt - nt * vn / nn;
    let reduced = tt - nt * nt / nn;
    let t_stick = if reduced > 0. { -t_free / reduced } else { 0. };
    if normal(t_stick) >= 0. && t_stick.abs() <= mu * normal(t_stick) {
        return [normal(t_stick), t_stick];
    }
    let sign = -t_free.signum();
    let n = -vn / (nn + nt * sign * mu);
    [n, sign * mu * n]
}
pub fn fine_step(s: &mut State, l: &Lane) -> Observation {
    let mut obs = Observation::default();
    let h = 0.001 / 16.;
    for k in 0..16 {
        let old = s.points(&l.arm)[1][1] - l.radius - l.ground_z;
        s.rk4(&l.arm, h);
        let [elbow, tip] = s.points(&l.arm);
        let gap = tip[1] - l.radius - l.ground_z;
        if gap <= 0. {
            let jn = [tip[0], tip[0] - elbow[0]];
            let jt = [l.radius - tip[1], l.radius - (tip[1] - elbow[1])];
            let an = mobility(s, l, jn);
            let at = mobility(s, l, jt);
            let nn = dot(jn, an);
            let [n, t] = impulse(
                dot(jn, s.w),
                dot(jt, s.w),
                nn,
                dot(jt, an),
                dot(jt, at),
                l.friction,
            );
            let fraction = if old > 0. { old / (old - gap) } else { 0. };
            obs.add(n, t, (k as f64 + fraction) * h);
            for i in 0..2 {
                s.w[i] += an[i] * n + at[i] * t;
                if nn > 0. {
                    s.q[i] -= an[i] * gap / nn;
                }
            }
        }
    }
    obs.tip = s.points(&l.arm)[1];
    obs
}
