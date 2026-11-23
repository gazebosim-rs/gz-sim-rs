// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The sphere center relative to a freely falling material. The time
//! parameter spans one public interval, from zero through one.
use super::*;

#[derive(Clone, Copy)]
pub(super) struct Trajectory {
    pub center: DVec3,
    pub delta: DVec3,
    pub curve: DVec3,
    pub orbit: Option<Orbit>,
}
#[derive(Clone, Copy)]
pub(super) struct Orbit {
    pub radial: DVec3,
    pub transverse: DVec3,
    pub angle: f64,
}
impl Trajectory {
    pub fn at(&self, t: f64) -> DVec3 {
        let linear = self.center + t * (self.delta + t * self.curve);
        self.orbit.map_or(linear, |o| {
            let theta = t * o.angle;
            // 1-cos(theta) loses the radial sagitta of a short arc.
            linear - (2. * (0.5 * theta).sin().powi(2)) * o.radial + theta.sin() * o.transverse
        })
    }
    pub fn tangent(&self, t: f64) -> DVec3 {
        let linear = self.delta + (2. * t) * self.curve;
        self.orbit.map_or(linear, |o| {
            let (s, c) = (t * o.angle).sin_cos();
            linear + o.angle * (c * o.transverse - s * o.radial)
        })
    }
    pub fn bounds(&self) -> (DVec3, DVec3) {
        let end = self.center + self.delta + self.curve;
        let mut lo = self.center.min(end);
        let mut hi = self.center.max(end);
        for k in 0..3 {
            if self.curve[k] != 0. {
                let t = -0.5 * self.delta[k] / self.curve[k];
                if (0.0..1.).contains(&t) {
                    let value = self.center[k] + t * (self.delta[k] + t * self.curve[k]);
                    lo[k] = lo[k].min(value);
                    hi[k] = hi[k].max(value);
                }
            }
        }
        let padding = self.orbit.map_or(DVec3::ZERO, |o| {
            let rotation = o.angle.abs();
            // Small steps use velocity and acceleration bounds. Large arcs
            // use a coordinate-wise bound over the complete circle.
            let half_chord = (0.5 * rotation * rotation).min(2.);
            half_chord * o.radial.abs() + rotation.min(1.) * o.transverse.abs()
        });
        (lo - padding, hi + padding)
    }
    pub fn speed_bound(&self) -> f64 {
        self.delta
            .length()
            .max((self.delta + 2. * self.curve).length())
            + self.orbit.map_or(0., |o| o.angle.abs() * o.radial.length())
    }
    pub fn acceleration_bound(&self) -> f64 {
        2. * self.curve.length()
            + self
                .orbit
                .map_or(0., |o| o.angle * o.angle * o.radial.length())
    }
}

/// Increasing root candidates on [0,1]. A quartic has at most four exact
/// roots. With roundoff at interval endpoints and derivative critical points,
/// the recursive monotone partition can emit up to seven candidates.
#[derive(Default)]
pub(super) struct Roots {
    values: [f64; 7],
    count: usize,
}
impl Roots {
    fn push(&mut self, t: f64) {
        if !(-16. * f64::EPSILON..=1. + 16. * f64::EPSILON).contains(&t) {
            return;
        }
        let t = t.clamp(0., 1.);
        if self.count == 0 || t > self.values[self.count - 1] {
            self.values[self.count] = t;
            self.count += 1;
        }
    }
    pub fn iter(&self) -> impl Iterator<Item = f64> + '_ {
        self.values[..self.count].iter().copied()
    }
}
fn eval(c: &[f64], t: f64) -> f64 {
    c.iter().rev().fold(0.0_f64, |v, a| v.mul_add(t, *a))
}
fn sign(c: &[f64], t: f64) -> f64 {
    let y = eval(c, t);
    // A repeated root can sit a few ulps on either side of zero. Preserve
    // its stationary time without manufacturing a pair of incoming roots.
    let absolute = c.iter().rev().fold(0.0_f64, |v, a| v.mul_add(t, a.abs()));
    if y.abs() <= 24. * f64::EPSILON * absolute {
        0.
    } else {
        y
    }
}
fn roots(c: &[f64]) -> Roots {
    let mut n = c.len();
    while n > 1 && c[n - 1] == 0. {
        n -= 1;
    }
    let c = &c[..n];
    let mut out = Roots::default();
    if n < 2 {
        return out;
    }
    if n == 2 {
        out.push(-c[0] / c[1]);
        return out;
    }
    let mut derivative = [0.; 4];
    for i in 1..n {
        derivative[i - 1] = i as f64 * c[i];
    }
    let critical = roots(&derivative[..n - 1]);
    let mut low = 0.;
    let mut left = sign(c, low);
    if left == 0. {
        out.push(low);
    }
    // Between successive derivative roots the polynomial is monotonic.
    for high in critical.iter().filter(|&t| t > 0. && t < 1.).chain([1.]) {
        let right = sign(c, high);
        if left != 0. && right != 0. && left.is_sign_positive() != right.is_sign_positive() {
            let (mut a, mut b) = (low, high);
            for _ in 0..64 {
                let m = a + (b - a) * 0.5;
                if m == a || m == b {
                    break;
                }
                let y = eval(c, m);
                if y == 0. {
                    a = m;
                    b = m;
                    break;
                }
                if y.is_sign_positive() == left.is_sign_positive() {
                    a = m;
                } else {
                    b = m;
                }
            }
            out.push(a + (b - a) * 0.5);
        }
        if right == 0. {
            out.push(high);
        }
        low = high;
        left = right;
    }
    out
}
pub(super) fn radial(offset: DVec3, delta: DVec3, curve: DVec3, radius: f64) -> Roots {
    if curve == DVec3::ZERO {
        let mut out = Roots::default();
        if let Some(t) = super::entry(offset, delta, radius) {
            out.push(t);
        }
        return out;
    }
    let scale = offset
        .abs()
        .max(delta.abs())
        .max(curve.abs())
        .max_element()
        .max(radius);
    let (r, d, e, g) = (offset / scale, delta / scale, curve / scale, radius / scale);
    roots(&[
        (r.length() - g) * (r.length() + g),
        2. * r.dot(d),
        d.length_squared() + 2. * r.dot(e),
        2. * d.dot(e),
        e.length_squared(),
    ])
}
pub(super) fn plane(distance: f64, rate: f64, acceleration: f64, target: f64) -> Roots {
    roots(&[distance - target, rate, acceleration])
}
