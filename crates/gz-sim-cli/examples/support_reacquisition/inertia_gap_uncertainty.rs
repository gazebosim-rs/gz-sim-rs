// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Forward enclosures for the actual root time, state, and single-site reaction.
//! All errors originate at a matched lower-impact cut and its upper-gap bracket.
use super::*;
#[derive(Default, Clone, Copy)]
pub(super) struct Error {
    pub time: f64,
    pub position: f64,
    pub angle: f64,
    pub velocity: f64,
    pub momentum: f64,
    pub impulses: [f64; 2],
}
impl Error {
    pub fn data(self) -> Value {
        json!({"time_s":self.time,"position_m":self.position,"angle_rad":self.angle,"velocity_m_s":self.velocity,"momentum_nms":self.momentum,"impulses_ns":self.impulses})
    }
    pub fn velocity(self, r: &Physical, x: Flight, rr: f64) -> f64 {
        let w = (x.l.length() + self.momentum) / r.body.lower;
        self.velocity + rr * (self.momentum / r.body.lower + 3. * w * self.angle)
    }
    pub fn accel(self, r: &Physical, x: Flight, rr: f64, t: f64) -> f64 {
        let w = (x.l.length() + self.momentum) / r.body.lower;
        let theta = (2. * w * t).exp() * (self.angle + t * self.momentum / r.body.lower);
        let diff = (r.body.upper - r.body.lower) / r.body.lower;
        let dw = self.momentum / r.body.lower + w * theta;
        rr * ((2. + diff) * w * dw + (1. + 0.5 * diff) * w * w * theta)
    }
    pub fn bend(
        self,
        r: &Physical,
        x: Flight,
        site: Site,
        plane: Face,
        t: f64,
        slices: usize,
    ) -> Option<(Interval, f64)> {
        let (mut b, a) = bend(r, x, site, plane, t, slices, true)?;
        let da = self.accel(r, x, vertices()[site.k].length(), t);
        let dz = 0.5 * t * (da * t);
        b.lo -= dz;
        b.hi += dz;
        Some((b, a - da))
    }
    pub fn covered(self, r: &Physical, x: Flight, h: f64, surfaces: [Face; 2]) -> f64 {
        let rr = vertices().iter().map(|r| r.length()).fold(0f64, f64::max);
        let speed = surfaces.iter().map(|p| p.v.length()).fold(0., f64::max);
        self.position
            + rr * self.angle
            + speed * self.time
            + self.velocity(r, x, rr) * h
            + 0.5 * self.accel(r, x, rr, h) * h * h
    }
    pub fn flight(self, r: &Physical, mut x: Flight, t: Interval) -> (Flight, Self) {
        let dt = t.mid;
        let steps = (dt / 2e-7).ceil().max(1.) as usize;
        let h = dt / steps as f64;
        let min = r.body.lower;
        let wm = (x.l.length() + self.momentum) / min;
        let max_d = (r.body.upper - min) / min;
        let a = 0.5 * max_d * wm * wm;
        let b = (1. + max_d) * wm * a;
        let q3 = 0.5 * b + 0.75 * wm * a + 0.125 * wm.powi(3);
        let grow = (2.5 * wm * h).exp();
        let skew = (t.hi - dt).max(dt - t.lo);
        let mut angular = self.angle;
        let mut xr = self;
        for _ in 0..steps {
            let origin = x;
            let w = r.body.omega(x.q, x.l);
            let alpha = r.body.accel(x.q, w);
            let dq = (pure(w) * x.q) * 0.5;
            let ddq = (pure(alpha) * x.q + pure(w) * dq) * 0.5;
            let expansion = x.q + dq * h + ddq * (0.5 * h * h);
            x.advance(r, h);
            // Twice the quaternion error bounds orientation distance locally,
            // with a 3x factor for conversion and roundoff in the short steps.
            let trunc =
                3. * ((x.q - expansion).length() + q3 * h.powi(3) / 6. + 16. * f64::EPSILON);
            angular = grow * (angular + h * self.momentum / min) + trunc;
            xr.position += h * xr.velocity
                + 16. * f64::EPSILON * (1. + origin.com.length() + origin.v.length() * h);
            xr.velocity += 16. * f64::EPSILON * (1. + origin.v.length() + r.gravity.length() * h);
        }
        xr.time += skew;
        xr.position += skew * (x.v.length() + xr.velocity + r.gravity.length() * skew);
        xr.velocity += r.gravity.length() * skew;
        xr.angle = angular + wm * skew;
        (x, xr)
    }
    /// For a fixed normal impulse, the tangential Schur minimizer is a
    /// projection onto a disk. Its Lipschitz constant versus disk radius is
    /// (k+1)/(2 sqrt(k)), where k is the Schur condition number. Eliminating
    /// the normal velocity gives a contraction in the disk radius. A change
    /// in Delassus can be treated as a velocity residual deltaK * true_J.
    pub fn impact(
        mut self,
        r: &Physical,
        x: Flight,
        arm: DVec3,
        law: Law,
        mu: f64,
        side: usize,
    ) -> Option<Self> {
        let gain = gz_physics::PointImpactSensitivity::new(
            law.effective.transpose().to_cols_array_2d(),
            mu,
        )?;
        let rr = arm.length();
        let rate = self.velocity(r, x, rr);
        let dk = 4. * rr * rr * self.angle / r.body.lower
            + 64. * f64::EPSILON * (1. / M + rr * rr / r.body.lower);
        let j = law.impulse.length();
        let round =
            64. * f64::EPSILON * (1. + law.before.length() + j * (1. / M + rr * rr / r.body.lower));
        let dj = gain.impulse_error(j, rate + round, dk)?;
        self.impulses[side] += dj;
        self.velocity += dj / M + 16. * f64::EPSILON * (1. + x.v.length() + j / M);
        self.momentum +=
            rr * (dj + (j + dj) * self.angle) + 16. * f64::EPSILON * (1. + x.l.length() + rr * j);
        Some(self)
    }
}
