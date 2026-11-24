// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Interval extension of the deck-edge acceleration and pressure equations.
use super::*;
use crate::interval::Interval as I;
use std::ops::{Add, Mul, Neg, Sub};
#[derive(Clone, Copy, Debug)]
pub struct DeckEdgeEnvelope {
    pub normal_force: [f64; 2],
    pub center_along_edge: [f64; 2],
    pub angular_speed_bound: f64,
    pub center_speed_bound: f64,
    /// Number of certified temporal domains covering the physical interval.
    pub domains: u32,
}
#[derive(Clone, Copy)]
struct D {
    x: I,
    dx: I,
}
impl D {
    fn constant(x: f64) -> Self {
        Self {
            x: I::point(x),
            dx: I::point(0.),
        }
    }
    fn interval(x: I) -> Self {
        Self {
            x,
            dx: I::point(0.),
        }
    }
    fn over(self, d: Self) -> Option<Self> {
        let v = self.x.over(d.x)?;
        Some(Self {
            x: v,
            dx: (self.dx - v * d.dx).over(d.x)?,
        })
    }
    fn trig(self) -> (Self, Self) {
        // Sine and cosine are 1-Lipschitz. An additional rounding margin covers
        // evaluation at the midpoint; wide ranges can use the full codomain.
        let mid = 0.5 * self.x.lo + 0.5 * self.x.hi;
        let r = (self.x.hi - mid).max(mid - self.x.lo) + 16. * f64::EPSILON * (1. + mid.abs());
        let (s, c) = mid.sin_cos();
        let bound = |v: f64| I::range((v - r).max(-1.), (v + r).min(1.));
        let (s, c) = (bound(s), bound(c));
        (
            Self {
                x: s,
                dx: c * self.dx,
            },
            Self {
                x: c,
                dx: -s * self.dx,
            },
        )
    }
}
impl Add for D {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self {
            x: self.x + b.x,
            dx: self.dx + b.dx,
        }
    }
}
impl Sub for D {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        self + -b
    }
}
impl Neg for D {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            dx: -self.dx,
        }
    }
}
impl Mul for D {
    type Output = Self;
    fn mul(self, b: Self) -> Self {
        Self {
            x: self.x * b.x,
            dx: self.x * b.dx + self.dx * b.x,
        }
    }
}
fn dot(a: DVec3, b: DVec3) -> I {
    a.to_array()
        .into_iter()
        .zip(b.to_array())
        .map(|(a, b)| I::point(a) * I::point(b))
        .fold(I::point(0.), |x, y| x + y)
}
#[derive(Clone, Copy)]
struct TaylorMotion {
    initial_acc: [I; 3],
    jerk: [I; 3],
}
#[derive(Clone, Copy)]
struct Tube {
    acc: [I; 3],
    eps: f64,
    increasing: bool,
    taylor: TaylorMotion,
}
struct Limits {
    acceleration: [D; 3],
    force: I,
    cp: I,
    theta: I,
    du: I,
    yaw_rate: I,
    tilt_rate: I,
    u: I,
    local_pressure: [I; 2],
}
impl BoundDeckEdge {
    fn limits(self, span: I, acc: [I; 3], taylor: Option<TaylorMotion>) -> Option<Limits> {
        let x = self.initial;
        let curvature = I::point(0.5) * span * span;
        let coordinate = |q: f64, v: f64, index: usize| {
            let slope = I::point(v);
            if let Some(m) = taylor {
                // The jerk enclosure follows by differentiating each solved
                // acceleration over the globally validated tube. Integrating
                // it from the same exact initial state avoids loss of the
                // time/velocity correlation near a material endpoint.
                let a = m.initial_acc[index];
                let j = m.jerk[index];
                let dq = slope + a * span + j * curvature;
                let position = I::point(q)
                    + slope * span
                    + a * curvature
                    + (j * curvature * span).over(I::point(3.))?;
                Some((
                    D {
                        x: position,
                        dx: dq,
                    },
                    D {
                        x: dq,
                        dx: a + j * span,
                    },
                ))
            } else {
                let a = acc[index];
                let dq = slope + a * span;
                Some((
                    D {
                        x: I::point(q) + slope * span + a * curvature,
                        dx: dq,
                    },
                    D { x: dq, dx: a },
                ))
            }
        };
        let (u, du) = coordinate(x.u, x.du, 0)?;
        let (theta, k) = coordinate(x.tilt, x.tilt_rate, 1)?;
        let (yaw, s) = coordinate(x.yaw, x.yaw_rate, 2)?;
        let (sn, cs) = yaw.trig();
        let (st, ct) = theta.trig();
        let l = self.local_axis;
        let z = self.sole_normal;
        let t = l.cross(z);
        let q = self.body.inertia_body;
        // Assemble every tensor projection from rounded coefficient intervals.
        let inertia = |a: DVec3, b: DVec3| {
            let cols = q.to_cols_array_2d();
            let v = std::array::from_fn::<I, 3, _>(|r| {
                (0..3)
                    .map(|c| I::point(cols[c][r]) * I::point(b[c]))
                    .fold(I::point(0.), |a, b| a + b)
            });
            D::interval(
                (0..3)
                    .map(|r| I::point(a[r]) * v[r])
                    .fold(I::point(0.), |a, b| a + b),
            )
        };
        let aa = inertia(l, l);
        let bb = inertia(t, t);
        let e = inertia(l, t);
        let b0 = inertia(l, z);
        let bt = inertia(t, z);
        let c = inertia(z, z);
        let a = aa * cs * cs + D::constant(2.) * e * sn * cs + bb * sn * sn;
        let tt = bb * cs * cs - D::constant(2.) * e * sn * cs + aa * sn * sn;
        let at = e * (cs * cs - sn * sn) + (bb - aa) * sn * cs;
        let b = b0 * cs + bt * sn;
        let bp = bt * cs - b0 * sn;
        let m = D::constant(self.body.mass);
        let depth = D::constant(self.depth);
        let g0 = D::interval(dot(self.body.gravity, self.n));
        let g1 = D::interval(dot(self.body.gravity, self.out));
        let gn = g0 * ct + g1 * st;
        let gt = g1 * ct - g0 * st;
        let metric = a + m * u * u - (b * b).over(c)?;
        let two = D::constant(2.);
        let bend = (-m * u * (gn + depth * k * k + two * du * k)
            - two * at * s * k
            - bp * s * s
            - (b * at * k * k).over(c)?)
        .over(metric)?;
        let axial = (at * k * k - b * bend).over(c)?;
        let shift = gt + u * k * k - depth * bend;
        let force = m * (-two * du * k - depth * k * k - u * bend - gn);
        let moment =
            at * bend + bp * axial + tt * k * s + k * (b * k + c * s) - s * (a * k + b * s);
        let offset = moment.x.over(force.x)?;
        let y = I::point(self.y0)
            + I::point(self.vy) * span
            + dot(self.body.gravity, self.axis) * curvature;
        let low = self.body.corners[0];
        let local_pressure = [self.body.corners[2] - low, self.body.corners[1] - low].map(|d| {
            let al = dot(d, l);
            let at = dot(d, t);
            let az = dot(d, z);
            let along = al * cs.x + at * sn.x;
            let across = at * cs.x - al * sn.x;
            offset * along - u.x * across - I::point(self.depth) * az - dot(d, low)
        });
        Some(Limits {
            acceleration: [shift, bend, axial],
            force: force.x,
            cp: y + offset,
            theta: theta.x,
            du: du.x,
            yaw_rate: s.x,
            tilt_rate: k.x,
            u: u.x,
            local_pressure,
        })
    }
    fn checked(self, v: &Limits, span: I, eps: f64, increasing: bool) -> Option<DeckEdgeEnvelope> {
        if !(v.theta.lo >= 0. || self.theta0 >= 0. && increasing)
            || v.theta.hi >= std::f64::consts::FRAC_PI_2 - eps
            || v.force.lo <= 0.
            || !v.force.finite()
            || !v.cp.finite()
            || v.cp.lo <= -self.half + eps
            || v.cp.hi >= self.half - eps
        {
            return None;
        }
        let low = self.body.corners[0];
        for (p, d) in v
            .local_pressure
            .into_iter()
            .zip([self.body.corners[2] - low, self.body.corners[1] - low])
        {
            let length = d.length();
            let length2 = dot(d, d);
            if !p.finite() || p.lo <= eps * length || p.hi >= length2.lo - eps * length {
                return None;
            }
        }
        let vy = I::point(self.vy) + dot(self.body.gravity, self.axis) * span;
        let angular_speed_bound = v.tilt_rate.maxabs() + v.yaw_rate.maxabs();
        let center_speed_bound =
            vy.maxabs() + v.du.maxabs() + (self.depth + v.u.maxabs()) * v.tilt_rate.maxabs();
        if !center_speed_bound.is_finite() || !angular_speed_bound.is_finite() {
            return None;
        }
        Some(DeckEdgeEnvelope {
            normal_force: [v.force.lo, v.force.hi],
            center_along_edge: [v.cp.lo, v.cp.hi],
            center_speed_bound,
            angular_speed_bound,
            domains: 1,
        })
    }
    fn subdivide(self, lo: f64, hi: f64, tube: Tube, depth: u8) -> Option<DeckEdgeEnvelope> {
        let Tube {
            acc,
            eps,
            increasing,
            taylor,
        } = tube;
        let span = I::range(lo, hi);
        if let Some(limits) = self.limits(span, acc, Some(taylor)) {
            if let Some(admitted) = self.checked(&limits, span, eps, increasing) {
                return Some(admitted);
            }
            if limits.force.hi <= 0. || limits.cp.lo >= self.half || limits.cp.hi <= -self.half {
                return None;
            }
        }
        let mid = 0.5 * lo + 0.5 * hi;
        if depth == 0 || mid <= lo || mid >= hi {
            return None;
        }
        let a = self.subdivide(lo, mid, tube, depth - 1)?;
        let b = self.subdivide(mid, hi, tube, depth - 1)?;
        Some(DeckEdgeEnvelope {
            normal_force: [
                a.normal_force[0].min(b.normal_force[0]),
                a.normal_force[1].max(b.normal_force[1]),
            ],
            center_along_edge: [
                a.center_along_edge[0].min(b.center_along_edge[0]),
                a.center_along_edge[1].max(b.center_along_edge[1]),
            ],
            center_speed_bound: a.center_speed_bound.max(b.center_speed_bound),
            angular_speed_bound: a.angular_speed_bound.max(b.angular_speed_bound),
            domains: a.domains + b.domains,
        })
    }
    /// Enclose the continuous unilateral pressure on this finite pad edge.
    /// Doubt about the contact side, load or either finite edge falls back to
    /// ordinary native stepping. The returned velocity bounds cover the COM
    /// in the pad frame and angular speed of the whole rigid assembly.
    pub fn enclose(self, seconds: f64) -> Result<Option<DeckEdgeEnvelope>, PhysicsError> {
        if !seconds.is_finite() || seconds < 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let x = self.initial;
        let start = self.stage(x)?.rates;
        let rates = [start.du, start.tilt_rate, start.yaw_rate];
        let mut acc = rates.map(|v| I::around(v, 0.125 * (1. + v.abs())));
        let eps =
            1024. * f64::EPSILON * (1. + self.start.center.length() + self.body.pad_half.length());
        for _ in 0..12 {
            let Some(v) = self.limits(I::range(0., seconds), acc, None) else {
                return Ok(None);
            };
            if v.acceleration
                .iter()
                .any(|r| !r.x.finite() || !r.dx.finite())
            {
                return Ok(None);
            }
            if !(0..3)
                .all(|k| v.acceleration[k].x.lo >= acc[k].lo && v.acceleration[k].x.hi <= acc[k].hi)
            {
                for (k, c) in acc.iter_mut().enumerate() {
                    let d = v.acceleration[k].x;
                    let pad = 0.125 * (1. + d.maxabs());
                    *c = I::range(c.lo.min(d.lo - pad), c.hi.max(d.hi + pad));
                }
                continue;
            }
            let Some(init) = self.limits(I::point(0.), acc, None) else {
                return Ok(None);
            };
            let bend = v.acceleration[1];
            let increasing = x.tilt_rate >= 0.
                && (bend.x.lo >= 0.
                    || (init.acceleration[1].x.lo >= 0. || self.face_exit) && bend.dx.lo > 0.);
            let span = I::range(0., seconds);
            if let Some(full) = self.checked(&v, span, eps, increasing) {
                return Ok(Some(full));
            }
            // The global inclusion above already encloses the true
            // accelerations over the entire trajectory. Intersect with the
            // resulting right-hand side before restricting the time domain.
            // Every strip is measured from the same certified incoming state.
            let tighter = v.acceleration.map(|d| d.x);
            let taylor = TaylorMotion {
                initial_acc: init.acceleration.map(|d| d.x),
                jerk: v.acceleration.map(|d| d.dx),
            };
            return Ok(self.subdivide(
                0.,
                seconds,
                Tube {
                    acc: tighter,
                    eps,
                    increasing,
                    taylor,
                },
                8,
            ));
        }
        Ok(None)
    }
}
