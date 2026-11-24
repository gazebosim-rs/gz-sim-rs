// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Sticking contact on a deck corner. The material contact coordinate is
//! fixed, so the locked assembly is a pendulum about a translating pivot.
use crate::{DeckDirection, EdgeMotion, PlanarEdgeAdvance, PlanarEdgeArc};
type State = [f64; 7]; // theta, pitch rate, two normal and two tangent impulses, scalar normal load.
struct Pivot {
    arc: PlanarEdgeArc,
    gravity: [f64; 2],
    inertia: f64,
}
impl Pivot {
    fn loads(&self, theta: f64, omega: f64) -> (f64, f64, f64) {
        let (sn, cs) = theta.sin_cos();
        let [gx, gz] = self.gravity;
        let ge = gx * cs - gz * sn;
        let gn = gx * sn + gz * cs;
        let c = self.arc.contact;
        let s = c.contact_x;
        let h = c.height;
        let alpha = (c.mass * (h * ge + s * gn) + c.external_torque) / self.inertia;
        let tangent = c.mass * (h * alpha + s * omega * omega - ge);
        let normal = c.mass * (s * alpha - h * omega * omega - gn);
        (alpha, normal, tangent)
    }
    fn rate(&self, y: State) -> State {
        let (sn, cs) = y[0].sin_cos();
        let (alpha, n, t) = self.loads(y[0], y[1]);
        [y[1], alpha, n * sn, n * cs, t * cs, -t * sn, n]
    }
    fn integrate(&self, mut y: State, dt: f64, calls: u32) -> State {
        let h = dt / f64::from(calls);
        let add = |a: State, k: State, h: f64| std::array::from_fn(|i| a[i] + h * k[i]);
        for _ in 0..calls {
            let a = self.rate(y);
            let b = self.rate(add(y, a, 0.5 * h));
            let c = self.rate(add(y, b, 0.5 * h));
            let d = self.rate(add(y, c, h));
            y = std::array::from_fn(|i| y[i] + h / 6. * (a[i] + 2. * b[i] + 2. * c[i] + d[i]));
        }
        y
    }
    fn admissible(&self, dt: f64) -> bool {
        let c = self.arc.contact;
        let s = c.contact_x;
        let h = c.height;
        let w = c.pitch_rate;
        let g = self.gravity[0].hypot(self.gravity[1]);
        let radial = h.hypot(s);
        let alpha = (c.mass * radial * g + c.external_torque.abs()) / self.inertia;
        let omega = w.abs() + dt * alpha;
        let jerk = c.mass * radial * g * omega / self.inertia;
        let dn = c.mass * (s.abs() * jerk + 2. * h * omega * alpha + g * omega);
        let dtan = c.mass * (h * jerk + 2. * s.abs() * omega * alpha + g * omega);
        let (_, n, t) = self.loads(self.arc.angle, w);
        let eps = 256. * f64::EPSILON;
        let lower = n - dn * dt;
        let upper = t.abs() + dtan * dt;
        let direction = if self.arc.deck == DeckDirection::Positive {
            -1.
        } else {
            1.
        };
        let theta = direction * self.arc.angle;
        // If the pivot reaches the opposite side of the deck, the other
        // footprint contacts must participate in the same solve.
        theta + omega * dt < std::f64::consts::FRAC_PI_2 - eps
            && (theta > omega * dt + eps || (theta >= 0. && direction * w > alpha * dt + eps))
            && s > self.arc.sole_bounds[0] + eps
            && s < self.arc.sole_bounds[1] - eps
            && lower > eps * (1. + n.abs())
            && c.friction * lower > upper + eps * (1. + upper)
    }
}
pub(super) fn advance(arc: PlanarEdgeArc, dt: f64) -> Option<PlanarEdgeAdvance> {
    let c = arc.contact;
    let (sn, cs) = arc.angle.sin_cos();
    let [ae, an] = c.acceleration;
    let g = [ae * cs + an * sn, -ae * sn + an * cs];
    let s = c.contact_x;
    let h = c.height;
    let p = Pivot {
        arc,
        gravity: g,
        inertia: c.pitch_inertia + c.mass * (s * s + h * h),
    };
    if !p.admissible(dt) {
        return None;
    }
    let start = [arc.angle, c.pitch_rate, 0., 0., 0., 0., 0.];
    let mut coarse = p.integrate(start, dt, 1);
    for n in [2, 4, 8, 16, 32] {
        let fine = p.integrate(start, dt, n);
        if fine
            .iter()
            .zip(coarse)
            .all(|(&a, b)| a.is_finite() && (a - b).abs() <= 2e-12 * (1. + a.abs() + b.abs()))
        {
            let (sn, cs) = fine[0].sin_cos();
            let w = fine[1];
            return Some(PlanarEdgeAdvance {
                motion: EdgeMotion::Sticking,
                next: PlanarEdgeArc {
                    angle: fine[0],
                    contact: crate::PlanarEdgeContact {
                        pitch_rate: w,
                        tangent_center_speed: h * w,
                        acceleration: [g[0] * cs - g[1] * sn, g[0] * sn + g[1] * cs],
                        ..c
                    },
                    ..arc
                },
                relative_center: [-s * cs + h * sn, s * sn + h * cs],
                relative_velocity: [(h * cs + s * sn) * w, (-h * sn + s * cs) * w],
                normal_impulse: [fine[2], fine[3]],
                tangent_impulse: [fine[4], fine[5]],
                compressive_impulse: fine[6],
                lost_energy: 0.,
                subintervals: n,
            });
        }
        coarse = fine;
    }
    None
}
