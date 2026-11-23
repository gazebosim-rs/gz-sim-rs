// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A smooth sliding mode of a rigid sole on one translating deck corner.
//! Integrate angular momentum about the corner and mechanical energy in
//! its inertial frame. Uniform gravity, constant pitch torque and friction
//! account for their changes without leaving the geometric constraint.
use crate::{PhysicsError, PlanarEdgeContact};
use std::time::Duration;
/// The finite top face of the deck extends from the loaded corner in this
/// direction along the initial positive or negative horizontal tangent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeckDirection {
    Positive,
    Negative,
}
#[derive(Clone, Copy, Debug)]
pub struct PlanarEdgeArc {
    pub contact: PlanarEdgeContact,
    pub angle: f64,
    /// Limits of the underside in body tangent coordinates about its COM.
    pub sole_bounds: [f64; 2],
    pub deck: DeckDirection,
}
#[derive(Clone, Copy, Debug)]
pub struct PlanarEdgeAdvance {
    pub motion: crate::EdgeMotion,
    pub next: PlanarEdgeArc,
    /// Horizontal and vertical COM coordinates from the deck corner.
    pub relative_center: [f64; 2],
    pub relative_velocity: [f64; 2],
    pub normal_impulse: [f64; 2],
    pub tangent_impulse: [f64; 2],
    pub compressive_impulse: f64,
    pub lost_energy: f64,
    /// Accepted subintervals for fourth-order integration.
    pub subintervals: u32,
}
type Y = [f64; 10]; // s, θ, mechanical energy, Jn x/z, Jt x/z, scalar Jn, L, friction loss.
#[derive(Clone, Copy)]
struct Mode {
    arc: PlanarEdgeArc,
    gravity: [f64; 2],
    initial_center: [f64; 2],
    travel_sign: f64,
}
impl Mode {
    fn center(self, y: Y) -> [f64; 2] {
        let (sn, cs) = y[1].sin_cos();
        let h = self.arc.contact.height;
        [-y[0] * cs + h * sn, y[0] * sn + h * cs]
    }
    fn motion(self, y: Y) -> Option<(f64, f64, f64)> {
        let c = self.arc.contact;
        let s = y[0];
        let [x, z] = self.center(y);
        let e = y[2]
            + c.mass
                * (self.gravity[0] * (x - self.initial_center[0])
                    + self.gravity[1] * (z - self.initial_center[1]));
        let angular_momentum = y[8];
        let k = c.pitch_inertia + c.mass * (c.height * c.height + s * s);
        let effective = c.mass * (c.pitch_inertia + c.mass * s * s) / k;
        let speed_sq = (2. * e - angular_momentum * angular_momentum / k) / effective;
        if !speed_sq.is_finite() || speed_sq <= 0. {
            return None;
        }
        let ds = self.travel_sign * speed_sq.sqrt();
        let w = (angular_momentum + c.mass * c.height * ds) / k;
        let u = c.height * w - ds;
        Some((ds, w, u))
    }
    fn rate(self, y: Y) -> Option<Y> {
        let (ds, w, u) = self.motion(y)?;
        let c = self.arc.contact;
        let s = y[0];
        let denom =
            1. / c.mass + (s * s + self.travel_sign * c.friction * s * c.height) / c.pitch_inertia;
        let (sn, cs) = y[1].sin_cos();
        let [gx, gz] = self.gravity;
        let an = gx * sn + gz * cs;
        let n =
            (c.height * w * w - 2. * w * u - an + s * c.external_torque / c.pitch_inertia) / denom;
        if denom <= 0. || !n.is_finite() || n <= 0. {
            return None;
        }
        let t = self.travel_sign * c.friction * n;
        let [x, z] = self.center(y);
        let loss = c.friction * n * ds.abs();
        Some([
            ds,
            w,
            c.external_torque * w - loss,
            n * sn,
            n * cs,
            t * cs,
            -t * sn,
            n,
            c.mass * (z * gx - x * gz) + c.external_torque,
            loss,
        ])
    }
    fn integrate(self, mut y: Y, dt: f64, calls: u32) -> Option<Y> {
        let h = dt / f64::from(calls);
        let add = |a: Y, k: Y, h: f64| std::array::from_fn(|i| a[i] + h * k[i]);
        for _ in 0..calls {
            let a = self.rate(y)?;
            let b = self.rate(add(y, a, 0.5 * h))?;
            let c = self.rate(add(y, b, 0.5 * h))?;
            let d = self.rate(add(y, c, h))?;
            y = std::array::from_fn(|i| y[i] + (h / 6.) * (a[i] + 2. * b[i] + 2. * c[i] + d[i]));
        }
        y.into_iter().all(f64::is_finite).then_some(y)
    }
}
impl PlanarEdgeArc {
    /// Bound absolute translational and angular speed through one smooth
    /// interval. Cauchy–Schwarz bounds the rate of sqrt(2K) by the norm
    /// of external loads in the inverse mass metric. Friction dissipates.
    fn smooth(self, dt: f64, energy: f64) -> bool {
        let c = self.contact;
        let (m, i, h, x, w, u, mu) = (
            c.mass,
            c.pitch_inertia,
            c.height,
            c.contact_x,
            c.pitch_rate,
            c.tangent_center_speed,
            c.friction,
        );
        let [ae, an] = c.acceleration;
        let g = ae.hypot(an);
        let tau = c.external_torque;
        let root_energy = (2. * energy).sqrt() + (m.sqrt() * g + tau.abs() / i.sqrt()) * dt;
        let v = root_energy / m.sqrt();
        let omega = root_energy / i.sqrt();
        let travel = (v + h * omega) * dt;
        let x_max = x.abs() + travel;
        let x_min = (x.abs() - travel).max(0.);
        let normal_mass = 1. / m + (x_min * x_min - mu * x_max * h) / i;
        if normal_mass <= 0. {
            return false;
        }
        let max_force =
            (h * omega * omega + 2. * omega * v + g + x_max * tau.abs() / i) / normal_mass;
        let dw = (max_force * (x_max + mu * h) + tau.abs()) / i;
        let du = g + mu * max_force / m + x_max * omega * omega;
        let turn = if self.deck == DeckDirection::Positive {
            -1.
        } else {
            1.
        };
        let margin = 256. * f64::EPSILON;
        let safe_angle = self.angle.abs() + omega * dt < std::f64::consts::FRAC_PI_2 - margin
            && (turn * self.angle > omega * dt + margin
                || (turn * self.angle >= 0. && turn * w > dw * dt + margin));
        safe_angle
            && x - travel > self.sole_bounds[0] + margin
            && x + travel < self.sole_bounds[1] - margin
            && (u - h * w).abs() > (du + h * dw) * dt + margin * (1. + v + h * omega)
            && h * w * w - 2. * w * u - an + x * tau / i
                > ((2. * h * omega + 2. * v) * dw
                    + 2. * omega * du
                    + g * omega
                    + (v + h * omega) * tau.abs() / i)
                    * dt
                    + margin * (1. + omega * v)
    }
    /// Advance a smooth unilateral sliding interval under constant world loads. `None`
    /// means the interval may include a topology or contact-mode event.
    pub fn advance(self, dt: Duration) -> Result<Option<PlanarEdgeAdvance>, PhysicsError> {
        let c = self.contact;
        let t = dt.as_secs_f64();
        let [lo, hi] = self.sole_bounds;
        if ![self.angle, lo, hi].into_iter().all(f64::is_finite) || lo >= hi || dt.is_zero() {
            return Err(PhysicsError::InvalidValue);
        }
        let reaction = match c.reaction() {
            Ok(x) => x,
            Err(PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(e) => return Err(e),
        };
        if reaction.motion == crate::EdgeMotion::Sticking {
            return Ok(crate::planar_edge_pivot::advance(self, t));
        }
        if reaction.motion != crate::EdgeMotion::Sliding {
            return Ok(None);
        }
        let (m, i, h, s, w, u) = (
            c.mass,
            c.pitch_inertia,
            c.height,
            c.contact_x,
            c.pitch_rate,
            c.tangent_center_speed,
        );
        let energy = 0.5 * (m * (u * u + s * s * w * w) + i * w * w);
        if !self.smooth(t, energy) {
            return Ok(None);
        }
        let (sn, cs) = self.angle.sin_cos();
        let [ae, an] = c.acceleration;
        let mode = Mode {
            arc: self,
            travel_sign: (h * w - u).signum(),
            gravity: [ae * cs + an * sn, -ae * sn + an * cs],
            initial_center: [-s * cs + h * sn, s * sn + h * cs],
        };
        let l = (i + m * s * s) * w + m * h * u;
        let y = [s, self.angle, energy, 0., 0., 0., 0., 0., l, 0.];
        let Some((ds, wi, ui)) = mode.motion(y) else {
            return Ok(None);
        };
        // Near a turning point E-L²/(2K) can lose the incoming relative
        // speed in floating point, even when the geometric proof succeeds.
        if (wi - w).abs() + (ui - u).abs() + (ds - (h * w - u)).abs()
            > 1e-11 * (1. + w.abs() + u.abs())
        {
            return Ok(None);
        }
        let Some(mut coarse) = mode.integrate(y, t, 1) else {
            return Ok(None);
        };
        for n in [2, 4, 8, 16, 32] {
            let Some(fine) = mode.integrate(y, t, n) else {
                return Ok(None);
            };
            if fine
                .into_iter()
                .zip(coarse)
                .all(|(a, b)| (a - b).abs() <= 2e-12 * (1. + a.abs() + b.abs()))
            {
                let Some((_, wn, un)) = mode.motion(fine) else {
                    return Ok(None);
                };
                let (sn, cs) = fine[1].sin_cos();
                let x = fine[0];
                let next = Self {
                    angle: fine[1],
                    contact: PlanarEdgeContact {
                        contact_x: x,
                        pitch_rate: wn,
                        tangent_center_speed: un,
                        acceleration: [
                            mode.gravity[0] * cs - mode.gravity[1] * sn,
                            mode.gravity[0] * sn + mode.gravity[1] * cs,
                        ],
                        ..c
                    },
                    ..self
                };
                return Ok(Some(PlanarEdgeAdvance {
                    motion: crate::EdgeMotion::Sliding,
                    next,
                    relative_center: [-x * cs + h * sn, x * sn + h * cs],
                    relative_velocity: [un * cs + x * wn * sn, -un * sn + x * wn * cs],
                    normal_impulse: [fine[3], fine[4]],
                    tangent_impulse: [fine[5], fine[6]],
                    compressive_impulse: fine[7],
                    lost_energy: fine[9],
                    subintervals: n,
                }));
            }
            coarse = fine;
        }
        Ok(None)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn arc() -> PlanarEdgeArc {
        PlanarEdgeArc {
            contact: PlanarEdgeContact {
                mass: 2.,
                pitch_inertia: 0.05,
                height: 0.13,
                contact_x: 0.05,
                tangent_center_speed: 3.22,
                pitch_rate: -2.5,
                friction: 0.,
                acceleration: [0.; 2],
                external_torque: 0.,
            },
            angle: 0.,
            sole_bounds: [-0.18, 0.18],
            deck: DeckDirection::Positive,
        }
    }
    #[test]
    fn advancing_a_sliding_edge_conserves_momentum_about_the_deck_corner_and_accounts_for_friction_work()
     {
        for mu in [0., 0.2, 0.6, 1.] {
            let mut a = arc();
            a.contact.friction = mu;
            let mut angular = None;
            let mut energy = None;
            let mut work = 0.;
            for _ in 0..20 {
                let s = a
                    .advance(Duration::from_micros(500))
                    .unwrap()
                    .expect("short smooth slide");
                a = s.next;
                work += s.lost_energy;
                let [x, z] = s.relative_center;
                let [vx, vz] = s.relative_velocity;
                let w = a.contact.pitch_rate;
                let l = 2. * (z * vx - x * vz) + 0.05 * w;
                let e = vx * vx + vz * vz + 0.025 * w * w + work;
                assert!(
                    (l - *angular.get_or_insert(l)).abs() < 1e-12
                        && (e - *energy.get_or_insert(e)).abs() < 1e-12
                );
                assert!(s.lost_energy >= 0. && s.compressive_impulse > 0.);
            }
        }
    }
    #[test]
    fn crossing_the_sole_end_releasing_or_reversing_the_loaded_side_declines() {
        let a = arc();
        let dt = Duration::from_micros(500);
        assert!(
            PlanarEdgeArc {
                sole_bounds: [0.049, 0.051],
                ..a
            }
            .advance(dt)
            .unwrap()
            .is_none()
        );
        assert!(
            PlanarEdgeArc {
                deck: DeckDirection::Negative,
                ..a
            }
            .advance(dt)
            .unwrap()
            .is_none()
        );
        assert!(
            PlanarEdgeArc {
                contact: PlanarEdgeContact {
                    tangent_center_speed: -3.22,
                    ..a.contact
                },
                ..a
            }
            .advance(dt)
            .unwrap()
            .is_none()
        );
    }
}
