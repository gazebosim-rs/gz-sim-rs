// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarRockingImpact {
    pub seconds: f64,
    pub numerical_time_bounds: [f64; 2],
    pub normal_impulse: f64,
    /// Signed distance along the transverse direction in units of half-width.
    pub pressure_center: f64,
    pub incoming_angular_rate: f64,
    pub outgoing_angular_rate: f64,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarRockingAdvance {
    pub signed_angle: f64,
    pub angular_rate: f64,
    pub center_height: f64,
    pub normal_velocity: f64,
    pub integrated_normal_impulse: f64,
    pub integrated_impact_impulse: f64,
    pub first_impact: Option<PlanarRockingImpact>,
    pub explicit_impacts: u32,
    /// A countably infinite sequence was included once its continuing time and
    /// impulse were bounded by the gravity and angular momentum equations.
    pub includes_infinite_tail: bool,
    pub settled: bool,
    /// Time within this interval at which the flat, stationary normal motion begins.
    pub settle_time_bounds: Option<[f64; 2]>,
    pub angular_speed_bound: f64,
}
impl PlanarSoleRocking {
    /// Advance a currently bearing, symmetric planar sole. Positive signed
    /// angle raises the positive transverse edge; `side` is +1 or −1 and
    /// selects the supporting branch at a level crossing. Tangential world motion
    /// and exclusion of all other contacts remain the caller's responsibility.
    pub fn advance(
        self,
        signed_angle: f64,
        angular_rate: f64,
        side: i8,
        seconds: f64,
    ) -> Result<Option<PlanarRockingAdvance>, PhysicsError> {
        if !signed_angle.is_finite()
            || !angular_rate.is_finite()
            || !matches!(side, -1 | 1)
            || !seconds.is_finite()
            || seconds <= 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let ratio = self.speed_ratio()?;
        let mut side = f64::from(side);
        if side * signed_angle < -64. * f64::EPSILON {
            return Ok(None);
        }
        let angle = (side * signed_angle).max(0.);
        let b = self.half_width;
        let mut result = PlanarRockingAdvance {
            signed_angle: 0.,
            angular_rate: 0.,
            center_height: self.center_height,
            normal_velocity: 0.,
            integrated_normal_impulse: 0.,
            integrated_impact_impulse: 0.,
            first_impact: None,
            explicit_impacts: 0,
            includes_infinite_tail: false,
            settled: false,
            settle_time_bounds: None,
            angular_speed_bound: 0.,
        };
        if angle == 0. && angular_rate == 0. {
            result.integrated_normal_impulse = self.mass * self.gravity * seconds;
            result.settled = true;
            return Ok(Some(result));
        }
        let (_, lever) = self.hr(angle);
        let dh = b * angle.sin() - 2. * self.center_height * (0.5 * angle).sin().powi(2);
        if lever <= 0. || dh < 0. {
            return Ok(None);
        }
        let initial_v = side * angular_rate * lever;
        let mut speed = (((self.roll_inertia + self.mass * lever * lever) * angular_rate.powi(2)
            + 2. * self.mass * self.gravity * dh)
            / self.m0())
        .sqrt();
        result.angular_speed_bound = speed * (self.m0() / self.roll_inertia).sqrt();
        let mut elapsed = 0.;
        let mut uncertainty = 0.;
        let mut active_angle = angle;
        let mut rate = side * angular_rate;
        for _ in 0..256 {
            let Some(c) = self.cycle(speed)? else {
                return Ok(None);
            };
            let rem = seconds - elapsed;
            if rem <= 0. {
                return Ok(None);
            }
            if result.explicit_impacts > 0
                && 2. * speed / (c.acceleration[0] * (1. - ratio)) + uncertainty < rem
            {
                let Some(tail) = self.settling_limit(speed, (seconds * 1e-7).max(2e-14))? else {
                    return Ok(None);
                };
                let bounds = tail.time_bounds();
                if elapsed + bounds[1] + uncertainty < seconds {
                    result.integrated_impact_impulse +=
                        self.mass * b * speed * (1. + ratio) / (1. - ratio);
                    result.settle_time_bounds = Some([
                        (elapsed + bounds[0] - uncertainty).max(0.),
                        elapsed + bounds[1] + uncertainty,
                    ]);
                    result.includes_infinite_tail = true;
                    result.settled = true;
                    break;
                }
            }
            let s = (1. - active_angle / c.turn_angle).clamp(0., 1.).sqrt();
            let outward = rate >= 0.;
            let exact_boundary = active_angle == 0. && !outward;
            let from_turn = if s == 1. {
                c.half_period
            } else {
                c.time_from_turn(s)
            };
            let remaining = if outward {
                c.half_period + from_turn
            } else {
                c.half_period - from_turn
            };
            let age = 2. * c.half_period - remaining;
            // Acceleration is strictly inward throughout the flight. Changing
            // rate from the current value to -speed therefore takes at least
            // this long irrespective of the quadrature.
            let lower = (rate + speed).max(0.) / c.acceleration[1];
            let need_bound = lower <= rem + uncertainty;
            let bounds = if exact_boundary {
                [0., 0.]
            } else if need_bound {
                let target = (0.08 * (remaining - rem).abs()).clamp(2e-14, seconds * 1e-6 + 2e-14);
                let Some((h, eh)) = c.bounded_half_period(target) else {
                    return Ok(None);
                };
                let (p, ep) = if s == 1. {
                    (h, eh)
                } else {
                    let Some(x) = c.bounded_from_turn(s, target) else {
                        return Ok(None);
                    };
                    x
                };
                let center = if outward { h + p } else { h - p };
                let rad = eh + ep + 32. * f64::EPSILON * c.half_period;
                [(center - rad).max(0.), center + rad]
            } else {
                [lower, f64::INFINITY]
            };
            if bounds[0] > rem + uncertainty {
                if age + rem > 2. * c.half_period {
                    return Ok(None);
                }
                let x = c.sample(age + rem)?;
                result.signed_angle = side * x.angle;
                result.angular_rate = side * x.angular_speed;
                result.center_height = x.center_height;
                result.normal_velocity = x.normal_velocity;
                break;
            }
            if bounds[1] + uncertainty >= rem || remaining < 0. || remaining > rem {
                return Ok(None);
            }
            let at = elapsed + remaining;
            let time_bounds = [
                (elapsed + bounds[0] - uncertainty).max(0.),
                elapsed + bounds[1] + uncertainty,
            ];
            let impulse = self.mass * b * speed * (1. + ratio);
            let impact = PlanarRockingImpact {
                seconds: at,
                numerical_time_bounds: time_bounds,
                normal_impulse: impulse,
                pressure_center: side * (self.roll_inertia / (self.mass * b * b)).min(1.),
                incoming_angular_rate: -side * speed,
                outgoing_angular_rate: -side * ratio * speed,
            };
            result.first_impact.get_or_insert(impact);
            result.integrated_impact_impulse += impulse;
            result.explicit_impacts += 1;
            if ratio == 0. {
                result.settled = true;
                result.settle_time_bounds = Some(time_bounds);
                break;
            }
            uncertainty = (at - time_bounds[0]).abs().max((at - time_bounds[1]).abs());
            elapsed = at;
            speed *= ratio;
            active_angle = 0.;
            rate = speed;
            side = -side;
        }
        if !result.settled && result.signed_angle == 0. && result.angular_rate == 0. {
            return Ok(None);
        }
        result.integrated_normal_impulse =
            self.mass * (result.normal_velocity - initial_v + self.gravity * seconds);
        if result.integrated_normal_impulse
            < result.integrated_impact_impulse - 1e-12 * (1. + result.integrated_impact_impulse)
        {
            return Ok(None);
        }
        Ok(Some(result))
    }
}
