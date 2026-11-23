// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exact energy integral and finite settling time of a symmetric rectangular
//! sole rolling about alternating frictionless horizontal edges. The COM starts
//! above the center of the rectangle. There is no yaw, restitution or friction.
mod advance;
use crate::PhysicsError;
pub use advance::{PlanarRockingAdvance, PlanarRockingImpact};

#[derive(Clone, Copy, Debug)]
pub struct PlanarSoleRocking {
    pub mass: f64,
    /// Half the distance between the two parallel bearing edges.
    pub half_width: f64,
    /// Perpendicular distance from the level sole to the assembled COM.
    pub center_height: f64,
    /// Principal COM moment about the direction of the bearing edge.
    pub roll_inertia: f64,
    /// Strictly positive acceleration directed into the plane.
    pub gravity: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct RockingCycle {
    model: PlanarSoleRocking,
    /// Outgoing angular speed at level. At the next level crossing it has
    /// the same magnitude and the opposite sign before the plastic impulse.
    pub launch_speed: f64,
    pub turn_angle: f64,
    pub half_period: f64,
    /// g minus the largest possible centripetal acceleration along the branch.
    pub free_compression_lower_bound: f64,
    acceleration: [f64; 2],
}
#[derive(Clone, Copy, Debug)]
pub struct RockingSample {
    /// Nonnegative angular displacement into this cycle's bearing edge.
    pub angle: f64,
    /// Positive on the outward half-cycle.
    pub angular_speed: f64,
    pub center_height: f64,
    pub normal_velocity: f64,
    pub supporting_force: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct RockingLimit {
    /// Time of the last explicitly integrated flat crossing, measured from launch.
    pub resolved_seconds: f64,
    pub resolved_cycles: u32,
    /// Rigorous acceleration bounds on the time occupied by all subsequent
    /// returns. The separate quadrature allowance applies to resolved_seconds.
    pub remaining_seconds: [f64; 2],
    pub quadrature_allowance: f64,
    /// Sum of all instantaneous normal jumps after the resolved prefix.
    pub remaining_impact_impulse: f64,
}
impl RockingLimit {
    pub fn time_bounds(self) -> [f64; 2] {
        [
            (self.resolved_seconds + self.remaining_seconds[0] - self.quadrature_allowance)
                .next_down(),
            (self.resolved_seconds + self.remaining_seconds[1] + self.quadrature_allowance)
                .next_up(),
        ]
    }
}
impl PlanarSoleRocking {
    fn valid(self) -> bool {
        [
            self.mass,
            self.half_width,
            self.center_height,
            self.roll_inertia,
            self.gravity,
        ]
        .into_iter()
        .all(|v| v.is_finite() && v > 0.)
    }
    fn m0(self) -> f64 {
        self.roll_inertia + self.mass * self.half_width.powi(2)
    }
    fn hr(self, a: f64) -> (f64, f64) {
        let (s, c) = a.sin_cos();
        (
            self.center_height * c + self.half_width * s,
            self.half_width * c - self.center_height * s,
        )
    }
    /// Ratio of consecutive outgoing speeds across a plastic, level crossing.
    /// Zero denotes a flat face that arrests normal motion in one impact.
    pub fn speed_ratio(self) -> Result<f64, PhysicsError> {
        if !self.valid() {
            return Err(PhysicsError::InvalidValue);
        }
        let mb2 = self.mass * self.half_width.powi(2);
        let r = (self.roll_inertia - mb2) / (self.roll_inertia + mb2);
        if !r.is_finite() || r >= 1. {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(r.max(0.))
    }
    pub fn cycle(self, speed: f64) -> Result<Option<RockingCycle>, PhysicsError> {
        self.speed_ratio()?;
        if !speed.is_finite() || speed <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let delta = self.m0() * speed.powi(2) / (2. * self.mass * self.gravity);
        let d = self.center_height;
        let b = self.half_width;
        // Rationalized positive root for tan(angle/2); stable arbitrarily
        // close to the level configuration.
        let disc = b * b - delta * (2. * d + delta);
        if !disc.is_finite() || disc <= 256. * f64::EPSILON * b * b {
            return Ok(None);
        }
        let angle = 2. * (delta / (b + disc.sqrt())).atan();
        let min_lever = disc.sqrt();
        let max_h = d + delta;
        let compression = self.gravity - max_h * speed.powi(2) * self.m0() / self.roll_inertia;
        if angle <= 0. || compression <= 0. {
            return Ok(None);
        }
        let mut c = RockingCycle {
            model: self,
            launch_speed: speed,
            turn_angle: angle,
            half_period: 0.,
            free_compression_lower_bound: compression,
            acceleration: [
                self.mass * min_lever * compression / self.m0(),
                self.mass * b * self.gravity / (self.roll_inertia + self.mass * min_lever.powi(2)),
            ],
        };
        c.half_period = c.time_from_turn(1.);
        Ok(Some(c))
    }
    /// Bound the finite sum of all future rocking times. A composite
    /// trapezoidal enclosure bounds the leading energy integrals; geometric
    /// speed decay bounds the infinite suffix using its largest launch speed.
    pub fn settling_limit(
        self,
        speed: f64,
        precision: f64,
    ) -> Result<Option<RockingLimit>, PhysicsError> {
        if !precision.is_finite() || precision <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let ratio = self.speed_ratio()?;
        let mut speed = speed;
        let mut clock = 0.;
        let mut allowance = 0.;
        for k in 0..=4096 {
            let Some(c) = self.cycle(speed)? else {
                return Ok(None);
            };
            let lower = 2. * speed / (c.acceleration[1] * (1. - ratio));
            let upper = 2. * speed / (c.acceleration[0] * (1. - ratio));
            let impulses = self.mass * self.half_width * speed * (1. + ratio) / (1. - ratio);
            if upper - lower <= precision * 0.5 {
                return Ok(Some(RockingLimit {
                    resolved_seconds: clock,
                    resolved_cycles: k,
                    remaining_seconds: [lower, upper],
                    quadrature_allowance: allowance,
                    remaining_impact_impulse: impulses,
                }));
            }
            let budget = precision * 0.10 / ((f64::from(k) + 1.) * (f64::from(k) + 2.));
            let Some((time, error)) = c.bounded_half_period(budget) else {
                return Ok(None);
            };
            clock += 2. * time;
            allowance += 2. * error + 8. * f64::EPSILON * clock;
            if ratio == 0. {
                return Ok(Some(RockingLimit {
                    resolved_seconds: clock,
                    resolved_cycles: k + 1,
                    remaining_seconds: [0.; 2],
                    quadrature_allowance: allowance,
                    remaining_impact_impulse: 0.,
                }));
            }
            speed *= ratio;
        }
        Ok(None)
    }
}
fn sinc(x: f64) -> f64 {
    if x == 0. { 1. } else { x.sin() / x }
}
impl RockingCycle {
    fn integrand(self, s: f64) -> f64 {
        let a = self.turn_angle;
        let th = a * (1. - s * s);
        let (_, hp) = self.model.hr(th);
        let (_, mid) = self.model.hr((a + th) * 0.5);
        let m = self.model.roll_inertia + self.model.mass * hp * hp;
        (2. * a * m / (self.model.mass * self.model.gravity * mid * sinc(0.5 * a * s * s))).sqrt()
    }
    /// Substitution angle = turn_angle (1−s²) removes the square-root
    /// endpoint singularity. Positive nodes and weights are Gauss–Legendre 16.
    fn time_from_turn(self, s: f64) -> f64 {
        const X: [f64; 8] = [
            0.09501250983763744,
            0.2816035507792589,
            0.4580167776572274,
            0.6178762444026438,
            0.755404408355003,
            0.8656312023878318,
            0.9445750230732326,
            0.9894009349916499,
        ];
        const W: [f64; 8] = [
            0.1894506104550685,
            0.1826034150449236,
            0.16915651939500254,
            0.14959598881657673,
            0.12462897125553388,
            0.09515851168249279,
            0.06225352393864789,
            0.027152459411754095,
        ];
        let half = 0.5 * s;
        half * X
            .into_iter()
            .zip(W)
            .map(|(x, w)| w * (self.integrand(half * (1. - x)) + self.integrand(half * (1. + x))))
            .sum::<f64>()
    }
    fn bounded_half_period(self, target: f64) -> Option<(f64, f64)> {
        self.bounded_from_turn(1., target)
    }
    fn bounded_from_turn(self, s: f64, target: f64) -> Option<(f64, f64)> {
        // For F = K sqrt(M / (L sinc)), bound both derivatives of log F
        // over s∈[0,1] by the chain rule. |sinc'|≤1/2, |sinc''|≤1/3
        // follow from sinc(x)=∫₀¹ cos(u x)du.
        let m = self.model;
        let a = self.turn_angle;
        let r = m.half_width.hypot(m.center_height);
        let lever = m.hr(a).1;
        let inertia = m.roll_inertia + m.mass * lever.powi(2);
        let sinc_min = sinc(0.5 * a);
        let metric1 = 4. * m.mass * r * r * a / inertia;
        let metric2 = (8. * m.mass * r * r * a * a + 4. * m.mass * r * r * a) / inertia;
        let lever1 = r * a / lever;
        let lever2 = r * (a + a * a) / lever;
        let sinc1 = 0.5 * a / sinc_min;
        let sinc2 = (a * a / 3. + a * 0.5) / sinc_min;
        let first = 0.5 * (metric1 + lever1 + sinc1);
        let second =
            0.5 * (metric2 + metric1 * metric1 + lever2 + lever1 * lever1 + sinc2 + sinc1 * sinc1);
        let max = (2. * a * m.m0() / (m.mass * m.gravity * lever * sinc_min)).sqrt();
        let curvature = max * (first * first + second) * (1. + 128. * f64::EPSILON);
        let count = (curvature * s.powi(3) / (12. * target))
            .sqrt()
            .ceil()
            .max(1.);
        if !count.is_finite() || count > 1_000_000. {
            return None;
        }
        let n = count as u32;
        let mut sum = 0.5 * (self.integrand(0.) + self.integrand(s));
        let mut correction = 0.;
        for i in 1..n {
            let y = self.integrand(s * f64::from(i) / count) - correction;
            let t = sum + y;
            correction = (t - sum) - y;
            sum = t;
        }
        let value = s * sum / count;
        let error = curvature * s.powi(3) / (12. * count * count) + 256. * f64::EPSILON * max * s;
        Some((value, error))
    }
    pub fn sample(self, seconds: f64) -> Result<RockingSample, PhysicsError> {
        if !seconds.is_finite() || seconds < 0. || seconds > 2. * self.half_period {
            return Err(PhysicsError::InvalidValue);
        }
        let t = (seconds - self.half_period).abs();
        let s = if t == 0. {
            0.
        } else if t == self.half_period {
            1.
        } else {
            // Positive integrand makes the inverse unique. Newton's step is
            // bracketed, including near the extremum where angular speed vanishes.
            let mut lo = 0.;
            let mut hi = 1.;
            let mut s = t / self.half_period;
            for _ in 0..55 {
                let gap = self.time_from_turn(s) - t;
                if gap.abs() <= 4. * f64::EPSILON * self.half_period {
                    break;
                }
                if gap > 0. {
                    hi = s;
                } else {
                    lo = s;
                }
                let trial = s - gap / self.integrand(s);
                s = if trial > lo && trial < hi {
                    trial
                } else {
                    0.5 * (lo + hi)
                };
            }
            s
        };
        let a = self.turn_angle;
        let angle = a * (1. - s * s);
        let (h, l) = self.model.hr(angle);
        let dh = 2. * (a * s * s * 0.5).sin() * self.model.hr(0.5 * (angle + a)).1;
        let m = self.model.mass;
        let g = self.model.gravity;
        let i = self.model.roll_inertia;
        let metric = i + m * l * l;
        let speed =
            (2. * m * g * dh / metric).sqrt() * if seconds <= self.half_period { 1. } else { -1. };
        let force = m * i * (g - h * speed * speed) / metric;
        Ok(RockingSample {
            angle,
            angular_speed: speed,
            center_height: h,
            normal_velocity: l * speed,
            supporting_force: force,
        })
    }
}
#[cfg(test)]
mod tests;
