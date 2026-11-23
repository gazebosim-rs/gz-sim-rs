// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Curved native sphere CCD against one complete convex material primitive.
//! The derivative of half the squared distance is (x - projection(x)) · x'.
//! The residual of convex projection is 1-Lipschitz, including across an
//! edge or vertex, giving |D''| <= |x'|² + distance * |x''|.
use super::*;

#[derive(Clone, Copy)]
struct Probe {
    time: f64,
    distance: f64,
    derivative: f64,
}
struct Query<'a> {
    path: &'a Trajectory,
    shape: &'a Primitive,
    nodes: &'a [crate::Particle],
    radius: f64,
    round: f64,
    speed: f64,
    acceleration: f64,
    budget: &'a mut usize,
}
impl Query<'_> {
    fn at(&mut self, time: f64) -> Result<Probe, XpbdError> {
        *self.budget = self.budget.checked_sub(1).ok_or(XpbdError::ImpactBudget)?;
        let (_, to_shape) = self.shape.witness(self.nodes, self.path.at(time));
        let distance = to_shape.length();
        let derivative = -to_shape.dot(self.path.tangent(time));
        if !distance.is_finite() || !derivative.is_finite() {
            return Err(XpbdError::Numerical);
        }
        Ok(Probe {
            time,
            distance,
            derivative,
        })
    }
    fn bracket(&mut self, mut a: Probe, mut b: Probe) -> Result<f64, XpbdError> {
        if a.distance <= self.radius {
            return Ok(a.time);
        }
        if b.distance >= self.radius {
            return Ok(b.time);
        }
        for _ in 0..64 {
            let t = a.time + (b.time - a.time) * 0.5;
            if t == a.time || t == b.time {
                break;
            }
            let m = self.at(t)?;
            if m.distance > self.radius {
                a = m;
            } else {
                b = m;
            }
        }
        Ok(a.time + (b.time - a.time) * 0.5)
    }
    fn interval(&mut self, a: Probe, b: Probe, depth: u8) -> Result<Option<f64>, XpbdError> {
        let half = (b.time - a.time) * 0.5;
        let t = a.time + half;
        let m = self.at(t)?;
        // The supporting plane at the projected midpoint provides a lower
        // distance bound, including the path curvature over this interval.
        let normal_rate = if m.distance > 0. {
            m.derivative.abs() / m.distance
        } else {
            self.speed
        };
        let lower = m.distance - normal_rate * half - 0.5 * self.acceleration * half * half;
        if lower > self.radius + self.round {
            return Ok(None);
        }
        let dmax = m.distance + self.speed * half;
        let lipschitz = self.speed * self.speed + dmax * self.acceleration;
        let derivative_round = 128. * f64::EPSILON * dmax * self.speed;
        let uncertainty = lipschitz * half + derivative_round;
        if m.derivative - uncertainty > 0. {
            return Ok(None);
        }
        if m.derivative + uncertainty < 0. {
            let gap_a = a.distance - self.radius;
            let gap_b = b.distance - self.radius;
            let resolved_slope =
                b.derivative * b.derivative > 16. * lipschitz * b.distance * self.round;
            if gap_a >= -self.round
                && (gap_b < -self.round
                    || (resolved_slope && (gap_b <= 0. || b.time == 1. && gap_b <= self.round)))
            {
                return self.bracket(a, b).map(Some);
            }
            return Ok(None);
        }
        // An unresolved minimum with no possible penetration beyond the
        // geometric roundoff bound is a graze with zero normal impulse.
        if lower >= self.radius - self.round
            && a.distance >= self.radius - self.round
            && b.distance >= self.radius - self.round
        {
            return Ok(None);
        }
        if depth >= 56 || t == a.time || t == b.time {
            return Err(XpbdError::UnresolvedImpact);
        }
        if let Some(t) = self.interval(a, m, depth + 1)? {
            return Ok(Some(t));
        }
        self.interval(m, b, depth + 1)
    }
}
pub(super) fn first(
    path: &Trajectory,
    shape: &Primitive,
    nodes: &[crate::Particle],
    radius: f64,
    round: f64,
    budget: &mut usize,
) -> Result<Option<f64>, XpbdError> {
    let mut query = Query {
        path,
        shape,
        nodes,
        radius,
        round,
        speed: path.speed_bound(),
        acceleration: path.acceleration_bound(),
        budget,
    };
    let (a, b) = (query.at(0.)?, query.at(1.)?);
    query.interval(a, b, 0)
}
