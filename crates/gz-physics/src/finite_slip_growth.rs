// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Separate the force needed to stop the material port from the whole
//! Coulomb cone. The resulting world direction fixes a lower speed growth
//! until the finite geometry or the compressive normal bound expires.
use super::*;
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlipGrowth {
    pub direction: DVec3,
    /// Dot product of the incoming material velocity with the witness direction.
    pub initial_component: f64,
    /// Lower bound on its time derivative (m/s²).
    pub acceleration: f64,
}
pub(super) struct EdgeMotionBounds {
    pub h: f64,
    pub radius: f64,
    pub spin: f64,
    pub alpha: f64,
    pub normal_rate: f64,
    pub force: f64,
    pub arm_rate: f64,
    pub port_acceleration: f64,
    pub sine: f64,
}
impl FiniteFeatureArc {
    pub(super) fn edge_slip_growth(
        self,
        s: FiniteFeatureSample,
        b: EdgeMotionBounds,
    ) -> Option<SlipGrowth> {
        let y = self.start();
        let m = self.material;
        let n = s.site.normal;
        let r = s.site.point - y.c;
        let free = self.flight.acceleration + y.w.cross(y.w.cross(r));
        // A resolved sticking endpoint impulse imposes V+ = 0. Re-forming
        // v + ω × r from the rounded component velocities can give either sign
        // to a 1e-17 residual and incorrectly veto the outgoing cone witness.
        // For any other start retain the entire measured material velocity.
        let port = if self.sticking_impact_launch {
            DVec3::ZERO
        } else {
            y.v + y.w.cross(r)
        };
        // Bootstrap local bounds from the already established global ones. In
        // particular, near a sticking impact the *material port* is at rest,
        // even when the center and the lever arm have considerable speed.
        let up = |x: f64| x * (1. + 2e-12) + 1e-14;
        let down = |x: f64| x - 2e-12 * (1. + x.abs());
        let migration = up((port.length() + b.h * b.port_acceleration) / b.sine);
        let dr = b.arm_rate.min(up(b.spin * b.radius + migration));
        let dn = b.normal_rate.min(up(s.normal_rate.length()
            + b.h * ((b.alpha + b.spin * b.spin) / b.sine + 3. * b.normal_rate.powi(2))));
        // W_nn = 1/m + |r × n|²/I and |W_nt| = |r·n| |r × n|/I.
        // Each factor changes by at most h (|r'| + |r| |n'|).
        let delta = up(b.h * (dr + b.radius * dn));
        let tangent = r.cross(n).length();
        let floor = down(
            1. / m.mass
                + ((tangent - delta).max(0.).powi(2)
                    - m.friction * (r.dot(n).abs() + delta) * (tangent + delta))
                    / m.inertia,
        );
        let force = if floor > 0. {
            b.force.min(up((self.flight.acceleration.length()
                + b.spin.powi(2) * b.radius
                + 2. * dn * (port.length() + b.h * b.port_acceleration))
                * m.friction.hypot(1.)
                / floor))
        } else {
            b.force
        };
        let alpha = b.alpha.min(up(b.radius * force / m.inertia));
        let dp = b.port_acceleration.min(up(self.flight.acceleration.length()
            + force / m.mass
            + alpha * b.radius
            + b.spin * dr));
        let b = EdgeMotionBounds {
            alpha,
            force,
            arm_rate: dr,
            normal_rate: dn,
            port_acceleration: dp,
            ..b
        };
        // For spherical inertia W = (1/m + r²/I) Id − rrᵀ/I.
        let inv = |x: DVec3| {
            (x + (m.mass / m.inertia) * r.dot(x) * r)
                / (1. / m.mass + r.length_squared() / m.inertia)
        };
        let required = inv(-free);
        let outward = required.reject_from_normalized(n).try_normalize()?;
        // e·F ≥ 0 for every initial cone force, including every slip direction.
        let witness = |tangent: DVec3| -> Option<SlipGrowth> {
            let e = m.friction * n - tangent;
            let inverse = inv(e);
            let length = inverse.length();
            let direction = inverse.try_normalize()?;
            let initial = direction.dot(port);
            if initial < 0. {
                return None;
            }
            // At fixed world d=W0^-1 e/|W0^-1 e|, d·V' = d·(b + WF + ω×u s').
            // Rotation of the normal can change the cone separation by at most
            // |e| |F| |n-n0|. Also ||W'|| <= 4 R |r'|/I and
            // |s'| <= (|V0| + h sup|V'|)/sin, including the launch from rest.
            let free_rate = 2. * b.alpha * b.spin * b.radius + b.spin * b.spin * b.arm_rate;
            let variation = free_rate
                + b.force
                    * (4. * b.radius * b.arm_rate / m.inertia
                        + e.length() * b.normal_rate / length)
                + b.spin * b.port_acceleration / b.sine;
            let start = direction.dot(free);
            let drift = b.h * variation + b.spin * port.length() / b.sine;
            let lower = start - drift - 4e-12 * (1. + start.abs() + drift.abs());
            (lower.is_finite() && lower > 0.).then_some(SlipGrowth {
                direction,
                initial_component: initial,
                acceleration: lower,
            })
        };
        if let Some(w) = witness(outward) {
            return Some(w);
        }
        // A different separating plane may see a forward component even if
        // the incoming material velocity points behind the central witness.
        // For unit tangent t, the condition is t·(W⁻¹ V) <= µ n·(W⁻¹ V).
        // Choose the closest feasible t to the required tangent force. Also
        // test a point just inside the angular boundary and its deepest point
        // so f64 normalization never forces us to round the incoming speed.
        let dual_velocity = inv(port);
        let tangent_velocity = dual_velocity.reject_from_normalized(n);
        let speed = tangent_velocity.length();
        let axis = tangent_velocity.try_normalize()?;
        let limit = m.friction * n.dot(dual_velocity) / speed;
        if !limit.is_finite() || limit <= -1. {
            return (limit == -1.).then(|| witness(-axis)).flatten();
        }
        let side = n.cross(axis);
        let side = if side.dot(outward) < 0. { -side } else { side };
        let boundary = limit.min(1.);
        for cos in [boundary, boundary - 1e-8 * (1. + boundary), -1.] {
            let sin = ((1. - cos) * (1. + cos)).max(0.).sqrt();
            if let Some(w) = witness((cos * axis + sin * side).try_normalize()?) {
                return Some(w);
            }
        }
        None
    }
}
