// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Wrench feasibility and measurable friction bounds on a continuous line.
use crate::{FiniteContactSegment, PhysicsError};
use glam::DVec3;
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LineWrench {
    /// Endpoint equivalents of the zeroth and first pressure moments.
    pub normal: [f64; 2],
    pub transverse: [f64; 2],
    pub longitudinal_total: f64,
    /// Moment about the contact line. An ideal line cannot supply it.
    pub free_couple: f64,
    /// Minimum additional tangential capacity in N·s at both endpoints
    /// needed to represent the observed tangential resultant. Tension is
    /// reported separately in the signed normal moments.
    pub required_cone_slack: f64,
    /// Normal velocities at the measured endpoints after the impulse.
    pub normal_speed: [f64; 2],
    /// Integrated normal impulse times post-impact normal speed (joules).
    pub normal_work: f64,
    /// Negative integrated tangential impulse times post-impact slip (joules).
    pub slip_dissipation: f64,
    /// Sharp lower/upper bounds on Coulomb dissipation over all nonnegative
    /// pressure distributions with those normal moments. A negative normal
    /// moment invalidates the bound.
    pub coulomb_dissipation: Option<[f64; 2]>,
}
impl FiniteContactSegment {
    /// Test a measured net contact impulse and torque about `center`.
    /// The two post velocities belong to the moving points in the deck frame.
    pub fn audit_wrench(
        self,
        center: DVec3,
        impulse: DVec3,
        torque: DVec3,
        after: [DVec3; 2],
        mu: f64,
    ) -> Result<LineWrench, PhysicsError> {
        let [p, q] = self.moving_points;
        let n = self.normal;
        if ![p, q, n, center, impulse, torque, after[0], after[1]]
            .into_iter()
            .all(|v| v.is_finite())
            || !mu.is_finite()
            || mu < 0.
            || (n.length_squared() - 1.).abs() > 1e-10
        {
            return Err(PhysicsError::InvalidValue);
        }
        let delta = q - p;
        if delta.length() <= 128. * f64::EPSILON * (1. + p.length() + q.length()) {
            return Err(PhysicsError::InvalidValue);
        }
        let x = delta.normalize();
        let y = n.cross(x).normalize();
        if n.dot(x).abs() > 1e-10
            || (after[0] - after[1]).dot(x).abs()
                > 1e-10 * (1. + after[0].length() + after[1].length())
        {
            return Err(PhysicsError::InvalidValue);
        }
        let residual = torque - (p - center).cross(impulse);
        let j1 = residual.cross(x) / delta.length();
        let j0 = impulse - j1;
        let normal = [j0.dot(n), j1.dot(n)];
        let transverse = [j0.dot(y), j1.dot(y)];
        let along = impulse.dot(x);
        let cap = normal.map(|v| mu * v.max(0.));
        // An admissible split of the longitudinal total exists when the sum
        // of the available longitudinal disk radii reaches its magnitude.
        let fits = |slack: f64| {
            let r = [cap[0] + slack, cap[1] + slack];
            (0..2).all(|i| r[i] >= transverse[i].abs())
                && (0..2)
                    .map(|i| {
                        (r[i] - transverse[i].abs()).sqrt() * (r[i] + transverse[i].abs()).sqrt()
                    })
                    .sum::<f64>()
                    >= along.abs()
        };
        let slack = if fits(0.) {
            0.
        } else {
            let mut lo = 0.;
            let mut hi = (along.abs() + transverse[0].abs().max(transverse[1].abs()))
                * (1. + 16. * f64::EPSILON);
            for _ in 0..56 {
                let mid = 0.5 * (lo + hi);
                if fits(mid) { hi = mid } else { lo = mid }
            }
            hi
        };
        let normal_speed = after.map(|v| v.dot(n));
        let slip = after.map(|v| v - n * v.dot(n));
        let dissipation = -(j0.dot(slip[0]) + j1.dot(slip[1]));
        let coulomb = normal.iter().all(|v| *v >= 0.).then(|| {
            // The minimum puts pressure at its barycentre. The maximum
            // puts the barycentric masses on the endpoints (triangle inequality).
            [
                mu * (normal[0] * slip[0] + normal[1] * slip[1]).length(),
                mu * (normal[0] * slip[0].length() + normal[1] * slip[1].length()),
            ]
        });
        let work = normal[0] * normal_speed[0] + normal[1] * normal_speed[1];
        if ![slack, dissipation, work, residual.dot(x)]
            .into_iter()
            .all(f64::is_finite)
            || coulomb.is_some_and(|c| c.iter().any(|v| !v.is_finite()))
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(LineWrench {
            normal,
            transverse,
            longitudinal_total: along,
            free_couple: residual.dot(x),
            required_cone_slack: slack,
            normal_speed,
            normal_work: work,
            slip_dissipation: dissipation,
            coulomb_dissipation: coulomb,
        })
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn distributed_slip_attains_jensen_and_endpoint_dissipation_limits() {
        let p = DVec3::new(-0.3, 0., 0.);
        let q = DVec3::new(0.5, 0., 0.);
        let c = DVec3::new(0.2, -0.1, 0.65);
        let n = DVec3::Z;
        let seg = FiniteContactSegment {
            normal: n,
            moving_points: [p, q],
            support_points: [p, q],
        };
        let mu = 0.6;
        let u = DVec3::new(0.6, -1., 0.);
        let v = DVec3::new(0.6, 2., 0.);
        // Every distribution has the same total pressure and centroid.
        for (k, field) in [
            vec![(0.25, 1.)],
            vec![(0., 0.75), (1., 0.25)],
            vec![(0.1, 0.5), (0.4, 0.5)],
        ]
        .into_iter()
        .enumerate()
        {
            let mut j = DVec3::ZERO;
            let mut l = DVec3::ZERO;
            let mut work = 0.;
            for (s, weight) in field {
                let slip = u.lerp(v, s);
                let point = p.lerp(q, s);
                let force = weight * (n - mu * slip.normalize());
                j += force;
                l += (point - c).cross(force);
                work += mu * weight * slip.length();
            }
            let audit = seg.audit_wrench(c, j, l, [u, v], mu).unwrap();
            let [lo, hi] = audit.coulomb_dissipation.unwrap();
            assert!(
                (audit.normal[0] - 0.75).abs() < 1e-12 && (audit.normal[1] - 0.25).abs() < 1e-12
            );
            assert!(
                (audit.slip_dissipation - work).abs() < 1e-12
                    && audit.free_couple.abs() < 1e-12
                    && audit.required_cone_slack < 1e-12
            );
            assert!(work >= lo - 1e-12 && work <= hi + 1e-12);
            if k == 0 {
                assert!((work - lo).abs() < 1e-12)
            } else if k == 1 {
                assert!((work - hi).abs() < 1e-12)
            }
        }
        let bad = seg
            .audit_wrench(
                c,
                4. * DVec3::X + n,
                (p - c).cross(4. * DVec3::X + n) + 0.13 * DVec3::X,
                [u, v],
                mu,
            )
            .unwrap();
        assert!((bad.free_couple - 0.13).abs() < 1e-12 && bad.required_cone_slack > 1.);
    }
}
