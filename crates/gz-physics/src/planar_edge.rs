// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Continuous reaction at the fixed corner of a deck underneath a pitching
//! flat sole. The support translates inertially and velocity is relative to it.
use crate::PhysicsError;
/// Coordinates use the moving sole tangent and outward (upward) normal.
/// The center is `height` above the sole and the contact is `contact_x`
/// along the sole from the center's projection. Pitch is positive about +y.
#[derive(Clone, Copy, Debug)]
pub struct PlanarEdgeContact {
    pub mass: f64,
    pub pitch_inertia: f64,
    pub height: f64,
    pub contact_x: f64,
    pub tangent_center_speed: f64,
    pub pitch_rate: f64,
    pub friction: f64,
    /// External translational acceleration in the tangent and normal axes.
    pub acceleration: [f64; 2],
    pub external_torque: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EdgeMotion {
    Released,
    Sliding,
    Sticking,
}
#[derive(Clone, Copy, Debug)]
pub struct PlanarEdgeReaction {
    pub motion: EdgeMotion,
    /// Tangential and compressive normal force in newtons.
    pub force: [f64; 2],
    /// Physical outgoing gap acceleration in m/s².
    pub gap_acceleration: f64,
    pub slip_acceleration: f64,
}
impl PlanarEdgeContact {
    /// Solve the acceleration-level unilateral and isotropic Coulomb laws at
    /// zero normal speed. On this constraint the normal COM speed is x·ω.
    pub fn reaction(self) -> Result<PlanarEdgeReaction, PhysicsError> {
        let Self {
            mass: m,
            pitch_inertia: i,
            height: h,
            contact_x: x,
            tangent_center_speed: u,
            pitch_rate: w,
            friction: mu,
            acceleration: [ae, an],
            external_torque: tau,
        } = self;
        if ![m, i, h, x, u, w, mu, ae, an, tau]
            .into_iter()
            .all(f64::is_finite)
            || m <= 0.
            || i <= 0.
            || h <= 0.
            || mu < 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let a = 1. / m + x * x / i;
        let b = x * h / i;
        let c = 1. / m + h * h / i;
        let bn = an - x * tau / i + 2. * w * u - h * w * w;
        let bt = ae - h * tau / i - x * w * w;
        let slip = u - h * w;
        let tol = 64. * f64::EPSILON * (u.abs() + h * w.abs());
        let reject = || PhysicsError::Unsupported {
            engine: "multifidelity",
            capability: "unique sustained sliding edge reaction",
        };
        if ![a, b, c, bn, bt, slip].into_iter().all(f64::is_finite) {
            return Err(PhysicsError::InvalidValue);
        }
        let response = |motion, t, n| -> Result<_, PhysicsError> {
            let r = PlanarEdgeReaction {
                motion,
                force: [t, n],
                gap_acceleration: bn + a * n + b * t,
                slip_acceleration: bt + b * n + c * t,
            };
            if [t, n, r.gap_acceleration, r.slip_acceleration]
                .into_iter()
                .all(f64::is_finite)
            {
                Ok(r)
            } else {
                Err(PhysicsError::InvalidValue)
            }
        };
        // Positive effective normal mass for every admitted friction direction
        // excludes Painlevé modes where separation and contact can coexist.
        if a <= mu * b.abs() {
            return Err(reject());
        }
        if bn >= 0. {
            return response(EdgeMotion::Released, 0., 0.);
        }
        let sliding = |sign: f64| {
            let n = -bn / (a - sign * mu * b);
            response(EdgeMotion::Sliding, -sign * mu * n, n)
        };
        if slip.abs() > tol {
            return sliding(slip.signum());
        }
        let det = (1. / m + (x * x + h * h) / i) / m;
        let n = (b * bt - c * bn) / det;
        let t = (b * bn - a * bt) / det;
        if n > 0. && t.abs() <= mu * n {
            return response(EdgeMotion::Sticking, t, n);
        }
        // At rest, the sign of the required sticking friction selects the
        // incipient slip. Check that its actual acceleration supports it.
        let sign = -t.signum();
        let r = sliding(sign)?;
        if sign * r.slip_acceleration > 0. {
            Ok(r)
        } else {
            Err(reject())
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn sample(x: f64, u: f64, w: f64, mu: f64) -> PlanarEdgeContact {
        PlanarEdgeContact {
            mass: 2.,
            pitch_inertia: 0.05,
            height: 0.13,
            contact_x: x,
            tangent_center_speed: u,
            pitch_rate: w,
            friction: mu,
            acceleration: [0., 0.],
            external_torque: 0.,
        }
    }
    #[test]
    fn a_moving_deck_edge_can_carry_positive_force_after_a_plastic_impact() {
        let c = sample(0.05, 3.22, -2.5, 0.);
        let r = c.reaction().unwrap();
        assert_eq!(r.motion, EdgeMotion::Sliding);
        assert!((r.force[1] - (0.13 * 6.25 + 2. * 2.5 * 3.22) / 0.55).abs() < 1e-12);
        assert!(r.gap_acceleration.abs() < 1e-12);
        assert_eq!(
            sample(0.05, -3.22, -2.5, 0.).reaction().unwrap().motion,
            EdgeMotion::Released
        );
    }
    #[test]
    fn static_sliding_and_reflected_acceleration_obey_complementarity() {
        for mu in [0., 0.15, 0.6, 1.] {
            for u in [-1.5, 0., 1.5] {
                for drive in [-14., 0., 14.] {
                    let mut c = sample(0.05, u, 0., mu);
                    c.acceleration = [drive, -9.81];
                    let r = c.reaction().unwrap();
                    let rev = PlanarEdgeContact {
                        contact_x: -c.contact_x,
                        tangent_center_speed: -u,
                        acceleration: [-drive, -9.81],
                        ..c
                    }
                    .reaction()
                    .unwrap();
                    assert_eq!(r.motion, rev.motion);
                    assert!(
                        (r.force[1] - rev.force[1]).abs() < 1e-12
                            && (r.force[0] + rev.force[0]).abs() < 1e-12
                    );
                    assert!(
                        r.force[1] >= 0.
                            && r.force[0].abs() <= mu * r.force[1] + 1e-12
                            && r.gap_acceleration.abs() < 1e-12
                    );
                    if r.motion == EdgeMotion::Sticking {
                        assert!(r.slip_acceleration.abs() < 1e-12);
                    } else {
                        assert!(r.force[0] * u <= 0.);
                    }
                }
            }
        }
    }
    #[test]
    fn rejects_singular_frictional_normal_mobility() {
        assert!(sample(0.1, 2., -1., 40.).reaction().is_err());
    }
}
