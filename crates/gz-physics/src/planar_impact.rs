// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Instantaneous plastic impact of a level, rigid sole on a translating flat support.
//! Symmetry about the longitudinal plane eliminates yaw and roll. The incoming
//! body translates without pitch. All velocities are relative to the support.
use crate::PhysicsError;

/// Coordinates are tangential x and upward z. Pitch is positive about +y.
/// The endpoints span the actual intersection of the sole and the support,
/// relative to the projection of the assembly COM, in metres.
#[derive(Debug, Clone, Copy)]
pub struct PlanarPatchImpact {
    pub mass: f64,
    pub pitch_inertia: f64,
    pub com_height: f64,
    pub supported_x: [f64; 2],
    pub tangent_speed: f64,
    pub closing_speed: f64,
    pub friction: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bearing {
    Face,
    LowerXEdge,
    UpperXEdge,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarImpactResponse {
    /// Signed tangential and compressive normal impulse on the sole, in N·s.
    pub impulse: [f64; 2],
    /// Resultant pressure center relative to the COM projection, in metres.
    pub center_of_pressure: f64,
    pub bearing: Bearing,
    pub sticking: bool,
    pub com_velocity: [f64; 2],
    pub pitch_rate: f64,
    pub dissipated_energy: f64,
}

impl PlanarPatchImpact {
    /// Solve normal complementarity and a single isotropic Coulomb law at
    /// the impact instant. Signed slip at every point of the level sole is
    /// identical, so the tangential impulses share a single friction bound.
    pub fn solve(self) -> Result<PlanarImpactResponse, PhysicsError> {
        let Self {
            mass: m,
            pitch_inertia: i,
            com_height: h,
            supported_x: [a, b],
            tangent_speed: u,
            closing_speed: c,
            friction: mu,
        } = self;
        if ![m, i, h, a, b, u, c, mu].into_iter().all(f64::is_finite)
            || m <= 0.
            || i <= 0.
            || h <= 0.
            || a >= b
            || c <= 0.
            || mu < 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let full_tangent = -m * u.clamp(-mu * c, mu * c);
        let full_normal = m * c;
        let required = -h * full_tangent / full_normal;
        let (x, bearing) = if required < a {
            (a, Bearing::LowerXEdge)
        } else if required > b {
            (b, Bearing::UpperXEdge)
        } else {
            (required, Bearing::Face)
        };
        let (jt, jn, stick) = if bearing == Bearing::Face {
            (full_tangent, full_normal, u.abs() <= mu * c)
        } else {
            // Delassus operator in the single loaded edge's tangent/normal frame.
            // Its determinant is expanded to avoid cancellation of h²x²/I².
            let kt = 1. / m + h * h / i;
            let kn = 1. / m + x * x / i;
            let cross = h * x / i;
            let determinant = (1. / m + (h * h + x * x) / i) / m;
            let tangent = (-kn * u - cross * c) / determinant;
            let normal = (cross * u + kt * c) / determinant;
            if normal > 0. && tangent.abs() <= mu * normal {
                (tangent, normal, true)
            } else {
                // The sticking trial points to the active cone face, including
                // cases where off-center normal loading starts tangential slip.
                let sign = (-tangent).signum();
                let normal = c / (kn - sign * mu * cross);
                (-sign * mu * normal, normal, false)
            }
        };
        let vx = u + jt / m;
        let vz = -c + jn / m;
        let pitch = (-h * jt - x * jn) / i;
        let dissipated = 0.5 * (m * (u * u + c * c - vx * vx - vz * vz) - i * pitch * pitch);
        let outputs = [jt, jn, x, vx, vz, pitch, dissipated];
        if !outputs.into_iter().all(f64::is_finite) || jn <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(PlanarImpactResponse {
            impulse: [jt, jn],
            center_of_pressure: x,
            bearing,
            sticking: stick,
            com_velocity: [vx, vz],
            pitch_rate: pitch,
            dissipated_energy: dissipated,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> PlanarPatchImpact {
        PlanarPatchImpact {
            mass: 2.,
            pitch_inertia: 0.05,
            com_height: 0.13,
            supported_x: [-0.18, 0.18],
            tangent_speed: 1.8,
            closing_speed: 1.4,
            friction: 0.5,
        }
    }
    #[test]
    fn full_sliding_face_shifts_pressure_to_balance_pitch() {
        let r = fixture().solve().unwrap();
        assert_eq!(r.bearing, Bearing::Face);
        assert_eq!(r.impulse, [-1.4, 2.8]);
        assert!((r.center_of_pressure - 0.065).abs() < 1e-15);
        assert!(r.pitch_rate.abs() < 1e-14);
        assert!(!r.sticking);
    }
    #[test]
    fn reflected_cases_conserve_momentum_and_obey_both_contact_laws() {
        let mut modes = [0; 3];
        for left in -12..6 {
            for width in [1, 3, 8, 16] {
                for u in [-2., -0.4, 0., 0.4, 2.] {
                    for mu in [0., 0.1, 0.4, 0.8, 1.2] {
                        let a = f64::from(left) * 0.025;
                        let b = a + f64::from(width) * 0.025;
                        let f = PlanarPatchImpact {
                            supported_x: [a, b],
                            friction: mu,
                            tangent_speed: u,
                            ..fixture()
                        };
                        let r = f.solve().unwrap();
                        let rev = PlanarPatchImpact {
                            supported_x: [-b, -a],
                            tangent_speed: -u,
                            ..f
                        }
                        .solve()
                        .unwrap();
                        assert!(
                            (r.impulse[0] + rev.impulse[0]).abs() < 1e-11
                                && (r.impulse[1] - rev.impulse[1]).abs() < 1e-11
                        );
                        let [t, n] = r.impulse;
                        let w = r.pitch_rate;
                        assert!(
                            (f.pitch_inertia * w + f.com_height * t + r.center_of_pressure * n)
                                .abs()
                                < 1e-11
                        );
                        assert!(t.abs() <= mu * n + 1e-11 && n > 0.);
                        let [vx, vz] = r.com_velocity;
                        assert!(vz - w * a >= -1e-11 && vz - w * b >= -1e-11);
                        assert!((vz - w * r.center_of_pressure).abs() < 1e-11);
                        let slip = vx - w * f.com_height;
                        if r.sticking {
                            assert!(slip.abs() < 1e-11);
                        } else {
                            assert!((t.abs() - mu * n).abs() < 1e-11 && t * slip <= 1e-11);
                        }
                        assert!(r.dissipated_energy >= -1e-12);
                        modes[match r.bearing {
                            Bearing::Face => 0,
                            Bearing::LowerXEdge => 1,
                            Bearing::UpperXEdge => 2,
                        }] += 1;
                    }
                }
            }
        }
        assert!(modes.into_iter().all(|n| n > 100));
    }
    #[test]
    fn refuses_empty_support_and_unrepresentable_scales() {
        for f in [
            PlanarPatchImpact {
                supported_x: [0.; 2],
                ..fixture()
            },
            PlanarPatchImpact {
                friction: f64::NAN,
                ..fixture()
            },
            PlanarPatchImpact {
                mass: f64::MAX,
                closing_speed: 2.,
                ..fixture()
            },
        ] {
            assert!(f.solve().is_err());
        }
    }
}
