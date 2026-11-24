// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A sufficient Lipschitz bound for a closing isotropic Coulomb point.
#[derive(Clone, Copy, Debug)]
pub struct PointImpactSensitivity {
    /// Impulse norm per unit incoming-velocity perturbation.
    gain: f64,
    /// Roundoff from interpreting the supplied physical matrix as symmetric.
    model_error: f64,
}
impl PointImpactSensitivity {
    /// Construct a bound in an orthonormal contact frame (normal axis 2).
    /// The matrix maps world-scale impulse components to relative velocities.
    /// Returns None for invalid input or when the sufficient uniqueness
    /// inequality does not separate the tangent and normal responses.
    pub fn new(inverse_mass: [[f64; 3]; 3], friction: f64) -> Option<Self> {
        if !friction.is_finite()
            || friction < 0.
            || !inverse_mass.iter().flatten().all(|v| v.is_finite())
        {
            return None;
        }
        let scale = inverse_mass
            .iter()
            .flatten()
            .map(|v| v.abs())
            .fold(0., f64::max);
        if scale == 0. {
            return None;
        }
        let m = inverse_mass.map(|r| r.map(|v| v / scale));
        let mut skew = 0f64;
        for (j, row) in m.iter().enumerate() {
            for (k, other) in m[..j].iter().enumerate() {
                skew = skew.max((row[k] - other[j]).abs());
            }
        }
        let eps = 128. * f64::EPSILON;
        if skew > eps || m[2][2] <= eps {
            return None;
        }
        let n = m[2][2];
        let nx = (m[2][0] + m[0][2]) * 0.5;
        let ny = (m[2][1] + m[1][2]) * 0.5;
        let a = m[0][0] - nx * nx / n;
        let d = m[1][1] - ny * ny / n;
        let b = (m[0][1] + m[1][0]) * 0.5 - nx * ny / n;
        let schur_err = eps * (1. + nx.abs().max(ny.abs()).powi(2) / n);
        let diff = (a - d).hypot(2. * b);
        let lo = (a + d - diff) * 0.5 - schur_err;
        let hi = (a + d + diff) * 0.5 + schur_err;
        if lo <= 0. || !hi.is_finite() {
            return None;
        }
        let n = n - eps;
        let ratio = (nx.hypot(ny) + eps) / n;
        let g = if friction == 0. {
            1. / n
        } else {
            // The disk-constrained minimizer of 1/2 y'Sy+b'y varies with
            // disk radius at rate at most (k+1)/(2 sqrt(k)). This follows
            // by differentiating (S+lambda I)y=-b along its active boundary.
            let root = (hi / lo).sqrt();
            let radius_gain = (root + 1. / root) * 0.5 * (1. + eps);
            let contraction = 1. - radius_gain * friction * ratio - eps;
            if contraction <= 0. {
                return None;
            }
            let tangent = ((1. + ratio) / lo + radius_gain * friction / n) / contraction;
            tangent.hypot(1. / n + ratio * tangent)
        };
        let gain = g / scale * (1. + eps);
        let model_error = (3. * skew) * scale;
        (gain.is_finite() && model_error.is_finite()).then_some(Self { gain, model_error })
    }
    /// Bound the Euclidean difference between two exact point reactions.
    /// Supply the nominal reaction norm, an absolute relative-velocity error
    /// (m/s) and a spectral norm bound on the matrix error (inverse kg).
    /// Both inputs must describe closing contacts with the same friction and
    /// normal frame. The perturbed physical matrix must be positive definite.
    /// Sticking/sliding transitions are supported. Returns None if the
    /// matrix-error feedback precludes a finite bound.
    pub fn impulse_error(
        self,
        nominal_impulse: f64,
        velocity_error: f64,
        matrix_error: f64,
    ) -> Option<f64> {
        if ![nominal_impulse, velocity_error, matrix_error]
            .into_iter()
            .all(|v| v.is_finite() && v >= 0.)
        {
            return None;
        }
        let matrix = matrix_error + self.model_error;
        let slack = 1. - self.gain * matrix;
        if slack <= 0. {
            return None;
        }
        let bound = self.gain * (matrix.mul_add(nominal_impulse, velocity_error)) / slack
            * (1. + 128. * f64::EPSILON);
        bound.is_finite().then_some(bound)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::PointImpact;
    use glam::{DMat3, DVec3};
    fn rows(m: DMat3) -> [[f64; 3]; 3] {
        m.transpose().to_cols_array_2d()
    }
    #[test]
    fn covers_velocity_and_inertia_perturbations_on_both_sides_of_sticking() {
        let base = DMat3::from_cols(
            DVec3::new(1.3, 0.21, 0.16),
            DVec3::new(0.21, 0.94, -0.09),
            DVec3::new(0.16, -0.09, 1.1),
        );
        let mut checked = 0;
        let mut changed = 0;
        for mu in [0., 0.15, 0.55, 0.9] {
            for rot in [0., 0.57, 2.23] {
                let u = DMat3::from_rotation_z(rot);
                let m = u * base * u.transpose();
                let b = PointImpactSensitivity::new(rows(m), mu).unwrap();
                for slip in [-0.01, -1e-5, 0.001, 0.4] {
                    let j = u * DVec3::new(-mu, 0., 1.);
                    let vel = u * DVec3::X * slip - m * j;
                    let eval = |m, v: DVec3| {
                        PointImpact {
                            inverse_mass: rows(m),
                            relative_velocity: v.to_array(),
                            friction: mu,
                        }
                        .solve()
                        .unwrap()
                    };
                    let a = eval(m, vel);
                    let nominal = DVec3::from(a.impulse);
                    for delta in [-0.02, 0.005, 0.035] {
                        let d = u * DVec3::new(delta, -0.31 * delta, 0.23 * delta);
                        let dv = u * DVec3::new(1.7 * delta, 0.81 * delta, 0.37 * delta);
                        let n = DMat3::from_cols(d * d.x, d * d.y, d * d.z);
                        let out = eval(m + n, vel + dv);
                        let bound = b
                            .impulse_error(nominal.length(), dv.length(), d.length_squared())
                            .unwrap();
                        assert!(DVec3::from(out.impulse).distance(nominal) <= bound + 2e-13);
                        checked += 1;
                        if a.sticking != out.sticking {
                            changed += 1;
                        }
                    }
                }
            }
        }
        assert_eq!(checked, 144);
        assert!(changed >= 10);
    }
    #[test]
    fn respects_mass_units_and_refuses_unbounded_feedback() {
        for scale in [1e-120, 1., 1e120] {
            let b = PointImpactSensitivity::new(rows(DMat3::IDENTITY * scale), 0.).unwrap();
            let j = b.impulse_error(0., 0.3, 0.).unwrap();
            assert!((j * scale / 0.3 - 1.).abs() < 1e-11);
            assert!(b.impulse_error(1., 0.3, 2. * scale).is_none());
        }
        assert!(PointImpactSensitivity::new([[0.; 3]; 3], 0.).is_none());
        assert!(PointImpactSensitivity::new(rows(DMat3::IDENTITY), f64::NAN).is_none());
    }
}
