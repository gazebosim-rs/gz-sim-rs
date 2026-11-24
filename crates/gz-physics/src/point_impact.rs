// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A perfectly inelastic spatial point impact with isotropic Coulomb friction.
use crate::PhysicsError;
mod sliding;
mod sensitivity;
pub use sensitivity::PointImpactSensitivity;
mod monotone_slip;

/// Data in an orthonormal contact frame. The third axis points into the moving
/// body. The row-major Delassus matrix maps an impulse to the relative point
/// velocity change. It includes the effects of all ideal joint constraints.
#[derive(Clone, Copy, Debug)]
pub struct PointImpact {
    pub inverse_mass: [[f64; 3]; 3],
    pub relative_velocity: [f64; 3],
    pub friction: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct PointImpactResponse {
    pub impulse: [f64; 3],
    pub relative_velocity: [f64; 3],
    pub sticking: bool,
    /// Loss of kinetic energy in the support frame, in joules.
    pub dissipated_energy: f64,
}
// Choose an order with a representable intermediate when restoring units.
pub(crate) fn mul_div(x: f64, b: f64, c: f64) -> f64 {
    if x == 0. {
        return x;
    }
    let prod = x * b;
    if prod.is_normal() {
        return prod / c;
    }
    let ratio = b / c;
    if ratio.is_normal() {
        return x * ratio;
    }
    (x / c) * b
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(x, y)| x * y).sum()
}
fn poly(p: &[f64], x: f64) -> f64 {
    p.iter().rev().fold(0., |v, a| v.mul_add(x, *a))
}
/// All real roots in [0,1]. Between adjacent roots of the derivative the
/// polynomial is monotone. Tangencies occur at a derivative root itself.
fn roots(p: &[f64]) -> Vec<f64> {
    let scale = p.iter().map(|c| c.abs()).fold(0., f64::max);
    if scale == 0. {
        return vec![];
    }
    let mut len = p.len();
    while len > 1 && p[len - 1].abs() <= 2. * f64::EPSILON * scale {
        len -= 1;
    }
    let p = &p[..len];
    if len == 1 {
        return vec![];
    }
    if len == 2 {
        let r = -p[0] / p[1];
        return if (0. ..=1.).contains(&r) {
            vec![r]
        } else {
            vec![]
        };
    }
    let dp = p
        .iter()
        .enumerate()
        .skip(1)
        .map(|(i, v)| i as f64 * v)
        .collect::<Vec<_>>();
    let mut cuts = vec![0.];
    cuts.extend(roots(&dp));
    cuts.push(1.);
    let mut out = vec![];
    for &t in &cuts {
        if poly(p, t).abs() <= 8. * f64::EPSILON * scale {
            out.push(t);
        }
    }
    for ab in cuts.windows(2) {
        let (mut lo, mut hi) = (ab[0], ab[1]);
        let sign = poly(p, lo);
        if sign == 0.
            || poly(p, hi) == 0.
            || sign.is_sign_positive() == poly(p, hi).is_sign_positive()
        {
            continue;
        }
        for _ in 0..80 {
            let mid = (lo + hi) * 0.5;
            if mid == lo || mid == hi {
                break;
            }
            let f = poly(p, mid);
            if f == 0. {
                lo = mid;
                hi = mid;
                break;
            }
            if f.is_sign_positive() == sign.is_sign_positive() {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        out.push((lo + hi) * 0.5);
    }
    out.sort_unstable_by(f64::total_cmp);
    out.dedup_by(|x, y| *x == *y);
    out
}
impl PointImpact {
    /// Find the contact impulse whose normal velocity vanishes and whose
    /// friction impulse opposes the final slip. Singular and ambiguous
    /// responses are declined. Inputs must describe a closing single contact.
    pub fn solve(self) -> Result<PointImpactResponse, PhysicsError> {
        self.solve_with_scalar(false)
    }
    pub(crate) fn solve_implicit(self)->Result<PointImpactResponse,PhysicsError>{
        self.solve_with_scalar(true)
    }
    fn solve_with_scalar(self,scalar:bool)->Result<PointImpactResponse,PhysicsError>{
        let Self {
            inverse_mass: m,
            relative_velocity: v,
            friction: mu,
        } = self;
        if !m
            .into_iter()
            .flatten()
            .chain(v)
            .chain([mu])
            .all(f64::is_finite)
            || mu < 0.
            || v[2] >= 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let ms = m.into_iter().flatten().map(f64::abs).fold(0., f64::max);
        let vs = v.into_iter().map(f64::abs).fold(0., f64::max);
        if ms == 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let w = m.map(|r| r.map(|u| u / ms));
        let u = v.map(|p| p / vs);
        for (i, row) in w.iter().enumerate() {
            for (j, col) in w[..i].iter().enumerate() {
                if (row[j] - col[i]).abs() > 1e-11 {
                    return Err(PhysicsError::InvalidValue);
                }
            }
        }
        let n = w[2][2];
        if n <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let ax = w[0][0] - w[0][2] * w[2][0] / n;
        let ay = w[1][1] - w[1][2] * w[2][1] / n;
        let xy = w[0][1] - w[0][2] * w[2][1] / n;
        let det = ax * ay - xy * xy;
        // The normalized tangent Schur complement and normal pivot must
        // each resolve an independent velocity component at f64 precision.
        if n < 64. * f64::EPSILON || ax <= 0. || ay <= 0. || det <= 64. * f64::EPSILON * (ax + ay) {
            return Err(PhysicsError::InvalidValue);
        }
        let c = -u[2];
        let ux = u[0] + w[0][2] * c / n;
        let uy = u[1] + w[1][2] * c / n;
        let jx = (xy * uy - ay * ux) / det;
        let jy = (xy * ux - ax * uy) / det;
        let stationary = [jx, jy, (c - w[2][0] * jx - w[2][1] * jy) / n];
        let stick = (stationary[2] > 0. && jx.hypot(jy) / mu.max(1.) <= mu.min(1.) * stationary[2])
            .then_some(stationary);
        let (j, sticking) = if mu == 0. {
            ([0., 0., c / n], stick.is_some())
        } else if scalar && stick.is_none() && let Some(j)=monotone_slip::solve(w,u,mu) {
            (j,false)
        } else {
            sliding::solve(w, u, mu, stick)?
        };
        let impulse = j.map(|x| mul_div(x, vs, ms));
        let after = std::array::from_fn(|i| v[i] + dot(m[i], impulse));
        let dissipated_energy = -0.5 * dot(impulse, std::array::from_fn(|i| v[i] + after[i]));
        if !impulse
            .into_iter()
            .chain(after)
            .chain([dissipated_energy])
            .all(f64::is_finite)
            || impulse[2] <= 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(PointImpactResponse {
            impulse,
            relative_velocity: after,
            sticking,
            dissipated_energy,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn spherical_point_mass_selects_stick_or_slide_at_cone_boundary() {
        for mu in [0., 0.25, 0.5, 0.50000001, 1.] {
            let r = PointImpact {
                inverse_mass: [[0.5, 0., 0.], [0., 0.5, 0.], [0., 0., 0.5]],
                relative_velocity: [0.3, 0.4, -1.],
                friction: mu,
            }
            .solve()
            .unwrap();
            assert!((r.impulse[2] - 2.).abs() < 1e-12);
            assert!((r.impulse[0] + 1.2 * mu.min(0.5)).abs() < 1e-12);
            assert!((r.impulse[1] + 1.6 * mu.min(0.5)).abs() < 1e-12);
            assert_eq!(r.sticking, mu >= 0.5);
        }
    }
    #[test]
    fn spatial_sliding_uses_postimpact_direction_and_conserves_energy_balance() {
        let matrix = [[2., 0.3, -0.6], [0.3, 0.9, 0.4], [-0.6, 0.4, 1.3]];
        for x in -9..=9 {
            for y in -7..=7 {
                for mu in [0., 0.01, 0.2, 0.8, 1.9] {
                    let v = [f64::from(x) * 0.16, f64::from(y) * 0.22, -0.79];
                    let r = PointImpact {
                        inverse_mass: matrix,
                        relative_velocity: v,
                        friction: mu,
                    }
                    .solve()
                    .unwrap();
                    let [a, b, n] = r.impulse;
                    let [u, v, w] = r.relative_velocity;
                    assert!(
                        n > 0.
                            && w.abs() < 1e-11
                            && a.hypot(b) <= mu * n + 1e-9
                            && r.dissipated_energy >= -1e-11
                    );
                    if r.sticking {
                        assert!(u.hypot(v) < 1e-11);
                    } else {
                        assert!(
                            (a.hypot(b) - mu * n).abs() < 1e-9
                                && (a * v - b * u).abs() < 1e-10
                                && a * u + b * v <= 1e-10
                        );
                    }
                }
            }
        }
    }
    #[test]
    fn refuses_invalid_matrices_and_opening_contact() {
        for (m, z) in [
            ([[0.; 3]; 3], -1.),
            ([[1., 1., 0.], [0., 1., 0.], [0., 0., 1.]], -1.),
            ([[1., 0., 0.], [0., -1., 0.], [0., 0., 1.]], -1.),
            ([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], 0.),
        ] {
            assert!(
                PointImpact {
                    inverse_mass: m,
                    relative_velocity: [0., 1., z],
                    friction: 0.2
                }
                .solve()
                .is_err()
            );
        }
        let r = roots(&[0.0625, -0.5, 1.]);
        assert_eq!(r, vec![0.25]);
    }
}

#[cfg(test)]
mod conditioning_tests;
