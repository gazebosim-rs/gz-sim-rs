// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Select every admissible sliding direction on the Coulomb cone.
use super::{PhysicsError, dot, roots};
use glam::DVec2;

fn ambiguous() -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "multifidelity",
        capability: "unique certified sliding point impact",
    }
}

pub(super) fn solve(
    w: [[f64; 3]; 3],
    u: [f64; 3],
    mu: f64,
    stick: Option<[f64; 3]>,
) -> Result<([f64; 3], bool), PhysicsError> {
    if let Some(j) = stick
        && unique_static(w, mu, j)
    {
        return Ok((j, true));
    }
    enumerate(w, u, mu, stick)
}

/// If a different solution generated slip s*d with s>0, its impulse would be
/// j + s W^-1 (d,0). Dot the tangent cone equation with d to get
/// mu*j_n + d·j_t + s (d·A d + mu*b·d) = 0, where A and b are the tangent
/// block and tangent/normal column of W^-1. Both terms must be positive
/// when j is strictly interior and lambda_min(A) > mu*|b|.
fn unique_static(w: [[f64; 3]; 3], mu: f64, j: [f64; 3]) -> bool {
    let n = w[2][2];
    let c = DVec2::new(w[0][2], w[1][2]);
    let scale = mu.max(1.);
    let scaled_mu = mu.min(1.);
    let normal = scaled_mu * j[2];
    let tangent = j[0].hypot(j[1]) / scale;
    if normal - tangent <= 2048. * f64::EPSILON * (normal + tangent) {
        return false;
    }
    // The normalized tangent Schur complement S has A = inv(S).
    // det(S)*b = -adj(S)*c/n. Bound lambda_min(S) from below using
    // det(S)/trace(S). This avoids subtracting nearly equal eigenvalues.
    let ax = w[0][0] - c.x * w[2][0] / n;
    let ay = w[1][1] - c.y * w[2][1] / n;
    let off = w[0][1] - c.x * w[2][1] / n;
    let det = ax * ay - off * off;
    let left = (det / (ax + ay)) * (n / scale);
    let right = scaled_mu * DVec2::new(ay * c.x - off * c.y, ax * c.y - off * c.x).length();
    // Conditioning of the Schur complement and symmetry matter here. The
    // generic solver permits mildly asymmetric input; use its full enumeration
    // whenever those deviations could decide the inequality.
    let guard = 4096. * f64::EPSILON;
    let symmetry = (w[0][1] - w[1][0])
        .abs()
        .max((w[0][2] - w[2][0]).abs())
        .max((w[1][2] - w[2][1]).abs());
    n > guard
        && ax + ay > guard
        && left - right > (guard + 8. * symmetry) * (1. + 1. / n) * (1. + 1. / (ax + ay))
}

fn enumerate(
    w: [[f64; 3]; 3],
    u: [f64; 3],
    mu: f64,
    stick: Option<[f64; 3]>,
) -> Result<([f64; 3], bool), PhysicsError> {
    // Impulse = scale * (-b * direction.x, -b * direction.y, a).
    // Either coefficient can approach zero without subtracting nearly equal
    // multipliers or forming the square of a large friction coefficient.
    let a = 1. / mu.max(1.);
    let b = mu.min(1.);
    let cross = DVec2::new(w[0][2], w[1][2]);
    let incoming = DVec2::new(u[0], u[1]);
    let c = -u[2];
    let tangent =
        |d: DVec2| DVec2::new(w[0][0] * d.x + w[0][1] * d.y, w[1][0] * d.x + w[1][1] * d.y);
    let mut result = stick.map(|j| (j, true));
    let mut direction = None::<DVec2>;
    for ex in [DVec2::X, DVec2::Y, DVec2::NEG_X, DVec2::NEG_Y] {
        let ey = ex.perp();
        // Rational parametrization of one quadrant. Directions cover the full
        // circle using four well-conditioned charts, including axial slip.
        let d = [ex, 2. * ey, -ex]; // unit direction times (1+t^2)
        let radius = [1., 0., 1.];
        let e: [DVec2; 3] = std::array::from_fn(|k| {
            let den = a * w[2][2] * radius[k] - b * cross.dot(d[k]);
            incoming * den + c * (a * radius[k] * cross - b * tangent(d[k]))
        });
        // The component perpendicular to the final slip must vanish.
        let mut poly = [0.; 5];
        for k in 0..3 {
            for l in 0..3 {
                poly[k + l] += d[k].perp_dot(e[l]);
            }
        }
        for t in roots(&poly) {
            let d = (ex * (1. - t * t) + 2. * t * ey) / (1. + t * t);
            let den = a * w[2][2] - b * cross.dot(d);
            if den <= 0. {
                continue;
            }
            let scale = c / den;
            let j = [-b * d.x * scale, -b * d.y * scale, a * scale];
            if !j.into_iter().all(f64::is_finite) {
                return Err(ambiguous());
            }
            let post = DVec2::new(u[0] + dot(w[0], j), u[1] + dot(w[1], j));
            let speed = post.length();
            let roundoff = 128.
                * f64::EPSILON
                * (incoming.length() + scale * (a * cross.length() + b * tangent(d).length()));
            if d.dot(post) < -roundoff {
                continue;
            }
            if speed <= roundoff
                && let Some(st) = stick
            {
                // Boundary shared by sticking and sliding descriptions.
                let delta = j
                    .iter()
                    .zip(st)
                    .map(|(a, b)| (a - b).abs())
                    .fold(0., f64::max);
                if delta <= 256. * f64::EPSILON * j.into_iter().map(f64::abs).fold(0., f64::max) {
                    continue;
                }
            }
            if speed == 0. || d.perp_dot(post).abs() > roundoff + 128. * f64::EPSILON * speed {
                return Err(ambiguous());
            }
            if let Some(prev) = direction
                && prev.distance(d) < 512. * f64::EPSILON
            {
                continue; // shared quadrant boundary
            }
            if result.is_some() {
                return Err(ambiguous());
            }
            // A repeated angular root is unstable under a small perturbation
            // of the incoming state. Decline the unresolved physical branch.
            let deriv = poly[1] + t * (2. * poly[2] + t * (3. * poly[3] + t * 4. * poly[4]));
            let coeff_norm = poly.into_iter().map(f64::abs).sum::<f64>();
            if deriv.abs() <= (128. * f64::EPSILON).sqrt() * coeff_norm {
                return Err(ambiguous());
            }
            result = Some((j, false));
            direction = Some(d);
        }
    }
    result.ok_or_else(ambiguous)
}

#[cfg(test)]
mod static_uniqueness_tests {
    use super::*;
    #[test]
    fn certified_static_impulse_matches_all_four_direction_charts() {
        // Real coupled triaxial mobilities, rotated relative to the contact.
        // Generate incoming velocity from a known interior stationary impulse.
        let mut qualified = 0;
        let mut enumerated = 0;
        for a in [0.08, 0.29, 0.83, 1.71] {
            for b in [0.13, 0.52, 1.25] {
                for c in [-0.86, -0.19, 0.22, 0.73] {
                    let q = glam::DQuat::from_rotation_y(a)
                        * glam::DQuat::from_rotation_x(b)
                        * glam::DQuat::from_rotation_z(c);
                    for axes in [
                        glam::DVec3::new(1., 1., 1.),
                        glam::DVec3::new(0.04, 0.33, 1.),
                        glam::DVec3::new(0.001, 0.66, 1.),
                    ] {
                        let col = [glam::DVec3::X, glam::DVec3::Y, glam::DVec3::Z]
                            .map(|d| q * (axes * (q.inverse() * d)));
                        let w = std::array::from_fn(|i| std::array::from_fn(|j| col[j][i]));
                        for mu in [0.1, 0.41, 0.9, 1.7, 4.0] {
                            for angle in [0.12_f64, 1.26, 2.58, 3.61, 5.32] {
                                for r in [0.06, 0.67, 0.999] {
                                    let j = [r * mu * angle.cos(), r * mu * angle.sin(), 1.];
                                    let u = w.map(|row| -dot(row, j));
                                    if u[2] >= 0. {
                                        continue;
                                    }
                                    if unique_static(w, mu, j) {
                                        assert_eq!(
                                            enumerate(w, u, mu, Some(j)).unwrap(),
                                            (j, true)
                                        );
                                        qualified += 1;
                                    } else {
                                        enumerated += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        assert!(qualified > 2000 && enumerated > 1000);
    }
}
