// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A scalar Coulomb solve for a strictly monotone angular response. Other
//! sliding configurations retain the complete angular root enumeration.
use glam::DVec2;
/// Normalized SPD mobility and incoming velocity, in a tangent/normal frame.
pub(super) fn solve(w: [[f64; 3]; 3], u: [f64; 3], mu: f64) -> Option<[f64; 3]> {
    let n = w[2][2];
    let c = DVec2::new(w[0][2], w[1][2]);
    let c0 = -u[2];
    let m = mu.max(1.);
    let a = 1. / m;
    let b = mu.min(1.);
    let sx = w[0][0] - c.x * c.x / n;
    let sy = w[1][1] - c.y * c.y / n;
    let off = w[0][1] - c.x * c.y / n;
    let det = sx * sy - off * off;
    let trace = sx + sy;
    let sym = (w[0][1] - w[1][0])
        .abs()
        .max((w[0][2] - w[2][0]).abs())
        .max((w[1][2] - w[2][1]).abs());
    if n < 1e-8
        || det < 1e-10 * trace * trace
        || sym > 64. * f64::EPSILON
        || !(det > 0. && trace > 0.)
    {
        return None;
    }
    // With x=t/(1+t), j_t=-x A^-1 v_f, A=(1-x)Id+xS, S the tangent
    // Schur complement. The derivative is -A^-2 v_f. Their cosine is at
    // least 2 sqrt(det S)/tr S since cond(A) <= cond(S) throughout [0,1].
    let min_cos = 2. * det.sqrt() / trace;
    let margin = a * n * min_cos - b * c.length();
    if margin <= 1e-11 * (1. + 1. / n) * (1. + trace / det.sqrt()) {
        return None;
    }
    let free = DVec2::new(u[0], u[1]) + c * (c0 / n);
    if !free.is_finite() || free.length_squared() == 0. {
        return None;
    }
    let (mut lo, mut hi) = (0., 1.);
    let mut x = b * c0 / (a * n * free.length() + b * c0);
    for _ in 0..32 {
        let (ax, ay, cross) = (1. - x + x * sx, 1. - x + x * sy, x * off);
        let den = ax * ay - cross * cross;
        let inverse = |v: DVec2| DVec2::new(ay * v.x - cross * v.y, ax * v.y - cross * v.x) / den;
        let inv = inverse(free);
        let tangent = -x * inv;
        let length = tangent.length();
        if !(length > 0. && den > 0.) {
            return None;
        }
        let normal = (c0 - c.dot(tangent)) / n;
        let residual = a * n * length - b * n * normal;
        let scale = a * n * length + b * (c0.abs() + c.length() * length);
        if residual.abs() <= 32. * f64::EPSILON * scale && x < 1. - 1e-8 && normal > 0. {
            let j = [tangent.x, tangent.y, normal];
            let slip = DVec2::new(u[0] + super::dot(w[0], j), u[1] + super::dot(w[1], j));
            let allowance =
                256. * f64::EPSILON * (free.length() + j[0].abs() + j[1].abs() + normal);
            let d = -tangent / length;
            if slip.dot(d) > allowance
                && d.perp_dot(slip).abs() < allowance
                && (u[2] + super::dot(w[2], j)).abs() < allowance
            {
                return Some(j);
            }
            return None;
        }
        if residual > 0. {
            hi = x;
        } else {
            lo = x;
        }
        let rate = -inverse(inv);
        let derivative = a * n * (tangent / length).dot(rate) + b * c.dot(rate);
        if derivative <= 0. || !derivative.is_finite() {
            return None;
        }
        let next = x - residual / derivative;
        x = if next > lo && next < hi {
            next
        } else {
            0.5 * (lo + hi)
        };
    }
    None
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn monotone_scalar_impulses_match_all_angular_roots_for_sliding_controls() {
        let mut solved = 0;
        let mut declined = 0;
        for k in 0..60 {
            let q = glam::DQuat::from_rotation_x(0.17 * k as f64)
                * glam::DQuat::from_rotation_z(0.23 * k as f64);
            let axes = glam::DVec3::new(
                0.035 + (k % 9) as f64 / 10.,
                0.021 + (k % 15) as f64 / 16.,
                1.,
            );
            let cols = [glam::DVec3::X, glam::DVec3::Y, glam::DVec3::Z]
                .map(|v| q * (axes * (q.inverse() * v)));
            let w = std::array::from_fn(|i| std::array::from_fn(|j| cols[j][i]));
            for mu in [0.01, 0.25, 0.6, 1.2, 3.] {
                for i in 0..12 {
                    let t = i as f64 * 0.52;
                    let u = [t.sin(), t.cos(), -0.35];
                    let Some(answer) = solve(w, u, mu) else {
                        declined += 1;
                        continue;
                    };
                    let physical = crate::PointImpact {
                        inverse_mass: w,
                        relative_velocity: u,
                        friction: mu,
                    }
                    .solve()
                    .unwrap();
                    assert!(
                        !physical.sticking
                            && glam::DVec3::from(answer).distance(physical.impulse.into()) < 2e-11
                    );
                    solved += 1;
                }
            }
        }
        assert!(solved > 1200 && declined > 250);
    }
}
