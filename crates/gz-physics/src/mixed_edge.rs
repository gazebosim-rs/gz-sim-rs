// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two-site impacts with at least one sticking vertex of a rigid edge.
use crate::{
    EdgeMobility, PhysicsError,
    edge_impact::{ScaledEdge, positive_semidefinite},
    point_impact::mul_div,
};
use glam::{DMat2, DMat3, DVec2, DVec3};

/// The first tangent axis runs from site zero to site one. The second
/// completes a right-handed frame with the common support normal.
#[derive(Clone, Copy, Debug)]
pub struct MixedEdgeImpact {
    pub inverse_mass: EdgeMobility,
    pub relative_velocity: [[f64; 3]; 2],
    pub friction: [f64; 2],
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EdgeStickMode {
    BothStick,
    FirstSticks,
    SecondSticks,
}
/// The complete feasible set (N·s) in the edge-aligned contact frame. The
/// first longitudinal impulse component determines the second by subtraction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EdgeImpulseSet {
    pub longitudinal_total: f64,
    pub transverse: [f64; 2],
    pub normal: [f64; 2],
    pub first_longitudinal_range: [f64; 2],
}
impl EdgeImpulseSet {
    /// Evaluate one admissible split without implying a measured allocation.
    pub fn split(self, first: f64) -> Option<[[f64; 3]; 2]> {
        let [lo, hi] = self.first_longitudinal_range;
        (first.is_finite() && first >= lo && first <= hi).then_some([
            [first, self.transverse[0], self.normal[0]],
            [
                self.longitudinal_total - first,
                self.transverse[1],
                self.normal[1],
            ],
        ])
    }
    fn scaled(self, velocity: f64, mobility: f64) -> Self {
        let scale = |x| mul_div(x, velocity, mobility);
        Self {
            longitudinal_total: scale(self.longitudinal_total),
            transverse: self.transverse.map(scale),
            normal: self.normal.map(scale),
            first_longitudinal_range: self.first_longitudinal_range.map(scale),
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MixedEdgeResponse {
    pub mode: EdgeStickMode,
    pub impulses: EdgeImpulseSet,
    pub relative_velocity: [[f64; 3]; 2],
    /// Sufficient, mode-independent bound for uniqueness of the observable
    /// velocity, both normal loads and the five resultant impulse components.
    pub uniqueness_bound: f64,
    pub dissipated_energy: f64,
}
const TOL: f64 = 512. * f64::EPSILON;
fn unsupported() -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "multifidelity",
        capability: "certified mixed sliding or sticking rigid-edge impact",
    }
}
type Matrix = [[f64; 5]; 5];
fn linear(mut a: Matrix, mut b: [f64; 5]) -> Option<[f64; 5]> {
    // Normalize each equation before partial pivoting. The Coulomb row can
    // have very different units and scale from the four closure equations.
    for (row, v) in a.iter_mut().zip(&mut b) {
        let norm = row.iter().copied().map(f64::abs).fold(0., f64::max);
        if !norm.is_finite() || norm == 0. {
            return None;
        }
        for x in row {
            *x /= norm;
        }
        *v /= norm;
    }
    for k in 0..5 {
        let p = (k..5)
            .max_by(|i, j| a[*i][k].abs().total_cmp(&a[*j][k].abs()))
            .unwrap();
        if a[p][k].abs() <= TOL {
            return None;
        }
        a.swap(p, k);
        b.swap(p, k);
        let pivot = a[k];
        let rhs = b[k];
        for (row, v) in a[k + 1..].iter_mut().zip(&mut b[k + 1..]) {
            let m = row[k] / pivot[k];
            for (j, val) in row.iter_mut().enumerate().skip(k + 1) {
                *val -= m * pivot[j];
            }
            *v -= m * rhs;
        }
    }
    let mut result = [0.; 5];
    for i in (0..5).rev() {
        let rest = a[i][i + 1..]
            .iter()
            .zip(&result[i + 1..])
            .map(|(w, x)| w * x)
            .sum::<f64>();
        result[i] = (b[i] - rest) / a[i][i];
    }
    result.iter().all(|x| x.is_finite()).then_some(result)
}
fn components(v: [DVec3; 2]) -> [f64; 5] {
    [0.5 * (v[0].x + v[1].x), v[0].y, v[0].z, v[1].y, v[1].z]
}
fn work_matrix(w: [[DMat3; 2]; 2]) -> Matrix {
    let basis = [
        [DVec3::X * 0.5, DVec3::X * 0.5],
        [DVec3::Y, DVec3::ZERO],
        [DVec3::Z, DVec3::ZERO],
        [DVec3::ZERO, DVec3::Y],
        [DVec3::ZERO, DVec3::Z],
    ];
    let cols = basis.map(|j| components(std::array::from_fn(|i| w[i][0] * j[0] + w[i][1] * j[1])));
    std::array::from_fn(|i| std::array::from_fn(|j| cols[j][i]))
}
fn uniqueness(h: Matrix, mu: [f64; 2]) -> Option<f64> {
    let n = [2, 4];
    let t = [0, 1, 3];
    let nn = DMat2::from_cols_array_2d(&n.map(|j| n.map(|i| h[i][j])));
    let nscale = nn.x_axis.length().max(nn.y_axis.length());
    if nn.x_axis.x <= TOL * nscale || nn.determinant() <= TOL * nscale.powi(2) {
        return None;
    }
    let inv = nn.inverse();
    let a = t.map(|j| inv * DVec2::new(h[n[0]][j], h[n[1]][j]));
    let s = DMat3::from_cols_array_2d(&std::array::from_fn(|j| {
        t.map(|i| h[i][t[j]] - h[i][n[0]] * a[j].x - h[i][n[1]] * a[j].y)
    }));
    let scale = s
        .x_axis
        .length()
        .max(s.y_axis.length())
        .max(s.z_axis.length());
    // Positive leading minors qualify the reduced work metric. A positive
    // determinant alone also occurs with two negative eigenvalues.
    let leading = s.x_axis.x * s.y_axis.y - s.x_axis.y * s.y_axis.x;
    if s.x_axis.x <= TOL * scale
        || leading <= TOL * scale.powi(2)
        || s.determinant() <= TOL * scale.powi(3)
    {
        return None;
    }
    let inverse = s.inverse();
    let mut bound = 0.;
    for (i, friction) in mu.into_iter().enumerate() {
        let y = i + 1;
        let radial = DVec3::from_array(a.map(|col| col[i]));
        let e = s.x_axis.x;
        let r = s.col(y)[y];
        let xy = s.col(y).x;
        let eig = 0.5 * (e + r + (e - r).hypot(2. * xy));
        let work = radial.dot(inverse * radial);
        if work < 0. {
            return None;
        }
        bound += friction * (eig * work).sqrt();
    }
    (bound.is_finite() && bound < 1. - TOL).then_some(bound)
}
fn candidate(
    h: Matrix,
    u: [f64; 5],
    mu: [f64; 2],
    mode: EdgeStickMode,
    sign: f64,
) -> Option<(EdgeImpulseSet, [[f64; 3]; 2])> {
    let sliding = match mode {
        EdgeStickMode::BothStick => None,
        EdgeStickMode::FirstSticks => Some(1),
        EdgeStickMode::SecondSticks => Some(0),
    };
    let (mut m, mut rhs) = (h, u.map(|x| -x));
    if let Some(i) = sliding {
        let y = 1 + 2 * i;
        m[y] = [0.; 5];
        m[y][y] = 1.;
        m[y][y + 1] = sign * mu[i];
        rhs[y] = 0.;
    }
    let g = linear(m, rhs)?;
    let tol = TOL * (1. + g.iter().copied().map(f64::abs).fold(0., f64::max));
    let post: [f64; 5] =
        std::array::from_fn(|i| u[i] + h[i].into_iter().zip(g).map(|(w, j)| w * j).sum::<f64>());
    let n = [g[2], g[4]];
    let ty = [g[1], g[3]];
    if n.into_iter().any(|n| n <= tol)
        || post[0].abs() > tol
        || post[2].abs() > tol
        || post[4].abs() > tol
    {
        return None;
    }
    let range = if let Some(i) = sliding {
        let k = 1 - i;
        if post[1 + 2 * i] * sign <= tol
            || post[1 + 2 * k].abs() > tol
            || g[0].hypot(ty[k]) > mu[k] * n[k]
        {
            return None;
        }
        let x = if i == 0 { 0. } else { g[0] };
        [x, x]
    } else {
        if post[1].abs() > tol || post[3].abs() > tol {
            return None;
        }
        let mut radius = [0.; 2];
        for i in 0..2 {
            let limit = mu[i] * n[i];
            let tang = ty[i].abs();
            if tang > limit || !limit.is_finite() {
                return None;
            }
            radius[i] = (limit - tang).sqrt() * (limit + tang).sqrt();
        }
        let lo = (-radius[0]).max(g[0] - radius[1]);
        let hi = radius[0].min(g[0] + radius[1]);
        if !(lo.is_finite() && hi.is_finite() && lo <= hi) {
            return None;
        }
        [lo, hi]
    };
    Some((
        EdgeImpulseSet {
            longitudinal_total: g[0],
            transverse: ty,
            normal: n,
            first_longitudinal_range: range,
        },
        [[post[0], post[1], post[2]], [post[0], post[3], post[4]]],
    ))
}
impl MixedEdgeImpact {
    /// Solve the closed two-site normal equations and enumerate both sticking
    /// and each possible transverse mixed slip. The opposite along-edge
    /// self-stress is left as a closed interval when both points stick.
    pub fn solve(self) -> Result<MixedEdgeResponse, PhysicsError> {
        let ScaledEdge { w, v, ws, vs } =
            ScaledEdge::new(self.inverse_mass, self.relative_velocity, self.friction)?;
        if vs == 0. {
            return Err(unsupported());
        }
        if !positive_semidefinite(w) {
            return Err(PhysicsError::InvalidValue);
        }
        // Equal longitudinal velocities and a longitudinal null impulse
        // characterize two points on the same rigid link and rigid support.
        if (v[0].x - v[1].x).abs() > TOL
            || w.iter().any(|r| r[0].x_axis.distance(r[1].x_axis) > TOL)
        {
            return Err(unsupported());
        }
        for i in 0..2 {
            if v[i].z >= -TOL {
                return Err(unsupported());
            }
            let k = 1 - i;
            let nt = |j: usize| w[j][i].x_axis.z.hypot(w[j][i].y_axis.z);
            let lower = w[i][i].z_axis.z - self.friction[i] * nt(i);
            if lower <= TOL * w[i][i].z_axis.z {
                return Err(unsupported());
            }
            let max_kick =
                (w[k][i].z_axis.z + self.friction[i] * nt(k)).max(0.) * (-v[i].z) / lower;
            if v[k].z + max_kick >= -TOL {
                return Err(unsupported());
            }
        }
        let h = work_matrix(w);
        let u = components(v);
        let mu = self.friction;
        let bound = uniqueness(h, mu).ok_or_else(unsupported)?;
        let modes = [
            (EdgeStickMode::BothStick, 0.),
            (EdgeStickMode::FirstSticks, -1.),
            (EdgeStickMode::FirstSticks, 1.),
            (EdgeStickMode::SecondSticks, -1.),
            (EdgeStickMode::SecondSticks, 1.),
        ];
        for (mode, sign) in modes {
            let Some((set, post)) = candidate(h, u, mu, mode, sign) else {
                continue;
            };
            let impulses = set.scaled(vs, ws);
            let [lo, hi] = impulses.first_longitudinal_range;
            let j = impulses
                .split(lo * 0.5 + hi * 0.5)
                .ok_or(PhysicsError::InvalidValue)?;
            let post = post.map(|v| v.map(|x| x * vs));
            let loss = -(0..2)
                .map(|i| {
                    DVec3::from(j[i]).dot(
                        0.5 * DVec3::from(self.relative_velocity[i]) + 0.5 * DVec3::from(post[i]),
                    )
                })
                .sum::<f64>();
            if !loss.is_finite()
                || loss < 0.
                || j.iter().chain(&post).flatten().any(|x| !x.is_finite())
                || impulses.normal.iter().any(|n| *n <= 0.)
            {
                return Err(PhysicsError::InvalidValue);
            }
            return Ok(MixedEdgeResponse {
                mode,
                impulses,
                relative_velocity: post,
                uniqueness_bound: bound,
                dissipated_energy: loss,
            });
        }
        Err(unsupported())
    }
}

#[cfg(test)]
mod tests;
