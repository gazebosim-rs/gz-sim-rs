// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Simultaneous sliding impulses at two spatial points with parallel normals.
use crate::PhysicsError;
use glam::{DMat2, DMat3, DVec2, DVec3};

/// Row-major world/contact-frame 3×3 mobility blocks. Block (i,j) maps a
/// spatial impulse at site j to a point velocity increment at site i.
pub type EdgeMobility = [[[[f64; 3]; 3]; 2]; 2];

/// Both sites share one orthonormal frame whose z-axis is the support normal.
/// A prescribed support may have a different incoming point velocity at each site.
#[derive(Clone, Copy, Debug)]
pub struct SlidingEdgeImpact {
    pub inverse_mass: EdgeMobility,
    pub relative_velocity: [[f64; 3]; 2],
    pub friction: [f64; 2],
}
#[derive(Clone, Copy, Debug)]
pub struct SlidingEdgeResponse {
    pub impulses: [[f64; 3]; 2],
    pub relative_velocity: [[f64; 3]; 2],
    /// Upper bound on the tangent fixed-point map's Lipschitz constant.
    pub contraction: f64,
    /// A posteriori full-vector impulse iteration error per site (N·s),
    /// excluding floating-point roundoff in the input and calculations.
    pub iterate_error_bound: f64,
    pub dissipated_energy: f64,
}
fn decline() -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "multifidelity",
        capability: "unique simultaneous sliding two-site impact",
    }
}
fn op_norm(m: DMat2) -> f64 {
    let a = m.x_axis.length_squared();
    let b = m.y_axis.length_squared();
    ((a + b + (a - b).hypot(2. * m.x_axis.dot(m.y_axis))) * 0.5).sqrt()
}
/// Dimensionless work coordinates shared by two-site active-set solvers.
pub(crate) struct ScaledEdge {
    pub w: [[DMat3; 2]; 2],
    pub v: [DVec3; 2],
    pub ws: f64,
    pub vs: f64,
}
impl ScaledEdge {
    pub fn new(
        mobility: EdgeMobility,
        velocity: [[f64; 3]; 2],
        friction: [f64; 2],
    ) -> Result<Self, PhysicsError> {
        if friction.iter().any(|m| !m.is_finite() || *m < 0.)
            || velocity.iter().flatten().any(|u| !u.is_finite())
            || mobility
                .iter()
                .flatten()
                .flatten()
                .flatten()
                .any(|x| !x.is_finite())
        {
            return Err(PhysicsError::InvalidValue);
        }
        let ws = mobility
            .iter()
            .flatten()
            .flatten()
            .flatten()
            .map(|x| x.abs())
            .fold(0., f64::max);
        let vs = velocity
            .iter()
            .flatten()
            .map(|x| x.abs())
            .fold(0., f64::max);
        if ws == 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let w = mobility.map(|r| {
            r.map(|b| DMat3::from_cols_array_2d(&b.map(|row| row.map(|x| x / ws))).transpose())
        });
        let v = if vs > 0. {
            velocity.map(|u| DVec3::from(u) / vs)
        } else {
            [DVec3::ZERO; 2]
        };
        Ok(Self { w, v, ws, vs })
    }
}
/// The six-site-component mobility can have a genuine geometric null space.
/// Symmetric complete diagonal pivoting certifies its remaining Schur blocks.
pub(crate) fn positive_semidefinite(w: [[DMat3; 2]; 2]) -> bool {
    let mut a: [[f64; 6]; 6] =
        std::array::from_fn(|i| std::array::from_fn(|j| w[i / 3][j / 3].col(j % 3)[i % 3]));
    let eps = 256. * f64::EPSILON;
    for (i, row) in a.iter().enumerate() {
        for (j, previous) in a[..i].iter().enumerate() {
            if (row[j] - previous[i]).abs() > eps {
                return false;
            }
        }
    }
    for k in 0..6 {
        let p = (k..6)
            .max_by(|i, j| a[*i][*i].total_cmp(&a[*j][*j]))
            .unwrap();
        a.swap(p, k);
        for row in &mut a {
            row.swap(p, k);
        }
        let pivot = a[k][k];
        if pivot <= eps {
            return a[k..]
                .iter()
                .all(|row| row[k..].iter().all(|u| u.abs() <= eps));
        }
        for i in k + 1..6 {
            for j in k + 1..6 {
                a[i][j] -= a[i][k] * a[k][j] / pivot;
            }
        }
    }
    true
}
struct Reduced {
    normal: DVec2,
    tangent: [DVec2; 2],
    normal_from_tangent: [[DVec2; 2]; 2],
    tangent_from_tangent: [[DMat2; 2]; 2],
    friction: [f64; 2],
}
impl Reduced {
    fn normals(&self, t: [DVec2; 2]) -> DVec2 {
        DVec2::from_array(std::array::from_fn(|i| {
            self.normal[i]
                - self.normal_from_tangent[i][0].dot(t[0])
                - self.normal_from_tangent[i][1].dot(t[1])
        }))
    }
    fn speeds(&self, t: [DVec2; 2]) -> [DVec2; 2] {
        std::array::from_fn(|i| {
            self.tangent[i]
                + self.tangent_from_tangent[i][0] * t[0]
                + self.tangent_from_tangent[i][1] * t[1]
        })
    }
    /// Bound every possible cone impulse before assuming either tangent slides.
    fn bounds(&self) -> Result<(f64, f64, f64), PhysicsError> {
        let a = self
            .normal_from_tangent
            .map(|r| r[0].length() + r[1].length());
        let k = self
            .tangent_from_tangent
            .map(|r| op_norm(r[0]) + op_norm(r[1]));
        let beta = (self.friction[0] * a[0]).max(self.friction[1] * a[1]);
        if beta >= 1. - 128. * f64::EPSILON {
            return Err(decline());
        }
        let tmax = (self.friction[0] * self.normal.x.abs())
            .max(self.friction[1] * self.normal.y.abs())
            / (1. - beta);
        let mut contraction = 0f64;
        for i in 0..2 {
            let nrange = a[i] * tmax;
            let slipmin = self.tangent[i].length() - k[i] * tmax;
            if self.normal[i] - nrange <= 128. * f64::EPSILON * (self.normal[i].abs() + nrange)
                || slipmin <= 128. * f64::EPSILON * (1. + self.tangent[i].length())
            {
                return Err(decline());
            }
            // D(normalize(u)) has spectral norm 1/|u| throughout this convex ball.
            contraction = contraction
                .max(self.friction[i] * (a[i] + (self.normal[i] + nrange) * k[i] / slipmin));
        }
        if !contraction.is_finite() || contraction >= 1. - 128. * f64::EPSILON {
            return Err(decline());
        }
        Ok((contraction, tmax, 1f64.hypot(a[0].max(a[1]))))
    }
}
impl SlidingEdgeImpact {
    /// Certify both normals active and both tangents sliding, then solve.
    /// Unresolved stick, release or high-friction coupling returns Unsupported.
    pub fn solve(self) -> Result<SlidingEdgeResponse, PhysicsError> {
        let invalid = PhysicsError::InvalidValue;
        let ScaledEdge { w, v, ws, vs } =
            ScaledEdge::new(self.inverse_mass, self.relative_velocity, self.friction)?;
        if self
            .relative_velocity
            .iter()
            .any(|u| u[2] >= -128. * f64::EPSILON * vs)
        {
            return Err(decline());
        }
        if !positive_semidefinite(w) {
            return Err(invalid);
        }
        let nn = DMat2::from_cols_array_2d(&std::array::from_fn(|j| {
            [w[0][j].z_axis.z, w[1][j].z_axis.z]
        }));
        if nn.determinant() <= 256. * f64::EPSILON * op_norm(nn).powi(2) {
            return Err(decline());
        }
        let nt = w.map(|row| row.map(|b| DVec2::new(b.x_axis.z, b.y_axis.z)));
        for i in 0..2 {
            let other = 1 - i;
            let lo = nn.col(i)[i] - self.friction[i] * nt[i][i].length();
            if lo <= 128. * f64::EPSILON * nn.col(i)[i] {
                return Err(decline());
            }
            let hi = (nn.col(i)[other] + self.friction[i] * nt[other][i].length()).max(0.);
            // With only i loaded, even its maximum possible separating kick
            // leaves the other site closing. Thus both normals must be active.
            if v[other].z + hi * (-v[i].z) / lo >= -128. * f64::EPSILON {
                return Err(decline());
            }
        }
        let inv = nn.inverse();
        let n0 = inv * (-DVec2::new(v[0].z, v[1].z));
        let a: [[DVec2; 2]; 2] = std::array::from_fn(|i| {
            std::array::from_fn(|j| inv.x_axis[i] * nt[0][j] + inv.y_axis[i] * nt[1][j])
        });
        let tangent = std::array::from_fn(|i| {
            (v[i] + n0.x * w[i][0].z_axis + n0.y * w[i][1].z_axis).truncate()
        });
        let schur = std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                let cols = std::array::from_fn(|k| {
                    (w[i][j].col(k) - w[i][0].z_axis * a[0][j][k] - w[i][1].z_axis * a[1][j][k])
                        .truncate()
                        .to_array()
                });
                DMat2::from_cols_array_2d(&cols)
            })
        });
        let system = Reduced {
            normal: n0,
            tangent,
            normal_from_tangent: a,
            tangent_from_tangent: schur,
            friction: self.friction,
        };
        let (lipschitz, radius, lift) = system.bounds()?;
        let mut t = [DVec2::ZERO; 2];
        for _ in 0..256 {
            let normal = system.normals(t);
            let speed = system.speeds(t);
            let next =
                std::array::from_fn(|i| -self.friction[i] * normal[i] * speed[i].normalize());
            let residual = t[0].distance(next[0]).max(t[1].distance(next[1]));
            let error = lift * lipschitz * residual / (1. - lipschitz);
            t = next;
            if error > 32. * f64::EPSILON * (radius + n0.length()) {
                continue;
            }
            let n = system.normals(t);
            let j = [t[0].extend(n.x), t[1].extend(n.y)];
            let post: [DVec3; 2] = std::array::from_fn(|i| v[i] + w[i][0] * j[0] + w[i][1] * j[1]);
            let impulse = j.map(|v| {
                v.to_array()
                    .map(|x| crate::point_impact::mul_div(x, vs, ws))
            });
            let after = post.map(|v| (v * vs).to_array());
            let loss = -(0..2)
                .map(|i| {
                    DVec3::from(impulse[i]).dot(
                        DVec3::from(self.relative_velocity[i]) * 0.5 + DVec3::from(after[i]) * 0.5,
                    )
                })
                .sum::<f64>();
            let bound = crate::point_impact::mul_div(error, vs, ws);
            if !(loss >= 0. && loss.is_finite() && bound.is_finite())
                || impulse
                    .iter()
                    .chain(&after)
                    .flatten()
                    .any(|x| !x.is_finite())
                || impulse.iter().any(|j| j[2] <= 0.)
            {
                return Err(invalid);
            }
            return Ok(SlidingEdgeResponse {
                impulses: impulse,
                relative_velocity: after,
                contraction: lipschitz,
                iterate_error_bound: bound,
                dissipated_energy: loss,
            });
        }
        Err(decline())
    }
}

#[cfg(test)]
mod tests;
