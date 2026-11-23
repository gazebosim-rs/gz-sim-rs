// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Unilateral normal pressure of a rigid, frictionless rectangular face.
//! Normal velocity is affine across a planar face. Its nonnegativity at the
//! four corners enforces it everywhere, without prescribing a pressure shape.
use crate::{PhysicsError, point_impact::mul_div};
use glam::{DMat3, DVec3};

const DIAGONAL: [f64; 4] = [1., -1., -1., 1.];
const TOL: f64 = 1024. * f64::EPSILON;
#[derive(Clone, Copy, Debug)]
pub struct RectangularNormalContact {
    /// Measured corner Delassus matrix in order (−x,−y),(−x,+y),(+x,−y),(+x,+y).
    pub inverse_mass: [[f64; 4]; 4],
    /// Incoming contact-normal velocities for impact or unconstrained normal
    /// accelerations for a sustained static contact problem.
    pub free_normal: [f64; 4],
}
/// Every equivalent nonnegative corner allocation is base + shift·[1,−1,−1,1].
/// These are point-load representations of a resultant. The physical pressure
/// measure over a face may have arbitrary spatial distributions with these moments.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalCornerEquivalents {
    pub base: [f64; 4],
    pub shift_range: [f64; 2],
}
impl NormalCornerEquivalents {
    pub fn at(self, shift: f64) -> Option<[f64; 4]> {
        if !shift.is_finite() || shift < self.shift_range[0] || shift > self.shift_range[1] {
            return None;
        }
        Some(std::array::from_fn(|i| {
            DIAGONAL[i].mul_add(shift, self.base[i])
        }))
    }
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RectangularNormalResponse {
    pub resultant: f64,
    /// Center of pressure divided by the respective contact half extents.
    /// Coordinates range from −1 to +1. An unloaded footprint has no center.
    pub normalized_center: Option<[f64; 2]>,
    pub normal: [f64; 4],
    pub equivalents: NormalCornerEquivalents,
}
fn unsupported() -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "multifidelity",
        capability: "resolved rank-three rectangular normal pressure",
    }
}
fn subset(w: [[f64; 4]; 4], u: [f64; 4], mask: u8) -> Option<[f64; 4]> {
    let mut sites = [0; 3];
    let mut n = 0;
    for i in 0..4 {
        if mask & (1 << i) != 0 {
            if n == 3 {
                return None;
            }
            sites[n] = i;
            n += 1;
        }
    }
    let [a, b, c] = sites;
    let mut out = [0.; 4];
    match n {
        0 => {}
        1 => {
            out[a] = -u[a] / w[a][a];
        }
        2 => {
            let d = w[a][a].mul_add(w[b][b], -w[a][b] * w[a][b]);
            if d <= TOL {
                return None;
            }
            out[a] = w[a][b].mul_add(u[b], -w[b][b] * u[a]) / d;
            out[b] = w[a][b].mul_add(u[a], -w[a][a] * u[b]) / d;
        }
        3 => {
            let m = DMat3::from_cols_array_2d(&sites.map(|i| sites.map(|j| w[j][i])));
            if m.determinant() <= TOL {
                return None;
            }
            let values = -(m.inverse() * DVec3::new(u[a], u[b], u[c]));
            for (i, x) in sites.into_iter().zip(values.to_array()) {
                out[i] = x;
            }
        }
        _ => unreachable!(),
    }
    Some(out)
}
impl RectangularNormalContact {
    /// The single null mode transfers equal and opposite load along the two
    /// diagonals. The net force, both bending moments and outgoing velocities
    /// are unique. The returned interval preserves the unresolved pressure.
    pub fn solve(self) -> Result<RectangularNormalResponse, PhysicsError> {
        let m = self.inverse_mass;
        let free = self.free_normal;
        if m.iter().flatten().chain(&free).any(|x| !x.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        let mass_scale = m.iter().flatten().map(|v| v.abs()).fold(0., f64::max);
        if mass_scale == 0. {
            return Err(unsupported());
        }
        let w = m.map(|r| r.map(|v| v / mass_scale));
        let norm = |x: [f64; 4]| x[0] - x[1] - x[2] + x[3];
        for (i, row) in w.iter().enumerate() {
            if row[i] <= 0. || (0..i).any(|j| (row[j] - w[j][i]).abs() > TOL) {
                return Err(PhysicsError::InvalidValue);
            }
            if norm(*row).abs() > TOL {
                return Err(unsupported());
            }
        }
        let w = std::array::from_fn::<_, 4, _>(|i| {
            std::array::from_fn(|j| 0.5 * w[i][j] + 0.5 * w[j][i])
        });
        let sub = DMat3::from_cols_array_2d(&[0, 1, 2].map(|j| [0, 1, 2].map(|i| w[i][j])));
        let minor = w[0][0].mul_add(w[1][1], -w[0][1] * w[0][1]);
        if minor < -TOL || sub.determinant() < -TOL {
            return Err(PhysicsError::InvalidValue);
        }
        if minor <= TOL
            || sub.determinant() <= TOL
            || sub
                .inverse()
                .to_cols_array()
                .into_iter()
                .any(|u| u.abs() > 1e10)
        {
            return Err(unsupported());
        }
        let vs = free.into_iter().map(f64::abs).fold(0., f64::max);
        if vs == 0. {
            return Ok(RectangularNormalResponse {
                resultant: 0.,
                normalized_center: None,
                normal: [0.; 4],
                equivalents: NormalCornerEquivalents {
                    base: [0.; 4],
                    shift_range: [0.; 2],
                },
            });
        }
        let u = free.map(|x| x / vs);
        if norm(u).abs() > TOL {
            return Err(unsupported());
        }
        // At least one nonnegative equivalent puts zero reaction on a corner:
        // move along the diagonal null direction to either extreme. Consequently
        // an active system of order at most three always suffices.
        let mut best = None;
        for mask in 0..15 {
            let Some(base) = subset(w, u, mask) else {
                continue;
            };
            if base.into_iter().any(|x| !x.is_finite() || x < 0.) {
                continue;
            }
            let residual = std::array::from_fn::<_, 4, _>(|i| {
                w[i].into_iter()
                    .zip(base)
                    .fold(u[i], |v, (wij, j)| wij.mul_add(j, v))
            });
            let scale = 1. + base.iter().sum::<f64>();
            if residual.iter().any(|v| *v < -TOL * scale)
                || (0..4).any(|i| base[i] > 0. && residual[i].abs() > TOL * scale)
            {
                continue;
            }
            let violation = residual
                .into_iter()
                .map(|v| (-v).max(0.))
                .fold(0., f64::max);
            if best.as_ref().is_none_or(|(prev, _, _)| violation < *prev) {
                best = Some((violation, base, residual));
            }
        }
        let (_, base, v) = best.ok_or_else(unsupported)?;
        let total = base.iter().sum::<f64>();
        let mut range = [(-base[0]).max(-base[3]), base[1].min(base[2])];
        let tol = TOL * (1. + total);
        // A strictly separating corner must have exactly zero reaction in
        // every equivalent. With affine fields this occurs on a boundary COP.
        for (k, r) in v.into_iter().enumerate() {
            if r > tol {
                let only = -base[k] * DIAGONAL[k];
                if only < range[0] - tol || only > range[1] + tol {
                    return Err(unsupported());
                }
                range = [only.clamp(range[0], range[1]); 2];
            }
        }
        let center = (total > 0.).then(|| {
            [
                (base[2] + base[3] - base[0] - base[1]) / total,
                (base[1] + base[3] - base[0] - base[2]) / total,
            ]
        });
        let restore = |j: f64| mul_div(j, vs, mass_scale);
        let normal = v.map(|r| if r.abs() <= tol { 0. } else { r * vs });
        let set = NormalCornerEquivalents {
            base: base.map(restore),
            shift_range: range.map(restore),
        };
        let resultant = restore(total);
        if !resultant.is_finite()
            || (total > 0. && resultant == 0.)
            || set
                .base
                .iter()
                .chain(&set.shift_range)
                .chain(&normal)
                .any(|v| !v.is_finite())
            || base
                .into_iter()
                .zip(set.base)
                .any(|(j, p)| j > 0. && p == 0.)
        {
            return Err(unsupported());
        }
        Ok(RectangularNormalResponse {
            resultant,
            normalized_center: center,
            normal,
            equivalents: set,
        })
    }
}
#[cfg(test)]
mod tests;
