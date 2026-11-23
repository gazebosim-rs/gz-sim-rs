// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Particle, XpbdError, correct};
use glam::DVec3;
use std::f64::consts::{PI, TAU};

/// Signed angle between adjacent triangles (a,b,c) and (b,a,d).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(
    feature = "one-way-checkpoint",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct DihedralConstraint {
    /// Oriented shared edge [a,b] followed by opposite vertices [c,d].
    pub vertices: [usize; 4],
    pub rest_angle: f64,
    /// In radians per newton metre per hinge.
    pub compliance: f64,
}

impl DihedralConstraint {
    pub(super) fn from_rest(
        vertices: [usize; 4],
        particles: &[Particle],
        compliance: f64,
    ) -> Result<Self, XpbdError> {
        let p = vertices.map(|i| DVec3::from(particles[i].position));
        let (rest_angle, _) = geometry(p).ok_or(XpbdError::InvalidConstraint)?;
        Ok(Self {
            vertices,
            rest_angle,
            compliance,
        })
    }

    pub(super) fn solve(
        &self,
        x: &mut [DVec3],
        particles: &mut [Particle],
        lambda: &mut f64,
        h: f64,
        reference: Option<[DVec3; 4]>,
    ) -> Result<(), XpbdError> {
        let weights = self.vertices.map(|i| particles[i].inverse_mass);
        if weights == [0.0; 4] {
            return Ok(());
        }
        let (angle, gradient) =
            geometry(self.vertices.map(|i| x[i])).ok_or(XpbdError::Numerical)?;
        let gradient = if let Some(prior) = reference {
            // Orthogonal projection of the current row onto the start gradient
            // in the inverse-mass metric. The directional derivative equals
            // the squared norm of the projected gradient and stays nonnegative.
            let mut norm = 0.;
            let mut dot = 0.;
            for i in 0..4 {
                norm += weights[i] * prior[i].length_squared();
                dot += weights[i] * prior[i].dot(gradient[i]);
            }
            if norm == 0. || !norm.is_finite() || !dot.is_finite() {
                return Err(XpbdError::Numerical);
            }
            prior.map(|g| g * (dot / norm))
        } else {
            gradient
        };
        let alpha = self.compliance / (h * h);
        let w: f64 = weights
            .iter()
            .zip(gradient)
            .map(|(w, g)| w * g.length_squared())
            .sum();
        let denominator = w + alpha;
        if !denominator.is_finite() {
            return Err(XpbdError::Numerical);
        }
        if denominator == 0.0 {
            return Ok(());
        }
        let dl = (-angle_difference(angle, self.rest_angle) - alpha * *lambda) / denominator;
        *lambda += dl;
        for ((i, g), w) in self.vertices.into_iter().zip(gradient).zip(weights) {
            correct(&mut x[i], &mut particles[i], (dl * w) * g, h);
        }
        Ok(())
    }
}

pub(super) fn angle_difference(angle: f64, rest: f64) -> f64 {
    let delta = angle - rest;
    if delta > PI {
        delta - TAU
    } else if delta < -PI {
        delta + TAU
    } else {
        delta
    }
}

/// Angle about edge e and its exact four position gradients, obtained by
/// differentiating the two oriented unit face normals. Zero-area triangles
/// have no defined normal and cannot supply a physical bend direction.
pub(super) fn geometry([a, b, c, d]: [DVec3; 4]) -> Option<(f64, [DVec3; 4])> {
    let e = b - a;
    let rc = c - a;
    let rd = d - a;
    let t = e.try_normalize()?;
    let left = e.cross(rc);
    let right = rd.cross(e);
    let n1 = left.try_normalize()?;
    let n2 = right.try_normalize()?;
    let angle = t.dot(n1.cross(n2)).atan2(n1.dot(n2));
    let dn1 = -t.cross(n1) / left.length();
    let dn2 = t.cross(n2) / right.length();
    let gb = rc.cross(dn1) + dn2.cross(rd);
    let gc = dn1.cross(e);
    let gd = e.cross(dn2);
    let gradient = [-(gb + gc + gd), gb, gc, gd];
    gradient
        .iter()
        .all(|g| g.is_finite())
        .then_some((angle, gradient))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_angle_gradients_match_finite_differences_and_balance_torque() {
        let x = [
            DVec3::new(-0.3, 0.1, 0.22),
            DVec3::new(0.8, 0.5, -0.13),
            DVec3::new(0.12, -0.71, 0.49),
            DVec3::new(-0.17, 0.62, 0.77),
        ];
        let (_, g) = geometry(x).unwrap();
        let mut torque = DVec3::ZERO;
        for i in 0..4 {
            torque += x[i].cross(g[i]);
            for axis in 0..3 {
                let mut plus = x;
                let mut minus = x;
                let h = 1e-6;
                plus[i][axis] += h;
                minus[i][axis] -= h;
                let derivative =
                    angle_difference(geometry(plus).unwrap().0, geometry(minus).unwrap().0)
                        / (2.0 * h);
                assert!((derivative - g[i][axis]).abs() < 1e-9);
            }
        }
        assert!(g.into_iter().sum::<DVec3>().length() < 1e-14);
        assert!(torque.length() < 1e-14);
        assert!((angle_difference(PI - 0.1, -PI + 0.2) + 0.3).abs() < 1e-14);
        assert!((angle_difference(-PI + 0.2, PI - 0.1) - 0.3).abs() < 1e-14);
    }
}
