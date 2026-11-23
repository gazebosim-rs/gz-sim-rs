// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One point can lift off while both edge vertices initially approach the pad.
use crate::{
    EdgeMobility, PhysicsError, PointImpact,
    edge_impact::{ScaledEdge, positive_semidefinite},
    point_impact::mul_div,
};
use glam::{DVec2, DVec3};

/// Frozen two-site mobility and incoming velocities in one orthonormal frame.
/// Both candidate contacts are tangent, with a common inward third normal.
#[derive(Clone, Copy, Debug)]
pub struct OpeningEdgeImpact {
    pub inverse_mass: EdgeMobility,
    pub relative_velocity: [[f64; 3]; 2],
    pub friction: [f64; 2],
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OpeningEdgeResponse {
    pub loaded_site: usize,
    /// Only the loaded site delivers an impulse (N·s).
    pub impulse: [f64; 3],
    pub relative_velocity: [[f64; 3]; 2],
    pub sticking: bool,
    pub dissipated_energy: f64,
    /// Lower bounds (kg⁻¹) on the two projected normal kicks per unit normal
    /// impulse, across the entire admissible Coulomb cone at each site.
    pub separating_margin: [f64; 2],
}
fn unsupported() -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "multifidelity",
        capability: "certified single active site at closing two-vertex impact",
    }
}
impl OpeningEdgeImpact {
    /// For any proposed normal kicks k, both sites closing would require
    /// k=-v_n. Prove all admissible kicks on both friction cones lie on the
    /// same strict side of that ray. The opposite single-site response is
    /// excluded by the normal opening inequality on its unloaded site.
    pub fn solve(self) -> Result<OpeningEdgeResponse, PhysicsError> {
        let ScaledEdge { w, v, ws, vs } =
            ScaledEdge::new(self.inverse_mass, self.relative_velocity, self.friction)?;
        if !positive_semidefinite(w) {
            return Err(PhysicsError::InvalidValue);
        }
        let closing = DVec2::new(-v[0].z, -v[1].z);
        const EPS: f64 = 512. * f64::EPSILON;
        if closing.min_element() <= EPS {
            return Err(unsupported());
        }
        let dir = closing.normalize();
        // The normal kicks contributed by site i fill an ellipse per unit
        // normal impulse. Its support along the perpendicular to closing
        // gives both extrema exactly, even with anisotropic mobility.
        let axis =
            std::array::from_fn::<_, 2, _>(|i| dir.x * w[1][i].row(2) - dir.y * w[0][i].row(2));
        let lower = std::array::from_fn::<_, 2, _>(|i| {
            axis[i].z - self.friction[i] * axis[i].truncate().length()
        });
        let opposite = std::array::from_fn::<_, 2, _>(|i| {
            -axis[i].z - self.friction[i] * axis[i].truncate().length()
        });
        let (i, margin) = if lower.iter().all(|x| *x > EPS) {
            (0, lower)
        } else if opposite.iter().all(|x| *x > EPS) {
            (1, opposite)
        } else {
            return Err(unsupported());
        };
        let point = PointImpact {
            inverse_mass: w[i][i].transpose().to_cols_array_2d(),
            relative_velocity: v[i].to_array(),
            friction: self.friction[i],
        }
        .solve()?;
        let j = DVec3::from(point.impulse);
        let after = std::array::from_fn::<_, 2, _>(|k| v[k] + w[k][i] * j);
        if !(after[1 - i].z > EPS && j.z > EPS) || after[i].z.abs() > EPS * (1. + j.length()) {
            return Err(unsupported());
        }
        let impulse = point.impulse.map(|x| mul_div(x, vs, ws));
        let post = after.map(|u| (vs * u).to_array());
        let lost = -DVec3::from(impulse)
            .dot(DVec3::from(self.relative_velocity[i]) * 0.5 + DVec3::from(post[i]) * 0.5);
        let separating_margin = margin.map(|m| m * ws);
        if !lost.is_finite()
            || lost < 0.
            || impulse
                .iter()
                .chain(post.iter().flatten())
                .chain(&separating_margin)
                .any(|x| !x.is_finite())
            || impulse[2] <= 0.
            || post[1 - i][2] <= 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(OpeningEdgeResponse {
            loaded_site: i,
            impulse,
            relative_velocity: post,
            sticking: point.sticking,
            dissipated_energy: lost,
            separating_margin,
        })
    }
}

#[cfg(test)]
mod tests;
