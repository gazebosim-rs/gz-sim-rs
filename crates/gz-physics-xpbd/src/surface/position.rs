// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Clearance correction at fixed velocities for an already penetrated closed shell.
use super::{Hit, Particle, XpbdError};
use glam::{DMat3, DVec3};
use std::collections::BTreeSet;

/// Project the ordinary mass-weighted displacement onto motions preserving
/// the local COM and angular momentum. A rigid infinitesimal rotation in
/// velocity coordinates carries the removed angular component.
pub(super) fn correct(
    hit: &Hit,
    supports: [&[usize]; 2],
    x: &mut [DVec3],
    p: &[Particle],
) -> Result<(), XpbdError> {
    if hit.depth == 0. {
        return Ok(());
    }
    let nodes: BTreeSet<_> = supports.into_iter().flatten().copied().collect();
    let mut mass = 0.;
    let mut mean = DVec3::ZERO;
    for &i in &nodes {
        let m = 1. / p[i].inverse_mass;
        mass += m;
        mean += m * DVec3::from(p[i].velocity);
    }
    mean /= mass;
    let mut inertia = DMat3::ZERO;
    let mut torque = DVec3::ZERO;
    let mut rows = Vec::with_capacity(nodes.len());
    let mut sum = 0.;
    for i in nodes {
        let w = p[i].inverse_mass;
        let velocity = DVec3::from(p[i].velocity) - mean;
        let u = velocity / w.sqrt();
        let coefficient = hit
            .stencil
            .ids
            .iter()
            .position(|&id| id == i)
            .map_or(0., |k| hit.weights[k]);
        let g = coefficient * w * hit.normal;
        torque += coefficient * hit.normal.cross(velocity);
        sum += u.length_squared();
        inertia +=
            DMat3::IDENTITY * u.length_squared() - DMat3::from_cols(u * u.x, u * u.y, u * u.z);
        rows.push((i, coefficient, velocity, g));
    }
    if !sum.is_finite() {
        return Err(XpbdError::Numerical);
    }
    let omega = if sum == 0. {
        DVec3::ZERO
    } else {
        let scaled = inertia / sum;
        if scaled.determinant() > 1e-12 {
            scaled.inverse() * (torque / sum)
        } else {
            // For collinear velocities the two transverse eigenvalues are
            // both sum; torque always lies in their span. Verify the residual.
            let candidate = torque / sum;
            if (inertia * candidate - torque).length() > 1e-10 * torque.length() {
                return Err(crate::observed_refusal());
            }
            candidate
        }
    };
    let mut response = 0.;
    let mut max_motion = 0.0_f64;
    for (_, coefficient, v, g) in &mut rows {
        *g += omega.cross(*v);
        response += *coefficient * hit.normal.dot(*g);
        max_motion = max_motion.max(g.length());
    }
    if !response.is_finite() || response <= 0. || max_motion > 8. * response {
        return Err(crate::observed_refusal());
    }
    for (i, _, _, g) in rows {
        x[i] += g * (hit.depth / response);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::surface::Stencil;
    #[test]
    fn immobile_seed_can_clear_but_a_pure_angular_velocity_space_mode_cannot() {
        let hit = Hit {
            endpoint_velocity: false,
            stencil: Stencil {
                ids: [0, 1, 2, 3],
                vf: true,
            },
            weights: [1., -1., 0., 0.],
            normal: DVec3::Z,
            time: 0.,
            depth: 0.001,
            speed: 0.,
        };
        let mut p = [Particle {
            position: [0.; 3],
            velocity: [0.; 3],
            inverse_mass: 1.,
        }; 6];
        let supports: [&[usize]; 2] = [&[0, 4, 5], &[1, 2, 3]];
        let mut x = [DVec3::ZERO; 6];
        correct(&hit, supports, &mut x, &p).unwrap();
        assert!((x[0].z - x[1].z - hit.depth).abs() < 1e-15);
        assert_eq!(x.iter().copied().sum::<DVec3>(), DVec3::ZERO);
        // No displacement can separate the two tips in z without changing
        // total angular momentum: only those opposing tips carry x momentum.
        p[0].velocity = DVec3::X.to_array();
        p[1].velocity = (-DVec3::X).to_array();
        let saved = x;
        assert!(matches!(
            correct(&hit, supports, &mut x, &p),
            Err(crate::observed_refusal())
        ));
        assert_eq!(x, saved);
    }
}
