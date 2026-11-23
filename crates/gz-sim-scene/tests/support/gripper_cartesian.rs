// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::fixture::*;
use glam::{DQuat, DVec3};
use gz_physics::BodyState;
use rapier3d_f64::na::DMatrix;

/// Independent maximal-coordinate oracle. The I/O consists of measured body
/// poses and authored COM, inertia, joint anchors and axes. Five bilateral
/// rows per revolute remove relative anchor velocity and off-axis spin.
pub fn cartesian(
    s: [BodyState; 3],
    center: [[f64; 3]; 3],
    grounded: bool,
    points: [[f64; 3]; 3],
) -> DMatrix<f64> {
    let mut inverse = DMatrix::<f64>::zeros(18, 18);
    let mut j = DMatrix::<f64>::zeros(9, 18);
    let mut g = DMatrix::<f64>::zeros(if grounded { 15 } else { 10 }, 18);
    let axes = [DVec3::X, DVec3::Y, DVec3::Z];
    for (i, state) in s.into_iter().enumerate() {
        let point = pose(state, points[i]);
        let arm = point - pose(state, center[i]);
        for (a, d) in axes.into_iter().enumerate() {
            inverse[(6 * i + a, 6 * i + a)] = 1. / MASS[i];
            let w = angular_mass(i, state, d, true);
            let twist = d.cross(arm);
            for b in 0..3 {
                inverse[(6 * i + 3 + b, 6 * i + 3 + a)] = w[b];
                j[(3 * i + b, 6 * i + 3 + a)] = twist[b];
            }
            j[(3 * i + a, 6 * i + a)] = 1.;
        }
    }
    let base = s[0];
    let axis = DQuat::from_array(base.orientation) * DVec3::Z;
    let u = axis.any_orthonormal_vector();
    let v = axis.cross(u);
    for (k, y) in [0.29, -0.29].into_iter().enumerate() {
        let child = k + 1;
        let a = pose(base, [0.04, y, 0.]);
        for (i, sign) in [(0, -1.), (child, 1.)] {
            let arm = a - pose(s[i], center[i]);
            for (q, d) in axes.into_iter().enumerate() {
                g[(5 * k + q, 6 * i + q)] = sign;
                let turn = sign * d.cross(arm);
                for b in 0..3 {
                    g[(5 * k + b, 6 * i + 3 + q)] = turn[b];
                }
                g[(5 * k + 3, 6 * i + 3 + q)] = sign * u[q];
                g[(5 * k + 4, 6 * i + 3 + q)] = sign * v[q];
            }
        }
    }
    if grounded {
        let arm = pose(base, [0.; 3]) - pose(base, center[0]);
        for (q, d) in axes.into_iter().enumerate() {
            g[(10 + q, q)] = 1.;
            let turn = d.cross(arm);
            for b in 0..3 {
                g[(10 + b, 3 + q)] = turn[b];
            }
            g[(13, 3 + q)] = u[q];
            g[(14, 3 + q)] = v[q];
        }
    }
    let raw = &inverse * j.transpose();
    let rows = (&g * &inverse * g.transpose()).cholesky().unwrap();
    &j * (&raw - &inverse * g.transpose() * rows.solve(&(&g * &raw)))
}
