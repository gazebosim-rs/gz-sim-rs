// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DMat3;
#[test]
fn unique_active_site_is_frame_covariant_and_delivers_no_impulse_to_opening_site() {
    let r = [DVec3::new(0.15, -0.1, -0.16), DVec3::new(0.15, 0.1, -0.16)];
    let basis = [DVec3::X, DVec3::Y, DVec3::Z];
    let w = r.map(|ri| {
        r.map(|rk| {
            DMat3::from_cols_array_2d(
                &basis.map(|b| (0.5 * b + (20. * rk.cross(b)).cross(ri)).to_array()),
            )
        })
    });
    for stick in [true, false] {
        for mu in [0.08, 0.2, 0.35] {
            let slip = if stick {
                DVec3::ZERO
            } else {
                DVec3::new(1.2, -0.6, 0.)
            };
            let tangent = if stick {
                mu * 0.8 * DVec3::new(0.2, 0.3, 0.)
            } else {
                -0.8 * mu * slip.normalize()
            };
            let j = DVec3::Z * 0.8 + tangent;
            let post = [slip, slip + DVec3::new(0.05, 0., 0.3)];
            let pre = std::array::from_fn::<_, 2, _>(|i| post[i] - w[i][0] * j);
            for mass in [1e-60, 1., 1e60] {
                for perm in [[0, 1], [1, 0]] {
                    for theta in [0., 0.31, 2.7] {
                        let rot = DMat3::from_rotation_z(theta)
                            * DMat3::from_diagonal(DVec3::new(-1., 1., 1.));
                        let q = OpeningEdgeImpact {
                            inverse_mass: perm.map(|a| {
                                perm.map(|b| {
                                    (rot * w[a][b] * rot.transpose() / mass)
                                        .transpose()
                                        .to_cols_array_2d()
                                })
                            }),
                            relative_velocity: perm.map(|a| (rot * pre[a]).to_array()),
                            friction: [mu; 2],
                        };
                        let h = q.solve().unwrap();
                        let i = h.loaded_site;
                        assert_eq!(perm[i], 0);
                        assert_eq!(h.sticking, stick);
                        assert!(h.separating_margin.into_iter().all(|v| v > 0.));
                        assert!(DVec3::from(h.impulse).distance(mass * (rot * j)) / mass < 1e-13);
                        for (idx, k) in perm.into_iter().enumerate() {
                            assert!(
                                DVec3::from(h.relative_velocity[idx]).distance(rot * post[k])
                                    < 1e-13
                            );
                        }
                        let loss = -0.5 * j.dot(pre[0] + post[0]);
                        assert!((h.dissipated_energy / mass - loss).abs() < 1e-13);
                    }
                }
            }
        }
    }
}
#[test]
fn shared_active_or_unresolved_cones_decline_and_invalid_data_is_rejected() {
    let d = DMat3::IDENTITY;
    let w = [[d, d * 0.1], [d * 0.1, d]];
    let mut q = OpeningEdgeImpact {
        inverse_mass: w.map(|r| r.map(|m| m.to_cols_array_2d())),
        relative_velocity: [[0., 0., -1.], [0., 0., -2.]],
        friction: [0.2, 0.2],
    };
    assert!(matches!(q.solve(), Err(PhysicsError::Unsupported { .. })));
    q.relative_velocity[1][2] = 0.;
    assert!(q.solve().is_err());
    q.friction[0] = f64::NAN;
    assert!(matches!(q.solve(), Err(PhysicsError::InvalidValue)));
}
