// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// Manufacture the incoming twist from an independent admissible distributed
/// impulse and its outgoing rigid motion. The output normally slides, or has
/// one sticking point with an admissible longitudinal/transverse friction load.
#[test]
fn known_distributed_solutions_survive_uncertain_pressure_and_inertia_scales() {
    let mut n = 0;
    let mut searched = 0;
    for mass_scale in [1e-5, 1., 1e5] {
        for (_, frac) in [(0, 0.), (1, 0.25), (2, 0.68), (3, 1.)] {
            for (mu, height) in [(0.15, 0.03), (0.5, 0.15), (1.25, 0.41)] {
                for shift in [-0.19, 0., 0.14] {
                    let p = DVec3::new(shift, 0.061, -height);
                    let q = p + 0.34 * DVec3::X;
                    for ix in 0..6 {
                        let attitude = DMat3::from_quat(glam::DQuat::from_euler(
                            glam::EulerRot::ZYX,
                            ix as f64 * 0.33,
                            0.63,
                            0.26,
                        ));
                        let inv = (attitude
                            * DMat3::from_diagonal(DVec3::new(13., 28., 18.))
                            * attitude.transpose())
                            * (1. / mass_scale);
                        let mass = 2. * mass_scale;
                        let basis = [DVec3::X, DVec3::Y, DVec3::Z];
                        let arms = [p, q];
                        let matrix = std::array::from_fn(|i| {
                            std::array::from_fn(|j| {
                                std::array::from_fn(|a| {
                                    std::array::from_fn(|b| {
                                        basis[a].dot(
                                            basis[b] / mass
                                                + (inv * arms[j].cross(basis[b])).cross(arms[i]),
                                        )
                                    })
                                })
                            })
                        });
                        for along in [0., 1e-8, 0.34] {
                            let spin = DVec3::new(0.4, 0., -1.8 + ix as f64 * 0.24);
                            let v = along * DVec3::X - spin.cross(p.lerp(q, frac));
                            let mut impulse = DVec3::ZERO;
                            let mut torque = DVec3::ZERO;
                            for (at, fraction) in
                                [(0.0, 0.125), (frac, 0.56), (0.84, 0.21), (1., 0.105)]
                            {
                                let arm = p.lerp(q, at);
                                let slip = v + spin.cross(arm);
                                assert!(slip.z.abs() < 1e-14);
                                let direction = if slip.length() > 1e-15 {
                                    -mu * slip.normalize()
                                } else {
                                    mu * DVec3::new(0.31, -0.29, 0.)
                                };
                                let j = mass_scale * fraction * (DVec3::Z + direction);
                                impulse += j;
                                torque += arm.cross(j);
                            }
                            let before_v = v - impulse / mass;
                            let before_w = spin - inv * torque;
                            let before = arms.map(|a| (before_v + before_w.cross(a)).to_array());
                            let result = exclude_closed_line(matrix, before, mu, 128).unwrap();
                            assert_eq!(
                                result.status,
                                LineClosure::Undetermined,
                                "feasible pressure rejected at {n}"
                            );
                            searched += usize::from(result.visited > 0);
                            n += 1;
                        }
                    }
                }
            }
        }
    }
    assert_eq!(n, 1944);
    assert!(searched > 1800);
}
