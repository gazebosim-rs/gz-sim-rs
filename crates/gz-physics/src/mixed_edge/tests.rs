// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn fixture(mode: EdgeStickMode) -> (MixedEdgeImpact, [DVec3; 2], [DVec3; 2]) {
    let r = [
        DVec3::new(-0.18, -0.07, -0.14),
        DVec3::new(0.18, -0.07, -0.14),
    ];
    let axes = [DVec3::X, DVec3::Y, DVec3::Z];
    let w = r.map(|target| {
        r.map(|source| {
            DMat3::from_cols_array_2d(
                &axes.map(|j| (0.5 * j + (source.cross(j) * 20.).cross(target)).to_array()),
            )
        })
    });
    let n = [0.48, 0.72];
    let mu = [0.15, 0.2];
    let mut j = [
        DVec3::new(0.023, -0.02, n[0]),
        DVec3::new(-0.012, 0.024, n[1]),
    ];
    let mut post = [DVec3::ZERO; 2];
    match mode {
        EdgeStickMode::FirstSticks => {
            post[1] = 0.45 * DVec3::Y;
            j[1] = DVec3::new(0., -mu[1] * n[1], n[1]);
        }
        EdgeStickMode::SecondSticks => {
            post[0] = -0.45 * DVec3::Y;
            j[0] = DVec3::new(0., mu[0] * n[0], n[0]);
        }
        EdgeStickMode::BothStick => (),
    }
    let pre = std::array::from_fn(|i| (post[i] - w[i][0] * j[0] - w[i][1] * j[1]).to_array());
    (
        MixedEdgeImpact {
            inverse_mass: w.map(|row| row.map(|m| m.transpose().to_cols_array_2d())),
            relative_velocity: pre,
            friction: mu,
        },
        j,
        post,
    )
}
#[test]
fn momentum_energy_and_observable_set_survive_scaling_and_site_reversal() {
    for mode in [
        EdgeStickMode::BothStick,
        EdgeStickMode::FirstSticks,
        EdgeStickMode::SecondSticks,
    ] {
        let (q, j, post) = fixture(mode);
        for perm in [[0, 1], [1, 0]] {
            let sign = if perm[0] == 0 { 1. } else { -1. };
            let v = DVec3::new(sign, sign, 1.);
            for mass in [1e-60, 1., 1e60] {
                let test = MixedEdgeImpact {
                    inverse_mass: perm.map(|i| {
                        perm.map(|k| {
                            std::array::from_fn(|a| {
                                std::array::from_fn(|b| {
                                    v[a] * q.inverse_mass[i][k][a][b] * v[b] / mass
                                })
                            })
                        })
                    }),
                    relative_velocity: perm
                        .map(|i| (v * DVec3::from(q.relative_velocity[i])).to_array()),
                    friction: perm.map(|i| q.friction[i]),
                };
                let res = test.solve().unwrap();
                let actual_mode = match (mode, perm[0]) {
                    (EdgeStickMode::FirstSticks, 1) => EdgeStickMode::SecondSticks,
                    (EdgeStickMode::SecondSticks, 1) => EdgeStickMode::FirstSticks,
                    _ => mode,
                };
                assert_eq!(res.mode, actual_mode);
                let [lo, hi] = res.impulses.first_longitudinal_range;
                assert!(res.uniqueness_bound < 1.);
                assert!(res.impulses.split(f64::NAN).is_none());
                if mode == EdgeStickMode::BothStick {
                    assert!(lo / mass < v.x * j[perm[0]].x && hi / mass > v.x * j[perm[0]].x);
                    assert!((hi - lo) / mass > 0.05);
                } else {
                    assert_eq!(lo, hi);
                    assert!((lo / mass - v.x * j[perm[0]].x).abs() < 1e-14);
                }
                for s in [lo, hi] {
                    let split = res.impulses.split(s).unwrap().map(DVec3::from);
                    let mut work = 0.;
                    for (i, k) in perm.into_iter().enumerate() {
                        let pred = (0..2).fold(DVec3::from(test.relative_velocity[i]), |a, l| {
                            a + DMat3::from_cols_array_2d(&test.inverse_mass[i][l]).transpose()
                                * split[l]
                        });
                        assert!(pred.distance(v * post[k]) < 8e-15);
                        assert!(DVec3::from(res.relative_velocity[i]).distance(pred) < 8e-15);
                        let impulse = split[i] / mass;
                        assert!(impulse.truncate().length() <= q.friction[k] * impulse.z + 1e-15);
                        assert!((impulse.z - j[k].z).abs() < 1e-14);
                        work += 0.5 * impulse.dot(DVec3::from(test.relative_velocity[i]) + pred);
                    }
                    assert!((work + res.dissipated_energy / mass).abs() < 1e-14);
                }
            }
        }
    }
}
#[test]
fn declines_opening_incompatible_longitudinal_and_uncertified_friction() {
    let (base, _, _) = fixture(EdgeStickMode::BothStick);
    let mut q = base;
    q.relative_velocity[0][0] += 1e-4;
    assert!(matches!(q.solve(), Err(PhysicsError::Unsupported { .. })));
    q = base;
    q.relative_velocity[1][2] = 0.;
    assert!(q.solve().is_err());
    q = base;
    q.friction = [10., 10.];
    assert!(q.solve().is_err());
    q = base;
    q.friction[0] = f64::NAN;
    assert!(matches!(q.solve(), Err(PhysicsError::InvalidValue)));
    q = base;
    q.inverse_mass[0][1][1][0] += 0.001;
    assert!(matches!(q.solve(), Err(PhysicsError::InvalidValue)));
}
