// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn model() -> ([[DMat3; 2]; 2], [DVec3; 2], [DVec3; 2], [f64; 2]) {
    let a = DMat3::from_cols(
        DVec3::new(1.9, 0.3, -0.1),
        DVec3::new(0.3, 1.1, 0.2),
        DVec3::new(-0.1, 0.2, 1.4),
    );
    let b = DMat3::from_cols(
        DVec3::new(0.2, -0.1, 0.08),
        DVec3::new(0.05, 0.3, 0.1),
        DVec3::new(0.06, -0.2, 0.15),
    );
    let w = [[a, b], [b.transpose(), a * 1.3]];
    let post = [DVec3::new(3.2, 1.1, 0.), DVec3::new(-0.7, 2.8, 0.)];
    let mu = [0.12, 0.25];
    let n = [0.45, 0.8];
    let j = std::array::from_fn(|i| (-mu[i] * n[i] * post[i].truncate().normalize()).extend(n[i]));
    let u = std::array::from_fn(|i| post[i] - w[i][0] * j[0] - w[i][1] * j[1]);
    (w, u, j, mu)
}
fn make(w: [[DMat3; 2]; 2], v: [DVec3; 2], mu: [f64; 2]) -> SlidingEdgeImpact {
    SlidingEdgeImpact {
        inverse_mass: w.map(|r| r.map(|v| v.transpose().to_cols_array_2d())),
        relative_velocity: v.map(|v| v.to_array()),
        friction: mu,
    }
}
#[test]
fn different_site_materials_and_nonparallel_slips_transform_with_the_contact_frame() {
    let (w, u, j, mu) = model();
    for theta in [0., 0.1, 1.4, 3.13] {
        for sign in [1., -1.] {
            for mass in [1e-60, 1., 1e60] {
                for perm in [[0, 1], [1, 0]] {
                    let rot = DMat3::from_rotation_z(theta)
                        * DMat3::from_diagonal(DVec3::new(sign, 1., 1.));
                    let m = perm.map(|i| perm.map(|k| rot * w[i][k] * rot.transpose() / mass));
                    let result = make(m, perm.map(|i| rot * u[i]), perm.map(|i| mu[i]))
                        .solve()
                        .unwrap();
                    let mut loss = 0.;
                    for (k, i) in perm.into_iter().enumerate() {
                        let got = DVec3::from(result.impulses[k]) / mass;
                        let va = DVec3::from(result.relative_velocity[k]);
                        assert!(got.distance(rot * j[i]) < 1e-13);
                        assert!(
                            va.z.abs() < 1e-14
                                && (got.truncate() + mu[i] * got.z * va.truncate().normalize())
                                    .length()
                                    < 2e-14
                        );
                        loss -= 0.5 * got.dot(rot * u[i] + va);
                    }
                    assert!((loss - result.dissipated_energy / mass).abs() < 1e-13);
                    assert!(result.contraction < 1. && result.iterate_error_bound < 1e-12 * mass);
                }
            }
        }
    }
}
#[test]
fn requires_a_valid_mobility_and_certified_simultaneous_sliding() {
    let (w, u, _, mu) = model();
    let mut nonsymmetric = w;
    nonsymmetric[0][1].x_axis.z += 0.1;
    assert!(matches!(
        make(nonsymmetric, u, mu).solve(),
        Err(PhysicsError::InvalidValue)
    ));
    let mut indefinite = w;
    indefinite[0][0].z_axis.z = -1.;
    assert!(matches!(
        make(indefinite, u, mu).solve(),
        Err(PhysicsError::InvalidValue)
    ));
    let d = DMat3::IDENTITY;
    let dependent = [[d, d], [d, d]];
    assert!(make(dependent, u, mu).solve().is_err());
    let uncoupled = [[d, DMat3::ZERO], [DMat3::ZERO, d]];
    assert!(make(uncoupled, [-DVec3::Z; 2], mu).solve().is_err());
    assert!(
        make(w, [u[0], DVec3::new(0., 0., 0.01)], mu)
            .solve()
            .is_err()
    );
    assert!(make(w, u, [f64::NAN, 0.]).solve().is_err());
    let open = [[d, d * 0.6], [d * 0.6, d]];
    assert!(
        make(
            open,
            [DVec3::new(2., 1., -0.1), DVec3::new(1., 2., -1.)],
            mu
        )
        .solve()
        .is_err()
    );
}
