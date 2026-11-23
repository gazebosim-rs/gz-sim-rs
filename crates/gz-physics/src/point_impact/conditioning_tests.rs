// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::{DMat3, DVec3};

fn input(w: DMat3, u: DVec3, mu: f64) -> PointImpact {
    PointImpact {
        inverse_mass: w.transpose().to_cols_array_2d(),
        relative_velocity: u.to_array(),
        friction: mu,
    }
}
#[test]
fn sliding_reaction_remains_resolved_for_negligible_or_extreme_friction() {
    let w = DMat3::from_cols(DVec3::new(1., 0., -0.3), DVec3::Y, DVec3::new(-0.3, 0., 1.));
    for mu in [0.0f64, 1e-250, 1e-100, 1e-22, 1e-12, 0.1, 1e8, 1e160, 1e300] {
        // Prescribe an actual sliding solution and derive its incoming twist.
        let normal = 1. / mu.max(1.);
        let expected = DVec3::new(-mu * normal, 0., normal);
        let post = DVec3::new(2., 0., 0.);
        let r = input(w, post - w * expected, mu).solve().unwrap();
        assert!(!r.sticking);
        assert!((r.impulse[0] - expected.x).abs() <= 1e-12 * expected.x.abs());
        assert!((r.impulse[2] - expected.z).abs() <= 1e-12 * expected.z);
        assert!(post.distance(r.relative_velocity.into()) < 1e-12);
    }
}
#[test]
fn physical_solution_is_covariant_across_tangent_frames_velocity_and_mass_units() {
    let w = DMat3::from_cols(
        DVec3::new(1.1, 0.27, 0.2),
        DVec3::new(0.27, 2.5, -0.7),
        DVec3::new(0.2, -0.7, 1.3),
    );
    let base = DVec3::new(-0.49, 1.32, -0.37);
    for mu in [0., 1e-15, 0.03, 0.5, 1.7, 5.] {
        let reference = input(w, base, mu).solve().unwrap();
        for deg in [0., 0.000001, 17., 89.999999, 90., 153., 223.5] {
            for mirror in [-1., 1.] {
                let rot = DMat3::from_rotation_z(f64::to_radians(deg))
                    * DMat3::from_diagonal(DVec3::new(1., mirror, 1.));
                for (mass, vel) in [
                    (1e-80, 1e-40),
                    (1e90, 1e-100),
                    (0.125, 100.),
                    (1., 1.),
                    (1e65, 1e50),
                ] {
                    let r = input((rot * w * rot.transpose()) / mass, (rot * base) * vel, mu)
                        .solve()
                        .unwrap();
                    assert_eq!(r.sticking, reference.sticking);
                    let expected = rot * DVec3::from(reference.impulse);
                    assert!(expected.distance(DVec3::from(r.impulse) / (mass * vel)) < 2e-12);
                    let post = rot * DVec3::from(reference.relative_velocity);
                    assert!(post.distance(DVec3::from(r.relative_velocity) / vel) < 3e-12);
                    assert!(
                        (r.dissipated_energy / (mass * vel * vel) - reference.dissipated_energy)
                            .abs()
                            < 3e-12
                    );
                }
            }
        }
    }
}
#[test]
fn near_sticking_continuity_and_exactly_zero_tangent_are_well_defined() {
    let threshold = 0.5;
    for delta in [-1e-9, -1e-12, 0., 1e-12, 1e-9] {
        let mu = threshold + delta;
        let r = input(DMat3::IDENTITY, DVec3::new(0.3, 0.4, -1.), mu)
            .solve()
            .unwrap();
        assert_eq!(r.sticking, delta >= 0.);
        assert!(
            DVec3::from(r.impulse).distance(DVec3::new(-0.6 * mu.min(0.5), -0.8 * mu.min(0.5), 1.))
                < 1e-13
        );
    }
    for mu in [0., 0.5, 1e300] {
        let r = input(DMat3::IDENTITY, -DVec3::Z, mu).solve().unwrap();
        assert!(r.sticking);
        assert_eq!(r.impulse, [0., 0., 1.]);
    }
}
#[test]
fn additional_sliding_or_unresolved_tangent_roots_decline_even_if_sticking_is_admissible() {
    // Two distinct sliding impulses also satisfy the law on this SPD matrix.
    let matrix = DMat3::from_cols(
        DVec3::new(10.25, 0., -6.),
        DVec3::new(0., 19.25, -14.),
        DVec3::new(-6., -14., 14.25),
    );
    let u = DVec3::new(6., -5., -1.);
    let stick = matrix.inverse() * -u;
    assert!(stick.truncate().length() < stick.z);
    for j in [
        DVec3::new(-0.1144918887045726, 0.7691207810932295, 0.7775957616194931),
        DVec3::new(
            -0.08579558128882342,
            0.09764283889436155,
            0.12998078995004358,
        ),
    ] {
        let after = u + matrix * j;
        assert!(after.z.abs() < 1e-13 && j.z > 0.);
        assert!((j.truncate() + j.z * after.truncate().normalize()).length() < 1e-13);
    }
    assert!(matches!(
        input(matrix, u, 1.).solve(),
        Err(PhysicsError::Unsupported { .. })
    ));
    // A double root in the +x chart has positive final slip. The first
    // derivative of the perpendicular slip constraint vanishes at angle 0.
    let matrix = DMat3::from_cols(
        DVec3::new(10.25, 0., 6.),
        DVec3::new(0., 19.25, 14.),
        DVec3::new(6., 14., 14.25),
    );
    let j = DVec3::new(-1., 0., 1.);
    let post = DVec3::new(196. / 8.25 - 19.25, 0., 0.);
    assert!(matches!(
        input(matrix, post - matrix * j, 1.).solve(),
        Err(PhysicsError::Unsupported { .. })
    ));
    assert!(
        input(DMat3::from_diagonal(DVec3::new(1e-18, 1., 2.)), u, 0.5)
            .solve()
            .is_err()
    );
}
