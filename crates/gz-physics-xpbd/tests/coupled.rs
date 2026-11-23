// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, Particle, SoftBody, SolverConfig, SphereMobility, XpbdError, step_coupled,
};
use std::time::Duration;

fn projectile(x: DVec3, velocity: DVec3, mass: f64, substeps: u32) -> SoftBody {
    SoftBody::new(
        vec![Particle {
            position: x.to_array(),
            velocity: velocity.to_array(),
            inverse_mass: 1. / mass,
        }],
        vec![],
        SolverConfig {
            gravity: [0.; 3],
            substeps,
            iterations: 4,
            particle_radius: 0.01,
            friction: 0.4,
            self_contact: false,
        },
    )
    .unwrap()
}
fn sphere(center: DVec3, velocity: DVec3, angular: DVec3) -> Boundary {
    Boundary::Sphere {
        center: center.to_array(),
        velocity: velocity.to_array(),
        angular_velocity: angular.to_array(),
        radius: 0.25,
    }
}

#[test]
fn one_dimensional_inelastic_impact_matches_exact_solution_for_three_mass_ratios() {
    for rigid_mass in [0.01, 0.1, 10.0] {
        for substeps in [1, 2, 4] {
            let mass = 0.1;
            let mut body = projectile(DVec3::new(-0.261, 0., 0.), DVec3::X * 2., mass, substeps);
            let dt = Duration::from_millis(1);
            let state = step_coupled(
                &mut [&mut body],
                dt,
                &[sphere(DVec3::ZERO, DVec3::ZERO, DVec3::ZERO)],
                &[Some(SphereMobility {
                    inverse_mass: 1. / rigid_mass,
                    inverse_inertia: 1. / (0.1 * rigid_mass),
                })],
            )
            .unwrap()[0]
                .unwrap();
            let v = 2. * mass / (rigid_mass + mass);
            assert!((body.particles()[0].velocity[0] - v).abs() < 1e-12);
            assert!((state.linear_velocity_correction[0] - v).abs() < 1e-12);
            assert!((state.translation_correction[0] - 0.0005 * v).abs() < 1e-12);
            let gap = state.translation_correction[0] - body.particles()[0].position[0];
            assert!((gap - 0.26).abs() < 1e-12);
        }
    }
}

#[test]
fn multiple_soft_bodies_share_the_same_receiver_momentum_and_spin() {
    let dt = Duration::from_millis(1);
    let h = dt.as_secs_f64();
    let rigid_mass = 0.02;
    let inertia = 0.001;
    let mobility = [Some(SphereMobility {
        inverse_mass: 1. / rigid_mass,
        inverse_inertia: 1. / inertia,
    })];
    let mut a = projectile(DVec3::new(-0.27, 0.045, 0.), DVec3::X * 2., 0.05, 4);
    let mut b = projectile(DVec3::new(-0.27, 0.10, 0.), DVec3::X * 2., 0.05, 4);
    let (mut x, mut v, mut w) = (DVec3::ZERO, DVec3::ZERO, DVec3::ZERO);
    let mut orientation = DQuat::IDENTITY;
    let initial_p = DVec3::X * 0.2;
    let initial_l = DVec3::Z * (-0.0145);
    let mut max_l: f64 = 0.0;
    for _ in 0..120 {
        let motion = step_coupled(&mut [&mut a, &mut b], dt, &[sphere(x, v, w)], &mobility)
            .unwrap()[0]
            .unwrap();
        orientation = DQuat::from_array(motion.rotation_correction)
            * DQuat::from_scaled_axis(h * w)
            * orientation;
        x += h * v + DVec3::from(motion.translation_correction);
        v += DVec3::from(motion.linear_velocity_correction);
        w += DVec3::from(motion.angular_velocity_correction);
        let mut p = rigid_mass * v;
        let mut l = x.cross(p) + inertia * w;
        let mut energy = 0.5 * (rigid_mass * v.length_squared() + inertia * w.length_squared());
        for body in [&a, &b] {
            let pt = body.particles()[0];
            let velocity = DVec3::from(pt.velocity);
            let dp = velocity / pt.inverse_mass;
            p += dp;
            l += DVec3::from(pt.position).cross(dp);
            energy += 0.5 * velocity.dot(dp);
        }
        assert!(p.distance(initial_p) < 1e-11);
        max_l = max_l.max(l.distance(initial_l));
        assert!(energy <= 0.2 + 1e-9, "{energy}");
    }
    assert!(max_l < 1e-10, "{max_l:e}");
    assert!(v.x > 0.1 && w.z.abs() > 0.1);
    assert!(orientation.z.abs() > 0.005);
}

#[test]
fn invalid_coupled_solve_preserves_every_participant_and_old_reactions() {
    let mut a = projectile(DVec3::new(-0.2605, 0., 0.), DVec3::X, 1., 2);
    let mut b = projectile(DVec3::ZERO, DVec3::ZERO, 1., 1);
    let boundary = [sphere(DVec3::ZERO, DVec3::ZERO, DVec3::ZERO)];
    let dt = Duration::from_millis(1);
    let mobility = [Some(SphereMobility {
        inverse_mass: 1.,
        inverse_inertia: 1.,
    })];
    step_coupled(&mut [&mut a], dt, &boundary, &mobility).unwrap();
    let before = (a.particles()[0], a.last_boundary_impulses()[0]);
    assert!(before.1.linear[0] > 0.0);
    assert!(matches!(
        step_coupled(&mut [&mut a, &mut b], dt, &boundary, &mobility),
        Err(XpbdError::InvalidConfig)
    ));
    for (boundaries, response) in [
        (&boundary[..], &[][..]),
        (
            &[Boundary::Plane {
                point: [0.; 3],
                normal: [0., 0., 1.],
                velocity: [0.; 3],
            }][..],
            &mobility[..],
        ),
        (
            &boundary[..],
            &[Some(SphereMobility {
                inverse_mass: 0.,
                inverse_inertia: 1.,
            })][..],
        ),
    ] {
        assert!(matches!(
            step_coupled(&mut [&mut a], dt, boundaries, response),
            Err(XpbdError::InvalidBoundary)
        ));
    }
    assert!(matches!(
        step_coupled(&mut [&mut a], Duration::from_nanos(1), &boundary, &mobility),
        Err(XpbdError::InvalidStep)
    ));
    assert_eq!((a.particles()[0], a.last_boundary_impulses()[0]), before);
    let result =
        step_coupled(&mut [&mut a], Duration::ZERO, &boundary, &mobility).unwrap()[0].unwrap();
    assert_eq!(result.linear_velocity_correction, [0.; 3]);
    assert_eq!(a.particles()[0], before.0);
}

#[test]
fn unperturbed_rotating_receiver_has_an_exact_identity_correction_and_overflow_rolls_back() {
    let mut a = projectile(DVec3::new(10., 0., 0.), DVec3::ZERO, 1., 3);
    let mut b = projectile(DVec3::new(2., 0., 0.), DVec3::X, 1., 3);
    let mobility = [Some(SphereMobility {
        inverse_mass: 1.,
        inverse_inertia: 1.,
    })];
    let dt = Duration::from_nanos(1_000_001);
    let center = DVec3::new(-2., 0., 0.);
    let result = step_coupled(
        &mut [&mut a, &mut b],
        dt,
        &[sphere(center, DVec3::Y, DVec3::new(0.1, 1.3, -2.7))],
        &mobility,
    )
    .unwrap()[0]
        .unwrap();
    assert_eq!(result.rotation_correction, [0., 0., 0., 1.]);
    let before = (
        a.particles()[0],
        b.particles()[0],
        b.last_boundary_impulses()[0],
    );
    assert!(matches!(
        step_coupled(
            &mut [&mut a, &mut b],
            dt,
            &[sphere(center, DVec3::ZERO, DVec3::X * f64::MAX)],
            &mobility
        ),
        Err(XpbdError::Numerical)
    ));
    assert_eq!(
        before,
        (
            a.particles()[0],
            b.particles()[0],
            b.last_boundary_impulses()[0]
        )
    );
}

#[test]
fn glancing_swept_impact_on_low_inertia_sphere_does_not_create_kinetic_energy() {
    let initial = Particle {
        position: [-0.35, 0.15, 0.0],
        velocity: [5.0, 0.0, 0.0],
        inverse_mass: 10.0,
    };
    let mut body = SoftBody::new(
        vec![initial],
        vec![],
        SolverConfig {
            gravity: [0.0; 3],
            substeps: 1,
            friction: 0.0,
            ..SolverConfig::default()
        },
    )
    .unwrap();
    let m = 0.05;
    let inertia = 1e-8;
    let dt = Duration::from_millis(60);
    let motion = step_coupled(
        &mut [&mut body],
        dt,
        &[sphere(DVec3::ZERO, DVec3::ZERO, DVec3::ZERO)],
        &[Some(SphereMobility {
            inverse_mass: 1. / m,
            inverse_inertia: 1. / inertia,
        })],
    )
    .unwrap()[0]
        .unwrap();
    let r = DVec3::from(motion.linear_velocity_correction);
    let w = DVec3::from(motion.angular_velocity_correction);
    let p = DVec3::from(body.particles()[0].velocity);
    // First intersection of the swept ray with the combined 0.26 m radius.
    let impact_x = -(0.26_f64.powi(2) - 0.15_f64.powi(2)).sqrt();
    let normal = DVec3::new(impact_x, 0.15, 0.0) / 0.26;
    let impulse = -(5.0 * normal.x) / (initial.inverse_mass + 1.0 / m) * normal;
    assert!((r + impulse / m).length() < 1e-12);
    assert!((p - DVec3::from(initial.velocity) - impulse * initial.inverse_mass).length() < 1e-12);
    let kinetic =
        0.5 * (m * r.length_squared() + inertia * w.length_squared() + 0.1 * p.length_squared());
    assert!(
        w.length() < 1e-8 && motion.rotation_correction == [0., 0., 0., 1.],
        "frictionless circular normal applied spin {w:?} or reorientation {:?}",
        motion.rotation_correction
    );
    assert!(kinetic <= 1.25 + 1e-10, "kinetic={kinetic:e} w={w:?}");
}
