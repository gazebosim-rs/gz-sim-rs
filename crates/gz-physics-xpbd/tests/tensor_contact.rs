// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DMat3, DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, Particle, PrincipalSphereMotion, RigidSphereMobility, SoftBody, SolverConfig,
    XpbdError, step_coupled_rigid,
};
use std::time::Duration;

fn motion() -> PrincipalSphereMotion {
    let pose = DQuat::from_rotation_y(0.47);
    PrincipalSphereMotion {
        inverse_mass: 0.5,
        inverse_principal_inertia: [12., 8., 6.],
        principal_orientation: DQuat::from_rotation_x(0.61).to_array(),
        start_orientation: pose.to_array(),
        end_orientation: pose.to_array(),
        start_angular_velocity: [0.; 3],
    }
}
fn projectile(mu: f64) -> SoftBody {
    SoftBody::new(
        vec![Particle {
            position: [-0.51, 0., 0.],
            velocity: [1., 0.6, -0.2],
            inverse_mass: 2.,
        }],
        vec![],
        SolverConfig {
            gravity: [0.; 3],
            substeps: 1,
            iterations: 2,
            friction: mu,
            ..Default::default()
        },
    )
    .unwrap()
}
fn boundary() -> Boundary {
    Boundary::Sphere {
        center: [0.; 3],
        velocity: [0.; 3],
        radius: 0.5,
        angular_velocity: [0.; 3],
    }
}
#[test]
fn sticking_and_sliding_glancing_impulses_obey_full_world_contact_operator() {
    let m = motion();
    let inverse = DMat3::from_quat(
        DQuat::from_array(m.start_orientation) * DQuat::from_array(m.principal_orientation),
    );
    let inverse = inverse
        * DMat3::from_diagonal(DVec3::from(m.inverse_principal_inertia))
        * inverse.transpose();
    let arm = DVec3::new(-0.51, 0., 0.);
    let v = DVec3::new(1., 0.6, -0.2);
    let effective = |j: DVec3| 2.5 * j + (inverse * arm.cross(j)).cross(arm);
    let matrix = DMat3::from_cols(
        effective(DVec3::X),
        effective(DVec3::Y),
        effective(DVec3::Z),
    );
    let expected = matrix.inverse() * v;
    // A 10 ns interval approximates the analytic instantaneous point contact.
    // Bound geometric/time-of-impact error independently of solver tolerance.
    for mu in [3., 0.12] {
        let mut body = projectile(mu);
        let result = step_coupled_rigid(
            &mut [&mut body],
            Duration::from_nanos(10),
            &[boundary()],
            &[Some(RigidSphereMobility::Principal(m))],
        )
        .unwrap()[0]
            .unwrap();
        let impulse = DVec3::from(body.last_boundary_impulses()[0].linear);
        let torque = DVec3::from(body.last_boundary_impulses()[0].angular);
        let spin = DVec3::from(result.angular_velocity_correction);
        assert!((DVec3::from(result.linear_velocity_correction) - 0.5 * impulse).length() < 1e-13);
        assert!((inverse.inverse() * spin - torque).length() < 1e-9);
        let slip = v - effective(impulse);
        if mu > 1. {
            assert!(
                impulse.distance(expected) < 1e-7 && slip.length() < 1e-7,
                "J={impulse}, Jexact={expected}, slip={slip}"
            );
        } else {
            let friction = DVec3::new(0., impulse.y, impulse.z);
            assert!((friction.length() - mu * impulse.x).abs() < 1e-8);
            assert!(friction.dot(slip) > 0. && friction.cross(slip).length() < 1e-8);
        }
        assert!(spin.cross(torque).length() > 1e-4);
    }
}
#[test]
fn invalid_principal_data_refuses_atomically_and_free_rotation_has_no_artificial_correction() {
    let mut body = projectile(1.);
    let initial = body.particles().to_vec();
    let base = motion();
    for invalid in [
        PrincipalSphereMotion {
            inverse_principal_inertia: [0.1, 1., 1.],
            ..base
        },
        PrincipalSphereMotion {
            start_angular_velocity: [4000., 0., 0.],
            ..base
        },
        PrincipalSphereMotion {
            inverse_mass: f64::NAN,
            ..base
        },
        PrincipalSphereMotion {
            inverse_principal_inertia: [1., 0., 1.],
            ..base
        },
        PrincipalSphereMotion {
            principal_orientation: [0.; 4],
            ..base
        },
        PrincipalSphereMotion {
            end_orientation: [f64::NAN; 4],
            ..base
        },
        PrincipalSphereMotion {
            start_angular_velocity: [f64::INFINITY; 3],
            ..base
        },
    ] {
        assert!(matches!(
            step_coupled_rigid(
                &mut [&mut body],
                Duration::from_millis(1),
                &[boundary()],
                &[Some(RigidSphereMobility::Principal(invalid))]
            ),
            Err(XpbdError::InvalidBoundary)
        ));
        assert_eq!(body.particles(), initial);
        assert!(body.last_boundary_impulses().is_empty());
    }
    let free = PrincipalSphereMotion {
        end_orientation: DQuat::from_rotation_z(0.95).to_array(),
        start_angular_velocity: [3., 4., 5.],
        ..base
    };
    let far = Boundary::Sphere {
        center: [20., 0., 0.],
        radius: 0.5,
        velocity: [1., 2., 3.],
        angular_velocity: [1., 4., -2.],
    };
    let m = step_coupled_rigid(
        &mut [&mut body],
        Duration::from_millis(1),
        &[far],
        &[Some(RigidSphereMobility::Principal(free))],
    )
    .unwrap()[0]
        .unwrap();
    assert_eq!(m.angular_velocity_correction, [0.; 3]);
    assert_eq!(m.rotation_correction, [0., 0., 0., 1.]);
    assert_eq!(m.linear_velocity_correction, [0.; 3]);
}
