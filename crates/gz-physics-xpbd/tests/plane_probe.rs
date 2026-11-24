// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics_xpbd::{
    Boundary, Particle, PlaneImpulseProbe, SoftBody, SolverConfig, step_coupled_damped_pads,
};
use std::time::Duration;

#[test]
fn individual_static_and_sliding_points_reproduce_both_weight_and_signed_friction() {
    let dt = Duration::from_millis(10);
    for substeps in [1, 4] {
        let mut material = SoftBody::new(
            vec![
                Particle {
                    position: [0., 0., 0.005],
                    velocity: [0.; 3],
                    inverse_mass: 2.,
                },
                Particle {
                    position: [0., 0.15, 0.005],
                    velocity: [0.3, 0., 0.],
                    inverse_mass: 0.8,
                },
                Particle {
                    position: [0., -0.15, 1.],
                    velocity: [0.; 3],
                    inverse_mass: 1.,
                },
            ],
            vec![],
            SolverConfig {
                gravity: [2., 0., -10.],
                friction: 0.1,
                particle_radius: 0.005,
                iterations: 4,
                substeps,
                self_contact: false,
            },
        )
        .unwrap()
        .with_static_plane_friction(0.45)
        .unwrap();
        let mut unobserved = material.clone();
        let boundary = [
            Boundary::Sphere {
                center: [8.; 3],
                radius: 0.02,
                velocity: [0.; 3],
                angular_velocity: [0.; 3],
            },
            Boundary::Plane {
                point: [0.; 3],
                normal: [0., 0., 1.],
                velocity: [0.; 3],
            },
        ];
        let mut probe = PlaneImpulseProbe::default();
        assert!(probe.readings().unwrap().is_empty());
        for _ in 0..5 {
            probe
                .step(&mut material, dt, &boundary, &[None, None], &[])
                .unwrap();
            step_coupled_damped_pads(&mut [&mut unobserved], dt, &boundary, &[None, None], &[])
                .unwrap();
            assert_eq!(material.particles(), unobserved.particles());
            #[cfg(feature = "one-way-checkpoint")]
            assert_eq!(
                material.one_way_state().unwrap(),
                unobserved.one_way_state().unwrap()
            );
            assert_eq!(
                material.last_boundary_impulses(),
                unobserved.last_boundary_impulses()
            );
            let [a, b] = probe.readings().unwrap() else {
                panic!("expected the two supported particles")
            };
            assert_eq!(
                (a.boundary, a.particle, b.boundary, b.particle),
                (1, 0, 1, 1)
            );
            for (row, mass, friction_accel) in [(a, 0.5, 2.), (b, 1.25, 1.)] {
                assert!((row.normal[2] + mass * 0.1).abs() < 1e-11);
                assert!((row.tangent[0] - mass * 0.01 * friction_accel).abs() < 1e-11);
            }
            let sum = DVec3::from(a.normal)
                + DVec3::from(a.tangent)
                + DVec3::from(b.normal)
                + DVec3::from(b.tangent);
            assert!(sum.distance(material.last_boundary_impulses()[1].linear.into()) < 1e-12);
        }
        let retained = probe.readings().unwrap().to_vec();
        assert!(probe.step(&mut material, dt, &boundary, &[], &[]).is_err());
        assert_eq!(probe.readings().unwrap(), retained.as_slice());
        probe.step(&mut material, dt, &[], &[], &[]).unwrap();
        assert!(probe.readings().unwrap().is_empty());
    }
}

#[test]
fn a_mismatched_probe_group_or_invalid_path_cannot_replace_the_last_physical_reading() {
    let mut material = SoftBody::new(
        vec![Particle {
            position: [0., 0., 0.01],
            velocity: [0.; 3],
            inverse_mass: 1.,
        }],
        vec![],
        SolverConfig {
            gravity: [0., 0., -10.],
            particle_radius: 0.01,
            ..Default::default()
        },
    )
    .unwrap();
    let floor = Boundary::Plane {
        point: [0.; 3],
        normal: [0., 0., 1.],
        velocity: [0.; 3],
    };
    let mut probe = PlaneImpulseProbe::default();
    let h = Duration::from_millis(2);
    probe
        .step_with_boundary_paths(
            &mut material,
            h,
            &[floor],
            &[],
            gz_physics_xpbd::DEFAULT_PATH_TOLERANCE,
        )
        .unwrap();
    let last = probe.readings().unwrap().to_vec();
    assert_eq!(last.len(), 1);
    assert!(
        PlaneImpulseProbe::step_group(&mut [], &mut [Some(&mut probe)], h, &[floor], &[None], &[])
            .is_err()
    );
    assert_eq!(probe.readings().unwrap(), last);
    let invalid = Boundary::Plane {
        normal: [0.; 3],
        point: [0.; 3],
        velocity: [0.; 3],
    };
    assert!(
        probe
            .step_with_boundary_paths(
                &mut material,
                h,
                &[invalid],
                &[],
                gz_physics_xpbd::DEFAULT_PATH_TOLERANCE
            )
            .is_err()
    );
    assert_eq!(probe.readings().unwrap(), last);
}
