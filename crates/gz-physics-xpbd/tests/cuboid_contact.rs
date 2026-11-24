// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, Particle, PrincipalRigidMotion, RigidMobility, SoftBody, SolverConfig, XpbdError,
    step_coupled_rigid,
};
use std::time::Duration;

fn particle(x: DVec3, v: DVec3, mass: f64, n: u32) -> SoftBody {
    SoftBody::new(
        vec![Particle {
            position: x.to_array(),
            velocity: v.to_array(),
            inverse_mass: mass.recip(),
        }],
        vec![],
        SolverConfig {
            gravity: [0.; 3],
            substeps: n,
            iterations: 12,
            particle_radius: 0.01,
            friction: 0.,
            self_contact: false,
        },
    )
    .unwrap()
}
fn boundary(half: DVec3, spin: DVec3) -> Boundary {
    Boundary::Cuboid {
        half_extents: half.to_array(),
        center: [0.; 3],
        velocity: [0.; 3],
        angular_velocity: spin.to_array(),
    }
}
fn mobility(start: DQuat, end: DQuat, spin: DVec3) -> RigidMobility {
    RigidMobility::Principal(PrincipalRigidMotion {
        inverse_mass: 0.5,
        inverse_principal_inertia: [10., 1. / 0.12, 1. / 0.16],
        start_orientation: start.to_array(),
        end_orientation: end.to_array(),
        principal_orientation: DQuat::IDENTITY.to_array(),
        start_angular_velocity: spin.to_array(),
    })
}
#[test]
fn normal_impulse_off_center_matches_the_closed_form_frictionless_spatial_mass() {
    let q = DQuat::from_rotation_z(0.6) * DQuat::from_rotation_x(-0.2);
    let half = DVec3::new(0.5, 0.25, 0.15);
    let n = q * (-DVec3::X);
    let arm = q * DVec3::new(-0.51, 0.18, 0.1);
    let tau = arm.cross(n);
    let inverse_tau = q * (DVec3::new(10., 1. / 0.12, 1. / 0.16) * (q.conjugate() * tau));
    let mass = 0.3;
    let j = 2. / (1. / mass + 0.5 + tau.dot(inverse_tau));
    for substeps in [1, 2, 4] {
        let mut body = particle(arm, -2. * n, mass, substeps);
        let result = step_coupled_rigid(
            &mut [&mut body],
            Duration::from_micros(1),
            &[boundary(half, DVec3::ZERO)],
            &[Some(mobility(q, q, DVec3::ZERO))],
        )
        .unwrap()[0]
            .unwrap();
        let actual = DVec3::from(body.last_boundary_impulses()[0].linear);
        assert!(
            actual.distance(-j * n) < 5e-7,
            "n={substeps} impulse={actual} expected={}",
            -j * n
        );
        assert!(DVec3::from(result.angular_velocity_correction).distance(-j * inverse_tau) < 5e-6);
        assert_ne!(result.rotation_correction, DQuat::IDENTITY.to_array());
    }
}
#[test]
fn rotating_box_transfers_angular_momentum_to_a_stationary_particle() {
    let half = DVec3::new(1., 0.08, 0.14);
    let q0 = DQuat::from_rotation_z(-0.4);
    let q1 = DQuat::from_rotation_z(0.4);
    let spin = DVec3::Z * 80.;
    let p_mass = 0.005;
    for n in [1, 2, 4] {
        let mut body = particle(DVec3::X * 0.8, DVec3::ZERO, p_mass, n);
        let motion = step_coupled_rigid(
            &mut [&mut body],
            Duration::from_millis(10),
            &[boundary(half, spin)],
            &[Some(mobility(q0, q1, spin))],
        )
        .unwrap()[0]
            .unwrap();
        let p = body.particles()[0];
        let pv = DVec3::from(p.velocity);
        let px = DVec3::from(p.position);
        let rv = DVec3::from(motion.linear_velocity_correction);
        let rx = DVec3::from(motion.translation_correction);
        let omega = spin + DVec3::from(motion.angular_velocity_correction);
        let orientation = DQuat::from_array(motion.rotation_correction) * q1;
        let angular =
            orientation * (DVec3::new(0.1, 0.12, 0.16) * (orientation.conjugate() * omega));
        let delta = (orientation.conjugate() * (px - rx)).abs() - half;
        let gap = delta.max(DVec3::ZERO).length() + delta.max_element().min(0.) - 0.01;
        let initial_k = 0.5 * 0.16 * 80. * 80.;
        let energy =
            0.5 * omega.dot(angular) + rv.length_squared() + 0.5 * p_mass * pv.length_squared();
        assert!(
            pv.y > 35. && omega.z < 80.,
            "n={n} particle={pv} spin={omega}"
        );
        assert!((2. * rv + p_mass * pv).length() < 1e-12);
        assert!(
            (angular + rx.cross(2. * rv) + px.cross(p_mass * pv) - 0.16 * spin).length() < 1e-11
        );
        assert!(
            gap > -1e-6 && energy <= initial_k + 1e-7,
            "n={n} gap={gap} K={energy}"
        );
    }
}
#[test]
fn a_cuboid_requires_a_full_oriented_receiver_even_for_zero_duration() {
    let q = DQuat::IDENTITY;
    let b = boundary(DVec3::ONE, DVec3::ZERO);
    let mut p = particle(DVec3::X * 2., DVec3::ZERO, 1., 1);
    let initial = p.particles().to_vec();
    assert_eq!(
        p.step(Duration::ZERO, &[b]),
        Err(XpbdError::InvalidBoundary)
    );
    assert!(matches!(
        step_coupled_rigid(&mut [&mut p], Duration::ZERO, &[b], &[None]),
        Err(XpbdError::InvalidBoundary)
    ));
    assert!(matches!(
        step_coupled_rigid(
            &mut [&mut p],
            Duration::ZERO,
            &[boundary(DVec3::new(1., -1., 1.), DVec3::ZERO)],
            &[Some(mobility(q, q, DVec3::ZERO))]
        ),
        Err(XpbdError::InvalidBoundary)
    ));
    assert_eq!(p.particles(), initial);
}

#[test]
fn unresolved_competing_contacts_and_deep_initial_overlap_leave_all_published_states_intact() {
    let half = DVec3::new(0.5, 0.2, 0.2);
    let q = DQuat::IDENTITY;
    let mass = RigidMobility::Principal(PrincipalRigidMotion {
        inverse_mass: 100.,
        inverse_principal_inertia: [1000., 800., 650.],
        principal_orientation: q.to_array(),
        start_orientation: q.to_array(),
        end_orientation: q.to_array(),
        start_angular_velocity: [0.; 3],
    });
    let make = |x: f64, v: f64| {
        SoftBody::new(
            vec![Particle {
                position: [x, 0., 0.],
                velocity: [v, 0., 0.],
                inverse_mass: 1.,
            }],
            vec![],
            SolverConfig {
                gravity: [0.; 3],
                iterations: 1,
                substeps: 1,
                particle_radius: 0.01,
                friction: 0.,
                self_contact: false,
            },
        )
        .unwrap()
    };
    for (x, v) in [(-0.525, 10.), (0.1, 0.)] {
        let (mut a, mut b) = (make(x, v), make(0.525, -10.));
        let before = [a.particles().to_vec(), b.particles().to_vec()];
        let result = step_coupled_rigid(
            &mut [&mut a, &mut b],
            Duration::from_millis(10),
            &[boundary(half, DVec3::ZERO)],
            &[Some(mass)],
        );
        assert!(
            matches!(result, Err(XpbdError::UnresolvedSweep)),
            "{result:?}"
        );
        assert_eq!(a.particles(), before[0]);
        assert_eq!(b.particles(), before[1]);
        assert!(a.last_boundary_impulses().is_empty() && b.last_boundary_impulses().is_empty());
    }
}

#[test]
fn tangent_pass_has_no_impulse_and_resting_shallow_overlap_spends_no_kinetic_energy() {
    let half = DVec3::new(0.5, 0.25, 0.15);
    let q = DQuat::IDENTITY;
    let zero = DVec3::ZERO;
    let shape = boundary(half, zero);
    for n in [1, 4, 16] {
        let mut p = particle(DVec3::new(-2., 0.26, 0.), 4. * DVec3::X, 0.3, n);
        let result = step_coupled_rigid(
            &mut [&mut p],
            Duration::from_secs(1),
            &[shape],
            &[Some(mobility(q, q, zero))],
        )
        .unwrap()[0]
            .unwrap();
        assert!(
            DVec3::from(p.last_boundary_impulses()[0].linear).length() < 1e-10,
            "n={n} {:?}",
            result.linear_velocity_correction
        );
        let mut overlap = particle(DVec3::new(0., 0.255, 0.), zero, 0.3, n);
        let result = step_coupled_rigid(
            &mut [&mut overlap],
            Duration::from_millis(2),
            &[shape],
            &[Some(mobility(q, q, zero))],
        )
        .unwrap()[0]
            .unwrap();
        assert!(
            DVec3::from(overlap.particles()[0].velocity).length() < 1e-9
                && DVec3::from(result.linear_velocity_correction).length() < 1e-9
        );
        assert!(
            (overlap.particles()[0].position[1] - result.translation_correction[1] - 0.26).abs()
                < 1e-9
        );
    }
}

#[test]
fn a_stationary_particle_grazed_by_a_rotating_corner_receives_zero_impulse() {
    let half = DVec3::new(1., 0.08, 0.14);
    let radial = half.x.hypot(half.y) + 0.01;
    let spin = DVec3::Z * 80.;
    let q0 = DQuat::from_rotation_z(-0.4);
    let q1 = DQuat::from_rotation_z(0.4);
    for n in [1, 2, 4, 16] {
        for x in [radial, radial + 1e-6] {
            let mut p = particle(DVec3::X * x, DVec3::ZERO, 0.005, n);
            let m = step_coupled_rigid(
                &mut [&mut p],
                Duration::from_millis(10),
                &[boundary(half, spin)],
                &[Some(mobility(q0, q1, spin))],
            )
            .unwrap()[0]
                .unwrap();
            assert_eq!(m.linear_velocity_correction, [0.; 3]);
            assert_eq!(p.particles()[0].velocity, [0.; 3]);
        }
    }
}

#[test]
fn brief_rotating_corner_impact_matches_its_instantaneous_analytical_impulse() {
    let half = DVec3::new(1., 0.08, 0.14);
    let q0 = DQuat::from_rotation_z(-0.4);
    let q1 = DQuat::from_rotation_z(0.4);
    let spin = DVec3::Z * 80.;
    let radial = half.x.hypot(half.y) + 0.01 - 1e-5;
    let corner = half.x.hypot(half.y);
    let polar = half.y.atan2(half.x)
        + ((radial * radial + corner * corner - 0.01_f64.powi(2)) / (2. * radial * corner)).acos();
    let normal_local = (DVec3::new(
        radial * polar.cos() - half.x,
        radial * polar.sin() - half.y,
        0.,
    ) / 0.01)
        .normalize();
    let normal = DQuat::from_rotation_z(-polar) * normal_local;
    let torque_arm = radial * normal.y;
    let expected = 80. * torque_arm / (1. + 0.005 * (0.5 + torque_arm * torque_arm / 0.16));
    for n in [1, 4, 16, 64, 256] {
        let mut body = particle(DVec3::X * radial, DVec3::ZERO, 0.005, n);
        let outcome = step_coupled_rigid(
            &mut [&mut body],
            Duration::from_millis(10),
            &[boundary(half, spin)],
            &[Some(mobility(q0, q1, spin))],
        )
        .unwrap();
        let v = DVec3::from(body.particles()[0].velocity);
        println!(
            "shallow rotating corner n={n} speed {:.8} analytic {expected:.8}",
            v.length()
        );
        assert!((v.length() - expected).abs() < 1e-4);
        let rigid = DVec3::from(outcome[0].unwrap().linear_velocity_correction);
        assert!((2. * rigid + 0.005 * v).length() < 1e-13);
    }
}

#[test]
fn simultaneous_adjacent_face_impacts_match_two_contact_effective_mass_in_both_orders() {
    let q = DQuat::IDENTITY;
    let half = DVec3::new(0.5, 0.25, 0.15);
    let shape = boundary(half, DVec3::ZERO);
    let m = 0.3;
    // Equal closing speeds; opposite contact torques cancel. With symmetry
    // the two angular diagonal/off-diagonal terms cancel in K*J.
    let expected = 2. / (1. / m + 0.5);
    for reverse in [false, true] {
        let mut a = particle(DVec3::new(-0.51, 0.10, 0.), 2. * DVec3::X, m, 1);
        let mut b = particle(DVec3::new(0.10, -0.26, 0.), 2. * DVec3::Y, m, 1);
        let mut bodies = if reverse {
            [&mut b, &mut a]
        } else {
            [&mut a, &mut b]
        };
        let state = step_coupled_rigid(
            &mut bodies,
            Duration::from_micros(1),
            &[shape],
            &[Some(mobility(q, q, DVec3::ZERO))],
        )
        .unwrap()[0]
            .unwrap();
        let ja = DVec3::from(a.last_boundary_impulses()[0].linear);
        let jb = DVec3::from(b.last_boundary_impulses()[0].linear);
        assert!(
            ja.distance(expected * DVec3::X) < 1e-6 && jb.distance(expected * DVec3::Y) < 1e-6,
            "reverse={reverse} Ja={ja} Jb={jb} exact={expected}"
        );
        assert!(DVec3::from(state.angular_velocity_correction).length() < 1e-5);
    }
}

#[test]
fn swept_contact_group_refuses_friction_before_publication() {
    let half = DVec3::new(1., 0.08, 0.14);
    let radial = half.x.hypot(half.y) + 0.01 - 1e-5;
    let spin = DVec3::Z * 80.;
    let q0 = DQuat::from_rotation_z(-0.4);
    let q1 = DQuat::from_rotation_z(0.4);
    let shape = boundary(half, spin);
    {
        let friction = 0.2;
        let mut a = SoftBody::new(
            vec![Particle {
                position: [radial, 0., -0.03],
                velocity: [0.; 3],
                inverse_mass: 200.,
            }],
            vec![],
            SolverConfig {
                gravity: [0.; 3],
                substeps: 1,
                iterations: 4,
                friction,
                particle_radius: 0.01,
                self_contact: false,
            },
        )
        .unwrap();
        let mut b = particle(DVec3::new(radial, 0., 0.03), DVec3::ZERO, 0.005, 1);
        let before = [a.particles().to_vec(), b.particles().to_vec()];
        let result = step_coupled_rigid(
            &mut [&mut a, &mut b],
            Duration::from_millis(10),
            &[shape],
            &[Some(mobility(q0, q1, spin))],
        );
        assert!(
            matches!(result, Err(XpbdError::UnresolvedSweep)),
            "friction={friction} result={result:?}"
        );
        assert_eq!(a.particles(), before[0]);
        assert_eq!(b.particles(), before[1]);
    }
}

#[test]
fn prescribed_pad_reports_the_equal_and_opposite_wrench_without_a_free_body_correction() {
    let q = DQuat::from_rotation_z(0.6) * DQuat::from_rotation_x(-0.2);
    let normal = q * -DVec3::X;
    let point = q * DVec3::new(-0.51, 0.18, 0.1);
    let mass = 0.3;
    for n in [1, 2, 4] {
        let mut material = particle(point, -2. * normal, mass, n);
        let result = step_coupled_rigid(
            &mut [&mut material],
            Duration::from_micros(1),
            &[boundary(DVec3::new(0.5, 0.25, 0.15), DVec3::ZERO)],
            &[Some(RigidMobility::PrescribedCuboid {
                normal_compliance: 0.,
                start_orientation: q.to_array(),
                end_orientation: q.to_array(),
                start_angular_velocity: [0.; 3],
            })],
        )
        .unwrap();
        let load = material.last_boundary_impulses()[0];
        let impulse = -2. * mass * normal;
        assert!(result[0].is_none());
        assert!(DVec3::from(load.linear).distance(impulse) < 1e-8);
        assert!(DVec3::from(load.angular).distance(point.cross(impulse)) < 1e-8);
        assert!(DVec3::from(material.particles()[0].velocity).length() < 1e-7);
    }
}

#[test]
fn rotating_prescribed_cuboid_sweeps_a_point_without_losing_its_prescribed_motion() {
    let q0 = DQuat::from_rotation_z(-0.4);
    let q1 = DQuat::from_rotation_z(0.4);
    let spin = DVec3::Z * 80.;
    for n in [1, 2, 4] {
        let mut material = particle(DVec3::X * 0.8, DVec3::ZERO, 0.005, n);
        let motion = RigidMobility::PrescribedCuboid {
            normal_compliance: 0.,
            start_orientation: q0.to_array(),
            end_orientation: q1.to_array(),
            start_angular_velocity: spin.to_array(),
        };
        let correction = step_coupled_rigid(
            &mut [&mut material],
            Duration::from_millis(10),
            &[boundary(DVec3::new(1., 0.08, 0.14), spin)],
            &[Some(motion)],
        )
        .unwrap();
        assert!(correction[0].is_none());
        let particle = material.particles()[0];
        let momentum = 0.005 * DVec3::from(particle.velocity);
        assert!(momentum.y > 0.15);
        let load = material.last_boundary_impulses()[0];
        assert!((momentum + DVec3::from(load.linear)).length() < 1e-10);
        assert!(
            (DVec3::from(particle.position).cross(momentum) + DVec3::from(load.angular)).length()
                < 1e-10
        );
    }
}

#[test]
fn prescribed_cuboid_rejects_nonbox_geometry_and_invalid_poses_before_publishing() {
    let mut material = particle(DVec3::X, DVec3::ZERO, 1., 2);
    let initial = material.particles().to_vec();
    let valid = DQuat::IDENTITY.to_array();
    let b = boundary(DVec3::ONE, DVec3::ZERO);
    let sphere = Boundary::Sphere {
        center: [0.; 3],
        radius: 1.,
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    };
    for (geometry, start, end, spin) in [
        (sphere, valid, valid, [0.; 3]),
        (b, [0.; 4], valid, [0.; 3]),
        (b, valid, [f64::NAN; 4], [0.; 3]),
        (b, valid, valid, [f64::INFINITY; 3]),
    ] {
        let m = RigidMobility::PrescribedCuboid {
            normal_compliance: 0.,
            start_orientation: start,
            end_orientation: end,
            start_angular_velocity: spin,
        };
        assert!(matches!(
            step_coupled_rigid(
                &mut [&mut material],
                Duration::from_millis(1),
                &[geometry],
                &[Some(m)]
            ),
            Err(XpbdError::InvalidBoundary)
        ));
        assert_eq!(material.particles(), initial);
    }
}

fn compliant_pad(compliance: f64) -> RigidMobility {
    RigidMobility::PrescribedCuboid {
        normal_compliance: compliance,
        start_orientation: DQuat::IDENTITY.to_array(),
        end_orientation: DQuat::IDENTITY.to_array(),
        start_angular_velocity: [0.; 3],
    }
}

#[test]
fn a_compressed_particle_supports_its_weight_at_the_hooke_law_depth() {
    let mass = 0.02;
    let gravity = 9.81;
    let compliance = 0.005;
    let top = 0.1;
    let radius = 0.01;
    let z = top + radius - mass * gravity * compliance;
    for substeps in [1, 2, 4] {
        let mut material = SoftBody::new(
            vec![Particle {
                position: [0., 0., z],
                velocity: [0.; 3],
                inverse_mass: 1. / mass,
            }],
            vec![],
            SolverConfig {
                gravity: [0., 0., -gravity],
                substeps,
                iterations: 8,
                particle_radius: radius,
                ..Default::default()
            },
        )
        .unwrap();
        for _ in 0..100 {
            step_coupled_rigid(
                &mut [&mut material],
                Duration::from_millis(1),
                &[boundary(DVec3::splat(top), DVec3::ZERO)],
                &[Some(compliant_pad(compliance))],
            )
            .unwrap();
            assert!((material.particles()[0].position[2] - z).abs() < 2e-12);
            assert!(
                (material.last_boundary_impulses()[0].linear[2] / 0.001 + mass * gravity).abs()
                    < 2e-10
            );
        }
    }
}

#[test]
fn compliant_skin_refuses_invalid_compliance_core_crossings_and_unresolved_glances() {
    let box_shape = boundary(DVec3::splat(0.1), DVec3::ZERO);
    for (point, speed, compliance, error) in [
        ([0.101, 0., 0.], [0.; 3], -0.1, XpbdError::InvalidBoundary),
        (
            [0.101, 0., 0.],
            [0.; 3],
            f64::NAN,
            XpbdError::InvalidBoundary,
        ),
        ([0., 0., 0.], [0.; 3], 0.001, XpbdError::UnresolvedSweep),
        (
            [-0.2, 0., 0.],
            [400., 0., 0.],
            0.001,
            XpbdError::UnresolvedSweep,
        ),
        (
            [-0.2, 0.105, 0.],
            [400., 0., 0.],
            0.001,
            XpbdError::UnresolvedSweep,
        ),
    ] {
        let mut body = particle(point.into(), speed.into(), 0.02, 1);
        let initial = body.particles().to_vec();
        assert!(
            matches!(step_coupled_rigid(&mut [&mut body],Duration::from_millis(1),
            &[box_shape],&[Some(compliant_pad(compliance))]),Err(e) if e == error)
        );
        assert_eq!(body.particles(), initial);
    }
}

#[test]
fn opposed_pads_share_static_weight_and_slide_at_their_coulomb_limits_in_either_order() {
    let mass = 0.02;
    let mu = 0.2;
    let compliance = 0.001; // 1 mm indentation gives 1 N per pad.
    for (gravity, expected_load) in [(9.81, mass * 9.81 / 2.), (40., mu)] {
        for substeps in [1, 2, 4] {
            for order in [[-1., 1.], [1., -1.]] {
                let pads = order.map(|sign| Boundary::Cuboid {
                    center: [sign * 0.039, 0., 0.],
                    half_extents: [0.03, 0.1, 0.1],
                    velocity: [0.; 3],
                    angular_velocity: [0.; 3],
                });
                let mut material = SoftBody::new(
                    vec![Particle {
                        position: [0.; 3],
                        velocity: [0.; 3],
                        inverse_mass: 1. / mass,
                    }],
                    vec![],
                    SolverConfig {
                        gravity: [0., 0., -gravity],
                        friction: mu,
                        substeps,
                        iterations: 12,
                        particle_radius: 0.01,
                        self_contact: false,
                    },
                )
                .unwrap();
                for _ in 0..20 {
                    step_coupled_rigid(
                        &mut [&mut material],
                        Duration::from_millis(1),
                        &pads,
                        &[Some(compliant_pad(compliance)); 2],
                    )
                    .unwrap();
                    for (j, sign) in material.last_boundary_impulses().iter().zip(order) {
                        let f = DVec3::from(j.linear) / 0.001;
                        assert!(
                            (f.x - sign).abs() < 1e-8 && (f.z + expected_load).abs() < 1e-8,
                            "{f:?}"
                        );
                    }
                }
                let final_v = material.particles()[0].velocity[2];
                assert!((final_v - 0.02 * (2. * expected_load / mass - gravity)).abs() < 1e-9);
                if gravity == 9.81 {
                    assert!(material.particles()[0].position[2].abs() < 1e-12);
                }
            }
        }
    }
}
