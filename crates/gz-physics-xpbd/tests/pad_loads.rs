// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, Particle, RigidMobility, SoftBody, SolverConfig, XpbdError, step_coupled_rigid,
};
use std::time::Duration;

const DT: Duration = Duration::from_millis(1);
const MASSES: [f64; 2] = [0.02, 0.06];
const MU: f64 = 0.2;

fn material(substeps: u32) -> SoftBody {
    SoftBody::new(
        MASSES
            .iter()
            .enumerate()
            .map(|(i, mass)| Particle {
                position: [0., 0., 0.04 * i as f64],
                velocity: [0.; 3],
                inverse_mass: 1. / mass,
            })
            .collect(),
        vec![],
        SolverConfig {
            substeps,
            iterations: 12,
            gravity: [0., 0., -9.81],
            friction: MU,
            particle_radius: 0.01,
            self_contact: false,
        },
    )
    .unwrap()
}
fn pad(x: f64) -> Boundary {
    Boundary::Cuboid {
        center: [x, 0., 0.],
        half_extents: [0.03, 0.1, 0.1],
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    }
}
fn solve(body: &mut SoftBody, centers: [f64; 2]) -> Result<(), XpbdError> {
    let mobility = RigidMobility::PrescribedCuboid {
        normal_compliance: 0.001,
        start_orientation: DQuat::IDENTITY.to_array(),
        end_orientation: DQuat::IDENTITY.to_array(),
        start_angular_velocity: [0.; 3],
    };
    step_coupled_rigid(&mut [body], DT, &centers.map(pad), &[Some(mobility); 2]).map(|_| ())
}
#[test]
fn two_layers_report_each_pads_normal_and_tangent_during_sticking_and_sliding() {
    let h = DT.as_secs_f64();
    for substeps in [1, 2, 4] {
        let mut quiet = material(substeps);
        let mut observed = quiet.clone().with_pad_loads();
        for _ in 0..12 {
            // A 1 mm indentation at 0.001 m/N gives 1 N per pad per particle.
            // The first layer sticks; the heavier second layer saturates friction.
            for body in [&mut quiet, &mut observed] {
                solve(body, [-0.039, 0.039]).unwrap();
            }
            assert_eq!(quiet.particles(), observed.particles());
            assert_eq!(
                quiet.last_boundary_impulses(),
                observed.last_boundary_impulses()
            );
            assert!(quiet.last_pad_loads().is_empty());
            let loads = observed.last_pad_loads();
            assert_eq!(loads.len(), 4);
            for (boundary, rows) in loads.chunks_exact(2).enumerate() {
                let side = if boundary == 0 { -1. } else { 1. };
                let mut total = DVec3::ZERO;
                for (i, row) in rows.iter().enumerate() {
                    assert_eq!((row.boundary, row.particle), (boundary, i));
                    let normal = DVec3::from(row.normal);
                    let tangent = DVec3::from(row.tangent);
                    let support = (0.5 * MASSES[i] * 9.81).min(MU);
                    assert!((normal - side * h * DVec3::X).length() < 2e-11);
                    assert!((tangent + support * h * DVec3::Z).length() < 2e-11);
                    total += normal + tangent;
                }
                assert!(
                    total.distance(DVec3::from(
                        observed.last_boundary_impulses()[boundary].linear
                    )) < 1e-15
                );
            }
        }
        let committed = observed.last_pad_loads().to_vec();
        // An impossible core intersection follows an already loaded interval.
        assert_eq!(
            solve(&mut observed, [-0.039, 0.02]),
            Err(XpbdError::UnresolvedSweep)
        );
        assert_eq!(observed.last_pad_loads(), committed);
        solve(&mut observed, [-0.2, 0.2]).unwrap();
        assert!(observed.last_pad_loads().is_empty());
    }
}

#[cfg(feature = "one-way-checkpoint")]
#[test]
fn restoring_one_way_reactions_clears_old_articulated_pad_readings() {
    let mut observed = material(2).with_pad_loads();
    solve(&mut observed, [-0.039, 0.039]).unwrap();
    assert!(!observed.last_pad_loads().is_empty());
    let mut one_way = material(2);
    one_way
        .step(
            DT,
            &[Boundary::Plane {
                point: [0., 0., -1.],
                normal: [0., 0., 1.],
                velocity: [0.; 3],
            }],
        )
        .unwrap();
    observed
        .install_one_way_state(one_way.one_way_state().unwrap())
        .unwrap();
    assert_eq!(
        observed.last_boundary_impulses(),
        one_way.last_boundary_impulses()
    );
    assert!(observed.last_pad_loads().is_empty());
}

#[test]
fn hard_pad_reports_each_particles_momentum_change_and_clears_after_release() {
    for substeps in [1, 2, 4] {
        let velocity = DVec3::new(-0.3, 0., 0.8);
        let mut body = SoftBody::new(
            MASSES
                .iter()
                .enumerate()
                .map(|(i, mass)| Particle {
                    position: [0., 0., i as f64 * 0.04],
                    velocity: velocity.to_array(),
                    inverse_mass: 1. / mass,
                })
                .collect(),
            vec![],
            SolverConfig {
                substeps,
                iterations: 12,
                gravity: [0.; 3],
                friction: MU,
                particle_radius: 0.01,
                self_contact: false,
            },
        )
        .unwrap();
        let mut observed = body.clone().with_pad_loads();
        let rigid = RigidMobility::PrescribedCuboid {
            normal_compliance: 0.,
            start_orientation: DQuat::IDENTITY.to_array(),
            end_orientation: DQuat::IDENTITY.to_array(),
            start_angular_velocity: [0.; 3],
        };
        let wall = pad(-0.03999);
        for b in [&mut body, &mut observed] {
            step_coupled_rigid(&mut [b], DT, &[wall], &[Some(rigid)]).unwrap();
        }
        assert_eq!(body.particles(), observed.particles());
        assert_eq!(
            body.last_boundary_impulses(),
            observed.last_boundary_impulses()
        );
        let readings = observed.last_pad_loads();
        assert_eq!(readings.len(), 2);
        let mut total = DVec3::ZERO;
        for (i, row) in readings.iter().enumerate() {
            assert_eq!((row.boundary, row.particle), (0, i));
            let n = DVec3::from(row.normal);
            let t = DVec3::from(row.tangent);
            assert!(n.x < 0. && n.y.abs() + n.z.abs() < 1e-12);
            assert!(t.z > 0. && t.x.abs() + t.y.abs() < 1e-12);
            let measured = MASSES[i] * (velocity - DVec3::from(observed.particles()[i].velocity));
            assert!(measured.distance(n + t) < 1e-13);
            total += n + t;
        }
        assert!(total.distance(DVec3::from(observed.last_boundary_impulses()[0].linear)) < 1e-13);
        step_coupled_rigid(&mut [&mut observed], DT, &[pad(-2.)], &[Some(rigid)]).unwrap();
        assert!(observed.last_pad_loads().is_empty());
    }
}

#[test]
fn viscous_pad_matches_implicit_compression_without_tension_or_lost_reaction() {
    use gz_physics_xpbd::step_coupled_damped_pads;
    let compliance = 0.001;
    let mobility = Some(RigidMobility::PrescribedCuboid {
        normal_compliance: compliance,
        start_orientation: DQuat::IDENTITY.to_array(),
        end_orientation: DQuat::IDENTITY.to_array(),
        start_angular_velocity: [0.; 3],
    });
    let tau = compliance * 4.0;
    let h = DT.as_secs_f64();
    for mass_inv in [0., 50.] {
        for (pad_speed, indent) in [
            (0.2, 0.001),
            (-0.2, 0.001),
            (-0.8, 0.001),
            (0.2, -0.0001),
            (-0.2, -0.001),
        ] {
            let mut b = SoftBody::new(
                vec![Particle {
                    position: [0.; 3],
                    velocity: [0.; 3],
                    inverse_mass: mass_inv,
                }],
                vec![],
                SolverConfig {
                    gravity: [0.; 3],
                    substeps: 1,
                    iterations: 8,
                    friction: 0.,
                    particle_radius: 0.01,
                    self_contact: false,
                },
            )
            .unwrap()
            .with_pad_loads();
            let mut boundary = pad(-0.04 + indent);
            if let Boundary::Cuboid {
                ref mut velocity, ..
            } = boundary
            {
                velocity[0] = pad_speed;
            }
            let trial = indent + pad_speed * h;
            let expected = if trial > 0. {
                ((h * trial + tau * (trial - indent.max(0.)))
                    / (compliance + mass_inv * h * (h + tau)))
                    .max(0.)
            } else {
                0.
            };
            step_coupled_damped_pads(&mut [&mut b], DT, &[boundary], &[mobility], &[(0, 4.)])
                .unwrap();
            assert!(
                (b.last_boundary_impulses()[0].linear[0] + expected).abs() < 1e-13,
                "{mass_inv} {pad_speed} {indent} {expected:?} {:?}",
                b.last_boundary_impulses()
            );
            assert!((b.particles()[0].velocity[0] - mass_inv * expected).abs() < 1e-12);
            assert!(
                (b.last_pad_loads()
                    .iter()
                    .map(|load| load.normal[0])
                    .sum::<f64>()
                    + expected)
                    .abs()
                    < 1e-13
            );
            let state = b.particles().to_vec();
            let prior = b.last_boundary_impulses().to_vec();
            for invalid in [
                vec![(1, 4.)],
                vec![(0, -1.)],
                vec![(0, 0.)],
                vec![(0, f64::NAN)],
                vec![(0, 1.), (0, 2.)],
            ] {
                assert!(matches!(
                    step_coupled_damped_pads(&mut [&mut b], DT, &[boundary], &[mobility], &invalid),
                    Err(XpbdError::InvalidBoundary)
                ));
                assert_eq!(b.particles(), state);
                assert_eq!(b.last_boundary_impulses(), prior);
            }
        }
    }
}

#[test]
fn spring_dashpot_release_converges_to_a_passive_damped_oscillator() {
    use gz_physics_xpbd::step_coupled_damped_pads;
    let m: f64 = 0.02;
    let k: f64 = 1000.;
    let c = 4.;
    let initial = 0.001;
    let t = 0.002;
    let a = c / (2. * m);
    let w = (k / m - a * a).sqrt();
    let exact = initial * (-a * t).exp() * ((w * t).cos() + a / w * (w * t).sin());
    let mobility = Some(RigidMobility::PrescribedCuboid {
        normal_compliance: 1. / k,
        start_orientation: DQuat::IDENTITY.to_array(),
        end_orientation: DQuat::IDENTITY.to_array(),
        start_angular_velocity: [0.; 3],
    });
    let mut errors = Vec::new();
    for steps in [8, 16, 32, 64] {
        let mut b = SoftBody::new(
            vec![Particle {
                position: [0.; 3],
                velocity: [0.; 3],
                inverse_mass: 1. / m,
            }],
            vec![],
            SolverConfig {
                gravity: [0.; 3],
                friction: 0.,
                particle_radius: 0.01,
                self_contact: false,
                iterations: 4,
                substeps: 1,
            },
        )
        .unwrap();
        let mut energy = 0.5 * k * initial * initial;
        let mut net = 0.;
        for _ in 0..steps {
            step_coupled_damped_pads(
                &mut [&mut b],
                Duration::from_nanos(2_000_000 / steps),
                &[pad(-0.04 + initial)],
                &[mobility],
                &[(0, c)],
            )
            .unwrap();
            let p = b.particles()[0];
            let current = 0.5 * (k * (initial - p.position[0]).powi(2) + m * p.velocity[0].powi(2));
            assert!(current <= energy + 1e-15 && current >= 0.);
            energy = current;
            net += b.last_boundary_impulses()[0].linear[0];
            assert!((net + m * p.velocity[0]).abs() < 1e-13);
        }
        errors.push((initial - b.particles()[0].position[0] - exact).abs());
    }
    assert!(
        errors.windows(2).all(|p| p[1] < 0.6 * p[0]) && errors[3] < 1e-6,
        "{errors:?}"
    );
}
