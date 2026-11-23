// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics_xpbd::{Boundary, DistanceConstraint, Particle, SoftBody, SolverConfig, XpbdError};
use std::time::Duration;

fn p(position: [f64; 3], inverse_mass: f64) -> Particle {
    Particle {
        position,
        inverse_mass,
        velocity: [0.0; 3],
    }
}
fn isolated() -> SolverConfig {
    SolverConfig {
        gravity: [0.0; 3],
        particle_radius: 0.0,
        friction: 0.0,
        ..Default::default()
    }
}
fn floor(v: [f64; 3]) -> Boundary {
    Boundary::Plane {
        point: [0.0; 3],
        normal: [0.0, 0.0, 2.0],
        velocity: v,
    }
}
fn ball() -> Boundary {
    Boundary::Sphere {
        center: [0.0; 3],
        radius: 0.5,
        velocity: [0.0; 3],
        angular_velocity: [0.0; 3],
    }
}
fn near(a: DVec3, b: DVec3, tol: f64) {
    assert!(
        (a - b).length() < tol,
        "{a:?} versus {b:?}, tolerance {tol}"
    );
}

#[test]
fn compliance_matches_static_load_across_step_sizes_and_iterations() {
    for ns in [2_000_003, 11_000_000] {
        for substeps in [1, 2, 4] {
            let particles = vec![p([0.0; 3], 0.0), p([0.0, 0.0, -1.0981], 1.0)];
            let constraint = DistanceConstraint {
                a: 0,
                b: 1,
                rest_length: 1.0,
                compliance: 0.01,
            };
            let mut body = SoftBody::new(
                particles,
                vec![constraint],
                SolverConfig {
                    substeps,
                    iterations: 3,
                    ..Default::default()
                },
            )
            .unwrap();
            for _ in 0..40 {
                body.step(Duration::from_nanos(ns), &[]).unwrap();
            }
            assert_eq!(body.particles()[0].position, [0.0; 3]);
            assert!((body.particles()[1].position[2] + 1.0981).abs() < 1e-11);
            assert!(DVec3::from(body.particles()[1].velocity).length() < 1e-10);
        }
    }
}

#[test]
fn rectangular_cloth_sags_without_excessive_stretch_or_moving_pins() {
    let particles = (0..16)
        .map(|i| {
            p(
                [(i % 4) as f64 * 0.1, (i / 4) as f64 * 0.1, 0.0],
                if i < 4 { 0.0 } else { 1.0 },
            )
        })
        .collect::<Vec<_>>();
    let mut body = SoftBody::cloth(
        particles.clone(),
        4,
        1e-7,
        1e-6,
        0.02,
        SolverConfig::default(),
    )
    .unwrap();
    // 24 structural edges, 9 shear diagonals and 21 two-triangle hinges.
    assert_eq!(body.constraints().len(), 33);
    assert_eq!(body.bending_constraints().len(), 21);
    for _ in 0..120 {
        body.step(Duration::from_millis(2), &[]).unwrap();
    }
    assert_eq!(body.particles()[..4], particles[..4]);
    assert!(body.particles()[15].position[2] < -0.1);
    for c in body.constraints().iter().filter(|c| c.compliance == 1e-7) {
        let d = DVec3::from(body.particles()[c.a].position)
            - DVec3::from(body.particles()[c.b].position);
        assert!((d.length() / c.rest_length - 1.0).abs() < 0.02);
    }
}

#[test]
fn free_cable_preserves_total_linear_momentum_and_replays_bit_for_bit() {
    let mut points = vec![
        p([0.0; 3], 1.0),
        p([0.4, 0.2, 0.0], 0.5),
        p([0.7, -0.1, 0.1], 3.0),
    ];
    points[0].velocity = [-0.3, 0.5, 0.12];
    points[2].velocity = [0.2, 0.4, -0.2];
    let momentum = |p: &[Particle]| {
        p.iter()
            .map(|p| DVec3::from(p.velocity) / p.inverse_mass)
            .sum::<DVec3>()
    };
    let initial = momentum(&points);
    let mut body = SoftBody::cable(points, 1e-5, 1e-3, isolated()).unwrap();
    let mut replay = body.clone();
    for _ in 0..100 {
        for sim in [&mut body, &mut replay] {
            sim.step(Duration::from_nanos(1_000_007), &[]).unwrap();
        }
    }
    assert_eq!(body.particles(), replay.particles());
    near(momentum(body.particles()), initial, 1e-10);
}

#[test]
fn self_contact_separates_non_neighbours_with_no_spurious_initial_energy() {
    let cfg = SolverConfig {
        self_contact: true,
        particle_radius: 0.1,
        substeps: 1,
        ..isolated()
    };
    let initial = vec![p([0.0; 3], 1.0), p([0.0; 3], 0.5)];
    let mut body = SoftBody::new(initial.clone(), vec![], cfg).unwrap();
    body.step(Duration::from_millis(1), &[]).unwrap();
    let [a, b] = body.particles() else {
        panic!("two particles")
    };
    assert!(((DVec3::from(a.position) - DVec3::from(b.position)).length() - 0.2).abs() < 1e-14);
    near(
        DVec3::from(a.position) + DVec3::from(b.position) * 2.0,
        DVec3::ZERO,
        1e-14,
    );
    near(DVec3::from(a.velocity), DVec3::ZERO, 1e-10);
    near(DVec3::from(b.velocity), DVec3::ZERO, 1e-10);
    let linked = vec![p([0.0; 3], 1.0), p([0.08, 0.0, 0.0], 1.0)];
    let mut cable = SoftBody::cable(linked.clone(), 0.0, 0.0, cfg).unwrap();
    cable.step(Duration::from_millis(10), &[]).unwrap();
    assert_eq!(cable.particles(), linked);
}

#[test]
fn sphere_sweep_and_initial_overlap_are_inelastic() {
    let mut fast = p([0.0, 0.0, 2.0], 0.5);
    fast.velocity = [0.0, 0.0, -4.0];
    let cfg = SolverConfig {
        substeps: 1,
        ..isolated()
    };
    let mut body = SoftBody::new(vec![fast], vec![], cfg).unwrap();
    body.step(Duration::from_secs(1), &[ball()]).unwrap();
    near(body.particles()[0].position.into(), DVec3::Z * 0.5, 1e-12);
    near(body.particles()[0].velocity.into(), DVec3::ZERO, 1e-12);
    near(
        body.last_boundary_impulses()[0].linear.into(),
        -DVec3::Z * 8.0,
        1e-12,
    );
    let mut inside = SoftBody::new(vec![p([0.0; 3], 1.0)], vec![], cfg).unwrap();
    inside.step(Duration::from_millis(1), &[ball()]).unwrap();
    assert_eq!(inside.particles()[0].position, [0.5, 0.0, 0.0]);
    near(inside.particles()[0].velocity.into(), DVec3::ZERO, 1e-10);
}

#[test]
fn moving_plane_weight_and_spinning_sphere_friction_have_measurable_reactions() {
    for substeps in [1, 2, 4] {
        let cfg = SolverConfig {
            substeps,
            particle_radius: 0.05,
            ..Default::default()
        };
        let mut particle = p([0.0, 0.0, 0.05], 0.5);
        particle.velocity = [0.0, 0.0, 0.2];
        let mut supported = SoftBody::new(vec![particle], vec![], cfg).unwrap();
        supported
            .step(Duration::from_millis(20), &[floor([0.0, 0.0, 0.2])])
            .unwrap();
        near(
            supported.particles()[0].position.into(),
            DVec3::Z * 0.054,
            1e-12,
        );
        near(
            supported.last_boundary_impulses()[0].linear.into(),
            -DVec3::Z * (2.0 * 9.81 * 0.02),
            1e-11,
        );
    }
    let mut body = SoftBody::new(
        vec![p([0.0, 0.0, 0.51], 1.0)],
        vec![],
        SolverConfig {
            substeps: 1,
            friction: 0.5,
            ..Default::default()
        },
    )
    .unwrap();
    let sphere = Boundary::Sphere {
        center: [0.0; 3],
        radius: 0.5,
        velocity: [0.0; 3],
        angular_velocity: [0.0, 2.0, 0.0],
    };
    body.step(Duration::from_millis(10), &[sphere]).unwrap();
    assert!((body.particles()[0].velocity[0] - 0.5 * 0.0981).abs() < 1e-13);
    assert!(body.last_boundary_impulses()[0].angular[1] < 0.0);
}

#[test]
fn offset_particle_impact_reports_both_components_of_orbital_momentum_change() {
    let mut particle = p([0.1, 0.2, 0.6], 0.4);
    particle.velocity = [1.2, -0.3, -3.5];
    let mass = particle.inverse_mass.recip();
    let origin = DVec3::new(0.1, -0.1, 0.0);
    let sphere = Boundary::Sphere {
        center: origin.to_array(),
        radius: 0.5,
        velocity: [0.7, 0.0, 0.1],
        angular_velocity: [0.4, 0.0, 0.5],
    };
    for substeps in [1, 2, 4] {
        let mut body = SoftBody::new(
            vec![particle],
            vec![],
            SolverConfig {
                substeps,
                friction: 0.5,
                particle_radius: 0.04,
                ..isolated()
            },
        )
        .unwrap();
        body.step(Duration::from_millis(120), &[sphere]).unwrap();
        let end = body.particles()[0];
        let p0 = mass * DVec3::from(particle.velocity);
        let p1 = mass * DVec3::from(end.velocity);
        let l0 = (DVec3::from(particle.position) - origin).cross(p0);
        let l1 = (DVec3::from(end.position) - origin).cross(p1);
        let r = body.last_boundary_impulses()[0];
        assert!(DVec3::from(r.linear).length() > 1.0);
        near(r.linear.into(), p0 - p1, 1e-10);
        near(r.angular.into(), l0 - l1, 1e-10);
    }
}

#[test]
fn errors_leave_last_valid_state_and_reactions_intact() {
    let cfg = SolverConfig::default();
    let mut body =
        SoftBody::new(vec![p([0.0, 0.0, cfg.particle_radius], 1.0)], vec![], cfg).unwrap();
    body.step(Duration::from_millis(2), &[floor([0.0; 3])])
        .unwrap();
    let state = body.particles().to_vec();
    let reactions = body.last_boundary_impulses().to_vec();
    assert_eq!(
        body.step(Duration::from_nanos(2), &[]),
        Err(XpbdError::InvalidStep)
    );
    assert_eq!(
        body.step(
            Duration::from_secs(1),
            &[Boundary::Plane {
                point: [0.0; 3],
                normal: [0.0; 3],
                velocity: [0.0; 3]
            }]
        ),
        Err(XpbdError::InvalidBoundary)
    );
    assert_eq!(
        body.step(Duration::MAX, &[floor([0.0, 0.0, f64::MAX])]),
        Err(XpbdError::Numerical)
    );
    assert_eq!(body.particles(), state);
    assert_eq!(body.last_boundary_impulses(), reactions);
    body.step(Duration::ZERO, &[floor([0.0; 3])]).unwrap();
    assert_eq!(body.particles(), state);
    assert_eq!(body.last_boundary_impulses()[0].linear, [0.0; 3]);
}

#[test]
fn folded_rest_panel_resists_applied_bend_in_any_world_frame() {
    use glam::DQuat;
    let axis = (DVec3::X + DVec3::Y).normalize();
    let rotation = DQuat::from_axis_angle(DVec3::new(1.0, 0.4, -0.7).normalize(), 1.1);
    let translation = DVec3::new(2.0, -0.4, 0.9);
    for fold in [-0.73, 0.58] {
        let corner = DQuat::from_axis_angle(axis, fold) * DVec3::Y;
        let mut particles =
            [DVec3::ZERO, DVec3::X, corner, DVec3::X + DVec3::Y].map(|v| p(v.to_array(), 0.0));
        particles[2].inverse_mass = 1.0;
        let cfg = SolverConfig {
            substeps: 1,
            iterations: 10,
            ..isolated()
        };
        let panel =
            |p: &[Particle], bend| SoftBody::cloth(p.to_vec(), 2, 0.0, 0.0, bend, cfg).unwrap();
        let mut rest = panel(&particles, 0.0);
        assert!((rest.bending_constraints()[0].rest_angle.abs() - fold.abs()).abs() < 1e-12);
        rest.step(Duration::from_millis(10), &[]).unwrap();
        for (rest, original) in rest.particles().iter().zip(particles) {
            near(rest.position.into(), original.position.into(), 1e-14);
            near(rest.velocity.into(), DVec3::ZERO, 1e-12);
        }
        particles[2].velocity = (2.0 * axis.cross(corner)).to_array();
        let mut stiff = panel(&particles, 0.0);
        let mut supple = panel(&particles, 0.01);
        let dt = Duration::from_millis(10);
        stiff.step(dt, &[]).unwrap();
        supple.step(dt, &[]).unwrap();
        let error = |body: &SoftBody| (DVec3::from(body.particles()[2].position) - corner).length();
        assert!(error(&stiff) < 1e-9, "{}", error(&stiff));
        assert!(error(&supple) > 1e-2);
        let transformed = particles.map(|p| Particle {
            position: (translation + rotation * DVec3::from(p.position)).to_array(),
            velocity: (rotation * DVec3::from(p.velocity)).to_array(),
            ..p
        });
        let mut world = panel(&transformed, 0.01);
        world.step(dt, &[]).unwrap();
        for _ in 0..20 {
            for (particle, reference) in world.particles().iter().zip(supple.particles()) {
                near(
                    particle.position.into(),
                    translation + rotation * DVec3::from(reference.position),
                    1e-11,
                );
                near(
                    particle.velocity.into(),
                    rotation * DVec3::from(reference.velocity),
                    1e-10,
                );
            }
            world.step(dt, &[]).unwrap();
            supple.step(dt, &[]).unwrap();
        }
    }
}

#[test]
fn invalid_geometry_configuration_and_overflow_are_typed() {
    let x = vec![p([0.0; 3], 0.0), p([1.0, 0.0, 0.0], 0.0)];
    assert!(matches!(
        SoftBody::new(vec![], vec![], isolated()),
        Err(XpbdError::InvalidParticle)
    ));
    for bad in [
        p([0.0; 3], -1.0),
        p([f64::NAN; 3], 1.0),
        p([0.0; 3], f64::from_bits(1)),
    ] {
        assert!(matches!(
            SoftBody::new(vec![bad], vec![], isolated()),
            Err(XpbdError::InvalidParticle)
        ));
    }
    for bad in [
        SolverConfig {
            substeps: 0,
            ..isolated()
        },
        SolverConfig {
            iterations: 0,
            ..isolated()
        },
        SolverConfig {
            friction: -1.0,
            ..isolated()
        },
        SolverConfig {
            self_contact: true,
            ..isolated()
        },
        SolverConfig {
            particle_radius: f64::INFINITY,
            ..isolated()
        },
    ] {
        assert!(matches!(
            SoftBody::new(x.clone(), vec![], bad),
            Err(XpbdError::InvalidConfig)
        ));
    }
    assert!(matches!(
        SoftBody::cable(x[..1].to_vec(), 0.0, 0.0, isolated()),
        Err(XpbdError::InvalidConstraint)
    ));
    for (a, b, c) in [(0, 2, 0.0), (2, 0, 0.0), (0, 0, 0.0), (0, 1, -1.0)] {
        assert_eq!(
            DistanceConstraint::between(a, b, &x, c),
            Err(XpbdError::InvalidConstraint)
        );
    }
    let c = DistanceConstraint::between(0, 1, &x, 0.0).unwrap();
    for constraints in [
        vec![c, c],
        vec![DistanceConstraint { a: 2, ..c }],
        vec![DistanceConstraint {
            rest_length: 0.0,
            ..c
        }],
    ] {
        assert!(matches!(
            SoftBody::new(x.clone(), constraints, isolated()),
            Err(XpbdError::InvalidConstraint)
        ));
    }
    assert!(matches!(
        SoftBody::new(
            vec![p([f64::MAX; 3], 1.0), p([-f64::MAX; 3], 1.0)],
            vec![c],
            isolated()
        ),
        Err(XpbdError::InvalidConstraint)
    ));
    for columns in [0, 1, 2, 3] {
        assert!(matches!(
            SoftBody::cloth(x.clone(), columns, 0.0, 0.0, 0.0, isolated()),
            Err(XpbdError::InvalidConstraint)
        ));
    }
    let collinear = (0..4).map(|i| p([i as f64, 0.0, 0.0], 1.0)).collect();
    assert!(matches!(
        SoftBody::cloth(collinear, 2, 0.0, 0.0, 0.0, isolated()),
        Err(XpbdError::InvalidConstraint)
    ));
    let mut pinned = SoftBody::new(x.clone(), vec![c], isolated()).unwrap();
    assert_eq!(pinned.config().substeps, isolated().substeps);
    pinned.step(Duration::from_secs(1), &[]).unwrap();
    assert_eq!(pinned.particles(), x);
    let mut bad = SoftBody::new(
        vec![Particle {
            velocity: [f64::MAX; 3],
            ..x[0]
        }],
        vec![],
        isolated(),
    )
    .unwrap();
    assert_eq!(bad.step(Duration::MAX, &[]), Err(XpbdError::Numerical));
    let sphere = |radius, angular_velocity| Boundary::Sphere {
        center: [0.0; 3],
        velocity: [0.0; 3],
        radius,
        angular_velocity,
    };
    for b in [
        sphere(0.0, [0.0; 3]),
        sphere(1.0, [f64::NAN; 3]),
        floor([f64::NAN; 3]),
    ] {
        assert_eq!(
            pinned.step(Duration::ZERO, &[b]),
            Err(XpbdError::InvalidBoundary)
        );
    }
}
