// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_xpbd::{Particle, SoftBody, SolverConfig, XpbdError};
use std::time::Duration;

fn particle(x: f64, y: f64, z: f64, vz: f64, w: f64) -> Particle {
    Particle {
        position: [x, y, z],
        velocity: [0., 0., vz],
        inverse_mass: w,
    }
}
fn surface(p: Vec<Particle>, t: Vec<[usize; 3]>, substeps: u32) -> SoftBody {
    SoftBody::new(
        p,
        vec![],
        SolverConfig {
            gravity: [0.; 3],
            substeps,
            iterations: 1,
            particle_radius: 0.01,
            self_contact: true,
            ..Default::default()
        },
    )
    .unwrap()
    .with_self_contact_surface(t)
    .unwrap()
}
fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 2e-10, "{a:.15} vs {b:.15}");
}
fn face(w: f64) -> Vec<Particle> {
    vec![
        particle(-1., -1., 0., 0., w),
        particle(1., -1., 0., 0., w),
        particle(0., 2., 0., 0., w),
        particle(0., 0., 0.1, -4., 1.),
    ]
}

#[test]
fn high_speed_vertex_triangle_crossing_obeys_barycentric_effective_mass() {
    for n in [1, 2, 4, 8] {
        for fixed in [true, false] {
            let mut b = surface(face(if fixed { 0. } else { 1. }), vec![[0, 1, 2]], n);
            b.step(Duration::from_millis(50), &[]).unwrap();
            let v = if fixed { 0. } else { -1. };
            for (i, p) in b.particles().iter().enumerate() {
                near(p.velocity[2], v);
                near(p.position[2], 0.03 * v + if i == 3 { 0.02 } else { 0. });
            }
        }
    }
}

fn crossing(near_miss: bool) -> Vec<Particle> {
    let offset = if near_miss { 1.0201 } else { 0. };
    vec![
        particle(-1., 0., 0., 0., 1.),
        particle(1., 0., 0., 0., 1.),
        particle(0., -1., -2., 0., 1.),
        particle(offset, -1., 0.1, -4., 1.),
        particle(offset, 1., 0.1, -4., 1.),
        particle(offset + 0.5, 0.5, 2.1, -4., 1.),
    ]
}

#[test]
fn skew_edges_collide_at_interior_points_and_a_tip_clearance_survives() {
    for n in [1, 2, 4, 8] {
        for miss in [true, false] {
            let mut s = surface(crossing(miss), vec![[2, 0, 1], [3, 4, 5]], n);
            s.step(Duration::from_millis(50), &[]).unwrap();
            for (i, p) in s.particles().iter().enumerate() {
                let expected = if i == 2 || (miss && i < 3) {
                    0.
                } else if i == 5 || miss {
                    -4.
                } else {
                    -2.
                };
                near(p.velocity[2], expected);
                if !miss && [0, 1, 3, 4].contains(&i) {
                    near(p.position[2], if i < 3 { -0.06 } else { -0.04 });
                }
            }
            let momentum = s.particles().iter().map(|p| p.velocity[2]).sum::<f64>();
            near(momentum, -12.);
        }
    }
}

#[test]
fn canonical_triangle_order_and_orientation_do_not_affect_edge_impact() {
    let mut a = surface(crossing(false), vec![[0, 1, 2], [3, 4, 5]], 2);
    let mut b = surface(crossing(false), vec![[4, 3, 5], [2, 1, 0]], 2);
    a.step(Duration::from_millis(50), &[]).unwrap();
    b.step(Duration::from_millis(50), &[]).unwrap();
    assert_eq!(a.particles(), b.particles());
}

#[test]
fn receding_and_grazing_face_keep_prescribed_normal_and_tangent_velocity() {
    for n in [1, 2, 4] {
        for (z, velocity) in [
            (0.020_001, [20., 0., 0.]),
            (0.02, [20., 0., 0.]),
            (0.02, [0., 0., 4.]),
        ] {
            let mut p = face(0.);
            p[3].position = [-0.3, 0., z];
            p[3].velocity = velocity;
            let mut b = surface(p, vec![[0, 1, 2]], n);
            b.step(Duration::from_millis(40), &[]).unwrap();
            assert_eq!(b.particles()[3].velocity, velocity);
            near(b.particles()[3].position[0], -0.3 + velocity[0] * 0.04);
        }
    }
}

#[test]
fn interpenetrating_immobile_surfaces_and_ambiguous_coincidence_refuse_atomically() {
    for z in [0.0, 0.009] {
        let mut particles = face(0.);
        particles[3] = particle(0., 0., z, 0., 0.);
        let mut b = surface(particles.clone(), vec![[0, 1, 2]], 1);
        assert_eq!(
            b.step(Duration::from_millis(10), &[]),
            Err(XpbdError::UnresolvedSurface)
        );
        assert_eq!(b.particles(), particles);
        assert!(b.last_boundary_impulses().is_empty());
    }
}

#[test]
fn invalid_topologies_refuse_before_simulation() {
    let make = || {
        SoftBody::new(
            face(1.),
            vec![],
            SolverConfig {
                self_contact: true,
                ..Default::default()
            },
        )
        .unwrap()
    };
    for tris in [
        vec![],
        vec![[0, 1, 4]],
        vec![[0, 1, 0]],
        vec![[0, 1, 2], [2, 1, 0]],
        vec![[0, 1, 2], [0, 1, 3]],
    ] {
        assert!(matches!(
            make().with_self_contact_surface(tris),
            Err(XpbdError::InvalidConstraint)
        ));
    }
}

#[test]
fn unequal_masses_and_glancing_barycentrics_conserve_spatial_momentum() {
    use glam::DVec3;
    let mut particles = face(1.);
    particles[0].inverse_mass = 0.5;
    particles[2].inverse_mass = 2.;
    particles[3].position = [0.19, -0.20, 0.10];
    particles[3].velocity = [0.5, 0., -4.];
    // At t=0.02 the point is (0.2,-0.2). The impact impulse weights
    // follow from independent affine coordinates of the horizontal triangle.
    let c = (1. - 0.2) / 3.;
    let a = (1. - c - 0.2) / 2.;
    let weights = [a, 1. - a - c, c, -1.];
    let denom = (0..4)
        .map(|i| weights[i] * weights[i] * particles[i].inverse_mass)
        .sum::<f64>();
    let momentum = |p: &[Particle]| {
        p.iter()
            .fold((DVec3::ZERO, DVec3::ZERO, 0.), |(lin, ang, ke), p| {
                let v = DVec3::from(p.velocity);
                let mv = v / p.inverse_mass;
                (
                    lin + mv,
                    ang + DVec3::from(p.position).cross(mv),
                    ke + 0.5 * v.dot(mv),
                )
            })
    };
    for n in [1, 2, 4, 8] {
        let mut surface = surface(particles.clone(), vec![[0, 1, 2]], n);
        surface.step(Duration::from_millis(20), &[]).unwrap();
        let actual = surface.particles();
        for (i, p) in actual.iter().enumerate() {
            near(
                p.velocity[2],
                particles[i].velocity[2] - weights[i] * p.inverse_mass * 4. / denom,
            );
            near(p.velocity[0], particles[i].velocity[0]);
        }
        let (p0, l0, k0) = momentum(&particles);
        let (p1, l1, k1) = momentum(actual);
        near(p0.distance(p1), 0.);
        near(l0.distance(l1), 0.);
        near(k0 - k1, 8. / denom);
    }
}

#[test]
fn external_projection_cannot_publish_a_new_self_crossing() {
    use gz_physics_xpbd::Boundary;
    let mut p = face(0.);
    p[3] = particle(0., 0., 0.05, 0., 1.);
    let mut body = surface(p.clone(), vec![[0, 1, 2]], 1);
    let ceiling = Boundary::Plane {
        point: [0., 0., 0.01],
        normal: [0., 0., -1.],
        velocity: [0.; 3],
    };
    assert_eq!(
        body.step(Duration::from_millis(2), &[ceiling]),
        Err(XpbdError::UnresolvedSurface)
    );
    assert_eq!(body.particles(), p);
    assert!(body.last_boundary_impulses().is_empty());
}

#[test]
fn shell_vertex_capacity_and_unrepresentable_clearance_return_typed_errors() {
    let mut many = face(1.);
    many.resize(4097, many[3]);
    let cfg = SolverConfig {
        self_contact: true,
        gravity: [0.; 3],
        ..Default::default()
    };
    let b = SoftBody::new(many, vec![], cfg).unwrap();
    assert!(matches!(
        b.with_self_contact_surface(vec![[0, 1, 2]]),
        Err(XpbdError::SurfaceBudget)
    ));
    let mut p = face(1.);
    p[3].position[2] = 1e13;
    let mut b = surface(p.clone(), vec![[0, 1, 2]], 1);
    assert_eq!(
        b.step(Duration::from_millis(2), &[]),
        Err(XpbdError::Numerical)
    );
    assert_eq!(b.particles(), p);
}

#[test]
fn vertical_grid_is_a_rigid_translation_without_spurious_contacts() {
    let n = 22;
    let p = (0..n * n)
        .map(|i| Particle {
            position: [0., (i % n) as f64 * 0.10, (i / n) as f64 * 0.10],
            velocity: [0.3, 0.2, 0.],
            inverse_mass: 1.,
        })
        .collect::<Vec<_>>();
    let cfg = SolverConfig {
        gravity: [0.; 3],
        substeps: 1,
        iterations: 1,
        particle_radius: 0.01,
        self_contact: true,
        ..Default::default()
    };
    let mut body = SoftBody::cloth(p.clone(), n, 0., 0., 0.01, cfg).unwrap();
    body.step(Duration::from_millis(10), &[]).unwrap();
    for (a, b) in body.particles().iter().zip(p) {
        near(a.position[0], b.position[0] + 0.003);
        near(a.position[1], b.position[1] + 0.002);
        near(a.position[2], b.position[2]);
    }
}

#[test]
fn loose_probes_sweep_one_another_and_do_not_add_energy_to_startup_overlap() {
    let mut p = face(0.);
    p[3] = particle(3., 0., 0.1, -4., 1.);
    p.push(particle(3., 0., 0., 0., 1.));
    let mut body = surface(p.clone(), vec![[0, 1, 2]], 1);
    body.step(Duration::from_millis(50), &[]).unwrap();
    near(body.particles()[3].velocity[2], -2.);
    near(body.particles()[4].velocity[2], -2.);
    p[3] = particle(3., 0., 0.005, 0., 1.);
    let mut body = surface(p, vec![[0, 1, 2]], 1);
    body.step(Duration::from_millis(2), &[]).unwrap();
    near(
        body.particles()[3].position[2] - body.particles()[4].position[2],
        0.02,
    );
    assert_eq!(body.particles()[3].velocity, [0.; 3]);
    assert_eq!(body.particles()[4].velocity, [0.; 3]);
}

#[test]
fn simultaneous_pairs_exceeding_projection_budget_keep_the_previous_commit() {
    use gz_physics_xpbd::Boundary;
    let boundary = Boundary::Plane {
        point: [0., 0., -10.],
        normal: [0., 0., 1.],
        velocity: [0.; 3],
    };
    for count in [4, 97] {
        let mut p = face(0.);
        p.truncate(3);
        for i in 0..count {
            let x = 3. + i as f64;
            p.push(particle(x, 0., 0.1, -4., 1.));
            p.push(particle(x, 0., 0., 0., 1.));
        }
        let mut body = surface(p, vec![[0, 1, 2]], 1);
        body.step(Duration::from_millis(1), &[boundary]).unwrap();
        let before = body.particles().to_vec();
        let previous_reactions = body.last_boundary_impulses().to_vec();
        let result = body.step(Duration::from_millis(50), &[]);
        if count == 4 {
            result.unwrap();
            for p in &body.particles()[3..] {
                near(p.velocity[2], -2.);
            }
        } else {
            assert_eq!(result, Err(XpbdError::SurfaceBudget));
            assert_eq!(body.particles(), before);
            assert_eq!(body.last_boundary_impulses(), previous_reactions);
        }
    }
}

#[test]
fn dense_interpenetrating_probes_refuse_the_distance_query_budget() {
    let mut p = face(0.);
    p.truncate(3);
    for i in 0..370 {
        p.push(particle(3. + i as f64 * 0.00004, 0., 0., 0., 1.));
    }
    let mut b = surface(p.clone(), vec![[0, 1, 2]], 1);
    assert_eq!(
        b.step(Duration::from_millis(1), &[]),
        Err(XpbdError::SurfaceBudget)
    );
    assert_eq!(b.particles(), p);
}

#[test]
fn grazing_initial_rotating_face_without_a_separating_plane_refuses() {
    // The probe touches the lower triangle edge. The opposite corner rotates
    // up, reducing clearance quadratically while the initial normal speed is zero.
    let p = vec![
        particle(-1., 0., 0., 0., 0.),
        particle(1., 0., 0., 0., 0.),
        particle(0., 1., 0., 0.5, 0.),
        particle(0., 0., 0.02, 0., 1.),
    ];
    let mut b = surface(p.clone(), vec![[0, 1, 2]], 1);
    assert_eq!(
        b.step(Duration::from_millis(10), &[]),
        Err(XpbdError::UnresolvedSurface)
    );
    assert_eq!(b.particles(), p);
}

#[test]
fn preinterlocked_triangles_without_close_boundary_features_are_rejected() {
    // Intersection is a segment in both face interiors, well away from vertex
    // samples and without an edge-edge intersection.
    let points = [
        [-2., -2., 0.],
        [2., -2., 0.],
        [0., 2., 0.],
        [0., -0.5, -1.],
        [0., -0.5, 1.],
        [0., 0.8, 1.],
    ];
    let make = |z: f64| {
        points
            .iter()
            .enumerate()
            .map(|(i, &[x, y, h])| particle(x, y, h + if i < 3 { 0. } else { z }, 0., 1.))
            .collect::<Vec<_>>()
    };
    let cfg = SolverConfig {
        self_contact: true,
        gravity: [0.; 3],
        particle_radius: 0.01,
        ..Default::default()
    };
    let topology = vec![[0, 1, 2], [3, 4, 5]];
    let bad = SoftBody::new(make(0.), vec![], cfg)
        .unwrap()
        .with_self_contact_surface(topology.clone());
    assert!(matches!(bad, Err(XpbdError::UnresolvedSurface)));
    let mut clean = surface(make(2.), topology, 1);
    clean.step(Duration::from_millis(1), &[]).unwrap();
}

#[test]
fn coplanar_remote_triangle_bounds_can_overlap_without_actual_intersection() {
    let p = vec![
        particle(0., 0., 0., 0., 1.),
        particle(2., 0., 0., 0., 1.),
        particle(0., 2., 0., 0., 1.),
        particle(1.1, 1.1, 0., 0., 1.),
        particle(3., 1.1, 0., 0., 1.),
        particle(1.1, 3., 0., 0., 1.),
    ];
    let mut b = surface(p.clone(), vec![[0, 1, 2], [3, 4, 5]], 1);
    b.step(Duration::from_millis(1), &[]).unwrap();
    assert_eq!(b.particles(), p);
}

#[test]
fn a_floor_impulse_reaches_the_touching_upper_layer_and_balances_momentum() {
    use gz_physics_xpbd::Boundary;
    let falling = |x, y, z| particle(x, y, z, -1., 1.);
    // A moving face and the tip above it land together. Only the face touches
    // the floor directly. Measure the external impulse on the complete system.
    let mut shell = surface(
        vec![
            falling(-1., -1., 0.011),
            falling(1., -1., 0.011),
            falling(0., 2., 0.011),
            falling(0., 0., 0.031),
            falling(0.4, 0.2, 0.4),
            falling(-0.3, 0.45, 0.49),
        ],
        vec![[0, 1, 2], [3, 4, 5]],
        1,
    )
    .with_self_contact_friction(0.3)
    .unwrap();
    let floor = Boundary::Plane {
        point: [0.; 3],
        normal: [0., 0., 1.],
        velocity: [0.; 3],
    };
    let mut reaction = 0.;
    for _ in 0..2 {
        shell.step(Duration::from_millis(1), &[floor]).unwrap();
        reaction += shell.last_boundary_impulses()[0].linear[2];
    }
    let p = shell.particles();
    assert!(p[..3].iter().all(|p| p.position[2] >= 0.01 - 1e-10));
    assert!(p[3].position[2] >= 0.03 - 1e-9);
    assert!(p[3].velocity[2].abs() < 1e-8);
    let momentum: f64 = p.iter().map(|p| p.velocity[2]).sum();
    assert!((momentum + 6. + reaction).abs() < 1e-10);
}
