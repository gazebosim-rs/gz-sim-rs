// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{Particle, SoftBody, SolverConfig, XpbdError};
use std::time::Duration;

fn pair(edge: bool, substeps: u32, mu: f64, rotation: DQuat) -> SoftBody {
    let xyz = if edge {
        [
            [-1., 0., 0.],
            [1., 0., 0.],
            [0., -1., -2.],
            [0., -1., 0.021],
            [0., 1., 0.021],
            [0.5, 0.5, 2.021],
        ]
    } else {
        [
            [-1., -1., 0.],
            [1., -1., 0.],
            [0., 2., 0.],
            [0.02, -0.01, 0.021],
            [0.4, 0.3, 0.4],
            [-0.3, 0.45, 0.49],
        ]
    };
    let p = xyz
        .into_iter()
        .enumerate()
        .map(|(i, pt)| Particle {
            position: (rotation * DVec3::from(pt)).to_array(),
            velocity: (if i < 3 {
                DVec3::ZERO
            } else {
                rotation * DVec3::new(0.8, -0.2, -0.5)
            })
            .to_array(),
            inverse_mass: [1., 0.7, 1.4, 0.8, 1.2, 0.65][i],
        })
        .collect();
    SoftBody::new(
        p,
        vec![],
        SolverConfig {
            gravity: [0.; 3],
            substeps,
            iterations: 2,
            particle_radius: 0.01,
            self_contact: true,
            friction: 0.,
        },
    )
    .unwrap()
    .with_self_contact_surface(vec![[0, 1, 2], [3, 4, 5]])
    .unwrap()
    .with_self_contact_friction(mu)
    .unwrap()
}

fn state(p: &[Particle]) -> (DVec3, DVec3, f64) {
    let mut linear = DVec3::ZERO;
    let mut angular = DVec3::ZERO;
    let mut e = 0.;
    for q in p {
        let v = DVec3::from(q.velocity);
        let momentum = v / q.inverse_mass;
        linear += momentum;
        angular += DVec3::from(q.position).cross(momentum);
        e += 0.5 * v.dot(momentum);
    }
    (linear, angular, e)
}

#[test]
fn frictional_free_vertex_face_and_edge_edge_transfer_tangential_momentum_and_dissipate() {
    for edge in [false, true] {
        for n in [1, 2, 4, 8] {
            let mut previous = f64::INFINITY;
            for mu in [0., 0.2, 2.] {
                let mut sheet = pair(edge, n, mu, DQuat::IDENTITY);
                let (p0, l0, e0) = state(sheet.particles());
                let side = state(&sheet.particles()[3..]).0;
                sheet.step(Duration::from_millis(4), &[]).unwrap();
                let (p1, l1, e1) = state(sheet.particles());
                let j = state(&sheet.particles()[3..]).0 - side;
                let jt = j.truncate().length();
                println!(
                    "edge={edge} substeps={n} mu={mu}: Jn={:.7} Jt={jt:.7} Lerr={:.3e} Perr={:.3e} energy {e0:.7}→{e1:.7}",
                    j.z,
                    l0.distance(l1),
                    p0.distance(p1)
                );
                assert!(p0.distance(p1) < 1e-10 && l0.distance(l1) < 1e-11);
                assert!(j.z > 0. && e1 < e0 && e1 < previous);
                let receipt = sheet.last_self_contact_impulse();
                assert!(receipt.normal > 0. && receipt.friction_work >= 0.);
                assert!(receipt.tangent <= mu * receipt.normal + 1e-12);
                assert_eq!(receipt.tangent > 0., mu > 0.);
                assert!(receipt.friction_work <= e0 - e1 + 1e-10);
                if mu > 0. {
                    assert!(jt > 0.01);
                } else {
                    assert!(jt < 1e-7);
                }
                previous = e1;
            }
        }
    }
}

#[test]
fn rigid_rotation_of_glancing_encounter_rotates_every_committed_result() {
    let q = DQuat::from_euler(glam::EulerRot::XYZ, 0.6, -0.27, 0.9);
    for e in [true, false] {
        let mut base = pair(e, 1, 0.6, DQuat::IDENTITY);
        let mut turned = pair(e, 1, 0.6, q);
        base.step(Duration::from_millis(4), &[]).unwrap();
        turned.step(Duration::from_millis(4), &[]).unwrap();
        for (a, b) in base.particles().iter().zip(turned.particles()) {
            assert!((q * DVec3::from(a.position)).distance(b.position.into()) < 1e-11);
            assert!((q * DVec3::from(a.velocity)).distance(b.velocity.into()) < 1e-8);
        }
    }
}

#[test]
fn unsupported_friction_and_unattached_point_refuse_during_construction() {
    for mu in [-1., f64::NAN, f64::INFINITY] {
        assert!(matches!(
            pair(false, 1, 0., DQuat::IDENTITY).with_self_contact_friction(mu),
            Err(XpbdError::InvalidConfig)
        ));
    }
    let p = pair(false, 1, 0., DQuat::IDENTITY);
    let s = SoftBody::new(p.particles().to_vec(), vec![], p.config())
        .unwrap()
        .with_self_contact_surface(vec![[0, 1, 2]])
        .unwrap();
    assert!(matches!(
        s.with_self_contact_friction(0.2),
        Err(XpbdError::InvalidConstraint)
    ));
}

#[test]
fn late_edge_sliding_refuses_without_publishing_particles_or_impulses() {
    let points = [
        [-1., 0., 0.],
        [1., 0., 0.],
        [0., 1., 0.],
        [0.2, -0.012, 0.016],
        [0.45, -0.3, 0.28],
        [-0.05, -0.4, 0.35],
    ];
    let particles = points
        .into_iter()
        .enumerate()
        .map(|(i, position)| Particle {
            position,
            velocity: if i < 3 { [0.; 3] } else { [0., -0.9, -1.3] },
            inverse_mass: if i < 3 { 0. } else { 1. },
        })
        .collect();
    let mut body = SoftBody::new(
        particles,
        vec![],
        pair(false, 1, 0., DQuat::IDENTITY).config(),
    )
    .unwrap()
    .with_self_contact_surface(vec![[0, 1, 2], [3, 4, 5]])
    .unwrap()
    .with_self_contact_friction(0.5)
    .unwrap();
    let before = body.particles().to_vec();
    let receipt = body.last_self_contact_impulse();
    assert!(matches!(
        body.step(Duration::from_millis(120), &[]),
        Err(XpbdError::UnresolvedSurface)
    ));
    assert_eq!(body.particles(), before);
    assert_eq!(body.last_self_contact_impulse(), receipt);
}

#[test]
fn a_uniform_observer_velocity_preserves_contact_and_tangential_work() {
    let mut base = pair(false, 2, 0.4, DQuat::IDENTITY);
    let boost = DVec3::new(3.1, -2.8, 0.7);
    let shift = DVec3::new(2.4, -10.2, -1.8);
    let p = base
        .particles()
        .iter()
        .cloned()
        .map(|mut p| {
            p.position = (DVec3::from(p.position) + shift).to_array();
            p.velocity = (DVec3::from(p.velocity) + boost).to_array();
            p
        })
        .collect();
    let mut boosted = SoftBody::new(p, vec![], base.config())
        .unwrap()
        .with_self_contact_surface(vec![[2, 1, 0], [5, 4, 3]])
        .unwrap()
        .with_self_contact_friction(0.4)
        .unwrap();
    let dt = Duration::from_millis(4);
    base.step(dt, &[]).unwrap();
    boosted.step(dt, &[]).unwrap();
    for (a, b) in base.particles().iter().zip(boosted.particles()) {
        assert!(
            (DVec3::from(a.position) + shift + dt.as_secs_f64() * boost)
                .distance(b.position.into())
                < 1e-10
        );
        assert!((DVec3::from(a.velocity) + boost).distance(b.velocity.into()) < 1e-8);
    }
    let a = base.last_self_contact_impulse();
    let b = boosted.last_self_contact_impulse();
    assert!(
        (a.friction_work - b.friction_work).abs() < 1e-10 && (a.normal - b.normal).abs() < 1e-8
    );
    // Successful zero-duration queries start a fresh empty impulse sample.
    boosted.step(Duration::ZERO, &[]).unwrap();
    assert_eq!(boosted.last_self_contact_impulse(), Default::default());
}

#[test]
fn free_shell_geometric_depenetration_preserves_momenta_without_a_friction_load() {
    let initial = pair(false, 1, 0.6, DQuat::IDENTITY);
    let particles = initial
        .particles()
        .iter()
        .copied()
        .enumerate()
        .map(|(i, mut q)| {
            q.velocity = if i < 3 { [0.; 3] } else { [0.8, 0., 0.] };
            if i >= 3 {
                q.position[2] -= 0.003;
            }
            q
        })
        .collect();
    let mut b = SoftBody::new(particles, vec![], initial.config())
        .unwrap()
        .with_self_contact_surface(vec![[0, 1, 2], [3, 4, 5]])
        .unwrap()
        .with_self_contact_friction(0.6)
        .unwrap();
    let (p0, l0, e0) = state(b.particles());
    b.step(Duration::from_micros(20), &[]).unwrap();
    let (p1, l1, e1) = state(b.particles());
    let receipt = b.last_self_contact_impulse();
    assert!(p0.distance(p1) < 1e-11 && l0.distance(l1) < 1e-11 && e1 <= e0 + 1e-12);
    // Any contact load here comes only from velocity closing against a
    // tilted corrected face. Pure overlap cleanup contributes no impulse.
    assert!(receipt.normal < 0.002 && receipt.tangent < 0.0012);
    let points = b.particles();
    let q = |i: usize| parry3d_f64::math::Vector::from_array(points[i].position);
    use parry3d_f64::query::PointQuery;
    let face = parry3d_f64::shape::Triangle::new(q(0), q(1), q(2));
    assert!(face.distance_to_local_point(q(3), false) >= 0.02 - 1e-9);
}

#[test]
fn one_material_vertex_sliding_on_a_supported_face_matches_coulomb_impulse() {
    // A single 0.7 kg tip arrives exactly at the final time. Its neighbours
    // are above the face. Reflection symmetry removes the y impulse.
    let dt = Duration::from_millis(10);
    for substeps in [1, 2, 4, 8] {
        let points = [
            [-1., -1., 0.],
            [1., -1., 0.],
            [0., 2., 0.],
            [0.15, 0., 0.022],
            [-0.2, -0.3, 0.22],
            [-0.2, 0.3, 0.22],
        ];
        let p = points
            .into_iter()
            .enumerate()
            .map(|(i, position)| Particle {
                position,
                velocity: if i < 3 { [0.; 3] } else { [0.4, 0., -0.2] },
                inverse_mass: if i < 3 {
                    0.
                } else if i == 3 {
                    1. / 0.7
                } else {
                    1.
                },
            })
            .collect();
        let mut b = SoftBody::new(
            p,
            vec![],
            pair(false, substeps, 0., DQuat::IDENTITY).config(),
        )
        .unwrap()
        .with_self_contact_surface(vec![[0, 1, 2], [3, 4, 5]])
        .unwrap()
        .with_self_contact_friction(0.1)
        .unwrap();
        let before = state(&b.particles()[3..]).0;
        b.step(dt, &[]).unwrap();
        let j = state(&b.particles()[3..]).0 - before;
        // A unit x skin impulse also changes the tip's normal velocity:
        // its thickness-offset couple is -0.01 e_y, the tip arm is
        // 2/3 * 0.35 e_x in x, and I_yy=2/3*(0.35²+0.198²).
        let cross = 0.01 * 0.35 / (0.35f64.powi(2) + 0.198f64.powi(2));
        let expected = 0.7 * 0.2 / (1. - 0.1 * cross);
        println!(
            "supported vertex, {substeps} substeps: measured normal {:.12}, analytic {expected:.12} N s, tangent {:.12} N s",
            j.z, j.x
        );
        assert!(
            (j.z - expected).abs() < 1e-11
                && (j.x + 0.1 * expected).abs() < 1e-11
                && j.y.abs() < 1e-11,
            "{j:?}"
        );
        let contacts = b.last_self_contact_impulse();
        assert!((contacts.tangent - 0.1 * expected).abs() < 1e-11 && contacts.friction_work > 0.);
    }
}
