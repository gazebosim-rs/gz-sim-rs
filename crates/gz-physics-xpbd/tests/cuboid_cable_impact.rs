// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, DistanceConstraint, Particle, PrincipalRigidMotion, ReceiverMotion, RigidMobility,
    SoftBody, SolverConfig, XpbdError, step_coupled_rigid,
};
use std::time::Duration;

const MASS: f64 = 0.005;
const RADIUS: f64 = 0.01;
const DT: f64 = 0.01;
const HALF: DVec3 = DVec3::new(1., 0.08, 0.14);
const INERTIA: DVec3 = DVec3::new(0.1, 0.12, 0.16);
fn r0() -> DQuat {
    DQuat::from_rotation_z(-0.4)
}
fn r1() -> DQuat {
    DQuat::from_rotation_z(0.4)
}
fn spin() -> DVec3 {
    DVec3::Z * 80.
}
fn box_shape() -> Boundary {
    Boundary::Cuboid {
        half_extents: HALF.to_array(),
        center: [0.; 3],
        velocity: [0.; 3],
        angular_velocity: spin().to_array(),
    }
}
fn mobility() -> RigidMobility {
    RigidMobility::Principal(PrincipalRigidMotion {
        inverse_mass: 0.5,
        inverse_principal_inertia: INERTIA.recip().to_array(),
        principal_orientation: DQuat::IDENTITY.to_array(),
        start_orientation: r0().to_array(),
        end_orientation: r1().to_array(),
        start_angular_velocity: spin().to_array(),
    })
}
fn radial() -> f64 {
    HALF.x.hypot(HALF.y) + RADIUS - 1e-5
}
fn particle(x: DVec3, mass: f64) -> Particle {
    Particle {
        position: x.to_array(),
        velocity: [0.; 3],
        inverse_mass: mass.recip(),
    }
}
fn cable(points: Vec<Particle>, n: u32, stretch: f64) -> SoftBody {
    let edges = points
        .windows(2)
        .enumerate()
        .map(|(i, p)| DistanceConstraint {
            a: i,
            b: i + 1,
            rest_length: DVec3::from(p[0].position).distance(p[1].position.into()),
            compliance: stretch,
        })
        .collect();
    SoftBody::new(
        points,
        edges,
        SolverConfig {
            gravity: [0.; 3],
            substeps: n,
            iterations: 16,
            particle_radius: RADIUS,
            friction: 0.,
            self_contact: false,
        },
    )
    .unwrap()
}
fn solve(body: &mut SoftBody) -> Result<ReceiverMotion, XpbdError> {
    Ok(step_coupled_rigid(
        &mut [body],
        Duration::from_millis(10),
        &[box_shape()],
        &[Some(mobility())],
    )?[0]
        .unwrap())
}
fn symmetric(reverse: bool, n: u32, stretch: f64) -> SoftBody {
    let mut p: Vec<_> = [-0.03, 0.03]
        .into_iter()
        .map(|z| particle(DVec3::new(radial(), 0., z), MASS))
        .collect();
    if reverse {
        p.reverse();
    }
    cable(p, n, stretch)
}
fn analytic() -> (DVec3, f64, f64) {
    let r = radial();
    let corner = HALF.x.hypot(HALF.y);
    let polar = HALF.y.atan2(HALF.x)
        + ((r * r + corner * corner - RADIUS * RADIUS) / (2. * r * corner)).acos();
    let n = (DVec3::X * r - DQuat::from_rotation_z(-polar) * DVec3::new(HALF.x, HALF.y, 0.))
        .normalize();
    let lever = r * n.y;
    // Equal rows. Symmetric upper/lower contacts cancel transverse torques.
    let speed = 80. * lever / (1. + 2. * MASS * (0.5 + lever * lever / INERTIA.z));
    (n * speed, (0.4 - polar) / 80., lever)
}
fn measures(b: &SoftBody, m: ReceiverMotion) -> (DVec3, DVec3, f64) {
    let com = DVec3::from(m.translation_correction);
    let v = DVec3::from(m.linear_velocity_correction);
    let w = spin() + DVec3::from(m.angular_velocity_correction);
    let q = DQuat::from_array(m.rotation_correction) * r1();
    let angular = q * (INERTIA * (q.conjugate() * w));
    let mut p = 2. * v;
    let mut l = angular + com.cross(p);
    let mut energy = v.length_squared() + 0.5 * w.dot(angular);
    for body in b.particles() {
        let mv = DVec3::from(body.velocity) / body.inverse_mass;
        p += mv;
        l += DVec3::from(body.position).cross(mv);
        energy += 0.5 * DVec3::from(body.velocity).dot(mv);
    }
    (p, l, energy)
}
#[test]
fn symmetric_rotating_corner_matches_coupled_two_contact_closed_form_and_native_clock_refinement() {
    let (expected, toi, lever) = analytic();
    let expected_loss = MASS * expected.length() * 80. * lever;
    let mut original = None;
    for n in [1, 2, 4, 16, 64, 256] {
        for stretch in [0., 0.001] {
            for reverse in [false, true] {
                let mut b = symmetric(reverse, n, stretch);
                let out = solve(&mut b).unwrap();
                let p = b.particles()[usize::from(reverse)];
                let v = DVec3::from(p.velocity);
                let error = v.distance(expected);
                let (momentum, angular, energy) = measures(&b, out);
                let loss = 0.5 * INERTIA.z * 80f64.powi(2) - energy;
                println!(
                    "n={n} compliance={stretch} reverse={reverse} speed={} error={error} dP={} dL={} loss={loss} expected_loss={expected_loss}",
                    v.length(),
                    momentum.length(),
                    angular.distance(spin() * INERTIA)
                );
                assert!(error < 1e-4, "v={v} expected={expected}");
                let initial = DVec3::new(radial(), 0., p.position[2]);
                assert!(DVec3::from(p.position).distance(initial + expected * (DT - toi)) < 1e-5);
                assert!(momentum.length() < 1e-12 && angular.distance(spin() * INERTIA) < 1e-9);
                assert!((loss - expected_loss).abs() < 1e-4);
                assert!(
                    (DVec3::from(b.particles()[usize::from(!reverse)].position)
                        .distance(p.position.into())
                        - 0.06)
                        .abs()
                        < 1e-10
                );
                if n == 1 && stretch == 0. {
                    if let Some((v0, prior)) = original {
                        assert_eq!(v, v0);
                        assert_eq!(out.linear_velocity_correction, prior);
                    }
                    original = Some((v, out.linear_velocity_correction));
                }
            }
        }
    }
}
#[test]
fn oversized_or_loaded_transient_group_refuses_without_publishing() {
    for (count, loaded) in [(5, false), (2, true)] {
        let mut points: Vec<_> = (0..count)
            .map(|i| particle(DVec3::new(radial(), 0., (i as f64 - 2.) * 0.025), MASS))
            .collect();
        if loaded {
            points[0].velocity[2] = 0.3;
        }
        let mut b = cable(points, 1, 0.);
        let initial = b.particles().to_vec();
        assert_eq!(solve(&mut b).unwrap_err(), XpbdError::UnresolvedSweep);
        assert_eq!(b.particles(), initial);
        assert!(b.last_boundary_impulses().is_empty());
    }
}

// Independent two-particle mechanics after the known contact instant.
// Eliminate the ideal length row analytically. The compliant case follows
// Newton's spring law with a fine RK4 integration of relative motion.
fn two_point_reference(
    points: &[Particle],
    normals: [DVec3; 2],
    q: DQuat,
    t: f64,
    compliance: f64,
) -> ([DVec3; 2], f64) {
    let x = [
        DVec3::from(points[0].position),
        DVec3::from(points[1].position),
    ];
    let w = [points[0].inverse_mass, points[1].inverse_mass];
    let axis = (x[0] - x[1]).normalize();
    let g = [w[0] * axis.dot(normals[0]), -w[1] * axis.dot(normals[1])];
    let angular = [x[0].cross(normals[0]), x[1].cross(normals[1])];
    let closing = [
        spin().cross(x[0]).dot(normals[0]),
        spin().cross(x[1]).dot(normals[1]),
    ];
    let k = |i: usize, j: usize| {
        let same = if i == j { w[i] } else { 0. };
        same + 0.5 * normals[i].dot(normals[j])
            + (q.conjugate() * angular[i]).dot(INERTIA.recip() * (q.conjugate() * angular[j]))
            - if compliance == 0. {
                g[i] * g[j] / (w[0] + w[1])
            } else {
                0.
            }
    };
    let det = k(0, 0) * k(1, 1) - k(0, 1).powi(2);
    let j = [
        (closing[0] * k(1, 1) - closing[1] * k(0, 1)) / det,
        (closing[1] * k(0, 0) - closing[0] * k(0, 1)) / det,
    ];
    assert!(j.iter().all(|&v| v > 0.));
    let internal = if compliance == 0. {
        -(g[0] * j[0] + g[1] * j[1]) / (w[0] + w[1])
    } else {
        0.
    };
    let v = [
        w[0] * (normals[0] * j[0] + axis * internal),
        w[1] * (normals[1] * j[1] - axis * internal),
    ];
    let com = (v[0] / w[0] + v[1] / w[1]) / (w[0].recip() + w[1].recip());
    let (mut r, mut u) = (x[0] - x[1], v[0] - v[1]);
    let length = r.length();
    if compliance == 0. {
        let angular = r.cross(u) / length.powi(2);
        u = angular.cross(DQuat::from_scaled_axis(angular * t) * r);
    } else {
        let acc = |x: DVec3| -(w[0] + w[1]) * (x.length() - length) / compliance * x.normalize();
        let h = t / 4096.;
        for _ in 0..4096 {
            let a = acc(r);
            let ub = u + h * 0.5 * a;
            let b = acc(r + h * 0.5 * u);
            let uc = u + h * 0.5 * b;
            let c = acc(r + h * 0.5 * ub);
            let ud = u + h * c;
            let d = acc(r + h * uc);
            r += h / 6. * (u + 2. * ub + 2. * uc + ud);
            u += h / 6. * (a + 2. * b + 2. * c + d);
        }
    }
    (
        [
            com + u * w[0] / (w[0] + w[1]),
            com - u * w[1] / (w[0] + w[1]),
        ],
        0.5 * (j[0] * closing[0] + j[1] * closing[1]),
    )
}

#[test]
fn asymmetric_cable_impacts_match_independent_rigid_dumbbell_and_compliant_spring() {
    let at = DQuat::from_rotation_z(-0.1);
    let mut points = vec![];
    let normals = [0.115_f64, 0.178].map(|a| DVec3::new(a.cos(), a.sin(), 0.));
    for (i, (z, mass)) in [(-0.018, 0.004), (0.018, 0.009)].into_iter().enumerate() {
        points.push(particle(
            at * (DVec3::new(HALF.x, HALF.y, z) + RADIUS * normals[i]),
            mass,
        ));
    }
    for stretch in [0., 0.001] {
        let (velocity, loss_ref) = two_point_reference(
            &points,
            normals.map(|v| at * v),
            at,
            DT - 0.3 / 80.,
            stretch,
        );
        let mut coarse = 0.;
        for n in [1, 4, 256, 4096] {
            let mut forward = cable(points.clone(), n, stretch);
            let mut reverse = cable(points.iter().copied().rev().collect(), n, stretch);
            let rigid = solve(&mut forward).unwrap();
            solve(&mut reverse).unwrap();
            for (a, b) in forward
                .particles()
                .iter()
                .zip(reverse.particles().iter().rev())
            {
                assert!(DVec3::from(a.position).distance(b.position.into()) < 1e-10);
                assert!(DVec3::from(a.velocity).distance(b.velocity.into()) < 1e-9);
            }
            let (p, l, k) = measures(&forward, rigid);
            let particles = forward.particles();
            let dv = particles
                .iter()
                .zip(velocity)
                .map(|(a, v)| DVec3::from(a.velocity).distance(v))
                .fold(0., f64::max);
            let dl = DVec3::from(particles[0].position).distance(particles[1].position.into())
                - forward.constraints()[0].rest_length;
            let spring = if stretch > 0. {
                0.5 * dl.powi(2) / stretch
            } else {
                0.
            };
            let loss = 512. - k - spring;
            println!(
                "asymmetric n={n} compliance={stretch} dv_oracle={dv} dP={} dL={} loss={loss} analytic_loss={loss_ref}",
                p.length(),
                l.distance(spin() * INERTIA)
            );
            assert!(p.length() < 1e-12 && l.distance(spin() * INERTIA) < 3e-6 && loss > 0.);
            assert!(dv < 0.03 && (loss - loss_ref).abs() < 4e-4);
            if n == 1 {
                coarse = dv;
            }
            if n == 4096 {
                assert!(dv < 0.1 * coarse && dv < 0.0025 && (loss - loss_ref).abs() < 3.5e-5);
            }
        }
    }
}

#[test]
fn two_particles_striking_the_same_spinning_face_share_its_axial_velocity() {
    let v = -20.;
    for count in [1, 4, 16] {
        let mut pts = vec![
            particle(DVec3::new(-0.04, 0., 0.25), MASS),
            particle(DVec3::new(0.04, 0., 0.25), MASS),
        ];
        for p in &mut pts {
            p.velocity[2] = v;
        }
        let mut b = cable(pts, count, 0.);
        let out = solve(&mut b).unwrap();
        let expected = 2. * MASS * v / (2. + 2. * MASS);
        let rv = DVec3::from(out.linear_velocity_correction);
        assert!(rv.distance(DVec3::Z * expected) < 1e-9);
        for p in b.particles() {
            assert!(DVec3::from(p.velocity).distance(DVec3::Z * expected) < 1e-8);
        }
        assert!(DVec3::from(out.angular_velocity_correction).length() < 1e-9);
    }
}

#[test]
fn three_and_four_simultaneous_edge_contacts_share_one_receiver_across_registration_orders() {
    let (expected, _, arm) = analytic();
    let normal = expected.normalize();
    for n in [3, 4] {
        let speed = 80. * arm / (1. + n as f64 * MASS * (0.5 + arm * arm / INERTIA.z));
        let points: Vec<_> = (0..n)
            .map(|i| {
                particle(
                    DVec3::new(radial(), 0., (i as f64 - 0.5 * (n - 1) as f64) * 0.04),
                    MASS,
                )
            })
            .collect();
        let mut cable_body = cable(points.clone(), 1, 0.);
        let reference = solve(&mut cable_body).unwrap();
        assert!(DVec3::from(cable_body.particles()[0].velocity).distance(normal * speed) < 1e-4);
        for reversed in [false, true] {
            let mut soft: Vec<_> = points.iter().map(|&p| cable(vec![p], 1, 0.)).collect();
            let mut refs: Vec<_> = soft.iter_mut().collect();
            refs.rotate_left(1);
            if reversed {
                refs.reverse();
            }
            let result = step_coupled_rigid(
                &mut refs,
                Duration::from_millis(10),
                &[box_shape()],
                &[Some(mobility())],
            )
            .unwrap()[0]
                .unwrap();
            assert!(
                DVec3::from(result.linear_velocity_correction)
                    .distance(reference.linear_velocity_correction.into())
                    < 2e-14
            );
            for b in soft {
                assert!(DVec3::from(b.particles()[0].velocity).distance(normal * speed) < 1e-4);
            }
        }
    }
}
#[test]
fn too_many_bilateral_rows_refuse_before_any_particles_or_reactions_commit() {
    // Only the first two particles graze the bar; the other twelve extend the
    // initially straight cable beyond its z end and supply 13 ideal rows.
    let points = (0..14)
        .map(|i| {
            particle(
                DVec3::new(
                    radial(),
                    0.,
                    if i < 2 {
                        0.05 * i as f64
                    } else {
                        0.2 * i as f64
                    },
                ),
                MASS,
            )
        })
        .collect();
    let mut b = cable(points, 1, 0.);
    let start = b.particles().to_vec();
    assert_eq!(solve(&mut b).unwrap_err(), XpbdError::UnresolvedSweep);
    assert_eq!(b.particles(), start);
    assert!(b.last_boundary_impulses().is_empty());
}
