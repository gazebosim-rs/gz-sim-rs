// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{Boundary, Particle, SoftBody, SolverConfig};
use std::time::Duration;
const R: f64 = 0.006;
const M: [f64; 4] = [0.02, 0.08, 0.02, 0.02];
const CENTER: [f64; 3] = [0.08571428571428572, -0.08571428571428572, 0.];
fn cloth(q: DQuat, h: f64, v: DVec3, mu: f64) -> SoftBody {
    let p = (0..4)
        .map(|i| Particle {
            position: (q * DVec3::new((i % 2) as f64 * 0.4 - 0.2, (i / 2) as f64 * 0.4 - 0.2, h))
                .to_array(),
            velocity: (q * v).to_array(),
            inverse_mass: 1. / M[i],
        })
        .collect();
    SoftBody::cloth(
        p,
        2,
        0.,
        0.,
        0.,
        SolverConfig {
            gravity: (q * DVec3::new(0., 0., -9.81)).to_array(),
            substeps: 4,
            iterations: 16,
            friction: mu,
            self_contact: false,
            particle_radius: R,
        },
    )
    .unwrap()
}
fn bench(q: DQuat, dx: f64, w: f64) -> Boundary {
    let center = q * (DVec3::from(CENTER) + DVec3::new(dx, 0., -0.02));
    Boundary::StaticCuboid {
        center: center.to_array(),
        half_extents: [w, w, 0.02],
        orientation: q.to_array(),
    }
}
fn momentum(s: &SoftBody) -> DVec3 {
    s.particles()
        .iter()
        .map(|p| DVec3::from(p.velocity) / p.inverse_mass)
        .sum()
}
#[test]
fn narrow_top_wholly_inside_one_triangle_carries_the_sheet_and_an_off_edge_box_misses() {
    for q in [
        DQuat::IDENTITY,
        DQuat::from_rotation_y(0.37) * DQuat::from_rotation_z(0.41),
    ] {
        let dt = Duration::from_millis(1);
        let mut narrow = cloth(q, 0.026, DVec3::ZERO, 0.3);
        let mut wide = narrow.clone();
        let mut miss = narrow.clone();
        let mut times = [None; 3];
        let mut load = [0.; 3];
        let mut total = [0.; 3];
        let mut heights = [0.; 3];
        for tick in 1..=600 {
            for (i, (body, b)) in [
                (&mut narrow, bench(q, 0., 0.035)),
                (&mut wide, bench(q, 0., 0.5)),
                (&mut miss, bench(q, 0.6, 0.035)),
            ]
            .into_iter()
            .enumerate()
            {
                let before = momentum(body);
                body.step(dt, &[b]).unwrap();
                let j = DVec3::from(body.last_boundary_impulses()[0].linear);
                assert!(
                    (momentum(body) - before + j + q * DVec3::Z * (9.81 * 0.001 * 0.14)).length()
                        < 1e-9
                );
                let z = -(q.conjugate() * j).z;
                if z > 1e-10 {
                    times[i].get_or_insert(tick);
                }
                total[i] += z;
                if tick > 550 {
                    load[i] += z / 0.050;
                }
                if tick == 600 {
                    heights[i] = body
                        .particles()
                        .iter()
                        .map(|p| {
                            (q.conjugate() * DVec3::from(p.position)).z / p.inverse_mass / 0.14
                        })
                        .sum();
                }
            }
        }
        println!("times={times:?}, mean final N={load:?}, J={total:?}, center heights={heights:?}");
        assert_eq!(times[0], times[1]);
        assert!(times[0].is_some());
        assert_eq!(times[2], None);
        assert!((load[0] - 0.14 * 9.81).abs() < 0.02);
        assert!((heights[0] - heights[1]).abs() < 0.001 && heights[2] < -0.15);
    }
}

fn angular(s: &SoftBody, origin: DVec3) -> DVec3 {
    s.particles()
        .iter()
        .map(|p| (DVec3::from(p.position) - origin).cross(DVec3::from(p.velocity) / p.inverse_mass))
        .sum()
}
#[test]
fn faster_than_box_thickness_hits_the_face_and_spends_only_its_coulomb_budget() {
    let q = DQuat::IDENTITY;
    for mu in [0., 0.25] {
        let mut s =
            cloth(q, 0.04, DVec3::new(3., 0., -80.), mu).with_angular_momentum_conservation();
        s.set_resolution(1, 16).unwrap();
        let origin = DVec3::from(CENTER) - DVec3::Z * 0.02;
        let p0 = momentum(&s);
        let l0 = angular(&s, origin);
        let dt = Duration::from_millis(1);
        s.step(dt, &[bench(q, 0., 0.035)]).unwrap();
        let impulse = s.last_boundary_impulses()[0];
        let j = DVec3::from(impulse.linear);
        assert!(j.z < -1.);
        assert!(j.x.hypot(j.y) <= mu * (-j.z) + 1e-8, "{j:?}");
        if mu > 0. {
            assert!(j.x > 0.01);
        }
        let weight = 0.14 * 9.81;
        assert!((momentum(&s) - p0 + j + DVec3::Z * weight * 0.001).length() < 1e-9);
        let residual = angular(&s, origin) - l0 + DVec3::from(impulse.angular);
        println!("friction={mu} impulse={j:?} moment-balance={residual:?}");
        assert!(residual.length() < 1e-10);
        for p in s.particles() {
            let a = DVec3::from(p.position) - origin;
            assert!((a.x.abs() - 0.035).max(a.y.abs() - 0.035) > R);
        }
    }
}

#[test]
fn zero_skin_and_pinned_material_do_not_invent_or_lose_a_face_reaction() {
    let q = DQuat::IDENTITY;
    let dt = Duration::from_millis(1);
    for pinned in [true, false] {
        let seed = cloth(q, 0., DVec3::ZERO, 0.);
        let mut cfg = seed.config();
        cfg.particle_radius = 0.;
        cfg.substeps = 1;
        let points = seed
            .particles()
            .iter()
            .map(|p| Particle {
                inverse_mass: if pinned { 0. } else { p.inverse_mass },
                ..*p
            })
            .collect();
        let mut s = SoftBody::cloth(points, 2, 0., 0., 0., cfg).unwrap();
        s.step(dt, &[bench(q, 0., 0.035)]).unwrap();
        let j = s.last_boundary_impulses()[0].linear[2];
        if pinned {
            assert_eq!(j, 0.);
        } else {
            assert!(j < -1e-7);
        }
    }
}

#[test]
fn skyward_sheet_hits_the_bottom_and_an_in_plane_translation_is_free() {
    let q = DQuat::IDENTITY;
    for (height, v, expected) in [
        (-0.060, DVec3::new(0., 0., 40.), true),
        (0.16, DVec3::new(6., 0., 0.), false),
    ] {
        let mut s = cloth(q, height, v, 0.);
        s.set_resolution(1, 12).unwrap();
        s.step(Duration::from_millis(1), &[bench(q, 0., 0.035)])
            .unwrap();
        let j = s.last_boundary_impulses()[0].linear;
        if expected {
            assert!(j[2] > 1.);
        } else {
            assert_eq!(j, [0.; 3]);
        }
    }
}
