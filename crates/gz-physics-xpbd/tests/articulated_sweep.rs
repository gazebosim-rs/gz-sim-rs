// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    MaterialFeature, Particle, SoftBody, SolverConfig, XpbdError,
    articulated_impact::{ImpactSphere, TranslatingSphere},
};
use parry3d_f64::{
    math::Pose,
    query::{self, PointQuery, ShapeCastOptions},
    shape::{Ball, Segment, Triangle},
};
use std::time::Duration;
fn body(at: &[DVec3], velocity: DVec3) -> SoftBody {
    let nodes = at
        .iter()
        .map(|p| Particle {
            position: p.to_array(),
            velocity: velocity.to_array(),
            inverse_mass: 1.,
        })
        .collect();
    let cfg = SolverConfig {
        gravity: [0.; 3],
        substeps: 2,
        iterations: 4,
        particle_radius: 0.012,
        friction: 0.,
        self_contact: false,
    };
    if at.len() == 2 {
        SoftBody::cable(nodes, 0.01, 0.02, cfg)
    } else {
        SoftBody::cloth(nodes, 2, 0.01, 0.01, 0.02, cfg)
    }
    .unwrap()
}
fn sphere(c: DVec3, v: DVec3) -> TranslatingSphere {
    TranslatingSphere {
        sphere: ImpactSphere {
            center: c.to_array(),
            radius: 0.070,
        },
        velocity: v.to_array(),
    }
}
#[test]
fn independent_parry_translation_casts_match_node_segment_and_face_entry_times() {
    let (mut hits, mut misses, mut worst, mut polished) = (0, 0, 0.0_f64, 0.0_f64);
    let mut kinds = [0; 3];
    for seed in 0..600 {
        let i = seed as f64;
        let origin = DVec3::new(0.3 * (0.21 * i).sin(), -0.12 * (0.31 * i).cos(), 0.25);
        let q = DQuat::from_rotation_x(0.38 * (0.13 * i).sin()) * DQuat::from_rotation_z(0.71 * i);
        let c = q * DVec3::new(0.57 * (0.542 * i).cos(), 0.40 * (0.43 * i).sin(), 0.52) + origin;
        let cm = DVec3::new(0.06 * (0.21 * i).sin(), 0.08 * (0.72 * i).cos(), 0.11);
        let v = cm + q * DVec3::new(0.14 * (0.61 * i).sin(), 0.20 * (0.26 * i).cos(), -1.1);
        let shape = sphere(c, v);
        let grid = if seed % 2 == 0 {
            vec![DVec3::new(-0.41, 0., 0.), DVec3::new(0.41, 0., 0.)]
        } else {
            vec![
                DVec3::new(-0.35, -0.23, 0.),
                DVec3::new(0.35, -0.23, 0.),
                DVec3::new(-0.35, 0.23, 0.),
                DVec3::new(0.35, 0.23, 0.),
            ]
        };
        let p = grid.iter().map(|x| q * *x + origin).collect::<Vec<_>>();
        let material = body(&p, cm);
        let found = material
            .first_translating_sphere_impact(&[shape], Duration::from_secs(1))
            .unwrap();
        let ball = Ball::new(0.082);
        let cast = |other: &dyn parry3d_f64::shape::Shape| {
            query::cast_shapes(
                &Pose::from_translation(c),
                v,
                &ball,
                &Pose::IDENTITY,
                cm,
                other,
                ShapeCastOptions::with_max_time_of_impact(1.),
            )
            .unwrap()
            .map(|v| v.time_of_impact)
        };
        let reference = if p.len() == 2 {
            cast(&Segment::new(p[0], p[1]))
        } else {
            [[0, 1, 3], [0, 3, 2]]
                .into_iter()
                .filter_map(|ids| cast(&Triangle::new(p[ids[0]], p[ids[1]], p[ids[2]])))
                .min_by(f64::total_cmp)
        };
        match (found, reference) {
            (Some(t), Some(r)) => {
                hits += 1;
                worst = worst.max((t.seconds() - r).abs());
                assert!((t.seconds() - r).abs() < 4e-6, "seed {seed}, {t:?} vs {r}");
                let gap = |at: f64| {
                    let q = c + at * (v - cm);
                    let closest =
                        |f: &dyn PointQuery| q.distance(f.project_local_point(q, false).point);
                    if p.len() == 2 {
                        closest(&Segment::new(p[0], p[1])) - 0.082
                    } else {
                        [[0, 1, 3], [0, 3, 2]]
                            .into_iter()
                            .map(|ids| closest(&Triangle::new(p[ids[0]], p[ids[1]], p[ids[2]])))
                            .fold(f64::INFINITY, f64::min)
                            - 0.082
                    }
                };
                let (mut lo, mut hi) = ((r - 2e-6).max(0.), (r + 2e-6).min(1.));
                assert!(gap(lo) > 0. && gap(hi) < 0.);
                for _ in 0..55 {
                    let mid = (lo + hi) * 0.5;
                    if gap(mid) > 0. {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                polished = polished.max((t.seconds() - (lo + hi) * 0.5).abs());
                assert!((t.seconds() - (lo + hi) * 0.5).abs() < 2e-12);
                assert!((t.head().as_secs_f64() - t.seconds()).abs() <= 0.51e-9);
                for (_, f) in t.features() {
                    kinds[match f {
                        MaterialFeature::Vertex(_) => 0,
                        MaterialFeature::Segment(_) => 1,
                        MaterialFeature::Triangle(_) => 2,
                    }] += 1;
                }
            }
            (None, None) => misses += 1,
            (a, b) => panic!("{seed}, {a:?} vs {b:?}"),
        }
    }
    println!(
        "600 independent complete-segment/triangle vs sphere casts, {hits} approaching, {misses} clear, vertex/segment/triangle first features {kinds:?}, Parry cast discrepancy {worst:e} s, independently polished static Parry distance discrepancy {polished:e} s"
    );
    assert!(hits > 120 && misses > 120 && kinds.iter().all(|v| *v > 0));
}
#[test]
fn true_tangent_miss_receding_contact_and_non_integral_time_at_both_tick_boundaries() {
    let ns = Duration::from_millis(1);
    let p = [DVec3::new(-0.5, 0., 0.), DVec3::new(0.5, 0., 0.)];
    let m = body(&p, DVec3::ZERO);
    let center = DVec3::new(0., 0., 0.082);
    assert!(
        m.first_translating_sphere_impact(&[sphere(center, DVec3::Z)], ns)
            .unwrap()
            .is_none()
    );
    assert!(
        m.first_translating_sphere_impact(&[sphere(center, DVec3::Y)], ns)
            .unwrap()
            .is_none()
    );
    assert!(
        m.first_translating_sphere_impact(&[sphere(center - DVec3::Z * 1e-4, DVec3::Z)], ns)
            .is_err()
    );
    for t in [0., 0.4e-9, 1.4e-9, 0.0005, 0.0009999987, 0.001] {
        let c = center + t * DVec3::Z;
        let dynamic = sphere(c, -DVec3::Z);
        let time = m
            .first_translating_sphere_impact(&[dynamic], ns)
            .unwrap()
            .unwrap();
        assert!((time.seconds() - t).abs() < 1e-14);
        let v = -DVec3::Z * time.head().as_secs_f64();
        let actual = [sphere(c + v, DVec3::ZERO).sphere];
        let mut m = m.clone();
        let query = m.sphere_impact_at_detected_barrier(&actual, &time).unwrap();
        assert_eq!(query.sites().len(), 1);
        assert!(query.sites()[0].clearance.abs() <= time.rounding_distance() + 2e-14);
    }
    let mut strained = m.clone();
    let impact = strained
        .sphere_impact_at_barrier(&[sphere(center, DVec3::ZERO).sphere])
        .unwrap();
    impact
        .prepare(&[[0., 0., -1.]], |_, _| [[0.; 3]; 3], 0.1)
        .unwrap()
        .commit();
    // Both cable particles receive equal impulses at the midpoint, so the cable
    // remains uniformly translating. An off-center impulse creates strain rate.
    let c = DVec3::new(0.21, 0., 0.082);
    let pred = strained
        .sphere_impact_at_barrier(&[sphere(c, DVec3::ZERO).sphere])
        .unwrap();
    pred.prepare(&[[0., 0., -4.]], |_, _| [[0.; 3]; 3], 0.2)
        .unwrap()
        .commit();
    assert_eq!(
        strained.first_translating_sphere_impact(&[sphere(c, DVec3::ZERO)], ns),
        Err(XpbdError::NonlinearApproach)
    );
}
