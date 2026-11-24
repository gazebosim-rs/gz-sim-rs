// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    MaterialFeature, Particle, SoftBody, SolverConfig, XpbdError,
    articulated_impact::{ImpactSphere, TranslatingSphere},
};
use parry3d_f64::{
    query::PointQuery,
    shape::{Segment, Triangle},
};
use std::time::Duration;
fn body(points: &[DVec3], v: DVec3, g: DVec3, radius: f64) -> SoftBody {
    let nodes = points
        .iter()
        .map(|p| Particle {
            position: p.to_array(),
            velocity: v.to_array(),
            inverse_mass: 1.,
        })
        .collect();
    let cfg = SolverConfig {
        gravity: g.to_array(),
        substeps: 2,
        iterations: 6,
        friction: 0.,
        particle_radius: radius,
        self_contact: false,
    };
    if points.len() == 2 {
        SoftBody::cable(nodes, 0.002, 0.003, cfg)
    } else {
        SoftBody::cloth(nodes, 2, 0.002, 0.003, 0.004, cfg)
    }
    .unwrap()
}
fn sphere(center: DVec3, velocity: DVec3, radius: f64) -> TranslatingSphere {
    TranslatingSphere {
        sphere: ImpactSphere {
            center: center.to_array(),
            radius,
        },
        velocity: velocity.to_array(),
    }
}
#[test]
fn ballistic_geometry_agrees_with_independent_static_parry_distance_crossings() {
    let (mut hits, mut misses, mut kinds, mut worst) = (0, 0, [0; 3], 0.0_f64);
    for seed in 0..1000 {
        let i = seed as f64;
        let origin = DVec3::new(0.33 * (0.13 * i).sin(), -0.17 * (0.29 * i).cos(), 0.32);
        let q = DQuat::from_euler(
            glam::EulerRot::ZYX,
            0.73 * i,
            0.73 * (0.21 * i).sin(),
            0.42 * (0.35 * i).cos(),
        );
        let vertices = if seed % 2 == 0 {
            vec![DVec3::new(-0.41, 0., 0.), DVec3::new(0.41, 0., 0.)]
        } else {
            vec![
                DVec3::new(-0.35, -0.23, 0.),
                DVec3::new(0.35, -0.23, 0.),
                DVec3::new(-0.35, 0.23, 0.),
                DVec3::new(0.35, 0.23, 0.),
            ]
        };
        let p: Vec<_> = vertices.iter().map(|&x| origin + q * x).collect();
        let c = origin + q * DVec3::new(0.51 * (0.37 * i).cos(), 0.32 * (0.58 * i).sin(), 0.45);
        let cm = q * DVec3::new(0.09 * (0.68 * i).sin(), 0.11 * (0.41 * i).cos(), 0.21);
        let sv = cm + q * DVec3::new(0.32 * (0.53 * i).cos(), 0.21 * (0.61 * i).sin(), -0.80);
        let accel = q * DVec3::new(
            0.65 * (0.19 * i).cos(),
            0.6 * (0.22 * i).sin(),
            1.75 * (0.31 * i).sin(),
        );
        let soft = body(&p, cm, accel, 0.012);
        let found = soft
            .first_ballistic_sphere_impact(&[sphere(c, sv, 0.07)], Duration::from_secs(1))
            .unwrap();
        let gap = |t: f64| {
            let p0 = c + (sv - cm) * t - 0.5 * accel * t * t;
            let closest = |s: &dyn PointQuery| p0.distance(s.project_local_point(p0, false).point);
            if p.len() == 2 {
                closest(&Segment::new(p[0], p[1])) - 0.082
            } else {
                [[0, 1, 3], [0, 3, 2]]
                    .map(|ids| closest(&Triangle::new(p[ids[0]], p[ids[1]], p[ids[2]])))
                    .into_iter()
                    .fold(f64::INFINITY, f64::min)
                    - 0.082
            }
        };
        let sample = 1. / 2048.;
        let reference = (1..=2048).find(|n| gap(*n as f64 * sample) < 0.).map(|n| {
            let mut a = (n - 1) as f64 * sample;
            let mut b = n as f64 * sample;
            for _ in 0..56 {
                let t = 0.5 * (a + b);
                if gap(t) > 0. {
                    a = t;
                } else {
                    b = t;
                }
            }
            (a + b) * 0.5
        });
        match (found, reference) {
            (Some(t), Some(reference)) => {
                let error = (t.seconds() - reference).abs();
                worst = worst.max(error);
                assert!(
                    error < 3e-12,
                    "rotated source {seed}, root {:?}, reference {reference}",
                    t.seconds()
                );
                hits += 1;
                for (_, feature) in t.features() {
                    kinds[match feature {
                        MaterialFeature::Vertex(_) => 0,
                        MaterialFeature::Segment(_) => 1,
                        MaterialFeature::Triangle(_) => 2,
                    }] += 1;
                }
            }
            (None, None) => misses += 1,
            (actual, reference) => {
                panic!("rotated source {seed}: actual {actual:?}, reference {reference:?}")
            }
        }
    }
    assert!(hits > 120 && misses > 120 && kinds.iter().all(|n| *n > 0), "{hits} {misses} {kinds:?}");
    println!(
        "1000 rotated curved segment/triangle approaches, {hits} impacts and {misses} misses, vertex/segment/face counts {kinds:?}, maximum time difference {worst:e} s against 2048-sample independent Parry static distance with bracket polishing"
    );
}
#[test]
fn curved_graze_has_no_impulse_and_a_thirty_microsecond_transient_has_the_expected_entry() {
    let p = [-DVec3::X, DVec3::X];
    let soft = body(&p, 0.5 * DVec3::Z, -8. * DVec3::Z, 0.03125);
    let h = Duration::from_millis(125);
    let apex = 0.0625;
    for clearance in [-1e-9_f64, 0., 1e-9] {
        let b = sphere(
            DVec3::Z * (0.125 + 0.015625 + clearance),
            DVec3::ZERO,
            0.09375,
        );
        let hit = soft.first_ballistic_sphere_impact(&[b], h).unwrap();
        if clearance < 0. {
            let reference = apex - (-clearance / 4.).sqrt();
            let cut = hit.unwrap();
            assert!((cut.seconds() - reference).abs() < 2e-12);
            let mut warm = soft.clone();
            warm.advance_articulated_free_flight(cut.head()).unwrap();
            assert_eq!(
                warm.sphere_impact_at_detected_barrier(&[b.sphere], &cut)
                    .unwrap()
                    .sites()
                    .len(),
                1
            );
            let mut changed = b.sphere;
            changed.radius += 1e-12;
            assert!(matches!(
                warm.sphere_impact_at_detected_barrier(&[changed], &cut),
                Err(XpbdError::UnresolvedImpact)
            ));
            let mut changed = b.sphere;
            changed.center[2] += 1e-7;
            assert!(matches!(
                warm.sphere_impact_at_detected_barrier(&[changed], &cut),
                Err(XpbdError::UnresolvedImpact)
            ));
        } else {
            assert!(hit.is_none(), "clearance {clearance:e}, {hit:?}");
        }
    }
}
#[test]
fn common_absolute_material_reference_retains_exact_closed_form_over_ten_thousand_steps() {
    let p = [DVec3::new(0.31, 0.42, -0.17), DVec3::new(0.67, 0.58, 0.21)];
    let v = DVec3::new(0.13, -0.16, 0.57);
    let g = DVec3::new(0.07, 0.11, -9.81);
    let mut soft = body(&p, v, g, 0.02);
    let seed = soft.particles().to_vec();
    let mut base = soft.clone();
    let dt = Duration::from_micros(57);
    for k in 0..10_000 {
        soft.advance_articulated_flight_from(&seed, k * dt, (k + 1) * dt)
            .unwrap();
    }
    base.advance_articulated_free_flight(10_000 * dt).unwrap();
    assert_eq!(soft.particles(), base.particles());
    let before = soft.particles().to_vec();
    let mut wrong = seed;
    wrong[0].position[0] += 1e-8;
    assert_eq!(
        soft.advance_articulated_flight_from(&wrong, 10_000 * dt, 10_001 * dt),
        Err(XpbdError::NonlinearApproach)
    );
    assert_eq!(soft.particles(), before);
}
