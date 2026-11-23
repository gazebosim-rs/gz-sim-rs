// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    MaterialFeature, Particle, SoftBody, SolverConfig, XpbdError,
    articulated_impact::{ImpactSphere, SphereMotion, SweptSphere},
};
use parry3d_f64::{
    query::PointQuery,
    shape::{Segment, Triangle},
};
use std::time::Duration;
fn body(points: &[DVec3], v: DVec3, g: DVec3) -> SoftBody {
    let nodes = points
        .iter()
        .map(|p| Particle {
            position: p.to_array(),
            velocity: v.to_array(),
            inverse_mass: 2.,
        })
        .collect();
    let cfg = SolverConfig {
        gravity: g.to_array(),
        substeps: 2,
        iterations: 6,
        particle_radius: 0.015625,
        friction: 0.,
        self_contact: false,
    };
    if points.len() == 2 {
        SoftBody::cable(nodes, 0.002, 0.003, cfg)
    } else {
        SoftBody::cloth(nodes, 2, 0.002, 0.003, 0.004, cfg)
    }
    .unwrap()
}
fn rotating(center: DVec3, pivot: DVec3, axis: DVec3, speed: f64) -> SweptSphere {
    SweptSphere {
        sphere: ImpactSphere {
            center: center.to_array(),
            radius: 0.046875,
        },
        motion: SphereMotion::Revolve {
            pivot: pivot.to_array(),
            axis: axis.to_array(),
            angular_speed: speed,
        },
    }
}
#[test]
fn independent_distance_chronology_on_offset_rotating_axes_and_ballistic_material() {
    let (mut hits, mut misses, mut kinds, mut worst) = (0, 0, [0; 3], 0f64);
    for seed in 0..800 {
        let i = seed as f64;
        let q = DQuat::from_euler(
            glam::EulerRot::ZYX,
            0.37 * i,
            0.91 * (0.51 * i).sin(),
            0.66 * (0.69 * i).cos(),
        );
        let origin = DVec3::new(-0.08, 0.12, -0.33);
        let axis = q * DVec3::Z;
        let start = origin + q * DVec3::new(0.45, 0., 0.14);
        let w = if seed % 3 == 0 { -4.7 } else { 5.9 };
        let phi = w * (0.21 + 0.19 * (0.19 * i).cos());
        let (s, c) = phi.sin_cos();
        let width = 0.18 * (0.19 * i).sin();
        let shift = origin
            + q * DVec3::new(
                (0.45 + width) * c,
                (0.45 + width) * s,
                0.14 + 0.21 * (0.43 * i).cos(),
            );
        let r = DQuat::from_euler(glam::EulerRot::XYZ, 0.87 * i, 0.39 * i, 0.68 * i);
        let vertices = if seed % 2 == 0 {
            vec![-DVec3::Z * 0.27, DVec3::Z * 0.27]
        } else {
            vec![
                DVec3::new(-0.21, 0., -0.19),
                DVec3::new(0.21, 0., -0.19),
                DVec3::new(-0.21, 0., 0.19),
                DVec3::new(0.21, 0., 0.19),
            ]
        };
        let x: Vec<_> = vertices.iter().map(|v| shift + r * *v).collect();
        let v = q * DVec3::new(0.05 * (i * 0.29).sin(), -0.07, 0.03);
        let g = q * DVec3::new(0.46 * (0.26 * i).sin(), 0.39 * (0.71 * i).cos(), -0.82);
        let probe = rotating(start, origin, axis, w);
        let distance = |t: f64| {
            let at = origin + DQuat::from_axis_angle(axis, w * t) * (start - origin)
                - v * t
                - 0.5 * g * t * t;
            let geom = |p: &dyn PointQuery| at.distance(p.project_local_point(at, false).point);
            if x.len() == 2 {
                geom(&Segment::new(x[0], x[1])) - 0.0625
            } else {
                [[0, 1, 3], [0, 3, 2]]
                    .map(|ids| geom(&Triangle::new(x[ids[0]], x[ids[1]], x[ids[2]])))
                    .into_iter()
                    .fold(f64::INFINITY, f64::min)
                    - 0.0625
            }
        };
        let actual = body(&x, v, g).first_swept_sphere_impact(&[probe], Duration::from_secs(1));
        if distance(0.) < 0. {
            assert_eq!(actual, Err(XpbdError::PenetratingImpact));
            continue;
        }
        let actual = actual.unwrap();
        let reference = (1..=4096)
            .find(|n| distance(*n as f64 / 4096.) < 0.)
            .map(|n| {
                let mut a = (n - 1) as f64 / 4096.;
                let mut b = n as f64 / 4096.;
                for _ in 0..55 {
                    let t = (a + b) * 0.5;
                    if distance(t) > 0. {
                        a = t;
                    } else {
                        b = t;
                    }
                }
                (a + b) * 0.5
            });
        match (actual, reference) {
            (Some(hit), Some(t)) => {
                let err = (hit.seconds() - t).abs();
                assert!(err < 4e-12, "{seed}: {} vs {t}", hit.seconds());
                worst = worst.max(err);
                hits += 1;
                for (_, f) in hit.features() {
                    kinds[match f {
                        MaterialFeature::Vertex(_) => 0,
                        MaterialFeature::Segment(_) => 1,
                        MaterialFeature::Triangle(_) => 2,
                    }] += 1;
                }
            }
            (None, None) => misses += 1,
            (a, b) => panic!("rotating source {seed}: {a:?} vs {b:?}"),
        }
    }
    assert!(
        hits > 120 && misses > 120 && kinds.iter().all(|n| *n > 0),
        "{hits}, {misses}, {kinds:?}"
    );
    println!(
        "800 rotating and ballistically moving source configurations, {hits} clear-to-contact, {misses} clear throughout, {} initially penetrated and refused. Vertex/edge/face first contacts {kinds:?}; largest discrepancy from the first 4,096-sample independently bracketed Parry static distance crossing {worst:e} s.",
        800 - hits - misses
    );
}
#[test]
fn multiple_turns_exact_graze_near_graze_and_invalid_axes_are_distinguished() {
    let w = 11.;
    let q = DQuat::from_rotation_z(-0.73);
    let r = 0.5;
    let gap = 0.0625;
    let sphere = rotating(q * (r * DVec3::X), DVec3::ZERO, DVec3::Z, w);
    for delta in [-1e-9_f64, 0., 1e-9] {
        let separation = r + gap + delta;
        let soft = body(
            &[
                separation * DVec3::X - DVec3::Z,
                separation * DVec3::X + DVec3::Z,
            ],
            DVec3::ZERO,
            DVec3::ZERO,
        );
        let actual = soft
            .first_swept_sphere_impact(&[sphere], Duration::from_secs(2))
            .unwrap();
        if delta >= 0. {
            assert!(actual.is_none(), "{delta:e}: {actual:?}");
        } else {
            // Stable half-angle identity for a small angular clearance.
            let angle = 2.
                * (((gap - (separation - r)) * (gap + (separation - r)) / (4. * r * separation))
                    .sqrt())
                .asin();
            let expected = (0.73 - angle) / w;
            assert!((actual.unwrap().seconds() - expected).abs() < 2e-12);
        }
        let bad = rotating(q * (r * DVec3::X), DVec3::ZERO, 2. * DVec3::Z, w);
        assert_eq!(
            soft.first_swept_sphere_impact(&[bad], Duration::from_secs(1)),
            Err(XpbdError::InvalidBoundary)
        );
        assert_eq!(
            soft.first_swept_sphere_impact(&[sphere], Duration::from_secs(3)),
            Err(XpbdError::ImpactBudget)
        );
    }
}
