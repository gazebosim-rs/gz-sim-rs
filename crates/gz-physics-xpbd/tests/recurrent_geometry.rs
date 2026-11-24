// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{Particle, SoftBody, SolverConfig, articulated_impact::ImpactSphere};
use parry3d_f64::{
    query::PointQuery,
    shape::{Segment, Triangle},
};
use std::time::Duration;
fn material(x: &[DVec3]) -> SoftBody {
    let mut config = SolverConfig {
        gravity: [0.; 3],
        particle_radius: 0.014,
        friction: 0.,
        self_contact: false,
        ..Default::default()
    };
    config.substeps = 2;
    let p = x
        .iter()
        .map(|p| Particle {
            position: p.to_array(),
            velocity: [0.; 3],
            inverse_mass: 2.,
        })
        .collect();
    if x.len() == 2 {
        SoftBody::cable(p, 0.002, 0.003, config)
    } else {
        SoftBody::cloth(p, 2, 0.002, 0.003, 0.004, config)
    }
    .unwrap()
}
fn pad(c: DVec3) -> ImpactSphere {
    ImpactSphere {
        center: c.to_array(),
        radius: 0.05,
    }
}
fn clearance(x: &[DVec3], c: DVec3) -> f64 {
    let d = if x.len() == 2 {
        c.distance(Segment::new(x[0], x[1]).project_local_point(c, false).point)
    } else {
        [[0, 1, 3], [0, 3, 2]]
            .into_iter()
            .map(|i| {
                c.distance(
                    Triangle::new(x[i[0]], x[i[1]], x[i[2]])
                        .project_local_point(c, false)
                        .point,
                )
            })
            .fold(f64::INFINITY, f64::min)
    };
    d - 0.064
}
#[test]
fn independent_deforming_simplex_sweeps_include_transient_entries_between_clear_endpoints() {
    let (mut hits, mut misses, mut transient, mut worst) = (0, 0, 0, 0f64);
    for n in 0..480 {
        let u = n as f64;
        let q = DQuat::from_euler(glam::EulerRot::ZYX, 0.68 * u, 0.32 * u, -0.29 * u);
        let origin = DVec3::new(0.13, -0.41, 0.27);
        let shape = if n % 2 == 0 {
            vec![DVec3::new(-0.52, 0., 0.), DVec3::new(0.52, 0., 0.)]
        } else {
            vec![
                DVec3::new(-0.48, -0.35, 0.),
                DVec3::new(0.48, -0.35, 0.),
                DVec3::new(-0.48, 0.35, 0.),
                DVec3::new(0.48, 0.35, 0.),
            ]
        };
        let a: Vec<_> = shape.iter().map(|&x| origin + q * x).collect();
        let b: Vec<_> = shape
            .iter()
            .enumerate()
            .map(|(j, &x)| {
                let v = j as f64;
                origin
                    + q * (x + DVec3::new(
                        0.25 * (u + v).sin(),
                        0.10 * (0.43 * u + v).cos(),
                        0.20 * (u + v).cos(),
                    ))
            })
            .collect();
        let from = origin + q * DVec3::new(0.66 * (0.47 * u).cos(), 0.44 * (0.60 * u).sin(), 0.36);
        let to = from + q * DVec3::new(0.22 * (0.34 * u).sin(), 0.40 * (0.81 * u).cos(), -0.72);
        let gap = |t| {
            let x: Vec<_> = a.iter().zip(&b).map(|(&a, &b)| a.lerp(b, t)).collect();
            clearance(&x, from.lerp(to, t))
        };
        assert!(gap(0.) > 0.);
        let mut true_entry = None;
        for i in 1..=1024 {
            let mut hi = i as f64 / 1024.;
            if gap(hi) < 0. {
                let mut lo = (i - 1) as f64 / 1024.;
                for _ in 0..48 {
                    let m = (lo + hi) / 2.;
                    if gap(m) <= 0. { hi = m } else { lo = m }
                }
                true_entry = Some(hi);
                break;
            }
        }
        let guess = material(&a)
            .deforming_chord_candidate(&material(&b), &[pad(from)], &[pad(to)])
            .unwrap();
        assert_eq!(guess.is_some(), true_entry.is_some(), "seed {n}");
        if let Some(t) = true_entry {
            hits += 1;
            transient += usize::from(gap(1.) > 0.);
            let error = (t - guess.unwrap()).abs();
            worst = worst.max(error);
            assert!(error < 2e-8, "seed {n} discrepancy {error}");
        } else {
            misses += 1;
        }
    }
    assert!(hits > 100 && misses > 100 && transient > 100);
    println!(
        "480 independent linearly deforming mesh/finite-pad sweeps: {hits} crossings, {misses} misses, {transient} transients ending clear, maximum crossing-fraction discrepancy {worst:e}"
    );
}
#[test]
fn bracket_requires_one_nanosecond_and_exact_radius_topology_and_relatively_swept_shell() {
    let a = [DVec3::new(-0.5, 0., 0.), DVec3::new(0.5, 0., 0.)];
    let b = material(&a);
    let before = pad(DVec3::new(0., 0., 0.064 + 0.4e-9));
    let after = pad(DVec3::new(2e-9, 0., 0.064 - 0.6e-9));
    let sec = Duration::from_secs(2);
    let contact = b
        .bracketed_articulated_return(&[before], &b, &[after], Duration::from_nanos(1), sec)
        .unwrap();
    assert_eq!(contact.head(), sec);
    assert!((contact.seconds() - 2. - 0.4e-9).abs() < 1e-14);
    assert!((contact.rounding_distance() - 5.0f64.sqrt() * 1e-9).abs() < 2e-17);
    let farther = ImpactSphere {
        radius: 0.051,
        ..after
    };
    assert!(
        b.bracketed_articulated_return(&[before], &b, &[farther], Duration::from_nanos(1), sec)
            .is_err()
    );
    assert!(
        b.bracketed_articulated_return(&[before], &b, &[after], Duration::from_nanos(2), sec)
            .is_err()
    );
    assert!(
        b.deforming_chord_candidate(
            &material(&[a[0], a[1], a[0] + DVec3::Y, a[1] + DVec3::Y]),
            &[before],
            &[after]
        )
        .is_err()
    );
}
