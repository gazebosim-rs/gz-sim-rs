// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::Boundary;

fn trial<const CULL: bool>(
    shapes: &[Boundary],
    points: &[Particle],
    ends: &[DVec3],
    interval: Interval,
    curves: &[(usize, DVec3, DVec3)],
) -> (Vec<u64>, usize) {
    let mut p = points.to_vec();
    let mut x = ends.to_vec();
    let mut b: Vec<_> = shapes
        .iter()
        .copied()
        .map(|s| PreparedBoundary::new(s).unwrap())
        .collect();
    for &(i, a, z) in curves {
        b[i].path_endpoints = Some((a, z));
    }
    let mut r: Vec<_> = (0..b.len())
        .map(|boundary| BoundaryImpulse {
            boundary,
            ..Default::default()
        })
        .collect();
    let mut contacts = Contacts::default();
    for _ in 0..3 {
        project_points::<CULL>(
            &mut x,
            &mut p,
            &mut b,
            &mut contacts,
            &mut r,
            interval,
            (false, false),
        )
        .unwrap();
    }
    let mut sig = Vec::new();
    for (v, p) in x.iter().zip(p) {
        for a in v.to_array().into_iter().chain(p.velocity) {
            sig.push(a.to_bits());
        }
    }
    for c in &contacts.items {
        sig.push(c.particle as u64);
        sig.push(c.boundary as u64);
        for a in c.normal.to_array().into_iter().chain([
            c.correction,
            c.projected_impulse,
            c.initial_overlap,
        ]) {
            sig.push(a.to_bits());
        }
        assert!(c.remaining.is_none());
    }
    for v in r {
        for j in v.linear.into_iter().chain(v.angular) {
            sig.push(j.to_bits());
        }
    }
    (sig, contacts.items.len())
}
fn sphere(x: [f64; 3], radius: f64) -> Boundary {
    Boundary::Sphere {
        center: x,
        velocity: [0.; 3],
        radius,
        angular_velocity: [0.; 3],
    }
}
fn verify(
    shapes: &[Boundary],
    p: &[Particle],
    x: &[DVec3],
    i: Interval,
    curves: &[(usize, DVec3, DVec3)],
) -> usize {
    let expected = trial::<false>(shapes, p, x, i, curves);
    assert_eq!(expected, trial::<true>(shapes, p, x, i, curves));
    expected.1
}
#[test]
fn earlier_projection_can_reach_an_originally_separated_surface() {
    let mut shapes = vec![
        Boundary::Plane {
            point: [0., 0., 1.],
            normal: [0., 0., 1.],
            velocity: [0.; 3],
        },
        sphere([0., 0., 0.73], 0.1),
    ];
    for j in 2..32 {
        shapes.push(sphere([j as f64, 1., 1.], 0.2));
    }
    let p = vec![
        Particle {
            position: [0., 0., 0.],
            velocity: [0.; 3],
            inverse_mass: 1.
        };
        67
    ];
    let i = Interval {
        t0: 0.,
        t1: 0.01,
        h: 0.01,
        radius: 0.012,
    };
    let x = vec![DVec3::ZERO; p.len()];
    let b: Vec<_> = shapes
        .iter()
        .copied()
        .map(|s| PreparedBoundary::new(s).unwrap())
        .collect();
    let excluded = Field::new(p.len(), &b, i)
        .unwrap()
        .block(&x[..BLOCK], &p[..BLOCK]);
    assert_eq!(excluded.skip.count_ones(), 31);
    assert!(excluded.skip & 2 != 0);
    assert_eq!(verify(&shapes, &p, &x, i, &[]), 2 * p.len());
}
#[test]
fn swept_chunk_masks_preserve_exact_narrow_phase_at_multiple_scales() {
    let mut rng = 42_735u64;
    let mut next = || {
        rng = rng
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((rng >> 11) as f64) * (1. / 9007199254740992.)
    };
    let mut total = 0;
    for n in 0..360 {
        let scale = [0.001, 1., 1000.][n % 3];
        let origin = DVec3::splat(if n % 5 == 0 { 100000. * scale } else { 0. });
        let h = 0.003;
        let interval = Interval {
            t0: 0.001,
            t1: 0.004,
            h,
            radius: 0.013 * scale,
        };
        let mut shapes = Vec::new();
        let mut segments = Vec::new();
        for j in 0..[8, 17, 33, 64][n % 4] {
            let c = origin + DVec3::new(j as f64 * 0.8, 0., 0.) * scale;
            let center = c.to_array();
            let v = [(next() - 0.5) * 6. * scale, 0., 0.];
            let b = if j % 2 == 0 {
                Boundary::Sphere {
                    center,
                    radius: 0.17 * scale,
                    velocity: v,
                    angular_velocity: [0.; 3],
                }
            } else {
                Boundary::Capsule {
                    center,
                    axis: [0.1, 1., 0.2],
                    half_length: 0.36 * scale,
                    radius: 0.11 * scale,
                    velocity: v,
                    angular_speed: 0.,
                }
            };
            shapes.push(b);
            if j % 5 == 0 {
                segments.push((
                    j,
                    c + DVec3::Z * 0.025 * scale,
                    c - DVec3::Z * 0.018 * scale,
                ));
            }
        }
        let mut p = Vec::new();
        let mut x = Vec::new();
        for j in 0..97 {
            let offset = if j < 32 {
                1.6
            } else if j < 64 {
                0.0
            } else {
                3.2
            };
            let a = origin
                + DVec3::new(
                    offset + (next() - 0.5) * 0.22,
                    (next() - 0.5) * 0.9,
                    (next() - 0.5) * 0.5,
                ) * scale;
            let end = a + DVec3::new(
                (next() - 0.5) * 0.23,
                (next() - 0.5) * 0.15,
                (next() - 0.5) * 0.59,
            ) * scale;
            p.push(Particle {
                position: a.to_array(),
                velocity: ((end - a) / h).to_array(),
                inverse_mass: if j % 19 == 0 { 0. } else { 2.5 },
            });
            x.push(end);
        }
        total += verify(&shapes, &p, &x, interval, &segments);
    }
    println!("360 layouts, 34,920 particle sweeps and {total} distinct contact rows");
    assert!(total > 3000);
}
