// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare axial rejection to all three exact feature queries, retaining their tie order.
use super::*;

fn unpruned(
    c: Capsule,
    a: DVec3,
    v: DVec3,
    r: f64,
    eps: f64,
    grazing: bool,
) -> Option<(f64, DVec3)> {
    let z = a.dot(c.axis);
    let dz = v.dot(c.axis);
    let mut hits = [None; 3];
    hits[0] = entry(a - z * c.axis, v - dz * c.axis, r, eps, grazing)
        .filter(|(t, _)| (z + t * dz).abs() <= c.half_length + eps);
    for (i, sign) in [-1., 1.].into_iter().enumerate() {
        hits[i + 1] = entry(a - c.axis * (sign * c.half_length), v, r, eps, grazing)
            .filter(|(t, _)| sign * (z + t * dz) >= c.half_length - eps);
    }
    hits.into_iter()
        .flatten()
        .min_by(|a, b| a.0.total_cmp(&b.0))
}

#[test]
fn culling_matches_capsule_strip_and_hemispheres_at_hits_clearance_and_seams() {
    let mut bits = 0x3b874210f5c691a3u64;
    let mut draw = || {
        bits ^= bits << 13;
        bits ^= bits >> 7;
        bits ^= bits << 17;
        (bits >> 11) as f64 * 2f64.powi(-53)
    };
    let mut hits = 0;
    for i in 0..180_000 {
        let scale = 10f64.powi(i % 13 - 6);
        let c = Capsule::new(
            [draw() - 0.5, draw() - 0.5, draw() - 0.5],
            scale * (0.1 + draw()),
            scale * (0.002 + draw()),
            0.,
        )
        .unwrap();
        let skin = c.radius * draw();
        let r = c.radius + skin;
        let ortho = c.axis.any_orthonormal_vector();
        let mut u = (draw() - 0.5) * 3. * c.half_length;
        match i % 6 {
            0 => u = c.half_length * (1. + (draw() - 0.5) * 1e-13),
            1 => u = -c.half_length * (1. + (draw() - 0.5) * 1e-13),
            _ => {}
        }
        let a = c.axis * u + ortho * r * (1. + 4. * draw());
        let v = match i % 7 {
            0 => DVec3::ZERO,
            1 => (c.axis * 0.7 - ortho) * 1e-14 * r,
            2 => -a * 2.,
            _ => {
                c.axis * ((draw() - 0.5) * 6. * c.half_length) - ortho * r * draw() * 8.
                    + c.axis.cross(ortho) * (draw() - 0.5) * 2. * r
            }
        };
        let b = a + v;
        let delta = b - a;
        let eps = 64. * f64::EPSILON * (a.length() + b.length() + c.half_length + r);
        for grazing in [false, true] {
            let expected = unpruned(c, a, delta, r, eps, grazing);
            hits += usize::from(expected.is_some());
            assert_eq!(
                c.trace(a, delta, r, eps, grazing),
                expected,
                "case {i}, {grazing}"
            );
        }
        if c.witness(a).length() - delta.length() > r + 4. * eps {
            assert!(unpruned(c, a, delta, r, eps, false).is_none());
            assert!(c.witness(b).length() >= r - eps);
            assert!(c.project(a, b, skin).unwrap().is_none());
        }
    }
    assert!(hits > 40_000);
}
