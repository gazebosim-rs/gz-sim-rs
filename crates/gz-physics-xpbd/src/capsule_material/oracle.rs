// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DQuat;
use parry3d_f64::{
    math::Pose,
    query::{self, ShapeCastOptions},
    shape::{Capsule, Segment, Shape, Triangle},
};
fn exercise(shear: bool) {
    let (mut matches, mut misses, mut maximum) = (0, 0, 0f64);
    for i in 0..240 {
        let t = i as f64;
        let turn =
            DQuat::from_rotation_y(0.33 * (t * 0.37).sin()) * DQuat::from_rotation_z(t * 0.39);
        let start = [
            DVec3::new(-0.46, -0.25, 0.63),
            DVec3::new(0.44, 0.03, 0.63),
            DVec3::new(-0.14, 0.44, 0.63),
        ]
        .map(|v| turn * v);
        let base = DVec3::new(0.09 * (0.45 * t).sin(), 0.15 * (0.31 * t).cos(), -1.4);
        let velocity = std::array::from_fn::<_, 3, _>(|j| {
            let a = t + 1.73 * j as f64;
            base + if shear {
                DVec3::new(
                    (a * 0.53).sin() * 0.12,
                    (a * 0.64).cos() * 0.16,
                    (a * 0.47).sin() * 0.23,
                )
            } else {
                DVec3::ZERO
            }
        });
        let end = std::array::from_fn::<_, 3, _>(|j| start[j] + velocity[j]);
        let ps = std::array::from_fn::<_, 3, _>(|j| Particle {
            position: start[j].to_array(),
            velocity: velocity[j].to_array(),
            inverse_mass: 1.,
        });
        let center = DVec3::new(0.52 * (t * 0.25).sin(), 0.46 * (t * 0.41).cos(), 0.);
        let axis = DVec3::new((t * 0.7).sin(), 0.3 + (t * 0.4).cos(), (t * 0.8).cos()).normalize();
        let arm = axis * 0.19;
        let motion = DVec3::new(0.12 * (t * 0.46).sin(), 0.04 * (t * 0.19).cos(), 0.02);
        let g = Guide {
            center,
            arm,
            delta: motion,
            velocity: motion,
            gap: 0.08,
        };
        let cloth = i % 2 == 0;
        let mut stencil = vec![Stencil::Edge([0, 1])];
        if cloth {
            stencil.extend([
                Stencil::Edge([0, 2]),
                Stencil::Edge([1, 2]),
                Stencil::FaceCap([0, 1, 2], -1),
                Stencil::FaceCap([0, 1, 2], 1),
            ]);
        }
        let mut time: Option<f64> = None;
        let mut queries = 0;
        for s in stencil {
            if let Some(c) = s.candidate(&end, &ps, g, &mut queries).unwrap() {
                time = Some(time.map_or(c.time, |v| v.min(c.time)));
            }
        }
        let segment = Segment::new(start[0], start[1]);
        let triangle = Triangle::new(start[0], start[1], start[2]);
        let shape: &dyn Shape = if cloth { &triangle } else { &segment };
        let reference = if !shear {
            let cap = Capsule::new(center - arm, center + arm, g.gap);
            query::cast_shapes(
                &Pose::IDENTITY,
                base,
                shape,
                &Pose::IDENTITY,
                motion,
                &cap,
                ShapeCastOptions::with_max_time_of_impact(1.),
            )
            .unwrap()
            .map(|h| h.time_of_impact)
        } else {
            // Dense static Parry separation on the actual deforming feature;
            // bracket from the left and bisect once the finite skin closes.
            let dist = |u: f64| {
                let q = std::array::from_fn::<_, 3, _>(|j| start[j] + u * velocity[j]);
                let c = center + motion * u;
                let core = Segment::new(c - arm, c + arm);
                let seg = Segment::new(q[0], q[1]);
                let tri = Triangle::new(q[0], q[1], q[2]);
                let a: &dyn Shape = if cloth { &tri } else { &seg };
                query::distance(&Pose::IDENTITY, a, &Pose::IDENTITY, &core)
                    .unwrap()
                    .distance
                    - g.gap
            };
            let mut result = None;
            for j in 1..=512 {
                let mut hi = j as f64 / 512.;
                if dist(hi) <= 0. {
                    let mut lo = (j - 1) as f64 / 512.;
                    for _ in 0..36 {
                        let mid = 0.5 * (lo + hi);
                        if dist(mid) > 0. {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    result = Some(hi);
                    break;
                }
            }
            result
        };
        match (time, reference) {
            (Some(a), Some(b)) => {
                maximum = maximum.max((a - b).abs());
                matches += 1;
            }
            (None, None) => misses += 1,
            mismatch => panic!("case {i} shear={shear}: {mismatch:?}"),
        }
    }
    println!(
        "240 capsule/material paths shear={shear}: {matches} hits, {misses} misses, max normalized TOI disagreement {maximum:e}"
    );
    assert!(matches > 50 && misses > 50 && maximum < 2e-6);
}
#[test]
fn moving_material_features_match_independent_parry_queries() {
    exercise(false);
    exercise(true);
}
