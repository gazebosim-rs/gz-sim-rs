// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn rect(center: [f64; 2], half: [f64; 2], angle: f64) -> Rectangle<f64> {
    let (s, c) = angle.sin_cos();
    Rectangle {
        center,
        half,
        x_axis: [c, s],
    }
}
#[test]
fn finite_faces_clip_to_the_shorter_edge_and_floor_uses_either_corner() {
    let p = rect([0., 0.], [0.024, 0.020], 0.);
    let fixed = rect([-0.03, 0.], [0.006, 0.011], 0.);
    let mut w = rectangles(fixed, p, 0., 1.);
    w.sort_by(|a, b| a.point[1].total_cmp(&b.point[1]));
    for (end, sign) in w.iter().zip([-1., 1.]) {
        assert!(end.gap.abs() < 1e-15);
        assert_eq!(end.normal, [1., 0.]);
        assert!(
            (end.point[0] + 0.024).abs() < 1e-15 && (end.point[1] - sign * 0.011).abs() < 1e-15
        );
    }
    for heading in [-0.8, 0., 0.24, 2.7] {
        let mut b = rect([0., 0.], [0.024, 0.020], heading);
        b.center[1] = b.radius([0., 1.], 0.);
        let witnesses = floor(b, 0., 0., 1.);
        assert!(witnesses.iter().all(|p| p.gap >= -1e-15));
        assert_eq!(
            witnesses.iter().filter(|p| p.gap.abs() < 1e-15).count(),
            if heading == 0. { 2 } else { 1 }
        );
    }
}
#[test]
fn rotated_reference_face_meets_one_support_corner() {
    let angle = 0.63_f64;
    let n = [angle.cos(), angle.sin()];
    let tip = [0.10, 0.05];
    let a = rect([0., 0.], tip, 0.);
    let b = rect(add(tip, mul(n, 0.08)), [0.08, 0.07], angle);
    let w = rectangles(a, b, 0., 1.);
    let touching: Vec<_> = w.iter().filter(|p| p.gap.abs() < 1e-15).collect();
    assert_eq!(touching.len(), 1);
    let p = touching[0];
    assert!((p.normal[0] - n[0]).hypot(p.normal[1] - n[1]) < 1e-15);
    assert!((p.point[0] - tip[0]).hypot(p.point[1] - tip[1]) < 1e-15);
}
#[test]
fn clipped_witnesses_agree_with_independent_corner_projections() {
    fn corners(b: Rectangle<f64>) -> [[f64; 2]; 4] {
        let [x, z] = b.axes(0.);
        [(-1., -1.), (-1., 1.), (1., -1.), (1., 1.)]
            .map(|(u, v)| add(b.center, add(mul(x, u * b.half[0]), mul(z, v * b.half[1]))))
    }
    let a = rect([0., 0.], [0.018, 0.009], 0.);
    for i in -5..=5 {
        for j in -4..=4 {
            for k in 0..14 {
                let b = rect(
                    [i as f64 * 0.0081, j as f64 * 0.0079],
                    [0.022, 0.013],
                    k as f64 * 0.207,
                );
                let aa = corners(a);
                let bb = corners(b);
                let intersects = a.axes(0.).into_iter().chain(b.axes(0.)).all(|n| {
                    let interval = |points: [[f64; 2]; 4]| {
                        points
                            .into_iter()
                            .map(|p| dot(p, n))
                            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                                (lo.min(v), hi.max(v))
                            })
                    };
                    let (lo, hi) = interval(aa);
                    let (min, max) = interval(bb);
                    lo <= max + 1e-14 && min <= hi + 1e-14
                });
                let w = rectangles(a, b, 0., 1.);
                assert_eq!(intersects, w.iter().any(|r| r.gap <= 1e-14), "{i} {j} {k}");
                for p in w.iter().filter(|p| p.gap <= 1e-14) {
                    let local = b.axes(0.).map(|n| dot(sub(p.point, b.center), n));
                    assert!(
                        local[0].abs() <= b.half[0] + 1e-12 && local[1].abs() <= b.half[1] + 1e-12
                    );
                    assert!(
                        (local[0].abs() - b.half[0])
                            .abs()
                            .min((local[1].abs() - b.half[1]).abs())
                            < 1e-12
                    );
                }
            }
        }
    }
}
