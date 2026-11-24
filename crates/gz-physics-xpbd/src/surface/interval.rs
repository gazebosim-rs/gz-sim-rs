// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Lower bounds for linearly moving sphere–triangle clearance.
use glam::DVec3;

type Line = [DVec3; 2];
const CHOOSE: [[f64; 7]; 7] = [
    [1., 0., 0., 0., 0., 0., 0.],
    [1., 1., 0., 0., 0., 0., 0.],
    [1., 2., 1., 0., 0., 0., 0.],
    [1., 3., 3., 1., 0., 0., 0.],
    [1., 4., 6., 4., 1., 0., 0.],
    [1., 5., 10., 10., 5., 1., 0.],
    [1., 6., 15., 20., 15., 6., 1.],
];

fn cross(a: Line, b: Line) -> [DVec3; 3] {
    [
        a[0].cross(b[0]),
        a[1].cross(b[0]) + a[0].cross(b[1]),
        a[1].cross(b[1]),
    ]
}
fn dot(a: &[DVec3], b: &[DVec3]) -> [f64; 7] {
    let mut out = [0.; 7];
    for (i, x) in a.iter().enumerate() {
        for (j, y) in b.iter().enumerate() {
            out[i + j] += x.dot(*y);
        }
    }
    out
}
fn subtract(a: [f64; 7], b: [f64; 7], factor: f64) -> [f64; 7] {
    std::array::from_fn(|i| a[i] - factor * b[i])
}
fn square(x: [f64; 7], degree: usize) -> [f64; 7] {
    let mut out = [0.; 7];
    for i in 0..=degree {
        for j in 0..=degree {
            out[i + j] += x[i] * x[j];
        }
    }
    out
}
fn positive(c: [f64; 7], degree: usize, error: f64) -> bool {
    // Values lie in the convex hull of their Bernstein coefficients.
    (0..=degree).all(|i| {
        (0..=i)
            .map(|j| c[j] * CHOOSE[i][j] / CHOOSE[degree][j])
            .sum::<f64>()
            > error
    })
}
fn sub(a: Line, b: Line) -> Line {
    [a[0] - b[0], a[1] - b[1]]
}
fn scale(a: &[DVec3]) -> f64 {
    a.iter().map(|p| p.length()).sum::<f64>()
}
fn edge_clear(r: Line, e: Line, g2: f64) -> bool {
    let rp = sub(r, e);
    for (q, direction) in [(r, e), (rp, [-e[0], -e[1]])] {
        let side = dot(&q, &direction).map(|x| -x);
        if positive(side, 2, 1e-13 * scale(&q) * scale(&e)) {
            let mut distance = dot(&q, &q);
            distance[0] -= g2;
            return positive(distance, 2, 1e-13 * (scale(&q).powi(2) + g2));
        }
    }
    let n = cross(r, e);
    let c = subtract(dot(&n, &n), dot(&e, &e), g2);
    positive(c, 4, 1e-13 * scale(&e).powi(2) * (scale(&r).powi(2) + g2))
}

pub(super) fn clear(a: [DVec3; 4], b: [DVec3; 4], gap: f64) -> bool {
    clear_triangle(a, b, gap, false)
}
pub(super) fn clear_triangle(a: [DVec3; 4], b: [DVec3; 4], gap: f64, sliding: bool) -> bool {
    // Express coordinates relative to the first triangle vertex.
    let line = |i: usize, j: usize| [a[i] - a[j], (b[i] - b[j]) - (a[i] - a[j])];
    let r = line(0, 1);
    let e = line(2, 1);
    let f = line(3, 1);
    let n = cross(e, f);
    let signed = dot(&r, &n);
    let g2 = gap * gap;
    let plane = subtract(square(signed, 3), dot(&n, &n), g2);
    let area_scale = scale(&e) * scale(&f);
    let signed_scale = signed.iter().map(|x| x.abs()).sum::<f64>();
    let round = 96. * f64::EPSILON;
    let signed_error = round * area_scale * scale(&r);
    let plane_error = 2. * signed_error * (signed_scale + signed_error)
        + round * (signed_scale.powi(2) + g2 * area_scale.powi(2));
    let plane_error = if sliding {
        plane_error
    } else {
        1e-13 * area_scale.powi(2) * (scale(&r).powi(2) + g2)
    };
    if positive(plane, 6, plane_error) {
        return true;
    }
    let u = cross(r, f);
    let v = cross(e, r);
    let w = cross(sub(e, r), sub(f, r));
    let outside = [u, v, w]
        .into_iter()
        .any(|q| positive(dot(&q, &n).map(|x| -x), 4, 1e-13 * scale(&q) * area_scale));
    outside && edge_clear(r, e, g2) && edge_clear(sub(r, e), sub(f, e), g2) && edge_clear(r, f, g2)
}

fn product(a: [f64; 7], b: [f64; 7], degree: usize) -> [f64; 7] {
    let mut c = [0.; 7];
    for i in 0..=degree {
        for j in 0..=degree {
            c[i + j] += a[i] * b[j];
        }
    }
    c
}

/// A lower bound between two swept edges. When the infinite-line witness is
/// outside either finite segment throughout, the minimum is on its boundary.
pub(super) fn clear_edges(a: [DVec3; 4], b: [DVec3; 4], gap: f64) -> bool {
    let line = |i: usize, j: usize| [a[i] - a[j], (b[i] - b[j]) - (a[i] - a[j])];
    let r = line(0, 2);
    let e = line(1, 0);
    let f = line(3, 2);
    let n = cross(e, f);
    let signed = dot(&r, &n);
    let nn = dot(&n, &n);
    let g2 = gap * gap;
    let area = scale(&e) * scale(&f);
    let signed_scale = signed.iter().map(|x| x.abs()).sum::<f64>();
    let round = 96. * f64::EPSILON;
    let signed_error = round * area * scale(&r);
    let plane_error = 2. * signed_error * (signed_scale + signed_error)
        + round * (signed_scale.powi(2) + g2 * area.powi(2));
    if positive(subtract(square(signed, 3), nn, g2), 6, plane_error) {
        return true;
    }
    let ee = dot(&e, &e);
    let ef = dot(&e, &f);
    let ff = dot(&f, &f);
    let er = dot(&e, &r);
    let fr = dot(&f, &r);
    let s = subtract(product(ef, fr, 2), product(ff, er, 2), 1.);
    let t = subtract(product(ee, fr, 2), product(ef, er, 2), 1.);
    let err = 1e-13 * area * (area + scale(&r) * (scale(&e) + scale(&f)));
    if !n.iter().all(|v| *v == DVec3::ZERO)
        && ![
            s.map(|x| -x),
            t.map(|x| -x),
            subtract(s, nn, 1.),
            subtract(t, nn, 1.),
        ]
        .into_iter()
        .any(|c| positive(c, 4, err))
    {
        return false;
    }
    edge_clear(line(0, 2), f, g2)
        && edge_clear(line(1, 2), f, g2)
        && edge_clear(line(2, 0), e, g2)
        && edge_clear(line(3, 0), e, g2)
}

/// A time-varying separating axis for two linearly swept convex features.
/// The projection of each vertex pair onto a linearly interpolated normal
/// is quadratic. Its three Bernstein coefficients bound the whole interval.
/// Convexity also bounds the normal length by its larger endpoint length.
pub(super) fn moving_plane(
    a: &[DVec3; 4],
    b: &[DVec3; 4],
    normals: [DVec3; 2],
    split: usize,
    gap: f64,
) -> bool {
    let [n0, n1] = normals;
    let threshold = gap * n0.length().max(n1.length());
    (0..split).all(|i| {
        (split..4).all(|j| {
            let d0 = a[i] - a[j];
            let d1 = b[i] - b[j];
            let floor = threshold + 128. * f64::EPSILON * (threshold + d0.length() + d1.length());
            d0.dot(n0) > floor && d1.dot(n1) > floor && (d0.dot(n1) + d1.dot(n0)) * 0.5 > floor
        })
    })
}
