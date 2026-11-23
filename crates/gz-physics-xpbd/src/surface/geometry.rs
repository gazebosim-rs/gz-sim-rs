// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;

/// Signed barycentric weights locate the material witnesses A and B.
/// The weighted combination of all four positions equals A - B.
#[derive(Clone, Copy)]
pub(crate) struct Witness {
    pub weights: [f64; 4],
    pub delta: DVec3,
}

impl Witness {
    fn at(p: &[DVec3; 4], weights: [f64; 4]) -> Self {
        // Relative coordinates avoid cancellation of a large shared translation.
        let delta =
            (p[1] - p[0]) * weights[1] + (p[2] - p[0]) * weights[2] + (p[3] - p[0]) * weights[3];
        Self { weights, delta }
    }

    fn nearer(self, other: Self) -> Self {
        if other.delta.length_squared() < self.delta.length_squared() {
            other
        } else {
            self
        }
    }
}

fn segment_parameter(p: DVec3, a: DVec3, b: DVec3) -> f64 {
    let v = b - a;
    let length2 = v.length_squared();
    if length2 == 0.0 {
        0.0
    } else {
        ((p - a).dot(v) / length2).clamp(0., 1.)
    }
}

pub(crate) fn vertex_face(p: &[DVec3; 4]) -> Witness {
    let [point, a, b, c] = *p;
    let ab = b - a;
    let ac = c - a;
    let n = ab.cross(ac);
    let n2 = n.length_squared();
    if n2 > 0. {
        let ap = point - a;
        let u = ap.cross(ac).dot(n) / n2;
        let v = ab.cross(ap).dot(n) / n2;
        if u >= 0. && v >= 0. && u + v <= 1. {
            return Witness::at(p, [1., -(1. - u - v), -u, -v]);
        }
    }
    let ab = segment_parameter(point, a, b);
    let bc = segment_parameter(point, b, c);
    let ca = segment_parameter(point, c, a);
    Witness::at(p, [1., ab - 1., -ab, 0.])
        .nearer(Witness::at(p, [1., 0., bc - 1., -bc]))
        .nearer(Witness::at(p, [1., -ca, 0., ca - 1.]))
}

pub(crate) fn edge_edge(p: &[DVec3; 4]) -> Witness {
    let [a, b, c, d] = *p;
    let ab = b - a;
    let cd = d - c;
    let n = ab.cross(cd);
    let n2 = n.length_squared();
    let witness = |s: f64, t: f64| Witness::at(p, [1. - s, s, t - 1., -t]);
    if n2 > 0. {
        let ca = c - a;
        let s = ca.cross(cd).dot(n) / n2;
        let t = ca.cross(ab).dot(n) / n2;
        if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
            return witness(s, t);
        }
    }
    witness(0., segment_parameter(a, c, d))
        .nearer(witness(1., segment_parameter(b, c, d)))
        .nearer(witness(segment_parameter(c, a, b), 0.))
        .nearer(witness(segment_parameter(d, a, b), 1.))
}

/// SAT for two triangle hulls. In-plane edge normals also cover coplanar pairs.
pub(super) fn intersect(a: [DVec3; 3], b: [DVec3; 3]) -> bool {
    let origin = a[0];
    let a = a.map(|p| p - origin);
    let b = b.map(|p| p - origin);
    let edges = |p: [DVec3; 3]| [p[1] - p[0], p[2] - p[1], p[0] - p[2]];
    let ea = edges(a);
    let eb = edges(b);
    let na = ea[0].cross(ea[1]);
    let nb = eb[0].cross(eb[1]);
    let separates = |axis: DVec3| {
        let Some(n) = axis.try_normalize() else {
            return false;
        };
        let project = |p: [DVec3; 3]| {
            let q = p.map(|p| p.dot(n));
            (q[0].min(q[1]).min(q[2]), q[0].max(q[1]).max(q[2]))
        };
        let (la, ha) = project(a);
        let (lb, hb) = project(b);
        let error = 32. * f64::EPSILON * la.abs().max(ha.abs()).max(lb.abs()).max(hb.abs());
        la - hb > error || lb - ha > error
    };
    !(separates(na)
        || separates(nb)
        || ea
            .iter()
            .any(|e| separates(na.cross(*e)) || eb.iter().any(|f| separates(e.cross(*f))))
        || eb.iter().any(|e| separates(nb.cross(*e))))
}
