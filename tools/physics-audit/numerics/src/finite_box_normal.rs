// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent material-box closest features and penetration axes at measured native poses.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
pub(crate) type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
pub(crate) const ONE_NM: f64 = 1e-9;
pub(crate) fn vec3(v: &Value) -> Result<DVec3> {
    let x = DVec3::from_array(serde_json::from_value(v.clone())?);
    if !x.is_finite() {
        return Err("nonfinite vector".into());
    }
    Ok(x)
}
pub(crate) fn number(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "missing finite scalar".into())
}
#[derive(Clone, Copy)]
struct BoxPose {
    c: DVec3,
    q: DQuat,
    half: DVec3,
}
impl BoxPose {
    fn measured(b: &Value, half: [f64; 3]) -> Result<Self> {
        let q = DQuat::from_array(serde_json::from_value(b["orientation"].clone())?);
        if !q.is_finite() || (q.length() - 1.).abs() > 1e-9 {
            return Err("invalid native quaternion".into());
        }
        Ok(Self {
            c: vec3(&b["position"])?,
            q: q.normalize(),
            half: DVec3::from_array(half),
        })
    }
    fn point(self, r: DVec3) -> DVec3 {
        self.c + self.q * r
    }
    fn clamp(self, p: DVec3) -> DVec3 {
        self.point((self.q.conjugate() * (p - self.c)).clamp(-self.half, self.half))
    }
    fn vertices(self) -> [DVec3; 8] {
        std::array::from_fn(|k| {
            self.point(
                self.half
                    * DVec3::from_array(std::array::from_fn(|d| {
                        if k & (1 << d) == 0 { -1. } else { 1. }
                    })),
            )
        })
    }
    fn edges(self) -> [(DVec3, DVec3); 12] {
        let v = self.vertices();
        std::array::from_fn(|i| {
            let d = i / 4;
            let k = i % 4;
            let absent = (k & ((1 << d) - 1)) | ((k >> d) << (d + 1));
            (v[absent], v[absent | (1 << d)])
        })
    }
    fn axes(self) -> [DVec3; 3] {
        [self.q * DVec3::X, self.q * DVec3::Y, self.q * DVec3::Z]
    }
    fn radius(self, n: DVec3) -> f64 {
        self.half.dot((self.q.conjugate() * n).abs())
    }
}
#[derive(Clone)]
struct Candidate {
    label: String,
    n: DVec3,
    dist: f64,
}
fn sat(f: BoxPose, p: BoxPose) -> Vec<Candidate> {
    let mut out = vec![];
    let fa = f.axes();
    let pa = p.axes();
    let mut push = |mut n: DVec3, label: String| {
        let l = n.length();
        if l <= 1e-12 {
            return;
        }
        n /= l;
        let off = f.c - p.c;
        if off.dot(n) < 0. {
            n = -n;
        }
        let dist = off.dot(n) - f.radius(n) - p.radius(n);
        if out.iter().any(|a: &Candidate| a.n.distance(n) < 1e-12) {
            return;
        }
        out.push(Candidate { label, n, dist });
    };
    for d in 0..3 {
        push(fa[d], format!("sole_face_{d}"));
        push(pa[d], format!("deck_face_{d}"));
    }
    for (i, &u) in fa.iter().enumerate() {
        for (j, &v) in pa.iter().enumerate() {
            push(u.cross(v), format!("edge_axes_{i}_{j}"));
        }
    }
    out.sort_by(|a, b| b.dist.total_cmp(&a.dist));
    out
}
fn projection(p: DVec3, a: DVec3, v: DVec3) -> DVec3 {
    a + v * ((p - a).dot(v) / v.length_squared()).clamp(0., 1.)
}
fn closest_edges((a, b): (DVec3, DVec3), (c, d): (DVec3, DVec3)) -> (DVec3, DVec3) {
    let u = b - a;
    let v = d - c;
    let w = a - c;
    let mut best = (a, projection(a, c, v));
    let mut check = |p: DVec3, q: DVec3| {
        if p.distance_squared(q) < best.0.distance_squared(best.1) {
            best = (p, q)
        }
    };
    check(b, projection(b, c, v));
    check(projection(c, a, u), c);
    check(projection(d, a, u), d);
    let cross = u.cross(v);
    let denom = cross.length_squared();
    if denom > 1e-28 * u.length_squared() * v.length_squared() {
        let t = v.cross(w).dot(cross) / denom;
        let s = u.cross(w).dot(cross) / denom;
        if (0.0..=1.).contains(&t) && (0.0..=1.).contains(&s) {
            check(a + u * t, c + v * s)
        }
    }
    best
}
fn separated(f: BoxPose, p: BoxPose) -> Vec<Candidate> {
    let mut out = vec![];
    let mut push = |a: DVec3, b: DVec3, label: String| {
        let delta = a - b;
        let dist = delta.length();
        if dist > 1e-15 {
            out.push(Candidate {
                label,
                n: delta / dist,
                dist,
            })
        }
    };
    for (i, x) in f.vertices().into_iter().enumerate() {
        push(x, p.clamp(x), format!("sole_vertex_{i}"))
    }
    for (i, x) in p.vertices().into_iter().enumerate() {
        push(f.clamp(x), x, format!("deck_vertex_{i}"))
    }
    for (i, e) in f.edges().into_iter().enumerate() {
        for (j, l) in p.edges().into_iter().enumerate() {
            let (a, b) = closest_edges(e, l);
            push(a, b, format!("edge_pair_{i}_{j}"));
        }
    }
    out.sort_by(|a, b| a.dist.total_cmp(&b.dist));
    out
}
fn angle(a: DVec3, b: DVec3) -> f64 {
    a.cross(b).length().atan2(a.dot(b))
}
fn score(bodies: &Value, rows: &Value) -> Result<Value> {
    let f = BoxPose::measured(&bodies[0], [0.18, 0.10, 0.06])?;
    let p = BoxPose::measured(&bodies[2], [0.33, 0.18, 0.06])?;
    let axes = sat(f, p);
    let sep = axes[0].dist;
    let (kind, candidates) = if sep > 1e-12 {
        ("separated", separated(f, p))
    } else {
        ("contact_or_overlap", axes)
    };
    let best = candidates.first().ok_or("no nearest feature")?;
    // A disjoint Euclidean distance is at least the largest separating-axis lower bound.
    if sep > 1e-12 && best.dist + 1e-11 < sep {
        return Err("Euclidean distance smaller than valid separation".into());
    }
    let near = candidates
        .iter()
        .filter(|c| (c.dist - best.dist).abs() <= ONE_NM)
        .collect::<Vec<_>>();
    let mut distinct = Vec::<DVec3>::new();
    for a in &near {
        if !distinct.iter().any(|&b| a.n.distance(b) < 1e-7) {
            distinct.push(a.n);
        }
    }
    let mut angle_span: f64 = 0.;
    for &a in &distinct {
        for &b in &distinct {
            angle_span = angle_span.max(angle(a, b));
        }
    }
    let input = rows.as_array().ok_or("missing native rows")?;
    let mut j = 0.;
    let mut nsum = DVec3::ZERO;
    let mut off_j = 0.;
    let mut max_gap = f64::NEG_INFINITY;
    let mut paid = 0;
    for row in input {
        let impulse = number(&row["normal_ns"])?;
        if impulse <= 0. {
            continue;
        }
        let n = vec3(row.get("normal").unwrap_or(&row["normal_on_foot"]))?;
        if (n.length() - 1.).abs() > 1e-8 {
            return Err("non-unit native reported direction".into());
        }
        let af = f.point(vec3(row.get("local_foot").unwrap_or(&row["foot_m"]))?);
        let ap = p.point(vec3(row.get("local_pad").unwrap_or(&row["pad_m"]))?);
        let gap = (af - ap).dot(best.n);
        if gap - best.dist > 1e-6 {
            off_j += impulse;
        }
        max_gap = max_gap.max(gap);
        j += impulse;
        nsum += impulse * n;
        paid += 1;
    }
    if j <= 0. || nsum.length() <= 1e-14 {
        return Err("first positive solve has no signed measured directions".into());
    }
    let native = nsum.normalize();
    let mut errors = distinct
        .iter()
        .map(|&n| angle(native, n) * 1000.)
        .collect::<Vec<_>>();
    errors.sort_by(f64::total_cmp);
    Ok(
        json!({"kind":kind,"sat_separation_m":sep,"signed_distance_m":best.dist,"feature":best.label,
   "normal_to_foot":best.n.to_array(),"native_normal_to_foot":native.to_array(),"sat_advantage_over_native_m":sep-((f.c-p.c).dot(native)-f.radius(native)-p.radius(native)), "native_angle_mrad":angle(native,best.n)*1000.,
   "near_min_angle_mrad":errors[0],"near_max_angle_mrad":errors.last(),"near_normal_span_mrad":angle_span*1000.,"near_direction_count":distinct.len(),
   "paid_rows":paid,"normal_ns":j,"max_paid_anchor_gap_m":max_gap,"over_one_um_from_nearest_fraction":off_j/j}),
    )
}
pub(crate) fn event(e: &Value) -> Result<Value> {
    Ok(
        json!({"elapsed_ns":e.get("elapsed_ns").unwrap_or(&e["ns"]),"before":score(&e["before"],&e["solver_rows"])?,"after":score(&e["after"],&e["solver_rows"])?}),
    )
}
#[cfg(test)]
mod tests {
    use super::*;
    fn boxed(c: DVec3, q: DQuat) -> BoxPose {
        BoxPose {
            c,
            q,
            half: DVec3::ONE,
        }
    }
    #[test]
    fn closed_and_open_face() {
        let pad = boxed(DVec3::ZERO, DQuat::IDENTITY);
        let foot = boxed(DVec3::Z * 2.01, DQuat::IDENTITY);
        let a = sat(foot, pad);
        let b = separated(foot, pad);
        assert!((a[0].dist - 0.01).abs() < 1e-14 && (b[0].dist - 0.01).abs() < 1e-14);
        assert!(b[0].n.distance(DVec3::Z) < 1e-14);
        assert!((sat(boxed(DVec3::Z * 1.99, DQuat::IDENTITY), pad)[0].dist + 0.01).abs() < 1e-14);
    }
    #[test]
    fn skew_segments_interior_and_clamped() {
        let (a, b) = closest_edges(
            (-DVec3::X, DVec3::X),
            (-DVec3::Y + DVec3::Z, DVec3::Y + DVec3::Z),
        );
        assert!(a.length() < 1e-15 && b.distance(DVec3::Z) < 1e-15);
        let (a, b) = closest_edges((-DVec3::X, DVec3::X), (DVec3::X * 2., DVec3::X * 3.));
        assert_eq!((a, b), (DVec3::X, DVec3::X * 2.));
    }
    #[test]
    fn diagonal_corner_has_euclidean_separation() {
        let p = boxed(DVec3::ZERO, DQuat::IDENTITY);
        let f = boxed(DVec3::splat(2.01), DQuat::IDENTITY);
        let s = separated(f, p);
        assert!((s[0].dist - 0.01 * 3f64.sqrt()).abs() < 1e-13);
        assert!(s[0].n.distance(DVec3::ONE.normalize()) < 1e-13);
    }
}
