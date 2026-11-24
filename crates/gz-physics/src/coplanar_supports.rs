// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Feasible normal-load shares for two frictionless coplanar support patches.
//! A rigid sole fixes the resultant and its center, while pressure within
//! either patch may be distributed in many ways.
mod capture;
pub use capture::{BeltCapture,BeltDualBound,BeltCaptureRange,BeltEquilibriumWitness,BeltVertexReaction};
mod sliding;
use crate::PhysicsError;
use glam::DVec2;
pub use sliding::{
    CoplanarSlidingContact, SlidingAccelerationSet, SlidingAccelerationVertex, SlidingLoadChoice,
    SlidingLoadSet,
};
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SupportLoadRange {
    /// Lowest and highest possible fraction of total load carried by site B.
    pub fraction_b: [f64; 2],
}
impl SupportLoadRange {
    pub fn force_bounds(self, total: f64) -> Result<[[f64; 2]; 2], PhysicsError> {
        if !total.is_finite() || total < 0. {
            return Err(PhysicsError::InvalidValue);
        }
        Ok([
            [
                total * (1. - self.fraction_b[1]),
                total * (1. - self.fraction_b[0]),
            ],
            [total * self.fraction_b[0], total * self.fraction_b[1]],
        ])
    }
}
#[derive(Clone, Copy)]
struct Patch {
    v: [DVec2; 16],
    len: usize,
}
impl Patch {
    fn from(raw: &[DVec2], c: DVec2, scale: f64) -> Result<Self, PhysicsError> {
        let mut v = [DVec2::ZERO; 16];
        for (p, raw) in v.iter_mut().zip(raw) {
            *p = (*raw - c) / scale;
        }
        let p = Self { v, len: raw.len() };
        for k in 0..p.len {
            let a = v[k];
            let edge = v[(k + 1) % p.len] - a;
            if v[..p.len]
                .iter()
                .any(|&b| edge.perp_dot(b - a) < -128. * f64::EPSILON * edge.length())
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        Ok(p)
    }
    fn support(self, n: DVec2) -> f64 {
        self.v[..self.len]
            .iter()
            .map(|p| p.dot(n))
            .fold(f64::NEG_INFINITY, f64::max)
    }
}
/// Bounded pressure splits of two convex CCW patches, measured in any
/// orthonormal coordinates of their shared plane. A missing patch is empty.
/// Lines and points are valid limiting contact sets. All patches have at most
/// sixteen vertices, including clipped and swept rectangular contacts.
pub fn coplanar_load_range(
    center: DVec2,
    patches: [&[DVec2]; 2],
) -> Result<Option<SupportLoadRange>, PhysicsError> {
    if !center.is_finite()
        || patches
            .iter()
            .any(|p| p.len() > 16 || p.iter().any(|v| !v.is_finite()))
    {
        return Err(PhysicsError::InvalidValue);
    }
    if patches.iter().all(|p| p.is_empty()) {
        return Ok(None);
    }
    let scale = patches
        .iter()
        .flat_map(|p| p.iter())
        .map(|v| (*v - center).abs().max_element())
        .fold(0., f64::max);
    if !scale.is_finite() {
        return Err(PhysicsError::InvalidValue);
    }
    let [a, b] = [
        Patch::from(patches[0], center, scale.max(f64::MIN_POSITIVE))?,
        Patch::from(patches[1], center, scale.max(f64::MIN_POSITIVE))?,
    ];
    let mut lo = if a.len == 0 { 1.0f64 } else { 0. };
    let mut hi = if b.len == 0 { 0.0f64 } else { 1. };
    // h((1-alpha)A + alpha B, n) = (1-alpha)h(A,n)+alpha h(B,n).
    // The Minkowski polygon's edge normals come from its two summands.
    // Tangents and separation directions also cover collinear and point cases.
    let mut feasible = true;
    let mut constrain = |n: DVec2| {
        let norm = n.length();
        if norm == 0. {
            return;
        }
        let n = n / norm;
        for unit in [n, -n] {
            let ha = a.support(unit);
            let hb = b.support(unit);
            let round = 32. * f64::EPSILON;
            if a.len == 0 || b.len == 0 {
                feasible &= ha.max(hb) >= -round;
                continue;
            }
            match (ha < 0., hb < 0.) {
                (true, true) => feasible &= ha.max(hb) >= -round,
                (true, false) => {
                    lo = lo.max((-ha / (hb - ha)).clamp(0., 1.));
                }
                (false, true) => {
                    hi = hi.min((ha / (ha - hb)).clamp(0., 1.));
                }
                _ => {}
            }
        }
    };
    constrain(DVec2::X);
    constrain(DVec2::Y);
    for p in [a, b] {
        for j in 0..p.len {
            let d = p.v[(j + 1) % p.len] - p.v[j];
            constrain(d.perp());
            constrain(d);
        }
    }
    if a.len > 0 && b.len > 0 {
        let separation = b.v[0] - a.v[0];
        constrain(separation);
        constrain(separation.perp());
    }
    if !feasible || lo > hi + 128. * f64::EPSILON {
        return Ok(None);
    }
    if lo > hi {
        lo = (lo + hi) * 0.5;
        hi = lo;
    }
    Ok(Some(SupportLoadRange {
        fraction_b: [lo, hi],
    }))
}
#[cfg(test)]
mod tests {
    use super::*;
    fn rect(a: f64, b: f64, y0: f64, y1: f64) -> [DVec2; 4] {
        [(a, y0), (b, y0), (b, y1), (a, y1)].map(DVec2::from)
    }
    #[test]
    fn separated_and_overlapping_pads_distinguish_mandatory_optional_and_lost_load() {
        let a = rect(-1., -0.2, -0.5, 0.5);
        let b = rect(0.2, 1., -0.5, 0.5);
        let r = coplanar_load_range(DVec2::ZERO, [&a, &b]).unwrap().unwrap();
        assert!(
            (r.fraction_b[0] - 1. / 6.).abs() < 1e-14 && (r.fraction_b[1] - 5. / 6.).abs() < 1e-14
        );
        let c = DVec2::new(-0.6, 0.);
        let r = coplanar_load_range(c, [&a, &b]).unwrap().unwrap();
        assert_eq!(r.fraction_b[0], 0.);
        assert!((r.fraction_b[1] - 1. / 3.).abs() < 1e-14);
        assert_eq!(
            coplanar_load_range(c, [&a, &[]])
                .unwrap()
                .unwrap()
                .force_bounds(20.)
                .unwrap(),
            [[20., 20.], [0., 0.]]
        );
        assert!(
            coplanar_load_range(DVec2::ZERO, [&a, &[]])
                .unwrap()
                .is_none()
        );
        assert_eq!(
            coplanar_load_range(c, [&a, &a])
                .unwrap()
                .unwrap()
                .fraction_b,
            [0., 1.]
        );
    }
    #[test]
    fn skewed_baseline_and_degenerate_lines_have_physical_load_intervals() {
        let a = [DVec2::new(-1., -1.)];
        let b = [DVec2::new(1., 1.)];
        let r = coplanar_load_range(DVec2::new(0.5, 0.5), [&a, &b])
            .unwrap()
            .unwrap();
        assert!((r.fraction_b[0] - 0.75).abs() < 1e-14 && (r.fraction_b[1] - 0.75).abs() < 1e-14);
        assert!(
            coplanar_load_range(DVec2::new(0.5, 0.49), [&a, &b])
                .unwrap()
                .is_none()
        );
        let a = [DVec2::new(-1., 0.), DVec2::new(-0.2, 0.)];
        let b = [DVec2::new(0.2, 0.), DVec2::new(1., 0.)];
        let q = coplanar_load_range(DVec2::ZERO, [&a, &b]).unwrap().unwrap();
        assert!((q.fraction_b[0] - 1. / 6.).abs() < 1e-14);
        assert!(
            coplanar_load_range(DVec2::new(0., 0.001), [&a, &b])
                .unwrap()
                .is_none()
        );
    }
}
