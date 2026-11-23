// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Conservative masks for particle sweeps past many prescribed surfaces. Point corrections
//! retain the solver's authored order. Dynamic receivers keep the scalar path.
use super::*;
pub(super) const BLOCK: usize = 32;
const MAX: usize = 64;
const NORMAL_RANGE: f64 = 1e40;

#[derive(Clone, Copy)]
struct Bounds {
    lo: DVec3,
    hi: DVec3,
}
impl Bounds {
    fn segment(a: DVec3, b: DVec3, pad: DVec3) -> Option<Self> {
        if !a.is_finite() || !b.is_finite() || !pad.is_finite() {
            return None;
        }
        let scale = a.abs().max(b.abs()).max(pad).max_element().max(1.);
        // Outside this finite range preserve the scalar projection's numerical error handling.
        if scale > NORMAL_RANGE {
            return None;
        }
        let pad = pad + DVec3::splat(256. * f64::EPSILON * scale);
        Some(Self {
            lo: a.min(b) - pad,
            hi: a.max(b) + pad,
        })
    }
    fn overlaps(self, other: Self) -> bool {
        !self.lo.cmpgt(other.hi).any() && !other.lo.cmpgt(self.hi).any()
    }
    fn contains(self, p: DVec3) -> bool {
        self.lo.cmple(p).all() && self.hi.cmpge(p).all()
    }
}
pub(super) struct Field {
    shapes: [Option<Bounds>; MAX],
    len: usize,
    radius: f64,
}
impl Field {
    pub(super) fn new(
        particles: usize,
        boundaries: &[PreparedBoundary],
        time: Interval,
    ) -> Option<Self> {
        if particles < BLOCK
            || !(8..=MAX).contains(&boundaries.len())
            || boundaries
                .iter()
                .any(|b| b.receiver.is_some() || matches!(b.shape, Shape::Oriented(_)))
        {
            return None;
        }
        let mut shapes = [None; MAX];
        for (i, b) in boundaries.iter().enumerate() {
            let half = match b.shape {
                Shape::Capsule(c) => c.axis.abs() * c.half_length + DVec3::splat(c.radius),
                Shape::Sphere(r, _) => DVec3::splat(r),
                _ => continue,
            };
            let (a, z) = b.centers(time);
            shapes[i] = Bounds::segment(a, z, half + DVec3::splat(time.radius));
        }
        Some(Self {
            shapes,
            len: boundaries.len(),
            radius: time.radius,
        })
    }
    pub(super) fn block(&self, ends: &[DVec3], starts: &[Particle]) -> Block {
        let mut lo = DVec3::splat(f64::INFINITY);
        let mut hi = DVec3::splat(f64::NEG_INFINITY);
        for (p, &b) in starts.iter().zip(ends) {
            let a = DVec3::from(p.position);
            if !a.is_finite() || !b.is_finite() {
                return Block::default();
            }
            lo = lo.min(a).min(b);
            hi = hi.max(a).max(b);
        }
        let Some(bounds) = Bounds::segment(lo, hi, DVec3::splat(self.radius)) else {
            return Block::default();
        };
        let mut skip = 0;
        for (i, s) in self.shapes[..self.len].iter().enumerate() {
            if s.is_some_and(|s| !s.overlaps(bounds)) {
                skip |= 1u64 << i;
            }
        }
        Block {
            bounds: Some(bounds),
            skip,
        }
    }
}
#[derive(Clone, Copy, Default)]
pub(super) struct Block {
    bounds: Option<Bounds>,
    pub(super) skip: u64,
}
impl Block {
    pub(super) fn contains(self, p: DVec3) -> bool {
        self.bounds.is_some_and(|b| b.contains(p))
    }
}

#[cfg(test)]
mod tests;
