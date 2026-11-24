// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reuse a lower bound on self-separation while a small shell keeps its shape.
use super::{Particle, Stencil, Surface, edge_edge, vertex_face};
use glam::DVec3;
use std::sync::Arc;

#[derive(Clone)]
pub(super) struct Clearance {
    pairs: Arc<[Stencil]>,
    reference: Vec<DVec3>,
    lower_bound: f64,
    may_rebuild: bool,
    retry_after: u8,
}

impl Clearance {
    pub(super) fn new(surface: &Surface) -> Option<Self> {
        // Construction stays bounded independently of the maximum authored mesh.
        // Even a fully overlapping sweep of these features fits the candidate budget.
        if surface.features.len() > 64 {
            return None;
        }
        let mut pairs = Vec::new();
        for (i, &a) in surface.features.iter().enumerate() {
            for &b in &surface.features[i + 1..] {
                if let Some(pair) = surface.candidate(a, b) {
                    pairs.push(pair);
                }
            }
        }
        if pairs.is_empty() {
            return None;
        }
        Some(Self {
            pairs: pairs.into(),
            reference: Vec::new(),
            lower_bound: 0.,
            may_rebuild: true,
            retry_after: 0,
        })
    }

    pub(super) fn begin_step(&mut self) {
        self.may_rebuild = self.retry_after == 0;
        self.retry_after = self.retry_after.saturating_sub(1);
    }

    fn contains(&self, x: &[DVec3], p: &[Particle], gap: f64, eps: f64) -> bool {
        let margin = (self.lower_bound - gap - 16. * eps) / 6.;
        if margin <= 0. || self.reference.len() != x.len() {
            return false;
        }
        let start = DVec3::from(p[0].position);
        let end = x[0];
        // Relative to particle zero, each endpoint moves by less than margin.
        // The initial convex gap shrinks by at most 2m; the largest relative
        // sweep is at most 4m. Thus every ordinary CCD query clears at t=0.
        self.reference.iter().zip(x).zip(p).all(|((&r, &q), node)| {
            ((DVec3::from(node.position) - start) - r)
                .abs()
                .element_sum()
                < margin
                && ((q - end) - r).abs().element_sum() < margin
        })
    }

    pub(super) fn clear(&mut self, x: &[DVec3], p: &[Particle], gap: f64, eps: f64) -> bool {
        if self.contains(x, p, gap, eps) {
            return true;
        }
        if !std::mem::take(&mut self.may_rebuild) {
            return false;
        }
        // Even a valid near-contact bound can expire on the following sweep.
        // Cap rebuilds during fast folding. Valid bounds remain usable on every sweep.
        self.retry_after = 31;
        self.reference.clear();
        let origin = DVec3::from(p[0].position);
        self.reference
            .extend(p.iter().map(|n| DVec3::from(n.position) - origin));
        self.lower_bound = f64::INFINITY;
        for pair in &*self.pairs {
            let points = pair.ids.map(|i| self.reference[i]);
            let (witness, split) = if pair.vf {
                (vertex_face(&points), 1)
            } else {
                (edge_edge(&points), 2)
            };
            let Some(axis) = witness.delta.try_normalize() else {
                self.lower_bound = 0.;
                return false;
            };
            // Any unit separating axis lower-bounds the true feature distance,
            // even when the closest-point estimate for a skinny face is inaccurate.
            let mut separation = f64::INFINITY;
            for a in &points[..split] {
                for b in &points[split..] {
                    let projected = (*a - *b).dot(axis);
                    if !projected.is_finite() {
                        self.lower_bound = 0.;
                        return false;
                    }
                    separation = separation.min(projected);
                }
            }
            self.lower_bound = self.lower_bound.min(separation - 16. * eps);
            if self.lower_bound <= gap + 16. * eps {
                self.lower_bound = 0.;
                return false;
            }
        }
        self.contains(x, p, gap, eps)
    }
}

#[cfg(test)]
#[path = "../../tests/private/clearance.rs"]
mod tests;
