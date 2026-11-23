// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{DVec3, Guide, Particle, Stencil};
use std::ops::Range;
#[derive(Clone, Copy)]
pub(super) struct Bounds {
    min: DVec3,
    max: DVec3,
}
impl Bounds {
    pub fn empty() -> Self {
        Self {
            min: DVec3::splat(f64::INFINITY),
            max: DVec3::splat(f64::NEG_INFINITY),
        }
    }
    fn include(&mut self, p: DVec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }
    pub fn particle(&mut self, i: usize, x: &[DVec3], p: &[Particle]) {
        self.include(x[i]);
        self.include(p[i].position.into());
    }
    pub fn swept_guide(g: Guide) -> Self {
        let a = g.center;
        let b = a + g.delta;
        let margin = g.arm.abs()
            + DVec3::splat(
                g.gap
                    + 64.
                        * f64::EPSILON
                        * (a.abs().max_element().max(b.abs().max_element())
                            + g.arm.length()
                            + g.gap),
            );
        Self {
            min: a.min(b) - margin,
            max: a.max(b) + margin,
        }
    }
    pub fn overlaps(self, o: Self) -> bool {
        self.min.cmple(o.max).all() && o.min.cmple(self.max).all()
    }
}
pub(super) struct Block {
    pub range: Range<usize>,
    vertices: Vec<usize>,
}
impl Block {
    pub fn new(range: Range<usize>, stencils: &[Stencil]) -> Self {
        let mut vertices = vec![];
        for s in &stencils[range.clone()] {
            match s {
                Stencil::Edge(ids) => vertices.extend_from_slice(ids),
                Stencil::FaceCap(ids, _) => vertices.extend_from_slice(ids),
            }
        }
        vertices.sort_unstable();
        vertices.dedup();
        Self { range, vertices }
    }
    pub fn bounds(&self, x: &[DVec3], p: &[Particle]) -> Bounds {
        let mut bounds = Bounds::empty();
        for &i in &self.vertices {
            bounds.particle(i, x, p);
        }
        bounds
    }
}
