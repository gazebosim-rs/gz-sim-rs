// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::surface::{
    geometry::{edge_edge, vertex_face},
    material_sweep,
};

#[derive(Clone, Copy)]
pub(super) enum Stencil {
    Edge([usize; 2]),
    FaceCap([usize; 3], i8),
}

#[derive(Clone, Copy)]
pub(super) struct Guide {
    pub center: DVec3,
    pub arm: DVec3,
    pub delta: DVec3,
    pub velocity: DVec3,
    pub gap: f64,
}
impl Stencil {
    pub fn describe(
        &self,
        w: [f64; 4],
        direction: DVec3,
    ) -> (MaterialFeature, [f64; 3], CapsuleRegion, DVec3) {
        let (ids, weights, part, normal) = match *self {
            Self::Edge([a, b]) => (
                [a, b, b],
                [w[0], w[1], 0.],
                if -w[3] < 1e-10 {
                    CapsuleRegion::NegativeCap
                } else if -w[2] < 1e-10 {
                    CapsuleRegion::PositiveCap
                } else {
                    CapsuleRegion::Barrel
                },
                direction,
            ),
            Self::FaceCap(ids, sign) => (
                ids,
                [-w[1], -w[2], -w[3]],
                if sign < 0 {
                    CapsuleRegion::NegativeCap
                } else {
                    CapsuleRegion::PositiveCap
                },
                -direction,
            ),
        };
        let (feature, weights) = MaterialFeature::from_weights(ids, weights);
        (feature, weights, part, normal)
    }
    pub fn coords(&self, soft: impl Fn(usize) -> DVec3, arm: DVec3) -> [DVec3; 4] {
        match *self {
            Self::Edge([a, b]) => [soft(a), soft(b), -arm, arm],
            Self::FaceCap([a, b, c], sign) => [f64::from(sign) * arm, soft(a), soft(b), soft(c)],
        }
    }
    pub fn candidate(
        &self,
        x: &[DVec3],
        p: &[Particle],
        g: Guide,
        queries: &mut usize,
    ) -> Result<Option<Candidate>, XpbdError> {
        // Capsule coordinates remove the common translation. A translating guide
        // then has fixed core endpoints, including during a high-speed crossing.
        let predicted = |i: usize| x[i] - (g.center + g.delta);
        let initial = |i: usize| DVec3::from(p[i].position) - g.center;
        let velocity = |i: usize| DVec3::from(p[i].velocity) - g.velocity;
        let (start, end, speed, face) = match *self {
            Self::Edge([a, b]) => (
                [initial(a), initial(b), -g.arm, g.arm],
                [predicted(a), predicted(b), -g.arm, g.arm],
                [velocity(a), velocity(b), DVec3::ZERO, DVec3::ZERO],
                false,
            ),
            Self::FaceCap([a, b, c], sign) => {
                let tip = f64::from(sign) * g.arm;
                (
                    [tip, initial(a), initial(b), initial(c)],
                    [tip, predicted(a), predicted(b), predicted(c)],
                    [DVec3::ZERO, velocity(a), velocity(b), velocity(c)],
                    true,
                )
            }
        };
        // AABB clearance is a certificate for the entire linear sweep.
        let n = if face { 1 } else { 2 };
        let bounds = |lo: usize, hi: usize| {
            let mut min = DVec3::splat(f64::INFINITY);
            let mut max = -min;
            for i in lo..hi {
                min = min.min(start[i]).min(end[i]);
                max = max.max(start[i]).max(end[i]);
            }
            (min, max)
        };
        let (a0, a1) = bounds(0, n);
        let (b0, b1) = bounds(n, 4);
        let rounding = 64.
            * f64::EPSILON
            * [a0, a1, b0, b1]
                .iter()
                .map(|v| v.abs().max_element())
                .fold(g.gap, f64::max);
        if (a0 - b1).max_element() > g.gap + rounding || (b0 - a1).max_element() > g.gap + rounding
        {
            return Ok(None);
        }
        let scale = start
            .iter()
            .chain(&end)
            .map(|v| v.length())
            .fold(g.gap, f64::max);
        let eps = 128. * f64::EPSILON * scale + 1e-10 * g.gap;
        let Some(hit) = material_sweep(start, end, speed, face, g.gap, eps, queries)? else {
            return Ok(None);
        };
        let (feature, weights, part, normal) = self.describe(hit.weights, hit.normal);
        let initial_distance = if face {
            vertex_face(&start)
        } else {
            edge_edge(&start)
        }
        .delta
        .length();
        let overlap = (g.gap - initial_distance).max(0.);
        let c = Candidate {
            feature,
            weights,
            part,
            normal,
            time: hit.time,
            depth: hit.depth,
            overlap,
            tolerance: eps,
        };
        if !hit.speed.is_finite() {
            return Err(XpbdError::Numerical);
        }
        Ok(Some(c))
    }
}
/// An axis piercing the initial face has no determined incident side from
/// which to depenetrate. Edge/cap witnesses alone can all be far apart.
pub(super) fn pierces(face: [DVec3; 3], center: DVec3, arm: DVec3) -> bool {
    let [a, b, c] = face.map(|q| q - center);
    let n = (b - a).cross(c - a);
    let d0 = (-arm - a).dot(n);
    let d1 = (arm - a).dot(n);
    if !((d0 < 0. && d1 > 0.) || (d1 < 0. && d0 > 0.)) {
        return false;
    }
    let at = -arm + 2. * arm * (d0 / (d0 - d1));
    let w = vertex_face(&[at, a, b, c]);
    let scale = a.length().max(b.length()).max(c.length()).max(arm.length());
    w.delta.length() <= 128. * f64::EPSILON * scale
}

impl Stencil {
    /// Exactly parallel finite segments can touch along a whole interval.
    /// Its two extremal material witnesses constrain every point in between
    /// and preserve the centered load on a horizontal cable laid along a guide.
    pub fn manifold(
        &self,
        hit: Candidate,
        x: &[DVec3],
        p: &[Particle],
        g: Guide,
    ) -> [Option<Candidate>; 2] {
        let Self::Edge([a, b]) = *self else {
            return [Some(hit), None];
        };
        let start = [a, b].map(|i| DVec3::from(p[i].position) - g.center);
        let end = [a, b].map(|i| x[i] - (g.center + g.delta));
        let at = std::array::from_fn::<_, 2, _>(|i| start[i].lerp(end[i], hit.time));
        let direction = at[1] - at[0];
        let length2 = direction.length_squared();
        let axis2 = g.arm.length_squared();
        if length2 <= f64::MIN_POSITIVE
            || direction.cross(g.arm).length() > 128. * f64::EPSILON * length2.sqrt() * axis2.sqrt()
        {
            return [Some(hit), None];
        }
        let u = (-g.arm - at[0]).dot(direction) / length2;
        let v = (g.arm - at[0]).dot(direction) / length2;
        let lo = u.min(v).max(0.);
        let hi = u.max(v).min(1.);
        if hi - lo <= 1e-12 {
            return [Some(hit), None];
        }
        [lo, hi].map(|s| {
            let soft = at[0] + s * direction;
            let axial = (soft.dot(g.arm) / axis2).clamp(-1., 1.);
            let core = g.arm * axial;
            let (feature, weights) = MaterialFeature::from_weights([a, b, b], [1. - s, s, 0.]);
            let normal = (soft - core).try_normalize().unwrap_or(hit.normal);
            let depth = (g.gap - (end[0].lerp(end[1], s) - core).dot(normal)).max(0.);
            let overlap = (g.gap - start[0].lerp(start[1], s).distance(core)).max(0.);
            let part = if axial < -1. + 1e-10 {
                CapsuleRegion::NegativeCap
            } else if axial > 1. - 1e-10 {
                CapsuleRegion::PositiveCap
            } else {
                CapsuleRegion::Barrel
            };
            Some(Candidate {
                feature,
                weights,
                part,
                normal,
                depth,
                overlap,
                ..hit
            })
        })
    }
}
