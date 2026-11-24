// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Physical warm material cuts for recurring contacts on a deforming surface.
use super::*;

#[derive(Clone, Copy)]
struct Separation {
    primitive: [usize; 3],
    feature: MaterialFeature,
    gap: f64,
}
#[derive(Clone)]
pub struct MaterialSeparations {
    rows: Vec<Separation>,
    per_sphere: usize,
    minimum: f64,
}
impl MaterialSeparations {
    pub fn sphere_minima(&self) -> Vec<f64> {
        self.rows
            .chunks_exact(self.per_sphere)
            .map(|rows| rows.iter().map(|r| r.gap).fold(f64::INFINITY, f64::min))
            .collect()
    }
    /// Signed distance of each loaded material region. One isolated contact
    /// may move across an internal simplex boundary. Multiple physical
    /// contacts on one sphere retain their distinct construction simplices.
    /// The second value bounds all other unrepresented regions.
    pub fn support_clearances(
        &self,
        active: &[(usize, MaterialFeature)],
    ) -> Result<(Vec<f64>, f64), XpbdError> {
        let n = self.rows.len() / self.per_sphere;
        if active.len() > MAX_ROWS
            || active
                .iter()
                .enumerate()
                .any(|(i, r)| r.0 >= n || active[..i].contains(r))
        {
            return Err(XpbdError::InvalidConfig);
        }
        let mut distances = vec![f64::INFINITY; active.len()];
        let mut outside = f64::INFINITY;
        for (sphere, items) in self.rows.chunks_exact(self.per_sphere).enumerate() {
            if !active.iter().any(|r| r.0 == sphere) {
                outside = items.iter().map(|r| r.gap).fold(outside, f64::min);
                continue;
            }
            for r in items {
                let mut covered = false;
                for (id, (_, feature)) in active.iter().enumerate().filter(|(_, r)| r.0 == sphere) {
                    let direct = feature.vertices().iter().all(|v| r.primitive.contains(v));
                    // Distances to a contained edge/vertex cannot constrain
                    // separately from the closer interior of its own simplex.
                    let subset = r
                        .feature
                        .vertices()
                        .iter()
                        .all(|v| feature.vertices().contains(v));
                    covered |= direct || subset;
                    if direct || subset {
                        distances[id] = distances[id].min(r.gap);
                    }
                }
                if !covered {
                    outside = outside.min(r.gap);
                }
            }
        }
        if distances.iter().any(|v| !v.is_finite()) {
            return Err(XpbdError::UnresolvedImpact);
        }
        Ok((distances, outside))
    }
    pub fn minimum(&self) -> f64 {
        self.minimum
    }
}
impl SoftBody {
    fn same_swept_topology(&self, other: &Self) -> bool {
        self.particles.len() == other.particles.len()
            && self.config.particle_radius == other.config.particle_radius
            && match (self.topology, other.topology) {
                (MaterialTopology::Cable, MaterialTopology::Cable) => true,
                (MaterialTopology::Cloth(a), MaterialTopology::Cloth(b)) => a == b,
                _ => false,
            }
    }
    pub(super) fn primitive_count(&self) -> Result<usize, XpbdError> {
        match self.topology {
            MaterialTopology::Cable => Ok(self.particles.len() - 1),
            MaterialTopology::Cloth(c) => Ok(2 * (self.particles.len() / c - 1) * (c - 1)),
            _ => Err(XpbdError::InvalidConfig),
        }
    }
    pub(super) fn primitive(&self, i: usize) -> Primitive {
        match self.topology {
            MaterialTopology::Cable => Primitive::Segment([i, i + 1]),
            MaterialTopology::Cloth(c) => {
                let cell = i / 2;
                let br = (cell / (c - 1) + 1) * c + cell % (c - 1) + 1;
                Primitive::Triangle(cell_faces(br, c)[i % 2])
            }
            _ => unreachable!(),
        }
    }
    /// Complete signed separation at a real native/material cut. Duplicate
    /// features remain indexed by construction element so successive cuts
    /// can detect a change of nearest feature without losing its history.
    pub fn articulated_separations(
        &self,
        spheres: &[ImpactSphere],
    ) -> Result<MaterialSeparations, XpbdError> {
        let n = self.primitive_count()?;
        if !self.supports_articulated_impact()
            || spheres.is_empty()
            || spheres.len() > 16
            || n > MAX_QUERIES / spheres.len()
        {
            return Err(XpbdError::ImpactBudget);
        }
        let mut rows = Vec::with_capacity(n * spheres.len());
        let mut minimum = f64::INFINITY;
        for sphere in spheres {
            let c = DVec3::from(sphere.center);
            let gap = sphere.radius + self.config.particle_radius;
            if !c.is_finite() || !gap.is_finite() || sphere.radius <= 0. {
                return Err(XpbdError::InvalidBoundary);
            }
            for k in 0..n {
                let e = self.primitive(k);
                let (w, d) = e.witness(&self.particles, c);
                let d = d.length() - gap;
                if !d.is_finite() || w.iter().any(|x| !x.is_finite()) {
                    return Err(XpbdError::Numerical);
                }
                let (feature, _) = MaterialFeature::from_weights(e.vertices(), w);
                rows.push(Separation {
                    primitive: e.vertices(),
                    feature,
                    gap: d,
                });
                minimum = minimum.min(d);
            }
        }
        Ok(MaterialSeparations {
            per_sphere: n,
            rows,
            minimum,
        })
    }

    /// Conservative CCD on the linearly swept simplices of a numerical
    /// native/XPBD substep. A true non-linear subinterval is reintegrated
    /// before any impulse. Return the earliest candidate fraction.
    pub fn deforming_chord_candidate(
        &self,
        next: &Self,
        starts: &[ImpactSphere],
        ends: &[ImpactSphere],
    ) -> Result<Option<f64>, XpbdError> {
        self.deforming_chord_outside_support(next, starts, ends, &[])
    }
    /// Test all other construction primitives, including on a sphere already
    /// carrying a load. Adjacent primitives containing the same active edge
    /// or node share that witness.
    pub fn deforming_chord_outside_support(
        &self,
        next: &Self,
        starts: &[ImpactSphere],
        ends: &[ImpactSphere],
        active: &[(usize, MaterialFeature)],
    ) -> Result<Option<f64>, XpbdError> {
        if active.len() > MAX_ROWS || active.iter().any(|(i, _)| *i >= starts.len()) {
            return Err(XpbdError::InvalidConfig);
        }
        let n = self.primitive_count()?;
        if starts.is_empty()
            || starts.len() > 16
            || starts.len() != ends.len()
            || n > MAX_QUERIES / starts.len()
            || !self.same_swept_topology(next)
        {
            return Err(XpbdError::ImpactBudget);
        }
        let mut queries = 0;
        let mut candidate = 1.0_f64;
        let mut found = false;
        for (source, (a, b)) in starts.iter().zip(ends).enumerate() {
            if a.radius != b.radius
                || !a.radius.is_finite()
                || a.radius <= 0.
                || !DVec3::from(a.center).is_finite()
                || !DVec3::from(b.center).is_finite()
            {
                return Err(XpbdError::InvalidBoundary);
            }
            let gap = a.radius + self.config.particle_radius;
            for k in 0..n {
                let primitive = self.primitive(k);
                let ids = primitive.vertices();
                if active
                    .iter()
                    .any(|(i, f)| *i == source && f.vertices().iter().all(|v| ids.contains(v)))
                {
                    continue;
                }
                let vf = ids[2] != usize::MAX;
                let stencil = |nodes: &[crate::Particle], center: [f64; 3]| {
                    let p = |i: usize| DVec3::from(nodes[i].position);
                    let c = DVec3::from(center);
                    if vf {
                        [c, p(ids[0]), p(ids[1]), p(ids[2])]
                    } else {
                        [c, c, p(ids[0]), p(ids[1])]
                    }
                };
                // A supporting plane at either cut separates the complete
                // linearly swept simplex when every vertex stays strictly on
                // its exterior side. This also resolves almost-tangent
                // positive distances inside the generic CCD search margin.
                let separated = |anchor: &[crate::Particle], center: [f64; 3]| {
                    let (_, delta) = primitive.witness(anchor, DVec3::from(center));
                    let Some(normal) = delta.try_normalize() else {
                        return false;
                    };
                    [(&self.particles, a.center), (&next.particles, b.center)]
                        .into_iter()
                        .all(|(nodes, c)| {
                            let c = DVec3::from(c);
                            ids.iter().copied().filter(|i| *i != usize::MAX).all(|i| {
                                let rel = DVec3::from(nodes[i].position) - c;
                                let error =
                                    16. * f64::EPSILON * (rel.abs().dot(normal.abs()) + gap);
                                rel.dot(normal) - gap > error
                            })
                        })
                };
                if separated(&self.particles, a.center) || separated(&next.particles, b.center) {
                    continue;
                }
                let mut s = stencil(&self.particles, a.center);
                let e = stencil(&next.particles, b.center);
                let scale = s
                    .iter()
                    .chain(&e)
                    .map(|p| p.abs().max_element())
                    .fold(gap, f64::max);
                let eps = 128. * f64::EPSILON * (scale + gap);
                let mut offset = 0.;
                let (weights, dist) = primitive.witness(&self.particles, DVec3::from(a.center));
                let initial = dist.length() - gap;
                if initial <= eps {
                    let relative = ids
                        .iter()
                        .zip(weights)
                        .filter(|(id, _)| **id != usize::MAX)
                        .map(|(&i, w)| {
                            w * (DVec3::from(next.particles[i].position)
                                - DVec3::from(self.particles[i].position))
                        })
                        .sum::<DVec3>()
                        - (DVec3::from(b.center) - DVec3::from(a.center));
                    let opening = dist
                        .try_normalize()
                        .ok_or(XpbdError::UnresolvedImpact)?
                        .dot(relative);
                    if opening <= 0. {
                        found = true;
                        candidate = 0.;
                        continue;
                    }
                    // Depart the shell by a measured floating-point gap before
                    // looking for a fresh entry of this same material element.
                    offset = ((4. * eps - initial) / opening).clamp(0., 1.);
                    if offset == 1. {
                        continue;
                    }
                    for i in 0..4 {
                        s[i] = s[i].lerp(e[i], offset);
                    }
                }
                if let Some(entry) =
                    crate::surface::first_entry_mode(s, e, vf, gap, eps, &mut queries, true)?
                {
                    let t = offset + (1. - offset) * entry.time;
                    let shift: [DVec3; 4] = std::array::from_fn(|i| e[i] - s[i]);
                    let relative = entry
                        .weights
                        .iter()
                        .zip(shift)
                        .map(|(w, v)| *w * v)
                        .sum::<DVec3>();
                    if relative.dot(entry.normal) < 0. {
                        found = true;
                        candidate = candidate.min(t);
                    }
                }
            }
        }
        Ok(found.then_some(candidate))
    }

    /// Authorize only the actually crossing features on two contiguous
    /// integer-nanosecond solver cuts. Apply at the measured earlier cut
    /// (still outside) and report its subnanosecond interpolated crossing.
    pub fn bracketed_articulated_return(
        &self,
        spheres: &[ImpactSphere],
        next: &Self,
        next_spheres: &[ImpactSphere],
        bracket_width: Duration,
        since_public_start: Duration,
    ) -> Result<MaterialImpactTime, XpbdError> {
        if !self.same_swept_topology(next)
            || spheres.len() != next_spheres.len()
            || spheres
                .iter()
                .zip(next_spheres)
                .any(|(a, b)| a.radius != b.radius)
        {
            return Err(XpbdError::InvalidBoundary);
        }
        let begin = self.articulated_separations(spheres)?;
        let after = next.articulated_separations(next_spheres)?;
        if bracket_width != Duration::from_nanos(1)
            || begin.rows.len() != after.rows.len()
            || begin.per_sphere != after.per_sphere
            || begin.minimum < -1e-12
            || after.minimum > 0.
        {
            return Err(XpbdError::UnresolvedImpact);
        }
        let mut first = 1.0_f64;
        let mut entries = Vec::new();
        for (i, (a, b)) in begin.rows.iter().zip(&after.rows).enumerate() {
            if a.gap >= -1e-13 && b.gap <= 0. && a.gap > b.gap {
                let fraction = (a.gap / (a.gap - b.gap)).clamp(0., 1.);
                first = first.min(fraction);
                entries.push((fraction, (i / begin.per_sphere, a.feature)));
            }
        }
        if entries.is_empty() {
            return Err(XpbdError::UnresolvedImpact);
        }
        let rows = entries
            .into_iter()
            .filter(|(t, _)| *t <= first + 2e-4)
            .map(|(_, k)| k)
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        if rows.len() > MAX_ROWS {
            return Err(XpbdError::ImpactBudget);
        }
        // For a simplex the motion of any fixed barycentric witness is a
        // convex combination of nodal motions. Bound all relative travel in
        // the complete 1 ns numerical bracket, including tangential motion.
        let mut margin = 0.0_f64;
        for &(sphere, feature) in &rows {
            let dc = DVec3::from(next_spheres[sphere].center) - DVec3::from(spheres[sphere].center);
            for &i in feature.vertices() {
                let dx = DVec3::from(next.particles[i].position)
                    - DVec3::from(self.particles[i].position);
                margin = margin.max((dx - dc).length());
            }
        }
        let nodes = rows
            .iter()
            .flat_map(|(_, f)| f.vertices().iter().copied())
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .map(|id| (id, self.particles[id].position))
            .collect();
        Ok(MaterialImpactTime {
            head: since_public_start,
            seconds: since_public_start.as_secs_f64() + first * 1e-9,
            rounding_distance: margin,
            nodes,
            spheres: spheres.to_vec(),
            rows,
        })
    }
}
