// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Continuous finite-sphere contact against a thick triangular shell.
use super::{
    BoundaryImpulse, Particle, PreparedBoundary, XpbdError,
    collision::Interval,
    coupled::Receiver,
    surface::{first_entry, geometry::vertex_face},
};
use glam::DVec3;
use std::collections::BTreeMap;

const MAX_FACES: usize = 512;

const MAX_CONTACTS: usize = 128;

pub(super) mod normal;
use crate::material::{Skin, patches};
#[derive(Clone, Copy)]
struct Candidate {
    boundary: usize,
    ids: [usize; 3],
    weights: [f64; 3],
    normal: DVec3,
    overlap: f64,
    depth: f64,
    time: f64,
}
#[derive(Clone)]
struct Contact {
    ids: [usize; 3],
    weights: [f64; 3],
    normal: DVec3,
    normal_impulse: f64,
    tangent_impulse: DVec3,
    overlap: f64,
}
#[derive(Clone)]
pub(super) struct Pads {
    faces: Vec<[usize; 3]>,
    patches: BTreeMap<[usize; 3], Vec<usize>>,
    contacts: BTreeMap<(usize, [usize; 3]), Contact>,
}
fn blend(weights: [f64; 3], q: [DVec3; 3]) -> DVec3 {
    q[0] + weights[1] * (q[1] - q[0]) + weights[2] * (q[2] - q[0])
}
fn feature(ids: [usize; 3], weights: [f64; 3]) -> [usize; 3] {
    let mut key = [usize::MAX; 3];
    for (slot, &id) in key.iter_mut().zip(
        ids.iter()
            .zip(weights)
            .filter(|(_, w)| *w > 1e-10)
            .map(|(id, _)| id),
    ) {
        *slot = id;
    }
    key
}
impl Pads {
    pub fn new(faces: Vec<[usize; 3]>) -> Result<Self, XpbdError> {
        if faces.len() > MAX_FACES {
            return Err(XpbdError::SurfaceBudget);
        }
        Ok(Self {
            patches: patches(&faces)?,
            faces,
            contacts: BTreeMap::new(),
        })
    }
    pub fn clear(&mut self) {
        self.contacts.clear();
    }
    pub fn project(
        &mut self,
        x: &mut [DVec3],
        p: &mut [Particle],
        b: &mut [PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
        dt: Interval,
        mu: f64,
    ) -> Result<(), XpbdError> {
        self.sweep(x, p, b, reactions, dt, false)?;
        self.sticking(x, p, b, reactions, dt, mu)
    }
    fn sweep(
        &mut self,
        x: &mut [DVec3],
        p: &mut [Particle],
        b: &mut [PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
        dt: Interval,
        endpoint: bool,
    ) -> Result<(), XpbdError> {
        let mut queries = 0;
        for _ in 0..128 {
            let mut selected: Option<Candidate> = None;
            for (k, b) in b.iter_mut().enumerate() {
                let Some(radius) = b.sphere_radius() else {
                    continue;
                };
                let r = b.receiver.as_mut().ok_or(XpbdError::InvalidBoundary)?;
                let c0 = b.origin + b.velocity * dt.t0 + r.start_offset;
                let gap = radius + dt.radius;
                let eps = 1e-9 * gap + 128. * f64::EPSILON * c0.abs().max_element();
                if eps > 1e-4 * gap {
                    return Err(XpbdError::Numerical);
                }
                for &ids in &self.faces {
                    let c1 = b.origin + b.velocity * dt.t1 + r.end_offset;
                    let [a, bp, c] = ids.map(|i| DVec3::from(p[i].position));
                    let [u, v, w] = ids.map(|i| x[i]);
                    let lo = a.min(bp).min(c).min(u).min(v).min(w);
                    let hi = a.max(bp).max(c).max(u).max(v).max(w);
                    if (c0.min(c1) - hi).cmpgt(DVec3::splat(gap + eps)).any()
                        || (lo - c0.max(c1)).cmpgt(DVec3::splat(gap + eps)).any()
                    {
                        continue;
                    }
                    let start = [c0, a, bp, c];
                    let end = [c1, u, v, w];
                    let (normal, weights, time) = if endpoint {
                        let w = vertex_face(&end);
                        let d = w.delta.length();
                        if d >= gap - eps {
                            continue;
                        }
                        if d <= eps {
                            return Err(XpbdError::UnresolvedSurface);
                        }
                        (
                            -w.delta / d,
                            [-w.weights[1], -w.weights[2], -w.weights[3]],
                            0.,
                        )
                    } else {
                        let Some(hit) =
                            first_entry(start, end, true, gap, eps.max(1e-7 * gap), &mut queries)?
                        else {
                            continue;
                        };
                        let w = vertex_face(&end);
                        let incident = (
                            -hit.normal,
                            [-hit.weights[1], -hit.weights[2], -hit.weights[3]],
                            hit.time,
                        );
                        if hit.time == 0.
                            && w.delta.dot(hit.normal) > 0.
                            && w.delta.length() <= gap + eps
                        {
                            let d = w.delta.length();
                            if d <= eps {
                                return Err(XpbdError::UnresolvedSurface);
                            }
                            (
                                -w.delta / d,
                                [-w.weights[1], -w.weights[2], -w.weights[3]],
                                0.,
                            )
                        } else {
                            // A starting contact can close and leave an edge within one
                            // interval. Its incident half-space still limits the motion.
                            incident
                        }
                    };
                    let depth = (gap - (blend(weights, [u, v, w]) - c1).dot(normal)).max(0.);
                    let overlap = (gap - vertex_face(&start).delta.length()).max(0.);
                    let candidate = Candidate {
                        boundary: k,
                        ids,
                        weights,
                        normal,
                        depth,
                        time,
                        overlap,
                    };
                    if depth > eps {
                        if selected.is_none_or(|old| {
                            time.total_cmp(&old.time)
                                .then(old.depth.total_cmp(&depth))
                                .is_lt()
                        }) {
                            selected = Some(candidate);
                        }
                    } else if !endpoint {
                        self.contacts
                            .entry((k, feature(ids, weights)))
                            .or_insert(Contact {
                                ids,
                                weights,
                                normal,
                                normal_impulse: 0.,
                                tangent_impulse: DVec3::ZERO,
                                overlap,
                            });
                    }
                }
            }
            if self.contacts.len() > MAX_CONTACTS {
                return Err(XpbdError::SurfaceBudget);
            }
            let Some(first) = selected else {
                return Ok(());
            };
            let own = |row: Candidate| {
                row.ids
                    .into_iter()
                    .zip(row.weights)
                    .map(|(i, f)| p[i].inverse_mass * f * f)
                    .sum::<f64>()
                    + b[row.boundary].receiver.as_ref().unwrap().inverse_mass
            };
            let aa = own(first);
            let mut response = [(first, first.depth / aa); 2];
            let mut count = 1;
            // A light membrane between two heavy opposing pads is poorly
            // conditioned for scalar projection. Predict which opposing row
            // the first correction would violate and solve both simultaneously.
            let mut worst = 0.0_f64;
            for (k, pad) in b.iter().enumerate() {
                if k == first.boundary {
                    continue;
                }
                let Some(radius) = pad.sphere_radius() else {
                    continue;
                };
                let r = pad.receiver.as_ref().unwrap();
                let center = pad.origin + dt.t1 * pad.velocity + r.end_offset;
                let gap = radius + dt.radius;
                for &ids in &self.faces {
                    if !ids.iter().any(|id| first.ids.contains(id)) {
                        continue;
                    }
                    let [a, b, c] = ids.map(|i| x[i]);
                    let w = vertex_face(&[center, a, b, c]);
                    let d = w.delta.length();
                    if d <= 1e-12 * gap {
                        return Err(XpbdError::UnresolvedSurface);
                    }
                    let n = -w.delta / d;
                    let weights = [-w.weights[1], -w.weights[2], -w.weights[3]];
                    let mut cross = 0.;
                    for (i, fi) in first.ids.into_iter().zip(first.weights) {
                        if let Some(j) = ids.iter().position(|&id| id == i) {
                            cross += p[i].inverse_mass * fi * weights[j];
                        }
                    }
                    cross *= n.dot(first.normal);
                    if cross >= 0. {
                        continue;
                    }
                    let depth = gap - d;
                    let violation = depth - cross * first.depth / aa;
                    if violation <= worst.max(1e-10 * gap) {
                        continue;
                    }
                    let second = Candidate {
                        boundary: k,
                        ids,
                        weights,
                        normal: n,
                        depth,
                        time: first.time,
                        overlap: 0.,
                    };
                    let bb = own(second);
                    let det = aa * bb - cross * cross;
                    if !det.is_finite() || det <= 1e-12 * aa * bb {
                        return Err(XpbdError::UnresolvedSurface);
                    }
                    worst = violation;
                    response = [
                        (first, (bb * first.depth - cross * depth) / det),
                        (second, violation * aa / det),
                    ];
                    count = 2;
                }
            }
            for (
                Candidate {
                    boundary: k,
                    ids,
                    weights,
                    normal,
                    overlap,
                    ..
                },
                lambda,
            ) in response.into_iter().take(count)
            {
                let j = lambda / dt.h;
                if !j.is_finite() || j < 0. {
                    return Err(XpbdError::Numerical);
                }
                let impulse = normal * j;
                for (id, f) in ids.into_iter().zip(weights) {
                    let dx = dt.h * p[id].inverse_mass * f * impulse;
                    super::correct(&mut x[id], &mut p[id], dx, dt.h);
                }
                let boundary = &mut b[k];
                let r = boundary.receiver.as_mut().unwrap();
                let c0 = boundary.origin + dt.t0 * boundary.velocity + r.start_offset;
                let q = blend(weights, ids.map(|i| p[i].position.into()));
                r.project_normal(q - c0, -impulse, dt.h);
                reactions[k].react(q - boundary.origin, impulse)?;
                let contact = self
                    .contacts
                    .entry((k, feature(ids, weights)))
                    .or_insert(Contact {
                        ids,
                        weights,
                        normal,
                        normal_impulse: 0.,
                        tangent_impulse: DVec3::ZERO,
                        overlap,
                    });
                contact.ids = ids;
                contact.weights = weights;
                contact.normal = normal;
                contact.normal_impulse += j;
            }
        }
        Err(XpbdError::SurfaceBudget)
    }

    fn sticking(
        &mut self,
        x: &mut [DVec3],
        p: &mut [Particle],
        b: &mut [PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
        dt: Interval,
        mu: f64,
    ) -> Result<(), XpbdError> {
        if mu == 0. || self.contacts.is_empty() {
            return Ok(());
        }
        for (&(k, key), row) in &mut self.contacts {
            if row.normal_impulse <= 0. {
                continue;
            }
            let pad = &mut b[k];
            let radius = pad.sphere_radius().unwrap();
            let r = pad.receiver.as_mut().unwrap();
            let center = pad.origin + dt.t0 * pad.velocity + r.start_offset;
            let start = row.ids.map(|i| DVec3::from(p[i].position));
            let gap = radius + dt.radius;
            let old = vertex_face(&[center, start[0], start[1], start[2]]);
            if old.delta.length() > gap + 1e-5 * gap {
                continue;
            }
            let arm = row.normal * radius;
            let point = center + arm;
            let q = blend(row.weights, start);
            let patch = Skin::new(
                &self.patches[&key],
                row.ids,
                row.weights,
                |i| p[i].position.into(),
                point - q,
            )?;
            let relative =
                patch.velocity(p) - pad.velocity - r.linear - r.angular_velocity().cross(arm);
            let tangent = relative - row.normal * relative.dot(row.normal);
            let speed = tangent.length();
            if speed <= 1e-14 {
                continue;
            }
            let dir = tangent / speed;
            let trial =
                row.tangent_impulse - dir * (speed / patch.inverse_along(p, r, arm, dir)?);
            let projected = trial - row.normal * trial.dot(row.normal);
            let cap = mu * row.normal_impulse;
            let limited = projected * (cap / projected.length().max(f64::MIN_POSITIVE)).min(1.);
            let j = limited - row.tangent_impulse;
            row.tangent_impulse = limited;
            for (id, impulse) in patch.nodal(j) {
                let dx = dt.h * p[id].inverse_mass * impulse;
                super::correct(&mut x[id], &mut p[id], dx, dt.h);
            }
            r.project_skin_tangent(arm, -j, dt.h)?;
            reactions[k].react(point - pad.origin, j)?;
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn finish(
        &self,
        x: &[DVec3],
        p: &mut [Particle],
        boundaries: &mut [PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
        mu: f64,
        h: f64,
        skin: f64,
    ) -> Result<(), XpbdError> {
        let totals = normal::resolve(&self.contacts, x, p, boundaries, reactions, h, skin)?;
        if mu == 0. {
            return Ok(());
        }
        for (&(k, key), contact) in &self.contacts {
            let b = &mut boundaries[k];
            let radius = b.sphere_radius().unwrap();
            let r = b.receiver.as_mut().unwrap();
            let center = b.origin + b.velocity * r.end_time + r.end_offset;
            let q = blend(contact.weights, contact.ids.map(|i| x[i]));
            let n = contact.normal;
            let arm = n * radius;
            let point = center + arm;
            let Some(&total) = totals.get(&(k, key)) else {
                continue;
            };
            // A grazing contact can detach before the end of a long interval.
            // Its tangential impulse belongs at the impact point. Refuse a
            // stale material witness beyond a quarter of the skin thickness.
            let offset = point - q;
            if (offset - n * offset.dot(n)).length() > 0.25 * skin {
                return Err(XpbdError::UnresolvedSurface);
            }
            let patch = Skin::new(
                &self.patches[&key],
                contact.ids,
                contact.weights,
                |i| x[i],
                point - q,
            )?;
            let relative = |r: &Receiver, p: &[Particle]| {
                patch.velocity(p) - b.velocity - r.linear - r.angular_velocity().cross(arm)
            };
            let rel = relative(r, p);
            let tangent = rel - n * rel.dot(n);
            let speed = tangent.length();
            let mut trial = contact.tangent_impulse - n * contact.tangent_impulse.dot(n);
            if speed > 1e-14 {
                let dir = tangent / speed;
                trial -= dir * (speed / patch.inverse_along(p, r, arm, dir)?);
            }
            let cap = mu * total;
            let result = trial * (cap / trial.length().max(f64::MIN_POSITIVE)).min(1.);
            let j = result - contact.tangent_impulse;
            patch.apply(j, p);
            r.apply_velocity_impulse(arm, -j);
            reactions[k].react(point - b.origin, j)?;
        }
        Ok(())
    }
    fn separated(&self, x: &[DVec3], boundaries: &[PreparedBoundary], skin: f64) -> bool {
        for b in boundaries.iter() {
            let Some(radius) = b.sphere_radius() else {
                continue;
            };
            let r = b.receiver.as_ref().unwrap();
            let center = b.origin + b.velocity * r.end_time + r.end_offset;
            let gap = radius + skin;
            for &ids in &self.faces {
                let [a, b, c] = ids.map(|i| x[i]);
                if vertex_face(&[center, a, b, c]).delta.length() < gap - 1e-7 * gap {
                    return false;
                }
            }
        }
        true
    }
    pub fn polish(
        &mut self,
        x: &mut [DVec3],
        p: &mut [Particle],
        b: &mut [PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
        dt: Interval,
    ) -> Result<(), XpbdError> {
        for _ in 0..32 {
            if self.separated(x, b, dt.radius) {
                for boundary in &*b {
                    if let Some((origin, n)) = boundary.end_plane(dt.t1)
                        && x.iter()
                            .any(|q| (*q - origin).dot(n) < dt.radius * (1. - 1e-6))
                    {
                        return Err(XpbdError::UnresolvedSurface);
                    }
                }
                return Ok(());
            }
            self.sweep(x, p, b, reactions, dt, true)?;
        }
        Err(XpbdError::UnresolvedSurface)
    }
}
