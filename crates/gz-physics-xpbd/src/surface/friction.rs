// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Tangential impulse between two physical material skins at a common point.
use super::loads::NodeImpulse;
use super::{
    Hit, Particle, XpbdError,
    geometry::{edge_edge, vertex_face},
};
use crate::material::{Skin, patches};
use glam::{DVec2, DVec3};
use std::collections::BTreeMap;

#[derive(Clone)]
pub(super) struct Friction {
    coefficient: f64,
    patches: BTreeMap<[usize; 3], Vec<usize>>,
}

struct Side {
    ids: [usize; 3],
    weights: [f64; 3],
    feature: [usize; 3],
}

impl Side {
    fn new(ids: &[usize], weights: &[f64]) -> Self {
        let mut side = Self {
            ids: [usize::MAX; 3],
            weights: [0.; 3],
            feature: [usize::MAX; 3],
        };
        side.ids[..ids.len()].copy_from_slice(ids);
        let mut active = 0;
        for (i, &w) in weights.iter().enumerate() {
            side.weights[i] = w.abs();
            if w.abs() > 1e-10 {
                side.feature[active] = ids[i];
                active += 1;
            }
        }
        side.feature.sort_unstable();
        side
    }
    fn point(&self, x: impl Fn(usize) -> DVec3) -> DVec3 {
        let origin = x(self.ids[0]);
        origin
            + self
                .ids
                .iter()
                .zip(self.weights)
                .filter(|(id, _)| **id != usize::MAX)
                .map(|(&id, w)| w * (x(id) - origin))
                .sum::<DVec3>()
    }
}

impl Friction {
    pub fn new(coefficient: f64, faces: &[[usize; 3]]) -> Result<Self, XpbdError> {
        Ok(Self {
            coefficient,
            patches: patches(faces)?,
        })
    }

    /// A common material point transfers a normal impulse at the event.
    /// Apply the corresponding displacement over the remaining flight time.
    /// Using x - tau*v for every nodal arm makes its orbital contribution
    /// cancel the angular impulse of that displacement exactly.
    pub fn normal(
        &self,
        hit: &Hit,
        x: &mut [DVec3],
        p: &mut [Particle],
        h: f64,
        mut loads: Option<&mut super::loads::Loads>,
    ) -> Result<f64, XpbdError> {
        let split = if hit.stencil.vf { 1 } else { 2 };
        let a = Side::new(&hit.stencil.ids[..split], &hit.weights[..split]);
        let b = Side::new(&hit.stencil.ids[split..], &hit.weights[split..]);
        let supports = [
            self.patches[&a.feature].as_slice(),
            self.patches[&b.feature].as_slice(),
        ];
        // Pinned material supplies an external reaction. Use the ordinary
        // normal signed-distance gradient for that constrained system.
        if supports
            .into_iter()
            .flatten()
            .any(|&i| p[i].inverse_mass == 0.)
        {
            let impulse = hit.apply(x, p)?;
            if let Some(loads) = loads {
                for (&id, weight) in hit.stencil.ids.iter().zip(hit.weights) {
                    loads.node(id, NodeImpulse::Normal((weight * impulse) * hit.normal))?;
                }
            }
            return Ok(impulse);
        }
        let travel = hit.depth / hit.speed;
        let tau = if travel.is_finite() && travel <= h * (1. + 1e-6) {
            travel
        } else {
            super::position::correct(hit, supports, x, p)?;
            0.
        };
        if hit.speed == 0. {
            return Ok(0.);
        }
        let at = |i: usize| x[i] - tau * DVec3::from(p[i].velocity);
        let arm = 0.5 * (b.point(at) - a.point(at));
        let a = Skin::new(&self.patches[&a.feature], a.ids, a.weights, at, arm)?;
        let b = Skin::new(&self.patches[&b.feature], b.ids, b.weights, at, -arm)?;
        let mut directions = BTreeMap::<usize, DVec3>::new();
        for (id, dir) in a.nodal(hit.normal) {
            *directions.entry(id).or_default() += dir;
        }
        for (id, dir) in b.nodal(hit.normal) {
            *directions.entry(id).or_default() -= dir;
        }
        let mut v = 0.;
        let mut k = 0.;
        let reference = DVec3::from(p[hit.stencil.ids[0]].velocity);
        let mut local_energy = 0.;
        for (&id, &d) in &directions {
            let velocity = DVec3::from(p[id].velocity) - reference;
            v += d.dot(velocity);
            local_energy += 0.5 * velocity.length_squared() / p[id].inverse_mass;
            k += p[id].inverse_mass * d.length_squared();
        }
        if !k.is_finite() || k <= 0. {
            return Err(crate::observed_refusal());
        }
        let response = hit
            .stencil
            .ids
            .iter()
            .zip(hit.weights)
            .map(|(&i, w)| {
                w * p[i].inverse_mass * directions.get(&i).map_or(0., |d| d.dot(hit.normal))
            })
            .sum::<f64>();
        if !response.is_finite() || response <= 0. {
            return Err(crate::observed_refusal());
        }
        let j = hit.speed / response;
        let quadratic = 0.5 * j * j * k;
        let roundoff = 128. * f64::EPSILON * (local_energy + j * v.abs() + quadratic);
        if !local_energy.is_finite() {
            return Err(crate::observed_refusal());
        }
        if j * v + quadratic > roundoff {
            // A grazing feature may turn as its geometric overlap is removed.
            // Do not spend energy on the pre-correction witness. Repair only
            // its clearance at fixed velocity and let the bounded CCD search
            // recompute the closest features before any velocity impulse.
            if hit.depth > 0. {
                if tau > 0. {
                    super::position::correct(hit, supports, x, p)?;
                }
                return Ok(0.);
            }
            return Err(crate::observed_refusal());
        }
        for (id, d) in directions {
            let dv = p[id].inverse_mass * j * d;
            if let Some(loads) = loads.as_deref_mut() {
                loads.node(id, NodeImpulse::Normal(j * d))?;
            }
            p[id].velocity = (DVec3::from(p[id].velocity) + dv).to_array();
            x[id] += tau * dv;
        }
        Ok(j)
    }

    pub fn apply(
        &self,
        hit: &Hit,
        normal_impulse: f64,
        x: &[DVec3],
        p: &mut [Particle],
        gap: f64,
        mut loads: Option<&mut super::loads::Loads>,
    ) -> Result<(f64, f64), XpbdError> {
        if normal_impulse == 0. || self.coefficient == 0. {
            return Ok((0., 0.));
        }
        let ids = hit.stencil.ids;
        let end = ids.map(|i| x[i]);
        let witness = if hit.stencil.vf {
            vertex_face(&end)
        } else {
            edge_edge(&end)
        };
        let distance = witness.delta.length();
        // Complete friction on the current skin. A detached event needs a
        // shorter interval to resolve its tangential impulse at the impact.
        if (distance - gap).abs() > 0.02 * gap {
            return Err(crate::observed_refusal());
        }
        let n = witness.delta / distance;
        let split = if hit.stencil.vf { 1 } else { 2 };
        let a = Side::new(&ids[..split], &witness.weights[..split]);
        let b = Side::new(&ids[split..], &witness.weights[split..]);
        let arm = 0.5 * (b.point(|i| x[i]) - a.point(|i| x[i]));
        let skin = |s: &Side, offset| {
            Skin::new(
                &self.patches[&s.feature],
                s.ids,
                s.weights,
                |i| x[i],
                offset,
            )
        };
        let a = skin(&a, arm)?;
        let b = skin(&b, -arm)?;
        let velocity = a.velocity(p) - b.velocity(p);
        let tangent = velocity - n * velocity.dot(n);
        let speed = tangent.length();
        if speed <= 1e-13 * velocity.length().max(1.) {
            return Ok((0., 0.));
        }
        let u = tangent / speed;
        let v = n.cross(u);
        // The incident one-rings may overlap. Form the difference of their
        // Jacobians per material vertex before computing the effective mass.
        let mut rows = BTreeMap::<usize, [DVec3; 2]>::new();
        for (k, axis) in [u, v].into_iter().enumerate() {
            for (id, j) in a.nodal(axis) {
                rows.entry(id).or_default()[k] += j;
            }
            for (id, j) in b.nodal(axis) {
                rows.entry(id).or_default()[k] -= j;
            }
        }
        let mut k = [0.; 3];
        for (&id, &[a, b]) in &rows {
            let w = p[id].inverse_mass;
            k[0] += w * a.length_squared();
            k[1] += w * a.dot(b);
            k[2] += w * b.length_squared();
        }
        let j = coulomb(k, speed, self.coefficient * normal_impulse)?;
        for (id, [a, b]) in rows {
            let impulse = a * j.x + b * j.y;
            let dv = p[id].inverse_mass * impulse;
            if let Some(loads) = loads.as_deref_mut() {
                loads.node(id, NodeImpulse::Tangent(impulse))?;
            }
            p[id].velocity = (DVec3::from(p[id].velocity) + dv).to_array();
        }
        let work =
            -(speed * j.x + 0.5 * (k[0] * j.x * j.x + 2. * k[1] * j.x * j.y + k[2] * j.y * j.y));
        if !work.is_finite() || work < -128. * f64::EPSILON * speed * j.length() {
            return Err(XpbdError::Numerical);
        }
        Ok((j.length(), work.max(0.)))
    }
}

/// Maximize dissipation within a Euclidean Coulomb disk using the full
/// symmetric tangent-space effective mass. The tangent basis aligns with slip.
fn coulomb([aa, ab, bb]: [f64; 3], speed: f64, cap: f64) -> Result<DVec2, XpbdError> {
    let scale = aa.max(bb);
    if !scale.is_finite() || scale <= 0. || !cap.is_finite() {
        return Err(XpbdError::Numerical);
    }
    let (aa, ab, bb) = (aa / scale, ab / scale, bb / scale);
    if aa * bb - ab * ab <= 1e-12 {
        return Err(crate::observed_refusal());
    }
    let rhs = speed / scale;
    if !rhs.is_finite() {
        return Err(XpbdError::Numerical);
    }
    let solve = |shift: f64| {
        let a = aa + shift;
        let b = bb + shift;
        DVec2::new(-b, ab) * (rhs / (a * b - ab * ab))
    };
    let free = solve(0.);
    if free.length() <= cap {
        return Ok(free);
    }
    if cap == 0. {
        return Ok(DVec2::ZERO);
    }
    let mut lo = 0.;
    let mut hi = rhs / cap;
    if !hi.is_finite() {
        return Err(XpbdError::Numerical);
    }
    for _ in 0..48 {
        let mid = (lo + hi) * 0.5;
        if solve(mid).length() > cap {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Ok(solve(hi))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn anisotropic_sliding_is_the_minimum_work_impulse_on_the_euclidean_disk() {
        let k = [1.7, 0.9, 3.2];
        let speed = 1.8;
        for cap in [0., 0.1, 0.4, 1.6, 4.] {
            let j = coulomb(k, speed, cap).unwrap();
            let cost = |v: DVec2| {
                speed * v.x + 0.5 * (k[0] * v.x * v.x + 2. * k[1] * v.x * v.y + k[2] * v.y * v.y)
            };
            assert!(j.length() <= cap + 1e-14 && cost(j) <= 1e-15);
            for i in 0..1440 {
                let normal = DVec2::from_angle(i as f64 * std::f64::consts::TAU / 1440.);
                assert!(cost(j) <= cost(cap * normal) + 1e-12);
            }
            let after = DVec2::new(speed + k[0] * j.x + k[1] * j.y, k[1] * j.x + k[2] * j.y);
            if j.length() < cap - 1e-8 {
                assert!(after.length() < 1e-13);
            } else if cap > 0. {
                assert!(j.dot(after) < 0. && j.perp_dot(after).abs() < 1e-12);
            }
        }
        assert!(matches!(
            coulomb([1., 0., 1.], 1., f64::INFINITY),
            Err(XpbdError::Numerical)
        ));
        assert!(matches!(
            coulomb([1e-320, 0., 1e-320], 1e100, 0.2),
            Err(XpbdError::Numerical)
        ));
        assert!(matches!(
            coulomb([1., 0., 1.], 1e10, 1e-320),
            Err(XpbdError::Numerical)
        ));
        assert!(matches!(
            coulomb([1., 1., 1.], 1., 0.2),
            Err(crate::observed_refusal())
        ));
    }
}
