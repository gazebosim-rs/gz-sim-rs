// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Geometric, frictionless sphere impacts on actual cable/cloth material at
//! a known time of contact. The rigid response is a full cross-link operator.
mod checkpoint;
mod friction;
mod support;
pub use support::{MaterialSupportLoad, MaterialSupportResponse, NormalContactResponse};
mod sweep;
use crate::{
    MaterialFeature, MaterialTopology, SoftBody, XpbdError, impact_group,
    surface::geometry::vertex_face, topology::cell_faces,
};
pub use checkpoint::MAX_MATERIAL_CHECKPOINT_NODES;
use glam::DVec3;
use std::collections::BTreeMap;
pub use sweep::{
    MaterialImpactTime, MaterialSeparations, SphereMotion, SweptSphere, TranslatingSphere,
};

const MAX_ROWS: usize = 8;
const MAX_QUERIES: usize = 65536;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImpactSphere {
    pub center: [f64; 3],
    pub radius: f64,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MaterialImpactSite {
    /// Index of the source sphere in the caller's batch.
    pub sphere: usize,
    pub feature: MaterialFeature,
    pub weights: [f64; 3],
    /// True world sphere surface point, metres.
    pub point: [f64; 3],
    /// Unit normal from the rigid sphere towards the material midsurface.
    pub normal: [f64; 3],
    pub clearance: f64,
}
#[derive(Clone, Debug, PartialEq)]
pub struct MaterialImpactLoad {
    pub site: MaterialImpactSite,
    /// Positive means separating. Both values are material minus rigid in m/s.
    pub initial_normal_velocity: f64,
    pub final_normal_velocity: f64,
    /// Signed world impulse ON the rigid body, N s, at the indicated point.
    pub on_rigid: [f64; 3],
    /// Equal and opposing impulses on construction particles, including pinned supports.
    pub on_particles: Vec<(usize, [f64; 3])>,
    /// Actual thick material skin response, when Coulomb friction is enabled.
    pub friction: Option<MaterialImpactFriction>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MaterialImpactFriction {
    pub coefficient: f64,
    /// Tangential world impulse on the material at the shared surface point, N s.
    pub on_material: [f64; 3],
    /// True skin-minus-rigid tangential velocity, m/s.
    pub initial_slip: [f64; 3],
    pub final_slip: [f64; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub struct MaterialImpactReport {
    pub loads: Vec<MaterialImpactLoad>,
    /// Kinetic energy change due to the simultaneous material and rigid impulses, joules.
    pub kinetic_energy_change: f64,
    /// Work supplied to moving material and rigid bodies by prescribed particles, joules.
    pub prescribed_support_work: f64,
    /// Nonnegative loss at the passive normal interface, joules.
    pub dissipated_energy: f64,
}

/// Borrows the movable material until the complete native and soft transaction
/// has either succeeded or been dropped.
pub struct MaterialImpact<'a> {
    body: &'a mut SoftBody,
    sites: Vec<MaterialImpactSite>,
}
pub struct PreparedMaterialImpact<'a> {
    body: &'a mut SoftBody,
    updates: Vec<(usize, [f64; 3])>,
    report: MaterialImpactReport,
}

impl SoftBody {
    /// An impact uses finite-compliance material with a positive contact skin.
    /// Frictional tangential impulses require a cloth patch that can carry the skin couple.
    pub fn supports_articulated_impact(&self) -> bool {
        !self.config.self_contact
            && (self.config.friction == 0. || matches!(self.topology, MaterialTopology::Cloth(_)))
            && self.config.particle_radius > 0.
            && !matches!(self.topology, MaterialTopology::Particles)
            && self.constraints.iter().all(|c| c.compliance > 0.)
            && self.bending.iter().all(|b| b.compliance > 0.)
    }
    /// Resolve only witnesses geometrically at contact, on a finite-compliance
    /// material. Penetration requires a position-level solve. A clear surface
    /// produces no rows. This query alone never mutates particles.
    pub fn sphere_impact_at_barrier(
        &mut self,
        spheres: &[ImpactSphere],
    ) -> Result<MaterialImpact<'_>, XpbdError> {
        self.sphere_impact_witnesses(spheres, None, 0.)
    }
    fn sphere_impact_witnesses(
        &mut self,
        spheres: &[ImpactSphere],
        detected: Option<&MaterialImpactTime>,
        extra: f64,
    ) -> Result<MaterialImpact<'_>, XpbdError> {
        if spheres.is_empty() || spheres.len() > 16 || !self.supports_articulated_impact() {
            return Err(XpbdError::InvalidConfig);
        }
        let count = self.particles.len();
        let elements = match self.topology {
            MaterialTopology::Particles => return Err(XpbdError::InvalidConfig),
            MaterialTopology::Cable => count - 1,
            MaterialTopology::Cloth(cols) => 2 * (count / cols - 1) * (cols - 1),
        };
        if elements > MAX_QUERIES / spheres.len() {
            return Err(XpbdError::ImpactBudget);
        }
        let pos = |i: usize| DVec3::from(self.particles[i].position);
        let mut contacts = BTreeMap::new();
        for (sphere_index, sphere) in spheres.iter().enumerate() {
            let center = DVec3::from(sphere.center);
            let gap = sphere.radius + self.config.particle_radius;
            if !center.is_finite()
                || !gap.is_finite()
                || !(sphere.radius.is_finite() && sphere.radius > 0.)
            {
                return Err(XpbdError::InvalidBoundary);
            }
            let mut witness =
                |ids: [usize; 3], weights: [f64; 3], delta: DVec3| -> Result<(), XpbdError> {
                    let d = delta.length();
                    let clearance = d - gap;
                    let round = 128. * f64::EPSILON * center.abs().max_element().max(gap);
                    if !d.is_finite()
                        || weights.iter().any(|x| !x.is_finite())
                        || weights.iter().all(|&x| x <= 1e-12)
                    {
                        return Err(XpbdError::Numerical);
                    }
                    let (feature, weights) = MaterialFeature::from_weights(ids, weights);
                    let admitted = detected
                        .is_none_or(|t| t.rows.binary_search(&(sphere_index, feature)).is_ok());
                    let margin = if admitted {
                        detected.map_or(0., |t| t.rounding_distance())
                    } else {
                        0.
                    };
                    let tolerance = 1e-10 * gap + round + margin + extra;
                    if !d.is_finite() {
                        return Err(XpbdError::Numerical);
                    }
                    if clearance < -tolerance {
                        return Err(XpbdError::PenetratingImpact);
                    }
                    if admitted && clearance <= tolerance {
                        let n = delta.try_normalize().ok_or(XpbdError::UnresolvedImpact)?;
                        let point = center + n * sphere.radius;
                        if !point.is_finite() {
                            return Err(XpbdError::Numerical);
                        }
                        let entry = MaterialImpactSite {
                            sphere: sphere_index,
                            feature,
                            weights,
                            point: point.to_array(),
                            normal: n.to_array(),
                            clearance,
                        };
                        if let Some(prev) = contacts.insert((sphere_index, feature), entry)
                            && DVec3::from(prev.point).distance(point) > 2. * tolerance
                        {
                            return Err(XpbdError::UnresolvedImpact);
                        }
                        if contacts.len() > MAX_ROWS {
                            return Err(XpbdError::ImpactBudget);
                        }
                    }
                    Ok(())
                };
            match self.topology {
                MaterialTopology::Cable => {
                    for i in 1..count {
                        let a = pos(i - 1) - center;
                        let b = pos(i) - center;
                        let edge = b - a;
                        let norm = edge.length_squared();
                        if norm <= 0. || !norm.is_finite() {
                            return Err(XpbdError::UnresolvedImpact);
                        }
                        let t = (-a.dot(edge) / norm).clamp(0., 1.);
                        witness([i - 1, i, usize::MAX], [1. - t, t, 0.], a + t * edge)?;
                    }
                }
                MaterialTopology::Cloth(cols) => {
                    for row in 1..count / cols {
                        for col in 1..cols {
                            for ids in cell_faces(row * cols + col, cols) {
                                let [a, b, c] = ids.map(|i| pos(i) - center);
                                let w = vertex_face(&[DVec3::ZERO, a, b, c]);
                                witness(
                                    ids,
                                    [-w.weights[1], -w.weights[2], -w.weights[3]],
                                    -w.delta,
                                )?;
                            }
                        }
                    }
                }
                MaterialTopology::Particles => unreachable!(),
            }
        }
        if detected.is_some_and(|t| contacts.len() != t.rows.len()) {
            return Err(XpbdError::UnresolvedImpact);
        }
        Ok(MaterialImpact {
            body: self,
            sites: contacts.into_values().collect(),
        })
    }
}

impl<'a> MaterialImpact<'a> {
    pub fn sites(&self) -> &[MaterialImpactSite] {
        &self.sites
    }
    /// Each response block maps a world impulse ON a rigid source to the
    /// world velocity at a rigid target. The same sign enters relative
    /// mobility twice, producing a positive rigid term in the normal system.
    pub fn prepare(
        self,
        native_velocity: &[[f64; 3]],
        mobility: impl Fn(usize, usize) -> [[f64; 3]; 3],
        restitution: f64,
    ) -> Result<PreparedMaterialImpact<'a>, XpbdError> {
        let n = self.sites.len();
        if !(0. ..=1.).contains(&restitution)
            || native_velocity.len() != n
            || native_velocity.iter().any(|&v| !DVec3::from(v).is_finite())
        {
            return Err(XpbdError::InvalidConfig);
        }
        if self.body.config.friction > 0. {
            return friction::prepare(self, native_velocity, mobility, restitution);
        }
        let normal = self.normal_response(native_velocity, mobility)?;
        let NormalContactResponse { k, incoming } = normal;
        let free: Vec<_> = incoming
            .iter()
            .map(|&v| v + restitution * v.min(0.))
            .collect();
        let impulse = if free.iter().all(|&v| v >= 0.) {
            vec![0.; n]
        } else {
            impact_group::inelastic(&k, &free, n).ok_or(XpbdError::UnresolvedImpact)?
        };
        let mut loads = Vec::with_capacity(n);
        let mut energy = 0.;
        let mut energy_scale = 0.;
        for (i, site) in self.sites.into_iter().enumerate() {
            let before = incoming[i];
            let after = before + k[i].iter().zip(&impulse).map(|(a, j)| a * j).sum::<f64>();
            let j = impulse[i] * DVec3::from(site.normal);
            let mut on_particles = vec![];
            for (&idx, w) in site.feature.vertices().iter().zip(site.weights) {
                on_particles.push((idx, (w * j).to_array()));
            }
            energy += 0.5 * impulse[i] * (before + after);
            energy_scale += 0.5 * impulse[i] * (before.abs() + after.abs());
            loads.push(MaterialImpactLoad {
                site,
                initial_normal_velocity: before,
                final_normal_velocity: after,
                on_rigid: (-j).to_array(),
                on_particles,
                friction: None,
            });
        }
        stage(self.body, loads, energy, energy_scale)
    }
}

fn stage(
    body: &mut SoftBody,
    loads: Vec<MaterialImpactLoad>,
    energy: f64,
    energy_scale: f64,
) -> Result<PreparedMaterialImpact<'_>, XpbdError> {
    let mut nodal = BTreeMap::<usize, DVec3>::new();
    for l in &loads {
        for &(id, impulse) in &l.on_particles {
            *nodal.entry(id).or_default() += DVec3::from(impulse);
        }
    }
    if !energy.is_finite() || energy > 256. * f64::EPSILON * energy_scale.max(1e-14) {
        return Err(XpbdError::UnresolvedImpact);
    }
    let mut support_work = 0.;
    let mut updates = Vec::with_capacity(nodal.len());
    for (i, j) in nodal {
        let p = body.particles[i];
        if p.inverse_mass == 0. {
            support_work -= DVec3::from(p.velocity).dot(j);
        }
        let v = DVec3::from(p.velocity) + p.inverse_mass * j;
        if !v.is_finite() {
            return Err(XpbdError::Numerical);
        }
        updates.push((i, v.to_array()));
    }
    if !support_work.is_finite() || !(energy + support_work).is_finite() {
        return Err(XpbdError::Numerical);
    }
    Ok(PreparedMaterialImpact {
        body,
        updates,
        report: MaterialImpactReport {
            loads,
            kinetic_energy_change: energy + support_work,
            prescribed_support_work: support_work,
            dissipated_energy: (-energy).max(0.),
        },
    })
}
impl PreparedMaterialImpact<'_> {
    pub fn report(&self) -> &MaterialImpactReport {
        &self.report
    }
    /// All numeric checks precede commit so the rigid owner may commit first.
    pub fn commit(self) -> MaterialImpactReport {
        for (i, v) in self.updates {
            self.body.particles[i].velocity = v;
        }
        self.report
    }
}
