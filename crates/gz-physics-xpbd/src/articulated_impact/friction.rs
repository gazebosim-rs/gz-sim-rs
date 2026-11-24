// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A finite-thickness cloth skin and native multibody share a single 3D impulse point.
use super::*;
use crate::{coulomb_manifold, material::Skin};
use std::collections::BTreeSet;

#[derive(Clone, Debug)]
struct Direction {
    axis: DVec3,
    nodal: Vec<(usize, DVec3)>,
}

/// A feature is contained only in cells incident on its first construction
/// vertex. A triangle uses its own nodes; tied edges and vertices use the full ring.
fn support(columns: usize, count: usize, feature: MaterialFeature) -> Vec<usize> {
    let vertices = feature.vertices();
    let i = vertices[0];
    let (r, c) = (i / columns, i % columns);
    let mut ring = BTreeSet::new();
    for row in r.max(1)..=(r + 1).min(count / columns - 1) {
        for col in c.max(1)..=(c + 1).min(columns - 1) {
            for ids in cell_faces(row * columns + col, columns) {
                if vertices.iter().all(|v| ids.contains(v)) {
                    ring.extend(ids);
                }
            }
        }
    }
    ring.into_iter().collect()
}

#[derive(Clone, Debug)]
pub(super) struct SkinResponse {
    pub(super) initial: Vec<f64>,
    pub(super) k: Vec<Vec<f64>>,
    dirs: Vec<Direction>,
    mu: f64,
}
pub(super) fn measure(
    impact: &MaterialImpact<'_>,
    native_velocity: &[[f64; 3]],
    mobility: impl Fn(usize, usize) -> [[f64; 3]; 3],
) -> Result<SkinResponse, XpbdError> {
    if native_velocity.len() != impact.sites.len()
        || native_velocity.iter().any(|v| !DVec3::from(*v).is_finite())
    {
        return Err(XpbdError::InvalidConfig);
    }
    let MaterialTopology::Cloth(columns) = impact.body.topology else {
        return Err(XpbdError::InvalidConfig);
    };
    let nodes = &impact.body.particles;
    let mut dirs = Vec::with_capacity(3 * impact.sites.len());
    let mut initial = Vec::with_capacity(3 * impact.sites.len());
    for (site, &v) in impact.sites.iter().zip(native_velocity) {
        let n = DVec3::from(site.normal);
        let u = n.any_orthonormal_vector();
        let t = n.cross(u);
        let mut ids = [usize::MAX; 3];
        let vertices = site.feature.vertices();
        ids[..vertices.len()].copy_from_slice(vertices);
        let ring = support(columns, nodes.len(), site.feature);
        if ring.len() < 3 {
            return Err(XpbdError::UnresolvedImpact);
        }
        let at = |i: usize| DVec3::from(nodes[i].position);
        let mid = vertices
            .iter()
            .zip(site.weights)
            .map(|(&i, w)| w * at(i))
            .sum::<DVec3>();
        let skin = Skin::new(&ring, ids, site.weights, at, DVec3::from(site.point) - mid)
            .map_err(|_| XpbdError::UnresolvedImpact)?;
        for axis in [n, u, t] {
            let nodal: Vec<_> = skin.nodal(axis).collect();
            let velocity = nodal
                .iter()
                .map(|&(i, g)| g.dot(DVec3::from(nodes[i].velocity)))
                .sum::<f64>()
                - axis.dot(v.into());
            if !velocity.is_finite() {
                return Err(XpbdError::Numerical);
            }
            initial.push(velocity);
            dirs.push(Direction { axis, nodal });
        }
    }
    let d = dirs.len();
    let mut k = vec![vec![0.; d]; d];
    for i in 0..impact.sites.len() {
        for (j, _) in impact.sites.iter().enumerate() {
            let block = mobility(i, j);
            for a in 0..3 {
                let left = &dirs[3 * i + a];
                for b in 0..3 {
                    let right = &dirs[3 * j + b];
                    let common = left
                        .nodal
                        .iter()
                        .filter_map(|&(id, p)| {
                            right
                                .nodal
                                .iter()
                                .find(|(v, _)| *v == id)
                                .map(|(_, q)| nodes[id].inverse_mass * p.dot(*q))
                        })
                        .sum::<f64>();
                    let dv = DVec3::from(block.map(|row| DVec3::from(row).dot(right.axis)));
                    let w = common + left.axis.dot(dv);
                    if !w.is_finite() {
                        return Err(XpbdError::Numerical);
                    }
                    k[3 * i + a][3 * j + b] = w;
                }
            }
        }
    }
    for i in 0..d {
        for j in 0..i {
            let scale = k[i][i].abs().sqrt() * k[j][j].abs().sqrt();
            if (k[i][j] - k[j][i]).abs() > 256. * f64::EPSILON * scale.max(1e-12) {
                return Err(XpbdError::InvalidBoundary);
            }
        }
    }
    Ok(SkinResponse {
        initial,
        k,
        dirs,
        mu: impact.body.config.friction,
    })
}
impl SkinResponse {
    pub(super) fn nodes(&self) -> impl Iterator<Item = usize> + '_ {
        self.dirs
            .iter()
            .step_by(3)
            .flat_map(|row| row.nodal.iter().map(|(id, _)| *id))
    }
    /// Relative displacement of the same material and rigid witness, projected
    /// on the frozen finite-skin Jacobian. The rigid source uses its fixed
    /// body-local point, including the native orientation change.
    pub(super) fn travel(
        &self,
        start: &[crate::Particle],
        end: &[crate::Particle],
        rigid: &[[f64; 3]],
    ) -> Result<Vec<f64>, XpbdError> {
        if start.len() != end.len() || rigid.len() * 3 != self.dirs.len() {
            return Err(XpbdError::InvalidConfig);
        }
        self.dirs
            .iter()
            .enumerate()
            .map(|(i, row)| {
                let mut delta = -row.axis.dot(DVec3::from(rigid[i / 3]));
                for &(node, g) in &row.nodal {
                    let before = start.get(node).ok_or(XpbdError::InvalidConfig)?;
                    delta += g.dot(DVec3::from(end[node].position) - DVec3::from(before.position));
                }
                delta
                    .is_finite()
                    .then_some(delta)
                    .ok_or(XpbdError::Numerical)
            })
            .collect()
    }
    pub(super) fn tangent_vectors(&self, rows: &[f64]) -> Result<Vec<[f64; 3]>, XpbdError> {
        if rows.len() != self.dirs.len() {
            return Err(XpbdError::InvalidConfig);
        }
        Ok(rows
            .chunks_exact(3)
            .zip(self.dirs.chunks_exact(3))
            .map(|(r, d)| (r[1] * d[1].axis + r[2] * d[2].axis).to_array())
            .collect())
    }
    pub(super) fn solve(&self, free: &[f64]) -> Result<Vec<f64>, XpbdError> {
        let k = &self.k;
        let mu = self.mu;
        let d = self.initial.len();
        if free.len() != d || free.iter().any(|v| !v.is_finite()) {
            return Err(XpbdError::InvalidConfig);
        }
        let active = free.iter().step_by(3).any(|&v| v < 0.);
        let mut seed = vec![0.; d];
        if active {
            let rows = (0..d)
                .step_by(3)
                .map(|i| (0..d).step_by(3).map(|j| k[i][j]).collect())
                .collect::<Vec<_>>();
            let speed = free.iter().step_by(3).copied().collect::<Vec<_>>();
            let normal = impact_group::inelastic(&rows, &speed, speed.len())
                .ok_or(XpbdError::UnresolvedImpact)?;
            for (dst, n) in seed.iter_mut().step_by(3).zip(normal) {
                *dst = n;
            }
        }
        let impulse = if active {
            coulomb_manifold::solve(k, free, &seed, mu).ok_or(XpbdError::UnresolvedImpact)?
        } else {
            seed
        };
        Ok(impulse)
    }
    pub(super) fn loads(
        &self,
        sites: &[MaterialImpactSite],
        impulse: &[f64],
    ) -> Result<(Vec<MaterialImpactLoad>, f64, f64), XpbdError> {
        let Self {
            initial,
            k,
            dirs,
            mu,
        } = self;
        if impulse.len() != dirs.len()
            || sites.len() * 3 != dirs.len()
            || impulse.chunks_exact(3).any(|j| {
                j.iter().any(|v| !v.is_finite())
                    || j[0] < 0.
                    || j[1].hypot(j[2]) > mu * j[0] * (1. + 1e-14)
            })
        {
            return Err(XpbdError::InvalidConfig);
        }
        let mut energy = 0.;
        let mut scale = 0.;
        let mut loads = vec![];
        for (i, site) in sites.iter().copied().enumerate() {
            let mut nodes = BTreeMap::<usize, DVec3>::new();
            let mut total = DVec3::ZERO;
            let mut slips = [DVec3::ZERO; 2];
            let mut normal = [0.; 2];
            let mut tangent = DVec3::ZERO;
            for a in 0..3 {
                let index = 3 * i + a;
                let dir = &dirs[index];
                let j = impulse[index];
                let speed = [
                    initial[index],
                    initial[index] + coulomb_manifold::dot(&k[index], impulse),
                ];
                total += dir.axis * j;
                for &(id, gradient) in &dir.nodal {
                    *nodes.entry(id).or_default() += gradient * j;
                }
                if a == 0 {
                    normal = speed;
                } else {
                    tangent += dir.axis * j;
                    slips[0] += speed[0] * dir.axis;
                    slips[1] += speed[1] * dir.axis;
                }
                energy += 0.5 * j * (speed[0] + speed[1]);
                scale += 0.5 * j.abs() * (speed[0].abs() + speed[1].abs());
            }
            loads.push(MaterialImpactLoad {
                site,
                initial_normal_velocity: normal[0],
                final_normal_velocity: normal[1],
                on_rigid: (-total).to_array(),
                on_particles: nodes
                    .into_iter()
                    .map(|(id, v)| (id, v.to_array()))
                    .collect(),
                friction: Some(MaterialImpactFriction {
                    coefficient: *mu,
                    on_material: tangent.to_array(),
                    initial_slip: slips[0].to_array(),
                    final_slip: slips[1].to_array(),
                }),
            });
        }
        Ok((loads, energy, scale))
    }
}
pub(super) fn prepare<'a>(
    impact: MaterialImpact<'a>,
    native_velocity: &[[f64; 3]],
    mobility: impl Fn(usize, usize) -> [[f64; 3]; 3],
    restitution: f64,
) -> Result<PreparedMaterialImpact<'a>, XpbdError> {
    let basis = measure(&impact, native_velocity, mobility)?;
    let mut free = basis.initial.clone();
    for i in (0..free.len()).step_by(3) {
        free[i] += restitution * basis.initial[i].min(0.);
    }
    let impulse = basis.solve(&free)?;
    let (loads, energy, scale) = basis.loads(&impact.sites, &impulse)?;
    stage(impact.body, loads, energy, scale)
}
