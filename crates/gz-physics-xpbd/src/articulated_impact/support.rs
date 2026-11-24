// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Positive finite-time support impulses on actual frictionless skin witnesses.
use super::*;
use std::time::Duration;
#[derive(Clone, Debug)]
pub struct NormalContactResponse {
    pub(super) k: Vec<Vec<f64>>,
    pub(super) incoming: Vec<f64>,
}
impl NormalContactResponse {
    pub fn speeds(&self) -> &[f64] {
        &self.incoming
    }
    pub fn solve_unilateral(&self, free: &[f64]) -> Result<Vec<f64>, XpbdError> {
        let n = self.incoming.len();
        if free.len() != n || free.iter().any(|v| !v.is_finite()) || n > MAX_ROWS {
            return Err(XpbdError::InvalidConfig);
        }
        if free.iter().all(|&v| v >= 0.) {
            return Ok(vec![0.; n]);
        }
        impact_group::inelastic(&self.k, free, n).ok_or(XpbdError::UnresolvedImpact)
    }
    pub fn induced(&self, j: &[f64]) -> Result<Vec<f64>, XpbdError> {
        if self.incoming.len() != j.len() || j.iter().any(|v| !v.is_finite()) {
            return Err(XpbdError::InvalidConfig);
        }
        Ok(self
            .k
            .iter()
            .map(|row| row.iter().zip(j).map(|(v, j)| v * j).sum())
            .collect())
    }
}
#[derive(Clone, Debug, PartialEq)]
pub struct MaterialSupportLoad {
    pub site: MaterialImpactSite,
    /// Compressive signed impulse on this native link in world N s.
    pub on_rigid: [f64; 3],
    /// Particle ID, world location at application, and simultaneous world impulse.
    pub on_particles: Vec<(usize, [f64; 3], [f64; 3])>,
    /// Tangential material impulse and actual skin-relative slip at this cut.
    pub friction: Option<MaterialImpactFriction>,
}
impl SoftBody {
    /// Integrate a free, deforming mesh in its uniformly accelerating frame.
    /// Translation invariance of the internal constraints leaves their complete
    /// relative XPBD dynamics intact, with exact gravitational CM motion.
    pub fn advance_supported_free_interval(&mut self, dt: Duration) -> Result<(), XpbdError> {
        if !self.supports_articulated_impact()
            || self.particles.iter().any(|p| p.inverse_mass <= 0.)
        {
            return Err(XpbdError::InvalidConfig);
        }
        let a = DVec3::from(self.config.gravity);
        if a == DVec3::ZERO || dt.is_zero() {
            return self.step(dt, &[]);
        }
        let h = dt.as_secs_f64();
        let dx = 0.5 * h * h * a;
        let dv = h * a;
        if !dx.is_finite() || !dv.is_finite() {
            return Err(XpbdError::Numerical);
        }
        self.config.gravity = [0.; 3];
        let moved = self.step(dt, &[]);
        self.config.gravity = a.to_array();
        moved?;
        if self.particles.iter().any(|p| {
            !(DVec3::from(p.position) + dx).is_finite()
                || !(DVec3::from(p.velocity) + dv).is_finite()
        }) {
            return Err(XpbdError::Numerical);
        }
        for p in &mut self.particles {
            p.position = (DVec3::from(p.position) + dx).to_array();
            p.velocity = (DVec3::from(p.velocity) + dv).to_array();
        }
        Ok(())
    }

    pub fn sphere_support_witnesses(
        &mut self,
        spheres: &[ImpactSphere],
        spatial_tolerance: f64,
    ) -> Result<MaterialImpact<'_>, XpbdError> {
        if self.particles.iter().any(|p| p.inverse_mass <= 0.)
            || !(0.0..=1e-6).contains(&spatial_tolerance)
        {
            return Err(XpbdError::InvalidConfig);
        }
        let mut patch = self.sphere_impact_witnesses(spheres, None, spatial_tolerance)?;
        // A point on an internal tessellation edge can fall inside the spatial
        // acquisition band at the same time as the actual closest triangle
        // interior. A subset cannot contribute a distinct local distance
        // minimum when its containing convex primitive has a closer witness.
        // Independent contacts on two folded triangle interiors stay distinct.
        let all = patch.sites.clone();
        patch.sites.retain(|s| {
            !all.iter().any(|other| {
                other.sphere == s.sphere
                    && other.feature.vertices().len() > s.feature.vertices().len()
                    && s.feature
                        .vertices()
                        .iter()
                        .all(|v| other.feature.vertices().contains(v))
                    && other.clearance
                        <= s.clearance + 16. * f64::EPSILON * (1. + s.clearance.abs())
            })
        });
        Ok(patch)
    }
}
impl MaterialImpact<'_> {
    pub fn normal_response(
        &self,
        native_velocity: &[[f64; 3]],
        mobility: impl Fn(usize, usize) -> [[f64; 3]; 3],
    ) -> Result<NormalContactResponse, XpbdError> {
        let n = self.sites.len();
        if native_velocity.len() != n
            || native_velocity.iter().any(|&v| !DVec3::from(v).is_finite())
        {
            return Err(XpbdError::InvalidConfig);
        }
        let mut k = vec![vec![0.; n]; n];
        let mut incoming = Vec::with_capacity(n);
        for (i, a) in self.sites.iter().enumerate() {
            let na = DVec3::from(a.normal);
            let material = a
                .feature
                .vertices()
                .iter()
                .zip(a.weights)
                .map(|(&v, w)| w * DVec3::from(self.body.particles[v].velocity))
                .sum::<DVec3>();
            incoming.push(na.dot(material - DVec3::from(native_velocity[i])));
            for (j, b) in self.sites.iter().enumerate() {
                let nb = DVec3::from(b.normal);
                let soft = a
                    .feature
                    .vertices()
                    .iter()
                    .zip(a.weights)
                    .filter_map(|(&v, wa)| {
                        b.feature
                            .vertices()
                            .iter()
                            .position(|&p| p == v)
                            .map(|p| wa * b.weights[p] * self.body.particles[v].inverse_mass)
                    })
                    .sum::<f64>();
                let block = mobility(i, j);
                let response = DVec3::new(
                    DVec3::from(block[0]).dot(nb),
                    DVec3::from(block[1]).dot(nb),
                    DVec3::from(block[2]).dot(nb),
                );
                k[i][j] = soft * na.dot(nb) + na.dot(response);
                if !k[i][j].is_finite() {
                    return Err(XpbdError::Numerical);
                }
            }
        }
        if incoming.iter().any(|v| !v.is_finite()) {
            return Err(XpbdError::Numerical);
        }
        for i in 0..n {
            for j in 0..i {
                let tol =
                    256. * f64::EPSILON * (k[i][i].abs().sqrt() * k[j][j].abs().sqrt()).max(1e-12);
                if (k[i][j] - k[j][i]).abs() > tol {
                    return Err(XpbdError::InvalidBoundary);
                }
            }
        }
        Ok(NormalContactResponse { k, incoming })
    }
    /// Apply a proposed positive normal impulse to movable construction nodes.
    /// The finite native owner must receive the stated opposite impulse.
    pub fn apply_normal_support(
        self,
        impulses: &[f64],
    ) -> Result<Vec<MaterialSupportLoad>, XpbdError> {
        if self.body.config.friction != 0.
            || impulses.len() != self.sites.len()
            || impulses.iter().any(|v| !v.is_finite() || *v < 0.)
        {
            return Err(XpbdError::InvalidConfig);
        }
        let mut aggregate = BTreeMap::<usize, DVec3>::new();
        let mut loads = Vec::with_capacity(impulses.len());
        for (s, &j) in self.sites.into_iter().zip(impulses) {
            let impulse = DVec3::from(s.normal) * j;
            let mut on_particles = Vec::new();
            for (&id, w) in s.feature.vertices().iter().zip(s.weights) {
                let at = &self.body.particles[id];
                if at.inverse_mass <= 0. {
                    return Err(XpbdError::InvalidConfig);
                }
                let change = w * impulse;
                *aggregate.entry(id).or_default() += change;
                on_particles.push((id, at.position, change.to_array()));
            }
            loads.push(MaterialSupportLoad {
                site: s,
                on_rigid: (-impulse).to_array(),
                on_particles,
                friction: None,
            });
        }
        let mut changes = vec![];
        for (id, j) in aggregate {
            let p = self.body.particles[id];
            let v = DVec3::from(p.velocity) + p.inverse_mass * j;
            if !v.is_finite() {
                return Err(XpbdError::Numerical);
            }
            changes.push((id, v.to_array()));
        }
        for (id, v) in changes {
            self.body.particles[id].velocity = v;
        }
        Ok(loads)
    }
}

/// Frozen relative mobility of the current material surface and native links.
/// Normal-only profiles use a single row per witness. Finite-thickness
/// frictional cloth uses an orthonormal normal/tangent/tangent triplet.
#[derive(Clone, Debug)]
pub struct MaterialSupportResponse(SupportDirections);
#[derive(Clone, Debug)]
enum SupportDirections {
    Normal(NormalContactResponse),
    Skin(Box<MeasuredSkin>),
}
#[derive(Clone, Debug)]
struct MeasuredSkin {
    response: friction::SkinResponse,
    columns: usize,
    count: usize,
    mu: f64,
    sites: Vec<MaterialImpactSite>,
    nodes: Vec<(usize, crate::Particle)>,
    velocities: Vec<[f64; 3]>,
    blocks: Vec<[[f64; 3]; 3]>,
}
impl MaterialSupportResponse {
    pub fn stride(&self) -> usize {
        match &self.0 {
            SupportDirections::Normal(_) => 1,
            SupportDirections::Skin(_) => 3,
        }
    }
    pub fn speeds(&self) -> &[f64] {
        match &self.0 {
            SupportDirections::Normal(n) => n.speeds(),
            SupportDirections::Skin(s) => &s.response.initial,
        }
    }
    pub fn solve(&self, free: &[f64]) -> Result<Vec<f64>, XpbdError> {
        match &self.0 {
            SupportDirections::Normal(n) => n.solve_unilateral(free),
            SupportDirections::Skin(s) => s.response.solve(free),
        }
    }
    pub fn induced(&self, j: &[f64]) -> Result<Vec<f64>, XpbdError> {
        match &self.0 {
            SupportDirections::Normal(n) => n.induced(j),
            SupportDirections::Skin(s) => {
                if j.len() != s.response.initial.len() || j.iter().any(|v| !v.is_finite()) {
                    return Err(XpbdError::InvalidConfig);
                }
                Ok(s.response
                    .k
                    .iter()
                    .map(|row| crate::coulomb_manifold::dot(row, j))
                    .collect())
            }
        }
    }
    pub fn tangent_vectors(&self, rows: &[f64]) -> Result<Vec<[f64; 3]>, XpbdError> {
        match &self.0 {
            SupportDirections::Normal(_) => Ok(vec![]),
            SupportDirections::Skin(s) => s.response.tangent_vectors(rows),
        }
    }
    /// Normal travel is supplied by exact, updated simplex geometry. Tangential
    /// travel follows the same fixed material and body-local contact locations.
    pub fn row_travel(
        &self,
        start: &[crate::Particle],
        end: &[crate::Particle],
        rigid: &[[f64; 3]],
        normal: &[f64],
    ) -> Result<Vec<f64>, XpbdError> {
        if normal.len() * self.stride() != self.speeds().len() {
            return Err(XpbdError::InvalidConfig);
        }
        match &self.0 {
            SupportDirections::Normal(_) => Ok(normal.to_vec()),
            SupportDirections::Skin(s) => {
                let mut r = s.response.travel(start, end, rigid)?;
                for (row, &n) in r.chunks_exact_mut(3).zip(normal) {
                    row[0] = n;
                }
                Ok(r)
            }
        }
    }
}
impl MaterialImpact<'_> {
    pub fn support_response(
        &self,
        native_velocity: &[[f64; 3]],
        mobility: impl Fn(usize, usize) -> [[f64; 3]; 3],
    ) -> Result<MaterialSupportResponse, XpbdError> {
        let rows = if self.body.config.friction > 0. {
            let MaterialTopology::Cloth(columns) = self.body.topology else {
                return Err(XpbdError::InvalidConfig);
            };
            let n = self.sites.len();
            let blocks: Vec<_> = (0..n * n).map(|ix| mobility(ix / n, ix % n)).collect();
            let response = friction::measure(self, native_velocity, |i, j| blocks[i * n + j])?;
            let mut ids = response.nodes().collect::<Vec<_>>();
            ids.sort_unstable();
            ids.dedup();
            let nodes = ids
                .into_iter()
                .map(|id| (id, self.body.particles[id]))
                .collect();
            SupportDirections::Skin(Box::new(MeasuredSkin {
                response,
                columns,
                count: self.body.particles.len(),
                mu: self.body.config.friction,
                sites: self.sites.clone(),
                nodes,
                velocities: native_velocity.to_vec(),
                blocks,
            }))
        } else {
            SupportDirections::Normal(self.normal_response(native_velocity, mobility)?)
        };
        Ok(MaterialSupportResponse(rows))
    }
    /// Apply simultaneous contact-frame impulses to the material and expose
    /// their exact opposite for the matching native transaction at this cut.
    pub fn apply_support(
        self,
        measured: &MaterialSupportResponse,
        j: &[f64],
        native_velocity: &[[f64; 3]],
        mobility: impl Fn(usize, usize) -> [[f64; 3]; 3],
    ) -> Result<Vec<MaterialSupportLoad>, XpbdError> {
        let SupportDirections::Skin(m) = &measured.0 else {
            return self.apply_normal_support(j);
        };
        let n = self.sites.len();
        // A branch can reuse the complete skin operator only at its bound
        // physical cut. A changed surface, particle state, native velocity or
        // cross-link mobility requires remeasurement before any mutation.
        if self.sites != m.sites
            || self.body.config.friction != m.mu
            || !matches!(self.body.topology,MaterialTopology::Cloth(c) if c==m.columns)
            || self.body.particles.len() != m.count
            || native_velocity != m.velocities
            || m.nodes.iter().any(|&(id, p)| self.body.particles[id] != p)
            || m.blocks
                .iter()
                .enumerate()
                .any(|(ix, k)| mobility(ix / n, ix % n) != *k)
        {
            return Err(XpbdError::InvalidBoundary);
        }
        let (loads, _, _) = m.response.loads(&self.sites, j)?;
        let mut summed = BTreeMap::<usize, DVec3>::new();
        let mut out = Vec::with_capacity(loads.len());
        for l in loads {
            let mut on_particles = vec![];
            for (id, impulse) in l.on_particles {
                let p = self.body.particles[id];
                if p.inverse_mass <= 0. {
                    return Err(XpbdError::InvalidConfig);
                }
                *summed.entry(id).or_default() += DVec3::from(impulse);
                on_particles.push((id, p.position, impulse));
            }
            out.push(MaterialSupportLoad {
                site: l.site,
                on_rigid: l.on_rigid,
                on_particles,
                friction: l.friction,
            });
        }
        let mut updates = Vec::with_capacity(summed.len());
        for (id, impulse) in summed {
            let p = self.body.particles[id];
            let v = DVec3::from(p.velocity) + p.inverse_mass * impulse;
            if !v.is_finite() {
                return Err(XpbdError::Numerical);
            }
            updates.push((id, v.to_array()));
        }
        for (id, v) in updates {
            self.body.particles[id].velocity = v;
        }
        Ok(out)
    }
}
