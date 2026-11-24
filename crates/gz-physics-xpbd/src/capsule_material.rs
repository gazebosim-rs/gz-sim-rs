// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Continuous contact between finite capsules and thick material features.
//! A segment/triangle first approaches a capsule either at a core endpoint
//! against a face or at a core segment against a material edge. Each sweep
//! uses the same certified moving-feature kernel as shell self-contact.
use crate::{BoundaryImpulse, Particle, PreparedBoundary, XpbdError, collision::Interval};
use glam::DVec3;
use std::collections::{BTreeMap, BTreeSet};
mod bounds;
mod finite;
mod friction;
use friction::Friction;
pub use friction::{CapsuleContactSample, CapsuleFrictionLoad};
mod geometry;
use bounds::{Block, Bounds};
use finite::Trajectory;
pub(crate) use finite::check_receiver_pair;
use geometry::{Guide, Stencil};

const MAX_STENCILS: usize = 4096;
const MAX_ROWS: usize = 128;
const MAX_CORRECTIONS: usize = 96;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum MaterialFeature {
    Vertex(usize),
    Segment([usize; 2]),
    Triangle([usize; 3]),
}
impl MaterialFeature {
    /// Construction particle indices in ascending order. Each load uses the
    /// same order for its nodal impulses, independently of triangle winding.
    pub fn vertices(&self) -> &[usize] {
        match self {
            Self::Vertex(i) => std::slice::from_ref(i),
            Self::Segment(ids) => ids,
            Self::Triangle(ids) => ids,
        }
    }
    pub(crate) fn from_weights(ids: [usize; 3], f: [f64; 3]) -> (Self, [f64; 3]) {
        let mut nodes = [(usize::MAX, 0.); 3];
        let mut count = 0;
        for (i, w) in ids.into_iter().zip(f) {
            if w > 1e-12 {
                nodes[count] = (i, w);
                count += 1;
            }
        }
        nodes[..count].sort_unstable_by_key(|a| a.0);
        let feature = match count {
            1 => Self::Vertex(nodes[0].0),
            2 => Self::Segment([nodes[0].0, nodes[1].0]),
            3 => Self::Triangle(nodes.map(|n| n.0)),
            _ => unreachable!("convex witness"),
        };
        let sum: f64 = nodes.iter().map(|n| n.1).sum();
        (feature, nodes.map(|(_, w)| w / sum))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CapsuleRegion {
    NegativeCap,
    Barrel,
    PositiveCap,
}

/// One committed contact load on a capsule. For a prescribed guide this
/// measures external support. A two-way receiver takes the rigid impulse.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapsuleMaterialLoad {
    /// True when the native finite receiver takes this impulse and changes motion.
    pub two_way: bool,
    pub boundary: usize,
    pub feature: MaterialFeature,
    pub region: CapsuleRegion,
    /// Earliest contributing impact or support time since this public tick began, in s.
    pub first_time: f64,
    /// Impulse on the capsule in world axes, N s.
    pub linear: [f64; 3],
    /// Moment on the capsule about its fixed public-tick starting center, N m s.
    pub angular: [f64; 3],
    /// Impulses on material degrees of freedom, in feature vertex order.
    /// Prescribed vertices transfer their share to their support.
    pub nodal: [[f64; 3]; 3],
}
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Key {
    two_way: bool,
    boundary: usize,
    feature: MaterialFeature,
    part: CapsuleRegion,
}
#[derive(Clone, Copy)]
struct Candidate {
    feature: MaterialFeature,
    weights: [f64; 3],
    part: CapsuleRegion,
    normal: DVec3,
    time: f64,
    depth: f64,
    overlap: f64,
    tolerance: f64,
}
impl Candidate {
    fn point(&self, get: impl Fn(usize) -> DVec3) -> DVec3 {
        let ids = self.feature.vertices();
        let a = get(ids[0]);
        let mut p = a;
        for (slot, &id) in ids.iter().enumerate().skip(1) {
            p += self.weights[slot] * (get(id) - a);
        }
        p
    }
    fn inverse_mass(&self, p: &[Particle]) -> f64 {
        self.feature
            .vertices()
            .iter()
            .zip(self.weights)
            .map(|(&i, w)| w * w * p[i].inverse_mass)
            .sum()
    }
}
#[derive(Clone, Copy)]
struct Contact {
    witness: Candidate,
    normal_impulse: f64,
    tangent_impulse: DVec3,
    first_time: f64,
}
pub(super) struct Mesh {
    stencils: Vec<Stencil>,
    blocks: Vec<Block>,
    patches: BTreeMap<[usize; 3], Vec<usize>>,
}
#[derive(Clone)]
pub(super) struct CapsuleMaterial {
    mesh: std::sync::Arc<Mesh>,
    pub(super) friction: Friction,
    contacts: BTreeMap<Key, Contact>,
    loads: BTreeMap<Key, CapsuleMaterialLoad>,
    pub published: Vec<CapsuleMaterialLoad>,
    queries: usize,
    checked_initial: bool,
    pub(super) retry_manifold: bool,
    bounds: Vec<Bounds>,
}
impl CapsuleMaterial {
    pub fn new(particles: usize, grid: Option<usize>, mu: f64) -> Result<Self, XpbdError> {
        // A vertex input stays a vertex input. The opt-in admits only actual
        // cable and rectangular cloth topologies installed by the constructors.
        let mut faces = vec![];
        let mut edges = BTreeSet::new();
        if let Some(columns) = grid {
            let cells = (particles / columns - 1) * (columns - 1);
            if cells > MAX_STENCILS / 4 {
                return Err(XpbdError::SurfaceBudget);
            }
            for row in 1..particles / columns {
                for col in 1..columns {
                    let i = row * columns + col;
                    for [a,b,c] in crate::topology::cell_faces(i,columns) {
                        faces.push([a, b, c]);
                        for (a, b) in [(a, b), (b, c), (c, a)] {
                            edges.insert([a.min(b), a.max(b)]);
                        }
                    }
                }
            }
        } else {
            if particles > MAX_STENCILS + 1 {
                return Err(XpbdError::SurfaceBudget);
            }
            edges.extend((1..particles).map(|i| [i - 1, i]));
        }
        if edges.len() + 2 * faces.len() > MAX_STENCILS {
            return Err(XpbdError::SurfaceBudget);
        }
        let mut stencils = edges.into_iter().map(Stencil::Edge).collect::<Vec<_>>();
        for &ids in &faces {
            for sign in [-1, 1] {
                stencils.push(Stencil::FaceCap(ids, sign));
            }
        }
        let blocks = (0..stencils.len())
            .step_by(12)
            .map(|i| Block::new(i..(i + 12).min(stencils.len()), &stencils))
            .collect::<Vec<_>>();
        let bounds = vec![Bounds::empty(); blocks.len()];
        let patches = if mu > 0. {
            for f in &mut faces {
                f.sort_unstable();
            }
            crate::material::patches(&faces)?
        } else {
            BTreeMap::new()
        };
        Ok(Self {
            friction: Friction::new(mu),
            bounds,
            checked_initial: false,
            retry_manifold: false,
            mesh: std::sync::Arc::new(Mesh {
                stencils,
                blocks,
                patches,
            }),
            contacts: BTreeMap::new(),
            loads: BTreeMap::new(),
            published: vec![],
            queries: 0,
        })
    }
    pub fn begin_step(&mut self) {
        self.loads.clear();
        self.friction.begin_step();
    }
    pub fn begin_substep(&mut self) {
        self.contacts.clear();
        self.queries = 0;
        self.checked_initial = false;
    }
    pub fn publish(&mut self) {
        self.friction.publish();
        self.published.clear();
        self.published.extend(self.loads.values().copied());
    }
    fn record(
        &mut self,
        key: Key,
        c: Candidate,
        time: f64,
        j: DVec3,
        arm: DVec3,
        r: &mut BoundaryImpulse,
    ) -> Result<(), XpbdError> {
        if j == DVec3::ZERO {
            return Ok(());
        }
        r.react(arm, j)?;
        let v = self.loads.entry(key).or_insert(CapsuleMaterialLoad {
            two_way: key.two_way,
            boundary: key.boundary,
            feature: key.feature,
            region: key.part,
            first_time: time,
            linear: [0.; 3],
            angular: [0.; 3],
            nodal: [[0.; 3]; 3],
        });
        v.first_time = v.first_time.min(time);
        v.linear = (DVec3::from(v.linear) - j).to_array();
        v.angular = (DVec3::from(v.angular) - arm.cross(j)).to_array();
        for (node, w) in v.nodal.iter_mut().zip(c.weights) {
            *node = (DVec3::from(*node) + w * j).to_array();
        }
        if !DVec3::from(v.angular).is_finite()
            || !DVec3::from(v.linear).is_finite()
            || v.nodal.iter().any(|v| !DVec3::from(*v).is_finite())
        {
            return Err(XpbdError::Numerical);
        }
        Ok(())
    }
    pub fn project(
        &mut self,
        x: &mut [DVec3],
        p: &mut [Particle],
        bs: &mut [PreparedBoundary],
        r: &mut [BoundaryImpulse],
        dt: Interval,
    ) -> Result<(), XpbdError> {
        if !bs.iter().any(PreparedBoundary::material_capsule) {
            return Ok(());
        }
        let mut whole = Bounds::empty();
        for i in 0..p.len() {
            whole.particle(i, x, p);
        }
        let mut guides = vec![];
        for (k, b) in bs.iter().enumerate() {
            if let Some(tr) = Trajectory::new(b, dt)? {
                guides.push((k, tr, tr.bounds()));
            }
        }
        if guides.iter().all(|(_, _, bound)| !whole.overlaps(*bound)) {
            self.checked_initial = true;
            return Ok(());
        }
        for _ in 0..MAX_CORRECTIONS {
            let mut earliest = f64::INFINITY;
            let mut group = BTreeMap::<Key, Candidate>::new();
            for (block, bounds) in self.mesh.blocks.iter().zip(&mut self.bounds) {
                *bounds = block.bounds(x, p);
            }
            for &(k, tr, guide_bounds) in &guides {
                let b = &bs[k];
                let g = tr.linear;
                for (block, bounds) in self.mesh.blocks.iter().zip(&self.bounds) {
                    if !bounds.overlaps(guide_bounds) {
                        continue;
                    }
                    for stencil in &self.mesh.stencils[block.range.clone()] {
                        if !self.checked_initial
                            && let Stencil::FaceCap(ids, -1) = stencil
                            && geometry::pierces(ids.map(|i| p[i].position.into()), g.center, g.arm)
                        {
                            return Err(XpbdError::UnresolvedSurface);
                        }
                        let candidates =
                            tr.candidates(*stencil, x, p, &mut self.queries)
                                .map_err(|e| {
                                    if b.receiver.is_some()
                                        && matches!(
                                            e,
                                            XpbdError::UnresolvedSurface | XpbdError::SurfaceBudget
                                        )
                                    {
                                        XpbdError::UnresolvedSweep
                                    } else {
                                        e
                                    }
                                })?;
                        for c in candidates.into_iter().flatten() {
                            let key = Key {
                                two_way: b.receiver.is_some(),
                                boundary: k,
                                feature: c.feature,
                                part: c.part,
                            };
                            if b.receiver.is_none() && c.inverse_mass(p) <= f64::MIN_POSITIVE {
                                if c.depth > c.tolerance
                                    || (c.point(|i| p[i].velocity.into()) - b.velocity)
                                        .dot(c.normal)
                                        < -c.tolerance / dt.h
                                {
                                    return Err(XpbdError::UnresolvedSurface);
                                }
                                continue;
                            }
                            if c.depth > c.tolerance {
                                // Co-temporal contacts on different material points or
                                // guides must share one solve. Selecting just the first
                                // stencil can tilt a symmetric sheet during impact.
                                const TIME_TOL: f64 = 1e-9;
                                if c.time + TIME_TOL < earliest {
                                    group.clear();
                                    earliest = c.time;
                                }
                                if c.time <= earliest + TIME_TOL {
                                    let row = group.entry(key).or_insert(c);
                                    if c.depth > row.depth {
                                        *row = c;
                                    }
                                    earliest = earliest.min(c.time);
                                }
                            } else {
                                self.contacts.entry(key).or_insert(Contact {
                                    witness: c,
                                    tangent_impulse: DVec3::ZERO,
                                    normal_impulse: 0.,
                                    first_time: dt.t0 + c.time * dt.h,
                                });
                            }
                        }
                    }
                }
            }
            self.checked_initial = true;
            if self.contacts.len() > MAX_ROWS {
                return Err(XpbdError::SurfaceBudget);
            }
            if group.is_empty() {
                return Ok(());
            }
            if group.len() > MAX_ROWS {
                return Err(XpbdError::SurfaceBudget);
            }
            // A nearly unloaded opposing jaw can be out of the earliest CCD
            // group but become active under its correction. Include its signed
            // endpoint gap in the same complementarity solve.
            if guides.len() == 2 && guides.iter().all(|(k, _, _)| bs[*k].receiver.is_some()) {
                let existing = group.iter().map(|(k, c)| (*k, *c)).collect::<Vec<_>>();
                for &(k, tr, _) in &guides {
                    for &stencil in &self.mesh.stencils {
                        let c = tr.endpoint(stencil, x, p)?;
                        let Some(c) = c else {
                            continue;
                        };
                        let key = Key {
                            two_way: true,
                            boundary: k,
                            part: c.part,
                            feature: c.feature,
                        };
                        if group.contains_key(&key) {
                            continue;
                        }
                        if existing.iter().any(|(source, row)| {
                            source.boundary != k
                                && row.normal.dot(c.normal) < -0.5
                                && c.depth >= -(4. * row.depth + 4. * c.tolerance)
                                && source
                                    .feature
                                    .vertices()
                                    .iter()
                                    .any(|id| c.feature.vertices().contains(id))
                        }) {
                            group.insert(key, c);
                        }
                    }
                }
                if group.len() > MAX_ROWS {
                    return Err(XpbdError::SurfaceBudget);
                }
            }
            let rows = group
                .iter()
                .map(|(key, c)| {
                    let b = &bs[key.boundary];
                    let arm = b.receiver.as_ref().map(|r| {
                        (
                            key.boundary,
                            c.point(|i| p[i].position.into())
                                - b.origin
                                - dt.t0 * b.velocity
                                - r.start_offset,
                        )
                    });
                    Row { c: *c, rigid: arm }
                })
                .collect::<Vec<_>>();
            let free = rows.iter().map(|row| -row.c.depth / dt.h).collect();
            let impulses = solve_normals(&rows, p, bs, free, &vec![0.; rows.len()])?;
            let mut wrenches = BTreeMap::<usize, (DVec3, DVec3)>::new();
            for (((key, c), j), row) in group.into_iter().zip(impulses).zip(rows) {
                if j <= 0. {
                    continue;
                }
                let impulse = j * c.normal;
                for (&i, w) in key.feature.vertices().iter().zip(c.weights) {
                    let dx = dt.h * p[i].inverse_mass * w * impulse;
                    crate::correct(&mut x[i], &mut p[i], dx, dt.h);
                }
                if let Some((k, arm)) = row.rigid {
                    let (lin, ang) = wrenches.entry(k).or_default();
                    *lin -= impulse;
                    *ang -= arm.cross(impulse);
                }
                let t = dt.t0 + c.time * dt.h;
                let row = self.contacts.entry(key).or_insert(Contact {
                    witness: c,
                    tangent_impulse: DVec3::ZERO,
                    normal_impulse: 0.,
                    first_time: t,
                });
                row.witness = c;
                row.normal_impulse += j;
                row.first_time = row.first_time.min(t);
                // A positional impulse changes the velocity for the entire substep.
                // Its discrete angular reaction acts about the initial material point.
                let arm = c.point(|i| p[i].position.into()) - bs[key.boundary].origin;
                self.record(key, c, t, impulse, arm, &mut r[key.boundary])?;
            }
            for (k, (linear, angular)) in wrenches {
                bs[k]
                    .receiver
                    .as_mut()
                    .unwrap()
                    .project_material_wrench(linear, angular, dt.h)?;
                let (_, tr, bounds) = guides.iter_mut().find(|(index, _, _)| *index == k).unwrap();
                *tr = Trajectory::new(&bs[k], dt)?.unwrap();
                *bounds = tr.bounds();
            }
        }
        // A moving finite receiver can change the opposing normal during a
        // steep sliding transition. Retry the whole uncommitted tick on a
        // smaller shared interval when this nonlinear projection stalls.
        Err(
            if guides.iter().any(|(k, _, _)| bs[*k].receiver.is_some()) {
                XpbdError::UnresolvedSweep
            } else {
                XpbdError::SurfaceBudget
            },
        )
    }
    pub fn finish(
        &mut self,
        x: &[DVec3],
        p: &mut [Particle],
        bs: &mut [PreparedBoundary],
        r: &mut [BoundaryImpulse],
        dt: Interval,
    ) -> Result<(), XpbdError> {
        let h = dt.h;
        self.check_finite_endpoint(x, bs, dt.radius)?;
        let rows = self
            .contacts
            .iter()
            .map(|(&k, &c)| (k, c))
            .collect::<Vec<_>>();
        let n = rows.len();
        if n == 0 {
            return Ok(());
        }
        let mut free = Vec::with_capacity(n);
        let mut seeded = Vec::with_capacity(n);
        let mut witnesses = Vec::with_capacity(n);
        for (a, row) in &rows {
            let c = row.witness;
            let b = &bs[a.boundary];
            let arm = b.receiver.as_ref().map(|r| {
                (
                    a.boundary,
                    c.point(|i| x[i]) - b.origin - r.end_time * b.velocity - r.end_offset,
                )
            });
            let point = Row { c, rigid: arm };
            let vn = point.normal_velocity(p, b);
            free.push(vn - (vn - c.overlap / h).max(0.));
            seeded.push(row.normal_impulse);
            witnesses.push(point);
        }
        if self.friction.active() {
            return self.finish_friction(x, p, bs, r, dt, &rows, &witnesses, free);
        }
        let total = solve_normals(&witnesses, p, bs, free, &seeded)?;
        for (((key, row), &j), point) in rows.iter().copied().zip(&total).zip(witnesses) {
            let c = row.witness;
            let delta = (j - row.normal_impulse) * c.normal;
            point.apply_normal(p, &mut bs[key.boundary], delta);
            let arm = c.point(|i| x[i]) - bs[key.boundary].origin;
            self.record(key, c, row.first_time, delta, arm, &mut r[key.boundary])?;
        }
        Ok(())
    }
}

// Every pair of normal rows interacts only through shared mobile vertices.
// The positional and velocity solves use the same symmetric mass matrix.
struct Row {
    c: Candidate,
    rigid: Option<(usize, DVec3)>,
}
impl Row {
    fn normal_velocity(&self, p: &[Particle], b: &PreparedBoundary) -> f64 {
        let v = b
            .receiver
            .as_ref()
            .zip(self.rigid)
            .map_or(b.velocity, |(r, (_, arm))| {
                b.velocity + r.linear + r.angular_velocity().cross(arm)
            });
        (self.c.point(|i| p[i].velocity.into()) - v).dot(self.c.normal)
    }
    fn apply_normal(&self, p: &mut [Particle], b: &mut PreparedBoundary, delta: DVec3) {
        for (&i, w) in self.c.feature.vertices().iter().zip(self.c.weights) {
            p[i].velocity = (DVec3::from(p[i].velocity) + w * p[i].inverse_mass * delta).to_array();
        }
        if let Some((_, arm)) = self.rigid {
            b.receiver
                .as_mut()
                .unwrap()
                .apply_velocity_impulse(arm, -delta);
        }
    }
}
fn solve_normals(
    rows: &[Row],
    p: &[Particle],
    bs: &[PreparedBoundary],
    mut free: Vec<f64>,
    seed: &[f64],
) -> Result<Vec<f64>, XpbdError> {
    let rigid_mass = |a: &Row, b: &Row| {
        let (Some((i, ra)), Some((j, rb))) = (a.rigid, b.rigid) else {
            return 0.;
        };
        if i != j {
            return 0.;
        }
        let r = bs[i].receiver.as_ref().unwrap();
        r.inverse_mass * a.c.normal.dot(b.c.normal)
            + ra.cross(a.c.normal)
                .dot(r.inverse_inertia(rb.cross(b.c.normal)))
    };
    if let [a] = rows {
        let w = a.c.inverse_mass(p) + rigid_mass(a, a);
        return Ok(vec![(seed[0] - free[0] / w).max(0.)]);
    }
    let n = rows.len();
    let mut k = vec![vec![0.; n]; n];
    for (i, a) in rows.iter().enumerate() {
        for (j, b) in rows[..=i].iter().enumerate() {
            let mut w = 0.;
            for (&idx, wa) in a.c.feature.vertices().iter().zip(a.c.weights) {
                if let Some(loc) = b.c.feature.vertices().iter().position(|&v| v == idx) {
                    w += wa * b.c.weights[loc] * p[idx].inverse_mass;
                }
            }
            let m = w * a.c.normal.dot(b.c.normal) + rigid_mass(a, b);
            k[i][j] = m;
            k[j][i] = m;
        }
    }
    for (v, row) in free.iter_mut().zip(&k) {
        *v -= row.iter().zip(seed).map(|(k, j)| k * j).sum::<f64>();
    }
    if n <= 8 {
        crate::impact_group::inelastic(&k, &free, n)
    } else {
        crate::pad::normal::iterative(&k, &free, seed)
    }
    .ok_or(XpbdError::UnresolvedSurface)
}

#[cfg(test)]
mod oracle;
