// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Finite-thickness continuous self-contact on linearly swept material features.
//! A feature pair is active when its distance reaches twice the particle radius.
//! Every pair includes all boundary Voronoi regions, so face edges and corners
//! participate. Endpoints belonging to the same material one-ring are excluded.
//! The earliest normal response is inelastic and frictionless. Its four signed
//! barycentric weights distribute position correction and impulse among both
//! features using the effective inverse mass. Search restarts after a response.
//!
//! Each projection admits 4,096 vertices and 8,192 triangles, up to one million
//! broad-phase pairs along the selected axis, 65,536 closest-point queries and
//! 96 contact responses. Each pair may advance at most 128 times. An unresolved
//! grazing interval, ambiguous coincidence, conflicting prescribed motion or
//! exhausted budget returns an error before the public step publishes.
mod friction;
pub(super) mod geometry;
mod interval;
mod loads;
pub use loads::SelfContactLoad;
mod position;
use super::{Particle, XpbdError};
use geometry::{Witness, edge_edge, vertex_face};
use glam::DVec3;
use std::collections::BTreeSet;

const MAX_VERTICES: usize = 4096;
const MAX_TRIANGLES: usize = 8192;
const MAX_CANDIDATES: usize = 1_000_000;
const MAX_DISTANCE_QUERIES: usize = 65_536;
const MAX_PROJECTIONS: usize = 96;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Feature {
    Point(usize),
    Edge([usize; 2]),
    Face([usize; 3]),
}

impl Feature {
    fn vertices(&self) -> &[usize] {
        match self {
            Self::Point(i) => std::slice::from_ref(i),
            Self::Edge(pair) => pair,
            Self::Face(tri) => tri,
        }
    }
}

#[derive(Clone, Copy)]
struct Bound {
    feature: Feature,
    lo: DVec3,
    hi: DVec3,
}

#[derive(Clone)]
pub(super) struct Surface {
    features: Vec<Feature>,
    connected: BTreeSet<[usize; 2]>,
    loose: Vec<bool>,
    bounds: Vec<Bound>,
    friction: Option<friction::Friction>,
    pub(super) impulse: crate::SelfContactImpulse,
    loads: Option<loads::Loads>,
    // A smaller interval can relieve projection or swept-distance exhaustion.
    // Topology and broad-phase capacity failures retain their hard limits.
    pub(super) subdivide_on_budget: bool,
    pub(super) retry_plane: bool,
    pub(super) prioritize_terminal_speed: bool,
}

fn edge(a: usize, b: usize) -> [usize; 2] {
    [a.min(b), a.max(b)]
}

impl Surface {
    pub fn set_friction(&mut self, coefficient: f64) -> Result<(), XpbdError> {
        if !coefficient.is_finite() || coefficient < 0.0 {
            return Err(XpbdError::InvalidConfig);
        }
        self.friction = if coefficient == 0. && self.loose.iter().any(|&loose| loose) {
            None
        } else {
            Some(friction::Friction::new(coefficient, &self.pad_faces()?)?)
        };
        Ok(())
    }
    pub fn pad_faces(&self) -> Result<Vec<[usize; 3]>, XpbdError> {
        if self.loose.iter().any(|&v| v) {
            return Err(XpbdError::InvalidConstraint);
        }
        Ok(self
            .features
            .iter()
            .filter_map(|f| {
                if let Feature::Face(ids) = f {
                    Some(*ids)
                } else {
                    None
                }
            })
            .collect())
    }

    pub fn validate_size(vertices: usize, triangles: usize) -> Result<(), XpbdError> {
        if vertices > MAX_VERTICES || triangles > MAX_TRIANGLES {
            Err(XpbdError::SurfaceBudget)
        } else {
            Ok(())
        }
    }

    pub fn new(triangles: Vec<[usize; 3]>, particles: &[Particle]) -> Result<Self, XpbdError> {
        if triangles.is_empty() {
            return Err(XpbdError::InvalidConstraint);
        }
        Self::validate_size(particles.len(), triangles.len())?;
        let mut faces = BTreeSet::new();
        let mut directed = BTreeSet::new();
        let mut connected = BTreeSet::new();
        let mut loose = vec![true; particles.len()];
        for triangle in triangles {
            let mut sorted = triangle;
            sorted.sort_unstable();
            if sorted[2] >= particles.len()
                || sorted[0] == sorted[1]
                || sorted[1] == sorted[2]
                || !faces.insert(sorted)
            {
                return Err(XpbdError::InvalidConstraint);
            }
            let [a, b, c] = triangle.map(|i| DVec3::from(particles[i].position));
            if (b - a).cross(c - a).try_normalize().is_none() {
                return Err(XpbdError::InvalidConstraint);
            }
            for [a, b] in [
                [triangle[0], triangle[1]],
                [triangle[1], triangle[2]],
                [triangle[2], triangle[0]],
            ] {
                if !directed.insert([a, b]) {
                    return Err(XpbdError::InvalidConstraint);
                }
                connected.insert(edge(a, b));
                loose[a] = false;
            }
        }
        let features = (0..particles.len())
            .map(Feature::Point)
            .chain(connected.iter().copied().map(Feature::Edge))
            .chain(faces.into_iter().map(Feature::Face))
            .collect();
        let mut surface = Self {
            features,
            connected,
            loose,
            bounds: Vec::new(),
            friction: None,
            impulse: crate::SelfContactImpulse::default(),
            loads: None,
            subdivide_on_budget: false,
            retry_plane: false,
            prioritize_terminal_speed: false,
        };
        let positions = particles
            .iter()
            .map(|p| DVec3::from(p.position))
            .collect::<Vec<_>>();
        surface.check_intersections(&positions)?;
        Ok(surface)
    }

    fn candidate(&self, a: Feature, b: Feature) -> Option<Stencil> {
        let (ids, vf) = match (a.min(b), a.max(b)) {
            (Feature::Point(v), Feature::Face([a, b, c])) => ([v, a, b, c], true),
            (Feature::Edge([a, b]), Feature::Edge([c, d])) => ([a, b, c, d], false),
            (Feature::Point(a), Feature::Point(b)) if self.loose[a] && self.loose[b] => {
                ([a, a, b, b], false)
            }
            _ => return None,
        };
        let split = if vf { 1 } else { 2 };
        if ids[..split].iter().any(|a| {
            ids[split..]
                .iter()
                .any(|b| a == b || self.connected.contains(&edge(*a, *b)))
        }) {
            return None;
        }
        Some(Stencil { ids, vf })
    }

    fn bounds(&mut self, p: Option<&[Particle]>, x: &[DVec3], margin: f64) -> usize {
        self.bounds.clear();
        let mut low = DVec3::splat(f64::INFINITY);
        let mut high = DVec3::splat(f64::NEG_INFINITY);
        let mut widths = DVec3::ZERO;
        for &feature in &self.features {
            if p.is_none() && !matches!(feature, Feature::Face(_)) {
                continue;
            }
            let mut lo = DVec3::splat(f64::INFINITY);
            let mut hi = DVec3::splat(f64::NEG_INFINITY);
            for &i in feature.vertices() {
                let start = p.map_or(x[i], |p| DVec3::from(p[i].position));
                lo = lo.min(start).min(x[i]);
                hi = hi.max(start).max(x[i]);
            }
            low = low.min(lo);
            high = high.max(hi);
            widths += hi - lo + DVec3::splat(2. * margin);
            self.bounds.push(Bound {
                feature,
                lo: lo - margin,
                hi: hi + margin,
            });
        }
        let score = (high - low) / widths.max(DVec3::splat(f64::MIN_POSITIVE));
        let axis = (0..3)
            .max_by(|&a, &b| score[a].total_cmp(&score[b]))
            .unwrap_or(0);
        self.bounds.sort_unstable_by(|a, b| {
            a.lo[axis]
                .total_cmp(&b.lo[axis])
                .then(a.feature.cmp(&b.feature))
        });
        axis
    }

    /// An already interlocked triangle pair need not have a close vertex-face
    /// or edge-edge witness. Swept boundary feature pairs assume a valid start.
    pub fn check_intersections(&mut self, x: &[DVec3]) -> Result<(), XpbdError> {
        let axis = self.bounds(None, x, 0.);
        let mut candidates = 0;
        for (i, a) in self.bounds.iter().enumerate() {
            for b in &self.bounds[i + 1..] {
                if b.lo[axis] > a.hi[axis] {
                    break;
                }
                candidates += 1;
                if candidates > MAX_CANDIDATES {
                    return Err(XpbdError::SurfaceBudget);
                }
                if a.lo.cmpgt(b.hi).any() || b.lo.cmpgt(a.hi).any() {
                    continue;
                }
                let (Feature::Face(a), Feature::Face(b)) = (a.feature, b.feature) else {
                    unreachable!("face bounds")
                };
                if a.iter().any(|i| b.contains(i)) {
                    continue;
                }
                if geometry::intersect(a.map(|i| x[i]), b.map(|i| x[i])) {
                    return Err(XpbdError::UnresolvedSurface);
                }
            }
        }
        Ok(())
    }

    pub fn project(
        &mut self,
        x: &mut [DVec3],
        p: &mut [Particle],
        gap: f64,
        h: f64,
        on_plane: bool,
    ) -> Result<(), XpbdError> {
        let eps = Self::tolerance(x, p, gap)?;
        let mut candidates = 0;
        let mut queries = 0;
        let mut count = 0;
        loop {
            let Some(hit) = self.next_hit(
                x,
                p,
                gap,
                eps,
                (&mut candidates, &mut queries),
                if on_plane || self.retry_plane {
                    Search::Supported
                } else {
                    Search::Ordinary
                },
            )?
            else {
                return Ok(());
            };
            if count == MAX_PROJECTIONS {
                self.subdivide_on_budget = true;
                return Err(XpbdError::SurfaceBudget);
            }
            self.resolve(&hit, x, p, gap, h)?;
            count += 1;
        }
    }
    fn resolve(
        &mut self,
        hit: &Hit,
        x: &mut [DVec3],
        p: &mut [Particle],
        gap: f64,
        h: f64,
    ) -> Result<(), XpbdError> {
        if let Some(friction) = &self.friction {
            let impulse = friction.normal(hit, x, p, h)?;
            let (tangent, work) = friction.apply(hit, impulse, x, p, gap)?;
            self.record_load(hit, impulse);
            self.impulse.normal += impulse;
            self.impulse.tangent += tangent;
            self.impulse.friction_work += work;
            if ![
                self.impulse.normal,
                self.impulse.tangent,
                self.impulse.friction_work,
            ]
            .into_iter()
            .all(f64::is_finite)
            {
                return Err(XpbdError::Numerical);
            }
        } else {
            hit.apply(x, p)?;
        }
        Ok(())
    }
    /// Resolve remaining closing speed at a touching shell after the support
    /// applies its end-of-step impulse. Leave the certified path unchanged.
    pub(super) fn finish_supported(
        &mut self,
        x: &mut [DVec3],
        p: &mut [Particle],
        gap: f64,
        mut support: impl FnMut(&[DVec3], &mut [Particle]) -> Result<(), XpbdError>,
    ) -> Result<(), XpbdError> {
        let eps = Self::tolerance(x, p, gap)?;
        let (mut candidates, mut queries) = (0, 0);
        for _ in 0..MAX_PROJECTIONS {
            let Some(mut hit) = self.next_hit(
                x,
                p,
                gap,
                eps,
                (&mut candidates, &mut queries),
                Search::TerminalVelocity,
            )?
            else {
                return Ok(());
            };
            let points = hit.stencil.ids.map(|i| x[i]);
            let witness = if hit.stencil.vf {
                vertex_face(&points)
            } else {
                edge_edge(&points)
            };
            if hit.speed == 0. || hit.depth > eps || (witness.delta.length() - gap).abs() > 4. * eps
            {
                return Err(XpbdError::UnresolvedSurface);
            }
            hit.depth = 0.;
            self.resolve(&hit, x, p, gap, 0.)?;
            support(x, p)?;
        }
        Err(XpbdError::UnresolvedSurface)
    }
    pub fn verify(&mut self, x: &[DVec3], p: &[Particle], gap: f64) -> Result<(), XpbdError> {
        let eps = Self::tolerance(x, p, gap)?;
        if self
            .next_hit(
                x,
                p,
                gap,
                eps,
                (&mut 0, &mut 0),
                if self.retry_plane {
                    Search::Supported
                } else {
                    Search::Ordinary
                },
            )?
            .is_some()
        {
            return Err(XpbdError::UnresolvedSurface);
        }
        Ok(())
    }
    fn tolerance(x: &[DVec3], p: &[Particle], gap: f64) -> Result<f64, XpbdError> {
        let scale = x
            .iter()
            .zip(p.iter())
            .map(|(x, p)| x.abs().max(DVec3::from(p.position).abs()).max_element())
            .fold(gap, f64::max);
        let eps = 1e-9 * gap + 128. * f64::EPSILON * scale;
        if !eps.is_finite() || eps > 1e-4 * gap {
            return Err(XpbdError::Numerical);
        }
        Ok(eps)
    }
    fn next_hit(
        &mut self,
        x: &[DVec3],
        p: &[Particle],
        gap: f64,
        eps: f64,
        budget: (&mut usize, &mut usize),
        search: Search,
    ) -> Result<Option<Hit>, XpbdError> {
        let (candidates, queries) = budget;
        let mode = if self.friction.is_none() {
            SweepMode::Geometric
        } else if search != Search::Ordinary {
            SweepMode::SupportedShell
        } else {
            SweepMode::MaterialShell
        };
        let axis = self.bounds(Some(p), x, (gap + eps) * 0.5);
        let mut earliest: Option<Hit> = None;
        for (i, a) in self.bounds.iter().enumerate() {
            for b in &self.bounds[i + 1..] {
                if b.lo[axis] > a.hi[axis] {
                    break;
                }
                *candidates += 1;
                if *candidates > MAX_CANDIDATES {
                    return Err(XpbdError::SurfaceBudget);
                }
                if b.lo.cmpgt(a.hi).any() || a.lo.cmpgt(b.hi).any() {
                    continue;
                }
                let Some(stencil) = self.candidate(a.feature, b.feature) else {
                    continue;
                };
                let hit = stencil
                    .sweep(x, p, gap, eps, queries, mode)
                    .inspect_err(|error| {
                        if *error == XpbdError::SurfaceBudget && *queries > MAX_DISTANCE_QUERIES {
                            self.subdivide_on_budget = true;
                        }
                    })?;
                if let Some(hit) = hit
                    && earliest.as_ref().is_none_or(|old| {
                        // Geometry has already been resolved in the terminal
                        // velocity pass. Resolve its most violated normal first
                        // when retrying a cluster of near-parallel constraints.
                        let velocity_order = if self.prioritize_terminal_speed
                            && search == Search::TerminalVelocity
                        {
                            old.speed.total_cmp(&hit.speed)
                        } else {
                            std::cmp::Ordering::Equal
                        };
                        velocity_order
                            .then(hit.time.total_cmp(&old.time))
                            .then(stencil.ids.cmp(&old.stencil.ids))
                            .is_lt()
                    })
                {
                    earliest = Some(hit);
                }
            }
        }
        Ok(earliest)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Search {
    Ordinary,
    Supported,
    TerminalVelocity,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SweepMode {
    Geometric,
    MaterialShell,
    SupportedShell,
    MaterialCapsule,
}

#[derive(Clone, Copy)]
struct Stencil {
    ids: [usize; 4],
    vf: bool,
}
pub(super) struct Hit {
    stencil: Stencil,
    pub(super) weights: [f64; 4],
    pub(super) normal: DVec3,
    pub(super) time: f64,
    pub(super) depth: f64,
    pub(super) speed: f64,
}

impl Stencil {
    fn sweep(
        self,
        x: &[DVec3],
        p: &[Particle],
        gap: f64,
        eps: f64,
        queries: &mut usize,
        mode: SweepMode,
    ) -> Result<Option<Hit>, XpbdError> {
        self.sweep_points(
            self.ids.map(|i| DVec3::from(p[i].position)),
            self.ids.map(|i| x[i]),
            self.ids.map(|i| DVec3::from(p[i].velocity)),
            gap,
            eps,
            queries,
            mode,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn sweep_points(
        self,
        start: [DVec3; 4],
        end: [DVec3; 4],
        velocities: [DVec3; 4],
        gap: f64,
        eps: f64,
        queries: &mut usize,
        mode: SweepMode,
    ) -> Result<Option<Hit>, XpbdError> {
        let material = mode != SweepMode::Geometric;
        let Some(Entry {
            weights,
            normal: n,
            time,
            certified,
        }) = first_entry_mode(start, end, self.vf, gap, eps, queries, material)?
        else {
            return Ok(None);
        };
        let end_gap = (0..4)
            .map(|i| ((end[i] - end[0]).dot(n)) * weights[i])
            .sum::<f64>();
        let speed = (0..4)
            .map(|i| velocities[i].dot(n) * weights[i])
            .sum::<f64>();
        let scale = velocities.iter().map(|v| v.length()).fold(1., f64::max);
        let inner_gap = || {
            let initial = if self.vf {
                vertex_face(&start)
            } else {
                edge_edge(&start)
            };
            // An initial overlap may resolve monotonically. Prove the curve
            // never penetrates beyond its already admitted starting depth.
            gap.min(initial.delta.length()) - 4. * eps
        };
        let clear_material = || {
            let lower = inner_gap();
            lower > 0.
                && if self.vf {
                    interval::clear_triangle(start, end, lower, true)
                } else {
                    interval::clear_edges(start, end, lower)
                }
        };
        // A touching feature may roll across its neighbour during a substep.
        // A certified non-penetrating trajectory uses the current closest
        // material normal for its velocity condition, instead of an obsolete
        // half-space at the start of the interval.
        if material && (time == 0. || mode == SweepMode::SupportedShell) {
            let endpoint = if self.vf {
                vertex_face(&end)
            } else {
                edge_edge(&end)
            };
            let distance = endpoint.delta.length();
            let touching = distance <= gap + 4. * eps;
            let supported =
                mode == SweepMode::SupportedShell && (speed < 0. || end_gap < gap - eps);
            if distance > eps && endpoint.delta.dot(n) > 0. && (touching || supported) {
                let penetrated = distance < gap - eps;
                let lower = inner_gap();
                let use_endpoint = penetrated
                    || clear_material()
                    || (supported
                        && lower > 0.
                        && first_entry_mode(start, end, self.vf, lower, eps, queries, true)?
                            .is_none());
                if use_endpoint {
                    // A support can change the terminal velocity without changing
                    // the path. A detached witness then carries no endpoint load.
                    if touching {
                        let normal = endpoint.delta / distance;
                        let vn = endpoint
                            .weights
                            .iter()
                            .zip(velocities)
                            .map(|(&w, v)| w * v.dot(normal))
                            .sum::<f64>();
                        if penetrated || vn < -128. * f64::EPSILON * scale {
                            return Ok(Some(Hit {
                                stencil: self,
                                weights: endpoint.weights,
                                normal,
                                time,
                                depth: (gap - distance).max(0.),
                                speed: (-vn).max(0.),
                            }));
                        }
                    }
                    return Ok(None);
                }
            }
        }
        if end_gap < gap - eps || speed < -128. * f64::EPSILON * scale {
            return Ok(Some(Hit {
                stencil: self,
                weights,
                normal: n,
                time,
                depth: (gap - end_gap).max(0.),
                speed: (-speed).max(0.),
            }));
        }
        if !material {
            return if certified {
                Ok(None)
            } else {
                Err(XpbdError::UnresolvedSurface)
            };
        }
        // During sliding the closest material point can move away from the
        // initial barycentric coordinates. Resolve an actual endpoint overlap
        // on the same incident side before accepting a separating certificate.
        let terminal = if self.vf {
            vertex_face(&end)
        } else {
            edge_edge(&end)
        };
        let distance = terminal.delta.length();
        if distance < gap - eps && distance > eps && terminal.delta.dot(n) > 0. {
            let normal = terminal.delta / distance;
            let speed = terminal
                .weights
                .iter()
                .zip(velocities)
                .map(|(&w, v)| w * v.dot(normal))
                .sum::<f64>();
            return Ok(Some(Hit {
                stencil: self,
                weights: terminal.weights,
                normal,
                time: 1.,
                depth: gap - distance,
                speed: (-speed).max(0.),
            }));
        }
        if certified || clear_material() {
            return Ok(None);
        }
        // Nearly parallel finite edges may graze at the very end of this
        // interval while the whole-interval polynomial is inconclusive.
        // For shell self-contact certify the same inner clearance with the
        // bounded continuous search. A capsule carries a separate trajectory
        // envelope and retains its existing coupled subdivision on refusal.
        if matches!(mode, SweepMode::MaterialShell | SweepMode::SupportedShell) {
            let lower = inner_gap();
            if lower > 0.
                && first_entry_mode(start, end, self.vf, lower, eps, queries, true)?.is_none()
            {
                return Ok(None);
            }
        }
        Err(XpbdError::UnresolvedSurface)
    }
}

/// Same swept material geometry for two soft features or a soft feature
/// against a prescribed finite segment. Velocities are in world axes.
pub(super) fn material_sweep(
    start: [DVec3; 4],
    end: [DVec3; 4],
    velocity: [DVec3; 4],
    vertex_face: bool,
    gap: f64,
    eps: f64,
    queries: &mut usize,
) -> Result<Option<Hit>, XpbdError> {
    Stencil {
        ids: [0, 1, 2, 3],
        vf: vertex_face,
    }
    .sweep_points(
        start,
        end,
        velocity,
        gap,
        eps,
        queries,
        SweepMode::MaterialCapsule,
    )
}

pub(super) struct Entry {
    pub weights: [f64; 4],
    pub normal: DVec3,
    pub time: f64,
    pub certified: bool,
}

/// Earliest contact of the swept convex features. At an already touching
/// start the caller decides whether the continuing motion needs a response.
pub(super) fn first_entry(
    start: [DVec3; 4],
    end: [DVec3; 4],
    vf: bool,
    gap: f64,
    eps: f64,
    queries: &mut usize,
) -> Result<Option<Entry>, XpbdError> {
    first_entry_mode(start, end, vf, gap, eps, queries, false)
}
pub(super) fn first_entry_mode(
    start: [DVec3; 4],
    end: [DVec3; 4],
    vf: bool,
    gap: f64,
    eps: f64,
    queries: &mut usize,
    material: bool,
) -> Result<Option<Entry>, XpbdError> {
    let path: [DVec3; 4] = std::array::from_fn(|i| end[i] - start[i]);
    let split = if vf { 1 } else { 2 };
    let relative = path[..split]
        .iter()
        .flat_map(|a| path[split..].iter().map(move |b| (*a - *b).length()))
        .fold(0., f64::max);
    let mut time = 0.;
    for _ in 0..128 {
        *queries += 1;
        if *queries > MAX_DISTANCE_QUERIES {
            return Err(XpbdError::SurfaceBudget);
        }
        let pos = std::array::from_fn(|i| start[i] + path[i] * time);
        let witness = if vf {
            vertex_face(&pos)
        } else {
            edge_edge(&pos)
        };
        let Witness { weights, delta } = witness;
        let distance = delta.length();
        if !distance.is_finite() || !relative.is_finite() {
            return Err(XpbdError::Numerical);
        }
        if distance > gap + eps && distance - gap > relative * (1. - time) + eps {
            return Ok(None);
        }
        if distance <= eps {
            return Err(XpbdError::UnresolvedSurface);
        }
        let n = delta / distance;
        // Every point of either swept convex feature is a convex combination.
        // This separating plane certifies the remaining interval, including slides.
        let plane_gap = |points: &[DVec3; 4]| {
            points[..split]
                .iter()
                .flat_map(|a| points[split..].iter().map(move |b| (*a - *b).dot(n)))
                .fold(f64::INFINITY, f64::min)
        };
        if distance <= gap + eps {
            return Ok(Some(Entry {
                weights,
                normal: n,
                time,
                certified: plane_gap(&end) >= gap - eps
                    || relative * (1. - time) <= eps
                    || time == 1.,
            }));
        }
        if plane_gap(&end) > gap + eps {
            return Ok(None);
        }
        // A fixed separating normal gives a tighter safe time than the scalar
        // Lipschitz bound for glancing motion. Each projected vertex pair is
        // affine in time; the plane certificate lasts until its first root.
        let mut plane_time = f64::INFINITY;
        for a in 0..split {
            for b in split..4 {
                let closing = (path[b] - path[a]).dot(n);
                if closing > 0.0 {
                    plane_time = plane_time.min(((pos[a] - pos[b]).dot(n) - gap) / closing);
                }
            }
        }
        let advance = ((distance - gap) / relative).max(plane_time);
        let next = (time + advance).min(1.);
        if next <= time {
            return Err(XpbdError::UnresolvedSurface);
        }
        time = next;
    }
    if vf || material {
        entry_intervals(start, end, vf, time, gap, eps, relative, queries, material)
    } else {
        Err(XpbdError::UnresolvedSurface)
    }
}

/// A moving remote corner can dominate the safe advance while the closest
/// point stays on an almost tangent edge. Bisect the remaining time interval
/// left first. A common separating plane or Lipschitz bound certifies an
/// interval clear; undecidable intervals refuse at the shared query budget.
#[allow(clippy::too_many_arguments)]
fn entry_intervals(
    start: [DVec3; 4],
    end: [DVec3; 4],
    vf: bool,
    time: f64,
    gap: f64,
    eps: f64,
    lipschitz: f64,
    queries: &mut usize,
    material: bool,
) -> Result<Option<Entry>, XpbdError> {
    let at = |t: f64| std::array::from_fn(|i| start[i].lerp(end[i], t));
    let witness = |p: &[DVec3; 4]| if vf { vertex_face(p) } else { edge_edge(p) };
    let split = if vf { 1 } else { 2 };
    let mut stack = vec![(time, 1.)];
    while let Some((lo, hi)) = stack.pop() {
        let mid = (lo + hi) * 0.5;
        let a = at(lo);
        let b = at(hi);
        let clear_interval = if vf {
            if material {
                interval::clear_triangle(a, b, gap, true)
            } else {
                interval::clear(a, b, gap + eps)
            }
        } else {
            interval::clear_edges(a, b, gap)
        };
        if clear_interval {
            continue;
        }
        let mut clear = false;
        let mut normals = [DVec3::ZERO; 2];
        for t in [lo, mid, hi] {
            *queries += 1;
            if *queries > MAX_DISTANCE_QUERIES {
                return Err(XpbdError::SurfaceBudget);
            }
            let w = witness(&at(t));
            let d = w.delta.length();
            if !d.is_finite() || d <= eps {
                return Err(XpbdError::UnresolvedSurface);
            }
            let n = w.delta / d;
            if t == lo {
                normals[0] = n;
            }
            if t == hi {
                normals[1] = n;
            }
            let separated = |p: [DVec3; 4]| {
                p[..split]
                    .iter()
                    .all(|a| p[split..].iter().all(|b| (*a - *b).dot(n) > gap + eps))
            };
            if t == lo && d <= gap + eps {
                return Ok(Some(Entry {
                    time: t,
                    normal: n,
                    weights: w.weights,
                    certified: separated(end),
                }));
            }
            if d - lipschitz * (hi - t).max(t - lo) > gap + eps || separated(a) && separated(b) {
                clear = true;
                break;
            }
        }
        if clear || (material && interval::moving_plane(&a, &b, normals, split, gap)) {
            continue;
        }
        if mid <= lo || mid >= hi {
            return Err(XpbdError::UnresolvedSurface);
        }
        // The entire time slab has less spatial motion than the geometric
        // resolution. Accept its left endpoint only within the same error bound.
        if lipschitz * (hi - lo) <= eps {
            let w = witness(&a);
            let d = w.delta.length();
            if d <= gap + 2. * eps {
                return Ok(Some(Entry {
                    time: lo,
                    normal: w.delta / d,
                    weights: w.weights,
                    certified: false,
                }));
            }
            continue;
        }
        stack.extend([(mid, hi), (lo, mid)]);
    }
    Ok(None)
}

impl Hit {
    fn apply(&self, x: &mut [DVec3], p: &mut [Particle]) -> Result<f64, XpbdError> {
        let denom = (0..4)
            .map(|i| p[self.stencil.ids[i]].inverse_mass * self.weights[i].powi(2))
            .sum::<f64>();
        if !denom.is_finite() || denom <= 0. {
            return Err(XpbdError::UnresolvedSurface);
        }
        for (i, &id) in self.stencil.ids.iter().enumerate() {
            let gradient = self.normal * (p[id].inverse_mass * self.weights[i] / denom);
            x[id] += gradient * self.depth;
            p[id].velocity = (DVec3::from(p[id].velocity) + gradient * self.speed).to_array();
        }
        Ok(self.speed / denom)
    }
}

#[cfg(test)]
#[path = "../tests/private/surface_geometry.rs"]
mod tests;
