// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Coulomb friction at a thick triangle's skin. The finite receiver and
//! every material degree of freedom see the same world-space contact point.
mod manifold;
use super::*;
use crate::material::Skin;
use glam::DVec2;

#[derive(Clone, Debug, PartialEq)]
pub struct CapsuleFrictionLoad {
    pub two_way: bool,
    pub boundary: usize,
    pub feature: MaterialFeature,
    pub region: CapsuleRegion,
    /// Tangential impulse on the guide/receiver, in world axes, N s.
    pub linear: [f64; 3],
    /// Its world moment about the public tick's starting guide center, N m s.
    pub angular: [f64; 3],
    /// All construction particle IDs carrying translational or skin-couple impulses.
    pub nodal: Vec<(usize, [f64; 3])>,
    /// Sums of converged per-substep magnitudes in N s.
    pub normal_impulse_sum: f64,
    pub tangent_impulse_sum: f64,
    /// Last converged endpoint in this public tick, in seconds since its start.
    pub last_time: f64,
    /// Material coordinates in the construction feature's sorted vertex order.
    pub last_weights: [f64; 3],
    /// True receiver surface point, contact normal and relative sliding velocity, in world axes.
    pub last_point: [f64; 3],
    pub last_normal: [f64; 3],
    pub last_slip_velocity: [f64; 3],
    /// Scalar normal and world tangential impulses at that endpoint on the material.
    pub last_normal_impulse: f64,
    pub last_tangent_impulse: [f64; 3],
    /// Sum of normal impulse times tangential relative speed, N m, for load-weighted slip.
    pub normal_weighted_slip: f64,
    /// Sum of endpoint sliding speed times active substep duration, m.
    pub slip_distance: f64,
}

/// One converged compressive contact on the actual shared soft interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapsuleContactSample {
    pub boundary: usize,
    pub feature: MaterialFeature,
    pub region: CapsuleRegion,
    pub two_way: bool,
    /// Start, end and first active time relative to the public tick, seconds.
    pub interval: [f64; 2],
    pub first_time: f64,
    pub weights: [f64; 3],
    pub point: [f64; 3],
    pub normal: [f64; 3],
    /// Positive normal impulse and tangential impulse on the material, N s.
    pub normal_impulse: f64,
    pub tangent_impulse: [f64; 3],
    pub slip_velocity: [f64; 3],
}
#[derive(Clone)]
pub(crate) struct Friction {
    mu: f64,
    loads: BTreeMap<Key, CapsuleFrictionLoad>,
    pub(crate) published: Vec<CapsuleFrictionLoad>,
    samples: Option<Vec<CapsuleContactSample>>,
    pub(crate) published_samples: Vec<CapsuleContactSample>,
}
impl Friction {
    pub(super) fn new(mu: f64) -> Self {
        Self {
            mu,
            loads: BTreeMap::new(),
            published: vec![],
            samples: None,
            published_samples: vec![],
        }
    }
    pub(crate) fn enable_samples(&mut self) -> Result<(), XpbdError> {
        if self.mu == 0. {
            return Err(XpbdError::InvalidConfig);
        }
        self.samples = Some(vec![]);
        Ok(())
    }
    pub(super) fn active(&self) -> bool {
        self.mu > 0.
    }
    pub(super) fn begin_step(&mut self) {
        self.loads.clear();
        if let Some(s) = &mut self.samples {
            s.clear();
        }
    }
    pub(super) fn publish(&mut self) {
        self.published.clear();
        if let Some(s) = &mut self.samples {
            std::mem::swap(s, &mut self.published_samples);
        }
        self.published.extend(self.loads.values().cloned());
    }
    fn load(&mut self, key: Key) -> &mut CapsuleFrictionLoad {
        self.loads
            .entry(key)
            .or_insert_with(|| CapsuleFrictionLoad {
                two_way: key.two_way,
                boundary: key.boundary,
                feature: key.feature,
                region: key.part,
                linear: [0.; 3],
                angular: [0.; 3],
                nodal: vec![],
                normal_impulse_sum: 0.,
                tangent_impulse_sum: 0.,
                last_time: 0.,
                last_weights: [0.; 3],
                last_point: [0.; 3],
                last_normal: [0.; 3],
                last_slip_velocity: [0.; 3],
                last_normal_impulse: 0.,
                last_tangent_impulse: [0.; 3],
                normal_weighted_slip: 0.,
                slip_distance: 0.,
            })
    }
    fn record(
        &mut self,
        key: Key,
        patch: &Skin<'_>,
        arm: DVec3,
        impulse: DVec3,
        reaction: &mut BoundaryImpulse,
    ) -> Result<(), XpbdError> {
        if impulse == DVec3::ZERO {
            return Ok(());
        }
        reaction.react(arm, impulse)?;
        let load = self.load(key);
        load.linear = (DVec3::from(load.linear) - impulse).to_array();
        load.angular = (DVec3::from(load.angular) - arm.cross(impulse)).to_array();
        for (id, j) in patch.nodal(impulse) {
            match load.nodal.binary_search_by_key(&id, |(id, _)| *id) {
                Ok(i) => load.nodal[i].1 = (DVec3::from(load.nodal[i].1) + j).to_array(),
                Err(i) => load.nodal.insert(i, (id, j.to_array())),
            }
        }
        if !DVec3::from(load.linear).is_finite()
            || !DVec3::from(load.angular).is_finite()
            || load.nodal.iter().any(|(_, j)| !DVec3::from(*j).is_finite())
        {
            return Err(XpbdError::Numerical);
        }
        Ok(())
    }
}

/// Minimize 1/2 jᵀ W j + gᵀ j over the circular Coulomb disk.
/// A positive multiplier regularizes the full 2D mass matrix on its boundary.
fn disk(a: f64, b: f64, c: f64, g: DVec2, limit: f64) -> Result<DVec2, XpbdError> {
    if !(limit.is_finite() && a.is_finite() && b.is_finite() && c.is_finite() && g.is_finite()) {
        return Err(XpbdError::Numerical);
    }
    if limit == 0. {
        return Ok(DVec2::ZERO);
    }
    if limit < 0. {
        return Err(XpbdError::Numerical);
    }
    // Ordinary coefficients retain the original rounding and deterministic
    // path. Outside that range, work in units of the Coulomb radius; neither
    // squaring a tiny impulse nor forming a huge multiplier is required.
    let m = a.max(c);
    let force = g.abs().max_element();
    if !(1e-120..=1e120).contains(&m)
        || !(1e-100..=1e100).contains(&limit)
        || (force > 0. && !(1e-100..=1e100).contains(&force))
        || force / limit > 1e140
    {
        return scaled_disk(a, b, c, g, limit);
    }
    let det = a * c - b * b;
    if !(a > 0. && c > 0. && det > 1e-13 * a * c) {
        return Err(crate::observed_refusal());
    }
    let solve = |l: f64| {
        let det = (a + l) * (c + l) - b * b;
        DVec2::new(b * g.y - (c + l) * g.x, b * g.x - (a + l) * g.y) / det
    };
    let free = solve(0.);
    if free.length() <= limit {
        return Ok(free);
    }
    let mut lo = 0.;
    let mut hi = g.length() / limit;
    // The smallest eigenvalue of W is positive, so |j(hi)| <= |g|/hi.
    for _ in 0..52 {
        let mid = 0.5 * (lo + hi);
        if solve(mid).length() > limit {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Ok(solve(hi))
}

fn scaled_disk(a: f64, b: f64, c: f64, g: DVec2, limit: f64) -> Result<DVec2, XpbdError> {
    let m = a.max(c);
    let (a, b, c) = (a / m, b / m, c / m);
    let det = a * c - b * b;
    if !(a > 0. && c > 0. && det > 1e-13 * a * c) {
        return Err(crate::observed_refusal());
    }
    let f = g.abs().max_element();
    if f == 0. {
        return Ok(DVec2::ZERO);
    }
    let h = g / f;
    let free = DVec2::new(b * h.y - c * h.x, b * h.x - a * h.y) / det;
    let scale = f / m;
    let j = free * scale;
    if j.is_finite() && j.x.hypot(j.y) <= limit {
        return Ok(j);
    }
    // For a sliding contact the unconstrained impulse exceeds this radius.
    // Its dimensionless mobility can thus never exceed |free|. Handle an
    // unrepresentable unconstrained scale without overflowing an intermediate.
    let q = if scale.is_infinite() {
        (limit.log2() + m.log2() - f.log2()).exp2()
    } else {
        limit / scale
    };
    let (a, b, c) = (q * a, q * b, q * c);
    let solve = |l: f64| {
        let d = (a + l) * (c + l) - b * b;
        DVec2::new(b * h.y - (c + l) * h.x, b * h.x - (a + l) * h.y) / d
    };
    let mut low = 0.;
    let mut high = h.x.hypot(h.y);
    for _ in 0..54 {
        let mid = 0.5 * (low + high);
        let j = solve(mid);
        if j.x.hypot(j.y) > 1. {
            low = mid;
        } else {
            high = mid;
        }
    }
    let j = limit * solve(high);
    if j.is_finite() {
        Ok(j)
    } else {
        Err(XpbdError::Numerical)
    }
}

#[cfg(test)]
#[test]
fn extreme_elliptical_mobility_and_coulomb_radius_satisfy_the_real_circular_bound() {
    let gradient = DVec2::new(3., 4.);
    for (m, gv, r) in [
        (1., 1., 1e-240),
        (1e180, 1e200, 1e17),
        (1e-180, 1e-90, 1e88),
    ] {
        let j = disk(2. * m, 0.7 * m, 1.4 * m, gv * gradient, r).unwrap();
        let unit = j / r;
        assert!((unit.x.hypot(unit.y) - 1.).abs() < 3e-14);
        let v = gradient
            + (m * (r / gv)) * DVec2::new(2. * unit.x + 0.7 * unit.y, 0.7 * unit.x + 1.4 * unit.y);
        assert!(unit.dot(v) < 0. && (unit.x * v.y - unit.y * v.x).abs() < 4e-13);
    }
}

struct ContactPoint<'a> {
    skin: Skin<'a>,
    point: DVec3,
    arm: DVec3,
    n: DVec3,
    distance: f64,
    gap: f64,
}
impl<'a> ContactPoint<'a> {
    fn new(
        ids: &'a [usize],
        c: Candidate,
        b: &PreparedBoundary,
        dt: Interval,
        end: bool,
        at: impl Fn(usize) -> DVec3,
    ) -> Result<Self, XpbdError> {
        let t = if end { 1. } else { 0. };
        let path = Trajectory::new(b, dt)?.ok_or(XpbdError::InvalidBoundary)?;
        let center = path.linear.center + t * path.linear.delta;
        let q = c.point(&at);
        let half = path.arm(t);
        let axis = half.normalize();
        let core = axis * ((q - center).dot(axis).clamp(-half.length(), half.length()));
        let radial = q - center - core;
        let distance = radial.length();
        let n = c.normal;
        let arm = core
            + (path.linear.gap - dt.radius)
                * radial.try_normalize().ok_or(XpbdError::UnresolvedSurface)?;
        let point = center + arm;
        let mut face = [usize::MAX; 3];
        face[..c.feature.vertices().len()].copy_from_slice(c.feature.vertices());
        let skin = Skin::new(ids, face, c.weights, at, point - q)?;
        Ok(Self {
            skin,
            point,
            arm,
            n,
            distance,
            gap: path.linear.gap,
        })
    }
    fn relative_velocity(&self, p: &[Particle], b: &PreparedBoundary) -> DVec3 {
        let omega = b.receiver.as_ref().map_or_else(
            || {
                b.prescribed_capsule()
                    .map_or(DVec3::ZERO, |c| c.axis * c.angular_speed)
            },
            |r| r.angular_velocity(),
        );
        let rigid = b.velocity
            + b.receiver.as_ref().map_or(DVec3::ZERO, |r| r.linear)
            + omega.cross(self.arm);
        self.skin.velocity(p) - rigid
    }
    fn solve(
        &self,
        p: &[Particle],
        b: &PreparedBoundary,
        old: DVec3,
        limit: f64,
    ) -> Result<DVec3, XpbdError> {
        let n = self.n;
        let u = n.any_orthonormal_vector();
        let v = n.cross(u);
        let rel = self.relative_velocity(p, b);
        let mut a = 0.;
        let mut cross = 0.;
        let mut c = 0.;
        for ((id, x), (_, y)) in self.skin.nodal(u).zip(self.skin.nodal(v)) {
            let w = p[id].inverse_mass;
            a += w * x.length_squared();
            cross += w * x.dot(y);
            c += w * y.length_squared();
        }
        if let Some(r) = &b.receiver {
            let ru = self.arm.cross(u);
            let rv = self.arm.cross(v);
            a += r.inverse_mass + ru.dot(r.inverse_inertia(ru));
            cross += ru.dot(r.inverse_inertia(rv));
            c += r.inverse_mass + rv.dot(r.inverse_inertia(rv));
        }
        if a == 0. && c == 0. && limit == 0. {
            return Ok(DVec3::ZERO);
        }
        let previous = DVec2::new(old.dot(u), old.dot(v));
        let g = DVec2::new(
            rel.dot(u) - a * previous.x - cross * previous.y,
            rel.dot(v) - cross * previous.x - c * previous.y,
        );
        let j = disk(a, cross, c, g, limit)?;
        Ok(u * j.x + v * j.y)
    }
}
impl MaterialFeature {
    fn patch_key(self) -> [usize; 3] {
        let mut key = [usize::MAX; 3];
        key[..self.vertices().len()].copy_from_slice(self.vertices());
        key
    }
}

impl CapsuleMaterial {
    pub(crate) fn stick(
        &mut self,
        x: &mut [DVec3],
        p: &mut [Particle],
        bs: &mut [PreparedBoundary],
        r: &mut [BoundaryImpulse],
        dt: Interval,
    ) -> Result<(), XpbdError> {
        let f = &mut self.friction;
        if f.mu == 0. {
            return Ok(());
        }
        for (&key, row) in &mut self.contacts {
            if row.normal_impulse == 0. {
                continue;
            }
            let b = &mut bs[key.boundary];
            let point = ContactPoint::new(
                &self.mesh.patches[&key.feature.patch_key()],
                row.witness,
                b,
                dt,
                false,
                |i| p[i].position.into(),
            )?;
            // A first arrival inside the interval changes frictional velocity
            // at the endpoint. Only an existing support sticks through it.
            if point.distance > point.gap * (1. + 1e-7) {
                continue;
            }
            let total = point.solve(p, b, row.tangent_impulse, f.mu * row.normal_impulse)?;
            let j = total - row.tangent_impulse;
            row.tangent_impulse = total;
            if j == DVec3::ZERO {
                continue;
            }
            for (i, impulse) in point.skin.nodal(j) {
                let dx = dt.h * p[i].inverse_mass * impulse;
                crate::correct(&mut x[i], &mut p[i], dx, dt.h);
            }
            if let Some(r) = &mut b.receiver {
                r.project_material_wrench(-j, -point.arm.cross(j), dt.h)?;
            }
            f.record(
                key,
                &point.skin,
                point.point - b.origin,
                j,
                &mut r[key.boundary],
            )?;
        }
        Ok(())
    }
    /// Solve unilateral normal complementarity together with exact local
    /// maximum dissipation. An off-axis tangent can change normal speed and
    /// either jaw can change the other's material velocity; resolve both before publication.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn finish_friction(
        &mut self,
        x: &[DVec3],
        p: &mut [Particle],
        bs: &mut [PreparedBoundary],
        r: &mut [BoundaryImpulse],
        dt: Interval,
        rows: &[(Key, Contact)],
        witnesses: &[Row],
        mut residual: Vec<f64>,
    ) -> Result<(), XpbdError> {
        let mesh = self.mesh.clone();
        let points = rows
            .iter()
            .map(|(k, c)| {
                ContactPoint::new(
                    &mesh.patches[&k.feature.patch_key()],
                    c.witness,
                    &bs[k.boundary],
                    dt,
                    true,
                    |i| x[i],
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let mut normals = rows
            .iter()
            .map(|(_, c)| c.normal_impulse)
            .collect::<Vec<_>>();
        let mut tangents = rows
            .iter()
            .map(|(_, c)| c.tangent_impulse)
            .collect::<Vec<_>>();
        let targets = rows
            .iter()
            .zip(witnesses)
            .zip(&residual)
            .map(|(((k, _), w), f)| w.normal_velocity(p, &bs[k.boundary]) - f)
            .collect::<Vec<_>>();
        for pass in 0..128 {
            let next = solve_normals(witnesses, p, bs, residual, &normals)?;
            let mut change = 0f64;
            let mut scale = 0f64;
            for (i, (&j, w)) in next.iter().zip(witnesses).enumerate() {
                let delta = j - normals[i];
                change = change.max(delta.abs());
                scale = scale.max(j);
                w.apply_normal(p, &mut bs[rows[i].0.boundary], delta * w.c.normal);
            }
            normals = next;
            for (i, point) in points.iter().enumerate() {
                let b = &mut bs[rows[i].0.boundary];
                let previous = tangents[i];
                let total = point.solve(p, b, previous, self.friction.mu * normals[i])?;
                let delta = total - previous;
                tangents[i] = total;
                point.skin.apply(delta, p);
                if let Some(r) = &mut b.receiver {
                    r.apply_velocity_impulse(point.arm, -delta);
                }
                change = change.max(delta.length());
                scale = scale.max(total.length());
            }
            let converged = change <= 2e-12 * scale + 2e-16;
            if converged
                || (pass == 127
                    && manifold::refine(
                        rows,
                        witnesses,
                        &points,
                        &targets,
                        &mut normals,
                        &mut tangents,
                        p,
                        bs,
                        self.friction.mu,
                        self.retry_manifold,
                    ))
            {
                for (i, &(key, row)) in rows.iter().enumerate() {
                    let b = &bs[key.boundary];
                    let delta = (normals[i] - row.normal_impulse) * row.witness.normal;
                    self.record(
                        key,
                        row.witness,
                        row.first_time,
                        delta,
                        row.witness.point(|i| x[i]) - b.origin,
                        &mut r[key.boundary],
                    )?;
                    let point = &points[i];
                    self.friction.record(
                        key,
                        &point.skin,
                        point.point - b.origin,
                        tangents[i] - row.tangent_impulse,
                        &mut r[key.boundary],
                    )?;
                    if normals[i] > 0. {
                        let load = self.friction.load(key);
                        load.normal_impulse_sum += normals[i];
                        load.tangent_impulse_sum += tangents[i].length();
                        load.last_time = dt.t1;
                        load.last_weights = row.witness.weights;
                        load.last_point = point.point.to_array();
                        load.last_normal = point.n.to_array();
                        let relative = point.relative_velocity(p, b);
                        let slip = relative - point.n * relative.dot(point.n);
                        load.last_slip_velocity = slip.to_array();
                        load.last_normal_impulse = normals[i];
                        load.last_tangent_impulse = tangents[i].to_array();
                        load.normal_weighted_slip += normals[i] * slip.length();
                        load.slip_distance +=
                            (dt.t1 - row.first_time.clamp(dt.t0, dt.t1)) * slip.length();
                        let sample = CapsuleContactSample {
                            boundary: key.boundary,
                            feature: key.feature,
                            region: key.part,
                            two_way: key.two_way,
                            interval: [dt.t0, dt.t1],
                            first_time: row.first_time.clamp(dt.t0, dt.t1),
                            weights: row.witness.weights,
                            point: point.point.to_array(),
                            normal: point.n.to_array(),
                            normal_impulse: normals[i],
                            tangent_impulse: tangents[i].to_array(),
                            slip_velocity: slip.to_array(),
                        };
                        if let Some(history) = &mut self.friction.samples {
                            if history.len() >= 131072 {
                                return Err(XpbdError::SurfaceBudget);
                            }
                            history.push(sample);
                        }
                    }
                }
                return Ok(());
            }
            residual = rows
                .iter()
                .zip(witnesses)
                .zip(&targets)
                .map(|(((k, _), w), target)| w.normal_velocity(p, &bs[k.boundary]) - target)
                .collect();
        }
        Err(if rows.iter().any(|(k, _)| k.two_way) {
            XpbdError::UnresolvedSweep
        } else {
            XpbdError::UnresolvedSurface
        })
    }
}
