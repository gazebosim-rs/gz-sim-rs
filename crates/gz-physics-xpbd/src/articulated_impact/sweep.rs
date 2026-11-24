// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exact first approaching sphere contact on the rounded segment or triangle
//! against a material in uniform gravitational free flight. Both endpoints may be clear.
mod circular;
mod recurrent;
pub use recurrent::MaterialSeparations;
mod trajectory;
use super::*;
use std::time::Duration;
use trajectory::Trajectory;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TranslatingSphere {
    pub sphere: ImpactSphere,
    pub velocity: [f64; 3],
}
/// Certified native sphere-center motion in world coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SphereMotion {
    Translate {
        velocity: [f64; 3],
    },
    Revolve {
        pivot: [f64; 3],
        axis: [f64; 3],
        angular_speed: f64,
    },
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SweptSphere {
    pub sphere: ImpactSphere,
    pub motion: SphereMotion,
}
impl From<TranslatingSphere> for SweptSphere {
    fn from(s: TranslatingSphere) -> Self {
        Self {
            sphere: s.sphere,
            motion: SphereMotion::Translate {
                velocity: s.velocity,
            },
        }
    }
}
impl SweptSphere {
    fn relative(
        self,
        h: f64,
        material_velocity: DVec3,
        acceleration: DVec3,
    ) -> Result<Trajectory, XpbdError> {
        let center = DVec3::from(self.sphere.center);
        let (native, orbit) = match self.motion {
            SphereMotion::Translate { velocity } => (DVec3::from(velocity), None),
            SphereMotion::Revolve {
                pivot,
                axis,
                angular_speed,
            } => {
                let (pivot, axis) = (DVec3::from(pivot), DVec3::from(axis));
                let angle = angular_speed * h;
                if !pivot.is_finite()
                    || !axis.is_finite()
                    || !angle.is_finite()
                    || (axis.length_squared() - 1.).abs() > 1e-12
                {
                    return Err(XpbdError::InvalidBoundary);
                }
                if angle.abs() > 4. * std::f64::consts::TAU {
                    return Err(XpbdError::ImpactBudget);
                }
                let axis = axis.normalize();
                let arm = center - pivot;
                let radial = arm - axis * arm.dot(axis);
                let orbit = (radial != DVec3::ZERO && angle != 0.).then_some(trajectory::Orbit {
                    radial,
                    transverse: axis.cross(radial),
                    angle,
                });
                (DVec3::ZERO, orbit)
            }
        };
        let path = Trajectory {
            center,
            delta: (native - material_velocity) * h,
            curve: -(0.5 * h * h) * acceleration,
            orbit,
        };
        if !center.is_finite()
            || !path.speed_bound().is_finite()
            || !path.acceleration_bound().is_finite()
        {
            return Err(XpbdError::InvalidBoundary);
        }
        Ok(path)
    }
}
#[derive(Clone, Debug, PartialEq)]
pub struct MaterialImpactTime {
    /// Nearest integrable instant on this interval's integer-nanosecond clock.
    head: Duration,
    /// Continuous geometric time in seconds after the interval start.
    seconds: f64,
    /// Maximum relative travel between the continuous and clocked impact.
    rounding_distance: f64,
    spheres: Vec<ImpactSphere>,
    nodes: Vec<(usize, [f64; 3])>,
    pub(crate) rows: Vec<(usize, MaterialFeature)>,
}
impl MaterialImpactTime {
    pub fn head(&self) -> Duration {
        self.head
    }
    pub fn seconds(&self) -> f64 {
        self.seconds
    }
    pub fn rounding_distance(&self) -> f64 {
        self.rounding_distance
    }

    pub fn features(&self) -> &[(usize, MaterialFeature)] {
        &self.rows
    }
}

fn chord(a: DVec3, b: DVec3, p: DVec3) -> ([f64; 3], DVec3) {
    let axis = b - a;
    let t = ((p - a).dot(axis) / axis.length_squared()).clamp(0., 1.);
    ([1. - t, t, 0.], a + t * axis - p)
}
enum Primitive {
    Segment([usize; 2]),
    Triangle([usize; 3]),
}
impl Primitive {
    fn vertices(&self) -> [usize; 3] {
        match self {
            Self::Segment([a, b]) => [*a, *b, usize::MAX],
            Self::Triangle(ids) => *ids,
        }
    }
    fn witness(&self, x: &[crate::Particle], p: DVec3) -> ([f64; 3], DVec3) {
        let at = |i: usize| DVec3::from(x[i].position);
        match *self {
            Self::Segment([a, b]) => chord(at(a), at(b), p),
            Self::Triangle(ids) => {
                let [a, b, c] = ids.map(at);
                let w = vertex_face(&[p, a, b, c]);
                ([-w.weights[1], -w.weights[2], -w.weights[3]], -w.delta)
            }
        }
    }
}
/// First *entering* root of |r+t v|² = radius². Rationalization avoids
/// subtracting nearly equal roots when the starting configuration is close.
fn entry(r: DVec3, v: DVec3, radius: f64) -> Option<f64> {
    let scale = r.abs().max_element().max(v.abs().max_element()).max(radius);
    let (r, v, g) = (r / scale, v / scale, radius / scale);
    let a = v.length_squared();
    let b = r.dot(v);
    let c = (r.length() - g) * (r.length() + g);
    if b >= 0. || a <= 0. || c < 0. {
        return None;
    }
    let disc = b.mul_add(b, -a * c);
    if disc <= 0. {
        return None;
    }
    Some(c / (-b + disc.sqrt()))
}
impl SoftBody {
    /// Certify the material's free approach from authored forces, current
    /// velocities, current contact profile and every structural strain.
    fn unloaded_motion(&self, dt: Duration) -> Result<DVec3, XpbdError> {
        if !self.supports_articulated_impact()
            || (self.config.gravity != [0.; 3]
                && self.particles.iter().any(|p| p.inverse_mass == 0.))
            || self.capsule_material.is_some()
            || self.pads.is_some()
            || self.surface.is_some()
        {
            return Err(XpbdError::NonlinearApproach);
        }
        let v = DVec3::from(self.particles[0].velocity);
        for p in &self.particles {
            let d = DVec3::from(p.velocity) - v;
            if d.length() > 256. * f64::EPSILON * (1. + v.length()) {
                return Err(XpbdError::NonlinearApproach);
            }
        }
        let at = |id: usize| DVec3::from(self.particles[id].position);
        for c in &self.constraints {
            let (a, b) = (at(c.a), at(c.b));
            let round = 256.
                * f64::EPSILON
                * (c.rest_length
                    + a.abs().max_element().max(b.abs().max_element())
                    + dt.as_secs_f64() * v.length());
            if (a.distance(b) - c.rest_length).abs() > round {
                return Err(XpbdError::NonlinearApproach);
            }
        }
        for b in &self.bending {
            let (angle, _) =
                crate::bending::geometry(b.vertices.map(at)).ok_or(XpbdError::NonlinearApproach)?;
            let difference = angle - b.rest_angle;
            if difference.sin().abs() > 1e-12 || difference.cos() < 0. {
                return Err(XpbdError::NonlinearApproach);
            }
        }
        Ok(v)
    }

    /// Integrate a certified gravity-only free flight exactly before the first
    /// event. This preserves common velocities without reconstructing them
    /// from subtracting two nearby projected positions on every public tick.
    pub fn advance_articulated_free_flight(&mut self, dt: Duration) -> Result<(), XpbdError> {
        self.unloaded_motion(dt)?;
        let h = dt.as_secs_f64();
        let acceleration = DVec3::from(self.config.gravity);
        for (p, next) in self.particles.iter().zip(&mut self.scratch) {
            let velocity = DVec3::from(p.velocity);
            let position = DVec3::from(p.position) + h * (velocity + (0.5 * h) * acceleration);
            let velocity = velocity + h * acceleration;
            if !position.is_finite() || !velocity.is_finite() {
                return Err(XpbdError::Numerical);
            }
            *next = *p;
            next.position = position.to_array();
            next.velocity = velocity.to_array();
        }
        std::mem::swap(&mut self.particles, &mut self.scratch);
        self.reactions.clear();
        self.sweep_refinements = 0;
        self.self_impulse = Default::default();
        Ok(())
    }

    /// Continue an unstrained material from a common, immutable initial cut.
    /// This evaluates the closed-form trajectory at absolute elapsed times.
    /// It prevents accumulated per-node position or velocity drift over many
    /// public ticks. Any intervening particle edit invalidates the reference.
    pub fn advance_articulated_flight_from(
        &mut self,
        initial: &[crate::Particle],
        start: Duration,
        end: Duration,
    ) -> Result<(), XpbdError> {
        let dt = end.checked_sub(start).ok_or(XpbdError::InvalidStep)?;
        self.unloaded_motion(dt)?;
        if initial.len() != self.particles.len() {
            return Err(XpbdError::NonlinearApproach);
        }
        let g = DVec3::from(self.config.gravity);
        let at = |p: crate::Particle, t: Duration| {
            if t.is_zero() {
                return p;
            }
            let h = t.as_secs_f64();
            let v = DVec3::from(p.velocity);
            crate::Particle {
                position: (DVec3::from(p.position) + h * (v + (0.5 * h) * g)).to_array(),
                velocity: (v + h * g).to_array(),
                inverse_mass: p.inverse_mass,
            }
        };
        for ((current, seed), next) in self.particles.iter().zip(initial).zip(&mut self.scratch) {
            if *current != at(*seed, start) {
                return Err(XpbdError::NonlinearApproach);
            }
            *next = at(*seed, end);
            if !DVec3::from(next.position).is_finite() || !DVec3::from(next.velocity).is_finite() {
                return Err(XpbdError::Numerical);
            }
        }
        std::mem::swap(&mut self.particles, &mut self.scratch);
        self.reactions.clear();
        self.sweep_refinements = 0;
        self.self_impulse = Default::default();
        Ok(())
    }

    /// Constant-velocity specialization for zero-gravity material sources.
    pub fn first_translating_sphere_impact(
        &self,
        spheres: &[TranslatingSphere],
        dt: Duration,
    ) -> Result<Option<MaterialImpactTime>, XpbdError> {
        if self.config.gravity != [0.; 3] {
            return Err(XpbdError::NonlinearApproach);
        }
        self.first_ballistic_sphere_impact(spheres, dt)
    }

    /// First approaching native sphere contact against a freely translating
    /// cable or cloth under constant gravity, including interior arc extrema.
    pub fn first_ballistic_sphere_impact(
        &self,
        spheres: &[TranslatingSphere],
        dt: Duration,
    ) -> Result<Option<MaterialImpactTime>, XpbdError> {
        self.first_sphere_impact(spheres, dt, false)
    }
    /// Earliest approach of entire rounded material primitives to native
    /// spherical pads translating or revolving about fixed world axes.
    pub fn first_swept_sphere_impact(
        &self,
        spheres: &[SweptSphere],
        dt: Duration,
    ) -> Result<Option<MaterialImpactTime>, XpbdError> {
        self.first_sphere_impact(spheres, dt, false)
    }
    /// An entering event whose integral clock cut is at or before the root.
    /// Used when the next step must start with a nonpenetrating geometry.
    pub fn first_swept_approach_cut(
        &self,
        spheres: &[SweptSphere],
        dt: Duration,
    ) -> Result<Option<MaterialImpactTime>, XpbdError> {
        self.first_sphere_impact(spheres, dt, true)
    }
    fn first_sphere_impact<S: Copy + Into<SweptSphere>>(
        &self,
        spheres: &[S],
        dt: Duration,
        outside: bool,
    ) -> Result<Option<MaterialImpactTime>, XpbdError> {
        let ns = u64::try_from(dt.as_nanos()).map_err(|_| XpbdError::InvalidStep)?;
        // Preserve one-nanosecond resolution when quantizing the continuous root.
        if ns == 0 || ns > (1u64 << 52) || spheres.is_empty() || spheres.len() > 16 {
            return Err(XpbdError::InvalidStep);
        }
        let soft = self.unloaded_motion(dt)?;
        let acceleration = DVec3::from(self.config.gravity);
        let h = dt.as_secs_f64();
        let x = &self.particles;
        let elements = self.primitive_count()?;
        if elements > MAX_QUERIES / spheres.len() {
            return Err(XpbdError::ImpactBudget);
        }
        let mut cloud_lo = DVec3::splat(f64::INFINITY);
        let mut cloud_hi = DVec3::splat(f64::NEG_INFINITY);
        for v in x {
            let p = DVec3::from(v.position);
            cloud_lo = cloud_lo.min(p);
            cloud_hi = cloud_hi.max(p);
        }
        let mut first = f64::INFINITY;
        let mut time_round = 0.0f64;
        let mut rows = BTreeMap::<(usize, MaterialFeature), f64>::new();
        let mut speed = 0.0_f64;
        let mut curve_budget = 4 * MAX_QUERIES;
        for (s, moving) in spheres.iter().copied().map(Into::into).enumerate() {
            let center = DVec3::from(moving.sphere.center);
            let gap = moving.sphere.radius + self.config.particle_radius;
            let path = moving.relative(h, soft, acceleration)?;
            let (delta, curve) = (path.delta, path.curve);
            if !center.is_finite()
                || !delta.is_finite()
                || !curve.is_finite()
                || !gap.is_finite()
                || !moving.sphere.radius.is_finite()
                || moving.sphere.radius <= 0.
            {
                return Err(XpbdError::InvalidBoundary);
            }
            speed = speed.max(path.speed_bound() / h);
            let at = |i: usize| DVec3::from(x[i].position);
            let spatial_round = 256.
                * f64::EPSILON
                * (gap
                    + center.abs().max_element()
                    + cloud_lo
                        .abs()
                        .max_element()
                        .max(cloud_hi.abs().max_element())
                    + path.speed_bound()
                    + curve.length());
            let radius = DVec3::splat(gap + spatial_round);
            let (lo, hi) = path.bounds();
            let (min, max) = (lo - radius, hi + radius);
            if min.cmpgt(cloud_hi).any() || max.cmplt(cloud_lo).any() {
                continue;
            }
            for i in 0..elements {
                let element = self.primitive(i);
                let ids = element.vertices();
                let a = at(ids[0]);
                let b = at(ids[1]);
                let c = if ids[2] == usize::MAX { a } else { at(ids[2]) };
                if min.cmpgt(a.max(b).max(c)).any() || max.cmplt(a.min(b).min(c)).any() {
                    continue;
                }
                let mut test = |t: f64| -> Result<(), XpbdError> {
                    if !t.is_finite()
                        || !(-16. * f64::EPSILON..=1. + 16. * f64::EPSILON).contains(&t)
                    {
                        return Ok(());
                    }
                    let t = t.clamp(0., 1.);
                    let (weights, distance) = element.witness(x, path.at(t));
                    let tangent = path.tangent(t);
                    let d = distance.length();
                    if !d.is_finite() {
                        return Err(XpbdError::Numerical);
                    }
                    if t == 0. && d < gap - spatial_round {
                        return Err(XpbdError::PenetratingImpact);
                    }
                    if (d - gap).abs() <= spatial_round
                        && distance.dot(tangent) > 128. * f64::EPSILON * d * path.speed_bound()
                    {
                        let time = t * h;
                        let uncertainty = spatial_round * d * h / distance.dot(tangent);
                        if time < first {
                            time_round = uncertainty.min(0.25e-9);
                        }

                        let (key, _) = MaterialFeature::from_weights(ids, weights);
                        first = first.min(time);
                        rows.entry((s, key))
                            .and_modify(|v| *v = v.min(time))
                            .or_insert(time);
                    }
                    Ok(())
                };
                test(0.)?;
                if path.orbit.is_some() {
                    if let Some(t) =
                        circular::first(&path, &element, x, gap, spatial_round, &mut curve_budget)?
                    {
                        test(t)?;
                    }
                    continue;
                }
                let edges = if ids[2] == usize::MAX { 1 } else { 3 };
                for &(p, q) in &[(a, b), (b, c), (c, a)][..edges] {
                    let axis = (q - p).try_normalize().ok_or(XpbdError::UnresolvedImpact)?;
                    let offset = center - p;
                    for t in trajectory::radial(
                        offset - axis * offset.dot(axis),
                        delta - axis * delta.dot(axis),
                        curve - axis * curve.dot(axis),
                        gap,
                    )
                    .iter()
                    {
                        test(t)?;
                    }
                    for t in trajectory::radial(offset, delta, curve, gap).iter() {
                        test(t)?;
                    }
                }
                if edges == 1 {
                    for t in trajectory::radial(center - b, delta, curve, gap).iter() {
                        test(t)?;
                    }
                } else {
                    let normal = (b - a)
                        .cross(c - a)
                        .try_normalize()
                        .ok_or(XpbdError::UnresolvedImpact)?;
                    let distance = normal.dot(center - a);
                    let rate = normal.dot(delta);
                    let bend = normal.dot(curve);
                    for target in [-gap, gap] {
                        for t in trajectory::plane(distance, rate, bend, target).iter() {
                            test(t)?;
                        }
                    }
                }
            }
        }
        if first == f64::INFINITY {
            return Ok(None);
        }
        let continuous = first * 1e9;
        let mut head = (if outside {
            continuous.floor()
        } else {
            continuous.round()
        })
        .clamp(0., ns as f64) as u64;
        let sub = u64::from(self.config.substeps);
        if head < sub {
            head = 0;
        } else if ns - head < sub && head != ns {
            head = if outside {
                let t = ns.saturating_sub(sub);
                if t < sub { 0 } else { t }
            } else {
                ns
            };
        }
        let head = Duration::from_nanos(head);
        // Only numerically simultaneous witnesses share the instantaneous batch;
        // a distinct later feature must not borrow the clock uncertainty as skin.
        let tie = time_round + 256. * f64::EPSILON * h.max(first);
        let rows = rows
            .into_iter()
            .filter(|(_, t)| *t <= first + tie)
            .map(|(key, _)| key)
            .collect::<Vec<_>>();
        if rows.len() > MAX_ROWS {
            return Err(XpbdError::ImpactBudget);
        }
        let check_nodes = rows
            .iter()
            .flat_map(|(_, f)| f.vertices().iter().copied())
            .collect::<std::collections::BTreeSet<_>>();
        let nodes = check_nodes
            .into_iter()
            .map(|id| {
                (
                    id,
                    (DVec3::from(x[id].position)
                        + head.as_secs_f64() * (soft + (0.5 * head.as_secs_f64()) * acceleration))
                        .to_array(),
                )
            })
            .collect();
        let spheres = spheres
            .iter()
            .copied()
            .map(Into::into)
            .map(|s| {
                let native = s.relative(head.as_secs_f64(), DVec3::ZERO, DVec3::ZERO)?;
                Ok(ImpactSphere {
                    center: native.at(1.).to_array(),
                    radius: s.sphere.radius,
                })
            })
            .collect::<Result<Vec<_>, XpbdError>>()?;
        Ok(Some(MaterialImpactTime {
            nodes,
            spheres,
            head,
            seconds: first,
            rounding_distance: (head.as_secs_f64() - first).abs() * speed,
            rows,
        }))
    }

    /// Reconstruct the current geometric witnesses at the detected time. Only
    /// the certified colliding features can use its measured quantization gap.
    pub fn sphere_impact_at_detected_barrier<'a>(
        &'a mut self,
        spheres: &[ImpactSphere],
        time: &MaterialImpactTime,
    ) -> Result<MaterialImpact<'a>, XpbdError> {
        let same = |a: [f64; 3], b: [f64; 3]| {
            let (a, b) = (DVec3::from(a), DVec3::from(b));
            a.distance(b)
                <= 512.
                    * f64::EPSILON
                    * (a.abs().max_element().max(b.abs().max_element())
                        + self.config.particle_radius)
        };
        if spheres.len() != time.spheres.len()
            || spheres
                .iter()
                .zip(&time.spheres)
                .any(|(a, b)| a.radius.to_bits() != b.radius.to_bits() || !same(a.center, b.center))
            || time
                .nodes
                .iter()
                .any(|&(i, p)| self.particles.get(i).is_none_or(|n| !same(n.position, p)))
        {
            return Err(XpbdError::UnresolvedImpact);
        }
        self.sphere_impact_witnesses(spheres, Some(time), 0.)
    }
}
