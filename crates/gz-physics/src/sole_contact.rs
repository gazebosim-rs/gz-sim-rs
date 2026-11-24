// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compliant contact between a rigid rectangular sole and the top of a finite deck.
//! The coating acts along the deck normal; integrals use undeformed sole area.
mod damped_tier;
mod flat_tier;
use crate::{DampedAreaFoundation, ElasticAreaFoundation};
use glam::{DQuat, DVec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum SoleError {
    #[error("invalid sole mass, inertia, shape, surface material, state or time grid")]
    InvalidInput,
    #[error("contact is outside the lower-face coating model")]
    OutsideCoating,
    #[error("nonfinite sole contact integral or motion")]
    Nonfinite,
    #[error("requested adaptive accuracy exceeds the available refinement budget")]
    AccuracyBudget,
}

#[derive(Debug, Clone, Copy)]
pub struct SoleBody {
    pub half_extents: DVec3,
    /// Body-frame vector from the center of mass to the center of the sole box.
    pub foot_offset: DVec3,
    pub mass: f64,
    /// Principal moments about the center of mass, in body axes.
    pub inertia: DVec3,
}

#[derive(Debug, Clone, Copy)]
pub struct SoleDeck {
    /// Center of the deck box at the start of a step.
    pub center: DVec3,
    pub orientation: DQuat,
    pub half_extents: DVec3,
    /// Constant world translation velocity.
    pub velocity: DVec3,
}

#[derive(Debug, Clone, Copy)]
pub struct SoleState {
    pub center: DVec3,
    pub orientation: DQuat,
    pub velocity: DVec3,
    /// Angular velocity in world coordinates (rad/s).
    pub spin: DVec3,
}

#[derive(Debug, Clone, Copy)]
pub struct SoleMaterial {
    elastic: ElasticAreaFoundation,
    damped: DampedAreaFoundation,
    alpha: f64,
    friction: f64,
    slip_speed: f64,
}
impl SoleMaterial {
    /// Bound the norm of the combined normal and tangential force from this material.
    pub fn force_norm_bound(self, normal_force: f64) -> f64 {
        self.friction.hypot(1.) * normal_force
    }

    /// Regularized Coulomb force per newton of normal load.
    fn shear_direction(self, velocity: DVec3) -> DVec3 {
        if self.friction == 0. {
            return DVec3::ZERO;
        }
        let slip = DVec3::new(velocity.x, velocity.y, 0.);
        let squared = slip.length_squared() + self.slip_speed * self.slip_speed;
        let scale = if squared.is_normal() {
            squared.sqrt()
        } else {
            slip.x.hypot(slip.y).hypot(self.slip_speed)
        };
        -self.friction * (slip / scale)
    }

    /// Layer stiffnesses in N/m³, damping in s/m, slip regularization in m/s.
    pub fn new(
        foot_stiffness: f64,
        deck_stiffness: f64,
        damping: f64,
        friction: f64,
        slip_speed: f64,
    ) -> Result<Self, SoleError> {
        let elastic = ElasticAreaFoundation::new(foot_stiffness, deck_stiffness)
            .ok_or(SoleError::InvalidInput)?;
        let damped = elastic
            .with_damping(damping)
            .ok_or(SoleError::InvalidInput)?;
        if !friction.is_finite() || friction < 0. || !slip_speed.is_finite() || slip_speed <= 0. {
            return Err(SoleError::InvalidInput);
        }
        Ok(Self {
            elastic,
            damped,
            alpha: damping,
            friction,
            slip_speed,
        })
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct SoleContact {
    pub force: DVec3,
    /// Contact torque about the center of mass.
    pub torque: DVec3,
    pub normal_force: f64,
    /// Weighted center of normal pressure on the undeformed contact face.
    pub center_of_pressure: Option<DVec3>,
    /// Load-carrying area on the undeformed sole.
    pub area: f64,
    pub strain_energy: f64,
    pub heat_power: f64,
    pub max_indentation: f64,
    /// Signed frictional work rate in the translating support frame.
    pub sliding_power: f64,
    /// Elastic energy carried across the deck rim per second (positive into contact).
    pub boundary_power: f64,
    polygon: Polygon,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct SoleStep {
    pub contact: SoleContact,
    pub impulse: DVec3,
    pub angular_impulse: DVec3,
    pub normal_impulse: f64,
    pub dissipated_energy: f64,
    pub sliding_work: f64,
    pub boundary_work: f64,
    pub peak_normal_force: f64,
    /// Substeps with three unloaded samples whose swept sole cannot be
    /// separated from the deck. Adaptive stepping resolves these intervals.
    pub possible_missed_contacts: u32,
}

impl SoleStep {
    pub(crate) fn unresolved(&self) -> bool {
        // A force observed only at a boundary or a first-stage sample still
        // needs a nonzero integration weight somewhere in the accepted tick.
        self.possible_missed_contacts > 0
            || (self.peak_normal_force > 0. && self.normal_impulse == 0.)
    }
}

// Each half-plane adds at most one vertex to a convex polygon. Four sole corners,
// four deck sides, the penetration plane and the pressure clamp need at most ten.
#[derive(Clone, Copy, Debug, Default)]
struct Polygon {
    vertices: [DVec3; 10],
    len: usize,
}
impl Polygon {
    fn push(&mut self, v: DVec3) -> Result<(), SoleError> {
        if !v.is_finite() || self.len == self.vertices.len() {
            return Err(SoleError::Nonfinite);
        }
        self.vertices[self.len] = v;
        self.len += 1;
        Ok(())
    }
    fn points(&self) -> &[DVec3] {
        &self.vertices[..self.len]
    }
    fn clip(self, inside: impl Fn(DVec3) -> f64) -> Result<Self, SoleError> {
        let Some(&initial) = self.points().last() else {
            return Ok(self);
        };
        let mut last = initial;
        let mut before = inside(last);
        let mut out = Self::default();
        for &p in self.points() {
            let after = inside(p);
            if !after.is_finite() || !before.is_finite() {
                return Err(SoleError::Nonfinite);
            }
            if (before < 0. && after > 0.) || (before > 0. && after < 0.) {
                let delta = before - after;
                let t = if delta.is_finite() {
                    before / delta
                } else {
                    (0.5 * before) / (0.5 * before - 0.5 * after)
                };
                out.push(if (p - last).is_finite() {
                    last.lerp(p, t)
                } else {
                    (1. - t) * last + t * p
                })?;
            }
            if after >= 0. {
                out.push(p)?;
            }
            last = p;
            before = after;
        }
        Ok(out)
    }
    fn triangles(&self) -> impl Iterator<Item = ([DVec3; 3], f64)> + '_ {
        (2..self.len).filter_map(|i| {
            let [a, b, c] = [self.vertices[0], self.vertices[i - 1], self.vertices[i]];
            let area = (b - a).cross(c - a).length() * 0.5;
            (area != 0.).then_some(([a, b, c], area))
        })
    }
}

fn quadrature(
    t: [DVec3; 3],
    area: f64,
    depth: u8,
    visit: &mut impl FnMut(DVec3, f64) -> Result<(), SoleError>,
) -> Result<(), SoleError> {
    let [a, b, c] = t;
    if depth > 0 {
        let (ab, bc, ca) = ((a + b) * 0.5, (b + c) * 0.5, (c + a) * 0.5);
        for child in [[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]] {
            quadrature(child, 0.25 * area, depth - 1, visit)?;
        }
    } else {
        let mut site = |x: f64, y: f64, w: f64| visit(a + x * (b - a) + y * (c - a), w * area);
        site(1. / 3., 1. / 3., 0.225)?;
        for (u, w) in [
            (0.470142064105115, 0.132394152788506),
            (0.101286507323456, 0.125939180544827),
        ] {
            let v = 1. - 2. * u;
            site(u, u, w)?;
            site(u, v, w)?;
            site(v, u, w)?;
        }
    }
    Ok(())
}

impl SoleState {
    fn finite(self) -> bool {
        self.center.is_finite()
            && self.orientation.is_finite()
            && (self.orientation.length_squared() - 1.).abs() < 1e-8
            && self.velocity.is_finite()
            && self.spin.is_finite()
    }
    fn in_deck(self, d: SoleDeck) -> Self {
        let r = d.orientation.conjugate();
        Self {
            center: r * (self.center - d.center),
            orientation: (r * self.orientation).normalize(),
            velocity: r * (self.velocity - d.velocity),
            spin: r * self.spin,
        }
    }
    fn world(self, d: SoleDeck, end: DVec3) -> Self {
        Self {
            center: end + d.orientation * self.center,
            orientation: (d.orientation * self.orientation).normalize(),
            velocity: d.velocity + d.orientation * self.velocity,
            spin: d.orientation * self.spin,
        }
    }
}
impl SoleContact {
    /// World vertices of the load-carrying patch in perimeter order.
    pub fn polygon(&self) -> &[DVec3] {
        self.polygon.points()
    }
    fn world(mut self, d: SoleDeck, end: DVec3) -> Self {
        for v in &mut self.polygon.vertices[..self.polygon.len] {
            *v = end + d.orientation * *v;
        }
        self.force = d.orientation * self.force;
        self.torque = d.orientation * self.torque;
        self.center_of_pressure = self.center_of_pressure.map(|p| end + d.orientation * p);
        self
    }
    fn finite(&self) -> bool {
        self.polygon.points().iter().all(|v| v.is_finite())
            && self.force.is_finite()
            && self.torque.is_finite()
            && self.center_of_pressure.is_none_or(|p| p.is_finite())
            && [
                self.normal_force,
                self.area,
                self.strain_energy,
                self.heat_power,
                self.max_indentation,
                self.sliding_power,
                self.boundary_power,
            ]
            .into_iter()
            .all(f64::is_finite)
    }
}

impl SoleBody {
    fn valid(&self, state: SoleState, deck: SoleDeck, depth: u8) -> bool {
        let positive = |v: DVec3| v.is_finite() && v.min_element() > 0.;
        positive(self.half_extents)
            && positive(self.inertia)
            && self.mass.is_finite()
            && self.mass > 0.
            && self.foot_offset.is_finite()
            && positive(deck.half_extents)
            && deck.center.is_finite()
            && deck.velocity.is_finite()
            && deck.orientation.is_finite()
            && (deck.orientation.length_squared() - 1.).abs() < 1e-8
            && state.finite()
            && depth <= 5
    }

    /// Instantaneous reaction and stored energy. Quadrature depth (0–5) subdivides
    /// each triangle into four children per level before a positive degree-five rule.
    pub fn contact(
        &self,
        state: &SoleState,
        deck: &SoleDeck,
        material: &SoleMaterial,
        quadrature_depth: u8,
    ) -> Result<SoleContact, SoleError> {
        if !self.valid(*state, *deck, quadrature_depth) {
            return Err(SoleError::InvalidInput);
        }
        let p = self
            .patch(
                state.in_deck(*deck),
                deck.half_extents,
                *material,
                quadrature_depth,
            )?
            .world(*deck, deck.center);
        p.finite().then_some(p).ok_or(SoleError::Nonfinite)
    }

    fn corners(&self, s: SoleState) -> [DVec3; 4] {
        [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)].map(|(x, y)| {
            s.center
                + s.orientation * (self.foot_offset + self.half_extents * DVec3::new(x, y, -1.))
        })
    }

    fn patch(
        &self,
        s: SoleState,
        half: DVec3,
        m: SoleMaterial,
        depth: u8,
    ) -> Result<SoleContact, SoleError> {
        if !s.finite() {
            return Err(SoleError::Nonfinite);
        }
        let corners = self.corners(s);
        if corners.iter().any(|p| !p.is_finite()) {
            return Err(SoleError::Nonfinite);
        }
        let mut p = SoleContact::default();
        if corners.iter().all(|v| v.z >= half.z) {
            return Ok(p);
        }
        let mut polygon = Polygon::default();
        for corner in corners {
            polygon.push(corner)?;
        }
        for axis in 0..2 {
            polygon = polygon
                .clip(|v| v[axis] + half[axis])?
                .clip(|v| half[axis] - v[axis])?;
        }
        polygon = polygon.clip(|v| half.z - v.z)?;
        if polygon.len < 3 {
            return Ok(p);
        }
        let normal = s.orientation * DVec3::Z;
        if normal.z <= 0. {
            return Err(SoleError::OutsideCoating);
        }
        for &v in polygon.points() {
            p.max_indentation = p.max_indentation.max(half.z - v.z);
        }
        if p.max_indentation > self.half_extents.z * normal.z {
            return Err(SoleError::OutsideCoating);
        }
        let velocity = |v: DVec3| s.velocity + s.spin.cross(v - s.center);
        let mut elastic_force = 0.;
        let mut elastic_moment = DVec3::ZERO;
        for (tri, area) in polygon.triangles() {
            let overlap = tri.map(|v| (half.z - v.z).max(0.));
            let integral = m
                .elastic
                .integrate_triangle(area, overlap)
                .ok_or(SoleError::Nonfinite)?;
            p.strain_energy += integral.energy_j;
            if s.spin == DVec3::ZERO {
                p.area += area;
                elastic_force += integral.normal_force_n;
                for (v, f) in tri.into_iter().zip(integral.nodal_forces_n) {
                    elastic_moment += (v - s.center) * f;
                }
            }
        }
        let rim_tolerance = [0, 1].map(|axis| {
            16. * f64::EPSILON
                * half[axis].max(corners.iter().map(|v| v[axis].abs()).fold(0., f64::max))
        });
        for (&a, &b) in polygon
            .points()
            .iter()
            .zip(polygon.points().iter().cycle().skip(1))
        {
            for axis in 0..2 {
                for sign in [-1., 1.] {
                    let side = sign * half[axis];
                    let tol = rim_tolerance[axis];
                    if (a[axis] - side).abs() > tol || (b[axis] - side).abs() > tol {
                        continue;
                    }
                    let grad = normal
                        .cross(if axis == 0 { DVec3::X } else { DVec3::Y })
                        .length();
                    for u in [0.5 - 0.5 / 3_f64.sqrt(), 0.5 + 0.5 / 3_f64.sqrt()] {
                        let pt = a.lerp(b, u);
                        let energy = m
                            .elastic
                            .sample(half.z - pt.z)
                            .ok_or(SoleError::Nonfinite)?
                            .energy_j_m2;
                        p.boundary_power +=
                            0.5 * a.distance(b) * energy * (-sign * velocity(pt)[axis]) / grad;
                    }
                }
            }
        }
        // With uniform surface velocity the rate factor and sliding direction
        // are constant. Exact triangle nodal loads recover the entire wrench,
        // including the first pressure moment of an inclined, clipped sole.
        if s.spin == DVec3::ZERO {
            let rate = -s.velocity.z;
            let factor = m.alpha.mul_add(rate, 1.).max(0.);
            p.normal_force = elastic_force * factor;
            p.heat_power = elastic_force * (m.alpha * rate).max(-1.) * rate;
            if p.normal_force > 0. {
                let arm = elastic_moment / elastic_force;
                let shear = p.normal_force * m.shear_direction(s.velocity);
                p.force = shear + DVec3::Z * p.normal_force;
                p.torque = arm.cross(p.force);
                p.center_of_pressure = Some(s.center + arm);
                p.sliding_power = shear.dot(s.velocity);
                p.polygon = polygon;
            } else {
                p.area = 0.;
            }
            return p.finite().then_some(p).ok_or(SoleError::Nonfinite);
        }
        let factor = |v: DVec3| {
            if m.alpha >= 1. {
                m.alpha.recip() - velocity(v).z
            } else {
                m.alpha.mul_add(-velocity(v).z, 1.)
            }
        };
        let (active, inactive) = if polygon.points().iter().all(|&v| factor(v) >= 0.) {
            (polygon, Polygon::default())
        } else {
            (polygon.clip(factor)?, polygon.clip(|v| -factor(v))?)
        };
        let mut moment = DVec3::ZERO;
        for (face, loaded) in [(active, true), (inactive, false)] {
            for (tri, area) in face.triangles() {
                if loaded {
                    p.area += area;
                }
                quadrature(
                    tri,
                    area,
                    if loaded && m.friction > 0. { depth } else { 0 },
                    &mut |pt, da| {
                        let v = velocity(pt);
                        let sample = m
                            .damped
                            .sample(half.z - pt.z, -v.z)
                            .ok_or(SoleError::Nonfinite)?;
                        p.heat_power += da * sample.dissipation_w_m2;
                        if loaded {
                            let load = da * sample.pressure_pa;
                            let shear = load * m.shear_direction(v);
                            let force = shear + load * DVec3::Z;
                            let arm = pt - s.center;
                            moment += arm * load;
                            p.force += force;
                            p.torque += arm.cross(force);
                            p.sliding_power += shear.dot(v);
                        }
                        Ok(())
                    },
                )?;
            }
        }
        p.normal_force = p.force.z;
        if p.normal_force > 0. {
            p.polygon = active;
            p.center_of_pressure = Some(s.center + moment / p.normal_force);
        } else {
            p.area = 0.;
        }
        p.finite().then_some(p).ok_or(SoleError::Nonfinite)
    }

    pub(super) fn advance(
        &self,
        s: SoleState,
        stage: SoleState,
        p: SoleContact,
        g: DVec3,
        applied_torque: DVec3,
        h: f64,
    ) -> SoleState {
        let torque = p.torque + applied_torque;
        let orientation = (DQuat::from_scaled_axis(h * stage.spin) * s.orientation).normalize();
        let spin = if self.inertia.x == self.inertia.y && self.inertia.y == self.inertia.z {
            s.spin + h * torque / self.inertia.x
        } else {
            let momentum =
                s.orientation * (self.inertia * (s.orientation.conjugate() * s.spin)) + h * torque;
            orientation * ((orientation.conjugate() * momentum) / self.inertia)
        };
        SoleState {
            center: s.center + h * stage.velocity,
            orientation,
            velocity: s.velocity + h * (g + p.force / self.mass),
            spin,
        }
    }

    /// A separating plane fixed in deck coordinates stays valid when the
    /// worst possible rigid sweep plus center translation cannot close its gap.
    fn separated(&self, s: SoleState, half: DVec3, g: DVec3, dt: f64, torque: DVec3) -> bool {
        let spin_bound = if self.inertia.x == self.inertia.y && self.inertia.y == self.inertia.z {
            s.spin.length() + dt * torque.length() / self.inertia.x
        } else {
            let momentum = (self.inertia * (s.orientation.conjugate() * s.spin)).length();
            (momentum + dt * torque.length()) / self.inertia.min_element()
        };
        let inertia_min = self.inertia.min_element();
        let angular_accel = (torque.length()
            + spin_bound * spin_bound * (self.inertia.max_element() - inertia_min))
            / inertia_min;
        self.separated_sweep(
            s,
            half,
            g,
            dt,
            spin_bound * dt,
            spin_bound * spin_bound + angular_accel,
            DVec3::ZERO,
        )
    }

    /// Sweep enclosure of one coupled midpoint trial. Translation follows the
    /// first-stage acceleration; the rotation exponent moves between the two
    /// stage spins and ends at dt times the midpoint spin.
    pub(super) fn separated_midpoint(
        &self,
        s: SoleState,
        mid: SoleState,
        deck: SoleDeck,
        g: DVec3,
        dt: f64,
    ) -> bool {
        self.separated_rotating_midpoint(s, mid, deck, DVec3::ZERO, g, dt)
    }

    pub(super) fn separated_rotating_midpoint(
        &self,
        s: SoleState,
        mid: SoleState,
        deck: SoleDeck,
        deck_spin: DVec3,
        g: DVec3,
        dt: f64,
    ) -> bool {
        let spin = s.spin.length().max(mid.spin.length());
        self.separated_sweep(
            s.in_deck(deck),
            deck.half_extents,
            deck.orientation.conjugate() * g,
            dt,
            spin * dt,
            spin * spin + 2. * s.spin.distance(mid.spin) / dt,
            deck.orientation.conjugate() * deck_spin,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn separated_sweep(
        &self,
        s: SoleState,
        half: DVec3,
        g: DVec3,
        dt: f64,
        rotation_bound: f64,
        rotation_curvature: f64,
        deck_spin: DVec3,
    ) -> bool {
        let corners = self.corners(s);
        if corners.iter().any(|p| !p.is_finite()) {
            return false;
        }
        let radius = [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)]
            .into_iter()
            .map(|(x, y)| (self.foot_offset + self.half_extents * DVec3::new(x, y, -1.)).length())
            .fold(0., f64::max);
        let sweep = radius * rotation_bound.min(2.);
        // Bound the extra change when viewing all sole points in a turning
        // deck frame. Projection along its rotation axis is unaffected.
        let deck_sweep = if deck_spin == DVec3::ZERO {
            0.
        } else {
            s.center.length() + radius + dt * s.velocity.length() + 0.5 * dt * dt * g.length()
        };
        let lowest_displacement = |v: f64, a: f64| {
            let t = if a > 0. { (-v / a).clamp(0., dt) } else { dt };
            (t * (v + 0.5 * t * a)).min(0.)
        };
        let clear = |axis: DVec3, exact_axis: bool| {
            let rotating_sweep = if deck_sweep == 0. {
                0.
            } else {
                deck_sweep * (dt * deck_spin.cross(axis).length()).min(2.)
            };

            let gap = corners
                .iter()
                .map(|v| v.dot(axis))
                .fold(f64::INFINITY, f64::min)
                - half.dot(axis.abs());
            let velocity = s.velocity.dot(axis);
            let acceleration = g.dot(axis);
            // For axis-aligned stationary/retreating faces allow exact touching:
            // a sole tangent to the deck with zero normal motion carries no pressure.
            if exact_axis
                && gap >= 0.
                && rotating_sweep == 0.
                && rotation_bound == 0.
                && velocity >= 0.
                && acceleration >= 0.
            {
                return true;
            }
            let translation = lowest_displacement(velocity, acceleration);
            let margin = 64.
                * f64::EPSILON
                * (s.center.abs().max_element()
                    + radius
                    + half.max_element()
                    + velocity.abs() * dt
                    + 0.5 * acceleration.abs() * dt * dt);
            if gap + translation - sweep - rotating_sweep > margin {
                return true;
            }
            // A magnitude-only sweep can never certify a just-departed edge
            // if it assumes the rim or the rotating corner could move backwards.
            // Retain each vertex's signed first derivative and bound only curvature.
            let turn = deck_spin.cross(axis);
            let material_speed = s.velocity.length() + s.spin.length() * radius;
            let reach =
                s.center.length() + radius + dt * material_speed + 0.5 * dt * dt * g.length();
            let turning_curvature = turn.length()
                * (deck_spin.length() * reach + 2. * material_speed + 3. * dt * g.length());
            let lower_acceleration = acceleration - radius * rotation_curvature - turning_curvature;
            corners.iter().all(|&v| {
                let gap = v.dot(axis) - half.dot(axis.abs());
                let speed = (s.velocity + s.spin.cross(v - s.center)).dot(axis) + turn.dot(v);
                gap + lowest_displacement(speed, lower_acceleration) > margin
            })
        };
        let fixed = [DVec3::X, DVec3::Y, DVec3::Z];
        if [DVec3::Z, DVec3::X, -DVec3::X, DVec3::Y, -DVec3::Y]
            .into_iter()
            .any(|axis| clear(axis, true))
        {
            return true;
        }
        // A panel can clear an inclined sole where it meets a rim while
        // remote, unsupported corners keep descending. Follow the *sole*
        // plane in time and bound the gaps of the four panel-top corners.
        let normal = s.orientation * DVec3::Z;
        if normal.z > (rotation_bound + deck_spin.length() * dt).min(2.) {
            let normal_derivative = s.spin.cross(normal);
            let plane_offset = self.foot_offset.z - self.half_extents.z;
            let clear_above = [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)]
                .into_iter()
                .all(|(x, y)| {
                    let q = half * DVec3::new(x, y, 1.);
                    let arm = s.center - q;
                    let velocity = s.velocity - deck_spin.cross(q);
                    let gap = arm.dot(normal) + plane_offset;
                    let rate = velocity.dot(normal) + arm.dot(normal_derivative);
                    let reach = arm.length() + dt * velocity.length() + 0.5 * dt * dt * g.length();
                    let lower = g.dot(normal) + 2. * velocity.dot(normal_derivative)
                        - dt * g.length() * normal_derivative.length()
                        - reach * rotation_curvature
                        - q.length() * deck_spin.length_squared();
                    let margin = 64.
                        * f64::EPSILON
                        * (s.center.length() + q.length() + reach + plane_offset.abs());
                    gap + lowest_displacement(rate, lower) > margin
                });
            if clear_above {
                return true;
            }
        }
        // Complete the separating-axis candidates for a quadrilateral and a
        // downward-unbounded deck prism. Only an upward or horizontal normal
        // has finite support on that prism.
        let sole_edges = [s.orientation * DVec3::X, s.orientation * DVec3::Y];
        std::iter::once(s.orientation * DVec3::Z)
            .chain(
                sole_edges
                    .into_iter()
                    .flat_map(|edge| fixed.map(|axis| edge.cross(axis))),
            )
            .filter_map(DVec3::try_normalize)
            .any(|n| (n.z >= 0. && clear(n, false)) || (n.z <= 0. && clear(-n, false)))
    }

    fn free_flight(
        &self,
        s: SoleState,
        half: DVec3,
        g: DVec3,
        dt: f64,
        torque: DVec3,
    ) -> Option<SoleState> {
        if torque != DVec3::ZERO
            || (s.spin != DVec3::ZERO
                && (self.inertia.x != self.inertia.y || self.inertia.y != self.inertia.z))
        {
            return None;
        }
        let clear = self.separated(s, half, g, dt, torque);
        clear.then(|| SoleState {
            center: s.center + dt * (s.velocity + 0.5 * dt * g),
            orientation: (DQuat::from_scaled_axis(dt * s.spin) * s.orientation).normalize(),
            velocity: s.velocity + dt * g,
            spin: s.spin,
        })
    }

    /// Certify in support coordinates and propagate in world coordinates:
    /// a distant or rapidly translating support must not round away a small
    /// COM displacement or a small velocity of an airborne foot.
    fn free_world(
        &self,
        world: SoleState,
        deck: SoleDeck,
        g: DVec3,
        dt: f64,
        torque: DVec3,
    ) -> Result<Option<SoleState>, SoleError> {
        let local = world.in_deck(deck);
        let Some(relative_end) = self.free_flight(
            local,
            deck.half_extents,
            deck.orientation.conjugate() * g,
            dt,
            torque,
        ) else {
            return Ok(None);
        };
        let end = SoleState {
            center: world.center + dt * (world.velocity + 0.5 * dt * g),
            orientation: (DQuat::from_scaled_axis(dt * world.spin) * world.orientation).normalize(),
            velocity: world.velocity + dt * g,
            spin: world.spin,
        };
        if !end.finite()
            || !(deck.center + dt * deck.velocity).is_finite()
            || self.corners(relative_end).iter().any(|p| !p.is_finite())
            || self.corners(end).iter().any(|p| !p.is_finite())
        {
            return Err(SoleError::Nonfinite);
        }
        Ok(Some(end))
    }

    /// Midpoint integration on a fixed microstep grid. No body state is committed
    /// until the entire interval, including its endpoint reaction, validates.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &self,
        state: &mut SoleState,
        deck: &SoleDeck,
        material: &SoleMaterial,
        gravity: DVec3,
        dt: f64,
        substeps: u32,
        quadrature_depth: u8,
    ) -> Result<SoleStep, SoleError> {
        self.step_with_wrench(
            state,
            deck,
            material,
            gravity,
            dt,
            substeps,
            quadrature_depth,
            DVec3::ZERO,
            DVec3::ZERO,
        )
    }

    /// Advance with an additional constant world force and torque at the COM.
    /// Impulse and work diagnostics describe contact only.
    #[allow(clippy::too_many_arguments)]
    pub fn step_with_wrench(
        &self,
        state: &mut SoleState,
        deck: &SoleDeck,
        material: &SoleMaterial,
        gravity: DVec3,
        dt: f64,
        substeps: u32,
        quadrature_depth: u8,
        force: DVec3,
        torque: DVec3,
    ) -> Result<SoleStep, SoleError> {
        if !self.valid(*state, *deck, quadrature_depth)
            || !dt.is_finite()
            || dt <= 0.
            || substeps == 0
            || !gravity.is_finite()
            || !force.is_finite()
            || !torque.is_finite()
        {
            return Err(SoleError::InvalidInput);
        }
        let d = *deck;
        let mut s = state.in_deck(d);
        let g = d.orientation.conjugate() * (gravity + force / self.mass);
        let applied_torque = d.orientation.conjugate() * torque;
        let h = dt / f64::from(substeps);
        if 0.5 * h == 0. {
            return Err(SoleError::InvalidInput);
        }
        let mut out = SoleStep::default();
        if let Some(end) = self.free_world(*state, d, gravity + force / self.mass, dt, torque)? {
            *state = end;
            return Ok(out);
        }
        let mut unresolved = false;
        for _ in 0..substeps {
            let a = self.patch(s, d.half_extents, *material, quadrature_depth)?;
            if unresolved && a.max_indentation == 0. {
                out.possible_missed_contacts += 1;
            }
            let mid = self.advance(s, s, a, g, applied_torque, 0.5 * h);
            let b = self.patch(mid, d.half_extents, *material, quadrature_depth)?;
            unresolved = a.max_indentation == 0.
                && b.max_indentation == 0.
                && !self.separated(s, d.half_extents, g, h, applied_torque);
            s = self.advance(s, mid, b, g, applied_torque, h);
            out.impulse += h * b.force;
            out.angular_impulse += h * b.torque;
            out.dissipated_energy += h * b.heat_power;
            out.sliding_work += h * b.sliding_power;
            out.boundary_work += h * b.boundary_power;
            out.peak_normal_force = out
                .peak_normal_force
                .max(a.normal_force)
                .max(b.normal_force);
        }
        let end = d.center + dt * d.velocity;
        out.contact = self
            .patch(s, d.half_extents, *material, quadrature_depth)?
            .world(d, end);
        if unresolved && out.contact.max_indentation == 0. {
            out.possible_missed_contacts += 1;
        }
        out.peak_normal_force = out.peak_normal_force.max(out.contact.normal_force);
        out.normal_impulse = out.impulse.z;
        out.impulse = d.orientation * out.impulse;
        out.angular_impulse = d.orientation * out.angular_impulse;
        let world = s.world(d, end);
        if !world.finite()
            || !out.contact.finite()
            || !out.impulse.is_finite()
            || !out.angular_impulse.is_finite()
            || ![out.dissipated_energy, out.sliding_work, out.boundary_work]
                .into_iter()
                .all(f64::is_finite)
        {
            return Err(SoleError::Nonfinite);
        }
        *state = world;
        Ok(out)
    }
}

pub(crate) mod adaptive;
pub use adaptive::{SoleAccuracy, SoleAdaptiveStep, SoleDifference};
#[cfg(test)]
mod tests;
