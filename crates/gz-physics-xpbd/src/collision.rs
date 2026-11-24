// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
mod compliant;
mod spatial;
mod turntable;
use super::oriented::{OrientedShape, Sweep};
use super::{Boundary, BoundaryImpulse, Particle, XpbdError, correct};
use glam::{DQuat, DVec3};
use std::collections::{HashMap, HashSet};
use turntable::Turntable;

#[derive(Clone, Copy)]
enum Shape {
    Capsule(super::capsule::Capsule),
    Sphere(f64, DVec3),
    Oriented(OrientedShape),
    FixedOrientationBox(DVec3, DQuat),
    Turntable(Turntable),
    Plane(DVec3),
}

fn box_geometry(
    half_extents: [f64; 3],
    orientation: [f64; 4],
) -> Result<(DVec3, DQuat), XpbdError> {
    let half = DVec3::from(half_extents);
    let q = DQuat::from_array(orientation);
    if !half.is_finite()
        || half.min_element() <= 0.
        || !half.length().is_finite()
        || !q.is_finite()
        || (q.length_squared() - 1.).abs() > 1e-10
    {
        return Err(XpbdError::InvalidBoundary);
    }
    Ok((half, q.normalize()))
}

#[derive(Clone)]
pub(super) struct PreparedBoundary {
    pub(super) receiver: Option<super::coupled::Receiver>,
    pub(super) origin: DVec3,
    pub(super) velocity: DVec3,
    pub(super) path_endpoints: Option<(DVec3, DVec3)>,
    shape: Shape,
    pub(super) normal_compliance: f64,
    /// Viscous damping times compliance, in seconds. Independent of pad quadrature weight.
    pub(super) normal_relaxation: f64,
}

impl PreparedBoundary {
    pub fn new(value: Boundary) -> Result<Self, XpbdError> {
        let (origin, velocity, shape) = match value {
            Boundary::Capsule {
                center,
                axis,
                radius,
                half_length,
                velocity,
                angular_speed,
            } => (
                center,
                velocity,
                Shape::Capsule(super::capsule::Capsule::new(
                    axis,
                    half_length,
                    radius,
                    angular_speed,
                )?),
            ),
            Boundary::DynamicCapsule {
                center,
                radius,
                half_length,
                velocity,
                angular_velocity,
            } => {
                if !DVec3::from(angular_velocity).is_finite() {
                    return Err(XpbdError::InvalidBoundary);
                }
                let geometry =
                    super::capsule::Capsule::new(DVec3::Z.to_array(), half_length, radius, 0.)?;
                (
                    center,
                    velocity,
                    Shape::Oriented(OrientedShape::Capsule(geometry)),
                )
            }
            Boundary::Sphere {
                center,
                radius,
                velocity,
                angular_velocity,
            } => {
                let spin = DVec3::from(angular_velocity);
                if !radius.is_finite() || radius <= 0.0 || !spin.is_finite() {
                    return Err(XpbdError::InvalidBoundary);
                }
                (center, velocity, Shape::Sphere(radius, spin))
            }
            Boundary::Cuboid {
                center,
                half_extents,
                velocity,
                angular_velocity,
            } => {
                let half = DVec3::from(half_extents);
                if !half.is_finite()
                    || half.min_element() <= 0.
                    || !half.length().is_finite()
                    || !DVec3::from(angular_velocity).is_finite()
                {
                    return Err(XpbdError::InvalidBoundary);
                }
                (
                    center,
                    velocity,
                    Shape::Oriented(OrientedShape::Cuboid(half)),
                )
            }
            Boundary::StaticCuboid {
                center,
                half_extents,
                orientation,
            } => (center, [0.; 3], {
                let (half, q) = box_geometry(half_extents, orientation)?;
                Shape::FixedOrientationBox(half, q)
            }),
            Boundary::TranslatingCuboid {
                center,
                half_extents,
                orientation,
                velocity,
            } => (center, velocity, {
                let (half, q) = box_geometry(half_extents, orientation)?;
                Shape::FixedOrientationBox(half, q)
            }),
            Boundary::Turntable {
                center,
                half_extents,
                orientation,
                velocity,
                angular_speed,
            } => {
                let (half, initial) = box_geometry(half_extents, orientation)?;
                if !angular_speed.is_finite() {
                    return Err(XpbdError::InvalidBoundary);
                }
                let spin = (initial * DVec3::Z) * angular_speed;
                (
                    center,
                    velocity,
                    Shape::Turntable(Turntable {
                        half,
                        initial,
                        spin,
                    }),
                )
            }
            Boundary::Plane {
                point,
                normal,
                velocity,
            } => {
                let normal = DVec3::from(normal)
                    .try_normalize()
                    .ok_or(XpbdError::InvalidBoundary)?;
                (point, velocity, Shape::Plane(normal))
            }
        };
        let (origin, velocity) = (DVec3::from(origin), DVec3::from(velocity));
        if !origin.is_finite() || !velocity.is_finite() {
            return Err(XpbdError::InvalidBoundary);
        }
        Ok(Self {
            receiver: None,
            path_endpoints: None,
            origin,
            velocity,
            shape,
            normal_compliance: 0.,
            normal_relaxation: 0.,
        })
    }

    fn centers(&self, interval: Interval) -> (DVec3, DVec3) {
        self.path_endpoints.unwrap_or_else(|| {
            (
                self.origin + self.velocity * interval.t0,
                self.origin + self.velocity * interval.t1,
            )
        })
    }

    pub(super) fn projection(
        &self,
        start: DVec3,
        end: DVec3,
        interval: Interval,
        all_entries: bool,
    ) -> Result<Option<Projection>, XpbdError> {
        let (c0, c1) = self.centers(interval);
        let mut a = start - c0;
        let mut b = end - c1;
        if let Some(receiver) = &self.receiver {
            a -= receiver.start_offset;
            b -= receiver.end_offset;
        }
        if !a.is_finite() || !b.is_finite() {
            return Err(XpbdError::Numerical);
        }
        let (normal, depth, initial_overlap) = match self.shape {
            Shape::Capsule(capsule) => {
                return Ok(capsule.project(a, b, interval.radius)?.map(
                    |(normal, depth, initial_overlap)| Projection {
                        normal,
                        depth,
                        initial_overlap,
                        event: None,
                    },
                ));
            }
            Shape::Plane(n) => (n, interval.radius - b.dot(n), interval.radius - a.dot(n)),
            Shape::FixedOrientationBox(half, q) => {
                let local_a = q.conjugate() * a;
                let local_b = q.conjugate() * b;
                if !(local_b - local_a).length_squared().is_finite() {
                    return Err(XpbdError::Numerical);
                }
                let Some(t) = super::cuboid::linear_entry(local_a, local_b, half, interval.radius)
                else {
                    return Ok(None);
                };
                let (_, n, surface) = super::cuboid::witness(local_a.lerp(local_b, t), half);
                let depth = interval.radius - (local_b - surface).dot(n);
                let roundoff = 64.
                    * f64::EPSILON
                    * (a.length() + b.length() + half.length() + interval.radius);
                let overlap = if t == 0. {
                    interval.radius - (local_a - surface).dot(n)
                } else {
                    0.
                };
                // A closing particle that reaches the skin exactly at the end of
                // the tick transfers momentum now. A grazing tangent transfers none.
                let closing = (local_a - local_b).dot(n);
                return Ok(
                    (depth > roundoff || (depth.abs() <= roundoff && closing > roundoff))
                        .then_some(Projection {
                            normal: q * n,
                            depth: depth.max(0.),
                            initial_overlap: overlap.max(0.),
                            event: None,
                        }),
                );
            }
            Shape::Turntable(table) => return table.projection(a, b, interval),
            Shape::Oriented(geometry) => {
                let (q0, q1) = self
                    .receiver
                    .as_ref()
                    .and_then(|r| r.orientations())
                    .ok_or(XpbdError::InvalidBoundary)?;
                let sweep = Sweep {
                    geometry,
                    radius: interval.radius,
                    start: a,
                    end: b,
                    q0,
                    q1,
                };
                if self.normal_compliance > 0. {
                    return compliant::projection(&sweep);
                }
                let Some(hit) = sweep.impact()? else {
                    return Ok(None);
                };
                if hit.initial_overlap > interval.radius + 1e-9 * geometry.extent() {
                    return Err(XpbdError::UnresolvedSweep);
                }
                let (normal, surface) = sweep.end_witness(&hit);
                let depth = interval.radius - (b - surface).dot(normal);
                // A rotating rounded feature can enter and leave before
                // either endpoint overlaps. Its transported tangent plane can
                // then create a fictitious normal impulse. Resolve the isolated
                // frictionless impact at first contact and advance only for
                // the remaining fraction of the substep.
                let angular = sweep.angular_travel();
                let rotates = sweep.shape_rotates(angular);
                if hit.time > 0. && (all_entries || rotates) {
                    let end_gap = geometry.witness(q1.conjugate() * b).0 - interval.radius;
                    let rounded = geometry.rounded(hit.local_normal);
                    let transient = rotates
                        && end_gap > 1e-8 * (geometry.extent() + interval.radius)
                        && (rounded || depth < 0.);
                    if all_entries || transient {
                        let orientation = sweep.rotation_at(hit.time);
                        let normal = orientation * hit.local_normal;
                        let arm = a.lerp(b, hit.time);
                        let closing = -(b - a - angular.cross(arm)).dot(normal) / interval.h;
                        if closing > 0. {
                            return Ok(Some(Projection {
                                normal,
                                depth: 0.,
                                initial_overlap: 0.,
                                event: Some(SweptImpact {
                                    time: hit.time,
                                    transient,
                                    orientation,
                                    arm,
                                    closing,
                                }),
                            }));
                        }
                    }
                }
                // Position error within floating-point resolution at the
                // endpoint must not generate a velocity impulse on a tangent.
                let spatial_floor = 64.
                    * f64::EPSILON
                    * (a.length() + b.length() + geometry.bound() + interval.radius);
                let depth = if depth.abs() <= spatial_floor {
                    0.
                } else {
                    depth
                };
                if matches!(geometry, OrientedShape::Capsule(_))
                    && depth == 0.
                    && hit.time > 0.
                    && (geometry.witness(q1.conjugate() * b).0 - interval.radius).abs()
                        <= spatial_floor
                    && (b - a - angular.cross(b)).dot(normal) < -spatial_floor
                {
                    // A contact arriving on the last instant changes velocity
                    // now and begins displacing the receiver on the next step.
                    return Ok(Some(Projection {
                        normal,
                        depth: 0.,
                        initial_overlap: hit.initial_overlap,
                        event: None,
                    }));
                }
                (normal, depth, hit.initial_overlap)
            }
            Shape::Sphere(radius, _) => {
                let radius = radius + interval.radius;
                let (d0, d1) = (a.length(), b.length());
                if !radius.is_finite() || !d0.is_finite() || !d1.is_finite() {
                    return Err(XpbdError::Numerical);
                }
                // Follow the relative segment to the first incident tangent plane.
                // An endpoint-only overlap would miss a sphere crossed in one substep.
                let travel = b - a;
                let length = travel.length();
                if !length.is_finite() {
                    return Err(XpbdError::Numerical);
                }
                let mut normal = None;
                if d0 >= radius && length > 0.0 {
                    let dir = travel / length;
                    let along = -a.dot(dir);
                    if along >= 0.0 {
                        let perpendicular = (a + dir * along).length();
                        if perpendicular <= radius {
                            let half_chord =
                                ((radius - perpendicular) * (radius + perpendicular)).sqrt();
                            let entry = along - half_chord;
                            if entry <= length && entry >= -f64::EPSILON * d0 {
                                normal = (a + dir * entry.max(0.0)).try_normalize();
                            }
                        }
                    }
                }
                let normal = normal.unwrap_or_else(|| {
                    b.try_normalize()
                        .or_else(|| a.try_normalize())
                        .unwrap_or(DVec3::X)
                });
                (normal, radius - b.dot(normal), radius - d0)
            }
        };
        if !depth.is_finite() || !initial_overlap.is_finite() {
            return Err(XpbdError::Numerical);
        }
        // A closing sweep can land exactly on the plane or sphere skin. Its
        // velocity impulse belongs to this interval even with zero penetration.
        let closing_endpoint = depth == 0. && (a - b).dot(normal) > 0.;
        Ok((depth > 0. || closing_endpoint).then_some(Projection {
            event: None,
            normal,
            depth,
            initial_overlap: initial_overlap.max(0.0),
        }))
    }

    /// Other particles can move the same receiver later in the last projection
    /// sweep. Refuse an unresolved endpoint before publishing any soft state.
    pub fn check_oriented_endpoint(
        &self,
        particles: &[Particle],
        radius: f64,
    ) -> Result<(), XpbdError> {
        let Shape::Oriented(geometry) = self.shape else {
            return Ok(());
        };
        let r = self.receiver.as_ref().ok_or(XpbdError::InvalidBoundary)?;
        let (_, q) = r.orientations().ok_or(XpbdError::InvalidBoundary)?;
        let center = self.origin + self.velocity * r.end_time + r.end_offset;
        for p in particles.iter().filter(|p| p.inverse_mass > 0.) {
            let local = q.conjugate() * (DVec3::from(p.position) - center);
            let tolerance = 1e-8 * (geometry.extent() + radius)
                + 64. * f64::EPSILON * center.abs().max_element();
            let minimum = if self.normal_compliance > 0. {
                0.
            } else {
                radius
            };
            if geometry.witness(local).0 < minimum - tolerance {
                return Err(XpbdError::UnresolvedSweep);
            }
        }
        Ok(())
    }
    /// Prescribed moving cuboids may report signed per-particle pad loads at
    /// zero compliance as well as through the compliant contact projection.
    fn prescribed_pad(&self) -> bool {
        matches!(self.shape, Shape::Oriented(OrientedShape::Cuboid(_)))
            && self.receiver.as_ref().is_some_and(|r| r.inverse_mass == 0.)
    }

    fn react_at_particle(
        &self,
        reaction: &mut BoundaryImpulse,
        loads: &mut Option<Box<crate::pad_loads::PadLoads>>,
        (boundary, particle): (usize, usize),
        point: DVec3,
        [normal, tangent]: [DVec3; 2],
    ) -> Result<(), XpbdError> {
        reaction.react(point - self.origin, normal + tangent)?;
        if let Some(loads) = loads
            && self.prescribed_pad()
        {
            loads.react(boundary, particle, normal, tangent)?;
        }
        Ok(())
    }

    fn record_plane_impulse(
        &self,
        loads: &mut Option<Box<super::pad_loads::PadLoads>>,
        boundary: usize,
        particle: usize,
        normal: DVec3,
        tangent: DVec3,
    ) {
        if let Some(loads) = loads
            && matches!(self.shape, Shape::Plane(_))
        {
            loads.observe_plane(boundary, particle, normal, tangent);
        }
    }

    fn has_planar_surface(&self) -> bool {
        matches!(
            self.shape,
            Shape::Plane(_) | Shape::FixedOrientationBox(_, _) | Shape::Turntable(_)
        )
    }

    /// Both projected endpoints must lie on the same flat face. Leave rounding
    /// around a cuboid's edge or corner to the normal swept-contact solver.
    fn on_friction_face(
        &self,
        endpoints: [DVec3; 2],
        normal: DVec3,
        radius: f64,
        tolerance: f64,
    ) -> bool {
        if let Shape::FixedOrientationBox(half, orientation) = self.shape {
            let inverse = orientation.conjugate();
            let direction = inverse * normal;
            let axis = direction.abs().max_position();
            let mut face_normal = DVec3::ZERO;
            face_normal[axis] = direction[axis].signum();
            if (direction - face_normal).length() > 64. * f64::EPSILON {
                return false;
            }
            let face = direction[axis].signum() * (half[axis] + radius);
            return endpoints.into_iter().all(|point| {
                let local = inverse * point;
                (local[axis] - face).abs() <= tolerance
                    && (0..3).all(|i| i == axis || local[i].abs() < half[i] - tolerance)
            });
        }
        endpoints
            .into_iter()
            .all(|p| (p.dot(normal) - radius).abs() <= tolerance)
    }

    pub(super) fn box_spin(&self) -> DVec3 {
        if let Shape::Turntable(table) = self.shape {
            table.spin
        } else {
            DVec3::ZERO
        }
    }
    pub(super) fn one_way_box(&self, t: f64) -> Option<(DVec3, DQuat)> {
        if let Shape::Turntable(table) = self.shape {
            return Some((table.half, table.orientation(t)));
        }
        if let Shape::FixedOrientationBox(half, q) = self.shape {
            Some((half, q))
        } else {
            None
        }
    }
    pub(super) fn oriented_shape(&self) -> Option<OrientedShape> {
        if let Shape::Oriented(geometry) = self.shape {
            Some(geometry)
        } else {
            None
        }
    }
    pub(super) fn end_plane(&self, t: f64) -> Option<(DVec3, DVec3)> {
        if let Shape::Plane(n) = self.shape {
            Some((self.origin + t * self.velocity, n))
        } else {
            None
        }
    }
    pub(super) fn material_capsule(&self) -> bool {
        matches!(
            self.shape,
            Shape::Capsule(_) | Shape::Oriented(OrientedShape::Capsule(_))
        )
    }
    pub(super) fn prescribed_capsule(&self) -> Option<super::capsule::Capsule> {
        if let Shape::Capsule(c) = self.shape {
            Some(c)
        } else {
            None
        }
    }
    pub(super) fn sphere_radius(&self) -> Option<f64> {
        if let Shape::Sphere(radius, _) = self.shape {
            Some(radius)
        } else {
            None
        }
    }
    pub fn is_oriented(&self) -> bool {
        matches!(self.shape, Shape::Oriented(_))
    }

    fn surface_velocity(&self, normal: DVec3, x: DVec3, t: f64) -> DVec3 {
        match self.shape {
            Shape::Capsule(c) => {
                self.velocity + (c.axis * c.angular_speed).cross(normal * c.radius)
            }
            Shape::Sphere(r, angular) => self.velocity + angular.cross(normal * r),
            Shape::Turntable(table) => {
                self.velocity + table.spin.cross(x - (self.origin + t * self.velocity))
            }
            Shape::FixedOrientationBox(_, _) | Shape::Plane(_) => self.velocity,
            Shape::Oriented(_) => unreachable!("oriented boundaries require a finite receiver"),
        }
    }
}

#[derive(Clone, Copy)]
pub(super) struct Interval {
    pub t0: f64,
    pub t1: f64,
    pub h: f64,
    pub radius: f64,
}

pub(super) struct SweptImpact {
    pub(super) transient: bool,
    pub(super) time: f64,
    orientation: DQuat,
    arm: DVec3,
    closing: f64,
}
pub(super) struct Projection {
    pub(super) event: Option<SweptImpact>,
    pub(super) normal: DVec3,
    depth: f64,
    initial_overlap: f64,
}

#[derive(Clone)]
struct Contact {
    particle: usize,
    boundary: usize,
    normal: DVec3,
    correction: f64,
    projected_impulse: f64,
    remaining: Option<DVec3>,
    tangent_impulse: DVec3,
    sticking_on_plane: bool,
    initial_overlap: f64,
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub(super) enum PlaneFrictionLaw {
    StaticOnly,
    #[cfg(feature = "one-way-checkpoint")]
    Projected,
    #[default]
    WithSlipArrest,
}

/// Impulse for the tangential position solve when straight-line slip stops
/// inside this substep. The endpoint velocity solve accounts for the remainder
/// of the interval spent at rest. A transverse load can turn the slip direction
/// and is left to the ordinary vector Coulomb solve.
fn straight_slip_stop(
    incoming: DVec3,
    position_target: DVec3,
    inverse_mass: f64,
    static_cap: f64,
    kinetic_cap: f64,
) -> Option<DVec3> {
    let speed = incoming.length();
    let direction = incoming / speed;
    // Without this contact's current tangential response, the predicted
    // relative displacement over h corresponds to this free velocity change.
    let drive = -inverse_mass * position_target - incoming;
    let along = drive.dot(direction);
    let braking = kinetic_cap * inverse_mass - along;
    if braking < speed || drive.length() > static_cap * inverse_mass {
        return None;
    }
    let transverse = drive - along * direction;
    if transverse.length() > 128. * f64::EPSILON * (drive.length() + speed) {
        return None;
    }
    // During t_stop, velocity decreases linearly from incoming to zero.
    // t_stop / h = speed / braking. The rest of the substep has zero slip.
    let arrest = position_target + (0.5 * (speed / braking) * incoming) / inverse_mass;
    arrest.is_finite().then_some(arrest)
}

impl Contact {
    /// Solve sustained tangential displacement against the static or sliding cone.
    /// A newly arriving point keeps the tangential travel preceding its impact.
    fn project_plane_friction(
        &mut self,
        end: &mut DVec3,
        (p, incoming): (&mut Particle, DVec3),
        plane: &PreparedBoundary,
        time: Interval,
        (static_friction, kinetic_friction, law): (f64, f64, PlaneFrictionLaw),
        reaction: &mut BoundaryImpulse,
    ) -> Result<DVec3, XpbdError> {
        let sliding_friction = (law != PlaneFrictionLaw::StaticOnly).then_some(kinetic_friction);
        let start = DVec3::from(p.position);
        let (c0, c1) = plane.centers(time);
        let normal = self.normal;
        let tolerance = 64.
            * f64::EPSILON
            * start
                .abs()
                .max(c0.abs())
                .max(end.abs())
                .max(c1.abs())
                .max_element()
                .max(time.radius);
        let (travel, start_velocity) = if let Shape::Turntable(table) = plane.shape {
            // Carry the foot of the particle with the actual surface arc.
            // The flat upper/lower faces have a stationary world normal.
            let a = table.orientation(time.t0).conjugate() * (start - c0);
            let b = table.orientation(time.t1).conjugate() * (*end - c1);
            let axis = table.initial * DVec3::Z;
            let sign = normal.dot(axis).signum();
            if normal.distance(sign * axis) > 1e-12
                || [a, b].into_iter().any(|x| {
                    (x.z - sign * (table.half.z + time.radius)).abs() > tolerance
                        || x.x.abs() >= table.half.x - tolerance
                        || x.y.abs() >= table.half.y - tolerance
                })
            {
                return Ok(DVec3::ZERO);
            }
            let carried = c1 + table.orientation(time.t1) * a;
            (
                carried - start,
                plane.surface_velocity(normal, start, time.t0),
            )
        } else {
            if !plane.on_friction_face([start - c0, *end - c1], normal, time.radius, tolerance) {
                return Ok(DVec3::ZERO);
            }
            (c1 - c0, (c1 - c0) / time.h)
        };
        let incoming_relative = incoming - start_velocity;
        let incoming_tangent = incoming_relative - normal * incoming_relative.dot(normal);
        let initially_still = incoming_tangent.length() * time.h <= tolerance;
        if sliding_friction.is_none() && !initially_still {
            return Ok(DVec3::ZERO);
        }
        let slip = (*end - start) - travel;
        let response = time.h * p.inverse_mass;
        let target = self.tangent_impulse - (slip - normal * slip.dot(normal)) / response;
        let normal_impulse = self.correction / response;
        // Preserve the rounding order in previously serialized static-only materials.
        let cap = if sliding_friction.is_some() {
            static_friction * normal_impulse
        } else {
            (static_friction * self.correction / time.h) / p.inverse_mass
        };
        if !target.is_finite() || !cap.is_finite() {
            return Err(XpbdError::Numerical);
        }
        let sticking = if matches!(plane.shape, Shape::Turntable(_)) {
            // A small initial slip can stop within this substep. A sustained
            // orbit also requires sufficient traction for its endpoint velocity,
            // including the second half of the centripetal impulse.
            let remaining =
                DVec3::from(p.velocity) - plane.surface_velocity(normal, start + travel, time.t1);
            let required = self.tangent_impulse
                - (remaining - normal * remaining.dot(normal)) / p.inverse_mass;
            let arrest = sliding_friction.unwrap_or(0.) * normal_impulse * p.inverse_mass;
            (initially_still || incoming_tangent.length() <= arrest)
                && target.length().max(required.length()) <= cap
        } else {
            initially_still && target.length() <= cap
        };
        let stopped = if law == PlaneFrictionLaw::WithSlipArrest
            && !initially_still
            && !matches!(plane.shape, Shape::Turntable(_))
        {
            straight_slip_stop(
                incoming_tangent,
                target,
                p.inverse_mass,
                cap,
                kinetic_friction * normal_impulse,
            )
        } else {
            None
        };
        let next = if let Some(arrest) = stopped {
            arrest
        } else if sticking {
            target
        } else {
            sliding_friction.map_or(DVec3::ZERO, |mu| {
                target.clamp_length_max(mu * normal_impulse)
            })
        };
        self.sticking_on_plane =
            stopped.is_some() || sticking && (sliding_friction.is_some() || next != DVec3::ZERO);
        let delta = next - self.tangent_impulse;
        self.tangent_impulse = next;
        if delta != DVec3::ZERO {
            correct(end, p, response * delta, time.h);
            reaction.react(start - plane.origin, delta)?;
        }
        Ok(delta)
    }
}

#[derive(Clone, Default)]
pub(super) struct Contacts {
    pub(super) pad_samples: Vec<crate::PadSample>,
    pub(super) pad_loads: Option<Box<super::pad_loads::PadLoads>>,
    pub(super) plane_loads: Option<Box<super::pad_loads::PadLoads>>,
    friction: f64,
    pub(super) static_plane_friction: Option<f64>,
    pub(super) plane_friction_law: PlaneFrictionLaw,
    incoming_on_plane: Vec<DVec3>,
    items: Vec<Contact>,
    index: HashMap<(usize, usize), usize>,
}

impl Contacts {
    pub(super) fn begin_plane_friction(&mut self, p: &[Particle], boundaries: &[PreparedBoundary]) {
        self.incoming_on_plane.clear();
        if self.static_plane_friction.is_some()
            && boundaries.iter().any(PreparedBoundary::has_planar_surface)
        {
            self.incoming_on_plane
                .extend(p.iter().map(|p| DVec3::from(p.velocity)));
        }
    }

    pub(super) fn on_single_plane(&self, boundaries: &[PreparedBoundary]) -> bool {
        self.items.first().is_some_and(|first| {
            matches!(boundaries[first.boundary].shape, Shape::Plane(_))
                && self.items.iter().all(|c| c.boundary == first.boundary)
        })
    }

    /// A terminal velocity correction is planar only while every swept skin stays
    /// inside the same face of the same prescribed, non-rotating box.
    pub(super) fn on_box_face(
        &self,
        x: &[DVec3],
        p: &[Particle],
        boundaries: &[PreparedBoundary],
        time: Interval,
    ) -> bool {
        let Some(first) = self.items.first() else {
            return false;
        };
        let b = &boundaries[first.boundary];
        let Shape::FixedOrientationBox(half, q) = b.shape else {
            return false;
        };
        let inverse = q.conjugate();
        let normal = inverse * first.normal;
        let axis = normal.abs().max_position();
        let mut face = DVec3::ZERO;
        face[axis] = normal[axis].signum();
        let (before, after) = b.centers(time);
        let epsilon = 128. * f64::EPSILON;
        (normal - face).length() <= epsilon
            && self.items.iter().all(|c| {
                c.boundary == first.boundary && (c.normal - first.normal).length() <= epsilon
            })
            && x.iter().zip(p).all(|(&end, node)| {
                let a = inverse * (DVec3::from(node.position) - before);
                let z = inverse * (end - after);
                let tolerance = time.radius * 1e-9
                    + epsilon * before.abs().max(after.abs()).max(end.abs()).max_element();
                let sign = face[axis];
                (sign * a[axis]).min(sign * z[axis]) >= half[axis] + time.radius - tolerance
                    && (0..3).all(|i| {
                        i == axis || a[i].abs().max(z[i].abs()) + time.radius < half[i] - tolerance
                    })
            })
    }
    pub(super) fn spring_pads_with_one_plane(&self, boundaries: &[PreparedBoundary]) -> bool {
        let mut plane = None;
        let mut pad = false;
        for c in &self.items {
            let b = &boundaries[c.boundary];
            if b.normal_compliance > 0. {
                pad = true;
            } else if !matches!(b.shape, Shape::Plane(_))
                || plane.replace(c.boundary).is_some_and(|p| p != c.boundary)
            {
                return false;
            }
        }
        pad
    }
    /// Spend only the additional normal impulse on friction. The ordinary
    /// impact has already spent its own Coulomb budget on these contacts.
    pub(super) fn finish_plane_increment(
        &mut self,
        x: &[DVec3],
        p: &mut [Particle],
        boundaries: &[PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
        friction: f64,
        time: Interval,
    ) -> Result<(), XpbdError> {
        for c in &self.items {
            let b = &boundaries[c.boundary];
            // A spring pad uses a finite force through this endpoint. Only a
            // rigid support imposes an additional nonclosing velocity constraint.
            let point = x[c.particle];
            let center = b.origin + time.t1 * b.velocity;
            let tolerance = 1e-9 * time.radius
                + 128. * f64::EPSILON * point.abs().max(center.abs()).max_element();
            let local = point - center;
            let touching = match b.shape {
                Shape::Plane(_) => local.dot(c.normal) - time.radius <= tolerance,
                Shape::FixedOrientationBox(..) => {
                    b.on_friction_face([local; 2], c.normal, time.radius, tolerance)
                }
                _ => false,
            };
            if !touching {
                continue;
            }
            let particle = &mut p[c.particle];
            let v = DVec3::from(particle.velocity);
            let relative = v - b.velocity;
            let vn = relative.dot(c.normal);
            if vn < 0. {
                let tangent = relative - c.normal * vn;
                let speed = tangent.length();
                let tangential_loss =
                    tangent * ((-vn * friction) / speed.max(f64::MIN_POSITIVE)).min(1.);
                let dv = -vn * c.normal - tangential_loss;
                particle.velocity = (v + dv).to_array();
                reactions[c.boundary]
                    .react(x[c.particle] - b.origin, dv / particle.inverse_mass)?;
                b.record_plane_impulse(
                    &mut self.plane_loads,
                    c.boundary,
                    c.particle,
                    -vn * c.normal / particle.inverse_mass,
                    -tangential_loss / particle.inverse_mass,
                );
            }
        }
        Ok(())
    }

    pub fn clear(&mut self, friction: f64) {
        self.friction = friction;
        self.items.clear();
        self.index.clear();
    }
    fn record(
        &mut self,
        particle: usize,
        boundary: usize,
        hit: &Projection,
        impulse: f64,
        remaining: Option<DVec3>,
    ) {
        let idx = *self.index.entry((particle, boundary)).or_insert_with(|| {
            self.items.push(Contact {
                particle,
                boundary,
                normal: hit.normal,
                correction: 0.0,
                projected_impulse: 0.,
                remaining,
                tangent_impulse: DVec3::ZERO,
                sticking_on_plane: false,
                initial_overlap: hit.initial_overlap,
            });
            self.items.len() - 1
        });
        self.items[idx].normal = hit.normal;
        self.items[idx].correction += hit.depth;
        self.items[idx].projected_impulse += impulse;
    }
}

impl BoundaryImpulse {
    pub(super) fn react(&mut self, arm: DVec3, particle_impulse: DVec3) -> Result<(), XpbdError> {
        let linear = DVec3::from(self.linear) - particle_impulse;
        let angular = DVec3::from(self.angular) - arm.cross(particle_impulse);
        if !linear.is_finite() || !angular.is_finite() {
            return Err(XpbdError::Numerical);
        }
        self.linear = linear.to_array();
        self.angular = angular.to_array();
        Ok(())
    }
}

pub(super) fn project_boundaries(
    x: &mut [DVec3],
    particles: &mut [Particle],
    boundaries: &mut [PreparedBoundary],
    contacts: &mut Contacts,
    reactions: &mut [BoundaryImpulse],
    time: Interval,
    feature_contacts: (bool, bool),
) -> Result<(), XpbdError> {
    project_points::<true>(
        x,
        particles,
        boundaries,
        contacts,
        reactions,
        time,
        feature_contacts,
    )
}
fn project_points<const SPATIAL: bool>(
    x: &mut [DVec3],
    particles: &mut [Particle],
    boundaries: &mut [PreparedBoundary],
    contacts: &mut Contacts,
    reactions: &mut [BoundaryImpulse],
    time: Interval,
    feature_contacts: (bool, bool),
) -> Result<(), XpbdError> {
    let (shell_pads, capsule_material) = feature_contacts;
    if boundaries.is_empty()
        || (capsule_material && boundaries.iter().all(PreparedBoundary::material_capsule))
    {
        return Ok(());
    }
    let elastic = boundaries.iter().any(|b| b.normal_compliance > 0.);
    let mut active = Vec::new();
    let field = if SPATIAL {
        spatial::Field::new(x.len(), boundaries, time)
    } else {
        None
    };
    for (k, (xs, ps)) in x
        .chunks_mut(spatial::BLOCK)
        .zip(particles.chunks_mut(spatial::BLOCK))
        .enumerate()
    {
        let block = field
            .as_ref()
            .map_or_else(spatial::Block::default, |f| f.block(xs, ps));
        for (local, (x, p)) in xs.iter_mut().zip(ps).enumerate() {
            let i = k * spatial::BLOCK + local;
            let mut skip = block.skip;
            let start = DVec3::from(p.position);
            // Prescribed nodes transmit a finite spring/dashpot force to the
            // pad. A rigid positional projection cannot move a prescribed node.
            let rigid_rows = if p.inverse_mass > 0. {
                &mut *boundaries
            } else {
                &mut []
            };
            for (j, boundary) in rigid_rows.iter_mut().enumerate() {
                if skip != 0 && skip & (1u64 << j) != 0 {
                    continue;
                }
                if boundary.normal_compliance > 0.
                    || (shell_pads && boundary.sphere_radius().is_some())
                    || (capsule_material && boundary.material_capsule())
                {
                    continue;
                }
                if (boundary.is_oriented() || matches!(boundary.shape, Shape::Turntable(_)))
                    && contacts
                        .index
                        .get(&(i, j))
                        .is_some_and(|&k| contacts.items[k].remaining.is_some())
                {
                    continue;
                }
                if let Some(hit) = boundary.projection(start, *x, time, false)? {
                    let mut normal_impulse = 0.;
                    let mut remaining = None;
                    let mut dx = hit.normal * hit.depth;
                    if let Some(receiver) = &mut boundary.receiver {
                        let inverse_mass = p.inverse_mass + receiver.inverse_mass;
                        if !inverse_mass.is_finite() {
                            return Err(XpbdError::Numerical);
                        }
                        let start_center =
                            boundary.origin + time.t0 * boundary.velocity + receiver.start_offset;
                        let arm = start - start_center;
                        let oriented = matches!(boundary.shape, Shape::Oriented(_));
                        if let Some(event) = &hit.event {
                            if !receiver.isolated_particle {
                                return Err(XpbdError::UnresolvedSweep);
                            }
                            let inverse = receiver.inverse_mass_at(
                                event.orientation,
                                inverse_mass,
                                event.arm,
                                hit.normal,
                            )?;
                            let full = event.closing / inverse;
                            normal_impulse = (1. - event.time) * full;
                            remaining = Some(hit.normal * (event.time * full));
                        } else {
                            let inverse = if oriented {
                                receiver.directional_inverse_mass(inverse_mass, arm, hit.normal)?
                            } else {
                                inverse_mass
                            };
                            normal_impulse = hit.depth / inverse / time.h;
                        }
                        let impulse = hit.normal * normal_impulse;
                        dx = time.h * p.inverse_mass * impulse;
                        if oriented {
                            receiver.project_oriented(arm, -impulse, time.h)?;
                        } else {
                            receiver.project_normal(arm, -impulse, time.h);
                        }
                    }
                    if let Shape::Turntable(table) = boundary.shape
                        && let Some(event) = &hit.event
                    {
                        let relative =
                            (*x - start) / time.h - boundary.velocity - table.spin.cross(event.arm);
                        let tangent = relative - hit.normal * relative.dot(hit.normal);
                        let delta_v = hit.normal * event.closing
                            - tangent.clamp_length_max(contacts.friction * event.closing);
                        dx = time.h * (1. - event.time) * delta_v;
                        remaining = Some((event.time / p.inverse_mass) * delta_v);
                    }
                    correct(x, p, dx, time.h);
                    // A prior surface may push this point into an originally disjoint bound.
                    if skip != 0 && !block.contains(*x) {
                        skip = 0;
                    }
                    // With x1 = x0 + h*v1, a positional correction contributes
                    // x0 cross (m*dx/h) to discrete orbital angular momentum.
                    let applied = dx / time.h / p.inverse_mass;
                    boundary.react_at_particle(
                        &mut reactions[j],
                        &mut contacts.pad_loads,
                        (j, i),
                        start,
                        [applied, DVec3::ZERO],
                    )?;
                    boundary.record_plane_impulse(
                        &mut contacts.plane_loads,
                        j,
                        i,
                        applied,
                        DVec3::ZERO,
                    );
                    contacts.record(i, j, &hit, normal_impulse, remaining);
                }
                if let Some(coefficient) = contacts.static_plane_friction
                    && boundary.has_planar_surface()
                    && (contacts.plane_friction_law != PlaneFrictionLaw::StaticOnly
                        || matches!(boundary.shape, Shape::Plane(_)))
                    && let Some(&incoming) = contacts.incoming_on_plane.get(i)
                    && let Some(&row) = contacts.index.get(&(i, j))
                {
                    let tangent = contacts.items[row].project_plane_friction(
                        x,
                        (p, incoming),
                        boundary,
                        time,
                        (coefficient, contacts.friction, contacts.plane_friction_law),
                        &mut reactions[j],
                    )?;
                    boundary.record_plane_impulse(
                        &mut contacts.plane_loads,
                        j,
                        i,
                        DVec3::ZERO,
                        tangent,
                    );
                    if skip != 0 && !block.contains(*x) {
                        skip = 0;
                    }
                }
            }
            if elastic && contacts.pad_samples.is_empty() {
                contacts.project_compliant(
                    (i, p, None),
                    x,
                    boundaries,
                    reactions,
                    time,
                    &mut active,
                )?;
            }
        }
    }
    if elastic {
        contacts.project_pad_samples(x, particles, boundaries, reactions, time, &mut active)?;
    }
    Ok(())
}

pub(super) fn finish_contacts(
    contacts: &mut Contacts,
    x: &[DVec3],
    particles: &mut [Particle],
    boundaries: &mut [PreparedBoundary],
    reactions: &mut [BoundaryImpulse],
    mu: f64,
    time: Interval,
) -> Result<(), XpbdError> {
    let h = time.h;
    for c in &contacts.items {
        let b = &mut boundaries[c.boundary];
        if b.normal_compliance > 0. {
            continue;
        }
        let p = &mut particles[c.particle];
        if b.receiver.is_some() {
            let [normal, tangent] = finish_receiver(c, x[c.particle], p, b, mu, h)?;
            b.react_at_particle(
                &mut reactions[c.boundary],
                &mut contacts.pad_loads,
                (c.boundary, c.particle),
                x[c.particle],
                [normal, tangent],
            )?;
            continue;
        }
        let v = DVec3::from(p.velocity);
        if let Some(remainder) = c.remaining {
            p.velocity = (v + p.inverse_mass * remainder).to_array();
            reactions[c.boundary].react(x[c.particle] - b.origin, remainder)?;
            continue;
        }
        let relative =
            v - b.surface_velocity(c.normal, x[c.particle] - time.radius * c.normal, time.t1);
        let vn = relative.dot(c.normal);
        // Remove residual closing speed. Resolving an initial overlap supplies
        // no restitution: discard separating speed created by its correction.
        let normal_change = if vn < 0.0 {
            -vn
        } else {
            -vn.min(c.initial_overlap / h)
        };
        let tangent = relative - c.normal * vn;
        let slip = tangent.length();
        let coefficient = if c.sticking_on_plane {
            contacts.static_plane_friction.unwrap_or(mu)
        } else {
            mu
        };
        let cap = coefficient * (c.correction / h + normal_change).max(0.0);
        if !slip.is_finite() || !cap.is_finite() {
            return Err(XpbdError::Numerical);
        }
        let friction = if c.tangent_impulse != DVec3::ZERO {
            // Cap the total tangential impulse, including the position solve.
            let prior = p.inverse_mass * c.tangent_impulse;
            (prior - tangent).clamp_length_max(cap) - prior
        } else if slip > 0.0 {
            -tangent * (cap.min(slip) / slip)
        } else {
            DVec3::ZERO
        };
        let change = c.normal * normal_change + friction;
        p.velocity = (v + change).to_array();
        reactions[c.boundary].react(x[c.particle] - b.origin, change / p.inverse_mass)?;
        b.record_plane_impulse(
            &mut contacts.plane_loads,
            c.boundary,
            c.particle,
            c.normal * normal_change / p.inverse_mass,
            friction / p.inverse_mass,
        );
    }
    Ok(())
}

fn finish_receiver(
    c: &Contact,
    x: DVec3,
    p: &mut Particle,
    b: &mut PreparedBoundary,
    mu: f64,
    h: f64,
) -> Result<[DVec3; 2], XpbdError> {
    let r = b.receiver.as_mut().expect("finite receiver");
    let arm = x - (b.origin + b.velocity * r.end_time + r.end_offset);
    if let Some(impulse) = c.remaining {
        p.velocity = (DVec3::from(p.velocity) + p.inverse_mass * impulse).to_array();
        r.apply_velocity_impulse(arm, -impulse);
        return Ok([impulse, DVec3::ZERO]);
    }
    let inverse_mass = p.inverse_mass + r.inverse_mass;
    let relative = |v: DVec3, r: &super::coupled::Receiver| {
        v - (b.velocity + r.linear + r.angular_velocity().cross(arm))
    };
    let mut v = DVec3::from(p.velocity);
    let vn = relative(v, r).dot(c.normal);
    let normal_change = if vn < 0.0 {
        -vn
    } else {
        -vn.min(c.initial_overlap / h)
    };
    let normal_impulse = normal_change / r.directional_inverse_mass(inverse_mass, arm, c.normal)?;
    let j = c.normal * normal_impulse;
    v += p.inverse_mass * j;
    r.apply_velocity_impulse(arm, -j);
    let relative = relative(v, r);
    let tangent = relative - c.normal * relative.dot(c.normal);
    let slip = tangent.length();
    let projected = if matches!(b.shape, Shape::Oriented(_)) {
        c.projected_impulse
    } else {
        c.correction / h / inverse_mass
    };
    let cap = mu * (projected + normal_impulse).max(0.);
    if !slip.is_finite() || !cap.is_finite() {
        return Err(XpbdError::Numerical);
    }
    let mut friction = DVec3::ZERO;
    if slip > 0.0 {
        friction = r.friction(arm, c.normal, tangent, slip, inverse_mass, cap)?;
        r.apply_velocity_impulse(arm, -friction);
    }
    p.velocity = (v + p.inverse_mass * friction).to_array();
    Ok([j, friction])
}

/// A hash lookup per neighbour cell with no dependence on hash iteration order.
/// Each iteration rebuilds the grid. Constraint neighbours are excluded.
#[derive(Clone)]
pub(super) struct SelfContact {
    edges: HashSet<(usize, usize)>,
    heads: HashMap<[i64; 3], usize>,
    next: Vec<Option<usize>>,
}

impl SelfContact {
    pub fn new(edges: HashSet<(usize, usize)>, n: usize) -> Self {
        Self {
            edges,
            heads: HashMap::with_capacity(n),
            next: vec![None; n],
        }
    }

    pub fn project(
        &mut self,
        x: &mut [DVec3],
        p: &mut [Particle],
        diameter: f64,
        h: f64,
    ) -> Result<(), XpbdError> {
        self.heads.clear();
        for i in 0..x.len() {
            let coords = (x[i] / diameter).floor();
            if !coords.is_finite() || coords.abs().max_element() >= i64::MAX as f64 {
                return Err(XpbdError::Numerical);
            }
            let [a, b, c] = coords.to_array().map(|v| v as i64);
            for da in -1..=1 {
                for db in -1..=1 {
                    for dc in -1..=1 {
                        let mut head = self.heads.get(&[a + da, b + db, c + dc]).copied();
                        while let Some(j) = head {
                            if !self.edges.contains(&(j, i)) {
                                separate(x, p, j, i, diameter, h)?;
                            }
                            head = self.next[j];
                        }
                    }
                }
            }
            self.next[i] = self.heads.insert([a, b, c], i);
        }
        Ok(())
    }
}

fn separate(
    x: &mut [DVec3],
    p: &mut [Particle],
    a: usize,
    b: usize,
    diameter: f64,
    h: f64,
) -> Result<(), XpbdError> {
    let delta = x[a] - x[b];
    let len = delta.length();
    let (wa, wb) = (p[a].inverse_mass, p[b].inverse_mass);
    if !len.is_finite() || !(wa + wb).is_finite() {
        return Err(XpbdError::Numerical);
    }
    if len >= diameter || wa + wb == 0.0 {
        return Ok(());
    }
    let start = DVec3::from(p[a].position) - DVec3::from(p[b].position);
    let n = if len > 0.0 {
        delta / len
    } else {
        start.try_normalize().unwrap_or(DVec3::X)
    };
    let projection = n * ((diameter - len) / (wa + wb));
    correct(&mut x[a], &mut p[a], wa * projection, h);
    correct(&mut x[b], &mut p[b], -wb * projection, h);
    let vn = (DVec3::from(p[a].velocity) - DVec3::from(p[b].velocity)).dot(n);
    let overlap = (diameter - start.length()).max(0.0);
    let dv = if vn < 0.0 { -vn } else { -vn.min(overlap / h) };
    let impulse = n * (dv / (wa + wb));
    p[a].velocity = (DVec3::from(p[a].velocity) + wa * impulse).to_array();
    p[b].velocity = (DVec3::from(p[b].velocity) - wb * impulse).to_array();
    Ok(())
}

#[cfg(test)]
#[path = "../tests/private/plane_impulse.rs"]
mod plane_impulse;

#[cfg(test)]
#[path = "../tests/private/hard_pad_impulses.rs"]
mod hard_pad_impulses;
