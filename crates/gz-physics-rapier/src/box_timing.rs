// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Causal timing for a floating rigid sole approaching the top of a translating cuboid.
use super::*;
use gz_physics::NativeStepGrid;
mod flight;
mod free_rotation;
pub(crate) use free_rotation::GuardedMotion;
mod compact;
mod opponent;
mod rigid_point;
pub use opponent::OpponentClearance;
pub use rigid_point::upper::{OpposingCornerReach, OpposingCornerReceipt,OpposingReturnReceipt,OpposingNormalReceipt,OpposingNormalEvent,OpposingNormalDecline,OpposingNormalStage,OpposingLimitStage};
pub(crate) use rigid_point::upper::{UpperPair,PriorUpper};
pub use rigid_point::{RigidPointDecline, RigidPointImpactReceipt};
mod pulse;
mod rotation;
mod sweep;
mod taylor;
pub use compact::{CompactDecline, CompactImpactReceipt};
pub(crate) use flight::FlightGuard;
pub use flight::{BoxFlightReceipt, ReleasedBoxStep};
pub use pulse::{PulseDecline, PulseReceipt, RotationContactPulse};
use rotation::{InertiaBound, RotationBound};
pub use sweep::RigidBoxSweep;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoxTimingDecline {
    MultipleCandidates,
    UnsupportedGeometry,
    NonrigidTree,
    /// The named pair uses generalized multibody constraints; separation
    /// suppression is qualified only for the ordinary rigid-body solver.
    GeneralizedContact,
    ExternalConstraint,
    DrivenBody,
    InvalidInertia,
    /// Torque-free rotation uncertainty would exceed the allowed corner gap or arrival time.
    AngularTrajectoryUncertain,
    /// Multiple sole vertices have indistinguishable prospective arrival intervals.
    AmbiguousCornerTiming,
    MovingSupportRotation,
    OutsideSupportFace,
    OtherContact,
}
/// Bound on one isolated, approaching corner crossing in nanoseconds from the public start.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoxCrossing {
    /// Bottom vertex indexed by the two positive x/y bits (x carries weight two).
    pub corner: u8,
    pub measured_ns: u64,
    pub earliest_ns: u64,
    pub latest_ns: u64,
}
/// Committed native schedule during one public contact tick.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BoxTimingReceipt {
    pub durations: Vec<Duration>,
    /// A non-spherical tensor required torque-free angular bounds on this tick.
    pub bounded_rotation: bool,
    /// Number of inspected four-vertex cubic timing horizons.
    pub cubic_queries: u32,
    /// Horizons whose terminal gap interval contained zero without certifying arrival.
    pub uncertain_horizons: u32,
    pub crossings: Vec<BoxCrossing>,
    /// Attempts to bound a single native contact pulse from a new measured cut.
    pub pulses: Vec<PulseReceipt>,
    pub compact: Option<CompactImpactReceipt>,
    /// Per actually measured pulse, elapsed start and any reason its rigid point projection declined.
    pub point_attempts: Option<Vec<(u64, Option<RigidPointDecline>)>>,
    /// Start and duration of each successful immediate departure from a projected point.
    pub departure_calls: Option<Vec<(Duration, Duration)>>,
    /// Largest inspected height and time uncertainty at a prospective non-spherical crossing.
    pub max_gap_uncertainty_nm: u64,
    pub max_time_uncertainty_ns: u64,
    /// Number of durations differing from an even division of the remaining time.
    pub retimed: u32,
    /// Number of completed native calls before the first decline.
    pub decline_after: Option<u32>,
    pub decline: Option<BoxTimingDecline>,
}
struct Member {
    body: RigidBodyHandle,
    mass: f64,
}
#[derive(Clone, Copy)]
struct TimingCut {
    cubic_elapsed: Option<u64>,
    swept: bool,
    pulse: Option<(RotationContactPulse, usize, bool)>,
}
struct Candidate {
    foot: RigidBodyHandle,
    pad: RigidBodyHandle,
    sole: ColliderHandle,
    deck: ColliderHandle,
    opponent: Option<opponent::Opponent>,
    links: Vec<Member>,
    mass: f64,
    gravity_scale: f64,
    inertia: Option<InertiaBound>,
}
pub(super) struct BoxTiming {
    candidate: Option<Candidate>,
    pub(super) receipt: BoxTimingReceipt,
    pub(super) cubic: bool,
    pub(super) cached: Option<compact::GuardedCall>,
    pub(super) swept: bool,
    pub(super) ballistic: bool,
    pub(crate) point_pair: Option<(BodyId, BodyId)>,
    pub(crate) point_neighbors: bool,
    pub(crate) departure_witness: Option<(Duration, u8)>,
    pub(super) pulse: Option<(RotationContactPulse, usize)>,
    calls: u32,
    duration: Duration,
    fallback: Option<(NativeStepGrid, u32)>,
}
fn center(body: &RigidBody) -> Vector {
    *body.position() * body.mass_properties().local_mprops.local_com
}
impl Candidate {
    fn new(
        engine: &RapierBackend,
        foot: RigidBodyHandle,
        pad: RigidBodyHandle,
    ) -> Result<Self, BoxTimingDecline> {
        use BoxTimingDecline::*;
        let world = &engine.world;
        let f = &world.bodies[foot];
        let d = &world.bodies[pad];
        let ([sole], [deck]) = (f.colliders(), d.colliders()) else {
            return Err(UnsupportedGeometry);
        };
        if !f.is_dynamic()
            || !(d.is_fixed() || d.is_kinematic())
            || [*sole, *deck].iter().any(|&h| {
                let c = &world.colliders[h];
                !c.is_enabled() || c.is_sensor() || c.shape().as_cuboid().is_none()
            })
        {
            return Err(UnsupportedGeometry);
        }
        if d.angvel() != Vector::ZERO {
            return Err(MovingSupportRotation);
        }
        let handles = if let Some(link) = world.multibody_joints.rigid_body_link(foot) {
            let tree = world
                .multibody_joints
                .get_multibody(link.multibody)
                .ok_or(NonrigidTree)?;
            if tree.ndofs() != 6
                || tree.root().joint.ndofs() != 6
                || !tree.couplings().is_empty()
                || tree
                    .links()
                    .any(|l| l.joint.kinematic || !l.is_root() && l.joint.ndofs() != 0)
            {
                return Err(NonrigidTree);
            }
            tree.links().map(|l| l.rigid_body_handle()).collect()
        } else {
            vec![foot]
        };
        let mut links = Vec::with_capacity(handles.len());
        let mut com = Vector::ZERO;
        let mut mass = 0.;
        for h in handles {
            let b = &world.bodies[h];
            if !b.is_dynamic()
                || !b.is_enabled()
                || !b.locked_axes().is_empty()
                || world.impulse_joints_with(h).next().is_some()
            {
                return Err(ExternalConstraint);
            }
            if b.gravity_scale() != f.gravity_scale()
                || b.linear_damping() != 0.
                || b.angular_damping() != 0.
                || b.is_ccd_enabled()
            {
                return Err(DrivenBody);
            }
            let m = b.mass();
            if !(m.is_finite() && m > 0.) {
                return Err(InvalidInertia);
            }
            com += center(b) * m;
            mass += m;
            links.push(Member { body: h, mass: m });
        }
        com /= mass;
        let axes = [Vector::X, Vector::Y, Vector::Z];
        let tensor = axes.map(|axis| {
            let mut column = Vector::ZERO;
            for member in &links {
                let b = &world.bodies[member.body];
                let props = &b.mass_properties().local_mprops;
                let r = center(b) - com;
                let frame = *b.rotation() * props.principal_inertia_local_frame;
                column += frame * (props.inv_principal_inertia.recip() * (frame.inverse() * axis))
                    + member.mass * (axis * r.length_squared() - r * r.dot(axis));
            }
            column
        });
        let (mut lower, mut upper) = (0., 0.);
        for member in &links {
            let body = &world.bodies[member.body];
            let moments = body
                .mass_properties()
                .local_mprops
                .inv_principal_inertia
                .recip();
            lower += moments.min_element();
            upper += moments.max_element() + member.mass * (center(body) - com).length_squared();
        }
        let inertia = InertiaBound::from_world(tensor, *f.rotation(), lower, upper)?;
        Ok(Self {
            foot,
            pad,
            sole: *sole,
            deck: *deck,
            opponent: None,
            links,
            mass,
            gravity_scale: f.gravity_scale(),
            inertia,
        })
    }
    fn kinematics(
        &self,
        engine: &RapierBackend,
    ) -> Result<(Vector, Vector, Vector), BoxTimingDecline> {
        use BoxTimingDecline::*;
        let world = &engine.world;
        let d = &world.bodies[self.pad];
        if d.angvel() != Vector::ZERO {
            return Err(MovingSupportRotation);
        }
        let mut com = Vector::ZERO;
        let mut v = Vector::ZERO;
        let mut spin = Vector::ZERO;
        for m in &self.links {
            let b = &world.bodies[m.body];
            if b.user_force() != Vector::ZERO || b.user_torque() != Vector::ZERO {
                return Err(DrivenBody);
            }
            for &collider in b.colliders() {
                if world.narrow_phase.contact_pairs_with(collider).any(|p| {
                    let other = if p.collider1 == collider {
                        p.collider2
                    } else {
                        p.collider1
                    };
                    let named_zero = collider == self.sole
                        && self.opponent.is_some_and(|o| o.collider == other)
                        && p.total_impulse_magnitude() == 0.;
                    !(collider == self.sole && other == self.deck)
                        && !named_zero
                        && p.has_any_active_contact()
                }) {
                    return Err(OtherContact);
                }
            }
            com += center(b) * m.mass;
            let (linear, angular) = engine
                .generalized_link_twist(m.body)
                .unwrap_or_else(|| (b.linvel(), b.angvel()));
            v += linear * m.mass;
            if m.body == self.foot {
                spin = angular;
            }
        }
        com /= self.mass;
        v = v / self.mass - d.linvel();
        Ok((com, v, spin))
    }
    fn rotation_motion(&self, foot: &RigidBody, spin: Vector, window: f64) -> RotationBound {
        self.rotation_motion_from_cut(*foot.rotation(),spin,window)
    }
    fn proposed(
        &self,
        engine: &RapierBackend,
        ns: u64,
        left: u32,
        receipt: &mut BoxTimingReceipt,
        cut: TimingCut,
    ) -> Result<u64, BoxTimingDecline> {
        use BoxTimingDecline::*;
        let TimingCut {
            cubic_elapsed,
            swept,
            pulse,
        } = cut;
        let world = &engine.world;
        let nleft = u64::from(left);
        let share = ns / nleft;
        let f = &world.bodies[self.foot];
        let d = &world.bodies[self.pad];
        let (com, v, spin) = self.kinematics(engine)?;
        let sole = &world.colliders[self.sole];
        let deck = &world.colliders[self.deck];
        let fpose = *f.position() * sole.position_wrt_parent().ok_or(UnsupportedGeometry)?;
        let dpose = *d.position() * deck.position_wrt_parent().ok_or(UnsupportedGeometry)?;
        let h = sole
            .shape()
            .as_cuboid()
            .ok_or(UnsupportedGeometry)?
            .half_extents;
        let support = deck
            .shape()
            .as_cuboid()
            .ok_or(UnsupportedGeometry)?
            .half_extents;
        let normal = dpose.rotation * Vector::Z;
        let omega = spin.length();
        let acceleration = world.gravity * self.gravity_scale;
        let an = acceleration.dot(normal);
        let window = ns as f64 * 1e-9;
        let motion = self.rotation_motion(f, spin, window);
        if !motion.speed.is_finite() || !motion.acceleration.is_finite() {
            return Err(AngularTrajectoryUncertain);
        }
        if (fpose.rotation.inverse() * normal).z <= motion.speed * window {
            return Err(OutsideSupportFace);
        }
        // A late touchdown needs three distinct boundaries: the flight to the
        // event, the short contact solve, and the remaining actuator interval.
        // When only three calls remain, inspect their whole available horizon.
        let horizon_ns = if left == 3 {
            ns - (nleft - 1)
        } else {
            (ns - (nleft - 1)).min(share.saturating_mul(2))
        };
        let horizon = horizon_ns as f64 * 1e-9;
        let axis = if omega > 0. {
            spin / omega
        } else {
            Vector::ZERO
        };
        let jet = if cubic_elapsed.is_some() {
            if world.multibody_joints.rigid_body_link(self.foot).is_some() {
                return Err(GeneralizedContact);
            }
            if world.colliders.len() > 2 {
                let ball = sole.shape().compute_bounding_sphere(&fpose);
                let radius = ball.radius + ball.center.distance(com);
                let margin = world.integration_parameters.prediction_distance()
                    + radius * motion.speed * horizon
                    + 0.5 * acceleration.length() * horizon.powi(2)
                    + 512. * f64::EPSILON * (1. + com.length() + radius);
                let sweep = crate::approach_clearance::ApproachSweep {
                    pose: fpose,
                    velocity: v + d.linvel(),
                    dt: Duration::from_nanos(horizon_ns),
                    margin,
                };
                if self.clear_neighbors(engine, sole, sweep).is_none() {
                    return Err(OtherContact);
                }
            }
            receipt.cubic_queries += 1;
            Some(self.inertia.map_or(
                taylor::AngularJet {
                    alpha: Vector::ZERO,
                    beta: Vector::ZERO,
                    snap: omega.powi(4),
                },
                |i| i.jet(*f.rotation(), spin, horizon),
            ))
        } else {
            None
        };
        let mut post = pulse.and_then(|(mode, temporal, guarded)| {
            let cut = cubic_elapsed?;
            let witness = receipt.crossings.last()?;
            (witness.earliest_ns == cut)
                .then(|| pulse::PulseWindow::new(*witness, mode, temporal, cut, guarded))
        });
        let mut corners = [taylor::Crossing::Clear; 4];
        let mut best = f64::INFINITY;
        let mut touching = false;
        for x in [-h.x, h.x] {
            for y in [-h.y, h.y] {
                let point = fpose * Vector::new(x, y, -h.z);
                let arm = point - com;
                let local = dpose.inverse_transform_point(point);
                let radius = arm.length();
                let drift = (v.length() + motion.speed * radius) * window
                    + 0.5 * acceleration.length() * window * window;
                let uncertain_edge =
                    local.x.abs() + drift >= support.x || local.y.abs() + drift >= support.y;
                if local.z < -support.z
                    || uncertain_edge
                        && !(swept
                            && jet.is_some_and(|j| {
                                j.over_pad(
                                    taylor::MovingVertex {
                                        point,
                                        com,
                                        v,
                                        gravity: acceleration,
                                        spin,
                                    },
                                    dpose,
                                    support,
                                    horizon,
                                )
                            }))
                {
                    return Err(OutsideSupportFace);
                }
                let gap = local.z - support.z;
                if let Some(jet) = jet {
                    let gap = gap - sole.contact_skin() - deck.contact_skin();
                    let ix = usize::from(x > 0.) * 2 + usize::from(y > 0.);
                    let height = jet.height(
                        arm,
                        normal,
                        spin,
                        (v, acceleration),
                        gap,
                        point.length() + com.length() + dpose.translation.length(),
                    );
                    corners[ix] = height.classify(horizon);
                    if let Some(post) = post.as_mut() {
                        post.record_vertex(
                            ix,
                            height,
                            gap,
                            (support.x - local.x.abs()).min(support.y - local.y.abs()),
                            radius,
                        );
                    }
                    continue;
                }
                let speed = (v + spin.cross(arm)).dot(normal);
                let s = axis.cross(arm).dot(normal);
                let c = axis.cross(axis.cross(arm)).dot(normal);
                let bound = if self.inertia.is_some() {
                    // |d²(Rr)/dt²| <= r (|angular acceleration| + |spin|²).
                    (an.abs() + radius * (motion.acceleration + motion.speed * motion.speed))
                        * horizon
                } else {
                    (an.abs() + omega * omega * (s * s + c * c).sqrt()) * horizon
                };
                if speed + bound >= -0.01 {
                    if self.inertia.is_some()
                        && gap > 2e-8
                        && gap + (speed.min(0.) - 0.5 * bound) * horizon <= 0.
                    {
                        return Err(AngularTrajectoryUncertain);
                    }
                    continue;
                }
                if (-3e-4..=2e-8).contains(&gap) {
                    touching = true;
                    continue;
                }
                if gap <= 0. {
                    continue;
                }
                let at = |t: f64| {
                    let angle = omega * t;
                    gap + v.dot(normal) * t
                        + 0.5 * an * t * t
                        + s * angle.sin()
                        + 2. * c * (angle * 0.5).sin().powi(2)
                };
                // The attitude error of frozen world spin is at most
                // 0.5 |angular acceleration| t² in radians. Bracket the
                // earliest possible crossing of the lower height envelope.
                let curvature = 0.5 * motion.acceleration * radius;
                let lower_height = |t: f64| at(t) - curvature * t * t;
                if lower_height(horizon) > 0. {
                    continue;
                }
                if self.inertia.is_some() {
                    let gap_error = curvature * horizon * horizon;
                    // If a crossing falls at the horizon, these bounds also cover
                    // an event just beyond it. The early boundary rechecks live state.
                    let time_error = 2. * gap_error / (-(speed + bound));
                    receipt.max_gap_uncertainty_nm = receipt
                        .max_gap_uncertainty_nm
                        .max((gap_error * 1e9).ceil() as u64);
                    receipt.max_time_uncertainty_ns = receipt
                        .max_time_uncertainty_ns
                        .max((time_error * 1e9).ceil() as u64);
                    if gap_error > 0.25e-6 || time_error > 0.5e-6 {
                        return Err(AngularTrajectoryUncertain);
                    }
                }
                let (mut lo, mut hi) = (0., horizon);
                for _ in 0..22 {
                    let mid = 0.5 * (lo + hi);
                    if lower_height(mid) > 0. {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                best = best.min(lo);
            }
        }
        if let Some(post) = post {
            let record = post.finish(self, engine, share, (com, v, spin));
            receipt.pulses.push(record);
            if let Some(h) = record.applied_ns {
                return Ok(h);
            }
        }
        if let Some(elapsed) = cubic_elapsed {
            return taylor::first(corners, share, elapsed, receipt);
        }
        if touching {
            Ok(share.clamp(1, 4000))
        } else if best.is_finite() {
            Ok(((best * 1e9).floor() as u64).clamp(500.min(share), ns - nleft + 1))
        } else {
            Ok(share)
        }
    }
}
impl Candidate {
    fn rotation_motion_from_cut(
        &self,
        q: Rotation,
        spin: Vector,
        t: f64,
    ) -> RotationBound {
        self.inertia.map_or(
            RotationBound {
                speed: spin.length(),
                acceleration: 0.,
            },
            |i| i.motion(q.inverse() * spin, t),
        )
    }
}
impl BoxTiming {
    pub(super) fn enroll_opponent(&mut self, e: &RapierBackend, id: BodyId) {
        if let Some(c) = &mut self.candidate {
            c.enroll_opponent(e, id);
        }
    }

    pub(super) fn new(
        engine: &RapierBackend,
        dt: Duration,
        calls: u32,
        pairs: &[(BodyId, BodyId)],
    ) -> Result<Self, PhysicsError> {
        u64::try_from(dt.as_nanos()).map_err(|_| PhysicsError::InvalidValue)?;
        let mut receipt = BoxTimingReceipt::default();
        // Validate every public handle before any queued force or motor effort is consumed.
        for &(a, b) in pairs {
            if !engine.bodies.contains_key(&a) || !engine.bodies.contains_key(&b) {
                return Err(PhysicsError::MissingBody);
            }
        }
        let candidate = match pairs {
            [(foot, pad)] => {
                match Candidate::new(engine, engine.bodies[foot].0, engine.bodies[pad].0) {
                    Ok(c) => Some(c),
                    Err(reason) => {
                        receipt.decline = Some(reason);
                        None
                    }
                }
            }
            _ => {
                receipt.decline = Some(BoxTimingDecline::MultipleCandidates);
                None
            }
        };
        receipt.bounded_rotation = candidate.as_ref().is_some_and(|c| c.inertia.is_some());
        Ok(Self {
            candidate,
            receipt,
            calls,
            cubic: false,
            cached: None,
            swept: false,
            ballistic: false,
            point_pair: None,
            point_neighbors: false,
            departure_witness: None,
            pulse: None,
            duration: dt,
            fallback: None,
        })
    }
    pub(super) fn next(
        &mut self,
        engine: &RapierBackend,
        elapsed: Duration,
        completed: u32,
        previous_guarded: bool,
    ) -> Duration {
        let remaining = self.duration - elapsed;
        let left = self.calls - completed;
        let ns = remaining.as_nanos() as u64;
        let compact = if self.receipt.compact.is_some() {
            self.compact_next(
                engine,
                elapsed.as_nanos() as u64,
                completed,
                previous_guarded,
            )
        } else {
            None
        };
        let proposed = if compact.is_some() {
            compact
        } else if self.receipt.decline.is_some() {
            None
        } else if left == 1 {
            Some(ns)
        } else {
            match self
                .candidate
                .as_ref()
                .expect("qualified candidate")
                .proposed(
                    engine,
                    ns,
                    left,
                    &mut self.receipt,
                    TimingCut {
                        cubic_elapsed: self.cubic.then_some(elapsed.as_nanos() as u64),
                        swept: self.swept,
                        pulse: self
                            .pulse
                            .map(|(mode, temporal)| (mode, temporal, previous_guarded)),
                    },
                ) {
                Ok(t) => Some(t),
                Err(reason) => {
                    self.receipt.decline = Some(reason);
                    None
                }
            }
        };
        let h = if let Some(t) = proposed {
            self.receipt.retimed += u32::from(t != ns / u64::from(left));
            Duration::from_nanos(t)
        } else {
            self.receipt.decline_after.get_or_insert(completed);
            let (grid, start) = self.fallback.get_or_insert_with(|| {
                (
                    NativeStepGrid::new(remaining, left)
                        .expect("remaining calls have positive durations"),
                    completed,
                )
            });
            grid.interval(completed - *start)
                .expect("validated fallback budget")
        };
        self.receipt.durations.push(h);
        h
    }
}
