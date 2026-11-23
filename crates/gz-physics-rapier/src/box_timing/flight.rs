// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Hide only a named native contact pair during a strictly separated rigid flight.
use super::*;
use crate::approach_clearance::ApproachSweep;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ReleasedBoxStep {
    /// True only after checking the complete interval AND preparing an accepted rigid-top update.
    pub integrated: bool,
    pub corner_spans: u16,
    /// A rigorous lower bound on all four normal gaps, rounded down to whole picometers.
    pub minimum_gap_pm: Option<u64>,
}
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BoxFlightReceipt {
    /// One entry per actual native call, including conservative declines.
    pub guarded: Vec<bool>,
    pub durations: Vec<Duration>,
    /// Per-call clearance for the explicitly registered third body.
    pub opposing: Option<Vec<Option<OpponentClearance>>>,
    /// Per-call qualification of the optional upper impact/flight, including the event call.
    pub opposing_corner_calls: Option<Vec<bool>>,
    /// Accepted fourth-order rigid-top subdivisions per native solve when requested.
    /// Zero leaves native angular evolution in that solve.
    pub rotation_slices: Option<Vec<u16>>,
    /// True for each accepted finite-gravity COM update from the actual starting state.
    pub ballistic_steps: Option<Vec<bool>>,
    /// Explicit, post-load departure checks per native call.
    pub released: Option<Vec<ReleasedBoxStep>>,
    /// Static qualification decline for the candidate set or initial body geometry.
    pub decline: Option<BoxTimingDecline>,
}
pub(crate) struct FlightGuard {
    own: Option<Candidate>,
    shared: bool,
    swept: bool,
    pub receipt: BoxFlightReceipt,
    pub(crate) auxiliary: Option<opponent::OpponentGuard>,
}
impl FlightGuard {
    pub(crate) fn new(
        e: &RapierBackend,
        pairs: &[(BodyId, BodyId)],
        shared: Option<&BoxTiming>,
        rotate: bool,
        released: bool,
        swept: bool,
        ballistic: bool,
    ) -> Result<Self, PhysicsError> {
        for &(a, b) in pairs {
            if !e.bodies.contains_key(&a) || !e.bodies.contains_key(&b) {
                return Err(PhysicsError::MissingBody);
            }
        }
        let mut receipt = BoxFlightReceipt {
            rotation_slices: rotate.then(Vec::new),
            ballistic_steps: ballistic.then(Vec::new),
            released: released.then(Vec::new),
            ..Default::default()
        };
        // Rapier's regular and generalized constraints update speculative
        // distances differently during relaxation. The early reaction occurs
        // in the regular rigid-body path. Leave generalized contact history intact.
        if let [(a, _)] = pairs
            && e.world
                .multibody_joints
                .rigid_body_link(e.bodies[a].0)
                .is_some()
        {
            return Ok(Self {
                own: None,
                auxiliary: None,
                shared: false,
                swept,
                receipt: BoxFlightReceipt {
                    decline: Some(BoxTimingDecline::GeneralizedContact),
                    ..receipt
                },
            });
        }

        let own = match (pairs, shared) {
            ([_], Some(t)) => {
                receipt.decline = t.receipt.decline;
                None
            }
            ([(a, b)], None) => match Candidate::new(e, e.bodies[a].0, e.bodies[b].0) {
                Ok(c) => Some(c),
                Err(reason) => {
                    receipt.decline = Some(reason);
                    None
                }
            },
            _ => {
                receipt.decline = Some(BoxTimingDecline::MultipleCandidates);
                None
            }
        };
        Ok(Self {
            own,
            auxiliary: None,
            shared: shared.is_some(),
            swept,
            receipt,
        })
    }
    pub(crate) fn enroll_opponent(&mut self, e: &RapierBackend, id: BodyId) {
        if let Some(c) = self.own.as_mut() {
            c.enroll_opponent(e, id);
        }
        self.receipt.opposing = Some(Vec::new());
    }
    pub(crate) fn projected_contact(&mut self, h: Duration) {
        self.auxiliary = None;
        self.receipt.durations.push(h);
        self.receipt.guarded.push(false);
        if let Some(s) = self.receipt.rotation_slices.as_mut() {
            s.push(0);
        }
        if let Some(b) = self.receipt.ballistic_steps.as_mut() {
            b.push(false);
        }
        if let Some(r) = self.receipt.released.as_mut() {
            r.push(ReleasedBoxStep::default());
        }
    }
    pub(crate) fn prepared(
        &mut self,
        h: Duration,
        call: compact::GuardedCall,
    ) -> (
        Option<(ColliderHandle, ColliderHandle)>,
        Option<GuardedMotion>,
    ) {
        self.receipt.durations.push(h);
        self.receipt.guarded.push(true);
        self.auxiliary = call.opposing;
        if let Some(s) = self.receipt.rotation_slices.as_mut() {
            s.push(call.motion.slices());
        }
        if let Some(b) = self.receipt.ballistic_steps.as_mut() {
            b.push(call.motion.ballistic());
        }
        if let Some(released) = self.receipt.released.as_mut() {
            released.push(call.release.unwrap_or_default());
        }
        (
            Some(call.pair),
            call.motion.present().then_some(call.motion),
        )
    }
    pub(crate) fn during(
        &mut self,
        e: &RapierBackend,
        dt: Duration,
        clock: Option<&BoxTiming>,
        other_action: bool,
    ) -> (
        Option<(ColliderHandle, ColliderHandle)>,
        Option<GuardedMotion>,
    ) {
        let c = if self.shared {
            clock.and_then(|c| c.candidate.as_ref())
        } else {
            self.own.as_ref()
        };
        let cubic = self.shared && clock.is_some_and(|t| t.cubic);
        let mut release = self
            .receipt
            .released
            .is_some()
            .then(ReleasedBoxStep::default);
        let ballistic = self.receipt.ballistic_steps.is_some();
        let mut opposing = None;
        let mut certified = c.filter(|c| {
            !other_action
                && c.separated(
                    e,
                    dt,
                    release.as_mut(),
                    &mut opposing,
                    SeparationCheck {
                        cubic,
                        swept: self.swept,
                        ballistic,
                        from_corner: None,
                    },
                )
        });
        self.receipt.durations.push(dt);
        let rotate = self.receipt.rotation_slices.is_some();
        let motion = certified
            .filter(|_| rotate || ballistic)
            .map(|c| GuardedMotion::prepare(c, e, dt, rotate, ballistic))
            .unwrap_or_default();
        if ballistic
            && certified.is_some_and(|c| e.world.gravity * c.gravity_scale != Vector::ZERO)
            && !motion.ballistic()
        {
            certified = None;
        }
        if let Some(slices) = self.receipt.rotation_slices.as_mut() {
            slices.push(motion.slices());
        }
        if let Some(b) = self.receipt.ballistic_steps.as_mut() {
            b.push(motion.ballistic());
        }
        if let Some(mut record) = release {
            record.integrated = motion.present() && certified.is_some();
            if !record.integrated {
                record.minimum_gap_pm = None;
                certified = None;
            }
            self.receipt
                .released
                .as_mut()
                .expect("explicit release mode")
                .push(record);
        }
        self.auxiliary = certified.and(opposing);
        self.receipt.guarded.push(certified.is_some());
        (
            certified.map(|c| (c.sole, c.deck)),
            (certified.is_some() && motion.present()).then_some(motion),
        )
    }
}
#[derive(Clone, Copy)]
pub(super) struct SeparationCheck {
    pub cubic: bool,
    pub swept: bool,
    pub ballistic: bool,
    // Only the immediate continuation after an applied point law may supply this vertex.
    pub from_corner: Option<u8>,
}
impl Candidate {
    pub(super) fn separated(
        &self,
        e: &RapierBackend,
        dt: Duration,
        mut released: Option<&mut ReleasedBoxStep>,
        opposing: &mut Option<opponent::OpponentGuard>,
        check: SeparationCheck,
    ) -> bool {
        let SeparationCheck {
            cubic,
            swept,
            ballistic,
            from_corner,
        } = check;
        let w = &e.world;
        let Ok((com, v, spin)) = self.kinematics(e) else {
            return false;
        };
        let t = dt.as_secs_f64();
        let f = &w.bodies[self.foot];
        let d = &w.bodies[self.pad];
        let a = &w.colliders[self.sole];
        let deck = &w.colliders[self.deck];
        let Some((fl, dl)) = a.position_wrt_parent().zip(deck.position_wrt_parent()) else {
            return false;
        };
        let fp = *f.position() * fl;
        let dp = *d.position() * dl;
        let (Some(sole), Some(pad)) = (a.shape().as_cuboid(), deck.shape().as_cuboid()) else {
            return false;
        };
        let normal = dp.rotation * Vector::Z;
        let omega = spin.length();
        let mut motion = self.rotation_motion(f, spin, t);
        motion.speed = motion
            .speed
            .min((omega + motion.acceleration * t) * (1. + 128. * f64::EPSILON));
        if !motion.speed.is_finite()
            || !motion.acceleration.is_finite()
            || (fp.rotation.inverse() * normal).z <= motion.speed * t
        {
            return false;
        }
        let (alpha, beta) = self.inertia.map_or((Vector::ZERO, 0.), |i| {
            i.changing_acceleration(*f.rotation(), spin, motion)
        });
        let angular_jerk = beta + 3. * motion.speed * motion.acceleration + motion.speed.powi(3);
        let gravity = w.gravity * self.gravity_scale;
        let an = gravity.dot(normal);
        let top = dp.translation + normal * pad.half_extents.z;
        let axis = if omega > 0. {
            spin / omega
        } else {
            Vector::ZERO
        };
        let rotation = omega * t;
        let (sin, cosc) = (rotation.sin(), 2. * (0.5 * rotation).sin().powi(2));
        let skin = a.contact_skin() + deck.contact_skin();
        let eps =
            128. * f64::EPSILON * (1. + com.length() + top.length() + fp.translation.length());
        let jet = (cubic || released.is_some() || swept).then(|| {
            self.inertia.map_or(
                taylor::AngularJet {
                    alpha: Vector::ZERO,
                    beta: Vector::ZERO,
                    snap: omega.powi(4),
                },
                |i| i.jet(*f.rotation(), spin, t),
            )
        });
        let h = sole.half_extents;
        // The local top face stays above the bottom for this entire interval.
        for x in [-h.x, h.x] {
            for y in [-h.y, h.y] {
                let point = fp * Vector::new(x, y, -h.z);
                let r = point - com;
                let radius = r.length();
                let gap = (point - top).dot(normal) - skin;
                let corner = 2 * u8::from(x > 0.) + u8::from(y > 0.);
                if from_corner == Some(corner) {
                    // The last committed solve followed the known impact from tangency to
                    // this strictly separated state. Its positive gap may still round to zero.
                    // Require a resolvable outward speed now and outward acceleration for this
                    // whole next interval. Every other vertex must pass its ordinary clearance.
                    let Some(jet) = jet else {
                        return false;
                    };
                    let rate = (v + spin.cross(r)).dot(normal);
                    let rate_floor =
                        1024. * f64::EPSILON * (1. + v.length() + spin.length() * radius);
                    if !gap.is_finite()
                        || gap < -eps
                        || !rate.is_finite()
                        || rate <= rate_floor
                        || jet
                            .height(
                                r,
                                normal,
                                spin,
                                (v, gravity),
                                gap,
                                com.length() + top.length() + point.length(),
                            )
                            .outward_acceleration(t)
                            .is_none()
                    {
                        return false;
                    }
                    if let Some(record) = released.as_mut() {
                        record.corner_spans += 1;
                        record.minimum_gap_pm = Some(0);
                    }
                    continue;
                }
                if gap <= eps {
                    return false;
                }
                if let Some(record) = released.as_mut() {
                    let height = jet.expect("departure jet").height(
                        r,
                        normal,
                        spin,
                        (v, gravity),
                        gap,
                        point.length() + com.length() + dp.translation.length(),
                    );
                    let Some((minimum, n)) =
                        height.released_span(t, if ballistic { 0. } else { an })
                    else {
                        return false;
                    };
                    record.corner_spans += n;
                    let picometers = (minimum * 1e12).floor() as u64;
                    record.minimum_gap_pm = Some(
                        record
                            .minimum_gap_pm
                            .map_or(picometers, |q| q.min(picometers)),
                    );
                    continue;
                }
                if swept
                    && jet.is_some_and(|j| {
                        j.height(
                            r,
                            normal,
                            spin,
                            (v, gravity),
                            gap,
                            point.length() + com.length() + dp.translation.length(),
                        )
                        .released_span(t, if ballistic { 0. } else { an })
                        .is_some()
                    })
                {
                    continue;
                }
                let speed = (v + spin.cross(r)).dot(normal);
                let curvature =
                    an.abs() + radius * (motion.acceleration + motion.speed * motion.speed);
                let lower = if speed + curvature * t <= 0. {
                    // Monotone flight: freeze the measured world spin, then subtract
                    // the entire bounded angular departure over this specific duration.
                    let s = axis.cross(r).dot(normal);
                    let c = axis.cross(axis.cross(r)).dot(normal);
                    gap + v.dot(normal) * t + 0.5 * an * t * t + s * sin + c * cosc
                        - 0.5 * motion.acceleration * radius * t * t
                } else if speed - curvature * t >= 0. {
                    gap
                } else {
                    gap + speed.min(0.) * t - 0.5 * curvature * t * t
                };
                // Use the actual boundary acceleration as an independent enclosure.
                // The third derivative of R(t)r is bounded by
                // |r| (|ω̈| + 3 |ω| |ω̇| + |ω|³). The minimum of the measured
                // quadratic over the entire span is at an endpoint or its vertex.
                let acc = an + (alpha.cross(r) + spin.cross(spin.cross(r))).dot(normal);
                let u = if acc > 0. {
                    (-speed / acc).clamp(0., t)
                } else if speed + 0.5 * acc * t < 0. {
                    t
                } else {
                    0.
                };
                let quadratic_lower =
                    gap + u * (speed + 0.5 * acc * u) - radius * angular_jerk * t.powi(3) / 6.;
                let lower = lower.max(quadratic_lower);
                if (!lower.is_finite() || lower <= eps)
                    && !jet.is_some_and(|j| {
                        j.height(
                            r,
                            normal,
                            spin,
                            (v, gravity),
                            gap,
                            point.length() + com.length() + dp.translation.length(),
                        )
                        .strictly_separated(t)
                    })
                {
                    return false;
                }
            }
        }
        let allowance = w.integration_parameters.prediction_distance() + eps;
        for link in &self.links {
            let b = &w.bodies[link.body];
            for cid in b.colliders() {
                let coll = &w.colliders[*cid];
                let Some(rel) = coll.position_wrt_parent() else {
                    return false;
                };
                let pose = *b.position() * rel;
                let ball = coll.shape().compute_bounding_sphere(&pose);
                let radius = ball.radius + ball.center.distance(com);
                let acceleration_travel = if released.is_some() || ballistic {
                    gravity.length() * t * t
                } else {
                    0.5 * gravity.length() * t * t
                };
                let extra = radius * motion.speed * t + acceleration_travel;
                if *cid != self.sole
                    && crate::collision::can_exchange_impulses(coll, deck, d.body_type())
                {
                    let lowest = coll.shape().compute_aabb(&(dp.inverse() * pose)).mins.z
                        - pad.half_extents.z;
                    if lowest + v.dot(normal).min(0.) * t - extra
                        <= allowance + coll.contact_skin() + deck.contact_skin()
                    {
                        return false;
                    }
                }
                let sweep = ApproachSweep {
                    pose,
                    velocity: v + d.linvel(),
                    dt,
                    margin: allowance + extra,
                };
                let Some(cleared) = self.clear_neighbors(e, coll, sweep) else {
                    return false;
                };
                if let Some(named) = cleared {
                    *opposing = Some(named);
                }
            }
        }
        true
    }
}
