// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measured impact of the free cuboid's upper vertex on a second finite box.
//! Both pairs must be isolated for the complete native interval.
use super::*;
#[path = "opposing_normals.rs"]
mod normals;
#[path = "upper_return.rs"]
mod sequence;
pub use normals::{
    OpposingLimitStage, OpposingNormalDecline, OpposingNormalEvent, OpposingNormalReceipt,
    OpposingNormalStage,
};
pub use sequence::OpposingReturnReceipt;
pub(crate) use sequence::PriorUpper;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpposingCornerReach {
    Unavailable,
    Distant,
    Nearby,
    Possible,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OpposingCornerReceipt {
    pub point: RigidPointImpactReceipt,
    /// Native foot-origin poses and COM/world twists immediately before and after the impulse.
    pub incoming: BodyState,
    pub outgoing: BodyState,
    /// Uncertainty of the contact time from signed gap and strictly positive approach-speed bound.
    pub timing_error_s: f64,
    pub minimum_other_gap_m: f64,
    pub minimum_footprint_margin_m: f64,
}
pub(crate) struct UpperPair {
    c: Candidate,
    ids: (BodyId, BodyId, BodyId),
    gate: RigidBodyHandle,
    roof: ColliderHandle,
}
pub(crate) struct UpperStep {
    pub pair: (ColliderHandle, ColliderHandle),
    pub ceiling: ColliderHandle,
    pub receipt: Option<OpposingCornerReceipt>,
    pub returned: Option<Box<OpposingReturnReceipt>>,
    pub normals: Option<Box<OpposingNormalReceipt>>,
    body: RigidBodyHandle,
    end: Cut,
}
impl UpperStep {
    pub fn commit(self, e: &mut RapierBackend) -> Option<OpposingCornerReceipt> {
        RotationUpdate::new(self.body, self.end.q, self.end.spin)
            .commit_with_translation(&mut e.world, Some((self.end.com, self.end.v)));
        self.receipt
    }
}
#[derive(Clone, Copy)]
struct Deck {
    pose: Pose,
    top: Vector,
    n: Vector,
    v: Vector,
    half: Vector,
}
impl Deck {
    fn at(self, t: f64) -> Self {
        Self {
            pose: Pose::from_parts(self.pose.translation + self.v * t, self.pose.rotation),
            top: self.top + self.v * t,
            ..self
        }
    }
}
#[derive(Clone, Copy)]
struct Incoming {
    rotor: Rotor,
    state: Cut,
    decks: [Deck; 2],
    // In body coordinates about the native center of mass.
    arms: [Vector; 8],
    friction: f64,
    tol: f64,
}
impl Rotor {
    fn bounded(&self, s: Cut, h: f64, depth: u8) -> Option<Cut> {
        if let Ok(end) = self.at(s, h) {
            return Some(end);
        }
        if depth == 0 {
            return None;
        }
        let mid = self.bounded(s, h * 0.5, depth - 1)?;
        self.bounded(mid, h * 0.5, depth - 1)
    }
}
struct Branch {
    elapsed: Duration,
    dt: Duration,
    separated: bool,
    lower_after: Option<u8>,
}
impl UpperPair {
    pub(crate) fn new(e: &RapierBackend, ids: (BodyId, BodyId, BodyId)) -> Option<Self> {
        if e.world.colliders.len() != 3 {
            return None;
        }
        let mut c = Candidate::new(e, e.bodies.get(&ids.0)?.0, e.bodies.get(&ids.1)?.0).ok()?;
        if c.links.len() != 1 {
            return None;
        }
        c.enroll_opponent(e, ids.2);
        let opponent = c.opponent?;
        Some(Self {
            c,
            ids,
            gate: opponent.body,
            roof: opponent.collider,
        })
    }
    fn incoming(&self, e: &RapierBackend) -> Option<Incoming> {
        let c = &self.c;
        let w = &e.world;
        let f = &w.bodies[c.foot];
        let low = &w.bodies[c.pad];
        let high = &w.bodies[self.gate];
        let sole = &w.colliders[c.sole];
        let floor = &w.colliders[c.deck];
        let roof = &w.colliders[self.roof];
        if !f.is_enabled()
            || f.is_sleeping()
            || !f.is_dynamic()
            || f.user_force() != Vector::ZERO
            || f.user_torque() != Vector::ZERO
            || (!f.gyroscopic_forces_enabled() && c.inertia.is_some() && f.angvel() != Vector::ZERO)
            || e.world.multibody_joints.rigid_body_link(c.foot).is_some()
            || e.world.impulse_joints_with(c.foot).next().is_some()
            || [low, high].iter().any(|b| {
                !b.is_enabled()
                    || b.angvel() != Vector::ZERO
                    || !matches!(
                        b.body_type(),
                        RigidBodyType::Fixed | RigidBodyType::KinematicVelocityBased
                    )
            })
            || [sole, floor, roof]
                .iter()
                .any(|coll| !coll.is_enabled() || coll.is_sensor() || coll.contact_skin() != 0.)
            || !crate::collision::can_exchange_impulses(sole, floor, low.body_type())
            || !crate::collision::can_exchange_impulses(sole, roof, high.body_type())
            || CoefficientCombineRule::combine(
                sole.restitution(),
                roof.restitution(),
                sole.restitution_combine_rule(),
                roof.restitution_combine_rule(),
            ) != 0.
            || [(c.sole, c.deck), (c.sole, self.roof)]
                .iter()
                .any(|&(a, b)| {
                    w.narrow_phase
                        .contact_pair(a, b)
                        .is_some_and(|p| p.total_impulse_magnitude() > 0.)
                })
        {
            return None;
        }
        let prop = &f.mass_properties().local_mprops;
        let rotor = Rotor {
            axes: prop.principal_inertia_local_frame,
            inv: prop.inv_principal_inertia,
            acc: w.gravity * c.gravity_scale,
        };
        let local = sole.position_wrt_parent()?;
        let half = sole.shape().as_cuboid()?.half_extents;
        let arms = std::array::from_fn(|k| {
            let p = Vector::new(
                if k & 2 == 0 { -half.x } else { half.x },
                if k & 1 == 0 { -half.y } else { half.y },
                if k & 4 == 0 { -half.z } else { half.z },
            );
            *local * p - prop.local_com
        });
        let deck = |b: &RigidBody, s: &Collider, sign: f64| {
            let pose = *b.position() * s.position_wrt_parent()?;
            let half = s.shape().as_cuboid()?.half_extents;
            let n = sign * (pose.rotation * Vector::Z);
            Some(Deck {
                top: pose.translation + n * half.z,
                pose,
                n,
                v: b.linvel(),
                half,
            })
        };
        let decks = [deck(low, floor, 1.)?, deck(high, roof, -1.)?];
        if decks[0].n.dot(decks[1].n) > -0.95 {
            return None;
        }
        let state = Cut {
            q: *f.rotation(),
            com: center(f),
            v: f.linvel(),
            spin: f.angvel(),
        };
        let scale =
            1. + state.com.length() + decks[0].top.length() + decks[1].top.length() + half.length();
        let tol = 512. * f64::EPSILON * scale;
        let friction = CoefficientCombineRule::combine(
            sole.friction(),
            roof.friction(),
            sole.friction_combine_rule(),
            roof.friction_combine_rule(),
        );
        (rotor.inv.is_finite()
            && rotor.inv.min_element() > 0.
            && tol.is_finite()
            && friction.is_finite()
            && friction >= 0.)
            .then_some(Incoming {
                rotor,
                state,
                decks,
                arms,
                friction,
                tol,
            })
    }
    pub(crate) fn reach(&self, e: &RapierBackend, dt: Duration) -> OpposingCornerReach {
        use OpposingCornerReach::*;
        if dt.is_zero() || dt > Duration::from_millis(1) {
            return Unavailable;
        }
        let Some(i) = self.incoming(e) else {
            return Unavailable;
        };
        let t = dt.as_secs_f64();
        let d = i.decks[1];
        let s = i.state;
        let jet = self.jet(s, t);
        let mut gap = f64::INFINITY;
        let mut possible = false;
        for local in &i.arms[4..] {
            let r = s.q * (*local);
            let g = (s.com + r - d.top).dot(d.n);
            gap = gap.min(g);
            if g < -i.tol {
                return Unavailable;
            }
            let z = jet.height(
                r,
                d.n,
                s.spin,
                (s.v - d.v, i.rotor.acc),
                g - i.tol,
                s.com.length() + d.top.length(),
            );
            if !z.strictly_separated(t) {
                possible = true;
            }
        }
        if possible {
            Possible
        } else if gap <= e.world.integration_parameters.prediction_distance() {
            Nearby
        } else {
            Distant
        }
    }
    fn jet(&self, s: Cut, t: f64) -> taylor::AngularJet {
        self.c.inertia.map_or(
            taylor::AngularJet {
                alpha: Vector::ZERO,
                beta: Vector::ZERO,
                snap: s.spin.length().powi(4),
            },
            |i| i.jet(s.q, s.spin, t),
        )
    }
    /// Preserve a truly separating measured tangency across a public boundary.
    /// Uncertain closing velocities and loaded native contacts decline.
    fn away(
        s: Cut,
        r: Vector,
        d: Deck,
        g: f64,
        jet: taylor::AngularJet,
        i: &Incoming,
        h: f64,
    ) -> bool {
        let speed = (s.v - d.v + s.spin.cross(r)).dot(d.n);
        g >= -i.tol
            && speed >= 0.
            && jet
                .height(
                    r,
                    d.n,
                    s.spin,
                    (s.v - d.v, i.rotor.acc),
                    0.,
                    s.com.length() + d.top.length(),
                )
                .outward_acceleration(h)
                .is_some()
    }
    fn footprint(i: &Incoming, s: Cut, deck: Deck, r: Vector, h: f64, speed: f64) -> Option<f64> {
        let point = s.com + r;
        let local = deck.pose.inverse_transform_point(point);
        let travel =
            ((s.v - deck.v).length() + r.length() * speed + 0.5 * i.rotor.acc.length() * h) * h;
        let margin =
            (deck.half.x - local.x.abs()).min(deck.half.y - local.y.abs()) - travel - i.tol;
        (margin.is_finite() && margin > 0.).then_some(margin)
    }
    // Exactly one upper vertex may arrive. All lower vertices and all other
    // upper vertices must have strictly positive gap throughout this branch.
    fn isolated(
        &self,
        i: &Incoming,
        s: Cut,
        base: f64,
        h: f64,
        active: Option<usize>,
        released: Option<(u8, bool)>,
    ) -> Option<(f64, f64)> {
        let jet = self.jet(s, h + if released.is_some() { 1e-9 } else { 0. });
        let speed = self.c.rotation_motion_from_cut(s.q, s.spin, h).speed;
        let mut gap = f64::INFINITY;
        let mut margin = f64::INFINITY;
        for (side, d) in i.decks.iter().enumerate() {
            let d = d.at(base);
            for (k, local) in i.arms.iter().enumerate() {
                let r = s.q * (*local);
                margin = margin.min(Self::footprint(i, s, d, r, h, speed)?);
                if side == 1 && Some(k) == active {
                    continue;
                }
                // The public solver names impact times at integer nanoseconds.
                // Two contacts inside one clock quantum cannot be ordered
                // reliably, even with a tighter fractional root fit.
                let geometric = (s.com + r - d.top).dot(d.n);
                let was_upper = side == 0 && released.is_some_and(|(id, _)| id as usize == k);
                let uncertainty = if was_upper {
                    jet.height(
                        r,
                        d.n,
                        s.spin,
                        (s.v - d.v, i.rotor.acc),
                        geometric,
                        s.com.length() + d.top.length(),
                    )
                    .clock_margin(h, 1e-9)
                        + i.tol
                } else if active.is_some() {
                    1e-9 * ((s.v - d.v).length() + speed * r.length() + i.rotor.acc.length() * h)
                        + i.tol
                } else {
                    0.
                };
                let g = geometric - uncertainty;
                let relative = (s.v - d.v, i.rotor.acc);
                let z = jet.height(
                    r,
                    d.n,
                    s.spin,
                    relative,
                    g - i.tol,
                    s.com.length() + d.top.length(),
                );
                let departing = side == 0
                    && released == Some((k as u8, true))
                    && Self::away(s, r, d, g + uncertainty, jet, i, h);
                if departing {
                    continue;
                }
                let min = if h == 0. {
                    (g > 2. * i.tol).then_some(g - i.tol)?
                } else if let Some((min, _)) = z.released_span(h, 0.) {
                    min
                } else if active.is_none() && side == 1 && Self::away(s, r, d, g, jet, i, h) {
                    0.
                } else {
                    return None;
                };
                gap = gap.min(min);
            }
        }
        Some((gap, margin))
    }
    pub(crate) fn prepare(
        &self,
        e: &RapierBackend,
        elapsed: Duration,
        dt: Duration,
        separated_approach: bool,
    ) -> Option<UpperStep> {
        if dt.is_zero() || dt > Duration::from_millis(1) {
            return None;
        }
        self.branch(
            e,
            self.incoming(e)?,
            Branch {
                elapsed,
                dt,
                separated: separated_approach,
                lower_after: None,
            },
        )
    }
    fn branch(&self, e: &RapierBackend, i: Incoming, request: Branch) -> Option<UpperStep> {
        self.branch_end(e, i, request, false)
    }
    fn isolated_fractional_prefix(
        &self,
        i: &Incoming,
        start: Cut,
        h: f64,
        k: usize,
    ) -> Option<(f64, f64)> {
        // All other material points must stay strictly clear until
        // the latest possible arrival. Only the named cold corner
        // may touch inside this measured prefix.
        let jet = self.jet(start, h);
        let speed = self
            .c
            .rotation_motion_from_cut(start.q, start.spin, h)
            .speed;
        let mut gap = f64::INFINITY;
        let mut margin = f64::INFINITY;
        for (side, d) in i.decks.iter().enumerate() {
            for (index, &arm) in i.arms.iter().enumerate() {
                let r = start.q * arm;
                margin = margin.min(Self::footprint(i, start, *d, r, h, speed)?);
                if side == 1 && index == k {
                    continue;
                }
                let g = (start.com + r - d.top).dot(d.n) - i.tol;
                let z = jet.height(
                    r,
                    d.n,
                    start.spin,
                    (start.v - d.v, i.rotor.acc),
                    g,
                    start.com.length() + d.top.length(),
                );
                gap = gap.min(z.released_span(h, 0.)?.0);
            }
        }
        Some((gap, margin))
    }
    /// The two-site path resolves later contacts itself with fractional-time
    /// certificates. Its initial measured impulse consumes no rounded native suffix.
    fn branch_to_measured_impact(
        &self,
        e: &RapierBackend,
        i: Incoming,
        request: Branch,
    ) -> Option<UpperStep> {
        self.branch_end(e, i, request, true)
    }
    fn branch_end(
        &self,
        e: &RapierBackend,
        i: Incoming,
        branch: Branch,
        stop_at_impact: bool,
    ) -> Option<UpperStep> {
        let Branch {
            elapsed,
            dt,
            separated: separated_approach,
            lower_after,
        } = branch;
        let h = dt.as_secs_f64();
        let start = i.state;
        let deck = i.decks[1];
        let jet = self.jet(start, h);
        // Only enter the isolated path inside the native predictive reach or
        // for an actual possible crossing during this call.
        let mut close = false;
        let mut candidate = None;
        let mut end = None;
        for k in 4..8 {
            let arm = start.q * i.arms[k];
            let g = (start.com + arm - deck.top).dot(deck.n);
            let z = jet.height(
                arm,
                deck.n,
                start.spin,
                (start.v - deck.v, i.rotor.acc),
                g - i.tol,
                start.com.length() + deck.top.length(),
            );
            close |= g <= e.world.integration_parameters.prediction_distance();
            if z.strictly_separated(h) || Self::away(start, arm, deck, g, jet, &i, h) {
                continue;
            }
            if g < -i.tol || candidate.is_some() {
                return None;
            }
            let speed = z.closing_speed(h)?;
            let finish = if let Some(s) = end {
                s
            } else {
                i.rotor.bounded(start, h, 5)?
            };
            end = Some(finish);
            let surface = Surface {
                arm: i.arms[k],
                top: deck.top,
                velocity: deck.v,
                n: deck.n,
            };
            let terminal = surface.gap(finish, h);
            let root = if g.abs() <= i.tol {
                0.
            } else {
                if terminal > i.tol || terminal > 0. && terminal / speed > 0.025e-9 {
                    return None;
                }
                if terminal >= 0. {
                    h
                } else {
                    // A strictly closing orbit brackets the first root. Use the
                    // measured point velocity to refine the fraction; bisection
                    // retains the bracket when curvature would take Newton outside.
                    let (mut lo, mut hi) = (0., h);
                    let mut t = h * g / (g - terminal);
                    for _ in 0..40 {
                        let x = i.rotor.bounded(start, t, 5)?;
                        let gap = surface.gap(x, t);
                        if gap.abs() <= i.tol / 16. + speed * 1e-15 {
                            break;
                        }
                        if gap > 0. {
                            lo = t
                        } else {
                            hi = t
                        }
                        if hi - lo <= 2e-14 {
                            t = (hi + lo) * 0.5;
                            break;
                        }
                        let derivative = surface.incoming(x).dot(surface.n);
                        let next = t - gap / derivative;
                        t = if next > lo && next < hi {
                            next
                        } else {
                            0.5 * (lo + hi)
                        };
                    }
                    t
                }
            };
            let cut = i.rotor.bounded(start, root, 5)?;
            let eps = (surface.gap(cut, root).abs() + i.tol) / speed;
            if eps > 0.1e-9 {
                return None;
            }
            candidate = Some((k, surface, root, cut, eps));
        }
        let (end, receipt) = if let Some((k, surface, root, cut, error)) = candidate {
            let (g0, m0) = if stop_at_impact {
                self.isolated_fractional_prefix(&i, start, root + error, k)?
            } else {
                self.isolated(
                    &i,
                    start,
                    0.,
                    root,
                    Some(k),
                    lower_after.map(|id| (id, true)),
                )?
            };
            let right = deck.pose.rotation * Vector::X;
            let basis = [right, deck.n.cross(right), deck.n];
            let (after, impulse, law, outgoing) = surface
                .solve(&i.rotor, cut, self.c.mass, basis, i.friction)
                .ok()?;
            let tail = if stop_at_impact { 0. } else { h - root };
            let (g1, m1) = if stop_at_impact {
                (g0, m0)
            } else {
                self.isolated(
                    &i,
                    after,
                    root,
                    tail,
                    Some(k),
                    lower_after.map(|id| (id, false)),
                )?
            };
            let arm = cut.q * i.arms[k];
            let height = self.jet(after, tail).height(
                arm,
                deck.n,
                after.spin,
                (after.v - deck.v, i.rotor.acc),
                0.,
                cut.com.length() + deck.top.length(),
            );
            let outward = height.outward_acceleration(tail)?;
            let end = i.rotor.bounded(after, tail, 5)?;
            if surface.gap(end, if stop_at_impact { root } else { h }) < -i.tol {
                return None;
            }
            let hit = RigidPointImpactReceipt {
                corner: (k - 4) as u8,
                after_cut_s: root,
                pulse_start: elapsed,
                pulse_duration: dt,
                outward_acceleration: outward,
                impact: CornerImpactReceipt {
                    elapsed: elapsed + Duration::from_secs_f64(root),
                    gap: surface.gap(cut, root),
                    foot: self.ids.0,
                    support: if lower_after.is_some() {
                        self.ids.1
                    } else {
                        self.ids.2
                    },
                    point: (cut.com + arm).into(),
                    normal: deck.n.into(),
                    impulse_on_foot: impulse.into(),
                    before_relative_velocity: surface.incoming(cut).into(),
                    after_relative_velocity: outgoing.into(),
                    friction: i.friction,
                    sticking: law.sticking,
                    kinetic_energy_change: impulse.dot(deck.v) - law.dissipated_energy,
                    dissipated_energy: law.dissipated_energy,
                },
            };
            let local_com = e.world.bodies[self.c.foot]
                .mass_properties()
                .local_mprops
                .local_com;
            let body_state = |x: Cut| BodyState {
                position: (x.com - x.q * local_com).into(),
                orientation: x.q.into(),
                velocity: x.v.into(),
                angular_velocity: x.spin.into(),
            };
            (
                end,
                Some(OpposingCornerReceipt {
                    point: hit,
                    incoming: body_state(cut),
                    outgoing: body_state(after),
                    timing_error_s: error,
                    minimum_other_gap_m: g0.min(g1),
                    minimum_footprint_margin_m: m0.min(m1),
                }),
            )
        } else {
            if !close || !separated_approach {
                return None;
            }
            self.isolated(&i, start, 0., h, None, None)?;
            (end.or_else(|| i.rotor.bounded(start, h, 5))?, None)
        };
        Some(UpperStep {
            pair: (self.c.sole, self.c.deck),
            ceiling: self.roof,
            receipt,
            returned: None,
            normals: None,
            body: self.c.foot,
            end,
        })
    }
}
impl RapierBackend {
    pub fn opposing_corner_reach(
        &self,
        foot: BodyId,
        pad: BodyId,
        gate: BodyId,
        dt: Duration,
    ) -> Result<OpposingCornerReach, PhysicsError> {
        if foot == pad || foot == gate || pad == gate || dt.is_zero() {
            return Err(PhysicsError::InvalidValue);
        }
        if [foot, pad, gate]
            .iter()
            .any(|id| !self.bodies.contains_key(id))
        {
            return Err(PhysicsError::MissingBody);
        }
        Ok(UpperPair::new(self, (foot, pad, gate))
            .map_or(OpposingCornerReach::Unavailable, |p| p.reach(self, dt)))
    }
}
