// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A short, native contact solve following a fresh, isolated crossing certificate.
use super::*;

/// Opt-in temporal policy for the one native solve immediately following an isolated crossing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RotationContactPulse {
    /// End after the fresh upper crossing bound plus enough time for the remaining native sweeps.
    Measured,
    /// Target a fixed native duration in nanoseconds. The upper crossing bound must fit.
    Fixed(u16),
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PulseDecline {
    UnprotectedApproach,
    Material,
    Arrival,
    Duration,
    Clearance,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PulseReceipt {
    pub corner: u8,
    pub cut_ns: u64,
    /// Latest time after the measured cut at which the unsolved vertex can reach the pad.
    pub arrival_by_ns: Option<u64>,
    /// Set only if this duration became the next actual native solve.
    pub applied_ns: Option<u64>,
    pub decline: Option<PulseDecline>,
}

pub(super) struct PulseWindow {
    pub policy: RotationContactPulse,
    pub guarded_approach: bool,
    pub temporal: usize,
    pub record: PulseReceipt,
    pub primary: Option<taylor::Height>,
    /// Clearance to all other vertices and to the finite support's perimeter.
    pub clearance: f64,
    pub radius: f64,
}
impl PulseWindow {
    pub fn new(
        hit: BoxCrossing,
        policy: RotationContactPulse,
        temporal: usize,
        cut: u64,
        guarded: bool,
    ) -> Self {
        Self {
            policy,
            guarded_approach: guarded,
            temporal,
            record: PulseReceipt {
                corner: hit.corner,
                cut_ns: cut,
                arrival_by_ns: None,
                applied_ns: None,
                decline: None,
            },
            primary: None,
            clearance: f64::INFINITY,
            radius: 0.,
        }
    }
    pub fn record_vertex(
        &mut self,
        ix: usize,
        height: taylor::Height,
        measured_gap: f64,
        edge: f64,
        radius: f64,
    ) {
        if ix == usize::from(self.record.corner) {
            self.primary = Some(height);
        } else {
            self.clearance = self.clearance.min(measured_gap);
        }
        self.clearance = self.clearance.min(edge);
        self.radius = self.radius.max(radius);
    }
    fn solve(
        &mut self,
        c: &Candidate,
        e: &RapierBackend,
        share: u64,
        com: Vector,
        v: Vector,
        spin: Vector,
    ) -> Result<u64, PulseDecline> {
        if !self.guarded_approach {
            return Err(PulseDecline::UnprotectedApproach);
        }
        let world = &e.world;
        let f = &world.bodies[c.foot];
        let d = &world.bodies[c.pad];
        let sole = &world.colliders[c.sole];
        let deck = &world.colliders[c.deck];
        // The physical no-work bound applies to passive, unpowered contact with
        // nonnegative friction and restitution at most one in the translating pad frame.
        if [sole, deck].iter().any(|c| {
            !(0. ..=1.).contains(&c.restitution()) || !c.friction().is_finite() || c.friction() < 0.
        }) {
            return Err(PulseDecline::Material);
        }
        let limit = share.min(4_000);
        let hit = self
            .primary
            .and_then(|p| p.arrives_by(limit))
            .ok_or(PulseDecline::Arrival)?;
        self.record.arrival_by_ns = Some(hit);
        let span = match self.policy {
            RotationContactPulse::Measured => (hit + 1).saturating_mul(self.temporal.max(1) as u64),
            RotationContactPulse::Fixed(ns) => u64::from(ns),
        };
        if span <= hit || span > limit {
            return Err(PulseDecline::Duration);
        }
        let h = span as f64 * 1e-9;
        let motion = c.pulse_motion(e, v, spin, h);
        let (speed, angular) = (motion.linear, motion.angular);
        let travel = (speed + self.radius * angular) * h;
        let eps =
            512. * f64::EPSILON * (1. + com.length() + d.translation().length() + self.radius);
        let relative = sole.position_wrt_parent().ok_or(PulseDecline::Clearance)?;
        let deck_local = deck.position_wrt_parent().ok_or(PulseDecline::Clearance)?;
        let normal = (*d.rotation() * deck_local.rotation) * Vector::Z;
        let up = (*f.rotation() * relative.rotation) * Vector::Z;
        if up.dot(normal) <= angular * h + 128. * f64::EPSILON
            || !travel.is_finite()
            || self.clearance <= travel + eps
        {
            return Err(PulseDecline::Clearance);
        }
        if world.colliders.len() > 2
            && c.pulse_neighbors_clear(e, Duration::from_nanos(span), com, motion)
                .is_none()
        {
            return Err(PulseDecline::Clearance);
        }
        Ok(span)
    }
    pub fn finish(
        mut self,
        c: &Candidate,
        e: &RapierBackend,
        share: u64,
        state: (Vector, Vector, Vector),
    ) -> PulseReceipt {
        match self.solve(c, e, share, state.0, state.1, state.2) {
            Ok(span) => self.record.applied_ns = Some(span),
            Err(reason) => self.record.decline = Some(reason),
        }
        self.record
    }
}

#[derive(Clone, Copy)]
pub(super) struct PulseMotion {
    linear: f64,
    angular: f64,
}
impl Candidate {
    pub(super) fn pulse_motion(
        &self,
        e: &RapierBackend,
        v: Vector,
        spin: Vector,
        h: f64,
    ) -> PulseMotion {
        let world = &e.world;
        let f = &world.bodies[self.foot];
        let g = (world.gravity * self.gravity_scale).length();
        let m = self.mass;
        let props = &f.mass_properties().local_mprops;
        let moments = props.inv_principal_inertia.recip();
        let principal = (*f.rotation() * props.principal_inertia_local_frame).inverse() * spin;
        // 2T = m |v|² + ω·Iω. Impact cannot increase T in this inertial frame.
        // Uniform gravity then increases sqrt(2T/m) by at most |g|h.
        let speed = ((m * v.length_squared() + principal.dot(principal * moments)) / m).sqrt()
            * (1. + 128. * f64::EPSILON)
            + g * h;
        let angular = speed * (m / moments.min_element()).sqrt();
        PulseMotion {
            linear: speed,
            angular,
        }
    }
    /// The selected collision cannot increase kinetic energy in the translating
    /// support frame. Bound all translations and orientations of the foot from
    /// that energy, including both sides of a spatial impulse during the pulse.
    pub(super) fn pulse_neighbors_clear(
        &self,
        e: &RapierBackend,
        h: Duration,
        com: Vector,
        motion: PulseMotion,
    ) -> Option<Option<opponent::OpponentGuard>> {
        let w = &e.world;
        let f = &w.bodies[self.foot];
        let d = &w.bodies[self.pad];
        let sole = &w.colliders[self.sole];
        let local = sole.position_wrt_parent()?;
        let pose = *f.position() * local;
        let sphere = sole.shape().compute_bounding_sphere(&pose);
        let radius = sphere.radius + sphere.center.distance(com);
        let eps = 512. * f64::EPSILON * (1. + com.length() + d.translation().length() + radius);
        let travel = (motion.linear + radius * motion.angular) * h.as_secs_f64();
        if !travel.is_finite() {
            return None;
        }
        self.clear_neighbors(
            e,
            sole,
            crate::approach_clearance::ApproachSweep {
                pose,
                velocity: d.linvel(),
                dt: h,
                margin: travel + w.integration_parameters.prediction_distance() + eps,
            },
        )
    }
}
