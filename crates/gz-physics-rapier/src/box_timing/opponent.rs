// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! An explicitly named translating box may be excluded only for its physically separated span.
use super::*;
use crate::approach_clearance::ApproachSweep;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OpponentClearance {
    pub opponent: BodyId,
    /// Outward facing obstacle axis. Positive X, negative X, then Y and Z.
    pub face: u8,
    /// Lower bound on the gap for the entire completed interval, rounded down to picometers.
    pub minimum_gap_pm: u64,
}
#[derive(Clone, Copy)]
pub(super) struct Opponent {
    pub body: RigidBodyHandle,
    pub collider: ColliderHandle,
    id: BodyId,
}
#[derive(Clone, Copy)]
pub(crate) struct OpponentGuard {
    pub collider: ColliderHandle,
    pub record: OpponentClearance,
}
impl Candidate {
    pub(crate) fn enroll_opponent(&mut self, e: &RapierBackend, id: BodyId) {
        self.opponent = None;
        let Some(h) = e.bodies.get(&id).map(|entry| entry.0) else {
            return;
        };
        if self.foot == h
            || self.pad == h
            || e.world
                .multibody_joints
                .rigid_body_link(self.foot)
                .is_some()
        {
            return;
        }
        let b = &e.world.bodies[h];
        let [c] = b.colliders() else {
            return;
        };
        let shape = &e.world.colliders[*c];
        if matches!(
            b.body_type(),
            RigidBodyType::Fixed | RigidBodyType::KinematicVelocityBased
        ) && b.is_enabled()
            && b.angvel() == Vector::ZERO
            && shape.is_enabled()
            && !shape.is_sensor()
            && shape.shape().as_cuboid().is_some()
        {
            self.opponent = Some(Opponent {
                body: h,
                collider: *c,
                id,
            });
        }
    }
    // Other shapes retain the native speculative reach and all native filters.
    // Only this explicit sole/opponent pair may use a smaller, physically proven reach.
    pub(super) fn clear_neighbors(
        &self,
        e: &RapierBackend,
        sole: &Collider,
        sweep: ApproachSweep,
    ) -> Option<Option<OpponentGuard>> {
        let guard = if let Some(g) = self.opponent {
            Some(g.clear(e, sole, &sweep)?)
        } else {
            None
        };
        let clear = e
            .approach_neighbors_clear(sole, sweep, |h| {
                h == self.pad
                    || self.links.iter().any(|l| l.body == h)
                    || self.opponent.is_some_and(|g| g.body == h)
            })
            .ok()?;
        clear.then_some(guard)
    }
}
impl Opponent {
    fn clear(self, e: &RapierBackend, sole: &Collider, s: &ApproachSweep) -> Option<OpponentGuard> {
        let w = &e.world;
        let b = w.bodies.get(self.body)?;
        let shape = w.colliders.get(self.collider)?;
        if !matches!(
            b.body_type(),
            RigidBodyType::Fixed | RigidBodyType::KinematicVelocityBased
        ) || b.angvel() != Vector::ZERO
            || !b.is_enabled()
            || !shape.is_enabled()
            || shape.is_sensor()
        {
            return None;
        }
        let obstacle = *b.position() * shape.position_wrt_parent()?;
        let a = sole.shape().as_cuboid()?.half_extents;
        let half = shape.shape().as_cuboid()?.half_extents;
        let offset = s.pose.translation - obstacle.translation;
        let v = s.velocity - b.linvel();
        let rotation = s.pose.rotation.inverse();
        // The caller includes uncertain angular displacement about the actual COM,
        // gravity travel, and (during impact) its full passive energy bound.
        if !s.margin.is_finite() || s.margin < 0. {
            return None;
        }
        let residual = (s.margin - w.integration_parameters.prediction_distance()).max(0.);
        let eps = 512.
            * f64::EPSILON
            * (1.
                + s.pose.translation.length()
                + obstacle.translation.length()
                + a.length()
                + half.length());
        let margin = residual + sole.contact_skin() + shape.contact_skin() + eps;
        if !margin.is_finite() || !v.is_finite() {
            return None;
        }
        let mut best = (0., 0u8);
        for (ix, (axis, h)) in [
            (Vector::X, half.x),
            (Vector::Y, half.y),
            (Vector::Z, half.z),
        ]
        .into_iter()
        .enumerate()
        {
            let n = obstacle.rotation * axis;
            let support = (rotation * n).abs().dot(a) + h;
            for (side, sign) in [(0, 1.), (1, -1.)] {
                let g = sign * offset.dot(n) - support
                    + (sign * v.dot(n)).min(0.) * s.dt.as_secs_f64()
                    - margin;
                if g.is_finite() && g > best.0 {
                    best = (g, 2 * ix as u8 + side);
                }
            }
        }
        (best.0 > 0.).then_some(OpponentGuard {
            collider: self.collider,
            record: OpponentClearance {
                opponent: self.id,
                face: best.1,
                minimum_gap_pm: (best.0 * 1e12).floor() as u64,
            },
        })
    }
}
