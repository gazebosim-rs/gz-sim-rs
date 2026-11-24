// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Select public intervals containing a possible impact of a strictly separated rigid sole.
use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RigidBoxSweep {
    /// Every lower vertex remains above the infinite pad plane for this interval.
    Clear,
    /// A positive starting gap can close. The bits identify the unconcluded vertices.
    Possible {
        vertices: u8,
    },
    /// At least one lower vertex starts on or below the surface or its numerical enclosure.
    Abutting,
    Unavailable(BoxTimingDecline),
}
impl RapierBackend {
    /// Read-only nomination for a free box and fixed or translating finite pad.
    /// A possible contact only selects subsequent native work; it is not a reaction.
    pub fn rigid_box_sweep(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<RigidBoxSweep, PhysicsError> {
        self.rigid_box_sweep_with_opponent(foot, pad, None, dt)
    }
    pub fn rigid_box_sweep_with_opponent(
        &self,
        foot: BodyId,
        pad: BodyId,
        opponent: Option<BodyId>,
        dt: Duration,
    ) -> Result<RigidBoxSweep, PhysicsError> {
        if let Some(id) = opponent {
            if id == foot || id == pad {
                return Err(PhysicsError::InvalidValue);
            }
            if !self.bodies.contains_key(&id) {
                return Err(PhysicsError::MissingBody);
            }
        }
        if dt.is_zero() || dt.as_nanos() > u64::MAX as u128 {
            return Err(PhysicsError::InvalidValue);
        }
        let sole = self.bodies.get(&foot).ok_or(PhysicsError::MissingBody)?.0;
        let support = self.bodies.get(&pad).ok_or(PhysicsError::MissingBody)?.0;
        if self.world.multibody_joints.rigid_body_link(sole).is_some() {
            return Ok(RigidBoxSweep::Unavailable(
                BoxTimingDecline::GeneralizedContact,
            ));
        }
        let prediction = Candidate::new(self, sole, support).and_then(|mut c| {
            if let Some(id) = opponent {
                c.enroll_opponent(self, id);
            }
            c.sweep(self, dt)
        });
        Ok(prediction.unwrap_or_else(RigidBoxSweep::Unavailable))
    }
}
impl Candidate {
    fn sweep(&self, e: &RapierBackend, dt: Duration) -> Result<RigidBoxSweep, BoxTimingDecline> {
        use BoxTimingDecline::*;
        let w = &e.world;
        let foot = &w.bodies[self.foot];
        let pad = &w.bodies[self.pad];
        let collider = &w.colliders[self.sole];
        let deck = &w.colliders[self.deck];
        if !crate::collision::can_exchange_impulses(collider, deck, pad.body_type()) {
            return Err(UnsupportedGeometry);
        }
        let fp = *foot.position() * collider.position_wrt_parent().ok_or(UnsupportedGeometry)?;
        let dp = *pad.position() * deck.position_wrt_parent().ok_or(UnsupportedGeometry)?;
        let foot_half = collider
            .shape()
            .as_cuboid()
            .ok_or(UnsupportedGeometry)?
            .half_extents;
        let pad_half = deck
            .shape()
            .as_cuboid()
            .ok_or(UnsupportedGeometry)?
            .half_extents;
        let (com, v, spin) = self.kinematics(e)?;
        let h = dt.as_secs_f64();
        let normal = dp.rotation * Vector::Z;
        let acc = w.gravity * self.gravity_scale;
        let az = acc.dot(normal);
        let motion = self.rotation_motion(foot, spin, h);
        if !motion.speed.is_finite()
            || !motion.acceleration.is_finite()
            || (fp.rotation.inverse() * normal).z <= motion.speed * h
        {
            return Err(OutsideSupportFace);
        }
        let jet = self.inertia.map_or(
            taylor::AngularJet {
                alpha: Vector::ZERO,
                beta: Vector::ZERO,
                snap: spin.length().powi(4),
            },
            |i| i.jet(*foot.rotation(), spin, h),
        );
        let skin = collider.contact_skin() + deck.contact_skin();
        let center_scale =
            com.length() + dp.translation.length() + fp.translation.length() + foot_half.length();
        let eps = 512. * f64::EPSILON * (1. + center_scale);
        let mut vertices = 0;
        for (i, x) in [-foot_half.x, foot_half.x].into_iter().enumerate() {
            for (j, y) in [-foot_half.y, foot_half.y].into_iter().enumerate() {
                let p = fp * Vector::new(x, y, -foot_half.z);
                let local = dp.inverse_transform_point(p);
                let arm = p - com;
                let drift = (v.length() + motion.speed * arm.length()) * h + acc.length() * h * h;
                if (local.x.abs() + drift >= pad_half.x || local.y.abs() + drift >= pad_half.y)
                    && !jet.over_pad(
                        taylor::MovingVertex {
                            point: p,
                            com,
                            v,
                            gravity: acc,
                            spin,
                        },
                        dp,
                        pad_half,
                        h,
                    )
                {
                    return Err(OutsideSupportFace);
                }
                let gap = local.z - pad_half.z - skin;
                if !gap.is_finite() || gap <= eps {
                    return Ok(RigidBoxSweep::Abutting);
                }
                let height =
                    jet.height(arm, normal, spin, (v, acc), gap, center_scale + p.length());
                if height.released_span(h, az).is_none() {
                    vertices |= 1 << (i * 2 + j);
                }
            }
        }
        // A changing third contact could invalidate the free-motion enclosure.
        if w.colliders.len() > 2 {
            let bound = collider.shape().compute_bounding_sphere(&fp);
            let radius = bound.radius + bound.center.distance(com);
            let sweep = crate::approach_clearance::ApproachSweep {
                pose: fp,
                velocity: v + pad.linvel(),
                dt,
                margin: w.integration_parameters.prediction_distance()
                    + radius * motion.speed * h
                    + acc.length() * h * h
                    + eps,
            };
            if self.clear_neighbors(e, collider, sweep).is_none() {
                return Err(OtherContact);
            }
        }
        Ok(if vertices == 0 {
            RigidBoxSweep::Clear
        } else {
            RigidBoxSweep::Possible { vertices }
        })
    }
}
