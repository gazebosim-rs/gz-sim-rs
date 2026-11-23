// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exclude unrelated shapes from a candidate's conservative swept volume.
use super::*;
use rapier3d_f64::parry::{bounding_volume::BoundingVolume, query};

mod peers;
use peers::PeerEnvelope;
mod shared;
pub(crate) use shared::SharedNeighborhood;

pub(crate) struct ApproachSweep {
    pub pose: Pose,
    /// World translational velocity of the reference point for the motion bound.
    pub velocity: Vector,
    pub dt: Duration,
    /// Native speculative reach, extra angular travel, acceleration and roundoff.
    pub margin: f64,
}
impl RapierBackend {
    pub(crate) fn approach_neighbors_clear(
        &self,
        c: &Collider,
        sweep: ApproachSweep,
        excluded: impl Fn(RigidBodyHandle) -> bool,
    ) -> Result<bool, PhysicsError> {
        let abb = c.shape().compute_aabb(&sweep.pose);
        for (_, b) in self.world.colliders.iter() {
            if b.parent().is_some_and(&excluded) {
                continue;
            }
            let rb = b.parent().map(|h| &self.world.bodies[h]);
            let kind = rb.map_or(RigidBodyType::Fixed, RigidBody::body_type);
            if !crate::collision::can_exchange_impulses(c, b, kind) {
                continue;
            }
            if rb.is_some_and(|body| {
                !matches!(
                    body.body_type(),
                    RigidBodyType::Fixed | RigidBodyType::KinematicVelocityBased
                ) || body.angvel() != Vector::ZERO
            }) {
                return Ok(false);
            }
            let db = if let Some(body) = rb {
                *body.position() * b.position_wrt_parent().ok_or(PhysicsError::InvalidValue)?
            } else {
                *b.position()
            };
            let relative = sweep.velocity - rb.map_or(Vector::ZERO, RigidBody::linvel);
            let clearance = sweep.margin
                + c.contact_skin()
                + b.contact_skin()
                + relative.length() * sweep.dt.as_secs_f64();
            if !abb
                .loosened(clearance)
                .intersects(&b.shape().compute_aabb(&db))
            {
                continue;
            }
            if !query::distance(&sweep.pose, c.shape(), &db, b.shape())
                .is_ok_and(|r| r.distance.is_finite() && r.distance > clearance)
            {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

impl RapierBackend {
    /// Reuse each obstacle's current transform and bounds across the whole
    /// assembly. A fixed obstacle outside the union needs no pairwise checks.
    pub(crate) fn approach_tree_neighbors_clear(
        &self,
        sweeps: &[(&Collider, ApproachSweep)],
        excluded: impl Fn(RigidBodyHandle) -> bool,
    ) -> Result<bool, PhysicsError> {
        self.approach_tree_neighbors_in(sweeps, excluded, None)
    }
    pub(crate) fn approach_tree_neighbors_in(
        &self,
        sweeps: &[(&Collider, ApproachSweep)],
        excluded: impl Fn(RigidBodyHandle) -> bool,
        batch: Option<&SharedNeighborhood>,
    ) -> Result<bool, PhysicsError> {
        let bounds: Vec<_> = sweeps
            .iter()
            .map(|(c, s)| {
                c.shape().compute_aabb(&s.pose).loosened(
                    s.margin + c.contact_skin() + s.velocity.length() * s.dt.as_secs_f64(),
                )
            })
            .collect();
        let Some(union) = bounds.iter().copied().reduce(|a, b| a.merged(&b)) else {
            return Ok(true);
        };
        let clear_obstacle = |obstacle: &Collider| -> Result<bool, PhysicsError> {
            if !obstacle.is_enabled()
                || obstacle.is_sensor()
                || obstacle.parent().is_some_and(&excluded)
            {
                return Ok(true);
            }
            let body = obstacle.parent().map(|h| &self.world.bodies[h]);
            let kind = body.map_or(RigidBodyType::Fixed, RigidBody::body_type);
            let pose = if let Some(b) = body {
                *b.position()
                    * obstacle
                        .position_wrt_parent()
                        .ok_or(PhysicsError::InvalidValue)?
            } else {
                *obstacle.position()
            };
            let bound = obstacle
                .shape()
                .compute_aabb(&pose)
                .loosened(obstacle.contact_skin());
            let velocity = body.map_or(Vector::ZERO, RigidBody::linvel);
            // A prescribed tree with every generalized speed zero cannot enter this
            // swept area during the step. Queued promotions run before this check.
            let parked =
                kind == RigidBodyType::Dynamic && self.stationary_prescribed(obstacle.parent());
            let stationary = kind == RigidBodyType::Fixed || parked;
            if stationary && !union.intersects(&bound) {
                return Ok(true);
            }
            for ((c, s), own) in sweeps.iter().zip(&bounds) {
                if !crate::collision::can_exchange_impulses(c, obstacle, kind) {
                    continue;
                }
                if !parked
                    && body.is_some_and(|b| {
                        !matches!(
                            b.body_type(),
                            RigidBodyType::Fixed | RigidBodyType::KinematicVelocityBased
                        ) || b.angvel() != Vector::ZERO
                    })
                {
                    return Ok(false);
                }
                // Translating obstacles can enter during the interval even
                // when the initial transform is outside the assembly union.
                if !own.intersects(&bound.loosened(velocity.length() * s.dt.as_secs_f64())) {
                    continue;
                }
                let margin = s.margin
                    + c.contact_skin()
                    + obstacle.contact_skin()
                    + (s.velocity - velocity).length() * s.dt.as_secs_f64();
                if !query::distance(&s.pose, c.shape(), &pose, obstacle.shape())
                    .is_ok_and(|q| q.distance.is_finite() && q.distance > margin)
                {
                    return Ok(false);
                }
            }
            Ok(true)
        };
        if let Some(batch) = batch {
            if !union.mins.is_finite() || !union.maxs.is_finite() {
                return Ok(false);
            }
            for h in batch.neighbors(&union) {
                if !clear_obstacle(&self.world.colliders[h])? {
                    return Ok(false);
                }
            }
            batch
                .sweep
                .set(Some(PeerEnvelope::new(union, sweeps, bounds)));
        } else {
            for (_, obstacle) in self.world.colliders.iter() {
                if !clear_obstacle(obstacle)? {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }
    fn stationary_prescribed(&self, handle: Option<RigidBodyHandle>) -> bool {
        handle
            .and_then(|h| self.world.multibody_joints.rigid_body_link(h))
            .and_then(|l| self.world.multibody_joints.get_multibody(l.multibody))
            .is_some_and(|tree| {
                tree.root().joint.kinematic
                    && tree.links().all(|l| l.joint.kinematic)
                    && tree.generalized_velocity().iter().all(|v| *v == 0.)
            })
    }
}
