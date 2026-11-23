// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Per-native-interval obstacle index for a jointly committed group of trees.
use super::*;
use rapier3d_f64::parry::{
    bounding_volume::Aabb,
    partitioning::{Bvh, BvhBuildStrategy},
};
use std::{cell::Cell, collections::HashSet};

pub(crate) struct SharedNeighborhood {
    pub(crate) sweep: Cell<Option<PeerEnvelope>>,
    handles: Vec<ColliderHandle>,
    tree: Bvh,
    unbounded: Vec<ColliderHandle>,
}
impl SharedNeighborhood {
    pub(crate) fn new(
        b: &RapierBackend,
        members: &HashSet<RigidBodyHandle>,
        dt: Duration,
    ) -> Result<Self, PhysicsError> {
        let mut handles = Vec::new();
        let mut boxes = Vec::new();
        let mut unbounded = Vec::new();
        for (id, c) in b.world.colliders.iter() {
            if !c.is_enabled() || c.is_sensor() || c.parent().is_some_and(|p| members.contains(&p))
            {
                continue;
            }
            let body = c.parent().map(|p| &b.world.bodies[p]);
            let bounded = body.is_none_or(|r| {
                r.is_fixed()
                    || r.body_type() == RigidBodyType::KinematicVelocityBased
                        && r.angvel() == Vector::ZERO
            }) || b.stationary_prescribed(c.parent());
            if !bounded {
                unbounded.push(id);
                continue;
            }
            let (pose, speed) = if let Some(body) = body {
                (
                    *body.position() * c.position_wrt_parent().ok_or(PhysicsError::InvalidValue)?,
                    body.linvel().length(),
                )
            } else {
                (*c.position(), 0.)
            };
            let bound = c
                .shape()
                .compute_aabb(&pose)
                .loosened(c.contact_skin() + speed * dt.as_secs_f64());
            // Infinite planes or extreme user input have no usable finite BVH
            // key. Always run their full collision/material and distance checks.
            if !(bound.mins.is_finite() && bound.maxs.is_finite() && bound.center().is_finite()) {
                unbounded.push(id);
                continue;
            }
            handles.push(id);
            boxes.push(bound);
        }
        Ok(Self {
            sweep: Cell::new(None),
            handles,
            tree: Bvh::from_leaves(BvhBuildStrategy::Binned, &boxes),
            unbounded,
        })
    }
    pub(crate) fn neighbors<'a>(
        &'a self,
        bounds: &'a Aabb,
    ) -> impl Iterator<Item = ColliderHandle> + 'a {
        self.tree
            .intersect_aabb(bounds)
            .map(|i| self.handles[i as usize])
            .chain(self.unbounded.iter().copied())
    }
}
