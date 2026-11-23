// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Stage independent, fully supported trees before consuming a native time slice.
use super::*;
use crate::approach_clearance::SharedNeighborhood;
use std::collections::HashSet;
impl RapierBackend {
    /// Recompute the *current* motions of all previously admitted neighbors.
    /// Their last public displacements are never used as a future reach bound.
    pub(crate) fn shared_group_remains_clear(&self, dt: Duration) -> Result<bool, PhysicsError> {
        let groups = &self.solved_shared_groups;
        Ok(groups.len() > 1
            && self.prepare_shared_batch(groups, Duration::ZERO, dt)?.len() == groups.len())
    }
    pub(crate) fn commit_shared_batch(
        &mut self,
        items: Vec<PreparedBearing>,
    ) -> Vec<BearingResult> {
        PreparedBearing::commit_batch(items, self)
    }
    pub(crate) fn prepare_shared_batch(
        &self,
        groups: &[SharedSupportRequest],
        start: Duration,
        dt: Duration,
    ) -> Result<Vec<PreparedBearing>, PhysicsError> {
        let mut members = HashSet::new();
        let mut supports = HashSet::new();
        for group in groups {
            if !group.supports.iter().all(|p| supports.insert(*p)) {
                return Ok(Vec::new());
            }
            let handle = self
                .bodies
                .get(&group.foot)
                .ok_or(PhysicsError::MissingBody)?
                .0;
            let Some(link) = self.world.multibody_joints.rigid_body_link(handle) else {
                return Ok(Vec::new());
            };
            let tree = self
                .world
                .multibody_joints
                .get_multibody(link.multibody)
                .ok_or(PhysicsError::InvalidJoint)?;
            if !tree.links().all(|l| members.insert(l.rigid_body_handle())) {
                return Ok(Vec::new());
            }
        }
        let neighborhood = SharedNeighborhood::new(self, &members, dt)?;
        let mut bounds = Vec::with_capacity(groups.len());
        let mut prepared = Vec::with_capacity(groups.len());
        for group in groups {
            let modes = BearingModes {
                edge: false,
                sole: false,
                rocking: false,
                face: true,
                deck: false,
                shared: Some(*group),
                batch: Some(&neighborhood),
            };
            let Some(bearing) =
                self.prepare_two_bearing(group.foot, group.supports[0], start, dt, modes)?
            else {
                return Ok(Vec::new());
            };
            let bound = neighborhood
                .sweep
                .take()
                .expect("a shared span passed its swept geometry checks");
            if bounds.iter().any(|other| !bound.separate(other)) {
                return Ok(Vec::new());
            }
            bounds.push(bound);
            prepared.push(bearing);
        }
        Ok(prepared)
    }
}
