// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Join complete independent native finite-box solvers before their next interaction.
use super::*;
mod dispatcher;
use dispatcher::{Cache, RestorePairs, identity};

/// Body IDs are local to a backend. Each returned map translates one donor.
pub type NativeBoxMaps = Vec<BTreeMap<BodyId, BodyId>>;

impl RapierBackend {
    /// Stage an independent copy of the resident and donors. Body dynamics and
    /// point-wise contact geometry survive. Rapier rebuilds the imported graph
    /// with its actual new collider/body indices on the next native step.
    pub fn join_box_solvers(
        &self,
        donors: &[&Self],
    ) -> Result<(Self, NativeBoxMaps), PhysicsError> {
        self.join_box_solvers_with_profile(donors, NativeSolverProfile::Standard32)
    }

    pub fn join_box_solvers_with_profile(
        &self,
        donors: &[&Self],
        profile: NativeSolverProfile,
    ) -> Result<(Self, NativeBoxMaps), PhysicsError> {
        let invalid = || PhysicsError::InvalidValue;
        let total = donors
            .iter()
            .try_fold(self.bodies.len(), |n, d| n.checked_add(d.bodies.len()))
            .ok_or_else(invalid)?;
        if donors.is_empty() || total > profile.max_bodies() {
            return Err(invalid());
        }
        let ground_surface=self.partition_plane()?.map(NativeGroundPlane::bits);
        // Existing contacts inside one source retain their solver history. A
        // cross-source contact already inside Rapier's prediction margin has
        // no shared history to recover, even if the two solvers are otherwise valid.
        let mut bounds: Vec<PartitionBounds> = Vec::with_capacity(total);
        for source in std::iter::once(self).chain(donors.iter().copied()) {
            let prior = bounds.len();
            let ground = partition_ground::ground_of(&source.world)?;
            if ground.map(|(_,g)|g.bits()) != ground_surface
                || source.world.colliders.len()
                    != source.bodies.len() + usize::from(ground.is_some())
            {
                return Err(invalid());
            }
            for (handle, c) in source.world.colliders.iter() {
                if ground.is_some_and(|(plane, _)| plane == handle) {
                    continue;
                }
                let parent=c.parent().and_then(|h|source.world.bodies.get(h)).ok_or_else(invalid)?;
                let pose=parent.position()*c.position_wrt_parent().ok_or_else(invalid)?;
                let current=PartitionBounds::from_collider(c,&pose)?;
                if bounds[..prior].iter().any(|b|b.predicts_overlap(&current)){return Err(invalid());}
                bounds.push(current);
            }
        }
        let mut result = Self::restore_solver_with_profile(
            &self.checkpoint_solver_with_profile(profile)?,
            profile,
        )?;
        result
            .next_id
            .checked_add((total - self.bodies.len()) as u64)
            .ok_or(PhysicsError::IdOverflow)?;
        let resident_ground = partition_ground::ground_of(&result.world)?
            .map(|(handle, _)| identity(result.world.colliders[handle].shape()));
        let mut cache = Cache::new();
        let mut maps = Vec::with_capacity(donors.len());
        for &donor in donors {
            // Validate native bodies and the exact admitted parameter/material
            // profile before their solver state enters the staged result.
            let donor = Self::restore_solver_with_profile(
                &donor.checkpoint_solver_with_profile(profile)?,
                profile,
            )?;
            if !donor.partition_step_matches(Duration::from_secs_f64(
                result.world.integration_parameters.dt,
            )) {
                return Err(invalid());
            }
            let mut remap = BTreeMap::new();
            let mut shapes = BTreeMap::new();
            // Every donor observed the same immovable half-space. Register its
            // handle against the resident shape before remapping the donor pairs.
            if let Some((handle, _)) = partition_ground::ground_of(&donor.world)? {
                shapes.insert(
                    handle.into_raw_parts(),
                    resident_ground.ok_or_else(invalid)?,
                );
            }
            for (&id, entry) in &donor.bodies {
                let live = &donor.world.bodies[entry.0];
                let old_collider = live.colliders()[0];
                let collider = &donor.world.colliders[old_collider];
                let handle = result.world.bodies.insert(live.clone());
                let new_collider = result.world.colliders.insert_with_parent(
                    collider.clone(),
                    handle,
                    &mut result.world.bodies,
                );
                // Attaching the collider initializes mass properties; retain the
                // source's current world tensor, forces, activation and CCD velocity.
                result.world.bodies[handle].copy_from(live);
                shapes.insert(
                    old_collider.into_raw_parts(),
                    identity(result.world.colliders[new_collider].shape()),
                );
                let mut copy = entry.clone();
                copy.0 = handle;
                let new_id = BodyId::from_raw(result.next_id);
                result.next_id += 1;
                result.bodies.insert(new_id, copy);
                if donor.pending_wrenches.contains(&id) {
                    result.pending_wrenches.insert(new_id);
                }
                remap.insert(id, new_id);
            }
            for pair in donor.world.narrow_phase.contact_pairs() {
                // Rapier keeps removed edges until its next update. Their retired
                // handles have no destination and may never seed a new collider.
                let (Some(&a), Some(&b)) = (
                    shapes.get(&pair.collider1.into_raw_parts()),
                    shapes.get(&pair.collider2.into_raw_parts()),
                ) else {
                    continue;
                };
                match pair.manifolds() {
                    [] => {}
                    [m] if !m.points.is_empty() => {
                        if m.subshape_poses.is_some() || m.subshape1 != 0 || m.subshape2 != 0 {
                            return Err(invalid());
                        }
                        let mut restored = m.clone();
                        // Solver colors, body indices and graph positions belong
                        // to the donor's old graph. Native selection regenerates them.
                        restored.data = Default::default();
                        if a == b || cache.insert((a, b), restored).is_some() {
                            return Err(invalid());
                        }
                    }
                    [_] => {}
                    _ => return Err(invalid()),
                }
            }
            maps.push(remap);
        }
        result.refresh_max_boundary_spin();
        if !cache.is_empty() {
            result
                .world
                .narrow_phase
                .set_query_dispatcher(crate::contact_dispatcher::NativeContacts(RestorePairs(cache.into())));
            result.join_queries_pending = true;
        }
        Ok((result, maps))
    }
}

#[cfg(test)]
#[path = "../tests/private/box_join.rs"]
mod tests;
