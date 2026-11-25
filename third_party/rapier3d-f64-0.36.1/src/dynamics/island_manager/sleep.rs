use crate::alloc_prelude::*;
use crate::dynamics::{RigidBodyHandle, RigidBodySet};
use crate::geometry::{ColliderHandle, ColliderSet, NarrowPhase};

use super::{Island, IslandManager};

impl IslandManager {
    /// Wakes up a sleeping body, forcing it back into the active simulation.
    ///
    /// Waking any body of a sleeping island wakes the **whole island** (its
    /// entire touching-contact/joint connected component) and resets every
    /// member's sleep timer.
    ///
    /// # Parameters
    /// * `strong` - If `true`, the body is guaranteed to stay awake for multiple frames.
    ///   If `false`, it might sleep again immediately if conditions are met.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut bodies = RigidBodySet::new();
    /// # let mut islands = IslandManager::new();
    /// # let body_handle = bodies.insert(RigidBodyBuilder::dynamic());
    /// islands.wake_up(&mut bodies, body_handle, true);
    /// let body = bodies.get_mut(body_handle).unwrap();
    /// // Wake up a body before applying force to it
    /// body.add_force(Vector::new(100.0, 0.0, 0.0), false);
    /// ```
    ///
    /// Only affects dynamic bodies (kinematic and fixed bodies don't sleep).
    pub fn wake_up(&mut self, bodies: &mut RigidBodySet, handle: RigidBodyHandle, strong: bool) {
        // NOTE: the use an Option here because there are many legitimate cases (like when
        //       deleting a joint attached to an already-removed body) where we could be
        //       attempting to wake-up a rigid-body that has already been deleted.
        if bodies.get(handle).map(|rb| !rb.is_fixed()) == Some(true) {
            let rb = bodies.index_mut_internal(handle);

            rb.activation.wake_up(strong);
            let persistent_id = rb.ids.island_id;
            let island_to_wake_up = rb.ids.active_island_id;

            // Whole-island wake: waking any body wakes the entire persistent island, with
            // a *strong* timer reset for every member — `RigidBody::sleep` leaves timers at the
            // eligibility threshold, so a freshly woken island would otherwise re-sleep next step.
            if persistent_id != crate::dynamics::INVALID_ISLAND {
                let sleeping_island = self
                    .persistent
                    .islands
                    .get(persistent_id as usize)
                    .is_some_and(|island| island.sleeping);
                if sleeping_island {
                    let island = &mut self.persistent.islands[persistent_id as usize];
                    island.sleeping = false;
                    // The island's bodies normally share one sleeping-chunk
                    // container, but joint-merged sleeping islands can span
                    // several: wake each body's chunk.
                    let handles = island.bodies.clone();
                    for h in &handles {
                        if let Some(rb) = bodies.get_mut(*h) {
                            rb.activation.wake_up(true);
                        }
                    }
                    for h in handles {
                        let chunk = match bodies.get(h) {
                            Some(rb) => rb.ids.active_island_id,
                            None => continue,
                        };
                        self.wake_up_island(bodies, chunk as usize);
                    }
                    return;
                }
            }

            self.wake_up_island(bodies, island_to_wake_up as usize);
        }
    }

    fn selected_islands(
        &self,
        bodies: &RigidBodySet,
        handles: &[RigidBodyHandle],
    ) -> Option<Vec<u32>> {
        let mut ids = Vec::new();
        for &h in handles {
            let id = self.persistent.body_island(bodies, h)?;
            if !ids.contains(&id) {
                ids.push(id);
            }
        }
        Some(ids)
    }

    /// Read-only check that sleeping this group will not strand a connected awake body.
    pub fn can_sleep_connected_group(
        &self,
        bodies: &RigidBodySet,
        handles: &[RigidBodyHandle],
    ) -> bool {
        self.closed_except(bodies, handles, &[])
    }

    fn closed_except(
        &self,
        bodies: &RigidBodySet,
        handles: &[RigidBodyHandle],
        retiring: &[u32],
    ) -> bool {
        let Some(ids) = self.selected_islands(bodies, handles) else {
            return false;
        };
        ids.into_iter().all(|id| {
            let island = &self.persistent.islands[id as usize];
            if !island.sleeping
                && !island
                    .bodies
                    .iter()
                    .all(|h| Some(bodies[*h].ids.active_island_id as usize) == self.awake_island)
            {
                return false;
            }
            if island.bodies.iter().all(|h| handles.contains(h)) {
                return true;
            }
            if (retiring.is_empty() && island.constraint_remove_count == 0) || island.sleeping {
                return false;
            }
            // After contact loss the incremental graph can still pack disconnected
            // components together until its budgeted split. Edges to a fixed
            // body or removed body do not connect two dynamic components.
            let closed = |a, b| {
                self.persistent.body_island(bodies, a) != Some(id)
                    || self.persistent.body_island(bodies, b) != Some(id)
                    || handles.contains(&a) == handles.contains(&b)
            };
            island
                .contact_links
                .iter()
                .all(|c| retiring.contains(&c.edge_id) || closed(c.body1, c.body2))
                && island.joint_links.iter().all(|c| closed(c.body1, c.body2))
        })
    }

    // Validate the prospective cut before touching the solver graph. Events must
    // retain their native lifecycle. Only rigid, externally monitored predictions
    // from an awake member to a distinct non-fixed body can be retired.
    fn separated_cut(
        &self,
        bodies: &RigidBodySet,
        handles: &[RigidBodyHandle],
        colliders: &ColliderSet,
        narrow_phase: &NarrowPhase,
        pairs: &[(ColliderHandle, ColliderHandle)],
    ) -> Option<Vec<u32>> {
        let mut edges = Vec::new();
        for &(a, b) in pairs {
            let (left, right) = (colliders.get(a)?, colliders.get(b)?);
            let from = left.parent()?;
            let to = right.parent()?;
            let id = self.persistent.body_island(bodies, from)?;
            if !handles.contains(&from)
                || handles.contains(&to)
                || self.persistent.body_island(bodies, to) != Some(id)
                || self.persistent.islands[id as usize].sleeping
                || !(left.active_events() | right.active_events()).is_empty()
            {
                return None;
            }
            let edge = narrow_phase.unloaded_prediction_edge(a, b)?;
            // Solver rows describe the previous collision pass. An edit to a shape,
            // skin, body pose, or attached collider pose can consume its gap before
            // another solve. Compose from live parent poses, then check both skins.
            let skins = [left.contact_skin(), right.contact_skin()];
            if skins.iter().any(|skin| !skin.is_finite() || *skin < 0.0) {
                return None;
            }
            let left_pose = *bodies.get(from)?.position() * *left.position_wrt_parent()?;
            let right_pose = *bodies.get(to)?.position() * *right.position_wrt_parent()?;
            let gap = narrow_phase
                .query_dispatcher()
                .distance(&left_pose.inv_mul(&right_pose), left.shape(), right.shape())
                .ok()?
                .distance;
            if !gap.is_finite() || gap <= skins[0] + skins[1] {
                return None;
            }
            if !edges.contains(&edge) {
                edges.push(edge);
            }
        }
        (!edges.is_empty() && self.closed_except(bodies, handles, &edges)).then_some(edges)
    }

    /// Read-only eligibility when a caller can monitor and filter specified, strictly
    /// separated predictive pairs during standby. Live joints and load-bearing
    /// contact edges continue to connect their bodies.
    pub fn can_sleep_with_separated_predictions(
        &self,
        bodies: &RigidBodySet,
        handles: &[RigidBodyHandle],
        colliders: &ColliderSet,
        narrow_phase: &NarrowPhase,
        pairs: &[(ColliderHandle, ColliderHandle)],
    ) -> bool {
        self.separated_cut(bodies, handles, colliders, narrow_phase, pairs)
            .is_some()
    }
    /// Atomically close and sleep the group by retiring its approved zero-load edges.
    /// The caller re-enables each pair before the bodies can close its measured gap.
    pub fn sleep_with_separated_predictions(
        &mut self,
        bodies: &mut RigidBodySet,
        handles: &[RigidBodyHandle],
        colliders: &ColliderSet,
        narrow_phase: &mut NarrowPhase,
        pairs: &[(ColliderHandle, ColliderHandle)],
    ) -> bool {
        let Some(edges) = self.separated_cut(bodies, handles, colliders, narrow_phase, pairs)
        else {
            return false;
        };
        let ids = self.selected_islands(bodies, handles).unwrap();
        for edge in edges {
            narrow_phase.clear_unloaded_prediction(edge);
            self.persistent.unlink_contact(edge);
        }
        for id in ids {
            if !self.persistent.islands[id as usize].sleeping {
                self.persistent.clear_pending_split_of(id);
                self.persistent.split_island_now(bodies, id);
            }
        }
        let slept = self.sleep_connected_group(bodies, narrow_phase, handles);
        assert!(slept, "validated graph cut must close the native group");
        true
    }

    /// Sleep a closed group between physics steps using ordinary whole-island
    /// membership, wake propagation and cache invalidation. A refused request
    /// leaves the graph untouched. The caller establishes physical eligibility.
    pub fn sleep_connected_group(
        &mut self,
        bodies: &mut RigidBodySet,
        narrow_phase: &mut NarrowPhase,
        handles: &[RigidBodyHandle],
    ) -> bool {
        if !self.can_sleep_connected_group(bodies, handles) {
            return false;
        }
        let ids = self.selected_islands(bodies, handles).unwrap();
        for id in ids {
            let island = &self.persistent.islands[id as usize];
            if !island.sleeping && island.constraint_remove_count > 0 {
                self.persistent.clear_pending_split_of(id);
                self.persistent.split_island_now(bodies, id);
            }
        }
        let mut chunks = Vec::new();
        for id in self.selected_islands(bodies, handles).unwrap() {
            let island = &self.persistent.islands[id as usize];
            debug_assert!(island.bodies.iter().all(|h| handles.contains(h)));
            if !island.sleeping {
                chunks.push(island.bodies.clone());
                self.persistent.mark_island_sleeping(id);
            }
        }
        if !chunks.is_empty() {
            let awake = self.awake_island.expect("validated awake group");
            self.commit_sleeping_chunks(
                bodies,
                narrow_phase,
                awake,
                self.islands[awake].len(),
                chunks,
            );
        }
        true
    }

    /// Puts `chunks` (disjoint subsets of the awake island's bodies, all
    /// sleep-eligible) to sleep: in place if they cover the entire awake
    /// island, otherwise by extracting each chunk into a new sleeping island.
    pub(super) fn commit_sleeping_chunks(
        &mut self,
        bodies: &mut RigidBodySet,
        narrow_phase: &mut NarrowPhase,
        active_island_id: usize,
        active_island_len: usize,
        mut chunks: Vec<Vec<RigidBodyHandle>>,
    ) {
        if chunks.len() == 1 && chunks[0].len() == active_island_len {
            // The whole island is asleep. No need to insert a new one.
            // Put all its bodies to sleep.
            let active_island = &mut self.islands[active_island_id];
            for handle in &active_island.bodies {
                bodies.index_mut_internal(*handle).sleep();
            }

            for handle in &active_island.bodies {
                let rb = &bodies[*handle];
                for co_handle in rb.colliders.0.iter().copied() {
                    narrow_phase.clear_asleep_pair_solver_hint_counts_of(co_handle);
                }
            }

            // Membership changed (the whole island leaves the active set): bump the epoch so
            // epoch-keyed caches can't go stale. The hint count-clears above only cover bodies
            // WITH colliders — collider-less (joint-only) bodies would otherwise sleep without invalidating e.g. the cached body qualification table.
            self.active_set_epoch = self.active_set_epoch.wrapping_add(1);

            // Mark the island as sleeping: no island is awake anymore.
            debug_assert_eq!(self.awake_island, Some(active_island_id));
            self.awake_island = None;
        } else {
            let slept: Vec<RigidBodyHandle> = chunks.iter().flatten().copied().collect();

            for chunk in &mut chunks {
                let new_island = Island {
                    bodies: core::mem::take(chunk),
                };
                self.extract_sleeping_sub_island(bodies, active_island_id, new_island);
            }

            // Clear hints after the extractions (which flag the bodies as
            // sleeping).
            for handle in &slept {
                let rb = &bodies[*handle];
                for co_handle in rb.colliders.0.iter().copied() {
                    narrow_phase.clear_asleep_pair_solver_hint_counts_of(co_handle);
                }
            }
        }
    }

    pub(super) fn wake_up_island(&mut self, bodies: &mut RigidBodySet, island_id: usize) {
        if self.awake_island == Some(island_id) {
            // Already awake.
            return;
        }

        let Some(island) = self.islands.get_mut(island_id) else {
            return;
        };

        match self.awake_island {
            None => {
                // Nothing is awake: this chunk becomes the awake island. No renumbering (bodies
                // keep their `active_set_id`s), but the active-set *membership* changes, so
                // epoch-keyed caches (body qualification table, persistent solver graph, solver constraint caches) must not survive — bump the epoch like the merge branch. (Direct field bump: `island` still borrows `self.islands`.)
                self.active_set_epoch = self.active_set_epoch.wrapping_add(1);
                self.awake_island = Some(island_id);

                for handle in &island.bodies {
                    if let Some(rb) = bodies.get_mut(*handle) {
                        rb.wake_up(false);
                    }
                }
            }
            Some(awake_id) => {
                // Merge the chunk's bodies into the single awake island.
                self.bump_active_set_epoch();
                let Some(removed) = self.islands.remove(island_id) else {
                    unreachable!()
                };
                self.free_islands.push(island_id);

                let target = &mut self.islands[awake_id];
                for handle in &removed.bodies {
                    let Some(rb) = bodies.get_mut(*handle) else {
                        // This body no longer exists.
                        continue;
                    };
                    rb.wake_up(false);
                    rb.ids.active_island_id = awake_id as u32;
                    rb.ids.active_set_id = (target.bodies.len()) as u32;
                    target.bodies.push(*handle);
                }
            }
        }
    }
}
