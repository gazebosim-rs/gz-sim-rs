// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Remap a cache-drained member's native coordinates into a newly coupled fleet.
use super::*;

const MAGIC: &[u8; 8] = b"GZFCLR01";
pub const CLEAR_GRASP_MEMBER_IMAGE_LIMIT: usize = 4096;
fn clear_error(stage: &'static str) -> PhysicsError {
    PhysicsError::Native {
        engine: "multifidelity",
        message: format!("invalid cache-drained grasp member {stage}"),
    }
}
const PART_BYTES: usize = 13 * 8;
const LEAF_BYTES: usize = 6 * 8;

impl RapierBackend {
    pub(super) fn clear_layout(
        &self,
        root: BodyId,
        part: BodyId,
    ) -> Result<TreeLayout, PhysicsError> {
        let &(root_handle, _, _, _, group) =
            self.bodies.get(&root).ok_or(PhysicsError::MissingBody)?;
        let group = group.ok_or_else(unsupported_tree)?;
        let mut layout =
            self.scoped_tree_layout(true, Some(TreeExternal::Grasp(part)), Some(group))?;
        if layout.root != root_handle
            || layout
                .joints
                .iter()
                .any(|id| self.joints.records[id].initial.exact_lock)
            || self.bodies[&part].1.can_sleep
        {
            return Err(unsupported_tree());
        }
        self.soft_sphere_mobility(part)?;
        let (tree, _) = self
            .world
            .multibody_joints
            .get(layout.handle)
            .ok_or_else(invalid)?;
        let mut signature = Sha256::new();
        signature.update(MAGIC);
        let mut params = self.world.integration_parameters;
        params.dt = STEP.as_secs_f64();
        signature.update(
            codec()
                .serialize(&(group, params, self.allow_isolation, tree.num_links() as u64))
                .map_err(|_| invalid())?,
        );
        // In a combined world Rapier allocates different body, collider and joint
        // handles. Name each tree link by its solver topology ordinal instead.
        for handle in tree
            .links()
            .map(|l| l.rigid_body_handle())
            .chain([self.bodies[&part].0])
        {
            let (_, initial, rot, inertia, group) = self
                .bodies
                .values()
                .find(|entry| entry.0 == handle)
                .ok_or_else(invalid)?;
            let inertia = inertia.map(|i| (i.center, i.orientation, i.tensor));
            signature.update(
                codec()
                    .serialize(&(
                        initial.position,
                        initial.velocity,
                        initial.mass,
                        initial.kinematic,
                        initial.can_sleep,
                        rot.orientation,
                        rot.angular_velocity,
                        inertia,
                        group,
                        &initial.collider,
                    ))
                    .map_err(|_| invalid())?,
            );
        }
        for (i, id) in layout.joints.iter().enumerate() {
            let c = &self.joints.records[id].initial;
            let parent = tree
                .link(i + 1)
                .and_then(|l| l.parent_id())
                .ok_or_else(invalid)?;
            signature.update(
                codec()
                    .serialize(&(parent as u64, c.frame1, c.frame2, c.limits))
                    .map_err(|_| invalid())?,
            );
        }
        layout.magic = MOTOR_MAGIC;
        layout.construction = signature.finalize().into();
        Ok(layout)
    }

    // Empty cached pairs without manifold points are safe to forget. A touching
    // pair that a fresh world hasn't detected yet must also prevent the import.
    fn clear_grasp_space(&self) -> Result<bool, PhysicsError> {
        if self.world.narrow_phase.contact_pairs().any(|pair| {
            pair.has_any_active_contact()
                || pair
                    .manifolds()
                    .iter()
                    .any(|m| !m.points.is_empty() || !m.data.solver_contacts.is_empty())
        }) {
            return Ok(false);
        }
        let mut spheres = Vec::with_capacity(self.bodies.len());
        for (handle, _, _, _, group) in self.bodies.values() {
            let body = &self.world.bodies[*handle];
            if body.is_sleeping() || body.is_ccd_enabled() || body.soft_ccd_prediction() != 0. {
                return Ok(false);
            }
            let [h] = body.colliders() else {
                return Err(unsupported_tree());
            };
            let c = &self.world.colliders[*h];
            let center = c.position().translation;
            let radius =
                c.shape().as_ball().ok_or_else(unsupported_tree)?.radius + c.contact_skin();
            for &(prior_center, prior_radius, prior_group) in &spheres {
                if group.is_some() && group == &prior_group {
                    continue;
                }
                let gap: Vector = center - prior_center;
                let skin: f64 =
                    radius + prior_radius + self.world.integration_parameters.prediction_distance();
                if gap.length_squared() <= skin * skin {
                    return Ok(false);
                }
            }
            spheres.push((center, radius, *group));
        }
        Ok(true)
    }
    pub(super) fn clear_handles(
        &self,
        layout: &TreeLayout,
        part: BodyId,
    ) -> Result<Vec<ColliderHandle>, PhysicsError> {
        let (tree, _) = self
            .world
            .multibody_joints
            .get(layout.handle)
            .ok_or_else(invalid)?;
        tree.links()
            .map(|l| l.rigid_body_handle())
            .chain([self.bodies[&part].0])
            .map(|h| {
                let [c] = self.world.bodies[h].colliders() else {
                    return Err(unsupported_tree());
                };
                Ok(*c)
            })
            .collect()
    }

    pub(super) fn encode_clear_member(
        &self,
        root: BodyId,
        part: BodyId,
    ) -> Result<Vec<u8>, PhysicsError> {
        let layout = self.clear_layout(root, part)?;
        self.check_tree_dynamics(&layout)?;
        let handles = self.clear_handles(&layout, part)?;
        let compact = self.encode_tree(layout, true)?;
        let part = LinkMotion::from_body(&self.world.bodies[self.bodies[&part].0]);
        part.pose()?;
        let mut out = Vec::with_capacity(40 + PART_BYTES + compact.len());
        out.extend_from_slice(MAGIC);
        out.extend_from_slice(&[0; 32]);
        append(&mut out, &part)?;
        let query = self.world.query_pipeline();
        for h in handles {
            let bounds = query
                .bvh
                .leaf_node(h.into_raw_parts().0)
                .ok_or_else(invalid)?
                .aabb();
            append(&mut out, &(bounds.mins.to_array(), bounds.maxs.to_array()))?;
        }
        out.extend_from_slice(&compact);
        let hash = Sha256::digest(&out[40..]);
        out[8..40].copy_from_slice(&hash);
        Ok(out)
    }
    /// A portable, cache-drained native member. A nonempty contact manifold or
    /// current geometric proximity returns None and leaves the member untouched.
    pub fn checkpoint_clear_grasp_member(
        &self,
        root: BodyId,
        payload: BodyId,
    ) -> Result<Option<Vec<u8>>, PhysicsError> {
        if self.bodies.len() != self.joints.records.len() + 2 || self.joints.open {
            return Err(unsupported_tree());
        }
        if !self.clear_grasp_space()? {
            return Ok(None);
        }
        self.encode_clear_member(root, payload).map(Some)
    }
    /// Atomically assemble two drained members into a fresh shared native world.
    /// Both topology signatures and all per-member physical readbacks must survive remapping.
    pub fn restore_clear_grasp_fleet(
        &mut self,
        groups: &[(BodyId, BodyId); 2],
        members: [&[u8]; 2],
    ) -> Result<(), PhysicsError> {
        *self = self.stage_grasp_members(groups, members, true)?;
        Ok(())
    }

    pub(super) fn stage_grasp_members(
        &self,
        groups: &[(BodyId, BodyId); 2],
        members: [&[u8]; 2],
        must_be_clear: bool,
    ) -> Result<Self, PhysicsError> {
        if !self.joints.open || !self.joints.efforts.is_empty() {
            return Err(unsupported_tree());
        }
        let layouts = self.fleet_layout(groups)?;
        if layouts.iter().any(|l| l.motors.iter().any(Option::is_some)) {
            return Err(unsupported_tree());
        }
        let mut staged = Self::new_multifidelity([0.; 3])?;
        staged.owner = Arc::clone(&self.owner);
        staged.allow_isolation = self.allow_isolation;
        staged.apply_initial(self.prepare_initial(&self.initial_state())?)?;
        // This step initializes Rapier's generalized matrices only. It must not
        // leave a synthetic contact history from the authored starting poses.
        let filters: Vec<_> = staged
            .world
            .colliders
            .iter()
            .map(|(h, c)| (h, c.collision_groups()))
            .collect();
        for &(h, _) in &filters {
            staged.world.colliders[h].set_collision_groups(InteractionGroups::none());
        }
        staged.step(STEP)?;
        for (h, g) in filters {
            staged.world.colliders[h].set_collision_groups(g);
        }

        // A moving body's fat BVH leaf only refits after crossing Rapier's change
        // detection margin. Resetting that phase can add or remove an active
        // speculative contact one interval early, even with identical rigid motion.
        let mut remapped_leaves = Vec::with_capacity(staged.bodies.len());
        for ((root, payload), bytes) in groups.iter().copied().zip(members) {
            if !(40 + PART_BYTES + FIXED_BYTES + 34..=CLEAR_GRASP_MEMBER_IMAGE_LIMIT)
                .contains(&bytes.len())
                || &bytes[..8] != MAGIC
                || bytes[8..40] != Sha256::digest(&bytes[40..])[..]
            {
                return Err(invalid());
            }
            let part: LinkMotion = codec()
                .deserialize(&bytes[40..40 + PART_BYTES])
                .map_err(|_| invalid())?;
            part.pose()?;
            let layout = staged
                .clear_layout(root, payload)
                .map_err(|_| clear_error("construction"))?;
            let handles = staged.clear_handles(&layout, payload)?;
            let end = 40 + PART_BYTES + handles.len() * LEAF_BYTES;
            if bytes.len() < end + FIXED_BYTES + 34 {
                return Err(invalid());
            }
            let mut leaves = &bytes[40 + PART_BYTES..end];
            for h in handles {
                let (lo, hi): ([f64; 3], [f64; 3]) = codec()
                    .deserialize_from(&mut leaves)
                    .map_err(|_| invalid())?;
                let lo = Vector::from_array(lo);
                let hi = Vector::from_array(hi);
                if !lo.is_finite() || !hi.is_finite() || (hi - lo).min_element() <= 0. {
                    return Err(invalid());
                }
                remapped_leaves.push((h, rapier3d_f64::parry::bounding_volume::Aabb::new(lo, hi)));
            }
            let compact = &bytes[end..];
            if &compact[..8] != MOTOR_MAGIC
                || compact[8..40] != Sha256::digest(&compact[40..])[..]
                || compact[40..72] != layout.construction
                || compact.len() > FIXED_BYTES + 42 * layout.joints.len()
            {
                return Err(clear_error("compact fingerprint or digest"));
            }
            let (root_motion, rows) = Self::decode_tree_motion(compact, &layout, true)?;
            staged
                .install_tree_motion(&layout, &root_motion, &rows)
                .map_err(|_| clear_error("generalized motion"))?;
            staged.place_detached_sphere(
                payload,
                BodyState {
                    position: part.position,
                    orientation: part.orientation,
                    velocity: part.linear,
                    angular_velocity: part.angular,
                },
            )?;
        }
        if staged.world.broad_phase.adaptive_change_detection_margin {
            return Err(unsupported_tree());
        }
        let mut broad = BroadPhaseBvh::new();
        let mut exact_bounds = staged.world.integration_parameters;
        exact_bounds.length_unit = 0.; // set_aabb receives the saved, already fattened native box.
        for (h, bounds) in remapped_leaves {
            let c = &staged.world.colliders[h];
            let nominal = c.compute_broad_phase_aabb(
                &staged.world.integration_parameters,
                &staged.world.bodies,
            );
            let margin = 0.04 * staged.world.integration_parameters.length_unit;
            // Awake spheres only. A stale leaf can lag by the fixed skin plus
            // one native step between end-of-step pose and last broad-phase update.
            let rb = &staged.world.bodies[c.parent().ok_or_else(invalid)?];
            let motion = (rb.linvel().length()
                + rb.angvel().length()
                    * c.position_wrt_parent()
                        .ok_or_else(invalid)?
                        .translation
                        .length())
                * STEP.as_secs_f64();
            let tolerance = 1e-9 * (1. + bounds.center().abs().max_element());
            let extra = bounds.half_extents() - nominal.half_extents();
            // Bulk-built leaves may remain tight for a stationary link. Once
            // refitted under change detection, Rapier fattens all three axes.
            let padding = if extra.abs().max_element() <= tolerance {
                0.
            } else if (extra - Vector::splat(margin)).abs().max_element() <= tolerance {
                margin
            } else {
                return Err(clear_error("broad-phase extent"));
            };
            if (bounds.center() - nominal.center()).abs().max_element()
                > padding + motion + tolerance
            {
                return Err(clear_error("broad-phase center"));
            }
            broad.set_aabb(&exact_bounds, h, bounds);
        }
        staged.world.broad_phase = broad;
        if must_be_clear && !staged.clear_grasp_space()? {
            return Err(unsupported_tree());
        }
        for ((root, payload), bytes) in groups.iter().copied().zip(members) {
            let actual = staged.encode_clear_member(root, payload)?;
            if actual != bytes {
                let offset = actual
                    .iter()
                    .zip(bytes)
                    .enumerate()
                    .skip(40)
                    .find(|(_, (a, b))| a != b)
                    .map(|p| p.0);
                return Err(PhysicsError::Native {
                    engine: "multifidelity",
                    message: format!(
                        "reconstructed member {} differs at offset {offset:?}",
                        root.raw()
                    ),
                });
            }
        }
        if let Some(mut observer) = staged.grasp_payload_observer {
            observer.incoming_velocity = staged.world.bodies[staged.bodies[&observer.parts[0]].0]
                .linvel()
                .to_array();
            observer.external_impulse = [0.; 3];
            staged.grasp_payload_observer = Some(observer);
        }
        Ok(staged)
    }
}
