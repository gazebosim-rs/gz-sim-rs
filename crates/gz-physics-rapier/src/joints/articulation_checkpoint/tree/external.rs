// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Preserve the native island and contacts between one articulated tree and one independent body.
use super::*;

pub const MAX_GROUNDED_TREE_LINKS: usize = 8;
pub const GROUNDED_TREE_IMAGE_LIMIT: usize = super::super::grounded::GROUNDED_REVOLUTE_LIMIT;

pub const MAX_PAYLOAD_TREE_LINKS: usize = MAX_GROUNDED_TREE_LINKS;
pub const PAYLOAD_TREE_IMAGE_LIMIT: usize = GROUNDED_TREE_IMAGE_LIMIT;
pub const GRASP_TREE_IMAGE_LIMIT: usize = PAYLOAD_TREE_IMAGE_LIMIT;

fn external_graph_codec(profile: TreeExternal) -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit((profile.image_limit() - 40) as u64)
        .reject_trailing_bytes()
}
impl RapierBackend {
    fn payload_of_tree(&self) -> Result<BodyId, PhysicsError> {
        let mut candidates = self.bodies.iter().filter(|(_, (_, s, ..))| {
            s.articulation.is_none() && s.collider.shape().as_ball().is_some()
        });
        let (&body, _) = candidates.next().ok_or_else(unsupported_tree)?;
        if candidates.next().is_some() {
            return Err(unsupported_tree());
        }
        Ok(body)
    }
    /// Native world-axis impulse on an articulated link from a specified free spherical payload.
    /// Other assemblies and their independent payloads may share the ordinary world.
    pub fn payload_tree_reaction(
        &self,
        link: BodyId,
        payload: BodyId,
    ) -> Result<MovingReaction, PhysicsError> {
        let (handle, spec, ..) = self.bodies.get(&payload).ok_or(PhysicsError::MissingBody)?;
        if link == payload
            || spec.articulation.is_some()
            || spec.collider.shape().as_ball().is_none()
            || !self.world.bodies[*handle].is_dynamic()
        {
            return Err(unsupported_tree());
        }
        self.link_reaction(link, payload)
            .map(Self::canonical_pair_reaction)
    }
    /// Observe the aggregate rigid friction impulse between two independent payloads.
    /// All other bodies must be articulated. Gravity and damping on the free bodies must be zero.
    pub fn configure_grasp_payload_pair(
        &mut self,
        mut parts: [BodyId; 2],
    ) -> Result<(), PhysicsError> {
        if !self.joints.open || self.world.gravity != Vector::ZERO || parts[0] == parts[1] {
            return Err(unsupported_tree());
        }
        parts.sort_unstable();
        for id in parts {
            let (h, initial, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
            let rb = &self.world.bodies[*h];
            if !rb.is_dynamic()
                || initial.articulation.is_some()
                || initial.collider.shape().as_ball().is_none()
                || rb.linear_damping() != 0.
                || rb.angular_damping() != 0.
                || !rb.mass().is_finite()
                || rb.mass() <= 0.
            {
                return Err(unsupported_tree());
            }
        }
        if self
            .bodies
            .iter()
            .any(|(id, b)| !parts.contains(id) && b.1.articulation.is_none())
        {
            return Err(unsupported_tree());
        }
        self.grasp_payload_observer = Some(GraspPayloadObserver {
            parts,
            incoming_velocity: self.world.bodies[self.bodies[&parts[0]].0]
                .linvel()
                .to_array(),
            external_impulse: [0.; 3],
        });
        Ok(())
    }
    /// Last native contact impulse on the first sphere from the second, in world axes.
    pub fn sphere_pair_reaction(
        &self,
        a: BodyId,
        b: BodyId,
    ) -> Result<MovingReaction, PhysicsError> {
        let mut reaction = self.sphere_pair_reaction_legacy(a, b)?;
        if reaction.normal_impulse > 0.
            && self.bodies[&a].1.articulation.is_none()
            && self.bodies[&b].1.articulation.is_none()
        {
            let observer = self.grasp_payload_observer.ok_or_else(unsupported_tree)?;
            if !observer.parts.contains(&a) || !observer.parts.contains(&b) {
                return Err(unsupported_tree());
            }
            let part = observer.parts[0];
            let rb = &self.world.bodies[self.bodies[&part].0];
            // Rigid twist friction reports its final substep tangent, but omits the
            // total tangent. Every remaining partner is a multibody and has complete
            // Coulomb impulse writeback. Subtract the known external force and all of
            // those signed impulses from the free body's actual momentum change.
            let mut total = (rb.linvel() - Vector::from_array(observer.incoming_velocity))
                * rb.mass()
                - Vector::from_array(observer.external_impulse);
            for (&id, (_, initial, ..)) in &self.bodies {
                if initial.articulation.is_some() {
                    let other = self.contact_reaction(id, part)?;
                    total += Vector::from_array(other.normal) * other.normal_impulse
                        + Vector::from_array(other.tangent_impulse);
                } else if !observer.parts.contains(&id) {
                    return Err(unsupported_tree());
                }
            }
            if a != part {
                total = -total;
            }
            let n = Vector::from_array(reaction.normal);
            reaction.tangent_impulse = (total - n * total.dot(n)).to_array();
        }
        Ok(reaction)
    }
    /// Raw Rapier impulse writeback used by the original two-gripper archive.
    /// Rigid sphere pairs on that profile retain the historical tangent readback.
    pub fn sphere_pair_reaction_legacy(
        &self,
        a: BodyId,
        b: BodyId,
    ) -> Result<MovingReaction, PhysicsError> {
        if a == b {
            return Err(invalid());
        }
        for id in [a, b] {
            let (handle, spec, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
            if !self.world.bodies[*handle].is_dynamic() || spec.collider.shape().as_ball().is_none()
            {
                return Err(unsupported_tree());
            }
        }
        Ok(Self::canonical_pair_reaction(self.contact_reaction(a, b)?))
    }
    fn canonical_pair_reaction(r: MovingReaction) -> MovingReaction {
        if !r.active
            && r.normal_impulse == 0.
            && r.warmstart_normal_impulse == 0.
            && r.tangent_impulse == [0.; 3]
        {
            MovingReaction::default()
        } else {
            r
        }
    }

    fn floor_of_tree(&self) -> Result<BodyId, PhysicsError> {
        self.bodies
            .iter()
            .find_map(|(&id, (_, spec, ..))| spec.collider.shape().as_halfspace().map(|_| id))
            .ok_or_else(unsupported_tree)
    }
    pub(super) fn check_tree_dynamics(&self, layout: &TreeLayout) -> Result<(), PhysicsError> {
        let world = &self.world;
        let (tree, _) = world
            .multibody_joints
            .get(layout.handle)
            .ok_or_else(invalid)?;
        let n = tree.ndofs();
        if !tree.couplings().is_empty()
            || tree.generalized_velocity().len() != n
            || !tree.generalized_velocity().iter().all(|x| x.is_finite())
            || [tree.armature(), tree.frictions(), tree.damping()]
                .into_iter()
                .any(|v| v.len() != n || v.iter().any(|x| *x != 0.))
            || world.bodies.iter().any(|(_, b)| {
                !b.translation().is_finite()
                    || rotation(b.rotation().to_array()).is_err()
                    || !b.linvel().is_finite()
                    || !b.angvel().is_finite()
            })
        {
            return Err(invalid());
        }
        Ok(())
    }
    fn check_external_graph(
        &self,
        layout: &TreeLayout,
        external: TreeExternal,
    ) -> Result<(), PhysicsError> {
        self.check_tree_dynamics(layout)?;
        let world = &self.world;
        let (h, ..) = self.bodies.get(&external.id()).ok_or_else(invalid)?;
        let [floor_collider] = world.bodies.get(*h).ok_or_else(invalid)?.colliders() else {
            return Err(invalid());
        };
        // Self-collision filtering can retain isolated broad-phase pairs. Each solved
        // contact must belong to the external body.
        if world.narrow_phase.contact_pairs().any(|p| {
            p.has_any_active_contact()
                && p.collider1 != *floor_collider
                && p.collider2 != *floor_collider
        }) {
            return Err(invalid());
        }
        let max_points = if matches!(external, TreeExternal::Robot { .. }) {
            4
        } else {
            1
        };
        super::super::grounded::validate_manifold_points(world, max_points)
    }
    /// Capture a free torque-controlled tree of 3–32 centered boxes and its fixed support plane.
    pub fn checkpoint_grounded_robot_tree(
        &self,
        root: BodyId,
        ground: BodyId,
    ) -> Result<Vec<u8>, PhysicsError> {
        self.checkpoint_external_tree(TreeExternal::Robot { root, ground })
    }
    /// Restore the complete native multibody, support contacts and signed pending joint efforts.
    pub fn restore_grounded_robot_tree(
        &mut self,
        root: BodyId,
        ground: BodyId,
        bytes: &[u8],
    ) -> Result<(), PhysicsError> {
        self.restore_external_tree(bytes, TreeExternal::Robot { root, ground })
    }
    /// Capture bounded, same-build Rapier data, including tangential and normal warm starts.
    pub fn checkpoint_grounded_revolute_tree(&self) -> Result<Vec<u8>, PhysicsError> {
        self.checkpoint_external_tree(TreeExternal::Floor(self.floor_of_tree()?))
    }
    /// Capture native contacts with one independent dynamic spherical payload in zero gravity.
    pub fn checkpoint_payload_revolute_tree(&self) -> Result<Vec<u8>, PhysicsError> {
        self.checkpoint_external_tree(TreeExternal::Payload(self.payload_of_tree()?))
    }
    /// Expanded native broad-phase box for a released gripper link or its free sphere.
    /// A coordinator can union boxes at the start and end of a privately prepared step
    /// and discard that step if any cross-owner envelopes intersect.
    pub fn grasp_collision_bounds(
        &self,
        id: BodyId,
        dt: Duration,
    ) -> Result<([f64; 3], [f64; 3]), PhysicsError> {
        if dt != STEP {
            return Err(unsupported_tree());
        }
        self.validate_moving_body(id)?;
        let rb = &self.world.bodies[self.bodies[&id].0];
        let [handle] = rb.colliders() else {
            return Err(unsupported_tree());
        };
        let c = &self.world.colliders[*handle];
        if rb.soft_ccd_prediction() != 0.
            || !rb.is_enabled()
            || !c.is_enabled()
            || c.contact_skin() < 0.
            || !c.contact_skin().is_finite()
        {
            return Err(unsupported_tree());
        }
        let local = c.position_wrt_parent().ok_or_else(unsupported_tree)?;
        let pose = rb.position() * local;
        if *c.position() != pose {
            return Err(unsupported_tree());
        }
        let mut params = self.world.integration_parameters;
        params.dt = dt.as_secs_f64();
        if !params.length_unit.is_finite()
            || params.length_unit <= 0.
            || !rb.linvel().is_finite()
            || !rb.angvel().is_finite()
            || !rb.center_of_mass().is_finite()
        {
            return Err(invalid());
        }
        let native = c.compute_broad_phase_aabb(&params, &self.world.bodies);
        if !native.mins.is_finite() || !native.maxs.is_finite() {
            return Err(invalid());
        }
        // In Rapier 0.36.1 a leaf stays at its old fat AABB throughout movements
        // within its change-detection skin. A previous center may be one skin
        // away from the current center; the stored leaf extends one skin farther.
        // A round collider with soft CCD off has constant broad-phase extents.
        let unit = params.length_unit;
        let skin = if self.world.broad_phase.adaptive_change_detection_margin {
            (0.125 * native.extents().min_element()).clamp(0.04 * unit, 0.25 * unit)
        } else {
            0.04 * unit
        };
        let displacement = rb.linvel() * params.dt;
        let arm = pose.translation.distance(rb.center_of_mass());
        let rotation = arm * (params.dt * rb.angvel().length()).min(2.);
        let scale = native
            .mins
            .abs()
            .max(native.maxs.abs())
            .max(displacement.abs())
            .max_element()
            .max(1.);
        let expansion = 2. * skin + rotation + 64. * f64::EPSILON * scale;
        let lo = native.mins + displacement.min(Vector::ZERO) - Vector::splat(expansion);
        let hi = native.maxs + displacement.max(Vector::ZERO) + Vector::splat(expansion);
        if !lo.is_finite() || !hi.is_finite() || !expansion.is_finite() {
            return Err(invalid());
        }
        Ok((lo.to_array(), hi.to_array()))
    }
    /// Export the motorized tree alone after every cross-body contact cache drains.
    /// The awake isotropic payload must be constructed last so the original tree's
    /// body identities and construction signature also identify the detached world.
    pub fn detached_grasp_tree_image(&self, payload: BodyId) -> Result<Vec<u8>, PhysicsError> {
        if self.payload_of_tree()? != payload {
            return Err(unsupported_tree());
        }
        self.require_drained_sphere(payload)?;
        for (&id, (handle, ..)) in &self.bodies {
            self.validate_moving_body(id)?;
            let body = &self.world.bodies[*handle];
            if body.soft_ccd_prediction() != 0. {
                return Err(unsupported_tree());
            }
        }
        let geometry = |id: BodyId| -> Result<_, PhysicsError> {
            let rb = &self.world.bodies[self.bodies[&id].0];
            let [handle] = rb.colliders() else {
                return Err(unsupported_tree());
            };
            let co = &self.world.colliders[*handle];
            let pose = rb.position() * co.position_wrt_parent().ok_or_else(unsupported_tree)?;
            let radius =
                co.shape().as_ball().ok_or_else(unsupported_tree)?.radius + co.contact_skin();
            Ok((pose.translation, radius))
        };
        let (part, radius) = geometry(payload)?;
        let padding = self.world.integration_parameters.prediction_distance();
        for &id in self.bodies.keys().filter(|&&k| k != payload) {
            let (link, r) = geometry(id)?;
            if part.distance(link) <= radius + r + padding {
                return Err(unsupported_tree());
            }
        }
        let layout = self.tree_layout(true, Some(TreeExternal::Releasing(payload)))?;
        self.encode_tree(layout, true)
    }
    /// Capture a motor-driven grasp with a separately moving payload and all warm contacts.
    pub fn checkpoint_grasp_revolute_tree(&self) -> Result<Vec<u8>, PhysicsError> {
        self.checkpoint_external_tree(TreeExternal::Grasp(self.payload_of_tree()?))
    }
    fn checkpoint_external_tree(&self, external: TreeExternal) -> Result<Vec<u8>, PhysicsError> {
        if self.joints.open || self.world.integration_parameters.dt != STEP.as_secs_f64() {
            return Err(unsupported_tree());
        }
        let layout =
            self.tree_layout(matches!(external, TreeExternal::Grasp(_)), Some(external))?;
        self.check_external_graph(&layout, external)?;
        let mut payload = layout.construction.to_vec();
        self.append_tree_motion(&layout, &mut payload)?;
        external_graph_codec(external)
            .serialize_into(&mut payload, &self.world)
            .map_err(|_| invalid())?;
        if payload.len() + 40 > external.image_limit() {
            return Err(invalid());
        }
        let mut bytes = Vec::with_capacity(40 + payload.len());
        bytes.extend_from_slice(external.magic());
        bytes.extend_from_slice(&Sha256::digest(&payload));
        bytes.extend_from_slice(&payload);
        Ok(bytes)
    }

    pub(super) fn append_tree_motion(
        &self,
        layout: &TreeLayout,
        payload: &mut Vec<u8>,
    ) -> Result<(), PhysicsError> {
        let (tree, _) = self
            .world
            .multibody_joints
            .get(layout.handle)
            .ok_or_else(invalid)?;
        for (i, id) in layout.joints.iter().enumerate() {
            let q = self.joints.records[id].position;
            let link = tree.link(i + 1).ok_or_else(invalid)?;
            let row = JointMotion {
                angle: q,
                velocity: if self.joints.records[id].initial.exact_lock {
                    0.
                } else {
                    *tree
                        .generalized_velocity()
                        .get(link.assembly_id())
                        .ok_or_else(invalid)?
                },
                effort: self.joints.efforts.get(id).copied(),
            };
            row.validate()?;
            if q.to_bits() != link.joint.coords()[3].to_bits() {
                return Err(invalid());
            }
            append(payload, &row)?;
        }
        Ok(())
    }

    /// Restore one committed same-build graph into fresh, identically authored construction.
    pub fn restore_grounded_revolute_tree(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.restore_external_tree(bytes, TreeExternal::Floor(self.floor_of_tree()?))
    }
    /// Restore a same-build dynamic payload, articulated links and all their native contacts.
    pub fn restore_payload_revolute_tree(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.restore_external_tree(bytes, TreeExternal::Payload(self.payload_of_tree()?))
    }
    /// Restore both persistent motor settings and native payload contact history.
    pub fn restore_grasp_revolute_tree(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.restore_external_tree(bytes, TreeExternal::Grasp(self.payload_of_tree()?))
    }
    fn restore_external_tree(
        &mut self,
        bytes: &[u8],
        external: TreeExternal,
    ) -> Result<(), PhysicsError> {
        if !self.joints.open || !self.joints.efforts.is_empty() {
            return Err(unsupported_tree());
        }
        if !(72..=external.image_limit()).contains(&bytes.len())
            || &bytes[..8] != external.magic()
            || bytes[8..40] != Sha256::digest(&bytes[40..])[..]
        {
            return Err(invalid());
        }
        let layout =
            self.tree_layout(matches!(external, TreeExternal::Grasp(_)), Some(external))?;
        if bytes[40..72] != layout.construction {
            return Err(invalid());
        }
        let mut input = &bytes[72..];
        let mut rows = Vec::with_capacity(layout.joints.len());
        for _ in &layout.joints {
            let row: JointMotion = codec()
                .deserialize_from(&mut input)
                .map_err(|_| invalid())?;
            row.validate()?;
            rows.push(row);
        }
        let world: PhysicsWorld = external_graph_codec(external)
            .deserialize(input)
            .map_err(|_| invalid())?;
        if world.gravity != self.world.gravity
            || world.bodies.len() != self.bodies.len()
            || world.colliders.len() != self.bodies.len()
            || world.integration_parameters
                != (IntegrationParameters {
                    dt: STEP.as_secs_f64(),
                    ..self.world.integration_parameters
                })
        {
            return Err(invalid());
        }
        self.validate_companion_bodies(&world, external.companion(), false)?;
        if matches!(external, TreeExternal::Robot { .. }) {
            for (h, ..) in self.bodies.values() {
                let initial = &self.world.bodies[*h];
                let restored = &world.bodies[*h];
                let c = &self.world.colliders[initial.colliders()[0]];
                let d = &world.colliders[restored.colliders()[0]];
                if restored.additional_solver_iterations() != initial.additional_solver_iterations()
                    || restored.is_ccd_enabled() != initial.is_ccd_enabled()
                    || restored.soft_ccd_prediction() != initial.soft_ccd_prediction()
                    || d.active_events() != c.active_events()
                    || d.contact_force_event_threshold() != c.contact_force_event_threshold()
                {
                    return Err(invalid());
                }
            }
        }
        let mut staged = Self::new_multifidelity(self.world.gravity.to_array())?;
        staged.owner = Arc::clone(&self.owner);
        staged.allow_isolation = self.allow_isolation;
        staged.apply_initial(self.prepare_initial(&self.initial_state())?)?;
        staged.world = world;
        staged.joints.open = false;
        let actual =
            staged.tree_layout(matches!(external, TreeExternal::Grasp(_)), Some(external))?;
        if actual.construction != layout.construction || actual.joints != layout.joints {
            return Err(invalid());
        }
        staged.install_graph_motion(&actual, rows)?;
        if staged
            .checkpoint_external_tree(external)
            .map_err(|_| invalid())?
            != bytes
        {
            return Err(invalid());
        }
        *self = staged;
        Ok(())
    }
    pub(super) fn install_graph_motion(
        &mut self,
        layout: &TreeLayout,
        rows: Vec<JointMotion>,
    ) -> Result<(), PhysicsError> {
        let (tree, _) = self
            .world
            .multibody_joints
            .get(layout.handle)
            .ok_or_else(invalid)?;
        for (i, (&id, row)) in layout.joints.iter().zip(rows).enumerate() {
            let link = tree.link(i + 1).ok_or_else(invalid)?;
            let rate = if self.joints.records[&id].initial.exact_lock {
                0.
            } else {
                *tree
                    .generalized_velocity()
                    .get(link.assembly_id())
                    .ok_or_else(invalid)?
            };
            if row.angle.to_bits() != link.joint.coords()[3].to_bits()
                || rate.to_bits() != row.velocity.to_bits()
            {
                return Err(invalid());
            }
            self.joints
                .records
                .get_mut(&id)
                .ok_or_else(invalid)?
                .position = row.angle;
            if let Some(e) = row.effort {
                self.joints.efforts.insert(id, e);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../../../../tests/private/grounded_tree_checkpoint.rs"]
mod tests;
