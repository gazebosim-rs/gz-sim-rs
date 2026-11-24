// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Whole-tree fidelity changes over the same reduced coordinates and link identities.
use super::*;
#[path = "articulation_coast.rs"]
mod coast;

/// How a floating reduced-coordinate tree or a world-mounted robot responds to forces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArticulationTier {
    /// The native solver accelerates each free degree of freedom.
    Dynamic,
    /// Floating trees supply a prescribed moving boundary. Mounted robots hold a stationary,
    /// unloaded equilibrium and wake on actuation or incoming contact.
    Prescribed,
}

// For torque-free transport the system center of mass travels in a straight line.
// The root COM generally traces a circle around it even for constant world omega.
pub(super) struct Transport {
    tree: MultibodyIndex,
    root: RigidBodyHandle,
    local_offset: Vector,
    system_velocity: Vector,
    omega: Vector,
}

pub(super) struct SwitchPlan {
    index: MultibodyIndex,
    members: Vec<(BodyId, RigidBodyHandle)>,
    pub(super) transport: Option<Transport>,
    supports: Vec<support_loss::LoadedSupport>,
}

impl Transport {
    /// Constant velocity of the native root COM chord for this integration interval.
    /// At a committed boundary, `None` returns its instantaneous tangent.
    pub(super) fn root_velocity(&self, world: &PhysicsWorld, step: Option<f64>) -> Vector {
        let arm = *world.bodies[self.root].rotation() * self.local_offset;
        let relative = if let Some(dt) = step {
            let speed = self.omega.length();
            let axis = self.omega / speed;
            let angle = speed * dt;
            let tangent = axis.cross(arm);
            (tangent * angle.sin() + axis.cross(tangent) * (2.0 * (0.5 * angle).sin().powi(2))) / dt
        } else {
            self.omega.cross(arm)
        };
        self.system_velocity - relative
    }
}

fn refused(capability: &'static str) -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "multifidelity",
        capability,
    }
}

fn deferred(reason: &'static str) -> PhysicsError {
    PhysicsError::TransitionDeferred { reason }
}

impl RapierBackend {
    pub(super) fn articulation_index(&self, root: BodyId) -> Result<MultibodyIndex, PhysicsError> {
        let entry = self.bodies.get(&root).ok_or(PhysicsError::MissingBody)?;
        let link = self
            .world
            .multibody_joints
            .rigid_body_link(entry.0)
            .filter(|link| link.id == 0 && entry.1.articulation == Some(root))
            .ok_or_else(|| refused("expected a floating switchable articulation root"))?;
        let tree = self
            .world
            .multibody_joints
            .get_multibody(link.multibody)
            .expect("registered articulation");
        if tree.num_links() < 2 || !self.world.bodies[entry.0].is_dynamic() {
            return Err(refused("expected a floating switchable articulation tree"));
        }
        Ok(link.multibody)
    }

    pub fn articulation_link_count(&self, root: BodyId) -> Result<usize, PhysicsError> {
        if self.mounted_robot_root(root) { return Ok(self.mounted_members(root).len()); }
        let index = self.articulation_index(root)?;
        Ok(self
            .world
            .multibody_joints
            .get_multibody(index)
            .expect("registered articulation")
            .num_links())
    }

    pub fn articulation_contains(&self, root: BodyId, member: BodyId) -> bool {
        if self.mounted_robot_root(root) { return self.robot_member(root, member); }
        let Ok(index) = self.articulation_index(root) else {
            return false;
        };
        self.bodies
            .get(&member)
            .filter(|entry| entry.1.articulation == Some(member))
            .and_then(|entry| self.world.multibody_joints.rigid_body_link(entry.0))
            .is_some_and(|link| link.multibody == index)
    }

    pub fn articulation_tier(&self, root: BodyId) -> Result<ArticulationTier, PhysicsError> {
        if self.mounted_robot_root(root) { return Ok(self.mounted_tier(root)); }
        let index = self.articulation_index(root)?;
        let tree = self
            .world
            .multibody_joints
            .get_multibody(index)
            .expect("registered articulation");
        Ok(if tree.root().joint.kinematic {
            ArticulationTier::Prescribed
        } else {
            ArticulationTier::Dynamic
        })
    }

    pub(super) fn prescribed_member(&self, id: BodyId) -> bool {
        self.bodies
            .get(&id)
            .filter(|e| e.1.articulation.is_some())
            .and_then(|entry| self.world.multibody_joints.rigid_body_link(entry.0))
            .and_then(|link| self.world.multibody_joints.get_multibody(link.multibody))
            .is_some_and(|tree| tree.root().joint.kinematic)
    }

    pub(super) fn require_dynamic_joint(&self, id: JointId) -> Result<(), PhysicsError> {
        if self
            .joint_child(id)
            .is_some_and(|body| self.prescribed_member(body))
        {
            return Err(refused("joint actuation while articulation is prescribed"));
        }
        Ok(())
    }

    /// Queued force at the center of mass and torque in world axes. A parked
    /// articulation holds these until promotion. Zero still denotes a queued input.
    pub fn pending_world_wrench(&self, id: BodyId) -> Option<([f64; 3], [f64; 3])> {
        if let Some((force, torque)) = self.held_wrenches.get(&id) {
            return Some((force.to_array(), torque.to_array()));
        }
        if !self.pending_wrenches.contains(&id) {
            return None;
        }
        let body = &self.world.bodies[self.bodies.get(&id)?.0];
        Some((body.user_force().to_array(), body.user_torque().to_array()))
    }

    pub(super) fn hold_wrench(
        &mut self,
        id: BodyId,
        force: Vector,
        torque: Vector,
    ) -> Result<(), PhysicsError> {
        let (prior_force, prior_torque) = self.held_wrenches.get(&id).copied().unwrap_or_default();
        let combined = (prior_force + force, prior_torque + torque);
        if !combined.0.is_finite() || !combined.1.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        self.held_wrenches.insert(id, combined);
        Ok(())
    }

    /// Check the same physical conditions as a demotion at this boundary. This
    /// neither mutates the assembly nor reserves eligibility for a later step.
    pub fn validate_articulation_demotion(
        &self,
        root: BodyId,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        if self.mounted_robot_root(root) { return self.validate_mounted_park(root, dt).map(drop); }
        self.prepare_switch(root, ArticulationTier::Prescribed, dt)
            .map(|_| ())
    }

    /// Prepare one read-only demotion and query multiple obstacles against the
    /// same prescribed transport the actual switch would install.
    pub fn demotion_contact_prediction(
        &self,
        root: BodyId,
        dt: Duration,
    ) -> Result<impl Fn(BodyId) -> Result<bool, PhysicsError> + '_, PhysicsError> {
        let plan = self.prepare_switch(root, ArticulationTier::Prescribed, dt)?;
        Ok(move |obstacle| self.predicts_contact_with_motion(root, obstacle, dt, plan.as_ref()))
    }

    fn prepare_switch(
        &self,
        root: BodyId,
        target: ArticulationTier,
        dt: Duration,
    ) -> Result<Option<SwitchPlan>, PhysicsError> {
        self.prepare_switch_in(root, target, dt, false)
    }
    fn prepare_switch_in(
        &self,
        root: BodyId,
        target: ArticulationTier,
        dt: Duration,
        checked_parked_neighbors: bool,
    ) -> Result<Option<SwitchPlan>, PhysicsError> {
        let index = self.articulation_index(root)?;
        if self.articulation_tier(root)? == target {
            return Ok(None);
        }
        if dt.is_zero() {
            return Err(PhysicsError::InvalidValue);
        }
        let tree = self
            .world
            .multibody_joints
            .get_multibody(index)
            .expect("registered tree");
        let mut members = Vec::with_capacity(tree.num_links());
        for link in tree.links() {
            let handle = link.rigid_body_handle();
            let body = &self.world.bodies[handle];
            let id = BodyId::from_raw(body.user_data as u64);
            let admitted = self
                .bodies
                .get(&id)
                .is_some_and(|entry| entry.0 == handle && entry.1.articulation == Some(id));
            if !admitted || self.world.impulse_joints_with(handle).next().is_some() {
                return Err(refused("mixed or incomplete articulation membership"));
            }
            if target == ArticulationTier::Prescribed {
                if let Some(parent) = link.parent_id() {
                    let parent = tree.link(parent).expect("registered joint parent");
                    if (body.angvel() - self.world.bodies[parent.rigid_body_handle()].angvel())
                        .length()
                        > 1e-8
                    {
                        return Err(deferred("articulation has moving internal joints"));
                    }
                }
                if !body.linvel().is_finite() || !body.angvel().is_finite() {
                    return Err(PhysicsError::InvalidValue);
                }
                self.validate_velocity_boundary(
                    body.linvel().to_array(),
                    body.angvel().to_array(),
                    dt,
                )
                .map_err(|_| deferred("articulation exceeds prescribed velocity limits"))?;
                if !link.is_root()
                    && tree
                        .joint_velocity(link)
                        .iter()
                        .any(|v| !v.is_finite() || v.abs() > 1e-8)
                {
                    return Err(deferred("articulation has moving internal joints"));
                }
                if let Some(limits) = link.joint.data.limits(JointAxis::AngX) {
                    let position = link.joint.coords()[3];
                    // A prescribed internal speed persists without a stop reaction.
                    // Admit an indefinite hold only for a stationary in-range joint.
                    if !(limits.min..=limits.max).contains(&position)
                        || tree.joint_velocity(link)[0] != 0.0
                        || (self.joints.construction_open()
                            && link.parent_id().is_some_and(|parent| {
                                let handle =
                                    tree.link(parent).expect("joint parent").rigid_body_handle();
                                body.angvel() != self.world.bodies[handle].angvel()
                            }))
                    {
                        return Err(deferred(
                            "bounded articulation joint requires an in-range stationary hold",
                        ));
                    }
                }
                if self.pending_wrenches.contains(&id) || !link.joint.data.motor_axes.is_empty() {
                    return Err(deferred("pending force or active articulation motor"));
                }
            }
            members.push((id, handle));
        }
        let (transport, supports) = if target == ArticulationTier::Prescribed {
            if self.joints.has_effort_for(&members) {
                return Err(deferred("pending articulation joint effort"));
            }
            let supports =
                self.check_support(root, index, &members, dt, checked_parked_neighbors)?;
            (self.spinning_transport(index, &members)?, supports)
        } else {
            (None, Vec::new())
        };

        Ok(Some(SwitchPlan {
            index,
            members,
            transport,
            supports,
        }))
    }

    /// A successful transition changes all generalized degrees of freedom together.
    /// Joint coordinates, link transforms and velocity caches retain their bits.
    pub fn switch_articulation(
        &mut self,
        root: BodyId,
        target: ArticulationTier,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        if self.mounted_robot_root(root) { return self.switch_mounted_robot(root, target, dt); }
        if let Some(plan) = self.prepare_switch(root, target, dt)? {
            self.commit_switch(root, target, dt, plan);
        }
        Ok(())
    }

    /// Number of trees with a measured, fully parked two-support interval.
    pub fn measured_parked_trees(&self) -> usize {
        self.parked_shared.len()
    }

    /// Validate independent registered parked trees under one simultaneous
    /// neighborhood proof. A true flag commits that tree to prescribed. A
    /// false result leaves every tree unchanged for ordinary individual checks.
    pub fn apply_parked_demotions(
        &mut self,
        requests: &[(BodyId, bool)],
        dt: Duration,
    ) -> Result<bool, PhysicsError> {
        if dt.is_zero() {
            return Err(PhysicsError::InvalidValue);
        }
        if requests.len() < 2 || self.parked_shared.len() < requests.len() {
            return Ok(false);
        }
        let mut seen = BTreeSet::new();
        for &(root, _) in requests {
            if !seen.insert(root) {
                return Err(PhysicsError::InvalidValue);
            }
            if !self.parked_shared.iter().any(|v| v.owns_root(root)) {
                return Ok(false);
            }
        }
        if !self.shared_group_remains_clear(dt)? {
            return Ok(false);
        }
        let mut changes = Vec::new();
        for &(root, apply) in requests {
            match self.prepare_switch_in(root, ArticulationTier::Prescribed, dt, true) {
                Ok(Some(plan)) => {
                    if apply {
                        changes.push((root, plan));
                    }
                }
                Ok(None) | Err(PhysicsError::TransitionDeferred { .. }) => return Ok(false),
                Err(err) => return Err(err),
            }
        }
        for (root, plan) in changes {
            self.commit_switch(root, ArticulationTier::Prescribed, dt, plan);
        }
        Ok(true)
    }

    fn commit_switch(
        &mut self,
        root: BodyId,
        target: ArticulationTier,
        dt: Duration,
        plan: SwitchPlan,
    ) {
        let SwitchPlan {
            index,
            members,
            transport,
            supports,
        } = plan;
        self.release_welded_projection();
        // All fallible checks precede mutation. The same multibody, coordinates,
        // base velocities, native handles and motion frames remain in place.
        let tree = self
            .world
            .multibody_joints
            .get_multibody_mut(index)
            .expect("validated tree");
        for link in tree.links_mut() {
            link.joint.kinematic = target == ArticulationTier::Prescribed;
            if link.joint.data.user_data == joints::BOUNDED_REVOLUTE {
                // All generalized inverse masses vanish in prescribed mode. The
                // dormant stop retains its bounds and resumes under the same ID.
                link.joint
                    .data
                    .limit_axes
                    .set(JointAxesMask::ANG_X, target == ArticulationTier::Dynamic);
            }
        }

        if target == ArticulationTier::Dynamic
            && self
                .free_coast
                .as_ref()
                .is_some_and(|coast| coast.root == root)
            && let Some(coast) = self.free_coast.take()
        {
            coast.materialize(&mut self.world);
        }
        if let Some(transport) = transport {
            self.spinning_assemblies.insert(root, transport);
        } else {
            self.spinning_assemblies.remove(&root);
        }
        self.solved_shared_groups
            .retain(|g| !members.iter().any(|(id, _)| *id == g.foot));
        self.support_watch.install(root, supports);
        for (id, handle) in members {
            if target == ArticulationTier::Dynamic
                && let Some((force, torque)) = self.held_wrenches.remove(&id)
            {
                let body = &mut self.world.bodies[handle];
                body.add_force(force, true);
                body.add_torque(torque, true);
                self.pending_wrenches.insert(id);
            }
            if self.retain_stationary_support(id, handle, target, dt) {
                continue;
            }
            // Regenerate manifolds whose effective mass changed on this boundary.
            // No cached contact history exists for isolated bodies.
            let collider = self.world.bodies[handle].colliders()[0];
            if self
                .world
                .narrow_phase
                .contact_pairs_with(collider)
                .next()
                .is_some()
            {
                // Replacing the foot removes all its old native point storage.
                // A previously queued reset must not reach its replacement pair.
                self.cold_support_pairs
                    .retain(|&(a, b)| a != handle && b != handle);
                let previous = collider;
                let collider = self
                    .world
                    .remove_collider(collider)
                    .expect("registered collider");
                let new = self.world.insert_collider(collider, Some(handle));
                self.support_watch
                    .replace_foot_collider(root, previous, new);
                self.world.bodies[handle]
                    .recompute_mass_properties_from_colliders(&self.world.colliders);
            }
        }
        self.parked_shared.retain(|v| !v.owns_root(root));
    }

    fn spinning_transport(
        &self,
        index: MultibodyIndex,
        members: &[(BodyId, RigidBodyHandle)],
    ) -> Result<Option<Transport>, PhysicsError> {
        let root = members[0].1;
        let base = &self.world.bodies[root];
        let omega = base.angvel();
        if omega == Vector::ZERO {
            return Ok(None);
        }
        let mut mass = 0.0;
        let mut offset = Vector::ZERO;
        let mut velocity = Vector::ZERO;
        for &(_, handle) in members {
            let link = &self.world.bodies[handle];
            mass += link.mass();
            offset += (link.center_of_mass() - base.center_of_mass()) * link.mass();
            velocity += link.linvel() * link.mass();
        }
        offset /= mass;
        velocity /= mass;
        let mut momentum = Vector::ZERO;
        for &(_, handle) in members {
            let link = &self.world.bodies[handle];
            let arm = (link.center_of_mass() - base.center_of_mass()) - offset;
            momentum += link.mass_properties().effective_angular_inertia() * link.angvel()
                + arm.cross((link.linvel() - velocity) * link.mass());
        }
        // A rigid assembly spinning about a principal axis can coast with
        // constant omega. General torque-free tumbling needs an Euler-top solve.
        let scale = omega.length() * momentum.length();
        if !scale.is_finite()
            || !offset.is_finite()
            || !velocity.is_finite()
            || omega.cross(momentum).length() > 1e-7 * scale + 1e-14
        {
            return Err(deferred(
                "torque-free motion requires a principal-axis spin",
            ));
        }
        Ok(Some(Transport {
            tree: index,
            root,
            local_offset: base.rotation().inverse() * offset,
            system_velocity: velocity,
            omega,
        }))
    }

    pub(super) fn validate_spinning_step(&self, dt: Duration) -> Result<(), PhysicsError> {
        let limit = boundary_spin_limit(dt);
        if self
            .spinning_assemblies
            .values()
            .any(|motion| motion.omega.length() > limit)
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }

    // Give Rapier the chord velocity for the upcoming step so the root lands
    // at the correct rotated offset from the translating aggregate COM. At
    // commit publish the endpoint's instantaneous tangent velocity.
    pub(super) fn transport_velocities(&mut self, step: Option<f64>) {
        for motion in self.spinning_assemblies.values() {
            let velocity = motion.root_velocity(&self.world, step);
            let tree = self
                .world
                .multibody_joints
                .get_multibody_mut(motion.tree)
                .expect("prescribed assembly");
            tree.generalized_velocity_mut()
                .rows_mut(0, 3)
                .copy_from_slice(&velocity.to_array());
        }
    }

    pub(super) fn release_for_removal(&mut self, id: BodyId) -> bool {
        if !self.prescribed_member(id) {
            return true;
        }
        let link = self
            .world
            .multibody_joints
            .rigid_body_link(self.bodies[&id].0)
            .expect("prescribed articulation link");
        let tree = self
            .world
            .multibody_joints
            .get_multibody(link.multibody)
            .expect("prescribed articulation");
        let root =
            BodyId::from_raw(self.world.bodies[tree.root().rigid_body_handle()].user_data as u64);
        self.switch_articulation(root, ArticulationTier::Dynamic, Duration::from_nanos(1))
            .is_ok()
    }

    fn check_support(
        &self,
        root: BodyId,
        index: MultibodyIndex,
        members: &[(BodyId, RigidBodyHandle)],
        dt: Duration,
        checked_parked_neighbors: bool,
    ) -> Result<Vec<support_loss::LoadedSupport>, PhysicsError> {
        let mut supports = Vec::new();
        // Resolve native collider identity only for actually loaded supports.
        let mut contact_owner = None;
        let gravity = self.world.gravity;
        let weightless = gravity == Vector::ZERO;
        let up = if weightless {
            Vector::ZERO
        } else {
            -gravity.normalize()
        };
        let origin = self.world.bodies[members[0].1].center_of_mass();
        let mut mass = 0.0;
        let mut com = Vector::ZERO;
        let mut support_impulse = 0.0;
        let mut support_moment = Vector::ZERO;
        let mut support_extent: f64 = 0.0;
        // A registered face suppresses the matching native row. Use its actual
        // application point and terminal normal force exactly once, preserving
        // the ordinary weight, moment, speed and command tolerances below.
        let registered = self
            .parked_shared
            .iter()
            .find(|v| v.owns_root(root))
            .map(|v| v.reactions(self, index, root, dt, checked_parked_neighbors))
            .transpose()?
            .flatten();
        if let Some(reactions) = registered {
            for r in reactions {
                if r.force <= 0. {
                    continue;
                }
                let offset = r
                    .point
                    .expect("positive registered force has a material point")
                    - origin;
                let impulse = r.force * self.world.integration_parameters.dt;
                support_impulse += impulse;
                support_moment += offset.cross(up) * impulse;
                support_extent = support_extent.max(offset.length());
                supports.push(support_loss::LoadedSupport::capture(
                    &self.world,
                    r.link,
                    r.support,
                    r.foot,
                    r.floor,
                ));
            }
        }
        for &(link, handle) in members {
            let body = &self.world.bodies[handle];
            mass += body.mass();
            com += (body.center_of_mass() - origin) * body.mass();
            if !weightless && (body.linvel().length() > 1e-3 || body.angvel().length() > 1e-3) {
                return Err(deferred("support requires settled assembly motion"));
            }
            for &co in body.colliders() {
                for pair in self
                    .world
                    .narrow_phase
                    .contact_pairs_with(co)
                    .filter(|p| p.has_any_active_contact())
                {
                    let first = pair.collider1 == co;
                    let other = if first {
                        pair.collider2
                    } else {
                        pair.collider1
                    };
                    if registered
                        .is_some_and(|v| v.iter().any(|r| r.foot == co && r.floor == other))
                    {
                        continue;
                    }
                    // A lifecycle edit can remove or move a floor after the last
                    // narrow-phase update but before this transition.
                    let collider = self
                        .world
                        .colliders
                        .get(other)
                        .ok_or_else(|| deferred("support contact retired since last step"))?;
                    let floor_collider = other;
                    let other = collider
                        .parent()
                        .ok_or_else(|| refused("unsupported contact owner"))?;
                    let parent = self
                        .world
                        .bodies
                        .get(other)
                        .ok_or_else(|| deferred("support contact retired since last step"))?;
                    if parent.position()
                        * collider.position_wrt_parent().expect("parented collider")
                        != *collider.position()
                    {
                        return Err(deferred("support geometry moved since last step"));
                    }
                    if self
                        .world
                        .multibody_joints
                        .rigid_body_link(other)
                        .is_some_and(|link| link.multibody == index)
                    {
                        continue;
                    }
                    if weightless || !support_loss::stationary_support(parent) {
                        return Err(deferred("active contact with an external body"));
                    }
                    if parent.is_kinematic()
                        && (self.curves.drives(other)
                            || self.support_watch.awaiting_solve.contains(&other))
                    {
                        return Err(deferred(
                            "support needs a solved stationary velocity boundary",
                        ));
                    }
                    let mut loaded = false;
                    for manifold in pair.solver_manifolds() {
                        let normal = if first {
                            -manifold.data.normal
                        } else {
                            manifold.data.normal
                        };
                        if normal.dot(up) < 0.99 {
                            return Err(deferred("non-supporting articulation contact"));
                        }
                        for i in crate::partition::solved_point_indices(manifold) {
                            let point = &manifold.points[i];
                            let arm = if first {
                                point.data.solver_dp1
                            } else {
                                point.data.solver_dp2
                            };
                            let offset = (body.center_of_mass() - origin) + arm;
                            let vertical = point.data.impulse * normal.dot(up);
                            loaded |= vertical > 0.;
                            support_impulse += vertical;
                            support_moment += offset.cross(up) * vertical;
                            support_extent = support_extent.max(offset.length());
                        }
                    }
                    if loaded {
                        if !collision::can_exchange_impulses(
                            &self.world.colliders[co],
                            collider,
                            parent.body_type(),
                        ) {
                            return Err(deferred("support contact filtered since last step"));
                        }
                        let owner = contact_owner.get_or_insert_with(|| self.contact_body_lookup());
                        let support = owner(floor_collider)
                            .ok_or_else(|| deferred("support owner retired since last step"))?;
                        supports.push(support_loss::LoadedSupport::capture(
                            &self.world,
                            link,
                            support,
                            co,
                            floor_collider,
                        ));
                    }
                }
            }
        }
        let expected = mass * gravity.length() * self.world.integration_parameters.dt;
        if !weightless
            && (!expected.is_finite()
                || !support_impulse.is_finite()
                || expected <= 0.0
                || (support_impulse / expected - 1.0).abs() > 0.05)
        {
            return Err(deferred(
                "articulation lacks measured support against gravity",
            ));
        }
        if !weightless {
            // Balancing weight alone would let a tipping object qualify. The
            // resultant floor reaction must pass near the projected system COM.
            let arm = support_moment / support_impulse - (com / mass).cross(up);
            if !arm.is_finite() || arm.length() > 0.001 + 0.05 * support_extent {
                return Err(deferred(
                    "support reaction does not balance assembly moment",
                ));
            }
        }
        Ok(supports)
    }
}
