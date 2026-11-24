// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A construction-bound moving-state image for one isolated floating revolute.
use super::*;
use bincode::Options;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

const MAGIC: &[u8; 8] = b"GZART001";
// The two encodings differ only by an eight-byte effort. Neither includes a native graph
// nor a variable-size matrix. The larger internal limit is for hashing trusted construction.
const EMPTY_BYTES: usize = 40 + 32 + 2 * 13 * 8 + 3 * 8 + 7 * 8 + 1 + 8;
const LIMIT: usize = 16 * 1024;
const STEP: Duration = Duration::from_millis(1);
fn invalid() -> PhysicsError {
    PhysicsError::InvalidValue
}
fn unsupported() -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "multifidelity",
        capability: "isolated dynamic two-link floating revolute native checkpoint",
    }
}
fn codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(LIMIT as u64)
        .reject_trailing_bytes()
}
#[derive(Serialize, Deserialize)]
struct LinkMotion {
    position: [f64; 3],
    orientation: [f64; 4],
    linear: [f64; 3],
    angular: [f64; 3],
}
impl LinkMotion {
    fn from_body(b: &RigidBody) -> Self {
        Self {
            position: b.translation().to_array(),
            orientation: b.rotation().to_array(),
            linear: b.linvel().to_array(),
            angular: b.angvel().to_array(),
        }
    }
    fn pose(&self) -> Result<Pose, PhysicsError> {
        // Native inertia, Jacobian and displacement updates take dot products.
        if [self.position, self.linear, self.angular]
            .into_iter()
            .any(|v| !Vector::from_array(v).length_squared().is_finite())
        {
            return Err(invalid());
        }
        Ok(Pose::from_parts(
            Vector::from(self.position),
            rotation(self.orientation)?,
        ))
    }
}
#[derive(Serialize, Deserialize)]
struct Image {
    construction: [u8; 32],
    links: [LinkMotion; 2],
    // Free-root angular displacement counters are distinct from its quaternion.
    root_rotation_coordinates: [f64; 3],
    velocities: [f64; 7],
    effort: Option<f64>,
    angle: f64,
}
impl Image {
    fn validate_values(&self) -> Result<(), PhysicsError> {
        for state in &self.links {
            state.pose()?;
        }
        if self.effort.is_some_and(|value| !value.is_finite())
            || !self.velocities.into_iter().all(f64::is_finite)
            || !self.angle.is_finite()
            || !finite(self.root_rotation_coordinates)
            || !Vector::from(self.root_rotation_coordinates)
                .length_squared()
                .is_finite()
        {
            return Err(invalid());
        }
        Ok(())
    }
}

#[derive(Clone, Copy)]
enum Companion {
    Ground(BodyId),
    Pair([BodyId; 2]),
    Sphere(BodyId),
    // Require a drained contact graph and emit the original hinge-only signature.
    Detaching(BodyId),
}

struct Layout {
    links: [BodyId; 2],
    joint: JointId,
    handle: MultibodyJointHandle,
    construction: [u8; 32],
}

impl RapierBackend {
    fn floating_layout(&self) -> Result<Layout, PhysicsError> {
        self.revolute_layout(None)
    }

    fn revolute_layout(&self, companion: Option<Companion>) -> Result<Layout, PhysicsError> {
        self.inspect_revolute_layout(companion, false)
    }

    fn inspect_revolute_layout(
        &self,
        companion: Option<Companion>,
        configuring_pair: bool,
    ) -> Result<Layout, PhysicsError> {
        let body_count = 2 + match companion {
            None => 0,
            Some(Companion::Pair(_)) => 2,
            Some(_) => 1,
        };
        if self.native_profile.is_some()
            || self.per_point_friction_enabled()
            || self.bodies.len() != body_count
            || self.world.bodies.len() != body_count
            || self.world.colliders.len() != body_count
            || !self.world.impulse_joints.is_empty()
            || !self.world.soft_bodies.is_empty()
            || self.joints.records.len() != 1
            || self.joints.anchor.is_some()
            || !self.joints.world_roots.is_empty()
            || (companion.is_none()
                && (self.world.gravity != Vector::ZERO
                    || self
                        .world
                        .narrow_phase
                        .contact_pairs()
                        .any(|p| p.has_any_active_contact())))
            || self.fleet_clock != FleetClock::Coupled
            || !self.spinning_assemblies.is_empty()
            || self.free_coast.is_some()
            || !self.held_wrenches.is_empty()
            || !self.pending_wrenches.is_empty()
            || self.join_queries_pending
        {
            return Err(unsupported());
        }
        let (&id, joint) = self
            .joints
            .records
            .first_key_value()
            .ok_or_else(unsupported)?;
        let Handle::Reduced(handle) = joint.handle else {
            return Err(unsupported());
        };
        let c = joint.initial;
        let root = c.parent.ok_or_else(unsupported)?;
        let links = [root, c.child];
        let (tree, n) = self
            .world
            .multibody_joints
            .get(handle)
            .ok_or_else(unsupported)?;
        if n != 1
            || tree.num_links() != 2
            || tree.ndofs() != 7
            || c.limits.is_some()
            || self.articulation_tier(root)? != ArticulationTier::Dynamic
            || self.joints.efforts.keys().any(|k| *k != id)
            || tree.link(0).unwrap().joint.data != GenericJoint::default()
            || tree.link(1).unwrap().joint.data != c.data()
        {
            return Err(unsupported());
        }
        let mut signature = Sha256::new();
        let mut parameters = self.world.integration_parameters;
        parameters.dt = STEP.as_secs_f64();
        signature.update(
            codec()
                .serialize(&(parameters, self.allow_isolation))
                .map_err(|_| invalid())?,
        );
        if let Some(Companion::Ground(ground)) = companion {
            let (h, initial, rotation, _, _) = self.bodies.get(&ground).ok_or_else(unsupported)?;
            let rb = self.world.bodies.get(*h).ok_or_else(unsupported)?;
            if self.world.gravity.x != 0.
                || self.world.gravity.y != 0.
                || self.world.gravity.z >= 0.
                || !self.world.gravity.is_finite()
                || !rb.is_fixed()
                || initial
                    .collider
                    .shape()
                    .as_halfspace()
                    .is_none_or(|s| s.normal != Vector::Z)
                || rotation.orientation != [0., 0., 0., 1.]
                || rb.translation().to_array() != initial.position
            {
                return Err(unsupported());
            }
            signature.update(
                codec()
                    .serialize(&(
                        self.world.gravity,
                        ground.raw(),
                        h,
                        initial.position,
                        rotation.orientation,
                        &initial.collider,
                    ))
                    .map_err(|_| invalid())?,
            );
        }
        let mut hinge_next = self.next_id;
        if let Some(Companion::Sphere(sphere) | Companion::Detaching(sphere)) = companion {
            if self.world.gravity != Vector::ZERO
                || self.world.integration_parameters.contact_recycling
                || self
                    .bodies
                    .values()
                    .any(|entry| entry.1.can_sleep || self.world.bodies[entry.0].is_sleeping())
                || links.iter().any(|link| link.raw() >= sphere.raw())
                || sphere.raw().checked_add(1) != Some(self.next_id)
            {
                return Err(unsupported());
            }
            self.soft_sphere_mobility(sphere)?;
            let (handle, initial, rotation, inertia, group) = &self.bodies[&sphere];
            let rb = &self.world.bodies[*handle];
            if initial.articulation.is_some()
                || rb.user_force() != Vector::ZERO
                || rb.user_torque() != Vector::ZERO
            {
                return Err(unsupported());
            }
            if matches!(companion, Some(Companion::Detaching(_))) {
                self.require_drained_sphere(sphere)?;
                hinge_next = sphere.raw();
            } else {
                signature.update(b"moving sphere v1");
                signature.update(
                    codec()
                        .serialize(&(
                            sphere.raw(),
                            handle,
                            initial.position,
                            initial.velocity,
                            initial.mass,
                            initial.can_sleep,
                            rotation.orientation,
                            rotation.angular_velocity,
                            inertia.map(|i| (i.center, i.orientation, i.tensor)),
                            group,
                            &initial.collider,
                        ))
                        .map_err(|_| invalid())?,
                );
            }
        }
        if let Some(Companion::Pair(spheres)) = companion {
            if self.world.gravity != Vector::ZERO
                || (!configuring_pair
                    && (self.world.integration_parameters.contact_recycling
                        || self.bodies.values().any(|entry| entry.1.can_sleep)))
                || !(links[0].max(links[1]) < spheres[0] && spheres[0] < spheres[1])
            {
                return Err(unsupported());
            }
            signature.update(b"two moving spheres v1");
            for id in spheres {
                let (h, recipe, rotation, inertia, group) = &self.bodies[&id];
                let rb = &self.world.bodies[*h];
                if rb.is_enabled() {
                    self.soft_sphere_mobility(id)?;
                }
                if recipe.articulation.is_some()
                    || rb.user_force() != Vector::ZERO
                    || rb.user_torque() != Vector::ZERO
                {
                    return Err(unsupported());
                }
                signature.update(
                    codec()
                        .serialize(&(
                            id.raw(),
                            h,
                            recipe.position,
                            recipe.velocity,
                            recipe.mass,
                            recipe.can_sleep,
                            rotation.orientation,
                            rotation.angular_velocity,
                            inertia.map(|i| (i.center, i.orientation, i.tensor)),
                            group,
                            &recipe.collider,
                        ))
                        .map_err(|_| invalid())?,
                );
            }
        }
        for (i, key) in links.into_iter().enumerate() {
            let &(h, ref initial, rotation, inertia, group) =
                self.bodies.get(&key).ok_or_else(unsupported)?;
            let link = tree.link(i).ok_or_else(unsupported)?;
            let rb = self.world.bodies.get(h).ok_or_else(unsupported)?;
            if initial.articulation != Some(key)
                || link.rigid_body_handle() != h
                || !rb.is_dynamic()
                || (companion.is_none() && rb.is_sleeping())
                || rb.user_force() != Vector::ZERO
                || rb.user_torque() != Vector::ZERO
                || initial.collider.shape().as_ball().is_none()
                || inertia.is_none()
                || link.joint.kinematic
                || (0..6).any(|a| link.joint.spring(a) != (0., 0.))
            {
                return Err(unsupported());
            }
            let inertia = inertia.unwrap();
            signature.update(
                codec()
                    .serialize(&(
                        key.raw(),
                        initial.position,
                        initial.velocity,
                        initial.mass,
                        initial.kinematic,
                        initial.can_sleep,
                        rotation.orientation,
                        rotation.angular_velocity,
                        inertia.center,
                        inertia.orientation,
                        inertia.tensor,
                        group,
                        &initial.collider,
                    ))
                    .map_err(|_| invalid())?,
            );
        }
        signature.update(
            codec()
                .serialize(&(id.raw(), hinge_next, self.joints.next, c.frame1, c.frame2))
                .map_err(|_| invalid())?,
        );
        Ok(Layout {
            links,
            joint: id,
            handle,
            construction: signature.finalize().into(),
        })
    }
    fn floating_motion(&self, l: &Layout) -> Result<Image, PhysicsError> {
        if self.joints.open
            || self.world.integration_parameters.dt.to_bits() != STEP.as_secs_f64().to_bits()
        {
            return Err(unsupported());
        }
        let (tree, _) = self
            .world
            .multibody_joints
            .get(l.handle)
            .ok_or_else(invalid)?;
        let j = &self.joints.records[&l.joint];
        if j.position.to_bits() != tree.link(1).ok_or_else(invalid)?.joint.coords()[3].to_bits() {
            return Err(unsupported());
        }
        // An embedder can issue a same-tick raw body change before generalized
        // coordinates have been committed. Such a mixed cut is not a native image.
        for (i, id) in l.links.iter().enumerate() {
            let rb = &self.world.bodies[self.bodies[id].0];
            let link = tree.link(i).ok_or_else(invalid)?;
            let v = tree.body_jacobian(i) * tree.generalized_velocity();
            if rb.position() != link.local_to_world()
                || rb.linvel() != Vector::new(v[0], v[1], v[2])
                || rb.angvel() != Vector::new(v[3], v[4], v[5])
            {
                return Err(unsupported());
            }
        }
        let image = Image {
            construction: l.construction,
            links: l
                .links
                .map(|id| LinkMotion::from_body(&self.world.bodies[self.bodies[&id].0])),
            root_rotation_coordinates: std::array::from_fn(|i| tree.root().joint.coords()[3 + i]),
            velocities: tree
                .generalized_velocity()
                .as_slice()
                .try_into()
                .map_err(|_| invalid())?,
            effort: self.joints.efforts.get(&l.joint).copied(),
            angle: j.position,
        };
        image.validate_values()?;
        Ok(image)
    }

    /// Same-build native profile for an isolated dynamic, floating two-sphere hinge
    /// at a committed 1 ms interval, with at most one accumulated joint effort.
    pub fn checkpoint_floating_revolute(&self) -> Result<Vec<u8>, PhysicsError> {
        self.encode_hinge_motion(&self.floating_layout()?)
    }

    fn encode_hinge_motion(&self, layout: &Layout) -> Result<Vec<u8>, PhysicsError> {
        let image = self.floating_motion(layout)?;
        let payload = codec().serialize(&image).map_err(|_| invalid())?;
        let mut bytes = Vec::with_capacity(40 + payload.len());
        bytes.extend_from_slice(MAGIC);
        bytes.extend_from_slice(&Sha256::digest(&payload));
        bytes.extend(payload);
        Ok(bytes)
    }

    /// Reconstitute the committed native coordinates against the original authored
    /// topology. A failed image leaves this backend and its existing handles untouched.
    pub fn restore_floating_revolute(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        let l = self.floating_layout()?;
        if !self.joints.open || !self.joints.efforts.is_empty() {
            return Err(unsupported());
        }
        if (bytes.len() != EMPTY_BYTES && bytes.len() != EMPTY_BYTES + 8)
            || &bytes[..8] != MAGIC
            || bytes[8..40] != Sha256::digest(&bytes[40..])[..]
        {
            return Err(invalid());
        }
        let mut raw = &bytes[40..];
        let image: Image = codec().deserialize_from(&mut raw).map_err(|_| invalid())?;
        if !raw.is_empty() || image.construction != l.construction {
            return Err(invalid());
        }
        image.validate_values()?;
        let mut staged = Self::new_multifidelity([0.; 3])?;
        // Retain the owner shared by the enclosing Bevy initial/reset resource.
        staged.owner = Arc::clone(&self.owner);
        staged.allow_isolation = self.allow_isolation;
        let prepared = self.prepare_initial(&self.initial_state())?;
        staged.apply_initial(prepared)?;
        staged.step(STEP)?;
        let Handle::Reduced(staged_handle) = staged
            .joints
            .records
            .get(&l.joint)
            .ok_or_else(invalid)?
            .handle
        else {
            return Err(invalid());
        };
        let (tree, _) = staged
            .world
            .multibody_joints
            .get_mut(staged_handle)
            .ok_or_else(invalid)?;
        // Only authored joint definitions enter the live solver. The checkpoint
        // supplies fixed-size generalized coordinates rather than native graph indices.
        for (i, state) in image.links.iter().enumerate() {
            let link = tree.link_mut(i).ok_or_else(invalid)?;
            let mut joint = MultibodyJoint::new(link.joint.data, false);
            if i == 0 {
                let mut displacement = [0.; 6];
                displacement[3..].copy_from_slice(&image.root_rotation_coordinates);
                joint.apply_displacement(&displacement);
            } else {
                joint.apply_displacement(&[image.angle]);
            }
            link.joint = joint;
            let rb = staged
                .world
                .bodies
                .get_mut(staged.bodies[&l.links[i]].0)
                .ok_or_else(invalid)?;
            rb.set_position(state.pose()?, false);
            rb.set_linvel(Vector::from(state.linear), false);
            rb.set_angvel(Vector::from(state.angular), false);
        }
        tree.generalized_velocity_mut()
            .as_mut_slice()
            .copy_from_slice(&image.velocities);
        staged.joints.finish_velocities(&mut staged.world, false);
        staged
            .world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut staged.world.colliders);
        let record = staged
            .joints
            .records
            .get_mut(&l.joint)
            .ok_or_else(invalid)?;
        record.position = image.angle;
        if let Some(e) = image.effort {
            staged.add_joint_effort(l.joint, e)?;
        }
        // This also checks the reconstructed forward kinematics and the native
        // observation rows. Canonical bytes distinguish a consumed effort from zero
        // and detect a stored twist that disagrees even at the signed-bit level.
        let staged_layout = Layout {
            handle: staged_handle,
            ..l
        };
        let motion = staged
            .floating_motion(&staged_layout)
            .map_err(|_| invalid())?;
        if codec().serialize(&motion).map_err(|_| invalid())? != bytes[40..] {
            return Err(invalid());
        }
        *self = staged;
        Ok(())
    }
}

#[cfg(test)]
#[path = "../../tests/private/articulation_checkpoint.rs"]
mod tests;

mod grounded;
pub use grounded::{GROUNDED_REVOLUTE_LIMIT, GroundReaction, MovingReaction};
mod moving;
mod pair;

mod tree;
pub use tree::{
    CLEAR_GRASP_MEMBER_IMAGE_LIMIT, FLOATING_TREE_IMAGE_LIMIT, GRASP_FLEET_IMAGE_LIMIT,
    GRASP_TREE_IMAGE_LIMIT, GROUNDED_ROBOT_IMAGE_LIMIT, GROUNDED_TREE_IMAGE_LIMIT,
    MAX_FLOATING_TREE_LINKS, MAX_GROUNDED_ROBOT_LINKS, MAX_GROUNDED_TREE_LINKS,
    MAX_PAYLOAD_TREE_LINKS, MOTOR_TREE_IMAGE_LIMIT, PAYLOAD_TREE_IMAGE_LIMIT,
    STOPPED_TREE_IMAGE_LIMIT, WARM_GRASP_MEMBER_IMAGE_LIMIT,
};

mod rotor;
