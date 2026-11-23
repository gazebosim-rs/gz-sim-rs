// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Generalized coordinates of one isolated, free-rooted tree of revolutes, optionally with finite angle stops.
use super::*;

type TreeMotion = (RootMotion, Vec<(JointMotion, Option<MotorRow>)>);

const TREE_MAGIC: &[u8; 8] = b"GZFTRE01";
mod clear;
mod warm;
pub use clear::CLEAR_GRASP_MEMBER_IMAGE_LIMIT;
pub use warm::WARM_GRASP_MEMBER_IMAGE_LIMIT;
mod external;
mod fleet;
pub use external::{
    GRASP_TREE_IMAGE_LIMIT, GROUNDED_TREE_IMAGE_LIMIT, MAX_GROUNDED_TREE_LINKS,
    MAX_PAYLOAD_TREE_LINKS, PAYLOAD_TREE_IMAGE_LIMIT,
};
pub use fleet::GRASP_FLEET_IMAGE_LIMIT;

const PAYLOAD_MAGIC: &[u8; 8] = b"GZFPAY01";
#[derive(Clone, Copy, PartialEq, Eq)]
enum TreeExternal {
    Floor(BodyId),
    Robot { root: BodyId, ground: BodyId },
    Payload(BodyId),
    Grasp(BodyId),
    Releasing(BodyId),
}
impl TreeExternal {
    fn id(self) -> BodyId {
        match self {
            Self::Robot { ground, .. } => ground,
            Self::Floor(id) | Self::Payload(id) | Self::Grasp(id) | Self::Releasing(id) => id,
        }
    }
    fn image_limit(self) -> usize {
        if matches!(self, Self::Robot { .. }) {
            GROUNDED_ROBOT_IMAGE_LIMIT
        } else {
            GROUNDED_TREE_IMAGE_LIMIT
        }
    }
    fn companion(self) -> Companion {
        match self {
            Self::Floor(id) | Self::Robot { ground: id, .. } => Companion::Ground(id),
            Self::Payload(id) | Self::Grasp(id) | Self::Releasing(id) => Companion::Sphere(id),
        }
    }
    fn magic(self) -> &'static [u8; 8] {
        match self {
            Self::Floor(_) => GROUND_MAGIC,
            Self::Robot { .. } => b"GZHRBT01",
            Self::Payload(_) => PAYLOAD_MAGIC,
            Self::Grasp(_) => b"GZFGRP01",
            Self::Releasing(_) => MOTOR_MAGIC,
        }
    }
}
/// Full native world, bounded for a 3–32 link torque-controlled cuboid robot.
pub const GROUNDED_ROBOT_IMAGE_LIMIT: usize = 1024 * 1024;
pub const MAX_GROUNDED_ROBOT_LINKS: usize = 32;
const GROUND_MAGIC: &[u8; 8] = b"GZFGND01";
const MOTOR_MAGIC: &[u8; 8] = b"GZFMOT01";
const STOPPED_MAGIC: &[u8; 8] = b"GZFSTP01";
/// Maximum number of dynamic spherical links in this bounded native profile.
pub const MAX_FLOATING_TREE_LINKS: usize = 64;
// Envelope, two SHA-256 digests, root pose, angular coordinates and six root velocities.
const FIXED_BYTES: usize = 40 + 64 + 16 * 8;
/// Largest same-build tree image, including a pending torque for every joint.
pub const FLOATING_TREE_IMAGE_LIMIT: usize = FIXED_BYTES + 25 * (MAX_FLOATING_TREE_LINKS - 1);

/// Rapier builds internal stop rows from the current coordinate with zero initial impulse.
/// Their recipe occupies a construction digest, with no additional mutable bytes.
pub const STOPPED_TREE_IMAGE_LIMIT: usize = FLOATING_TREE_IMAGE_LIMIT;

/// A per-joint presence flag and two f64 motor parameters extend the passive image.
pub const MOTOR_TREE_IMAGE_LIMIT: usize =
    FLOATING_TREE_IMAGE_LIMIT + 17 * (MAX_FLOATING_TREE_LINKS - 1);

#[derive(Clone, Copy, Serialize, Deserialize)]
struct MotorRow {
    velocity: f64,
    force: f64,
}
impl MotorRow {
    fn install(self, data: &mut GenericJoint) -> Result<(), PhysicsError> {
        if !self.velocity.is_finite() || !self.force.is_finite() || self.force < 0.0 {
            return Err(invalid());
        }
        data.set_motor_model(JointAxis::AngX, MotorModel::AccelerationBased)
            .set_motor_velocity(JointAxis::AngX, self.velocity, 1.0)
            .set_motor_max_force(JointAxis::AngX, self.force);
        Ok(())
    }
    fn read(
        actual: &GenericJoint,
        mut initial: GenericJoint,
    ) -> Result<Option<Self>, PhysicsError> {
        let row = actual.motor(JointAxis::AngX).map(|m| Self {
            velocity: m.target_vel,
            force: m.max_force,
        });
        if let Some(row) = row {
            row.install(&mut initial)?;
        }
        // Rapier internal reduced motor rows start with zero impulse, including at
        // saturation. Refuse any other persisted coefficient, axis, or warm cache.
        if initial != *actual {
            return Err(unsupported_tree());
        }
        Ok(row)
    }
}

fn unsupported_tree() -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "multifidelity",
        capability: "isolated, self-collision-excluded floating revolute tree checkpoint",
    }
}

struct TreeLayout {
    handle: MultibodyJointHandle,
    root: RigidBodyHandle,
    /// Children in the same order as Rapier's topological links 1..N.
    joints: Vec<JointId>,
    motors: Vec<Option<MotorRow>>,
    construction: [u8; 32],
    magic: &'static [u8; 8],
}
impl TreeLayout {
    fn bind_duration(mut self, last: Option<std::time::Duration>) -> Self {
        if let Some(dt) = last {
            let mut h = Sha256::new();
            h.update(b"GZFTIM01 physical construction and last solver interval");
            h.update(self.construction);
            h.update((dt.as_nanos() as u64).to_le_bytes());
            self.construction = h.finalize().into();
        }
        self
    }
}

#[derive(Serialize, Deserialize)]
struct RootMotion {
    translation: [f64; 3],
    orientation: [f64; 4],
    angular_coordinates: [f64; 3],
    velocity: [f64; 6],
}
impl RootMotion {
    fn pose(&self) -> Result<Pose, PhysicsError> {
        if [self.translation, self.angular_coordinates]
            .into_iter()
            .any(|v| !Vector::from_array(v).length_squared().is_finite())
            || !self.velocity.iter().all(|v| (v * v).is_finite())
        {
            return Err(invalid());
        }
        Ok(Pose::from_parts(
            Vector::from_array(self.translation),
            rotation(self.orientation)?,
        ))
    }
}

#[derive(Serialize, Deserialize)]
struct JointMotion {
    angle: f64,
    velocity: f64,
    effort: Option<f64>,
}
impl JointMotion {
    fn validate(&self) -> Result<(), PhysicsError> {
        if !(self.angle * self.angle).is_finite()
            || !(self.velocity * self.velocity).is_finite()
            || self
                .effort
                .is_some_and(|e| !e.is_finite() || e.to_bits() == (-0.0f64).to_bits())
        {
            return Err(invalid());
        }
        Ok(())
    }
}

fn append<T: Serialize>(bytes: &mut Vec<u8>, value: &T) -> Result<(), PhysicsError> {
    codec().serialize_into(bytes, value).map_err(|_| invalid())
}

impl RapierBackend {
    fn floating_tree_layout(&self, motorized: bool) -> Result<TreeLayout, PhysicsError> {
        self.tree_layout(motorized, None)
    }
    fn tree_layout(
        &self,
        motorized: bool,
        external: Option<TreeExternal>,
    ) -> Result<TreeLayout, PhysicsError> {
        self.scoped_tree_layout(motorized, external, None)
    }

    /// A coupled fleet passes the collision group of one constituent articulation.
    /// Its one independent payload belongs to that group's construction fingerprint.
    fn scoped_tree_layout(
        &self,
        motorized: bool,
        external: Option<TreeExternal>,
        selected: Option<u64>,
    ) -> Result<TreeLayout, PhysicsError> {
        let total = self.bodies.len();
        let admitted = |key: BodyId| {
            selected.is_none()
                || external.is_some_and(|e| e.id() == key)
                || self.bodies.get(&key).is_some_and(|body| body.4 == selected)
        };
        let count = if selected.is_none() {
            total
        } else {
            self.bodies.keys().filter(|&&id| admitted(id)).count()
        }
        .saturating_sub(usize::from(external.is_some()));
        let robot = matches!(external, Some(TreeExternal::Robot { .. }));
        let max = if robot {
            MAX_GROUNDED_ROBOT_LINKS
        } else if external.is_some() {
            MAX_GROUNDED_TREE_LINKS
        } else {
            MAX_FLOATING_TREE_LINKS
        };
        if !(3..=max).contains(&count)
            || selected.is_none() && self.joints.records.len() != count - 1
            || self.world.bodies.len() != total
            || self.world.colliders.len() != total
            || self.native_profile.is_some()
            || self.pair_observer.is_some()
            || self.per_point_friction_enabled()
            || self.joints.anchor.is_some()
            || !self.joints.world_roots.is_empty()
            || !self.world.impulse_joints.is_empty()
            || !self.world.soft_bodies.is_empty()
            || external.is_none()
                && (self.world.gravity != Vector::ZERO
                    || self
                        .world
                        .narrow_phase
                        .contact_pairs()
                        .any(|p| p.has_any_active_contact()))
            || self.fleet_clock != FleetClock::Coupled
            || !self.spinning_assemblies.is_empty()
            || self.free_coast.is_some()
            || !self.held_wrenches.is_empty()
            || !self.pending_wrenches.is_empty()
            || self.join_queries_pending
        {
            return Err(unsupported_tree());
        }
        let mut children = BTreeMap::new();
        for (&id, joint) in &self.joints.records {
            if !admitted(joint.initial.child) {
                continue;
            }
            if !matches!(joint.handle, Handle::Reduced(_))
                || joint.initial.parent.is_none()
                || children.insert(joint.initial.child, id).is_some()
            {
                return Err(unsupported_tree());
            }
        }
        let mut roots = self.bodies.keys().filter(|id| {
            admitted(**id)
                && Some(**id) != external.map(TreeExternal::id)
                && !children.contains_key(id)
        });
        let &root_id = roots.next().ok_or_else(unsupported_tree)?;
        if roots.next().is_some()
            || self.articulation_tier(root_id)? != ArticulationTier::Dynamic
            || matches!(external, Some(TreeExternal::Robot { root, .. }) if root != root_id)
        {
            return Err(unsupported_tree());
        }
        let group = self.bodies[&root_id].4.ok_or_else(unsupported_tree)?;
        let root = self.bodies[&root_id].0;
        let tree_id = self.articulation_index(root_id)?;
        let tree = self
            .world
            .multibody_joints
            .get_multibody(tree_id)
            .ok_or_else(unsupported_tree)?;
        let exact_locks = self
            .joints
            .records
            .values()
            .filter(|r| r.initial.exact_lock && admitted(r.initial.child))
            .count();
        if tree.num_links() != count
            || tree.ndofs() != 6 + count - 1 - exact_locks
            || tree.root().joint.data != GenericJoint::default()
            || tree.root().rigid_body_handle() != root
        {
            return Err(unsupported_tree());
        }
        let magic = if let Some(external) = external {
            external.magic()
        } else if motorized {
            MOTOR_MAGIC
        } else if self
            .joints
            .records
            .values()
            .any(|j| j.initial.limits.is_some())
        {
            STOPPED_MAGIC
        } else {
            TREE_MAGIC
        };
        let mut ordered = vec![None; count - 1];
        let mut motors = if motorized {
            vec![None; count - 1]
        } else {
            Vec::new()
        };
        let next_body = if let Some(TreeExternal::Releasing(payload)) = external {
            if payload.raw().checked_add(1) != Some(self.next_id) {
                return Err(unsupported_tree());
            }
            payload.raw()
        } else {
            self.next_id
        };
        let mut hash = Sha256::new();
        let mut params = self.world.integration_parameters;
        params.dt = STEP.as_secs_f64();
        hash.update(
            codec()
                .serialize(&(
                    magic,
                    params,
                    self.allow_isolation,
                    next_body,
                    self.joints.next,
                ))
                .map_err(|_| invalid())?,
        );
        for (&key, (handle, initial, rotation, inertia, body_group)) in &self.bodies {
            if !admitted(key) {
                continue;
            }
            let rb = self.world.bodies.get(*handle).ok_or_else(invalid)?;
            if matches!(external, Some(TreeExternal::Floor(id) | TreeExternal::Robot { ground: id, .. }) if id == key)
            {
                let g = self.world.gravity;
                if !g.is_finite()
                    || g.x != 0.
                    || g.y != 0.
                    || g.z >= 0.
                    || !rb.is_fixed()
                    || !rb.is_enabled()
                    || initial
                        .collider
                        .shape()
                        .as_halfspace()
                        .is_none_or(|s| s.normal != Vector::Z)
                    || rotation.orientation != [0., 0., 0., 1.]
                    || rb.translation().to_array() != initial.position
                    || rb.rotation().to_array() != rotation.orientation
                {
                    return Err(unsupported_tree());
                }
                hash.update(
                    codec()
                        .serialize(&(
                            g,
                            key.raw(),
                            handle,
                            initial.position,
                            rotation.orientation,
                            &initial.collider,
                        ))
                        .map_err(|_| invalid())?,
                );
                continue;
            }
            if matches!(external, Some(TreeExternal::Payload(id) | TreeExternal::Grasp(id) | TreeExternal::Releasing(id)) if id == key)
            {
                if self.world.gravity != Vector::ZERO
                    || !rb.is_dynamic()
                    || !rb.is_enabled()
                    || initial.articulation.is_some()
                    || initial.collider.shape().as_ball().is_none()
                    || body_group.is_some()
                    || rb.user_force() != Vector::ZERO
                    || rb.user_torque() != Vector::ZERO
                    || self
                        .world
                        .multibody_joints
                        .rigid_body_link(*handle)
                        .is_some()
                {
                    return Err(unsupported_tree());
                }
                if matches!(external, Some(TreeExternal::Releasing(_))) {
                    if initial.can_sleep || rb.is_sleeping() {
                        return Err(unsupported_tree());
                    }
                    self.soft_sphere_mobility(key)?;
                    continue;
                }
                let inertia = inertia.as_ref().ok_or_else(unsupported_tree)?;
                hash.update(
                    codec()
                        .serialize(&(
                            key.raw(),
                            handle,
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
                            body_group,
                            &initial.collider,
                        ))
                        .map_err(|_| invalid())?,
                );
                continue;
            }
            let l = self
                .world
                .multibody_joints
                .rigid_body_link(*handle)
                .ok_or_else(unsupported_tree)?;
            let link = tree.link(l.id).ok_or_else(unsupported_tree)?;
            if l.multibody != tree_id
                || initial.articulation != Some(key)
                || *body_group != Some(group)
                || !rb.is_dynamic()
                || (external.is_none() && rb.is_sleeping())
                || rb.user_force() != Vector::ZERO
                || rb.user_torque() != Vector::ZERO
                || (if robot {
                    let current = rb
                        .colliders()
                        .first()
                        .and_then(|h| self.world.colliders.get(*h));
                    initial.collider.shape().as_cuboid().is_none()
                        || current.is_none_or(|c| {
                            c.shape().as_cuboid().map(|shape| shape.half_extents)
                                != initial
                                    .collider
                                    .shape()
                                    .as_cuboid()
                                    .map(|shape| shape.half_extents)
                                || c.position_wrt_parent() != Some(&Pose::IDENTITY)
                        })
                } else {
                    initial.collider.shape().as_ball().is_none()
                })
                || link.joint.kinematic
                || (0..6).any(|a| link.joint.spring(a) != (0., 0.))
            {
                return Err(unsupported_tree());
            }
            let inertia = inertia.as_ref().ok_or_else(unsupported_tree)?;
            hash.update(
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
            if let Some(&id) = children.get(&key) {
                let record = &self.joints.records[&id];
                let Handle::Reduced(joint_handle) = record.handle else {
                    return Err(unsupported_tree());
                };
                let c = record.initial;
                if c.exact_lock {
                    if !matches!(
                        external,
                        Some(TreeExternal::Payload(_) | TreeExternal::Grasp(_))
                    ) || c.limits != Some([0., 0.])
                        || link.joint.coords()[3] != 0.
                        || link.joint.data.user_data != super::super::EXACT_REVOLUTE_LOCK
                    {
                        return Err(unsupported_tree());
                    }
                    // Existing passive images retain their published construction fingerprint.
                    hash.update(b"exact-revolute-lock-v1");
                }
                let parent = c.parent.ok_or_else(unsupported_tree)?;
                let parent_link = link
                    .parent_id()
                    .and_then(|index| tree.link(index))
                    .ok_or_else(unsupported_tree)?;
                if l.id == 0
                    || parent_link.rigid_body_handle()
                        != self.bodies.get(&parent).ok_or_else(unsupported_tree)?.0
                    || self
                        .world
                        .multibody_joints
                        .get(joint_handle)
                        .is_none_or(|(other, index)| !std::ptr::eq(other, tree) || index != l.id)
                    || (!motorized && link.joint.data != c.data())
                    || link.joint.ndofs() != usize::from(!c.exact_lock)
                {
                    return Err(unsupported_tree());
                }
                ordered[l.id - 1] = Some(id);
                if motorized {
                    motors[l.id - 1] = MotorRow::read(&link.joint.data, c.data())?;
                }
                hash.update(
                    codec()
                        .serialize(&(id.raw(), parent.raw(), c.child.raw(), c.frame1, c.frame2))
                        .map_err(|_| invalid())?,
                );
                if magic != TREE_MAGIC {
                    hash.update(codec().serialize(&c.limits).map_err(|_| invalid())?);
                }
            }
        }
        let joints = ordered
            .into_iter()
            .collect::<Option<Vec<_>>>()
            .ok_or_else(unsupported_tree)?;
        let Handle::Reduced(handle) = self.joints.records[&joints[0]].handle else {
            return Err(unsupported_tree());
        };
        Ok(TreeLayout {
            handle,
            root,
            joints,
            motors,
            construction: hash.finalize().into(),
            magic,
        })
    }

    /// Same-build passive floating tree at a committed positive native subinterval up to 10 ms.
    /// The integer duration and complete inner graph share one digest. Legacy 1 ms decoders
    /// retain their fixed-step construction and admission rules.
    pub fn checkpoint_timed_revolute_tree(
        &self,
        last: std::time::Duration,
    ) -> Result<Vec<u8>, PhysicsError> {
        if last.is_zero() || last > std::time::Duration::from_millis(10) {
            return Err(unsupported_tree());
        }
        let motion = self.encode_tree_at(
            self.floating_tree_layout(false)?.bind_duration(Some(last)),
            false,
            last,
        )?;
        let mut out = Vec::with_capacity(48 + motion.len());
        out.extend(b"GZFTIM01");
        out.extend([0; 32]);
        out.extend((last.as_nanos() as u64).to_le_bytes());
        out.extend(motion);
        let hash: [u8; 32] = Sha256::digest(&out[40..]).into();
        out[8..40].copy_from_slice(&hash);
        Ok(out)
    }
    /// Rebuild a timed tree into fresh matching construction and return its verified last solver duration.
    pub fn restore_timed_revolute_tree(
        &mut self,
        image: &[u8],
    ) -> Result<std::time::Duration, PhysicsError> {
        if image.len() <= 48
            || image.len() > 48 + FLOATING_TREE_IMAGE_LIMIT
            || &image[..8] != b"GZFTIM01"
            || image[8..40] != Sha256::digest(&image[40..])[..]
        {
            return Err(invalid());
        }
        let last = std::time::Duration::from_nanos(u64::from_le_bytes(
            image[40..48].try_into().map_err(|_| invalid())?,
        ));
        if last.is_zero() || last > std::time::Duration::from_millis(10) {
            return Err(invalid());
        }
        self.restore_tree_at(&image[48..], false, Some(last))?;
        Ok(last)
    }
    /// Capture one dynamic 3–64 link revolute tree at a committed 1 ms interval.
    pub fn checkpoint_floating_revolute_tree(&self) -> Result<Vec<u8>, PhysicsError> {
        self.checkpoint_tree(false)
    }
    /// Capture a floating revolute tree, including each torque-bounded persistent velocity motor.
    pub fn checkpoint_motor_revolute_tree(&self) -> Result<Vec<u8>, PhysicsError> {
        self.checkpoint_tree(true)
    }
    /// Return the native velocity request on this reduced joint. Other actuation policies
    /// are rejected because they have different solver coefficients and persistence rules.
    pub fn motor_revolute_velocity(
        &self,
        id: JointId,
    ) -> Result<Option<JointVelocityMotor>, PhysicsError> {
        let j = self
            .joints
            .records
            .get(&id)
            .ok_or(PhysicsError::MissingJoint)?;
        let Handle::Reduced(handle) = j.handle else {
            return Err(unsupported_tree());
        };
        let (tree, index) = self
            .world
            .multibody_joints
            .get(handle)
            .ok_or_else(invalid)?;
        let data = &tree.link(index).ok_or_else(invalid)?.joint.data;
        Ok(
            MotorRow::read(data, j.initial.data())?.map(|r| JointVelocityMotor {
                target_velocity: r.velocity,
                max_force: r.force,
            }),
        )
    }
    fn checkpoint_tree(&self, motorized: bool) -> Result<Vec<u8>, PhysicsError> {
        let layout = self.floating_tree_layout(motorized)?;
        self.encode_tree(layout, motorized)
    }
    fn encode_tree(&self, layout: TreeLayout, motorized: bool) -> Result<Vec<u8>, PhysicsError> {
        self.encode_tree_at(layout, motorized, STEP)
    }
    fn encode_tree_at(
        &self,
        layout: TreeLayout,
        motorized: bool,
        last: std::time::Duration,
    ) -> Result<Vec<u8>, PhysicsError> {
        if self.joints.open
            || self.world.integration_parameters.dt.to_bits() != last.as_secs_f64().to_bits()
        {
            return Err(unsupported_tree());
        }
        let (tree, _) = self
            .world
            .multibody_joints
            .get(layout.handle)
            .ok_or_else(invalid)?;
        let root_body = &self.world.bodies[layout.root];
        let gv = tree.generalized_velocity();
        let root = RootMotion {
            translation: root_body.translation().to_array(),
            orientation: root_body.rotation().to_array(),
            angular_coordinates: std::array::from_fn(|i| tree.root().joint.coords()[i + 3]),
            velocity: std::array::from_fn(|i| gv[i]),
        };
        root.pose()?;
        // Bind the visible link poses/twists as an independent integrity check.
        // Descendant motion is implied by the coordinates and occupies no per-link body array.
        let mut observed = Sha256::new();
        for link in tree.links() {
            let b = &self.world.bodies[link.rigid_body_handle()];
            let twist = tree.body_jacobian(link.link_id()) * gv;
            if b.position() != link.local_to_world()
                || b.linvel() != Vector::new(twist[0], twist[1], twist[2])
                || b.angvel() != Vector::new(twist[3], twist[4], twist[5])
            {
                return Err(unsupported_tree());
            }
            let motion = LinkMotion::from_body(b);
            motion.pose()?;
            observed.update(codec().serialize(&motion).map_err(|_| invalid())?);
        }
        let mut bytes = Vec::with_capacity(
            FIXED_BYTES + (if motorized { 42 } else { 25 }) * layout.joints.len(),
        );
        bytes.extend_from_slice(layout.magic);
        bytes.extend_from_slice(&[0; 32]);
        bytes.extend_from_slice(&layout.construction);
        bytes.extend_from_slice(&observed.finalize());
        append(&mut bytes, &root)?;
        for (i, id) in layout.joints.iter().enumerate() {
            let link = tree.link(i + 1).ok_or_else(invalid)?;
            let angle = link.joint.coords()[3];
            if self.joints.records[id].position.to_bits() != angle.to_bits() {
                return Err(unsupported_tree());
            }
            let joint = JointMotion {
                angle,
                velocity: gv[link.assembly_id()],
                effort: self.joints.efforts.get(id).copied(),
            };
            joint.validate()?;
            append(&mut bytes, &joint)?;
            if motorized {
                append(&mut bytes, &layout.motors[i])?;
            }
        }
        let digest = Sha256::digest(&bytes[40..]);
        bytes[8..40].copy_from_slice(&digest);
        Ok(bytes)
    }

    /// Restore against fresh, identical construction. On failure all original handles, state
    /// and pending inputs retain their previous values.
    pub fn restore_floating_revolute_tree(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.restore_tree(bytes, false)
    }
    /// Restore the dedicated velocity-motor profile into a fresh identical tree.
    pub fn restore_motor_revolute_tree(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.restore_tree(bytes, true)
    }
    fn restore_tree(&mut self, bytes: &[u8], motorized: bool) -> Result<(), PhysicsError> {
        self.restore_tree_at(bytes, motorized, None)
    }
    fn restore_tree_at(
        &mut self,
        bytes: &[u8],
        motorized: bool,
        timed: Option<std::time::Duration>,
    ) -> Result<(), PhysicsError> {
        let last = timed.unwrap_or(STEP);

        // Legacy handoff accepts a fresh recipient. A timed managed observer may also
        // replace its previously acknowledged motion in the same checked construction.
        if (timed.is_none() && !self.joints.open) || !self.joints.efforts.is_empty() {
            return Err(unsupported_tree());
        }
        let limit = if motorized {
            MOTOR_TREE_IMAGE_LIMIT
        } else {
            FLOATING_TREE_IMAGE_LIMIT
        };
        if !(FIXED_BYTES + 34..=limit).contains(&bytes.len())
            || (motorized && &bytes[..8] != MOTOR_MAGIC)
            || (!motorized && &bytes[..8] != TREE_MAGIC && &bytes[..8] != STOPPED_MAGIC)
            || bytes[8..40] != Sha256::digest(&bytes[40..])[..]
        {
            return Err(invalid());
        }
        let layout = self.floating_tree_layout(motorized)?.bind_duration(timed);
        if layout.motors.iter().any(Option::is_some) {
            return Err(unsupported_tree());
        }
        if &bytes[..8] != layout.magic
            || bytes[40..72] != layout.construction
            || bytes.len() > FIXED_BYTES + (if motorized { 42 } else { 25 }) * layout.joints.len()
        {
            return Err(invalid());
        }
        let (root, rows) = Self::decode_tree_motion(bytes, &layout, motorized)?;
        let mut staged = Self::new_multifidelity([0.; 3])?;
        staged.owner = Arc::clone(&self.owner);
        staged.allow_isolation = self.allow_isolation;
        staged.apply_initial(self.prepare_initial(&self.initial_state())?)?;
        staged.step(last)?;
        let staged_layout = staged.floating_tree_layout(motorized)?.bind_duration(timed);
        if staged_layout.construction != layout.construction
            || staged_layout.joints != layout.joints
        {
            return Err(invalid());
        }
        staged.install_tree_motion(&staged_layout, &root, &rows)?;
        // Installing warm rows can change enabled motors and their targets. Read the
        // installed solver's layout before comparing its canonical image to the source.
        let installed = staged.floating_tree_layout(motorized)?.bind_duration(timed);
        if staged
            .encode_tree_at(installed, motorized, last)
            .map_err(|_| invalid())?
            != bytes
        {
            return Err(invalid());
        }
        *self = staged;
        Ok(())
    }
    fn decode_tree_motion(
        bytes: &[u8],
        layout: &TreeLayout,
        motorized: bool,
    ) -> Result<TreeMotion, PhysicsError> {
        let mut input = &bytes[104..];
        let root: RootMotion = codec()
            .deserialize_from(&mut input)
            .map_err(|_| invalid())?;
        root.pose()?;
        // Construction bounds the allocation and number of decoded rows.
        let mut rows = Vec::with_capacity(layout.joints.len());
        for _ in &layout.joints {
            let row: JointMotion = codec()
                .deserialize_from(&mut input)
                .map_err(|_| invalid())?;
            row.validate()?;
            let motor: Option<MotorRow> = if motorized {
                codec()
                    .deserialize_from(&mut input)
                    .map_err(|_| invalid())?
            } else {
                None
            };
            rows.push((row, motor));
        }
        if !input.is_empty() {
            return Err(invalid());
        }
        Ok((root, rows))
    }
    fn install_tree_motion(
        &mut self,
        layout: &TreeLayout,
        root: &RootMotion,
        rows: &[(JointMotion, Option<MotorRow>)],
    ) -> Result<(), PhysicsError> {
        let (tree, _) = self
            .world
            .multibody_joints
            .get_mut(layout.handle)
            .ok_or_else(invalid)?;
        self.world.bodies[layout.root].set_position(root.pose()?, false);
        let mut root_joint = MultibodyJoint::new(tree.root().joint.data, false);
        let mut displacement = [0.; 6];
        displacement[3..].copy_from_slice(&root.angular_coordinates);
        root_joint.apply_displacement(&displacement);
        tree.link_mut(0).ok_or_else(invalid)?.joint = root_joint;
        tree.generalized_velocity_mut().as_mut_slice()[..6].copy_from_slice(&root.velocity);
        for (i, (id, (row, motor))) in layout.joints.iter().zip(rows).enumerate() {
            let link = tree.link_mut(i + 1).ok_or_else(invalid)?;
            let mut data = link.joint.data;
            if let Some(m) = motor {
                m.install(&mut data)?;
            }
            let mut joint = MultibodyJoint::new(data, false);
            joint.apply_displacement(&[row.angle]);
            link.joint = joint;
            let offset = link.assembly_id();
            tree.generalized_velocity_mut()[offset] = row.velocity;
            self.joints
                .records
                .get_mut(id)
                .ok_or_else(invalid)?
                .position = row.angle;
            if let Some(effort) = row.effort {
                self.joints.efforts.insert(*id, effort);
            }
        }
        tree.forward_kinematics(&self.world.bodies, true);
        for link in tree.links() {
            self.world.bodies[link.rigid_body_handle()].set_position(*link.local_to_world(), false);
        }
        self.joints.finish_velocities(&mut self.world, false);
        self.world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut self.world.colliders);
        Ok(())
    }
}

#[cfg(test)]
#[path = "../../../tests/private/floating_tree_checkpoint.rs"]
mod tests;
