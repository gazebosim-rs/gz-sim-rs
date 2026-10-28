// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Clone, Copy)]
struct Construction {
    child: BodyId,
    frame1: Pose,
    frame2: Pose,
}
impl Construction {
    fn data(self) -> GenericJoint {
        GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES)
            .local_frame1(self.frame1)
            .local_frame2(self.frame2)
            .contacts_enabled(false)
            .build()
    }
}
struct Joint {
    handle: ImpulseJointHandle,
    initial: Construction,
    previous_angle: f64,
    position: f64,
}

pub(super) struct WorldJoints {
    records: BTreeMap<JointId, Joint>,
    anchor: Option<RigidBodyHandle>,
    next: u64,
    open: bool,
}
impl Default for WorldJoints {
    fn default() -> Self {
        Self {
            records: BTreeMap::new(),
            anchor: None,
            next: 0,
            open: true,
        }
    }
}
#[derive(Clone)]
pub(super) struct InitialJoints {
    records: BTreeMap<JointId, Construction>,
    next: u64,
}
impl WorldJoints {
    pub(super) fn update(&mut self, world: &PhysicsWorld, bodies: &BTreeMap<BodyId, BodyEntry>) {
        for joint in self.records.values_mut() {
            let current = angle(world, joint.handle, bodies[&joint.initial.child].0);
            // Rapier's retained default caps full-step rotation at pi/4. No fast-rotation
            // API is exposed, so nearest-angle unwrapping cannot miss a complete turn.
            let delta = (current - joint.previous_angle + std::f64::consts::PI)
                .rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            joint.position += delta;
            joint.previous_angle = current;
        }
    }
    pub(super) fn close(&mut self) {
        self.open = false;
    }
    pub(super) fn initial(&self) -> InitialJoints {
        InitialJoints {
            records: self
                .records
                .iter()
                .map(|(&id, j)| (id, j.initial))
                .collect(),
            next: self.next,
        }
    }
    pub(super) fn restore(
        initial: &InitialJoints,
        world: &mut PhysicsWorld,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) -> Self {
        let anchor =
            (!initial.records.is_empty()).then(|| world.insert_body(RigidBodyBuilder::fixed()));
        let records = initial
            .records
            .iter()
            .map(|(&id, &construction)| {
                // Removal retires connected records; snapshots only contain registered endpoints.
                let child = bodies
                    .get(&construction.child)
                    .expect("joint construction retains its body")
                    .0;
                *world.bodies[child].activation_mut() = RigidBodyActivation::cannot_sleep();
                let handle = world.insert_impulse_joint(
                    anchor.expect("nonempty joint map owns an anchor"),
                    child,
                    construction.data(),
                );
                (
                    id,
                    Joint {
                        handle,
                        initial: construction,
                        previous_angle: angle(world, handle, child),
                        position: 0.0,
                    },
                )
            })
            .collect();
        Self {
            records,
            anchor,
            next: initial.next,
            open: true,
        }
    }
    pub(super) fn preserve_high_water(&mut self, previous: &Self) {
        self.next = self.next.max(previous.next);
    }
    pub(super) fn forget_body(&mut self, id: BodyId) {
        self.records.retain(|_, joint| joint.initial.child != id);
    }
    pub(super) fn release_unused_anchor(&mut self, world: &mut PhysicsWorld) {
        if self.records.is_empty()
            && let Some(anchor) = self.anchor.take()
        {
            world.remove_body(anchor);
        }
    }
}

fn angle(world: &PhysicsWorld, joint: ImpulseJointHandle, child: RigidBodyHandle) -> f64 {
    world
        .impulse_joints
        .get(joint)
        .expect("registered joint handle")
        .data
        .as_revolute()
        .expect("registered world revolute constraint")
        .angle(&Rotation::IDENTITY, world.bodies[child].rotation())
}

impl RapierBackend {
    /// Construct an unbounded revolute joint from a dynamic body to the world.
    /// Anchor/axis use world coordinates; the current construction pose is angle zero.
    /// Construction closes at the first advancing step and reopens after reset.
    /// Invalid inputs leave body, joint and anchor state unchanged.
    pub(crate) fn insert_world_revolute(
        &mut self,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        if !self.joints.open {
            return Err(PhysicsError::JointConstructionClosed);
        }
        let (handle, spec, ..) = self.bodies.get(&child).ok_or(PhysicsError::MissingBody)?;
        if spec.mass.is_none() || !finite(anchor) || !finite(axis) {
            return Err(PhysicsError::InvalidJoint);
        }
        let axis = Vector::from_array(axis)
            .try_normalize()
            .ok_or(PhysicsError::InvalidJoint)?;
        let basis = Rotation::from_rotation_arc(Vector::X, axis);
        let frame1 = Pose::from_parts(Vector::from_array(anchor), basis);
        let frame2 = self.world.bodies[*handle].position().inverse() * frame1;
        if !finite(frame2.translation.to_array()) || !frame2.rotation.is_finite() {
            return Err(PhysicsError::InvalidJoint);
        }
        let next = self
            .joints
            .next
            .checked_add(1)
            .ok_or(PhysicsError::JointIdOverflow)?;
        let construction = Construction {
            child,
            frame1,
            frame2,
        };
        let world = &mut self.world;
        let parent = *self
            .joints
            .anchor
            .get_or_insert_with(|| world.insert_body(RigidBodyBuilder::fixed()));
        // Default sleeping would stop low-amplitude articulated motion mid-cycle.
        *world.bodies[*handle].activation_mut() = RigidBodyActivation::cannot_sleep();
        let handle = world.insert_impulse_joint(parent, *handle, construction.data());
        let id = JointId::from_raw(self.joints.next);
        self.joints.records.insert(
            id,
            Joint {
                handle,
                initial: construction,
                previous_angle: angle(world, handle, self.bodies[&child].0),
                position: 0.0,
            },
        );
        self.joints.next = next;
        Ok(id)
    }

    /// Registered dynamic endpoint of a world-anchored constraint.
    pub(crate) fn world_joint_child(&self, id: JointId) -> Option<BodyId> {
        self.joints
            .records
            .get(&id)
            .map(|joint| joint.initial.child)
    }

    pub(crate) fn world_joint_state(&self, id: JointId) -> Option<JointState> {
        let joint = self.joints.records.get(&id)?;
        let handle = self.bodies.get(&joint.initial.child)?.0;
        let body = &self.world.bodies[handle];
        Some(JointState {
            position: joint.position,
            velocity: body.angvel().dot(joint.initial.frame1.rotation * Vector::X),
        })
    }

    pub(crate) fn delete_world_joint(&mut self, id: JointId) -> bool {
        let Some(joint) = self.joints.records.remove(&id) else {
            return false;
        };
        self.world.remove_impulse_joint(joint.handle);
        if let Some((body, ..)) = self.bodies.get(&joint.initial.child)
            && self.world.impulse_joints_with(*body).next().is_none()
        {
            *self.world.bodies[*body].activation_mut() = RigidBodyActivation::active();
        }
        self.joints.release_unused_anchor(&mut self.world);
        true
    }

    /// Whether a registered body participates in a solver constraint.
    pub(crate) fn has_joint(&self, id: BodyId) -> Option<bool> {
        let (handle, ..) = self.bodies.get(&id)?;
        Some(self.world.impulse_joints_with(*handle).next().is_some())
    }
}

#[cfg(test)]
#[path = "../tests/private/joints.rs"]
mod tests;
