// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

mod reduced;

#[derive(Clone, Copy)]
pub(crate) enum Solver {
    Impulse,
    Reduced,
}
#[derive(Clone, Copy)]
enum Handle {
    Impulse(ImpulseJointHandle),
    Reduced(MultibodyJointHandle),
}
#[derive(Clone, Copy)]
struct Construction {
    parent: Option<BodyId>,
    child: BodyId,
    frame1: Pose,
    frame2: Pose,
    solver: Solver,
}
impl Construction {
    fn data(self) -> GenericJoint {
        GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES)
            .local_frame1(self.frame1)
            .local_frame2(self.frame2)
            .contacts_enabled(false)
            .build()
    }
    fn insert(
        self,
        world: &mut PhysicsWorld,
        parent: RigidBodyHandle,
        child: RigidBodyHandle,
    ) -> Handle {
        match self.solver {
            Solver::Impulse => {
                Handle::Impulse(world.insert_impulse_joint(parent, child, self.data()))
            }
            Solver::Reduced => Handle::Reduced(
                world
                    .insert_multibody_joint(parent, child, self.data())
                    .expect("preflighted articulation topology"),
            ),
        }
    }
}
struct WorldRoot {
    handle: RigidBodyHandle,
    joint: JointId,
}
struct Joint {
    handle: Handle,
    initial: Construction,
    previous_angle: f64,
    position: f64,
}

pub(super) struct Joints {
    records: BTreeMap<JointId, Joint>,
    efforts: BTreeMap<JointId, f64>,
    anchor: Option<RigidBodyHandle>,
    world_roots: BTreeMap<BodyId, WorldRoot>,
    next: u64,
    open: bool,
}
impl Default for Joints {
    fn default() -> Self {
        Self {
            records: BTreeMap::new(),
            efforts: BTreeMap::new(),
            anchor: None,
            world_roots: BTreeMap::new(),
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
impl Joints {
    pub(super) fn add_effort(
        &mut self,
        id: JointId,
        torque: f64,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) -> Result<(), PhysicsError> {
        let joint = self.records.get(&id).ok_or(PhysicsError::MissingJoint)?;
        // Holding a world couple does not qualify effort on a moving joint axis.
        if joint
            .initial
            .parent
            .is_some_and(|parent| bodies[&parent].1.mass.is_some())
        {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "moving-parent joint effort commands",
            });
        }
        let next = self.efforts.get(&id).copied().unwrap_or(0.0) + torque;
        if !torque.is_finite() || !next.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        self.efforts.insert(id, next);
        Ok(())
    }

    pub(super) fn prepare_efforts(
        &self,
        world: &PhysicsWorld,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) -> Result<BTreeMap<BodyId, Vector>, PhysicsError> {
        let mut torques = BTreeMap::<BodyId, Vector>::new();
        for (&id, &effort) in &self.efforts {
            let joint = self.records[&id].initial;
            let rotation = joint.parent.map_or(Rotation::IDENTITY, |parent| {
                *world.bodies[bodies[&parent].0].rotation()
            });
            let torque = rotation * joint.frame1.rotation * Vector::X * effort;
            let next = torques.entry(joint.child).or_default();
            *next += torque;
            if !finite(next.to_array()) {
                return Err(PhysicsError::InvalidValue);
            }
        }
        for (&id, torque) in &torques {
            if !finite((world.bodies[bodies[&id].0].user_torque() + *torque).to_array()) {
                return Err(PhysicsError::InvalidValue);
            }
        }
        Ok(torques)
    }

    pub(super) fn consume_efforts(&mut self) {
        self.efforts.clear();
    }

    pub(super) fn update(&mut self, world: &PhysicsWorld, bodies: &BTreeMap<BodyId, BodyEntry>) {
        for joint in self.records.values_mut() {
            let current = angle(world, joint.handle, joint.initial, bodies);
            if matches!(joint.handle, Handle::Reduced(_)) {
                joint.position = current;
                continue;
            }
            // Each body's full-step rotation is capped at pi/4, hence relative
            // rotation stays below pi and nearest-angle unwrapping is unambiguous.
            let delta = (current - joint.previous_angle + std::f64::consts::PI)
                .rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            joint.position += delta;
            joint.previous_angle = current;
        }
    }
    pub(super) fn construction_open(&self) -> bool {
        self.open
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
        let anchor = initial
            .records
            .values()
            .any(|joint| joint.parent.is_none() && matches!(joint.solver, Solver::Impulse))
            .then(|| world.insert_body(RigidBodyBuilder::fixed()));
        let mut world_roots = BTreeMap::new();
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
                let parent = construction.parent.map_or_else(
                    || {
                        if matches!(construction.solver, Solver::Reduced) {
                            let root = world.insert_body(RigidBodyBuilder::fixed());
                            world_roots.insert(
                                construction.child,
                                WorldRoot {
                                    handle: root,
                                    joint: id,
                                },
                            );
                            root
                        } else {
                            anchor.expect("world joint owns an anchor")
                        }
                    },
                    |parent| bodies[&parent].0,
                );
                if world.bodies[parent].is_dynamic() {
                    *world.bodies[parent].activation_mut() = RigidBodyActivation::cannot_sleep();
                }
                let handle = construction.insert(world, parent, child);
                (
                    id,
                    Joint {
                        handle,
                        initial: construction,
                        previous_angle: angle(world, handle, construction, bodies),
                        position: 0.0,
                    },
                )
            })
            .collect();
        Self {
            records,
            efforts: BTreeMap::new(),
            anchor,
            world_roots,
            next: initial.next,
            open: true,
        }
    }
    pub(super) fn preserve_high_water(&mut self, previous: &Self) {
        self.next = self.next.max(previous.next);
    }
    pub(super) fn forget_body(&mut self, id: BodyId) -> Vec<BodyId> {
        let mut survivors = Vec::new();
        self.records.retain(|_, joint| {
            let endpoints = joint.initial;
            if endpoints.child == id {
                survivors.extend(endpoints.parent);
                false
            } else if endpoints.parent == Some(id) {
                survivors.push(endpoints.child);
                false
            } else {
                true
            }
        });
        self.efforts.retain(|id, _| self.records.contains_key(id));
        survivors
    }
    pub(super) fn release_unused_anchor(&mut self, world: &mut PhysicsWorld) {
        self.world_roots.retain(|_, root| {
            let retained = self.records.contains_key(&root.joint);
            if !retained {
                world.remove_body(root.handle);
            }
            retained
        });
        if self.records.values().all(|joint| {
            joint.initial.parent.is_some() || matches!(joint.initial.solver, Solver::Reduced)
        }) && let Some(anchor) = self.anchor.take()
        {
            world.remove_body(anchor);
        }
    }
}

fn angle(
    world: &PhysicsWorld,
    handle: Handle,
    construction: Construction,
    bodies: &BTreeMap<BodyId, BodyEntry>,
) -> f64 {
    let handle = match handle {
        Handle::Reduced(handle) => {
            let (multibody, link) = world
                .multibody_joints
                .get(handle)
                .expect("registered articulation handle");
            return multibody
                .link(link)
                .expect("registered articulation link")
                .joint
                .coords()[3];
        }
        Handle::Impulse(handle) => handle,
    };
    let parent_rotation = construction.parent.map_or(Rotation::IDENTITY, |parent| {
        *world.bodies[bodies[&parent].0].rotation()
    });
    world
        .impulse_joints
        .get(handle)
        .expect("registered joint handle")
        .data
        .as_revolute()
        .expect("registered revolute constraint")
        .angle(
            &parent_rotation,
            world.bodies[bodies[&construction.child].0].rotation(),
        )
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
        self.insert_revolute(None, child, anchor, axis, Solver::Impulse)
    }

    pub(crate) fn insert_revolute(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
        solver: Solver,
    ) -> Result<JointId, PhysicsError> {
        if !self.joints.open {
            return Err(PhysicsError::JointConstructionClosed);
        }
        let (handle, spec, ..) = self.bodies.get(&child).ok_or(PhysicsError::MissingBody)?;
        if parent == Some(child) {
            return Err(PhysicsError::InvalidJoint);
        }
        let parent_handle = parent
            .map(|id| {
                self.bodies
                    .get(&id)
                    .map(|entry| entry.0)
                    .ok_or(PhysicsError::MissingBody)
            })
            .transpose()?;
        if spec.mass.is_none() || !finite(anchor) || !finite(axis) {
            return Err(PhysicsError::InvalidJoint);
        }
        self.preflight_solver(parent, child, solver)?;
        let axis = Vector::from_array(axis)
            .try_normalize()
            .ok_or(PhysicsError::InvalidJoint)?;
        let basis = Rotation::from_rotation_arc(Vector::X, axis);
        let world_frame = Pose::from_parts(Vector::from_array(anchor), basis);
        let frame1 = parent_handle.map_or(world_frame, |parent| {
            self.world.bodies[parent].position().inverse() * world_frame
        });
        let frame2 = self.world.bodies[*handle].position().inverse() * world_frame;
        if !finite(frame1.translation.to_array())
            || !frame1.rotation.is_finite()
            || !finite(frame2.translation.to_array())
            || !frame2.rotation.is_finite()
        {
            return Err(PhysicsError::InvalidJoint);
        }
        if matches!(solver, Solver::Reduced) {
            self.preflight_velocities(parent, child, Vector::from_array(anchor), axis)?;
        }
        let next = self
            .joints
            .next
            .checked_add(1)
            .ok_or(PhysicsError::JointIdOverflow)?;
        let construction = Construction {
            parent,
            child,
            frame1,
            frame2,
            solver,
        };
        let world = &mut self.world;
        let parent = parent_handle.unwrap_or_else(|| {
            if matches!(solver, Solver::Reduced) {
                let root = world.insert_body(RigidBodyBuilder::fixed());
                self.joints.world_roots.insert(
                    child,
                    WorldRoot {
                        handle: root,
                        joint: JointId::from_raw(self.joints.next),
                    },
                );
                root
            } else {
                *self
                    .joints
                    .anchor
                    .get_or_insert_with(|| world.insert_body(RigidBodyBuilder::fixed()))
            }
        });
        if world.bodies[parent].is_dynamic() {
            *world.bodies[parent].activation_mut() = RigidBodyActivation::cannot_sleep();
        }
        // Default sleeping would stop low-amplitude articulated motion mid-cycle.
        *world.bodies[*handle].activation_mut() = RigidBodyActivation::cannot_sleep();
        let handle = construction.insert(world, parent, *handle);
        let id = JointId::from_raw(self.joints.next);
        self.joints.records.insert(
            id,
            Joint {
                handle,
                initial: construction,
                previous_angle: angle(world, handle, construction, &self.bodies),
                position: 0.0,
            },
        );
        self.joints.next = next;
        Ok(id)
    }

    /// Registered child endpoint of a revolute constraint.
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
        let (parent_rotation, parent_velocity) =
            joint
                .initial
                .parent
                .map_or((Rotation::IDENTITY, Vector::ZERO), |parent| {
                    let body = &self.world.bodies[self.bodies[&parent].0];
                    (*body.rotation(), body.angvel())
                });
        Some(JointState {
            position: joint.position,
            velocity: (body.angvel() - parent_velocity)
                .dot(parent_rotation * joint.initial.frame1.rotation * Vector::X),
        })
    }

    pub(crate) fn delete_world_joint(&mut self, id: JointId) -> bool {
        let Some(joint) = self.joints.records.remove(&id) else {
            return false;
        };
        self.joints.efforts.remove(&id);
        match joint.handle {
            Handle::Impulse(handle) => {
                self.world.remove_impulse_joint(handle);
            }
            Handle::Reduced(handle) => self.world.remove_multibody_joint(handle),
        }
        for body in joint
            .initial
            .parent
            .into_iter()
            .chain([joint.initial.child])
        {
            self.release_unconstrained_body(body);
        }
        self.joints.release_unused_anchor(&mut self.world);
        if matches!(joint.handle, Handle::Reduced(_)) {
            self.joints.initialize_velocities(&mut self.world);
        }
        true
    }

    pub(super) fn release_unconstrained_body(&mut self, id: BodyId) {
        // Construction records retain registered endpoints; removal excludes its retired body.
        let handle = self.bodies[&id].0;
        if self.world.bodies[handle].is_dynamic()
            && !self.has_joint(id).expect("registered endpoint")
        {
            *self.world.bodies[handle].activation_mut() = RigidBodyActivation::active();
        }
    }

    /// Whether a registered body participates in a solver constraint.
    pub(crate) fn has_joint(&self, id: BodyId) -> Option<bool> {
        let (handle, ..) = self.bodies.get(&id)?;
        Some(
            self.world.impulse_joints_with(*handle).next().is_some()
                || self.world.multibody_joints_with(*handle).next().is_some(),
        )
    }
}

#[cfg(test)]
#[path = "../tests/private/joints.rs"]
mod tests;
