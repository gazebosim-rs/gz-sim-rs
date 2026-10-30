// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! MuJoCo 3.11.0 rigid bodies behind the shared dynamic backend contract.
//!
//! MuJoCo compiles an immutable model, so spawning, removal and joint edits are
//! topology changes. Like upstream gz-physics, edits only update construction
//! records and mark the model dirty; one compile happens when native state is
//! next needed. Edits convert and check every value MuJoCo will receive, so an
//! admitted edit always compiles. The library is loaded at runtime; see
//! `docs/MUJOCO.md`.

use gz_physics::{
    BodyId, DynamicBackend, InitialPhysicsState, JointId, JointState, PhysicsError,
    PreparedPhysicsReset,
};
use std::{
    any::Any,
    collections::{BTreeMap, BTreeSet},
    sync::{
        Arc, OnceLock,
        atomic::{AtomicU64, Ordering},
    },
    time::Duration,
};

pub use gz_physics::{BodySpec, BodyState, Inertia, RotationSpec, Shape};

mod motion;
#[allow(unsafe_code)]
mod native;
mod validate;

use motion::{Hinge, free_coordinates, representable};
use native::{BodyJoint, BodyModel, Inertial, World};
use validate::{finite, normalized, quaternion_rotate_inverse};

const ENGINE: &str = "mujoco";

/// Upstream gz-physics' default step before the first advancing step, in seconds.
const DEFAULT_TIMESTEP: f64 = 0.001;

/// Bound on a hinge anchor's body-frame coordinates, in meters; with MuJoCo's
/// velocity limit it keeps hinge-to-world velocity conversions finite.
const MAX_ANCHOR: f64 = 1e100;

fn native_error(message: String) -> PhysicsError {
    PhysicsError::Native {
        engine: ENGINE,
        message,
    }
}

fn wxyz_to_xyzw([w, x, y, z]: [f64; 4]) -> [f64; 4] {
    [x, y, z, w]
}

#[derive(Debug, Clone, Copy)]
struct Construction {
    spec: BodySpec,
    rotation: RotationSpec,
    inertial: Inertial,
    /// Free-joint qvel for the construction velocity, checked at spawn.
    free_start: [f64; 6],
}

/// A world revolute joint expressed in its child's construction frame.
#[derive(Debug, Clone, Copy)]
struct JointConstruction {
    child: BodyId,
    hinge: Hinge,
    /// Hinge rate projected from the construction velocity, used after reset.
    start_rate: f64,
    /// Hinge rate projected from the velocity the body had when the joint was
    /// added, used by the first compile.
    rate: f64,
}

#[derive(Clone)]
struct Records {
    gravity: [f64; 3],
    bodies: BTreeMap<BodyId, Construction>,
    joints: BTreeMap<JointId, JointConstruction>,
    /// The joint of each jointed body; a body has at most one.
    hinges: BTreeMap<BodyId, JointId>,
    next_body: u64,
    next_joint: u64,
}

impl Records {
    fn joint(&self, body: BodyId) -> Option<&JointConstruction> {
        self.hinges.get(&body).map(|joint| &self.joints[joint])
    }

    fn models(&self) -> Vec<BodyModel> {
        self.bodies
            .iter()
            .map(|(id, body)| {
                let joint = match (body.spec.mass, self.joint(*id)) {
                    (None, _) => BodyJoint::Fixed,
                    (Some(_), None) => BodyJoint::Free,
                    (Some(_), Some(joint)) => BodyJoint::Hinge {
                        anchor: joint.hinge.anchor,
                        axis: joint.hinge.axis,
                    },
                };
                validate::body_model(body.spec, body.rotation, body.inertial, joint)
            })
            .collect()
    }

    /// A body's state while it is still at its construction state: its pose as
    /// given and its construction velocity, projected onto its hinge if any.
    fn construction_state(&self, id: BodyId) -> BodyState {
        let body = &self.bodies[&id];
        let (velocity, angular_velocity) = match self.joint(id) {
            None => (body.spec.velocity, body.rotation.angular_velocity),
            Some(joint) => {
                let [wx, wy, wz, vx, vy, vz] = joint.hinge.world_velocity(
                    &body.inertial,
                    body.rotation.orientation,
                    joint.start_rate,
                );
                ([vx, vy, vz], [wx, wy, wz])
            }
        };
        BodyState {
            position: body.spec.position,
            velocity,
            orientation: body.rotation.orientation,
            angular_velocity,
        }
    }
}

/// Engine state of one body captured before a topology change.
struct Snapshot {
    joint: Option<JointId>,
    qpos: Vec<f64>,
    qvel: Vec<f64>,
    position: [f64; 3],
    quaternion: [f64; 4],
    /// World [angular, linear] COM velocity.
    velocity: [f64; 6],
}

impl Snapshot {
    fn state(&self) -> BodyState {
        let [wx, wy, wz, vx, vy, vz] = self.velocity;
        BodyState {
            position: self.position,
            velocity: [vx, vy, vz],
            orientation: wxyz_to_xyzw(self.quaternion),
            angular_velocity: [wx, wy, wz],
        }
    }
}

/// A compiled world with the records it was built from.
struct Engine {
    records: Records,
    world: World,
    slots: BTreeMap<BodyId, usize>,
    /// Warning counts after the last step, to detect new warnings.
    warnings: [i32; native::WARNING_NAMES.len()],
}

impl Engine {
    /// Compiles records with every body at its construction state. Velocities
    /// were converted and checked by the edits, so only the native compile can fail.
    fn build(records: Records, timestep: f64) -> Result<Self, PhysicsError> {
        let mut world = World::compile(records.gravity, &records.models()).map_err(native_error)?;
        world.set_timestep(timestep);
        let slots: BTreeMap<_, _> = records
            .bodies
            .keys()
            .zip(1..)
            .map(|(&id, b)| (id, b))
            .collect();
        for (id, body) in &records.bodies {
            let start = match records.joint(*id) {
                Some(joint) => vec![joint.start_rate],
                None => body.free_start.to_vec(),
            };
            write_qvel(&mut world, slots[id], &start);
        }
        world.refresh();
        let warnings = world.warnings();
        Ok(Self {
            records,
            world,
            slots,
            warnings,
        })
    }

    fn snapshot(&self, id: BodyId) -> Option<Snapshot> {
        let body = *self.slots.get(&id)?;
        let (qpos, qvel) = match self.world.joint_address(body) {
            Some((qpos, dof)) => {
                let (_, count) = self.world.dofs(body);
                let width = if count == 6 { 7 } else { 1 };
                (
                    self.world.qpos()[qpos..qpos + width].to_vec(),
                    self.world.qvel()[dof..dof + count].to_vec(),
                )
            }
            None => (Vec::new(), Vec::new()),
        };
        Some(Snapshot {
            joint: self.records.hinges.get(&id).copied(),
            qpos,
            qvel,
            position: self.world.body_position(body),
            quaternion: self.world.body_quaternion(body),
            velocity: self.world.com_velocity(body),
        })
    }

    /// Builds `records` and carries each body's state into it. Bodies still at
    /// their construction state are built as such. Other bodies with unchanged
    /// joints copy MuJoCo coordinates exactly; a body that gained a hinge (only
    /// possible at its construction pose) takes the rate checked when the joint
    /// was added; a body that lost one keeps its world pose and velocity.
    fn rebuild(
        &self,
        records: Records,
        pristine: &BTreeSet<BodyId>,
        timestep: f64,
    ) -> Result<Self, PhysicsError> {
        let snapshots: Vec<_> = records
            .bodies
            .keys()
            .filter(|id| !pristine.contains(id))
            .filter_map(|&id| Some((id, self.snapshot(id)?)))
            .collect();
        let mut next = Self::build(records, timestep)?;
        for (id, snapshot) in snapshots {
            let body = next.slots[&id];
            let Some((qpos, dof)) = next.world.joint_address(body) else {
                continue;
            };
            let joint = next.records.joint(id).copied();
            if joint.is_some() && next.records.hinges.get(&id).copied() == snapshot.joint {
                next.world.qpos_mut()[qpos] = snapshot.qpos[0];
                next.world.qvel_mut()[dof] = snapshot.qvel[0];
            } else if let Some(joint) = joint {
                next.world.qvel_mut()[dof] = joint.rate;
            } else if snapshot.joint.is_none() {
                next.world.qpos_mut()[qpos..qpos + 7].copy_from_slice(&snapshot.qpos);
                next.world.qvel_mut()[dof..dof + 6].copy_from_slice(&snapshot.qvel);
            } else {
                let pose = next.world.qpos_mut();
                pose[qpos..qpos + 3].copy_from_slice(&snapshot.position);
                pose[qpos + 3..qpos + 7].copy_from_slice(&snapshot.quaternion);
                let [wx, wy, wz, vx, vy, vz] = snapshot.velocity;
                // remove_joint checks this conversion before admitting the edit.
                let free = free_coordinates(
                    wxyz_to_xyzw(snapshot.quaternion),
                    next.records.bodies[&id].inertial.center,
                    [vx, vy, vz],
                    [wx, wy, wz],
                );
                next.world.qvel_mut()[dof..dof + 6].copy_from_slice(&free);
            }
        }
        next.world.set_time(self.world.time());
        next.world.refresh();
        Ok(next)
    }
}

fn write_qvel(world: &mut World, body: usize, velocity: &[f64]) {
    let (start, count) = world.dofs(body);
    world.qvel_mut()[start..start + count].copy_from_slice(&velocity[..count]);
}

/// Whether native state must be recompiled, and how.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Edit {
    /// Carry surviving bodies' state into the new model.
    Transfer,
    /// Every body at its construction state and time zero (retained reset).
    Fresh,
}

/// A MuJoCo world stepped by the simulator's explicit Bevy schedule.
pub struct MujocoBackend {
    owner: Arc<()>,
    /// Authoritative construction records, including uncompiled edits.
    records: Records,
    /// The last committed compile.
    engine: Engine,
    edit: Option<Edit>,
    /// A compile made by a shared query while `edit` is pending.
    pending: OnceLock<Result<Engine, PhysicsError>>,
    /// Bodies at their construction state: not stepped and given no input since
    /// spawn or reset. Their state is answered from the records.
    pristine: BTreeSet<BodyId>,
    joints_open: bool,
    /// Pending world [force, torque] per body, applied at the COM for one step.
    wrenches: BTreeMap<BodyId, [f64; 6]>,
    joint_efforts: BTreeMap<JointId, f64>,
    timestep: f64,
    warnings: Vec<&'static str>,
    compiles: AtomicU64,
}

struct Prepared {
    engine: Engine,
}

impl MujocoBackend {
    /// Gravity uses Gazebo's Z-up coordinate convention; no implicit axis conversion.
    /// Fails with [`PhysicsError::Native`] when MuJoCo 3.11.0 cannot be loaded.
    pub fn new(gravity: [f64; 3]) -> Result<Self, PhysicsError> {
        if !finite(gravity) {
            return Err(PhysicsError::InvalidValue);
        }
        let records = Records {
            gravity,
            bodies: BTreeMap::new(),
            joints: BTreeMap::new(),
            hinges: BTreeMap::new(),
            next_body: 0,
            next_joint: 0,
        };
        Ok(Self {
            owner: Arc::new(()),
            engine: Engine::build(records.clone(), DEFAULT_TIMESTEP)?,
            records,
            edit: None,
            pending: OnceLock::new(),
            pristine: BTreeSet::new(),
            joints_open: true,
            wrenches: BTreeMap::new(),
            joint_efforts: BTreeMap::new(),
            timestep: DEFAULT_TIMESTEP,
            warnings: Vec::new(),
            compiles: AtomicU64::new(1),
        })
    }

    pub fn spawn(&mut self, spec: BodySpec) -> Result<BodyId, PhysicsError> {
        self.spawn_link(spec, RotationSpec::default(), None)
    }

    /// World gravity, in m/s^2.
    pub fn gravity(&self) -> [f64; 3] {
        self.records.gravity
    }

    /// Step of the most recent advancing step (initially 1 ms), in seconds.
    /// Recompiles keep it.
    pub fn timestep(&self) -> f64 {
        self.timestep
    }

    /// Recoverable MuJoCo warnings first raised by the last advancing step, such
    /// as a full contact or constraint buffer. They do not fault the run. MuJoCo
    /// also logs the first occurrence of each per compiled model through the
    /// installed log handler.
    pub fn step_warnings(&self) -> &[&'static str] {
        &self.warnings
    }

    fn compile(&self, edit: Edit) -> Result<Engine, PhysicsError> {
        self.compiles.fetch_add(1, Ordering::Relaxed);
        match edit {
            Edit::Transfer => {
                self.engine
                    .rebuild(self.records.clone(), &self.pristine, self.timestep)
            }
            Edit::Fresh => Engine::build(self.records.clone(), self.timestep),
        }
    }

    /// The engine for pending edits, compiled once and shared with the next commit.
    fn pending_engine(&self) -> Option<&Engine> {
        self.edit.and_then(|edit| {
            self.pending
                .get_or_init(|| self.compile(edit))
                .as_ref()
                .ok()
        })
    }

    /// Commits pending edits. On failure nothing changes and the edits stay pending.
    fn settle(&mut self) -> Result<(), PhysicsError> {
        if let Some(edit) = self.edit {
            let compiled = match self.pending.take() {
                Some(compiled) => compiled,
                None => self.compile(edit),
            };
            self.engine = compiled?;
            self.edit = None;
        }
        Ok(())
    }

    fn mark(&mut self, edit: Edit) {
        self.pending = OnceLock::new();
        self.edit = Some(match self.edit {
            Some(Edit::Fresh) => Edit::Fresh,
            _ => edit,
        });
    }

    /// The committed engine when it already holds `id` with its current joint;
    /// pending edits of other bodies leave such a body's state unchanged.
    fn committed(&self, id: BodyId) -> Option<&Engine> {
        (self.edit != Some(Edit::Fresh)
            && self.engine.slots.contains_key(&id)
            && self.engine.records.hinges.get(&id) == self.records.hinges.get(&id))
        .then_some(&self.engine)
    }

    /// Compiles pending edits and returns the body's slot and inertial for an input.
    fn input_target(&mut self, id: BodyId) -> Result<(usize, Inertial), PhysicsError> {
        let inertial = self
            .records
            .bodies
            .get(&id)
            .ok_or(PhysicsError::MissingBody)?
            .inertial;
        self.settle()?;
        Ok((self.engine.slots[&id], inertial))
    }

    /// Writes checked generalized velocities for a body after an input.
    fn commit_velocity(
        &mut self,
        id: BodyId,
        body: usize,
        velocity: &[f64],
    ) -> Result<(), PhysicsError> {
        if !representable(velocity) {
            return Err(PhysicsError::InvalidValue);
        }
        write_qvel(&mut self.engine.world, body, velocity);
        self.engine.world.refresh();
        self.pristine.remove(&id);
        Ok(())
    }
}

impl DynamicBackend for MujocoBackend {
    fn engine(&self) -> &'static str {
        ENGINE
    }

    /// Admission applies MuJoCo's compiler and state limits to the exact values
    /// lowered, so an admitted body always compiles; that compile happens at the
    /// next query, input or step that needs it.
    fn spawn_link(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        let inertial = validate::spawn(spec, rotation, inertia)?;
        let free_start = free_coordinates(
            rotation.orientation,
            inertial.center,
            spec.velocity,
            rotation.angular_velocity,
        );
        if !representable(&free_start) {
            return Err(PhysicsError::InvalidValue);
        }
        let id = BodyId::from_raw(self.records.next_body);
        self.records.next_body = self
            .records
            .next_body
            .checked_add(1)
            .ok_or(PhysicsError::IdOverflow)?;
        self.records.bodies.insert(
            id,
            Construction {
                spec,
                rotation,
                inertial,
                free_start,
            },
        );
        self.pristine.insert(id);
        self.mark(Edit::Transfer);
        Ok(id)
    }

    fn is_dynamic(&self, id: BodyId) -> Option<bool> {
        let body = self.records.bodies.get(&id)?;
        Some(body.spec.mass.is_some())
    }

    /// Bodies at their construction state answer from the records without a
    /// compile, as do compiled bodies whose joint is unchanged. Only a body
    /// whose joint changed after it moved needs a compile here.
    fn state(&self, id: BodyId) -> Option<BodyState> {
        self.records.bodies.get(&id)?;
        if self.pristine.contains(&id) {
            return Some(self.records.construction_state(id));
        }
        let engine = self.committed(id).or_else(|| self.pending_engine())?;
        Some(engine.snapshot(id)?.state())
    }

    fn remove(&mut self, id: BodyId) -> bool {
        if self.records.bodies.remove(&id).is_none() {
            return false;
        }
        if let Some(joint) = self.records.hinges.remove(&id) {
            self.records.joints.remove(&joint);
            self.joint_efforts.remove(&joint);
        }
        self.wrenches.remove(&id);
        self.pristine.remove(&id);
        self.mark(Edit::Transfer);
        true
    }

    /// Replaces world-frame velocities. For a hinged body this applies the
    /// momentum-consistent projection onto the hinge, as a constraint impulse
    /// would. Generalized velocities beyond MuJoCo's mjMAXVAL are rejected.
    fn set_world_velocity(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        if !finite(linear) || !finite(angular) {
            return Err(PhysicsError::InvalidValue);
        }
        let (body, inertial) = self.input_target(id)?;
        let rotation = wxyz_to_xyzw(self.engine.world.body_quaternion(body));
        let velocity = match (self.engine.world.dofs(body), self.records.joint(id)) {
            ((_, 0), _) => return Ok(()),
            (_, Some(joint)) => vec![
                joint
                    .hinge
                    .rate(&inertial, rotation, linear, angular, false),
            ],
            (_, None) => free_coordinates(rotation, inertial.center, linear, angular).to_vec(),
        };
        self.commit_velocity(id, body, &velocity)
    }

    fn add_world_wrench(
        &mut self,
        id: BodyId,
        force: [f64; 3],
        torque: [f64; 3],
    ) -> Result<(), PhysicsError> {
        if !finite(force) || !finite(torque) {
            return Err(PhysicsError::InvalidValue);
        }
        if !self.records.bodies.contains_key(&id) {
            return Err(PhysicsError::MissingBody);
        }
        let current = self.wrenches.get(&id).copied().unwrap_or([0.0; 6]);
        let [fx, fy, fz] = force;
        let [tx, ty, tz] = torque;
        let next = [0, 1, 2, 3, 4, 5].map(|i| current[i] + [fx, fy, fz, tx, ty, tz][i]);
        if !next.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        self.wrenches.insert(id, next);
        Ok(())
    }

    /// Adds world impulses to the reported velocity. Results beyond MuJoCo's
    /// mjMAXVAL are rejected.
    fn apply_impulse(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        if !finite(linear) || !finite(angular) {
            return Err(PhysicsError::InvalidValue);
        }
        let (body, inertial) = self.input_target(id)?;
        let current = self.state(id).ok_or(PhysicsError::MissingBody)?;
        let rotation = wxyz_to_xyzw(self.engine.world.body_quaternion(body));
        let velocity = match (self.engine.world.dofs(body), self.records.joint(id)) {
            ((_, 0), _) => return Ok(()),
            ((start, _), Some(joint)) => vec![
                self.engine.world.qvel()[start]
                    + joint.hinge.rate(&inertial, rotation, linear, angular, true),
            ],
            (_, None) => {
                let spin = motion::angular_impulse_response(&inertial, rotation, angular);
                free_coordinates(
                    rotation,
                    inertial.center,
                    [0, 1, 2].map(|i| current.velocity[i] + linear[i] / inertial.mass),
                    [0, 1, 2].map(|i| current.angular_velocity[i] + spin[i]),
                )
                .to_vec()
            }
        };
        self.commit_velocity(id, body, &velocity)
    }

    /// Replaces the child's free joint with a hinge. A body supports one world
    /// joint; a second returns [`PhysicsError::Unsupported`]. The body's current
    /// and construction velocities are projected onto the hinge here; rates
    /// beyond MuJoCo's mjMAXVAL return [`PhysicsError::InvalidValue`].
    fn add_world_revolute(
        &mut self,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        if !self.joints_open {
            return Err(PhysicsError::JointConstructionClosed);
        }
        let body = self
            .records
            .bodies
            .get(&child)
            .ok_or(PhysicsError::MissingBody)?;
        if body.spec.mass.is_none() || !finite(anchor) || !finite(axis) {
            return Err(PhysicsError::InvalidJoint);
        }
        let axis = normalized(axis).ok_or(PhysicsError::InvalidJoint)?;
        let offset = [0, 1, 2].map(|i| anchor[i] - body.spec.position[i]);
        let orientation = body.rotation.orientation;
        let hinge = Hinge {
            anchor: quaternion_rotate_inverse(orientation, offset),
            axis: quaternion_rotate_inverse(orientation, axis),
        };
        if !hinge.anchor.into_iter().all(|c| c.abs() <= MAX_ANCHOR) || !finite(hinge.axis) {
            return Err(PhysicsError::InvalidJoint);
        }
        if self.records.hinges.contains_key(&child) {
            return Err(PhysicsError::Unsupported {
                engine: ENGINE,
                capability: "more than one world joint on a body",
            });
        }
        let start_rate = hinge.rate(
            &body.inertial,
            orientation,
            body.spec.velocity,
            body.rotation.angular_velocity,
            false,
        );
        // Construction is open, so the body is at its construction pose; a body
        // given a velocity since then has compiled and keeps it until projected.
        let current = match self.pristine.contains(&child) {
            true => None,
            false => self.engine.snapshot(child),
        };
        let rate = current.map_or(start_rate, |snapshot| {
            let [wx, wy, wz, vx, vy, vz] = snapshot.velocity;
            hinge.rate(
                &body.inertial,
                orientation,
                [vx, vy, vz],
                [wx, wy, wz],
                false,
            )
        });
        if !representable(&[start_rate, rate]) {
            return Err(PhysicsError::InvalidValue);
        }
        let id = JointId::from_raw(self.records.next_joint);
        self.records.next_joint = self
            .records
            .next_joint
            .checked_add(1)
            .ok_or(PhysicsError::JointIdOverflow)?;
        self.records.joints.insert(
            id,
            JointConstruction {
                child,
                hinge,
                start_rate,
                rate,
            },
        );
        self.records.hinges.insert(child, id);
        self.mark(Edit::Transfer);
        Ok(id)
    }

    fn validate_joint_effort(&self, id: JointId, torque: f64) -> Result<(), PhysicsError> {
        if !self.records.joints.contains_key(&id) {
            return Err(PhysicsError::MissingJoint);
        }
        if !torque.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }

    fn add_joint_effort(&mut self, id: JointId, torque: f64) -> Result<(), PhysicsError> {
        self.validate_joint_effort(id, torque)?;
        let next = self.joint_efforts.get(&id).copied().unwrap_or(0.0) + torque;
        if !next.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        self.joint_efforts.insert(id, next);
        Ok(())
    }

    fn joint_child(&self, id: JointId) -> Option<BodyId> {
        self.records.joints.get(&id).map(|joint| joint.child)
    }

    /// Answers without a compile: an uncompiled joint is at angle zero with the
    /// rate its first compile writes, which is exactly what MuJoCo then reports.
    fn joint_state(&self, id: JointId) -> Option<JointState> {
        let joint = self.records.joints.get(&id)?;
        if self.pristine.contains(&joint.child) {
            return Some(JointState {
                position: 0.0,
                velocity: joint.start_rate,
            });
        }
        let Some(engine) = self.committed(joint.child) else {
            return Some(JointState {
                position: 0.0,
                velocity: joint.rate,
            });
        };
        let (qpos, dof) = engine.world.joint_address(engine.slots[&joint.child])?;
        Some(JointState {
            position: engine.world.qpos()[qpos],
            velocity: engine.world.qvel()[dof],
        })
    }

    /// Restores the child's free joint, keeping its current world pose and velocity.
    /// Refuses removal if that velocity exceeds MuJoCo's free-joint limits.
    fn remove_joint(&mut self, id: JointId) -> bool {
        let Some(joint) = self.records.joints.get(&id) else {
            return false;
        };
        let child = joint.child;
        if !self.state(child).is_some_and(|state| {
            representable(&free_coordinates(
                state.orientation,
                self.records.bodies[&child].inertial.center,
                state.velocity,
                state.angular_velocity,
            ))
        }) || self.settle().is_err()
        {
            return false;
        }
        // Even an unstepped hinge has projected the construction velocity.
        // Retain its compiled state instead of restoring the free start record.
        self.pristine.remove(&child);
        self.records.joints.remove(&id);
        self.joint_efforts.remove(&id);
        self.records.hinges.remove(&child);
        self.mark(Edit::Transfer);
        true
    }

    fn is_jointed(&self, id: BodyId) -> Option<bool> {
        self.records.bodies.get(&id)?;
        Some(self.records.hinges.contains_key(&id))
    }

    /// Compiles pending edits, then applies pending wrenches for one mj_step.
    /// A compile failure, possible only through an engine fault, returns
    /// [`PhysicsError::Native`] and changes nothing. MuJoCo's bad qpos/qvel/qacc
    /// warnings return `Native` too: MuJoCo has then already reset its data,
    /// moving every body to its initial configuration and time to zero, so the
    /// run cannot continue. Other warnings are in [`MujocoBackend::step_warnings`].
    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        if dt.is_zero() {
            return Ok(());
        }
        self.settle()?;
        self.joints_open = false;
        self.pristine.clear();
        self.timestep = dt.as_secs_f64();
        let efforts: Vec<_> = self
            .joint_efforts
            .iter()
            .map(|(&id, &effort)| {
                let body = self.engine.slots[&self.records.joints[&id].child];
                // Every admitted world hinge compiles to one generalized coordinate.
                let (_, dof) = self
                    .engine
                    .world
                    .joint_address(body)
                    .expect("compiled hinge");
                (dof, effort)
            })
            .collect();
        let engine = &mut self.engine;
        if !efforts.is_empty() {
            let applied = engine.world.qfrc_applied_mut();
            for &(dof, effort) in &efforts {
                applied[dof] = effort;
            }
        }
        let applied = engine.world.xfrc_applied_mut();
        for (id, wrench) in &self.wrenches {
            let body = engine.slots[id];
            applied[6 * body..6 * body + 6].copy_from_slice(wrench);
        }
        self.wrenches.clear();
        let start = engine.world.time();
        engine.world.step(self.timestep);
        engine.world.xfrc_applied_mut().fill(0.0);
        if !efforts.is_empty() {
            let applied = engine.world.qfrc_applied_mut();
            for &(dof, _) in &efforts {
                applied[dof] = 0.0;
            }
        }
        // MuJoCo's own reset after a bad state zeroes time and all counts, then
        // counts the cause once; time going back reveals a repeated reset.
        let reset = engine.world.time() < start + 0.5 * self.timestep;
        let counts = engine.world.warnings();
        let previous = std::mem::replace(&mut engine.warnings, counts);
        let (fatal, recoverable) = native::classify_warnings(counts, previous, reset);
        self.warnings = recoverable;
        if fatal.is_empty() {
            self.joint_efforts.clear();
            return Ok(());
        }
        Err(native_error(format!(
            "MuJoCo reset its data after {} warnings: every body returned to its \
             initial configuration and time to zero",
            fatal.join(", ")
        )))
    }

    /// Restores remaining bodies and joints to their construction state; the
    /// model is recompiled when next needed.
    fn reset(&mut self) -> Result<(), PhysicsError> {
        self.mark(Edit::Fresh);
        self.pristine = self.records.bodies.keys().copied().collect();
        self.wrenches.clear();
        self.joint_efforts.clear();
        self.warnings.clear();
        self.joints_open = true;
        Ok(())
    }

    fn initial_state(&self) -> InitialPhysicsState {
        InitialPhysicsState::new(&self.owner, self.records.clone())
    }

    fn owns_initial_state(&self, initial: &InitialPhysicsState) -> bool {
        initial.is_owned_by(&self.owner)
    }

    /// Compiles the initial state now so that committing it cannot fail.
    fn prepare_initial(
        &self,
        initial: &InitialPhysicsState,
    ) -> Result<PreparedPhysicsReset, PhysicsError> {
        let records = initial.data::<Records>(&self.owner)?.clone();
        self.compiles.fetch_add(1, Ordering::Relaxed);
        Ok(PreparedPhysicsReset::new(
            &self.owner,
            Prepared {
                engine: Engine::build(records, self.timestep)?,
            },
        ))
    }

    fn apply_initial(&mut self, prepared: PreparedPhysicsReset) -> Result<(), PhysicsError> {
        let mut engine = prepared.into_data::<Prepared>(&self.owner)?.engine;
        let records = &mut engine.records;
        records.next_body = records.next_body.max(self.records.next_body);
        records.next_joint = records.next_joint.max(self.records.next_joint);
        engine.world.set_timestep(self.timestep);
        self.records = engine.records.clone();
        self.pristine = self.records.bodies.keys().copied().collect();
        self.engine = engine;
        self.edit = None;
        self.pending = OnceLock::new();
        self.wrenches.clear();
        self.joint_efforts.clear();
        self.warnings.clear();
        self.joints_open = true;
        Ok(())
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

#[cfg(test)]
#[path = "../tests/private/backend.rs"]
mod tests;
