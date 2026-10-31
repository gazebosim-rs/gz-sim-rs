// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native DART 6.13 rigid bodies configured like the gz-physics dartsim plugin.
//!
//! Bodies start as independent skeletons. A world-hinged dynamic parent may take
//! one dynamic revolute child. Body and joint states are read back after each
//! mutation, so queries never enter native code.

use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, InitialPhysicsState, JointId, JointState,
    PhysicsError, PlaneSpec, PreparedPhysicsReset, RotationSpec,
};
use std::{any::Any, collections::BTreeMap, path::Path, sync::Arc, time::Duration};

mod body;
mod ffi;
mod native;

use ffi::bridge::{BodyConfig, JointEffort, Twist, Wrench};
use native::NativeWorld;

const ENGINE: &str = "dart";

struct Body {
    slot: usize,
    config: BodyConfig,
    state: BodyState,
    joint: JointState,
}

struct Joint {
    parent: Option<BodyId>,
    child: BodyId,
    /// World anchor (linear) and unit world axis (angular) at construction.
    anchor_axis: Twist,
}

/// One native world with its bodies and joints, built before it replaces another.
struct Built {
    native: NativeWorld,
    bodies: BTreeMap<BodyId, Body>,
    joints: BTreeMap<JointId, Joint>,
}

/// Construction records captured by [`DynamicBackend::initial_state`].
struct Initial {
    bodies: BTreeMap<BodyId, BodyConfig>,
    joints: BTreeMap<JointId, (Option<BodyId>, BodyId, Twist)>,
    next_body: u64,
    next_joint: u64,
}

struct Prepared {
    built: Built,
    next_body: u64,
    next_joint: u64,
}

/// A DART world with Gazebo's dartsim solver settings: ODE collision detection,
/// 10000 contacts, Dantzig boxed LCP with PGS fallback, and SDF default friction.
pub struct DartBackend {
    owner: Arc<()>,
    gravity: [f64; 3],
    world: Built,
    next_body: u64,
    next_joint: u64,
    joints_open: bool,
    /// Accumulated world force (linear) and torque (angular) for the next step.
    pending: BTreeMap<BodyId, Twist>,
    joint_efforts: BTreeMap<JointId, f64>,
}

fn finite(values: [f64; 3]) -> bool {
    values.into_iter().all(f64::is_finite)
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// Copies one body's native pose, velocities and joint coordinate into its cache.
fn read(native: &mut NativeWorld, body: &mut Body) -> Result<(), PhysicsError> {
    let out = native.read(body.slot)?;
    body.state = BodyState {
        position: out.position,
        velocity: out.velocity.linear,
        orientation: out.orientation,
        angular_velocity: out.velocity.angular,
    };
    body.joint = JointState {
        position: out.joint_position,
        velocity: out.joint_velocity,
    };
    Ok(())
}

/// The DART and ODE library directories this crate was compiled and linked
/// against, in the order of the bridge's `library_paths`.
const BUILT_LIBDIRS: [&str; 4] = [
    env!("GZ_PHYSICS_DART_LIBDIR"),
    env!("GZ_PHYSICS_DART_LIBDIR"),
    env!("GZ_PHYSICS_DART_LIBDIR"),
    env!("GZ_PHYSICS_DART_ODE_LIBDIR"),
];

/// Refuses a process that loaded DART or ODE from elsewhere: the rpath pins only
/// this crate's own link outputs, and a mismatched build would break parity.
fn check_libraries(paths: Vec<String>, expected: [&str; 4]) -> Result<(), PhysicsError> {
    if paths.len() != expected.len() {
        return Err(PhysicsError::Native {
            engine: ENGINE,
            message: format!(
                "expected {} native libraries, found {paths:?}",
                expected.len()
            ),
        });
    }
    for (path, expected) in paths.into_iter().zip(expected) {
        let expected = Path::new(expected);
        let expected_dir = expected.canonicalize().ok();
        let loaded = Path::new(&path)
            .parent()
            .and_then(|dir| dir.canonicalize().ok());
        if loaded.is_none() || loaded != expected_dir {
            return Err(PhysicsError::Native {
                engine: ENGINE,
                message: format!(
                    "loaded {path}, but this crate was built against {}",
                    expected.display()
                ),
            });
        }
    }
    Ok(())
}

impl Built {
    fn new(
        gravity: [f64; 3],
        bodies: impl IntoIterator<Item = (BodyId, BodyConfig)>,
        joints: impl IntoIterator<Item = (JointId, (Option<BodyId>, BodyId, Twist))>,
    ) -> Result<Self, PhysicsError> {
        let mut built = Self {
            native: NativeWorld::new(gravity)?,
            bodies: BTreeMap::new(),
            joints: BTreeMap::new(),
        };
        for (id, config) in bodies {
            built.insert(id, config)?;
        }
        for (id, (parent, child, anchor_axis)) in joints {
            built.attach(id, parent, child, anchor_axis)?;
        }
        Ok(built)
    }

    fn insert(&mut self, id: BodyId, config: BodyConfig) -> Result<(), PhysicsError> {
        let slot = self.native.add_body(&config)?;
        // A body spawned into another can meet contacts far outside its shape.
        if !self.native.contacts_conditioned()? {
            self.native.remove_body(slot)?;
            return Err(PhysicsError::InvalidValue);
        }
        let mut body = Body {
            slot,
            config,
            state: BodyState {
                position: [0.0; 3],
                velocity: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                angular_velocity: [0.0; 3],
            },
            joint: JointState {
                position: 0.0,
                velocity: 0.0,
            },
        };
        read(&mut self.native, &mut body)?;
        self.bodies.insert(id, body);
        Ok(())
    }

    fn attach(
        &mut self,
        id: JointId,
        parent: Option<BodyId>,
        child: BodyId,
        anchor_axis: Twist,
    ) -> Result<(), PhysicsError> {
        let parent_slot = parent.map(|id| self.bodies[&id].slot);
        let body = self
            .bodies
            .get_mut(&child)
            .ok_or(PhysicsError::MissingBody)?;
        // A refused rate or articulation contact leaves native topology unchanged.
        let attached = match parent_slot {
            Some(parent) => self
                .native
                .add_body_revolute(parent, body.slot, &anchor_axis)?,
            None => self.native.add_revolute(body.slot, &anchor_axis)?,
        };
        if !attached {
            return Err(PhysicsError::InvalidJoint);
        }
        self.joints.insert(
            id,
            Joint {
                parent,
                child,
                anchor_axis,
            },
        );
        self.refresh_all()
    }

    fn refresh_all(&mut self) -> Result<(), PhysicsError> {
        self.bodies
            .values_mut()
            .try_for_each(|body| read(&mut self.native, body))
    }

    fn records(&self) -> Initial {
        Initial {
            bodies: self
                .bodies
                .iter()
                .map(|(&id, b)| (id, b.config.clone()))
                .collect(),
            joints: self
                .joints
                .iter()
                .map(|(&id, j)| (id, (j.parent, j.child, j.anchor_axis)))
                .collect(),
            next_body: 0,
            next_joint: 0,
        }
    }
}

impl DartBackend {
    fn insert_config(&mut self, config: BodyConfig) -> Result<BodyId, PhysicsError> {
        let next = self
            .next_body
            .checked_add(1)
            .ok_or(PhysicsError::IdOverflow)?;
        let id = BodyId::from_raw(self.next_body);
        self.world.insert(id, config)?;
        self.next_body = next;
        Ok(id)
    }

    /// Gravity uses Gazebo's Z-up coordinate convention; no implicit axis conversion.
    /// Its magnitude is at most 1e60 m/s². Fails with [`PhysicsError::Native`] if
    /// the loaded DART is not the one this crate was built against.
    pub fn new(gravity: [f64; 3]) -> Result<Self, PhysicsError> {
        if !body::small(gravity) {
            return Err(PhysicsError::InvalidValue);
        }
        let mut world = Built::new(gravity, [], [])?;
        check_libraries(world.native.library_paths()?, BUILT_LIBDIRS)?;
        Ok(Self {
            owner: Arc::new(()),
            gravity,
            world,
            next_body: 0,
            next_joint: 0,
            joints_open: true,
            pending: BTreeMap::new(),
            joint_efforts: BTreeMap::new(),
        })
    }

    fn dynamic_slot(&self, id: BodyId) -> Result<Option<usize>, PhysicsError> {
        let body = self
            .world
            .bodies
            .get(&id)
            .ok_or(PhysicsError::MissingBody)?;
        Ok(body.config.mobile.then_some(body.slot))
    }

    fn replace(&mut self, built: Built) {
        self.world = built;
        self.pending.clear();
        self.joint_efforts.clear();
        self.joints_open = true;
    }
}

impl DynamicBackend for DartBackend {
    fn engine(&self) -> &'static str {
        ENGINE
    }

    fn spawn_link(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        self.insert_config(body::config(spec, rotation, inertia)?)
    }

    fn spawn_static_plane(&mut self, spec: PlaneSpec) -> Result<BodyId, PhysicsError> {
        self.insert_config(body::plane_config(spec)?)
    }

    fn is_dynamic(&self, id: BodyId) -> Option<bool> {
        self.world.bodies.get(&id).map(|body| body.config.mobile)
    }

    fn state(&self, id: BodyId) -> Option<BodyState> {
        self.world.bodies.get(&id).map(|body| body.state)
    }

    fn remove(&mut self, id: BodyId) -> bool {
        let Some(body) = self.world.bodies.get(&id) else {
            return false;
        };
        if self.world.native.remove_body(body.slot).is_err() {
            return false;
        }
        self.world.bodies.remove(&id);
        self.world
            .joints
            .retain(|_, joint| joint.child != id && joint.parent != Some(id));
        self.joint_efforts
            .retain(|id, _| self.world.joints.contains_key(id));
        self.pending.remove(&id);
        // Native removal can release the other endpoint into its own skeleton.
        if self.world.refresh_all().is_err() {
            return false;
        }
        true
    }

    /// A world-revolute body takes the joint rate whose COM twist best fits the
    /// request in the least-squares sense; free bodies take it exactly. Each
    /// vector's magnitude is at most 1e60.
    fn set_world_velocity(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        if !body::small(linear) || !body::small(angular) {
            return Err(PhysicsError::InvalidValue);
        }
        if let Some(slot) = self.dynamic_slot(id)? {
            if !self
                .world
                .native
                .set_velocity(slot, &Twist { linear, angular })?
            {
                return Err(PhysicsError::InvalidValue);
            }
            self.world.refresh_all()?;
        }
        Ok(())
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
        if self.dynamic_slot(id)?.is_none() {
            return Ok(());
        }
        let zero = Twist {
            linear: [0.0; 3],
            angular: [0.0; 3],
        };
        let current = self.pending.get(&id).copied().unwrap_or(zero);
        let next = Twist {
            linear: add(current.linear, force),
            angular: add(current.angular, torque),
        };
        // The accumulated force and torque each stay within 1e60.
        if !body::small(next.linear) || !body::small(next.angular) {
            return Err(PhysicsError::InvalidValue);
        }
        self.pending.insert(id, next);
        Ok(())
    }

    /// Free bodies change COM and angular velocity by m⁻¹·linear and I⁻¹·angular;
    /// a world-revolute body changes its joint rate by M⁻¹·Jᵀ·impulse. A result
    /// above 1e60 is rejected and leaves the velocities unchanged.
    fn apply_impulse(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        if !finite(linear) || !finite(angular) {
            return Err(PhysicsError::InvalidValue);
        }
        if let Some(slot) = self.dynamic_slot(id)? {
            if !self
                .world
                .native
                .apply_impulse(slot, &Twist { linear, angular })?
            {
                return Err(PhysicsError::InvalidValue);
            }
            self.world.refresh_all()?;
        }
        Ok(())
    }

    /// DART joints form trees, so one body takes at most one world revolute.
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
            .world
            .bodies
            .get(&child)
            .ok_or(PhysicsError::MissingBody)?;
        let length = axis.into_iter().map(|v| v * v).sum::<f64>().sqrt();
        // An anchor inside the collision bounds keeps the hinge lever arm, and the
        // joint frames DART composes from it, small and precise.
        if !body.config.mobile || !body::inside(anchor) || !positive_normal(length) {
            return Err(PhysicsError::InvalidJoint);
        }
        let unit = axis.map(|v| v / length);
        if !body::hinge_conditioned(&body.config, anchor, unit) {
            return Err(PhysicsError::InvalidJoint);
        }
        if self.is_jointed(child) == Some(true) {
            return Err(PhysicsError::Unsupported {
                engine: ENGINE,
                capability: "more than one world joint on a body",
            });
        }
        let next = self
            .next_joint
            .checked_add(1)
            .ok_or(PhysicsError::JointIdOverflow)?;
        let id = JointId::from_raw(self.next_joint);
        let anchor_axis = Twist {
            linear: anchor,
            angular: unit,
        };
        self.world.attach(id, None, child, anchor_axis)?;
        self.next_joint = next;
        Ok(id)
    }

    fn add_revolute(
        &mut self,
        parent: BodyId,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        if !self.joints_open {
            return Err(PhysicsError::JointConstructionClosed);
        }
        let parent_body = self
            .world
            .bodies
            .get(&parent)
            .ok_or(PhysicsError::MissingBody)?;
        let child_body = self
            .world
            .bodies
            .get(&child)
            .ok_or(PhysicsError::MissingBody)?;
        // One world root and one new leaf. This also excludes cycles and branches.
        let world_root = self
            .world
            .joints
            .values()
            .any(|j| j.child == parent && j.parent.is_none());
        if parent == child
            || !parent_body.config.mobile
            || !child_body.config.mobile
            || !world_root
            || self.is_jointed(child) == Some(true)
            || self.world.joints.values().any(|j| j.parent == Some(parent))
        {
            return Err(PhysicsError::Unsupported {
                engine: ENGINE,
                capability: "body-to-body revolute joint",
            });
        }
        let length = axis.into_iter().map(|v| v * v).sum::<f64>().sqrt();
        if !body::inside(anchor) || !positive_normal(length) {
            return Err(PhysicsError::InvalidJoint);
        }
        let unit = axis.map(|v| v / length);
        if !body::hinge_conditioned(&child_body.config, anchor, unit) {
            return Err(PhysicsError::InvalidJoint);
        }
        let next = self
            .next_joint
            .checked_add(1)
            .ok_or(PhysicsError::JointIdOverflow)?;
        let id = JointId::from_raw(self.next_joint);
        self.world.attach(
            id,
            Some(parent),
            child,
            Twist {
                linear: anchor,
                angular: unit,
            },
        )?;
        self.next_joint = next;
        Ok(id)
    }

    fn validate_joint_effort(&self, id: JointId, torque: f64) -> Result<(), PhysicsError> {
        if !self.world.joints.contains_key(&id) {
            return Err(PhysicsError::MissingJoint);
        }
        if !torque.is_finite() || torque.abs() > body::MAGNITUDE_LIMIT {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }

    fn add_joint_effort(&mut self, id: JointId, torque: f64) -> Result<(), PhysicsError> {
        self.validate_joint_effort(id, torque)?;
        let next = self.joint_efforts.get(&id).copied().unwrap_or(0.0) + torque;
        if !next.is_finite() || next.abs() > body::MAGNITUDE_LIMIT {
            return Err(PhysicsError::InvalidValue);
        }
        self.joint_efforts.insert(id, next);
        Ok(())
    }

    fn joint_child(&self, id: JointId) -> Option<BodyId> {
        self.world.joints.get(&id).map(|joint| joint.child)
    }

    fn joint_state(&self, id: JointId) -> Option<JointState> {
        let joint = self.world.joints.get(&id)?;
        self.world.bodies.get(&joint.child).map(|body| body.joint)
    }

    /// The body keeps its current pose and velocity as a free body. Returns
    /// false, keeping the hinge, if a current contact of the freed body would
    /// exceed the contact inverse-mass bound. A world root with a live body child
    /// remains attached until that child joint or body is removed.
    fn remove_joint(&mut self, id: JointId) -> bool {
        if self.world.joints.get(&id).is_some_and(|joint| {
            joint.parent.is_none()
                && self
                    .world
                    .joints
                    .values()
                    .any(|leaf| leaf.parent == Some(joint.child))
        }) {
            return false;
        }
        let Built {
            native,
            bodies,
            joints,
        } = &mut self.world;
        let Some(body) = joints
            .get(&id)
            .and_then(|joint| bodies.get_mut(&joint.child))
        else {
            return false;
        };
        if !matches!(native.remove_revolute(body.slot), Ok(true)) {
            return false;
        }
        joints.remove(&id);
        self.joint_efforts.remove(&id);
        self.world.refresh_all().is_ok()
    }

    fn is_jointed(&self, id: BodyId) -> Option<bool> {
        self.world.bodies.get(&id)?;
        Some(
            self.world
                .joints
                .values()
                .any(|joint| joint.child == id || joint.parent == Some(id)),
        )
    }

    /// Wrenches apply all-or-none inside the native step. A step whose result
    /// leaves the admitted bounds restores the previous positions, velocities
    /// and time, keeps pending wrenches, and returns a native error naming why.
    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        if dt.is_zero() {
            return Ok(());
        }
        // Prepare all commands without mutating either pending queue or native state.
        let combined = if self.joint_efforts.is_empty() {
            None
        } else {
            let mut pending = self.pending.clone();
            for (&id, &effort) in &self.joint_efforts {
                let joint = &self.world.joints[&id];
                if joint.parent.is_some() {
                    continue;
                }
                let wrench = pending.entry(joint.child).or_insert(Twist {
                    linear: [0.0; 3],
                    angular: [0.0; 3],
                });
                let next = add(
                    wrench.angular,
                    joint.anchor_axis.angular.map(|v| v * effort),
                );
                if !body::small(next) {
                    return Err(PhysicsError::InvalidValue);
                }
                wrench.angular = next;
            }
            Some(pending)
        };
        let pending = combined.as_ref().unwrap_or(&self.pending);
        // Removal drops a body's pending wrench, so every entry has a body.
        let wrenches: Vec<Wrench> = self
            .world
            .bodies
            .iter()
            .filter_map(|(id, body)| {
                pending.get(id).map(|wrench| Wrench {
                    slot: body.slot,
                    force: wrench.linear,
                    torque: wrench.angular,
                })
            })
            .collect();
        let efforts: Vec<JointEffort> = self
            .joint_efforts
            .iter()
            .filter_map(|(id, torque)| {
                let joint = &self.world.joints[id];
                joint.parent.map(|_| JointEffort {
                    slot: self.world.bodies[&joint.child].slot,
                    torque: *torque,
                })
            })
            .collect();
        let outcome = if efforts.is_empty() {
            self.world.native.step(dt.as_secs_f64(), &wrenches)?
        } else {
            self.world
                .native
                .step_with_efforts(dt.as_secs_f64(), &wrenches, &efforts)?
        };
        let refusal = match outcome {
            0 => None,
            1 => Some("a velocity became nonfinite or exceeded 1e60"),
            2 => Some("a collision bound left the 5e8 m grid"),
            4 => Some(
                "an active joint limit exceeded the inverse-inertia bound or involved an unsupported articulation",
            ),
            _ => Some(
                "a contact exceeded the inverse-mass bound or involved an unsupported articulation",
            ),
        };
        if let Some(reason) = refusal {
            return Err(PhysicsError::Native {
                engine: ENGINE,
                message: format!("step refused: {reason}; previous state restored"),
            });
        }
        self.pending.clear();
        self.joint_efforts.clear();
        self.joints_open = false;
        self.world.refresh_all()
    }

    /// Rebuilds remaining bodies and joints; removed bodies stay removed.
    fn reset(&mut self) -> Result<(), PhysicsError> {
        let records = self.world.records();
        let built = Built::new(self.gravity, records.bodies, records.joints)?;
        self.replace(built);
        Ok(())
    }

    fn initial_state(&self) -> InitialPhysicsState {
        let records = Initial {
            next_body: self.next_body,
            next_joint: self.next_joint,
            ..self.world.records()
        };
        InitialPhysicsState::new(&self.owner, records)
    }

    fn owns_initial_state(&self, initial: &InitialPhysicsState) -> bool {
        initial.is_owned_by(&self.owner)
    }

    fn prepare_initial(
        &self,
        initial: &InitialPhysicsState,
    ) -> Result<PreparedPhysicsReset, PhysicsError> {
        let records = initial.data::<Initial>(&self.owner)?;
        let built = Built::new(
            self.gravity,
            records
                .bodies
                .iter()
                .map(|(&id, config)| (id, config.clone())),
            records.joints.iter().map(|(&id, &joint)| (id, joint)),
        )?;
        Ok(PreparedPhysicsReset::new(
            &self.owner,
            Prepared {
                built,
                next_body: records.next_body,
                next_joint: records.next_joint,
            },
        ))
    }

    /// Later body and joint IDs stay above every ID allocated before commit.
    fn apply_initial(&mut self, prepared: PreparedPhysicsReset) -> Result<(), PhysicsError> {
        let prepared = prepared.into_data::<Prepared>(&self.owner)?;
        self.next_body = self.next_body.max(prepared.next_body);
        self.next_joint = self.next_joint.max(prepared.next_joint);
        self.replace(prepared.built);
        Ok(())
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

fn positive_normal(value: f64) -> bool {
    value.is_normal() && value > 0.0
}

#[cfg(test)]
#[path = "../tests/private/lib.rs"]
mod tests;
