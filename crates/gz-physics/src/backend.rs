// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The contract every dynamic engine adapter (Rapier, DART, MuJoCo) implements.

use crate::{BodySpec, BodyState, Inertia, PlaneSpec, RotationSpec};
use bevy_ecs::prelude::*;
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo};
use std::{any::Any, sync::Arc, time::Duration};

/// A backend-local stable body identifier. Always pair it with its owning backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyId(u64);

/// A backend-local joint identifier. Always pair it with its owning backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct JointId(u64);

macro_rules! ids {
    ($($name:ident),*) => {$(
        impl $name {
            /// Backends allocate identifiers monotonically from zero.
            pub fn from_raw(value: u64) -> Self {
                Self(value)
            }

            pub fn raw(self) -> u64 {
                self.0
            }
        }
    )*};
}
ids!(BodyId, JointId);

/// Angle from construction pose, in radians, and world-axis angular velocity in rad/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointState {
    pub position: f64,
    pub velocity: f64,
}

#[derive(Debug, Clone, thiserror::Error, PartialEq, Eq)]
pub enum PhysicsError {
    #[error("body or gravity contains an invalid numeric value")]
    InvalidValue,
    #[error("body identifier space exhausted")]
    IdOverflow,
    #[error("a physics backend is already installed in this world")]
    AlreadyInstalled,
    #[error("body identifier is not present in this backend")]
    MissingBody,
    #[error("joint identifier is not present in this backend")]
    MissingJoint,
    #[error("initial physics state belongs to a different backend")]
    ForeignInitialState,
    #[error("joint requires a dynamic child and representable finite anchor/axis")]
    InvalidJoint,
    #[error("joint construction is closed until physics reset")]
    JointConstructionClosed,
    #[error("joint identifier space exhausted")]
    JointIdOverflow,
    /// The selected engine does not implement this capability; nothing changed.
    #[error("{engine} does not support {capability}")]
    Unsupported {
        engine: &'static str,
        capability: &'static str,
    },
    /// A native engine reported a failure; the message is the engine's diagnostic.
    #[error("{engine} failed: {message}")]
    Native {
        engine: &'static str,
        message: String,
    },
}

/// Remaining bodies' construction records, owned by the backend that captured them.
/// Capture before stepping to define an initial scene; this is not a runtime snapshot.
#[derive(Clone)]
pub struct InitialPhysicsState {
    owner: Arc<()>,
    data: Arc<dyn Any + Send + Sync>,
}

/// A fully constructed reset state, ready to commit without further fallible work.
pub struct PreparedPhysicsReset {
    owner: Arc<()>,
    data: Box<dyn Any + Send + Sync>,
}

impl InitialPhysicsState {
    pub fn new(owner: &Arc<()>, data: impl Any + Send + Sync) -> Self {
        Self {
            owner: owner.clone(),
            data: Arc::new(data),
        }
    }

    /// The captured data when `owner` captured it as type `T`.
    pub fn data<T: Any>(&self, owner: &Arc<()>) -> Result<&T, PhysicsError> {
        if !Arc::ptr_eq(&self.owner, owner) {
            return Err(PhysicsError::ForeignInitialState);
        }
        self.data
            .downcast_ref()
            .ok_or(PhysicsError::ForeignInitialState)
    }

    pub fn is_owned_by(&self, owner: &Arc<()>) -> bool {
        Arc::ptr_eq(&self.owner, owner)
    }
}

impl PreparedPhysicsReset {
    pub fn new(owner: &Arc<()>, data: impl Any + Send + Sync) -> Self {
        Self {
            owner: owner.clone(),
            data: Box::new(data),
        }
    }

    /// The prepared data when `owner` prepared it as type `T`.
    pub fn into_data<T: Any>(self, owner: &Arc<()>) -> Result<T, PhysicsError> {
        if !Arc::ptr_eq(&self.owner, owner) {
            return Err(PhysicsError::ForeignInitialState);
        }
        self.data
            .downcast()
            .map(|data| *data)
            .map_err(|_| PhysicsError::ForeignInitialState)
    }
}

/// A dynamic engine stepped by the simulator's Bevy schedule.
///
/// Positions and velocities use world axes, SI units and Gazebo's Z-up convention;
/// quaternions are [x,y,z,w]. Failed calls leave the backend unchanged unless a
/// method documents otherwise. Unimplemented features return
/// [`PhysicsError::Unsupported`] rather than approximating them.
pub trait DynamicBackend: Send + Sync + 'static {
    /// Short engine name used in diagnostics, such as "rapier".
    fn engine(&self) -> &'static str;

    /// Creates one centered collider with optional explicit link inertia.
    fn spawn_link(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError>;

    /// Construct a fixed local Z-up plane without shifting its reported link frame.
    /// Unsupported engines return an explicit error without allocating a body.
    fn spawn_static_plane(&mut self, _spec: PlaneSpec) -> Result<BodyId, PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "static plane collider",
        })
    }

    /// Whether a registered body has dynamic mass rather than a fixed solver body.
    fn is_dynamic(&self, id: BodyId) -> Option<bool>;

    fn state(&self, id: BodyId) -> Option<BodyState>;

    fn remove(&mut self, id: BodyId) -> bool;

    /// Replaces world-frame velocities of a dynamic body. Fixed bodies stay fixed.
    fn set_world_velocity(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError>;

    /// Accumulates world force at the center of mass and world torque for the next
    /// advancing step only.
    fn add_world_wrench(
        &mut self,
        id: BodyId,
        force: [f64; 3],
        torque: [f64; 3],
    ) -> Result<(), PhysicsError>;

    /// Applies linear and angular impulses in world axes.
    fn apply_impulse(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError>;

    /// Constructs an unbounded revolute joint from a dynamic body to the world.
    /// Anchor/axis use world coordinates; the construction pose is angle zero.
    /// Construction closes at the first advancing step and reopens after reset.
    fn add_world_revolute(
        &mut self,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
    ) -> Result<JointId, PhysicsError>;

    /// Constructs an unbounded revolute between registered parent and child bodies.
    /// The child must be dynamic; the distinct parent may be fixed or dynamic.
    /// Anchor/axis use world coordinates; the construction pose is angle zero.
    /// Construction closes at the first advancing step and reopens after reset.
    /// Engines without this capability reject it before argument validation.
    fn add_revolute(
        &mut self,
        _parent: BodyId,
        _child: BodyId,
        _anchor: [f64; 3],
        _axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "body-to-body revolute joint",
        })
    }

    /// Constructs an unbounded reduced-coordinate revolute in an acyclic articulation.
    /// `None` parent anchors to the world; the construction pose is angle zero.
    /// Current and reset construction velocities must satisfy the joint constraints.
    /// Engines may
    /// reject offset centers of mass, mixed impulse constraints and direct
    /// articulated velocity edits.
    /// Errors leave the backend unchanged.
    fn add_reduced_revolute(
        &mut self,
        _parent: Option<BodyId>,
        _child: BodyId,
        _anchor: [f64; 3],
        _axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "reduced-coordinate revolute joint",
        })
    }

    /// Excludes mutual contacts between bodies with the same assigned group.
    /// `None` restores ordinary contact filtering. Group IDs are backend-local.
    /// This is a construction property; runtime edits may be unsupported.
    fn set_self_collision_group(
        &mut self,
        _body: BodyId,
        _group: Option<u64>,
    ) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "self-collision groups",
        })
    }

    /// Checks a single generalized torque without changing pending commands.
    /// This excludes accumulation and later combined world-wrench validation.
    fn validate_joint_effort(&self, _id: JointId, _torque: f64) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "joint effort commands",
        })
    }

    /// Accumulates generalized torque in Nm, positive along increasing joint angle.
    /// The next successful nonzero step consumes it. Joint retirement or either
    /// reset discards it; rejected inputs and failed steps preserve pending commands.
    fn add_joint_effort(&mut self, _id: JointId, _torque: f64) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "joint effort commands",
        })
    }

    /// Registered child endpoint of a joint.
    fn joint_child(&self, id: JointId) -> Option<BodyId>;

    fn joint_state(&self, id: JointId) -> Option<JointState>;

    fn remove_joint(&mut self, id: JointId) -> bool;

    /// Whether a registered body participates in a solver constraint.
    fn is_jointed(&self, id: BodyId) -> Option<bool>;

    /// Advances by `dt`. A zero step changes nothing.
    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError>;

    /// Restores remaining bodies and joints to their construction state.
    fn reset(&mut self) -> Result<(), PhysicsError>;

    fn initial_state(&self) -> InitialPhysicsState;

    fn owns_initial_state(&self, initial: &InitialPhysicsState) -> bool;

    /// Builds an independent engine state before any scene or backend mutation.
    fn prepare_initial(
        &self,
        initial: &InitialPhysicsState,
    ) -> Result<PreparedPhysicsReset, PhysicsError>;

    /// Restores initial IDs and retires runtime bodies; later IDs remain monotonic.
    fn apply_initial(&mut self, prepared: PreparedPhysicsReset) -> Result<(), PhysicsError>;

    fn as_any(&self) -> &dyn Any;

    fn as_any_mut(&mut self) -> &mut dyn Any;
}

/// The single dynamic engine installed in a simulation.
#[derive(Resource)]
pub struct PhysicsBackend(Box<dyn DynamicBackend>);

impl<T: DynamicBackend> From<T> for PhysicsBackend {
    fn from(backend: T) -> Self {
        Self(Box::new(backend))
    }
}

impl PhysicsBackend {
    /// The concrete adapter, for engine-specific operations and tests.
    pub fn downcast_ref<T: DynamicBackend>(&self) -> Option<&T> {
        self.0.as_any().downcast_ref()
    }

    pub fn downcast_mut<T: DynamicBackend>(&mut self) -> Option<&mut T> {
        self.0.as_any_mut().downcast_mut()
    }
}

impl std::ops::Deref for PhysicsBackend {
    type Target = dyn DynamicBackend;

    fn deref(&self) -> &Self::Target {
        self.0.as_ref()
    }
}

impl std::ops::DerefMut for PhysicsBackend {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.as_mut()
    }
}

/// Orders reset handlers that depend on restored physics.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PhysicsReset;

/// Marks physics already restored by a validated transaction for one reset epoch.
/// The transaction removes this after Reset handlers finish. Other epochs rebuild.
#[derive(Resource)]
pub struct RestoredPhysics {
    pub epoch: u64,
}

/// Installs one backend into the simulation's physics and reset phases.
/// Step and reset failures are reported through [`StepFault`].
pub fn install(
    sim: &mut Simulation,
    backend: impl Into<PhysicsBackend>,
) -> Result<(), PhysicsError> {
    if sim.world().contains_resource::<PhysicsBackend>()
        || !sim.register_extension::<crate::PhysicsInstallation>()
    {
        return Err(PhysicsError::AlreadyInstalled);
    }
    sim.world_mut().insert_resource(backend.into());
    sim.add_systems(
        Phase::PhysicsStep,
        |time: Res<UpdateInfo>, fault: Res<StepFault>, mut backend: ResMut<PhysicsBackend>| {
            if let Err(error) = backend.step(time.dt) {
                fault.report(error);
            }
        },
    );
    sim.add_systems(
        Phase::Reset,
        (|time: Res<UpdateInfo>,
          restored: Option<Res<RestoredPhysics>>,
          fault: Res<StepFault>,
          mut backend: ResMut<PhysicsBackend>| {
            if restored.is_none_or(|state| state.epoch != time.epoch)
                && let Err(error) = backend.reset()
            {
                fault.report(error);
            }
        })
        .in_set(PhysicsReset),
    );
    Ok(())
}

#[cfg(test)]
#[path = "../tests/private/backend.rs"]
mod tests;
