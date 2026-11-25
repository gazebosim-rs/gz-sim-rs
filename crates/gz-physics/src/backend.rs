// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The contract implemented by the Rapier dynamic engine adapter.

use crate::{BodySpec, BodyState, Inertia, MeshBodySpec, PlaneSpec, RotationSpec};
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

/// Displacement from the construction pose and rate along a scalar joint axis.
/// A revolute uses radians and rad/s. A prismatic uses metres and m/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointState {
    pub position: f64,
    pub velocity: f64,
}

/// Persistent scalar velocity target, positive along the joint axis.
/// Hinges use rad/s and N·m. Sliders use m/s and N.
/// This limits actuator effort; the target is not a hard physical speed cap.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointVelocityMotor {
    pub target_velocity: f64,
    pub max_force: f64,
}

/// Continuous position request for the next matching-duration native solver step.
/// Reissue each advancing step; the backend prepares a bounded local waypoint.
/// `max_velocity` bounds requested solver bias (rad/s), not physical speed;
/// `max_force` is a finite nonnegative actuator torque cap in Nm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointPositionServo {
    pub target_position: f64,
    pub max_velocity: f64,
    pub max_force: f64,
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
    /// The assembly and target are valid, but a physical or pending-command
    /// guard prevents a fidelity change at this boundary. Retrying is safe.
    #[error("fidelity transition deferred: {reason}")]
    TransitionDeferred { reason: &'static str },
    /// The solver disabled non-finite objects after integration. The attempted
    /// step may have changed solver internals; a scheduled simulation is terminal.
    #[error("{engine} disabled non-finite physics objects during integration")]
    NonFiniteStep { engine: &'static str },
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

/// A validated reset state bound to its backend. In-process engines stage the
/// replacement solver here. A durable backend may still report process or
/// publication errors while committing under its exclusive ownership lease.
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

/// Scoped access to body states in a single committed backend observation.
pub trait BodyStateLookup {
    fn state(&mut self, id: BodyId) -> Option<BodyState>;
}

impl<F: FnMut(BodyId) -> Option<BodyState>> BodyStateLookup for F {
    fn state(&mut self, id: BodyId) -> Option<BodyState> {
        self(id)
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

    /// Checks a prescribed world twist for a given step duration, without
    /// changing the body, solver, or pending inputs. Zero dt checks the twist
    /// alone. Implemented by engines that support velocity-prescribed boundaries.
    fn validate_kinematic_velocity(
        &self,
        _linear: [f64; 3],
        _angular: [f64; 3],
        _dt: Duration,
    ) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "kinematic link",
        })
    }

    /// Creates a velocity-prescribed primitive boundary. The world-space linear
    /// velocity is at the center of mass. Its mass, if supplied, is nominal:
    /// dynamic contacts cannot accelerate it. Refusal allocates no identifier.
    fn spawn_kinematic_link(
        &mut self,
        _spec: BodySpec,
        _rotation: RotationSpec,
        _inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "kinematic link",
        })
    }

    /// Admit a static triangle surface or a closed convex dynamic mesh.
    /// Unsupported engines refuse without allocating bodies or decoding assets.
    fn spawn_mesh_link(
        &mut self,
        _spec: MeshBodySpec<'_>,
        _rotation: RotationSpec,
        _inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "mesh link collision",
        })
    }

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

    /// Whether a registered body's motion is prescribed by a velocity command.
    /// Fixed and force-driven dynamic bodies return false. Missing IDs return None.
    fn is_kinematic(&self, id: BodyId) -> Option<bool> {
        self.is_dynamic(id).map(|_| false)
    }

    fn state(&self, id: BodyId) -> Option<BodyState>;

    /// Invoke `visit` once with a reader for many public body states. Arbitrary
    /// order, repeated IDs and removed IDs have the same meaning as `state`.
    /// Engines may accelerate dense reads without publishing private handles.
    fn read_states(&self, visit: &mut dyn FnMut(&mut dyn BodyStateLookup)) {
        visit(&mut |id| self.state(id));
    }

    /// Closest solid collision along a world-space ray, in metres, or no hit.
    ///
    /// The origin and endpoint must be finite; `direction` squared length must
    /// differ from one by at most 1e-12. `max_distance` is finite and positive. Hits include
    /// both endpoints; an origin inside a solid returns zero. `exclude` omits
    /// only that body's colliders and must name an existing body. Sensor minimum
    /// range and no-return conventions belong to the caller.
    ///
    /// This query does not step physics, update contacts, consume commands or
    /// allocate body IDs. Unsupported engines refuse before validating inputs.
    fn cast_ray(
        &self,
        _origin: [f64; 3],
        _direction: [f64; 3],
        _max_distance: f64,
        _exclude: Option<BodyId>,
    ) -> Result<Option<f64>, PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "ray queries",
        })
    }

    /// IDs whose collision shapes' world-axis-aligned bounds intersect the closed box,
    /// sorted and unique. This is a discrete geometry observation with no contact impulses.
    /// A zero-width interval is valid. Standalone scenery has no body ID.
    fn bodies_overlapping_aabb(
        &self,
        _min: [f64; 3],
        _max: [f64; 3],
    ) -> Result<Vec<BodyId>, PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "body AABB overlap",
        })
    }

    fn remove(&mut self, id: BodyId) -> bool;

    /// Preflights an atomic world-space pose and velocity replacement.
    ///
    /// `BodyState::position` is the body origin and `BodyState::velocity` is
    /// center-of-mass velocity. Rapier admits unjointed bodies only, requires
    /// finite values and a unit quaternion within its construction tolerance,
    /// and refuses nonzero velocities for fixed bodies. Unsupported backends
    /// refuse before ID or numeric validation. This does not change reset data,
    /// pending commands, IDs or any native state.
    fn validate_body_state(&self, _id: BodyId, _state: BodyState) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "body-state replacement",
        })
    }

    /// Replaces an admitted body's pose and both velocities without stepping.
    ///
    /// Validation errors leave all state unchanged. Success preserves spawn/reset
    /// records and pending world forces/torques. This explicit pose edit bypasses
    /// collision resolution; the next normal step resolves resulting contacts.
    fn set_body_state(&mut self, _id: BodyId, _state: BodyState) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "body-state replacement",
        })
    }

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
    ) -> Result<JointId, PhysicsError>;

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
    ) -> Result<JointId, PhysicsError>;

    /// Excludes mutual contacts between bodies with the same assigned group.
    /// `None` restores ordinary contact filtering. Group IDs are backend-local.
    /// This is a construction property; runtime edits may be unsupported.
    fn set_self_collision_group(
        &mut self,
        _body: BodyId,
        _group: Option<u64>,
    ) -> Result<(), PhysicsError>;

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
    fn add_joint_effort(&mut self, _id: JointId, _torque: f64) -> Result<(), PhysicsError>;

    /// Validates a persistent revolute velocity motor without changing any state.
    /// `None` disables actuation; a zero target with positive force applies bounded braking.
    /// Unsupported engines refuse before identifier or numeric validation.
    fn validate_joint_velocity_motor(
        &self,
        _id: JointId,
        _motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "joint velocity motors",
        })
    }

    /// Replaces the persistent motor configuration without advancing physics or changing
    /// body/joint state. Invalid input preserves the old motor and pending commands.
    /// Targets act through the maintained solver on subsequent nonzero steps. Both resets
    /// clear all motors, including motors present when an initial state was captured;
    /// joint/body removal retires them. Torque/target times dt must remain representable.
    fn set_joint_velocity_motor(
        &mut self,
        _id: JointId,
        _motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "joint velocity motors",
        })
    }

    /// Validates an opt-in torque-bounded native velocity constraint, without mutation.
    /// This is distinct from the accepted compliant velocity motor policy. Rapier uses
    /// its maintained zero-stiffness/zero-damping motor coefficients (zero compliance).
    /// `None`, units, numeric admission, reset and removal follow `JointVelocityMotor`.
    /// Unsupported engines refuse before identifier or numeric validation.
    fn validate_joint_velocity_servo(
        &self,
        _id: JointId,
        _motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "joint velocity servos",
        })
    }

    /// Replaces the native velocity constraint, preserving state and pending work on
    /// refusal. Zero target brakes within the torque cap; `None` disables and clears
    /// warm-start state. Finite torque limits still permit tracking error or overspeed
    /// under loads; this does not impose a hard physical velocity or position limit.
    fn set_joint_velocity_servo(
        &mut self,
        _id: JointId,
        _motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "joint velocity servos",
        })
    }

    /// Prepares all fallible values for a native continuous-coordinate position servo
    /// without mutation. Rapier supports reduced revolute joints only. `Some` requires
    /// positive `dt` matching the upcoming step, finite target, positive finite target
    /// speed and nonnegative finite torque. `None` ignores dt and disables a registered
    /// joint of either kind. Other engines refuse before validation.
    fn validate_joint_position_servo(
        &self,
        _id: JointId,
        _servo: Option<JointPositionServo>,
        _dt: Duration,
    ) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "joint position servos",
        })
    }

    /// Atomically installs the waypoint prepared from current native continuous state.
    /// The caller retains the ultimate target and reissues before each advancing step.
    /// Rapier uses its maintained acceleration-based spring with k100/d20 and zero
    /// target velocity; fixed nominal critical damping is not a convergence guarantee
    /// for an arbitrary articulation. Torque cap, None, reset and removal follow the
    /// velocity motor contract. Refusal preserves mode, state, IDs and pending work.
    fn set_joint_position_servo(
        &mut self,
        _id: JointId,
        _servo: Option<JointPositionServo>,
        _dt: Duration,
    ) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "joint position servos",
        })
    }

    /// Registered child endpoint of a joint.
    fn joint_child(&self, id: JointId) -> Option<BodyId>;

    fn joint_state(&self, id: JointId) -> Option<JointState>;

    fn remove_joint(&mut self, id: JointId) -> bool;

    /// Whether a registered body participates in a solver constraint.
    fn is_jointed(&self, id: BodyId) -> Option<bool>;

    /// Advances by `dt`. A zero step changes nothing. A solver that detects
    /// non-finite state after integration can report [`PhysicsError::NonFiniteStep`];
    /// its internal state may have advanced and must not be published.
    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError>;

    /// Restores remaining bodies and joints to their construction state.
    fn reset(&mut self) -> Result<(), PhysicsError>;

    fn initial_state(&self) -> InitialPhysicsState;

    fn owns_initial_state(&self, initial: &InitialPhysicsState) -> bool;

    /// Prepares a reset before any scene or backend mutation. Local engines
    /// build their replacement solver during this preflight.
    fn prepare_initial(
        &self,
        initial: &InitialPhysicsState,
    ) -> Result<PreparedPhysicsReset, PhysicsError>;

    /// Restores initial IDs and retires runtime bodies; later IDs remain monotonic.
    /// Callers must propagate a durable commit failure before rebuilding the ECS.
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
    install_with_step(
        sim,
        backend,
        |time: Res<UpdateInfo>, fault: Res<StepFault>, mut backend: ResMut<PhysicsBackend>| {
            if let Err(error) = backend.step(time.dt) {
                fault.report(error);
            }
        },
    )
}

/// Install a coupled engine which owns all native subintervals of a physical
/// tick, with the standard single-owner and epoch-aware native reset behavior.
pub fn install_with_step<M>(
    sim: &mut Simulation,
    backend: impl Into<PhysicsBackend>,
    step: impl IntoScheduleConfigs<bevy_ecs::system::ScheduleSystem, M>,
) -> Result<(), PhysicsError> {
    if sim.world().contains_resource::<PhysicsBackend>()
        || !sim.register_extension::<crate::PhysicsInstallation>()
    {
        return Err(PhysicsError::AlreadyInstalled);
    }
    sim.world_mut().insert_resource(backend.into());
    sim.add_systems(Phase::PhysicsStep, step);
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
