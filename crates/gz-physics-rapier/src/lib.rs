// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Full-f64 rigid bodies stepped by the simulator's explicit Bevy schedule.

use bevy_ecs::prelude::*;
use rapier3d_f64::prelude::*;
use std::sync::Arc;
use std::{
    collections::{BTreeMap, BTreeSet},
    time::Duration,
};

mod analytic_coast;
mod free_rotation;
mod trajectory;
#[cfg(feature = "partition-checkpoint")]
pub use trajectory::TRAJECTORY_IMAGE_LIMIT;
mod articulation;
mod articulation_support;
mod support_loss;
pub use support_loss::{SupportLoss, SupportLossReason};
mod box_features;
mod contact_dispatcher;
#[cfg(feature = "landing-audit")]
pub use box_features::ReturnFeatureMode;
pub use box_features::{ReturnFeatureCounts, ReturnPlaneEvidence, ReturningBoxFeatureReceipt};
mod approach_clearance;
mod box_timing;
pub use articulation_support::{HeldSupportStatus, SupportExpiry};
pub use box_timing::{
    BoxCrossing, BoxFlightReceipt, BoxTimingDecline, BoxTimingReceipt, CompactDecline,
    CompactImpactReceipt, OpponentClearance, OpposingCornerReach, OpposingCornerReceipt,
    OpposingLimitStage, OpposingNormalDecline, OpposingNormalEvent, OpposingNormalReceipt,
    OpposingNormalStage, OpposingReturnReceipt, PulseDecline, PulseReceipt, ReleasedBoxStep,
    RigidBoxSweep, RigidPointDecline, RigidPointImpactReceipt, RotationContactPulse,
};
mod body_state;
mod contact_interval;
mod sole_receiver;
pub use contact_interval::{
    BoxContactActions, BoxContactOutcome, CompactPointTail, ContactIntervalPair,
    ContactIntervalReceipt, ContactSegment, ContactSolverIterations, ContactSubstep,
};
pub use joints::{CoatedAnkle, CoatedAnkleStep, ShankContact};
pub use sole_receiver::SoleFoot;
#[cfg(feature = "partition-checkpoint")]
mod box_join;
#[cfg(feature = "partition-checkpoint")]
pub use box_join::NativeBoxMaps;
mod collision;
#[cfg(feature = "landing-audit")]
mod landing_row_order;
#[cfg(feature = "landing-audit")]
pub use landing_row_order::{LandingRowCounts, LandingRowOrder};
mod compound;
mod material;
pub use compound::RigidSurfaceWitness;
pub use material::{IsotropicFriction, MaterialLinkMotion, Restitution};
mod contact_prediction;
mod fleet;
mod friction;
pub use friction::NativeFrictionModel;
mod fleet_coordinates;
mod fleet_isolation;
use fleet::FleetClock;
pub use fleet_isolation::FleetIsolationStats;
#[cfg(feature = "partition-checkpoint")]
mod checkpoint;
mod inertia_frame;
mod initial;
mod joints;
pub use joints::ExchangeInputs;
pub use joints::{
    ArticulatedContactSample, ArticulatedContactSite, ArticulatedContactTransaction,
    ArticulatedImpulseReport, ArticulatedNativeInterval, ArticulatedSweep, BoundPointInterval,
    BoundTransferReceipt, CornerImpactReceipt, DeckEdgeBearingReceipt, EdgeImpactReceipt,
    FaceBearingReceipt, FacePressureReceipt, FiniteEndpointCertificate, FiniteImpactReceipt,
    FractionalBearingSpan, FractionalPointEvent, FractionalPointReceipt,
    FrictionlessRecontactReceipt, InternalPointImpact, MAX_ARTICULATED_CONTACT_SITES,
    MixedEdgeImpactReceipt, OpeningEdgeImpactReceipt, PlanarImpactReceipt, PointSpanLoad,
    RockingLimitReceipt, SharedFaceReceipt, SharedSupportLoad, SharedSupportRequest,
    SlidingEdgeImpactReceipt, SlidingPointReceipt, SlidingVertexReceipt, SoleTransferReceipt,
    StickingPointReceipt, SustainedEdgeReceipt, TwoBearingReceipt,
};
pub use joints::{
    BallJointState, JointGravityLoad, RobotJoint, RobotSpring, UniversalJointState,
    WeldedAssemblyState,
};
pub(crate) use joints::{BearingModes, BearingResult};
#[cfg(feature = "partition-checkpoint")]
pub use joints::{
    CLEAR_GRASP_MEMBER_IMAGE_LIMIT, FLOATING_TREE_IMAGE_LIMIT, GRASP_FLEET_IMAGE_LIMIT,
    GRASP_TREE_IMAGE_LIMIT, GROUNDED_REVOLUTE_LIMIT, GROUNDED_ROBOT_IMAGE_LIMIT,
    GROUNDED_TREE_IMAGE_LIMIT, GroundReaction, MAX_FLOATING_TREE_LINKS, MAX_GROUNDED_ROBOT_LINKS,
    MAX_GROUNDED_TREE_LINKS, MAX_PAYLOAD_TREE_LINKS, MOTOR_TREE_IMAGE_LIMIT, MovingReaction,
    PAYLOAD_TREE_IMAGE_LIMIT, STOPPED_TREE_IMAGE_LIMIT, WARM_GRASP_MEMBER_IMAGE_LIMIT,
};
mod mesh;
#[cfg(feature = "partition-checkpoint")]
pub use checkpoint::{
    MAX_CHECKPOINT_BODIES, NativeSolverProfile, PARTITION_CHECKPOINT_LIMIT,
    partition_linear_speed_limit, partition_velocity_change,
};
#[cfg(feature = "partition-checkpoint")]
mod partition_ground;
#[cfg(feature = "partition-checkpoint")]
pub use partition_ground::{
    GroundPatchPoint, NativeGroundContact, NativeGroundPatch, NativeGroundPlane,
};
mod partition;
mod ray;
pub use ray::SolidRay;
mod soft_receiver;
pub use soft_receiver::{SoftRigidInertia, SoftSphereInertia};

pub use articulation::ArticulationTier;
pub use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, InitialPhysicsState, JointId,
    JointPositionServo, JointState, JointVelocityMotor, MeshBodySpec, PhysicsBackend, PhysicsError,
    PhysicsReset, PreparedPhysicsReset, RestoredPhysics, RotationSpec, Shape, install,
};
#[cfg(feature = "landing-audit")]
pub use joints::{
    LandingCachedPoint, LandingFork, LandingRestartState, LandingSolverPoint, LandingTrial,
    RestartPairHistory,
};
pub use partition::{
    ContactWarmstart, NormalReaction, PartitionBounds, PartitionBoxBounds, RigidContactObservation,
    RigidContactPatch, RigidPatchPoint, StoredContactPoint, partition_prediction_distance,
    predicts_box_contact,
};
use std::any::Any;

/// Immutable admitted geometry and spawn kinematics. Collider cloning shares the
/// maintained shape and preserves its local frame without asset IO during reset.
#[derive(Clone)]
struct Construction {
    position: [f64; 3],
    velocity: [f64; 3],
    mass: Option<f64>,
    kinematic: bool,
    can_sleep: bool,
    continuous_collision: bool,
    // Switchable links use a native body frame at COM; the public link frame
    // is restored using the retained inertial offset in BodyEntry.
    articulation: Option<BodyId>,
    collider: Collider,
}

type BodyEntry = (
    RigidBodyHandle,
    Construction,
    RotationSpec,
    Option<Inertia>,
    Option<u64>,
);

pub struct RapierBackend {
    // Explicitly negotiated worker graphs retain their complete identity.
    // Historical unbound native constructors select a capacity at capture.
    #[cfg(feature = "partition-checkpoint")]
    native_profile: Option<NativeSolverProfile>,
    #[cfg(feature = "partition-checkpoint")]
    join_queries_pending: bool,
    #[cfg(feature = "partition-checkpoint")]
    pair_observer: Option<PairObserver>,
    #[cfg(feature = "partition-checkpoint")]
    grasp_payload_observer: Option<GraspPayloadObserver>,
    curves: trajectory::Curves,
    fleet_clock: FleetClock,
    allow_isolation: bool,
    fleet_dispatch: fleet_isolation::Dispatch,
    fleet_resume_positions: std::collections::HashMap<RigidBodyHandle, Vector>,
    translation_epochs: std::collections::HashMap<RigidBodyHandle, Duration>,
    translation_rows: std::sync::OnceLock<Vec<fleet::TranslationRow>>,
    translation_rays: std::sync::OnceLock<Option<ray::TranslationIndex>>,
    owner: Arc<()>,
    exchange_epoch: Arc<()>,
    world: PhysicsWorld,
    bodies: BTreeMap<BodyId, BodyEntry>,
    joints: joints::Joints,
    welded_projection: Option<Box<joints::WeldedProjection>>,
    welded_exclusion: Option<(usize, usize, usize)>,
    next_id: u64,
    pending_wrenches: BTreeSet<BodyId>,
    exchange_reactions:
        Option<BTreeMap<(BodyId, BodyId), (RigidContactObservation, NormalReaction)>>,
    // An upper bound after velocity edits or removal. Recompute the live maximum
    // only when this bound could exceed the allowed angular travel for a step.
    max_boundary_spin: f64,
    held_wrenches: BTreeMap<BodyId, (Vector, Vector)>,
    #[cfg(feature = "landing-audit")]
    landing_rows: Option<landing_row_order::LandingRows>,
    support_watch: support_loss::SupportWatch,
    parked_robots: BTreeMap<BodyId, Vec<RigidBodyHandle>>,
    parked_speed: BTreeMap<BodyId, joints::parked_speed::ParkedSpeed>,
    parked_shared: Vec<articulation_support::ParkedSharedSupport>,
    solved_shared_groups: Vec<SharedSupportRequest>,
    held_supports: BTreeMap<BodyId, Vec<articulation_support::SupportHold>>,
    cold_support_pairs: Vec<(RigidBodyHandle, RigidBodyHandle)>,
    spinning_assemblies: BTreeMap<BodyId, articulation::Transport>,
    free_coast: Option<analytic_coast::FreeCoast>,
    conserved_rotor: Option<BodyId>,
}

// The grouped rigid solver retains a warm friction vector, but not the accumulated
// applied tangential impulse. Its two-sphere observer uses the first sphere's
// momentum change and the two hinge-link impulses. Raw native caches stay untouched.
#[cfg(feature = "partition-checkpoint")]
#[derive(Clone, Copy)]
struct PairObserver {
    // Set only on an extracted sphere-pair graph without a multibody joint.
    island_construction: Option<[u8; 32]>,
    spheres: [BodyId; 2],
    links: [BodyId; 2],
    initial_velocity: [f64; 3],
}
#[cfg(feature = "partition-checkpoint")]
#[derive(Clone, Copy)]
struct GraspPayloadObserver {
    parts: [BodyId; 2],
    incoming_velocity: [f64; 3],
    external_impulse: [f64; 3],
}

#[derive(Clone, Copy, PartialEq)]
enum PrimitiveMode {
    Ordinary,
    Prescribed,
    Articulation,
    AwakeBox { prescribed: bool, friction: f64 },
}

fn rotation(value: [f64; 4]) -> Result<Rotation, PhysicsError> {
    let rotation = Rotation::from_xyzw(value[0], value[1], value[2], value[3]);
    if !rotation.is_finite() || (rotation.length_squared() - 1.0).abs() > 1e-12 {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(rotation)
}

fn inertia_properties(inertia: Inertia, mass: f64) -> Result<MassProperties, PhysicsError> {
    if !finite(inertia.center)
        || !inertia
            .tensor
            .into_iter()
            .map(|v| v * v)
            .sum::<f64>()
            .is_finite()
    {
        return Err(PhysicsError::InvalidValue);
    }
    let q = rotation(inertia.orientation)?;
    let [xx, yy, zz, xy, xz, yz] = inertia.tensor;
    let matrix = Matrix::from_cols_array(&[xx, xy, xz, xy, yy, yz, xz, yz, zz]);
    let properties = inertia_frame::properties(Vector::from(inertia.center), mass, q, matrix)?;
    let inverse = properties.inv_principal_inertia.to_array();
    if !inverse.into_iter().all(|v| v.is_normal() && v > 0.0) {
        return Err(PhysicsError::InvalidValue);
    }
    let principal = inverse.map(|v| 1.0 / v);
    let sum: f64 = principal.into_iter().sum();
    if !sum.is_finite() || principal.into_iter().any(|v| v > sum - v + sum * 1e-12) {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(properties)
}

fn finite(values: [f64; 3]) -> bool {
    values.into_iter().all(f64::is_finite)
}

// Parry uses local Y as the cylinder axis. Gazebo uses local Z.
fn cylinder_frame() -> Pose {
    Pose::from_parts(
        Vector::ZERO,
        Rotation::from_scaled_axis(Vector::X * std::f64::consts::FRAC_PI_2),
    )
}

fn collider(shape: Shape) -> Result<ColliderBuilder, PhysicsError> {
    match shape {
        Shape::Sphere { radius } if radius.is_finite() && radius > 0.0 => {
            Ok(ColliderBuilder::ball(radius))
        }
        Shape::Cylinder {
            radius,
            half_height,
        } if [radius, half_height]
            .into_iter()
            .all(|v| v.is_finite() && v > 0.0) =>
        {
            // Rapier's cylinder axis is Y; the body and inertial frames stay Z-up.
            Ok(ColliderBuilder::cylinder(half_height, radius).position(cylinder_frame()))
        }
        Shape::Capsule {
            radius,
            half_length,
        } if [radius, half_length]
            .into_iter()
            .all(|v| v.is_finite() && v > 0.0) =>
        {
            Ok(ColliderBuilder::capsule_z(half_length, radius))
        }
        Shape::Box {
            half_extents: [x, y, z],
        } if [x, y, z].into_iter().all(|v| v.is_finite() && v > 0.0) => {
            Ok(ColliderBuilder::cuboid(x, y, z))
        }
        _ => Err(PhysicsError::InvalidValue),
    }
}

fn validated_collider(spec: BodySpec, orientation: Rotation) -> Result<Collider, PhysicsError> {
    let shape = collider(spec.shape)?.mass(spec.mass.unwrap_or(1.0)).build();
    let mass = shape.mass_properties();
    let aabb = shape.shape().compute_aabb(
        &(Pose::from_parts(Vector::from(spec.position), orientation) * *shape.position()),
    );
    let positive = |v: f64| v.is_normal() && v > 0.0;
    if !positive(mass.inv_mass)
        || !mass
            .inv_principal_inertia
            .to_array()
            .into_iter()
            .all(positive)
        || !finite(aabb.mins.to_array())
        || !finite(aabb.maxs.to_array())
        || !(aabb.maxs - aabb.mins).to_array().into_iter().all(positive)
    {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(shape)
}

fn insert(
    world: &mut PhysicsWorld,
    spec: &Construction,
    rotational: RotationSpec,
    mut shape: Collider,
    inertia: Option<MassProperties>,
) -> RigidBodyHandle {
    let body = if spec.kinematic {
        RigidBodyBuilder::kinematic_velocity_based()
    } else if spec.mass.is_some() {
        RigidBodyBuilder::dynamic()
    } else {
        RigidBodyBuilder::fixed()
    };
    let mut body = body
        .can_sleep(spec.can_sleep)
        .ccd_enabled(spec.continuous_collision)
        .pose(Pose::from_parts(
            Vector::from(spec.position),
            Rotation::from_xyzw(
                rotational.orientation[0],
                rotational.orientation[1],
                rotational.orientation[2],
                rotational.orientation[3],
            ),
        ))
        .linvel(Vector::from(spec.velocity))
        .angvel(Vector::from(rotational.angular_velocity));
    if let Some(id) = spec.articulation {
        body = body.user_data(u128::from(id.raw()));
    }
    if let Some(mut inertia) = inertia {
        if spec.articulation.is_some() {
            inertia.local_com = Vector::ZERO;
        }
        shape.set_density(0.0);
        if *shape.position() != Pose::IDENTITY {
            // Parry tests zero mass properties by exact structural equality, including
            // their centre and principal frame. A massless offset geometry otherwise
            // forces a second, unnecessary diagonalization of the authored body inertia.
            let local = shape.position().inverse();
            shape.set_mass_properties(MassProperties {
                local_com: local.translation,
                principal_inertia_local_frame: local.rotation,
                ..MassProperties::default()
            });
        }
        body = body.additional_mass_properties(inertia);
    }
    let handle = world.insert(body, shape).0;
    // Impulses may arrive before the first step; initialize effective mass now.
    world.bodies[handle].recompute_mass_properties_from_colliders(&world.colliders);
    handle
}

// Rapier 0.36.1 staged_island_solver/worker.rs uses MAX_ROTATION * inv_dt.
// Keep the same multiplication and rounding for the public-step guard.
fn boundary_spin_limit(dt: Duration) -> f64 {
    std::f64::consts::FRAC_PI_4 * dt.as_secs_f64().recip()
}

impl RapierBackend {
    /// Gravity uses Gazebo's Z-up coordinate convention; no implicit axis conversion.
    pub fn new(gravity: [f64; 3]) -> Result<Self, PhysicsError> {
        if !finite(gravity) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Self {
            #[cfg(feature = "partition-checkpoint")]
            native_profile: None,
            #[cfg(feature = "partition-checkpoint")]
            join_queries_pending: false,
            #[cfg(feature = "partition-checkpoint")]
            pair_observer: None,
            #[cfg(feature = "partition-checkpoint")]
            grasp_payload_observer: None,
            curves: Default::default(),
            fleet_clock: FleetClock::Disabled,
            allow_isolation: false,
            fleet_dispatch: fleet_isolation::Dispatch::Disabled,
            fleet_resume_positions: Default::default(),
            translation_epochs: Default::default(),
            translation_rows: Default::default(),
            translation_rays: Default::default(),
            owner: Arc::new(()),
            exchange_epoch: Arc::new(()),
            world: PhysicsWorld {
                gravity: Vector::from(gravity),
                ..PhysicsWorld::new()
            },
            bodies: BTreeMap::new(),
            joints: joints::Joints::default(),
            next_id: 0,
            pending_wrenches: BTreeSet::new(),
            exchange_reactions: None,
            max_boundary_spin: 0.0,
            held_wrenches: BTreeMap::new(),
            #[cfg(feature = "landing-audit")]
            landing_rows: None,
            support_watch: Default::default(),
            parked_shared: Vec::new(),
            parked_robots: BTreeMap::new(),
            parked_speed: BTreeMap::new(),
            solved_shared_groups: Vec::new(),
            held_supports: BTreeMap::new(),
            cold_support_pairs: Vec::new(),
            spinning_assemblies: BTreeMap::new(),
            free_coast: None,
            conserved_rotor: None,
            welded_projection: None,
            welded_exclusion: None,
        })
    }

    pub fn spawn(&mut self, spec: BodySpec) -> Result<BodyId, PhysicsError> {
        self.spawn_link(spec, RotationSpec::default(), None)
    }

    /// Inserts an infinite-mass moving primitive in this solver's contact domain.
    /// Used by the multi-fidelity adapter. Ordinary Rapier scene admission
    /// continues to use `spawn_link` for dynamic and fixed links.
    pub fn spawn_velocity_boundary(
        &mut self,
        spec: BodySpec,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        self.spawn_primitive(spec, rotational, inertia, PrimitiveMode::Prescribed)
    }

    /// Active shape pairs in the most recently computed narrow phase.
    pub fn active_contact_pairs(&self) -> usize {
        self.world
            .narrow_phase
            .contact_pairs()
            .filter(|pair| pair.has_any_active_contact())
            .count()
    }

    /// Whether the last advancing step disabled non-finite solver objects.
    pub fn last_step_quarantined(&self) -> bool {
        self.welded_projection
            .as_ref()
            .map_or_else(|| !self.world.quarantine().is_empty(), |p| p.quarantined())
    }

    fn validate_boundary_twist(
        &self,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        let linear = Vector::from_array(linear);
        let angular = Vector::from_array(angular);
        let speed = linear.length();
        let spin = angular.length();
        // The pinned solver caps linear speed, and considers a twist with
        // squared norm zero stationary. Refuse before either changes a command.
        if !speed.is_finite()
            || speed > self.world.integration_parameters.max_linear_velocity()
            || !spin.is_finite()
            || (speed == 0.0 && linear != Vector::ZERO)
            || (spin == 0.0 && angular != Vector::ZERO)
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }

    fn refresh_max_boundary_spin(&mut self) {
        self.max_boundary_spin = self
            .bodies
            .values()
            .filter(|entry| entry.1.kinematic)
            .map(|entry| self.world.bodies[entry.0].angvel().length())
            .fold(0.0, f64::max);
    }

    fn observe_boundary_spin(&mut self, angular: [f64; 3]) {
        self.max_boundary_spin = self
            .max_boundary_spin
            .max(Vector::from_array(angular).length());
    }

    /// Project a COM impulse and world angular impulse using the resident
    /// native mass properties, without changing any solver state.
    pub fn velocity_after_impulse(
        &self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<([f64; 3], [f64; 3]), PhysicsError> {
        if !finite(linear) || !finite(angular) {
            return Err(PhysicsError::InvalidValue);
        }
        let (handle, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
        if self.world.multibody_joints_with(*handle).next().is_some() {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "direct reduced-coordinate velocity edits and impulses",
            });
        }
        let body = &self.world.bodies[*handle];
        if body.is_kinematic() {
            return Err(PhysicsError::Unsupported {
                engine: self.engine(),
                capability: "impulse on a prescribed kinematic boundary",
            });
        }
        let mass = body.mass_properties();
        // Rapier skips the corresponding velocity update for a zero impulse.
        // Preserve the current bits, including signed zeros, on that path.
        let next_linear = if linear == [0.; 3] || !body.is_dynamic() {
            body.linvel()
        } else {
            body.linvel() + Vector::from(linear) * mass.effective_inv_mass
        };
        let next_angular = if angular == [0.; 3] || !body.is_dynamic() {
            body.angvel()
        } else {
            body.angvel() + mass.effective_world_inv_inertia * Vector::from(angular)
        };
        if !finite(next_linear.to_array()) || !finite(next_angular.to_array()) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok((next_linear.to_array(), next_angular.to_array()))
    }

    /// Reject speeds the shared solver would clamp at this step duration.
    pub fn validate_velocity_boundary(
        &self,
        linear: [f64; 3],
        angular: [f64; 3],
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        self.validate_boundary_twist(linear, angular)?;
        if !dt.is_zero() && Vector::from_array(angular).length() > boundary_spin_limit(dt) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }

    /// Add a fixed horizontal halfspace with a normal along world +Z. SDF size
    /// controls the visual extent. The physical plane is infinite.
    pub fn spawn_horizontal_halfspace(
        &mut self,
        plane: gz_physics::PlaneSpec,
    ) -> Result<BodyId, PhysicsError> {
        self.horizontal_halfspace(plane, None)
    }
    fn horizontal_halfspace(
        &mut self,
        plane: gz_physics::PlaneSpec,
        friction: Option<IsotropicFriction>,
    ) -> Result<BodyId, PhysicsError> {
        if !finite(plane.position)
            || plane.orientation != [0., 0., 0., 1.]
            || !plane.size.into_iter().all(|x| x.is_finite() && x > 0.)
        {
            return Err(PhysicsError::InvalidValue);
        }
        let next = self
            .next_id
            .checked_add(1)
            .ok_or(PhysicsError::IdOverflow)?;
        let id = BodyId::from_raw(self.next_id);
        let mut collider = ColliderBuilder::new(SharedShape::halfspace(Vector::Z)).build();
        if let Some(material) = friction {
            material.apply(&mut collider);
        }
        let spec = Construction {
            position: plane.position,
            velocity: [0.; 3],
            mass: None,
            kinematic: false,
            can_sleep: true,
            continuous_collision: false,
            articulation: None,
            collider,
        };
        self.rejoin_fleet();
        self.materialize_fleet();
        let rotation = RotationSpec::default();
        let handle = insert(
            &mut self.world,
            &spec,
            rotation,
            spec.collider.clone(),
            None,
        );
        self.bodies.insert(id, (handle, spec, rotation, None, None));
        self.next_id = next;
        Ok(id)
    }

    /// Creates a floating articulation link with its native frame at the COM.
    pub fn spawn_articulation_link(
        &mut self,
        spec: BodySpec,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        self.spawn_primitive(spec, rotational, inertia, PrimitiveMode::Articulation)
    }

    fn spawn_primitive(
        &mut self,
        spec: BodySpec,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
        mode: PrimitiveMode,
    ) -> Result<BodyId, PhysicsError> {
        self.spawn_primitive_with_surface(spec, rotational, inertia, mode, None)
    }

    fn spawn_primitive_with_surface(
        &mut self,
        spec: BodySpec,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
        mode: PrimitiveMode,
        friction: Option<IsotropicFriction>,
    ) -> Result<BodyId, PhysicsError> {
        self.spawn_primitive_geometry(spec, rotational, inertia, mode, friction, &[])
    }

    fn spawn_primitive_geometry(
        &mut self,
        spec: BodySpec,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
        mode: PrimitiveMode,
        friction: Option<IsotropicFriction>,
        parts: &[gz_physics::CollisionSpec],
    ) -> Result<BodyId, PhysicsError> {
        let kinematic = matches!(
            mode,
            PrimitiveMode::Prescribed
                | PrimitiveMode::AwakeBox {
                    prescribed: true,
                    ..
                }
        );
        let awake_box = matches!(mode, PrimitiveMode::AwakeBox { .. });
        if mode == PrimitiveMode::Articulation && spec.mass.is_none() {
            return Err(PhysicsError::InvalidValue);
        }
        if !finite(spec.position)
            || !finite(spec.velocity)
            || spec.mass.is_some_and(|v| !v.is_finite() || v <= 0.0)
            || (!kinematic && spec.mass.is_none() && spec.velocity != [0.0; 3])
            || !finite(rotational.angular_velocity)
            || (!kinematic && spec.mass.is_none() && rotational.angular_velocity != [0.0; 3])
        {
            return Err(PhysicsError::InvalidValue);
        }
        if kinematic {
            self.validate_boundary_twist(spec.velocity, rotational.angular_velocity)?;
        }
        let orientation = rotation(rotational.orientation)?;
        let mut shape = if parts.is_empty() {
            validated_collider(spec, orientation)?
        } else {
            compound::collider_for_link(
                parts,
                spec,
                orientation,
                inertia.map_or(Vector::ZERO, |i| Vector::from(i.center)),
            )?
        };
        if let Some(material) = friction {
            material.apply(&mut shape);
        }
        if let PrimitiveMode::AwakeBox { friction, .. } = mode {
            self.require_separated_box(&shape, spec.position, orientation)?;
            shape.set_friction(friction);
            // Gazebo ODE uses the smaller coefficient of the two surfaces.
            // Preserve the native material bytes of legacy frictionless boxes.
            if friction != 0. {
                shape.set_friction_combine_rule(CoefficientCombineRule::Min);
            }
        }
        let properties = inertia
            .map(|value| inertia_properties(value, spec.mass.unwrap_or(1.0)))
            .transpose()?;
        let next = self
            .next_id
            .checked_add(1)
            .ok_or(PhysicsError::IdOverflow)?;
        let id = BodyId::from_raw(self.next_id);
        let articulation = (mode == PrimitiveMode::Articulation).then_some(id);
        let mut native_position = Vector::from_array(spec.position);
        if let (Some(_), Some(inertia)) = (articulation, inertia) {
            let offset = Vector::from_array(inertia.center);
            native_position += orientation * offset;
            if !native_position.is_finite() {
                return Err(PhysicsError::InvalidValue);
            }
            let mut local = *shape.position();
            local.translation -= offset;
            shape.set_position(local);
        }
        self.rejoin_fleet();
        if !(kinematic
            && rotational.angular_velocity == [0.0; 3]
            && inertia.is_none_or(|i| i.center == [0.0; 3]))
        {
            self.materialize_fleet();
        }
        let spec = Construction {
            position: native_position.to_array(),
            velocity: spec.velocity,
            mass: spec.mass,
            kinematic,
            can_sleep: !awake_box,
            continuous_collision: false,
            articulation,
            collider: shape,
        };
        let handle = insert(
            &mut self.world,
            &spec,
            rotational,
            spec.collider.clone(),
            properties,
        );
        self.bodies
            .insert(id, (handle, spec, rotational, inertia, None));
        self.next_id = next;
        if kinematic {
            self.record_translation_start(id, handle);
            self.observe_boundary_spin(rotational.angular_velocity);
        }
        Ok(id)
    }
}

impl DynamicBackend for RapierBackend {
    fn engine(&self) -> &'static str {
        "rapier"
    }

    /// Creates one centered collider with optional explicit link inertia.
    fn spawn_link(
        &mut self,
        spec: BodySpec,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        self.spawn_primitive(spec, rotational, inertia, PrimitiveMode::Ordinary)
    }

    fn spawn_mesh_link(
        &mut self,
        spec: MeshBodySpec<'_>,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        if !finite(spec.position)
            || !finite(spec.velocity)
            || !finite(spec.collision_position)
            || spec.mass.is_some_and(|m| !m.is_finite() || m <= 0.0)
            || !finite(rotational.angular_velocity)
            || (spec.mass.is_none()
                && (spec.velocity != [0.0; 3] || rotational.angular_velocity != [0.0; 3]))
        {
            return Err(PhysicsError::InvalidValue);
        }
        let orientation = rotation(rotational.orientation)?;
        let collision_orientation = rotation(spec.collision_orientation)?;
        let builder = mesh::collider(spec.vertices, spec.triangles, spec.mass.is_some())?.position(
            Pose::from_parts(Vector::from(spec.collision_position), collision_orientation),
        );
        let shape = if let Some(mass) = spec.mass {
            builder.mass(mass)
        } else {
            builder.density(0.0)
        }
        .build();
        let bounds = shape.shape().compute_aabb(
            &(Pose::from_parts(Vector::from(spec.position), orientation) * *shape.position()),
        );
        if !finite(bounds.mins.to_array()) || !finite(bounds.maxs.to_array()) {
            return Err(PhysicsError::InvalidValue);
        }
        if spec.mass.is_some() {
            let properties = shape.mass_properties();
            if !properties.inv_mass.is_normal()
                || properties.inv_mass <= 0.0
                || !properties
                    .inv_principal_inertia
                    .to_array()
                    .into_iter()
                    .all(|v| v.is_normal() && v > 0.0)
                || !(bounds.maxs - bounds.mins)
                    .to_array()
                    .into_iter()
                    .all(|v| v.is_normal() && v > 0.0)
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        let properties = inertia
            .map(|v| inertia_properties(v, spec.mass.unwrap_or(1.0)))
            .transpose()?;
        let next = self
            .next_id
            .checked_add(1)
            .ok_or(PhysicsError::IdOverflow)?;
        let id = BodyId::from_raw(self.next_id);
        self.rejoin_fleet();
        self.materialize_fleet();
        let construction = Construction {
            position: spec.position,
            velocity: spec.velocity,
            mass: spec.mass,
            kinematic: false,
            can_sleep: true,
            continuous_collision: false,
            articulation: None,
            collider: shape,
        };
        let handle = insert(
            &mut self.world,
            &construction,
            rotational,
            construction.collider.clone(),
            properties,
        );
        self.bodies
            .insert(id, (handle, construction, rotational, inertia, None));
        self.next_id = next;
        Ok(id)
    }

    /// Whether a registered body has dynamic mass rather than a fixed solver body.
    fn is_dynamic(&self, id: BodyId) -> Option<bool> {
        self.bodies.get(&id).map(|(_, spec, ..)| {
            spec.mass.is_some()
                && !spec.kinematic
                && (spec.articulation.is_none() || !self.prescribed_member(id))
        })
    }

    fn is_kinematic(&self, id: BodyId) -> Option<bool> {
        self.bodies.get(&id).map(|(_, spec, ..)| {
            spec.kinematic || (spec.articulation.is_some() && self.prescribed_member(id))
        })
    }

    fn state(&self, id: BodyId) -> Option<BodyState> {
        Some(self.state_from_entry(self.bodies.get(&id)?))
    }

    fn read_states(&self, visit: &mut dyn FnMut(&mut dyn gz_physics::BodyStateLookup)) {
        self.read_body_states(visit);
    }

    fn cast_ray(
        &self,
        origin: [f64; 3],
        direction: [f64; 3],
        max_distance: f64,
        exclude: Option<BodyId>,
    ) -> Result<Option<f64>, PhysicsError> {
        ray::cast(self, origin, direction, max_distance, exclude)
    }

    fn remove(&mut self, id: BodyId) -> bool {
        if !self.release_for_removal(id) {
            return false;
        }
        if self.bodies.contains_key(&id) {
            self.rejoin_fleet();
        }
        if let Some((handle, ..)) = self.bodies.remove(&id) {
            if let Some(rows) = &mut self.exchange_reactions {
                rows.retain(|&(a, b), _| a != id && b != id);
            }
            self.release_welded_projection();
            self.support_watch.retire(handle);
            self.retire_support_body(handle);
            if self.conserved_rotor == Some(id) {
                self.conserved_rotor = None;
            }
            #[cfg(feature = "partition-checkpoint")]
            {
                self.pair_observer = None;
            }
            self.translation_rows.take();
            self.translation_rays.take();
            self.translation_epochs.remove(&handle);
            self.curves.remove(id, handle);
            self.fleet_resume_positions.remove(&handle);
            let reduced = self
                .world
                .multibody_joints
                .rigid_body_link(handle)
                .is_some();
            #[cfg(feature = "partition-checkpoint")]
            {
                self.grasp_payload_observer = None;
            }
            self.pending_wrenches.remove(&id);
            self.held_wrenches.remove(&id);
            self.resume_parked_speed_groups();
            let survivors = self.joints.forget_body(id, &mut self.world);
            self.world.remove_body(handle);
            for survivor in survivors {
                self.release_unconstrained_body(survivor);
            }
            self.joints.release_unused_anchor(&mut self.world);
            if reduced {
                self.joints.initialize_velocities(&mut self.world);
            }
            true
        } else {
            false
        }
    }

    fn validate_body_state(&self, id: BodyId, state: BodyState) -> Result<(), PhysicsError> {
        body_state::prepare(self, id, state).map(|_| ())
    }

    fn set_body_state(&mut self, id: BodyId, state: BodyState) -> Result<(), PhysicsError> {
        let (handle, pose) = body_state::prepare(self, id, state)?;
        self.release_welded_projection();
        self.note_support_edit(handle, Some(pose), state.velocity, state.angular_velocity);
        self.curves.override_link(id);
        self.rejoin_fleet();
        self.fleet_resume_positions.remove(&handle);
        if state.angular_velocity != [0.0; 3] {
            self.materialize_fleet();
        }
        if self.world.bodies[handle].is_kinematic() {
            self.observe_boundary_spin(state.angular_velocity);
        }
        let body = &mut self.world.bodies[handle];
        body.set_position(pose, true);
        body.set_linvel(Vector::from(state.velocity), true);
        body.set_angvel(Vector::from(state.angular_velocity), true);
        self.support_watch.update(&self.world, Some(handle));
        self.record_translation_start(id, handle);
        Ok(())
    }

    /// Replaces world-frame velocities and wakes dynamic bodies. Fixed bodies stay fixed.
    /// Invalid inputs leave both velocities unchanged. Reset keeps construction values.
    fn set_world_velocity(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        if !finite(linear) || !finite(angular) {
            return Err(PhysicsError::InvalidValue);
        }
        let (handle, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
        if self.world.multibody_joints_with(*handle).next().is_some() {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "direct reduced-coordinate velocity edits and impulses",
            });
        }
        let handle = *handle;
        if self.world.bodies[handle].is_kinematic() {
            self.validate_boundary_twist(linear, angular)?;
            self.note_support_edit(handle, None, linear, angular);
            self.observe_boundary_spin(angular);
            if angular == [0.; 3] {
                self.detach_translating_deck(id);
            } else {
                self.rejoin_fleet();
            }
        }
        if angular != [0.; 3] {
            self.materialize_fleet();
        } else if matches!(self.fleet_clock, FleetClock::Translating(_)) {
            let current = self.current_translation(handle, &self.world.bodies[handle]);
            self.world.bodies[handle].set_translation(current, true);
        }
        let body = &mut self.world.bodies[handle];
        body.set_linvel(Vector::from(linear), true);
        body.set_angvel(Vector::from(angular), true);
        self.curves.override_link(id);
        self.record_translation_start(id, handle);
        Ok(())
    }

    /// Accumulates world force at the center of mass and world-frame torque.
    /// The next unpaused step consumes them; fixed bodies remain fixed.
    /// Nonfinite inputs/accumulation leave both force and torque unchanged.
    fn add_world_wrench(
        &mut self,
        id: BodyId,
        force: [f64; 3],
        torque: [f64; 3],
    ) -> Result<(), PhysicsError> {
        if !finite(force) || !finite(torque) {
            return Err(PhysicsError::InvalidValue);
        }
        if self.prescribed_member(id) {
            return self.hold_wrench(id, Vector::from_array(force), Vector::from_array(torque));
        }
        let (handle, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
        let body = &mut self.world.bodies[*handle];
        if body.is_kinematic() {
            return Err(PhysicsError::Unsupported {
                engine: self.engine(),
                capability: "wrench on a prescribed kinematic boundary",
            });
        }
        let force = Vector::from(force);
        let torque = Vector::from(torque);
        if !finite((body.user_force() + force).to_array())
            || !finite((body.user_torque() + torque).to_array())
        {
            return Err(PhysicsError::InvalidValue);
        }
        body.add_force(force, true);
        body.add_torque(torque, true);
        self.pending_wrenches.insert(id);
        Ok(())
    }

    /// Applies linear and angular impulses in world axes. Fixed bodies remain fixed.
    fn apply_impulse(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        self.velocity_after_impulse(id, linear, angular)?;
        let handle = self.bodies[&id].0;
        let body = &mut self.world.bodies[handle];
        body.apply_impulse(Vector::from(linear), true);
        body.apply_torque_impulse(Vector::from(angular), true);
        Ok(())
    }

    fn set_self_collision_group(
        &mut self,
        id: BodyId,
        group: Option<u64>,
    ) -> Result<(), PhysicsError> {
        if !self.joints.construction_open() {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "runtime self-collision group edits",
            });
        }
        let entry = self.bodies.get_mut(&id).ok_or(PhysicsError::MissingBody)?;
        collision::assign(&mut self.world, entry.0, group);
        entry.4 = group;
        self.support_watch.update(&self.world, Some(entry.0));
        Ok(())
    }

    /// Restores remaining bodies to their spawn state. Removed bodies stay removed.
    fn reset(&mut self) -> Result<(), PhysicsError> {
        let joints = self.joints.initial();
        #[cfg(feature = "partition-checkpoint")]
        {
            self.join_queries_pending = false;
        }
        self.pending_wrenches.clear();
        self.exchange_reactions = None;
        self.exchange_epoch = Arc::new(());
        self.release_welded_projection();
        self.held_wrenches.clear();
        self.spinning_assemblies.clear();
        self.parked_robots.clear();
        self.parked_speed.clear();
        self.held_supports.clear();
        self.support_watch = Default::default();
        self.cold_support_pairs.clear();
        self.free_coast = None;
        #[cfg(feature = "partition-checkpoint")]
        let ground = partition_ground::initial_surface(&self.world);
        self.world = PhysicsWorld {
            gravity: self.world.gravity,
            integration_parameters: self.world.integration_parameters,
            ..PhysicsWorld::new()
        };
        #[cfg(feature = "partition-checkpoint")]
        if let Some(h) = ground {
            self.world
                .colliders
                .insert(partition_ground::collider_at(h));
        }
        for (handle, spec, rotational, inertia, group) in self.bodies.values_mut() {
            *handle = initial::insert_initial(&mut self.world, spec, *rotational, *inertia);
            collision::assign(&mut self.world, *handle, *group);
        }
        self.joints = joints::Joints::restore(&joints, &mut self.world, &self.bodies);
        self.refresh_max_boundary_spin();
        self.restore_fleet_clock();
        Ok(())
    }

    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        self.advance_ordinary(dt)
    }

    fn validate_joint_effort(&self, id: JointId, torque: f64) -> Result<(), PhysicsError> {
        self.require_dynamic_joint(id)?;
        self.joints.validate_effort(id, torque, &self.bodies)
    }

    fn add_joint_effort(&mut self, id: JointId, torque: f64) -> Result<(), PhysicsError> {
        self.require_dynamic_joint(id)?;
        self.joints.add_effort(id, torque, &self.bodies)
    }

    fn validate_joint_velocity_motor(
        &self,
        id: JointId,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.require_dynamic_joint(id)?;
        self.joints.validate_motor(id, motor)
    }

    fn set_joint_velocity_motor(
        &mut self,
        id: JointId,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.require_dynamic_joint(id)?;
        self.replace_joint_motor(id, motor)
    }

    fn validate_joint_velocity_servo(
        &self,
        id: JointId,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.require_dynamic_joint(id)?;
        self.joints.validate_motor(id, motor)
    }

    fn set_joint_velocity_servo(
        &mut self,
        id: JointId,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.require_dynamic_joint(id)?;
        self.replace_joint_servo(id, motor)
    }

    fn validate_joint_position_servo(
        &self,
        id: JointId,
        servo: Option<JointPositionServo>,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        self.require_dynamic_joint(id)?;
        self.prepare_joint_position_servo(id, servo, dt).map(|_| ())
    }

    fn set_joint_position_servo(
        &mut self,
        id: JointId,
        servo: Option<JointPositionServo>,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        self.require_dynamic_joint(id)?;
        self.replace_joint_position_servo(id, servo, dt)
    }

    fn add_world_revolute(
        &mut self,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        self.insert_world_revolute(child, anchor, axis)
    }

    fn add_revolute(
        &mut self,
        parent: BodyId,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        self.insert_revolute(Some(parent), child, anchor, axis, joints::Solver::Impulse)
    }

    fn add_reduced_revolute(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        self.insert_revolute(parent, child, anchor, axis, joints::Solver::Reduced)
    }

    fn joint_child(&self, id: JointId) -> Option<BodyId> {
        self.world_joint_child(id)
    }

    fn joint_state(&self, id: JointId) -> Option<JointState> {
        self.world_joint_state(id)
    }

    fn remove_joint(&mut self, id: JointId) -> bool {
        if let Some(child) = self.joint_child(id)
            && !self.release_for_removal(child)
        {
            return false;
        }
        if self.joint_child(id).is_some() {
            self.release_welded_projection();
        }
        let removed = self.delete_world_joint(id);
        #[cfg(feature = "partition-checkpoint")]
        if removed {
            self.pair_observer = None;
        }
        removed
    }

    fn is_jointed(&self, id: BodyId) -> Option<bool> {
        self.has_joint(id)
    }

    fn initial_state(&self) -> InitialPhysicsState {
        self.capture_initial()
    }

    fn owns_initial_state(&self, initial: &InitialPhysicsState) -> bool {
        initial.is_owned_by(&self.owner)
    }

    fn prepare_initial(
        &self,
        initial: &InitialPhysicsState,
    ) -> Result<PreparedPhysicsReset, PhysicsError> {
        self.build_initial(initial)
    }

    fn apply_initial(&mut self, prepared: PreparedPhysicsReset) -> Result<(), PhysicsError> {
        self.commit_initial(prepared)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

#[cfg(test)]
#[path = "../tests/private/ids.rs"]
mod ids;

#[cfg(test)]
#[path = "../tests/private/query_parity.rs"]
mod query_parity;
