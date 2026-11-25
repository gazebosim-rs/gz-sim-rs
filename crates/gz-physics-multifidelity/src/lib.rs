// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Velocity-prescribed boundaries and dynamic links share a single f64 contact
//! solver. The Bevy physics installation controls advancing, pause and reset.
//! Kinematic translations advance at commanded COM velocity. Rotations use
//! Rapier's normalized quaternion integration. Prescribed speed must be at
//! most 400 m/s at the default length scale; the angular command must travel
//! at most pi/4 radians per public step. Excess speeds are refused before the
//! solver can clamp them. World wrenches and impulses are
//! refused for kinematic bodies. On dynamic links, a pending wrench survives
//! pause and applies for the next advancing step; reset clears pending commands.

#[cfg(feature = "partition-checkpoint")]
mod floating_tree;
#[cfg(feature = "partition-checkpoint")]
pub use floating_tree::{
    CLEAR_GRASP_MEMBER_IMAGE_LIMIT, FLOATING_TREE_IMAGE_LIMIT, GRASP_FLEET_IMAGE_LIMIT,
    GRASP_TREE_IMAGE_LIMIT, GROUNDED_ROBOT_IMAGE_LIMIT, GROUNDED_TREE_IMAGE_LIMIT,
    MAX_FLOATING_TREE_LINKS, MAX_GROUNDED_ROBOT_LINKS, MAX_GROUNDED_TREE_LINKS,
    MAX_PAYLOAD_TREE_LINKS, MOTOR_TREE_IMAGE_LIMIT, PAYLOAD_TREE_IMAGE_LIMIT,
    STOPPED_TREE_IMAGE_LIMIT, WARM_GRASP_MEMBER_IMAGE_LIMIT,
};

use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, InitialPhysicsState, JointId,
    JointPositionServo, JointState, JointVelocityMotor, MeshBodySpec, PhysicsError,
    PreparedPhysicsReset, RotationSpec,
};
use gz_physics_rapier::RapierBackend;
pub use gz_physics_rapier::{CoatedAnkle, CoatedAnkleStep, ExchangeInputs, ShankContact};
mod contact_window;
pub use contact_window::ContactWindow;
pub use gz_physics::NormalApproach;
pub use gz_physics::{Bearing, PlanarImpactResponse, PlanarPatchImpact};
pub use gz_physics::{PointImpact, PointImpactResponse};
#[cfg(feature = "landing-audit")]
pub use gz_physics_rapier::ReturnFeatureMode;
pub use gz_physics_rapier::{
    ArticulatedContactSample, ArticulatedContactSite, ArticulatedContactTransaction,
    ArticulatedImpulseReport, ArticulatedNativeInterval, ArticulatedSweep, ArticulationTier,
    BallJointState, BoundPointInterval, BoundTransferReceipt, ContactWarmstart,
    CornerImpactReceipt, DeckEdgeBearingReceipt, EdgeImpactReceipt, FaceBearingReceipt,
    FacePressureReceipt, FiniteEndpointCertificate, FiniteImpactReceipt, FleetIsolationStats,
    FractionalBearingSpan, FractionalPointEvent, FractionalPointReceipt,
    FrictionlessRecontactReceipt, HeldSupportStatus, InternalPointImpact, JointGravityLoad,
    MAX_ARTICULATED_CONTACT_SITES, MixedEdgeImpactReceipt, NormalReaction,
    OpeningEdgeImpactReceipt, PartitionBounds, PartitionBoxBounds, PlanarImpactReceipt,
    RigidContactObservation, RigidContactPatch, RigidPatchPoint, RigidSurfaceWitness, RobotJoint,
    RobotSpring, RockingLimitReceipt, SharedFaceReceipt, SharedSupportLoad, SharedSupportRequest,
    SlidingEdgeImpactReceipt, SlidingPointReceipt, SlidingVertexReceipt, SoftRigidInertia,
    SoftSphereInertia, SoleFoot, SoleTransferReceipt, SolidRay, StickingPointReceipt,
    StoredContactPoint, SupportExpiry, SupportLoss, SupportLossReason, SustainedEdgeReceipt,
    TwoBearingReceipt, UniversalJointState, WeldedAssemblyState, partition_prediction_distance,
    predicts_box_contact,
};
pub use gz_physics_rapier::{
    BoxContactActions, BoxContactOutcome, CompactPointTail, ContactIntervalPair,
    ContactIntervalReceipt, ContactSegment, ContactSolverIterations, ContactSubstep, PointSpanLoad,
};
pub use gz_physics_rapier::{
    BoxCrossing, BoxFlightReceipt, BoxTimingDecline, BoxTimingReceipt, CompactDecline,
    CompactImpactReceipt, IsotropicFriction, MaterialLinkMotion, OpponentClearance,
    OpposingCornerReach, OpposingCornerReceipt, OpposingLimitStage, OpposingNormalDecline,
    OpposingNormalEvent, OpposingNormalReceipt, OpposingNormalStage, OpposingReturnReceipt,
    PulseDecline, PulseReceipt, ReleasedBoxStep, Restitution, RigidBoxSweep, RigidPointDecline,
    RigidPointImpactReceipt, RotationContactPulse,
};
#[cfg(feature = "partition-checkpoint")]
pub use gz_physics_rapier::{
    GROUNDED_REVOLUTE_LIMIT, GroundPatchPoint, GroundReaction, MAX_CHECKPOINT_BODIES,
    MovingReaction, NativeFrictionModel, NativeGroundPatch, NativeGroundPlane, NativeSolverProfile,
    PARTITION_CHECKPOINT_LIMIT, partition_linear_speed_limit, partition_velocity_change,
};
pub use gz_physics_rapier::{ReturnFeatureCounts, ReturnPlaneEvidence, ReturningBoxFeatureReceipt};
use std::{any::Any, time::Duration};

#[cfg(feature = "landing-audit")]
pub use gz_physics_rapier::{
    LandingCachedPoint, LandingFork, LandingRestartState, LandingRowCounts, LandingRowOrder,
    LandingSolverPoint, LandingTrial, RestartPairHistory,
};

#[cfg(feature = "landing-audit")]
impl MultiFidelityBackend {
    /// Build the frictionless, zero-restitution horizontal pad for analytic impact trials.
    pub fn spawn_ideal_landing_pad(&mut self, spec: BodySpec) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_ideal_landing_pad(spec)
            .map_err(engine_error)
    }

    /// Build an oriented, finite frictionless cuboid for analytical reaction audits.
    pub fn spawn_ideal_landing_surface(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
    ) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_ideal_landing_surface(spec, rotation)
            .map_err(engine_error)
    }

    /// Read the four-corner normal mobility from the native locked tree's mass factorization.
    pub fn landing_normal_mobility(
        &self,
        root: BodyId,
        arms: [[f64; 3]; 4],
    ) -> Result<[[f64; 4]; 4], PhysicsError> {
        self.solver
            .landing_normal_mobility(root, arms)
            .map_err(engine_error)
    }

    /// Temporarily select unbiased post-integration sweeps for one native audit solve.
    pub fn step_landing_with_relaxation(
        &mut self,
        dt: Duration,
        passes: usize,
    ) -> Result<(), PhysicsError> {
        self.solver
            .step_landing_with_relaxation(dt, passes)
            .map_err(engine_error)
    }

    /// Recompute native box features on selected dynamic foot and prescribed pad pairs.
    pub fn step_landing_with_fresh_box_pairs(
        &mut self,
        dt: Duration,
        passes: usize,
        pairs: &[(BodyId, BodyId)],
    ) -> Result<(), PhysicsError> {
        self.solver
            .step_landing_with_fresh_box_pairs(dt, passes, pairs)
            .map_err(engine_error)
    }

    /// Run one local response audit with explicit relaxation, bias and warm-start controls.
    pub fn step_landing_response_control(
        &mut self,
        dt: Duration,
        passes: usize,
        bias: bool,
        warmstart: bool,
        recycle: bool,
    ) -> Result<(), PhysicsError> {
        self.solver
            .step_landing_response_control(dt, passes, bias, warmstart, recycle)
            .map_err(engine_error)
    }

    /// Temporarily change the native iteration count for one measured landing solve.
    pub fn step_landing_with_iterations(
        &mut self,
        dt: Duration,
        iterations: usize,
    ) -> Result<(), PhysicsError> {
        self.solver
            .step_landing_with_iterations(dt, iterations)
            .map_err(engine_error)
    }

    /// Permute only native rows in a qualified, observed landing interval.
    pub fn observe_landing_row_order(
        &mut self,
        pair: (BodyId, BodyId),
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
        order: LandingRowOrder,
        observer: impl FnMut(ContactSubstep<'_>),
    ) -> Result<(ContactIntervalReceipt, LandingRowCounts), PhysicsError> {
        self.solver
            .observe_landing_row_order(pair, dt, calls, iterations, order, observer)
            .map_err(engine_error)
    }
    /// Test cached finite-box separating planes on a detached native return.
    pub fn step_returning_box_features(
        &mut self,
        pair: (BodyId, BodyId),
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
        mode: ReturnFeatureMode,
        observer: Option<&mut dyn FnMut(ContactSubstep<'_>)>,
    ) -> Result<(ContactIntervalReceipt, ReturnFeatureCounts), PhysicsError> {
        self.solver
            .step_returning_box_features(pair, dt, calls, iterations, mode, observer)
            .map_err(engine_error)
    }
    /// Time a named native ordering without installing a substep observer.
    pub fn step_landing_row_order(
        &mut self,
        pair: (BodyId, BodyId),
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
        order: LandingRowOrder,
    ) -> Result<(ContactIntervalReceipt, LandingRowCounts), PhysicsError> {
        self.solver
            .step_landing_row_order(pair, dt, calls, iterations, order)
            .map_err(engine_error)
    }

    /// Inspect raw geometry and applied impulses on an enrolled diagnostic pair.
    pub fn landing_solver_points(
        &self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<Vec<LandingSolverPoint>, PhysicsError> {
        self.solver
            .landing_solver_points(foot, pad)
            .map_err(engine_error)
    }
    /// Create an independent local landing continuation at the incoming outer
    /// clock. Its optional donor supplies all rigid and reduced motion.
    pub fn landing_fork(
        &self,
        motion: Option<&Self>,
        root: BodyId,
        pad: BodyId,
        outer_dt: Duration,
    ) -> Result<LandingFork, PhysicsError> {
        self.solver
            .landing_fork(motion.map(|s| &s.solver), root, pad, outer_dt)
            .map_err(engine_error)
    }

    /// Read complete motion, held inputs and all pair histories for an airborne landing audit.
    pub fn airborne_restart_state(
        &self,
        root: BodyId,
        pad: BodyId,
    ) -> Result<LandingRestartState, PhysicsError> {
        self.solver.airborne_restart_state(root, pad)
    }

    /// Clone a qualified local landing. A cold copy recreates both collision graphs on the next native solve.
    pub fn copy_airborne_landing(
        &self,
        root: BodyId,
        pad: BodyId,
        cold: bool,
    ) -> Result<Self, PhysicsError> {
        Ok(Self {
            solver: self.solver.copy_airborne_landing(root, pad, cold)?,
        })
    }

    /// Detached qualification of a locked foot landing on a finite, rising lift.
    /// A donor supplies full rigid-body and reduced motion, retaining this scene's
    /// collider attachments and contact graph. Neither live backend is modified.
    pub fn landing_trial(
        &self,
        motion: Option<&Self>,
        root: BodyId,
        pad: BodyId,
        steps: usize,
        dt: Duration,
        pad_height_offset_m: f64,
    ) -> Result<LandingTrial, PhysicsError> {
        self.solver
            .landing_trial(
                motion.map(|s| &s.solver),
                root,
                pad,
                steps,
                dt,
                pad_height_offset_m,
            )
            .map_err(engine_error)
    }
}

const ENGINE: &str = "multifidelity";

/// One shared collision domain with stable body and joint identifiers across tiers.
pub struct MultiFidelityBackend {
    solver: RapierBackend,
}

impl MultiFidelityBackend {
    pub fn validate_coated_ankle_edge(
        &self,
        ankle: CoatedAnkle,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        self.solver
            .validate_coated_ankle_edge(ankle, dt)
            .map_err(engine_error)
    }
    pub fn step_coated_ankle_edge(
        &mut self,
        ankle: CoatedAnkle,
        dt: Duration,
    ) -> Result<CoatedAnkleStep, PhysicsError> {
        self.solver
            .step_coated_ankle_edge(ankle, dt)
            .map_err(engine_error)
    }
    pub fn validate_coated_ankle(
        &self,
        ankle: CoatedAnkle,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        self.solver
            .validate_coated_ankle(ankle, dt)
            .map_err(engine_error)
    }

    /// One transactional native ankle step with a single owner for sole traction.
    pub fn step_coated_ankle(
        &mut self,
        ankle: CoatedAnkle,
        dt: Duration,
    ) -> Result<CoatedAnkleStep, PhysicsError> {
        self.solver
            .step_coated_ankle(ankle, dt)
            .map_err(engine_error)
    }

    /// Pending world-frame force and torque on the independent coated foot.
    pub fn sole_wrench(
        &self,
        foot: impl Into<SoleFoot>,
        supports: impl ExactSizeIterator<Item = BodyId> + Clone,
    ) -> Result<([f64; 3], [f64; 3]), PhysicsError> {
        self.solver
            .sole_wrench(foot, supports)
            .map_err(engine_error)
    }

    /// Publish both endpoints of an externally integrated sole interval.
    pub fn commit_sole_panels(
        &mut self,
        foot: (impl Into<SoleFoot>, BodyState),
        supports: &[(BodyId, BodyState)],
    ) -> Result<(), PhysicsError> {
        self.solver
            .commit_sole_panels(foot, supports)
            .map_err(engine_error)
    }

    /// Start a local exchange grid after public commands have reached the native solver.
    pub fn exchange_inputs(
        &self,
        dt: Duration,
        count: u32,
    ) -> Result<ExchangeInputs, PhysicsError> {
        self.solver.exchange_inputs(dt, count).map_err(engine_error)
    }
    pub fn repeat_exchange_inputs(&mut self, held: &ExchangeInputs) -> Result<(), PhysicsError> {
        self.solver
            .repeat_exchange_inputs(held)
            .map_err(engine_error)
    }
    pub fn observe_exchange(&self, held: &mut ExchangeInputs) -> Result<(), PhysicsError> {
        self.solver.observe_exchange(held).map_err(engine_error)
    }
    pub fn finish_exchanges(&mut self, held: ExchangeInputs) -> Result<(), PhysicsError> {
        self.solver.finish_exchanges(held).map_err(engine_error)
    }
    pub fn validate_contact_interval(
        &self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
    ) -> Result<(), PhysicsError> {
        self.solver
            .validate_contact_interval(dt, calls, iterations)
            .map_err(engine_error)
    }

    /// Advance one public tick on a native contact grid while holding its inputs.
    pub fn step_contact_interval(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
    ) -> Result<ContactIntervalReceipt, PhysicsError> {
        self.solver
            .step_contact_interval(dt, calls, iterations)
            .map_err(engine_error)
    }

    /// Advance one public tick with different solve budgets on contiguous time spans.
    pub fn step_contact_schedule(
        &mut self,
        dt: Duration,
        segments: &[ContactSegment],
        pairs: &[(BodyId, BodyId)],
    ) -> Result<(ContactIntervalReceipt, u64), PhysicsError> {
        self.solver
            .step_contact_schedule(dt, segments, pairs)
            .map_err(engine_error)
    }

    /// Inspect the committed native state at each scheduled boundary.
    pub fn observe_contact_schedule(
        &mut self,
        dt: Duration,
        segments: &[ContactSegment],
        pairs: &[(BodyId, BodyId)],
        observer: impl FnMut(ContactSubstep<'_>),
    ) -> Result<(ContactIntervalReceipt, u64), PhysicsError> {
        self.solver
            .observe_contact_schedule(dt, segments, pairs, observer)
            .map_err(engine_error)
    }

    /// Active native geometry on the last step for a single-collider box pair.
    pub fn has_box_contact(&self, a: BodyId, b: BodyId) -> bool {
        self.solver.has_box_contact(a, b)
    }

    /// Refine the public interval while recomputing native convex features for selected box pairs.
    pub fn step_contact_interval_with_box_features(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
        pairs: &[(BodyId, BodyId)],
    ) -> Result<(ContactIntervalReceipt, u64), PhysicsError> {
        self.solver
            .step_contact_interval_with_box_features(dt, calls, iterations, pairs)
            .map_err(engine_error)
    }

    /// Update selected box pair geometry at the unmodified public cadence.
    pub fn step_box_features(
        &mut self,
        dt: Duration,
        pairs: &[(BodyId, BodyId)],
    ) -> Result<u64, PhysicsError> {
        self.solver
            .step_box_features(dt, pairs)
            .map_err(engine_error)
    }
    /// Causal fixed-budget native time registration for selected finite box contacts.
    pub fn step_prospective_box_interval(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
        fresh_pairs: &[(BodyId, BodyId)],
        timing_pairs: &[(BodyId, BodyId)],
    ) -> Result<(ContactIntervalReceipt, u64, BoxTimingReceipt), PhysicsError> {
        self.solver
            .step_prospective_box_interval(dt, calls, iterations, fresh_pairs, timing_pairs)
            .map_err(engine_error)
    }

    /// Observe each native commit resulting from live prospective time registration.
    pub fn observe_prospective_box_interval(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
        fresh_pairs: &[(BodyId, BodyId)],
        timing_pairs: &[(BodyId, BodyId)],
        observer: impl FnMut(ContactSubstep<'_>),
    ) -> Result<(ContactIntervalReceipt, u64, BoxTimingReceipt), PhysicsError> {
        self.solver
            .observe_prospective_box_interval(
                dt,
                calls,
                iterations,
                fresh_pairs,
                timing_pairs,
                observer,
            )
            .map_err(engine_error)
    }

    /// Observe the committed native contact states on a single actuator tick.
    pub fn observe_contact_interval(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: impl Into<ContactSolverIterations>,
        observer: impl FnMut(ContactSubstep<'_>),
    ) -> Result<ContactIntervalReceipt, PhysicsError> {
        self.solver
            .observe_contact_interval(dt, calls, iterations, observer)
            .map_err(engine_error)
    }

    /// Set the number of fixed-time PGS sweeps per native solver iteration.
    pub fn set_contact_pgs_passes(&mut self, passes: usize) -> Result<(), PhysicsError> {
        self.solver
            .set_contact_pgs_passes(passes)
            .map_err(engine_error)
    }

    pub fn contact_pgs_passes(&self) -> usize {
        self.solver.contact_pgs_passes()
    }

    /// Install a cubic Hermite translation on an authored free kinematic primitive.
    pub fn set_initial_translation_path(
        &mut self,
        id: BodyId,
        path: gz_physics::trajectory::TranslationSpline,
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_initial_translation_path(id, path)
            .map_err(engine_error)
    }
    pub fn translation_path(
        &self,
        id: BodyId,
    ) -> Option<(
        gz_physics::trajectory::TranslationSpline,
        [f64; 3],
        Duration,
    )> {
        self.solver.translation_path(id)
    }
    pub fn translation_sample(
        &self,
        id: BodyId,
    ) -> Option<gz_physics::trajectory::TranslationSample> {
        self.solver.translation_sample(id)
    }
    /// Count of contact substeps and the union of their solved and speculative pairs.
    pub fn translation_interval(&self) -> (usize, &[RigidContactObservation]) {
        self.solver.translation_interval()
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn prepare_trajectory_owner(&mut self) -> Result<(), PhysicsError> {
        self.solver.prepare_trajectory_owner().map_err(engine_error)
    }
    #[cfg(feature = "partition-checkpoint")]
    pub fn checkpoint_trajectory(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_trajectory().map_err(engine_error)
    }
    #[cfg(feature = "partition-checkpoint")]
    pub fn restore_trajectory(&mut self, image: &[u8]) -> Result<(), PhysicsError> {
        self.solver.restore_trajectory(image).map_err(engine_error)
    }
    #[cfg(feature = "partition-checkpoint")]
    pub fn trajectory_construction(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.trajectory_construction().map_err(engine_error)
    }
    #[cfg(feature = "partition-checkpoint")]
    pub fn from_trajectory_construction(bytes: &[u8]) -> Result<Self, PhysicsError> {
        Ok(Self {
            solver: RapierBackend::from_trajectory_construction(bytes).map_err(engine_error)?,
        })
    }
    #[cfg(feature = "partition-checkpoint")]
    pub fn install_trajectory(&mut self, image: &[u8]) -> Result<(), PhysicsError> {
        self.solver.install_trajectory(image).map_err(engine_error)
    }
    /// Capture the isolated two-link moving native hinge in its bounded dynamic profile.
    #[cfg(feature = "partition-checkpoint")]
    pub fn checkpoint_floating_revolute(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_floating_revolute()
    }

    /// Atomically restore an isolated hinge into a fresh, identically authored backend.
    #[cfg(feature = "partition-checkpoint")]
    pub fn restore_floating_revolute(&mut self, image: &[u8]) -> Result<(), PhysicsError> {
        self.solver.restore_floating_revolute(image)
    }

    /// Native substeps in one tick of the moving two-link worker profile.
    #[cfg(feature = "partition-checkpoint")]
    pub fn moving_solver_substeps() -> usize {
        RapierBackend::moving_solver_substeps()
    }

    /// Accumulated one-step torque on a resident native joint.
    pub fn pending_joint_effort(&self, id: JointId) -> Option<f64> {
        self.solver.pending_joint_effort(id)
    }

    /// Read pending torque from a compact or coupled moving-hinge checkpoint.
    #[cfg(feature = "partition-checkpoint")]
    pub fn moving_checkpoint_effort(bytes: &[u8]) -> Result<Option<f64>, PhysicsError> {
        RapierBackend::moving_checkpoint_effort(bytes)
    }

    /// Capture the complete native contact and articulated state at a 1 ms barrier.
    #[cfg(feature = "partition-checkpoint")]
    pub fn checkpoint_grounded_revolute(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_grounded_revolute()
    }

    /// Restore an image from a trusted, same-build worker with identical construction.
    #[cfg(feature = "partition-checkpoint")]
    pub fn restore_grounded_revolute(&mut self, image: &[u8]) -> Result<(), PhysicsError> {
        self.solver.restore_grounded_revolute(image)
    }

    /// Impulses on one articulated link from its fixed horizontal ground.
    #[cfg(feature = "partition-checkpoint")]
    pub fn ground_reaction(
        &self,
        link: BodyId,
        ground: BodyId,
    ) -> Result<GroundReaction, PhysicsError> {
        self.solver.ground_reaction(link, ground)
    }

    /// Honor an authored no-sleep policy while preserving native contact integration settings.
    pub fn keep_body_awake(&mut self, id: BodyId) -> Result<(), PhysicsError> {
        self.solver.keep_body_awake(id)
    }

    /// Configure awake bodies and per-tick contact geometry for moving hinge workers.
    pub fn configure_moving_body(&mut self, id: BodyId) -> Result<(), PhysicsError> {
        self.solver.configure_moving_body(id)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn configure_pair_revolute(&mut self) -> Result<(), PhysicsError> {
        self.solver.configure_pair_revolute()
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn checkpoint_pair_revolute(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_pair_revolute()
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn restore_pair_revolute(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.solver.restore_pair_revolute(bytes)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn pair_sphere_active(&self, sphere: BodyId) -> Result<bool, PhysicsError> {
        self.solver.pair_sphere_active(sphere)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn set_pair_sphere_active(
        &mut self,
        sphere: BodyId,
        active: bool,
        state: Option<BodyState>,
    ) -> Result<(), PhysicsError> {
        self.solver.set_pair_sphere_active(sphere, active, state)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn pair_reaction(
        &self,
        first: BodyId,
        second: BodyId,
    ) -> Result<MovingReaction, PhysicsError> {
        self.solver.pair_reaction(first, second)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn extract_sphere_island(&mut self, keep_spheres: bool) -> Result<(), PhysicsError> {
        self.solver.extract_sphere_island(keep_spheres)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn sphere_island_drained(&self) -> Result<bool, PhysicsError> {
        self.solver.sphere_island_drained()
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn checkpoint_sphere_island(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_sphere_island()
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn restore_sphere_island(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.solver.restore_sphere_island(bytes)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn merge_sphere_island(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.solver.merge_sphere_island(bytes)
    }

    /// Capture the coupled tree and moving sphere, including live contact caches.
    #[cfg(feature = "partition-checkpoint")]
    pub fn checkpoint_moving_revolute(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_moving_revolute()
    }

    /// Reinstall a trusted same-build, same-construction moving-sphere graph.
    #[cfg(feature = "partition-checkpoint")]
    pub fn restore_moving_revolute(&mut self, image: &[u8]) -> Result<(), PhysicsError> {
        self.solver.restore_moving_revolute(image)
    }

    /// World-space native contact impulse on the hinge link from a separate sphere.
    #[cfg(feature = "partition-checkpoint")]
    pub fn moving_reaction(
        &self,
        link: BodyId,
        sphere: BodyId,
    ) -> Result<MovingReaction, PhysicsError> {
        self.solver.moving_reaction(link, sphere)
    }

    /// Export the compact native hinge motion once the third-body cache has drained.
    #[cfg(feature = "partition-checkpoint")]
    pub fn detached_hinge_image(&self, sphere: BodyId) -> Result<Vec<u8>, PhysicsError> {
        self.solver.detached_hinge_image(sphere)
    }

    /// Install the physical motion of a sphere from a detached worker.
    #[cfg(feature = "partition-checkpoint")]
    pub fn place_detached_sphere(
        &mut self,
        sphere: BodyId,
        state: BodyState,
    ) -> Result<(), PhysicsError> {
        self.solver.place_detached_sphere(sphere, state)
    }

    /// Current native/analytic counts when mixed fleet isolation is active.
    pub fn fleet_isolation_stats(&self) -> Option<FleetIsolationStats> {
        self.solver.fleet_isolation_stats()
    }

    /// Native contact participation of a prescribed deck in an active fleet.
    pub fn fleet_deck_is_native(&self, id: BodyId) -> Option<bool> {
        self.solver.fleet_deck_is_native(id)
    }

    /// Keep mixed worlds fully native, including after reset.
    pub fn disable_fleet_isolation(&mut self) {
        self.solver.disable_fleet_isolation();
    }

    pub fn new(gravity: [f64; 3]) -> Result<Self, PhysicsError> {
        Ok(Self {
            solver: RapierBackend::new_multifidelity(gravity)?,
        })
    }

    /// Select a Coulomb constraint per point on local rigid contacts.
    pub fn new_per_point_coulomb(gravity: [f64; 3]) -> Result<Self, PhysicsError> {
        Ok(Self {
            solver: RapierBackend::new_multifidelity_with_friction(
                gravity,
                gz_physics_rapier::NativeFrictionModel::PerPoint,
            )?,
        })
    }

    /// Construct a native worker solver with its negotiated friction model.
    #[cfg(feature = "partition-checkpoint")]
    pub fn new_with_profile(
        gravity: [f64; 3],
        profile: NativeSolverProfile,
    ) -> Result<Self, PhysicsError> {
        Ok(Self {
            solver: RapierBackend::new_multifidelity_with_profile(gravity, profile)?,
        })
    }

    /// Project an immediate world impulse against this native body's current inertia.
    pub fn velocity_after_impulse(
        &self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<([f64; 3], [f64; 3]), PhysicsError> {
        self.solver
            .velocity_after_impulse(id, linear, angular)
            .map_err(engine_error)
    }

    /// Use the momentum-preserving rotation integrator while this body is isolated.
    pub fn enable_momentum_preserving_rotation(&mut self, id: BodyId) -> Result<(), PhysicsError> {
        self.solver.enable_momentum_preserving_rotation(id)
    }

    /// Native inverse mass and scalar inverse inertia of an eligible free sphere.
    pub fn soft_sphere_mobility(&self, id: BodyId) -> Result<(f64, f64), PhysicsError> {
        self.solver.soft_sphere_mobility(id)
    }

    /// Native principal tensor, including orientation relative to the link.
    pub fn soft_sphere_inertia(&self, id: BodyId) -> Result<SoftSphereInertia, PhysicsError> {
        self.solver.soft_sphere_inertia(id)
    }

    /// Principal inertia of a centered cuboid without other native colliders.
    pub fn soft_cuboid_inertia(&self, id: BodyId) -> Result<SoftRigidInertia, PhysicsError> {
        self.solver.soft_cuboid_inertia(id)
    }
    /// Commit the complete coupled state of a single isolated cuboid.
    pub fn commit_soft_cuboid(&mut self, id: BodyId, state: BodyState) -> Result<(), PhysicsError> {
        self.solver.commit_soft_cuboid(id, state)
    }
    /// Principal inertia of a free, centered capsule without other native colliders.
    pub fn soft_capsule_inertia(&self, id: BodyId) -> Result<SoftRigidInertia, PhysicsError> {
        self.solver.soft_capsule_inertia(id)
    }
    /// Commit the complete coupled state of a single isolated capsule.
    pub fn commit_soft_capsule(
        &mut self,
        id: BodyId,
        state: BodyState,
    ) -> Result<(), PhysicsError> {
        self.solver.commit_soft_capsule(id, state)
    }
    /// Validated native motion of a translating tree or a one-coordinate rotor.
    pub fn begin_articulated_material_interval(
        &mut self,
        sites: &[BodyId],
    ) -> Result<ArticulatedNativeInterval<'_>, PhysicsError> {
        self.solver
            .begin_articulated_material_interval(sites)
            .map_err(engine_error)
    }
    pub fn articulated_sweep(
        &self,
        sites: &[BodyId],
        dt: Duration,
    ) -> Result<ArticulatedSweep, PhysicsError> {
        self.solver.articulated_sweep(sites, dt)
    }

    /// Uniform native world-frame velocity of an isolated passive tree during a certified approach.
    pub fn articulated_translation(
        &self,
        sites: &[BodyId],
        dt: Duration,
    ) -> Result<[f64; 3], PhysicsError> {
        self.solver.articulated_translation(sites, dt)
    }

    /// Measured world center and radius of one native articulated sphere link.
    pub fn articulated_sphere_geometry(&self, id: BodyId) -> Result<([f64; 3], f64), PhysicsError> {
        self.solver.articulated_sphere_geometry(id)
    }
    /// Frozen native multi-link mobility and a single-use aggregate impulse transaction.
    pub fn prepare_articulated_contacts(
        &mut self,
        sites: &[ArticulatedContactSite],
    ) -> Result<ArticulatedContactTransaction<'_>, PhysicsError> {
        self.solver.prepare_articulated_contacts(sites)
    }

    /// Query an isolated finite-box point at tangency or during isotropic locked flight.
    pub fn tangent_material_point(&self, foot: BodyId, pad: BodyId) -> Result<bool, PhysicsError> {
        self.solver
            .tangent_material_point(foot, pad)
            .map_err(engine_error)
    }
    pub fn finite_segment_closing(&self, foot: BodyId, pad: BodyId) -> Result<bool, PhysicsError> {
        self.solver
            .finite_segment_closing(foot, pad)
            .map_err(engine_error)
    }
    pub fn finite_point_closing(&self, foot: BodyId, pad: BodyId) -> Result<bool, PhysicsError> {
        self.solver
            .finite_point_closing(foot, pad)
            .map_err(engine_error)
    }
    pub fn finite_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        self.solver
            .finite_arrival(foot, pad, dt)
            .map_err(engine_error)
    }
    pub fn fractional_point_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        self.solver
            .fractional_point_arrival(foot, pad, dt)
            .map_err(engine_error)
    }
    /// Nominated skew edge contact with a bounded outgoing material response.
    pub fn certified_skew_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
        also_faces: bool,
    ) -> Result<Option<Duration>, PhysicsError> {
        self.solver
            .finite_arrival_with_witnesses(foot, pad, dt, also_faces, true)
            .map_err(engine_error)
    }
    /// Nominated arrival using a finite face and its strictly isolated incoming vertex.
    pub fn certified_face_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        self.solver
            .finite_arrival_certified(foot, pad, dt, true)
            .map_err(engine_error)
    }
    /// Whether the named pad's interior is tangent to a unique cuboid vertex.
    pub fn corner_at_support_face(
        &self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<bool, PhysicsError> {
        self.solver
            .corner_at_support_face(foot, support)
            .map_err(engine_error)
    }

    /// Predict an isolated translating tree's first cold corner arrival.
    pub fn planar_at_support_face(&self, foot: BodyId, pad: BodyId) -> Result<bool, PhysicsError> {
        self.solver
            .planar_at_support_face(foot, pad)
            .map_err(engine_error)
    }
    pub fn planar_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        self.solver
            .planar_arrival(foot, pad, dt)
            .map_err(engine_error)
    }
    pub fn corner_arrival(
        &self,
        foot: BodyId,
        support: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        self.solver
            .corner_arrival(foot, support, dt)
            .map_err(engine_error)
    }

    /// Ordinary temporal work on the local native clock.
    pub fn ordinary_contact_iterations(&self) -> ContactSolverIterations {
        self.solver.ordinary_contact_iterations()
    }

    /// Use selected native box actions and report every signed interval reaction.
    pub fn opposing_corner_reach(
        &self,
        foot: BodyId,
        pad: BodyId,
        gate: BodyId,
        dt: Duration,
    ) -> Result<OpposingCornerReach, PhysicsError> {
        self.solver
            .opposing_corner_reach(foot, pad, gate, dt)
            .map_err(engine_error)
    }
    /// Nominate a primary landing with a second registered, translating box.
    pub fn rigid_box_sweep_with_opponent(
        &self,
        foot: BodyId,
        pad: BodyId,
        opponent: BodyId,
        dt: Duration,
    ) -> Result<RigidBoxSweep, PhysicsError> {
        self.solver
            .rigid_box_sweep_with_opponent(foot, pad, Some(opponent), dt)
            .map_err(engine_error)
    }
    pub fn rigid_box_sweep(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<RigidBoxSweep, PhysicsError> {
        self.solver
            .rigid_box_sweep(foot, pad, dt)
            .map_err(engine_error)
    }
    pub fn advance_box_contacts(
        &mut self,
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
        actions: BoxContactActions<'_>,
        observer: Option<&mut dyn FnMut(ContactSubstep<'_>)>,
    ) -> Result<BoxContactOutcome, PhysicsError> {
        self.solver
            .advance_box_contacts(dt, calls, iterations, actions, observer)
            .map_err(engine_error)
    }

    /// Does the current two-vertex contact qualify under any supported edge regime?
    pub fn coplanar_face_now(
        &self,
        foot: BodyId,
        pad: BodyId,
        allow_kinetic: bool,
    ) -> Result<bool, PhysicsError> {
        self.solver
            .coplanar_face_now(foot, pad, allow_kinetic)
            .map_err(engine_error)
    }
    pub fn frictionless_face_now(&self, foot: BodyId, pad: BodyId) -> Result<bool, PhysicsError> {
        self.solver
            .frictionless_face_now(foot, pad)
            .map_err(engine_error)
    }
    pub fn frictionless_bearing_now(
        &self,
        foot: BodyId,
        pad: BodyId,
        allow_flat: bool,
    ) -> Result<bool, PhysicsError> {
        self.solver
            .frictionless_bearing_now(foot, pad, allow_flat)
            .map_err(engine_error)
    }
    pub fn frictionless_sole_closing(
        &self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<bool, PhysicsError> {
        self.solver
            .frictionless_sole_closing(foot, pad)
            .map_err(engine_error)
    }
    pub fn frictionless_sole_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: std::time::Duration,
    ) -> Result<Option<std::time::Duration>, PhysicsError> {
        self.solver
            .frictionless_sole_arrival(foot, pad, dt)
            .map_err(engine_error)
    }
    pub fn project_normal_face_impact(
        &mut self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<Option<FacePressureReceipt>, PhysicsError> {
        self.solver
            .project_normal_face_impact(foot, pad)
            .map_err(engine_error)
    }
    pub fn edge_at_support_face(&self, foot: BodyId, pad: BodyId) -> Result<bool, PhysicsError> {
        self.solver
            .edge_at_support_face(foot, pad)
            .map_err(engine_error)
    }
    pub fn edge_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        self.solver
            .edge_arrival(foot, pad, dt)
            .map_err(engine_error)
    }
    /// Classify and commit a cold two-site impact in the live native articulation.
    pub fn project_edge_impact(
        &mut self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<Option<EdgeImpactReceipt>, PhysicsError> {
        self.solver
            .project_edge_impact(foot, pad)
            .map_err(engine_error)
    }
    pub fn mixed_edge_at_support_face(
        &self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<bool, PhysicsError> {
        self.solver
            .mixed_edge_at_support_face(foot, pad)
            .map_err(engine_error)
    }
    pub fn mixed_edge_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        self.solver
            .mixed_edge_arrival(foot, pad, dt)
            .map_err(engine_error)
    }
    pub fn sliding_edge_at_support_face(
        &self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<bool, PhysicsError> {
        self.solver
            .sliding_edge_at_support_face(foot, pad)
            .map_err(engine_error)
    }
    pub fn sliding_edge_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        self.solver
            .sliding_edge_arrival(foot, pad, dt)
            .map_err(engine_error)
    }
    /// Apply the simultaneous plastic sliding impact at two vertices of a cold cuboid edge.
    pub fn project_sliding_edge_impact(
        &mut self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<Option<SlidingEdgeImpactReceipt>, PhysicsError> {
        self.solver
            .project_sliding_edge_impact(foot, support)
            .map_err(engine_error)
    }

    /// Apply a cold two-vertex impact with at least one sticking endpoint.
    /// Both-stick results retain the full feasible longitudinal impulse range.
    pub fn project_mixed_edge_impact(
        &mut self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<Option<MixedEdgeImpactReceipt>, PhysicsError> {
        self.solver
            .project_mixed_edge_impact(foot, support)
            .map_err(engine_error)
    }

    /// Apply the isolated plastic Coulomb impact at a single cold cuboid corner.
    /// Unqualified geometry or a coupled constraint leaves the native state untouched.
    pub fn project_corner_impact(
        &mut self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<Option<CornerImpactReceipt>, PhysicsError> {
        self.solver
            .project_corner_impact(foot, support)
            .map_err(engine_error)
    }

    /// Uniform world acceleration from native gravity and the actual applied COM force.
    pub fn soft_linear_acceleration(&self, id: BodyId) -> Result<Option<[f64; 3]>, PhysicsError> {
        self.solver.soft_linear_acceleration(id)
    }
    /// Both free capsules exclusively own the two native colliders of this world.
    pub fn soft_capsule_pair_inertia(
        &self,
        ids: [BodyId; 2],
    ) -> Result<[SoftRigidInertia; 2], PhysicsError> {
        self.solver.soft_capsule_pair_inertia(ids)
    }
    /// Atomically validate and commit both full states of the enrolled capsule pair.
    pub fn commit_soft_capsule_pair(
        &mut self,
        updates: [(BodyId, BodyState, BodyState); 2],
    ) -> Result<(), PhysicsError> {
        self.solver.commit_soft_capsule_pair(updates)
    }
    /// Apply the end states from coupled sphere/particle integration as a batch.
    pub fn commit_soft_spheres(
        &mut self,
        updates: &[(BodyId, BodyState)],
    ) -> Result<(), PhysicsError> {
        self.solver.commit_soft_spheres(updates)
    }

    /// Insert an initially separated, always-awake, frictionless box. A prescribed
    /// box retains its supplied mass for later materialization as a dynamic body.
    pub fn spawn_awake_frictionless_box(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        prescribed: bool,
    ) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_awake_frictionless_box(spec, rotation, prescribed)
            .map_err(engine_error)
    }

    /// Centered native box with isotropic sliding friction (pairwise minimum).
    pub fn spawn_partition_box(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        prescribed: bool,
        friction: f64,
    ) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_partition_box(spec, rotation, prescribed, friction)
            .map_err(engine_error)
    }

    pub fn spawn_partition_primitive(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        prescribed: bool,
        friction: f64,
    ) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_partition_primitive(spec, rotation, prescribed, friction)
            .map_err(engine_error)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn partition_friction(&self, id: BodyId) -> Option<f64> {
        self.solver.partition_friction(id)
    }

    /// Revalidate an unloaded finite-box pair when its retained contact plane
    /// predicts a constraint and the current boxes have a stronger separating axis.
    pub fn recheck_separated_boxes(
        &mut self,
        moving: BodyId,
        fixed: BodyId,
        dt: Duration,
    ) -> Result<bool, PhysicsError> {
        self.solver
            .recheck_separated_boxes(moving, fixed, dt)
            .map_err(engine_error)
    }

    /// Native persistent features and the impulse seed at each contact point.
    pub fn stored_contact_points(&self) -> Vec<StoredContactPoint> {
        self.solver.stored_contact_points()
    }

    /// Effective per-pair warm start, separate from impulses delivered during a step.
    /// A pair marked for discard has no seeds for its next solve.
    pub fn contact_warmstart(
        &self,
        a: BodyId,
        b: BodyId,
    ) -> Result<ContactWarmstart, PhysicsError> {
        self.solver.contact_warmstart(a, b).map_err(engine_error)
    }

    /// Raw native history, including dormant points awaiting a selective discard.
    pub fn native_contact_warmstart(
        &self,
        a: BodyId,
        b: BodyId,
    ) -> Result<ContactWarmstart, PhysicsError> {
        self.solver
            .native_contact_warmstart(a, b)
            .map_err(engine_error)
    }

    /// A selective cold requery is pending for this exact body pair.
    pub fn contact_warmstart_discard_pending(
        &self,
        a: BodyId,
        b: BodyId,
    ) -> Result<bool, PhysicsError> {
        self.solver
            .contact_warmstart_discard_pending(a, b)
            .map_err(engine_error)
    }

    /// Constant-time check for any prescribed assembly with a latched support loss.
    pub fn has_pending_support_loss(&self) -> bool {
        self.solver.has_pending_support_loss()
    }

    /// The first lost load-bearing contact in this prescribed interval.
    pub fn pending_support_loss(&self, root: BodyId) -> Option<SupportLoss> {
        self.solver.pending_support_loss(root)
    }

    /// Eligibility and original face-anchor projection for a prescribed link's retained support.
    pub fn held_support_status(&self, link: BodyId) -> Option<HeldSupportStatus> {
        self.solver.held_support_status(link)
    }

    /// Status of a captured fixed or parked velocity-based support beneath a prescribed link.
    pub fn held_support_pair_status(
        &self,
        link: BodyId,
        support: BodyId,
    ) -> Option<HeldSupportStatus> {
        self.solver.held_support_pair_status(link, support)
    }

    /// Signed normal load over the ordinary public step, including prescribed motion slices.
    pub fn normal_reaction_on(
        &self,
        body: BodyId,
        support: BodyId,
    ) -> Result<Option<NormalReaction>, PhysicsError> {
        self.solver
            .normal_reaction_on(body, support)
            .map_err(engine_error)
    }

    /// All native narrow-phase pairs, including speculative contact caches.
    pub fn rigid_contact_observations(&self) -> Vec<RigidContactObservation> {
        self.solver.rigid_contact_observations()
    }

    /// Geometric material points, public link IDs and child shape indices at their current poses.
    pub fn rigid_surface_witnesses(&self) -> Vec<[RigidSurfaceWitness; 2]> {
        self.solver.rigid_surface_witnesses()
    }

    /// Contact-point geometry and per-point normal impulses from the last native rigid solve.
    pub fn rigid_contact_patches(&self) -> Vec<RigidContactPatch> {
        self.solver.rigid_contact_patches()
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn set_partition_step(&mut self, step: std::time::Duration) -> Result<(), PhysicsError> {
        self.solver.set_partition_step(step).map_err(engine_error)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn finish_refined_partition_step(&mut self) {
        self.solver.finish_refined_partition_step();
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn partition_step_matches(&self, step: std::time::Duration) -> bool {
        self.solver.partition_step_matches(step)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn set_partition_ground(&mut self, height: f64) -> Result<(), PhysicsError> {
        self.solver
            .set_partition_ground(height)
            .map_err(engine_error)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn set_partition_ground_material(
        &mut self,
        h: f64,
        friction: f64,
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_partition_ground_material(h, friction)
            .map_err(engine_error)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn set_partition_plane(
        &mut self,
        point: [f64; 3],
        normal: [f64; 3],
        friction: f64,
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_partition_plane(point, normal, friction)
            .map_err(engine_error)
    }
    #[cfg(feature = "partition-checkpoint")]
    pub fn partition_plane(&self) -> Result<Option<NativeGroundPlane>, PhysicsError> {
        self.solver.partition_plane().map_err(engine_error)
    }
    #[cfg(feature = "partition-checkpoint")]
    pub fn partition_ground_material(&self) -> Result<Option<(f64, f64)>, PhysicsError> {
        self.solver
            .partition_ground_material()
            .map_err(engine_error)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn partition_ground_height(&self) -> Result<Option<f64>, PhysicsError> {
        self.solver.partition_ground_height().map_err(engine_error)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn partition_ground_contacts(&self) -> Vec<gz_physics_rapier::NativeGroundContact> {
        self.solver.partition_ground_contacts()
    }

    /// On-demand halfspace manifold points from the latest native substep.
    #[cfg(feature = "partition-checkpoint")]
    pub fn partition_ground_patches(&self) -> Vec<NativeGroundPatch> {
        self.solver.partition_ground_patches()
    }

    /// On-demand points and loads on an authored fixed plane, in world coordinates.
    #[cfg(feature = "partition-checkpoint")]
    pub fn fixed_ground_patches(
        &self,
        ground: BodyId,
    ) -> Result<Vec<NativeGroundPatch>, PhysicsError> {
        self.solver
            .fixed_ground_patches(ground)
            .map_err(engine_error)
    }

    /// Refill reusable sorted native pair and ground receipts after one solve.
    #[cfg(feature = "partition-checkpoint")]
    pub fn read_partition_contacts(
        &self,
        pairs: &mut Vec<RigidContactObservation>,
        supports: Option<&mut Vec<gz_physics_rapier::NativeGroundContact>>,
    ) {
        self.solver.read_partition_contacts(pairs, supports);
    }

    /// Set uniform world acceleration for a native distributed solver interval.
    #[cfg(feature = "partition-checkpoint")]
    pub fn set_partition_gravity(&mut self, gravity: [f64; 3]) -> Result<(), PhysicsError> {
        self.solver
            .set_partition_gravity(gravity)
            .map_err(engine_error)
    }

    /// Serialize this complete dynamic-box partition at a barrier.
    #[cfg(feature = "partition-checkpoint")]
    pub fn checkpoint_boxes(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_boxes()
    }

    /// Stage a native union of independent finite-box solvers for their next advancing step.
    #[cfg(feature = "partition-checkpoint")]
    pub fn join_box_solvers(
        &self,
        donors: &[&Self],
    ) -> Result<(Self, gz_physics_rapier::NativeBoxMaps), PhysicsError> {
        self.join_box_solvers_with_profile(donors, NativeSolverProfile::Standard32)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn join_box_solvers_with_profile(
        &self,
        donors: &[&Self],
        profile: NativeSolverProfile,
    ) -> Result<(Self, gz_physics_rapier::NativeBoxMaps), PhysicsError> {
        let solvers = donors.iter().map(|d| &d.solver).collect::<Vec<_>>();
        let (solver, maps) = self
            .solver
            .join_box_solvers_with_profile(&solvers, profile)
            .map_err(engine_error)?;
        Ok((Self { solver }, maps))
    }

    /// Capture a complete worker solver, including prescribed members or a singleton.
    #[cfg(feature = "partition-checkpoint")]
    pub fn checkpoint_solver(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_solver()
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn checkpoint_solver_with_profile(
        &self,
        profile: NativeSolverProfile,
    ) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_solver_with_profile(profile)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn restore_solver_with_profile(
        bytes: &[u8],
        profile: NativeSolverProfile,
    ) -> Result<Self, PhysicsError> {
        Ok(Self {
            solver: RapierBackend::restore_solver_with_profile(bytes, profile)?,
        })
    }

    /// Restore a complete trusted worker solver from the same native build.
    #[cfg(feature = "partition-checkpoint")]
    pub fn restore_trusted_solver(bytes: &[u8]) -> Result<Self, PhysicsError> {
        Ok(Self {
            solver: RapierBackend::restore_trusted_solver(bytes)?,
        })
    }

    /// Restore the pinned native solver from a verified checkpoint from a trusted peer.
    #[cfg(feature = "partition-checkpoint")]
    pub fn restore_trusted_boxes(bytes: &[u8]) -> Result<Self, PhysicsError> {
        Ok(Self {
            solver: RapierBackend::restore_trusted_boxes(bytes)?,
        })
    }

    /// Count native manifolds and retain the exact bits of every per-point warm-start impulse.
    #[cfg(feature = "partition-checkpoint")]
    pub fn partition_warmstart(&self) -> (usize, Vec<u64>) {
        self.solver.partition_warmstart()
    }

    /// Number of bodies in this solver, for validating complete checkpoint bindings.
    #[cfg(feature = "partition-checkpoint")]
    pub fn partition_box_count(&self) -> usize {
        self.solver.partition_box_count()
    }

    /// Original full box sides and assigned construction mass of a partition body.
    #[cfg(feature = "partition-checkpoint")]
    pub fn partition_box(&self, id: BodyId) -> Option<([f64; 3], f64)> {
        self.solver.partition_box(id)
    }

    #[cfg(feature = "partition-checkpoint")]
    pub fn partition_shape(&self, id: BodyId) -> Option<(gz_physics::Shape, f64)> {
        self.solver.partition_shape(id)
    }

    /// Number of active collider pairs at the last contact solve.
    pub fn active_contact_pairs(&self) -> usize {
        self.solver.active_contact_pairs()
    }

    /// Attach offset primitive shapes to one body with explicit link inertia.
    pub fn spawn_compound_link(
        &mut self,
        parts: &[gz_physics::CollisionSpec],
        body: BodySpec,
        rotation: RotationSpec,
        inertia: Inertia,
        motion: MaterialLinkMotion,
        friction: Option<IsotropicFriction>,
    ) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_compound_link(parts, body, rotation, inertia, motion, friction)
            .map_err(engine_error)
    }

    /// Set the world-space linear tolerance for native geometry, including swept impacts.
    pub fn set_contact_linear_slop(&mut self, meters: f64) -> Result<(), PhysicsError> {
        self.solver
            .set_contact_linear_slop(meters)
            .map_err(engine_error)
    }

    /// Enable native swept collision for this independent dynamic link.
    pub fn enable_continuous_collision(&mut self, body: BodyId) -> Result<(), PhysicsError> {
        self.solver
            .enable_continuous_collision(body)
            .map_err(engine_error)
    }

    pub fn set_initial_restitution(
        &mut self,
        id: BodyId,
        value: Restitution,
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_initial_restitution(id, value)
            .map_err(engine_error)
    }

    /// Admit a primitive with an explicit isotropic contact material.
    pub fn spawn_material_link(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        inertia: Option<Inertia>,
        motion: MaterialLinkMotion,
        friction: IsotropicFriction,
    ) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_material_link(spec, rotation, inertia, motion, friction)
            .map_err(engine_error)
    }

    pub fn spawn_material_plane(
        &mut self,
        plane: gz_physics::PlaneSpec,
        friction: IsotropicFriction,
    ) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_material_plane(plane, friction)
            .map_err(engine_error)
    }

    /// Construct a member of a floating reduced-coordinate articulation.
    pub fn spawn_articulation_link(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_articulation_link(spec, rotation, inertia)
            .map_err(engine_error)
    }

    /// Add a rigid joint to a mixed local robot.
    pub fn set_robot_root_iterations(
        &mut self,
        root: BodyId,
        iterations: u8,
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_robot_root_iterations(root, iterations)
            .map_err(engine_error)
    }
    pub fn set_joint_angular_speed_limits(
        &mut self,
        id: JointId,
        limits: [Option<f64>; 2],
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_joint_angular_speed_limits(id, limits)
            .map_err(engine_error)
    }
    pub fn set_world_joint_speed_limits(
        &mut self,
        id: JointId,
        limits: [Option<f64>; 2],
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_world_joint_speed_limits(id, limits)
            .map_err(engine_error)
    }

    /// Install separate torsional compliance on a universal joint's two moving axes.
    pub fn add_universal_joint_with_springs(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        joint: RobotJoint,
        springs: [Option<RobotSpring>; 2],
    ) -> Result<JointId, PhysicsError> {
        self.solver
            .add_universal_joint_with_springs(parent, child, anchor, joint, springs)
            .map_err(engine_error)
    }

    pub fn add_robot_joint_with_spring(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        joint: RobotJoint,
        spring: RobotSpring,
    ) -> Result<JointId, PhysicsError> {
        self.solver
            .add_robot_joint_with_spring(parent, child, anchor, joint, spring)
            .map_err(engine_error)
    }
    pub fn add_robot_joint(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        joint: RobotJoint,
    ) -> Result<JointId, PhysicsError> {
        self.solver
            .add_robot_joint(parent, child, anchor, joint)
            .map_err(engine_error)
    }
    pub fn is_robot_link(&self, link: BodyId) -> bool {
        self.solver.is_robot_link(link)
    }
    /// Relative angles and angular rates about the two universal axes.
    pub fn universal_joint_state(&self, id: JointId) -> Option<UniversalJointState> {
        self.solver.universal_joint_state(id)
    }

    /// Validate the target and effort bound of a selected universal axis.
    pub fn validate_universal_motor(
        &self,
        id: JointId,
        axis: usize,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.solver
            .validate_universal_motor(id, axis, motor)
            .map_err(engine_error)
    }
    pub fn set_universal_motor(
        &mut self,
        id: JointId,
        axis: usize,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_universal_motor(id, axis, motor)
            .map_err(engine_error)
    }
    /// Accumulate one step of torque around each moving universal axis.
    pub fn add_universal_effort(
        &mut self,
        id: JointId,
        efforts: [f64; 2],
    ) -> Result<(), PhysicsError> {
        self.solver
            .add_universal_effort(id, efforts)
            .map_err(engine_error)
    }

    /// Sum of torques awaiting the next spherical-joint solve in parent joint axes.
    pub fn pending_ball_effort(&self, id: JointId) -> Option<[f64; 3]> {
        self.solver.pending_ball_effort(id)
    }

    /// Accumulate one step of spherical joint torque in the parent joint frame.
    pub fn add_ball_effort(&mut self, id: JointId, torque: [f64; 3]) -> Result<(), PhysicsError> {
        self.solver
            .add_ball_effort(id, torque)
            .map_err(engine_error)
    }

    /// Gravity force and moment of the downstream rigid tree in parent joint axes.
    pub fn joint_gravity_load(&self, id: JointId) -> Result<JointGravityLoad, PhysicsError> {
        self.solver.joint_gravity_load(id).map_err(engine_error)
    }

    /// Three-axis relative rotation and velocity for a spherical joint.
    pub fn ball_joint_state(&self, id: JointId) -> Option<BallJointState> {
        self.solver.ball_joint_state(id)
    }
    /// Construct a six-axis fixed joint between two native links, or to world.
    pub fn add_fixed_joint(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        self.solver
            .add_fixed_joint(parent, child, anchor)
            .map_err(engine_error)
    }
    /// Aggregate dynamics of one floating, fully welded native assembly.
    pub fn welded_assembly_state(&self, link: BodyId) -> Result<WeldedAssemblyState, PhysicsError> {
        self.solver
            .welded_assembly_state(link)
            .map_err(engine_error)
    }

    pub fn is_welded_link(&self, id: BodyId) -> bool {
        self.solver.is_welded_link(id)
    }

    /// Construct a revolute joint with finite, ordered position stops containing
    /// its initial angle of zero. Angles are unwrapped radians. The axis and
    /// anchor are expressed in the world frame.
    pub fn add_limited_revolute(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
        limits: [f64; 2],
    ) -> Result<JointId, PhysicsError> {
        self.solver
            .add_limited_revolute(parent, child, anchor, axis, limits)
            .map_err(engine_error)
    }

    /// Hold stationary zero-width revolutes at construction while retaining their IDs.
    pub fn enable_exact_revolute_locks(&mut self, root: BodyId) -> Result<(), PhysicsError> {
        self.solver
            .enable_exact_revolute_locks(root)
            .map_err(engine_error)
    }
    pub fn is_exact_revolute_lock(&self, id: JointId) -> Result<bool, PhysicsError> {
        self.solver.is_exact_revolute_lock(id).map_err(engine_error)
    }

    /// Preflight a requested demotion when the model has a contact guard.
    pub fn validate_prediction_motion(&self, root: BodyId) -> Result<(), PhysicsError> {
        self.solver
            .validate_prediction_motion(root)
            .map_err(engine_error)
    }

    /// Predict a contact with a named free dynamic, fixed or prescribed primitive during the next step.
    pub fn predicts_articulation_contact(
        &self,
        root: BodyId,
        obstacle: BodyId,
        dt: Duration,
    ) -> Result<bool, PhysicsError> {
        self.solver
            .predicts_articulation_contact(root, obstacle, dt)
            .map_err(engine_error)
    }

    pub fn articulation_link_count(&self, root: BodyId) -> Result<usize, PhysicsError> {
        self.solver
            .articulation_link_count(root)
            .map_err(engine_error)
    }

    pub fn articulation_contains(&self, root: BodyId, member: BodyId) -> bool {
        self.solver.articulation_contains(root, member)
    }

    pub fn articulation_tier(&self, root: BodyId) -> Result<ArticulationTier, PhysicsError> {
        self.solver.articulation_tier(root).map_err(engine_error)
    }

    /// Read-only demotion check at the current boundary using the same native
    /// preflight as a real transition. Subsequent steps can change its result.
    pub fn validate_articulation_demotion(
        &self,
        root: BodyId,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        self.solver
            .validate_articulation_demotion(root, dt)
            .map_err(engine_error)
    }

    /// Number of fully stationary registered two-pad trees measured on the latest solve.
    pub fn measured_parked_trees(&self) -> usize {
        self.solver.measured_parked_trees()
    }
    /// Check a stationary group under one geometric proof. A true flag asks
    /// to demote its root. False means this entire batch remained unchanged.
    pub fn apply_parked_demotions(
        &mut self,
        requests: &[(BodyId, bool)],
        dt: Duration,
    ) -> Result<bool, PhysicsError> {
        self.solver
            .apply_parked_demotions(requests, dt)
            .map_err(engine_error)
    }
    /// Preflight a demotion and sweep its prospective prescribed motion against any
    /// number of obstacles without changing tier, contact state, or body IDs.
    pub fn demotion_contact_prediction(
        &self,
        root: BodyId,
        dt: Duration,
    ) -> Result<impl Fn(BodyId) -> Result<bool, PhysicsError> + '_, PhysicsError> {
        let query = self
            .solver
            .demotion_contact_prediction(root, dt)
            .map_err(engine_error)?;
        Ok(move |obstacle| query(obstacle).map_err(engine_error))
    }

    /// Switch at a public simulation boundary, with `dt` set to the next public
    /// step duration. Poses, twists, joint coordinates and public IDs are retained.
    /// Demotion checks internal quiescence, input and actuator ownership, contacts,
    /// and either gravity-free coasting or measured, balanced support from fixed
    /// bodies or parked velocity boundaries. A parked support must have zero
    /// linear and angular velocity, a fresh stationary solve after stopping, and
    /// no active authored translation trajectory, including a dwell on that trajectory.
    /// Torque-free rotation requires a principal axis of the whole assembly.
    /// An incompatible physical state returns [`PhysicsError::TransitionDeferred`].
    /// Body wrenches queued during prescribed motion apply once after promotion.
    pub fn switch_articulation(
        &mut self,
        root: BodyId,
        target: ArticulationTier,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        self.solver
            .switch_articulation(root, target, dt)
            .map_err(engine_error)
    }

    /// Switch a contact-free passive tree to prescribed transport after proving
    /// that the current rigid twist requires no external or hinge torque.
    pub fn begin_articulation_free_coast(
        &mut self,
        root: BodyId,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        self.solver
            .begin_articulation_free_coast(root, dt)
            .map_err(engine_error)
    }

    /// Prescribe constant world-frame COM and angular velocities for a primitive.
    /// Gravity and contact impulses do not change this twist. An explicit
    /// velocity or body-state edit replaces it until reset restores the original.
    pub fn spawn_kinematic_link(
        &mut self,
        spec: BodySpec,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_velocity_boundary(spec, rotational, inertia)
    }
}

// Preserve the public backend identity in diagnostics from the shared solver.
fn engine_error(error: PhysicsError) -> PhysicsError {
    match error {
        PhysicsError::Unsupported { capability, .. } => PhysicsError::Unsupported {
            engine: ENGINE,
            capability,
        },
        PhysicsError::Native { message, .. } => PhysicsError::Native {
            engine: ENGINE,
            message,
        },
        PhysicsError::NonFiniteStep { .. } => PhysicsError::NonFiniteStep { engine: ENGINE },
        error => error,
    }
}

impl DynamicBackend for MultiFidelityBackend {
    fn engine(&self) -> &'static str {
        ENGINE
    }

    fn spawn_link(
        &mut self,
        spec: BodySpec,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_link(spec, rotational, inertia)
            .map_err(engine_error)
    }

    fn validate_kinematic_velocity(
        &self,
        linear: [f64; 3],
        angular: [f64; 3],
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        self.solver.validate_velocity_boundary(linear, angular, dt)
    }

    fn spawn_kinematic_link(
        &mut self,
        spec: BodySpec,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        Self::spawn_kinematic_link(self, spec, rotational, inertia)
    }

    fn spawn_static_plane(&mut self, plane: gz_physics::PlaneSpec) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_horizontal_halfspace(plane)
            .map_err(engine_error)
    }

    fn spawn_mesh_link(
        &mut self,
        spec: MeshBodySpec<'_>,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        self.solver
            .spawn_mesh_link(spec, rotational, inertia)
            .map_err(engine_error)
    }

    fn is_dynamic(&self, id: BodyId) -> Option<bool> {
        self.solver.is_dynamic(id)
    }

    fn is_kinematic(&self, id: BodyId) -> Option<bool> {
        self.solver.is_kinematic(id)
    }

    fn state(&self, id: BodyId) -> Option<BodyState> {
        self.solver.state(id)
    }

    fn read_states(&self, visit: &mut dyn FnMut(&mut dyn gz_physics::BodyStateLookup)) {
        self.solver.read_states(visit);
    }

    fn bodies_overlapping_aabb(
        &self,
        min: [f64; 3],
        max: [f64; 3],
    ) -> Result<Vec<BodyId>, PhysicsError> {
        self.solver
            .bodies_overlapping_aabb(min, max)
            .map_err(engine_error)
    }

    fn cast_ray(
        &self,
        origin: [f64; 3],
        direction: [f64; 3],
        max_distance: f64,
        exclude: Option<BodyId>,
    ) -> Result<Option<f64>, PhysicsError> {
        self.solver
            .cast_ray(origin, direction, max_distance, exclude)
            .map_err(engine_error)
    }

    fn remove(&mut self, id: BodyId) -> bool {
        self.solver.remove(id)
    }

    fn validate_body_state(&self, id: BodyId, state: BodyState) -> Result<(), PhysicsError> {
        self.solver
            .validate_body_state(id, state)
            .map_err(engine_error)
    }

    fn set_body_state(&mut self, id: BodyId, state: BodyState) -> Result<(), PhysicsError> {
        self.solver.set_body_state(id, state).map_err(engine_error)
    }

    fn set_world_velocity(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_world_velocity(id, linear, angular)
            .map_err(engine_error)
    }

    fn add_world_wrench(
        &mut self,
        id: BodyId,
        force: [f64; 3],
        torque: [f64; 3],
    ) -> Result<(), PhysicsError> {
        self.solver
            .add_world_wrench(id, force, torque)
            .map_err(engine_error)
    }

    fn apply_impulse(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        self.solver
            .apply_impulse(id, linear, angular)
            .map_err(engine_error)
    }

    fn set_self_collision_group(
        &mut self,
        id: BodyId,
        group: Option<u64>,
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_self_collision_group(id, group)
            .map_err(engine_error)
    }

    fn reset(&mut self) -> Result<(), PhysicsError> {
        self.solver.reset().map_err(engine_error)
    }

    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        self.solver.step(dt).map_err(engine_error)?;
        if !dt.is_zero() && self.solver.last_step_quarantined() {
            return Err(PhysicsError::NonFiniteStep { engine: ENGINE });
        }
        Ok(())
    }

    fn validate_joint_effort(&self, id: JointId, torque: f64) -> Result<(), PhysicsError> {
        self.solver
            .validate_joint_effort(id, torque)
            .map_err(engine_error)
    }

    fn add_joint_effort(&mut self, id: JointId, torque: f64) -> Result<(), PhysicsError> {
        self.solver
            .add_joint_effort(id, torque)
            .map_err(engine_error)
    }

    fn validate_joint_velocity_motor(
        &self,
        id: JointId,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.solver
            .validate_joint_velocity_motor(id, motor)
            .map_err(engine_error)
    }

    fn set_joint_velocity_motor(
        &mut self,
        id: JointId,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_joint_velocity_motor(id, motor)
            .map_err(engine_error)
    }

    fn validate_joint_velocity_servo(
        &self,
        id: JointId,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.solver
            .validate_joint_velocity_servo(id, motor)
            .map_err(engine_error)
    }

    fn set_joint_velocity_servo(
        &mut self,
        id: JointId,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_joint_velocity_servo(id, motor)
            .map_err(engine_error)
    }

    fn validate_joint_position_servo(
        &self,
        id: JointId,
        servo: Option<JointPositionServo>,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        self.solver
            .validate_joint_position_servo(id, servo, dt)
            .map_err(engine_error)
    }

    fn set_joint_position_servo(
        &mut self,
        id: JointId,
        servo: Option<JointPositionServo>,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        self.solver
            .set_joint_position_servo(id, servo, dt)
            .map_err(engine_error)
    }

    fn add_world_revolute(
        &mut self,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        self.solver
            .add_world_revolute(child, anchor, axis)
            .map_err(engine_error)
    }

    fn add_revolute(
        &mut self,
        parent: BodyId,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        self.solver
            .add_revolute(parent, child, anchor, axis)
            .map_err(engine_error)
    }

    fn add_reduced_revolute(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        self.solver
            .add_reduced_revolute(parent, child, anchor, axis)
            .map_err(engine_error)
    }

    fn joint_child(&self, id: JointId) -> Option<BodyId> {
        self.solver.joint_child(id)
    }

    fn joint_state(&self, id: JointId) -> Option<JointState> {
        self.solver.joint_state(id)
    }

    fn remove_joint(&mut self, id: JointId) -> bool {
        self.solver.remove_joint(id)
    }

    fn is_jointed(&self, id: BodyId) -> Option<bool> {
        self.solver.is_jointed(id)
    }

    fn initial_state(&self) -> InitialPhysicsState {
        self.solver.initial_state()
    }

    fn owns_initial_state(&self, initial: &InitialPhysicsState) -> bool {
        self.solver.owns_initial_state(initial)
    }

    fn prepare_initial(
        &self,
        initial: &InitialPhysicsState,
    ) -> Result<PreparedPhysicsReset, PhysicsError> {
        self.solver.prepare_initial(initial).map_err(engine_error)
    }

    fn apply_initial(&mut self, prepared: PreparedPhysicsReset) -> Result<(), PhysicsError> {
        self.solver.apply_initial(prepared).map_err(engine_error)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
