// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Atomic construction of a supported SDF world and its Bevy simulation systems.

mod apply_link_wrench;
pub mod articulated_contact;
mod articulation;
pub mod contact_promotion;
mod contact_promotion_sdf;
mod continuous_collision;
pub mod damped_sole;
pub mod deformable;
pub mod fidelity_zone;
mod free_rotation;
mod grasp_fleet_sdf;
pub mod grasp_guard;
pub mod sole_episodes;
pub mod support_loss_promotion;
pub use articulation::{
    ArticulationTier, FidelityError, articulation_root_pose, begin_model_free_coast,
    model_fidelity, set_model_fidelity,
};
mod contact_material;
mod joint_controller;
pub mod joint_posture;
mod joint_state_publisher;
mod odometry_publisher;
mod pose_publisher;
mod prescribed_motion;
pub mod primary_contact;
pub mod receiving_support;
mod scheduled_link_force;
#[cfg(feature = "distributed-physics")]
pub mod trajectory_workers;
pub use apply_link_wrench::{
    APPLY_LINK_WRENCH_CAPACITY, ApplyLinkWrenchConfiguration, ApplyLinkWrenchError,
    ApplyLinkWrenchMode, apply_link_wrench_active, clear_apply_link_wrench,
    discard_apply_link_wrenches, queue_apply_link_wrench,
};
pub use joint_controller::{
    JointControllerCommandError, JointControllerConfiguration, JointControllerMode,
    held_joint_controller_target, joint_controller_binding_matches,
    joint_controller_binding_matches_configuration, set_joint_controller_command,
    set_joint_position_response_period,
};
pub use joint_state_publisher::{
    JointStateJointPublication, JointStateKind, JointStatePublication, JointStateSample,
    JointStateValue,
};
pub use odometry_publisher::{
    GroundTruthOdometryDimensions, GroundTruthOdometryError, GroundTruthOdometryNoise,
    GroundTruthOdometryOffsets, GroundTruthOdometryPublication, GroundTruthOdometrySample,
    GroundTruthOdometryScheduleFault,
};
pub use pose_publisher::{
    PoseFramePublication, PoseFrameValue, PosePublication, PoseSample, StaticPosePublication,
    StaticPoseSample,
};
mod diff_drive;
#[cfg(feature = "distributed-physics")]
mod distributed;
#[cfg(feature = "distributed-physics")]
pub mod distributed_hinge;
#[cfg(feature = "distributed-physics")]
pub use distributed::step_with_placement;
#[cfg(feature = "distributed-physics")]
pub use distributed_hinge::{
    distributed_floating_tree_spec, distributed_grounded_tree_spec, distributed_hinge_spec,
    distributed_motor_tree_spec, distributed_moving_hinge_spec, distributed_payload_tree_spec,
    distributed_stopped_tree_spec,
};
#[cfg(feature = "distributed-physics")]
pub mod passive_checkpoint;
pub use diff_drive::{
    DiffDriveCommandError, DiffDriveConfiguration, DiffDriveSample, diff_drive_binding_matches,
    diff_drive_binding_matches_configuration, set_diff_drive_command,
};
mod joint_forces;
mod producer_guard;
mod world_systems;
pub use joint_forces::JointForceConfiguration;

mod joints;
mod local_replay;
mod noise;
pub use joints::{
    BallJointSample, JointDefinition, JointHandle, JointSample, UniversalJointSample,
};
pub use local_replay::load_local_replay_world;

mod inputs;
pub use inputs::{
    InputError, JointForceCommand, pending_free_link_wrench, queue_ball_joint_effort,
    queue_joint_effort, queue_link_force_at_point, queue_link_world_velocity,
    queue_link_world_wrench, queue_universal_joint_effort, set_joint_force,
};

mod schedule;
use schedule::{SampleSchedule, ScheduleError};
pub use schedule::{SampleSchedule as SensorSchedule, ScheduleError as SensorScheduleError};

mod rates;
pub use rates::{ImuRateError, ImuRateInfo, RateChange, imu_rate, set_imu_rate};

mod activation;
pub use activation::{ImuActivationError, imu_active, set_imu_active};

mod tpe;
pub use tpe::PhysicsEngine;

mod lifecycle;
pub use lifecycle::{RemoveError, remove_model};
mod world_edit;
pub use world_edit::{ModelState, WorldEditError, model_state, model_visual_pose, set_model_state};
mod world_spawn;
pub use world_spawn::{ModelTemplate, prepare_model_template, spawn_model_template};
mod imu_lifecycle;
mod imu_reference;
mod lidar;
pub use lidar::{LidarPublication, LidarSample, clear_lidar_interest, set_lidar_interest};
mod logical_camera;
pub use logical_camera::{
    LogicalCameraPublication, LogicalCameraSample, clear_logical_camera_interest,
    set_logical_camera_interest,
};
mod air_speed;
pub use air_speed::{
    AirSpeedPublication, AirSpeedSample, AirSpeedSampleError, clear_air_speed_interest,
    set_air_speed_interest,
};
mod air_pressure;
pub use air_pressure::{
    AirPressurePublication, AirPressureSample, AirPressureSampleError, clear_air_pressure_interest,
    set_air_pressure_interest,
};
mod altimeter;
pub use altimeter::{
    AltimeterPublication, AltimeterSample, AltimeterSampleError, clear_altimeter_interest,
    set_altimeter_interest,
};
mod magnetometer;
pub use magnetometer::{
    MagnetometerPublication, MagnetometerSample, MagnetometerSampleError,
    clear_magnetometer_interest, set_magnetometer_interest,
};
#[cfg(feature = "gpu-batch")]
pub mod gpu_scene;
mod initial;
mod joint_force_lifecycle;
mod joint_lifecycle;
pub use initial::{
    InitialDiffDriveIdentity, InitialJointControllerIdentity, PendingResetRemoval, SceneResetError,
    SceneResetInfo, initial_air_pressure_publications,
    initial_air_pressure_publications_with_identity, initial_air_speed_publications,
    initial_air_speed_publications_with_identity, initial_altimeter_publications,
    initial_altimeter_publications_with_identity, initial_apply_link_wrench_configuration,
    initial_diff_drive_configurations, initial_diff_drive_configurations_with_identity,
    initial_ground_truth_odometry_publications,
    initial_ground_truth_odometry_publications_with_identity, initial_imu_publications,
    initial_joint_controller_configurations, initial_joint_controller_configurations_with_identity,
    initial_joint_force_configurations, initial_joint_state_publications,
    initial_joint_state_publications_with_identity, initial_lidar_publications,
    initial_lidar_publications_with_identity, initial_logical_camera_identities,
    initial_logical_camera_publications, initial_magnetometer_publications,
    initial_magnetometer_publications_with_identity, initial_pose_publications,
    initial_pose_publications_with_identity, reset_loaded_world,
};

use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use gz_physics::{
    BodyId, BodySpec, CollisionSpec, Inertia, MeshBodySpec, PhysicsBackend, PhysicsError,
    PhysicsReset, PlaneSpec, RotationSpec, Shape,
};
use gz_physics_multifidelity::{IsotropicFriction, MaterialLinkMotion, MultiFidelityBackend};
use gz_physics_rapier::RapierBackend;
use gz_sensors::{ImuInput, ImuReading, InvalidImuInput};
use gz_sim_core::{
    EntityId, EntityLookup, IdentityError, Phase, Simulation, StepError, StepFault, UpdateInfo,
    assign_id,
};
use sdformat::{Geometry, Model, ResourceResolver, Sdf, Transform};
use std::{collections::BTreeMap, sync::Arc, time::Duration};

// Semantic frame composition introduces rounding even for coincident frames.
const POSE_TOLERANCE: f64 = 1e-12;

fn centered(translation: [f64; 3]) -> bool {
    translation.into_iter().all(|v| v.abs() <= POSE_TOLERANCE)
}

fn identity(pose: Transform) -> bool {
    centered(pose.translation)
        && centered([pose.rotation[0], pose.rotation[1], pose.rotation[2]])
        && (pose.rotation[3].abs() - 1.0).abs() <= POSE_TOLERANCE
}

#[derive(Component, Debug)]
pub struct Name(pub String);

/// The ownership hierarchy uses Bevy ChildOf/Children, separately from SDF frames.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityKind {
    World,
    Model,
    Link,
    Collision,
    Visual,
    Sensor,
    Joint,
    Light,
    /// Native Light child identity; it has no renderable geometry.
    LightVisual,
}

/// Loaded profiles and the caller's explicit step override. Exact stepping is unpaced.
/// Editing this metadata does not reconfigure the running backend or clock.
#[derive(Resource, Debug, Clone)]
pub struct WorldPhysics {
    pub profiles: Vec<sdformat::Physics>,
    pub step_override: Option<Duration>,
}

impl WorldPhysics {
    /// Gazebo's runner uses the first declaration, independently of the DOM default flag.
    pub fn selected(&self) -> Option<&sdformat::Physics> {
        self.profiles.first()
    }
}

/// Owned visual/lidar metadata. Rendering and lidar effects are separate systems.
#[derive(Component, Debug, Clone)]
pub struct VisualProperties(pub sdformat::VisualProperties);

/// Owned material data; rendering and resource loading are separate systems.
#[derive(Component, Debug, Clone)]
pub struct VisualMaterial(pub Box<sdformat::Material>);

/// Explicit collision density in kg/m³; absence means omitted source metadata.
/// Editing it does not recompute inertia or reconfigure the installed backend.
#[derive(Component, Debug, Clone, Copy, PartialEq)]
pub struct CollisionDensity(pub f64);

/// Complete owned world-light construction metadata, including raw pose and source.
/// Editing it does not activate rendering or issue native light commands.
#[derive(Component, Debug, Clone, PartialEq)]
pub struct WorldLight(pub Box<sdformat::Light>);

/// Parsed visual geometry, shared with the reset blueprint; not a collision shape.
/// Editing this metadata does not reconfigure physics or load rendering assets.
#[derive(Component, Debug, Clone)]
pub struct VisualGeometry(pub Arc<Geometry>);

/// Complete parsed camera sensor metadata, shared with the reset blueprint.
/// This component does not activate sampling, rendering, or publication.
#[derive(Component, Debug, Clone)]
pub struct CameraSensor(pub Arc<sdformat::Sensor>);

/// Recognized bare world Sensors intent. Metadata alone installs no callbacks,
/// rendering resources or workers; an explicit camera application consumes it.
#[derive(Component, Clone, Copy, Debug)]
#[component(immutable)]
pub struct RenderingSensorsConfiguration;

/// User visibility bits supported by bounded RGB/depth snapshots, including the
/// original all-visible default. Backend-reserved high bits remain unsupported.
pub fn supports_camera_visibility_bits(value: u32) -> bool {
    value == u32::MAX || value & !0x0FFF_FFFF == 0
}

/// Whether RGB/depth metadata is in the bounded 64×64 calibrated camera domain.
/// This admits metadata only and does not schedule or publish sensor samples.
pub fn supports_camera_sensor(sensor: &sdformat::Sensor) -> bool {
    let Some(camera) = sensor.camera() else {
        return false;
    };
    matches!(
        sensor.effective_type(),
        Some(sdformat::SensorType::Camera | sdformat::SensorType::DepthCamera)
    ) && !sensor.visualize
        && !sensor.enable_metrics
        && sensor.plugins.is_empty()
        && sensor.extensions.is_empty()
        && camera.image_width == 64
        && camera.image_height == 64
        && camera.pixel_format == sdformat::CameraPixelFormat::RgbInt8
        && camera.anti_aliasing == 4
        && !camera.triggered
        && !camera.save_frames
        && camera.image_noise == sdformat::Noise::default()
        && camera.distortion == sdformat::CameraDistortion::default()
        && camera.pose == sdformat::Pose::default()
        && supports_camera_visibility_bits(camera.visibility_mask)
        && !camera.has_depth_near_clip
        && !camera.has_depth_far_clip
        && !camera.has_segmentation_type
        && !camera.has_bounding_box_type
        && (!camera.lens.has_intrinsics || {
            let k = &camera.lens.intrinsics;
            [k.fx, k.fy, k.cx, k.cy, k.skew]
                .iter()
                .all(|value| value.is_finite())
                && k.fx > 0.0
                && k.fy > 0.0
        })
        && (!camera.lens.has_projection || {
            let p = &camera.lens.projection;
            [p.fx, p.fy, p.cx, p.cy, p.tx, p.ty]
                .iter()
                .all(|value| value.is_finite())
                && p.fx > 0.0
                && p.fy > 0.0
        })
        && camera.horizontal_fov.is_finite()
        && (0.001..std::f64::consts::PI).contains(&camera.horizontal_fov)
        && camera.near_clip.is_finite()
        && camera.far_clip.is_finite()
        && camera.near_clip > 0.0
        && camera.far_clip > camera.near_clip
}

/// Construction metadata. Editing it does not reconfigure an installed backend.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkProperties(pub sdformat::LinkProperties);

#[derive(Component, Debug)]
pub struct LocalName(pub String);

#[derive(Component, Debug, Clone, Copy)]
pub struct LocalPose(pub WorldPose);

#[derive(Component, Debug, Clone, Copy)]
pub struct WorldPose {
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}

impl From<Transform> for WorldPose {
    fn from(value: Transform) -> Self {
        Self {
            translation: value.translation,
            rotation: value.rotation,
        }
    }
}

impl WorldPose {
    fn compose(self, child: Self) -> Self {
        // Translating model, link and visual frames are common in large fleets.
        // Their composed translation requires no quaternion arithmetic.
        if self.rotation == [0., 0., 0., 1.] && child.rotation == [0., 0., 0., 1.] {
            return Self {
                translation: (DVec3::from_array(self.translation)
                    + DVec3::from_array(child.translation))
                .to_array(),
                rotation: [0., 0., 0., 1.],
            };
        }
        let rotation = DQuat::from_array(self.rotation);
        let child_rotation = DQuat::from_array(child.rotation);
        let mut composed = rotation * child_rotation;
        // Two valid near-unit rotations can yield an out-of-budget derived
        // frame, e.g. relative link motion on resuming an older native cut.
        // Preserve exact native body observations and ordinary products.
        if (composed.to_array().iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-12
            && (rotation.length_squared() - 1.).abs() <= 1e-12
            && (child_rotation.length_squared() - 1.).abs() <= 1e-12
        {
            composed = composed.normalize();
        }
        Self {
            translation: (DVec3::from_array(self.translation)
                + rotation * DVec3::from_array(child.translation))
            .to_array(),
            rotation: composed.to_array(),
        }
    }

    fn inverse(self) -> Self {
        let rotation = DQuat::from_array(self.rotation).conjugate();
        Self {
            translation: (rotation * -DVec3::from_array(self.translation)).to_array(),
            rotation: rotation.to_array(),
        }
    }
}

#[derive(Component, Clone)]
struct ModelAnchor {
    link: u64,
    link_to_model: WorldPose,
}

#[derive(Component)]
struct AttachedPose;

/// World-axis velocity at the link origin for dynamic backends.
/// TPE retains its model-motion proxy.
#[derive(Component, Debug, Clone, Copy)]
pub struct Velocity {
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}

#[derive(Component, Debug, Clone, Copy)]
struct LinkCenterOfMass([f64; 3]);

impl LinkCenterOfMass {
    fn velocity_shift(self, orientation: [f64; 4], angular: [f64; 3]) -> DVec3 {
        let arm = DQuat::from_array(orientation) * DVec3::from_array(self.0);
        DVec3::from_array(angular).cross(arm)
    }
}

#[derive(Component)]
pub struct BodyHandle(BodyId);

impl BodyHandle {
    pub fn id(&self) -> BodyId {
        self.0
    }
}

#[derive(Component, Clone)]
struct Imu {
    link: u64,
    rotation: DQuat,
    reference: DQuat,
    initial_velocity: DVec3,
    previous_velocity: DVec3,
    schedule: SampleSchedule,
    sdf_rate_hz: f64,
    rate_hz: f64,
    active: bool,
    noise: Option<Box<noise::ImuNoise>>,
}

#[derive(Component, Debug)]
pub struct ImuSample {
    pub time: Duration,
    pub epoch: u64,
    pub reading: Result<ImuReading, InvalidImuInput>,
}

#[derive(Component, Debug, Clone)]
pub struct ImuPublication {
    /// Include the orientation field; covariances and motion channels remain present.
    pub enable_orientation: bool,
    pub entity_name: String,
    pub frame_id: String,
    /// Requested topic; the transport boundary applies Gazebo name repair.
    pub topic: String,
}

struct SceneContext<'a> {
    gpu_tool_scene: bool,
    sole_observers: bool,
    world: &'a str,
    engine: PhysicsEngine,
    spherical_coordinates: Option<sdformat::SphericalCoordinates>,
    links: BTreeMap<String, u64>,
    visuals: Vec<Entity>,
    cameras: Vec<Entity>,
    named_imu_streams: bool,
    noise_seed: [u8; 32],
    field_world: [f64; 3],
    wind_world: [f64; 3],
}

struct ParentModel {
    entity: Entity,
    is_static: bool,
    topic: String,
}

#[derive(Resource, Default)]
struct ImuInterest(Vec<u64>);

/// Selects transport interest for this world's loaded IMUs. Any due connected
/// sensor enables all due sensors, matching native Imu::PostUpdate.
/// Handles are scoped to this simulation; despawned or identity-less handles
/// contribute no interest. Its public IDs preserve the selection across reset.
pub fn set_imu_interest(sim: &mut Simulation, connected: impl IntoIterator<Item = Entity>) {
    let world = sim.world_mut();
    if !world.contains_resource::<imu_lifecycle::Installed>() {
        world.remove_resource::<ImuInterest>();
        return;
    }
    world.init_resource::<ImuInterest>();
    world.resource_scope(|world, mut interest: Mut<ImuInterest>| {
        interest.0.clear();
        let connected = connected.into_iter();
        if let Some(limit) = connected.size_hint().1 {
            interest.0.reserve(limit);
        }
        interest.0.extend(
            connected.filter_map(|entity| world.get::<EntityId>(entity).map(EntityId::get)),
        );
        interest.0.sort_unstable();
        interest.0.dedup();
    });
}

/// Restores local acquisition independent of transport subscribers.
pub fn clear_imu_interest(sim: &mut Simulation) {
    sim.world_mut().remove_resource::<ImuInterest>();
}

#[derive(Resource)]
struct Gravity([f64; 3]);

#[derive(Debug, thiserror::Error)]
pub enum LoadError {
    #[error("translation trajectory for {entity}: {reason}")]
    TranslationPath { entity: String, reason: String },
    #[error("{0}")]
    Sdf(#[from] sdformat::Diagnostic),
    #[error("world {0} was not found")]
    MissingWorld(String),
    #[error("{entity}: unsupported {feature}")]
    Unsupported {
        entity: String,
        feature: &'static str,
    },
    #[error("{entity}: mesh collision: {source}")]
    Mesh {
        entity: String,
        source: gz_assets::mesh::MeshError,
    },
    #[error("{entity}: joint posture configuration: {reason}")]
    JointPosture { entity: String, reason: String },
    #[error("scheduled link force on {entity}: {reason}")]
    ScheduledLinkForce { entity: String, reason: String },
    #[error("{entity}: contact promotion configuration: {reason}")]
    ContactPromotion { entity: String, reason: String },
    #[error("{entity}: primary contact refinement configuration: {reason}")]
    PrimaryContactRefinement { entity: String, reason: String },
    #[error("{world}: damped sole configuration: {reason}")]
    DampedSole { world: String, reason: String },
    #[error("{world}: deformable configuration: {reason}")]
    Deformable { world: String, reason: String },
    #[cfg(feature = "gpu-batch")]
    #[error("{entity}: GPU batch configuration: {reason}")]
    GpuScene { entity: String, reason: String },
    #[cfg(feature = "gpu-batch")]
    #[error(transparent)]
    Gpu(#[from] gz_physics_gpu::GpuError),
    #[error(transparent)]
    Physics(#[from] PhysicsError),
    #[error(transparent)]
    Clock(#[from] StepError),
    #[error(transparent)]
    Identity(#[from] IdentityError),
    #[error(transparent)]
    Tpe(#[from] gz_physics_tpe::MotionError),
    #[error(transparent)]
    TpeInstall(#[from] gz_physics_tpe::InstallError),
    #[error(transparent)]
    TpeContact(#[from] gz_physics_tpe::ContactError),
}

fn require(supported: bool, entity: &str, feature: &'static str) -> Result<(), LoadError> {
    if supported {
        Ok(())
    } else {
        Err(LoadError::Unsupported {
            entity: entity.into(),
            feature,
        })
    }
}

/// Creates a fresh world or returns an error without changing any existing simulation.
/// Loads one centered sphere, box or Z-axis cylinder per link.
/// Capability guards reject unsupported geometry and joint configurations.
pub fn load_world(
    xml: &str,
    source: &str,
    world_name: &str,
    step: Duration,
    resolver: &dyn ResourceResolver,
) -> Result<Simulation, LoadError> {
    load_world_with_engine(
        xml,
        source,
        world_name,
        step,
        resolver,
        PhysicsEngine::Rapier,
    )
}

/// Selects one physics engine at construction; see each backend's supported scope.
pub fn load_world_with_engine(
    xml: &str,
    source: &str,
    world_name: &str,
    step: Duration,
    resolver: &dyn ResourceResolver,
    engine: PhysicsEngine,
) -> Result<Simulation, LoadError> {
    load_world_seeded(
        xml,
        source,
        world_name,
        Some(step),
        resolver,
        engine,
        [0; 32],
    )
}

/// Uses the first SDF physics profile's step without adding wall-clock pacing.
/// Nondefault contact limits require a backend capability not yet implemented.
pub fn load_world_with_sdf_step(
    xml: &str,
    source: &str,
    world_name: &str,
    resolver: &dyn ResourceResolver,
    engine: PhysicsEngine,
) -> Result<Simulation, LoadError> {
    load_world_seeded(xml, source, world_name, None, resolver, engine, [0; 32])
}

/// Uses independent deterministic streams keyed by public sensor identity.
/// Reset repeats each initial stream; omitted step uses the first SDF profile.
/// Existing loader entry points use an all-zero noise seed.
pub fn load_world_seeded(
    xml: &str,
    source: &str,
    world_name: &str,
    step: Option<Duration>,
    resolver: &dyn ResourceResolver,
    engine: PhysicsEngine,
    seed: [u8; 32],
) -> Result<Simulation, LoadError> {
    load_world_configured(
        xml,
        source,
        world_name,
        resolver,
        WorldLoadOptions {
            step,
            engine,
            noise_seed: seed,
            distributed_worker: None,
        },
    )
}

/// Runtime settings supplied by the application. SDF cannot select an executable.
#[derive(Clone, Copy, Debug)]
pub struct WorldLoadOptions<'a> {
    pub engine: PhysicsEngine,
    /// An omitted override uses the first SDF physics profile.
    pub step: Option<Duration>,
    pub noise_seed: [u8; 32],
    /// Trusted local physics worker. Distributed and resident GPU worlds default to the sibling
    /// executables `physics_worker` and `gz_pair_worker`, respectively.
    pub distributed_worker: Option<&'a std::path::Path>,
}

/// Loads the complete SDF scene and initializes its selected backend before returning.
pub fn load_world_configured(
    xml: &str,
    source: &str,
    world_name: &str,
    resolver: &dyn ResourceResolver,
    options: WorldLoadOptions<'_>,
) -> Result<Simulation, LoadError> {
    load_world_on_adapter(xml, source, world_name, resolver, options, None)
}
/// Load the same SDF world with an explicitly selected hardware adapter for GPU batches.
/// CPU engines reject any GPU selector.
pub fn load_world_on_adapter(
    xml: &str,
    source: &str,
    world_name: &str,
    resolver: &dyn ResourceResolver,
    options: WorldLoadOptions<'_>,
    gpu_adapter: Option<&str>,
) -> Result<Simulation, LoadError> {
    load_world_with_runtime(
        xml,
        source,
        world_name,
        resolver,
        options,
        WorldRuntimeOptions {
            gpu_adapter,
            ..Default::default()
        },
    )
}

#[cfg(feature = "distributed-physics")]
pub use gz_physics_distributed::pool::PeriodicPlacement;

/// Application-selected execution policies. Omitted policies retain the engine defaults.
#[derive(Clone, Copy, Debug, Default)]
pub struct WorldRuntimeOptions<'a> {
    /// Select one Coulomb constraint per rigid contact point in local Rapier, local multi-fidelity or a three- to eight-worker pool.
    pub rapier_per_point_friction: bool,
    /// Select packed warm handoff payloads on supporting distributed workers.
    pub distributed_packed_handoffs: bool,
    /// Optional explicit adapter for GPU batches.
    pub gpu_adapter: Option<&'a str>,
    /// Keep prospective contact groups on one owner during distributed cold start and reset.
    pub distributed_contact_placement: bool,
    /// Negotiate a 64-member native solver profile on all three to eight pool workers.
    pub distributed_native_64: bool,
    /// Construct one native solver in ascending public-ID order on cold start and reset.
    pub distributed_canonical_reference: bool,
    /// Maximum worker-local native solver helpers granted per distributed barrier.
    pub distributed_local_helper_limit: Option<usize>,
    /// Wall-clock measurements can move one independent native island inside a completed tick.
    #[cfg(feature = "distributed-physics")]
    pub distributed_periodic_placement: Option<PeriodicPlacement>,
}

/// Load an SDF scene with application-selected execution policies.
pub fn load_world_with_runtime(
    xml: &str,
    source: &str,
    world_name: &str,
    resolver: &dyn ResourceResolver,
    options: WorldLoadOptions<'_>,
    runtime: WorldRuntimeOptions<'_>,
) -> Result<Simulation, LoadError> {
    load_world_at_clock(xml, source, world_name, resolver, options, runtime, None)
}

fn load_world_at_clock(
    xml: &str,
    source: &str,
    world_name: &str,
    resolver: &dyn ResourceResolver,
    options: WorldLoadOptions<'_>,
    runtime: WorldRuntimeOptions<'_>,
    clock: Option<(UpdateInfo, bool)>,
) -> Result<Simulation, LoadError> {
    load_world_with_automatic_owner(
        xml,
        source,
        world_name,
        resolver,
        options,
        runtime,
        clock,
        false,
        #[cfg(feature = "distributed-physics")]
        None,
    )
}
#[allow(clippy::too_many_arguments)]
fn load_world_with_automatic_owner(
    xml: &str,
    source: &str,
    world_name: &str,
    resolver: &dyn ResourceResolver,
    options: WorldLoadOptions<'_>,
    runtime: WorldRuntimeOptions<'_>,
    clock: Option<(UpdateInfo, bool)>,
    named_imu_streams: bool,
    #[cfg(feature = "distributed-physics")] managed_material: Option<
        deformable::managed_impact::MaterialOwner,
    >,
) -> Result<Simulation, LoadError> {
    require(
        !runtime.rapier_per_point_friction || options.engine.supports_per_point_friction(),
        world_name,
        "per-point Coulomb friction requires local Rapier, local multi-fidelity or three to eight distributed workers",
    )?;
    let gpu_adapter = runtime.gpu_adapter;
    #[cfg(feature = "distributed-physics")]
    if let Some(config) = runtime.distributed_periodic_placement {
        require(
            config.validate().is_ok()
                && !runtime.distributed_canonical_reference
                && matches!(
                    options.engine,
                    PhysicsEngine::DistributedPool(3..=8)
                        | PhysicsEngine::DistributedAdaptive(3..=8)
                        | PhysicsEngine::DistributedIslands(3..=8)
                ),
            world_name,
            "periodic placement requires a valid window on an ordinary three- to eight-worker pool",
        )?;
    }
    require(
        !runtime.distributed_native_64
            || matches!(
                options.engine,
                PhysicsEngine::DistributedPool(3..=8)
                    | PhysicsEngine::DistributedAdaptive(3..=8)
                    | PhysicsEngine::DistributedIslands(3..=8)
            ),
        world_name,
        "extended native capacity requires three to eight distributed workers",
    )?;
    require(
        !runtime.distributed_canonical_reference
            || (runtime.distributed_native_64
                && !runtime.distributed_contact_placement
                && matches!(
                    options.engine,
                    PhysicsEngine::DistributedPool(3..=8)
                        | PhysicsEngine::DistributedAdaptive(3..=8)
                )),
        world_name,
        "canonical reference placement requires extended native capacity and uniform distributed stepping",
    )?;
    require(
        !runtime.distributed_packed_handoffs || options.engine.is_distributed(),
        world_name,
        "packed handoffs require the distributed engine",
    )?;
    require(
        !runtime.distributed_contact_placement || options.engine.is_distributed(),
        world_name,
        "contact placement requires the distributed engine",
    )?;
    require(
        runtime.distributed_local_helper_limit.is_none() || options.engine.is_distributed(),
        world_name,
        "a physics worker helper limit requires the distributed engine",
    )?;
    let WorldLoadOptions {
        step: step_override,
        engine,
        noise_seed,
        distributed_worker,
    } = options;
    require(
        distributed_worker.is_none()
            || engine.is_distributed()
            || engine == PhysicsEngine::GpuBatch
            || (cfg!(feature = "distributed-physics") && engine == PhysicsEngine::MultiFidelity),
        world_name,
        "explicit physics worker requires a distributed, cubic multifidelity or resident GPU world",
    )?;
    require(
        gpu_adapter.is_none() || engine == PhysicsEngine::GpuBatch,
        world_name,
        "GPU adapter requires the gpu-batch engine",
    )?;
    #[cfg(not(feature = "gpu-batch"))]
    require(
        engine != PhysicsEngine::GpuBatch,
        world_name,
        "gpu-batch engine requires the gpu-batch Cargo feature",
    )?;
    let mut sdf = sdformat::load_str(xml, source, resolver)?;
    sdf.validate_non_frame_poses()?;
    let world_index = sdf
        .worlds
        .iter()
        .position(|world| world.name == world_name)
        .ok_or_else(|| LoadError::MissingWorld(world_name.into()))?;
    let world = &sdf.worlds[world_index];
    if named_imu_streams {
        local_replay::validate(world)?;
    }
    require(
        engine == PhysicsEngine::GpuBatch
            || !world.plugins.iter().any(|p| {
                p.name == "gz::sim::systems::GpuBatch" || p.filename == "gz-sim-gpu-batch-system"
            }),
        world_name,
        "GpuBatch plugin requires the gpu-batch engine",
    )?;
    let magnetic_consumer = magnetometer::preflight(&sdf, world, engine)?;
    let air_speed_consumer = air_speed::preflight(&sdf, world, engine)?;
    logical_camera::preflight(&sdf, world, engine)?;
    producer_guard::validate_for_engine(
        world,
        magnetic_consumer,
        air_speed_consumer,
        engine == PhysicsEngine::MultiFidelity,
    )?;
    let contact_policy = contact_promotion_sdf::parse(world, engine)?;
    let primary_contact_plan = primary_contact::sdf::parse(&sdf, world, engine)?;
    let sole_plan = damped_sole::parse(&sdf, world, engine)?;
    let joint_postures = joint_posture::parse(&sdf, world, engine)?;
    let force_schedule = scheduled_link_force::parse(world, engine)?;
    let wrench_plan = apply_link_wrench::parse(world)?;
    if let Some(plan) = &wrench_plan {
        plan.validate(world, engine)?;
    }
    let rendering_sensors = world_systems::validate(world)?;
    // The pinned parser supplies an implicit profile when none is declared.
    let profile = world
        .physics
        .first()
        .expect("SDF supplies a physics profile");
    let profile_name = format!("{world_name}::physics::{}", profile.name);
    require(
        profile.max_contacts == 20,
        &profile_name,
        "physics maximum contacts",
    )?;
    let step = match step_override {
        Some(step) => step,
        None => {
            let nanoseconds = profile.max_step_size * 1_000_000_000.0;
            require(
                nanoseconds.is_finite() && nanoseconds >= 1.0 && nanoseconds < i64::MAX as f64,
                &profile_name,
                "unrepresentable physics step",
            )?;
            // Preserve a step authored as an exact integral nanosecond. Its
            // correctly rounded seconds can multiply back to 59.99999999999999
            // for 60 ns. Keep truncation for representable fractional inputs.
            let integral = nanoseconds.round();
            let clock_ns = if profile.max_step_size == integral / 1_000_000_000.0 {
                integral
            } else {
                nanoseconds
            };
            Duration::from_nanos(clock_ns as u64)
        }
    };
    #[cfg(feature = "gpu-batch")]
    let gpu_plan = if engine == PhysicsEngine::GpuBatch {
        let plan = gpu_scene::LoadedPlan::load(&sdf, world, step, gpu_adapter)?;
        require(
            distributed_worker.is_none() || plan.uses_processes(),
            world_name,
            "explicit GPU worker requires resident_block workers",
        )?;
        Some(plan)
    } else {
        None
    };
    let mut sim = match clock {
        Some((info, paused)) => Simulation::from_committed_clock(step, info, paused)?,
        None => Simulation::new(step)?,
    };
    sim.world_mut().insert_resource(WorldPhysics {
        profiles: world.physics.clone(),
        step_override,
    });
    let mut backend = match engine {
        PhysicsEngine::Rapier => Some(
            if runtime.rapier_per_point_friction {
                RapierBackend::new_per_point_coulomb(world.gravity)?
            } else {
                RapierBackend::new(world.gravity)?
            }
            .into(),
        ),
        #[cfg(not(feature = "distributed-physics"))]
        PhysicsEngine::Distributed
        | PhysicsEngine::DistributedPool(_)
        | PhysicsEngine::DistributedAdaptive(_)
        | PhysicsEngine::DistributedIslands(_) => {
            return Err(LoadError::Unsupported {
                entity: world_name.to_owned(),
                feature: "distributed physics requires the distributed-physics Cargo feature",
            });
        }
        #[cfg(feature = "distributed-physics")]
        PhysicsEngine::Distributed
        | PhysicsEngine::DistributedPool(_)
        | PhysicsEngine::DistributedAdaptive(_)
        | PhysicsEngine::DistributedIslands(_) => {
            let width = match engine {
                PhysicsEngine::Distributed => 2,
                PhysicsEngine::DistributedPool(n @ 3..=8)
                | PhysicsEngine::DistributedAdaptive(n @ 2..=8)
                | PhysicsEngine::DistributedIslands(n @ 2..=8) => n,
                PhysicsEngine::DistributedAdaptive(_) | PhysicsEngine::DistributedIslands(_) => {
                    return Err(LoadError::Unsupported {
                        entity: world_name.into(),
                        feature: "adaptive distributed worker count must be 2 through 8",
                    });
                }
                _ => {
                    return Err(LoadError::Unsupported {
                        entity: world_name.into(),
                        feature: "worker pool width must be 3 through 8",
                    });
                }
            };
            let backend = distributed::new_backend(&sdf, world, step, distributed_worker)?
                .with_worker_count(width)?;
            use gz_physics_distributed::physics::{NativeFrictionModel, NativeSolverProfile};
            let capacity = if runtime.distributed_native_64 {
                NativeSolverProfile::Extended64
            } else {
                NativeSolverProfile::Standard32
            };
            let profile = if runtime.rapier_per_point_friction {
                capacity.with_friction(NativeFrictionModel::PerPoint)
            } else {
                capacity
            };
            let backend = backend.with_native_solver_profile(profile)?;
            let backend = if runtime.distributed_canonical_reference {
                backend.with_canonical_reference()?
            } else if runtime.distributed_contact_placement {
                backend.with_contact_placement()?
            } else {
                backend
            };
            let backend = match engine {
                PhysicsEngine::DistributedAdaptive(_) => backend.with_adaptive_contacts()?,
                PhysicsEngine::DistributedIslands(_) => backend.with_island_refinement()?,
                _ => backend,
            };
            let mut backend = backend;
            backend.set_packed_handoffs(runtime.distributed_packed_handoffs);
            backend.set_local_helper_limit(runtime.distributed_local_helper_limit);
            backend.set_periodic_placement(runtime.distributed_periodic_placement)?;
            Some(backend.into())
        }
        PhysicsEngine::Tpe | PhysicsEngine::GpuBatch => None,
        PhysicsEngine::MultiFidelity => Some(
            if runtime.rapier_per_point_friction {
                MultiFidelityBackend::new_per_point_coulomb(world.gravity)?
            } else {
                MultiFidelityBackend::new(world.gravity)?
            }
            .into(),
        ),
    };
    #[cfg(feature = "gpu-batch")]
    let gpu_tool_scene = matches!(
        gpu_plan,
        Some(gpu_scene::LoadedPlan::Tool(_) | gpu_scene::LoadedPlan::Gripper(_))
    );
    #[cfg(not(feature = "gpu-batch"))]
    let gpu_tool_scene = false;
    let mut context = SceneContext {
        gpu_tool_scene,
        sole_observers: sole_plan.is_some(),
        world: world_name,
        engine,
        spherical_coordinates: world.spherical_coordinates.clone(),
        links: BTreeMap::new(),
        visuals: Vec::new(),
        cameras: Vec::new(),
        named_imu_streams,
        noise_seed,
        field_world: world.magnetic_field,
        wind_world: world.wind_linear_velocity,
    };
    let world_entity = spawn_node(
        &mut sim,
        None,
        world_name,
        EntityKind::World,
        Transform::default(),
        Transform::default(),
    )?;
    if rendering_sensors {
        sim.world_mut()
            .entity_mut(world_entity)
            .insert(RenderingSensorsConfiguration);
    }
    let parent = ParentModel {
        entity: world_entity,
        is_static: false,
        topic: format!("/world/{world_name}"),
    };
    for model in &world.models {
        build_model(&sdf, model, &mut context, &parent, &mut sim, &mut backend)?;
    }
    if articulation::moving_contact_profile(world)
        && let Some(native) = backend
            .as_mut()
            .and_then(|b| b.downcast_mut::<MultiFidelityBackend>())
    {
        if world.models.len() == 2 {
            for body in sim
                .world()
                .iter_entities()
                .filter_map(|e| e.get::<BodyHandle>())
            {
                native.configure_moving_body(body.id())?;
            }
        } else {
            #[cfg(feature = "distributed-physics")]
            match native.configure_pair_revolute() {
                Ok(()) | Err(PhysicsError::Unsupported { .. }) => (),
                Err(error) => return Err(error.into()),
            }
        }
    }
    #[cfg(feature = "distributed-physics")]
    grasp_fleet_sdf::install_observer(&sim, backend.as_mut(), world)?;
    continuous_collision::install(&sim, world, &context.links, backend.as_mut())?;
    free_rotation::install(&sim, world, &context.links, backend.as_mut())?;
    deformable::install(&mut sim, world, &context.links, backend.as_ref())?;
    let parent = Some(world_entity);
    for light in &world.lights {
        let pose = sdf.resolve_pose(&light.pose, world_name, world_name, world_name)?;
        let name = format!("{world_name}::{}", light.name);
        let entity = spawn_node(&mut sim, parent, &name, EntityKind::Light, pose, pose)?;
        sim.world_mut()
            .entity_mut(entity)
            .insert(WorldLight(Box::new(light.clone())));
        let visual_name = format!("{name}::{}Visual", light.name);
        let kind = EntityKind::LightVisual;
        let local = Transform::default();
        let visual = spawn_node(&mut sim, Some(entity), &visual_name, kind, local, pose)?;
        sim.world_mut().entity_mut(visual).insert((
            AttachedPose,
            VisualProperties(sdformat::VisualProperties {
                cast_shadows: false,
                transparency: 0.0,
                ..Default::default()
            }),
        ));
    }
    sim.world_mut().insert_resource(Gravity(world.gravity));
    #[cfg(feature = "gpu-batch")]
    if let Some(plan) = gpu_plan {
        gpu_scene::install_loaded(&mut sim, plan, &context.links, distributed_worker)?;
        if gpu_tool_scene {
            for model in &world.models {
                if let Some(plan) = joint_state_publisher::parse(model)? {
                    let member = context.links[&model.links[0].scoped_name];
                    let entity =
                        gz_sim_core::entity_by_id(sim.world(), member).expect("loaded GPU link");
                    let model_entity = sim
                        .world()
                        .get::<ChildOf>(entity)
                        .expect("loaded GPU model")
                        .parent();
                    let engine = if gpu_scene::parallel_gripper::registered(sim.world()) {
                        "gpu-gripper"
                    } else {
                        "gpu-planar"
                    };
                    plan.validate(model, &sim, Some(engine), (true, model.is_static))?;
                    joint_state_publisher::build(&sdf, &mut sim, model, model_entity, None, &plan)?;
                }
            }
        }
    }
    let mut visuals = context.visuals.into_iter();
    retain_visual_geometry(
        &mut sdf.worlds[world_index].models,
        &mut visuals,
        sim.world_mut(),
    );
    debug_assert!(visuals.next().is_none());
    let mut cameras = context.cameras.into_iter();
    retain_camera_sensors(
        &mut sdf.worlds[world_index].models,
        &mut cameras,
        sim.world_mut(),
    );
    debug_assert!(cameras.next().is_none());
    inputs::install(&mut sim);
    match backend {
        Some(backend) => {
            #[cfg(feature = "distributed-physics")]
            let mut backend = backend;
            #[cfg(feature = "distributed-physics")]
            if clock.is_none()
                && let Some(engine) = backend.downcast_mut::<gz_physics_distributed::SceneBackend>()
            {
                // A recovery load first verifies the public SDF hierarchy against its manifest.
                // Its trusted native images then start each authoritative process directly.
                engine.initialize()?;
            }
            if let Some(plan) = sole_plan {
                if distributed_worker.is_some()
                    || deformable::installed(sim.world())
                    || primary_contact_plan.is_some()
                {
                    return Err(damped_sole::invalid(
                        world_name,
                        "requires exclusive local PhysicsStep ownership",
                    ));
                }
                damped_sole::install(&mut sim, backend, &context.links, plan)?;
            } else if let Some(plan) = primary_contact_plan {
                if distributed_worker.is_some() || deformable::installed(sim.world()) {
                    return Err(primary_contact::invalid(
                        world_name,
                        "requires the local rigid PhysicsStep owner; remove managed workers or deformable coupling",
                    ));
                }
                primary_contact::install(&mut sim, backend, plan)?;
            } else if deformable::exchange::enabled(sim.world()) {
                if distributed_worker.is_some() {
                    return Err(LoadError::Deformable {
                        world: world_name.into(),
                        reason: "rigid_exchanges requires local material ownership".into(),
                    });
                }
                gz_physics::install_with_step(&mut sim, backend, deformable::exchange::step)?;
            } else if deformable::automatic_impact(sim.world()) {
                #[cfg(feature = "distributed-physics")]
                if let Some(exe) =
                    distributed_worker.filter(|_| engine == PhysicsEngine::MultiFidelity)
                {
                    let owner = match managed_material {
                        Some(pool) => pool,
                        None => {
                            deformable::managed_impact::transient(xml, source, world_name, exe)?
                        }
                    };
                    deformable::managed_impact::install(&mut sim, backend, owner, world_name)?;
                } else {
                    gz_physics::install_with_step(
                        &mut sim,
                        backend,
                        deformable::articulated_impact::automatic::advance,
                    )?;
                }
                #[cfg(not(feature = "distributed-physics"))]
                if distributed_worker.is_some() && engine == PhysicsEngine::MultiFidelity {
                    return Err(LoadError::Deformable {
                        world: world_name.into(),
                        reason: "managed automatic impact requires distributed-physics".into(),
                    });
                } else {
                    gz_physics::install_with_step(
                        &mut sim,
                        backend,
                        deformable::articulated_impact::automatic::advance,
                    )?;
                }
            } else {
                #[cfg(feature = "distributed-physics")]
                if let Some(exe) =
                    distributed_worker.filter(|_| engine == PhysicsEngine::MultiFidelity)
                {
                    trajectory_workers::install(&mut sim, backend, exe, world_name)?;
                } else {
                    gz_physics::install(&mut sim, backend)?;
                }
                #[cfg(not(feature = "distributed-physics"))]
                gz_physics::install(&mut sim, backend)?;
            }
            sim.add_systems(Phase::PhysicsCommit, sync_bodies.before(sync_models));
            sim.add_systems(
                Phase::Reset,
                sync_bodies.after(PhysicsReset).before(sync_models),
            );
        }
        None if engine == PhysicsEngine::Tpe => tpe::install(&mut sim)?,
        None => {}
    }
    sim.add_systems(
        Phase::PhysicsCommit,
        (sync_models, sync_local_poses, sync_attached_poses).chain(),
    );
    sim.add_systems(
        Phase::Reset,
        (sync_models, sync_local_poses, sync_attached_poses)
            .chain()
            .after(PhysicsReset),
    );
    imu_lifecycle::install(&mut sim);
    lidar::install(&mut sim);
    altimeter::install(&mut sim);
    air_pressure::install(&mut sim);
    air_speed::install(&mut sim);
    logical_camera::install(&mut sim);
    magnetometer::install(&mut sim);
    joint_lifecycle::install(&mut sim);
    joint_force_lifecycle::install(&mut sim);
    diff_drive::install(&mut sim);
    joint_controller::install(&mut sim);
    joint_state_publisher::install(&mut sim);
    pose_publisher::install(&mut sim);
    odometry_publisher::install(&mut sim);
    if let Some(plan) = wrench_plan {
        apply_link_wrench::build(plan, &mut sim, world_entity)?;
    }
    initial::capture(&mut sim);
    contact_promotion_sdf::install(&mut sim, contact_policy)?;
    #[cfg(feature = "distributed-physics")]
    distributed_hinge::remember_robot_program(&mut sim, xml, &sdf.worlds[world_index]);
    joint_posture::install(&mut sim, joint_postures, &sdf, &sdf.worlds[world_index])?;
    scheduled_link_force::install(&mut sim, force_schedule)?;
    apply_link_wrench::install(&mut sim);
    #[cfg(feature = "distributed-physics")]
    if sim
        .world()
        .contains_resource::<trajectory_workers::TrajectoryProcessPool>()
    {
        trajectory_workers::checkpoint::bind(&mut sim, xml, world_name, step_override, noise_seed);
    }
    Ok(sim)
}

fn retain_camera_sensors(
    models: &mut [Model],
    cameras: &mut impl Iterator<Item = Entity>,
    world: &mut World,
) {
    for model in models {
        for link in &mut model.links {
            for sensor in &mut link.sensors {
                if sensor.camera().is_some() {
                    let entity = cameras
                        .next()
                        .expect("camera was constructed from this SDF");
                    world
                        .entity_mut(entity)
                        .insert(CameraSensor(Arc::new(std::mem::take(sensor))));
                }
            }
        }
        retain_camera_sensors(&mut model.models, cameras, world);
    }
}

#[cfg(test)]
#[path = "../tests/private/camera_binding.rs"]
mod camera_binding_tests;

fn retain_visual_geometry(
    models: &mut [Model],
    visuals: &mut impl Iterator<Item = Entity>,
    world: &mut World,
) {
    // Frame resolution borrows the DOM. Move geometry afterward, in build_model order.
    for model in models {
        for link in &mut model.links {
            for visual in &mut link.visuals {
                let entity = visuals
                    .next()
                    .expect("visual was constructed from this SDF");
                if let Some(geometry) = visual.geometry.take() {
                    world
                        .entity_mut(entity)
                        .insert(VisualGeometry(Arc::new(geometry)));
                }
            }
        }
        retain_visual_geometry(&mut model.models, visuals, world);
    }
}

/// An explicitly authored isotropic ODE friction law. Other friction models
/// and collision filtering need separate native mappings.
fn local_isotropic_friction(surface: &sdformat::Surface) -> Option<IsotropicFriction> {
    let law = &surface.friction;
    let ode = &law.ode;
    (ode.source.is_some()
        && ode.mu == ode.mu2
        && ode.fdir1 == [0.; 3]
        && ode.slip1 == 0.
        && ode.slip2 == 0.
        && law.bullet.is_none()
        && law.torsional.is_none()
        && surface.contact.source.is_none())
    .then(|| IsotropicFriction::new(ode.mu).ok())
    .flatten()
}

/// The SDF surface is immutable construction data, including when a server
/// clones a previously loaded model after simulation has started.
fn spawn_surface_link(
    backend: &mut PhysicsBackend,
    spec: BodySpec,
    rotation: RotationSpec,
    inertia: Inertia,
    prescribed: bool,
    friction: Option<f64>,
) -> Result<BodyId, PhysicsError> {
    #[cfg(feature = "distributed-physics")]
    if let Some(distributed) = backend.downcast_mut::<gz_physics_distributed::SceneBackend>() {
        use gz_physics_distributed::physics::Tier;
        return distributed.spawn_primitive_link(
            spec,
            rotation,
            Some(inertia),
            if prescribed {
                Tier::Prescribed
            } else {
                Tier::Dynamic
            },
            friction.unwrap_or(sdformat::OdeFriction::default().mu),
        );
    }
    if let Some(mu) = friction
        && let Some(native) = backend.downcast_mut::<MultiFidelityBackend>()
    {
        let motion = if prescribed {
            MaterialLinkMotion::Prescribed
        } else {
            MaterialLinkMotion::Ordinary
        };
        return native.spawn_material_link(
            spec,
            rotation,
            Some(inertia),
            motion,
            IsotropicFriction::new(mu)?,
        );
    }
    if prescribed {
        backend.spawn_kinematic_link(spec, rotation, Some(inertia))
    } else {
        backend.spawn_link(spec, rotation, Some(inertia))
    }
}

fn build_model(
    sdf: &Sdf,
    model: &Model,
    context: &mut SceneContext<'_>,
    parent: &ParentModel,
    sim: &mut Simulation,
    backend: &mut Option<PhysicsBackend>,
) -> Result<(), LoadError> {
    if context.engine == PhysicsEngine::Tpe {
        require(
            model.models.is_empty(),
            &model.scoped_name,
            "TPE nested models",
        )?;
    }
    let restitution = contact_material::parse(model, context.engine)?;
    let top_level = sim.world().get::<EntityKind>(parent.entity) == Some(&EntityKind::World);
    let prescribed = prescribed_motion::parse(model, context.engine, top_level)?;
    // The DampedSole loader has admitted at most one moving tree. All other
    // registered panels and static observers are jointless.
    let coated_tree = context.sole_observers && !model.joints.is_empty();
    let initial_velocity =
        prescribed_motion::initial_velocity(model, context.engine, top_level, coated_tree)?;
    let motion = initial_velocity
        .or_else(|| prescribed.as_ref().map(|p| p.twist))
        .unwrap_or_default();
    let assembly_center = if initial_velocity.is_some() && !model.joints.is_empty() {
        Some(prescribed_motion::assembly_center(
            sdf,
            model,
            context.world,
        )?)
    } else {
        None
    };
    let gpu_arm = context.gpu_tool_scene && !model.joints.is_empty();
    let controller_plan = if gpu_arm {
        None
    } else {
        joint_controller::parse(model)?
    };
    if let Some(plan) = &controller_plan {
        plan.validate(
            model,
            sim,
            backend.as_ref().map(|backend| backend.engine()),
            (top_level, parent.is_static || model.is_static),
        )?;
    }
    let ground_truth_plan = odometry_publisher::parse(model)?;
    if let Some(plan) = &ground_truth_plan {
        plan.validate(
            model,
            sim,
            backend.as_ref().map(|backend| backend.engine()),
            (top_level, parent.is_static || model.is_static),
        )?;
    }
    let pose_plan = pose_publisher::parse(model)?;
    if let Some(plan) = &pose_plan {
        plan.validate(
            model,
            sim,
            backend.as_ref().map(|backend| backend.engine()),
            (top_level, parent.is_static || model.is_static),
        )?;
    }
    let joint_state_plan = if gpu_arm {
        None
    } else {
        joint_state_publisher::parse(model)?
    };
    if let Some(plan) = &joint_state_plan {
        plan.validate(
            model,
            sim,
            backend.as_ref().map(|backend| backend.engine()),
            (top_level, parent.is_static || model.is_static),
        )?;
    }
    let drive_plan = diff_drive::parse(model)?;
    let drive_pose = sdf.pose_of(&model.scoped_name, context.world)?;
    if let Some(plan) = &drive_plan {
        plan.validate(
            model,
            sim,
            backend.as_ref().map(|backend| backend.engine()),
            (top_level, parent.is_static || model.is_static),
            drive_pose,
        )?;
    }
    let actuated: Vec<_> = drive_plan
        .as_ref()
        .into_iter()
        .flat_map(|plan| plan.names())
        .chain(
            controller_plan
                .as_ref()
                .into_iter()
                .flat_map(|plan| plan.names()),
        )
        .collect();
    require(
        actuated
            .iter()
            .copied()
            .collect::<std::collections::BTreeSet<_>>()
            .len()
            == actuated.len(),
        &model.scoped_name,
        "conflicting joint actuator ownership",
    )?;
    // GPU tool admission validates the complete joint graph and both axes before scene construction.
    let reduced_plan = if gpu_arm {
        None
    } else {
        joints::validate_model(
            sdf,
            model,
            context.world,
            top_level,
            backend.as_ref().map(|backend| backend.engine()),
            parent.is_static || model.is_static,
            &actuated,
            coated_tree,
        )?
    };
    let force_configurations = joint_forces::parse(model)?;
    require(
        force_configurations
            .iter()
            .all(|configuration| !actuated.contains(&configuration.joint_name())),
        &model.scoped_name,
        if drive_plan.is_some() {
            "conflicting wheel actuator ownership"
        } else {
            "conflicting joint actuator ownership"
        },
    )?;
    let is_static = parent.is_static || model.is_static;
    let model_topic = format!("{}/model/{}", parent.topic, model.name);
    let parent_name = &sim
        .world()
        .get::<Name>(parent.entity)
        .expect("scene owns hierarchy")
        .0;
    let local = sdf.pose_of(&model.scoped_name, parent_name)?;
    let pose = sdf.pose_of(&model.scoped_name, context.world)?;
    let model_entity = spawn_node(
        sim,
        Some(parent.entity),
        &model.scoped_name,
        EntityKind::Model,
        local,
        pose,
    )?;
    if context.engine == PhysicsEngine::Tpe {
        tpe::add_model(sim, model_entity, pose, is_static)?;
    }
    let parent = ParentModel {
        entity: model_entity,
        is_static,
        topic: model_topic.clone(),
    };
    let observer = context.sole_observers && top_level && damped_sole::observer_model(model);
    for link in &model.links {
        let name = &link.scoped_name;
        require(link.plugins.is_empty(), name, "link plugins")?;
        require(!link.properties.enable_wind, name, "link wind")?;
        require(
            link.properties.enable_gravity,
            name,
            "disabled link gravity",
        )?;
        require(
            !link.properties.kinematic || prescribed.is_some(),
            name,
            "kinematic links",
        )?;
        require(!link.properties.self_collide, name, "link self collision")?;
        require(
            link.visuals.iter().all(|visual| visual.plugins.is_empty()),
            name,
            "visual plugins",
        )?;
        require(
            observer
                || (gpu_arm && link.collisions.is_empty())
                || link.collisions.len() == 1
                || (context.engine == PhysicsEngine::MultiFidelity && !link.collisions.is_empty()),
            name,
            "collision count (expected one)",
        )?;
        require(
            link.inertial.fluid_added_mass.is_none(),
            name,
            "fluid added mass",
        )?;
        let mut solids = Vec::with_capacity(link.collisions.len());
        let mut plane = None;
        let mut mesh_collision = None;
        let mut local_friction = None;
        for collision in &link.collisions {
            if context.engine == PhysicsEngine::MultiFidelity || context.gpu_tool_scene {
                let material = local_isotropic_friction(&collision.surface);
                if !solids.is_empty() {
                    require(
                        local_friction == material,
                        name,
                        "mixed materials on one compound collider",
                    )?;
                }
                local_friction = material;
            }
            require(
                context.engine.is_distributed()
                    || collision.surface.source.is_none()
                    || local_friction.is_some(),
                name,
                "collision surface parameters",
            )?;
            if matches!(&collision.geometry, Geometry::Mesh(_)) {
                require(
                    backend.as_ref().is_some_and(|b| b.engine() == "rapier"),
                    name,
                    "collision geometry",
                )?;
            }
            let local = sdf.resolve_pose(&collision.pose, &model.scoped_name, name, name)?;
            require(
                identity(local)
                    || (gpu_arm && matches!(collision.geometry, Geometry::Sphere { .. }))
                    || (context.engine == PhysicsEngine::MultiFidelity
                        && matches!(
                            collision.geometry,
                            Geometry::Sphere { .. }
                                | Geometry::Box { .. }
                                | Geometry::Cylinder { .. }
                                | Geometry::Capsule { .. }
                        ))
                    || (context.engine.is_distributed()
                        && matches!(&collision.geometry, Geometry::Plane { .. }))
                    || (matches!(&collision.geometry, Geometry::Mesh(_))
                        && backend.as_ref().is_some_and(|b| b.engine() == "rapier")),
                name,
                "offset collision pose",
            )?;
            if let Geometry::Mesh(mesh) = &collision.geometry {
                require(link.collisions.len() == 1, name, "compound mesh collision")?;
                let geometry = gz_assets::mesh::load(mesh).map_err(|source| LoadError::Mesh {
                    entity: name.clone(),
                    source,
                })?;
                mesh_collision = Some((geometry, local));
                continue;
            }
            let shape = match collision.geometry {
                Geometry::Sphere { radius } => Shape::Sphere { radius },
                Geometry::Cylinder { radius, length } => Shape::Cylinder {
                    radius,
                    half_height: length * 0.5,
                },
                Geometry::Capsule { radius, length } => Shape::Capsule {
                    radius,
                    half_length: length * 0.5,
                },
                Geometry::Box { size } => Shape::Box {
                    half_extents: size.map(|v| v * 0.5),
                },
                Geometry::Plane { normal, size } => {
                    require(
                        link.collisions.len() == 1,
                        name,
                        "plane in compound collision",
                    )?;
                    plane = Some((normal, size));
                    continue;
                }
                _ => {
                    return Err(LoadError::Unsupported {
                        entity: name.clone(),
                        feature: "collision geometry",
                    });
                }
            };
            solids.push(CollisionSpec {
                shape,
                position: local.translation,
                orientation: local.rotation,
            });
        }
        let pose = sdf.pose_of(name, context.world)?;
        let inertial_pose = if context.engine.is_distributed() {
            // Distributed admission already resolved and checked the inertial
            // frame. Canonicalize link-frame roundoff for the strict native API.
            Transform::default()
        } else {
            sdf.resolve_pose(&link.inertial.pose, &model.scoped_name, name, name)?
        };
        let free_mass_offset = context.engine == PhysicsEngine::MultiFidelity
            && top_level
            && !is_static
            && model.links.len() == 1
            && model.joints.is_empty()
            && model.models.is_empty()
            && prescribed.is_none()
            && plane.is_none()
            && mesh_collision.is_none();
        require(
            centered(inertial_pose.translation)
                || gpu_arm
                || ((reduced_plan.is_some() || free_mass_offset)
                    && inertial_pose.translation.into_iter().all(f64::is_finite)),
            name,
            "offset center of mass",
        )?;
        let mut velocity = motion;
        if let Some(center) = assembly_center {
            let link_com = DVec3::from(pose.translation)
                + DQuat::from_array(pose.rotation) * DVec3::from(inertial_pose.translation);
            let linear = DVec3::from(motion.linear_at_com)
                + DVec3::from(motion.angular).cross(link_com - center);
            require(
                linear.is_finite(),
                name,
                "unrepresentable initial assembly twist",
            )?;
            velocity.linear_at_com = linear.to_array();
        }
        // The directive supplies world COM velocity; ECS observations use the link origin.
        let origin_velocity = if (prescribed.is_some() || initial_velocity.is_some())
            && inertial_pose.translation != [0.0; 3]
        {
            let linear = DVec3::from_array(velocity.linear_at_com)
                - LinkCenterOfMass(inertial_pose.translation)
                    .velocity_shift(pose.rotation, velocity.angular);
            require(
                linear.is_finite(),
                name,
                "unrepresentable prescribed link velocity",
            )?;
            linear
        } else {
            DVec3::from_array(velocity.linear_at_com)
        };
        let (id, tpe_link) = match plane {
            _ if observer => (None, None),
            _ if context.engine == PhysicsEngine::GpuBatch => (None, None),
            Some((normal, size)) => {
                require(is_static, name, "dynamic plane collision geometry")?;
                require(
                    context.engine.is_distributed() || normal == [0.0, 0.0, 1.0],
                    name,
                    "plane normal (expected local +Z)",
                )?;
                require(
                    link.inertial.mass == 1.0
                        && link.inertial.inertia == [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]
                        && identity(inertial_pose),
                    name,
                    "custom plane inertia",
                )?;
                let backend = backend.as_mut().ok_or_else(|| LoadError::Unsupported {
                    entity: name.clone(),
                    feature: "TPE plane collision",
                })?;
                let plane = PlaneSpec {
                    position: pose.translation,
                    orientation: pose.rotation,
                    size,
                };
                #[cfg(feature = "distributed-physics")]
                let distributed = if let Some(d) =
                    backend.downcast_mut::<gz_physics_distributed::SceneBackend>()
                {
                    let collision = &link.collisions[0];
                    let frame =
                        sdf.resolve_pose(&collision.pose, &model.scoped_name, name, context.world)?;
                    let n = DVec3::from_array(normal)
                        .try_normalize()
                        .ok_or(PhysicsError::InvalidValue)?;
                    let n = (DQuat::from_array(frame.rotation) * n).normalize();
                    let ground = gz_physics_distributed::physics::GroundPlane::oriented(
                        frame.translation,
                        n.to_array(),
                        collision.surface.friction.ode.mu,
                    )?;
                    Some(d.spawn_ground_geometry(plane, ground))
                } else {
                    None
                };
                #[cfg(not(feature = "distributed-physics"))]
                let distributed: Option<Result<BodyId, PhysicsError>> = None;
                let id = match distributed {
                    Some(id) => id?,
                    None if let Some(mu) = local_friction => backend
                        .downcast_mut::<MultiFidelityBackend>()
                        .expect("local material backend")
                        .spawn_material_plane(plane, mu)?,
                    None => backend.spawn_static_plane(plane)?,
                };
                (Some(id), None)
            }
            None if mesh_collision.is_some() => {
                let (geometry, local) = mesh_collision.as_ref().expect("matched mesh collision");
                let backend = backend
                    .as_mut()
                    .expect("mesh backend admitted before asset IO");
                let id = backend.spawn_mesh_link(
                    MeshBodySpec {
                        vertices: &geometry.vertices,
                        triangles: &geometry.triangles,
                        collision_position: local.translation,
                        collision_orientation: local.rotation,
                        position: pose.translation,
                        velocity: [0.0; 3],
                        mass: (!is_static).then_some(link.inertial.mass),
                    },
                    RotationSpec {
                        orientation: pose.rotation,
                        ..Default::default()
                    },
                    Some(Inertia {
                        center: inertial_pose.translation,
                        orientation: inertial_pose.rotation,
                        tensor: link.inertial.inertia,
                    }),
                )?;
                if !model.joints.is_empty() || (top_level && model.links.len() > 1) {
                    let group = sim
                        .world()
                        .get::<EntityId>(model_entity)
                        .expect("loader constructed model")
                        .get();
                    backend.set_self_collision_group(id, Some(group))?;
                }
                (Some(id), None)
            }
            None => {
                let spec = BodySpec {
                    shape: solids[0].shape,
                    position: pose.translation,
                    velocity: velocity.linear_at_com,
                    mass: (!is_static).then_some(link.inertial.mass),
                };
                if let Some(backend) = backend {
                    let rotation = RotationSpec {
                        orientation: pose.rotation,
                        angular_velocity: velocity.angular,
                    };
                    let inertia = Inertia {
                        center: inertial_pose.translation,
                        orientation: inertial_pose.rotation,
                        tensor: link.inertial.inertia,
                    };
                    if prescribed.is_some() {
                        backend.validate_kinematic_velocity(
                            spec.velocity,
                            rotation.angular_velocity,
                            sim.step_duration(),
                        )?;
                    }
                    let compound = solids.len() > 1
                        || solids.first().is_some_and(|part| {
                            !identity(Transform {
                                translation: part.position,
                                rotation: part.orientation,
                            })
                        });
                    let id = if compound {
                        let native =
                            backend
                                .downcast_mut::<MultiFidelityBackend>()
                                .ok_or_else(|| LoadError::Unsupported {
                                    entity: name.clone(),
                                    feature: "compound collision on this backend",
                                })?;
                        let motion = if prescribed.is_some() {
                            MaterialLinkMotion::Prescribed
                        } else if reduced_plan.is_some() {
                            MaterialLinkMotion::Articulation
                        } else {
                            MaterialLinkMotion::Ordinary
                        };
                        native.spawn_compound_link(
                            &solids,
                            spec,
                            rotation,
                            inertia,
                            motion,
                            local_friction,
                        )?
                    } else if reduced_plan.is_some() && prescribed.is_none() {
                        let native = backend
                            .downcast_mut::<MultiFidelityBackend>()
                            .expect("selected reduced model uses the multi-fidelity engine");
                        if let Some(mu) = local_friction {
                            native.spawn_material_link(
                                spec,
                                rotation,
                                Some(inertia),
                                MaterialLinkMotion::Articulation,
                                mu,
                            )?
                        } else {
                            native.spawn_articulation_link(spec, rotation, Some(inertia))?
                        }
                    } else {
                        spawn_surface_link(
                            backend,
                            spec,
                            rotation,
                            inertia,
                            prescribed.is_some(),
                            link.collisions[0]
                                .surface
                                .source
                                .as_ref()
                                .map(|_| link.collisions[0].surface.friction.ode.mu),
                        )?
                    };
                    if let Some(path) = prescribed.as_ref().and_then(|p| p.path.as_ref()) {
                        backend
                            .downcast_mut::<MultiFidelityBackend>()
                            .expect("validated trajectory backend")
                            .set_initial_translation_path(id, path.clone())
                            .map_err(|e| LoadError::TranslationPath {
                                entity: name.clone(),
                                reason: e.to_string(),
                            })?;
                    }
                    if !model.allow_auto_disable
                        && model.joints.is_empty()
                        && backend.is_dynamic(id) == Some(true)
                    {
                        if let Some(native) = backend.downcast_mut::<MultiFidelityBackend>() {
                            native.keep_body_awake(id)?;
                        } else if let Some(native) = backend.downcast_mut::<RapierBackend>() {
                            native.keep_body_awake(id)?;
                        }
                    }
                    if !model.joints.is_empty() || (top_level && model.links.len() > 1) {
                        let group = sim
                            .world()
                            .get::<EntityId>(model_entity)
                            .expect("loader constructed model")
                            .get();
                        backend.set_self_collision_group(id, Some(group))?;
                    }
                    (Some(id), None)
                } else {
                    (None, Some(tpe::validate_link(spec, pose)?))
                }
            }
        };
        if let Some(&value) = restitution.get(&link.name) {
            let body = id.ok_or_else(|| LoadError::Unsupported {
                entity: name.clone(),
                feature: "restitution requires a physical link",
            })?;
            backend
                .as_mut()
                .and_then(|b| b.downcast_mut::<MultiFidelityBackend>())
                .expect("validated ContactMaterial engine")
                .set_initial_restitution(body, value)?;
        }
        let local = sdf.pose_of(name, &model.scoped_name)?;
        let link_entity = spawn_node(sim, Some(model_entity), name, EntityKind::Link, local, pose)?;
        sim.world_mut()
            .entity_mut(link_entity)
            .insert(LinkProperties(link.properties));
        sim.world_mut().entity_mut(link_entity).insert(Velocity {
            linear: origin_velocity.to_array(),
            angular: velocity.angular,
        });
        if let Some(id) = id {
            sim.world_mut()
                .entity_mut(link_entity)
                .insert(BodyHandle(id));
            if plane.is_none() && inertial_pose.translation != [0.; 3] {
                sim.world_mut()
                    .entity_mut(link_entity)
                    .insert(LinkCenterOfMass(inertial_pose.translation));
            }
        } else if let Some(link) = tpe_link {
            sim.world_mut().entity_mut(link_entity).insert(link);
        }
        let link_id = sim
            .world()
            .get::<EntityId>(link_entity)
            .expect("loader assigned the link identity")
            .get();
        context.links.insert(name.clone(), link_id);
        for (child_name, child_pose, kind, visual, density) in link
            .collisions
            .iter()
            .map(|child| {
                (
                    &child.name,
                    &child.pose,
                    EntityKind::Collision,
                    None,
                    child.density,
                )
            })
            .chain(link.visuals.iter().map(|child| {
                (
                    &child.name,
                    &child.pose,
                    EntityKind::Visual,
                    Some(child),
                    None,
                )
            }))
        {
            let local = sdf.resolve_pose(child_pose, &model.scoped_name, name, name)?;
            let world = sdf.resolve_pose(child_pose, &model.scoped_name, name, context.world)?;
            let entity = spawn_node(
                sim,
                Some(link_entity),
                &format!("{name}::{child_name}"),
                kind,
                local,
                world,
            )?;
            sim.world_mut().entity_mut(entity).insert(AttachedPose);
            if let Some(density) = density {
                sim.world_mut()
                    .entity_mut(entity)
                    .insert(CollisionDensity(density));
            }
            if let Some(visual) = visual {
                context.visuals.push(entity);
                let mut entity = sim.world_mut().entity_mut(entity);
                entity.insert(VisualProperties(visual.properties.clone()));
                if let Some(material) = &visual.material {
                    entity.insert(VisualMaterial(material.clone()));
                }
            }
        }
        for sensor in &link.sensors {
            let name = format!("{}::{}", link.scoped_name, sensor.name);
            require(!sensor.enable_metrics, &name, "sensor metrics")?;
            if sensor.camera().is_some() {
                let local = sdf.resolve_pose(
                    &sensor.pose,
                    &model.scoped_name,
                    &link.scoped_name,
                    &link.scoped_name,
                )?;
                let world = sdf.resolve_pose(
                    &sensor.pose,
                    &model.scoped_name,
                    &link.scoped_name,
                    context.world,
                )?;
                let entity = spawn_node(
                    sim,
                    Some(link_entity),
                    &name,
                    EntityKind::Sensor,
                    local,
                    world,
                )?;
                sim.world_mut().entity_mut(entity).insert(AttachedPose);
                context.cameras.push(entity);
                continue;
            }
            if sensor.lidar().is_some() {
                lidar::spawn(
                    sdf,
                    model,
                    sensor,
                    context,
                    sim,
                    backend.as_ref(),
                    lidar::Parent {
                        link,
                        entity: link_entity,
                        id: link_id,
                        topic: &model_topic,
                    },
                )?;
                continue;
            }
            if sensor.effective_type() == Some(sdformat::SensorType::LogicalCamera) {
                logical_camera::spawn(
                    sdf,
                    model,
                    sensor,
                    context,
                    sim,
                    backend.as_ref(),
                    logical_camera::Parent {
                        link,
                        entity: link_entity,
                        id: link_id,
                        topic: &model_topic,
                    },
                )?;
                continue;
            }
            if sensor.air_speed().is_some() {
                air_speed::spawn(
                    sdf,
                    model,
                    sensor,
                    context,
                    sim,
                    backend.as_ref(),
                    air_speed::Parent {
                        link,
                        entity: link_entity,
                        id: link_id,
                        topic: &model_topic,
                    },
                )?;
                continue;
            }
            if sensor.air_pressure().is_some() {
                air_pressure::spawn(
                    sdf,
                    model,
                    sensor,
                    context,
                    sim,
                    backend.as_ref(),
                    air_pressure::Parent {
                        link,
                        entity: link_entity,
                        topic: &model_topic,
                    },
                )?;
                continue;
            }
            if sensor.magnetometer().is_some() {
                magnetometer::spawn(
                    sdf,
                    model,
                    sensor,
                    context,
                    sim,
                    backend.as_ref(),
                    magnetometer::Parent {
                        link,
                        entity: link_entity,
                        topic: &model_topic,
                        field_world: context.field_world,
                    },
                )?;
                continue;
            }
            if sensor.altimeter().is_some() {
                altimeter::spawn(
                    sdf,
                    model,
                    sensor,
                    context,
                    sim,
                    backend.as_ref(),
                    altimeter::Parent {
                        link,
                        entity: link_entity,
                        id: link_id,
                        topic: &model_topic,
                    },
                )?;
                continue;
            }
            let config = sensor.imu().ok_or_else(|| LoadError::Unsupported {
                entity: name.clone(),
                feature: "non-IMU sensors",
            })?;
            let reference_frame =
                imu_reference::validate(config, context.spherical_coordinates.as_ref(), &name)?;
            let schedule =
                SampleSchedule::new(sensor.update_rate).map_err(|_| LoadError::Unsupported {
                    entity: name.clone(),
                    feature: "unrepresentable sensor rate",
                })?;
            let local = sdf.resolve_pose(
                &sensor.pose,
                &model.scoped_name,
                &link.scoped_name,
                &link.scoped_name,
            )?;
            require(centered(local.translation), &name, "offset IMU position")?;
            let entity_name = name
                .strip_prefix(&format!("{}::", context.world))
                .expect("scene names include their world scope")
                .to_owned();
            let topic = if sensor.topic.is_empty() {
                format!(
                    "{model_topic}/link/{}/sensor/{}/imu",
                    link.name, sensor.name
                )
            } else {
                sensor.topic.clone()
            };
            let world = sdf.resolve_pose(
                &sensor.pose,
                &model.scoped_name,
                &link.scoped_name,
                context.world,
            )?;
            let sensor_entity = spawn_node(
                sim,
                Some(link_entity),
                &name,
                EntityKind::Sensor,
                local,
                world,
            )?;
            let identity = sim
                .world()
                .get::<EntityId>(sensor_entity)
                .expect("spawned scene nodes own public identities")
                .get();
            let noise = noise::ImuNoise::new(
                config,
                context.noise_seed,
                identity,
                context.named_imu_streams.then_some(name.as_str()),
            )
            .map_err(|_| LoadError::Unsupported {
                entity: name.clone(),
                feature: "unrepresentable IMU noise",
            })?;
            sim.world_mut().entity_mut(sensor_entity).insert((
                AttachedPose,
                ImuPublication {
                    enable_orientation: config.enable_orientation,
                    frame_id: if sensor.frame_id.is_empty() {
                        sensor
                            .gz_frame_id
                            .clone()
                            .unwrap_or_else(|| entity_name.clone())
                    } else {
                        sensor.frame_id.clone()
                    },
                    entity_name,
                    topic,
                },
                Imu {
                    link: link_id,
                    rotation: DQuat::from_array(local.rotation),
                    reference: imu_reference::reference(
                        reference_frame,
                        DQuat::from_array(pose.rotation) * DQuat::from_array(local.rotation),
                    ),
                    initial_velocity: origin_velocity,
                    previous_velocity: origin_velocity,
                    schedule,
                    sdf_rate_hz: sensor.update_rate,
                    rate_hz: sensor.update_rate,
                    active: true,
                    noise,
                },
            ));
        }
    }
    if let Some(backend) = backend {
        let construction = match &reduced_plan {
            Some(plan) => joints::JointConstruction::Reduced(&plan.joint_order),
            None if controller_plan.is_some() => joints::JointConstruction::Controller,
            None => joints::JointConstruction::Impulse,
        };
        joints::build(
            sdf,
            model,
            context,
            model_entity,
            sim,
            backend,
            construction,
        )?;
        if let Some(root_index) = reduced_plan.as_ref().and_then(|plan| plan.switchable_root) {
            let root = context.links[&model.links[root_index].scoped_name];
            let plan = reduced_plan.as_ref().expect("constructed switchable root");
            if plan.exact_revolute_locks || plan.additional_pgs_iterations.is_some() {
                let entity = gz_sim_core::entity_by_id(sim.world(), root)
                    .expect("constructed switchable root");
                let body = sim
                    .world()
                    .get::<BodyHandle>(entity)
                    .expect("constructed root body")
                    .id();
                let local = backend
                    .downcast_mut::<MultiFidelityBackend>()
                    .expect("validated switchable articulation engine");
                if let Some(iterations) = plan.additional_pgs_iterations {
                    local.set_robot_root_iterations(body, iterations)?;
                }
                if plan.exact_revolute_locks {
                    local.enable_exact_revolute_locks(body)?;
                }
            }
            articulation::bind(sim, model_entity, root);
        }
        if let Some(plan) = &ground_truth_plan {
            odometry_publisher::build(
                sdf,
                sim,
                model,
                model_entity,
                backend,
                plan,
                context.noise_seed,
            )?;
        }
        if let Some(plan) = &pose_plan {
            pose_publisher::build(sdf, sim, model, model_entity, backend, plan)?;
        }
        if let Some(plan) = &joint_state_plan {
            joint_state_publisher::build(sdf, sim, model, model_entity, Some(backend), plan)?;
        }
        joint_forces::install(sim, model, model_entity, backend, &force_configurations)?;
        if let Some(plan) = &drive_plan {
            diff_drive::build(sim, model, model_entity, backend, plan)?;
        }
        if let Some(plan) = &controller_plan {
            joint_controller::build(sim, model, model_entity, backend, plan)?;
        }
    }
    for child in &model.models {
        build_model(sdf, child, context, &parent, sim, backend)?;
    }
    if context.engine == PhysicsEngine::Tpe {
        tpe::add_bounds(sim, model_entity, is_static)?;
    }
    if !is_static && context.engine != PhysicsEngine::Tpe {
        let (canonical, _) = model.canonical_link_and_relative_name();
        let canonical = canonical.ok_or_else(|| LoadError::Unsupported {
            entity: model.scoped_name.clone(),
            feature: "dynamic model without a canonical link",
        })?;
        let link = context
            .links
            .get(&canonical.scoped_name)
            .copied()
            .expect("scene constructs every canonical link before its model anchor");
        let model_to_link =
            WorldPose::from(sdf.pose_of(&canonical.scoped_name, &model.scoped_name)?);
        sim.world_mut()
            .entity_mut(model_entity)
            .insert(ModelAnchor {
                link,
                link_to_model: model_to_link.inverse(),
            });
    }
    Ok(())
}

fn spawn_node(
    sim: &mut Simulation,
    parent: Option<Entity>,
    name: &str,
    kind: EntityKind,
    local: Transform,
    world: Transform,
) -> Result<Entity, LoadError> {
    let entity = sim
        .world_mut()
        .spawn((
            Name(name.into()),
            LocalName(
                name.rsplit("::")
                    .next()
                    .expect("name contains at least one segment")
                    .into(),
            ),
            kind,
            LocalPose(local.into()),
            WorldPose::from(world),
        ))
        .id();
    if let Some(parent) = parent {
        sim.world_mut().entity_mut(entity).insert(ChildOf(parent));
    }
    assign_id(sim.world_mut(), entity)?;
    Ok(entity)
}

// These queries have only archetypal filters. Counting their matching tables
// avoids starting a task pool or dispatching small scenes on worker threads.
fn parallel_pose_updates(rows: usize) -> bool {
    rows >= 8192
        && bevy_tasks::ComputeTaskPool::get_or_init(bevy_tasks::TaskPool::new).thread_num() > 1
}

fn pose_batch_rows(rows: usize) -> usize {
    rows.div_ceil(bevy_tasks::ComputeTaskPool::get().thread_num())
        .max(2048)
}

fn sync_models(
    identities: EntityLookup,
    links: Query<(&EntityId, &WorldPose), Without<ModelAnchor>>,
    mut models: Query<(&ModelAnchor, &mut WorldPose)>,
) {
    if parallel_pose_updates(models.count())
        && let (Ok(link_tables), Ok(model_chunks)) =
            (links.contiguous_iter(), models.contiguous_iter_mut())
    {
        let tables: Vec<_> = link_tables.collect();
        // Fetch whole Bevy tables before splitting their disjoint rows. This keeps
        // each pose slice paired with the entity rows that produced it.
        let tables = &tables;
        let identities = &identities;
        let links = &links;
        bevy_tasks::ComputeTaskPool::get().scope(|scope| {
            for (anchors, poses) in model_chunks {
                let batch_rows = pose_batch_rows(anchors.len());
                let (poses, _) = bevy_ecs::change_detection::ContiguousMut::split(poses);
                for (anchors, poses) in anchors.chunks(batch_rows).zip(poses.chunks_mut(batch_rows))
                {
                    scope.spawn(async move {
                        let mut resolve = identities.reader();
                        for (anchors, poses) in anchors.chunks(64).zip(poses.chunks_mut(64)) {
                            // Both tables often retain construction order. Seek once per
                            // small block; swap-removes and restored IDs can break a run.
                            let source = tables.iter().find_map(|(ids, values)| {
                                let start = ids.partition_point(|id| id.get() < anchors[0].link);
                                (ids.get(start)?.get() == anchors[0].link)
                                    .then(|| (&ids[start..], &values[start..]))
                            });
                            for (i, (anchor, pose)) in
                                anchors.iter().zip(poses.iter_mut()).enumerate()
                            {
                                let adjacent = source.and_then(|(ids, values)| {
                                    (ids.get(i)?.get() == anchor.link).then(|| &values[i])
                                });
                                let link = adjacent.unwrap_or_else(|| {
                                    let entity = resolve(anchor.link)
                                        .expect("scene owns canonical link identity");
                                    links.get(entity).expect("scene owns canonical link pose").1
                                });
                                *pose = link.compose(anchor.link_to_model);
                            }
                        }
                    });
                }
            }
        });
        return;
    }
    let mut resolve = identities.reader();
    for (anchor, mut pose) in &mut models {
        let entity = resolve(anchor.link).expect("scene owns canonical link identity");
        let (_, link) = links.get(entity).expect("scene owns canonical link pose");
        *pose = link.compose(anchor.link_to_model);
    }
}

type PoseUpdateFilter = (
    Without<AttachedPose>,
    Without<PendingResetRemoval>,
    Without<tpe::TpeLink>,
);

// Use the parent entities from the same Bevy table as their poses. A restored,
// moved or out-of-order parent always falls back to the current ECS relation.
fn publish_parent_tables<'w, S: Sync + 'w, T: Send + 'w>(
    parents: Vec<(&[Entity], &[WorldPose])>,
    nodes: impl Iterator<
        Item = (
            &'w [ChildOf],
            &'w [S],
            bevy_ecs::change_detection::ContiguousMut<'w, T>,
        ),
    >,
    lookup: impl Fn(Entity) -> WorldPose + Sync,
    compose: impl Fn(WorldPose, &S) -> T + Sync,
) {
    let parents = &parents;
    let lookup = &lookup;
    let compose = &compose;
    bevy_tasks::ComputeTaskPool::get().scope(|scope| {
        for (children, sources, targets) in nodes {
            let size = pose_batch_rows(children.len());
            let (targets, _) = bevy_ecs::change_detection::ContiguousMut::split(targets);
            for ((children, sources), targets) in children
                .chunks(size)
                .zip(sources.chunks(size))
                .zip(targets.chunks_mut(size))
            {
                scope.spawn(async move {
                    publish_parented(children, sources, targets, parents, lookup, compose);
                });
            }
        }
    });
}

fn publish_parented<S, T>(
    children: &[ChildOf],
    sources: &[S],
    targets: &mut [T],
    parents: &[(&[Entity], &[WorldPose])],
    lookup: impl Fn(Entity) -> WorldPose,
    compose: impl Fn(WorldPose, &S) -> T,
) {
    for ((children, sources), targets) in children
        .chunks(64)
        .zip(sources.chunks(64))
        .zip(targets.chunks_mut(64))
    {
        let first = children[0].parent();
        let run = parents.iter().find_map(|(ids, poses)| {
            let start = ids.partition_point(|entity| entity.index_u32() < first.index_u32());
            (ids.get(start) == Some(&first)).then(|| (&ids[start..], &poses[start..]))
        });
        for (i, ((child, source), target)) in children.iter().zip(sources).zip(targets).enumerate()
        {
            let id = child.parent();
            let pose = run
                .and_then(|(ids, poses)| {
                    if id == first {
                        Some(poses[0])
                    } else {
                        (ids.get(i) == Some(&id)).then(|| poses[i])
                    }
                })
                .unwrap_or_else(|| lookup(id));
            *target = compose(pose, source);
        }
    }
}

fn local_from_world(parent: WorldPose, world: &WorldPose) -> LocalPose {
    LocalPose(
        if parent.rotation == [0., 0., 0., 1.] && world.rotation == [0., 0., 0., 1.] {
            WorldPose {
                translation: (DVec3::from_array(world.translation)
                    - DVec3::from_array(parent.translation))
                .to_array(),
                rotation: [0., 0., 0., 1.],
            }
        } else {
            parent.inverse().compose(*world)
        },
    )
}

fn sync_local_poses(
    parents: Query<(Entity, &WorldPose)>,
    mut nodes: Query<(&ChildOf, &WorldPose, &mut LocalPose), PoseUpdateFilter>,
) {
    let lookup = |id| *parents.get(id).expect("scene owns hierarchy").1;
    if parallel_pose_updates(nodes.count())
        && let (Ok(parents), Ok(nodes)) = (parents.contiguous_iter(), nodes.contiguous_iter_mut())
    {
        publish_parent_tables(parents.collect(), nodes, lookup, local_from_world);
    } else {
        for (child, world, mut local) in &mut nodes {
            *local = local_from_world(lookup(child.parent()), world);
        }
    }
}

fn sync_attached_poses(
    parents: Query<(Entity, &WorldPose), Without<AttachedPose>>,
    mut nodes: Query<(&ChildOf, &LocalPose, &mut WorldPose), With<AttachedPose>>,
) {
    let lookup = |id| *parents.get(id).expect("attached poses have link parents").1;
    let compose = |parent: WorldPose, local: &LocalPose| parent.compose(local.0);
    if parallel_pose_updates(nodes.count())
        && let (Ok(parents), Ok(nodes)) = (parents.contiguous_iter(), nodes.contiguous_iter_mut())
    {
        publish_parent_tables(parents.collect(), nodes, lookup, compose);
    } else {
        for (child, local, mut world) in &mut nodes {
            *world = compose(lookup(child.parent()), local);
        }
    }
}

fn sync_bodies(
    backend: Res<PhysicsBackend>,
    mut bodies: Query<(
        &BodyHandle,
        Option<&LinkCenterOfMass>,
        &mut WorldPose,
        &mut Velocity,
    )>,
) {
    let publish = |state: gz_physics::BodyState,
                   center: Option<&LinkCenterOfMass>,
                   pose: &mut WorldPose,
                   velocity: &mut Velocity| {
        pose.translation = state.position;
        pose.rotation = state.orientation;
        velocity.linear = center.map_or(state.velocity, |center| {
            (DVec3::from_array(state.velocity)
                - center.velocity_shift(state.orientation, state.angular_velocity))
            .to_array()
        });
        velocity.angular = state.angular_velocity;
    };
    if parallel_pose_updates(bodies.count())
        && let Ok(tables) = bodies.contiguous_iter_mut()
    {
        let backend = &*backend;
        bevy_tasks::ComputeTaskPool::get().scope(|scope| {
            for (handles, centers, poses, velocities) in tables {
                let batch_rows = pose_batch_rows(handles.len());
                let (poses, _) = bevy_ecs::change_detection::ContiguousMut::split(poses);
                let (velocities, _) = bevy_ecs::change_detection::ContiguousMut::split(velocities);
                for (batch, ((handles, poses), velocities)) in handles
                    .chunks(batch_rows)
                    .zip(poses.chunks_mut(batch_rows))
                    .zip(velocities.chunks_mut(batch_rows))
                    .enumerate()
                {
                    let start = batch * batch_rows;
                    let centers = centers.map(|all| &all[start..start + handles.len()]);
                    scope.spawn(async move {
                        // Each task owns contiguous ECS writes and a scoped native cursor.
                        // A missing/repeated/out-of-order ID has the scalar read semantics.
                        backend.read_states(&mut |states| {
                            for (i, ((handle, pose), velocity)) in handles
                                .iter()
                                .zip(poses.iter_mut())
                                .zip(velocities.iter_mut())
                                .enumerate()
                            {
                                let state =
                                    states.state(handle.0).expect("scene owns body lifecycle");
                                publish(state, centers.map(|s| &s[i]), pose, velocity);
                            }
                        });
                    });
                }
            }
        });
        return;
    }
    backend.read_states(&mut |states| {
        for (body, center, mut pose, mut velocity) in &mut bodies {
            publish(
                states.state(body.0).expect("scene owns body lifecycle"),
                center,
                &mut pose,
                &mut velocity,
            );
        }
    });
}

#[derive(Debug, thiserror::Error)]
#[error("IMU {entity:?}: {source}")]
struct SensorScheduleFault {
    entity: Entity,
    source: ScheduleError,
}

#[derive(Debug, thiserror::Error)]
#[error("IMU noise {entity:?}: {source}")]
struct SensorNoiseFault {
    entity: Entity,
    source: gz_sensors::InvalidNoise,
}

fn sample_imus(
    (time, interest): (Res<UpdateInfo>, Option<Res<ImuInterest>>),
    fault: Res<StepFault>,
    gravity: Res<Gravity>,
    identities: EntityLookup,
    bodies: Query<(&WorldPose, &Velocity)>,
    mut sensors: Query<(Entity, Option<&EntityId>, &mut Imu)>,
    mut commands: Commands,
) {
    if time.dt.is_zero() {
        return;
    }
    let needed = sensors.iter().any(|(_, id, sensor)| {
        let connected = interest.as_ref().is_none_or(|interest| {
            id.is_some_and(|id| interest.0.binary_search(&id.get()).is_ok())
        });
        connected && sensor.schedule.is_due(time.sim_time)
    });
    for (entity, _, mut sensor) in &mut sensors {
        let body = identities
            .get(sensor.link)
            .expect("scene owns IMU link identity");
        let (pose, body_velocity) = bodies.get(body).expect("scene owns IMU link state");
        let velocity = DVec3::from_array(body_velocity.linear);
        let acceleration = (velocity - sensor.previous_velocity) / time.dt.as_secs_f64();
        sensor.previous_velocity = velocity;
        if !needed || !sensor.active {
            continue;
        }
        match sensor.schedule.take_sample(time.sim_time, false) {
            Ok(true) => {}
            Ok(false) => continue,
            Err(source) => {
                fault.report(SensorScheduleFault { entity, source });
                return;
            }
        }
        let rotation = DQuat::from_array(pose.rotation) * sensor.rotation;
        let mut reading = gz_sensors::imu(ImuInput {
            world_orientation: rotation.to_array(),
            reference_orientation: sensor.reference.to_array(),
            acceleration_sensor: (rotation.inverse() * acceleration).to_array(),
            angular_velocity_sensor: (rotation.inverse()
                * DVec3::from_array(body_velocity.angular))
            .to_array(),
            gravity_world: gravity.0,
        });
        if let Ok(reading) = &mut reading
            && let Some(noise) = &mut sensor.noise
            && let Err(source) = noise.apply(reading, time.sim_time, time.iteration)
        {
            fault.report(SensorNoiseFault { entity, source });
            return;
        }
        commands.entity(entity).insert(ImuSample {
            time: time.sim_time,
            epoch: time.epoch,
            reading,
        });
    }
}

fn reset_imus(mut sensors: Query<(Entity, &mut Imu)>, mut commands: Commands) {
    for (entity, mut sensor) in &mut sensors {
        sensor.previous_velocity = sensor.initial_velocity;
        sensor.schedule.reset();
        if let Some(noise) = &mut sensor.noise {
            noise.reset();
        }
        commands.entity(entity).remove::<ImuSample>();
    }
}

#[cfg(test)]
#[path = "../tests/private/observations.rs"]
mod observation_tests;

#[cfg(test)]
#[path = "../tests/private/canonical_publication.rs"]
mod canonical_publication_tests;
