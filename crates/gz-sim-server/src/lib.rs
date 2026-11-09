// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded world-clock publication and control around an explicitly stepped simulation.

mod air_pressure;
mod air_pressure_io;
mod air_speed;
mod air_speed_io;
mod altimeter;
mod altimeter_io;
mod apply_link_wrench;
mod apply_link_wrench_io;
mod diff_drive;
mod diff_drive_io;
mod force_io;
mod forces;
mod imu_io;
mod joint_controller_io;
mod joint_controllers;
mod joint_state;
mod joint_state_io;
mod lidar;
mod lidar_io;
mod logical_camera;
mod logical_camera_io;
mod magnetometer;
mod magnetometer_io;
mod odometry_publisher;
mod odometry_publisher_io;
mod options;
mod pose_publisher;
mod pose_publisher_io;
mod poses;
mod rates;
mod removals;
mod reset;
mod world_io;

pub use options::{ServerOptions, SetupError, as_valid_topic};

use bevy_ecs::entity::Entity;
use gz_sim_core::{EntityId, Simulation, StepError, UpdateInfo, entity_by_id};
use gz_sim_proto::{
    msgs::{Boolean, Clock, Time, WorldControl},
    prost::Message,
};
use gz_sim_scene::{ImuPublication, ImuSample};
use std::{
    collections::{BTreeMap, BTreeSet},
    num::NonZeroUsize,
    sync::{Arc, Weak},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use zenoh::Wait;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("configured camera Sensors execution requires a camera application owner")]
    CameraOutputRequired,
    #[error("external camera ownership must match exactly one configured loaded camera")]
    CameraIdentity,
    #[error("camera-owned world editing requires a later renderer admission phase")]
    CameraWorldEdit,
    #[error(transparent)]
    Setup(#[from] SetupError),
    #[error("Zenoh world endpoint: {0}")]
    WorldNetwork(String),
    #[error("world Zenoh session is closed")]
    WorldClosed,
    #[error("pending reset requires IO preparation before ticking")]
    IoPreparationRequired,
    #[error(transparent)]
    Step(#[from] StepError),
    #[error("timestamp exceeds the signed protobuf seconds range")]
    TimeRange,
    #[error(transparent)]
    Reset(#[from] gz_sim_scene::SceneResetError),
    #[error("reset handler changed the prepared IMU stream configuration")]
    ResetStreams,
    #[error("entity ID cannot be represented by the uint32 pose protocol")]
    PoseIdRange,
    #[error("pose entity lacks a name, identity or valid parent-relative transform")]
    PoseData,
    #[error("light pose metadata or its native visual hierarchy is invalid")]
    PoseLighting,
    #[error("IMU publication lacks a unique public identity")]
    ImuIdentity,
    #[error("lidar publication lacks its unique supported identity or metadata")]
    LidarIdentity,
    #[error("reset handler changed the prepared lidar stream configuration")]
    ResetLidars,
    #[error("altimeter publication lacks its unique supported identity or metadata")]
    AltimeterIdentity,
    #[error("reset handler changed the prepared altimeter stream configuration")]
    ResetAltimeters,
    #[error("air-pressure publication lacks its unique supported identity or metadata")]
    AirPressureIdentity,
    #[error("air-speed publication lacks its unique supported identity or metadata")]
    AirSpeedIdentity,
    #[error("logical-camera publication lacks its unique supported identity or metadata")]
    LogicalCameraIdentity,
    #[error("reset handler changed the prepared logical-camera streams")]
    ResetLogicalCameras,
    #[error("reset handler changed the prepared air-speed stream configuration")]
    ResetAirSpeeds,
    #[error("pose publication lacks its unique supported identity or metadata")]
    PosePublisherIdentity,
    #[error("ground-truth odometry lacks its unique supported identity or metadata")]
    GroundTruthOdometryIdentity,
    #[error("link-wrench input lacks its unique supported world identity or metadata")]
    ApplyLinkWrenchIdentity,
    #[error(
        "Server template spawn cannot reactivate dormant link-wrench IO; use an admitted Scene or loaded-reset boundary"
    )]
    ApplyLinkWrenchReactivation,
    #[error("reset handler changed the prepared pose publication configuration")]
    ResetPosePublishers,
    #[error("reset handler changed the prepared air-pressure stream configuration")]
    ResetAirPressures,
    #[error("magnetometer publication lacks its unique supported identity or metadata")]
    MagnetometerIdentity,
    #[error("reset handler changed the prepared magnetometer stream configuration")]
    ResetMagnetometers,
    #[error("joint-state publication lacks its unique supported identity or metadata")]
    JointStateIdentity,
    #[error("reset handler changed the prepared joint-state stream configuration")]
    ResetJointStates,
    #[error("joint-force controller lacks its original identity, configuration or command")]
    JointForceIdentity,
    #[error("DiffDrive configuration lacks its original unique model identity or endpoints")]
    DiffDriveIdentity,
    #[error("reset handler changed the prepared DiffDrive configuration")]
    ResetDiffDrive,
    #[error(
        "joint controller lacks its original typed joint identity, metadata or physics binding"
    )]
    JointControllerIdentity,
    #[error("reset handler changed the prepared joint controller identities or configuration")]
    ResetJointControllers,
    #[error(transparent)]
    ImuActivation(#[from] gz_sim_scene::ImuActivationError),
    #[error(transparent)]
    WorldEdit(#[from] gz_sim_scene::WorldEditError),
    #[error(transparent)]
    WorldRemove(#[from] gz_sim_scene::RemoveError),
}

/// Whether pose publication and loaded IMU acquisition follow known subscriber interest.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum InterestPolicy {
    /// Native behavior; registration hints do not guarantee delivery readiness.
    #[default]
    Subscribers,
    /// Explicit local acquisition/publication, including without subscribers.
    Always,
}

/// Locally observed loss at the shared bounded world-input queues.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct IngressStatistics {
    pub queue_drops: u64,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Counters {
    pub ground_truth_odometry_queued: u64,
    pub ground_truth_odometry_covariance_queued: u64,
    pub ground_truth_odometry_tf_queued: u64,
    pub ground_truth_odometry_failures: u64,
    pub ground_truth_odometry_invalid_samples: u64,
    pub wrench_messages_received: u64,
    pub wrench_messages_rejected: u64,
    pub link_wrenches_queued: u64,
    pub link_wrench_clears_queued: u64,
    pub clocks_queued: u64,
    pub clock_failures: u64,
    pub root_clocks_queued: u64,
    /// Failed root subscription snapshots or publication attempts.
    pub root_clock_failures: u64,
    pub controls_accepted: u64,
    pub controls_rejected: u64,
    pub response_failures: u64,
    pub imus_queued: u64,
    pub imu_failures: u64,
    pub invalid_imu_samples: u64,
    pub lidars_queued: u64,
    pub lidar_failures: u64,
    pub invalid_lidar_samples: u64,
    pub altimeters_queued: u64,
    pub altimeter_failures: u64,
    pub invalid_altimeter_samples: u64,
    pub logical_camera_images_queued: u64,
    pub logical_camera_frustums_queued: u64,
    pub logical_camera_image_failures: u64,
    pub logical_camera_frustum_failures: u64,
    pub invalid_logical_camera_samples: u64,
    pub air_speeds_queued: u64,
    pub air_speed_failures: u64,
    pub invalid_air_speed_samples: u64,
    pub pose_publications_queued: u64,
    pub pose_publisher_failures: u64,
    pub invalid_pose_publication_samples: u64,
    pub air_pressures_queued: u64,
    pub air_pressure_failures: u64,
    pub invalid_air_pressure_samples: u64,
    pub magnetometers_queued: u64,
    pub magnetometer_failures: u64,
    pub invalid_magnetometer_samples: u64,
    pub joint_states_queued: u64,
    pub joint_state_failures: u64,
    pub invalid_joint_state_samples: u64,
    pub poses_queued: u64,
    /// Ticks intentionally omitted by the configured wall-time pose publication period.
    pub poses_throttled: u64,
    /// Ticks with pose work omitted because no matching registration is known.
    pub poses_unobserved: u64,
    /// Failed atomic registration reads; a shared endpoint may be read per sensor.
    pub interest_failures: u64,
    pub pose_failures: u64,
    pub invalid_pose_frames: u64,
    pub removals_queued: u64,
    pub removals_rejected: u64,
    pub removals_completed: u64,
    pub removals_failed: u64,
    pub removals_cancelled: u64,
    pub imu_rates_applied: u64,
    pub imu_rates_ignored: u64,
    pub imu_rates_rejected: u64,
    /// Double packets consumed within the joint-force ingress budget.
    pub force_messages_received: u64,
    /// Malformed packets or packets with at least one rejected target.
    pub force_messages_rejected: u64,
    /// Successful persistent-command updates, including shared endpoint fanout.
    pub joint_forces_applied: u64,
    pub diff_drive_messages_received: u64,
    pub diff_drive_messages_rejected: u64,
    pub diff_drive_commands_applied: u64,
    pub odometries_queued: u64,
    pub odometry_failures: u64,
    pub invalid_odometry_samples: u64,
    pub joint_controller_messages_received: u64,
    pub joint_controller_messages_rejected: u64,
    pub joint_controller_commands_applied: u64,
}

struct ImuStream {
    entity: Entity,
    metadata: ImuPublication,
    endpoint: String,
    publisher: Arc<world_io::ImuOutput>,
    sequence: u64,
    last_sample: Option<(u64, Duration)>,
}

/// Resolve the shared world, input and service families before sensor declarations.
/// Include loaded metadata so a removed original cannot collide when reset restores it.
fn sensor_reserved_keys(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<BTreeSet<String>, Error> {
    let mut keys = io.reserved_keys();
    keys.insert(config.topic_key("/clock")?);
    let imu = |keys: &mut BTreeSet<String>, metadata: &ImuPublication| -> Result<(), Error> {
        let topic = options::as_valid_topic(&metadata.topic);
        keys.insert(config.topic_key(&topic)?);
        keys.insert(config.topic_key(&format!("{topic}/set_rate"))?);
        Ok(())
    };
    let drive = |keys: &mut BTreeSet<String>,
                 metadata: &gz_sim_scene::DiffDriveConfiguration|
     -> Result<(), Error> {
        keys.insert(config.topic_key(&metadata.topic)?);
        keys.insert(config.topic_key(&metadata.odom_topic)?);
        Ok(())
    };
    let force = |keys: &mut BTreeSet<String>,
                 metadata: &gz_sim_scene::JointForceConfiguration|
     -> Result<(), Error> {
        let topic = options::as_valid_topic(&format!(
            "/model/{}/joint/{}/cmd_force",
            metadata.model_name(),
            metadata.joint_name(),
        ));
        keys.insert(config.topic_key(&topic)?);
        Ok(())
    };
    for entity in simulation.world().iter_entities() {
        if entity.contains::<gz_sim_scene::PendingResetRemoval>() {
            continue;
        }
        if let Some(metadata) = entity.get::<ImuPublication>() {
            imu(&mut keys, metadata)?;
        }
        if let Some(metadata) = entity.get::<gz_sim_scene::LidarPublication>() {
            keys.insert(config.topic_key(&options::as_valid_topic(&metadata.topic))?);
        }
        if let Some(metadata) = entity.get::<gz_sim_scene::DiffDriveConfiguration>() {
            drive(&mut keys, metadata)?;
        }
        if let Some(metadata) = entity.get::<gz_sim_scene::JointForceConfiguration>() {
            force(&mut keys, metadata)?;
        }
        if let Some(metadata) = entity.get::<gz_sim_scene::JointControllerConfiguration>() {
            keys.insert(config.topic_key(&metadata.topic)?);
        }
    }
    match gz_sim_scene::initial_imu_publications(simulation) {
        Ok(values) => {
            for metadata in values {
                imu(&mut keys, metadata)?;
            }
            for metadata in gz_sim_scene::initial_lidar_publications(simulation)? {
                keys.insert(config.topic_key(&options::as_valid_topic(&metadata.topic))?);
            }
            for metadata in gz_sim_scene::initial_diff_drive_configurations(simulation)? {
                drive(&mut keys, metadata)?;
            }
            for (_, metadata) in gz_sim_scene::initial_joint_force_configurations(simulation)? {
                force(&mut keys, metadata)?;
            }
            for (_, metadata) in gz_sim_scene::initial_joint_controller_configurations(simulation)?
            {
                keys.insert(config.topic_key(&metadata.topic)?);
            }
        }
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
        Err(error) => return Err(error.into()),
    }
    Ok(keys)
}

// Every typed family excludes itself and reserves all live and loaded-original
// peers. A removed peer cannot donate a key that loaded reset will restore.
#[derive(Clone, Copy, PartialEq)]
enum PublicationFamily {
    Altimeter,
    AirPressure,
    AirSpeed,
    PosePublisher,
    GroundTruthOdometry,
    LinkWrench,
    LogicalCamera,
    Magnetometer,
    JointState,
}
// Current and loaded-original roles are both reserved. Identical restored/live
// roles may repeat, but another owner/channel/raw topic may not alias their key.
fn new_system_roles(
    sim: &Simulation,
) -> Result<Vec<(PublicationFamily, u64, usize, String)>, Error> {
    let mut roles = Vec::new();
    let mut odometry = |owner, m: &gz_sim_scene::GroundTruthOdometryPublication| {
        for (channel, topic) in [&m.odom_topic, &m.covariance_topic, &m.tf_topic]
            .into_iter()
            .enumerate()
        {
            roles.push((
                PublicationFamily::GroundTruthOdometry,
                owner,
                channel,
                topic.clone(),
            ));
        }
    };
    for entity in sim
        .world()
        .iter_entities()
        .filter(|e| !e.contains::<gz_sim_scene::PendingResetRemoval>())
    {
        if let Some(m) = entity.get::<gz_sim_scene::GroundTruthOdometryPublication>() {
            odometry(
                entity
                    .get::<EntityId>()
                    .ok_or(Error::GroundTruthOdometryIdentity)?
                    .get(),
                m,
            );
        }
    }
    match gz_sim_scene::initial_ground_truth_odometry_publications(sim) {
        Ok(values) => {
            for (id, m) in values {
                odometry(id, m);
            }
        }
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
        Err(error) => return Err(error.into()),
    }
    let mut wrench = |owner, m: &gz_sim_scene::ApplyLinkWrenchConfiguration| {
        for (channel, topic) in [&m.topic, &m.persistent_topic, &m.clear_topic]
            .into_iter()
            .enumerate()
        {
            roles.push((PublicationFamily::LinkWrench, owner, channel, topic.clone()));
        }
    };
    for entity in sim
        .world()
        .iter_entities()
        .filter(|e| !e.contains::<gz_sim_scene::PendingResetRemoval>())
    {
        if let Some(m) = entity.get::<gz_sim_scene::ApplyLinkWrenchConfiguration>() {
            wrench(
                entity
                    .get::<EntityId>()
                    .ok_or(Error::ApplyLinkWrenchIdentity)?
                    .get(),
                m,
            );
        }
    }
    match gz_sim_scene::initial_apply_link_wrench_configuration(sim) {
        Ok(Some((id, m))) => wrench(id, m),
        Ok(None) => {}
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
        Err(error) => return Err(error.into()),
    }
    Ok(roles)
}
fn new_system_reserved_keys(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
    family: PublicationFamily,
) -> Result<BTreeSet<String>, Error> {
    typed_reserved_keys(sim, io, config, family)
}
fn validate_new_system_topics(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<(), Error> {
    let roles = new_system_roles(sim)?;
    for family in [
        PublicationFamily::GroundTruthOdometry,
        PublicationFamily::LinkWrench,
    ] {
        let mut selected = roles.iter().filter(|role| role.0 == family).peekable();
        if selected.peek().is_none() {
            continue;
        }
        let occupied = typed_reserved_keys(sim, io, config, family)?;
        let mut mapped = BTreeMap::new();
        for (_, owner, channel, raw) in selected {
            let error = || {
                if family == PublicationFamily::GroundTruthOdometry {
                    Error::GroundTruthOdometryIdentity
                } else {
                    Error::ApplyLinkWrenchIdentity
                }
            };
            if options::as_valid_topic(raw) != *raw {
                return Err(error());
            }
            let key = config.topic_key(raw)?;
            if occupied.contains(&key) {
                return Err(error());
            }
            let identity = (*owner, *channel, raw.clone());
            if mapped
                .insert(key, identity.clone())
                .is_some_and(|old| old != identity)
            {
                return Err(error());
            }
        }
    }
    Ok(())
}
fn typed_reserved_keys(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
    own: PublicationFamily,
) -> Result<BTreeSet<String>, Error> {
    let mut keys = sensor_reserved_keys(simulation, io, config)?;
    for entity in simulation.world().iter_entities() {
        if entity.contains::<gz_sim_scene::PendingResetRemoval>() {
            continue;
        }
        let peers = [
            (
                PublicationFamily::AirSpeed,
                entity
                    .get::<gz_sim_scene::AirSpeedPublication>()
                    .map(|m| &m.topic),
            ),
            (
                PublicationFamily::PosePublisher,
                entity
                    .get::<gz_sim_scene::PosePublication>()
                    .map(|m| &m.topic),
            ),
            (
                PublicationFamily::PosePublisher,
                entity
                    .get::<gz_sim_scene::PosePublication>()
                    .and_then(|m| m.static_publication.as_ref())
                    .map(|m| &m.topic),
            ),
            (
                PublicationFamily::LogicalCamera,
                entity
                    .get::<gz_sim_scene::LogicalCameraPublication>()
                    .map(|m| &m.topic),
            ),
            (
                PublicationFamily::LogicalCamera,
                entity
                    .get::<gz_sim_scene::LogicalCameraPublication>()
                    .map(|m| &m.frustum_topic),
            ),
            (
                PublicationFamily::Altimeter,
                entity
                    .get::<gz_sim_scene::AltimeterPublication>()
                    .map(|m| &m.topic),
            ),
            (
                PublicationFamily::AirPressure,
                entity
                    .get::<gz_sim_scene::AirPressurePublication>()
                    .map(|m| &m.topic),
            ),
            (
                PublicationFamily::Magnetometer,
                entity
                    .get::<gz_sim_scene::MagnetometerPublication>()
                    .map(|m| &m.topic),
            ),
            (
                PublicationFamily::JointState,
                entity
                    .get::<gz_sim_scene::JointStatePublication>()
                    .map(|m| &m.topic),
            ),
        ];
        for (family, topic) in peers {
            if family != own
                && let Some(topic) = topic
            {
                keys.insert(config.topic_key(&options::as_valid_topic(topic))?);
            }
        }
    }
    // The initial getters share the loaded-world blueprint lifetime. Do not turn
    // other reconstruction errors into an apparently vacant endpoint.
    macro_rules! initial {
        ($family:ident, $getter:ident) => {
            if own != PublicationFamily::$family {
                match gz_sim_scene::$getter(simulation) {
                    Ok(values) => {
                        for metadata in values {
                            keys.insert(
                                config.topic_key(&options::as_valid_topic(&metadata.topic))?,
                            );
                        }
                    }
                    Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
                    Err(error) => return Err(error.into()),
                }
            }
        };
    }
    initial!(Altimeter, initial_altimeter_publications);
    initial!(AirPressure, initial_air_pressure_publications);
    initial!(AirSpeed, initial_air_speed_publications);
    initial!(Magnetometer, initial_magnetometer_publications);
    if own != PublicationFamily::PosePublisher {
        match gz_sim_scene::initial_pose_publications(simulation) {
            Ok(values) => {
                for (_, metadata) in values {
                    keys.insert(config.topic_key(&options::as_valid_topic(&metadata.topic))?);
                    if let Some(fixed) = &metadata.static_publication {
                        keys.insert(config.topic_key(&options::as_valid_topic(&fixed.topic))?);
                    }
                }
            }
            Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
            Err(error) => return Err(error.into()),
        }
    }
    if own != PublicationFamily::LogicalCamera {
        match gz_sim_scene::initial_logical_camera_publications(simulation) {
            Ok(values) => {
                for metadata in values {
                    keys.insert(config.topic_key(&options::as_valid_topic(&metadata.topic))?);
                    keys.insert(
                        config.topic_key(&options::as_valid_topic(&metadata.frustum_topic))?,
                    );
                }
            }
            Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
            Err(error) => return Err(error.into()),
        }
    }
    if own != PublicationFamily::JointState {
        match gz_sim_scene::initial_joint_state_publications(simulation) {
            Ok(values) => {
                for (_, metadata) in values {
                    keys.insert(config.topic_key(&options::as_valid_topic(&metadata.topic))?);
                }
            }
            Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
            Err(error) => return Err(error.into()),
        }
    }
    for (family, _, _, raw) in new_system_roles(simulation)? {
        if family != own {
            keys.insert(config.topic_key(&options::as_valid_topic(&raw))?);
        }
    }
    Ok(keys)
}
fn altimeter_reserved_keys(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<BTreeSet<String>, Error> {
    typed_reserved_keys(simulation, io, config, PublicationFamily::Altimeter)
}
fn air_pressure_reserved_keys(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<BTreeSet<String>, Error> {
    typed_reserved_keys(simulation, io, config, PublicationFamily::AirPressure)
}
fn air_speed_reserved_keys(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<BTreeSet<String>, Error> {
    typed_reserved_keys(simulation, io, config, PublicationFamily::AirSpeed)
}
fn pose_publisher_reserved_keys(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<BTreeSet<String>, Error> {
    typed_reserved_keys(simulation, io, config, PublicationFamily::PosePublisher)
}

fn logical_camera_reserved_keys(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<BTreeSet<String>, Error> {
    typed_reserved_keys(simulation, io, config, PublicationFamily::LogicalCamera)
}

fn magnetometer_reserved_keys(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<BTreeSet<String>, Error> {
    typed_reserved_keys(simulation, io, config, PublicationFamily::Magnetometer)
}
fn joint_state_reserved_keys(
    simulation: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<BTreeSet<String>, Error> {
    typed_reserved_keys(simulation, io, config, PublicationFamily::JointState)
}

/// Reserve restored as well as current new-family topics before any reset IO.
fn validate_air_speed_and_pose_topics(
    sim: &Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
) -> Result<(), Error> {
    let mut speeds: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|e| !e.contains::<gz_sim_scene::PendingResetRemoval>())
        .filter_map(|e| {
            e.get::<gz_sim_scene::AirSpeedPublication>()
                .map(|m| m.topic.clone())
        })
        .collect();
    match gz_sim_scene::initial_air_speed_publications(sim) {
        Ok(values) => speeds.extend(values.map(|m| m.topic.clone())),
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
        Err(error) => return Err(error.into()),
    }
    if !speeds.is_empty() {
        let occupied = air_speed_reserved_keys(sim, io, config)?;
        for topic in speeds {
            if occupied.contains(&config.topic_key(&options::as_valid_topic(&topic))?) {
                return Err(Error::AirSpeedIdentity);
            }
        }
    }
    let mut poses = Vec::new();
    let mut cameras = Vec::new();
    for entity in sim
        .world()
        .iter_entities()
        .filter(|e| !e.contains::<gz_sim_scene::PendingResetRemoval>())
    {
        if let Some(m) = entity.get::<gz_sim_scene::PosePublication>() {
            poses.push((false, m.topic.clone()));
            if let Some(fixed) = &m.static_publication {
                poses.push((true, fixed.topic.clone()));
            }
        }
        if let Some(m) = entity.get::<gz_sim_scene::LogicalCameraPublication>() {
            cameras.extend([(false, m.topic.clone()), (true, m.frustum_topic.clone())]);
        }
    }
    match gz_sim_scene::initial_pose_publications(sim) {
        Ok(values) => {
            for (_, m) in values {
                poses.push((false, m.topic.clone()));
                if let Some(fixed) = &m.static_publication {
                    poses.push((true, fixed.topic.clone()));
                }
            }
        }
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
        Err(error) => return Err(error.into()),
    }
    match gz_sim_scene::initial_logical_camera_publications(sim) {
        Ok(values) => {
            for m in values {
                cameras.extend([(false, m.topic.clone()), (true, m.frustum_topic.clone())]);
            }
        }
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => {}
        Err(error) => return Err(error.into()),
    }
    // The same loaded-original/live role may repeat, but no other raw endpoint
    // or channel may alias it. Check every new key before the first reset IO.
    for (topics, family) in [
        (poses, PublicationFamily::PosePublisher),
        (cameras, PublicationFamily::LogicalCamera),
    ] {
        if topics.is_empty() {
            continue;
        }
        let occupied = typed_reserved_keys(sim, io, config, family)?;
        let mut mapped = BTreeMap::new();
        for (channel, topic) in topics {
            let key = config.topic_key(&options::as_valid_topic(&topic))?;
            let error = || {
                if family == PublicationFamily::PosePublisher {
                    Error::PosePublisherIdentity
                } else {
                    Error::LogicalCameraIdentity
                }
            };
            if occupied.contains(&key) {
                return Err(error());
            }
            if let Some(prior) = mapped.insert(key, (channel, topic.clone()))
                && prior != (channel, topic)
            {
                return Err(error());
            }
        }
    }
    Ok(())
}

fn imu_packet(
    metadata: &ImuPublication,
    sample: &ImuSample,
    sequence: u64,
) -> Option<gz_sim_proto::msgs::Imu> {
    use gz_sim_proto::msgs::{FloatV, Header, header::Map};
    let reading = sample.reading.as_ref().ok()?;
    let covariance = |diagonal: [f64; 3]| {
        Some(FloatV {
            data: vec![
                diagonal[0] as f32,
                0.0,
                0.0,
                0.0,
                diagonal[1] as f32,
                0.0,
                0.0,
                0.0,
                diagonal[2] as f32,
            ],
            ..Default::default()
        })
    };
    Some(gz_sim_proto::msgs::Imu {
        header: Some(Header {
            stamp: Some(time(sample.time.as_nanos() as i128).ok()?),
            data: vec![
                Map {
                    key: "frame_id".into(),
                    value: vec![metadata.frame_id.clone()],
                },
                Map {
                    key: "seq".into(),
                    value: vec![sequence.to_string()],
                },
            ],
        }),
        entity_name: metadata.entity_name.clone(),
        orientation: metadata
            .enable_orientation
            .then(|| quaternion(reading.orientation)),
        orientation_covariance: covariance([0.0; 3]),
        angular_velocity: Some(vector3(reading.angular_velocity)),
        angular_velocity_covariance: covariance(reading.angular_velocity_covariance),
        linear_acceleration: Some(vector3(reading.specific_force)),
        linear_acceleration_covariance: covariance(reading.linear_acceleration_covariance),
    })
}

fn vector3([x, y, z]: [f64; 3]) -> gz_sim_proto::msgs::Vector3d {
    gz_sim_proto::msgs::Vector3d {
        x,
        y,
        z,
        ..Default::default()
    }
}

fn quaternion([x, y, z, w]: [f64; 4]) -> gz_sim_proto::msgs::Quaternion {
    gz_sim_proto::msgs::Quaternion {
        x,
        y,
        z,
        w,
        ..Default::default()
    }
}

#[derive(Default)]
struct ActiveTime {
    elapsed: Duration,
    started: Option<Instant>,
}

impl ActiveTime {
    fn set_running(&mut self, running: bool, now: Instant) {
        match (self.started, running) {
            (Some(start), false) => {
                self.elapsed += now.duration_since(start);
                self.started = None;
            }
            (None, true) => self.started = Some(now),
            _ => {}
        }
    }

    fn elapsed(&self, now: Instant) -> Duration {
        self.elapsed
            + self
                .started
                .map_or(Duration::ZERO, |start| now.duration_since(start))
    }
}

fn time(nanos: i128) -> Result<Time, Error> {
    Ok(Time {
        sec: (nanos / 1_000_000_000)
            .try_into()
            .map_err(|_| Error::TimeRange)?,
        nsec: (nanos % 1_000_000_000) as i32,
    })
}

fn clock(sim: Duration, real: Duration, system: SystemTime) -> Result<Clock, Error> {
    let system = match system.duration_since(UNIX_EPOCH) {
        Ok(duration) => duration.as_nanos() as i128,
        Err(error) => -(error.duration().as_nanos() as i128),
    };
    Ok(Clock {
        sim: Some(time(sim.as_nanos() as i128)?),
        real: Some(time(real.as_nanos() as i128)?),
        system: Some(time(system)?),
        ..Default::default()
    })
}

/// Owns network endpoints and polls a finite number of controls after each tick.
/// Construction, shutdown and network replies run outside the core simulation step.
/// Control acknowledgements report acceptance, not completion of requested steps.
pub struct Server {
    simulation: Simulation,
    world_io: world_io::WorldIo,
    network_config: ServerOptions,
    root_clock: Option<world_io::RootClock>,
    pose_rate: poses::PublicationRate,
    interest_policy: InterestPolicy,
    removals: removals::Removals,
    rates: Option<rates::Rates>,
    forces: Option<forces::Forces>,
    diff_drive: Option<diff_drive::Stream>,
    joint_controllers: Option<joint_controllers::Controllers>,
    command_budget: NonZeroUsize,
    pending_steps: u64,
    pending_reset: bool,
    prepared_reset: Option<UpdateInfo>,
    reset_failed: bool,
    is_shutdown: bool,
    partition: String,
    namespace: String,
    active_time: ActiveTime,
    counters: Counters,
    imus: Vec<ImuStream>,
    lidars: Vec<lidar::Stream>,
    altimeters: Vec<altimeter::Stream>,
    air_pressures: Vec<air_pressure::Stream>,
    air_speeds: Vec<air_speed::Stream>,
    logical_cameras: Vec<logical_camera::Stream>,
    pose_publishers: Vec<pose_publisher::Stream>,
    ground_truth_odometries: Vec<odometry_publisher::Stream>,
    link_wrenches: Option<apply_link_wrench::ApplyLinkWrenches>,
    magnetometers: Vec<magnetometer::Stream>,
    joint_states: Vec<joint_state::Stream>,
    lidar_viewer_interest: bool,
}

impl Server {
    pub fn new(
        simulation: Simulation,
        world_name: &str,
        config: ServerOptions,
        command_budget: NonZeroUsize,
    ) -> Result<Self, Error> {
        Self::build(simulation, world_name, config, None, command_budget, None)
    }

    /// Supplies an upstream Zenoh configuration without reading ZENOH_CONFIG.
    pub fn with_zenoh_config(
        simulation: Simulation,
        world_name: &str,
        config: ServerOptions,
        zenoh_config: zenoh::Config,
        command_budget: NonZeroUsize,
    ) -> Result<Self, Error> {
        Self::build(
            simulation,
            world_name,
            config,
            Some(zenoh_config),
            command_budget,
            None,
        )
    }

    /// Construct after an external camera application has prepared its actual
    /// output for this one configured public camera ID. This Server owns no
    /// renderer or camera publisher; the caller must maintain that ownership.
    pub fn with_external_camera_output(
        simulation: Simulation,
        world_name: &str,
        config: ServerOptions,
        zenoh_config: zenoh::Config,
        command_budget: NonZeroUsize,
        camera_id: u64,
    ) -> Result<Self, Error> {
        Self::build(
            simulation,
            world_name,
            config,
            Some(zenoh_config),
            command_budget,
            Some(camera_id),
        )
    }

    fn build(
        simulation: Simulation,
        world_name: &str,
        config: ServerOptions,
        zenoh_config: Option<zenoh::Config>,
        command_budget: NonZeroUsize,
        external_camera: Option<u64>,
    ) -> Result<Self, Error> {
        time(simulation.elapsed().as_nanos() as i128)?;
        poses::packet(simulation.world(), simulation.elapsed())?;
        // Like native Gazebo, sanitize the SDF name once for every world endpoint.
        let world_name = options::as_valid_topic(world_name);
        if world_name.is_empty() {
            return Err(SetupError::InvalidName.into());
        }
        let partition = config.partition.clone();
        let namespace = config.namespace.clone();
        config.validate()?;
        let configured = simulation.world().iter_entities().any(|entity| {
            entity.get::<gz_sim_scene::EntityKind>() == Some(&gz_sim_scene::EntityKind::World)
                && entity.contains::<gz_sim_scene::RenderingSensorsConfiguration>()
        });
        let mut cameras = simulation
            .world()
            .iter_entities()
            .filter(|entity| entity.contains::<gz_sim_scene::CameraSensor>());
        match (configured, cameras.next(), external_camera) {
            (true, Some(_), None) => return Err(Error::CameraOutputRequired),
            (true, Some(camera), Some(id))
                if cameras.next().is_none()
                    && camera.get::<gz_sim_scene::EntityKind>()
                        == Some(&gz_sim_scene::EntityKind::Sensor)
                    && camera.get::<EntityId>().map(EntityId::get) == Some(id)
                    && entity_by_id(simulation.world(), id) == Some(camera.id())
                    && camera
                        .get::<gz_sim_scene::CameraSensor>()
                        .is_some_and(|sensor| gz_sim_scene::supports_camera_sensor(&sensor.0)) => {}
            (_, _, Some(_)) => return Err(Error::CameraIdentity),
            _ => {}
        }
        drop(cameras);
        let world_key = |topic: &str| {
            crate::options::qualified_name(
                &partition,
                &namespace,
                config.topic_remap(topic).unwrap_or(topic),
            )
        };
        let clock_key = world_key(&format!("/world/{world_name}/clock"))?;
        let control_key = world_key(&format!("/world/{world_name}/control"))?;
        let poses_key = world_key(&format!("/world/{world_name}/pose/info"))?;
        let world_io = match zenoh_config {
            Some(zenoh_config) => world_io::WorldIo::new_with_config(
                clock_key,
                control_key,
                poses_key,
                config.capacity,
                config.max_message_bytes,
                zenoh_config,
            ),
            None => world_io::WorldIo::new(
                clock_key,
                control_key,
                poses_key,
                config.capacity,
                config.max_message_bytes,
            ),
        }
        .map_err(|error| Error::WorldNetwork(error.to_string()))?;
        let removals = removals::Removals::new(world_io.removal_inputs([
            world_key(&format!("/world/{world_name}/remove"))?,
            world_key(&format!("/world/{world_name}/remove/blocking"))?,
        ])?);
        validate_air_speed_and_pose_topics(&simulation, &world_io, &config)?;
        validate_new_system_topics(&simulation, &world_io, &config)?;
        let mut imus = Vec::new();
        let mut imu_publishers: BTreeMap<String, Weak<world_io::ImuOutput>> = BTreeMap::new();
        for entity in simulation.world().iter_entities() {
            if let Some(metadata) = entity.get::<ImuPublication>() {
                let topic = options::as_valid_topic(&metadata.topic);
                let key = crate::options::qualified_name(
                    &partition,
                    &namespace,
                    config.topic_remap(&topic).unwrap_or(&topic),
                )?;
                let publisher = match imu_publishers.get(&key).and_then(Weak::upgrade) {
                    Some(publisher) => publisher,
                    None => {
                        let publisher = world_io
                            .imu_output(key.clone())
                            .map_err(|error| Error::WorldNetwork(error.to_string()))?;
                        imu_publishers.insert(key.clone(), Arc::downgrade(&publisher));
                        publisher
                    }
                };
                imus.push(ImuStream {
                    entity: entity.id(),
                    metadata: metadata.clone(),
                    endpoint: key,
                    publisher,
                    sequence: 0,
                    last_sample: None,
                });
            }
        }
        let lidars = lidar::streams(&simulation, &world_io, &config, &imus, None)?;
        let (mut rates, rate_setup) = match gz_sim_scene::initial_imu_publications(&simulation) {
            Ok(sensors) => rates::Rates::prepare(&config, sensors, &world_io.ingress_drops)?,
            Err(gz_sim_scene::SceneResetError::MissingInitialState) => rates::Rates::prepare(
                &config,
                imus.iter().map(|stream| &stream.metadata),
                &world_io.ingress_drops,
            )?,
            Err(error) => return Err(error.into()),
        };
        rates.bind(&simulation)?;
        world_io.install_rates(rate_setup)?;
        let rates = rates.into_active();
        let configurations: Vec<_> =
            match gz_sim_scene::initial_joint_force_configurations(&simulation) {
                Ok(values) => values
                    .map(|(id, configuration)| (id, configuration.clone()))
                    .collect(),
                Err(gz_sim_scene::SceneResetError::MissingInitialState) => Vec::new(),
                Err(error) => return Err(error.into()),
            };
        let (mut forces, force_setup) =
            forces::Forces::prepare(&config, configurations, &world_io.ingress_drops)?;
        forces.bind(&simulation)?;
        world_io.install_forces(force_setup)?;
        let forces = forces.into_active();
        let diff_drive = diff_drive::stream(&simulation, &world_io, &config, &imus, &lidars, None)?;
        let controller_metadata = joint_controllers::initial(&simulation)?;
        let joint_controllers =
            joint_controllers::prepare(&simulation, &world_io, &config, &controller_metadata)?;
        let altimeters = if simulation
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<gz_sim_scene::AltimeterPublication>())
        {
            let occupied = altimeter_reserved_keys(&simulation, &world_io, &config)?;
            altimeter::streams(&simulation, &world_io, &config, &occupied, None)?
        } else {
            Vec::new()
        };
        let air_pressures = if simulation
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<gz_sim_scene::AirPressurePublication>())
        {
            let occupied = air_pressure_reserved_keys(&simulation, &world_io, &config)?;
            air_pressure::streams(&simulation, &world_io, &config, &occupied, None)?
        } else {
            Vec::new()
        };
        let magnetometers = if simulation
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<gz_sim_scene::MagnetometerPublication>())
        {
            let occupied = magnetometer_reserved_keys(&simulation, &world_io, &config)?;
            magnetometer::streams(&simulation, &world_io, &config, &occupied, None)?
        } else {
            Vec::new()
        };
        let joint_states = if simulation
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<gz_sim_scene::JointStatePublication>())
        {
            let occupied = joint_state_reserved_keys(&simulation, &world_io, &config)?;
            joint_state::streams(&simulation, &world_io, &config, &occupied, None)?
        } else {
            Vec::new()
        };
        let air_speeds = if simulation
            .world()
            .iter_entities()
            .any(|e| e.contains::<gz_sim_scene::AirSpeedPublication>())
        {
            let occupied = air_speed_reserved_keys(&simulation, &world_io, &config)?;
            air_speed::streams(&simulation, &world_io, &config, &occupied, None)?
        } else {
            Vec::new()
        };
        let logical_cameras = if simulation
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<gz_sim_scene::LogicalCameraPublication>())
        {
            let occupied = logical_camera_reserved_keys(&simulation, &world_io, &config)?;
            logical_camera::streams(&simulation, &world_io, &config, &occupied, None)?
        } else {
            Vec::new()
        };
        let pose_publishers = if simulation
            .world()
            .iter_entities()
            .any(|e| e.contains::<gz_sim_scene::PosePublication>())
        {
            let occupied = pose_publisher_reserved_keys(&simulation, &world_io, &config)?;
            pose_publisher::streams(&simulation, &world_io, &config, &occupied, None)?
        } else {
            Vec::new()
        };
        let ground_truth_odometries = if simulation
            .world()
            .iter_entities()
            .any(|e| e.contains::<gz_sim_scene::GroundTruthOdometryPublication>())
        {
            let occupied = new_system_reserved_keys(
                &simulation,
                &world_io,
                &config,
                PublicationFamily::GroundTruthOdometry,
            )?;
            odometry_publisher::streams(&simulation, &world_io, &config, &occupied, None)?
        } else {
            Vec::new()
        };
        let occupied = new_system_reserved_keys(
            &simulation,
            &world_io,
            &config,
            PublicationFamily::LinkWrench,
        )?;
        let link_wrenches = if let Some((owner, setup)) =
            apply_link_wrench::ApplyLinkWrenches::prepare(
                &simulation,
                &config,
                &occupied,
                &world_io.ingress_drops,
            )? {
            world_io.install_wrenches(setup)?;
            Some(owner)
        } else {
            None
        };
        Ok(Self {
            simulation,
            world_io,
            network_config: config,
            root_clock: None,
            pose_rate: poses::PublicationRate::default(),
            interest_policy: InterestPolicy::default(),
            removals,
            rates,
            forces,
            diff_drive,
            joint_controllers,
            command_budget,
            pending_steps: 0,
            pending_reset: false,
            prepared_reset: None,
            reset_failed: false,
            is_shutdown: false,
            partition,
            namespace,
            active_time: ActiveTime::default(),
            counters: Counters::default(),
            imus,
            lidars,
            altimeters,
            air_pressures,
            air_speeds,
            logical_cameras,
            pose_publishers,
            ground_truth_odometries,
            link_wrenches,
            magnetometers,
            joint_states,
            lidar_viewer_interest: false,
        })
    }

    /// Prepare a run boundary outside stepping. Worker waits share one timeout;
    /// timeout or transport failure is not evidence that the root topic is vacant.
    /// Retry an occupied root topic between runs. An admitted publisher survives reset
    /// and later competing publishers. The result reports ownership, not peer readiness.
    /// A failed refresh retains an admitted publisher for the next preparation attempt.
    pub fn prepare_run(&mut self, timeout: Duration) -> Result<bool, Error> {
        if self.reset_failed {
            return Err(Error::ResetStreams);
        }
        self.simulation.validate_steps(0)?;
        if timeout.is_zero() {
            return Err(SetupError::Timeout.into());
        }
        if Instant::now().checked_add(timeout).is_none() {
            return Err(SetupError::InvalidConfig.into());
        }
        if self.root_clock.is_none() {
            let topic = self
                .network_config
                .topic_remap("/clock")
                .unwrap_or("/clock");
            let key = crate::options::qualified_name(&self.partition, &self.namespace, topic)?;
            self.root_clock = self
                .world_io
                .prepare_root_clock(key, timeout)
                .map_err(|error| Error::WorldNetwork(error.to_string()))?;
        }
        self.world_io
            .clock_interest()
            .map_err(|error| Error::WorldNetwork(error.to_string()))?;
        Ok(self.root_clock.is_some())
    }

    /// Minimum wall-time interval between pose publication attempts; native default is 60 Hz.
    pub fn pose_publication_period(&self) -> Duration {
        self.pose_rate.period()
    }

    /// Selects a pose interval without pacing physics. Zero publishes every tick.
    /// A policy change makes the next frame eligible; simulation resets retain the wall timer.
    pub fn set_pose_publication_period(&mut self, period: Duration) {
        self.pose_rate.set_period(period);
    }

    /// Selects subscriber-driven native behavior or explicit local acquisition.
    /// Changing interest does not reset sampling phase, noise or the pose wall timer.
    pub fn set_interest_policy(&mut self, policy: InterestPolicy) {
        self.interest_policy = policy;
    }

    pub fn interest_policy(&self) -> InterestPolicy {
        self.interest_policy
    }

    pub fn simulation(&self) -> &Simulation {
        &self.simulation
    }

    fn admit_world_edit(&self) -> Result<(), Error> {
        if self.is_shutdown {
            return Err(Error::WorldClosed);
        }
        if self.reset_failed {
            return Err(Error::ResetStreams);
        }
        if self.pending_reset || self.prepared_reset.is_some() {
            return Err(Error::IoPreparationRequired);
        }
        self.simulation.validate_steps(0)?;
        if self
            .simulation
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<gz_sim_scene::CameraSensor>())
        {
            return Err(Error::CameraWorldEdit);
        }
        Ok(())
    }

    /// Clone one startup-prepared standalone model between completed frames.
    /// This never exposes general mutable Simulation access or adds IO streams.
    pub fn spawn_model_template(
        &mut self,
        template: &gz_sim_scene::ModelTemplate,
        pose: gz_sim_scene::WorldPose,
    ) -> Result<u64, Error> {
        self.admit_world_edit()?;
        if !gz_sim_scene::apply_link_wrench_active(&self.simulation)
            && self
                .simulation
                .world()
                .iter_entities()
                .any(|entity| entity.contains::<gz_sim_scene::ApplyLinkWrenchConfiguration>())
        {
            return Err(Error::ApplyLinkWrenchReactivation);
        }
        Ok(gz_sim_scene::spawn_model_template(
            &mut self.simulation,
            template,
            pose,
        )?)
    }

    /// Explicit world-state replacement, separate from controller actuation.
    pub fn set_model_state(
        &mut self,
        id: u64,
        state: gz_sim_scene::ModelState,
    ) -> Result<gz_sim_scene::ModelState, Error> {
        self.admit_world_edit()?;
        Ok(gz_sim_scene::set_model_state(
            &mut self.simulation,
            id,
            state,
        )?)
    }

    /// Remove only a model in the bounded world-edit domain: no registered
    /// sensor/controller endpoints can be introduced or retired through this API.
    pub fn remove_editable_model(&mut self, id: u64) -> Result<Vec<u64>, Error> {
        self.admit_world_edit()?;
        gz_sim_scene::model_state(&self.simulation, id)?;
        let removed = gz_sim_scene::remove_model(&mut self.simulation, id)?;
        if !self.ground_truth_odometries.is_empty() {
            self.retire_ground_truth_odometry();
        }
        if let Some(owner) = &mut self.link_wrenches {
            owner.retire(&self.simulation);
            if owner.is_empty() {
                self.link_wrenches = None;
            }
        }
        Ok(removed)
    }

    /// Explicit application shutdown is terminal for frame admission.
    /// This does not report an unexpected closure of the underlying Session.
    pub fn is_shutdown(&self) -> bool {
        self.is_shutdown
    }

    /// Whether an accepted reset still requires the explicit IO preparation phase.
    /// Terminal shutdown and reset failures have their own admission errors.
    pub fn io_preparation_required(&self) -> bool {
        self.pending_reset
    }

    /// Immutable application naming and bounds data; no runtime or Session handle.
    pub fn options(&self) -> &ServerOptions {
        &self.network_config
    }

    /// Changes local IMU acquisition without altering registrations or its phase.
    /// A committed reset failure rejects further control of the invalid run.
    pub fn set_imu_active(&mut self, id: u64, active: bool) -> Result<(), Error> {
        if self.reset_failed {
            return Err(Error::ResetStreams);
        }
        gz_sim_scene::set_imu_active(&mut self.simulation, id, active)?;
        Ok(())
    }
    pub fn counters(&self) -> Counters {
        let mut counters = self.counters;
        counters.response_failures = counters.response_failures.saturating_add(
            self.world_io
                .reply_failures
                .load(std::sync::atomic::Ordering::Relaxed),
        );
        let failures = &self.world_io.publication_failures;
        counters.clock_failures = counters
            .clock_failures
            .saturating_add(failures[0].load(std::sync::atomic::Ordering::Relaxed));
        counters.pose_failures = counters
            .pose_failures
            .saturating_add(failures[1].load(std::sync::atomic::Ordering::Relaxed));
        counters.imu_failures = counters
            .imu_failures
            .saturating_add(failures[2].load(std::sync::atomic::Ordering::Relaxed));
        counters.root_clock_failures = counters
            .root_clock_failures
            .saturating_add(failures[3].load(std::sync::atomic::Ordering::Relaxed));
        counters.lidar_failures = counters
            .lidar_failures
            .saturating_add(failures[4].load(std::sync::atomic::Ordering::Relaxed));
        counters.odometry_failures = counters
            .odometry_failures
            .saturating_add(failures[5].load(std::sync::atomic::Ordering::Relaxed));
        counters.altimeter_failures = counters
            .altimeter_failures
            .saturating_add(failures[6].load(std::sync::atomic::Ordering::Relaxed));
        counters.air_pressure_failures = counters.air_pressure_failures.saturating_add(
            failures[world_io::AIR_PRESSURE_FAILURE_INDEX]
                .load(std::sync::atomic::Ordering::Relaxed),
        );
        counters.magnetometer_failures = counters.magnetometer_failures.saturating_add(
            failures[world_io::MAGNETOMETER_FAILURE_INDEX]
                .load(std::sync::atomic::Ordering::Relaxed),
        );
        counters.joint_state_failures = counters.joint_state_failures.saturating_add(
            failures[world_io::JOINT_STATE_FAILURE_INDEX]
                .load(std::sync::atomic::Ordering::Relaxed),
        );
        counters.air_speed_failures = counters.air_speed_failures.saturating_add(
            failures[world_io::AIR_SPEED_FAILURE_INDEX].load(std::sync::atomic::Ordering::Relaxed),
        );
        counters.pose_publisher_failures = counters.pose_publisher_failures.saturating_add(
            failures[world_io::POSE_PUBLISHER_FAILURE_INDEX]
                .load(std::sync::atomic::Ordering::Relaxed),
        );
        counters.logical_camera_image_failures =
            counters.logical_camera_image_failures.saturating_add(
                failures[world_io::LOGICAL_CAMERA_IMAGE_FAILURE_INDEX]
                    .load(std::sync::atomic::Ordering::Relaxed),
            );
        counters.logical_camera_frustum_failures =
            counters.logical_camera_frustum_failures.saturating_add(
                failures[world_io::LOGICAL_CAMERA_FRUSTUM_FAILURE_INDEX]
                    .load(std::sync::atomic::Ordering::Relaxed),
            );
        for index in [
            world_io::GROUND_TRUTH_ODOMETRY_FAILURE_INDEX,
            world_io::GROUND_TRUTH_ODOMETRY_COVARIANCE_FAILURE_INDEX,
            world_io::GROUND_TRUTH_ODOMETRY_TF_FAILURE_INDEX,
        ] {
            counters.ground_truth_odometry_failures = counters
                .ground_truth_odometry_failures
                .saturating_add(failures[index].load(std::sync::atomic::Ordering::Relaxed));
        }
        counters
    }
    /// Return observed bounded-input losses, independent of remote delivery.
    pub fn ingress_statistics(&self) -> IngressStatistics {
        IngressStatistics {
            queue_drops: self
                .world_io
                .ingress_drops
                .load(std::sync::atomic::Ordering::Relaxed),
        }
    }
    pub fn pending_steps(&self) -> u64 {
        self.pending_steps
    }

    /// Prepare pending reset resources outside the simulation's critical section.
    /// This phase can wait for IO; the following tick publishes the reset snapshot.
    /// After application shutdown, return `WorldClosed` without preparing a reset.
    pub fn prepare_io(&mut self) -> Result<(), Error> {
        if self.is_shutdown {
            return Err(Error::WorldClosed);
        }
        if self.reset_failed {
            return Err(Error::ResetStreams);
        }
        if self.pending_reset {
            self.prepared_reset = Some(self.reset_scene()?);
        }
        Ok(())
    }

    /// Run a complete frame, including potentially waiting reset preparation.
    /// Use separate `prepare_io` and `tick` calls for a critical ticking section.
    pub fn update(&mut self) -> Result<UpdateInfo, Error> {
        self.prepare_io()?;
        self.tick()
    }

    /// Tick without performing reset setup or waiting for network operations.
    /// Call `prepare_run` at the run boundary and `prepare_io` before ticking.
    /// Application shutdown rejects frames with `WorldClosed` before any state changes.
    pub fn tick(&mut self) -> Result<UpdateInfo, Error> {
        if self.is_shutdown {
            return Err(Error::WorldClosed);
        }
        if self.reset_failed {
            return Err(Error::ResetStreams);
        }
        if self.pending_reset {
            return Err(Error::IoPreparationRequired);
        }
        self.reconcile_joint_controllers()?;
        self.simulation.validate_steps(0)?;
        if self.prepared_reset.is_none() {
            self.validate_steps(u64::from(!self.simulation.is_paused()))?;
        }
        let now = Instant::now();
        self.active_time
            .set_running(!self.simulation.is_paused(), now);
        self.refresh_imu_interest();
        self.refresh_lidar_interest();
        if !self.altimeters.is_empty() {
            self.refresh_altimeter_interest();
        }
        if !self.air_pressures.is_empty() {
            self.refresh_air_pressure_interest();
        }
        if !self.magnetometers.is_empty() {
            self.refresh_magnetometer_interest();
        }
        if !self.air_speeds.is_empty() {
            self.refresh_air_speed_interest();
        }
        if !self.logical_cameras.is_empty() {
            self.refresh_logical_camera_interest();
        }
        let resetting = self.prepared_reset.is_some();
        let info = if let Some(info) = self.prepared_reset.take() {
            info
        } else {
            self.simulation.tick()?
        };
        let real = if resetting {
            Duration::ZERO
        } else {
            self.active_time.elapsed(now)
        };
        let message = clock(info.sim_time, real, SystemTime::now())?;
        let data = message.encode_to_vec();
        let root_data = self.root_clock.as_ref().and_then(|publisher| {
            match (
                publisher
                    .publisher
                    .matching_status()
                    .wait()
                    .map(|status| status.matching()),
                self.world_io.clock_interest(),
            ) {
                (Ok(root), Ok(world)) => (root || world).then(|| data.clone()),
                _ => {
                    self.counters.root_clock_failures += 1;
                    None
                }
            }
        });
        if self.world_io.publish_clock(data).is_ok() {
            self.counters.clocks_queued += 1;
        } else {
            self.counters.clock_failures += 1;
        }
        if let (Some(publisher), Some(data)) = (&self.root_clock, root_data) {
            if self.world_io.publish_root_clock(publisher, data).is_ok() {
                self.counters.root_clocks_queued += 1;
            } else {
                self.counters.root_clock_failures += 1;
            }
        }
        self.publish_poses(info.sim_time);
        self.publish_imus();
        self.publish_lidars();
        if !self.altimeters.is_empty() {
            self.publish_altimeters();
        }
        if !self.air_pressures.is_empty() {
            self.publish_air_pressures();
        }
        if !self.magnetometers.is_empty() {
            self.publish_magnetometers();
        }
        if !self.joint_states.is_empty() {
            self.publish_joint_states();
        }
        if !self.air_speeds.is_empty() {
            self.publish_air_speeds();
        }
        if !self.logical_cameras.is_empty() {
            self.publish_logical_cameras();
        }
        if !self.pose_publishers.is_empty() {
            self.publish_pose_publishers();
        }
        if !self.ground_truth_odometries.is_empty() {
            self.publish_ground_truth_odometry();
        }
        self.publish_odometry();
        self.removals
            .execute(&self.world_io, &mut self.simulation, &mut self.counters);
        self.imus.retain(|stream| {
            self.simulation
                .world()
                .get::<ImuPublication>(stream.entity)
                .is_some()
        });
        if self.imus.is_empty() {
            self.imus = Vec::new();
        }
        self.retire_lidars();
        if !self.altimeters.is_empty() {
            self.retire_altimeters();
        }
        if !self.air_pressures.is_empty() {
            self.retire_air_pressures();
        }
        if !self.magnetometers.is_empty() {
            self.retire_magnetometers();
        }
        if !self.joint_states.is_empty() {
            self.retire_joint_states();
        }
        if !self.air_speeds.is_empty() {
            self.retire_air_speeds();
        }
        if !self.logical_cameras.is_empty() {
            self.retire_logical_cameras();
        }
        if !self.pose_publishers.is_empty() {
            self.retire_pose_publishers();
        }
        if !self.ground_truth_odometries.is_empty() {
            self.retire_ground_truth_odometry();
        }
        if let Some(owner) = &mut self.link_wrenches {
            owner.retire(&self.simulation);
            if owner.is_empty() {
                self.link_wrenches = None;
            }
        }
        self.retire_diff_drive();
        if let Some(rates) = &mut self.rates {
            rates.retire(&self.simulation);
            if rates.is_empty() {
                self.rates = None;
            }
        }
        if let Some(forces) = &mut self.forces {
            forces.retire(&self.simulation);
            if forces.is_empty() {
                self.forces = None;
            }
        }
        if !info.paused && self.pending_steps > 0 {
            self.pending_steps -= 1;
            if self.pending_steps == 0 {
                self.set_paused(true);
            }
        }
        self.counters.controls_rejected = self.counters.controls_rejected.saturating_add(
            self.world_io
                .rejected
                .swap(0, std::sync::atomic::Ordering::Relaxed),
        );
        if self.world_io.session.is_closed() {
            return Err(Error::WorldClosed);
        }
        for _ in 0..self.command_budget.get() {
            let request = match self.world_io.controls.try_recv() {
                Ok(request) => request,
                Err(std::sync::mpsc::TryRecvError::Empty) => break,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    return Err(Error::WorldNetwork("world control receiver closed".into()));
                }
            };
            self.respond(request);
        }
        if let Some(rates) = &mut self.rates {
            rates.poll(
                &mut self.simulation,
                self.command_budget,
                &mut self.counters,
            )?;
        }
        self.removals
            .poll(&self.world_io, self.command_budget, &mut self.counters)?;
        if let Some(forces) = &mut self.forces {
            forces.poll(
                &mut self.simulation,
                self.command_budget,
                &mut self.counters,
            )?;
        }
        if let Some(owner) = &mut self.link_wrenches {
            owner.poll(
                &mut self.simulation,
                self.command_budget,
                &mut self.counters,
            )?;
        }
        self.poll_diff_drive()?;
        self.poll_joint_controllers()?;
        Ok(info)
    }

    fn respond(&mut self, request: world_io::ControlRequest) {
        let accepted = WorldControl::decode(request.payload.as_slice())
            .is_ok_and(|control| self.apply_control(control));
        if accepted {
            self.counters.controls_accepted += 1;
        } else {
            self.counters.controls_rejected += 1;
        }
        let bytes = Boolean {
            data: accepted,
            ..Default::default()
        }
        .encode_to_vec();
        if bytes.len() > self.world_io.max_message_bytes
            || self.world_io.session.is_closed()
            || request.reply(bytes).is_err()
        {
            self.counters.response_failures += 1;
        }
    }

    fn refresh_imu_interest(&mut self) {
        if self.interest_policy == InterestPolicy::Always {
            gz_sim_scene::clear_imu_interest(&mut self.simulation);
            return;
        }
        let counters = &mut self.counters;
        gz_sim_scene::set_imu_interest(
            &mut self.simulation,
            self.imus.iter().filter_map(|stream| {
                match if self.world_io.session.is_closed() {
                    Err("world session is closed")
                } else {
                    stream.publisher.matching()
                } {
                    Ok(true) => Some(stream.entity),
                    Ok(false) => None,
                    Err(_) => {
                        counters.interest_failures += 1;
                        None
                    }
                }
            }),
        );
    }

    fn publish_poses(&mut self, time: Duration) {
        if self.interest_policy == InterestPolicy::Subscribers {
            let interest = if self.world_io.session.is_closed() {
                Err("world session is closed".into())
            } else {
                self.world_io
                    .poses
                    .matching_status()
                    .wait()
                    .map(|status| status.matching())
            };
            match interest {
                Ok(true) => {}
                Ok(false) => {
                    self.counters.poses_unobserved += 1;
                    return;
                }
                Err(_) => {
                    self.counters.interest_failures += 1;
                    return;
                }
            }
        }
        if !self.pose_rate.ready(Instant::now()) {
            self.counters.poses_throttled += 1;
            return;
        }
        match poses::packet(self.simulation.world(), time) {
            Ok(packet) => {
                self.pose_rate.attempted(Instant::now());
                let bytes = packet.encode_to_vec();
                if self.world_io.publish_poses(bytes).is_ok() {
                    self.counters.poses_queued += 1;
                } else {
                    self.counters.pose_failures += 1;
                }
            }
            Err(_) => self.counters.invalid_pose_frames += 1,
        }
    }

    fn publish_imus(&mut self) {
        for stream in &mut self.imus {
            let Some(sample) = self.simulation.world().get::<ImuSample>(stream.entity) else {
                continue;
            };
            let key = (sample.epoch, sample.time);
            if stream.last_sample == Some(key) {
                continue;
            }
            stream.last_sample = Some(key);
            let Some(message) = imu_packet(&stream.metadata, sample, stream.sequence) else {
                self.counters.invalid_imu_samples += 1;
                continue;
            };
            // Native Sensor::AddSequence counts attempts and wraps its unsigned counter.
            stream.sequence = stream.sequence.wrapping_add(1);
            let bytes = message.encode_to_vec();
            if self.world_io.publish_imu(&stream.publisher, bytes).is_ok() {
                self.counters.imus_queued += 1;
            } else {
                self.counters.imu_failures += 1;
            }
        }
    }

    fn validate_steps(&self, count: u64) -> Result<(), Error> {
        self.simulation.validate_steps(count)?;
        let end = self.simulation.elapsed().as_nanos()
            + self.simulation.step_duration().as_nanos() * u128::from(count);
        time(end as i128)?;
        Ok(())
    }

    fn set_paused(&mut self, paused: bool) {
        self.active_time.set_running(!paused, Instant::now());
        self.simulation.set_paused(paused);
    }

    /// Queue an application-owned control outside the critical tick.
    /// Acceptance does not imply reset completion: call prepare_io and tick before
    /// acknowledging the resulting epoch. An in-flight reset cannot be overwritten.
    pub fn apply_local_control(&mut self, control: WorldControl) -> bool {
        if self.is_shutdown
            || self.reset_failed
            || self.pending_reset
            || self.prepared_reset.is_some()
        {
            return false;
        }
        self.apply_control(control)
    }

    fn apply_control(&mut self, control: WorldControl) -> bool {
        if self.simulation.validate_steps(0).is_err() {
            return false;
        }
        if control.seed != 0 || control.run_to_sim_time.is_some() {
            return false;
        }
        let rewind = control
            .reset
            .is_some_and(|reset| reset.all || reset.time_only);
        if rewind
            && (gz_sim_scene::initial_imu_publications(&self.simulation).is_err()
                || self.simulation.validate_reset().is_err())
        {
            return false;
        }
        let count = if control.multi_step != 0 {
            u64::from(control.multi_step)
        } else {
            u64::from(control.step)
        };
        let pending = if control.pause {
            let Some(pending) = self.pending_steps.checked_add(count) else {
                return false;
            };
            let validation = if rewind {
                self.validate_reset_steps(pending.saturating_sub(u64::from(count != 0)))
            } else {
                self.validate_steps(pending)
            };
            if validation.is_err() {
                return false;
            }
            pending
        } else {
            self.pending_steps
        };
        self.pending_steps = pending;
        self.pending_reset = rewind;
        self.set_paused(control.pause && count == 0);
        true
    }

    /// Stop and join all IO workers, returning the final simulation for inspection.
    pub fn into_simulation(mut self) -> Simulation {
        self.shutdown();
        self.simulation
    }

    /// End frame admission and join IO outside stepping. Repeated calls are safe.
    /// `tick`, `update` and `prepare_io` then return `WorldClosed` without changing
    /// the scene or counters. Dropping the server also joins IO; `into_simulation`
    /// can still transfer the preserved scene.
    pub fn shutdown(&mut self) {
        self.is_shutdown = true;
        if let Some(mut owner) = self.link_wrenches.take() {
            owner.shutdown();
        }
        gz_sim_scene::discard_apply_link_wrenches(&mut self.simulation);
        self.ground_truth_odometries = Vec::new();
        if let Some(mut rates) = self.rates.take() {
            rates.shutdown();
        }
        if let Some(mut forces) = self.forces.take() {
            forces.shutdown();
        }
        self.imus = Vec::new();
        self.lidars = Vec::new();
        self.altimeters = Vec::new();
        self.air_pressures = Vec::new();
        self.air_speeds = Vec::new();
        self.logical_cameras = Vec::new();
        self.pose_publishers = Vec::new();
        self.magnetometers = Vec::new();
        self.joint_states = Vec::new();
        self.diff_drive = None;
        self.joint_controllers = None;
        self.world_io.shutdown();
    }
}

#[cfg(test)]
#[path = "../tests/private/server.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/pipeline_benchmark.rs"]
mod pipeline_benchmark;

#[cfg(test)]
#[path = "../tests/private/streams_benchmark.rs"]
mod streams_benchmark;

#[cfg(test)]
#[path = "../tests/private/camera_ownership.rs"]
mod camera_ownership_tests;

#[cfg(test)]
#[path = "../tests/private/m34_m35_endpoints.rs"]
mod m34_m35_endpoint_tests;
