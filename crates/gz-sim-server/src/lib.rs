// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded world-clock publication and control around an explicitly stepped simulation.

mod force_io;
mod forces;
mod imu_io;
mod options;
mod poses;
mod rates;
mod removals;
mod reset;
mod world_io;

pub use options::{ServerOptions, SetupError};

use bevy_ecs::entity::Entity;
use gz_sim_core::{Simulation, StepError, UpdateInfo};
use gz_sim_proto::{
    msgs::{Boolean, Clock, Time, WorldControl},
    prost::Message,
};
use gz_sim_scene::{ImuPublication, ImuSample};
use std::{
    collections::BTreeMap,
    num::NonZeroUsize,
    sync::{Arc, Weak},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use zenoh::Wait;

#[derive(Debug, thiserror::Error)]
pub enum Error {
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
    #[error("IMU publication lacks a unique public identity")]
    ImuIdentity,
    #[error("joint-force controller lacks its original identity, configuration or command")]
    JointForceIdentity,
    #[error(transparent)]
    ImuActivation(#[from] gz_sim_scene::ImuActivationError),
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
}

struct ImuStream {
    entity: Entity,
    metadata: ImuPublication,
    endpoint: String,
    publisher: Arc<world_io::ImuOutput>,
    sequence: u64,
    last_sample: Option<(u64, Duration)>,
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
    rates: rates::Rates,
    forces: forces::Forces,
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
}

impl Server {
    pub fn new(
        simulation: Simulation,
        world_name: &str,
        config: ServerOptions,
        command_budget: NonZeroUsize,
    ) -> Result<Self, Error> {
        Self::build(simulation, world_name, config, None, command_budget)
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
        )
    }

    fn build(
        simulation: Simulation,
        world_name: &str,
        config: ServerOptions,
        zenoh_config: Option<zenoh::Config>,
        command_budget: NonZeroUsize,
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
        counters
    }
    /// Return observed bounded-input losses, independent of remote delivery.
    pub fn ingress_statistics(&self) -> IngressStatistics {
        IngressStatistics {
            queue_drops: self.forces.queue_drops(),
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
        self.simulation.validate_steps(0)?;
        if self.prepared_reset.is_none() {
            self.validate_steps(u64::from(!self.simulation.is_paused()))?;
        }
        let now = Instant::now();
        self.active_time
            .set_running(!self.simulation.is_paused(), now);
        self.refresh_imu_interest();
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
        self.rates.retire(&self.simulation);
        self.forces.retire(&self.simulation);
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
        self.rates.poll(
            &mut self.simulation,
            self.command_budget,
            &mut self.counters,
        )?;
        self.removals
            .poll(&self.world_io, self.command_budget, &mut self.counters)?;
        self.forces.poll(
            &mut self.simulation,
            self.command_budget,
            &mut self.counters,
        )?;
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
        self.rates.shutdown();
        self.forces.shutdown();
        self.imus = Vec::new();
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
