// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded world-clock publication and control around an explicitly stepped simulation.

mod poses;
mod removals;
mod reset;

use bevy_ecs::entity::Entity;
use gz_msgs::{
    Clock, Time,
    msgs::{Boolean, WorldControl},
    prost::Message,
};
use gz_sim_core::{Simulation, StepError, UpdateInfo};
use gz_sim_scene::{ImuPublication, ImuSample};
use gz_transport::{Config, Error as TransportError, Node, Publisher, Service, Statistics};
use std::{
    collections::BTreeMap,
    num::NonZeroUsize,
    sync::{Arc, Weak},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Transport(#[from] TransportError),
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
}

/// Whether publication and loaded IMU acquisition follow known subscriber interest.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum InterestPolicy {
    /// Native behavior; registration hints do not guarantee delivery readiness.
    #[default]
    Subscribers,
    /// Explicit local acquisition/publication, including without subscribers.
    Always,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Counters {
    pub clocks_queued: u64,
    pub clock_failures: u64,
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
}

struct ImuStream {
    entity: Entity,
    metadata: ImuPublication,
    endpoint: String,
    publisher: Arc<Publisher>,
    sequence: u64,
    last_sample: Option<(u64, Duration)>,
}

fn imu_packet(
    metadata: &ImuPublication,
    sample: &ImuSample,
    sequence: u64,
) -> Option<gz_msgs::msgs::Imu> {
    use gz_msgs::{
        msgs::{FloatV, Header, header::Map},
        quaternion, vector3,
    };
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
    Some(gz_msgs::msgs::Imu {
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
/// Construction and shutdown may wait for OS resources; `tick` never waits for IO.
/// Control acknowledgements report acceptance, not completion of requested steps.
pub struct Server {
    simulation: Simulation,
    node: Node,
    publisher: Publisher,
    poses: Publisher,
    pose_rate: poses::PublicationRate,
    interest_policy: InterestPolicy,
    controls: Service,
    removals: removals::Removals,
    command_budget: NonZeroUsize,
    pending_steps: u64,
    pending_reset: bool,
    reset_failed: bool,
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
        config: Config,
        command_budget: NonZeroUsize,
    ) -> Result<Self, Error> {
        time(simulation.elapsed().as_nanos() as i128)?;
        poses::packet(simulation.world(), simulation.elapsed())?;
        if world_name.is_empty() || world_name.contains('/') {
            return Err(TransportError::InvalidName.into());
        }
        let partition = config.partition.clone();
        let namespace = config.namespace.clone();
        let node = Node::new(config)?;
        let publisher = node.advertise(&format!("/world/{world_name}/clock"), "gz.msgs.Clock")?;
        let controls = node.serve(
            &format!("/world/{world_name}/control"),
            "gz.msgs.WorldControl",
            "gz.msgs.Boolean",
        )?;
        let poses = node.advertise(&format!("/world/{world_name}/pose/info"), "gz.msgs.Pose_V")?;
        let removals = removals::Removals::new(&node, world_name)?;
        let mut imus = Vec::new();
        let mut imu_publishers: BTreeMap<String, Weak<Publisher>> = BTreeMap::new();
        for entity in simulation.world().iter_entities() {
            if let Some(metadata) = entity.get::<ImuPublication>() {
                let topic = gz_transport::as_valid_topic(&metadata.topic);
                let key = gz_transport::qualified_name(&partition, &namespace, &topic)?;
                let publisher = match imu_publishers.get(&key).and_then(Weak::upgrade) {
                    Some(publisher) => publisher,
                    None => {
                        let publisher = Arc::new(node.advertise(&topic, "gz.msgs.IMU")?);
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
        Ok(Self {
            simulation,
            node,
            publisher,
            poses,
            pose_rate: poses::PublicationRate::default(),
            interest_policy: InterestPolicy::default(),
            controls,
            removals,
            command_budget,
            pending_steps: 0,
            pending_reset: false,
            reset_failed: false,
            partition,
            namespace,
            active_time: ActiveTime::default(),
            counters: Counters::default(),
            imus,
        })
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
    pub fn counters(&self) -> Counters {
        self.counters
    }
    pub fn transport_statistics(&self) -> Statistics {
        self.node.statistics()
    }
    pub fn pending_steps(&self) -> u64 {
        self.pending_steps
    }

    /// The caller supplies pacing and cancellation outside the simulation's step.
    pub fn tick(&mut self) -> Result<UpdateInfo, Error> {
        if self.reset_failed {
            return Err(Error::ResetStreams);
        }
        self.simulation.validate_steps(0)?;
        if !self.pending_reset {
            self.validate_steps(u64::from(!self.simulation.is_paused()))?;
        }
        let now = Instant::now();
        self.active_time
            .set_running(!self.simulation.is_paused(), now);
        self.refresh_imu_interest();
        let resetting = self.pending_reset;
        let info = if resetting {
            self.reset_scene()?
        } else {
            self.simulation.tick()?
        };
        let real = if resetting {
            Duration::ZERO
        } else {
            self.active_time.elapsed(now)
        };
        let message = clock(info.sim_time, real, SystemTime::now())?;
        if self.publisher.publish(message.encode_to_vec()).is_ok() {
            self.counters.clocks_queued += 1;
        } else {
            self.counters.clock_failures += 1;
        }
        self.publish_poses(info.sim_time);
        self.publish_imus();
        self.removals
            .execute(&mut self.simulation, &mut self.counters);
        self.imus.retain(|stream| {
            self.simulation
                .world()
                .get::<ImuPublication>(stream.entity)
                .is_some()
        });
        if !info.paused && self.pending_steps > 0 {
            self.pending_steps -= 1;
            if self.pending_steps == 0 {
                self.set_paused(true);
            }
        }
        for _ in 0..self.command_budget.get() {
            let request = match self.controls.recv_timeout(Duration::ZERO) {
                Ok(request) => request,
                Err(TransportError::Timeout) => break,
                Err(error) => return Err(error.into()),
            };
            self.respond(request);
        }
        self.removals
            .poll(self.command_budget, &mut self.counters)?;
        Ok(info)
    }

    fn respond(&mut self, request: gz_transport::Request) {
        let accepted = WorldControl::decode(request.data.as_slice())
            .is_ok_and(|control| self.apply_control(control));
        if accepted {
            self.counters.controls_accepted += 1;
        } else {
            self.counters.controls_rejected += 1;
        }
        if request
            .respond(
                Boolean {
                    data: accepted,
                    ..Default::default()
                }
                .encode_to_vec(),
                true,
            )
            .is_err()
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
            self.imus
                .iter()
                .filter_map(|stream| match stream.publisher.connection_snapshot() {
                    Ok(true) => Some(stream.entity),
                    Ok(false) => None,
                    Err(_) => {
                        counters.interest_failures += 1;
                        None
                    }
                }),
        );
    }

    fn publish_poses(&mut self, time: Duration) {
        if self.interest_policy == InterestPolicy::Subscribers {
            match self.poses.connection_snapshot() {
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
                if self.poses.publish(packet.encode_to_vec()).is_ok() {
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
            if stream.publisher.publish(message.encode_to_vec()).is_ok() {
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

    /// Join the IO worker outside stepping. Dropping the server also joins it.
    pub fn shutdown(&mut self) {
        self.node.shutdown();
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
