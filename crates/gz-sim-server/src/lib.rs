// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded world-clock publication and control around an explicitly stepped simulation.

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
    num::NonZeroUsize,
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
}

struct ImuStream {
    entity: Entity,
    metadata: ImuPublication,
    publisher: Publisher,
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
    let covariance = || {
        Some(FloatV {
            data: vec![0.0; 9],
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
        orientation: Some(quaternion(reading.orientation)),
        orientation_covariance: covariance(),
        angular_velocity: Some(vector3(reading.angular_velocity)),
        angular_velocity_covariance: covariance(),
        linear_acceleration: Some(vector3(reading.specific_force)),
        linear_acceleration_covariance: covariance(),
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
    controls: Service,
    command_budget: NonZeroUsize,
    pending_steps: u64,
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
        if world_name.is_empty() || world_name.contains('/') {
            return Err(TransportError::InvalidName.into());
        }
        let node = Node::new(config)?;
        let publisher = node.advertise(&format!("/world/{world_name}/clock"), "gz.msgs.Clock")?;
        let controls = node.serve(
            &format!("/world/{world_name}/control"),
            "gz.msgs.WorldControl",
            "gz.msgs.Boolean",
        )?;
        let mut imus = Vec::new();
        for entity in simulation.world().iter_entities() {
            if let Some(metadata) = entity.get::<ImuPublication>() {
                imus.push(ImuStream {
                    entity: entity.id(),
                    metadata: metadata.clone(),
                    publisher: node.advertise(
                        &gz_transport::as_valid_topic(&metadata.topic),
                        "gz.msgs.IMU",
                    )?,
                    sequence: 0,
                    last_sample: None,
                });
            }
        }
        Ok(Self {
            simulation,
            node,
            publisher,
            controls,
            command_budget,
            pending_steps: 0,
            active_time: ActiveTime::default(),
            counters: Counters::default(),
            imus,
        })
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
        self.validate_steps(u64::from(!self.simulation.is_paused()))?;
        let now = Instant::now();
        self.active_time
            .set_running(!self.simulation.is_paused(), now);
        let real = self.active_time.elapsed(now);
        let info = self.simulation.tick()?;
        let message = clock(info.sim_time, real, SystemTime::now())?;
        if self.publisher.publish(message.encode_to_vec()).is_ok() {
            self.counters.clocks_queued += 1;
        } else {
            self.counters.clock_failures += 1;
        }
        self.publish_imus();
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
        if control.seed != 0
            || control.run_to_sim_time.is_some()
            || control
                .reset
                .is_some_and(|reset| reset.all || reset.time_only || reset.model_only)
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
            if self.validate_steps(pending).is_err() {
                return false;
            }
            pending
        } else {
            self.pending_steps
        };
        self.pending_steps = pending;
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
