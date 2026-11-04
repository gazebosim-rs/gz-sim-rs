// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical command admission and encoder odometry; physics belongs to Scene.
use super::*;
use crate::diff_drive_io::{Output, Setup};
use gz_sim_proto::msgs::{Header, Odometry, Pose, Twist, header::Map};
use gz_sim_scene::{DiffDriveConfiguration, DiffDriveSample, EntityKind, PendingResetRemoval};
use std::sync::mpsc;

type Metadata = Vec<DiffDriveConfiguration>;

fn valid(metadata: &DiffDriveConfiguration) -> bool {
    !metadata.model_name.is_empty()
        && !metadata.frame_id.is_empty()
        && !metadata.child_frame_id.is_empty()
        && [&metadata.topic, &metadata.odom_topic]
            .into_iter()
            .all(|topic| topic.starts_with('/') && options::as_valid_topic(topic) == *topic)
        && metadata.topic != metadata.odom_topic
}
fn packet(
    metadata: &DiffDriveConfiguration,
    sample: &DiffDriveSample,
    sequence: u64,
) -> Option<Odometry> {
    if !valid(metadata)
        || !sample
            .pose
            .into_iter()
            .chain(sample.twist)
            .all(f64::is_finite)
    {
        return None;
    }
    let (sin, cos) = (sample.pose[2] * 0.5).sin_cos();
    Some(Odometry {
        header: Some(Header {
            stamp: Some(time(sample.time.as_nanos() as i128).ok()?),
            data: vec![
                Map {
                    key: "frame_id".into(),
                    value: vec![metadata.frame_id.clone()],
                },
                Map {
                    key: "child_frame_id".into(),
                    value: vec![metadata.child_frame_id.clone()],
                },
                Map {
                    key: "seq".into(),
                    value: vec![sequence.to_string()],
                },
            ],
        }),
        pose: Some(Pose {
            position: Some(vector3([sample.pose[0], sample.pose[1], 0.0])),
            orientation: Some(quaternion([0.0, 0.0, sin, cos])),
            ..Default::default()
        }),
        twist: Some(Twist {
            linear: Some(vector3([sample.twist[0], 0.0, 0.0])),
            angular: Some(vector3([0.0, 0.0, sample.twist[1]])),
            ..Default::default()
        }),
    })
}

pub(super) fn initial(sim: &Simulation) -> Result<Metadata, Error> {
    let values: Metadata = match gz_sim_scene::initial_diff_drive_configurations(sim) {
        Ok(values) => values.cloned().collect(),
        Err(gz_sim_scene::SceneResetError::MissingInitialState) => Vec::new(),
        Err(error) => return Err(error.into()),
    };
    if values.len() > 1 || values.iter().any(|value| !valid(value)) {
        return Err(Error::DiffDriveIdentity);
    }
    Ok(values)
}

pub(super) struct Stream {
    entity: Entity,
    identity: u64,
    metadata: DiffDriveConfiguration,
    output: Arc<Output>,
    receiver: mpsc::Receiver<crate::diff_drive_io::Command>,
    sequence: u64,
    last_sample: Option<(u64, Duration)>,
}
impl Stream {
    fn live(&self, sim: &Simulation) -> bool {
        let world = sim.world();
        world.get::<EntityKind>(self.entity) == Some(&EntityKind::Model)
            && world.get::<DiffDriveConfiguration>(self.entity) == Some(&self.metadata)
            && world.get::<EntityId>(self.entity).map(EntityId::get) == Some(self.identity)
            && entity_by_id(world, self.identity) == Some(self.entity)
            && !world.entity(self.entity).contains::<PendingResetRemoval>()
    }
}

pub(super) fn stream(
    sim: &Simulation,
    io: &world_io::WorldIo,
    options: &ServerOptions,
    imus: &[ImuStream],
    lidars: &[lidar::Stream],
    expected: Option<&Metadata>,
) -> Result<Option<Stream>, Error> {
    let mut owners = sim.world().iter_entities().filter(|entity| {
        entity.contains::<DiffDriveConfiguration>() && !entity.contains::<PendingResetRemoval>()
    });
    let first = owners.next();
    if owners.next().is_some() {
        return Err(Error::DiffDriveIdentity);
    }
    let Some(entity) = first else {
        if expected.is_some_and(|expected| !expected.is_empty()) {
            return Err(Error::ResetDiffDrive);
        }
        return Ok(None);
    };
    let metadata = entity
        .get::<DiffDriveConfiguration>()
        .expect("selected configured owner");
    let id = entity
        .get::<EntityId>()
        .ok_or(Error::DiffDriveIdentity)?
        .get();
    if !valid(metadata)
        || entity.get::<EntityKind>() != Some(&EntityKind::Model)
        || entity_by_id(sim.world(), id) != Some(entity.id())
    {
        return Err(Error::DiffDriveIdentity);
    }
    if expected.is_some_and(|expected| expected.as_slice() != std::slice::from_ref(metadata)) {
        return Err(Error::ResetDiffDrive);
    }
    let command_key = options.topic_key(&metadata.topic)?;
    let odometry_key = options.topic_key(&metadata.odom_topic)?;
    if [&command_key, &odometry_key].into_iter().any(|key| {
        imus.iter().any(|stream| stream.endpoint == *key)
            || lidars.iter().any(|stream| stream.endpoint == *key)
    }) {
        return Err(Error::DiffDriveIdentity);
    }
    let root_clock = options.topic_key("/clock")?;
    if command_key == root_clock || odometry_key == root_clock {
        return Err(Error::DiffDriveIdentity);
    }
    let (input, receiver) = mpsc::sync_channel(options.capacity);
    let output = io.diff_drive_output(Setup {
        command_key,
        odometry_key,
        input,
        max_bytes: options.max_message_bytes,
        drops: Arc::clone(&io.ingress_drops),
        epoch: Arc::clone(&io.epoch),
    })?;
    Ok(Some(Stream {
        entity: entity.id(),
        identity: id,
        metadata: metadata.clone(),
        output,
        receiver,
        sequence: 0,
        last_sample: None,
    }))
}

impl Server {
    pub(super) fn retire_diff_drive(&mut self) {
        if self
            .diff_drive
            .as_ref()
            .is_some_and(|stream| !stream.live(&self.simulation))
        {
            self.diff_drive = None;
        }
    }
    pub(super) fn poll_diff_drive(&mut self) -> Result<(), Error> {
        let Some(stream) = &self.diff_drive else {
            return Ok(());
        };
        for _ in 0..self.command_budget.get() {
            let message = match stream.receiver.try_recv() {
                Ok(message) => message,
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => return Err(Error::WorldClosed),
            };
            self.counters.diff_drive_messages_received += 1;
            let accepted = message.0 == self.simulation.epoch()
                && stream.live(&self.simulation)
                && message.1.is_some_and(|[linear, angular]| {
                    gz_sim_scene::set_diff_drive_command(
                        &mut self.simulation,
                        stream.identity,
                        linear,
                        angular,
                    )
                    .is_ok()
                });
            if accepted {
                self.counters.diff_drive_commands_applied += 1;
            } else {
                self.counters.diff_drive_messages_rejected += 1;
            }
        }
        Ok(())
    }
    pub(super) fn publish_odometry(&mut self) {
        let Some(stream) = &mut self.diff_drive else {
            return;
        };
        if !stream.live(&self.simulation) {
            return;
        }
        let Some(sample) = self
            .simulation
            .world()
            .get::<DiffDriveSample>(stream.entity)
        else {
            return;
        };
        let key = (sample.epoch, sample.time);
        if stream.last_sample == Some(key) {
            return;
        }
        stream.last_sample = Some(key);
        let Some(message) = (sample.epoch == self.simulation.epoch()
            && sample.time <= self.simulation.elapsed())
        .then(|| packet(&stream.metadata, sample, stream.sequence))
        .flatten() else {
            self.counters.invalid_odometry_samples += 1;
            return;
        };
        let interested = if self.interest_policy == InterestPolicy::Always {
            true
        } else {
            match stream.output.matching() {
                Ok(value) => value,
                Err(_) => {
                    self.counters.interest_failures += 1;
                    return;
                }
            }
        };
        if !interested {
            return;
        }
        stream.sequence = stream.sequence.wrapping_add(1);
        if self
            .world_io
            .publish_odometry(&stream.output, message.encode_to_vec())
            .is_ok()
        {
            self.counters.odometries_queued += 1;
        } else {
            self.counters.odometry_failures += 1;
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/diff_drive.rs"]
mod tests;
