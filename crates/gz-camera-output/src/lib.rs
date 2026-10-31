// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicit, single-camera output. Call only outside the simulation step.
//!
//! This owner does not schedule samples. `request` captures committed state;
//! `pump` advances at most one nonblocking IPC operation. If the worker must be
//! retired, call `retire` outside the critical step: process teardown can wait.

pub mod camera_messages;

use crate::camera_messages::{
    CameraMessageError, IntrinsicValues, camera_info_packet, image_packet,
};
use gz_render_client::{Client, ClientError, SubmitError, TrustedWorker};
use gz_render_protocol::{FailureKind, Reply, Request, Stamp};
use gz_render_scene::{SnapshotError, extract_camera};
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::CameraSensor;
use gz_transport::{Node, Publisher};
use std::{path::PathBuf, time::Duration};

#[derive(Debug, thiserror::Error)]
pub enum CameraOutputError {
    #[error("camera snapshot: {0}")]
    Snapshot(#[from] SnapshotError),
    #[error("camera message: {0}")]
    Message(#[from] CameraMessageError),
    #[error("transport: {0}")]
    Transport(#[from] gz_transport::Error),
    #[error("renderer client: {0:?}")]
    Client(ClientError),
    #[error("renderer rejected the request or returned an unexpected reply")]
    Protocol,
    #[error("renderer failure {kind:?} for request {request_id:?}: {message}")]
    Worker {
        kind: FailureKind,
        request_id: Option<u64>,
        message: String,
    },
    #[error("request identifiers exhausted")]
    IdExhausted,
    #[error("frame ID must not be empty")]
    InvalidFrameId,
    #[error("camera acquisition already in progress")]
    Busy,
    #[error("retire the existing renderer before requesting another acquisition")]
    RetireRequired,
    #[error("camera owner is invalid; drop it before constructing a replacement")]
    OwnerInvalid,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Progress {
    NoDemand,
    InfoPublished,
    Pending,
    Backpressure,
    SnapshotAccepted,
    CaptureSubmitted,
    FrameReady,
    ImagePublished,
    RetireRequired,
}

struct Acquisition {
    stamp: Stamp,
    capture_id: u64,
    phase: Phase,
}

enum Phase {
    Submit(Request),
    SnapshotReply,
    CaptureReply,
    PublishFrame { stamp: Stamp, rgba: Vec<u8> },
}

/// One explicitly constructed sensor output. The caller owns topic and frame
/// naming and trusted executable configuration; these are never read from SDF.
/// Dropping or retiring this owner can wait and must happen outside physics.
pub struct CameraOutput {
    sensor_id: u64,
    bound_entity: bevy_ecs::entity::Entity,
    bound_sensor: CameraSensor,
    origin: [f64; 3],
    frame_id: String,
    worker: TrustedWorker,
    image: Publisher,
    info: Publisher,
    client: Option<Client>,
    acquisition: Option<Acquisition>,
    retiring: bool,
    invalidated: bool,
    next_id: u64,
    sequence: u64,
}

impl CameraOutput {
    /// Return cached image and camera-info interest without a worker query.
    /// The values are advisory and do not acknowledge delivery.
    pub fn demand(&self) -> Result<(bool, bool), CameraOutputError> {
        Ok((
            self.image.connection_snapshot()?,
            self.info.connection_snapshot()?,
        ))
    }

    /// Validate the camera and worker before advertising the (image, info) topics.
    pub fn new(
        sim: &Simulation,
        node: &Node,
        sensor_id: u64,
        origin: [f64; 3],
        frame_id: String,
        topics: (&str, &str),
        worker_path: PathBuf,
    ) -> Result<Self, CameraOutputError> {
        extract_camera(sim, sensor_id, 1, origin)?;
        let bound_entity =
            entity_by_id(sim.world(), sensor_id).ok_or(SnapshotError::MissingCamera(sensor_id))?;
        let bound_sensor = sim
            .world()
            .get::<CameraSensor>(bound_entity)
            .ok_or(SnapshotError::MissingCamera(sensor_id))?
            .clone();
        if frame_id.is_empty() {
            return Err(CameraOutputError::InvalidFrameId);
        }
        let worker =
            TrustedWorker::from_operator_config(worker_path).map_err(CameraOutputError::Client)?;
        let image = node.advertise_typed::<gz_msgs::msgs::Image>(topics.0)?;
        let info = node.advertise_typed::<gz_msgs::msgs::CameraInfo>(topics.1)?;
        Ok(Self {
            sensor_id,
            bound_entity,
            bound_sensor,
            origin,
            frame_id,
            worker,
            image,
            info,
            client: None,
            acquisition: None,
            retiring: false,
            invalidated: false,
            next_id: 1,
            sequence: 0,
        })
    }

    /// Request one sample at the current committed simulation time. Cached
    /// interest is advisory and does not guarantee delivery.
    pub fn request(&mut self, sim: &Simulation) -> Result<Progress, CameraOutputError> {
        if !self.binding_valid(sim) {
            self.invalidated = true;
            self.acquisition = None;
            self.retiring = self.client.is_some();
            return Err(CameraOutputError::OwnerInvalid);
        }
        if self.retiring {
            return Err(CameraOutputError::RetireRequired);
        }
        if self.acquisition.is_some() {
            return Err(CameraOutputError::Busy);
        }
        let (image_demand, info_demand) = self.demand()?;
        if !image_demand && self.client.is_some() {
            self.retiring = true;
            return Ok(Progress::RetireRequired);
        }
        if !image_demand && !info_demand {
            return Ok(Progress::NoDemand);
        }
        let snapshot_id = self.next_id;
        let capture_id = snapshot_id
            .checked_add(1)
            .ok_or(CameraOutputError::IdExhausted)?;
        let next = capture_id
            .checked_add(1)
            .ok_or(CameraOutputError::IdExhausted)?;
        let request = extract_camera(sim, self.sensor_id, snapshot_id, self.origin)?;
        let Request::CalibratedSnapshot {
            stamp, ref camera, ..
        } = request
        else {
            unreachable!()
        };
        let acquisition = Duration::new(stamp.seconds, stamp.nanoseconds);
        let info = camera_info_packet(
            acquisition,
            &self.frame_id,
            IntrinsicValues {
                fx: camera.fx,
                fy: camera.fy,
                cx: camera.cx,
                cy: camera.cy,
            },
        )?;
        if info_demand {
            self.info.publish_typed(&info)?;
        }
        if !image_demand {
            return Ok(Progress::InfoPublished);
        }
        if self.client.is_none() {
            self.client =
                Some(Client::start(self.worker.clone()).map_err(CameraOutputError::Client)?);
        }
        self.next_id = next;
        self.acquisition = Some(Acquisition {
            stamp,
            capture_id,
            phase: Phase::Submit(request),
        });
        Ok(Progress::Pending)
    }

    /// Advance one IPC transition. No renderer startup, waiting or teardown is
    /// performed here. Reset, removal, fault or lost image demand invalidate the
    /// acquisition before any frame can be published.
    pub fn pump(&mut self, sim: &Simulation) -> Result<Progress, CameraOutputError> {
        if !self.binding_valid(sim) {
            self.invalidated = true;
            self.acquisition = None;
            if self.client.is_some() {
                self.retiring = true;
                return Ok(Progress::RetireRequired);
            }
            return Err(CameraOutputError::OwnerInvalid);
        }
        if self.retiring {
            return Ok(Progress::RetireRequired);
        }
        if self.client.is_some()
            && (!self.image.connection_snapshot()?
                || self
                    .acquisition
                    .as_ref()
                    .is_some_and(|a| a.stamp.epoch != sim.epoch()))
        {
            self.acquisition = None;
            self.retiring = true;
            return Ok(Progress::RetireRequired);
        }
        let Some(mut acquisition) = self.acquisition.take() else {
            return Ok(Progress::NoDemand);
        };
        let client = self.client.as_ref().ok_or(CameraOutputError::Protocol)?;
        let snapshot_phase = matches!(
            &acquisition.phase,
            Phase::Submit(Request::CalibratedSnapshot { .. }) | Phase::SnapshotReply
        );
        match acquisition.phase {
            Phase::Submit(request) => match client.try_send(request) {
                Ok(()) => {
                    acquisition.phase = if snapshot_phase {
                        Phase::SnapshotReply
                    } else {
                        Phase::CaptureReply
                    };
                    if !snapshot_phase {
                        self.acquisition = Some(acquisition);
                        return Ok(Progress::CaptureSubmitted);
                    }
                }
                Err(SubmitError::Busy(request) | SubmitError::NotReady(request)) => {
                    acquisition.phase = Phase::Submit(request);
                    self.acquisition = Some(acquisition);
                    return Ok(Progress::Backpressure);
                }
                Err(SubmitError::Closed(_)) => {
                    self.retiring = true;
                    return match client.try_reply() {
                        Err(error) => Err(CameraOutputError::Client(error)),
                        Ok(Some(Reply::Error {
                            kind,
                            request_id,
                            message,
                        })) => Err(CameraOutputError::Worker {
                            kind,
                            request_id,
                            message,
                        }),
                        _ => Err(CameraOutputError::Client(ClientError::Closed)),
                    };
                }
                Err(_) => {
                    self.retiring = true;
                    return Err(CameraOutputError::Protocol);
                }
            },
            Phase::SnapshotReply | Phase::CaptureReply => {
                let reply = match client.try_reply() {
                    Ok(reply) => reply,
                    Err(error) => {
                        self.retiring = true;
                        return Err(CameraOutputError::Client(error));
                    }
                };
                match reply {
                    None | Some(Reply::Ready { .. }) => {
                        acquisition.phase = if snapshot_phase {
                            Phase::SnapshotReply
                        } else {
                            Phase::CaptureReply
                        };
                    }
                    Some(Reply::SnapshotAccepted { request_id })
                        if snapshot_phase
                            && request_id.checked_add(1) == Some(acquisition.capture_id) =>
                    {
                        acquisition.phase = Phase::Submit(Request::Capture {
                            request_id: acquisition.capture_id,
                        });
                        self.acquisition = Some(acquisition);
                        return Ok(Progress::SnapshotAccepted);
                    }
                    Some(Reply::Frame {
                        request_id,
                        stamp,
                        rgba,
                    }) if !snapshot_phase
                        && request_id == acquisition.capture_id
                        && stamp == acquisition.stamp =>
                    {
                        acquisition.phase = Phase::PublishFrame { stamp, rgba };
                        self.acquisition = Some(acquisition);
                        return Ok(Progress::FrameReady);
                    }
                    Some(Reply::Error {
                        kind,
                        request_id,
                        message,
                    }) => {
                        self.retiring = true;
                        return Err(CameraOutputError::Worker {
                            kind,
                            request_id,
                            message,
                        });
                    }
                    Some(Reply::Busy { .. } | Reply::SnapshotBusy { .. }) => {
                        self.retiring = true;
                        return Err(CameraOutputError::Busy);
                    }
                    Some(_) => {
                        self.retiring = true;
                        return Err(CameraOutputError::Protocol);
                    }
                }
            }
            Phase::PublishFrame { stamp, rgba } => {
                let image = image_packet(
                    &rgba,
                    Duration::new(stamp.seconds, stamp.nanoseconds),
                    &self.frame_id,
                    self.sequence,
                )?;
                self.sequence = self.sequence.wrapping_add(1);
                self.image.publish_typed(&image)?;
                return Ok(Progress::ImagePublished);
            }
        }
        self.acquisition = Some(acquisition);
        Ok(Progress::Pending)
    }

    /// Retire the worker outside a latency-sensitive step. A hard kernel stall
    /// may delay the client join indefinitely.
    pub fn retire(&mut self) {
        self.acquisition = None;
        drop(self.client.take());
        self.retiring = false;
    }

    fn binding_valid(&self, sim: &Simulation) -> bool {
        !self.invalidated
            && !sim.is_faulted()
            && entity_by_id(sim.world(), self.sensor_id) == Some(self.bound_entity)
            && sim
                .world()
                .get::<CameraSensor>(self.bound_entity)
                .is_some_and(|current| std::sync::Arc::ptr_eq(&current.0, &self.bound_sensor.0))
    }
}

#[cfg(test)]
#[path = "../tests/private/camera_output.rs"]
mod tests;
