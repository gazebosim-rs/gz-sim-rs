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
use gz_sim_proto::prost::{Message, Name};
use gz_sim_scene::CameraSensor;
use std::{
    collections::BTreeSet,
    num::NonZeroUsize,
    path::PathBuf,
    sync::{Arc, Mutex},
    time::Duration,
};
use zenoh::{Session, Wait, liveliness::LivelinessToken, pubsub::Publisher};

#[derive(Debug, thiserror::Error)]
pub enum CameraOutputError {
    #[error("camera snapshot: {0}")]
    Snapshot(#[from] SnapshotError),
    #[error("camera message: {0}")]
    Message(#[from] CameraMessageError),
    #[error("Zenoh: {0}")]
    Zenoh(#[from] zenoh::Error),
    #[error("Zenoh session is closed")]
    SessionClosed,
    #[error("camera topics must be distinct")]
    DuplicateTopics,
    #[error("camera publication key must be an exact native-qualified partition/topic")]
    InvalidPublicationKey,
    #[error("camera topic is already reserved in this writer scope")]
    AlreadyPublishing,
    #[error("camera writer scope is full")]
    WriterLimit,
    #[error("publication is {size} bytes, exceeding the configured {limit} bytes")]
    PublicationTooLarge { size: usize, limit: usize },
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

/// Focused, bounded admission for camera writers sharing one application owner.
/// Clones share reservations. Drop the final scope after the last camera to
/// release its allocation; this manual API alone does not install ECS demand.
#[derive(Clone)]
pub struct CameraWriterScope {
    entries: Arc<Mutex<BTreeSet<(String, String)>>>,
    max_live_cameras: NonZeroUsize,
}

impl CameraWriterScope {
    pub fn new(max_live_cameras: NonZeroUsize) -> Self {
        Self {
            entries: Arc::new(Mutex::new(BTreeSet::new())),
            max_live_cameras,
        }
    }

    fn reserve(
        &self,
        session: &Session,
        keys: [&str; 2],
    ) -> Result<WriterReservation, CameraOutputError> {
        if keys[0] == keys[1] {
            return Err(CameraOutputError::DuplicateTopics);
        }
        let zid = session.zid().to_string();
        let entries = [(zid.clone(), keys[0].to_owned()), (zid, keys[1].to_owned())];
        let mut held = self
            .entries
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        if entries.iter().any(|entry| held.contains(entry)) {
            return Err(CameraOutputError::AlreadyPublishing);
        }
        if held.len() / 2 >= self.max_live_cameras.get() {
            return Err(CameraOutputError::WriterLimit);
        }
        held.extend(entries.iter().cloned());
        Ok(WriterReservation {
            scope: self.clone(),
            entries,
        })
    }
}

struct WriterReservation {
    scope: CameraWriterScope,
    entries: [(String, String); 2],
}

impl Drop for WriterReservation {
    fn drop(&mut self) {
        let mut held = self
            .scope
            .entries
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        for entry in &self.entries {
            held.remove(entry);
        }
    }
}

/// Explicit application configuration; keys are already qualified native Zenoh
/// keys, e.g. `@/partition@/image`, with nonempty slash-prefixed partition
/// and topic and no wildcards. No environment or name resolver is installed.
pub struct CameraOutputConfig {
    pub origin: [f64; 3],
    pub frame_id: String,
    pub image_key: String,
    pub info_key: String,
    pub worker_path: PathBuf,
    /// Encoded protobuf payload limit; Zenoh framing and attachment are separate.
    pub max_message_bytes: NonZeroUsize,
}

fn publisher_token(
    key: &str,
    session: &Session,
    node: &str,
    message: &str,
) -> Result<String, CameraOutputError> {
    let Some((partition, topic)) = key.strip_prefix('@').and_then(|rest| rest.split_once('@'))
    else {
        return Err(CameraOutputError::InvalidPublicationKey);
    };
    // Empty partitions cannot form the native liveliness expression. Do not
    // silently substitute another partition or normalize caller-owned keys.
    // TopicUtils::{IsValidNamespace,FullyQualifiedName} at the pinned native
    // source rejects these bytes even when Zenoh accepts the expression.
    if !partition.starts_with('/')
        || !topic.starts_with('/')
        || key.len() > usize::from(u16::MAX)
        || [partition, topic].iter().any(|name| {
            *name == "/"
                || name.contains(['~', ' ', '@'])
                || name.contains("//")
                || name.contains(":=")
        })
    {
        return Err(CameraOutputError::InvalidPublicationKey);
    }
    let key = zenoh::key_expr::KeyExpr::try_from(key)
        .map_err(|_| CameraOutputError::InvalidPublicationKey)?;
    if key.is_wild() {
        return Err(CameraOutputError::InvalidPublicationKey);
    }
    // The native process-ID slot carries this owned session's zid; the node
    // UUID belongs to this output. This does not recreate native global Nodes.
    let token = format!(
        "@gz/{}/{}/{node}/{node}/MP/%/%/%/{}/{message}/%/%",
        partition.replace('/', "%"),
        session.zid(),
        topic.replace('/', "%")
    );
    zenoh::key_expr::KeyExpr::try_from(token.as_str())
        .map_err(|_| CameraOutputError::InvalidPublicationKey)?;
    Ok(token)
}

fn publish<M: Message + Name>(
    publisher: &Publisher<'_>,
    message: M,
    limit: NonZeroUsize,
) -> Result<(), CameraOutputError> {
    let size = message.encoded_len();
    if size > limit.get() {
        return Err(CameraOutputError::PublicationTooLarge {
            size,
            limit: limit.get(),
        });
    }
    publisher
        .put(message.encode_to_vec())
        .attachment(M::full_name())
        .wait()?;
    Ok(())
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
    image: Publisher<'static>,
    info: Publisher<'static>,
    _image_token: LivelinessToken,
    _info_token: LivelinessToken,
    max_message_bytes: NonZeroUsize,
    client: Option<Client>,
    acquisition: Option<Acquisition>,
    retiring: bool,
    invalidated: bool,
    next_id: u64,
    sequence: u64,
    session: Session,
    _reservation: WriterReservation,
}

impl CameraOutput {
    /// Return cached image and camera-info interest without a worker query.
    /// The values are advisory and do not acknowledge delivery.
    pub fn demand(&self) -> Result<(bool, bool), CameraOutputError> {
        if self.session.is_closed() {
            return Err(CameraOutputError::SessionClosed);
        }
        // Zenoh 1.8 resolves this ready value directly from local routing tables:
        // there is no network wait or async listener lag. Local drop/rebind is
        // visible immediately; remote discovery remains advisory.
        Ok((
            self.image.matching_status().wait()?.matching(),
            self.info.matching_status().wait()?.matching(),
        ))
    }

    /// Validate the camera and worker before advertising the (image, info) topics.
    pub fn new(
        sim: &Simulation,
        session: &Session,
        scope: &CameraWriterScope,
        sensor_id: u64,
        config: CameraOutputConfig,
    ) -> Result<Self, CameraOutputError> {
        let CameraOutputConfig {
            origin,
            frame_id,
            image_key,
            info_key,
            worker_path,
            max_message_bytes,
        } = config;
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
        if session.is_closed() {
            return Err(CameraOutputError::SessionClosed);
        }
        let node = uuid::Uuid::new_v4().to_string();
        let image_token = publisher_token(
            &image_key,
            session,
            &node,
            &gz_sim_proto::msgs::Image::full_name(),
        )?;
        let info_token = publisher_token(
            &info_key,
            session,
            &node,
            &gz_sim_proto::msgs::CameraInfo::full_name(),
        )?;
        let reservation = scope.reserve(session, [&image_key, &info_key])?;
        let image = session
            .declare_publisher(image_key)
            .congestion_control(zenoh::qos::CongestionControl::Drop)
            .wait()?;
        let info = session
            .declare_publisher(info_key)
            .congestion_control(zenoh::qos::CongestionControl::Drop)
            .wait()?;
        let image_token = session.liveliness().declare_token(image_token).wait()?;
        let info_token = session.liveliness().declare_token(info_token).wait()?;
        Ok(Self {
            sensor_id,
            bound_entity,
            bound_sensor,
            origin,
            frame_id,
            worker,
            image,
            info,
            _image_token: image_token,
            _info_token: info_token,
            max_message_bytes,
            session: session.clone(),
            _reservation: reservation,
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
            publish(&self.info, info, self.max_message_bytes)?;
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
            && (!self.demand()?.0
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
                publish(&self.image, image, self.max_message_bytes)?;
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
