// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
mod exit_deadline;

use exit_deadline::ExitDeadline;
use glam::{DQuat, DVec3};
use gz_render_frame::{
    WorldPose,
    camera::{Calibration, CameraConfig},
};
use gz_render_protocol::{
    Camera, FailureKind, PROTOCOL_VERSION, Reply, Request, Stamp, Visual, read_request, write_reply,
};
use gz_renderer::{BoxVisual, DepthFrame, Frame, RenderError, Renderer, Snapshot, UpdateStatus};
use rustix::process::{Signal, getpid, getppid, set_parent_process_death_signal};
use std::{
    io::{self, BufReader, BufWriter, Write},
    sync::mpsc::{self, RecvTimeoutError},
    time::Duration,
};

// All wire snapshot variants use the same ordering and lifecycle boundary.
enum WorkerRequest {
    Snapshot {
        request_id: u64,
        stamp: Stamp,
        origin: [f64; 3],
        visuals: Vec<Visual>,
        camera: Option<Box<Camera>>,
        depth: bool,
    },
    Capture {
        request_id: u64,
    },
    Cancel {
        request_id: u64,
    },
    Shutdown,
}
impl From<Request> for WorkerRequest {
    fn from(request: Request) -> Self {
        match request {
            Request::Snapshot {
                request_id,
                stamp,
                origin,
                visuals,
            } => Self::Snapshot {
                request_id,
                stamp,
                origin,
                visuals,
                camera: None,
                depth: false,
            },
            Request::CalibratedSnapshot {
                request_id,
                stamp,
                origin,
                visuals,
                camera,
            } => Self::Snapshot {
                request_id,
                stamp,
                origin,
                visuals,
                camera: Some(camera),
                depth: false,
            },
            Request::CalibratedDepthSnapshot {
                request_id,
                stamp,
                origin,
                visuals,
                camera,
            } => Self::Snapshot {
                request_id,
                stamp,
                origin,
                visuals,
                camera: Some(camera),
                depth: true,
            },
            Request::Capture { request_id } => Self::Capture { request_id },
            Request::Cancel { request_id } => Self::Cancel { request_id },
            Request::Shutdown => Self::Shutdown,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Capture {
    request_id: u64,
    cancelled: bool,
}

enum ReadbackOutcome {
    Pending,
    Complete(Reply),
    Failed(Reply),
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum CaptureKind {
    #[default]
    Rgba,
    Depth,
}

#[derive(Default)]
struct CaptureSlot(Option<Capture>, CaptureKind);
impl CaptureSlot {
    fn begin(&mut self, request_id: u64) -> bool {
        if self.0.is_some() {
            return false;
        }
        self.0 = Some(Capture {
            request_id,
            cancelled: false,
        });
        self.1 = CaptureKind::Rgba;
        true
    }
    fn begin_with_kind(&mut self, request_id: u64, kind: CaptureKind) -> bool {
        if !self.begin(request_id) {
            return false;
        }
        self.1 = kind;
        true
    }
    fn kind(&self) -> CaptureKind {
        self.1
    }
    fn cancel(&mut self, request_id: u64) -> bool {
        match &mut self.0 {
            Some(capture) if capture.request_id == request_id => {
                capture.cancelled = true;
                true
            }
            _ => false,
        }
    }
    fn snapshot_replaced(&mut self) {
        if let Some(capture) = &mut self.0 {
            capture.cancelled = true;
        }
    }
    fn active(&self) -> Option<Capture> {
        self.0
    }
    fn clear(&mut self) {
        self.0 = None;
        self.1 = CaptureKind::Rgba;
    }
    fn readback(
        &mut self,
        result: Result<Option<Frame>, RenderError>,
        in_flight: bool,
    ) -> ReadbackOutcome {
        let capture = self.0.expect("readback requires an active capture");
        let reply = result.map(|frame| {
            frame.map(|frame| Reply::Frame {
                request_id: capture.request_id,
                stamp: Stamp {
                    epoch: frame.epoch,
                    seconds: frame.time.as_secs(),
                    nanoseconds: frame.time.subsec_nanos(),
                },
                rgba: frame.rgba,
            })
        });
        self.finish_readback(reply, in_flight)
    }
    fn readback_depth(
        &mut self,
        result: Result<Option<DepthFrame>, RenderError>,
        in_flight: bool,
    ) -> ReadbackOutcome {
        let capture = self.0.expect("readback requires an active capture");
        let reply = result.map(|frame| {
            frame.map(|frame| Reply::DepthFrame {
                request_id: capture.request_id,
                stamp: Stamp {
                    epoch: frame.epoch,
                    seconds: frame.time.as_secs(),
                    nanoseconds: frame.time.subsec_nanos(),
                },
                metres_le: frame
                    .metres
                    .into_iter()
                    .flat_map(f32::to_le_bytes)
                    .collect(),
            })
        });
        self.finish_readback(reply, in_flight)
    }
    fn finish_readback(
        &mut self,
        result: Result<Option<Reply>, RenderError>,
        in_flight: bool,
    ) -> ReadbackOutcome {
        let capture = self.0.expect("readback requires an active capture");
        let outcome = match result {
            Ok(Some(reply)) => ReadbackOutcome::Complete(reply),
            Ok(None) if capture.cancelled && !in_flight => {
                ReadbackOutcome::Complete(Reply::Cancelled {
                    request_id: capture.request_id,
                })
            }
            Ok(None) => return ReadbackOutcome::Pending,
            Err(error) => ReadbackOutcome::Failed(failure(
                FailureKind::Readback,
                Some(capture.request_id),
                error,
            )),
        };
        self.clear();
        outcome
    }
}

enum SnapshotFailure {
    Busy,
    Invalid(String),
}
impl From<RenderError> for SnapshotFailure {
    fn from(error: RenderError) -> Self {
        match error {
            RenderError::SnapshotBusy => Self::Busy,
            other => Self::Invalid(other.to_string()),
        }
    }
}

fn snapshot_reply(
    request_id: u64,
    stamp: Stamp,
    result: Result<(), SnapshotFailure>,
    last_accepted_stamp: &mut Option<Stamp>,
    scene_needs_update: &mut bool,
    capture: &mut CaptureSlot,
) -> Reply {
    match result {
        Ok(()) => {
            *last_accepted_stamp = Some(stamp);
            *scene_needs_update = true;
            capture.snapshot_replaced();
            Reply::SnapshotAccepted { request_id }
        }
        Err(SnapshotFailure::Busy) => Reply::SnapshotBusy { request_id },
        Err(SnapshotFailure::Invalid(error)) => {
            failure(FailureKind::InvalidSnapshot, Some(request_id), error)
        }
    }
}

fn send(out: &mut impl Write, reply: Reply) -> io::Result<()> {
    write_reply(out, &reply)?;
    out.flush()
}
fn send_or_exit(out: &mut impl Write, reply: Reply, exit: &ExitDeadline) {
    if let Err(error) = send(out, reply) {
        // Arm before stderr or C/LLVM exit handlers can block. The renderer is
        // process-owned: a broken reply channel cannot wait for its destruction.
        exit.arm();
        let _ = writeln!(io::stderr().lock(), "Error: {error:?}");
        std::process::exit(1);
    }
}

fn failure(kind: FailureKind, request_id: Option<u64>, message: impl ToString) -> Reply {
    let mut message = message.to_string();
    if message.len() > 1024 {
        let mut end = 1024;
        while !message.is_char_boundary(end) {
            end -= 1;
        }
        message.truncate(end);
    }
    Reply::Error {
        kind,
        request_id,
        message,
    }
}
fn update_failure(request_id: Option<u64>, error: RenderError) -> Reply {
    let kind = if matches!(error, RenderError::Shader(_)) {
        FailureKind::Shader
    } else {
        FailureKind::Device
    };
    failure(kind, request_id, error)
}
fn apply_update(
    result: Result<UpdateStatus, RenderError>,
    request_id: Option<u64>,
    scene_needs_update: &mut bool,
) -> Result<(), Reply> {
    match result {
        Ok(UpdateStatus::Advanced) => *scene_needs_update = false,
        Ok(UpdateStatus::Waiting) => {}
        Err(error) => return Err(update_failure(request_id, error)),
    }
    Ok(())
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let parent: i32 = std::env::args()
        .nth(1)
        .ok_or("parent pid required")?
        .parse()?;
    set_parent_process_death_signal(Some(Signal::KILL))?;
    if getppid().map(|pid| pid.as_raw_nonzero().get()) != Some(parent) {
        return Err("parent already gone".into());
    }
    let mut output = BufWriter::new(io::stdout().lock());
    let (tx, rx) = mpsc::sync_channel(1);
    let input_thread = std::thread::Builder::new()
        .name("render-input".into())
        .spawn(move || {
            let mut input = BufReader::new(io::stdin().lock());
            loop {
                match read_request(&mut input) {
                    Ok(Some(request)) => {
                        if tx.send(Ok(request)).is_err() {
                            break;
                        }
                    }
                    Ok(None) => break,
                    Err(error) => {
                        let _ = tx.send(Err(error));
                        break;
                    }
                }
            }
        });
    if let Err(error) = input_thread {
        send(&mut output, failure(FailureKind::Startup, None, error))?;
        return Err("input thread startup failed".into());
    }
    let exit_deadline = match ExitDeadline::new(getpid()) {
        Ok(exit) => exit,
        Err(error) => {
            send(&mut output, failure(FailureKind::Startup, None, error))?;
            return Err("exit deadline thread startup failed".into());
        }
    };
    let mut renderer = match Renderer::new() {
        Ok(renderer) => renderer,
        Err(error) => {
            send_or_exit(
                &mut output,
                failure(FailureKind::Startup, None, error),
                &exit_deadline,
            );
            return Err("renderer startup failed".into());
        }
    };
    send_or_exit(
        &mut output,
        Reply::Ready {
            version: PROTOCOL_VERSION,
        },
        &exit_deadline,
    );
    let mut capture = CaptureSlot::default();
    let mut scene_needs_update = false;
    let mut snapshot_kind = CaptureKind::Rgba;
    let mut last_snapshot_id = 0_u64;
    let mut last_accepted_stamp: Option<Stamp> = None;
    'worker: loop {
        let request = if capture.active().is_some() || scene_needs_update {
            match rx.recv_timeout(Duration::from_millis(2)) {
                Ok(value) => Some(value),
                Err(RecvTimeoutError::Timeout) => None,
                Err(RecvTimeoutError::Disconnected) => break,
            }
        } else {
            match rx.recv() {
                Ok(value) => Some(value),
                Err(_) => break,
            }
        };
        if let Some(request) = request {
            let request = match request {
                Ok(request) => request,
                Err(_) => {
                    send_or_exit(
                        &mut output,
                        failure(FailureKind::Protocol, None, "invalid protocol message"),
                        &exit_deadline,
                    );
                    return Err("invalid protocol message".into());
                }
            };
            let (reply, skip_update) = 'dispatch: {
                match WorkerRequest::from(request) {
                    WorkerRequest::Snapshot {
                        request_id,
                        stamp,
                        origin,
                        visuals,
                        camera,
                        depth,
                    } => {
                        if request_id <= last_snapshot_id {
                            break 'dispatch (
                                Some(failure(
                                    FailureKind::StaleRequest,
                                    Some(request_id),
                                    "stale snapshot request",
                                )),
                                true,
                            );
                        }
                        last_snapshot_id = request_id;
                        if last_accepted_stamp.is_some_and(|previous| stamp < previous) {
                            break 'dispatch (
                                Some(failure(
                                    FailureKind::StaleRequest,
                                    Some(request_id),
                                    "stale snapshot time",
                                )),
                                true,
                            );
                        }
                        let snapshot = Snapshot {
                            epoch: stamp.epoch,
                            time: Duration::new(stamp.seconds, stamp.nanoseconds),
                            origin: DVec3::from_array(origin),
                            visuals: visuals
                                .into_iter()
                                .map(|v| BoxVisual {
                                    id: v.id,
                                    pose: WorldPose {
                                        translation: DVec3::from_array(v.translation),
                                        rotation: DQuat::from_array(v.rotation),
                                    },
                                    size: DVec3::from_array(v.size),
                                    color: v.color,
                                })
                                .collect(),
                        };
                        let result = if let Some(camera) = camera {
                            let config = CameraConfig {
                                width: camera.width,
                                height: camera.height,
                                fx: camera.fx,
                                fy: camera.fy,
                                cx: camera.cx,
                                cy: camera.cy,
                                skew: camera.skew,
                                tx: camera.tx,
                                ty: camera.ty,
                                near: camera.near,
                                far: camera.far,
                            };
                            match Calibration::new(config) {
                                Ok(calibration) => {
                                    let pose = WorldPose {
                                        translation: DVec3::from_array(camera.translation),
                                        rotation: DQuat::from_array(camera.rotation),
                                    };
                                    if depth {
                                        renderer.set_calibrated_depth_snapshot(
                                            &snapshot,
                                            calibration,
                                            pose,
                                        )
                                    } else {
                                        renderer.set_calibrated_snapshot(
                                            &snapshot,
                                            calibration,
                                            pose,
                                        )
                                    }
                                    .map_err(SnapshotFailure::from)
                                }
                                Err(error) => Err(SnapshotFailure::Invalid(error.to_string())),
                            }
                        } else {
                            renderer
                                .set_snapshot(&snapshot)
                                .map_err(SnapshotFailure::from)
                        };
                        if result.is_ok() {
                            snapshot_kind = if depth {
                                CaptureKind::Depth
                            } else {
                                CaptureKind::Rgba
                            };
                        }
                        let reply = snapshot_reply(
                            request_id,
                            stamp,
                            result,
                            &mut last_accepted_stamp,
                            &mut scene_needs_update,
                            &mut capture,
                        );
                        (Some(reply), false)
                    }
                    WorkerRequest::Capture { request_id } => {
                        let reply = (!capture.begin_with_kind(request_id, snapshot_kind))
                            .then_some(Reply::Busy { request_id });
                        (reply, false)
                    }
                    WorkerRequest::Cancel { request_id } => {
                        let reply = if capture.cancel(request_id) {
                            renderer.cancel_readback();
                            Reply::CancelAccepted { request_id }
                        } else {
                            failure(
                                FailureKind::UnknownCapture,
                                Some(request_id),
                                "unknown capture",
                            )
                        };
                        (Some(reply), false)
                    }
                    WorkerRequest::Shutdown => break 'worker,
                }
            };
            if let Some(reply) = reply {
                send_or_exit(&mut output, reply, &exit_deadline);
            }
            if skip_update {
                continue;
            }
        }
        let active = capture.active();
        if let Some(Capture {
            request_id,
            cancelled: true,
        }) = active
            && !renderer.readback_in_flight()
        {
            send_or_exit(&mut output, Reply::Cancelled { request_id }, &exit_deadline);
            capture.clear();
            continue;
        }
        if (active.is_some() || scene_needs_update)
            && let Err(reply) = apply_update(
                renderer.update(),
                active.map(|capture| capture.request_id),
                &mut scene_needs_update,
            )
        {
            send_or_exit(&mut output, reply, &exit_deadline);
            break;
        }
        if let Some(Capture {
            request_id,
            cancelled,
        }) = active
        {
            if !cancelled && !renderer.readback_in_flight() {
                let requested = match capture.kind() {
                    CaptureKind::Rgba => renderer.request_readback(),
                    CaptureKind::Depth => renderer.request_depth_readback(),
                };
                match requested {
                    Ok(()) | Err(RenderError::NotReady) => {}
                    Err(error) => {
                        send_or_exit(
                            &mut output,
                            failure(FailureKind::Capture, Some(request_id), error),
                            &exit_deadline,
                        );
                        capture.clear();
                    }
                }
            }
            if renderer.readback_in_flight() {
                // Retirement follows the admitted capture, even after a replacement snapshot.
                let outcome = match capture.kind() {
                    CaptureKind::Rgba => {
                        let result = renderer.take_frame();
                        capture.readback(result, renderer.readback_in_flight())
                    }
                    CaptureKind::Depth => {
                        let result = renderer.take_depth_frame();
                        capture.readback_depth(result, renderer.readback_in_flight())
                    }
                };
                match outcome {
                    ReadbackOutcome::Pending => {}
                    ReadbackOutcome::Complete(reply) => {
                        send_or_exit(&mut output, reply, &exit_deadline)
                    }
                    ReadbackOutcome::Failed(reply) => {
                        send_or_exit(&mut output, reply, &exit_deadline);
                        break;
                    }
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
#[path = "../tests/private/worker.rs"]
mod tests;
