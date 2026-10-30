// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Experimental worker supervisor. Not an accepted API.
use gz_render_protocol::{
    FailureKind, PROTOCOL_VERSION, Reply, Request, Stamp, read_reply, write_request,
};
use std::{
    io,
    path::PathBuf,
    process::{Child, Command, Stdio},
    sync::{
        Arc, Mutex, TryLockError, Weak,
        atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering},
        mpsc::{self, Receiver, SyncSender, TryRecvError, TrySendError},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

#[derive(Debug)]
pub enum ClientError {
    Busy,
    Closed,
    Io(io::Error),
    WorkerExited(Option<i32>),
    ProtocolVersion,
    ProtocolCorrelation,
    InvalidTimeout,
    InvalidWorkerPath,
    IdentityExhausted,
    Timeout,
}
#[derive(Debug)]
pub enum SubmitError {
    Invalid(Request),
    NotReady(Request),
    NoSnapshot(Request),
    Stale(Request),
    Busy(Request),
    Closed(Request),
}

#[derive(Debug, PartialEq, Eq)]
pub enum ShutdownOutcome {
    Graceful,
    Forced,
    Exited(Option<i32>),
}

/// Non-owning diagnostic for the client, supervisor, and I/O thread lifetime.
///
/// Retirement does not attest that an OS task has disappeared from `/proc`,
/// nor does it independently prove that a failed child wait reaped the process.
/// Keeping this witness retains the existing Arc allocation until it is dropped;
/// it does not retain the client, threads, child, pipes, or live atomic value.
#[derive(Debug, Clone)]
pub struct ClientLifetime(Weak<AtomicBool>);

impl ClientLifetime {
    /// True once the client and all its supervisor and I/O threads release their state.
    pub fn is_retired(&self) -> bool {
        self.0.upgrade().is_none()
    }
}

static NEXT_CLIENT: AtomicU64 = AtomicU64::new(1);

#[derive(Default)]
struct SnapshotCorrelation {
    last_id: u64,
    pending_id: Option<u64>,
    pending_stamp: Option<Stamp>,
    committed_stamp: Option<Stamp>,
    last_capture_id: u64,
    pending_capture: Option<(u64, Stamp, bool)>,
    pending_cancel: Option<(u64, bool)>,
}

/// Worker executable selected by trusted operator configuration, never scene data.
///
/// The caller is responsible for trusting the installation and maintaining its
/// permissions. This does not authenticate the file or protect against replacement.
/// Resolve this configuration before starting simulation work.
#[derive(Debug)]
pub struct TrustedWorker(PathBuf);

impl TrustedWorker {
    pub fn from_operator_config(path: PathBuf) -> Result<Self, ClientError> {
        if !path.is_absolute() {
            return Err(ClientError::InvalidWorkerPath);
        }
        Ok(Self(path))
    }
}

/// A bounded worker client. Destroy it outside simulation's critical step.
///
/// `Drop` attempts shutdown, then joins the supervisor if the attempt times out.
/// A stalled kernel or driver can delay that final join indefinitely.
pub struct Client {
    token: u64,
    worker_pid: Arc<AtomicU32>,
    input: Option<SyncSender<Request>>,
    output: Receiver<Reply>,
    failed: Receiver<ClientError>,
    stop: Arc<AtomicBool>,
    output_blocked: Arc<AtomicBool>,
    ready: Arc<AtomicBool>,
    snapshots: Arc<Mutex<SnapshotCorrelation>>,
    done: Arc<AtomicBool>,
    shutdown_requested: Arc<AtomicBool>,
    finished: Receiver<Option<i32>>,
    supervisor: Option<JoinHandle<()>>,
}

// Observe child exit until shutdown is requested or the OS reports a wait error.
fn monitor_child(child: &mut Child, stop: &AtomicBool, failed: &SyncSender<ClientError>) {
    while !stop.load(Ordering::Acquire) {
        match child.try_wait() {
            Ok(Some(_)) => break,
            Ok(None) => thread::sleep(Duration::from_millis(5)),
            Err(error) => {
                let _ = failed.try_send(ClientError::Io(error));
                break;
            }
        }
    }
}

impl Client {
    /// Starts a supervisor; the worker's Ready or startup failure arrives asynchronously.
    pub fn start(worker: TrustedWorker) -> Result<Self, ClientError> {
        let binary = worker.0;
        let token = NEXT_CLIENT
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |id| {
                (id < ((1_u64 << 52) - 1)).then_some(id + 1)
            })
            .map_err(|_| ClientError::IdentityExhausted)?;
        let (tx, rx) = mpsc::sync_channel::<Request>(1);
        let (out_tx, output) = mpsc::sync_channel::<Reply>(1);
        let (fail_tx, failed) = mpsc::sync_channel::<ClientError>(1);
        let (finish_tx, finished) = mpsc::sync_channel::<Option<i32>>(1);
        let worker_pid = Arc::new(AtomicU32::new(0));
        let supervisor_pid = worker_pid.clone();
        let stop = Arc::new(AtomicBool::new(false));
        let output_blocked = Arc::new(AtomicBool::new(false));
        let ready = Arc::new(AtomicBool::new(false));
        let snapshots = Arc::new(Mutex::new(SnapshotCorrelation::default()));
        let reader_snapshots = snapshots.clone();
        let reader_ready = ready.clone();
        let blocked_reader = output_blocked.clone();
        let done = Arc::new(AtomicBool::new(false));
        let shutdown_requested = Arc::new(AtomicBool::new(false));
        let stop_supervisor = stop.clone();
        let done_supervisor = done.clone();
        let supervisor_shutdown = shutdown_requested.clone();
        let reader_shutdown = shutdown_requested.clone();
        let supervisor = thread::Builder::new()
            .name(format!("r{token:013x}s"))
            .spawn(move || {
                let mut child = match Command::new(binary)
                    .arg(std::process::id().to_string())
                    .stdin(Stdio::piped())
                    .stdout(Stdio::piped())
                    .stderr(Stdio::null())
                    .spawn()
                {
                    Ok(child) => child,
                    Err(error) => {
                        let _ = fail_tx.try_send(ClientError::Io(error));
                        done_supervisor.store(true, Ordering::Release);
                        let _ = finish_tx.try_send(None);
                        return;
                    }
                };
                supervisor_pid.store(child.id(), Ordering::Release);
                let mut stdin = child.stdin.take().expect("piped stdin");
                let mut stdout = child.stdout.take().expect("piped stdout");
                let input_stop = stop_supervisor.clone();
                let input_fail = fail_tx.clone();
                let writer_lifetime = done_supervisor.clone();
                let writer = thread::Builder::new()
                    .name(format!("r{token:013x}w"))
                    .spawn(move || {
                        let _lifetime = writer_lifetime;
                        while !input_stop.load(Ordering::Acquire) {
                            match rx.recv_timeout(Duration::from_millis(5)) {
                                Ok(request) => {
                                    if let Err(error) = write_request(&mut stdin, &request) {
                                        let _ = input_fail.try_send(ClientError::Io(error));
                                        input_stop.store(true, Ordering::Release);
                                        break;
                                    }
                                }
                                Err(mpsc::RecvTimeoutError::Timeout) => {}
                                Err(mpsc::RecvTimeoutError::Disconnected) => break,
                            }
                        }
                    });
                let writer = match writer {
                    Ok(writer) => writer,
                    Err(error) => {
                        let _ = fail_tx.try_send(ClientError::Io(error));
                        let _ = child.kill();
                        let _ = child.wait();
                        supervisor_pid.store(0, Ordering::Release);
                        done_supervisor.store(true, Ordering::Release);
                        let _ = finish_tx.try_send(None);
                        return;
                    }
                };
                let output_stop = stop_supervisor.clone();
                let output_fail = fail_tx.clone();
                let reader_lifetime = done_supervisor.clone();
                let reader = thread::Builder::new()
                    .name(format!("r{token:013x}r"))
                    .spawn(move || {
                        let _lifetime = reader_lifetime;
                        let mut first = true;
                        let mut startup_failed = false;
                        loop {
                            match read_reply(&mut stdout) {
                                Ok(Some(mut reply)) => {
                                    if first {
                                        match &reply {
                                            Reply::Ready {
                                                version: PROTOCOL_VERSION,
                                            } => {
                                                reader_ready.store(true, Ordering::Release);
                                            }
                                            Reply::Error {
                                                kind: FailureKind::Startup,
                                                ..
                                            } => {
                                                startup_failed = true;
                                            }
                                            _ => {
                                                let _ = output_fail
                                                    .try_send(ClientError::ProtocolVersion);
                                                output_stop.store(true, Ordering::Release);
                                                return;
                                            }
                                        }
                                        first = false;
                                    } else if startup_failed || matches!(reply, Reply::Ready { .. })
                                    {
                                        let _ = output_fail.try_send(ClientError::ProtocolVersion);
                                        output_stop.store(true, Ordering::Release);
                                        return;
                                    }
                                    let fatal = matches!(
                                        &reply,
                                        Reply::Error {
                                            kind: FailureKind::Device | FailureKind::Shader,
                                            ..
                                        }
                                    );
                                    let correlated = matches!(
                                        &reply,
                                        Reply::SnapshotAccepted { .. }
                                            | Reply::SnapshotBusy { .. }
                                            | Reply::Frame { .. }
                                            | Reply::Cancelled { .. }
                                            | Reply::CancelAccepted { .. }
                                            | Reply::Busy { .. }
                                            | Reply::Error {
                                                kind: FailureKind::Device | FailureKind::Shader,
                                                request_id: Some(_),
                                                ..
                                            }
                                            | Reply::Error {
                                                kind: FailureKind::InvalidSnapshot
                                                    | FailureKind::StaleRequest
                                                    | FailureKind::Capture
                                                    | FailureKind::Readback
                                                    | FailureKind::UnknownCapture,
                                                ..
                                            }
                                    );
                                    if correlated {
                                        let Ok(mut state) = reader_snapshots.lock() else {
                                            let _ = output_fail
                                                .try_send(ClientError::ProtocolCorrelation);
                                            output_stop.store(true, Ordering::Release);
                                            return;
                                        };
                                        let valid = match &reply {
                                            Reply::SnapshotAccepted { request_id }
                                                if state.pending_id == Some(*request_id) =>
                                            {
                                                state.pending_id = None;
                                                state.committed_stamp = state.pending_stamp.take();
                                                if let Some((_, _, invalidated)) =
                                                    &mut state.pending_capture
                                                {
                                                    *invalidated = true;
                                                }
                                                true
                                            }
                                            Reply::SnapshotBusy { request_id }
                                            | Reply::Error {
                                                kind:
                                                    FailureKind::InvalidSnapshot
                                                    | FailureKind::StaleRequest,
                                                request_id: Some(request_id),
                                                ..
                                            } if state.pending_id == Some(*request_id) => {
                                                state.pending_id = None;
                                                state.pending_stamp = None;
                                                true
                                            }
                                            Reply::Frame {
                                                request_id, stamp, ..
                                            } if state.pending_capture
                                                == Some((*request_id, *stamp, false))
                                                && state.pending_cancel
                                                    != Some((*request_id, true)) =>
                                            {
                                                state.pending_capture = None;
                                                true
                                            }
                                            Reply::CancelAccepted { request_id }
                                                if state.pending_cancel
                                                    == Some((*request_id, false)) =>
                                            {
                                                state.pending_cancel = Some((*request_id, true));
                                                true
                                            }
                                            Reply::Error {
                                                kind: FailureKind::UnknownCapture,
                                                request_id: Some(request_id),
                                                ..
                                            } if state.pending_cancel
                                                == Some((*request_id, false)) =>
                                            {
                                                state.pending_cancel = None;
                                                true
                                            }
                                            Reply::Cancelled { request_id }
                                            | Reply::Busy { request_id }
                                            | Reply::Error {
                                                kind:
                                                    FailureKind::Capture
                                                    | FailureKind::Readback
                                                    | FailureKind::Device
                                                    | FailureKind::Shader,
                                                request_id: Some(request_id),
                                                ..
                                            } if state
                                                .pending_capture
                                                .is_some_and(|(id, _, _)| id == *request_id) =>
                                            {
                                                state.pending_capture = None;
                                                if state.pending_cancel == Some((*request_id, true))
                                                {
                                                    state.pending_cancel = None;
                                                }
                                                true
                                            }
                                            _ => false,
                                        };
                                        if !valid {
                                            let _ = output_fail
                                                .try_send(ClientError::ProtocolCorrelation);
                                            output_stop.store(true, Ordering::Release);
                                            return;
                                        }
                                    }
                                    if fatal {
                                        output_stop.store(true, Ordering::Release);
                                    }
                                    loop {
                                        match out_tx.try_send(reply) {
                                            Ok(()) => {
                                                blocked_reader.store(false, Ordering::Release);
                                                if fatal {
                                                    return;
                                                }
                                                break;
                                            }
                                            Err(TrySendError::Full(returned)) => {
                                                reply = returned;
                                                blocked_reader.store(true, Ordering::Release);
                                                if (output_stop.load(Ordering::Acquire) && !fatal)
                                                    || reader_shutdown.load(Ordering::Acquire)
                                                {
                                                    return;
                                                }
                                                thread::sleep(Duration::from_millis(5));
                                            }
                                            Err(TrySendError::Disconnected(_)) => {
                                                let _ = output_fail.try_send(ClientError::Closed);
                                                output_stop.store(true, Ordering::Release);
                                                return;
                                            }
                                        }
                                    }
                                }
                                Ok(None) => {
                                    output_stop.store(true, Ordering::Release);
                                    break;
                                }
                                Err(error) => {
                                    let _ = output_fail.try_send(ClientError::Io(error));
                                    output_stop.store(true, Ordering::Release);
                                    break;
                                }
                            }
                        }
                    });
                let reader = match reader {
                    Ok(reader) => reader,
                    Err(error) => {
                        let _ = fail_tx.try_send(ClientError::Io(error));
                        stop_supervisor.store(true, Ordering::Release);
                        let _ = child.kill();
                        let _ = child.wait();
                        let _ = writer.join();
                        supervisor_pid.store(0, Ordering::Release);
                        done_supervisor.store(true, Ordering::Release);
                        let _ = finish_tx.try_send(None);
                        return;
                    }
                };
                monitor_child(&mut child, &stop_supervisor, &fail_tx);
                let _ = child.kill();
                let status = child.wait().ok().and_then(|status| status.code());
                supervisor_pid.store(0, Ordering::Release);
                let _ = writer.join();
                let _ = reader.join();
                if !supervisor_shutdown.load(Ordering::Acquire) {
                    let _ = fail_tx.try_send(ClientError::WorkerExited(status));
                }
                done_supervisor.store(true, Ordering::Release);
                let _ = finish_tx.try_send(status);
            })
            .map_err(ClientError::Io)?;
        Ok(Self {
            token,
            worker_pid,
            input: Some(tx),
            output,
            failed,
            stop,
            output_blocked,
            ready,
            snapshots,
            done,
            shutdown_requested,
            finished,
            supervisor: Some(supervisor),
        })
    }
    pub fn worker_pid(&self) -> Option<u32> {
        match self.worker_pid.load(Ordering::Acquire) {
            0 => None,
            pid => Some(pid),
        }
    }
    pub fn is_stopping(&self) -> bool {
        self.stop.load(Ordering::Acquire)
    }
    pub fn is_output_blocked(&self) -> bool {
        self.output_blocked.load(Ordering::Acquire)
    }
    pub fn diagnostic_thread_prefix(&self) -> String {
        format!("r{:013x}", self.token)
    }
    /// Observe client/supervisor retirement without keeping either alive.
    pub fn diagnostic_lifetime(&self) -> ClientLifetime {
        ClientLifetime(Arc::downgrade(&self.done))
    }
    /// True after the supervisor is joined and its I/O owners have retired.
    /// This does not independently establish that the child was reaped.
    pub fn diagnostic_shutdown_complete(&self) -> bool {
        self.supervisor.is_none() && Arc::strong_count(&self.done) == 1
    }
    /// A rejected request is returned intact so callers can retry or discard it.
    pub fn try_send(&self, request: Request) -> Result<(), SubmitError> {
        if !request.wire_valid() {
            return Err(SubmitError::Invalid(request));
        }
        let Some(input) = &self.input else {
            return Err(SubmitError::Closed(request));
        };
        if self.stop.load(Ordering::Acquire) {
            return Err(SubmitError::Closed(request));
        }
        if !self.ready.load(Ordering::Acquire) {
            return Err(SubmitError::NotReady(request));
        }
        if let Request::Snapshot {
            request_id, stamp, ..
        }
        | Request::CalibratedSnapshot {
            request_id, stamp, ..
        } = &request
        {
            let id = *request_id;
            let stamp = *stamp;
            let mut state = match self.snapshots.try_lock() {
                Ok(state) => state,
                Err(TryLockError::WouldBlock) => return Err(SubmitError::Busy(request)),
                Err(TryLockError::Poisoned(_)) => return Err(SubmitError::Closed(request)),
            };
            if id <= state.last_id
                || state
                    .committed_stamp
                    .is_some_and(|previous| stamp < previous)
            {
                return Err(SubmitError::Stale(request));
            }
            if state.pending_id.is_some() {
                return Err(SubmitError::Busy(request));
            }
            return match input.try_send(request) {
                Ok(()) => {
                    state.last_id = id;
                    state.pending_id = Some(id);
                    state.pending_stamp = Some(stamp);
                    Ok(())
                }
                Err(TrySendError::Full(request)) => Err(SubmitError::Busy(request)),
                Err(TrySendError::Disconnected(request)) => Err(SubmitError::Closed(request)),
            };
        }
        if let Request::Capture { request_id } = &request {
            let id = *request_id;
            let mut state = match self.snapshots.try_lock() {
                Ok(state) => state,
                Err(TryLockError::WouldBlock) => return Err(SubmitError::Busy(request)),
                Err(TryLockError::Poisoned(_)) => return Err(SubmitError::Closed(request)),
            };
            if id <= state.last_capture_id {
                return Err(SubmitError::Stale(request));
            }
            if state.pending_id.is_some()
                || state.pending_capture.is_some()
                || state.pending_cancel.is_some()
            {
                return Err(SubmitError::Busy(request));
            }
            let Some(stamp) = state.committed_stamp else {
                return Err(SubmitError::NoSnapshot(request));
            };
            return match input.try_send(request) {
                Ok(()) => {
                    state.last_capture_id = id;
                    state.pending_capture = Some((id, stamp, false));
                    Ok(())
                }
                Err(TrySendError::Full(request)) => Err(SubmitError::Busy(request)),
                Err(TrySendError::Disconnected(request)) => Err(SubmitError::Closed(request)),
            };
        }
        if let Request::Cancel { request_id } = &request {
            let id = *request_id;
            let mut state = match self.snapshots.try_lock() {
                Ok(state) => state,
                Err(TryLockError::WouldBlock) => return Err(SubmitError::Busy(request)),
                Err(TryLockError::Poisoned(_)) => return Err(SubmitError::Closed(request)),
            };
            if !state
                .pending_capture
                .is_some_and(|(active, _, _)| active == id)
            {
                return Err(SubmitError::Stale(request));
            }
            if state.pending_cancel.is_some() {
                return Err(SubmitError::Busy(request));
            }
            return match input.try_send(request) {
                Ok(()) => {
                    state.pending_cancel = Some((id, false));
                    Ok(())
                }
                Err(TrySendError::Full(request)) => Err(SubmitError::Busy(request)),
                Err(TrySendError::Disconnected(request)) => Err(SubmitError::Closed(request)),
            };
        }
        match input.try_send(request) {
            Ok(()) => Ok(()),
            Err(TrySendError::Full(request)) => Err(SubmitError::Busy(request)),
            Err(TrySendError::Disconnected(request)) => Err(SubmitError::Closed(request)),
        }
    }
    pub fn try_reply(&self) -> Result<Option<Reply>, ClientError> {
        match self.output.try_recv() {
            Ok(reply) => Ok(Some(reply)),
            Err(output_state) => match self.failed.try_recv() {
                Ok(error) => Err(error),
                Err(_)
                    if matches!(output_state, TryRecvError::Disconnected)
                        && self.done.load(Ordering::Acquire) =>
                {
                    Err(ClientError::Closed)
                }
                Err(_) => Ok(None),
            },
        }
    }
    /// Attempt graceful shutdown, then kill. Timeout retains the handle for another wait.
    pub fn shutdown(&mut self, timeout: Duration) -> Result<ShutdownOutcome, ClientError> {
        if self.supervisor.is_none() {
            return Ok(ShutdownOutcome::Graceful);
        }
        let deadline = Instant::now()
            .checked_add(timeout)
            .ok_or(ClientError::InvalidTimeout)?;
        self.shutdown_requested.store(true, Ordering::Release);
        if let Some(input) = self.input.take() {
            let _ = input.try_send(Request::Shutdown);
        }
        let grace = timeout / 2;
        let (status, forced) = match self.finished.recv_timeout(grace) {
            Ok(status) => (status, false),
            Err(_) => {
                self.stop.store(true, Ordering::Release);
                let status = self
                    .finished
                    .recv_timeout(deadline.saturating_duration_since(Instant::now()))
                    .map_err(|_| ClientError::Timeout)?;
                (status, true)
            }
        };
        if let Some(handle) = self.supervisor.take() {
            let _ = handle.join();
        }
        Ok(if forced {
            ShutdownOutcome::Forced
        } else if status == Some(0) {
            ShutdownOutcome::Graceful
        } else {
            ShutdownOutcome::Exited(status)
        })
    }
}

impl Drop for Client {
    fn drop(&mut self) {
        // Callers needing the result use shutdown explicitly, outside physics steps.
        if self.shutdown(Duration::from_secs(3)).is_err() {
            self.stop.store(true, Ordering::Release);
            if let Some(handle) = self.supervisor.take() {
                let _ = handle.join();
            }
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/client.rs"]
mod tests;
