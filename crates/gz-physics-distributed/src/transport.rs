// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A private TCP listener and Zenoh client session for each worker process.
//! The only operation that advances time is a validated protocol step.
use crate::{
    physics::{Body, GroundPlane, NativeSeedLayout, NativeSolverProfile, SimulationStep, Snapshot},
    protocol::{Command as WorkerCommand, ProtocolError, Reply, Request, Worker, WorkerImage},
};
use serde::{Deserialize, Serialize};
use std::{
    borrow::Borrow,
    io::{self, BufRead, BufReader, Read, Write},
    net::{Ipv4Addr, SocketAddrV4},
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::mpsc,
    thread,
    time::{Duration, Instant},
};
use zenoh::{Wait, handlers::FifoChannel};

mod checkpoint;
mod handoff;
pub(crate) mod hinge;
pub use hinge::{serve_managed_ground_hinge, serve_managed_hinge};
mod profile;
pub use profile::managed_worker_profile;
use profile::wire_profile;
mod observation;

const MAX_PAYLOAD: usize = 256 * 1024;
/// A full 256-body frame can carry ordinary state, plane reactions and many local pairs.
const MAX_OBSERVATION_PAYLOAD: usize = 1024 * 1024;
/// A restart image contains multiple separately bounded native components.
const MAX_WORKER_IMAGE_PAYLOAD: usize = 24 * 1024 * 1024;
// A native byte needs at most three decimal digits and one separator in the
// existing JSON representation. Reserve another small frame for the snapshot,
// contacts, command ledger, ticket, receipts and transport envelopes.
pub(crate) const MAX_CHECKPOINT_PAYLOAD: usize =
    4 * gz_physics_multifidelity::PARTITION_CHECKPOINT_LIMIT + MAX_PAYLOAD;
#[derive(Copy, Clone, PartialEq, Eq)]
enum LocalHint {
    None,
    Helper,
    ScheduledProbe,
    PackedCheckpoint,
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum ProcessFault { Withhold, ExitAfterReply }
struct PendingQuery {
    exit_after_reply: bool,
    accepts_packed_image: bool,
    replies: zenoh::handlers::FifoChannelHandler<zenoh::query::Reply>,
    deadline: Instant,
    timeout: Duration,
    reply_limit: usize,
    accepts_packed_export: bool,
}

const MAX_SEED_REASON: usize = 320;
const EXTENDED_OFFER: &[u8] = b"GZNATIVE64/1\n";
const START_TIMEOUT: Duration = Duration::from_secs(5);
pub(crate) const REQUEST_TIMEOUT: Duration = Duration::from_secs(1);

#[derive(Debug, thiserror::Error)]
pub enum TransportError {
    #[error("worker transport unavailable: {0}")]
    Unavailable(String),
    #[error("invalid worker wire message: {0}")]
    Wire(String),
    #[error(transparent)]
    Protocol(#[from] ProtocolError),
    #[error(transparent)]
    Io(#[from] io::Error),
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Traffic {
    pub requests: u64,
    pub replies: u64,
    /// Serialized application payload bytes, including readiness and shutdown queries.
    pub request_bytes: u64,
    pub reply_bytes: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Ready {
    #[serde(default, skip_serializing_if = "NativeSolverProfile::is_standard")]
    profile: NativeSolverProfile,
    run_id: u64,
    worker_id: u8,
    endpoint: String,
    session_id: String,
    // Established over the child stdout pipe and rotated on replacement.
    capability: [u8; 16],
}

/// A managed worker may reject its seed before it has a listener or capability.
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct RejectedSeed {
    run_id: u64,
    worker_id: u8,
    reason: String,
}

impl Ready {
    fn read(output: impl Read, run_id: u64, worker_id: u8) -> io::Result<Self> {
        let mut line = Vec::new();
        BufReader::new(output)
            .take(1025)
            .read_until(b'\n', &mut line)?;
        if line.len() > 1024 || line.last() != Some(&b'\n') {
            return Err(io_error("missing or oversized child readiness"));
        }
        #[derive(Deserialize)]
        #[serde(untagged)]
        enum Startup {
            Ready(Ready),
            Rejected(RejectedSeed),
        }
        let ready = match serde_json::from_slice(&line).map_err(io_error)? {
            Startup::Ready(ready) => ready,
            Startup::Rejected(r) => {
                if (r.run_id, r.worker_id) != (run_id, worker_id) {
                    return Err(io_error("child rejection has the wrong worker identity"));
                }
                if r.reason.is_empty()
                    || r.reason.len() > MAX_SEED_REASON
                    || !r.reason.bytes().all(|b| b.is_ascii_graphic() || b == b' ')
                {
                    return Err(io_error("invalid child rejection reason"));
                }
                return Err(io_error(format!(
                    "worker rejected initial state: {}",
                    r.reason
                )));
            }
        };
        if ready.run_id != run_id
            || ready.worker_id != worker_id
            || !endpoint_valid(&ready.endpoint)
        {
            return Err(io_error("child announced the wrong worker or endpoint"));
        }
        Ok(ready)
    }
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
enum Operation<T> {
    Ping,
    Save {
        sequence: u64,
        tick: u64,
    },
    StartProbe {
        tick: u64,
        span: u16,
    },
    ReadProbe {
        start: u64,
        span: u16,
    },
    Request(T),
    DropReply(T),
    PreviewImpulse {
        sequence: u64,
        tick: u64,
        impulse: crate::protocol::SpatialImpulse,
    },
    Shutdown,
}

impl<T: Borrow<Request>> Operation<T> {
    /// Permit a large application payload only in the direction carrying the
    /// native image. Ping, readiness, errors, and phase receipts remain small.
    fn payload_limits(&self) -> (usize, usize) {
        if matches!(self, Self::Save { .. }) {
            return (MAX_PAYLOAD, MAX_WORKER_IMAGE_PAYLOAD);
        }
        if let Self::Request(request) | Self::DropReply(request) = self {
            match request.borrow().command {
                WorkerCommand::PrepareWarm { .. }
                | WorkerCommand::PrepareSolver { .. }
                | WorkerCommand::PrepareOrdered { .. } => {
                    return (MAX_CHECKPOINT_PAYLOAD, MAX_PAYLOAD);
                }
                WorkerCommand::ExportWarm { .. } | WorkerCommand::ExportSolver { .. } => {
                    return (MAX_PAYLOAD, MAX_CHECKPOINT_PAYLOAD);
                }
                WorkerCommand::Prepare { .. }
                | WorkerCommand::Release { .. }
                | WorkerCommand::ReleaseWarm { .. } => {}
                _ => return (MAX_PAYLOAD, MAX_OBSERVATION_PAYLOAD),
            }
        }
        (MAX_PAYLOAD, MAX_PAYLOAD)
    }
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
enum Response {
    Ready(Ready),
    HandoffReadyV1(Ready),
    Saved(Result<Box<WorkerImage>, ProtocolError>),
    Answer(Result<Reply, ProtocolError>),
    ProbeStarted(Result<(), ProtocolError>),
    ScheduledProbeStarted(Result<(), ProtocolError>),
    ProbeResult(Option<Vec<crate::physics::ComponentWork>>),
    Stopped,
    ProspectiveTwist(Result<crate::protocol::VelocityEdit, ProtocolError>),
}

fn unavailable(e: impl std::fmt::Display) -> TransportError {
    TransportError::Unavailable(e.to_string())
}
fn wire(e: impl std::fmt::Display) -> TransportError {
    TransportError::Wire(e.to_string())
}
fn io_error(e: impl std::fmt::Display) -> io::Error {
    io::Error::other(e.to_string())
}

struct Limited {
    data: Vec<u8>,
    limit: usize,
}
impl Write for Limited {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        if bytes.len() > self.limit - self.data.len() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("worker payload exceeds {} KiB", self.limit / 1024),
            ));
        }
        self.data.extend_from_slice(bytes);
        Ok(bytes.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}
fn encode<T: Serialize + ?Sized>(value: &T) -> Result<Vec<u8>, TransportError> {
    encode_limited(value, MAX_PAYLOAD)
}
fn encode_limited<T: Serialize + ?Sized>(
    value: &T,
    limit: usize,
) -> Result<Vec<u8>, TransportError> {
    let mut bytes = Limited {
        data: Vec::new(),
        limit,
    };
    serde_json::to_writer(&mut bytes, value).map_err(wire)?;
    Ok(bytes.data)
}

fn encode_answer(
    mut answer: Result<Reply, ProtocolError>,
    profile: NativeSolverProfile,
    packed: bool,
    packed_handoff: bool,
    limit: usize,
) -> Result<Vec<u8>, TransportError> {
    if packed_handoff
        && let Ok(reply) = &mut answer
        && let Some(bytes) = handoff::encode_reply(reply)?
    {
        if bytes.len() > limit {
            return Err(wire("packed handoff exceeds reply limit"));
        }
        return Ok(bytes);
    }
    if packed
        && let Ok(reply) = &answer
        && let Some(data) = observation::encode_with_profile(reply, profile)?
    {
        if data.len() > limit {
            return Err(wire("packed observation exceeds reply limit"));
        }
        Ok(data)
    } else {
        encode_limited(&Response::Answer(answer), limit)
    }
}

fn key(run_id: u64, worker_id: u8) -> String {
    format!("gz/physics/distributed/v1/{run_id}/worker/{worker_id}")
}
pub(crate) fn endpoint_valid(endpoint: &str) -> bool {
    endpoint
        .strip_prefix("tcp/")
        .and_then(|ep| ep.parse::<SocketAddrV4>().ok())
        .is_some_and(|addr| *addr.ip() == Ipv4Addr::LOCALHOST && addr.port() != 0)
}
pub(crate) fn config(endpoint: Option<&str>) -> io::Result<zenoh::Config> {
    let mut cfg = zenoh::Config::default();
    if let Some(ep) = endpoint
        && !endpoint_valid(ep)
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "expected numeric 127.0.0.1 TCP endpoint",
        ));
    }
    let listen: Vec<&str> = if endpoint.is_some() {
        vec![]
    } else {
        vec!["tcp/127.0.0.1:0"]
    };
    let connect: Vec<&str> = endpoint.into_iter().collect();
    for (k, v) in [
        (
            "mode",
            if endpoint.is_some() {
                "\"client\""
            } else {
                "\"peer\""
            }
            .into(),
        ),
        ("listen/endpoints", serde_json::to_string(&listen)?),
        ("connect/endpoints", serde_json::to_string(&connect)?),
        ("scouting/multicast/enabled", "false".into()),
        ("scouting/gossip/enabled", "false".into()),
        ("connect/timeout_ms", "1000".into()),
        (
            "transport/link/rx/max_message_size",
            (MAX_WORKER_IMAGE_PAYLOAD + 8192).to_string(),
        ),
    ] {
        cfg.insert_json5(k, &v).map_err(io_error)?;
    }
    Ok(cfg)
}

/// Managed children keep their launcher's stdin pipe open after a single seed
/// line. Losing the last writer fences the worker even if the launcher cannot
/// run Rust destructors. A subsequent launcher receives a fresh capability.
pub fn serve_managed(run_id: u64, worker_id: u8) -> Result<(), TransportError> {
    serve_owned_pipe(run_id, worker_id, false, NativeSolverProfile::Standard32)
}

/// Start a private managed child from the launcher's trusted native image.
pub fn serve_managed_image(run_id: u64, worker_id: u8) -> Result<(), TransportError> {
    serve_owned_pipe(run_id, worker_id, true, NativeSolverProfile::Standard32)
}

/// Offer the bounded 64-member profile on the private pipe before accepting cold
/// construction data or reading trusted same-build native images.
pub fn serve_managed_extended(
    run_id: u64,
    worker_id: u8,
    image: bool,
) -> Result<(), TransportError> {
    serve_managed_profile(run_id, worker_id, image, NativeSolverProfile::Extended64)
}

/// Announce the exact nonlegacy solver before reading cold or native-image bytes.
pub fn serve_managed_profile(
    run: u64,
    worker: u8,
    image: bool,
    profile: NativeSolverProfile,
) -> Result<(), TransportError> {
    if let Some(offer) = wire_profile(profile).offer {
        let mut output = io::stdout().lock();
        output.write_all(offer)?;
        output.flush()?;
    }
    serve_owned_pipe(run, worker, image, profile)
}

fn serve_owned_pipe(
    run_id: u64,
    worker_id: u8,
    image: bool,
    profile: NativeSolverProfile,
) -> Result<(), TransportError> {
    let limit = if image {
        MAX_WORKER_IMAGE_PAYLOAD
    } else {
        MAX_PAYLOAD
    };
    let seed = read_owned_seed(limit)?;
    let worker = if image {
        worker_from_image_seed(run_id, worker_id, &seed, profile)
    } else {
        worker_from_seed(run_id, worker_id, &seed, profile)
    };
    let mut stdout = io::stdout().lock();
    let worker = match worker {
        Ok(worker) => worker,
        Err(error) => {
            // Stay within the 1 KiB handshake limit even after JSON escaping.
            let reason = error
                .to_string()
                .chars()
                .filter(|c| c.is_ascii_graphic() || *c == ' ')
                .take(MAX_SEED_REASON)
                .collect();
            serde_json::to_writer(
                &mut stdout,
                &RejectedSeed {
                    run_id,
                    worker_id,
                    reason,
                },
            )
            .map_err(wire)?;
            writeln!(stdout)?;
            stdout.flush()?;
            return Err(error);
        }
    };
    serve_worker(run_id, worker_id, worker, stdout)
}

/// Read one bounded seed, then fence the managed process when its launcher pipe closes.
fn read_owned_seed(limit: usize) -> Result<Vec<u8>, TransportError> {
    let mut seed = Vec::new();
    let mut input = BufReader::new(io::stdin());
    (&mut input)
        .take((limit + 2) as u64)
        .read_until(b'\n', &mut seed)?;
    if seed.pop() != Some(b'\n') || seed.len() > limit {
        return Err(wire("incomplete or oversized managed seed"));
    }
    thread::Builder::new()
        .name("physics-worker-owner".into())
        .spawn(move || {
            let mut byte = [0u8; 1];
            // The owned pipe carries no further messages. EOF means owner death.
            loop {
                match input.read(&mut byte) {
                    Ok(0) => std::process::exit(0),
                    Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                    _ => std::process::exit(1),
                }
            }
        })?;
    Ok(seed)
}

fn worker_from_image_seed(
    run_id: u64,
    worker_id: u8,
    seed: &[u8],
    expected: NativeSolverProfile,
) -> Result<Worker, TransportError> {
    let worker = if seed.starts_with(b"{\"pool_width\":") {
        #[derive(Deserialize)]
        #[serde(deny_unknown_fields)]
        struct PoolSeed {
            pool_width: u8,
            image: WorkerImage,
        }
        let pool: PoolSeed = serde_json::from_slice(seed).map_err(wire)?;
        if !(3..=8).contains(&pool.pool_width) {
            return Err(wire("invalid pool image width"));
        }
        if pool.image.native_solver_profile() != expected {
            return Err(wire("native image profile differs from startup offer"));
        }
        Worker::from_scoped_image(run_id, worker_id, &pool.image, pool.pool_width)?
    } else {
        let image: WorkerImage = serde_json::from_slice(seed).map_err(wire)?;
        if !expected.is_standard() || !image.native_solver_profile().is_standard() {
            return Err(wire("two-process images use the standard native profile"));
        }
        Worker::from_trusted_image(run_id, worker_id, &image)?
    };
    Ok(worker)
}

/// Serve the worker until a coordinator shutdown. The seed is one JSON array of
/// initial bodies written to stdin followed by EOF; no simulation tick runs on a timer.
pub fn serve(
    run_id: u64,
    worker_id: u8,
    seed: impl Read,
    stdout: impl Write,
) -> Result<(), TransportError> {
    let mut bytes = Vec::new();
    seed.take((MAX_PAYLOAD + 1) as u64)
        .read_to_end(&mut bytes)?;
    if bytes.len() > MAX_PAYLOAD {
        return Err(wire("oversized seed"));
    }
    let worker = worker_from_seed(run_id, worker_id, &bytes, NativeSolverProfile::Standard32)?;
    serve_worker(run_id, worker_id, worker, stdout)
}

fn worker_from_seed(
    run_id: u64,
    worker_id: u8,
    bytes: &[u8],
    expected: NativeSolverProfile,
) -> Result<Worker, TransportError> {
    // Historical cold seeds are a bare body array. The timed form remains an array
    // so the managed pipe can distinguish cold seeds from sealed native image objects.
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum Seed {
        ProfileV1(
            (
                NativeSeedLayout,
                u8,
                SimulationStep,
                Option<GroundPlane>,
                Vec<Body>,
            ),
        ),
        Pool((u8, SimulationStep, Option<GroundPlane>, Vec<Body>)),
        Terrain((SimulationStep, GroundPlane, Vec<Body>)),
        Timed((SimulationStep, Vec<Body>)),
        Legacy(Vec<Body>),
    }
    let (layout, width, step, ground, bodies) = match serde_json::from_slice(bytes).map_err(wire)? {
        Seed::ProfileV1((
            layout,
            width,
            step,
            ground,
            bodies,
        )) if !layout.profile().is_standard() && (3..=8).contains(&width) => (layout, width, step, ground, bodies),
        Seed::ProfileV1(_) => return Err(wire("invalid explicit native pool profile")),
        Seed::Pool((width, step, ground, bodies)) if (3..=8).contains(&width) => {
            (NativeSeedLayout::Standard32, width, step, ground, bodies)
        }
        Seed::Pool(_) => return Err(wire("invalid worker pool width")),
        Seed::Terrain((step, ground, bodies)) => {
            (NativeSeedLayout::Standard32, 2, step, Some(ground), bodies)
        }
        Seed::Timed((step, bodies)) => (NativeSeedLayout::Standard32, 2, step, None, bodies),
        Seed::Legacy(bodies) => (
            NativeSeedLayout::Standard32,
            2,
            SimulationStep::default(),
            None,
            bodies,
        ),
    };
    if layout.profile() != expected {
        return Err(wire("cold seed profile differs from startup offer"));
    }
    let snapshot = Snapshot {
        step,
        ground,
        supports: Vec::new(),
        tick: 0,
        bodies,
        contacts: 0,
        contact_history: false,
        impulses: vec![],
    };
    Ok(Worker::with_seed_layout(
        run_id, worker_id, snapshot, width, layout,
    )?)
}

fn serve_worker(
    run_id: u64,
    worker_id: u8,
    mut worker: Worker,
    mut stdout: impl Write,
) -> Result<(), TransportError> {
    let session = zenoh::open(config(None)?).wait().map_err(unavailable)?;
    let scope = key(run_id, worker_id);
    let queries = session
        .declare_queryable(scope.as_str())
        .complete(true)
        .with(FifoChannel::new(8))
        .wait()
        .map_err(unavailable)?;
    let endpoint = session
        .info()
        .locators()
        .wait()
        .iter()
        .map(ToString::to_string)
        .find(|ep| endpoint_valid(ep))
        .ok_or_else(|| wire("no loopback TCP listener"))?;
    let ready = Ready {
        profile: worker.native_solver_profile(),
        run_id,
        worker_id,
        endpoint,
        session_id: session.zid().to_string(),
        capability: uuid::Uuid::new_v4().into_bytes(),
    };
    serde_json::to_writer(&mut stdout, &ready).map_err(wire)?;
    writeln!(stdout)?;
    stdout.flush()?;
    loop {
        let query = queries.recv().map_err(unavailable)?;
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        let request_stamp = crate::barrier_trace::now();
        // Native checkpoints are a trusted same-build format. Only the
        // launching coordinator may supply one, request a step, or stop us.
        if !query
            .attachment()
            .is_some_and(|a| a.len() == 16 && a.to_bytes().as_ref() == ready.capability)
        {
            let _ = query.reply_err("unknown worker capability").wait();
            continue;
        }
        let packed_handoff = query.parameters().get(handoff::PARAMETER) == Some("1");
        let operation = query
            .payload()
            .filter(|p| p.len() <= MAX_CHECKPOINT_PAYLOAD)
            .and_then(|p| {
                let bytes = p.to_bytes();
                let operation = if handoff::is_packed(&bytes) {
                    if !packed_handoff {
                        return None;
                    }
                    handoff::decode_request(&bytes).ok()?
                } else {
                    serde_json::from_slice::<Operation<Request>>(&bytes).ok()?
                };
                (p.len() <= operation.payload_limits().0).then_some(operation)
            });
        let reply_limit = operation
            .as_ref()
            .map_or(MAX_PAYLOAD, |op| op.payload_limits().1);
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        let _trace = match &operation {
            Some(Operation::Request(Request {
                command: WorkerCommand::StepIslands { tick, .. },
                ..
            })) => crate::barrier_trace::Scope::begin(tick.saturating_add(1), Some(worker_id)),
            _ => None,
        };
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        crate::barrier_trace::at("request", request_stamp);
        let stop = matches!(operation, Some(Operation::Shutdown));
        worker.set_local_helper(query.parameters().get("gz_world_helper") == Some("1"));
        let response = match operation {
            Some(Operation::Save { sequence, tick }) => checkpoint::encode(
                worker.recovery_image(sequence,tick),
                query.parameters().get(checkpoint::PARAMETER) == Some("1")
            ),
            Some(Operation::StartProbe { tick, span }) if query.parameters().get("gz_scheduled_probe") == Some("1") => {
                encode(&Response::ScheduledProbeStarted(worker.start_scheduled_probe(tick, span)))
            }
            Some(Operation::StartProbe { tick, span }) => {
                encode(&Response::ProbeStarted(worker.start_probe(tick, span)))
            }
            Some(Operation::ReadProbe { start, span }) => {
                encode(&Response::ProbeResult(worker.read_probe(start, span)))
            }
            Some(Operation::Ping) if packed_handoff => {
                encode(&Response::HandoffReadyV1(ready.clone()))
            }
            Some(Operation::Ping) => encode(&Response::Ready(ready.clone())),
            Some(Operation::Request(request)) => {
                #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
                crate::barrier_trace::mark("handle_start", None);
                let answer = worker.handle(request);
                #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
                crate::barrier_trace::mark("handle_end", None);
                encode_answer(
                    answer,
                    worker.native_solver_profile(),
                    query
                        .parameters()
                        .get(observation::parameter(worker.native_solver_profile()))
                        == Some("1"),
                    packed_handoff,
                    reply_limit,
                )
            }
            Some(Operation::DropReply(request)) => {
                let _ = worker.handle(request);
                continue;
            }
            Some(Operation::PreviewImpulse {
                sequence,
                tick,
                impulse,
            }) => encode(&Response::ProspectiveTwist(
                worker.preview_impulse(sequence, tick, &impulse),
            )),
            Some(Operation::Shutdown) => encode(&Response::Stopped),
            None => Err(wire("malformed or oversized worker request")),
        };
        match response {
            Ok(data) => {
                #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
                crate::barrier_trace::mark("reply_submit", None);
                let _ = query.reply(scope.as_str(), data).express(true).wait();
                #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
                crate::barrier_trace::mark("reply_end", None);
            }
            Err(_) => {
                let _ = query
                    .reply_err("invalid worker message or reply exceeds its payload limit")
                    .wait();
            }
        }
        if stop {
            break;
        }
    }
    queries.undeclare().wait().map_err(unavailable)?;
    session.close().wait().map_err(unavailable)
}

struct ChildGuard(Child);
impl ChildGuard {
    fn kill(&mut self) -> io::Result<()> {
        // std::process::Child::kill also succeeds for an already exited child.
        self.0.kill()?;
        self.0.wait()?;
        Ok(())
    }
}
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.kill();
    }
}

/// Writing the seed and reading readiness can each block on an OS pipe.
fn bootstrap<W: Write + Send + 'static>(
    child: &mut ChildGuard,
    input: W,
    output: impl Read + Send + 'static,
    seed: Vec<u8>,
    address: (u64, u8),
    profile: NativeSolverProfile,
    timeout: Duration,
) -> io::Result<(Ready, W)> {
    bootstrap_offer(
        child,
        input,
        output,
        seed,
        address,
        (wire_profile(profile).offer, profile),
        timeout,
    )
}

fn bootstrap_offer<W: Write + Send + 'static>(
    child: &mut ChildGuard,
    mut input: W,
    output: impl Read + Send + 'static,
    seed: Vec<u8>,
    address: (u64, u8),
    expected: (Option<&'static [u8]>, NativeSolverProfile),
    timeout: Duration,
) -> io::Result<(Ready, W)> {
    let (offer, profile) = expected;
    let (tx, rx) = mpsc::sync_channel(1);
    let reader = thread::Builder::new()
        .name("physics-worker-readiness".into())
        .spawn(move || {
            let result = (|| {
                let mut output = BufReader::new(output);
                if let Some(expected) = offer {
                    let mut offer = Vec::new();
                    output.by_ref().take(64).read_until(b'\n', &mut offer)?;
                    if offer != expected {
                        return Err(io_error(format!(
                            "worker does not offer the requested managed capability {}",
                            String::from_utf8_lossy(expected).trim_end()
                        )));
                    }
                }
                input.write_all(&seed)?;
                input.write_all(b"\n")?;
                let ready = Ready::read(output, address.0, address.1)?;
                if ready.profile != profile {
                    return Err(io_error(
                        "worker did not acknowledge the requested native solver profile",
                    ));
                }
                Ok((ready, input))
            })();
            let _ = tx.send(result);
        })?;
    let result = rx.recv_timeout(timeout);
    if result.is_err() {
        child.kill()?;
    }
    reader
        .join()
        .map_err(|_| io_error("readiness thread failed"))?;
    result.map_err(io_error)?
}

/// Owner of exactly one child process. Drop fences and reaps it even when a
/// setup/transport error interrupts the happy-path shutdown.
pub struct WorkerProcess {
    child: ChildGuard,
    session: zenoh::Session,
    ready: Ready,
    key: String,
    executable: PathBuf,
    seed: Vec<u8>,
    traffic: Traffic,
    pub(crate) unavailable: bool,
    packed_observations: bool,
    pub(crate) native_work_timeout: Duration,
    packed_handoffs: bool,
    supports_packed_handoffs: bool,
    fault_after: Option<(usize, ProcessFault)>,
}
impl WorkerProcess {
    pub fn spawn(
        executable: &Path,
        run_id: u64,
        worker_id: u8,
        initial: &[Body],
    ) -> io::Result<Self> {
        Self::spawn_with_step(
            executable,
            run_id,
            worker_id,
            initial,
            SimulationStep::default(),
        )
    }

    pub fn spawn_with_step(
        executable: &Path,
        run_id: u64,
        worker_id: u8,
        initial: &[Body],
        step: SimulationStep,
    ) -> io::Result<Self> {
        Self::spawn_with_ground(executable, run_id, worker_id, initial, step, None)
    }

    pub fn spawn_with_ground(
        executable: &Path,
        run_id: u64,
        worker_id: u8,
        initial: &[Body],
        step: SimulationStep,
        ground: Option<GroundPlane>,
    ) -> io::Result<Self> {
        let bytes = if let Some(ground) = ground {
            encode(&(step, ground, initial))
        } else if step.is_default() {
            encode(initial)
        } else {
            encode(&(step, initial))
        };
        Self::spawn_encoded(executable, run_id, worker_id, bytes.map_err(io_error)?,NativeSolverProfile::Standard32)
    }

    /// Bootstrap a local endpoint with a width-bound authority and receipt
    /// domain. All participants of the same pool use the same width.
    pub fn spawn_in_pool(
        executable: &Path,
        run: u64,
        worker: u8,
        width: u8,
        initial: &[Body],
        step: SimulationStep,
        ground: Option<GroundPlane>,
    ) -> io::Result<Self> {
        Self::spawn_in_pool_with_layout(
            executable,
            run,
            worker,
            width,
            initial,
            step,
            ground,
            NativeSeedLayout::Standard32,
        )
    }
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn spawn_in_pool_with_layout(
        executable: &Path,
        run: u64,
        worker: u8,
        width: u8,
        initial: &[Body],
        step: SimulationStep,
        ground: Option<GroundPlane>,
        layout: NativeSeedLayout,
    ) -> io::Result<Self> {
        if !(3..=8).contains(&width) || worker >= width {
            return Err(io_error("invalid pool worker or width"));
        }
        let bytes = if layout == NativeSeedLayout::Standard32 {
            encode(&(width, step, ground, initial))
        } else {
            encode(&(layout, width, step, ground, initial))
        }.map_err(io_error)?;
        Self::spawn_encoded(executable, run, worker, bytes, layout.profile())
    }

    pub fn spawn_from_image(executable: &Path, image: &WorkerImage) -> io::Result<Self> {
        if !image.native_solver_profile().is_standard() {
            return Err(io_error("the two-worker entry point requires the standard native model"));
        }
        Self::spawn_encoded(
            executable,
            image.run,
            image.worker,
            encode_limited(image, MAX_WORKER_IMAGE_PAYLOAD).map_err(io_error)?,
            NativeSolverProfile::Standard32,
        )
    }

    pub(crate) fn spawn_from_pool_image(
        executable: &Path,
        width: u8,
        image: &WorkerImage,
    ) -> io::Result<Self> {
        Self::spawn_encoded(
            executable,
            image.run,
            image.worker,
            Self::encode_pool_image(width, image).map_err(io_error)?,
            image.native_solver_profile(),
        )
    }

    fn spawn_encoded(
        executable: &Path,
        run_id: u64,
        worker_id: u8,
        seed: Vec<u8>,
        profile: NativeSolverProfile,
    ) -> io::Result<Self> {
        // Both encodings are produced by serde_json: cold seeds are arrays,
        // worker images objects. Only the private child pipe accepts the latter.
        let mode = wire_profile(profile).mode(seed.first() == Some(&b'{'));
        let mut child = ChildGuard(
            Command::new(executable)
                .args([
                    "--run",
                    &run_id.to_string(),
                    "--worker",
                    &worker_id.to_string(),
                    mode,
                ])
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .stderr(Stdio::inherit())
                .spawn()?,
        );
        let input = child.0.stdin.take().expect("piped worker stdin");
        let output = child.0.stdout.take().expect("piped worker stdout");
        let (ready, owner_pipe) = bootstrap(
            &mut child,
            input,
            output,
            seed.clone(),
            (run_id,worker_id),
            profile,
            START_TIMEOUT,
        )?;
        child.0.stdin = Some(owner_pipe);
        let session = zenoh::open(config(Some(&ready.endpoint))?)
            .wait()
            .map_err(io_error)?;
        let mut this = Self {
            child,
            session,
            ready,
            key: key(run_id, worker_id),
            executable: executable.to_owned(),
            seed,
            traffic: Traffic::default(),
            unavailable: false,
            packed_observations: true,
            native_work_timeout: REQUEST_TIMEOUT,
            packed_handoffs: false,
            supports_packed_handoffs: false,
            fault_after: None,
        };
        this.confirm_ready()?;
        Ok(this)
    }

    fn confirm_ready(&mut self) -> io::Result<()> {
        let deadline = Instant::now() + START_TIMEOUT;
        loop {
            match self.exchange(Operation::Ping, Duration::from_millis(200)) {
                Ok(Response::Ready(r)) if r == self.ready => {
                    self.supports_packed_handoffs = false;
                    return Ok(());
                }
                Ok(Response::HandoffReadyV1(r)) if r == self.ready => {
                    self.supports_packed_handoffs = true;
                    return Ok(());
                }
                Err(TransportError::Unavailable(_)) if Instant::now() < deadline => {
                    thread::sleep(Duration::from_millis(10))
                }
                Err(e) => return Err(io_error(e)),
                Ok(_) => return Err(io_error("remote readiness identity mismatch")),
            }
        }
    }

    pub fn pid(&self) -> u32 {
        self.child.0.id()
    }
    pub fn endpoint(&self) -> &str {
        &self.ready.endpoint
    }
    pub fn session_id(&self) -> &str {
        &self.ready.session_id
    }
    pub(crate) fn drop_acknowledgement_after(&mut self, earlier: usize) {
        self.fault_after = Some((earlier, ProcessFault::Withhold));
    }
    pub(crate) fn terminate_after_command(&mut self, earlier: usize) {
        self.fault_after = Some((earlier, ProcessFault::ExitAfterReply));
    }
    /// Request lossless packed observations. Historical workers answer the same
    /// query with JSON. Physics operations and journal contents are identical.
    pub fn set_packed_observations(&mut self, enabled: bool) {
        self.packed_observations = enabled;
    }
    /// Select packed native exports and prepares when the connected worker advertised v1.
    /// Returns whether this connection can use the requested encoding.
    pub fn set_packed_handoffs(&mut self, enabled: bool) -> bool {
        self.packed_handoffs = enabled;
        self.uses_packed_handoffs()
    }
    pub(crate) fn uses_packed_handoffs(&self) -> bool {
        self.packed_handoffs && self.supports_packed_handoffs
    }
    pub fn traffic(&self) -> Traffic {
        self.traffic
    }
    pub fn kill(&mut self) -> io::Result<()> {
        self.child.kill()?;
        self.session.close().wait().map_err(io_error)
    }

    /// Fence the old process before its replacement binds an independent port.
    /// The caller replays committed requests to restore the qualified barrier.
    pub fn restart(&mut self) -> io::Result<()> {
        self.kill()?;
        let mut next = Self::spawn_encoded(
            &self.executable,
            self.ready.run_id,
            self.ready.worker_id,
            self.seed.clone(),
            self.ready.profile,
        )?;
        next.packed_observations = self.packed_observations;
        next.native_work_timeout = self.native_work_timeout;
        next.packed_handoffs = self.packed_handoffs;
        next.traffic.requests += self.traffic.requests;
        next.traffic.replies += self.traffic.replies;
        next.traffic.request_bytes += self.traffic.request_bytes;
        next.traffic.reply_bytes += self.traffic.reply_bytes;
        *self = next;
        Ok(())
    }

    fn exchange(
        &mut self,
        op: Operation<&Request>,
        timeout: Duration,
    ) -> Result<Response, TransportError> {
        let reply = self
            .dispatch(op, timeout)
            .and_then(|pending| self.collect(pending));
        self.unavailable = matches!(reply, Err(TransportError::Unavailable(_)));
        reply
    }

    fn dispatch(
        &mut self,
        op: Operation<&Request>,
        timeout: Duration,
    ) -> Result<PendingQuery, TransportError> {
        self.dispatch_local(op, timeout, LocalHint::None)
    }

    fn dispatch_local(
        &mut self,
        op: Operation<&Request>,
        timeout: Duration,
        hint: LocalHint,
    ) -> Result<PendingQuery, TransportError> {
        if self.child.0.try_wait()?.is_some() {
            return Err(unavailable("worker already exited"));
        }
        if timeout.is_zero() || timeout > START_TIMEOUT {
            return Err(wire(
                "request timeout must be positive and at most five seconds",
            ));
        }
        // Qualification can discard a real worker acknowledgement after the
        // handler runs. Readiness and baseline captures never consume the arm.
        let fault = if matches!(op, Operation::Request(_)) && let Some((remaining, kind)) = &mut self.fault_after {
            if *remaining == 0 {
                let selected = *kind;
                self.fault_after = None;
                Some(selected)
            } else {
                *remaining -= 1;
                None
            }
        } else {
            None
        };
        let op = match (op, fault) {
            (Operation::Request(r), Some(ProcessFault::Withhold)) => Operation::DropReply(r),
            (op, _) => op,
        };
        let (request_limit, reply_limit) = op.payload_limits();
        let negotiate = matches!(op, Operation::Ping);
        let packed_handoff = self.uses_packed_handoffs() && handoff::use_selector(&op);
        let payload = if packed_handoff && let Some(bytes) = handoff::encode_request(&op)? {
            bytes
        } else {
            encode_limited(&op, request_limit)?
        };
        let size = payload.len();
        let deadline = Instant::now() + timeout;
        let key = zenoh::key_expr::KeyExpr::try_from(self.key.as_str()).map_err(wire)?;
        let packed = self.packed_observations && matches!(op, Operation::Request(_));
        let params = if negotiate || packed_handoff {
            handoff::SELECTOR
        } else if hint == LocalHint::PackedCheckpoint {
            checkpoint::SELECTOR
        } else if hint == LocalHint::ScheduledProbe {
            "gz_scheduled_probe=1"
        } else {
            match (packed, hint == LocalHint::Helper) {
                (false, false) => "",
                (true, helper) => observation::selector(self.ready.profile, helper),
                (false, true) => "gz_world_helper=1",
            }
        };
        let selector = zenoh::query::Selector::owned(key, params);
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        let fifo = crate::barrier_trace::reply_fifo(self.ready.worker_id);
        #[cfg(not(all(target_os = "linux", feature = "barrier-trace")))]
        let fifo = FifoChannel::new(1);
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        crate::barrier_trace::mark("submit", Some(self.ready.worker_id));
        let replies = self
            .session
            .get(selector)
            // A private worker has exactly one responder and one application reply.
            // Deliver it immediately instead of waiting for a consolidation window.
            .consolidation(zenoh::query::ConsolidationMode::None)
            .express(true)
            .attachment(self.ready.capability.to_vec())
            .payload(payload)
            .timeout(timeout)
            .with(fifo)
            .wait()
            .map_err(unavailable)?;
        self.traffic.requests += 1;
        self.traffic.request_bytes += size as u64;
        Ok(PendingQuery {
            exit_after_reply: fault == Some(ProcessFault::ExitAfterReply),
            accepts_packed_image: hint == LocalHint::PackedCheckpoint && matches!(op,Operation::Save { .. }),
            replies,
            deadline,
            timeout,
            reply_limit,
            accepts_packed_export: packed_handoff && handoff::expects_export(&op),
        })
    }

    fn collect(&mut self, query: PendingQuery) -> Result<Response, TransportError> {
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        crate::barrier_trace::mark("collect_start", Some(self.ready.worker_id));
        let reply = query
            .replies
            .recv_deadline(query.deadline)
            .map_err(unavailable)?
            .ok_or_else(|| unavailable(format!("worker {} reply deadline expired after {:?}", self.ready.worker_id, query.timeout)))?;
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        crate::barrier_trace::mark("received", Some(self.ready.worker_id));
        let sample = reply.result().map_err(|error| {
            // Zenoh's Display reports only byte count and encoding. Preserve a
            // bounded UTF-8 error payload so the caller can diagnose a refusal.
            let payload = error.payload();
            if payload.len() <= 1024
                && let Ok(reason) = payload.try_to_string()
            {
                return unavailable(format!("worker query refused: {reason}"));
            }
            unavailable(error)
        })?;
        if sample.key_expr().as_str() != self.key || sample.payload().len() > query.reply_limit {
            return Err(wire("reply escaped the scoped key or payload limit"));
        }
        self.traffic.replies += 1;
        self.traffic.reply_bytes += sample.payload().len() as u64;
        if query.exit_after_reply {
            // The genuine process responded. Kill it before delivering the
            // result to the caller, exercising an ambiguous completed command.
            self.kill()?;
            return Err(unavailable("worker terminated after delivering the selected reply"));
        }
        let bytes = sample.payload().to_bytes();
        if checkpoint::is_packed(&bytes) {
            if !query.accepts_packed_image { return Err(wire("unsolicited packed worker image")); }
            Ok(Response::Saved(Ok(Box::new(checkpoint::decode(&bytes)?))))
        } else if handoff::is_packed(&bytes) {
            if !query.accepts_packed_export {
                return Err(wire("unexpected packed export reply"));
            }
            Ok(Response::Answer(Ok(handoff::decode_reply(&bytes)?)))
        } else if observation::is_packed(&bytes) {
            Ok(Response::Answer(Ok(observation::decode_with_profile(
                &bytes,
                self.ready.profile,
            )?)))
        } else {
            serde_json::from_slice(&bytes).map_err(wire)
        }
    }

    pub(crate) fn preview_impulse(
        &mut self,
        sequence: u64,
        tick: u64,
        impulse: crate::protocol::SpatialImpulse,
    ) -> Result<crate::protocol::VelocityEdit, TransportError> {
        match self.exchange(
            Operation::PreviewImpulse {
                sequence,
                tick,
                impulse,
            },
            self.native_work_timeout,
        )? {
            Response::ProspectiveTwist(value) => Ok(value?),
            _ => Err(wire("expected native impulse preview")),
        }
    }

    pub(crate) fn start_probe(&mut self, tick: u64, span: u16) -> Result<(), TransportError> {
        match self.exchange(Operation::StartProbe { tick, span }, REQUEST_TIMEOUT)? {
            Response::ProbeStarted(result) => Ok(result?),
            _ => Err(wire("expected timing probe acknowledgement")),
        }
    }
    /// Submit independent reads before collecting their replies. The per-peer
    /// result records which sessions failed without dropping later valid replies.
    fn read_pool<'a>(
        peers: &mut [Self], participants: u8, operation: impl Fn(usize) -> Operation<&'a Request>,
        timeout: Duration, hint: LocalHint,
    ) -> Vec<(usize, Result<Response, TransportError>)> {
        let queries: Vec<_> = peers.iter_mut().enumerate()
            .filter(|(i, _)| participants & (1 << i) != 0)
            .map(|(i, peer)| (i, peer.dispatch_local(operation(i), timeout, hint))).collect();
        queries.into_iter().map(|(i,query)| {
            let peer = &mut peers[i];
            let result = query.and_then(|q| peer.collect(q));
            peer.unavailable = matches!(result, Err(TransportError::Unavailable(_)));
            (i,result)
        }).collect()
    }
    pub(crate) fn start_pool_probes(peers: &mut [Self], tick: u64, span: u16, scheduled: bool, timeout: Duration, participants: u8) -> Result<u8, TransportError> {
        let hint = if scheduled { LocalHint::ScheduledProbe } else { LocalHint::None };
        let mut mask = 0;
        for (w, reply) in Self::read_pool(peers, participants, |_| Operation::StartProbe {tick, span}, timeout, hint) {
            match reply? {
                Response::ScheduledProbeStarted(result) if scheduled => { result?; mask |= 1 << w; }
                // Historical peers ignore the scheduling selector and sample serially.
                Response::ProbeStarted(result) => result?,
                _ => return Err(wire("expected timing probe acknowledgement")),
            }
        }
        Ok(mask)
    }
    pub(crate) fn read_pool_probes(peers: &mut [Self], start: u64, span: u16, timeout: Duration, participants: u8) -> Result<Vec<Option<Vec<crate::physics::ComponentWork>>>, TransportError> {
        let mut reports=vec![Some(Vec::new()); peers.len()];
        for (w,reply) in Self::read_pool(peers, participants, |_| Operation::ReadProbe {start,span}, timeout, LocalHint::None) {
            reports[w]=match reply? {
                Response::ProbeResult(rows) => rows,
                _ => return Err(wire("expected timing probe report")),
            };
        }
        Ok(reports)
    }
    pub(crate) fn read_probe(
        &mut self,
        start: u64,
        span: u16,
    ) -> Result<Option<Vec<crate::physics::ComponentWork>>, TransportError> {
        match self.exchange(Operation::ReadProbe { start, span }, REQUEST_TIMEOUT)? {
            Response::ProbeResult(rows) => Ok(rows),
            _ => Err(wire("expected timing probe report")),
        }
    }

    pub(crate) fn fetch_pool_images(peers: &mut [Self], sequences: &[u64], tick: u64) -> Vec<Result<WorkerImage, TransportError>> {
        debug_assert_eq!(peers.len(), sequences.len());
        Self::read_pool(peers, u8::MAX, |i| Operation::Save {sequence:sequences[i], tick}, START_TIMEOUT, LocalHint::PackedCheckpoint)
            .into_iter().map(|(i, reply)| Self::image_answer(reply?, peers[i].ready.profile)).collect()
    }

    fn image_answer(reply: Response, profile: NativeSolverProfile) -> Result<WorkerImage, TransportError> {
        let Response::Saved(image) = reply else {
            return Err(wire("unexpected checkpoint response"));
        };
        let image = image?;
        if image.native_solver_profile() != profile {
            return Err(wire("worker checkpoint differs from negotiated solver profile"));
        }
        Ok(*image)
    }

    pub(crate) fn fetch_image(
        &mut self,
        sequence: u64,
        tick: u64,
    ) -> Result<WorkerImage, TransportError> {
        let reply = self.exchange(Operation::Save { sequence, tick }, START_TIMEOUT)?;
        Self::image_answer(reply, self.ready.profile)
    }

    pub(crate) fn encode_restart_image(image: &WorkerImage) -> Result<Vec<u8>, TransportError> {
        encode_limited(image, MAX_WORKER_IMAGE_PAYLOAD)
    }

    pub(crate) fn encode_pool_image(
        width: u8,
        image: &WorkerImage,
    ) -> Result<Vec<u8>, TransportError> {
        #[derive(Serialize)]
        struct PoolSeed<'a> {
            pool_width: u8,
            image: &'a WorkerImage,
        }
        encode_limited(
            &PoolSeed {
                pool_width: width,
                image,
            },
            MAX_WORKER_IMAGE_PAYLOAD,
        )
    }

    pub(crate) fn retain_restart_seed(&mut self, encoded: Vec<u8>) {
        self.seed = encoded;
    }

    pub fn request(&mut self, request: &Request) -> Result<Reply, TransportError> {
        self.request_with_timeout(request, self.native_work_timeout)
    }
    pub fn request_with_timeout(
        &mut self,
        request: &Request,
        timeout: Duration,
    ) -> Result<Reply, TransportError> {
        self.receive_request(request, Operation::Request(request), timeout)
    }
    /// Exercise ambiguous delivery: execute this command in the real process and
    /// omit its entire reply. Retry the same sequence and unchanged payload.
    pub fn request_dropping_reply(
        &mut self,
        request: &Request,
        timeout: Duration,
    ) -> Result<Reply, TransportError> {
        self.receive_request(request, Operation::DropReply(request), timeout)
    }
    fn receive_request(
        &mut self,
        request: &Request,
        op: Operation<&Request>,
        timeout: Duration,
    ) -> Result<Reply, TransportError> {
        let profile = self.ready.profile;
        match &request.command {
            WorkerCommand::PrepareWarm { handoff }
            | WorkerCommand::PrepareSolver { handoff }
            | WorkerCommand::PrepareOrdered { handoff, .. }
                if !handoff.checkpoint.matches_profile(profile) => {
                    return Err(wire("native handoff differs from destination solver profile"));
                }
            _ => {}
        }
        let reply = Self::checked_answer(request, self.exchange(op, timeout)?)?;
        if let crate::protocol::Outcome::WarmExported(handoff) = &reply.outcome
            && !handoff.checkpoint.matches_profile(profile)
        {
            return Err(wire("native export differs from source solver profile"));
        }
        Ok(reply)
    }

    fn checked_answer(request: &Request, answer: Response) -> Result<Reply, TransportError> {
        match answer {
            Response::Answer(Ok(reply))
                if (reply.run_id, reply.worker_id, reply.sequence)
                    == (request.run_id, request.worker_id, request.sequence) =>
            {
                Ok(reply)
            }
            Response::Answer(Err(error)) => Err(TransportError::Protocol(error)),
            _ => Err(wire(
                "reply did not match the request identity and sequence",
            )),
        }
    }
    // Submit both independent queries before receiving either answer. Each
    // receiver retains its original absolute deadline and capability-scoped key.
    pub(crate) fn simultaneous_steps(
        peers: &mut [Self; 2],
        requests: &[Request; 2],
    ) -> [Result<Reply, TransportError>; 2] {
        let [a, b] = peers;
        let send_a = a.dispatch(Operation::Request(&requests[0]), a.native_work_timeout);
        let send_b = b.dispatch(Operation::Request(&requests[1]), b.native_work_timeout);
        let finish =
            |worker: &mut Self, pending: Result<PendingQuery, TransportError>, r: &Request| {
                let result = pending
                    .and_then(|query| worker.collect(query))
                    .and_then(|reply| Self::checked_answer(r, reply));
                worker.unavailable = matches!(result, Err(TransportError::Unavailable(_)));
                result
            };
        [
            finish(a, send_a, &requests[0]),
            finish(b, send_b, &requests[1]),
        ]
    }

    fn submit_pool_step(
        &mut self,
        request: &Request,
        helper_mask: u8,
        timeout: Duration,
    ) -> Result<PendingQuery, TransportError> {
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        crate::barrier_trace::mark("dispatch_start", Some(self.ready.worker_id));
        self.dispatch_local(
            Operation::Request(request),
            timeout,
            if helper_mask & (1 << request.worker_id) != 0 { LocalHint::Helper } else { LocalHint::None },
        )
    }
    fn receive_pool_step(
        &mut self,
        request: &Request,
        query: Result<PendingQuery, TransportError>,
    ) -> Result<Reply, TransportError> {
        let result = query
            .and_then(|q| self.collect(q))
            .and_then(|reply| Self::checked_answer(request, reply));
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        crate::barrier_trace::mark("decoded", Some(self.ready.worker_id));
        self.unavailable = matches!(result, Err(TransportError::Unavailable(_)));
        result
    }
    pub(crate) fn pool_steps(
        peers: &mut [Self],
        requests: &[Request],
        helper_mask: u8,
    ) -> Vec<Result<Reply, TransportError>> {
        assert_eq!(peers.len(), requests.len());
        let queries = peers
            .iter_mut()
            .zip(requests)
            .map(|(p, r)| p.submit_pool_step(r, helper_mask, p.native_work_timeout))
            .collect::<Vec<_>>();
        peers
            .iter_mut()
            .zip(requests)
            .zip(queries)
            .map(|((p, r), q)| p.receive_pool_step(r, q))
            .collect()
    }
    /// Retained replies already have verified run/worker/sequence envelopes. Query only
    /// workers whose original reply is missing, sending all requests before waiting.
    pub(crate) fn confirm_pool_steps(
        peers: &mut [Self],
        requests: &[Request],
        helper_mask: u8,
        seen: Vec<Option<Reply>>,
        timeout: Duration,
    ) -> Vec<Result<Reply, TransportError>> {
        assert_eq!(peers.len(), requests.len());
        assert_eq!(peers.len(), seen.len());
        let mut submitted = Vec::new();
        for ((peer, request), cached) in peers.iter_mut().zip(requests).zip(&seen) {
            if cached.is_none() {
                submitted.push(peer.submit_pool_step(request, helper_mask, timeout));
            }
        }
        let mut queries = submitted.into_iter();
        peers
            .iter_mut()
            .zip(requests)
            .zip(seen)
            .map(|((peer, request), cached)| match cached {
                Some(reply) => Ok(reply),
                None => peer.receive_pool_step(
                    request,
                    queries.next().expect("one query per missing reply"),
                ),
            })
            .collect()
    }
    pub(crate) fn has_exited(&mut self) -> std::io::Result<bool> {
        Ok(self.child.0.try_wait()?.is_some())
    }

    pub fn shutdown(&mut self) -> io::Result<()> {
        if let Some(status) = self.child.0.try_wait()? {
            self.session.close().wait().map_err(io_error)?;
            return if status.success() {
                Ok(())
            } else {
                Err(io_error(format!("worker exited {status}")))
            };
        }
        let response = self.exchange(Operation::Shutdown, REQUEST_TIMEOUT);
        let deadline = Instant::now() + REQUEST_TIMEOUT;
        let status = loop {
            if let Some(status) = self.child.0.try_wait()? {
                break status;
            }
            if Instant::now() >= deadline {
                self.kill()?;
                return Err(io_error("worker shutdown timed out"));
            }
            thread::sleep(Duration::from_millis(5));
        };
        self.session.close().wait().map_err(io_error)?;
        if !status.success() {
            return Err(io_error(format!("worker exited {status}")));
        }
        match response {
            Ok(Response::Stopped) => Ok(()),
            Err(e) => Err(io_error(e)),
            Ok(_) => Err(io_error("unexpected shutdown reply")),
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/transport.rs"]
mod tests;
