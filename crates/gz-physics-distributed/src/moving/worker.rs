// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One synchronous capability on the managed child's private pipes. The child
//! retains its process lease until EOF or death. Transport timeouts fence the child.
use super::*;
use bincode::Options;
use serde::de::DeserializeOwned;
use std::{
    fs::OpenOptions,
    io::{self, Read, Write},
    process::{Child, Command, Stdio},
    sync::mpsc::{self, Receiver, Sender},
    time::{Duration, Instant},
};
pub(crate) mod lifetime;
#[cfg(target_os = "linux")]
mod local_pipe;
mod network;
pub(crate) use network::ForwardedPorts;

/// Transport for a managed native solver. Both modes keep a local process lease.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkerTransport {
    #[default]
    Pipe,
    LoopbackZenoh,
}

const TIMEOUT: Duration = Duration::from_secs(6);
pub(crate) const LIMIT: usize = 192 * 1024;
pub(crate) fn codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(LIMIT as u64)
        .reject_trailing_bytes()
}
pub(crate) fn encode<T: Serialize>(v: &T) -> io::Result<Vec<u8>> {
    codec().serialize(v).map_err(io::Error::other)
}
fn read_frame(r: &mut impl Read) -> io::Result<Vec<u8>> {
    let mut len = [0; 4];
    r.read_exact(&mut len)?;
    let n = u32::from_le_bytes(len) as usize;
    if n > LIMIT {
        return Err(io::Error::other("oversized moving worker frame"));
    }
    let mut bytes = vec![0; n];
    r.read_exact(&mut bytes)?;
    Ok(bytes)
}
fn write_frame(w: &mut impl Write, bytes: &[u8]) -> io::Result<()> {
    if bytes.len() > LIMIT {
        return Err(io::Error::other("oversized moving worker frame"));
    }
    w.write_all(&(bytes.len() as u32).to_le_bytes())?;
    w.write_all(bytes)?;
    w.flush()
}
#[derive(Serialize, Deserialize)]
pub(super) enum Op {
    Install(Box<Seed>),
    Fence,
    Step,
    Torque(f64),
    Detach,
}
#[derive(Serialize, Deserialize)]
struct Request<C> {
    token: [u8; 16],
    seq: u64,
    tick: u64,
    cmd: C,
}
#[derive(Serialize, Deserialize)]
pub(super) enum Answer {
    Empty,
    State(Box<Part>),
    Detached(Option<(Motion, ArticulationLinkState)>),
}
#[derive(Serialize, Deserialize)]
struct Receipt<A> {
    digest: [u8; 32],
    steps: u64,
    answer: std::result::Result<A, String>,
}
#[derive(Serialize, Deserialize)]
struct Bootstrap {
    magic: [u8; 8],
    slot: u8,
    token: [u8; 16],
    spec_json: Vec<u8>,
}
const MAGIC: [u8; 8] = *b"GZMVWK01";

/// Serve the versioned local moving-encounter worker. Parent EOF ends the lease.
pub fn serve(slot: u8, lease: &Path) -> io::Result<()> {
    serve_profile(slot, lease, MAGIC, |spec: MovingSpec| {
        spec.validate()?;
        let mut resident: Option<Resident> = None;
        let mut tick = 0;
        Ok(move |request_tick, cmd, steps: &mut u64| {
            let answer = (|| -> Result<Answer> {
                match cmd {
                    Op::Fence => {
                        resident = None;
                        Ok(Answer::Empty)
                    }
                    Op::Install(part) if resident.is_none() => {
                        let world = Resident::load(&spec, request_tick, &part)?;
                        let snapshot = world.capture(&spec)?;
                        resident = Some(world);
                        tick = request_tick;
                        Ok(Answer::State(Box::new(snapshot)))
                    }
                    Op::Install(_) => Err(invalid("installation requires a fenced slot")),
                    _ if request_tick != tick => {
                        Err(invalid("moving command at incorrect barrier"))
                    }
                    cmd => {
                        let world = resident
                            .as_mut()
                            .ok_or(invalid("empty moving worker slot"))?;
                        match cmd {
                            Op::Step => {
                                world.advance()?;
                                tick += 1;
                                *steps += 1;
                            }
                            Op::Torque(e) => world.effort(e)?,
                            Op::Detach => return Ok(Answer::Detached(world.detached(&spec)?)),
                            _ => unreachable!(),
                        }
                        Ok(Answer::State(Box::new(world.capture(&spec)?)))
                    }
                }
            })();
            // A failed integration may have dirtied a private world. Keep it fenced.
            if answer.is_err() {
                resident = None
            }
            answer
        })
    })
}

pub(crate) fn serve_profile<S, C, A, F>(
    slot: u8,
    lease: &Path,
    magic: [u8; 8],
    setup: impl FnOnce(S) -> Result<F>,
) -> io::Result<()>
where
    S: DeserializeOwned,
    C: DeserializeOwned,
    A: Serialize,
    F: FnMut(u64, C, &mut u64) -> Result<A>,
{
    serve_profile_at(slot, lease, magic, None, setup)
}

pub(crate) fn serve_profile_at<S, C, A, F>(
    slot: u8,
    lease: &Path,
    magic: [u8; 8],
    forward: Option<ForwardedPorts>,
    setup: impl FnOnce(S) -> Result<F>,
) -> io::Result<()>
where
    S: DeserializeOwned,
    C: DeserializeOwned,
    A: Serialize,
    F: FnMut(u64, C, &mut u64) -> Result<A>,
{
    let (mut reader, mut writer) = (io::stdin().lock(), io::stdout().lock());
    let boot = read_frame(&mut reader)?;
    let network = boot.strip_prefix(network::OFFER);
    if forward.is_some() && network.is_none() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "forwarded worker requires a Zenoh bootstrap",
        ));
    }
    let payload = network.unwrap_or(&boot);
    let seed: Bootstrap = codec().deserialize(payload).map_err(io::Error::other)?;
    if seed.magic != magic || seed.slot != slot {
        return Err(io::Error::other("incorrect moving worker offer"));
    }
    let mut handle = setup(serde_json::from_slice(&seed.spec_json)?).map_err(io::Error::other)?;
    let mut options = OpenOptions::new();
    options.read(true).write(true).create(true).truncate(false);
    #[cfg(target_os = "linux")]
    if forward.is_some() {
        use rustix::fs::OFlags;
        use std::os::unix::fs::OpenOptionsExt;
        options.custom_flags((OFlags::NONBLOCK | OFlags::NOCTTY).bits() as i32);
    }
    let lease = options.open(lease)?;
    if forward.is_some() && !lease.metadata()?.is_file() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "forwarded worker lease must be a regular file",
        ));
    }
    let deadline = Instant::now() + TIMEOUT / 2;
    loop {
        match lease.try_lock() {
            Ok(()) => break,
            Err(std::fs::TryLockError::WouldBlock) if Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(5))
            }
            Err(e) => return Err(io::Error::other(e)),
        }
    }
    let mut commands = Commands {
        token: seed.token,
        sequence: 0,
        steps: 0,
        last: None,
        #[cfg(feature = "sphere-pair-trace")]
        magic,
        #[cfg(feature = "sphere-pair-trace")]
        slot,
    };
    if network.is_some() {
        drop(reader);
        return network::serve(&mut writer, payload, seed.token, commands, handle, forward);
    }
    write_frame(&mut writer, &Sha256::digest(payload))?;
    loop {
        let bytes = match read_frame(&mut reader) {
            Ok(b) => b,
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(()),
            Err(e) => return Err(e),
        };
        write_frame(&mut writer, commands.apply(&bytes, &mut handle)?)?;
    }
}

struct Commands {
    token: [u8; 16],
    sequence: u64,
    steps: u64,
    last: Option<([u8; 32], Vec<u8>)>,
    #[cfg(feature = "sphere-pair-trace")]
    magic: [u8; 8],
    #[cfg(feature = "sphere-pair-trace")]
    slot: u8,
}
impl Commands {
    fn apply<C: DeserializeOwned, A: Serialize>(
        &mut self,
        bytes: &[u8],
        handle: &mut impl FnMut(u64, C, &mut u64) -> Result<A>,
    ) -> io::Result<&[u8]> {
        let digest: [u8; 32] = Sha256::digest(bytes).into();
        if let Some((prev, _)) = &self.last
            && *prev == digest
        {
            return Ok(self.last.as_ref().unwrap().1.as_slice());
        }
        let req: Request<C> = codec().deserialize(bytes).map_err(io::Error::other)?;
        if req.token != self.token || self.sequence.checked_add(1) != Some(req.seq) {
            return Err(io::Error::other("stale moving worker command"));
        }
        #[cfg(feature = "sphere-pair-trace")]
        let _timing = (self.magic == crate::multi_sphere::MAGIC).then(|| {
            crate::multi_sphere::timing::Scope::begin(
                "worker",
                "request",
                req.tick,
                Some(self.slot),
            )
        });
        let answer = handle(req.tick, req.cmd, &mut self.steps).map_err(|e| e.to_string());
        let reply = crate::multi_sphere::timed!(
            receipt_encode_ns,
            encode(&Receipt {
                digest,
                steps: self.steps,
                answer
            })
        )?;
        self.sequence = req.seq;
        Ok(&self.last.insert((digest, reply)).1)
    }
}

/// Receipt identity for the sole outstanding request on a native worker.
pub(crate) struct Pending {
    seq: u64,
    digest: [u8; 32],
    deadline: Instant,
    replay: Option<Vec<u8>>,
}

enum Channel {
    #[cfg(target_os = "linux")]
    Pipe(local_pipe::LocalPipe),
    Threaded {
        outbound: Sender<(Vec<u8>, Instant)>,
        inbound: std::sync::Mutex<Receiver<io::Result<Vec<u8>>>>,
    },
}
impl Channel {
    fn new(child: &mut Child, transport: WorkerTransport, token: [u8; 16]) -> io::Result<Self> {
        let (mut input, mut output) = (child.stdin.take().unwrap(), child.stdout.take().unwrap());
        #[cfg(target_os = "linux")]
        if transport == WorkerTransport::Pipe {
            return Ok(Self::Pipe(local_pipe::LocalPipe::new(input, output)?));
        }
        let (outbound, commands) = mpsc::channel::<(Vec<u8>, Instant)>();
        let (responses, inbound) = mpsc::channel();
        std::thread::Builder::new()
            .name("native-physics-rpc".into())
            .spawn(move || {
                if transport == WorkerTransport::LoopbackZenoh {
                    if let Err(e) =
                        network::exchange(&mut input, &mut output, commands, &responses, token)
                    {
                        let _ = responses.send(Err(e));
                    }
                    return;
                }
                while let Ok((cmd, _deadline)) = commands.recv() {
                    let response =
                        write_frame(&mut input, &cmd).and_then(|()| read_frame(&mut output));
                    let failed = response.is_err();
                    if responses.send(response).is_err() || failed {
                        break;
                    }
                }
            })?;
        Ok(Self::Threaded {
            outbound,
            inbound: std::sync::Mutex::new(inbound),
        })
    }
    fn send(&mut self, bytes: Vec<u8>, deadline: Instant) -> io::Result<()> {
        match self {
            #[cfg(target_os = "linux")]
            Self::Pipe(p) => p.send(&bytes, deadline),
            Self::Threaded { outbound, .. } => outbound.send((bytes, deadline)).map_err(|_| {
                io::Error::new(io::ErrorKind::BrokenPipe, "native worker channel closed")
            }),
        }
    }
    fn receive(&mut self, deadline: Instant) -> io::Result<Vec<u8>> {
        match self {
            #[cfg(target_os = "linux")]
            Self::Pipe(p) => p.receive(deadline),
            Self::Threaded { inbound, .. } => inbound
                .get_mut()
                .expect("exclusive worker receiver")
                .recv_timeout(deadline.saturating_duration_since(Instant::now()))
                .map_err(|e| {
                    io::Error::from(match e {
                        mpsc::RecvTimeoutError::Timeout => io::ErrorKind::TimedOut,
                        mpsc::RecvTimeoutError::Disconnected => io::ErrorKind::BrokenPipe,
                    })
                })?,
        }
    }
}

pub(crate) struct Peer {
    child: Child,
    channel: Channel,
    token: [u8; 16],
    seq: u64,
    pub steps: u64,
    pub withhold: bool,
    awaiting_receipt: bool,
}
impl Peer {
    pub fn new(
        exe: &Path,
        slot: u8,
        path: &Path,
        spec: &MovingSpec,
        transport: WorkerTransport,
    ) -> Result<Self> {
        Self::using(
            exe,
            slot,
            path,
            spec,
            "--managed-moving-v1",
            MAGIC,
            transport,
        )
    }
    pub(crate) fn new_profile(
        exe: &Path,
        slot: u8,
        path: &Path,
        spec: &impl Serialize,
        flag: &str,
        magic: [u8; 8],
    ) -> Result<Self> {
        Self::using(exe, slot, path, spec, flag, magic, WorkerTransport::Pipe)
    }
    pub(crate) fn using(
        exe: &Path,
        slot: u8,
        path: &Path,
        spec: &impl Serialize,
        flag: &str,
        magic: [u8; 8],
        transport: WorkerTransport,
    ) -> Result<Self> {
        let child = Self::command(exe, slot, path, flag).spawn()?;
        Self::connect(child, slot, spec, magic, transport)
    }
    pub(crate) fn owned_profile(
        exe: &Path,
        slot: u8,
        path: &Path,
        spec: &impl Serialize,
        flag: &str,
        magic: [u8; 8],
        transport: WorkerTransport,
    ) -> Result<Self> {
        let child = lifetime::spawn(Self::command(exe, slot, path, flag))?;
        Self::connect(child, slot, spec, magic, transport)
    }
    fn command(exe: &Path, slot: u8, path: &Path, flag: &str) -> Command {
        let mut c = Command::new(exe);
        c.arg(flag)
            .arg(slot.to_string())
            .arg(path)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit());
        c
    }
    fn connect(
        mut child: Child,
        slot: u8,
        spec: &impl Serialize,
        magic: [u8; 8],
        transport: WorkerTransport,
    ) -> Result<Self> {
        let token = *uuid::Uuid::new_v4().as_bytes();
        let channel = match Channel::new(&mut child, transport, token) {
            Ok(channel) => channel,
            Err(e) => {
                let _ = child.kill();
                let _ = child.wait();
                return Err(e.into());
            }
        };
        let mut me = Self {
            child,
            channel,
            token,
            seq: 0,
            steps: 0,
            withhold: false,
            awaiting_receipt: false,
        };
        let bytes = encode(&Bootstrap {
            magic,
            slot,
            token,
            spec_json: serde_json::to_vec(spec).map_err(io::Error::from)?,
        })?;
        let want = Sha256::digest(&bytes);
        if me.exchange(bytes)?[..] != want[..] {
            return Err(invalid("moving worker offer or lease mismatch"));
        }
        Ok(me)
    }
    pub fn pid(&self) -> u32 {
        self.child.id()
    }
    fn exchange(&mut self, bytes: Vec<u8>) -> Result<Vec<u8>> {
        let deadline = Instant::now() + TIMEOUT;
        self.channel.send(bytes, deadline)?;
        Ok(self.channel.receive(deadline)?)
    }
    pub(super) fn call(&mut self, tick: u64, cmd: Op) -> Result<Answer> {
        self.call_typed(tick, cmd)
    }
    pub(crate) fn call_typed<C: Serialize, A: DeserializeOwned>(
        &mut self,
        tick: u64,
        cmd: C,
    ) -> Result<A> {
        let pending = self.submit(tick, cmd)?;
        self.collect(pending)
    }
    pub(crate) fn submit<C: Serialize>(&mut self, tick: u64, cmd: C) -> Result<Pending> {
        self.submit_before(tick, cmd, Instant::now() + TIMEOUT)
    }
    pub(crate) fn call_before<C: Serialize, A: DeserializeOwned>(
        &mut self,
        tick: u64,
        cmd: C,
        deadline: Option<Instant>,
    ) -> Result<A> {
        let pending = self.submit_before(
            tick,
            cmd,
            deadline.unwrap_or_else(|| Instant::now() + TIMEOUT),
        )?;
        self.collect(pending)
    }
    fn submit_before<C: Serialize>(
        &mut self,
        tick: u64,
        cmd: C,
        deadline: Instant,
    ) -> Result<Pending> {
        if Instant::now() >= deadline {
            return Err(io::Error::from(io::ErrorKind::TimedOut).into());
        }
        if self.awaiting_receipt {
            return Err(io::Error::other(
                "uncollected native worker receipt; reinstall this worker",
            )
            .into());
        }
        let seq = self
            .seq
            .checked_add(1)
            .ok_or(invalid("worker sequence exhausted"))?;
        let bytes = crate::multi_sphere::timed!(
            request_encode_ns,
            encode(&Request {
                token: self.token,
                seq,
                tick,
                cmd,
            })
        )?;
        let digest: [u8; 32] =
            crate::multi_sphere::timed!(request_encode_ns, Sha256::digest(&bytes)).into();
        let replay = std::mem::take(&mut self.withhold).then(|| bytes.clone());
        self.awaiting_receipt = true;
        self.channel.send(bytes, deadline)?;
        Ok(Pending {
            seq,
            digest,
            deadline,
            replay,
        })
    }

    pub(crate) fn collect<A: DeserializeOwned>(&mut self, pending: Pending) -> Result<A> {
        let Pending {
            seq,
            digest,
            deadline,
            replay,
        } = pending;
        let first = self.channel.receive(deadline)?;
        let reply = if let Some(bytes) = replay {
            self.exchange(bytes)?
        } else {
            first
        };
        let r: Receipt<A> =
            crate::multi_sphere::timed!(receipt_decode_ns, codec().deserialize(&reply))
                .map_err(io::Error::other)?;
        if r.digest != digest {
            return Err(invalid("moving worker receipt mismatch"));
        }
        self.awaiting_receipt = false;
        self.seq = seq;
        self.steps = r.steps;
        r.answer.map_err(|e| {
            ArticulationError::Native(NativeError::Physics(gz_physics::PhysicsError::Native {
                engine: "moving-worker",
                message: e,
            }))
        })
    }
}
impl Drop for Peer {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}
