// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One complete curved contact graph may move between local process leases at a public barrier.
mod deadline;
pub mod material;
pub mod partitions;
mod worker;
const MATERIAL_FLAG: &str = "--managed-trajectory-material-v1";
const MATERIAL_MAGIC: [u8; 8] = *b"GZTCMW01";
pub use crate::moving::WorkerTransport;
use crate::{articulation::ArticulationError, moving::worker::Peer};
use gz_physics_multifidelity::MultiFidelityBackend;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    io,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};
pub use worker::serve_material;
pub use worker::{arm_launch_owner, serve};
type Result<T> = std::result::Result<T, ArticulationError>;
const MAGIC: [u8; 8] = *b"GZTCWK01";
const FLAG: &str = "--managed-trajectory-v1";
const MAX: usize = 4 * 1024 * 1024;
/// Maximum serialized operator metadata for a self-contained native/material world.
pub const SCENE_MANIFEST_LIMIT: usize = 2 * 1024 * 1024;
const PAGE: usize = 96 * 1024;
type Hash = [u8; 32];
fn hash(b: &[u8]) -> Hash {
    Sha256::digest(b).into()
}
fn invalid(reason: &'static str) -> ArticulationError {
    ArticulationError::Invalid(reason)
}

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq)]
enum Blob {
    Recipe,
    State,
}
#[derive(Serialize, Deserialize)]
enum Operation {
    Start(Blob, u32, Hash),
    Write(u32, #[serde(with = "serde_bytes")] Vec<u8>),
    Finish,
    Step {
        source: Hash,
        dt_ns: u64,
    },
    Read(u32),
    ProfiledStep {
        source: Hash,
        dt_ns: u64,
    },
    SetMaterialThreadLimit(u8),
    PackedStep {
        source: Hash,
        dt_ns: u64,
        profile: bool,
    },
}
#[derive(Serialize, Deserialize)]
enum Reply {
    Ok,
    NeedState,
    Success {
        bytes: u32,
        digest: Hash,
        #[serde(with = "serde_bytes")]
        first: Vec<u8>,
    },
    Page(#[serde(with = "serde_bytes")] Vec<u8>),
    ProfiledSuccess {
        bytes: u32,
        digest: Hash,
        #[serde(with = "serde_bytes")]
        first: Vec<u8>,
        solve_ns: u64,
        snapshot_ns: u64,
    },
    BatchProfile {
        bytes: u32,
        digest: Hash,
        #[serde(with = "serde_bytes")]
        first: Vec<u8>,
        solve_ns: u64,
        snapshot_ns: u64,
        threads: u8,
    },
    Packed {
        block: crate::paged_native::PackedGraph,
        stages: Option<(u64, u64, u8)>,
    },
    PackedFallbackProfile {
        bytes: u32,
        digest: Hash,
        #[serde(with = "serde_bytes")]
        first: Vec<u8>,
        stages: (u64, u64, u8),
        attempted: bool,
    },
}
/// Elapsed time for one public interval. Child timings are nested within exchange.
#[derive(Debug, Clone, Copy, Default)]
pub struct TrajectoryProfile {
    pub capture: Duration,
    pub exchange: Duration,
    pub install: Duration,
    pub child_solve: Duration,
    pub child_snapshot: Duration,
    pub material_threads: usize,
    /// Number of worker groups which actually attempted to compress this successor.
    pub packing_attempts: usize,
}
/// Evidence from one completely validated publication.
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryWorkerStep {
    pub slot: usize,
    pub pid: u32,
    pub restarted: bool,
    /// Warm deadline selected for the first exchange, if a warmed-up budget was active.
    pub warm_deadline: Option<Duration>,
    /// The first exchange exhausted its shortened deadline. The complete cut used the ordinary retry.
    pub warm_timeout: bool,
    pub transferred_source: bool,
    pub source_bytes: usize,
    pub successor_bytes: usize,
    /// Successor payload bytes, excluding RPC framing and cold source uploads.
    pub successor_payload_bytes: usize,
    pub wall: Duration,
    pub profile: Option<TrajectoryProfile>,
}
/// The simulator owns the committed graph. A worker solves only from that exact source. A failed
/// lease, dropped reply or owner change cannot publish a partially solved graph to the simulator.
pub struct TrajectoryWorkers {
    workers: Vec<Option<Peer>>,
    dir: tempfile::TempDir,
    executable: PathBuf,
    recipe: Vec<u8>,
    identity: Hash,
    selected: usize,
    transport: WorkerTransport,
    initial_transport: WorkerTransport,
    warm: deadline::WarmBudget,
    private_digests: Vec<Option<Hash>>,
    packed_replies: bool,
    material_surfaces: usize,
    material_identities: Vec<Hash>,
    profiling: bool,
    material_threads: u8,
    last: Option<TrajectoryWorkerStep>,
}
impl TrajectoryWorkers {
    pub fn new(executable: &Path, width: usize, backend: &MultiFidelityBackend) -> Result<Self> {
        Self::create(executable, width, backend, None, 1, WorkerTransport::Pipe)
    }
    pub fn with_material(
        executable: &Path,
        width: usize,
        backend: &MultiFidelityBackend,
        recipe: material::MaterialRecipe,
    ) -> Result<Self> {
        Self::with_material_threads(executable, width, backend, recipe, 1)
    }
    /// Limit total solve threads (including the calling thread) inside each material worker.
    pub fn with_material_threads(
        executable: &Path,
        width: usize,
        backend: &MultiFidelityBackend,
        recipe: material::MaterialRecipe,
        threads: u8,
    ) -> Result<Self> {
        Self::with_material_transport(
            executable,
            width,
            backend,
            recipe,
            threads,
            WorkerTransport::Pipe,
        )
    }
    /// Start a leased local native/XPBD child with a selected RPC transport.
    pub fn with_material_transport(
        executable: &Path,
        width: usize,
        backend: &MultiFidelityBackend,
        recipe: material::MaterialRecipe,
        threads: u8,
        transport: WorkerTransport,
    ) -> Result<Self> {
        if !(1..=4).contains(&threads) {
            return Err(invalid(
                "material worker needs a thread budget from one to four",
            ));
        }
        Self::create(executable, width, backend, Some(recipe), threads, transport)
    }
    fn create(
        executable: &Path,
        width: usize,
        backend: &MultiFidelityBackend,
        material: Option<material::MaterialRecipe>,
        material_threads: u8,
        transport: WorkerTransport,
    ) -> Result<Self> {
        if !(1..=8).contains(&width) {
            return Err(invalid("trajectory worker count must be one through eight"));
        }
        let recipe = backend.trajectory_construction()?;
        let material_surfaces = material
            .as_ref()
            .map_or(0, material::MaterialRecipe::surface_count);
        let material_identities = material
            .as_ref()
            .map(material::MaterialRecipe::identities)
            .transpose()?
            .unwrap_or_default();
        let recipe = if let Some(m) = material {
            material::recipe(recipe, m)?
        } else {
            recipe
        };
        let identity = hash(&recipe);
        let mut pool = Self {
            workers: (0..width).map(|_| None).collect(),
            dir: tempfile::Builder::new()
                .prefix("gz-curve-owner-")
                .tempdir()?,
            executable: executable.to_owned(),
            recipe,
            identity,
            transport,
            initial_transport: transport,
            warm: Default::default(),
            private_digests: vec![None; width],
            packed_replies: false,
            material_surfaces,
            material_identities,
            profiling: false,
            material_threads,
            selected: 0,
            last: None,
        };
        pool.start(0, 0)?;
        Ok(pool)
    }
    /// Set the minimum deadline for an already-resident successor request. A replacement always uses the full transport budget.
    pub fn set_warm_budget(&mut self, floor: Option<Duration>) -> Result<()> {
        self.warm.configure(floor)
    }
    /// Allow single-frame compressed replies when the bounded block reduces page requests.
    pub fn set_packed_replies(&mut self, enabled: bool) {
        self.packed_replies = enabled;
    }
    /// RPC transport for the next selected lease.
    pub fn transport(&self) -> WorkerTransport {
        self.transport
    }
    /// Retire every private slot before changing this group's transport. The caller's committed
    /// graph remains the source for the next request, including a pending owner change.
    pub fn set_transport(&mut self, transport: WorkerTransport) {
        if self.transport != transport {
            for peer in &mut self.workers {
                peer.take();
            }
            self.transport = transport;
            self.warm.clear();
            self.private_digests.fill(None);
        }
    }
    /// Include nested solver and snapshot timings in subsequent receipts.
    pub fn set_profiling(&mut self, enabled: bool) {
        self.profiling = enabled;
    }
    pub fn owner(&self) -> usize {
        self.selected
    }
    pub fn workers(&self) -> usize {
        self.workers.len()
    }
    pub fn pid(&self, slot: usize) -> Option<u32> {
        self.workers.get(slot)?.as_ref().map(Peer::pid)
    }
    pub fn last(&self) -> Option<TrajectoryWorkerStep> {
        self.last
    }
    pub fn select(&mut self, slot: usize) -> Result<()> {
        if slot >= self.workers.len() {
            return Err(invalid(
                "trajectory destination is outside the bounded pool",
            ));
        }
        self.selected = slot;
        Ok(())
    }
    /// Drop private speculative state and return to the initial layout for a loaded reset.
    pub fn reset(&mut self) {
        for peer in &mut self.workers {
            peer.take();
        }
        self.transport = self.initial_transport;
        self.warm.clear();
        self.private_digests.fill(None);
        self.selected = 0;
        self.last = None;
    }
    fn start(&mut self, slot: usize, tick: u64) -> Result<()> {
        let path = self.dir.path().join(format!("slot-{slot}.lock"));
        let (flag, magic) = if self.material_surfaces > 0 {
            (MATERIAL_FLAG, MATERIAL_MAGIC)
        } else {
            (FLAG, MAGIC)
        };
        let mut peer = Peer::owned_profile(
            &self.executable,
            slot as u8,
            &path,
            &self.identity,
            flag,
            magic,
            self.transport,
        )?;
        if self.material_threads > 1
            && !matches!(
                peer.call_typed(
                    tick,
                    Operation::SetMaterialThreadLimit(self.material_threads)
                )?,
                Reply::Ok
            )
        {
            return Err(invalid("worker rejected its material thread budget"));
        }
        send_blob(&mut peer, tick, Blob::Recipe, &self.recipe, self.identity)?;
        self.workers[slot] = Some(peer);
        Ok(())
    }
    /// A positive physical interval. The caller holds exclusive access to both the public backend
    /// and this resource through the request, validation and installation of its successor.
    pub fn advance(
        &mut self,
        backend: &mut MultiFidelityBackend,
        tick: u64,
        dt: Duration,
    ) -> Result<TrajectoryWorkerStep> {
        self.advance_with_material(backend, &mut [], tick, dt)
    }
    pub fn advance_with_material(
        &mut self,
        backend: &mut MultiFidelityBackend,
        soft: &mut [&mut gz_physics_xpbd::SoftBody],
        tick: u64,
        dt: Duration,
    ) -> Result<TrajectoryWorkerStep> {
        if self.material_identities.len() != soft.len() {
            return Err(invalid("worker and material population disagree"));
        }
        for (body, expected) in soft.iter().zip(&self.material_identities) {
            if body.one_way_identity()? != *expected {
                return Err(invalid("worker and public material laws differ"));
            }
        }
        let ns = u64::try_from(dt.as_nanos())
            .map_err(|_| invalid("curve interval overflows nanoseconds"))?;
        if ns == 0 {
            return Err(invalid("curve worker needs a positive interval"));
        }
        let start = Instant::now();
        let source = if self.material_surfaces > 0 {
            material::capture(backend, soft.iter().map(|p| &**p))?
        } else {
            backend.checkpoint_trajectory()?
        };
        let digest = hash(&source);
        let prepared = Instant::now();
        let slot = self.selected;
        let (received, restarted) = self.request(tick, ns, &source, digest)?;
        let received_at = Instant::now();
        if self.material_surfaces > 0 {
            material::install(
                backend,
                soft,
                &received.bytes,
                self.material_surfaces,
                false,
            )?;
        } else {
            backend.install_trajectory(&received.bytes)?;
        }
        let installed = Instant::now();
        let profile = received
            .stages
            .map(|(solve, snapshot, threads)| TrajectoryProfile {
                capture: prepared - start,
                exchange: received_at - prepared,
                install: installed - received_at,
                material_threads: usize::from(threads),
                packing_attempts: usize::from(received.packing_attempted),
                child_solve: Duration::from_nanos(solve),
                child_snapshot: Duration::from_nanos(snapshot),
            });
        let result = TrajectoryWorkerStep {
            slot,
            pid: self.workers[slot].as_ref().unwrap().pid(),
            restarted,
            warm_deadline: received.budget,
            warm_timeout: received.timed_out,
            transferred_source: received.sent_source,
            source_bytes: if received.sent_source {
                source.len()
            } else {
                0
            },
            successor_bytes: received.bytes.len(),
            successor_payload_bytes: received.payload_bytes,
            wall: installed - start,
            profile,
        };
        self.last = Some(result);
        Ok(result)
    }
    fn request(
        &mut self,
        tick: u64,
        ns: u64,
        source: &[u8],
        digest: Hash,
    ) -> Result<(Received, bool)> {
        let slot = self.selected;
        let resident = self.workers[slot].is_some() && self.private_digests[slot] == Some(digest);
        let budget = if resident { self.warm.limit() } else { None };
        let start = Instant::now();
        let attempt = |pool: &mut Self, deadline: Option<Instant>| -> Result<Received> {
            if pool.workers[slot].is_none() {
                pool.start(slot, tick)?;
            }
            exchange(
                pool.workers[slot].as_mut().unwrap(),
                tick,
                ns,
                source,
                digest,
                ReplyMode {
                    profile: pool.profiling,
                    packed: pool.packed_replies,
                },
                deadline,
            )
        };
        let (mut result, restarted, timed_out) = match attempt(self, budget.map(|b| start + b)) {
            Ok(v) => (v, false, false),
            Err(e @ (ArticulationError::Io(_) | ArticulationError::Transport(_))) => {
                let transport_timeout = matches!(e, ArticulationError::Io(ref error)
                    if error.kind() == io::ErrorKind::TimedOut);
                let timed_out =
                    budget.is_some_and(|limit| transport_timeout || start.elapsed() >= limit);
                self.workers[slot].take();
                self.private_digests[slot] = None;
                // Replacement is always an ordinary full-budget attempt from the public source.
                (attempt(self, None)?, true, timed_out)
            }
            Err(e) => return Err(e),
        };
        // Include the full successful recovery latency in the estimate following a timeout.
        if timed_out || (resident && !restarted && !result.sent_source) {
            self.warm.observe(start.elapsed());
        }
        self.private_digests[slot] = Some(result.digest);
        result.budget = budget;
        result.timed_out = timed_out;
        Ok((result, restarted))
    }
}

struct Received {
    digest: Hash,
    budget: Option<Duration>,
    timed_out: bool,
    payload_bytes: usize,
    packing_attempted: bool,
    bytes: Vec<u8>,
    sent_source: bool,
    stages: Option<(u64, u64, u8)>,
}
struct ReplyMode {
    profile: bool,
    packed: bool,
}
fn exchange(
    peer: &mut Peer,
    tick: u64,
    ns: u64,
    source: &[u8],
    source_digest: Hash,
    mode: ReplyMode,
    mut deadline: Option<Instant>,
) -> Result<Received> {
    let ReplyMode { profile, packed } = mode;
    let step = || {
        if packed {
            Operation::PackedStep {
                source: source_digest,
                dt_ns: ns,
                profile,
            }
        } else if profile {
            Operation::ProfiledStep {
                source: source_digest,
                dt_ns: ns,
            }
        } else {
            Operation::Step {
                source: source_digest,
                dt_ns: ns,
            }
        }
    };
    let reply = peer.call_before(tick, step(), deadline)?;
    let sent_source = matches!(reply, Reply::NeedState);
    let reply = if sent_source {
        deadline = None;
        send_blob(peer, tick, Blob::State, source, source_digest)?;
        peer.call_typed(tick, step())?
    } else {
        reply
    };
    if let Reply::Packed { block, stages } = reply {
        if !packed
            || stages.is_some() != profile
            || stages.is_some_and(|(_, _, n)| !(1..=4).contains(&n))
        {
            return Err(invalid("unexpected packed trajectory receipt"));
        }
        let (payload_bytes, digest) = block.payload();
        let bytes = block.open(MAX)?;
        if bytes.len() <= PAGE {
            return Err(invalid("packed trajectory must replace multiple raw pages"));
        }
        return Ok(Received {
            digest,
            budget: None,
            timed_out: false,
            payload_bytes,
            packing_attempted: true,
            bytes,
            sent_source,
            stages,
        });
    }
    let mut packing_attempted = false;
    let (size, digest, mut bytes, stages) = match reply {
        Reply::PackedFallbackProfile {
            bytes,
            digest,
            first,
            stages,
            attempted,
        } if packed && profile && (1..=4).contains(&stages.2) => {
            packing_attempted = attempted;
            (bytes as usize, digest, first, Some(stages))
        }
        Reply::Success {
            bytes,
            digest,
            first,
        } if !profile => (bytes as usize, digest, first, None),
        Reply::ProfiledSuccess {
            bytes,
            digest,
            first,
            solve_ns,
            snapshot_ns,
        } if profile => (
            bytes as usize,
            digest,
            first,
            Some((solve_ns, snapshot_ns, 1)),
        ),
        Reply::BatchProfile {
            bytes,
            digest,
            first,
            solve_ns,
            snapshot_ns,
            threads,
        } if profile && (1..=4).contains(&threads) => (
            bytes as usize,
            digest,
            first,
            Some((solve_ns, snapshot_ns, threads)),
        ),
        _ => {
            return Err(invalid(
                "trajectory worker failed to solve its assigned source",
            ));
        }
    };
    if !(1..=MAX).contains(&size) || bytes.len() != size.min(PAGE) {
        return Err(invalid("invalid curve successor length"));
    }
    bytes.reserve_exact(size - bytes.len());
    while bytes.len() < size {
        let Reply::Page(page) =
            peer.call_before(tick, Operation::Read(bytes.len() as u32), deadline)?
        else {
            return Err(invalid("incomplete curve successor"));
        };
        if page.len() != PAGE.min(size - bytes.len()) {
            return Err(invalid("invalid curve successor page"));
        }
        bytes.extend_from_slice(&page);
    }
    if hash(&bytes) != digest {
        return Err(invalid("curve worker returned a damaged successor"));
    }
    Ok(Received {
        digest,
        budget: None,
        timed_out: false,
        payload_bytes: bytes.len(),
        packing_attempted,
        bytes,
        sent_source,
        stages,
    })
}
fn send_blob(peer: &mut Peer, tick: u64, kind: Blob, bytes: &[u8], digest: Hash) -> Result<()> {
    let mut cmd = |op| -> Result<()> {
        if !matches!(peer.call_typed(tick, op)?, Reply::Ok) {
            return Err(invalid("trajectory worker refused an image transfer"));
        }
        Ok(())
    };
    cmd(Operation::Start(kind, bytes.len() as u32, digest))?;
    for (i, page) in bytes.chunks(PAGE).enumerate() {
        cmd(Operation::Write((i * PAGE) as u32, page.to_vec()))?;
    }
    cmd(Operation::Finish)
}

/// Write a self-contained public clock/scene manifest together with the warm native graph as one
/// checksummed, immutable file. Existing generations retain their previous names and contents.
pub fn save_scene_cut(
    path: &Path,
    manifest: &[u8],
    state: &[u8],
) -> std::result::Result<(), crate::coordinator::RecoveryError> {
    use crate::coordinator::{RecoveryError, write_immutable};
    if manifest.is_empty()
        || manifest.len() > SCENE_MANIFEST_LIMIT
        || state.is_empty()
        || state.len() > MAX
    {
        return Err(RecoveryError::Format(
            "curve scene cut exceeds its manifest or native limit",
        ));
    }
    write_immutable(path, b"GZTCSD01", |w| {
        w.write_all(&(manifest.len() as u32).to_le_bytes())?;
        w.write_all(manifest)?;
        w.write_all(state)?;
        Ok(())
    })
}
pub fn read_scene_cut(
    path: &Path,
) -> std::result::Result<(Vec<u8>, Vec<u8>), crate::coordinator::RecoveryError> {
    use crate::coordinator::{RecoveryError, read_verified};
    if std::fs::metadata(path)?.len() > 44 + (SCENE_MANIFEST_LIMIT + MAX) as u64 {
        return Err(RecoveryError::Format("oversized curve scene cut"));
    }
    let (_, mut bytes) = read_verified(path, &[b"GZTCSD01"])?;
    let n = u32::from_le_bytes(
        bytes
            .get(..4)
            .ok_or(RecoveryError::Format("truncated curve header"))?
            .try_into()
            .unwrap(),
    ) as usize;
    if n == 0 || n > SCENE_MANIFEST_LIMIT || bytes.len() <= n + 4 || bytes.len() - 4 - n > MAX {
        return Err(RecoveryError::Format("invalid curve cut lengths"));
    }
    let state = bytes.split_off(4 + n);
    Ok((bytes[4..].to_vec(), state))
}
