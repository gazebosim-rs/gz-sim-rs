// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Managed, versioned Zenoh lane for one complete floating revolute articulation.
//! A private slot lease outlives its session and prevents two live copies of a worker slot.
use super::*;
use crate::articulation::{ArticulationObservation, ArticulationSpec, MAX_ARTICULATION_IMAGE};
use bincode::Options;
use serde::de::DeserializeOwned;
use sha2::{Digest, Sha256};
use std::fs::{File, OpenOptions};

mod participant;
use participant::Participant;

const OFFER: &[u8] = b"GZARTICULATION/FLOATING-REVOLUTE/1\n";
const MAX_HINGE_BYTES: usize = 16 * 1024;
const WITHHOLD_REPLY: &str = "gz_hinge_withhold_reply";

/// Grounded native worlds include both solver contact caches and the joint graph.
/// Their bounded binary framing avoids converting each byte to decimal on every tick.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Profile {
    Floating,
    Grounded,
}

impl Profile {
    fn for_spec(spec: &ArticulationSpec) -> Self {
        if spec.ground.is_some() {
            Self::Grounded
        } else {
            Self::Floating
        }
    }
    fn flag(self) -> &'static str {
        match self {
            Self::Floating => "--managed-hinge-v1",
            Self::Grounded => "--managed-ground-hinge-v1",
        }
    }
    fn offer(self) -> &'static [u8] {
        match self {
            Self::Floating => OFFER,
            Self::Grounded => b"GZARTICULATION/GROUNDED-REVOLUTE/1\n",
        }
    }
    fn limit(self) -> usize {
        match self {
            Self::Floating => MAX_HINGE_BYTES,
            Self::Grounded => MAX_ARTICULATION_IMAGE + MAX_HINGE_BYTES,
        }
    }
    fn encode<T: Serialize>(self, value: &T) -> Result<Vec<u8>, TransportError> {
        match self {
            Self::Floating => encode_limited(value, self.limit()),
            Self::Grounded => bincode::options()
                .with_fixint_encoding()
                .with_limit(self.limit() as u64)
                .serialize(value)
                .map_err(wire),
        }
    }
    fn decode<T: DeserializeOwned>(self, data: &[u8]) -> Result<T, TransportError> {
        if data.len() > self.limit() {
            return Err(wire("oversized hinge operation"));
        }
        match self {
            Self::Floating => serde_json::from_slice(data).map_err(wire),
            Self::Grounded => bincode::options()
                .with_fixint_encoding()
                .with_limit(self.limit() as u64)
                .reject_trailing_bytes()
                .deserialize(data)
                .map_err(wire),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct HingeRequest {
    pub run: u64,
    pub worker: u8,
    pub sequence: u64,
    pub generation: u64,
    pub tick: u64,
    pub command: HingeCommand,
}

impl HingeRequest {
    fn digest(&self) -> Result<[u8; 32], TransportError> {
        Ok(Sha256::digest(bincode::serialize(self).map_err(wire)?).into())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) enum HingeCommand {
    Stage { image: Vec<u8> },
    Activate { digest: [u8; 32] },
    Fence { digest: [u8; 32] },
    QueueEffort { joint_id: u64, effort: f64 },
    Advance,
    Observe,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct HingeReply {
    request_digest: [u8; 32],
    pub run: u64,
    pub worker: u8,
    pub sequence: u64,
    pub generation: u64,
    pub tick: u64,
    pub state: Option<ArticulationObservation>,
    pub image: Vec<u8>,
}

impl HingeReply {
    fn matches(&self, request: &HingeRequest, digest: [u8; 32], expected_tick: u64) -> bool {
        if self.request_digest != digest
            || self.run != request.run
            || self.worker != request.worker
            || self.sequence != request.sequence
            || self.generation != request.generation
            || self.tick != expected_tick
        {
            return false;
        }
        match request.command {
            HingeCommand::Fence { .. } => self.state.is_none() && self.image.is_empty(),
            _ => {
                self.state.is_some_and(|s| s.tick == self.tick)
                    && (1..=MAX_ARTICULATION_IMAGE).contains(&self.image.len())
            }
        }
    }
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Seed {
    spec: ArticulationSpec,
    slot_lock: PathBuf,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
enum Operation<T> {
    Ready,
    Command(T),
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
enum Response {
    Ready {
        identity: Ready,
        spec_digest: [u8; 32],
    },
    Answer(Result<Box<HingeReply>, String>),
}

fn scope(profile: Profile, run: u64, worker: u8) -> String {
    let lane = match profile {
        Profile::Floating => "hinge",
        Profile::Grounded => "ground-hinge",
    };
    format!("gz/physics/distributed/{lane}/v1/{run}/worker/{worker}")
}

fn spec_digest(spec: &ArticulationSpec) -> Result<[u8; 32], TransportError> {
    Ok(Sha256::digest(bincode::serialize(spec).map_err(wire)?).into())
}

fn lease(path: &Path) -> io::Result<File> {
    if !path.is_absolute() {
        return Err(io_error(
            "managed hinge slot lease requires an absolute path",
        ));
    }
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(path)?;
    if !file.metadata()?.is_file() {
        return Err(io_error("managed hinge slot lease must be a regular file"));
    }
    // Parent times out and reaps this candidate if an old participant remains
    // stopped, with its lease still held. Closing our owner's pipe also fences us.
    file.lock()?;
    Ok(file)
}

pub fn serve_managed_hinge(run: u64, worker: u8) -> Result<(), TransportError> {
    serve_profile(run, worker, Profile::Floating)
}
pub fn serve_managed_ground_hinge(run: u64, worker: u8) -> Result<(), TransportError> {
    serve_profile(run, worker, Profile::Grounded)
}
fn serve_profile(run: u64, worker: u8, profile: Profile) -> Result<(), TransportError> {
    if worker >= 8 {
        return Err(wire("invalid articulation worker ID"));
    }
    let mut output = io::stdout().lock();
    output.write_all(profile.offer())?;
    output.flush()?;
    let seed: Seed = serde_json::from_slice(&read_owned_seed(MAX_HINGE_BYTES)?).map_err(wire)?;
    if profile != Profile::for_spec(&seed.spec) {
        return Err(wire("hinge seed disagrees with negotiated profile"));
    }
    seed.spec.validate().map_err(wire)?;
    let _slot = lease(&seed.slot_lock)?;
    let digest = spec_digest(&seed.spec)?;
    let mut participant = Participant::new(run, worker, seed.spec);
    let session = zenoh::open(config(None)?).wait().map_err(unavailable)?;
    let scope = scope(profile, run, worker);
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
        .find(|s| endpoint_valid(s))
        .ok_or_else(|| wire("missing loopback hinge endpoint"))?;
    // The separate offer and scoped queryable identify the lane. The primitive
    // profile field in the shared process descriptor is unused on this endpoint.
    let ready = Ready {
        profile: NativeSolverProfile::Standard32,
        run_id: run,
        worker_id: worker,
        endpoint,
        session_id: session.zid().to_string(),
        capability: uuid::Uuid::new_v4().into_bytes(),
    };
    serde_json::to_writer(&mut output, &ready).map_err(wire)?;
    writeln!(output)?;
    output.flush()?;
    drop(output);
    loop {
        let query = queries.recv().map_err(unavailable)?;
        if !query
            .attachment()
            .is_some_and(|a| a.len() == 16 && a.to_bytes().as_ref() == ready.capability)
        {
            let _ = query.reply_err("unknown worker capability").wait();
            continue;
        }
        let op = query
            .payload()
            .filter(|p| p.len() <= profile.limit())
            .and_then(|p| {
                profile
                    .decode::<Operation<HingeRequest>>(&p.to_bytes())
                    .ok()
            });
        let response = match op {
            Some(Operation::Ready) => Response::Ready {
                identity: ready.clone(),
                spec_digest: digest,
            },
            Some(Operation::Command(request)) => {
                let answer = participant.apply(request);
                if query.parameters().get(WITHHOLD_REPLY) == Some("1") {
                    // The fault applies after the worker committed its private
                    // transition. A retry must reconstruct the original receipt.
                    continue;
                }
                Response::Answer(answer.map(Box::new).map_err(|e| e.to_string()))
            }
            None => {
                let _ = query.reply_err("invalid hinge operation").wait();
                continue;
            }
        };
        // The bootstrap descriptor has a historical JSON-only profile omission.
        let encoded = match response {
            Response::Ready { .. } => encode_limited(&response, MAX_HINGE_BYTES),
            Response::Answer(_) => profile.encode(&response),
        };
        match encoded {
            Ok(data) => {
                let _ = query.reply(scope.as_str(), data).express(true).wait();
            }
            Err(_) => {
                let _ = query.reply_err("hinge receipt exceeds limit").wait();
            }
        }
    }
}

/// A child launched specifically for this articulation recipe, with a private
/// endpoint and a different capability on each replacement.
pub(crate) struct HingePeer {
    child: ChildGuard,
    session: zenoh::Session,
    ready: Ready,
    profile: Profile,
    key: String,
    withhold_next: bool,
}

impl HingePeer {
    pub(crate) fn spawn(
        exe: &Path,
        run: u64,
        worker: u8,
        spec: &ArticulationSpec,
        slot_lock: &Path,
    ) -> Result<Self, TransportError> {
        if worker >= 8 || !slot_lock.is_absolute() {
            return Err(wire("invalid hinge worker slot or lease path"));
        }
        spec.validate().map_err(wire)?;
        let profile = Profile::for_spec(spec);
        let seed = encode_limited(
            &Seed {
                spec: *spec,
                slot_lock: slot_lock.to_path_buf(),
            },
            MAX_HINGE_BYTES,
        )?;
        let mut child = ChildGuard(
            Command::new(exe)
                .args([
                    "--run",
                    &run.to_string(),
                    "--worker",
                    &worker.to_string(),
                    profile.flag(),
                ])
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .stderr(Stdio::inherit())
                .spawn()?,
        );
        let input = child.0.stdin.take().expect("piped worker stdin");
        let output = child.0.stdout.take().expect("piped worker stdout");
        let (ready, owner_pipe) = bootstrap_offer(
            &mut child,
            input,
            output,
            seed,
            (run, worker),
            (Some(profile.offer()), NativeSolverProfile::Standard32),
            START_TIMEOUT,
        )?;
        child.0.stdin = Some(owner_pipe);
        let session = zenoh::open(config(Some(&ready.endpoint))?)
            .wait()
            .map_err(unavailable)?;
        let peer = Self {
            child,
            ready,
            session,
            key: scope(profile, run, worker),
            profile,
            withhold_next: false,
        };
        let digest = spec_digest(spec)?;
        let until = Instant::now() + START_TIMEOUT;
        loop {
            match peer.exchange(
                Operation::<&HingeRequest>::Ready,
                Duration::from_millis(200),
                false,
            ) {
                Ok(Response::Ready {
                    identity,
                    spec_digest,
                }) if identity == peer.ready && spec_digest == digest => return Ok(peer),
                Err(TransportError::Unavailable(_)) if Instant::now() < until => {
                    thread::sleep(Duration::from_millis(10))
                }
                Err(e) => return Err(e),
                Ok(_) => {
                    return Err(wire(
                        "worker advertised a different hinge identity or recipe",
                    ));
                }
            }
        }
    }

    pub(crate) fn pid(&self) -> u32 {
        self.child.0.id()
    }
    pub(crate) fn kill(&mut self) -> io::Result<()> {
        self.child.kill()
    }

    pub(crate) fn withhold_next_reply(&mut self) {
        self.withhold_next = true;
    }

    pub(crate) fn call(&mut self, request: &HingeRequest) -> Result<HingeReply, TransportError> {
        if request.run != self.ready.run_id || request.worker != self.ready.worker_id {
            return Err(wire("hinge request address mismatch"));
        }
        let expected_tick = if matches!(request.command, HingeCommand::Advance) {
            request
                .tick
                .checked_add(1)
                .ok_or_else(|| wire("hinge tick overflow"))?
        } else {
            request.tick
        };
        let digest = request.digest()?;
        let withhold = std::mem::take(&mut self.withhold_next);
        let response = match self.exchange(Operation::Command(request), REQUEST_TIMEOUT, withhold) {
            Err(TransportError::Unavailable(_)) if self.child.0.try_wait()?.is_none() => {
                self.exchange(Operation::Command(request), REQUEST_TIMEOUT, false)?
            }
            result => result?,
        };
        match response {
            Response::Answer(Ok(r)) if r.matches(request, digest, expected_tick) => Ok(*r),
            Response::Answer(Err(e)) => Err(wire(e)),
            _ => Err(wire("hinge receipt identity mismatch")),
        }
    }

    fn exchange(
        &self,
        op: Operation<&HingeRequest>,
        timeout: Duration,
        withhold: bool,
    ) -> Result<Response, TransportError> {
        let ready = matches!(op, Operation::Ready);
        let payload = self.profile.encode(&op)?;
        let until = Instant::now() + timeout;
        let key = zenoh::key_expr::KeyExpr::try_from(self.key.as_str()).map_err(wire)?;
        let selector = zenoh::query::Selector::owned(
            key,
            if withhold {
                "gz_hinge_withhold_reply=1"
            } else {
                ""
            },
        );
        let replies = self
            .session
            .get(selector)
            .consolidation(zenoh::query::ConsolidationMode::None)
            .express(true)
            .attachment(self.ready.capability.to_vec())
            .payload(payload)
            .timeout(timeout)
            .with(FifoChannel::new(1))
            .wait()
            .map_err(unavailable)?;
        let reply = replies
            .recv_deadline(until)
            .map_err(unavailable)?
            .ok_or_else(|| unavailable("hinge reply deadline expired"))?;
        let s = reply.result().map_err(unavailable)?;
        if s.key_expr().as_str() != self.key || s.payload().len() > self.profile.limit() {
            return Err(wire("unexpected hinge reply key or size"));
        }
        let bytes = s.payload().to_bytes();
        if ready {
            Profile::Floating.decode(&bytes)
        } else {
            self.profile.decode(&bytes)
        }
    }
}

#[cfg(test)]
mod tests;
