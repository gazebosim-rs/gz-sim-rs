// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two-worker barriers. Only complete observations are published to callers.
//! A worker can be one speculative local step ahead during an interrupted barrier.
//! Fenced replacement replays that worker's acknowledged commands, including contacts.
use crate::{
    physics::{Body, PendingImpulse, partition_can_step, partitions_may_interact},
    protocol::{Command, MAX_BODY_IDS, Observation, Outcome, Reply, Request, Ticket, WarmTicket},
    transport::{TransportError, WorkerProcess},
};
use std::{
    collections::{BTreeMap, BTreeSet},
    io,
};

mod balance;
pub use balance::{BalanceDecision, BalancePolicy, LoadSample, LoadWindow};
mod cold;
mod compact;
mod edits;
mod forced;
pub use compact::{CheckpointPeer, Compaction};
mod native_file;
pub use native_file::NativeRecoveryPlan;
mod overlap;
pub use overlap::OverlappingPeers;
mod insert;
pub(crate) use insert::validate_separated_birth;
mod island;
pub(crate) use island::project_component;
mod recovery;
mod replace;
mod routing;
mod supervise;
pub use recovery::{RecoveryError, RecoveryPlan};
pub(crate) use recovery::{read_verified, write_immutable};
pub use routing::BodyRoute;

/// Separate endpoints per worker. Restart must fence the old owner before a
/// fresh process with the same seed can answer requests.
pub trait Peer {
    /// Availability remains stable for the lifetime of a peer.
    fn checkpoint_peer(&mut self) -> Option<&mut dyn CheckpointPeer> {
        None
    }
    fn exchange(&mut self, request: &Request) -> Result<Reply, TransportError>;
    fn restart(&mut self) -> io::Result<()>;
}
impl Peer for WorkerProcess {
    fn checkpoint_peer(&mut self) -> Option<&mut dyn CheckpointPeer> {
        Some(self)
    }
    fn exchange(&mut self, request: &Request) -> Result<Reply, TransportError> {
        self.request(request)
    }
    fn restart(&mut self) -> io::Result<()> {
        WorkerProcess::restart(self)
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("worker exchange failed: {0}")]
    Transport(#[from] TransportError),
    #[error("worker replacement failed: {0}")]
    Restart(#[from] io::Error),
    #[error("inconsistent worker result: {0}")]
    Invalid(&'static str),
    #[error("finish the interrupted barrier before starting a different one")]
    Pending,
    #[error("worker {0} requires fenced replacement")]
    Offline(u8),
    #[error("worker {0} failed deterministic recovery at sequence {1}")]
    Diverged(u8, u64),
    #[error("body edit deferred: {0}")]
    EditDeferred(&'static str),
    #[error("worker capacity would be exceeded")]
    Capacity,
    #[error("swept box bounds require a common native solver before advancing this tick")]
    Colocation,
}

/// Each stable global body ID appears exactly once in a published frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub tick: u64,
    pub workers: [Observation; 2],
}
impl Frame {
    pub fn ground(&self) -> Option<crate::physics::GroundPlane> {
        self.workers[0].snapshot.ground
    }
    pub fn step(&self) -> crate::physics::SimulationStep {
        self.workers[0].snapshot.step
    }
    pub fn body(&self, id: u64) -> Option<(u8, &Body)> {
        self.workers.iter().enumerate().find_map(|(worker, o)| {
            o.snapshot
                .bodies
                .binary_search_by_key(&id, |body| body.id)
                .ok()
                .map(|index| (worker as u8, &o.snapshot.bodies[index]))
        })
    }
    fn new(tick: u64, workers: [Observation; 2]) -> Result<Self, Error> {
        if workers[0].snapshot.ground != workers[1].snapshot.ground {
            return Err(Error::Invalid("worker ground planes disagree"));
        }
        if workers[0].snapshot.step != workers[1].snapshot.step {
            return Err(Error::Invalid("worker solver intervals disagree"));
        }
        let mut ids = BTreeSet::new();
        for obs in &workers {
            if obs.snapshot.tick != tick || obs.generations.len() != obs.snapshot.bodies.len() {
                return Err(Error::Invalid("observation tick or generation count"));
            }
            if obs.snapshot.bodies.windows(2).any(|p| p[0].id > p[1].id) {
                return Err(Error::Invalid("unsorted body observations"));
            }
            for body in &obs.snapshot.bodies {
                if !ids.insert(body.id) || !obs.generations.contains_key(&body.id) {
                    return Err(Error::Invalid("duplicate or unowned global ID"));
                }
            }
        }
        for obs in &workers {
            validate_memberships(obs)?;
        }
        Ok(Self { tick, workers })
    }
    fn check_sweeps(&self, pending: &[PendingImpulse]) -> Result<(), Error> {
        let [a, b] = &self.workers;
        if self
            .workers
            .iter()
            .any(|w| !partition_can_step(&w.snapshot, &w.components, pending))
            || partitions_may_interact(
                &a.snapshot,
                &a.components,
                &b.snapshot,
                &b.components,
                pending,
            )
        {
            return Err(Error::Colocation);
        }
        Ok(())
    }

    // A positive generation means both peers have owned this ID. The worker
    // retains a tombstone after release, which also occupies its admission budget.
    fn can_receive(&self, destination: usize, incoming: &[u64]) -> bool {
        let sender = &self.workers[1 - destination].generations;
        let unseen = incoming.iter().filter(|id| sender[id] == 0).count();
        self.known_ids(destination) + unseen <= MAX_BODY_IDS
    }

    fn known_ids(&self, worker: usize) -> usize {
        self.workers[worker].generations.len()
            + self.workers[1 - worker]
                .generations
                .values()
                .filter(|&&g| g > 0)
                .count()
    }

    fn unchanged_owners(&self, next: &Self) -> bool {
        self.workers.iter().zip(&next.workers).all(|(a, b)| {
            a.snapshot.ground == b.snapshot.ground
                && a.generations == b.generations
                && a.snapshot.step == b.snapshot.step
        })
    }
}

// A worker exposes both its committed contacts and the membership of its
// native solvers. A native solver can retain history after two boxes separate.
pub(crate) fn validate_memberships(obs: &Observation) -> Result<(), Error> {
    validate_memberships_with_profile(obs, crate::physics::NativeSolverProfile::Standard32)
}
pub(crate) fn validate_memberships_with_profile(
    obs: &Observation,
    profile: crate::physics::NativeSolverProfile,
) -> Result<(), Error> {
    if obs.generations.len() > MAX_BODY_IDS {
        return Err(Error::Capacity);
    }
    let mut membership = BTreeMap::new();
    let mut last = None;
    for component in &obs.components {
        if component.ids.len() > profile.max_bodies() {
            return Err(Error::Capacity);
        }
        let Some(&first) = component.ids.first() else {
            return Err(Error::Invalid("empty solver component"));
        };
        if last.is_some_and(|n| n >= first) || component.ids.windows(2).any(|w| w[0] >= w[1]) {
            return Err(Error::Invalid("unsorted solver components"));
        }
        last = Some(first);
        for id in &component.ids {
            if !obs.generations.contains_key(id) || membership.insert(*id, first).is_some() {
                return Err(Error::Invalid("incorrect component membership"));
            }
        }
    }
    if (!obs.snapshot.supports.is_empty() && obs.snapshot.ground.is_none())
        || obs
            .snapshot
            .supports
            .windows(2)
            .any(|w| w[0].body >= w[1].body)
        || obs.snapshot.supports.iter().any(|p| {
            !p.normal_impulse.is_finite()
                || p.normal_impulse < 0.
                || !obs
                    .components
                    .iter()
                    .any(|c| c.contact_history && c.ids.binary_search(&p.body).is_ok())
        })
        || membership.len() != obs.generations.len()
        || obs.snapshot.contact_history != obs.components.iter().any(|c| c.contact_history)
        || obs.snapshot.contacts != obs.snapshot.impulses.len()
        || obs.snapshot.impulses.iter().any(|p| {
            p.a >= p.b
                || !p.normal_impulse.is_finite()
                || p.normal_impulse < 0.
                || !obs.components.iter().any(|c| {
                    c.contact_history
                        && c.ids.binary_search(&p.a).is_ok()
                        && c.ids.binary_search(&p.b).is_ok()
                })
        })
        || obs
            .snapshot
            .impulses
            .windows(2)
            .any(|w| (w[0].a, w[0].b) >= (w[1].a, w[1].b))
    {
        return Err(Error::Invalid(
            "inconsistent component contact observations",
        ));
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq)]
enum Action {
    Advance,
    Transfer(Ticket),
    Queue(PendingImpulse),
    Velocity(crate::protocol::VelocityEdit),
    Forced(Vec<crate::physics::BodyWrench>),
    Retire(u64),
    Insert(Body),
    Replace(Body),
    ImpulseNow(crate::protocol::SpatialImpulse),
    TransferIsland(WarmTicket),
    TransferSolver(WarmTicket),
    Split(u64),
    Field(crate::physics::StepField),
    Refined(crate::physics::StepField),
    Islands(crate::physics::StepField, Vec<u64>),
    SceneWithSeparation(crate::physics::StepField, bool),
}

#[derive(Clone, Debug)]
struct Pending {
    action: Action,
    start: [usize; 2],
}

type Journal = Vec<(Request, Reply)>;

pub struct Coordinator<P> {
    run: u64,
    peers: [P; 2],
    journals: [Journal; 2],
    baseline_sequences: [u64; 2],
    offline: [bool; 2],
    limit: usize,
    rolling_journals: bool,
    serial: u64,
    frame: Frame,
    last_refined: bool,
    last_island_ids: Option<Vec<u64>>,
    issued: insert::IssuedIds,
    pending: Option<Pending>,
    aborting: bool,
    // Accepted inputs across both workers, in submission order. A failed
    // barrier keeps this ledger at its last published value.
    queued: Vec<PendingImpulse>,
    recording: Option<recovery::Recorder>,
}

fn observation(result: Outcome) -> Result<Observation, Error> {
    match result {
        Outcome::State(o) => Ok(o),
        _ => Err(Error::Invalid("expected worker observation")),
    }
}

fn exchange<P: Peer>(peer: &mut P, request: &Request) -> Result<Reply, Error> {
    let reply = peer.exchange(request)?;
    validate_envelope(request, &reply)?;
    Ok(reply)
}

fn validate_envelope(request: &Request, reply: &Reply) -> Result<(), Error> {
    if (reply.run_id, reply.worker_id, reply.sequence)
        != (request.run_id, request.worker_id, request.sequence)
    {
        return Err(Error::Invalid("reply envelope"));
    }
    Ok(())
}

impl<P: Peer> Coordinator<P> {
    /// Start with two freshly seeded endpoints. The caller selects the maximum
    /// number of acknowledged commands retained per worker for deterministic recovery.
    pub fn new(run: u64, peers: [P; 2], limit: usize) -> Result<Self, Error> {
        let c = Self::from_seeds(run, peers, limit)?;
        if c.frame
            .workers
            .iter()
            .map(|w| w.generations.len())
            .sum::<usize>()
            < 2
        {
            return Err(Error::Invalid(
                "the distributed world requires at least two boxes",
            ));
        }
        Ok(c)
    }

    pub(crate) fn from_seeds(run: u64, mut peers: [P; 2], limit: usize) -> Result<Self, Error> {
        if limit == 0 {
            return Err(Error::Capacity);
        }
        let mut journals: [Journal; 2] = Default::default();
        let mut observe = |worker: usize| -> Result<Observation, Error> {
            let req = Request {
                run_id: run,
                worker_id: worker as u8,
                sequence: 1,
                command: Command::Observe,
            };
            let reply = exchange(&mut peers[worker], &req)?;
            let obs = observation(reply.outcome.clone())?;
            journals[worker].push((req, reply));
            Ok(obs)
        };
        let frame = Frame::new(0, [observe(0)?, observe(1)?])?;
        if frame
            .workers
            .iter()
            .any(|w| w.generations.values().any(|&v| v != 0))
        {
            return Err(Error::Invalid("initial generation is nonzero"));
        }
        Ok(Self {
            run,
            peers,
            journals,
            baseline_sequences: [0; 2],
            offline: [false; 2],
            limit,
            rolling_journals: false,
            serial: 0,
            last_refined: false,
            last_island_ids: None,
            issued: insert::IssuedIds::from_frame(&frame),
            frame,
            pending: None,
            aborting: false,
            queued: Vec::new(),
            recording: None,
        })
    }
    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Maximum native solves on any island in the latest completed interval.
    pub fn last_substeps(&self) -> Option<u8> {
        (self.frame.tick > 0).then_some(if self.last_refined { 10 } else { 1 })
    }

    /// Fine body IDs in the last island-selective interval, in increasing order.
    /// A completed all-coarse island step returns an empty slice. Startup and
    /// the other stepping policies return None.
    pub fn last_island_refined_bodies(&self) -> Option<&[u64]> {
        self.last_island_ids.as_deref()
    }

    /// Acknowledged requests that a replacement replays from its most recent
    /// native baseline (or from its original seed before the first compaction).
    pub fn retained_entries(&self) -> [usize; 2] {
        self.journals.each_ref().map(Vec::len)
    }

    fn begin(&mut self, action: Action, counts: [usize; 2]) -> Result<[usize; 2], Error> {
        if self.aborting {
            return Err(Error::Pending);
        }
        if let Some(i) = self.offline.iter().position(|offline| *offline) {
            return Err(Error::Offline(i as u8));
        }
        if let Some(p) = &self.pending {
            return if p.action == action {
                Ok(p.start)
            } else {
                Err(Error::Pending)
            };
        }
        if self
            .recording
            .as_ref()
            .is_some_and(|a| a.len() >= recovery::MAX_ACTIONS)
        {
            return Err(Error::Capacity);
        }
        self.reserve_journal(counts)?;
        for (w, (log, n)) in self.journals.iter().zip(counts).enumerate() {
            if self.baseline_sequences[w]
                .checked_add(log.len() as u64)
                .and_then(|v| v.checked_add(n as u64))
                .is_none()
            {
                return Err(Error::Capacity);
            }
        }
        let start = self.journals.each_ref().map(Vec::len);
        self.pending = Some(Pending { action, start });
        Ok(start)
    }
    /// Re-entering a barrier reuses its acknowledged prefix. An ambiguous
    /// delivery has no journal entry, so its exact same sequence is retried.
    fn call(
        &mut self,
        cursors: &mut [usize; 2],
        worker: usize,
        command: Command,
    ) -> Result<Outcome, Error> {
        let index = cursors[worker];
        let req = Request {
            run_id: self.run,
            worker_id: worker as u8,
            sequence: self.baseline_sequences[worker] + index as u64 + 1,
            command,
        };
        let log = &mut self.journals[worker];
        if index == log.len() {
            let reply = exchange(&mut self.peers[worker], &req)?;
            log.push((req, reply));
        } else if log.get(index).is_none_or(|(old, _)| old != &req) {
            return Err(Error::Invalid("re-entered barrier differs from journal"));
        }
        cursors[worker] += 1;
        Ok(log[index].1.outcome.clone())
    }

    pub fn advance(&mut self) -> Result<&Frame, Error> {
        let (end, cursor) = self.begin_step()?;
        self.complete_step(end, cursor, self.step_commands())
    }

    fn step_commands(&self) -> [Command; 2] {
        std::array::from_fn(|_| Command::Step {
            tick: self.frame.tick,
        })
    }

    fn begin_step(&mut self) -> Result<(u64, [usize; 2]), Error> {
        if matches!(&self.pending, Some(p) if p.action != Action::Advance) {
            return Err(Error::Pending);
        }
        self.frame.check_sweeps(&self.queued)?;
        let tick = self.frame.tick;
        let end = tick.checked_add(1).ok_or(Error::Capacity)?;
        let cursor = self.begin(Action::Advance, [1, 1])?;
        Ok((end, cursor))
    }

    fn complete_step(
        &mut self,
        end: u64,
        mut cursor: [usize; 2],
        [left, right]: [Command; 2],
    ) -> Result<&Frame, Error> {
        let refined = matches!(left, Command::StepRefined { .. })
            || self.pending.as_ref().is_some_and(|p| matches!(p.action, Action::SceneWithSeparation(_, true)));
        let a = observation(self.call(&mut cursor, 0, left)?)?;
        let b = observation(self.call(&mut cursor, 1, right)?)?;
        let next = Frame::new(end, [a, b])?;
        if !self.frame.unchanged_owners(&next) {
            return Err(Error::Invalid("step changed ownership"));
        }
        self.frame = next;
        self.last_island_ids = self.pending.as_ref().and_then(|p| match &p.action {
            Action::Islands(_, ids) => Some(ids.clone()),
            _ => None,
        });
        self.last_refined = self
            .last_island_ids
            .as_ref()
            .map_or(refined, |ids| !ids.is_empty());
        self.queued.clear();
        self.commit_action();
        Ok(&self.frame)
    }

    /// Fence and replace a peer from its latest retained bootstrap. Verify every
    /// acknowledged reply in the remaining journal before publishing another barrier.
    pub fn recover(&mut self, worker: u8) -> Result<(), Error> {
        if self.aborting {
            return Err(Error::Pending);
        }
        let i = usize::from(worker);
        let len = self
            .journals
            .get(i)
            .ok_or(Error::Invalid("unknown worker"))?
            .len();
        self.replay_prefix(i, len)
    }

    /// Abandon a partially delivered operation by fencing and rebuilding both workers
    /// at the last published frame, even when a reply was lost after native mutation.
    /// A failed rollback must itself be retried before any further advancement.
    pub fn abort_pending(&mut self) -> Result<(), Error> {
        let Some(pending) = &self.pending else {
            return Ok(());
        };
        let start = pending.start;
        self.aborting = true;
        for (i, count) in start.iter().copied().enumerate() {
            self.replay_prefix(i, count)?;
        }
        for (log, count) in self.journals.iter_mut().zip(start) {
            log.truncate(count);
        }
        self.pending = None;
        self.aborting = false;
        Ok(())
    }

    pub fn into_peers(self) -> [P; 2] {
        self.peers
    }

    fn replay_prefix(&mut self, i: usize, len: usize) -> Result<(), Error> {
        let worker = i as u8;
        let peer = &mut self.peers[i];
        self.offline[i] = true;
        peer.restart()?;
        for (req, expected) in &self.journals[i][..len] {
            let actual = exchange(peer, req)?;
            // Body and ContactImpulse compare every f64 by its bits.
            if &actual != expected {
                return Err(Error::Diverged(worker, req.sequence));
            }
        }
        self.offline[i] = false;
        Ok(())
    }
}
