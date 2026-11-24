// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Barrier-atomic rigid simulation across three to eight local worker processes.
use crate::adaptive::Resolution;
use crate::{
    coordinator::{Error, project_component, validate_memberships_with_profile},
    physics::NativeSeedLayout,
    physics::{
        Body, BodyWrench, GroundPlane, MAX_PENDING_IMPULSES, PendingImpulse, SimulationStep,
        Snapshot, StepField,
    },
    protocol::{Command, MAX_BODY_IDS, Observation, Outcome, Reply, Request, WarmTicket},
    transport::WorkerProcess,
};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
};

mod balance;
pub use balance::{PoolBalanceDecision, PoolBalancePolicy, PoolLoadMode, PoolLoadSample, PoolLoadWindow};
mod edits;
mod handoff;
mod impulses;
mod native_file;
mod population;
mod periodic;
pub use periodic::{PeriodicPlacement,PlacementStatus};
mod placement_replay;
pub use placement_replay::{PlacementMember, PlacementMove, PlacementRecord};
mod recovery;
mod retry;
pub use native_file::{MAX_SCENE_METADATA_BYTES, NativeContactSnapshot, PoolNativeRecovery, ScenePopulation, WorkerContactPatches};
pub use crate::physics::{NativeFrictionModel, NativeSolverProfile};
mod routing;
mod startup;
const MAX_JOURNAL_BYTES: u64 = 64 * 1024 * 1024;

#[derive(Clone, Debug, PartialEq)]
pub struct PoolFrame {
    pub tick: u64,
    pub workers: Vec<Observation>,
}
impl PoolFrame {
    pub fn body(&self, id: u64) -> Option<(u8, &Body)> {
        self.workers.iter().enumerate().find_map(|(w, o)| {
            o.snapshot
                .bodies
                .binary_search_by_key(&id, |b| b.id)
                .ok()
                .map(|i| (w as u8, &o.snapshot.bodies[i]))
        })
    }
    pub fn step(&self) -> SimulationStep {
        self.workers[0].snapshot.step
    }
    pub fn ground(&self) -> Option<GroundPlane> {
        self.workers[0].snapshot.ground
    }
    fn accept_step(&mut self, worker: usize, reply: Outcome) -> Result<(), Error> {
        let Outcome::State(next) = reply else {
            return Err(Error::Invalid("expected pool step observation"));
        };
        let old = &self.workers[worker];
        if next.generations != old.generations
            || next.snapshot.ground != old.snapshot.ground
            || next.snapshot.step != old.snapshot.step
            || next.snapshot.bodies.len() != old.snapshot.bodies.len()
            || next
                .snapshot
                .bodies
                .iter()
                .zip(&old.snapshot.bodies)
                .any(|(a, b)| !a.same_definition(b))
        {
            return Err(Error::Invalid(
                "pool step changed ownership or immutable scene parameters",
            ));
        }
        self.workers[worker] = next;
        Ok(())
    }
    fn check(&self, profile: NativeSolverProfile) -> Result<(), Error> {
        let mut global = BTreeSet::new();
        for obs in &self.workers {
            let s = &obs.snapshot;
            if s.tick != self.tick
                || s.step != self.step()
                || s.ground != self.ground()
                || s.bodies.iter().map(|b| &b.id).ne(obs.generations.keys())
            {
                return Err(Error::Invalid("inconsistent pool observation"));
            }
            validate_memberships_with_profile(obs, profile)?;
            if obs.generations.keys().any(|id| !global.insert(*id)) {
                return Err(Error::Invalid("duplicate pool body ID"));
            }
        }
        Ok(())
    }
}

fn reply_digest(reply: &Reply) -> Result<[u8; 32], Error> {
    let mut hash = Sha256::new();
    // Canonical bincode receipts include scalar and byte-array writes. Feed
    // SHA whole chunks without allocating a second full native handoff.
    {
        use std::io::{BufWriter, Write};
        let mut bytes = BufWriter::with_capacity(8192, &mut hash);
        bincode::serialize_into(&mut bytes, reply)
            .map_err(|_| Error::Invalid("unserializable pool reply"))?;
        bytes.flush()?;
    }
    Ok(hash.finalize().into())
}
struct Entry {
    request: Request,
    digest: [u8; 32],
}
impl Entry {
    fn verify(&self, reply: &Reply) -> Result<(), Error> {
        if reply_digest(reply)? != self.digest {
            return Err(Error::Diverged(
                self.request.worker_id,
                self.request.sequence,
            ));
        }
        Ok(())
    }
}
#[derive(Default)]
struct Journal {
    entries: Vec<Entry>,
    bytes: u64,
}
impl Journal {
    fn push(&mut self, request: Request, reply: &Reply) -> Result<(), Error> {
        let digest = reply_digest(reply)?;
        self.bytes += bincode::serialized_size(&request)
            .map_err(|_| Error::Invalid("unserializable pool command"))?
            + 32;
        self.entries.push(Entry { request, digest });
        Ok(())
    }
}
// A worker may be ahead of this frame only while speculative requests are
// in flight or after an interrupted barrier. Its journal always stops at the
// last fully published barrier. Fencing also discards pending native transfers.
pub struct WorkerPool {
    profile: NativeSolverProfile,
    run: u64,
    peers: Vec<WorkerProcess>,
    history: Vec<Journal>,
    baseline: Vec<u64>,
    dirty: Vec<bool>,
    limit: usize,
    serial: u64,
    // Present after the first birth or permanent retirement. Earlier archive
    // formats have immutable membership and all initially issued IDs still live.
    population_high_water: Option<u64>,
    retired: Vec<BTreeMap<u64, u64>>,
    queued: Vec<PendingImpulse>,
    serials: Vec<u64>,
    frame: PoolFrame,
    local_lanes: usize,
    local_helper_limit: Option<usize>,
    last_helper_mask: Option<u8>,
    pending_step: Option<Box<retry::PendingStep>>,
    placement_replay: placement_replay::PlacementReplay,
    handoff_confirmation: Option<std::time::Duration>,
    step_confirmation: std::time::Duration,
    periodic: Option<Box<periodic::PlacementSampler>>,
    active_probe: Option<balance::ProbeId>,
    last_refined: bool,
    last_island_ids: Option<Vec<u64>>,
}
struct Stage {
    queued: Vec<PendingImpulse>,
    serials: Vec<u64>,
    retired: Vec<BTreeMap<u64, u64>>,
    logs: Vec<Journal>,
    serial: u64,
    frame: PoolFrame,
}
impl WorkerPool {
    pub fn spawn(
        executable: &Path,
        run: u64,
        seeds: Vec<Vec<Body>>,
        step: SimulationStep,
        ground: Option<GroundPlane>,
        journal_limit: usize,
    ) -> Result<Self, Error> {
        Self::spawn_with_profile(
            executable,
            run,
            seeds,
            step,
            ground,
            journal_limit,
            NativeSolverProfile::Standard32,
        )
    }

    /// Require every child to acknowledge the selected native solver profile before publishing the first barrier.
    pub fn spawn_with_profile(
        executable: &Path,
        run: u64,
        seeds: Vec<Vec<Body>>,
        step: SimulationStep,
        ground: Option<GroundPlane>,
        journal_limit: usize,
        profile: NativeSolverProfile,
    ) -> Result<Self, Error> {
        Self::spawn_with_layout(
            executable,
            run,
            seeds,
            step,
            ground,
            journal_limit,
            profile.into(),
        )
    }
    /// Construct every body in one native solver in ascending public-ID order.
    /// This reference placement keeps a serial native world on owner zero and
    /// initially leaves the other endpoints empty. Uniform stepping and complete
    /// migrations retain this graph; explicit fission or population edits can change it.
    pub fn spawn_canonical_scene(
        executable: &Path,
        run: u64,
        bodies: Vec<Body>,
        width: u8,
        step: SimulationStep,
        ground: Option<GroundPlane>,
        journal_limit: usize,
    ) -> Result<Self, Error> {
        Self::spawn_canonical_scene_with_friction(
            executable, run, bodies, width, step, ground, journal_limit, NativeFrictionModel::Grouped,
        )
    }

    /// Seed the same ID-ordered reference with an explicitly negotiated contact law.
    #[allow(clippy::too_many_arguments)]
    pub fn spawn_canonical_scene_with_friction(
        executable: &Path,
        run: u64,
        bodies: Vec<Body>,
        width: u8,
        step: SimulationStep,
        ground: Option<GroundPlane>,
        journal_limit: usize,
        friction: NativeFrictionModel,
    ) -> Result<Self, Error> {
        if !(3..=8).contains(&width) || bodies.len() > NativeSolverProfile::Extended64.max_bodies() {
            return Err(Error::Capacity);
        }
        let mut seeds = vec![bodies];
        seeds.resize_with(width.into(), Vec::new);
        Self::spawn_with_layout(
            executable,
            run,
            seeds,
            step,
            ground,
            journal_limit,
            NativeSeedLayout::Canonical64.with_friction(friction),
        )
    }
    fn spawn_with_layout(
        executable: &Path,
        run: u64,
        seeds: Vec<Vec<Body>>,
        step: SimulationStep,
        ground: Option<GroundPlane>,
        journal_limit: usize,
        layout: NativeSeedLayout,
    ) -> Result<Self, Error> {
        let profile = layout.profile();
        let width = seeds.len();
        if !(3..=8).contains(&width) {
            return Err(Error::Invalid("worker pool requires three to eight peers"));
        }
        if journal_limit == 0 || seeds.iter().any(|s| s.len() > MAX_BODY_IDS) {
            return Err(Error::Capacity);
        }
        let mut ids = BTreeSet::new();
        if seeds.iter().flatten().any(|b| !ids.insert(b.id)) {
            return Err(Error::Invalid("duplicate pool seed ID"));
        }
        // A seed on another worker must meet the same primitive separation
        // rule as a local cold seed. Otherwise the initial published frame
        // could contain a contact with no native solver owning both sides.
        let mut previous = Vec::new();
        for shard in &seeds {
            let current = shard
                .iter()
                .map(Body::bounds)
                .collect::<Result<Vec<_>, _>>()
                .map_err(|_| Error::Invalid("invalid pool seed geometry"))?;
            if current
                .iter()
                .any(|b| previous.iter().any(|a| b.predicts_overlap(a)))
            {
                return Err(Error::Invalid(
                    "pool seeds intersect across worker boundaries",
                ));
            }
            previous.extend(current);
        }
        let started=startup::start_in_pairs(&seeds,|w, seed| {
            let mut peer = WorkerProcess::spawn_in_pool_with_layout(
                executable,
                run,
                w as u8,
                width as u8,
                seed,
                step,
                ground,
                layout,
            )?;
            let request = Request {
                run_id: run,
                worker_id: w as u8,
                sequence: 1,
                command: Command::Observe,
            };
            let reply = peer.request(&request)?;
            let mut log = Journal::default();
            log.push(request, &reply)?;
            let Outcome::State(obs) = reply.outcome else {
                return Err(Error::Invalid("expected initial pool observation"));
            };
            if layout.is_canonical()
                && (obs.components.len() != usize::from(!seed.is_empty())
                    || obs.snapshot.bodies.len() != seed.len())
            {
                return Err(Error::Invalid(
                    "worker did not construct the complete canonical native solver",
                ));
            }
            if obs.generations.values().any(|g| *g != 0) {
                return Err(Error::Invalid("invalid initial pool generation"));
            }
            Ok((peer,log,obs))
        })?;
        let mut peers=Vec::with_capacity(width);
        let mut history=Vec::with_capacity(width);
        let mut workers=Vec::with_capacity(width);
        for (peer,log,obs) in started {
            peers.push(peer);
            history.push(log);
            workers.push(obs);
        }
        let frame = PoolFrame { tick: 0, workers };
        frame.check(profile)?;
        Ok(Self {
            profile,
            run,
            peers,
            history,
            population_high_water: None,
            retired: vec![BTreeMap::new(); width],
            baseline: vec![0; width],
            queued: Vec::new(),
            serials: vec![0; width],
            limit: journal_limit,
            serial: 0,
            dirty: vec![false; width],
            frame,
            local_lanes: std::thread::available_parallelism().map_or(1, usize::from),
            local_helper_limit: None,
            last_helper_mask: None,
            pending_step: None,
            placement_replay: Default::default(),
            handoff_confirmation: None,
            step_confirmation: crate::transport::REQUEST_TIMEOUT,
            periodic:None,
            active_probe: None,
            last_refined: false,
            last_island_ids: None,
        })
    }
    pub fn native_solver_profile(&self) -> NativeSolverProfile {
        self.profile
    }
    pub fn frame(&self) -> &PoolFrame {
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
    /// Cap the number of worker-local solver helpers granted per subsequent barrier.
    /// `None` uses all spare local CPUs allowed by the automatic scheduler. Zero
    /// disables helper grants. This runtime setting survives worker replacement
    /// and compaction. A newly spawned or restored coordinator starts in automatic mode.
    pub fn set_local_helper_limit(&mut self, limit: Option<usize>) {
        if self.local_helper_limit != limit {
            self.pending_step = None;
            self.local_helper_limit = limit;
        }
    }
    /// Local helper grants sent for the last step committed by this coordinator.
    /// Bit N grants worker N permission to use its helper. Workers may keep an
    /// eligible batch serial, for example when a service probe is active.
    /// Spawning or restoring a pool starts a new observation period.
    pub fn last_local_helper_mask(&self) -> Option<u8> {
        self.last_helper_mask
    }
    /// Deadline for a native request, including a packed step and its reply.
    /// The default is one second. Explicit budgets up to five seconds apply to
    /// this controller and its replacement workers. A newly reopened pool uses
    /// its normal default until the caller applies its runtime policy again.
    pub fn set_native_work_timeout(&mut self, timeout: std::time::Duration) -> Result<(), Error> {
        if timeout.is_zero() || timeout > std::time::Duration::from_secs(5) {
            return Err(Error::Invalid(
                "native work timeout must be positive and at most five seconds",
            ));
        }
        for peer in &mut self.peers {
            peer.native_work_timeout = timeout;
        }
        Ok(())
    }

    pub fn native_work_timeout(&self) -> std::time::Duration {
        self.peers[0].native_work_timeout
    }

    /// Configure the observation encoding for all current and replacement peers.
    pub fn set_packed_observations(&mut self, enabled: bool) {
        for peer in &mut self.peers {
            peer.set_packed_observations(enabled);
        }
    }
    /// Select packed native handoffs on peers that negotiated this encoding.
    /// Returns the number of current peers using it. Replacement peers negotiate again.
    pub fn set_packed_handoffs(&mut self, enabled: bool) -> usize {
        self.peers
            .iter_mut()
            .map(|p| usize::from(p.set_packed_handoffs(enabled)))
            .sum()
    }
    pub(crate) fn packed_handoff_workers(&self) -> usize {
        self.peers.iter().filter(|p| p.uses_packed_handoffs()).count()
    }
    /// Cumulative application traffic per worker, including fenced replacements.
    pub fn traffic(&self) -> Vec<crate::transport::Traffic> {
        self.peers.iter().map(WorkerProcess::traffic).collect()
    }
    pub fn worker_endpoint(&self, worker: u8) -> Option<&str> {
        self.peers.get(worker as usize).map(WorkerProcess::endpoint)
    }
    pub fn worker_pid(&self, worker: u8) -> Option<u32> {
        self.peers.get(worker as usize).map(WorkerProcess::pid)
    }
    /// Exercise recovery when a real worker executes a sequenced command but
    /// its acknowledgement is lost. The transport retains the normal deadline.
    pub fn drop_next_acknowledgement(&mut self, worker: u8) -> Result<(), Error> {
        self.drop_acknowledgement_after(worker, 0)
    }
    /// Withhold the reply after a number of preceding sequenced commands on
    /// this endpoint. Native previews and recovery image reads do not count.
    pub fn drop_acknowledgement_after(&mut self, worker: u8, earlier: usize) -> Result<(), Error> {
        self.peers
            .get_mut(worker as usize)
            .ok_or(Error::Invalid("unknown pool worker"))?
            .drop_acknowledgement_after(earlier);
        Ok(())
    }
    /// Qualification hook: kill the real worker after the selected sequenced
    /// operation responds, withholding that result from the coordinator.
    pub fn terminate_after_command(&mut self, worker: u8, earlier: usize) -> Result<(), Error> {
        self.peers.get_mut(worker as usize).ok_or(Error::Invalid("unknown pool worker"))?
            .terminate_after_command(earlier);
        Ok(())
    }

    pub fn terminate_worker(&mut self, worker: u8) -> Result<(), Error> {
        self.peers
            .get_mut(worker as usize)
            .ok_or(Error::Invalid("unknown pool worker"))?
            .kill()?;
        Ok(())
    }
    /// Fence a single endpoint and replay only its fully committed commands.
    pub fn recover(&mut self, worker: u8) -> Result<(), Error> {
        let w = usize::from(worker);
        let p = self
            .peers
            .get_mut(w)
            .ok_or(Error::Invalid("unknown pool worker"))?;
        self.pending_step = None;
        self.dirty[w] = true;
        p.restart()?;
        for entry in &self.history[w].entries {
            let reply = p.request(&entry.request)?;
            entry.verify(&reply)?;
        }
        self.dirty[w] = false;
        Ok(())
    }
    fn stage(&mut self) -> Result<Stage, Error> {
        if self.placement_replay.attempt.is_some() {
            return Err(Error::Pending);
        }
        self.stage_for_advance()
    }
    fn stage_for_advance(&mut self) -> Result<Stage, Error> {
        self.pending_step = None;
        for w in 0..self.peers.len() {
            if self.dirty[w] {
                self.recover(w as u8)?;
            }
        }
        Ok(Stage {
            queued: self.queued.clone(),
            serials: self.serials.clone(),
            retired: self.retired.clone(),
            frame: self.frame.clone(),
            serial: self.serial,
            logs: (0..self.peers.len()).map(|_| Journal::default()).collect(),
        })
    }
    fn request(&self, stage: &Stage, w: usize, command: Command) -> Result<Request, Error> {
        let count = self.history[w].entries.len() + stage.logs[w].entries.len();
        if count >= self.limit {
            return Err(Error::Capacity);
        }
        let r = Request {
            run_id: self.run,
            worker_id: w as u8,
            sequence: self.baseline[w]
                .checked_add(count as u64 + 1)
                .ok_or(Error::Capacity)?,
            command,
        };
        let bytes = bincode::serialized_size(&r)
            .map_err(|_| Error::Invalid("unserializable pool command"))?;
        if self.history[w].bytes + stage.logs[w].bytes + bytes + 32 > MAX_JOURNAL_BYTES {
            return Err(Error::Capacity);
        }
        Ok(r)
    }
    fn exchange(
        &mut self,
        stage: &mut Stage,
        w: usize,
        command: Command,
    ) -> Result<Outcome, Error> {
        self.exchange_in_stage(stage, w, command, None)
    }

    fn exchange_in_stage(
        &mut self, stage: &mut Stage, w: usize, command: Command, confirmation: Option<std::time::Duration>,
    ) -> Result<Outcome, Error> {
        let r = self.request(stage, w, command)?;
        self.dirty[w] = true;
        let reply = match self.peers[w].request(&r) {
            Err(crate::transport::TransportError::Unavailable(_)) if let Some(window) = confirmation => {
                self.confirm_handoff_request(stage, w, &r, window)?
            }
            result => result?,
        };
        stage.logs[w].push(r, &reply)?;
        Ok(reply.outcome)
    }
    fn commit(&mut self, stage: Stage) -> Result<&PoolFrame, Error> {
        stage.frame.check(self.profile)?;
        for (h, mut fresh) in self.history.iter_mut().zip(stage.logs) {
            h.entries.append(&mut fresh.entries);
            h.bytes += fresh.bytes;
        }
        self.retired = stage.retired;
        self.queued = stage.queued;
        self.serials = stage.serials;
        self.frame = stage.frame;
        self.serial = stage.serial;
        self.dirty.fill(false);
        Ok(&self.frame)
    }
    pub fn advance(&mut self) -> Result<&PoolFrame, Error> {
        self.advance_in_gravity([0.; 3])
    }
    /// Route prospective interactions, submit every step, then publish one frame.
    /// An unavailable final reply leaves this frame committed. Repeating the
    /// same inputs can confirm the original worker sequences without fencing.
    /// A missing worker is reconstructed from its journal and staged handoffs.
    /// A changed field first restores all speculative peers to the committed barrier.
    pub fn advance_in_gravity(&mut self, gravity: [f64; 3]) -> Result<&PoolFrame, Error> {
        self.advance_in_field(gravity, &[])
    }

    /// Apply a uniform world acceleration and ordered COM forces and torques
    /// across exactly one public interval, including all native substeps.
    pub fn advance_in_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<&PoolFrame, Error> {
        self.advance_selected(gravity, inputs, Resolution::Coarse)
    }

    /// Inspect the subdivision chosen for the next public barrier.
    pub fn adaptive_substeps(&self, gravity: [f64; 3], inputs: &[BodyWrench]) -> Result<u8, Error> {
        crate::adaptive::for_workers(&self.frame.workers, &self.queued, gravity, inputs)
    }

    /// Pick one common interval before staging any handoffs or dispatches.
    pub fn advance_adaptive_in_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<&PoolFrame, Error> {
        let refined = self.adaptive_substeps(gravity, inputs)? == 10;
        self.advance_selected(
            gravity,
            inputs,
            if refined {
                Resolution::Fine
            } else {
                Resolution::Coarse
            },
        )
    }

    /// Choose the scene-wide solve rate and split safely separated native worlds after
    /// each committed barrier. Uses the existing island step command at a uniform rate.
    /// A retried step must use this same policy to confirm cached worker replies.
    pub fn advance_adaptive_with_separation(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<&PoolFrame, Error> {
        let fine = self.adaptive_substeps(gravity, inputs)? == 10;
        self.advance_selected(gravity, inputs, Resolution::SceneWithSeparation { fine })
    }

    /// Stable IDs of bodies that would receive ten solves at the next barrier.
    pub fn refined_island_bodies(
        &self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<Vec<u64>, Error> {
        Ok(self
            .plan_field(&self.frame, gravity, &self.queued, inputs, true)?
            .1)
    }

    /// Keep one public barrier while independently choosing each prospective solver island's resolution.
    pub fn advance_islands_in_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<&PoolFrame, Error> {
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        let _trace = crate::barrier_trace::Scope::begin(self.frame.tick.saturating_add(1), None);
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        crate::barrier_trace::mark("api_start", None);
        let result = self.advance_selected(gravity, inputs, Resolution::Islands(Vec::new()));
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        crate::barrier_trace::mark("api_end", None);
        result
    }

    fn advance_selected(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
        resolution: Resolution,
    ) -> Result<&PoolFrame, Error> {
        self.advance_placed(gravity, inputs, resolution, None)
    }

    fn advance_placed(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
        mut resolution: Resolution,
        replay: Option<&PlacementRecord>,
    ) -> Result<&PoolFrame, Error> {
        self.check_attempt(gravity, inputs, replay)?;
        // Mandatory routing, local resolution and any measured transfer use the
        // same conservative sweep with the actual queued impulses and held field.
        let plan = routing::ProspectiveGroups::new(
            &self.frame, gravity, &self.queued, inputs, self.profile,
        )?;
        let (mut moves, refined) = self.route_plan(
            &self.frame, &self.queued, matches!(resolution, Resolution::Islands(_)), &plan,
        )?;
        if let Resolution::Islands(ids) = &mut resolution {
            *ids = refined;
        }
        if self.retry_step(gravity, inputs, &resolution)? {
            return Ok(&self.frame);
        }
        let next = self.frame.tick.checked_add(1).ok_or(Error::Capacity)?;
        let sampled_move = if let Some(record) = replay {
            self.validate_replay_routes(record, next, &mut moves, &plan)?;
            None
        } else {
            let choice = self.sampled_move(&plan, !moves.is_empty())?;
            if let Some(m) = &choice {
                moves.push((m.members[0], usize::from(m.destination)));
            }
            choice
        };
        drop(plan);
        self.reserve_step(&self.command_counts(&moves))?;
        let mut stage = self.stage_for_advance()?;
        if let Some(record) = replay {
            self.begin_replay_attempt(record, gravity, inputs)?;
        }
        for (member, d) in moves {
            self.transfer(&mut stage, member, d)?;
        }
        let requests: Vec<_> = (0..self.peers.len())
            .map(|w| {
                let tick = stage.frame.tick;
                let local: Vec<_> = inputs
                    .iter()
                    .filter(|input| stage.frame.workers[w].generations.contains_key(&input.id))
                    .cloned()
                    .collect();
                let command = resolution.command(
                    &stage.frame.workers[w],
                    StepField {
                        gravity,
                        wrenches: local,
                    },
                    |field| {
                        if gravity == [0.; 3] && field.wrenches.is_empty() {
                            Command::Step { tick }
                        } else {
                            Command::StepInField { tick, field }
                        }
                    },
                );
                self.request(&stage, w, command)
            })
            .collect::<Result<_, _>>()?;
        self.dirty.fill(true);
        let helper_mask = local_helpers(
            &stage.frame,
            &requests,
            self.local_lanes,
            self.local_helper_limit,
        );
        let replies = WorkerProcess::pool_steps(&mut self.peers, &requests, helper_mask);
        let batch = retry::StepBatch {
            sampled_move,
            stage,
            requests,
            helper_mask,
            resolution,
        };
        if replies.iter().any(Result::is_err) {
            return self.remember_interrupted_step(batch, gravity, inputs, replies);
        }
        self.finish_step(batch, next, replies)?;
        Ok(&self.frame)
    }

    fn finish_step(
        &mut self,
        batch: retry::StepBatch,
        next: u64,
        replies: Vec<Result<Reply, crate::transport::TransportError>>,
    ) -> Result<(), Error> {
        let retry::StepBatch {
            sampled_move,
            mut stage,
            requests,
            helper_mask,
            resolution,
        } = batch;
        for (w, (request, reply)) in requests.into_iter().zip(replies).enumerate() {
            let reply = reply?;
            stage.logs[w].push(request, &reply)?;
            stage.frame.accept_step(w, reply.outcome)?;
        }
        stage.frame.tick = next;
        stage.queued.clear();
        // Capture generations while the preceding authority is still published.
        let receipt = (matches!(resolution, Resolution::Islands(_))
            && self.placement_replay.attempt.is_none())
            .then(|| self.measured_placement_record(next, sampled_move.as_ref()));
        self.commit(stage)?;
        self.placement_replay.committed = if let Some(attempt) = self.placement_replay.attempt.take() {
            self.interrupt_periodic_window();
            Some(attempt.record)
        } else {
            receipt
        };
        self.record_sampled_move(sampled_move);
        self.last_helper_mask = Some(helper_mask);
        self.last_refined = match &resolution {
            Resolution::Fine => true,
            Resolution::Coarse => false,
            Resolution::Islands(ids) => !ids.is_empty(),
            Resolution::SceneWithSeparation { fine } => *fine,
        };
        self.last_island_ids = match resolution {
            Resolution::Islands(ids) => Some(ids),
            _ => None,
        };
        Ok(())
    }
}

// Only populated peers execute native solves. An idle CPU can assist a worker
// with at least two distinct fine worlds; each worker gets at most one helper.
fn local_helpers(
    frame: &PoolFrame,
    requests: &[Request],
    lanes: usize,
    limit: Option<usize>,
) -> u8 {
    let active = frame
        .workers
        .iter()
        .filter(|s| !s.snapshot.bodies.is_empty())
        .count();
    let spare = lanes
        .saturating_sub(active)
        .min(limit.unwrap_or(usize::MAX));
    if spare == 0 {
        return 0;
    }
    let mut mask = 0u8;
    for (worker, (obs, req)) in frame.workers.iter().zip(requests).enumerate() {
        let Command::StepIslands { refined, .. } = &req.command else {
            continue;
        };
        let mut fine = obs
            .components
            .iter()
            .filter(|c| refined.binary_search(&c.ids[0]).is_ok());
        if fine.nth(1).is_some() {
            mask |= 1 << worker;
        }
    }
    if mask.count_ones() as usize <= spare {
        return mask;
    }
    // Every owner must finish before publication. Give a contested helper to
    // the eligible owner with the largest estimated serial barrier cost.
    let mut ranked = Vec::with_capacity(mask.count_ones() as usize);
    for (worker, (obs, req)) in frame.workers.iter().zip(requests).enumerate() {
        if mask & (1 << worker) != 0
            && let Command::StepIslands { refined, .. } = &req.command
        {
            ranked.push((std::cmp::Reverse(owner_work(obs, refined)), worker));
        }
    }
    ranked.sort_unstable();
    ranked
        .iter()
        .take(spare)
        .fold(0, |bits, (_, w)| bits | (1 << w))
}

fn owner_work(obs: &Observation, refined: &[u64]) -> usize {
    let solves = |id| {
        if refined.binary_search(&id).is_ok() {
            10
        } else {
            1
        }
    };
    // Count per-world overhead, body integration and committed constraint
    // receipts at each world's selected rate. A retained pair belongs wholly
    // to one world, so either member identifies its rate.
    let worlds: usize = obs
        .components
        .iter()
        .map(|c| (1 + c.ids.len()) * solves(c.ids[0]))
        .sum();
    let pairs: usize = obs.snapshot.impulses.iter().map(|p| solves(p.a)).sum();
    let supports: usize = obs.snapshot.supports.iter().map(|p| solves(p.body)).sum();
    worlds + pairs + supports
}

#[cfg(test)]
#[path = "../tests/private/pool.rs"]
mod tests;
