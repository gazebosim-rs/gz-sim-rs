// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Transfer two cache-drained independent assemblies into one interacting native world.
use super::profile::sealed::Recipe;
use super::{
    cluster::{self, CoupledGraspFleetPool, GraspFleetPairReaction},
    placement::fleet::{FleetCut, GraspFleetPlacement, GraspFleetPool, GraspFleetSpec},
    *,
};
use crate::articulation::ledger::{ADAPTIVE_GRASP_FLEET_FILE as FILE, Ledger};
use bincode::Options;
use gz_physics::JointVelocityMotor;
use gz_physics_multifidelity::MovingReaction;
use std::{
    fs::File,
    io::{self, Read},
    path::{Path, PathBuf},
    sync::Arc,
};
#[cfg(test)]
mod crash_tests;
mod replay;
use replay::{Event, Replay};
pub use replay::{GraspReplayBudget, GraspReplayTiming};
const MAGIC: [u8; 8] = *b"GZFACU01";
const WARM_MAGIC: [u8; 8] = *b"GZFACU03";
/// How an independently hosted pair of grippers enters a shared native contact world.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GraspFleetMerge {
    ClearMembers,
    VerifiedReplay(GraspReplayBudget),
    WarmMembers,
}
/// Costs of the last successful direct native graph promotion on this controller.
#[derive(Clone, Copy, Debug)]
pub struct GraspMemberTiming {
    pub source_packet_bytes: [usize; 2],
    /// Both speculative native worker processes contributed their confirmed prior images.
    pub reused_source_workers: bool,
    pub export_us: u64,
    pub install_us: u64,
    pub first_step_us: u64,
}
const REPLAY_MAGIC: [u8; 8] = *b"GZFACU02";
const REPLAY_LIMIT: usize = 4 * 1024 * 1024;
enum Change {
    Input(Event),
    Topology,
    Reset,
}
// Two 128KiB independent members or one 256KiB coupled graph, plus public rows/recipe.
const LIMIT: usize = 384 * 1024;

#[derive(Clone, Serialize, Deserialize)]
enum Saved {
    Separate(FleetCut),
    Coupled {
        cut: cluster::Cut,
        joined_at: Option<u64>,
    },
}
enum Active {
    Separate(GraspFleetPool),
    Coupled {
        pool: CoupledGraspFleetPool,
        joined_at: Option<u64>,
    },
}
impl Active {
    fn snapshot(&self) -> Saved {
        match self {
            Self::Separate(p) => Saved::Separate(p.committed_cut().clone()),
            Self::Coupled { pool, joined_at } => Saved::Coupled {
                cut: pool.committed_cut().clone(),
                joined_at: *joined_at,
            },
        }
    }
    fn restore(
        exe: &Path,
        ledger: Arc<Ledger>,
        spec: &GraspFleetSpec,
        saved: &Saved,
        warm: bool,
    ) -> Result<Self> {
        match saved {
            Saved::Separate(c) => Ok(Self::Separate(GraspFleetPool::resume_contained(
                exe,
                ledger,
                spec.clone(),
                c.clone(),
                warm,
            )?)),
            Saved::Coupled { cut, joined_at } => {
                if joined_at.is_some_and(|t| t >= cut.snapshot.tick) {
                    return Err(invalid("invalid adaptive grasp fleet join interval"));
                }
                Ok(Self::Coupled {
                    pool: CoupledGraspFleetPool::resume_contained(
                        exe,
                        ledger,
                        spec.clone(),
                        cut.clone(),
                    )?,
                    joined_at: *joined_at,
                })
            }
        }
    }
    fn cold(exe: &Path, l: Arc<Ledger>, width: u8, s: &GraspFleetSpec, warm: bool) -> Result<Self> {
        cluster::admissible(s)?;
        match GraspFleetPool::cold_contained(
            exe,
            l.clone(),
            width,
            s.clone(),
            GraspFleetPlacement::RoundRobin,
            warm,
        ) {
            Ok(p) => Ok(Self::Separate(p)),
            Err(ArticulationError::PossibleCrossAssemblyContact { .. }) => Ok(Self::Coupled {
                pool: CoupledGraspFleetPool::cold_contained(exe, l, width, s.clone())?,
                joined_at: None,
            }),
            Err(e) => Err(e),
        }
    }
}
impl Saved {
    fn epoch(&self) -> u64 {
        match self {
            Self::Separate(c) => c.epoch,
            Self::Coupled { cut, .. } => cut.epoch,
        }
    }
    fn generation(&self) -> u64 {
        match self {
            Self::Separate(c) => c.generation,
            Self::Coupled { cut, .. } => cut.generation,
        }
    }
    fn tick(&self) -> u64 {
        match self {
            Self::Separate(c) => c.tick,
            Self::Coupled { cut, .. } => cut.snapshot.tick,
        }
    }
    fn width(&self) -> u8 {
        match self {
            Self::Separate(c) => c.width,
            Self::Coupled { cut, .. } => cut.width,
        }
    }
    fn owners(&self) -> &[u8] {
        match self {
            Self::Separate(c) => &c.owners,
            Self::Coupled { cut, .. } => &cut.owners,
        }
    }
    fn rebase_cold(&mut self, epoch: u64, generation: u64) {
        match self {
            Self::Separate(c) => {
                c.epoch = epoch;
                c.generation = generation;
                c.leases.fill(generation);
            }
            Self::Coupled { cut, joined_at } => {
                cut.epoch = epoch;
                cut.generation = generation;
                *joined_at = None;
            }
        }
    }
    fn member(
        &self,
        i: usize,
    ) -> Option<(
        &FloatingTreeObservation,
        &PayloadReceipt,
        &[Option<MotorSetting>],
    )> {
        match self {
            Self::Separate(c) => {
                let s = c.groups.get(i)?;
                Some((&s.state, s.image.payload()?, s.image.motors()?))
            }
            Self::Coupled { cut, .. } => {
                let m = cut.snapshot.members.get(i)?;
                Some((&m.tree, &m.part, &m.motors))
            }
        }
    }
}
fn codec(limit: usize) -> impl Options {
    bincode::DefaultOptions::new()
        .with_limit(limit as u64)
        .reject_trailing_bytes()
}
fn boundary(_name: &str) {
    #[cfg(test)]
    crate::moving::crash_tests::gate(_name);
}
/// A real two-worker split may become one native contact world on the first interval
/// whose swept bounds overlap. Both private trials and the transfer share one publication.
pub struct AdaptiveGraspFleetPool {
    ledger: Arc<Ledger>,
    exe: PathBuf,
    spec: GraspFleetSpec,
    committed: Saved,
    replay: Option<Replay>,
    warm: bool,
    member_timing: Option<GraspMemberTiming>,
    #[cfg(test)]
    kill_before_proof: Option<u8>,
    timing: Option<GraspReplayTiming>,
    active: Option<Active>,
}
impl AdaptiveGraspFleetPool {
    fn ledger(dir: &Path, create: bool) -> Result<Arc<Ledger>> {
        let l = Ledger::lock(dir, create)?;
        if l.other_archive_exists(FILE)? {
            return Err(invalid(
                "session belongs to another distributed physics profile",
            ));
        }
        Ok(Arc::new(l))
    }
    fn read(l: &Ledger) -> Result<(GraspFleetSpec, Saved, Option<Replay>, bool)> {
        let mut b = Vec::new();
        File::open(l.archive_path(FILE))?
            .take((REPLAY_LIMIT + 41) as u64)
            .read_to_end(&mut b)?;
        if !(40..=REPLAY_LIMIT + 40).contains(&b.len()) || Sha256::digest(&b[40..])[..] != b[8..40]
        {
            return Err(invalid(
                "incorrect adaptive grasp fleet archive size or digest",
            ));
        }
        if (b[..8] == MAGIC || b[..8] == WARM_MAGIC) && b.len() <= LIMIT + 40 {
            let (s, c) = codec(LIMIT)
                .deserialize(&b[40..])
                .map_err(io::Error::other)?;
            Ok((s, c, None, b[..8] == WARM_MAGIC))
        } else if b[..8] == REPLAY_MAGIC {
            let (s, c, r) = codec(REPLAY_LIMIT)
                .deserialize(&b[40..])
                .map_err(io::Error::other)?;
            Ok((s, c, Some(r), false))
        } else {
            Err(invalid("unsupported adaptive grasp fleet archive version"))
        }
    }
    fn store(&self, s: &Saved, replay: Option<&Replay>) -> io::Result<()> {
        let (magic, bytes) = match replay {
            Some(r) => (
                REPLAY_MAGIC,
                codec(REPLAY_LIMIT).serialize(&(&self.spec, s, r)),
            ),
            None => (
                if self.warm { WARM_MAGIC } else { MAGIC },
                codec(LIMIT).serialize(&(&self.spec, s)),
            ),
        };
        self.ledger.publish_bytes(
            &self.ledger.archive_path(FILE),
            &magic,
            &bytes.map_err(io::Error::other)?,
            boundary,
        )
    }
    fn fresh(
        exe: &Path,
        l: Arc<Ledger>,
        width: u8,
        s: GraspFleetSpec,
        policy: GraspFleetMerge,
    ) -> Result<Self> {
        let exe = exe.canonicalize()?;
        let active = Active::cold(
            &exe,
            l.clone(),
            width,
            &s,
            policy == GraspFleetMerge::WarmMembers,
        )?;
        let saved = active.snapshot();
        let p = Self {
            ledger: l,
            exe,
            spec: s,
            replay: match policy {
                GraspFleetMerge::VerifiedReplay(b) => Some(Replay::new(b, &saved)?),
                _ => None,
            },
            warm: policy == GraspFleetMerge::WarmMembers,
            member_timing: None,
            #[cfg(test)]
            kill_before_proof: None,
            timing: None,
            committed: saved,
            active: Some(active),
        };
        p.store(&p.committed, p.replay.as_ref())?;
        Ok(p)
    }
    fn open(
        exe: &Path,
        l: Arc<Ledger>,
        spec: GraspFleetSpec,
        committed: Saved,
        replay: Option<Replay>,
        warm: bool,
    ) -> Result<Self> {
        cluster::admissible(&spec)?;
        let exe = exe.canonicalize()?;
        if let Some(r) = &replay {
            r.validate(&committed)?;
        }
        let state = Active::restore(&exe, l.clone(), &spec, &committed, warm)?;
        Ok(Self {
            ledger: l,
            exe,
            spec,
            committed,
            replay,
            warm,
            member_timing: None,
            #[cfg(test)]
            kill_before_proof: None,
            timing: None,
            active: Some(state),
        })
    }
    pub fn create(exe: &Path, dir: &Path, width: u8, s: GraspFleetSpec) -> Result<Self> {
        Self::create_with_policy(exe, dir, width, s, GraspFleetMerge::ClearMembers)
    }
    pub fn create_with_replay(
        exe: &Path,
        dir: &Path,
        width: u8,
        s: GraspFleetSpec,
        budget: GraspReplayBudget,
    ) -> Result<Self> {
        Self::create_with_policy(exe, dir, width, s, GraspFleetMerge::VerifiedReplay(budget))
    }
    pub fn create_with_policy(
        exe: &Path,
        dir: &Path,
        width: u8,
        s: GraspFleetSpec,
        policy: GraspFleetMerge,
    ) -> Result<Self> {
        if let GraspFleetMerge::VerifiedReplay(b) = policy {
            b.validate()?;
        }
        let l = Self::ledger(dir, true)?;
        if l.archive_path(FILE).try_exists()? {
            return Err(invalid("adaptive grasp fleet session already exists"));
        }
        Self::fresh(exe, l, width, s, policy)
    }
    pub fn resume(exe: &Path, dir: &Path) -> Result<Self> {
        let l = Self::ledger(dir, false)?;
        let (s, c, r, warm) = Self::read(&l)?;
        Self::open(exe, l, s, c, r, warm)
    }
    pub fn open_scene(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: GraspFleetSpec,
    ) -> Result<Self> {
        Self::open_scene_with_policy(exe, dir, width, spec, GraspFleetMerge::ClearMembers)
    }
    pub fn open_scene_with_replay(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: GraspFleetSpec,
        budget: GraspReplayBudget,
    ) -> Result<Self> {
        Self::open_scene_with_policy(
            exe,
            dir,
            width,
            spec,
            GraspFleetMerge::VerifiedReplay(budget),
        )
    }
    pub fn open_scene_with_policy(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: GraspFleetSpec,
        policy: GraspFleetMerge,
    ) -> Result<Self> {
        if let GraspFleetMerge::VerifiedReplay(b) = policy {
            b.validate()?;
        }
        let l = Self::ledger(dir, true)?;
        if !l.archive_path(FILE).try_exists()? {
            return Self::fresh(exe, l, width.unwrap_or(2), spec, policy);
        }
        let (s, c, r, warm) = Self::read(&l)?;
        let saved_policy = if warm {
            GraspFleetMerge::WarmMembers
        } else {
            r.as_ref().map_or(GraspFleetMerge::ClearMembers, |r| {
                GraspFleetMerge::VerifiedReplay(r.budget)
            })
        };
        if policy != saved_policy
            || width.is_some_and(|w| w != c.width())
            || s.assemblies.len() != spec.assemblies.len()
            || !s
                .assemblies
                .iter()
                .zip(&spec.assemblies)
                .all(|(a, b)| a.digest() == b.digest())
        {
            return Err(invalid("saved adaptive fleet and authored scene differ"));
        }
        Self::open(exe, l, s, c, r, warm)
    }
    fn live(&mut self) -> Result<&mut Active> {
        self.active
            .as_mut()
            .ok_or(invalid("adaptive grasp fleet requires recovery"))
    }
    fn rollback(&mut self) -> Result<()> {
        self.member_timing = None;
        self.timing = None;
        self.active.take();
        self.active = Some(Active::restore(
            &self.exe,
            self.ledger.clone(),
            &self.spec,
            &self.committed,
            self.warm,
        )?);
        Ok(())
    }
    fn recording(&self, make: impl FnOnce() -> Event) -> Change {
        if self.replay.is_some() && !self.is_coupled() {
            Change::Input(make())
        } else {
            Change::Topology
        }
    }
    fn transact(&mut self, change: Change, op: impl FnOnce(&mut Self) -> Result<()>) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        if let Err(e) = op(self) {
            // A failed merge may already have closed the old owner processes. Only the
            // accepted graph may regain a lease after either private candidate failed.
            self.rollback()?;
            return Err(e);
        }
        let next = self.live()?.snapshot();
        let mut next_replay = self.replay.clone();
        if let Some(r) = &mut next_replay {
            let update = match change {
                Change::Input(e) => r.record(&next, e),
                Change::Reset => Replay::new(r.budget, &next).map(|fresh| *r = fresh),
                Change::Topology => Ok(()),
            };
            if let Err(e) = update {
                self.rollback()?;
                return Err(e);
            }
        }
        if next_replay.is_some() {
            boundary("adaptive-journal-prepared");
        }
        boundary("adaptive-cut-prepared");
        if let Err(e) = self.store(&next, next_replay.as_ref()) {
            self.active.take();
            self.member_timing = None;
            self.timing = None;
            return Err(ArticulationError::Publication(e));
        }
        boundary("after-sync");
        self.committed = next;
        self.replay = next_replay;
        Ok(())
    }
    fn advance_private(
        &mut self,
        efforts: &[(u64, f64)],
        forces: &[(u64, [f64; 3])],
    ) -> Result<()> {
        let warm = self.warm;
        let mut independent_trial = Vec::new();
        match self.live()? {
            Active::Coupled { pool, .. } => return pool.advance_with_inputs(efforts, forces),
            Active::Separate(p) => {
                match p.trial_with_inputs(efforts, forces, warm.then_some(&mut independent_trial)) {
                    Ok(()) => return Ok(()),
                    Err(ArticulationError::PossibleCrossAssemblyContact { .. }) => (),
                    Err(e) => return Err(e),
                }
            }
        }
        boundary("adaptive-crossing-trials-discarded");
        #[cfg(test)]
        if let Some(slot) = self.kill_before_proof.take() {
            let pid = self
                .worker_pid(slot)
                .ok_or(invalid("expected live test source"))?;
            assert!(
                std::process::Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
            );
        }
        let Saved::Separate(prior) = &self.committed else {
            return Err(invalid("expected authoritative disjoint graph"));
        };
        let t = std::time::Instant::now();
        let replayed = self
            .replay
            .as_ref()
            .map(|r| r.recreate(&self.spec, prior))
            .transpose()?;
        let mut timing = replayed.as_ref().map(|(_, events)| GraspReplayTiming {
            events: *events,
            replay_us: t.elapsed().as_micros() as u64,
            ..Default::default()
        });
        let (sources, reused_source_workers) = if replayed.is_none() && prior.tick != 0 {
            let Active::Separate(p) = self.active.as_mut().unwrap() else {
                unreachable!()
            };
            p.export_members(warm, &independent_trial)?
        } else {
            ([Vec::new(), Vec::new()], false)
        };
        let mut direct = replayed.is_none().then(|| GraspMemberTiming {
            source_packet_bytes: sources.each_ref().map(Vec::len),
            reused_source_workers,
            export_us: t.elapsed().as_micros() as u64,
            install_us: 0,
            first_step_us: 0,
        });
        if replayed.is_some() {
            boundary("adaptive-replay-history-verified");
        } else if warm {
            if reused_source_workers {
                boundary("adaptive-prior-proof-verified");
            }
            boundary("adaptive-warm-members-prepared");
        } else {
            boundary("adaptive-clear-members-prepared");
        }
        self.active.take();
        boundary("adaptive-independent-workers-closed");
        let installed = std::time::Instant::now();
        let mut coupled = if let Some((snapshot, _)) = replayed {
            CoupledGraspFleetPool::from_verified_snapshot(
                &self.exe,
                self.ledger.clone(),
                self.spec.clone(),
                prior,
                snapshot,
            )?
        } else {
            CoupledGraspFleetPool::import_contained(
                &self.exe,
                self.ledger.clone(),
                self.spec.clone(),
                prior,
                [&sources[0], &sources[1]],
                warm,
            )?
        };
        let install_us = installed.elapsed().as_micros() as u64;
        if let Some(row) = &mut direct {
            row.install_us = install_us;
        }
        if let Some(row) = &mut timing {
            row.install_us = install_us;
        }
        boundary("adaptive-coupled-prior-installed");
        let tick = prior.tick;
        let solve = std::time::Instant::now();
        coupled.advance_with_inputs_at(prior.epoch, tick, efforts, forces)?;
        let first_step_us = solve.elapsed().as_micros() as u64;
        if let Some(row) = &mut direct {
            row.first_step_us = first_step_us;
        }
        if let Some(row) = &mut timing {
            row.first_step_us = first_step_us;
        }
        if warm
            && coupled.cross_reactions().iter().all(|p| {
                let r = p.reaction;
                !r.active
                    && r.normal_impulse == 0.
                    && r.warmstart_normal_impulse == 0.
                    && r.tangent_impulse == [0.; 3]
            })
        {
            let members = &coupled.committed_cut().snapshot.members;
            if independent_trial.len() != members.len() {
                return Err(invalid(
                    "missing independent native trials at grasp promotion",
                ));
            }
            for (i, (expected, observed)) in independent_trial.iter().zip(members).enumerate() {
                if expected.state != observed.tree
                    || expected.image.payload() != Some(&observed.part)
                    || expected.image.motors() != Some(&observed.motors)
                {
                    return Err(ArticulationError::GraspWarmContinuationChanged {
                        epoch: prior.epoch,
                        tick,
                        model: self.spec.assemblies[i].payload.tree.model_id,
                    });
                }
            }
            boundary("adaptive-warm-continuation-verified");
        }
        self.member_timing = direct;
        self.timing = timing;
        self.active = Some(Active::Coupled {
            pool: coupled,
            joined_at: Some(tick),
        });
        boundary("adaptive-coupled-interval-prepared");
        Ok(())
    }
    pub fn advance_with_inputs(
        &mut self,
        efforts: &[(u64, f64)],
        forces: &[(u64, [f64; 3])],
    ) -> Result<()> {
        self.transact(self.recording(|| Event::step(efforts, forces)), |p| {
            p.advance_private(efforts, forces)
        })
    }
    pub fn advance_with_inputs_at(
        &mut self,
        epoch: u64,
        tick: u64,
        efforts: &[(u64, f64)],
        forces: &[(u64, [f64; 3])],
    ) -> Result<()> {
        self.check_interval(epoch, tick)?;
        self.advance_with_inputs(efforts, forces)
    }
    pub fn advance(&mut self) -> Result<()> {
        self.advance_with_inputs(&[], &[])
    }
    pub fn queue_effort(&mut self, id: u64, e: f64) -> Result<()> {
        self.transact(
            self.recording(|| Event::Queue(id, e.to_bits())),
            |p| match p.live()? {
                Active::Separate(s) => s.queue_effort(id, e),
                Active::Coupled { pool, .. } => pool.queue_effort(id, e),
            },
        )
    }
    pub fn set_velocity_motor(&mut self, id: u64, m: Option<JointVelocityMotor>) -> Result<()> {
        self.transact(
            self.recording(|| Event::Motor(id, m.map(MotorSetting::from))),
            |p| match p.live()? {
                Active::Separate(s) => s.set_velocity_motor(id, m),
                Active::Coupled { pool, .. } => pool.set_velocity_motor(id, m),
            },
        )
    }
    pub fn transfer(&mut self, model: u64, dest: u8) -> Result<()> {
        if self.model(model).is_none() {
            return Err(invalid("unknown adaptive fleet model"));
        }
        self.transact(Change::Topology, |p| match p.live()? {
            Active::Separate(s) => s.transfer(model, dest),
            Active::Coupled { pool, .. } => pool.transfer(dest),
        })
    }
    pub fn reset(&mut self) -> Result<()> {
        let epoch = self
            .epoch()
            .checked_add(1)
            .ok_or(invalid("adaptive grasp epoch exhausted"))?;
        let generation = self
            .generation()
            .checked_add(1)
            .ok_or(invalid("adaptive grasp generation exhausted"))?;
        self.transact(Change::Reset, |p| {
            p.active.take();
            let cold = Active::cold(&p.exe, p.ledger.clone(), p.width(), &p.spec, p.warm)?;
            let mut cut = cold.snapshot();
            drop(cold);
            cut.rebase_cold(epoch, generation);
            p.active = Some(Active::restore(
                &p.exe,
                p.ledger.clone(),
                &p.spec,
                &cut,
                p.warm,
            )?);
            p.member_timing = None;
            p.timing = None;
            boundary("adaptive-cold-roles-installed");
            Ok(())
        })
    }
    fn model(&self, id: u64) -> Option<usize> {
        self.spec
            .assemblies
            .binary_search_by_key(&id, |s| s.payload.tree.model_id)
            .ok()
    }
    fn joint(&self, id: u64) -> Option<(&FloatingTreeJointState, Option<MotorSetting>)> {
        (0..2).find_map(|i| {
            let (t, _, m) = self.committed.member(i)?;
            let j = t.joints.binary_search_by_key(&id, |r| r.joint_id).ok()?;
            Some((&t.joints[j], m[j]))
        })
    }
    pub fn spec(&self) -> &GraspFleetSpec {
        &self.spec
    }
    pub fn epoch(&self) -> u64 {
        self.committed.epoch()
    }
    pub fn tick(&self) -> u64 {
        self.committed.tick()
    }
    pub fn width(&self) -> u8 {
        self.committed.width()
    }
    pub fn generation(&self) -> u64 {
        self.committed.generation()
    }
    pub fn owners(&self) -> &[u8] {
        self.committed.owners()
    }
    pub fn merge_policy(&self) -> GraspFleetMerge {
        if self.warm {
            GraspFleetMerge::WarmMembers
        } else {
            self.replay
                .as_ref()
                .map_or(GraspFleetMerge::ClearMembers, |r| {
                    GraspFleetMerge::VerifiedReplay(r.budget)
                })
        }
    }
    pub fn member_timing(&self) -> Option<GraspMemberTiming> {
        self.member_timing
    }
    pub fn replay_budget(&self) -> Option<GraspReplayBudget> {
        self.replay.as_ref().map(|r| r.budget)
    }
    pub fn replay_timing(&self) -> Option<GraspReplayTiming> {
        self.timing
    }
    pub fn is_coupled(&self) -> bool {
        matches!(self.committed, Saved::Coupled { .. })
    }
    /// Input tick of the first accepted interval with both assemblies in one native world.
    pub fn joined_at(&self) -> Option<u64> {
        match self.committed {
            Saved::Separate(_) => None,
            Saved::Coupled { joined_at, .. } => joined_at,
        }
    }
    pub fn observation(&self, model: u64) -> Option<&FloatingTreeObservation> {
        Some(self.committed.member(self.model(model)?)?.0)
    }
    pub fn payload_state(&self, model: u64) -> Option<&ArticulationLinkState> {
        Some(&self.committed.member(self.model(model)?)?.1.body)
    }
    pub fn payload_reactions(&self, model: u64) -> Option<&[MovingReaction]> {
        Some(&self.committed.member(self.model(model)?)?.1.reactions)
    }
    pub fn image(&self, model: u64) -> Option<&[u8]> {
        let i = self.model(model)?;
        match &self.committed {
            Saved::Separate(c) => c.groups[i].image.native(),
            Saved::Coupled { cut, .. } => cut.snapshot.image.as_deref(),
        }
    }
    pub fn cross_reactions(&self) -> &[GraspFleetPairReaction] {
        match &self.committed {
            Saved::Separate(_) => &[],
            Saved::Coupled { cut, .. } => &cut.snapshot.pairs,
        }
    }
    pub fn velocity_motor(&self, id: u64) -> Option<JointVelocityMotor> {
        self.joint(id)?.1.map(MotorSetting::native)
    }
    pub fn pending_joint_effort(&self, id: u64) -> Option<f64> {
        self.joint(id)?.0.pending_effort
    }
    pub fn check_interval(&self, epoch: u64, tick: u64) -> Result<()> {
        crate::articulation::check_interval(
            self.active.is_some(),
            epoch,
            tick,
            self.epoch(),
            self.tick(),
        )
    }
    pub fn worker_pid(&self, slot: u8) -> Option<u32> {
        match self.active.as_ref()? {
            Active::Separate(p) => p.worker_pid(slot),
            Active::Coupled { pool, .. } => pool.worker_pid(slot),
        }
    }
    pub fn replace_worker(&mut self, slot: u8) -> Result<()> {
        match self.live()? {
            Active::Separate(p) => p.replace_worker(slot),
            Active::Coupled { pool, .. } => pool.replace_worker(slot),
        }
    }
    pub fn drop_next_acknowledgement(&mut self, slot: u8) -> Result<()> {
        match self.live()? {
            Active::Separate(p) => p.drop_next_acknowledgement(slot),
            Active::Coupled { pool, .. } => pool.drop_next_acknowledgement(slot),
        }
    }
}
