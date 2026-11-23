// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    worker::{Answer, Job},
    *,
};
use crate::floating_tree::fleet_session::FleetSession;
use crate::{
    articulation::ledger::Ledger,
    moving::worker::{Peer, Pending},
};
use bincode::Options;
use gz_physics::JointVelocityMotor;
use std::sync::Arc;
use std::{
    fs::File,
    io::{self, Read},
    path::{Path, PathBuf},
};
fn disk_codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_limit(FLEET_LIMIT as u64)
        .reject_trailing_bytes()
}

/// One durable barrier across every gripper and part in the authored fleet.
pub struct GraspFleetPool {
    ledger: FleetSession,
    exe: PathBuf,
    spec: GraspFleetSpec,
    cut: FleetCut,
    peers: Vec<Option<Peer>>,
    healthy: bool,
    lane: MemberLane,
}
impl GraspFleetPool {
    pub(in crate::floating_tree) fn committed_cut(&self) -> &FleetCut {
        &self.cut
    }
    pub(in crate::floating_tree) fn resume_contained(
        exe: &Path,
        l: Arc<Ledger>,
        s: GraspFleetSpec,
        c: FleetCut,
        warm: bool,
    ) -> Result<Self> {
        Self::open(exe, FleetSession::Contained(l), s, c, false, warm)
    }
    pub(in crate::floating_tree) fn cold_contained(
        exe: &Path,
        l: Arc<Ledger>,
        width: u8,
        s: GraspFleetSpec,
        placement: GraspFleetPlacement,
        warm: bool,
    ) -> Result<Self> {
        Self::fresh(exe, FleetSession::Contained(l), width, s, placement, warm)
    }
    pub(in crate::floating_tree) fn export_members(
        &mut self,
        warm: bool,
        trial: &[MemberSnapshot],
    ) -> Result<([Vec<u8>; 2], bool)> {
        if self.cut.groups.len() != 2 {
            return Err(invalid("promotion requires exactly two assemblies"));
        }
        let retained = warm && self.lane == MemberLane::RetainedPrior && !self.healthy;
        if retained && trial.len() != 2 {
            return Err(invalid("missing private trials for warm prior proof"));
        }
        match self.export_pair(warm, retained.then_some(trial)) {
            Ok(packets) => Ok((packets, retained)),
            Err(ArticulationError::Io(_)) if retained => {
                // A lost speculative owner can always be reconstructed from the durable cut.
                self.recover()?;
                Ok((self.export_pair(warm, None)?, false))
            }
            Err(e) => Err(e),
        }
    }
    fn export_pair(
        &mut self,
        warm: bool,
        trial: Option<&[MemberSnapshot]>,
    ) -> Result<[Vec<u8>; 2]> {
        let (magic, limit) = if warm {
            (
                b"GZFWAM01",
                gz_physics_multifidelity::WARM_GRASP_MEMBER_IMAGE_LIMIT,
            )
        } else {
            (
                b"GZFCLR01",
                gz_physics_multifidelity::CLEAR_GRASP_MEMBER_IMAGE_LIMIT,
            )
        };
        let mut members = [Vec::new(), Vec::new()];
        for (i, member) in members.iter_mut().enumerate() {
            let tick = self.tick();
            let lease = self.cut.leases[i];
            let owner = self.cut.owners[i];
            let packet = if let Some(trial) = trial {
                let old = self.cut.groups[i].digest;
                let new = trial[i].digest;
                let next = tick
                    .checked_add(1)
                    .ok_or(invalid("grasp fleet clock exhausted"))?;
                match self.one(
                    i,
                    owner,
                    next,
                    lease,
                    Op::PriorWarm {
                        prior_tick: tick,
                        prior_digest: old,
                        trial_digest: new,
                    },
                )? {
                    Reply::PriorWarm {
                        prior_digest,
                        trial_digest,
                        packet,
                    } if prior_digest == old && trial_digest == new => packet,
                    _ => {
                        return Err(invalid(
                            "retained source did not prove both native member receipts",
                        ));
                    }
                }
            } else {
                let op = if warm {
                    Op::WarmMember
                } else {
                    Op::ClearMember
                };
                match self.one(i, owner, tick, lease, op)? {
                    Reply::ClearMember(None) if warm => {
                        match self.one(i, owner, tick, lease, Op::ClearMember)? {
                            Reply::ClearMember(v) => v,
                            _ => return Err(invalid("incorrect clear grasp member response")),
                        }
                    }
                    Reply::ClearMember(v) => v,
                    _ => return Err(invalid("incorrect native grasp member export")),
                }
            };
            let model = self.spec.assemblies[i].payload.tree.model_id;
            *member = match packet {
                Some(bytes)
                    if (bytes.starts_with(magic) && bytes.len() <= limit)
                        || (warm
                            && bytes.starts_with(b"GZFCLR01")
                            && bytes.len()
                                <= gz_physics_multifidelity::CLEAR_GRASP_MEMBER_IMAGE_LIMIT) =>
                {
                    bytes
                }
                None if warm => {
                    return Err(ArticulationError::GraspWarmMemberUnavailable {
                        epoch: self.epoch(),
                        tick,
                        model,
                    });
                }
                None => {
                    return Err(ArticulationError::GraspMergeUnavailable {
                        epoch: self.epoch(),
                        tick,
                        model,
                    });
                }
                _ => return Err(invalid("incorrect native grasp member export")),
            };
        }
        Ok(members)
    }
    fn ledger(dir: &Path, create: bool) -> Result<Ledger> {
        let l = Ledger::lock(dir, create)?;
        if l.other_archive_exists(FILE)? {
            return Err(invalid(
                "directory belongs to a different fleet or physics profile",
            ));
        }
        Ok(l)
    }
    fn read(l: &Ledger) -> Result<(GraspFleetSpec, FleetCut)> {
        let mut b = Vec::new();
        File::open(l.archive_path(FILE))?
            .take((FLEET_LIMIT + 41) as u64)
            .read_to_end(&mut b)?;
        if !(40..=FLEET_LIMIT + 40).contains(&b.len())
            || b[..8] != ARCHIVE
            || b[8..40] != Sha256::digest(&b[40..])[..]
        {
            return Err(invalid("invalid grasp fleet archive format or checksum"));
        }
        Ok(disk_codec()
            .deserialize(&b[40..])
            .map_err(io::Error::other)?)
    }
    pub fn create(
        exe: &Path,
        dir: &Path,
        width: u8,
        spec: GraspFleetSpec,
        placement: GraspFleetPlacement,
    ) -> Result<Self> {
        let l = Self::ledger(dir, true)?;
        if l.archive_path(FILE).try_exists()? {
            return Err(invalid("grasp fleet session already exists"));
        }
        Self::fresh(exe, l.into(), width, spec, placement, false)
    }
    pub fn resume(exe: &Path, dir: &Path) -> Result<Self> {
        let l = Self::ledger(dir, false)?;
        let (s, c) = Self::read(&l)?;
        Self::open(exe, l.into(), s, c, false, false)
    }
    pub fn open_scene(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: GraspFleetSpec,
        placement: GraspFleetPlacement,
    ) -> Result<Self> {
        let l = Self::ledger(dir, true)?;
        if !l.archive_path(FILE).try_exists()? {
            return Self::fresh(exe, l.into(), width.unwrap_or(2), spec, placement, false);
        }
        let (old, c) = Self::read(&l)?;
        if width.is_some_and(|w| w != c.width)
            || c.placement != placement
            || old.assemblies.len() != spec.assemblies.len()
            || !old
                .assemblies
                .iter()
                .zip(&spec.assemblies)
                .all(|(a, b)| a.digest() == b.digest())
        {
            return Err(invalid(
                "grasp fleet session and authored construction or placement differ",
            ));
        }
        Self::open(exe, l.into(), old, c, false, false)
    }
    fn fresh(
        exe: &Path,
        l: FleetSession,
        width: u8,
        s: GraspFleetSpec,
        placement: GraspFleetPlacement,
        warm_lane: bool,
    ) -> Result<Self> {
        if !(2..=8).contains(&width) {
            return Err(invalid(
                "grasp fleet expects two to eight real worker processes",
            ));
        }
        let (groups, bounds) = s.cold()?;
        s.check_separation(&bounds, 0, 0)?;
        let n = groups.len();
        let c = FleetCut {
            epoch: 0,
            tick: 0,
            generation: 0,
            width,
            placement,
            owners: placement.owners(n, width),
            leases: vec![0; n],
            groups,
        };
        let p = Self::open(exe, l, s, c, true, warm_lane)?;
        p.publish(&p.cut)?;
        Ok(p)
    }
    fn open(
        exe: &Path,
        ledger: FleetSession,
        spec: GraspFleetSpec,
        cut: FleetCut,
        cold_checked: bool,
        warm_lane: bool,
    ) -> Result<Self> {
        spec.check_ids()?;
        cut.validate(&spec)?;
        if !cold_checked {
            let bounds = spec
                .assemblies
                .iter()
                .zip(&cut.groups)
                .map(|(s, snap)| {
                    let w = TreeWorld::restore(s, snap)?;
                    let (tree, part) = w.collision_bounds()?;
                    Ok(Bounds { tree, part })
                })
                .collect::<Result<Vec<_>>>()?;
            spec.check_separation(&bounds, cut.epoch, cut.tick)?;
        }
        let mut p = Self {
            ledger,
            exe: exe.canonicalize()?,
            spec,
            cut,
            peers: vec![],
            healthy: false,
            lane: if !warm_lane {
                MemberLane::Standard
            } else if crate::floating_tree::pool::worker_supports(
                exe,
                "--probe-grasp-fleet-prior-v3",
            )? {
                MemberLane::RetainedPrior
            } else {
                MemberLane::Warm
            },
        };
        p.recover()?;
        Ok(p)
    }
    fn publish(&self, c: &FleetCut) -> io::Result<()> {
        if !self.ledger.standalone() {
            return Ok(());
        }
        let bytes = disk_codec()
            .serialize(&(&self.spec, c))
            .map_err(io::Error::other)?;
        self.ledger
            .publish_bytes(&self.ledger.archive_path(FILE), &ARCHIVE, &bytes, boundary)
    }
    fn spawn(&self, slot: u8) -> Result<Peer> {
        Peer::new_profile(
            &self.exe,
            slot,
            &self.ledger.slot(slot),
            &self.spec,
            self.lane.flag(),
            self.lane.wire(),
        )
    }
    fn peer(&mut self, slot: u8) -> Result<&mut Peer> {
        self.peers
            .get_mut(slot as usize)
            .and_then(Option::as_mut)
            .ok_or(invalid("unknown or fenced fleet worker"))
    }
    fn recover(&mut self) -> Result<()> {
        self.healthy = false;
        self.peers.clear();
        for i in 0..self.width() {
            let peer = self.spawn(i)?;
            self.peers.push(Some(peer));
        }
        for i in 0..self.spec.assemblies.len() {
            self.restore_member(i)?;
        }
        self.healthy = true;
        Ok(())
    }
    fn restore_member(&mut self, i: usize) -> Result<()> {
        let snap = self.cut.groups[i].clone();
        let owner = self.cut.owners[i];
        let lease = self.cut.leases[i];
        let tick = self.tick();
        let digest = self.stage(i, owner, tick, lease, snap)?;
        self.ack(i, owner, tick, lease, Op::Activate(digest))
    }
    fn transact(
        &mut self,
        retain_rejected: bool,
        mut run: impl FnMut(&mut Self, &mut FleetCut) -> Result<()>,
    ) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        let mut retry = true;
        loop {
            let mut c = self.cut.clone();
            if let Err(e) = run(self, &mut c).and_then(|()| c.validate(&self.spec)) {
                if retain_rejected
                    && matches!(e, ArticulationError::PossibleCrossAssemblyContact { .. })
                {
                    // Only the containing owner can now query the discarded proposal. No
                    // command can advance or publish this fleet until it has recovered.
                    self.healthy = false;
                    return Err(e);
                }
                self.recover()?;
                if retry && matches!(e, ArticulationError::Io(_)) {
                    retry = false;
                    continue;
                }
                return Err(e);
            }
            if let Err(e) = self.publish(&c) {
                self.healthy = false;
                self.peers.clear();
                return Err(ArticulationError::Publication(e));
            }
            if self.ledger.standalone() {
                boundary("after-sync");
            }
            self.cut = c;
            return Ok(());
        }
    }
    fn one(&mut self, group: usize, slot: u8, tick: u64, generation: u64, op: Op) -> Result<Reply> {
        match self.peer(slot)?.call_typed(
            tick,
            Job::Batch(vec![(group as u8, Request { generation, op })]),
        )? {
            Answer::Inline(mut rows) if rows.len() == 1 && rows[0].0 == group as u8 => {
                Ok(rows.pop().expect("checked length").1)
            }
            _ => Err(invalid(
                "grasp fleet request received incorrect member receipt",
            )),
        }
    }
    fn stage(
        &mut self,
        i: usize,
        slot: u8,
        tick: u64,
        generation: u64,
        snap: Box<Snapshot>,
    ) -> Result<[u8; 32]> {
        let piece = Piece::Coupled(snap);
        let d = piece.digest();
        match self.one(i, slot, tick, generation, Op::Stage(Seed::Install(piece)))? {
            Reply::State(p) if p.digest() == d => Ok(d),
            _ => Err(invalid("staged fleet member differs from accepted graph")),
        }
    }
    fn ack(&mut self, i: usize, slot: u8, tick: u64, generation: u64, op: Op) -> Result<()> {
        if !matches!(self.one(i, slot, tick, generation, op)?, Reply::Ack) {
            return Err(invalid("missing grasp fleet ownership acknowledgement"));
        }
        Ok(())
    }
    fn collect(
        &mut self,
        slot: u8,
        tick: u64,
        expected: &[u8],
        pending: Pending,
    ) -> Result<Vec<(u8, Reply)>> {
        let answer = self.peer(slot)?.collect(pending)?;
        let rows = match answer {
            Answer::Inline(r) => r,
            Answer::Paged(ids) if ids == expected => {
                let mut rows = Vec::with_capacity(ids.len());
                for id in ids {
                    match self.peer(slot)?.call_typed(tick, Job::Fetch(id))? {
                        Answer::Inline(mut one) if one.len() == 1 && one[0].0 == id => {
                            rows.push(one.pop().unwrap())
                        }
                        _ => return Err(invalid("incorrect grasp fleet continuation receipt")),
                    }
                }
                rows
            }
            _ => return Err(invalid("invalid grasp fleet page manifest")),
        };
        if !rows.iter().map(|r| r.0).eq(expected.iter().copied()) {
            return Err(invalid("incomplete grasp fleet step batch"));
        }
        Ok(rows)
    }
    fn advance_cut(
        &mut self,
        c: &mut FleetCut,
        commands: &[Vec<(u64, f64)>],
        forces: &[[f64; 3]],
        rejected: &mut Option<&mut Vec<MemberSnapshot>>,
    ) -> Result<()> {
        let tick = c.tick;
        let next = tick
            .checked_add(1)
            .ok_or(invalid("grasp fleet clock exhausted"))?;
        let mut work: Vec<Vec<(u8, Request)>> = (0..c.width).map(|_| Vec::new()).collect();
        for (i, input) in commands.iter().enumerate() {
            work[c.owners[i] as usize].push((
                i as u8,
                Request {
                    generation: c.leases[i],
                    op: Op::Step {
                        efforts: input.clone(),
                        force: forces[i],
                        bounds: true,
                    },
                },
            ));
        }
        let mut waiting = Vec::new();
        for (slot, jobs) in work
            .into_iter()
            .enumerate()
            .filter(|(_, jobs)| !jobs.is_empty())
        {
            let ids = jobs.iter().map(|r| r.0).collect::<Vec<_>>();
            let s = slot as u8;
            let pending = self.peer(s)?.submit(tick, Job::Batch(jobs))?;
            waiting.push((s, ids, pending));
        }
        let mut checks: Vec<Bounds> = (0..commands.len()).map(|_| Bounds::default()).collect();
        for (slot, ids, pending) in waiting {
            for (id, reply) in self.collect(slot, tick, &ids, pending)? {
                let Reply::Advanced(clock, Piece::Coupled(snapshot), bounds) = reply else {
                    return Err(invalid("fleet expected a coupled advancing receipt"));
                };
                if clock != next || snapshot.state.tick != next {
                    return Err(invalid("grasp fleet member advanced an incorrect interval"));
                }
                c.groups[id as usize] = snapshot;
                checks[id as usize] = bounds;
            }
            boundary("grasp-fleet-worker-prepared");
        }
        boundary("grasp-fleet-all-prepared");
        if let Err(e) = self.spec.check_separation(&checks, c.epoch, tick) {
            if let Some(out) = rejected {
                **out = std::mem::take(&mut c.groups);
            }
            boundary("grasp-fleet-crossing-rejected");
            return Err(e);
        }
        c.tick = next;
        Ok(())
    }
    pub fn advance_with_inputs(
        &mut self,
        efforts: &[(u64, f64)],
        payload_forces: &[(u64, [f64; 3])],
    ) -> Result<()> {
        self.trial_with_inputs(efforts, payload_forces, None)
    }
    pub(in crate::floating_tree) fn trial_with_inputs(
        &mut self,
        efforts: &[(u64, f64)],
        payload_forces: &[(u64, [f64; 3])],
        mut rejected: Option<&mut Vec<MemberSnapshot>>,
    ) -> Result<()> {
        let n = self.spec.assemblies.len();
        if payload_forces.len() > n || efforts.len() > n * 7 {
            return Err(invalid("too many fleet inputs"));
        }
        let mut commands: Vec<Vec<(u64, f64)>> = (0..n).map(|_| Vec::new()).collect();
        let mut seen = BTreeSet::new();
        for &(id, e) in efforts {
            let i = self
                .spec
                .joint(id)
                .ok_or(invalid("unknown fleet torque target"))?;
            if !seen.insert(id) || !(self.pending_joint_effort(id).unwrap_or(0.) + e).is_finite() {
                return Err(invalid("invalid, duplicate or overflowing fleet torque"));
            }
            commands[i].push((id, e));
        }
        let mut forces = vec![[0.; 3]; n];
        for &(id, f) in payload_forces {
            let i = self
                .spec
                .part(id)
                .ok_or(invalid("unknown fleet payload force target"))?;
            if !seen.insert(id) || !f.iter().all(|x| x.is_finite()) {
                return Err(invalid("duplicate or nonfinite fleet payload force"));
            }
            forces[i] = f;
        }
        let retain = self.lane == MemberLane::RetainedPrior
            && rejected.is_some()
            && !self.ledger.standalone();
        self.transact(retain, |p, c| {
            p.advance_cut(c, &commands, &forces, &mut rejected)
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
    fn command(&mut self, id: u64, make: impl Fn() -> Op) -> Result<()> {
        let i = self.spec.joint(id).ok_or(invalid("unknown fleet joint"))?;
        self.transact(false, |p, c| {
            let Reply::State(Piece::Coupled(snap)) =
                p.one(i, c.owners[i], c.tick, c.leases[i], make())?
            else {
                return Err(invalid("invalid fleet joint command receipt"));
            };
            if snap.state.tick != c.tick {
                return Err(invalid("fleet joint command changed time"));
            }
            c.groups[i] = snap;
            boundary("grasp-fleet-command-prepared");
            Ok(())
        })
    }
    pub fn queue_effort(&mut self, id: u64, effort: f64) -> Result<()> {
        if !(self.pending_joint_effort(id).unwrap_or(0.) + effort).is_finite() {
            return Err(invalid("invalid accumulated fleet effort"));
        }
        self.command(id, || Op::Queue(id, effort))
    }
    pub fn set_velocity_motor(&mut self, id: u64, input: Option<JointVelocityMotor>) -> Result<()> {
        let m = input.map(MotorSetting::from);
        if m.is_some_and(|v| !v.valid()) {
            return Err(invalid("invalid fleet motor setting"));
        }
        if let Some(g) = self.spec.joint(id)
            && m.is_some()
            && self.spec.assemblies[g].payload.exact_revolute_locks
            && self.spec.assemblies[g]
                .payload
                .limits
                .iter()
                .any(|l| l.joint_id == id && l.lower == 0. && l.upper == 0.)
        {
            return Err(invalid("fleet motor on an exact lock"));
        }
        self.command(id, || Op::Motor(id, m))
    }
    pub fn transfer(&mut self, model: u64, destination: u8) -> Result<()> {
        let i = self
            .spec
            .group(model)
            .ok_or(invalid("unknown grasp fleet model"))?;
        if destination >= self.width() || destination == self.cut.owners[i] {
            return Err(invalid("invalid fleet migration destination"));
        }
        self.transact(false, |p, c| {
            let generation = c
                .generation
                .checked_add(1)
                .ok_or(invalid("fleet generation exhausted"))?;
            let d = p.stage(i, destination, c.tick, generation, c.groups[i].clone())?;
            boundary("grasp-fleet-destination-staged");
            p.ack(i, c.owners[i], c.tick, c.leases[i], Op::Fence(d))?;
            boundary("grasp-fleet-source-fenced");
            p.ack(i, destination, c.tick, generation, Op::Activate(d))?;
            boundary("grasp-fleet-destination-active");
            c.leases[i] = generation;
            c.owners[i] = destination;
            c.generation = generation;
            Ok(())
        })
    }
    pub fn reset(&mut self) -> Result<()> {
        self.transact(false, |p, c| {
            let generation = c
                .generation
                .checked_add(1)
                .ok_or(invalid("fleet generation exhausted"))?;
            let epoch = c
                .epoch
                .checked_add(1)
                .ok_or(invalid("fleet epoch exhausted"))?;
            let (groups, bounds) = p.spec.cold()?;
            p.spec.check_separation(&bounds, epoch, 0)?;
            let owners = c.placement.owners(groups.len(), c.width);
            let mut new = Vec::new();
            for (i, s) in groups.iter().enumerate() {
                new.push(p.stage(i, owners[i], 0, generation, s.clone())?);
            }
            boundary("grasp-fleet-destination-staged");
            for (i, s) in c.groups.iter().enumerate() {
                p.ack(
                    i,
                    c.owners[i],
                    c.tick,
                    c.leases[i],
                    Op::Fence(Piece::Coupled(s.clone()).digest()),
                )?;
            }
            boundary("grasp-fleet-source-fenced");
            for (i, &d) in new.iter().enumerate() {
                p.ack(i, owners[i], 0, generation, Op::Activate(d))?;
            }
            boundary("grasp-fleet-destination-active");
            c.groups = groups;
            c.owners = owners;
            c.leases.fill(generation);
            c.epoch = epoch;
            c.generation = generation;
            c.tick = 0;
            Ok(())
        })
    }
    pub fn spec(&self) -> &GraspFleetSpec {
        &self.spec
    }
    pub fn owners(&self) -> &[u8] {
        &self.cut.owners
    }
    pub fn width(&self) -> u8 {
        self.cut.width
    }
    pub fn epoch(&self) -> u64 {
        self.cut.epoch
    }
    pub fn tick(&self) -> u64 {
        self.cut.tick
    }
    pub fn generation(&self) -> u64 {
        self.cut.generation
    }
    pub fn placement(&self) -> GraspFleetPlacement {
        self.cut.placement
    }
    pub fn observation(&self, model: u64) -> Option<&FloatingTreeObservation> {
        Some(&self.cut.groups[self.spec.group(model)?].state)
    }
    pub fn payload_state(&self, model: u64) -> Option<&ArticulationLinkState> {
        Some(
            &self.cut.groups[self.spec.group(model)?]
                .image
                .payload()?
                .body,
        )
    }
    pub fn payload_reactions(&self, model: u64) -> Option<&[MovingReaction]> {
        Some(
            &self.cut.groups[self.spec.group(model)?]
                .image
                .payload()?
                .reactions,
        )
    }
    pub fn image(&self, model: u64) -> Option<&[u8]> {
        self.cut.groups[self.spec.group(model)?].image.native()
    }
    fn joint_slot(&self, id: u64) -> Option<(&Snapshot, usize)> {
        let s = self.cut.groups[self.spec.joint(id)?].as_ref();
        let j = s
            .state
            .joints
            .binary_search_by_key(&id, |r| r.joint_id)
            .ok()?;
        Some((s, j))
    }
    pub fn velocity_motor(&self, id: u64) -> Option<JointVelocityMotor> {
        let (s, i) = self.joint_slot(id)?;
        s.image.motors()?[i].map(MotorSetting::native)
    }
    pub fn pending_joint_effort(&self, id: u64) -> Option<f64> {
        let (s, i) = self.joint_slot(id)?;
        s.state.joints[i].pending_effort
    }
    pub fn check_interval(&self, epoch: u64, tick: u64) -> Result<()> {
        crate::articulation::check_interval(self.healthy, epoch, tick, self.epoch(), self.tick())
    }
    pub fn worker_pid(&self, slot: u8) -> Option<u32> {
        self.peers.get(slot as usize)?.as_ref().map(Peer::pid)
    }
    pub fn drop_next_acknowledgement(&mut self, slot: u8) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        self.peer(slot)?.withhold = true;
        Ok(())
    }
    pub fn replace_worker(&mut self, slot: u8) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        if slot >= self.width() {
            return Err(invalid("unknown grasp fleet slot"));
        }
        self.healthy = false;
        self.peers[slot as usize].take();
        self.peers[slot as usize] = Some(self.spawn(slot)?);
        for i in 0..self.spec.assemblies.len() {
            if self.cut.owners[i] == slot {
                self.restore_member(i)?;
            }
        }
        self.healthy = true;
        Ok(())
    }
}
