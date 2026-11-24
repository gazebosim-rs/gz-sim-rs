// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    native::ClusterWorld,
    worker::{Answer, Op, Req},
    *,
};
use crate::floating_tree::fleet_session::FleetSession;
use crate::{articulation::ledger::Ledger, moving::worker::Peer};
use gz_physics::JointVelocityMotor;
use std::sync::Arc;
use std::{
    fs::File,
    io::{self, Read},
    path::{Path, PathBuf},
};
#[derive(Clone, Serialize, Deserialize)]
pub(in crate::floating_tree) struct Cut {
    pub(in crate::floating_tree) width: u8,
    pub(in crate::floating_tree) owners: [u8; 2],
    pub(in crate::floating_tree) epoch: u64,
    pub(in crate::floating_tree) generation: u64,
    pub(in crate::floating_tree) snapshot: ClusterSnapshot,
}
impl Cut {
    fn validate(&self, s: &GraspFleetSpec, version: Version) -> Result<()> {
        if !(2..=8).contains(&self.width)
            || self.owners[0] >= self.width
            || self.owners[0] != self.owners[1]
            || self.epoch > self.generation
        {
            return Err(invalid("invalid coupled grasp fleet authority"));
        }
        self.snapshot.check(s, version)
    }
}
/// Diagnostic counts of bounded native graph pages sent through the managed worker protocol.
#[derive(Default, Clone, Copy, Debug)]
pub struct GraspGraphTraffic {
    pub uploaded_pages: u64,
    pub downloaded_pages: u64,
}
pub struct CoupledGraspFleetPool {
    ledger: FleetSession,
    version: Version,
    exe: PathBuf,
    spec: GraspFleetSpec,
    cut: Cut,
    slots: Vec<Option<Peer>>,
    healthy: bool,
    traffic: GraspGraphTraffic,
}
fn boundary(_edge: &str) {
    #[cfg(test)]
    crate::moving::crash_tests::gate(_edge);
}
impl CoupledGraspFleetPool {
    pub(in crate::floating_tree) fn committed_cut(&self) -> &Cut {
        &self.cut
    }
    pub(in crate::floating_tree) fn resume_contained(
        exe: &Path,
        l: Arc<Ledger>,
        s: GraspFleetSpec,
        c: Cut,
    ) -> Result<Self> {
        Self::open(
            exe,
            FleetSession::Contained(l),
            s,
            c,
            Version::ObservedShear,
        )
    }
    pub(in crate::floating_tree) fn cold_contained(
        exe: &Path,
        l: Arc<Ledger>,
        width: u8,
        s: GraspFleetSpec,
    ) -> Result<Self> {
        Self::fresh(exe, FleetSession::Contained(l), width, s)
    }
    pub(in crate::floating_tree) fn import_contained(
        exe: &Path,
        l: Arc<Ledger>,
        s: GraspFleetSpec,
        prior: &crate::floating_tree::placement::fleet::FleetCut,
        members: [&[u8]; 2],
        warm: bool,
    ) -> Result<Self> {
        let snapshot = ClusterWorld::restore_members(&s, prior, members, warm)?.capture()?;
        Self::from_verified_snapshot(exe, l, s, prior, snapshot)
    }
    pub(in crate::floating_tree) fn from_verified_snapshot(
        exe: &Path,
        l: Arc<Ledger>,
        s: GraspFleetSpec,
        prior: &crate::floating_tree::placement::fleet::FleetCut,
        snapshot: ClusterSnapshot,
    ) -> Result<Self> {
        let generation = prior
            .generation
            .checked_add(1)
            .ok_or(invalid("promotion generation exhausted"))?;
        let c = Cut {
            width: prior.width,
            owners: [prior.owners[0]; 2],
            epoch: prior.epoch,
            generation,
            snapshot,
        };
        Self::resume_contained(exe, l, s, c)
    }
    fn ledger(dir: &Path, create: bool) -> Result<Ledger> {
        let l = Ledger::lock(dir, create)?;
        if l.other_archive_exists(FILE)? {
            return Err(invalid("session belongs to a different grasp profile"));
        }
        Ok(l)
    }
    fn read(l: &Ledger) -> Result<(GraspFleetSpec, Cut, Version)> {
        let mut b = Vec::new();
        File::open(l.archive_path(FILE))?
            .take((FILE_LIMIT + 41) as u64)
            .read_to_end(&mut b)?;
        if !(40..=FILE_LIMIT + 40).contains(&b.len()) || Sha256::digest(&b[40..])[..] != b[8..40] {
            return Err(invalid(
                "invalid coupled grasp fleet archive size, type or checksum",
            ));
        }
        let version = [Version::Legacy, Version::ObservedShear]
            .into_iter()
            .find(|v| b[..8] == v.archive())
            .ok_or(invalid("unsupported coupled fleet archive version"))?;
        let (spec, cut) = codec(FILE_LIMIT)
            .deserialize(&b[40..])
            .map_err(io::Error::other)?;
        Ok((spec, cut, version))
    }
    pub fn create(exe: &Path, dir: &Path, width: u8, s: GraspFleetSpec) -> Result<Self> {
        let l = Self::ledger(dir, true)?;
        if l.archive_path(FILE).try_exists()? {
            return Err(invalid("coupled grasp fleet session already exists"));
        }
        Self::fresh(exe, l.into(), width, s)
    }
    pub fn resume(exe: &Path, dir: &Path) -> Result<Self> {
        let l = Self::ledger(dir, false)?;
        let (s, c, v) = Self::read(&l)?;
        Self::open(exe, l.into(), s, c, v)
    }
    pub fn open_scene(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        s: GraspFleetSpec,
    ) -> Result<Self> {
        let l = Self::ledger(dir, true)?;
        if !l.archive_path(FILE).try_exists()? {
            return Self::fresh(exe, l.into(), width.unwrap_or(2), s);
        }
        let (old, c, v) = Self::read(&l)?;
        if width.is_some_and(|w| w != c.width)
            || s.assemblies.len() != old.assemblies.len()
            || !s
                .assemblies
                .iter()
                .zip(&old.assemblies)
                .all(|(a, b)| a.digest() == b.digest())
        {
            return Err(invalid(
                "coupled grasp fleet construction differs from archive",
            ));
        }
        Self::open(exe, l.into(), old, c, v)
    }
    fn fresh(exe: &Path, l: FleetSession, width: u8, s: GraspFleetSpec) -> Result<Self> {
        let cut = Cut {
            width,
            owners: [0; 2],
            epoch: 0,
            generation: 0,
            snapshot: ClusterWorld::new(&s, Version::ObservedShear)?.capture()?,
        };
        let p = Self::open(exe, l, s, cut, Version::ObservedShear)?;
        p.publish(&p.cut)?;
        Ok(p)
    }
    fn open(
        exe: &Path,
        ledger: FleetSession,
        spec: GraspFleetSpec,
        cut: Cut,
        version: Version,
    ) -> Result<Self> {
        admissible(&spec)?;
        cut.validate(&spec, version)?;
        let mut p = Self {
            ledger,
            version,
            exe: exe.canonicalize()?,
            spec,
            cut,
            slots: Vec::new(),
            healthy: false,
            traffic: Default::default(),
        };
        p.recover()?;
        Ok(p)
    }
    fn publish(&self, c: &Cut) -> io::Result<()> {
        if !self.ledger.standalone() {
            return Ok(());
        }
        let b = codec(FILE_LIMIT)
            .serialize(&(&self.spec, c))
            .map_err(io::Error::other)?;
        self.ledger.publish_bytes(
            &self.ledger.archive_path(FILE),
            &self.version.archive(),
            &b,
            boundary,
        )
    }
    fn spawn(&self, i: u8) -> Result<Peer> {
        Peer::new_profile(
            &self.exe,
            i,
            &self.ledger.slot(i),
            &self.spec,
            self.version.flag(),
            self.version.wire(),
        )
    }
    fn peer(&mut self, i: u8) -> Result<&mut Peer> {
        self.slots
            .get_mut(i as usize)
            .and_then(Option::as_mut)
            .ok_or(invalid("missing coupled fleet process"))
    }
    fn recover(&mut self) -> Result<()> {
        self.healthy = false;
        self.slots.clear();
        for i in 0..self.width() {
            let p = self.spawn(i)?;
            self.slots.push(Some(p));
        }
        let c = self.cut.clone();
        self.install(&c)?;
        self.healthy = true;
        Ok(())
    }
    fn install(&mut self, c: &Cut) -> Result<()> {
        let slot = c.owners[0];
        let d = self.stage(slot, c.generation, &c.snapshot)?;
        self.ack(slot, c.snapshot.tick, c.generation, Op::Activate(d))
    }
    fn call(&mut self, i: u8, t: u64, generation: u64, op: Op) -> Result<Answer> {
        self.peer(i)?.call_typed(t, Req { generation, op })
    }
    fn ack(&mut self, i: u8, t: u64, g: u64, op: Op) -> Result<()> {
        if !matches!(self.call(i, t, g, op)?, Answer::Ack) {
            return Err(invalid("missing coupled fleet acknowledgement"));
        }
        Ok(())
    }
    fn encoded(s: &ClusterSnapshot) -> Result<Vec<u8>> {
        Ok(codec(SNAP_LIMIT).serialize(s).map_err(io::Error::other)?)
    }
    fn stage(&mut self, slot: u8, generation: u64, snap: &ClusterSnapshot) -> Result<[u8; 32]> {
        let bytes = Self::encoded(snap)?;
        let transfer = generation != self.cut.generation;
        let mut pages = 0;
        let result = crate::paged_native::stage(
            bytes,
            SNAP_LIMIT,
            |op| self.call(slot, snap.tick, generation, op),
            || {
                pages += 1;
                if transfer {
                    boundary("cluster-transfer-upload-page");
                }
            },
        );
        self.traffic.uploaded_pages = self.traffic.uploaded_pages.saturating_add(pages);
        result
    }
    fn next_snapshot(&mut self, c: &Cut, op: Op, tick: u64) -> Result<ClusterSnapshot> {
        let slot = c.owners[0];
        let generation = c.generation;
        let answer = self.call(slot, c.snapshot.tick, generation, op)?;
        let mut pages = 0;
        let bytes = crate::paged_native::receive(
            answer,
            SNAP_LIMIT,
            |op| self.call(slot, tick, generation, op),
            || {
                pages += 1;
                boundary("cluster-receipt-page");
            },
        );
        self.traffic.downloaded_pages = self.traffic.downloaded_pages.saturating_add(pages);
        let bytes = bytes?;
        let snap: ClusterSnapshot = codec(SNAP_LIMIT)
            .deserialize(&bytes)
            .map_err(io::Error::other)?;
        if snap.tick != tick {
            return Err(invalid("incorrect coupled fleet worker clock"));
        }
        Ok(snap)
    }
    fn transact(
        &mut self,
        mut prepare: impl FnMut(&mut Self, &mut Cut) -> Result<()>,
    ) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        let mut retry = true;
        loop {
            let mut c = self.cut.clone();
            if let Err(e) =
                prepare(self, &mut c).and_then(|()| c.validate(&self.spec, self.version))
            {
                self.recover()?;
                if retry && matches!(e, ArticulationError::Io(_)) {
                    retry = false;
                    continue;
                }
                return Err(e);
            }
            if let Err(e) = self.publish(&c) {
                self.healthy = false;
                self.slots.clear();
                return Err(ArticulationError::Publication(e));
            }
            if self.ledger.standalone() {
                boundary("after-sync");
            }
            self.cut = c;
            return Ok(());
        }
    }
    fn joint(&self, id: u64) -> Option<(usize, usize)> {
        self.cut
            .snapshot
            .members
            .iter()
            .enumerate()
            .find_map(|(g, m)| {
                m.tree
                    .joints
                    .binary_search_by_key(&id, |j| j.joint_id)
                    .ok()
                    .map(|i| (g, i))
            })
    }
    pub fn queue_effort(&mut self, id: u64, e: f64) -> Result<()> {
        if self.joint(id).is_none()
            || !(self.pending_joint_effort(id).unwrap_or(0.) + e).is_finite()
        {
            return Err(invalid("unknown or invalid coupled fleet torque"));
        }
        self.transact(|p, c| {
            c.snapshot = p.next_snapshot(c, Op::Queue(id, e), c.snapshot.tick)?;
            boundary("cluster-command-prepared");
            Ok(())
        })
    }
    pub fn set_velocity_motor(
        &mut self,
        id: u64,
        setting: Option<JointVelocityMotor>,
    ) -> Result<()> {
        let (g, _) = self
            .joint(id)
            .ok_or(invalid("unknown coupled fleet motor joint"))?;
        let s = &self.spec.assemblies[g].payload;
        let m = setting.map(MotorSetting::from);
        if m.is_some_and(|v| !v.valid())
            || (m.is_some()
                && s.exact_revolute_locks
                && s.limits
                    .iter()
                    .any(|v| v.joint_id == id && v.lower == 0. && v.upper == 0.))
        {
            return Err(invalid("invalid coupled fleet motor or exact lock"));
        }
        self.transact(|p, c| {
            c.snapshot = p.next_snapshot(c, Op::Motor(id, m), c.snapshot.tick)?;
            boundary("cluster-command-prepared");
            Ok(())
        })
    }
    pub fn advance_with_inputs(
        &mut self,
        efforts: &[(u64, f64)],
        forces: &[(u64, [f64; 3])],
    ) -> Result<()> {
        if efforts.len() > 14 || forces.len() > 2 {
            return Err(invalid("too many coupled fleet inputs"));
        }
        let mut seen = BTreeSet::new();
        for &(id, e) in efforts {
            if !seen.insert(id)
                || self.joint(id).is_none()
                || !(self.pending_joint_effort(id).unwrap_or(0.) + e).is_finite()
            {
                return Err(invalid(
                    "invalid, duplicate or overflowing coupled fleet torque",
                ));
            }
        }
        for &(id, f) in forces {
            if !seen.insert(id)
                || !self
                    .spec
                    .assemblies
                    .iter()
                    .any(|g| g.payload.payload.id == id)
                || !f.iter().all(|v| v.is_finite())
            {
                return Err(invalid("invalid or duplicate coupled fleet load"));
            }
        }
        self.transact(|p, c| {
            let t = c
                .snapshot
                .tick
                .checked_add(1)
                .ok_or(invalid("coupled fleet clock exhausted"))?;
            c.snapshot = p.next_snapshot(
                c,
                Op::Step {
                    efforts: efforts.to_vec(),
                    forces: forces.to_vec(),
                },
                t,
            )?;
            boundary("cluster-step-prepared");
            Ok(())
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
    fn switch(&mut self, c: &mut Cut, dest: u8, next: ClusterSnapshot) -> Result<()> {
        let generation = c
            .generation
            .checked_add(1)
            .ok_or(invalid("coupled fleet generation exhausted"))?;
        let d = self.stage(dest, generation, &next)?;
        boundary("cluster-destination-staged");
        let old = Sha256::digest(Self::encoded(&c.snapshot)?).into();
        self.ack(c.owners[0], c.snapshot.tick, c.generation, Op::Fence(old))?;
        boundary("cluster-source-fenced");
        self.ack(dest, next.tick, generation, Op::Activate(d))?;
        boundary("cluster-destination-active");
        c.snapshot = next;
        c.owners = [dest; 2];
        c.generation = generation;
        Ok(())
    }
    pub fn transfer(&mut self, dest: u8) -> Result<()> {
        if dest >= self.width() || dest == self.owner() {
            return Err(invalid("invalid coupled fleet destination"));
        }
        self.transact(|p, c| p.switch(c, dest, c.snapshot.clone()))
    }
    pub fn reset(&mut self) -> Result<()> {
        self.transact(|p, c| {
            let fresh = ClusterWorld::new(&p.spec, p.version)?.capture()?;
            p.switch(c, 0, fresh)?;
            c.epoch = c
                .epoch
                .checked_add(1)
                .ok_or(invalid("coupled fleet epoch exhausted"))?;
            Ok(())
        })
    }
    pub fn spec(&self) -> &GraspFleetSpec {
        &self.spec
    }
    pub fn tick(&self) -> u64 {
        self.cut.snapshot.tick
    }
    pub fn epoch(&self) -> u64 {
        self.cut.epoch
    }
    pub fn owner(&self) -> u8 {
        self.cut.owners[0]
    }
    pub fn owners(&self) -> &[u8] {
        &self.cut.owners
    }
    pub fn width(&self) -> u8 {
        self.cut.width
    }
    pub fn generation(&self) -> u64 {
        self.cut.generation
    }
    pub fn traffic(&self) -> GraspGraphTraffic {
        self.traffic
    }
    pub fn image(&self) -> Option<&[u8]> {
        self.cut.snapshot.image.as_deref()
    }
    pub fn cross_reactions(&self) -> &[GraspFleetPairReaction] {
        &self.cut.snapshot.pairs
    }
    fn member(&self, model: u64) -> Option<&Member> {
        let i = self
            .spec
            .assemblies
            .binary_search_by_key(&model, |s| s.payload.tree.model_id)
            .ok()?;
        self.cut.snapshot.members.get(i)
    }
    pub fn observation(&self, model: u64) -> Option<&FloatingTreeObservation> {
        Some(&self.member(model)?.tree)
    }
    pub fn payload_reactions(&self, model: u64) -> Option<&[MovingReaction]> {
        Some(&self.member(model)?.part.reactions)
    }
    pub fn payload_state(&self, model: u64) -> Option<&ArticulationLinkState> {
        Some(&self.member(model)?.part.body)
    }
    pub fn pending_joint_effort(&self, id: u64) -> Option<f64> {
        let (g, i) = self.joint(id)?;
        self.cut.snapshot.members[g].tree.joints[i].pending_effort
    }
    pub fn velocity_motor(&self, id: u64) -> Option<JointVelocityMotor> {
        let (g, i) = self.joint(id)?;
        self.cut.snapshot.members[g].motors[i].map(MotorSetting::native)
    }
    pub fn worker_pid(&self, id: u8) -> Option<u32> {
        self.slots.get(id as usize)?.as_ref().map(Peer::pid)
    }
    pub fn check_interval(&self, epoch: u64, tick: u64) -> Result<()> {
        crate::articulation::check_interval(self.healthy, epoch, tick, self.epoch(), self.tick())
    }
    pub fn drop_next_acknowledgement(&mut self, id: u8) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        self.peer(id)?.withhold = true;
        Ok(())
    }
    pub fn replace_worker(&mut self, id: u8) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        if id >= self.width() {
            return Err(invalid("unknown coupled fleet slot"));
        }
        self.healthy = false;
        self.slots[id as usize].take();
        self.slots[id as usize] = Some(self.spawn(id)?);
        if id == self.owner() {
            let c = self.cut.clone();
            self.install(&c)?;
        }
        self.healthy = true;
        Ok(())
    }
}
