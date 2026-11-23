// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::articulation::ledger::Ledger;
use crate::moving::worker::Peer;
use crate::paged_native::{self, Answer, Op, Req};
use std::{
    fs::File,
    io::{self, Read},
    path::PathBuf,
};

#[path = "import.rs"]
mod import;
#[path = "local_owner.rs"]
mod local_owner;
pub use local_owner::{ReturnedGroundedRobot, RobotLocalInputs};

#[derive(Clone, Serialize, Deserialize)]
struct Cut {
    #[serde(skip)]
    posture: Option<RobotPostureRecord>,
    width: u8,
    owner: u8,
    epoch: u64,
    generation: u64,
    snapshot: Snapshot,
}
#[derive(Clone, Copy, Default, Debug)]
pub struct RobotGraphTraffic {
    pub uploaded_pages: u64,
    pub downloaded_pages: u64,
    pub packed_uploads: u64,
    pub packed_downloads: u64,
    /// Sum of actual graph payload bytes, excluding the transport envelopes.
    pub uploaded_bytes: u64,
    pub downloaded_bytes: u64,
}

/// Selects the encoding of managed worker traffic. All modes publish identical full archives.
#[derive(Debug, Default, Copy, Clone, Eq, PartialEq)]
pub enum RobotGraphEncoding {
    /// Negotiate packed graphs with current workers and fall back to the original paged protocol.
    #[default]
    Auto,
    FullPages,
    /// Lossless Rust LZ4 block. Incompressible or unusually large results fall back to bounded pages.
    Packed,
}
/// Managed single-owner robot graph with atomic, whole-world transfer of the loaded floor.
pub struct GroundedRobotPool {
    ledger: Option<std::sync::Arc<Ledger>>,
    exe: PathBuf,
    spec: GroundedRobotSpec,
    cut: Cut,
    slots: Vec<Peer>,
    healthy: bool,
    packed: bool,
    controlled: bool,
    traffic: RobotGraphTraffic,
}
fn boundary(_name: &str) {
    #[cfg(test)]
    crate::moving::crash_tests::gate(_name);
}
impl GroundedRobotPool {
    fn ledger(dir: &Path, create: bool) -> Result<Ledger> {
        let l = Ledger::lock(dir, create)?;
        if l.other_archive_exists(FILE)? {
            return Err(invalid("another physics profile owns this session"));
        }
        Ok(l)
    }
    fn read(l: &Ledger) -> Result<(GroundedRobotSpec, Cut)> {
        let mut bytes = Vec::new();
        File::open(l.archive_path(FILE))?
            .take((FILE_LIMIT + 41) as u64)
            .read_to_end(&mut bytes)?;
        if !(40..=FILE_LIMIT + 40).contains(&bytes.len())
            || (bytes[..8] != ARCHIVE && bytes[..8] != CONTROL_ARCHIVE)
            || Sha256::digest(&bytes[40..])[..] != bytes[8..40]
        {
            return Err(invalid(
                "invalid grounded robot archive header, size or checksum",
            ));
        }
        if bytes[..8] == ARCHIVE {
            return Ok(codec(FILE_LIMIT)
                .deserialize(&bytes[40..])
                .map_err(io::Error::other)?);
        }
        let (spec, mut cut, record): (GroundedRobotSpec, Cut, RobotPostureRecord) =
            codec(FILE_LIMIT)
                .deserialize(&bytes[40..])
                .map_err(io::Error::other)?;
        record.program.check(&spec)?;
        let hash = Sha256::digest(record.program.encode()?).into();
        record
            .receipt
            .check(&record.program, hash, record.receipt.start)?;
        if record.epoch != cut.epoch || record.receipt.end != cut.snapshot.state.tick {
            return Err(invalid("saved robot feedback belongs to a different cut"));
        }
        cut.posture = Some(record);
        Ok((spec, cut))
    }
    pub fn create(exe: &Path, dir: &Path, width: u8, spec: GroundedRobotSpec) -> Result<Self> {
        Self::create_with_encoding(exe, dir, width, spec, RobotGraphEncoding::Auto)
    }
    pub fn create_with_encoding(
        exe: &Path,
        dir: &Path,
        width: u8,
        spec: GroundedRobotSpec,
        encoding: RobotGraphEncoding,
    ) -> Result<Self> {
        let l = Self::ledger(dir, true)?;
        if l.archive_path(FILE).try_exists()? {
            return Err(invalid("grounded robot session already exists"));
        }
        Self::fresh(exe, l, width, spec, encoding)
    }
    pub fn resume(exe: &Path, dir: &Path) -> Result<Self> {
        Self::resume_with_encoding(exe, dir, RobotGraphEncoding::Auto)
    }
    pub fn resume_with_encoding(
        exe: &Path,
        dir: &Path,
        encoding: RobotGraphEncoding,
    ) -> Result<Self> {
        let l = Self::ledger(dir, false)?;
        let (s, c) = Self::read(&l)?;
        Self::open(exe, l, s, c, encoding)
    }
    pub fn open_scene(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: GroundedRobotSpec,
    ) -> Result<Self> {
        let l = Self::ledger(dir, true)?;
        if !l.archive_path(FILE).try_exists()? {
            return Self::fresh(exe, l, width.unwrap_or(2), spec, RobotGraphEncoding::Auto);
        }
        let (s, c) = Self::read(&l)?;
        if width.is_some_and(|w| w != c.width)
            || codec(FILE_LIMIT).serialize(&s).map_err(io::Error::other)?
                != codec(FILE_LIMIT)
                    .serialize(&spec)
                    .map_err(io::Error::other)?
        {
            return Err(invalid(
                "requested robot construction or width differs from archive",
            ));
        }
        Self::open(exe, l, s, c, RobotGraphEncoding::Auto)
    }
    fn fresh(
        exe: &Path,
        l: Ledger,
        width: u8,
        spec: GroundedRobotSpec,
        encoding: RobotGraphEncoding,
    ) -> Result<Self> {
        let snapshot = native::RobotWorld::new(&spec, 0, None)?.capture()?;
        let c = Cut {
            posture: None,
            width,
            owner: 0,
            epoch: 0,
            generation: 0,
            snapshot,
        };
        let p = Self::open(exe, l, spec, c, encoding)?;
        p.publish(&p.cut)?;
        Ok(p)
    }
    fn open(
        exe: &Path,
        ledger: Ledger,
        spec: GroundedRobotSpec,
        cut: Cut,
        encoding: RobotGraphEncoding,
    ) -> Result<Self> {
        Self::open_locked(exe, std::sync::Arc::new(ledger), spec, cut, encoding)
    }
    fn open_locked(
        exe: &Path,
        ledger: std::sync::Arc<Ledger>,
        spec: GroundedRobotSpec,
        cut: Cut,
        encoding: RobotGraphEncoding,
    ) -> Result<Self> {
        spec.root()?;
        if !(2..=8).contains(&cut.width) || cut.owner >= cut.width || cut.epoch > cut.generation {
            return Err(invalid("invalid grounded robot authority"));
        }
        cut.snapshot.check(&spec)?;
        let packed = match encoding {
            RobotGraphEncoding::Auto => {
                crate::floating_tree::worker_supports(exe, "--probe-grounded-robot-v2")?
            }
            RobotGraphEncoding::Packed => true,
            RobotGraphEncoding::FullPages => false,
        };
        let controlled =
            packed && crate::floating_tree::worker_supports(exe, "--probe-grounded-robot-v3")?;
        let mut p = Self {
            controlled,
            packed,
            ledger: Some(ledger),
            exe: exe.canonicalize()?,
            spec,
            cut,
            slots: Vec::new(),
            healthy: false,
            traffic: RobotGraphTraffic::default(),
        };
        p.recover()?;
        Ok(p)
    }
    fn owned_ledger(&self) -> &Ledger {
        self.ledger
            .as_deref()
            .expect("managed owner holds the lease until local release")
    }
    fn publish(&self, c: &Cut) -> io::Result<()> {
        let (magic, bytes) = match &c.posture {
            Some(record) => (
                CONTROL_ARCHIVE,
                codec(FILE_LIMIT).serialize(&(&self.spec, c, record)),
            ),
            None => (ARCHIVE, codec(FILE_LIMIT).serialize(&(&self.spec, c))),
        };
        self.owned_ledger().publish_bytes(
            &self.owned_ledger().archive_path(FILE),
            &magic,
            &bytes.map_err(io::Error::other)?,
            boundary,
        )
    }
    fn recover(&mut self) -> Result<()> {
        self.healthy = false;
        self.slots.clear();
        for i in 0..self.cut.width {
            self.slots.push(Peer::new_profile(
                &self.exe,
                i,
                &self.owned_ledger().slot(i),
                &self.spec,
                if self.controlled {
                    CONTROL_FLAG
                } else if self.packed {
                    PACKED_FLAG
                } else {
                    FLAG
                },
                if self.controlled {
                    CONTROL_WIRE
                } else if self.packed {
                    PACKED_WIRE
                } else {
                    WIRE
                },
            )?);
        }
        let c = self.cut.clone();
        let hash = self.stage(c.owner, c.generation, &c.snapshot, false)?;
        self.ack(
            c.owner,
            c.snapshot.state.tick,
            c.generation,
            Op::Activate(hash),
        )?;
        self.healthy = true;
        Ok(())
    }
    fn call(&mut self, slot: u8, tick: u64, generation: u64, op: Op) -> Result<Answer> {
        let peer = self
            .slots
            .get_mut(slot as usize)
            .ok_or(invalid("robot worker unavailable"))?;
        peer.call_typed(tick, Req { generation, op })
    }
    fn ack(&mut self, slot: u8, tick: u64, g: u64, op: Op) -> Result<()> {
        paged_native::acknowledge(self.call(slot, tick, g, op)?)
    }
    fn stage(&mut self, dest: u8, g: u64, snap: &Snapshot, transfer: bool) -> Result<[u8; 32]> {
        let bytes = codec(SNAP_LIMIT)
            .serialize(snap)
            .map_err(io::Error::other)?;
        let mut count = 0;
        let mut packed = 0;
        let mut sent = 0;
        let ans = paged_native::stage_encoded(
            bytes,
            SNAP_LIMIT,
            self.packed,
            |op| self.call(dest, snap.state.tick, g, op),
            |chunk| match chunk {
                paged_native::TransferChunk::Page(n) => {
                    sent += n;
                    count += 1;
                    if transfer {
                        boundary("robot-upload-page");
                        if count == 3 {
                            boundary("robot-upload-third-page");
                        }
                    }
                }
                paged_native::TransferChunk::Packed(n) => {
                    sent += n;
                    packed += 1;
                    if transfer {
                        boundary("robot-packed-upload");
                    }
                }
                paged_native::TransferChunk::Inline(n) => sent += n,
            },
        );
        self.traffic.uploaded_pages = self.traffic.uploaded_pages.saturating_add(count);
        self.traffic.packed_uploads = self.traffic.packed_uploads.saturating_add(packed);
        self.traffic.uploaded_bytes = self.traffic.uploaded_bytes.saturating_add(sent as u64);

        ans
    }
    fn prepare(&mut self, c: &mut Cut, op: Op, advance: u8) -> Result<()> {
        let t = c.snapshot.state.tick;
        let next = t
            .checked_add(u64::from(advance))
            .ok_or(invalid("robot clock exhausted"))?;
        let a = self.call(c.owner, t, c.generation, op)?;
        let mut count = 0;
        let mut packed = 0;
        let mut received = 0;
        let bytes = paged_native::receive_encoded(
            a,
            SNAP_LIMIT,
            self.packed,
            |op| self.call(c.owner, next, c.generation, op),
            |chunk| match chunk {
                paged_native::TransferChunk::Page(n) => {
                    received += n;
                    count += 1;
                    boundary("robot-receipt-page");
                    if count == 3 {
                        boundary("robot-receipt-third-page");
                    }
                }
                paged_native::TransferChunk::Packed(n) => {
                    received += n;
                    packed += 1;
                    boundary("robot-packed-receipt");
                }
                paged_native::TransferChunk::Inline(n) => received += n,
            },
        );
        self.traffic.downloaded_pages = self.traffic.downloaded_pages.saturating_add(count);
        self.traffic.packed_downloads = self.traffic.packed_downloads.saturating_add(packed);
        self.traffic.downloaded_bytes = self
            .traffic
            .downloaded_bytes
            .saturating_add(received as u64);
        let snap: Snapshot = codec(SNAP_LIMIT)
            .deserialize(&bytes?)
            .map_err(io::Error::other)?;
        if snap.state.tick != next {
            return Err(invalid("robot receipt clock differs"));
        }
        snap.check(&self.spec)?;
        c.snapshot = snap;
        if advance > 0 {
            c.posture = None;
        }
        boundary("robot-receipt-verified");
        Ok(())
    }
    fn transact(&mut self, mut trial: impl FnMut(&mut Self, &mut Cut) -> Result<()>) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        let mut retry = true;
        loop {
            let mut c = self.cut.clone();
            if let Err(e) = trial(self, &mut c) {
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
            boundary("after-sync");
            self.cut = c;
            return Ok(());
        }
    }
    pub fn check_interval(&self, epoch: u64, tick: u64) -> Result<()> {
        crate::articulation::check_interval(self.healthy, epoch, tick, self.epoch(), self.tick())
    }
    fn validate_joint(&self, id: u64, e: f64) -> Result<()> {
        let joints = &self.cut.snapshot.state.joints;
        let i = joints
            .binary_search_by_key(&id, |j| j.joint_id)
            .map_err(|_| invalid("unknown robot joint ID"))?;
        if !(joints[i].pending_effort.unwrap_or(0.) + e).is_finite() {
            return Err(invalid("invalid or overflowing robot torque"));
        }
        Ok(())
    }
    pub fn queue_effort(&mut self, id: u64, effort: f64) -> Result<()> {
        self.validate_joint(id, effort)?;
        self.transact(|p, c| p.prepare(c, Op::Queue(id, effort), 0))
    }
    pub fn advance_with_inputs_at(
        &mut self,
        epoch: u64,
        tick: u64,
        efforts: &[(u64, f64)],
        forces: &[(u64, [f64; 3])],
    ) -> Result<()> {
        self.check_interval(epoch, tick)?;
        if efforts.len() > self.spec.joints.len() || forces.len() > self.spec.links.len() {
            return Err(invalid("too many robot inputs"));
        }
        let mut seen = BTreeSet::new();
        for &(id, e) in efforts {
            self.validate_joint(id, e)?;
            if !seen.insert(id) {
                return Err(invalid("duplicate robot joint input"));
            }
        }
        for &(id, f) in forces {
            if self.spec.links.binary_search_by_key(&id, |l| l.id).is_err()
                || !seen.insert(id)
                || !f.iter().all(|x| x.is_finite())
            {
                return Err(invalid("unknown, repeated or nonfinite robot link force"));
            }
        }
        self.transact(|p, c| {
            p.prepare(
                c,
                Op::Step {
                    efforts: efforts.to_vec(),
                    forces: forces.to_vec(),
                },
                1,
            )
        })
    }
    pub fn advance(&mut self) -> Result<()> {
        self.advance_with_inputs_at(self.epoch(), self.tick(), &[], &[])
    }
    fn switch(&mut self, c: &mut Cut, dest: u8, snap: Snapshot) -> Result<()> {
        let g = c
            .generation
            .checked_add(1)
            .ok_or(invalid("robot generation exhausted"))?;
        let hash = self.stage(dest, g, &snap, true)?;
        boundary("robot-destination-staged");
        let old = Sha256::digest(
            codec(SNAP_LIMIT)
                .serialize(&c.snapshot)
                .map_err(io::Error::other)?,
        )
        .into();
        self.ack(c.owner, c.snapshot.state.tick, c.generation, Op::Fence(old))?;
        boundary("robot-source-fenced");
        self.ack(dest, snap.state.tick, g, Op::Activate(hash))?;
        boundary("robot-destination-active");
        c.owner = dest;
        c.generation = g;
        c.snapshot = snap;
        Ok(())
    }
    pub fn transfer(&mut self, dest: u8) -> Result<()> {
        if dest >= self.width() || dest == self.owner() {
            return Err(invalid("invalid robot destination"));
        }
        self.transact(|p, c| p.switch(c, dest, c.snapshot.clone()))
    }
    pub fn reset(&mut self) -> Result<()> {
        self.transact(|p, c| {
            let snap = native::RobotWorld::new(&p.spec, 0, None)?.capture()?;
            p.switch(c, 0, snap)?;
            c.posture = None;
            c.epoch = c
                .epoch
                .checked_add(1)
                .ok_or(invalid("robot epoch exhausted"))?;
            Ok(())
        })
    }
    /// Evaluate the supplied feedback law at every 1 ms native interval and publish once at the end.
    /// A failed process or rejected interval leaves the entire original accepted cut available.
    pub fn advance_posture_batch_at(
        &mut self,
        epoch: u64,
        tick: u64,
        program: &RobotPostureProgram,
    ) -> Result<&RobotPostureRecord> {
        self.check_interval(epoch, tick)?;
        if !self.controlled {
            return Err(invalid(
                "worker does not support local robot posture; select a capable packed worker",
            ));
        }
        program.check(&self.spec)?;
        let count = program.intervals.len() as u8;
        tick.checked_add(count.into())
            .ok_or(invalid("robot posture batch clock exhausted"))?;
        let bytes = program.encode()?;
        let digest = Sha256::digest(&bytes).into();
        self.transact(|p, c| {
            p.prepare(
                c,
                Op::PostureBatch {
                    count,
                    program: bytes.clone(),
                },
                count,
            )?;
            boundary("robot-posture-final-solver-cut");
            let Answer::Posture(rows) = p.call(
                c.owner,
                c.snapshot.state.tick,
                c.generation,
                Op::ReadPosture(digest),
            )?
            else {
                return Err(invalid("missing robot posture feedback proof"));
            };
            let receipt: RobotPostureReceipt = codec(posture::PROGRAM_LIMIT)
                .deserialize(&rows)
                .map_err(io::Error::other)?;
            receipt.check(program, digest, tick)?;
            c.posture = Some(RobotPostureRecord {
                epoch,
                program: program.clone(),
                receipt,
            });
            boundary("robot-posture-proof-verified");
            Ok(())
        })?;
        Ok(self.cut.posture.as_ref().expect("just accepted"))
    }
    /// Final pose, solved floor patches and per-interval feedback are jointly durable at the batch endpoint.
    pub fn last_posture_batch(&self) -> Option<&RobotPostureRecord> {
        self.cut.posture.as_ref()
    }
    pub fn observation(&self) -> &FloatingTreeObservation {
        &self.cut.snapshot.state
    }
    pub fn ground_patches(&self) -> &[RobotGroundPatch] {
        &self.cut.snapshot.patches
    }
    pub fn ground_reactions(&self) -> &[GroundReaction] {
        &self.cut.snapshot.reactions
    }
    pub fn native_image(&self) -> Option<&[u8]> {
        self.cut.snapshot.image.as_deref()
    }
    pub fn spec(&self) -> &GroundedRobotSpec {
        &self.spec
    }
    pub fn tick(&self) -> u64 {
        self.cut.snapshot.state.tick
    }
    pub fn epoch(&self) -> u64 {
        self.cut.epoch
    }
    pub fn generation(&self) -> u64 {
        self.cut.generation
    }
    pub fn owner(&self) -> u8 {
        self.cut.owner
    }
    pub fn width(&self) -> u8 {
        self.cut.width
    }
    pub fn encoding(&self) -> RobotGraphEncoding {
        if self.packed {
            RobotGraphEncoding::Packed
        } else {
            RobotGraphEncoding::FullPages
        }
    }
    pub fn traffic(&self) -> RobotGraphTraffic {
        self.traffic
    }
    pub fn worker_pid(&self, slot: u8) -> Option<u32> {
        self.slots.get(slot as usize).map(Peer::pid)
    }
    pub fn drop_next_acknowledgement(&mut self, slot: u8) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        self.slots
            .get_mut(slot as usize)
            .ok_or(invalid("invalid robot slot"))?
            .withhold = true;
        Ok(())
    }
}
