// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    worker::{Op, Reply, Request, Seed},
    *,
};
use crate::{
    articulation::ledger::Ledger,
    moving::worker::{LIMIT, Peer, codec, encode},
};
use bincode::Options;
use gz_physics::JointVelocityMotor;
use std::{
    fs::File,
    io::{self, Read},
    path::{Path, PathBuf},
};

/// A transactional grasp whose released part can move to a separate process.
/// A step has one durable publication even when both workers contribute.
pub struct SplitGraspPool {
    ledger: Ledger,
    exe: PathBuf,
    spec: GraspTreeSpec,
    cut: Cut,
    slots: Vec<Option<Peer>>,
    healthy: bool,
    empty: Vec<MovingReaction>,
}
impl SplitGraspPool {
    fn ledger(dir: &Path, create: bool) -> Result<Ledger> {
        let ledger = Ledger::lock(dir, create)?;
        if ledger.other_archive_exists(FILE)? {
            return Err(invalid("directory belongs to another physics profile"));
        }
        Ok(ledger)
    }
    fn read(l: &Ledger) -> Result<(GraspTreeSpec, Cut)> {
        let mut buf = Vec::new();
        File::open(l.archive_path(FILE))?
            .take((LIMIT + 41) as u64)
            .read_to_end(&mut buf)?;
        if !(40..=LIMIT + 40).contains(&buf.len())
            || buf[..8] != ARCHIVE
            || buf[8..40] != Sha256::digest(&buf[40..])[..]
        {
            return Err(invalid(
                "invalid selective grasp barrier format or checksum",
            ));
        }
        Ok(codec().deserialize(&buf[40..]).map_err(io::Error::other)?)
    }
    pub fn create(
        exe: &Path,
        dir: &Path,
        width: u8,
        spec: GraspTreeSpec,
        policy: GraspPlacement,
    ) -> Result<Self> {
        let l = Self::ledger(dir, true)?;
        if l.archive_path(FILE).try_exists()? {
            return Err(invalid("selective grasp session exists"));
        }
        Self::fresh(exe, l, width, spec, policy)
    }
    pub fn resume(exe: &Path, dir: &Path) -> Result<Self> {
        let l = Self::ledger(dir, false)?;
        let (spec, cut) = Self::read(&l)?;
        Self::open(exe, l, spec, cut)
    }
    pub fn open_scene(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: GraspTreeSpec,
        policy: GraspPlacement,
    ) -> Result<Self> {
        let l = Self::ledger(dir, true)?;
        if !l.archive_path(FILE).try_exists()? {
            return Self::fresh(exe, l, width.unwrap_or(2), spec, policy);
        }
        let (saved, cut) = Self::read(&l)?;
        if spec.digest() != saved.digest()
            || policy != cut.policy
            || width.is_some_and(|w| w != cut.width)
        {
            return Err(invalid(
                "selective grasp construction, width or policy differs from archive",
            ));
        }
        Self::open(exe, l, saved, cut)
    }
    fn fresh(
        exe: &Path,
        l: Ledger,
        width: u8,
        spec: GraspTreeSpec,
        policy: GraspPlacement,
    ) -> Result<Self> {
        let cut = Cut {
            width,
            owners: [0; 2],
            epoch: 0,
            generation: 0,
            policy,
            streak: 0,
            stats: Default::default(),
            mode: Mode::Coupled(Box::new(TreeWorld::new(&spec)?.capture()?)),
        };
        let p = Self::open(exe, l, spec, cut)?;
        p.publish(&p.cut)?;
        Ok(p)
    }
    fn open(exe: &Path, ledger: Ledger, spec: GraspTreeSpec, cut: Cut) -> Result<Self> {
        spec.validate()?;
        cut.validate(&spec)?;
        let mut p = Self {
            exe: exe.canonicalize()?,
            ledger,
            empty: vec![Default::default(); spec.payload.tree.links.len()],
            spec,
            cut,
            slots: vec![],
            healthy: false,
        };
        p.recover()?;
        Ok(p)
    }
    fn spawn(&self, slot: u8) -> Result<Peer> {
        Peer::new_profile(
            &self.exe,
            slot,
            &self.ledger.slot(slot),
            &self.spec,
            FLAG,
            WIRE,
        )
    }
    fn recover(&mut self) -> Result<()> {
        self.healthy = false;
        self.slots.clear();
        for i in 0..self.cut.width {
            let p = self.spawn(i)?;
            self.slots.push(Some(p));
        }
        let tick = self.tick();
        let gen0 = self.generation();
        for (slot, piece) in self.cut.mode.pieces(self.owners()) {
            let d = piece.digest();
            let resp = self.state_call(slot, tick, gen0, Op::Stage(Seed::Install(piece)))?;
            if resp.digest() != d {
                return Err(invalid("recovered grasp image differs from archive"));
            }
            self.ack(slot, tick, gen0, Op::Activate(d))?;
        }
        self.healthy = true;
        Ok(())
    }
    fn publish(&self, c: &Cut) -> io::Result<()> {
        let bytes = encode(&(&self.spec, c))?;
        self.ledger
            .publish_bytes(&self.ledger.archive_path(FILE), &ARCHIVE, &bytes, boundary)
    }
    fn transact(
        &mut self,
        mut prepare: impl FnMut(&mut Self, &mut Cut) -> Result<()>,
    ) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        let mut retry = true;
        loop {
            let mut c = self.cut.clone();
            if let Err(e) = prepare(self, &mut c).and_then(|()| c.validate(&self.spec)) {
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
    fn peer(&mut self, slot: u8) -> Result<&mut Peer> {
        self.slots
            .get_mut(slot as usize)
            .and_then(Option::as_mut)
            .ok_or(invalid("unknown or fenced grasp worker"))
    }
    fn call(&mut self, slot: u8, tick: u64, generation: u64, op: Op) -> Result<Reply> {
        self.peer(slot)?
            .call_typed(tick, Request { generation, op })
    }
    fn ack(&mut self, slot: u8, tick: u64, generation: u64, op: Op) -> Result<()> {
        if !matches!(self.call(slot, tick, generation, op)?, Reply::Ack) {
            return Err(invalid("unexpected grasp acknowledgement"));
        }
        Ok(())
    }
    fn state_call(&mut self, slot: u8, tick: u64, generation: u64, op: Op) -> Result<Piece> {
        match self.call(slot, tick, generation, op)? {
            Reply::State(p) => Ok(p),
            _ => Err(invalid("missing grasp role receipt")),
        }
    }
    /// Stage every new role first, fence every former capability, then activate.
    /// Source and target ticks can differ when an unsafe private preview is discarded.
    fn switch(
        &mut self,
        c: &mut Cut,
        sources: Vec<(u8, Piece)>,
        source_tick: u64,
        target_tick: u64,
        targets: Vec<(u8, Seed)>,
    ) -> Result<()> {
        let generation = c
            .generation
            .checked_add(1)
            .ok_or(invalid("grasp generation exhausted"))?;
        let mut ready = Vec::with_capacity(targets.len());
        for (slot, seed) in targets {
            let expected = match &seed {
                Seed::Install(piece) => Some(piece.digest()),
                Seed::Combine(..) => None,
            };
            let piece = self.state_call(slot, target_tick, generation, Op::Stage(seed))?;
            if expected.is_some_and(|d| d != piece.digest()) {
                return Err(invalid("new grasp role changed accepted physical state"));
            }
            ready.push((slot, piece));
        }
        boundary("grasp-destinations-staged");
        for (slot, piece) in sources {
            self.ack(slot, source_tick, c.generation, Op::Fence(piece.digest()))?;
        }
        boundary("grasp-sources-fenced");
        for (slot, piece) in &ready {
            self.ack(*slot, target_tick, generation, Op::Activate(piece.digest()))?;
        }
        boundary("grasp-destinations-active");
        match ready.as_mut_slice() {
            [(a, Piece::Coupled(s))] => {
                c.owners = [*a; 2];
                c.mode = Mode::Coupled(s.clone());
            }
            [(a, Piece::Tree(s)), (b, Piece::Part(p))] if a != b => {
                c.owners = [*a, *b];
                c.mode = Mode::Split {
                    tree: s.clone(),
                    part: *p,
                };
            }
            _ => return Err(invalid("selective grasp prepared incorrect role set")),
        }
        c.generation = generation;
        c.streak = 0;
        Ok(())
    }
    fn coupled_step(&mut self, c: &mut Cut, efforts: &[(u64, f64)], force: [f64; 3]) -> Result<()> {
        let tick = c.tick();
        let owner = c.owners[0];
        let dwell = c.policy.threshold().filter(|_| self.spec.partitionable());
        let Reply::Advanced(clock, Piece::Coupled(next), bounds) = self.call(
            owner,
            tick,
            c.generation,
            Op::Step {
                efforts: efforts.to_vec(),
                force,
                bounds: dwell.is_some(),
            },
        )?
        else {
            return Err(invalid("missing joined grasp step"));
        };
        if clock != next.state.tick
            || next.state.tick
                != tick
                    .checked_add(1)
                    .ok_or(invalid("grasp clock exhausted"))?
        {
            return Err(invalid("joined grasp did not advance once"));
        }
        c.mode = Mode::Coupled(next);
        let Some(dwell) = dwell else {
            return Ok(());
        };
        let n = self.spec.payload.tree.links.len();
        let part = c
            .mode
            .tree()
            .image
            .payload()
            .ok_or(invalid("missing joined payload receipt"))?;
        if !bounds.clear(&bounds, n)
            || part
                .reactions
                .iter()
                .any(|r| *r != MovingReaction::default())
        {
            c.streak = 0;
            return Ok(());
        }
        c.streak = c.streak.saturating_add(1).min(dwell);
        if c.streak < dwell {
            return Ok(());
        }
        let tick = c.tick();
        match self.call(owner, tick, c.generation, Op::Detach)? {
            Reply::Detached(Some(tree)) => {
                let payload = *c.mode.part();
                if tree.state != c.mode.tree().state
                    || tree.image.motors() != c.mode.tree().image.motors()
                {
                    return Err(invalid("detached grasp changed joint state"));
                }
                let src = c.mode.pieces(c.owners);
                let target = vec![
                    (owner, Seed::Install(Piece::Tree(tree))),
                    ((owner + 1) % c.width, Seed::Install(Piece::Part(payload))),
                ];
                self.switch(c, src, tick, tick, target)?;
                c.stats.splits = c
                    .stats
                    .splits
                    .checked_add(1)
                    .ok_or(invalid("grasp splits exhausted"))?;
            }
            Reply::Detached(None) => {}
            _ => return Err(invalid("unexpected native detach reply")),
        }
        Ok(())
    }
    fn advance_cut(&mut self, c: &mut Cut, efforts: &[(u64, f64)], force: [f64; 3]) -> Result<()> {
        if c.mode.joined() {
            return self.coupled_step(c, efforts, force);
        }
        let tick = c.tick();
        let next = tick
            .checked_add(1)
            .ok_or(invalid("grasp clock exhausted"))?;
        let [a, b] = c.owners;
        let tree = self.peer(a)?.submit(
            tick,
            Request {
                generation: c.generation,
                op: Op::Step {
                    efforts: efforts.to_vec(),
                    force: [0.; 3],
                    bounds: true,
                },
            },
        )?;
        let part = self.peer(b)?.submit(
            tick,
            Request {
                generation: c.generation,
                op: Op::Step {
                    efforts: vec![],
                    force,
                    bounds: true,
                },
            },
        )?;
        let Reply::Advanced(tree_tick, Piece::Tree(tree), tree_bounds) =
            self.peer(a)?.collect(tree)?
        else {
            return Err(invalid("missing isolated gripper advance"));
        };
        boundary("grasp-private-tree-advanced");
        let Reply::Advanced(part_tick, Piece::Part(part), part_bounds) =
            self.peer(b)?.collect(part)?
        else {
            return Err(invalid("missing isolated part advance"));
        };
        if tree_tick != next || part_tick != next || tree.state.tick != next {
            return Err(invalid("isolated gripper advanced incorrect clock"));
        }
        let trial = Mode::Split { tree, part };
        if tree_bounds.clear(&part_bounds, self.spec.payload.tree.links.len()) {
            c.mode = trial;
            c.stats.split_steps = c
                .stats
                .split_steps
                .checked_add(1)
                .ok_or(invalid("grasp split step count exhausted"))?;
            return Ok(());
        }
        boundary("grasp-private-preview-rejected");
        let Mode::Split { tree, part } = &c.mode else {
            unreachable!()
        };
        let old_tree = tree.clone();
        let old_part = *part;
        self.switch(
            c,
            trial.pieces([a, b]),
            next,
            tick,
            vec![(a, Seed::Combine(old_tree, old_part))],
        )?;
        c.stats.joins = c
            .stats
            .joins
            .checked_add(1)
            .ok_or(invalid("grasp join count exhausted"))?;
        c.stats.rejected_previews = c
            .stats
            .rejected_previews
            .checked_add(1)
            .ok_or(invalid("grasp preview count exhausted"))?;
        self.coupled_step(c, efforts, force)
    }
    pub fn spec(&self) -> &GraspTreeSpec {
        &self.spec
    }
    pub fn topology(&self) -> &FloatingTreeSpec {
        &self.spec.payload.tree
    }
    pub fn joint_limits(&self) -> &[FloatingTreeJointLimit] {
        &self.spec.payload.limits
    }
    pub fn observation(&self) -> &FloatingTreeObservation {
        &self.cut.mode.tree().state
    }
    pub fn tick(&self) -> u64 {
        self.cut.tick()
    }
    pub fn width(&self) -> u8 {
        self.cut.width
    }
    pub fn epoch(&self) -> u64 {
        self.cut.epoch
    }
    pub fn generation(&self) -> u64 {
        self.cut.generation
    }
    pub fn owners(&self) -> [u8; 2] {
        self.cut.owners
    }
    pub fn placement(&self) -> GraspPlacement {
        self.cut.policy
    }
    pub fn stats(&self) -> PlacementStats {
        self.cut.stats
    }
    pub fn payload_state(&self) -> &ArticulationLinkState {
        self.cut.mode.part()
    }
    pub fn payload_reactions(&self) -> &[MovingReaction] {
        self.cut
            .mode
            .tree()
            .image
            .payload()
            .map_or(&self.empty, |p| &p.reactions)
    }
    /// The coupled graph when joined, or the isolated gripper's native motion when split.
    pub fn image(&self) -> Option<&[u8]> {
        self.cut.mode.tree().image.native()
    }
    pub fn worker_pid(&self, slot: u8) -> Option<u32> {
        self.slots.get(slot as usize)?.as_ref().map(Peer::pid)
    }
    pub fn pending_joint_effort(&self, id: u64) -> Option<f64> {
        self.joint_index(id)
            .and_then(|i| self.observation().joints[i].pending_effort)
    }
    pub fn velocity_motor(&self, id: u64) -> Option<JointVelocityMotor> {
        self.cut.mode.tree().image.motors()?[self.joint_index(id)?].map(MotorSetting::native)
    }
    fn joint_index(&self, id: u64) -> Option<usize> {
        self.observation()
            .joints
            .binary_search_by_key(&id, |j| j.joint_id)
            .ok()
    }
    fn validate_inputs(&self, inputs: &[(u64, f64)]) -> Result<()> {
        validate_efforts(inputs, self.observation().joints.len(), |id| {
            self.joint_index(id)
                .map(|i| (i, self.observation().joints[i].pending_effort))
        })
    }
    pub fn check_interval(&self, epoch: u64, tick: u64) -> Result<()> {
        crate::articulation::check_interval(self.healthy, epoch, tick, self.epoch(), self.tick())
    }
    fn change(&mut self, op: impl Fn() -> Op) -> Result<()> {
        self.transact(|p, c| {
            let reply = p.state_call(c.owners[0], c.tick(), c.generation, op())?;
            match (&mut c.mode, reply) {
                (Mode::Coupled(old), Piece::Coupled(new))
                | (Mode::Split { tree: old, .. }, Piece::Tree(new))
                    if old.state.tick == new.state.tick =>
                {
                    *old = new
                }
                _ => return Err(invalid("incorrect selective grasp command receipt")),
            }
            boundary("grasp-command-prepared");
            Ok(())
        })
    }
    pub fn queue_effort(&mut self, joint: u64, effort: f64) -> Result<()> {
        self.validate_inputs(&[(joint, effort)])?;
        self.change(|| Op::Queue(joint, effort))
    }
    pub fn set_velocity_motor(
        &mut self,
        joint: u64,
        setting: Option<JointVelocityMotor>,
    ) -> Result<()> {
        let motor = setting.map(MotorSetting::from);
        if self.joint_index(joint).is_none()
            || motor.is_some_and(|m| !m.valid())
            || (motor.is_some()
                && self.spec.payload.exact_revolute_locks
                && self
                    .joint_limits()
                    .iter()
                    .any(|s| s.joint_id == joint && s.lower == 0. && s.upper == 0.))
        {
            return Err(invalid("invalid selective grasp motor target"));
        }
        self.change(|| Op::Motor(joint, motor))
    }
    pub fn advance_with_payload_force(
        &mut self,
        efforts: &[(u64, f64)],
        force: [f64; 3],
    ) -> Result<()> {
        self.validate_inputs(efforts)?;
        if !force.iter().all(|x| x.is_finite()) {
            return Err(invalid("nonfinite selective grasp load"));
        }
        self.transact(|p, c| {
            p.advance_cut(c, efforts, force)?;
            boundary("grasp-step-prepared");
            Ok(())
        })
    }
    pub fn advance_with_payload_force_at(
        &mut self,
        epoch: u64,
        tick: u64,
        efforts: &[(u64, f64)],
        force: [f64; 3],
    ) -> Result<()> {
        self.check_interval(epoch, tick)?;
        self.advance_with_payload_force(efforts, force)
    }
    pub fn advance(&mut self) -> Result<()> {
        self.advance_with_efforts(&[])
    }
    pub fn advance_with_effort(&mut self, id: u64, e: f64) -> Result<()> {
        self.advance_with_efforts(&[(id, e)])
    }
    pub fn advance_with_efforts(&mut self, inputs: &[(u64, f64)]) -> Result<()> {
        self.advance_with_payload_force(inputs, [0.; 3])
    }
    pub fn advance_with_effort_at(&mut self, epoch: u64, tick: u64, id: u64, e: f64) -> Result<()> {
        self.advance_with_efforts_at(epoch, tick, &[(id, e)])
    }
    pub fn advance_with_efforts_at(
        &mut self,
        epoch: u64,
        tick: u64,
        inputs: &[(u64, f64)],
    ) -> Result<()> {
        self.advance_with_payload_force_at(epoch, tick, inputs, [0.; 3])
    }
    pub fn transfer(&mut self, dest: u8) -> Result<()> {
        self.migrate(0, dest)
    }
    pub fn transfer_payload(&mut self, dest: u8) -> Result<()> {
        self.migrate(1, dest)
    }
    fn migrate(&mut self, role: usize, dest: u8) -> Result<()> {
        let owners = self.owners();
        if dest >= self.width() || owners.contains(&dest) {
            return Err(invalid("grasp destination must be a distinct free slot"));
        }
        self.transact(|p, c| {
            let mut new = c.owners;
            if c.mode.joined() {
                new = [dest; 2]
            } else {
                new[role] = dest;
            }
            let sources = c.mode.pieces(c.owners);
            let targets = c
                .mode
                .pieces(new)
                .into_iter()
                .map(|(s, p)| (s, Seed::Install(p)))
                .collect();
            let tick = c.tick();
            let streak = c.streak;
            p.switch(c, sources, tick, tick, targets)?;
            c.streak = streak;
            Ok(())
        })
    }
    pub fn drop_next_acknowledgement(&mut self, slot: u8) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        self.peer(slot)?.withhold = true;
        Ok(())
    }
    pub fn replace_worker(&mut self, slot: u8) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        if slot >= self.width() {
            return Err(invalid("unknown selective grasp worker"));
        }
        self.healthy = false;
        self.slots[slot as usize].take();
        self.slots[slot as usize] = Some(self.spawn(slot)?);
        let tick = self.tick();
        let generation = self.generation();
        if let Some((_, piece)) = self
            .cut
            .mode
            .pieces(self.owners())
            .into_iter()
            .find(|(id, _)| *id == slot)
        {
            let digest = piece.digest();
            let staged =
                self.state_call(slot, tick, generation, Op::Stage(Seed::Install(piece)))?;
            if staged.digest() != digest {
                return Err(invalid("replacement grasp worker changed state"));
            }
            self.ack(slot, tick, generation, Op::Activate(digest))?;
        }
        self.healthy = true;
        Ok(())
    }
    pub fn reset(&mut self) -> Result<()> {
        self.transact(|p, c| {
            let fresh = TreeWorld::new(&p.spec)?.capture()?;
            let sources = c.mode.pieces(c.owners);
            let tick = c.tick();
            p.switch(
                c,
                sources,
                tick,
                0,
                vec![(0, Seed::Install(Piece::Coupled(Box::new(fresh))))],
            )?;
            c.epoch = c
                .epoch
                .checked_add(1)
                .ok_or(invalid("grasp epoch exhausted"))?;
            c.stats = PlacementStats::default();
            Ok(())
        })
    }
}
