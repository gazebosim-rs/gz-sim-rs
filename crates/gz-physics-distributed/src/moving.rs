// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Durable local ownership of one floating hinge and an independently moving sphere.
mod ledger;
mod native;
pub(crate) mod routing;
pub(crate) mod worker;
use crate::articulation::{
    ArticulationError, ArticulationLinkSpec, ArticulationLinkState, ArticulationObservation,
    ArticulationSpec, NativeError,
};
pub use gz_physics_multifidelity::MovingReaction;
use ledger::{Cut, Disk};
use native::{HingeSample, Motion, Part, Resident, Seed};
use routing::RoutingBounds;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    io,
    path::{Path, PathBuf},
};
pub use worker::WorkerTransport;
pub use worker::serve;
use worker::{Answer, Op, Peer};

type Result<T> = std::result::Result<T, ArticulationError>;
fn invalid(msg: &'static str) -> ArticulationError {
    ArticulationError::Invalid(msg)
}
impl From<gz_physics::PhysicsError> for ArticulationError {
    fn from(error: gz_physics::PhysicsError) -> Self {
        Self::Native(NativeError::Physics(error))
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MovingSpec {
    pub hinge: ArticulationSpec,
    pub sphere: ArticulationLinkSpec,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct MovingObservation {
    pub hinge: ArticulationObservation,
    pub sphere: ArticulationLinkState,
    /// World-space impulse on each hinge link from the sphere.
    pub reactions: [MovingReaction; 2],
}
impl PartialEq for MovingObservation {
    fn eq(&self, other: &Self) -> bool {
        self.hinge == other.hinge
            && self.sphere == other.sphere
            && self.reactions.iter().zip(&other.reactions).all(|(a, b)| {
                a.active == b.active
                    && a.normal.map(f64::to_bits) == b.normal.map(f64::to_bits)
                    && a.normal_impulse.to_bits() == b.normal_impulse.to_bits()
                    && a.warmstart_normal_impulse.to_bits() == b.warmstart_normal_impulse.to_bits()
                    && a.tangent_impulse.map(f64::to_bits) == b.tangent_impulse.map(f64::to_bits)
            })
    }
}
impl Eq for MovingObservation {}

#[derive(Debug, PartialEq, Eq)]
enum Routing {
    Keep,
    Join,
    CheckRelease,
}

impl Cut {
    fn joined(&self) -> bool {
        self.owners[0] == self.owners[1]
    }
    fn state(&self) -> MovingObservation {
        MovingObservation {
            hinge: ArticulationObservation {
                tick: self.tick,
                joint_id: self.spec.hinge.joint_id,
                links: self.hinge.links,
                q: self.hinge.q,
                qdot: self.hinge.qdot,
                ground_reactions: None,
            },
            sphere: self.sphere,
            reactions: self.reactions,
        }
    }
    fn accept(&mut self, worker: u8, part: Part) -> Result<()> {
        match part {
            Part::Sphere(s) if !self.joined() && worker == self.owners[1] => {
                self.sphere = s;
            }
            Part::Hinge(m, h) if !self.joined() && worker == self.owners[0] => {
                self.motion = m;
                self.hinge = h;
                self.reactions = Default::default();
            }
            Part::Joined(m, h, s, r) if self.joined() && worker == self.owners[0] => {
                self.motion = m;
                self.hinge = *h;
                self.sphere = s;
                self.reactions = r;
            }
            _ => return Err(invalid("unexpected moving role response")),
        }
        if self.sphere.id != self.spec.sphere.id
            || self
                .hinge
                .links
                .iter()
                .zip(self.spec.hinge.links)
                .any(|(s, l)| s.id != l.id)
        {
            return Err(invalid("incorrect moving body identity"));
        }
        Ok(())
    }
    fn new(spec: MovingSpec, width: u8) -> Result<Self> {
        if !(2..=8).contains(&width) {
            return Err(invalid("expected two to eight moving workers"));
        }
        spec.validate()?;
        let origin = spec.initial_sphere();
        let hinge = Resident::load(&spec, 0, &Seed::Hinge(Motion::Cold(None)))?;
        let Part::Hinge(motion, hinge) = hinge.capture(&spec)? else {
            unreachable!()
        };
        let sphere = Resident::load(&spec, 0, &Seed::Sphere(origin))?;
        let Part::Sphere(sphere) = sphere.capture(&spec)? else {
            unreachable!()
        };
        Ok(Self {
            spec,
            width,
            tick: 0,
            generation: 0,
            epoch: 0,
            owners: [0, 1],
            pending_effort: None,
            motion,
            hinge,
            sphere,
            reactions: Default::default(),
        })
    }

    #[cfg(test)]
    fn next_routing(&self) -> Routing {
        RoutingBounds::new(&self.spec).next(self)
    }
}

/// Local transactional simulation. Separate ownership is represented by distinct
/// hinge/sphere slots. Equal owners mean both bodies solve in the same native world.
pub struct MovingPool {
    disk: Disk,
    exe: PathBuf,
    transport: WorkerTransport,
    slots: Vec<Option<Peer>>,
    cut: Cut,
    bounds: RoutingBounds,
    observation: MovingObservation,
    healthy: bool,
}
impl MovingPool {
    pub fn create(exe: &Path, dir: &Path, width: u8, spec: MovingSpec) -> Result<Self> {
        Self::create_with_transport(exe, dir, width, spec, WorkerTransport::Pipe)
    }
    pub fn create_with_transport(
        exe: &Path,
        dir: &Path,
        width: u8,
        spec: MovingSpec,
        transport: WorkerTransport,
    ) -> Result<Self> {
        let disk = Disk::lock(dir, true)?;
        if disk.exists()? {
            return Err(invalid("moving session already exists"));
        }
        Self::open(exe, disk, Cut::new(spec, width)?, true, transport)
    }
    pub fn resume(exe: &Path, dir: &Path) -> Result<Self> {
        Self::resume_with_transport(exe, dir, WorkerTransport::Pipe)
    }
    pub fn resume_with_transport(
        exe: &Path,
        dir: &Path,
        transport: WorkerTransport,
    ) -> Result<Self> {
        let disk = Disk::lock(dir, false)?;
        let cut = disk.read()?;
        Self::open(exe, disk, cut, false, transport)
    }
    pub fn open_scene(exe: &Path, dir: &Path, width: Option<u8>, spec: MovingSpec) -> Result<Self> {
        Self::open_scene_with_transport(exe, dir, width, spec, WorkerTransport::Pipe)
    }
    pub fn open_scene_with_transport(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: MovingSpec,
        transport: WorkerTransport,
    ) -> Result<Self> {
        let disk = Disk::lock(dir, true)?;
        if !disk.exists()? {
            return Self::open(
                exe,
                disk,
                Cut::new(spec, width.unwrap_or(2))?,
                true,
                transport,
            );
        }
        let cut = disk.read()?;
        if width.is_some_and(|w| w != cut.width)
            || serde_json::to_vec(&cut.spec).map_err(io::Error::from)?
                != serde_json::to_vec(&spec).map_err(io::Error::from)?
        {
            return Err(invalid("moving session recipe or width differs from scene"));
        }
        Self::open(exe, disk, cut, false, transport)
    }
    fn open(
        exe: &Path,
        disk: Disk,
        cut: Cut,
        fresh: bool,
        transport: WorkerTransport,
    ) -> Result<Self> {
        let mut pool = Self {
            exe: exe.canonicalize()?,
            transport,
            slots: (0..cut.width).map(|_| None).collect(),
            observation: cut.state(),
            bounds: RoutingBounds::new(&cut.spec),
            cut,
            disk,
            healthy: true,
        };
        for i in 0..pool.width() {
            pool.spawn(i)?
        }
        let mut recovered = pool.cut.clone();
        pool.install_cut(&mut recovered)?;
        pool.check_restored(&recovered)?;
        if fresh {
            pool.disk.publish(&pool.cut)?
        }
        Ok(pool)
    }
    pub fn spec(&self) -> &MovingSpec {
        &self.cut.spec
    }
    pub fn observation(&self) -> &MovingObservation {
        &self.observation
    }
    /// Accumulated torque (N·m) committed for the next step, including a queued zero.
    pub fn pending_joint_effort(&self) -> Option<f64> {
        self.cut.pending_effort
    }
    pub fn width(&self) -> u8 {
        self.cut.width
    }
    pub fn owners(&self) -> [u8; 2] {
        self.cut.owners
    }
    pub fn epoch(&self) -> u64 {
        self.cut.epoch
    }
    pub fn generation(&self) -> u64 {
        self.cut.generation
    }
    pub fn image(&self) -> Option<&[u8]> {
        if self.cut.joined()
            && let Motion::Warm(b) = &self.cut.motion
        {
            Some(b)
        } else {
            None
        }
    }
    pub fn worker_pid(&self, slot: u8) -> Option<u32> {
        Some(self.slots.get(usize::from(slot))?.as_ref()?.pid())
    }
    pub fn worker_steps(&self, slot: u8) -> Option<u64> {
        Some(self.slots.get(usize::from(slot))?.as_ref()?.steps)
    }
    pub fn drop_next_acknowledgement(&mut self, slot: u8) -> Result<()> {
        self.slots
            .get_mut(usize::from(slot))
            .and_then(Option::as_mut)
            .ok_or(invalid("unknown moving worker"))?
            .withhold = true;
        Ok(())
    }
    fn spawn(&mut self, i: u8) -> Result<()> {
        self.slots[i as usize] = Some(Peer::new(
            &self.exe,
            i,
            &self.disk.slot(i),
            &self.cut.spec,
            self.transport,
        )?);
        Ok(())
    }
    fn cmd(&mut self, i: u8, tick: u64, op: Op) -> Result<Answer> {
        let result = self.slots[i as usize]
            .as_mut()
            .ok_or(invalid("moving worker unavailable"))?
            .call(tick, op);
        if result.is_err() {
            self.slots[i as usize].take();
        }
        result
    }
    fn update(&mut self, cut: &mut Cut, i: u8, op: Op) -> Result<()> {
        let Answer::State(part) = self.cmd(i, cut.tick, op)? else {
            return Err(invalid("missing native moving state"));
        };
        cut.accept(i, *part)
    }
    fn install_cut(&mut self, cut: &mut Cut) -> Result<()> {
        let [h, s] = cut.owners;
        if cut.joined() {
            let seed = Seed::Joined(cut.motion.clone());
            self.update(cut, h, Op::Install(Box::new(seed)))?;
        } else {
            let a = Seed::Hinge(cut.motion.clone());
            let b = Seed::Sphere(cut.sphere);
            self.update(cut, h, Op::Install(Box::new(a)))?;
            self.update(cut, s, Op::Install(Box::new(b)))?;
        }
        Ok(())
    }
    fn check_restored(&self, cut: &Cut) -> Result<()> {
        if worker::encode(&cut.motion)? != worker::encode(&self.cut.motion)?
            || cut.state() != self.observation
        {
            return Err(invalid(
                "recovered moving worker state differs from durable barrier",
            ));
        }
        Ok(())
    }

    /// Fence every private role before reinstalling either side of the last
    /// public barrier. Only missing process slots need fresh PIDs.
    fn rollback(&mut self) -> Result<()> {
        self.healthy = false;
        for i in 0..self.width() {
            if self.slots[i as usize].is_some() {
                let _ = self.cmd(i, self.cut.tick, Op::Fence);
            }
        }
        for i in 0..self.width() {
            if self.slots[i as usize].is_none() {
                self.spawn(i)?
            }
        }
        let mut old = self.cut.clone();
        self.install_cut(&mut old)?;
        self.check_restored(&old)?;
        self.healthy = true;
        Ok(())
    }
    fn transact(&mut self, mut op: impl FnMut(&mut Self, &mut Cut) -> Result<()>) -> Result<()> {
        if !self.healthy {
            return Err(invalid("reopen the fenced moving session"));
        }
        let mut retry = true;
        loop {
            let mut next = self.cut.clone();
            if let Err(e) = op(self, &mut next) {
                self.rollback()?;
                if retry && matches!(e, ArticulationError::Io(_)) {
                    retry = false;
                    continue;
                }
                return Err(e);
            }
            if let Err(e) = self.disk.publish(&next) {
                self.slots.clear();
                self.healthy = false;
                return Err(ArticulationError::Publication(e));
            }
            self.observation = next.state();
            #[cfg(test)]
            crash_tests::gate("after-sync");
            self.cut = next;
            return Ok(());
        }
    }
    fn fence(&mut self, i: u8, tick: u64) -> Result<()> {
        match self.cmd(i, tick, Op::Fence)? {
            Answer::Empty => Ok(()),
            _ => Err(invalid("moving fence not acknowledged")),
        }
    }
    fn regroup(&mut self, cut: &mut Cut, owners: [u8; 2], motion: Motion) -> Result<()> {
        let joined_before = cut.joined();
        let [h, s] = cut.owners;
        self.fence(h, cut.tick)?;
        if s != h {
            self.fence(s, cut.tick)?
        }
        #[cfg(test)]
        crash_tests::gate("regroup-fenced");
        let previous = cut.state();
        let handoff = cut.joined() && owners[0] == owners[1];
        cut.owners = owners;
        cut.generation = cut
            .generation
            .checked_add(1)
            .ok_or(invalid("moving generation exhausted"))?;
        cut.motion = motion;
        let previous_image = handoff.then(|| worker::encode(&cut.motion)).transpose()?;
        if cut.joined() && !joined_before {
            let seed = Seed::Combine(cut.motion.clone(), cut.sphere);
            self.update(cut, owners[0], Op::Install(Box::new(seed)))?;
        } else {
            self.install_cut(cut)?;
        }
        if cut.state() != previous {
            return Err(invalid("regroup changed observed physical state"));
        }
        if let Some(bytes) = previous_image
            && bytes != worker::encode(&cut.motion)?
        {
            return Err(invalid("handoff changed the native solver image"));
        }
        Ok(())
    }
    fn advance_cut(&mut self, cut: &mut Cut) -> Result<()> {
        match self.bounds.next(cut) {
            Routing::Keep => (),
            Routing::Join => {
                let h = cut.owners[0];
                self.regroup(cut, [h, h], cut.motion.clone())?;
            }
            Routing::CheckRelease => {
                let h = cut.owners[0];
                if let Answer::Detached(Some((motion, sphere))) =
                    self.cmd(h, cut.tick, Op::Detach)?
                {
                    if sphere != cut.sphere {
                        return Err(invalid("detachment changed the moving sphere"));
                    }
                    self.regroup(cut, [h, (h + 1) % cut.width], motion)?;
                }
            }
        }
        let [h, s] = cut.owners;
        self.update(cut, h, Op::Step)?;
        #[cfg(test)]
        if h != s {
            crash_tests::gate("split-after-hinge");
        }
        if s != h {
            self.update(cut, s, Op::Step)?;
        }
        cut.pending_effort = None;
        cut.tick = cut
            .tick
            .checked_add(1)
            .ok_or(invalid("moving tick exhausted"))?;
        Ok(())
    }
    pub fn advance(&mut self) -> Result<()> {
        self.transact(Self::advance_cut)
    }
    fn validate_effort(&self, joint: u64, effort: f64) -> Result<()> {
        if joint != self.cut.spec.hinge.joint_id || !effort.is_finite() {
            return Err(invalid("invalid moving joint effort"));
        }
        Ok(())
    }
    fn add_effort(&mut self, c: &mut Cut, effort: f64) -> Result<()> {
        self.update(c, c.owners[0], Op::Torque(effort))?;
        // Mirror the ordered sum accepted by Joints::add_effort, including Some(0).
        c.pending_effort = Some(c.pending_effort.unwrap_or(0.) + effort);
        Ok(())
    }
    pub fn queue_effort(&mut self, joint: u64, effort: f64) -> Result<()> {
        self.validate_effort(joint, effort)?;
        self.transact(|p, c| p.add_effort(c, effort))
    }
    /// Add to the pending torque and advance the moving contact in one durable interval.
    /// After an uncertain publication, check the reopened tick before resubmission.
    pub fn advance_with_effort(&mut self, joint: u64, effort: f64) -> Result<()> {
        self.validate_effort(joint, effort)?;
        self.transact(|p, c| {
            p.add_effort(c, effort)?;
            #[cfg(test)]
            crash_tests::gate("atomic-effort-accepted");
            p.advance_cut(c)
        })
    }
    /// Validate a logical interval under the current session lease.
    pub fn check_interval(&self, expected_epoch: u64, expected_tick: u64) -> Result<()> {
        crate::articulation::check_interval(
            self.healthy,
            expected_epoch,
            expected_tick,
            self.cut.epoch,
            self.cut.tick,
        )
    }
    /// Commit the supplied effort only if the session is still at the specified
    /// epoch and tick. A retry of a committed interval reports its current clock.
    /// Ownership transfers at the same clock keep the interval eligible.
    pub fn advance_with_effort_at(
        &mut self,
        expected_epoch: u64,
        expected_tick: u64,
        joint: u64,
        effort: f64,
    ) -> Result<()> {
        self.check_interval(expected_epoch, expected_tick)?;
        self.advance_with_effort(joint, effort)
    }
    pub fn transfer(&mut self, destination: u8) -> Result<()> {
        if !self.cut.joined() || destination >= self.width() || destination == self.owners()[0] {
            return Err(invalid(
                "moving handoff requires a joined world and distinct valid destination",
            ));
        }
        self.transact(|p, c| p.regroup(c, [destination, destination], c.motion.clone()))
    }
    pub fn replace_worker(&mut self, slot: u8) -> Result<()> {
        if !self.healthy || slot >= self.width() {
            return Err(invalid("unknown moving worker or fenced session"));
        }
        self.slots[slot as usize].take();
        self.rollback()
    }
    pub fn reset(&mut self) -> Result<()> {
        self.transact(|p, c| {
            let [h, s] = c.owners;
            p.fence(h, c.tick)?;
            if s != h {
                p.fence(s, c.tick)?
            }
            let mut fresh = Cut::new(c.spec, c.width)?;
            fresh.epoch = c
                .epoch
                .checked_add(1)
                .ok_or(invalid("moving epoch exhausted"))?;
            fresh.generation = c
                .generation
                .checked_add(1)
                .ok_or(invalid("moving generation exhausted"))?;
            p.install_cut(&mut fresh)?;
            *c = fresh;
            Ok(())
        })
    }
}

#[cfg(test)]
pub(crate) mod crash_tests;
