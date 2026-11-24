// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
mod contact;
use super::{
    native::TreeWorld,
    worker::{Action, Answer, Request},
    *,
};
use crate::{
    articulation::ledger::Ledger,
    moving::worker::{LIMIT, Peer, codec, encode},
};
use bincode::Options;
use std::{
    fs::File,
    io::{self, Read},
    path::{Path, PathBuf},
    process::{Command, Stdio},
    time::{Duration, Instant},
};

#[derive(Clone, Serialize, Deserialize)]
pub(super) struct Cut {
    pub(super) width: u8,
    pub(super) owner: u8,
    pub(super) epoch: u64,
    pub(super) generation: u64,
    pub(super) snapshot: Snapshot,
}
impl Cut {
    pub(super) fn tick(&self) -> u64 {
        self.snapshot.state.tick
    }
    fn validate(&self, spec: &impl TreeConstruction, recipe: [u8; 32]) -> Result<()> {
        if !(2..=8).contains(&self.width)
            || self.owner >= self.width
            || self.epoch > self.generation
        {
            return Err(invalid("invalid floating tree barrier ownership"));
        }
        self.snapshot.check(spec, recipe, self.tick())
    }
    fn generation_after(&self) -> Result<u64> {
        self.generation
            .checked_add(1)
            .ok_or(invalid("floating tree generation exhausted"))
    }
}
fn boundary(_name: &str) {
    #[cfg(test)]
    crate::moving::crash_tests::gate(_name);
}
fn session<S: TreeConstruction>(dir: &Path, create: bool) -> Result<Ledger> {
    let ledger = Ledger::lock(dir, create)?;
    if ledger.other_archive_exists(S::FILE)? {
        return Err(invalid("directory belongs to another hinge profile"));
    }
    Ok(ledger)
}
fn read<S: TreeConstruction>(ledger: &Ledger) -> Result<(S, Cut)> {
    let mut bytes = Vec::new();
    File::open(ledger.archive_path(S::FILE))?
        .take((LIMIT + 41) as u64)
        .read_to_end(&mut bytes)?;
    if bytes.len() < 40
        || bytes.len() > LIMIT + 40
        || bytes[8..40] != Sha256::digest(&bytes[40..])[..]
    {
        return Err(invalid(
            "invalid floating tree barrier format, size or checksum",
        ));
    }
    let (spec, cut): (S, Cut) = codec()
        .deserialize(&bytes[40..])
        .map_err(io::Error::other)?;
    if bytes[..8] != spec.archive_magic() {
        return Err(invalid(
            "floating tree archive profile disagrees with its recipe",
        ));
    }
    Ok((spec, cut))
}

/// Older executables reject the unrecognized capability query. The actual
/// startup handshake subsequently binds the chosen protocol and recipe to every slot.
fn worker_profile(executable: &Path) -> Result<FloatingTreeWorkerProfile> {
    if worker_supports(executable, "--probe-floating-tree-v2")? {
        Ok(FloatingTreeWorkerProfile::VectorV2)
    } else {
        Ok(FloatingTreeWorkerProfile::ScalarV1)
    }
}
pub(crate) fn worker_supports(executable: &Path, flag: &str) -> Result<bool> {
    let mut child = Command::new(executable)
        .arg(flag)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()?;
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = child.try_wait()? {
            return match status.code() {
                Some(code) => Ok(code == 0),
                None => Err(invalid(
                    "native worker capability query terminated by signal",
                )),
            };
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            child.wait()?;
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "native worker capability query timed out",
            )
            .into());
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}
/// Exclusive controller of one connected floating tree. Every published step
/// retains the native multibody coordinates, generalized velocities and inputs.
pub type FloatingTreePool = TreePool<FloatingTreeSpec>;
pub type ContactTreePool = TreePool<ContactTreeSpec>;
/// Controller of a tree with one or more finite revolute stops.
pub type StoppedTreePool = TreePool<StoppedTreeSpec>;
/// Controller of a tree with persistent bounded velocity motors.
pub type MotorTreePool = TreePool<MotorTreeSpec>;
/// Controller of a floating tree and its external fixed support.
pub type GroundedTreePool = TreePool<GroundedTreeSpec>;
/// A tree and its separately massed moving payload on the same native worker.
pub type PayloadTreePool = TreePool<PayloadTreeSpec>;
/// A gripper with persistent capped motors and atomic external payload forces.
pub type GraspTreePool = TreePool<GraspTreeSpec>;
/// Complete durable controller for an admitted tree construction.
pub struct TreePool<S: TreeConstruction> {
    ledger: super::fleet_session::FleetSession,
    executable: PathBuf,
    slots: Vec<Option<Peer>>,
    profile: FloatingTreeWorkerProfile,
    spec: S,
    recipe: [u8; 32],
    cut: Cut,
    healthy: bool,
    contact_session: Option<uuid::Uuid>,
}
impl<S: TreeConstruction> TreePool<S> {
    pub fn create(exe: &Path, dir: &Path, width: u8, spec: S) -> Result<Self> {
        let ledger = session::<S>(dir, true)?;
        if ledger.archive_path(S::FILE).try_exists()? {
            return Err(invalid("floating tree session already exists"));
        }
        Self::fresh(exe, ledger.into(), width, spec)
    }
    pub fn resume(exe: &Path, dir: &Path) -> Result<Self> {
        let ledger = session::<S>(dir, false)?;
        let (spec, cut) = read(&ledger)?;
        Self::open(exe, ledger.into(), spec, cut)
    }
    pub fn open_scene(exe: &Path, dir: &Path, width: Option<u8>, spec: S) -> Result<Self> {
        let ledger = session::<S>(dir, true)?;
        if !ledger.archive_path(S::FILE).try_exists()? {
            return Self::fresh(exe, ledger.into(), width.unwrap_or(2), spec);
        }
        let (saved, cut) = read::<S>(&ledger)?;
        if width.is_some_and(|w| w != cut.width) || spec.digest() != saved.digest() {
            return Err(invalid(
                "floating tree construction or width differs from session",
            ));
        }
        Self::open(exe, ledger.into(), saved, cut)
    }
    fn fresh(
        exe: &Path,
        ledger: super::fleet_session::FleetSession,
        width: u8,
        spec: S,
    ) -> Result<Self> {
        let snapshot = TreeWorld::new(&spec)?.capture()?;
        let cut = Cut {
            width,
            owner: 0,
            epoch: 0,
            generation: 0,
            snapshot,
        };
        let p = Self::open(exe, ledger, spec, cut)?;
        p.publish(&p.cut)?;
        Ok(p)
    }
    fn open(
        exe: &Path,
        ledger: super::fleet_session::FleetSession,
        spec: S,
        cut: Cut,
    ) -> Result<Self> {
        spec.validate_configuration()?;
        spec.base().root()?;
        let recipe = spec.digest();
        cut.validate(&spec, recipe)?;
        let executable = exe.canonicalize()?;
        let profile = match spec.required_worker() {
            Some(profile) => profile,
            None => worker_profile(&executable)?,
        };
        let mut p = Self {
            executable,
            slots: Vec::new(),
            profile,
            ledger,
            spec,
            recipe,
            cut,
            healthy: false,
            contact_session: (profile == FloatingTreeWorkerProfile::ContactV1)
                .then(uuid::Uuid::new_v4),
        };
        p.finish_startup()?;
        Ok(p)
    }
    fn publish(&self, cut: &Cut) -> io::Result<()> {
        if !self.ledger.standalone() {
            return Ok(());
        }
        let bytes = encode(&(&self.spec, cut))?;
        self.ledger.publish_bytes(
            &self.ledger.archive_path(S::FILE),
            &self.spec.archive_magic(),
            &bytes,
            boundary,
        )
    }
    fn spawn(&self, i: u8) -> Result<Peer> {
        Peer::new_profile(
            &self.executable,
            i,
            &self.ledger.slot(i),
            &self.spec,
            self.profile.flag(),
            self.profile.magic(),
        )
    }
    fn peer(&mut self, slot: u8) -> Result<&mut Peer> {
        self.slots
            .get_mut(slot as usize)
            .and_then(Option::as_mut)
            .ok_or(invalid("unknown or fenced floating tree worker"))
    }
    fn ack(&mut self, slot: u8, tick: u64, generation: u64, action: Action) -> Result<()> {
        match self
            .peer(slot)?
            .call_typed(tick, Request { generation, action })?
        {
            Answer::Acknowledged => Ok(()),
            _ => Err(invalid("unexpected floating tree acknowledgement")),
        }
    }
    fn stage(&mut self, owner: u8, cut: &Cut, generation: u64) -> Result<()> {
        self.ack(
            owner,
            cut.tick(),
            generation,
            Action::Stage(Box::new(cut.snapshot.clone())),
        )
    }
    fn activate(&mut self, owner: u8, cut: &Cut, generation: u64) -> Result<()> {
        self.ack(
            owner,
            cut.tick(),
            generation,
            Action::Activate(cut.snapshot.digest),
        )
    }
    fn fence_owner(&mut self, cut: &Cut) -> Result<()> {
        self.ack(
            cut.owner,
            cut.tick(),
            cut.generation,
            Action::Fence(cut.snapshot.digest),
        )
    }
    fn recover(&mut self) -> Result<()> {
        self.healthy = false;
        self.slots.clear();
        self.finish_startup()
    }
    fn finish_startup(&mut self) -> Result<()> {
        for i in self.slots.len() as u8..self.cut.width {
            let peer = self.spawn(i)?;
            self.slots.push(Some(peer));
        }
        let c = self.cut.clone();
        self.stage(c.owner, &c, c.generation)?;
        self.activate(c.owner, &c, c.generation)?;
        self.healthy = true;
        Ok(())
    }
    fn transact(
        &mut self,
        mut prepare: impl FnMut(&mut Self, &mut Cut) -> Result<()>,
    ) -> Result<()> {
        self.check_interval(self.epoch(), self.cut.tick())?;
        let mut retry = true;
        loop {
            let mut next = self.cut.clone();
            if let Err(e) =
                prepare(self, &mut next).and_then(|()| next.validate(&self.spec, self.recipe))
            {
                self.recover()?;
                if retry && matches!(e, ArticulationError::Io(_)) {
                    retry = false;
                    continue;
                }
                return Err(e);
            }
            if let Err(e) = self.publish(&next) {
                self.healthy = false;
                self.slots.clear();
                return Err(ArticulationError::Publication(e));
            }
            if self.ledger.standalone() {
                boundary("after-sync");
            }
            self.cut = next;
            return Ok(());
        }
    }
    fn validate_inputs(&self, inputs: &[(u64, f64)]) -> Result<()> {
        let joints = &self.observation().joints;
        validate_efforts(inputs, joints.len(), |id| {
            let i = joints.binary_search_by_key(&id, |j| j.joint_id).ok()?;
            Some((i, joints[i].pending_effort))
        })
    }
    fn prepare(&mut self, c: &mut Cut, action: Action, next_tick: u64) -> Result<()> {
        let answer: Answer = self.peer(c.owner)?.call_typed(
            c.tick(),
            Request {
                generation: c.generation,
                action,
            },
        )?;
        let Answer::State(next) = answer else {
            return Err(invalid("missing floating tree state"));
        };
        if next.state.tick != next_tick {
            return Err(invalid("incorrect floating tree step clock"));
        }
        c.snapshot = *next;
        Ok(())
    }
    /// Negotiated input capability of the current executable.
    pub fn worker_profile(&self) -> FloatingTreeWorkerProfile {
        self.profile
    }
    /// Authored body and joint topology shared by all native tree profiles.
    pub fn topology(&self) -> &FloatingTreeSpec {
        self.spec.base()
    }
    pub fn joint_limits(&self) -> &[FloatingTreeJointLimit] {
        self.spec.limits()
    }
    pub fn spec(&self) -> &S {
        &self.spec
    }
    pub fn observation(&self) -> &FloatingTreeObservation {
        &self.cut.snapshot.state
    }
    /// External body and per-link contact readback, in public link order.
    pub fn observed_payload(
        &self,
    ) -> Option<(
        &ArticulationLinkState,
        &[gz_physics_multifidelity::MovingReaction],
    )> {
        let p = self.cut.snapshot.image.payload()?;
        Some((&p.body, &p.reactions))
    }
    /// Solver-effective persistent settings, in public joint order.
    pub fn observed_velocity_motors(
        &self,
    ) -> Option<impl ExactSizeIterator<Item = Option<gz_physics::JointVelocityMotor>> + '_> {
        Some(
            self.cut
                .snapshot
                .image
                .motors()?
                .iter()
                .map(|slot| slot.map(MotorSetting::native)),
        )
    }
    /// Native multibody checkpoint for a committed advancing interval.
    pub fn image(&self) -> Option<&[u8]> {
        self.cut.snapshot.image.native()
    }
    pub fn epoch(&self) -> u64 {
        self.cut.epoch
    }
    pub fn generation(&self) -> u64 {
        self.cut.generation
    }
    pub fn width(&self) -> u8 {
        self.cut.width
    }
    pub fn owner(&self) -> u8 {
        self.cut.owner
    }
    pub fn worker_pid(&self, slot: u8) -> Option<u32> {
        self.slots.get(slot as usize)?.as_ref().map(Peer::pid)
    }
    pub fn pending_joint_effort(&self, joint: u64) -> Option<f64> {
        let i = self
            .observation()
            .joints
            .binary_search_by_key(&joint, |s| s.joint_id)
            .ok()?;
        self.observation().joints[i].pending_effort
    }
    pub fn check_interval(&self, epoch: u64, tick: u64) -> Result<()> {
        crate::articulation::check_interval(
            self.healthy,
            epoch,
            tick,
            self.epoch(),
            self.cut.tick(),
        )
    }
    pub fn queue_effort(&mut self, joint: u64, effort: f64) -> Result<()> {
        self.validate_inputs(&[(joint, effort)])?;
        self.transact(|p, c| {
            p.prepare(c, Action::Queue { joint, effort }, c.tick())?;
            boundary("tree-worker-prepared");
            Ok(())
        })
    }
    pub fn advance(&mut self) -> Result<()> {
        self.advance_with_efforts(&[])
    }
    pub fn advance_with_effort(&mut self, joint: u64, effort: f64) -> Result<()> {
        self.advance_with_efforts(&[(joint, effort)])
    }
    pub fn advance_with_effort_at(
        &mut self,
        epoch: u64,
        tick: u64,
        joint: u64,
        effort: f64,
    ) -> Result<()> {
        self.advance_with_efforts_at(epoch, tick, &[(joint, effort)])
    }
    /// Apply one signed contribution per target in a single durable interval.
    /// Existing pending torques on every joint are included. An empty slice advances time.
    pub fn advance_with_efforts(&mut self, inputs: &[(u64, f64)]) -> Result<()> {
        self.validate_inputs(inputs)?;
        self.transact(|p, c| {
            let tick = c.tick();
            let next = tick
                .checked_add(1)
                .ok_or(invalid("floating tree clock exhausted"))?;
            let action = match inputs {
                [] => Action::Advance { effort: None },
                [only] => Action::Advance {
                    effort: Some(*only),
                },
                many if p.profile != FloatingTreeWorkerProfile::ScalarV1 => {
                    Action::AdvanceMany(many.to_vec())
                }
                many => {
                    let (last, queued) = many.split_last().expect("multiple efforts");
                    for &(joint, effort) in queued {
                        p.prepare(c, Action::Queue { joint, effort }, tick)?;
                        boundary("tree-vector-partial");
                        #[cfg(test)]
                        super::crash_tests::kill_partial_owner_once(p.peer(c.owner)?.pid());
                    }
                    Action::Advance {
                        effort: Some(*last),
                    }
                }
            };
            p.prepare(c, action, next)?;
            boundary("tree-worker-prepared");
            Ok(())
        })
    }
    pub fn advance_with_efforts_at(
        &mut self,
        epoch: u64,
        tick: u64,
        inputs: &[(u64, f64)],
    ) -> Result<()> {
        self.check_interval(epoch, tick)?;
        self.advance_with_efforts(inputs)
    }
    pub fn transfer(&mut self, destination: u8) -> Result<()> {
        if destination >= self.width() || destination == self.owner() {
            return Err(invalid("invalid floating tree transfer destination"));
        }
        self.transact(|p, c| {
            let generation = c.generation_after()?;
            p.stage(destination, c, generation)?;
            boundary("tree-destination-staged");
            p.fence_owner(c)?;
            boundary("tree-source-fenced");
            p.activate(destination, c, generation)?;
            boundary("tree-destination-active");
            c.owner = destination;
            c.generation = generation;
            Ok(())
        })
    }
    pub fn reset(&mut self) -> Result<()> {
        self.transact(|p, c| {
            let next_epoch = c
                .epoch
                .checked_add(1)
                .ok_or(invalid("floating tree epoch exhausted"))?;
            let next_generation = c.generation_after()?;
            let fresh = TreeWorld::new(&p.spec)?.capture()?;
            p.fence_owner(c)?;
            c.epoch = next_epoch;
            c.generation = next_generation;
            c.snapshot = fresh;
            c.owner = 0;
            p.stage(c.owner, c, c.generation)?;
            p.activate(c.owner, c, c.generation)?;
            Ok(())
        })
    }
    /// Inspect transport replay handling with a single deliberately lost acknowledgement.
    pub fn drop_next_acknowledgement(&mut self, worker: u8) -> Result<()> {
        self.check_interval(self.epoch(), self.cut.tick())?;
        self.peer(worker)?.withhold = true;
        Ok(())
    }
    pub fn replace_worker(&mut self, slot: u8) -> Result<()> {
        self.check_interval(self.epoch(), self.cut.tick())?;
        if slot >= self.width() {
            return Err(invalid("unknown floating tree worker"));
        }
        // Close and reap the old capability before opening its lifetime lease.
        self.healthy = false;
        self.slots[slot as usize].take();
        self.slots[slot as usize] = Some(self.spawn(slot)?);
        if slot == self.owner() {
            let c = self.cut.clone();
            self.stage(slot, &c, c.generation)?;
            self.activate(slot, &c, c.generation)?;
        }
        self.healthy = true;
        Ok(())
    }
}

impl<S: MotorizedTreeConstruction> TreePool<S> {
    /// Effective persistent velocity target and torque cap on the committed interval.
    pub fn velocity_motor(&self, joint: u64) -> Option<gz_physics::JointVelocityMotor> {
        let i = self
            .observation()
            .joints
            .binary_search_by_key(&joint, |s| s.joint_id)
            .ok()?;
        self.cut.snapshot.image.motors()?[i].map(MotorSetting::native)
    }
    /// Durably replace or disable one persistent velocity motor without advancing time.
    pub fn set_velocity_motor(
        &mut self,
        joint: u64,
        input: Option<gz_physics::JointVelocityMotor>,
    ) -> Result<()> {
        let setting = input.map(MotorSetting::from);
        if setting.is_some()
            && self.spec.exact_locks()
            && self
                .spec
                .limits()
                .iter()
                .any(|s| s.joint_id == joint && s.lower == 0. && s.upper == 0.)
        {
            return Err(invalid("an exact revolute lock cannot accept a motor"));
        }
        if setting.is_some_and(|m| !m.valid())
            || self
                .observation()
                .joints
                .binary_search_by_key(&joint, |s| s.joint_id)
                .is_err()
        {
            return Err(invalid("invalid floating tree motor input or joint"));
        }
        self.transact(|p, c| {
            p.prepare(c, Action::SetMotor { joint, setting }, c.tick())?;
            boundary("tree-motor-prepared");
            Ok(())
        })
    }
}

impl GroundedTreePool {
    /// Per-link solved reactions from the owned floor in ascending public link ID order.
    pub fn ground_reactions(&self) -> &[gz_physics_multifidelity::GroundReaction] {
        self.cut
            .snapshot
            .image
            .reactions()
            .expect("grounded tree captures per-link reactions")
    }
}

impl<S: PayloadTreeConstruction> TreePool<S> {
    pub fn payload_state(&self) -> &ArticulationLinkState {
        &self
            .cut
            .snapshot
            .image
            .payload()
            .expect("checked payload receipt")
            .body
    }
    /// World-space solved impulses on each articulated link from the moving payload.
    pub fn payload_reactions(&self) -> &[gz_physics_multifidelity::MovingReaction] {
        &self
            .cut
            .snapshot
            .image
            .payload()
            .expect("checked payload receipt")
            .reactions
    }
}

impl GraspTreePool {
    /// Apply this force at the payload COM for one advancing 1 ms interval.
    /// Motors and independently queued joint efforts retain their usual semantics.
    pub fn advance_with_payload_force(
        &mut self,
        efforts: &[(u64, f64)],
        force: [f64; 3],
    ) -> Result<()> {
        if !force.iter().all(|v| v.is_finite()) {
            return Err(invalid("non-finite grasp payload force"));
        }
        self.validate_inputs(efforts)?;
        self.transact(|p, c| {
            let next = c
                .tick()
                .checked_add(1)
                .ok_or(invalid("floating tree clock exhausted"))?;
            p.prepare(
                c,
                Action::AdvanceLoaded {
                    efforts: efforts.to_vec(),
                    force,
                },
                next,
            )?;
            boundary("tree-worker-prepared");
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
}
