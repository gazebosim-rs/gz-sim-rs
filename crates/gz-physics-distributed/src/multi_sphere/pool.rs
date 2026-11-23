// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{disk::Disk, *};
use crate::moving::{WorkerTransport, routing::RoutingBounds};
use glam::DVec3;

pub struct MultiSpherePool {
    disk: Disk,
    exe: PathBuf,
    transport: WorkerTransport,
    slots: Vec<Option<Peer>>,
    spec: MultiSphereSpec,
    bounds: [RoutingBounds; 2],
    cut: Cut,
    observation: MultiSphereObservation,
    healthy: bool,
    reopen_attempts: u8,
    reopen_attempts_used: u64,
    recovery_status: Option<RecoveryStatus>,
    recovery_monitor: Option<RecoveryMonitor>,
    operator_fence: Option<FenceReceipt>,
}
/// Keep the I/O kind available to recovery policy, and format the location only on failure.
fn rpc_error(
    error: ArticulationError,
    transport: WorkerTransport,
    slot: u8,
    epoch: u64,
    tick: u64,
    action: &str,
) -> ArticulationError {
    match error {
        ArticulationError::Io(e) => io::Error::new(
            e.kind(),
            format!(
                "multi-sphere {transport:?} epoch {epoch} tick {tick} slot {slot} {action}: {e}"
            ),
        )
        .into(),
        other => other,
    }
}

fn recovery_failure(
    operation: ArticulationError,
    recovery: ArticulationError,
) -> ArticulationError {
    ArticulationError::MultiSphereRecovery {
        operation: Box::new(operation),
        recovery: Box::new(recovery),
    }
}

// A nested rollback records both attempts. Every observed failure must be a
// transport loss before it is safe to resubmit a physics operation.
fn transport_loss(error: &ArticulationError) -> bool {
    match error {
        ArticulationError::Io(e) => matches!(
            e.kind(),
            io::ErrorKind::TimedOut
                | io::ErrorKind::BrokenPipe
                | io::ErrorKind::UnexpectedEof
                | io::ErrorKind::ConnectionReset
                | io::ErrorKind::ConnectionAborted
        ),
        ArticulationError::MultiSphereRecovery {
            operation,
            recovery,
        } => transport_loss(operation) && transport_loss(recovery),
        _ => false,
    }
}

impl MultiSpherePool {
    /// Maximum number of additional full worker-pool starts per advancing call.
    pub const MAX_REOPEN_ATTEMPTS: u8 = 8;

    /// Allow up to this many fresh worker cohorts when transport loss fences
    /// an advancing call. The call keeps its writer lease, waits 100 ms before
    /// the first start and doubles the wait up to 800 ms. Each worker uses its
    /// normal RPC deadlines. A successful reinstall rechecks the original
    /// interval and replays the step. Exhaustion leaves the pool fenced.
    ///
    /// Zero disables additional starts and is the default on create or resume.
    /// Values above [Self::MAX_REOPEN_ATTEMPTS] are rejected without changing
    /// the current setting.
    pub fn set_reopen_attempts(&mut self, attempts: u8) -> Result<()> {
        if attempts > Self::MAX_REOPEN_ATTEMPTS {
            return Err(invalid("expected 0 through 8 multi-sphere reopen attempts"));
        }
        self.reopen_attempts = attempts;
        Ok(())
    }

    pub fn reopen_attempt_budget(&self) -> u8 {
        self.reopen_attempts
    }

    /// Subscribe to the same progress snapshot as other readers of this pool.
    /// Healthy ticks neither acquire the monitor lock nor publish a heartbeat.
    pub fn subscribe_recovery(&mut self) -> RecoveryMonitor {
        self.recovery_monitor
            .get_or_insert_with(|| {
                RecoveryMonitor::new(self.recovery_status, self.operator_fence.clone())
            })
            .clone()
    }

    #[cold]
    fn report_recovery(&mut self, phase: RecoveryPhase, attempt: u8) {
        let status = Some(RecoveryStatus {
            epoch: self.cut.epoch,
            tick: self.cut.tick,
            phase,
            attempt,
            budget: self.reopen_attempts,
        });
        self.recovery_status = status;
        if let Some(monitor) = &self.recovery_monitor {
            monitor.update(status, self.operator_fence.clone());
        }
    }

    fn recovery_attempt(&self) -> u8 {
        self.recovery_status.map_or(0, |s| s.attempt)
    }

    fn finish_recovery(&mut self, result: Result<()>) -> Result<()> {
        if self.healthy {
            if self.recovery_status.take().is_some()
                && let Some(monitor) = &self.recovery_monitor
            {
                monitor.update(None, None);
            }
        } else if self
            .recovery_status
            .is_none_or(|s| s.phase != RecoveryPhase::Fenced)
        {
            self.report_recovery(RecoveryPhase::Fenced, self.recovery_attempt());
        }
        result
    }

    /// Full automatic pool starts attempted during this coordinator's lifetime.
    pub fn reopen_attempts_used(&self) -> u64 {
        self.reopen_attempts_used
    }

    pub fn create(exe: &Path, dir: &Path, width: u8, spec: MultiSphereSpec) -> Result<Self> {
        Self::create_with_transport(exe, dir, width, spec, WorkerTransport::Pipe)
    }
    pub fn create_with_transport(
        exe: &Path,
        dir: &Path,
        width: u8,
        spec: MultiSphereSpec,
        transport: WorkerTransport,
    ) -> Result<Self> {
        let disk = Disk::lock(dir, true)?;
        if disk.exists()? {
            return Err(invalid("multi-sphere session already exists"));
        }
        Self::open(exe, disk, width, spec, None, transport)
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
        let (spec, cut) = disk.read()?;
        Self::open(exe, disk, cut.width, spec, Some(cut), transport)
    }
    /// Reopen an existing scene at an operator-selected, coordinator-issued
    /// fence. Acquire the writer lease before reading, then verify and install
    /// the same decoded native graphs without advancing or consuming input.
    pub fn resume_scene_guarded(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: MultiSphereSpec,
        transport: WorkerTransport,
        receipt: &FenceReceipt,
    ) -> Result<Self> {
        let disk = Disk::lock_matching(dir, receipt.writer())?;
        Self::open_verified(
            exe,
            disk,
            width,
            spec,
            transport,
            (receipt.epoch(), receipt.tick(), receipt.sha256()),
        )
    }

    /// Attach a replacement supervisor only to the operator-selected cold cut.
    /// The old supervisor's in-memory fence offer is never used for this path.
    pub fn resume_scene_selected(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: MultiSphereSpec,
        transport: WorkerTransport,
        selected: &ColdSelection,
    ) -> Result<Self> {
        let disk = Disk::lock_matching(dir, selected.writer())?;
        selected.check_origin(&disk)?;
        let inspected = selected.cut();
        Self::open_verified(
            exe,
            disk,
            width,
            spec,
            transport,
            (
                inspected.cut.epoch,
                inspected.cut.observation.hinge.tick,
                &inspected.sha256,
            ),
        )
    }

    fn open_verified(
        exe: &Path,
        disk: Disk,
        width: Option<u8>,
        spec: MultiSphereSpec,
        transport: WorkerTransport,
        expected: (u64, u64, &str),
    ) -> Result<Self> {
        let (saved, cut, digest) = disk.read_identified()?;
        if (cut.epoch, cut.tick, digest.as_str()) != expected {
            return Err(invalid(
                "multi-sphere archive differs from the selected checkpoint",
            ));
        }
        Self::validate_saved_scene(width, &spec, &saved, cut.width)?;
        disk.reclaim_staging()?;
        Self::open(exe, disk, cut.width, spec, Some(cut), transport)
    }

    /// Prove that the package's already-read archive is the exact publication
    /// of this healthy live writer, including pending inputs and both native graphs.
    pub fn verify_live_cut(&self, origin: &ColdOrigin, bytes: &[u8]) -> Result<()> {
        if !self.healthy {
            return Err(invalid(
                "cannot capture a live checkpoint from a fenced pool",
            ));
        }
        origin.check_disk(&self.disk)?;
        if !disk::matches_publication(&self.spec, &self.cut, bytes)? {
            return Err(invalid(
                "captured archive differs from the retained live checkpoint",
            ));
        }
        Ok(())
    }

    pub fn operator_fence_receipt(&self) -> Option<&FenceReceipt> {
        self.operator_fence.as_ref()
    }

    pub fn open_scene(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: MultiSphereSpec,
    ) -> Result<Self> {
        Self::open_scene_with_transport(exe, dir, width, spec, WorkerTransport::Pipe)
    }
    pub fn open_scene_with_transport(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: MultiSphereSpec,
        transport: WorkerTransport,
    ) -> Result<Self> {
        let disk = Disk::lock(dir, true)?;
        if disk.exists()? {
            let (saved, cut) = disk.read()?;
            Self::validate_saved_scene(width, &spec, &saved, cut.width)?;
            Self::open(exe, disk, cut.width, spec, Some(cut), transport)
        } else {
            Self::open(exe, disk, width.unwrap_or(3), spec, None, transport)
        }
    }
    fn validate_saved_scene(
        width: Option<u8>,
        spec: &MultiSphereSpec,
        saved: &MultiSphereSpec,
        saved_width: u8,
    ) -> Result<()> {
        if width.is_some_and(|w| w != saved_width) || !saved.same_recipe(spec)? {
            return Err(invalid("multi-sphere session recipe or width differs"));
        }
        Ok(())
    }

    fn open(
        exe: &Path,
        disk: Disk,
        width: u8,
        spec: MultiSphereSpec,
        old: Option<Cut>,
        transport: WorkerTransport,
    ) -> Result<Self> {
        if !(3..=8).contains(&width) {
            return Err(invalid("multi-sphere sessions need 3 through 8 workers"));
        }
        spec.validate()?;
        let exe = exe.canonicalize()?;
        let mut slots = (0..width)
            .map(|i| {
                Peer::using(&exe, i, &disk.slot(i), &spec, WORKER_FLAG, MAGIC, transport)
                    .map(Some)
                    .map_err(|e| {
                        rpc_error(
                            e,
                            transport,
                            i,
                            old.as_ref().map_or(0, |c| c.epoch),
                            old.as_ref().map_or(0, |c| c.tick),
                            "bootstrap",
                        )
                    })
            })
            .collect::<Result<Vec<_>>>()?;
        let new = old.is_none();
        let cut = if let Some(c) = old {
            c
        } else {
            let h = slots[0].as_mut().unwrap();
            let Reply::Hinge(hinge) = h.call_typed(0, Command::Install(Seed::Hinge(None)))? else {
                return Err(invalid("missing initial hinge"));
            };
            let _: Reply = h.call_typed(0, Command::Fence)?;
            Cut {
                width,
                tick: 0,
                epoch: 0,
                generation: 0,
                owners: initial_owners(hinge.sample.mask),
                hinge: *hinge,
                spheres: std::array::from_fn(|i| spec.single(i).initial_sphere()),
                pending: None,
                pair: None,
            }
        };
        cut.validate(&spec)?;
        let observation = cut.state(spec.hinge.joint_id);
        let mut pool = Self {
            disk,
            exe,
            transport,
            slots,
            spec,
            bounds: std::array::from_fn(|i| RoutingBounds::new(&spec.single(i))),
            cut,
            observation,
            healthy: true,
            reopen_attempts: 0,
            reopen_attempts_used: 0,
            recovery_monitor: None,
            operator_fence: None,
            recovery_status: None,
        };
        pool.install_exact(&pool.cut.clone())?;
        if new {
            pool.disk.publish(&spec, &pool.cut)?;
        }
        Ok(pool)
    }
    pub fn spec(&self) -> &MultiSphereSpec {
        &self.spec
    }
    pub fn observation(&self) -> &MultiSphereObservation {
        &self.observation
    }
    pub fn pending_joint_effort(&self) -> Option<f64> {
        self.cut.pending
    }
    pub fn owners(&self) -> [u8; 3] {
        self.cut.owners
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
    pub fn image(&self) -> Option<&[u8]> {
        Some(&self.cut.hinge.image)
    }
    /// Exact native contact world of the independently owned pair, when present.
    pub fn pair_image(&self) -> Option<&[u8]> {
        self.cut.pair.as_ref().map(|p| p.image.as_slice())
    }
    pub fn worker_pid(&self, slot: u8) -> Option<u32> {
        Some(self.slots.get(slot as usize)?.as_ref()?.pid())
    }
    pub fn worker_steps(&self, slot: u8) -> Option<u64> {
        Some(self.slots.get(slot as usize)?.as_ref()?.steps)
    }
    pub fn drop_next_acknowledgement(&mut self, slot: u8) -> Result<()> {
        self.slots
            .get_mut(slot as usize)
            .and_then(Option::as_mut)
            .ok_or(invalid("unknown multi-sphere worker"))?
            .withhold = true;
        Ok(())
    }
    fn call(&mut self, slot: u8, tick: u64, cmd: Command) -> Result<Reply> {
        #[cfg(feature = "sphere-pair-trace")]
        let _timing = timing::Scope::begin("rpc", cmd.name(), tick, Some(slot));
        let action = cmd.name();
        let s = &mut self.slots[slot as usize];
        let r = s
            .as_mut()
            .ok_or(invalid("missing multi-sphere worker"))?
            .call_typed(tick, cmd)
            .map_err(|e| rpc_error(e, self.transport, slot, self.cut.epoch, tick, action));
        if r.is_err() {
            s.take();
        }
        r
    }
    fn install_exact(&mut self, c: &Cut) -> Result<()> {
        self.install_worlds(c).map(|_| ())
    }
    fn install_worlds(&mut self, c: &Cut) -> Result<Option<PairPart>> {
        c.install_worlds(|slot, seed| self.call(slot, c.tick, Command::Install(seed)))
    }
    fn reassign(
        &mut self,
        c: &mut Cut,
        owners: [u8; 3],
        h: HingePart,
        pair: Option<PairPart>,
    ) -> Result<()> {
        let old = worker::encode(&c.state(self.spec.hinge.joint_id))?;
        for (i, &slot) in c.owners.iter().enumerate() {
            if !c.owners[..i].contains(&slot) {
                self.call(slot, c.tick, Command::Fence)?;
            }
        }
        #[cfg(test)]
        crate::moving::crash_tests::gate("regroup-fenced");
        c.owners = owners;
        c.accept_hinge(h)?;
        c.pair = pair;
        c.pair = self.install_worlds(c)?;
        if worker::encode(&c.state(self.spec.hinge.joint_id))? != old {
            return Err(invalid(
                "multi-sphere regroup changed its public observation",
            ));
        }
        c.generation = c
            .generation
            .checked_add(1)
            .ok_or(invalid("multi-sphere generation exhausted"))?;
        Ok(())
    }
    fn rollback(&mut self) -> Result<()> {
        self.report_recovery(RecoveryPhase::Restoring, self.recovery_attempt());
        self.healthy = false;
        let first = self.restore_committed();
        let result = match first {
            Err(e) if transport_loss(&e) => {
                // A failed bootstrap or install has dropped the suspect peer. Fence
                // any peers already restored and install the same committed cut.
                self.restore_committed()
                    .map_err(|second| recovery_failure(e, second))
            }
            other => other,
        };
        self.healthy = result.is_ok();
        if !self.healthy {
            self.slots.fill_with(|| None);
        }
        result
    }
    fn restore_committed(&mut self) -> Result<()> {
        for i in 0..self.width() {
            if self.slots[i as usize].is_some() {
                match self.call(i, self.cut.tick, Command::Fence) {
                    Ok(Reply::Empty) => (),
                    Err(e) if transport_loss(&e) => (),
                    Err(e) => return Err(e),
                    Ok(_) => return Err(invalid("unexpected multi-sphere fence response")),
                }
            }
            if self.slots[i as usize].is_none() {
                self.slots[i as usize] = Some(
                    Peer::using(
                        &self.exe,
                        i,
                        &self.disk.slot(i),
                        &self.spec,
                        WORKER_FLAG,
                        MAGIC,
                        self.transport,
                    )
                    .map_err(|e| {
                        rpc_error(
                            e,
                            self.transport,
                            i,
                            self.cut.epoch,
                            self.cut.tick,
                            "recovery bootstrap",
                        )
                    })?,
                );
            }
        }
        self.install_exact(&self.cut.clone())
    }
    fn transact(&mut self, op: impl FnMut(&mut Self, &mut Cut) -> Result<()>) -> Result<()> {
        let result = self.try_transaction(op);
        self.finish_recovery(result)
    }

    fn try_transaction(
        &mut self,
        mut op: impl FnMut(&mut Self, &mut Cut) -> Result<()>,
    ) -> Result<()> {
        if !self.healthy {
            return Err(invalid("reopen fenced multi-sphere session"));
        }
        for retry in [true, false] {
            let mut next = self.cut.clone();
            if let Err(e) = op(self, &mut next) {
                if let Err(recovery) = self.rollback() {
                    return Err(recovery_failure(e, recovery));
                }
                if retry && transport_loss(&e) {
                    self.report_recovery(RecoveryPhase::Replaying, self.recovery_attempt());
                    continue;
                }
                return Err(e);
            }
            if let Err(e) = self.disk.publish(&self.spec, &next) {
                self.slots.clear();
                self.healthy = false;
                return Err(ArticulationError::Publication(e));
            }
            #[cfg(test)]
            crate::moving::crash_tests::gate("after-sync");
            self.observation = next.state(self.spec.hinge.joint_id);
            self.cut = next;
            return Ok(());
        }
        unreachable!()
    }
    fn hinge_mask(
        &self,
        c: &Cut,
        independent: bool,
        padding: f64,
        ends: Option<[[f64; 3]; 2]>,
    ) -> u8 {
        let obs = c.state(self.spec.hinge.joint_id);
        let before = c.hinge.sample.mask;
        let mut mask = 0;
        for i in 0..2 {
            let mut pair = crate::moving::MovingObservation {
                hinge: obs.hinge,
                sphere: c.spheres[i],
                reactions: obs.reactions[i],
            };
            let mut spec = self.spec.single(i);
            spec.sphere.radius += padding;
            if let Some(ends) = ends {
                let delta = DVec3::from(ends[i]) - DVec3::from(pair.sphere.position);
                pair.sphere.velocity = (delta / 0.001).to_array();
            }
            if self.bounds[i].needs_worker(
                &spec,
                &pair,
                c.pending,
                !independent && includes(before, i),
            ) {
                mask |= 1 << i;
            }
        }
        mask
    }
    fn route_pair(&mut self, c: &mut Cut) -> Result<()> {
        // First retain the incoming-velocity sweeps, including high-speed crossings.
        // Once the companions interact their free-flight endpoints cease to be valid.
        // Rapier's non-CCD worker checks collisions at the start and end of its pass.
        // The endpoint of the pair's own native solve is authoritative for the second check.
        let mut join = self.hinge_mask(c, true, 0., None) != 0;
        if !join {
            let Reply::PairEndpoint(ends) = self.call(c.owners[1], c.tick, Command::PreviewPair)?
            else {
                return Err(invalid("missing native pair endpoints"));
            };
            #[cfg(test)]
            crate::moving::crash_tests::gate("pair-previewed");
            join = self.hinge_mask(c, true, 0., Some(ends)) != 0;
        }
        if join {
            let pair = c
                .pair
                .as_ref()
                .ok_or(invalid("missing pair image on merge"))?;
            let Reply::Hinge(h) = self.call(
                c.owners[0],
                c.tick,
                Command::MergePair(Box::new(pair.clone())),
            )?
            else {
                return Err(invalid("missing merged hinge"));
            };
            #[cfg(test)]
            crate::moving::crash_tests::gate("pair-merge-prepared");
            self.reassign(c, [c.owners[0]; 3], *h, None)?;
        }
        Ok(())
    }
    fn route(&mut self, c: &mut Cut) -> Result<()> {
        let close = spheres_may_meet(c, &self.spec);
        if let Some(pair) = &c.pair {
            if close || !pair.drained {
                return self.route_pair(c);
            }
            let [h, a, _] = c.owners;
            let b = (0..c.width)
                .find(|s| *s != a && *s != h)
                .ok_or(invalid("no free sphere worker"))?;
            self.reassign(c, [h, a, b], c.hinge.clone(), None)?;
        }
        let before = c.hinge.sample.mask;
        let mut mask = self.hinge_mask(c, false, 0., None);
        if close {
            if before == 0 && mask == 0 {
                let [h, a, _] = c.owners;
                self.reassign(c, [h, a, a], c.hinge.clone(), None)?;
                return self.route_pair(c);
            }
            if before == 3 && self.hinge_mask(c, true, 0.05, None) == 0 {
                match self.call(c.owners[0], c.tick, Command::SplitPair)? {
                    Reply::SplitPair(parts) => {
                        let (hinge, pair) = *parts;
                        let h = c.owners[0];
                        let a = (h + 1) % c.width;
                        self.reassign(c, [h, a, a], hinge, Some(pair))?;
                        return self.route_pair(c);
                    }
                    Reply::Empty => (),
                    _ => return Err(invalid("unexpected sphere pair split reply")),
                }
            }
            mask = 3;
        }
        // Completely free motion already has certificates for all three pairs.
        // A partially assembled world can receive contact impulses from its
        // resident sphere; the worker compares full and split discrete solves.
        if (before == 0 || before == 3) && before == mask {
            return Ok(());
        }
        let h = match self.call(
            c.owners[0],
            c.tick,
            Command::Plan {
                mask,
                spheres: c.spheres,
            },
        )? {
            Reply::Empty => return Ok(()),
            Reply::Hinge(h) => h,
            _ => return Err(invalid("missing multi-sphere routing decision")),
        };
        if h.sample.mask != before {
            let mut owners = c.owners;
            let hinge = owners[0];
            for i in 0..2 {
                if includes(h.sample.mask, i) {
                    owners[i + 1] = hinge;
                }
            }
            for i in 0..2 {
                if !includes(h.sample.mask, i) && owners[i + 1] == hinge {
                    owners[i + 1] = (0..c.width)
                        .find(|slot| !owners.contains(slot))
                        .ok_or(invalid("no free sphere worker"))?;
                }
            }
            self.reassign(c, owners, *h, None)?;
        }
        Ok(())
    }
    fn advance_cut(&mut self, c: &mut Cut) -> Result<()> {
        self.route(c)?;
        // Routing and the pair preview must finish before committing any step.
        // The resulting disjoint native worlds can now solve concurrently. Keep
        // receipt validation and archive order stable for every worker layout.
        let mut tickets = [None, None, None];
        let mut failure = None;
        for (i, &slot) in c.owners.iter().enumerate() {
            if c.owners[..i].contains(&slot) {
                continue;
            }
            #[cfg(feature = "sphere-pair-trace")]
            let _timing = timing::Scope::begin("submit", "step", c.tick, Some(slot));
            let peer = &mut self.slots[slot as usize];
            match peer
                .as_mut()
                .ok_or(invalid("missing multi-sphere worker"))
                .and_then(|w| w.submit(c.tick, Command::Step))
            {
                Ok(ticket) => tickets[i] = Some(ticket),
                Err(e) => {
                    peer.take();
                    failure = Some(rpc_error(
                        e,
                        self.transport,
                        slot,
                        c.epoch,
                        c.tick,
                        "submit step",
                    ));
                    break;
                }
            }
        }
        // A failed send or receive can leave a previous healthy peer's request
        // in flight. Drain every submitted ticket before rollback fences peers.
        let mut replies: [Option<Reply>; 3] = [None, None, None];
        for (i, ticket) in tickets.into_iter().enumerate() {
            let Some(ticket) = ticket else { continue };
            let slot = c.owners[i];
            #[cfg(feature = "sphere-pair-trace")]
            let _timing = timing::Scope::begin("rpc", "step", c.tick, Some(slot));
            let peer = &mut self.slots[slot as usize];
            match peer
                .as_mut()
                .expect("submitted worker is retained until receipt")
                .collect(ticket)
            {
                Ok(reply) => replies[i] = Some(reply),
                Err(e) => {
                    peer.take();
                    // A protocol failure from a second peer must not be hidden
                    // by an earlier connection loss and authorize a replay.
                    if failure.is_none() || !transport_loss(&e) {
                        failure = Some(rpc_error(
                            e,
                            self.transport,
                            slot,
                            c.epoch,
                            c.tick,
                            "collect step",
                        ));
                    }
                }
            }
        }
        if let Some(e) = failure {
            return Err(e);
        }
        let mut collect = |i: usize| {
            replies[i]
                .take()
                .ok_or(invalid("missing native step receipt"))
        };
        let Reply::Hinge(h) = collect(0)? else {
            return Err(invalid("missing hinge step"));
        };
        c.accept_hinge(*h)?;
        #[cfg(test)]
        crate::moving::crash_tests::gate("after-hinge");
        if c.pair.is_some() {
            let Reply::Pair(pair) = collect(1)? else {
                return Err(invalid("missing committed sphere pair"));
            };
            c.spheres = pair.spheres;
            c.pair = Some(*pair);
            #[cfg(test)]
            crate::moving::crash_tests::gate("after-pair");
        } else {
            for i in 0..2 {
                if !includes(c.hinge.sample.mask, i) {
                    let Reply::Sphere(s) = collect(i + 1)? else {
                        return Err(invalid("missing sphere step"));
                    };
                    c.spheres[i] = s;
                    #[cfg(test)]
                    if i == 0 {
                        crate::moving::crash_tests::gate("after-sphere-A");
                    }
                }
            }
        }
        c.pending = None;
        c.tick = c
            .tick
            .checked_add(1)
            .ok_or(invalid("multi-sphere tick exhausted"))?;
        Ok(())
    }
    fn step_transaction(
        &mut self,
        mut step: impl FnMut(&mut Self, &mut Cut) -> Result<()>,
    ) -> Result<()> {
        let (epoch, tick) = (self.cut.epoch, self.cut.tick);
        let result = match self.try_transaction(&mut step) {
            Err(e @ ArticulationError::MultiSphereRecovery { .. })
                if self.reopen_attempts > 0 && transport_loss(&e) =>
            {
                self.reopen_committed(e).and_then(|()| {
                    self.check_interval(epoch, tick)?;
                    self.report_recovery(RecoveryPhase::Replaying, self.recovery_attempt());
                    // Replay only the original operation. Further exhaustion returns
                    // to the caller instead of creating another full-pool budget.
                    self.try_transaction(step)
                })
            }
            result => result,
        };
        // Only a failed numerical step and failed rollback, both due entirely to
        // transport loss, can issue authority. A publication error can be
        // ambiguous even if opening the path still returns the previous cut.
        if !self.healthy
            && matches!(&result, Err(e @ ArticulationError::MultiSphereRecovery { .. }) if transport_loss(e))
            && let Ok((spec, cut, digest)) = self.disk.read_identified()
            && self.matches_retained_cut(spec, &cut).unwrap_or(false)
            && let Ok(writer) = self.disk.writer_identity()
        {
            self.operator_fence = Some(FenceReceipt::verified(epoch, tick, digest, writer));
        }
        self.finish_recovery(result)
    }

    #[cold]
    fn matches_retained_cut(&self, spec: MultiSphereSpec, cut: &Cut) -> Result<bool> {
        Ok(worker::encode(&(spec, cut, &cut.pair))?
            == worker::encode(&(self.spec, &self.cut, &self.cut.pair))?)
    }

    #[cold]
    fn reopen_committed(&mut self, mut failure: ArticulationError) -> Result<()> {
        // Rollback exhausted before publication, all peers are gone, and this
        // object still owns the sole writer lease. Verify the durable receipt,
        // including both native graphs, before reinstalling the retained cut.
        let (spec, cut) = self.disk.read()?;
        if !self.matches_retained_cut(spec, &cut)? {
            return Err(invalid("published multi-sphere cut changed while fenced"));
        }
        for index in 0..self.reopen_attempts {
            self.report_recovery(RecoveryPhase::Reopening, index + 1);
            std::thread::sleep(std::time::Duration::from_millis(100 << index.min(3)));
            self.reopen_attempts_used = self.reopen_attempts_used.saturating_add(1);
            match self.restore_committed() {
                Ok(()) => {
                    self.healthy = true;
                    return Ok(());
                }
                Err(e) => {
                    // Even a partly bootstrapped worker owns a lease. Every
                    // subsequent attempt, and the terminal fence, starts empty.
                    self.slots.fill_with(|| None);
                    let retry = transport_loss(&e);
                    failure = recovery_failure(failure, e);
                    if !retry {
                        break;
                    }
                }
            }
        }
        Err(failure)
    }

    pub fn advance(&mut self) -> Result<()> {
        self.step_transaction(Self::advance_cut)
    }
    fn validate_effort(&self, joint: u64, effort: f64) -> Result<()> {
        if !effort.is_finite() || joint != self.spec.hinge.joint_id {
            return Err(invalid("invalid multi-sphere effort"));
        }
        Ok(())
    }
    fn add_effort(&mut self, c: &mut Cut, effort: f64) -> Result<()> {
        let Reply::Hinge(h) = self.call(c.owners[0], c.tick, Command::Torque(effort))? else {
            return Err(invalid("missing hinge torque"));
        };
        c.accept_hinge(*h)?;
        c.pending = Some(c.pending.unwrap_or(0.) + effort);
        Ok(())
    }
    pub fn queue_effort(&mut self, joint: u64, effort: f64) -> Result<()> {
        self.validate_effort(joint, effort)?;
        self.transact(|p, c| p.add_effort(c, effort))
    }
    /// Add a signed torque to any pending effort and commit one 1 ms interval.
    /// The routing decision and solve use the accumulated effort. The only
    /// published receipt is the advanced state with its effort consumed.
    /// Following a publication failure, reopen and inspect the committed tick
    /// before issuing the command again.
    pub fn advance_with_effort(&mut self, joint: u64, effort: f64) -> Result<()> {
        self.validate_effort(joint, effort)?;
        self.step_transaction(|p, c| {
            p.add_effort(c, effort)?;
            #[cfg(test)]
            crate::moving::crash_tests::gate("atomic-effort-accepted");
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
        self.move_component(self.owners()[0], destination)
    }
    /// Move the independently coupled sphere pair to a new worker. Moving onto
    /// the hinge's worker swaps their leases in the same durable transaction.
    pub fn transfer_pair(&mut self, destination: u8) -> Result<()> {
        if !pair_owned(self.owners()) {
            return Err(invalid("no independently owned sphere pair"));
        }
        self.move_component(self.owners()[1], destination)
    }
    fn move_component(&mut self, old: u8, destination: u8) -> Result<()> {
        if destination >= self.width() || destination == old {
            return Err(invalid("expected a distinct multi-sphere destination"));
        }
        self.transact(|p, c| {
            let owners = c.owners.map(|s| {
                if s == old {
                    destination
                } else if s == destination {
                    old
                } else {
                    s
                }
            });
            p.reassign(c, owners, c.hinge.clone(), c.pair.clone())
        })
    }
    pub fn replace_worker(&mut self, slot: u8) -> Result<()> {
        if !self.healthy || slot >= self.width() {
            return Err(invalid("unknown multi-sphere worker"));
        }
        self.slots[slot as usize].take();
        let result = self.rollback();
        self.finish_recovery(result)
    }
    pub fn reset(&mut self) -> Result<()> {
        self.transact(|p, c| {
            for (i, &slot) in c.owners.iter().enumerate() {
                if !c.owners[..i].contains(&slot) {
                    p.call(slot, c.tick, Command::Fence)?;
                }
            }
            let Reply::Hinge(h) = p.call(0, 0, Command::Install(Seed::Hinge(None)))? else {
                return Err(invalid("missing reset hinge"));
            };
            p.call(0, 0, Command::Fence)?;
            c.tick = 0;
            c.owners = initial_owners(h.sample.mask);
            c.hinge = *h;
            c.pair = None;
            c.spheres = std::array::from_fn(|i| p.spec.single(i).initial_sphere());
            c.epoch = c
                .epoch
                .checked_add(1)
                .ok_or(invalid("multi-sphere epoch exhausted"))?;
            c.generation = c
                .generation
                .checked_add(1)
                .ok_or(invalid("multi-sphere generation exhausted"))?;
            c.pending = None;
            p.install_exact(c)
        })
    }
}
fn spheres_may_meet(c: &Cut, spec: &MultiSphereSpec) -> bool {
    let dr = DVec3::from(c.spheres[0].position) - DVec3::from(c.spheres[1].position);
    let dv = (DVec3::from(c.spheres[0].velocity) - DVec3::from(c.spheres[1].velocity)) * 0.001;
    let length = dv.length_squared();
    let t = if length == 0. {
        0.
    } else {
        (-dr.dot(dv) / length).clamp(0., 1.)
    };
    let roundoff = 32768.
        * f64::EPSILON
        * (1.
            + DVec3::from(c.spheres[0].position).length()
            + DVec3::from(c.spheres[1].position).length()
            + dv.length());
    let margin = if c.hinge.sample.mask == 3 || c.pair.is_some() {
        0.075
    } else {
        0.025
    };
    let gap = (dr + t * dv).length() - spec.spheres[0].radius - spec.spheres[1].radius - roundoff;
    !gap.is_finite()
        || gap <= margin
        || c.pair
            .as_ref()
            .map_or(c.hinge.sample.sphere_reaction.active, |p| p.reaction.active)
}

fn initial_owners(mask: u8) -> [u8; 3] {
    [
        0,
        if includes(mask, 0) { 0 } else { 1 },
        if includes(mask, 1) { 0 } else { 2 },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reopen_requires_transport_loss_in_the_entire_rollback_history() {
        let lost = || ArticulationError::from(io::Error::from(io::ErrorKind::BrokenPipe));
        let timed_out = || io::Error::from(io::ErrorKind::TimedOut);
        assert!(transport_loss(&recovery_failure(
            lost(),
            recovery_failure(timed_out().into(), lost())
        )));
        for error in [
            invalid("invalid receipt"),
            io::Error::from(io::ErrorKind::InvalidData).into(),
            ArticulationError::Publication(timed_out()),
            ArticulationError::StaleInterval {
                expected_epoch: 0,
                expected_tick: 0,
                epoch: 0,
                tick: 1,
            },
        ] {
            assert!(!transport_loss(&recovery_failure(error, lost())));
        }
        assert!(!transport_loss(&recovery_failure(
            lost(),
            recovery_failure(lost(), invalid("nonmatching native graph"))
        )));
    }
}
