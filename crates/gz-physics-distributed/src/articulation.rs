// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Single-host, durable ownership of a floating revolute tree and its optional static ground.
//! Each successful mutation publishes one complete recoverable cut.
pub(crate) mod native;
pub use native::{
    ArticulationGroundSpec, ArticulationLinkSpec, ArticulationLinkState, ArticulationObservation,
    ArticulationSpec, GroundReaction, MAX_ARTICULATION_IMAGE, NativeArticulation, NativeError,
};
pub(crate) mod ledger;

use crate::transport::{
    TransportError,
    hinge::{HingeCommand, HingePeer, HingeReply, HingeRequest},
};
use ledger::{Cut, Ledger};
use sha2::{Digest, Sha256};
use std::{
    io,
    path::{Path, PathBuf},
};

#[derive(Debug, thiserror::Error)]
pub enum ArticulationError {
    #[error(transparent)]
    Io(#[from] io::Error),
    #[error("{operation}; restoring the committed multi-sphere barrier failed: {recovery}")]
    MultiSphereRecovery {
        operation: Box<Self>,
        #[source]
        recovery: Box<Self>,
    },
    #[error(transparent)]
    Native(#[from] NativeError),
    #[error(transparent)]
    Material(#[from] gz_physics_xpbd::XpbdError),
    #[error(transparent)]
    Transport(#[from] TransportError),
    #[error("distributed articulation: {0}")]
    Invalid(&'static str),
    #[error(
        "distributed articulation: expected epoch {expected_epoch} tick {expected_tick}, committed epoch {epoch} tick {tick}"
    )]
    StaleInterval {
        expected_epoch: u64,
        expected_tick: u64,
        epoch: u64,
        tick: u64,
    },
    #[error(
        "adaptive grasp fleet epoch {epoch} tick {tick}: replay history exhausted after {recorded_intervals} intervals and {recorded_events} inputs"
    )]
    GraspReplayExhausted {
        epoch: u64,
        tick: u64,
        recorded_intervals: u32,
        recorded_events: u32,
    },
    #[error(
        "adaptive grasp fleet epoch {epoch} tick {tick}: ordinary solver first disagreed with accepted event {first_event} at physical tick {first_physical_tick}"
    )]
    GraspReplayDiverged {
        epoch: u64,
        tick: u64,
        first_event: u32,
        first_physical_tick: u64,
    },
    #[error(
        "adaptive grasp fleet epoch {epoch} tick {tick}: model {model} cannot export cache-drained native motion"
    )]
    GraspMergeUnavailable { epoch: u64, tick: u64, model: u64 },
    #[error(
        "adaptive grasp fleet epoch {epoch} tick {tick}: model {model} cannot export its warm contact graph"
    )]
    GraspWarmMemberUnavailable { epoch: u64, tick: u64, model: u64 },
    #[error(
        "adaptive grasp fleet epoch {epoch} tick {tick}: the first merged interval changed the independently verified continuation for model {model} before any foreign contact"
    )]
    GraspWarmContinuationChanged { epoch: u64, tick: u64, model: u64 },
    /// The independent-world fleet cannot accept a step whose conservative envelopes may meet.
    #[error(
        "distributed grasp fleet epoch {epoch} tick {tick}: predicted collision bounds for model {first_model} body {first_body} intersect model {second_model} body {second_body}"
    )]
    PossibleCrossAssemblyContact {
        epoch: u64,
        tick: u64,
        first_model: u64,
        first_body: u64,
        second_model: u64,
        second_body: u64,
    },
    #[error("distributed articulation: publication failed; reopen the session: {0}")]
    Publication(io::Error),
}

type Result<T> = std::result::Result<T, ArticulationError>;

struct Slot {
    peer: HingePeer,
    sequence: u64,
}

/// Validate a caller's interval under the pool's exclusive session lease.
pub(crate) fn check_interval(
    healthy: bool,
    expected_epoch: u64,
    expected_tick: u64,
    epoch: u64,
    tick: u64,
) -> Result<()> {
    if !healthy {
        return Err(ArticulationError::Invalid("reopen the fenced session"));
    }
    if (epoch, tick) != (expected_epoch, expected_tick) {
        return Err(ArticulationError::StaleInterval {
            expected_epoch,
            expected_tick,
            epoch,
            tick,
        });
    }
    Ok(())
}

/// The coordinator is the exclusive writer of its trusted local session directory.
/// A successful operation is durable before it is returned to the caller. On a
/// coordinator crash, resume and inspect the published tick/owner before retrying.
pub struct ArticulationPool {
    slots: Vec<Option<Slot>>,
    executable: PathBuf,
    transport_run: u64,
    cut: Cut,
    ledger: Ledger,
    healthy: bool,
}

impl ArticulationPool {
    fn lock(directory: &Path, create: bool) -> Result<Ledger> {
        let ledger = Ledger::lock(directory, create)?;
        if ledger.other_archive_exists(ledger::HINGE_FILE)? {
            return Err(ArticulationError::Invalid(
                "session belongs to a moving encounter",
            ));
        }
        Ok(ledger)
    }

    /// Create a new session on worker zero. Slots are numbered from zero.
    pub fn create(
        executable: &Path,
        directory: &Path,
        width: u8,
        spec: ArticulationSpec,
    ) -> Result<Self> {
        let ledger = Self::lock(directory, true)?;
        if ledger.exists()? {
            return Err(ArticulationError::Invalid(
                "session already has a committed cut",
            ));
        }
        Self::create_locked(executable, ledger, width, spec)
    }

    /// Open an authored scene under the exclusive session lock. An omitted width
    /// adopts the committed width, or two on creation.
    pub fn open_scene(
        executable: &Path,
        directory: &Path,
        width: Option<u8>,
        expected: ArticulationSpec,
    ) -> Result<Self> {
        let ledger = Self::lock(directory, true)?;
        if !ledger.exists()? {
            return Self::create_locked(executable, ledger, width.unwrap_or(2), expected);
        }
        let cut = ledger.read()?;
        // Use the exact native recipe representation (including signed zero).
        if width.is_some_and(|n| n != cut.width)
            || expected
                .verify_image(cut.observation.tick, &cut.image)
                .is_err()
        {
            return Err(ArticulationError::Invalid(
                "authored hinge recipe or worker count differs from session",
            ));
        }
        Self::start(executable, ledger, cut)
    }

    fn create_locked(
        executable: &Path,
        ledger: Ledger,
        width: u8,
        spec: ArticulationSpec,
    ) -> Result<Self> {
        if !(2..=8).contains(&width) {
            return Err(ArticulationError::Invalid(
                "expected two to eight worker slots",
            ));
        }
        let native = NativeArticulation::new(&spec)?;
        let cut = Cut {
            width,
            owner: 0,
            generation: 0,
            epoch: 0,
            spec,
            observation: native.observe(),
            pending_effort: native.pending_joint_effort(),
            image: native.capture()?,
        };
        let pool = Self::start(executable, ledger, cut)?;
        pool.ledger.publish(&pool.cut)?;
        Ok(pool)
    }

    /// Recover the last durable cut after the previous coordinator has exited.
    pub fn resume(executable: &Path, directory: &Path) -> Result<Self> {
        let ledger = Self::lock(directory, false)?;
        let cut = ledger.read()?;
        Self::start(executable, ledger, cut)
    }

    fn start(executable: &Path, ledger: Ledger, cut: Cut) -> Result<Self> {
        let mut pool = Self {
            executable: executable.canonicalize()?,
            transport_run: uuid::Uuid::new_v4().as_u128() as u64,
            slots: (0..cut.width).map(|_| None).collect(),
            ledger,
            cut,
            healthy: true,
        };
        // Acquire every process lifetime lease before granting any writing authority.
        pool.start_workers()?;
        Ok(pool)
    }

    fn start_workers(&mut self) -> Result<()> {
        for id in 0..self.cut.width {
            self.spawn_slot(id)?;
        }
        self.restore_owner()
    }

    pub fn epoch(&self) -> u64 {
        self.cut.epoch
    }

    pub fn spec(&self) -> &ArticulationSpec {
        &self.cut.spec
    }
    pub fn observation(&self) -> &ArticulationObservation {
        &self.cut.observation
    }
    /// Already-admitted effort on the committed native hinge, including Some(0).
    pub fn pending_joint_effort(&self) -> Option<f64> {
        self.cut.pending_effort
    }
    pub fn owner(&self) -> u8 {
        self.cut.owner
    }
    pub fn generation(&self) -> u64 {
        self.cut.generation
    }
    pub fn width(&self) -> u8 {
        self.cut.width
    }
    pub fn image(&self) -> &[u8] {
        &self.cut.image
    }
    pub fn worker_pid(&self, worker: u8) -> Option<u32> {
        Some(self.slots.get(usize::from(worker))?.as_ref()?.peer.pid())
    }

    /// Inject one lost managed-worker reply after execution, for supervision diagnostics.
    pub fn drop_next_acknowledgement(&mut self, worker: u8) -> Result<()> {
        self.ready()?;
        let slot = self
            .slots
            .get_mut(usize::from(worker))
            .and_then(Option::as_mut)
            .ok_or(ArticulationError::Invalid("invalid worker slot"))?;
        slot.peer.withhold_next_reply();
        Ok(())
    }

    fn spawn_slot(&mut self, id: u8) -> Result<()> {
        let lock = self.ledger.slot(id);
        let peer = HingePeer::spawn(
            &self.executable,
            self.transport_run,
            id,
            &self.cut.spec,
            &lock,
        )?;
        self.slots[usize::from(id)] = Some(Slot { peer, sequence: 0 });
        Ok(())
    }

    fn request(&mut self, id: u8, generation: u64, command: HingeCommand) -> Result<HingeReply> {
        let slot = self.slots[usize::from(id)]
            .as_mut()
            .ok_or(ArticulationError::Invalid("unavailable slot"))?;
        let sequence = slot
            .sequence
            .checked_add(1)
            .ok_or(ArticulationError::Invalid("worker sequence exhausted"))?;
        let request = HingeRequest {
            run: self.transport_run,
            worker: id,
            sequence,
            generation,
            tick: self.cut.observation.tick,
            command,
        };
        let reply = slot.peer.call(&request)?;
        slot.sequence = sequence;
        Ok(reply)
    }

    fn check_installed(&self, reply: &HingeReply) -> Result<()> {
        if reply.state.as_ref() != Some(&self.cut.observation) || reply.image != self.cut.image {
            return Err(ArticulationError::Invalid(
                "worker installed a different articulated state",
            ));
        }
        Ok(())
    }
    fn stage(&mut self, id: u8, generation: u64) -> Result<()> {
        let reply = self.request(
            id,
            generation,
            HingeCommand::Stage {
                image: self.cut.image.clone(),
            },
        )?;
        self.check_installed(&reply)
    }
    fn activate(&mut self, id: u8, generation: u64) -> Result<()> {
        let digest = Sha256::digest(&self.cut.image).into();
        let reply = self.request(id, generation, HingeCommand::Activate { digest })?;
        self.check_installed(&reply)
    }
    fn restore_owner(&mut self) -> Result<()> {
        self.stage(self.cut.owner, self.cut.generation)?;
        self.activate(self.cut.owner, self.cut.generation)
    }

    fn ready(&self) -> Result<()> {
        if self.healthy {
            Ok(())
        } else {
            Err(ArticulationError::Invalid("reopen the fenced session"))
        }
    }
    fn stop(&mut self, id: u8) -> Result<()> {
        if let Some(mut slot) = self.slots[usize::from(id)].take() {
            slot.peer.kill()?;
        }
        Ok(())
    }
    fn replace(&mut self, id: u8) -> Result<()> {
        self.stop(id)?;
        self.spawn_slot(id)?;
        if id == self.cut.owner {
            self.restore_owner()?;
        }
        Ok(())
    }

    /// Reap and recreate a slot from the last committed cut. A standby starts empty.
    pub fn replace_worker(&mut self, worker: u8) -> Result<()> {
        self.ready()?;
        if worker >= self.cut.width {
            return Err(ArticulationError::Invalid("invalid worker slot"));
        }
        self.healthy = false;
        self.replace(worker)?;
        self.healthy = true;
        Ok(())
    }

    fn commit(&mut self, cut: Cut) -> Result<()> {
        if let Err(error) = self.ledger.publish(&cut) {
            // rename may already have made a new cut visible. Stop every writer;
            // only reopening the file can establish the authoritative decision.
            self.healthy = false;
            self.slots.clear();
            return Err(ArticulationError::Publication(error));
        }
        #[cfg(test)]
        crash_tests::gate("after-sync");
        self.cut = cut;
        Ok(())
    }

    fn prepare_mutation(
        &mut self,
        command: HingeCommand,
        next_tick: u64,
        pending_effort: Option<f64>,
    ) -> Result<Cut> {
        let r = self.request(self.cut.owner, self.cut.generation, command)?;
        let state = r.state.ok_or(ArticulationError::Invalid(
            "missing articulated observation",
        ))?;
        if r.tick != next_tick
            || state.tick != next_tick
            || state.joint_id != self.cut.spec.joint_id
            || state
                .links
                .iter()
                .zip(&self.cut.spec.links)
                .any(|(s, l)| s.id != l.id)
            || r.image.is_empty()
            || r.image.len() > self.cut.spec.image_limit()
            || state.ground_reactions.is_some() != self.cut.spec.ground.is_some()
        {
            return Err(ArticulationError::Invalid("invalid articulated worker cut"));
        }
        self.cut.spec.verify_image(next_tick, &r.image)?;
        Ok(Cut {
            observation: state,
            image: r.image,
            pending_effort,
            spec: self.cut.spec,
            width: self.cut.width,
            owner: self.cut.owner,
            generation: self.cut.generation,
            epoch: self.cut.epoch,
        })
    }

    fn mutate(&mut self, mut prepare: impl FnMut(&mut Self) -> Result<Cut>) -> Result<()> {
        self.ready()?;
        let owner = self.cut.owner;
        for retry in [true, false] {
            match prepare(self) {
                Ok(next) => return self.commit(next),
                Err(error) => {
                    let transient = matches!(
                        error,
                        ArticulationError::Transport(
                            TransportError::Unavailable(_) | TransportError::Io(_)
                        )
                    );
                    self.replace_worker(owner)?;
                    if !retry || !transient {
                        return Err(error);
                    }
                }
            }
        }
        unreachable!()
    }

    fn validate_effort(&self, joint_id: u64, effort: f64) -> Result<()> {
        if joint_id != self.cut.spec.joint_id || !effort.is_finite() {
            return Err(ArticulationError::Invalid(
                "incorrect joint identity or nonfinite effort",
            ));
        }
        Ok(())
    }
    fn effort_cut(&mut self, joint_id: u64, effort: f64) -> Result<Cut> {
        self.prepare_mutation(
            HingeCommand::QueueEffort { joint_id, effort },
            self.cut.observation.tick,
            Some(self.cut.pending_effort.unwrap_or(0.) + effort),
        )
    }
    fn next_tick(&self) -> Result<u64> {
        self.cut
            .observation
            .tick
            .checked_add(1)
            .ok_or(ArticulationError::Invalid("tick exhausted"))
    }
    /// Accumulate a signed generalized torque for the next interval.
    pub fn queue_effort(&mut self, joint_id: u64, effort: f64) -> Result<()> {
        self.validate_effort(joint_id, effort)?;
        self.mutate(|p| p.effort_cut(joint_id, effort))
    }
    /// Advance the owner by one millisecond, including any admitted pending effort.
    pub fn advance(&mut self) -> Result<()> {
        let tick = self.next_tick()?;
        self.mutate(|p| p.prepare_mutation(HingeCommand::Advance, tick, None))
    }
    /// Add to the committed pending effort and publish the completed interval.
    /// A transport retry restores the committed native image before reissuing both commands.
    /// After a publication error, reopen and check the tick before resubmission.
    pub fn advance_with_effort(&mut self, joint_id: u64, effort: f64) -> Result<()> {
        self.validate_effort(joint_id, effort)?;
        let tick = self.next_tick()?;
        self.mutate(|p| {
            p.effort_cut(joint_id, effort)?;
            #[cfg(test)]
            crash_tests::gate("atomic-effort-accepted");
            let next = p.prepare_mutation(HingeCommand::Advance, tick, None)?;
            #[cfg(test)]
            crash_tests::gate("atomic-step-prepared");
            Ok(next)
        })
    }

    /// Validate a logical interval under the current session lease.
    pub fn check_interval(&self, expected_epoch: u64, expected_tick: u64) -> Result<()> {
        crate::articulation::check_interval(
            self.healthy,
            expected_epoch,
            expected_tick,
            self.cut.epoch,
            self.cut.observation.tick,
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
    /// Rebuild the authored tree with fresh private worker capabilities. A reset
    /// advances both the simulation epoch and ownership fence at the original pose.
    pub fn reset(&mut self) -> Result<()> {
        self.ready()?;
        let native = NativeArticulation::new(&self.cut.spec)?;
        let fresh = Cut {
            epoch: self
                .cut
                .epoch
                .checked_add(1)
                .ok_or(ArticulationError::Invalid("epoch exhausted"))?,
            generation: self
                .cut
                .generation
                .checked_add(1)
                .ok_or(ArticulationError::Invalid("generation exhausted"))?,
            pending_effort: native.pending_joint_effort(),
            image: native.capture()?,
            observation: native.observe(),
            ..self.cut.clone()
        };
        self.healthy = false;
        self.slots.iter_mut().for_each(|slot| {
            slot.take();
        });
        self.transport_run = uuid::Uuid::new_v4().as_u128() as u64;
        let old = std::mem::replace(&mut self.cut, fresh);
        if let Err(error) = self.start_workers() {
            self.slots.iter_mut().for_each(|slot| {
                slot.take();
            });
            self.cut = old;
            self.start_workers()?;
            self.healthy = true;
            return Err(error);
        }
        if let Err(error) = self.ledger.publish(&self.cut) {
            self.slots.clear();
            self.cut = old;
            return Err(ArticulationError::Publication(error));
        }
        self.healthy = true;
        Ok(())
    }

    /// Move both links and their joint at one physical barrier. Only a positively
    /// fenced source can be followed by destination activation.
    pub fn transfer(&mut self, destination: u8) -> Result<()> {
        self.ready()?;
        if destination >= self.cut.width || destination == self.cut.owner {
            return Err(ArticulationError::Invalid(
                "expected a different valid slot",
            ));
        }
        let source = self.cut.owner;
        let generation = self
            .cut
            .generation
            .checked_add(1)
            .ok_or(ArticulationError::Invalid("ownership generation exhausted"))?;
        let mut retry = true;
        loop {
            let attempt = (|| {
                self.stage(destination, generation)?;
                #[cfg(test)]
                crash_tests::gate("staged");
                let digest = Sha256::digest(&self.cut.image).into();
                self.request(source, self.cut.generation, HingeCommand::Fence { digest })?;
                #[cfg(test)]
                crash_tests::gate("fenced");
                self.activate(destination, generation)?;
                #[cfg(test)]
                crash_tests::gate("activated");
                Ok(())
            })();
            if let Err(error) = attempt {
                // The activation or fence reply may have been lost. Reap both
                // before reactivating the source under the still-committed cut.
                self.healthy = false;
                self.stop(destination)?;
                self.stop(source)?;
                self.spawn_slot(destination)?;
                self.spawn_slot(source)?;
                self.restore_owner()?;
                self.healthy = true;
                if retry
                    && matches!(
                        error,
                        ArticulationError::Transport(
                            TransportError::Unavailable(_) | TransportError::Io(_)
                        )
                    )
                {
                    retry = false;
                    continue;
                }
                return Err(error);
            }
            return self.commit(Cut {
                owner: destination,
                generation,
                ..self.cut.clone()
            });
        }
    }
}

#[cfg(test)]
mod crash_tests;
