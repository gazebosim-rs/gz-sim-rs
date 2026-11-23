// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read-only status from the physics coordinator and admission of a boundary Pause.
use super::*;

pub(super) const FENCED_MESSAGE: &str = "Worker session fenced. Inspect the committed checkpoint.";

pub(crate) struct Recovery {
    #[cfg(feature = "distributed-physics")]
    monitor: gz_sim_scene::distributed_hinge::RecoveryMonitor,
    pub(super) restored: Option<String>,
    /// A Pause admitted during recovery survives viewer disconnection.
    pub(super) pause_pending: AtomicBool,
    pub(super) terminal_sent: AtomicU64,
    /// The actual server mode at the boundary entering prepare_io/tick.
    pub(super) step_paused: AtomicBool,
}

#[derive(Clone, Copy, PartialEq, Eq, Serialize)]
pub(super) struct Status {
    version: u8,
    kind: &'static str,
    pub(super) phase: &'static str,
    pub(super) epoch: u64,
    pub(super) tick: u64,
    attempt: u8,
    budget: u8,
    pub(super) pause_pending: bool,
    #[serde(skip)]
    paused: bool,
}

impl Status {
    pub(super) fn refusal(self, id: u64, reason: &'static str) -> Ack {
        let elapsed = Duration::from_millis(self.tick);
        Ack {
            version: 1,
            kind: "ack",
            request_id: id,
            ok: false,
            epoch: self.epoch,
            seconds: elapsed.as_secs(),
            nanoseconds: elapsed.subsec_nanos(),
            paused: self.paused,
            error: Some(reason),
        }
    }

    pub(super) fn idle(self) -> Self {
        Self {
            phase: "idle",
            pause_pending: false,
            ..self
        }
    }
}

impl Recovery {
    #[cfg(feature = "distributed-physics")]
    pub(crate) fn subscribe(
        sim: &mut Simulation,
        supervised: bool,
        restored: Option<String>,
    ) -> Option<Arc<Self>> {
        use gz_physics::PhysicsBackend;
        use gz_sim_scene::distributed_hinge::DistributedMultiSphereBackend;
        let paused = sim.is_paused();
        let mut backend = sim.world_mut().get_resource_mut::<PhysicsBackend>()?;
        let pool = backend.downcast_mut::<DistributedMultiSphereBackend>()?;
        (supervised || pool.pool().reopen_attempt_budget() > 0).then(|| {
            Arc::new(Self {
                monitor: pool.subscribe_recovery(),
                restored,
                pause_pending: AtomicBool::new(false),
                terminal_sent: AtomicU64::new(0),
                step_paused: AtomicBool::new(paused),
            })
        })
    }

    #[cfg(not(feature = "distributed-physics"))]
    pub(crate) fn subscribe(_: &mut Simulation, _: bool, _: Option<String>) -> Option<Arc<Self>> {
        None
    }

    #[cfg(feature = "distributed-physics")]
    pub(super) fn status(&self) -> Option<Status> {
        use gz_sim_scene::distributed_hinge::RecoveryPhase;
        // An observer can be descheduled across a completed recovery and the
        // next control boundary. Pair the cut with its actual server mode.
        let (status, paused) = loop {
            let status = self.monitor.snapshot()?;
            let paused = self.step_paused.load(Ordering::Acquire);
            if self.monitor.snapshot() == Some(status) {
                break (status, paused);
            }
        };
        Some(Status {
            version: 1,
            kind: "recovery_status",
            phase: match status.phase {
                RecoveryPhase::Restoring => "restoring",
                RecoveryPhase::Reopening => "reopening",
                RecoveryPhase::Replaying => "replaying",
                RecoveryPhase::Fenced => "fenced",
            },
            paused,
            epoch: status.epoch,
            tick: status.tick,
            attempt: status.attempt,
            budget: status.budget,
            pause_pending: status.phase != RecoveryPhase::Fenced
                && self.pause_pending.load(Ordering::Acquire),
        })
    }

    #[cfg(not(feature = "distributed-physics"))]
    pub(super) fn status(&self) -> Option<Status> {
        None
    }
}

/// Keep admission separate from the actual acknowledgement at the next boundary.
pub(super) fn admit(
    subscribed: bool,
    pending: &AtomicBool,
    status: Option<Status>,
    action: &Action,
) -> Result<Admission, (Status, &'static str)> {
    // An unnegotiated viewer retains ordinary controls, but reconnecting
    // cannot cancel a boundary Pause already admitted by a previous peer.
    if !subscribed && !pending.load(Ordering::Acquire) {
        return Ok(Admission::Boundary);
    }
    if let Some(active) = status {
        if active.phase == "fenced" {
            return Err((active, FENCED_MESSAGE));
        }
        if !matches!(action, Action::Pause) {
            return Err((active, "only Pause can be queued during worker recovery"));
        }
        if pending.swap(true, Ordering::AcqRel) {
            return Err((active, "Pause already queued for the next boundary"));
        }
        return Ok(Admission::RecoveryPause);
    }
    // A worker can be unreachable while its RPC is still detecting the loss.
    // Admit the Pause at this point so the first recovery frame includes it.
    Ok(match action {
        Action::Pause if subscribed && !pending.swap(true, Ordering::AcqRel) => {
            Admission::RecoveryPause
        }
        Action::Resume if subscribed => Admission::AppliedResume,
        _ => Admission::Boundary,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovery_pause_survives_detection_window_and_conflicting_requests() {
        let pending = AtomicBool::new(false);
        let status = Status {
            version: 1,
            kind: "recovery_status",
            phase: "restoring",
            epoch: 0,
            tick: 150,
            attempt: 0,
            budget: 3,
            pause_pending: false,
            paused: false,
        };
        assert!(matches!(
            admit(false, &pending, Some(status), &Action::Reset),
            Ok(Admission::Boundary)
        ));
        assert!(matches!(
            admit(true, &pending, None, &Action::Resume),
            Ok(Admission::AppliedResume)
        ));
        assert!(matches!(
            admit(true, &pending, None, &Action::Pause),
            Ok(Admission::RecoveryPause)
        ));
        for subscribed in [false, true] {
            for action in [Action::Reset, Action::Resume, Action::Pause] {
                assert!(admit(subscribed, &pending, Some(status), &action).is_err());
                assert!(pending.load(Ordering::Acquire));
            }
        }
        pending.store(false, Ordering::Release);
        assert!(matches!(
            admit(true, &pending, Some(status), &Action::Pause),
            Ok(Admission::RecoveryPause)
        ));
        let fenced = Status {
            phase: "fenced",
            ..status
        };
        for action in [Action::Pause, Action::Reset, Action::Resume] {
            assert!(admit(true, &pending, Some(fenced), &action).is_err());
        }
    }
    #[test]
    fn rejected_controls_preserve_the_interrupted_server_mode() {
        for paused in [false, true] {
            let status = Status {
                version: 1,
                kind: "recovery_status",
                phase: "fenced",
                epoch: 3,
                tick: 1150,
                attempt: 1,
                budget: 1,
                pause_pending: false,
                paused,
            };
            let ack = status.refusal(7, "session fenced");
            assert!(!ack.ok);
            assert_eq!(ack.paused, paused);
            assert_eq!(
                (ack.request_id, ack.epoch, ack.seconds, ack.nanoseconds),
                (7, 3, 1, 150_000_000)
            );
            assert!(ack.error.is_some());
            assert!(
                serde_json::to_value(status)
                    .unwrap()
                    .get("paused")
                    .is_none()
            );
        }
    }
}
