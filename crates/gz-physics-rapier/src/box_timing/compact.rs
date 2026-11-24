// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Spend one guarded approach, one measured impact and one verified free tail when possible.
use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CompactDecline {
    Budget,
    AdditionalDynamicBody,
    AcceleratedFlight,
    SharedActions,
    Crossing,
    Approach,
    Pulse,
    NoLoad,
    Tail,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CompactImpactReceipt {
    pub approach_planned: bool,
    pub pulse_planned: bool,
    pub pulse_loaded: bool,
    /// The positive pulse was delivered by the registered instantaneous rigid point law.
    pub projected_load: bool,
    pub tail_checks: u32,
    pub complete: bool,
    pub decline: Option<CompactDecline>,
    pub tail_gap_pm: Option<u64>,
}
pub(crate) struct GuardedCall {
    pub pair: (ColliderHandle, ColliderHandle),
    pub motion: GuardedMotion,
    pub release: Option<ReleasedBoxStep>,
    pub from_point: bool,
    pub opposing: Option<opponent::OpponentGuard>,
}
impl Candidate {
    fn isolated_call(
        &self,
        e: &RapierBackend,
        h: Duration,
        depart: bool,
        ballistic: bool,
        from_corner: Option<u8>,
    ) -> Option<(GuardedCall, Option<u64>)> {
        let mut record = ReleasedBoxStep::default();
        let mut opposing = None;
        if !self.separated(
            e,
            h,
            depart.then_some(&mut record),
            &mut opposing,
            flight::SeparationCheck {
                cubic: true,
                swept: true,
                ballistic,
                from_corner,
            },
        ) {
            return None;
        }
        let f = &e.world.bodies[self.foot];
        let motion = GuardedMotion::prepare(self, e, h, true, ballistic);
        if self.inertia.is_some() && f.angvel() != Vector::ZERO && motion.rotation.is_none()
            || e.world.gravity * self.gravity_scale != Vector::ZERO && !motion.ballistic()
        {
            return None;
        }
        Some((
            GuardedCall {
                pair: (self.sole, self.deck),
                release: depart.then_some(ReleasedBoxStep {
                    integrated: motion.present(),
                    minimum_gap_pm: motion.present().then_some(record.minimum_gap_pm).flatten(),
                    ..record
                }),
                from_point: from_corner.is_some(),
                opposing,
                motion,
            },
            record.minimum_gap_pm,
        ))
    }
}
impl BoxTiming {
    fn departure_corner(&self, elapsed: Duration) -> Option<u8> {
        self.departure_witness
            .filter(|(cut, _)| *cut == elapsed)
            .map(|(_, corner)| corner)
    }
    pub(crate) fn departure_call(
        &self,
        e: &RapierBackend,
        elapsed: Duration,
        h: Duration,
    ) -> Option<GuardedCall> {
        let corner = self.departure_corner(elapsed)?;
        self.candidate
            .as_ref()?
            .isolated_call(e, h, true, self.ballistic, Some(corner))
            .map(|p| p.0)
    }
    pub(crate) fn enroll_compact(&mut self, e: &RapierBackend, other_action: bool) {
        let mut record = CompactImpactReceipt::default();
        if self.calls <= 3 {
            record.decline = Some(CompactDecline::Budget);
        } else if other_action || self.candidate.is_none() {
            record.decline = Some(CompactDecline::SharedActions);
        } else if !self.ballistic
            && self
                .candidate
                .as_ref()
                .is_some_and(|c| e.world.gravity * c.gravity_scale != Vector::ZERO)
        {
            // Native translation under acceleration is step-size dependent. The
            // reduced call budget is qualified for force-free translation only.
            record.decline = Some(CompactDecline::AcceleratedFlight);
        } else if let Some(c) = &self.candidate
            && e.world
                .bodies
                .iter()
                .any(|(h, b)| h != c.foot && b.is_dynamic() && b.is_enabled())
        {
            // Fewer global solves may also advance another free or constrained dynamic object.
            record.decline = Some(CompactDecline::AdditionalDynamicBody);
        }
        self.receipt.compact = Some(record);
    }
    pub(super) fn compact_next(
        &mut self,
        e: &RapierBackend,
        elapsed: u64,
        completed: u32,
        guarded: bool,
    ) -> Option<u64> {
        let from_corner = self.departure_corner(Duration::from_nanos(elapsed));
        let record = self.receipt.compact?;
        if record.decline.is_some() || (completed < 2 && self.receipt.decline.is_some()) {
            return None;
        }
        // With one remaining solve there is no work left to save. Preserve the
        // ordinary full-budget solver on that final boundary.
        if completed >= self.calls - 1 {
            self.receipt.compact.as_mut()?.decline = Some(CompactDecline::Tail);
            return None;
        }
        let c = self.candidate.as_ref()?;
        let remaining = self.duration.as_nanos() as u64 - elapsed;
        // A rejected later stage must leave at least one nanosecond for *every*
        // unused full-budget solve. The normal interval allocator handles that fallback.
        let max_first = remaining - u64::from(self.calls - completed - 1);
        let planned = match completed {
            0 | 1 => {
                let mut trial = self.receipt.clone();
                let limited = if completed == 0 { 3 } else { 2 };
                let h = c
                    .proposed(
                        e,
                        remaining,
                        limited,
                        &mut trial,
                        TimingCut {
                            cubic_elapsed: Some(elapsed),
                            swept: true,
                            pulse: self.pulse.map(|(mode, n)| (mode, n, guarded)),
                        },
                    )
                    .ok();
                if completed == 0 {
                    if let Some(t) = h.filter(|&t| t <= max_first && trial.crossings.len() == 1) {
                        if let Some((call, _)) =
                            c.isolated_call(e, Duration::from_nanos(t), false, self.ballistic, None)
                        {
                            self.cached = Some(call);
                            trial.compact.as_mut()?.approach_planned = true;
                            self.receipt = trial;
                            Ok(t)
                        } else {
                            Err(CompactDecline::Approach)
                        }
                    } else {
                        Err(CompactDecline::Crossing)
                    }
                } else if let Some(t) = h.filter(|&t| {
                    guarded
                        && t <= max_first
                        && trial
                            .pulses
                            .last()
                            .is_some_and(|p| p.cut_ns == elapsed && p.applied_ns == Some(t))
                }) {
                    trial.compact.as_mut()?.pulse_planned = true;
                    self.receipt = trial;
                    Ok(t)
                } else {
                    Err(CompactDecline::Pulse)
                }
            }
            _ => {
                let loaded = record.pulse_loaded
                    || (completed == 2
                        && e.world
                            .narrow_phase
                            .contact_pair(c.sole, c.deck)
                            .is_some_and(|p| p.total_impulse_magnitude() > 0.));
                let receipt = self.receipt.compact.as_mut()?;
                receipt.pulse_loaded = loaded;
                receipt.tail_checks += 1;
                if !loaded {
                    Err(CompactDecline::NoLoad)
                } else if let Some((call, gap)) = c.isolated_call(
                    e,
                    Duration::from_nanos(remaining),
                    true,
                    self.ballistic,
                    from_corner,
                ) {
                    self.cached = Some(call);
                    let receipt = self.receipt.compact.as_mut()?;
                    receipt.tail_gap_pm = gap;
                    receipt.complete = true;
                    Ok(remaining)
                } else {
                    // The just-loaded corner can still be touching. Spend the next
                    // original native solve, then reconsider its measured outgoing state.
                    return None;
                }
            }
        };
        match planned {
            Ok(h) => Some(h),
            Err(why) => {
                self.receipt.compact.as_mut()?.decline = Some(why);
                None
            }
        }
    }
}
