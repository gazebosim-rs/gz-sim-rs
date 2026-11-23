// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optional periodic sampling and one measured transfer inside an ordinary advancing barrier.
mod cadence;
use super::routing::ProspectiveGroups;
use super::{PoolBalanceDecision, PoolBalancePolicy, PoolLoadWindow, WorkerPool};
use crate::{coordinator::Error, measured, transport::TransportError};
use cadence::IdleCadence;
use serde::{Deserialize, Serialize};
use std::{num::NonZeroU16, str::FromStr, time::Duration};

// Background diagnostics run on local processes and may be skipped. The
// authoritative solve and explicit user probes retain their ordinary deadlines.
const ADVISORY_TIMEOUT: Duration = Duration::from_millis(25);

/// Public tick counts and the expected full time for one warm transfer.
/// A sample starts at a multiple of the period and spans consecutive steps.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PeriodicPlacement {
    pub sample_ticks: NonZeroU16,
    pub period_ticks: NonZeroU16,
    pub horizon_ticks: NonZeroU16,
    pub handoff_us: u32,
    /// Maximum interval between samples of unchanged unprofitable work, in periods.
    #[serde(default = "one", skip_serializing_if = "is_one")]
    pub max_idle_periods: NonZeroU16,
}
fn one() -> NonZeroU16 {
    NonZeroU16::MIN
}
fn is_one(n: &NonZeroU16) -> bool {
    *n == NonZeroU16::MIN
}
impl PeriodicPlacement {
    pub fn validate(self) -> Result<(), &'static str> {
        if self.sample_ticks > self.period_ticks {
            Err("measurement window exceeds placement period")
        } else {
            Ok(())
        }
    }
}
impl FromStr for PeriodicPlacement {
    type Err = &'static str;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut fields = s.split(',');
        let mut ticks = || {
            fields
                .next()
                .and_then(|x| x.parse::<NonZeroU16>().ok())
                .ok_or("expected positive sample, period and horizon ticks")
        };
        let config = Self {
            sample_ticks: ticks()?,
            period_ticks: ticks()?,
            horizon_ticks: ticks()?,
            handoff_us: fields
                .next()
                .and_then(|x| x.parse().ok())
                .ok_or("expected handoff time in microseconds")?,
            max_idle_periods: fields
                .next()
                .map(str::parse::<NonZeroU16>)
                .transpose()
                .map_err(|_| "expected positive maximum idle periods")?
                .unwrap_or(NonZeroU16::MIN),
        };
        if fields.next().is_some() {
            return Err("expected SAMPLE,PERIOD,HORIZON,HANDOFF_US[,MAX_IDLE_PERIODS]");
        }
        config.validate()?;
        Ok(config)
    }
}

/// Diagnostics for this coordinator incarnation. Placement is recorded only
/// after both the transfer and the physical step publish together.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct PlacementStatus {
    pub windows_started: u64,
    /// Scheduled opportunities with no possible load-reducing whole-island move.
    pub windows_ineligible: u64,
    pub windows_deferred: u64,
    /// A change in measured-work inputs caused sampling before the idle deadline.
    pub windows_woken: u64,
    pub windows_completed: u64,
    pub windows_interrupted: u64,
    pub unavailable_probes: u64,
    pub committed_moves: u64,
    pub last_move: Option<(u64, PoolBalanceDecision)>,
}

pub(super) struct PlacementSampler {
    config: PeriodicPlacement,
    window: Option<(u64, PoolLoadWindow)>,
    last_poll: Option<u64>,
    idle: Option<IdleCadence>,
    status: PlacementStatus,
}

impl WorkerPool {
    /// Enable or replace the local timing policy. New sessions and restored
    /// coordinators start with fresh measurements. Raw pools default to fixed placement.
    pub fn set_periodic_placement(
        &mut self,
        config: Option<PeriodicPlacement>,
    ) -> Result<(), Error> {
        if self.placement_replay.attempt.is_some()
            || self.pending_step.is_some()
            || self.dirty.iter().any(|&d| d)
        {
            return Err(Error::Pending);
        }
        if let Some(c) = config {
            c.validate().map_err(Error::Invalid)?;
        }
        self.periodic = config.map(|config| {
            Box::new(PlacementSampler {
                config,
                window: None,
                last_poll: None,
                idle: None,
                status: Default::default(),
            })
        });
        Ok(())
    }
    pub fn placement_status(&self) -> Option<&PlacementStatus> {
        self.periodic.as_ref().map(|s| &s.status)
    }
    pub(super) fn sampled_move(
        &mut self,
        plan: &ProspectiveGroups,
        mandatory: bool,
    ) -> Result<Option<PoolBalanceDecision>, Error> {
        let Some(mut sampler) = self.periodic.take() else {
            return Ok(None);
        };
        let result = sampler.poll(self, plan, mandatory);
        self.periodic = Some(sampler);
        result
    }
    // Work measured across an externally selected barrier cannot authorize a
    // later automatic handoff. No timing request is issued while replaying.
    pub(super) fn interrupt_periodic_window(&mut self) {
        if let Some(sampler) = &mut self.periodic {
            if sampler.window.take().is_some() {
                sampler.status.windows_interrupted += 1;
            }
            sampler.idle = None;
        }
    }
    pub(super) fn record_sampled_move(&mut self, decision: Option<PoolBalanceDecision>) {
        if let (Some(decision), Some(sampler)) = (decision, &mut self.periodic) {
            sampler.status.committed_moves += 1;
            sampler.status.last_move = Some((self.frame.tick, decision));
        }
    }
}
fn unavailable(error: &Error) -> bool {
    matches!(
        error,
        Error::Transport(TransportError::Unavailable(_) | TransportError::Io(_))
    )
}
impl PlacementSampler {
    fn poll(
        &mut self,
        pool: &mut WorkerPool,
        plan: &ProspectiveGroups,
        mandatory: bool,
    ) -> Result<Option<PoolBalanceDecision>, Error> {
        let tick = pool.frame.tick;
        if self.last_poll == Some(tick) {
            return Ok(None);
        }
        self.last_poll = Some(tick);
        let mut decision = None;
        // A window spanning a real change can still measure work, but its
        // average cannot justify delaying the next fresh measurement.
        if mandatory
            || (self.window.is_some() && self.idle.as_ref().is_some_and(|s| !s.unchanged(pool)))
        {
            self.idle = None;
        }
        if self.window.as_ref().is_some_and(|(end, _)| *end <= tick) {
            let (end, window) = self.window.take().unwrap();
            let previous_idle = self.idle.take();
            match measured("placement_read_start", "placement_read_end", || {
                pool.try_finish_load_sample(window, ADVISORY_TIMEOUT)
            }) {
                Ok(Some(sample)) => {
                    self.status.windows_completed += 1;
                    // A sampled move is batched with the final solve, which adds
                    // a fourth source command. Mandatory collision routing goes first.
                    if !mandatory && pool.limit >= 4 {
                        let policy = PoolBalancePolicy {
                            horizon: self.config.horizon_ticks,
                            handoff_cost: Duration::from_micros(u64::from(self.config.handoff_us)),
                        };
                        decision = measured("placement_plan_start", "placement_plan_end", || {
                            pool.plan_rebalance_for(&sample, policy, plan)
                        })?;
                        if decision.is_none() {
                            self.idle = previous_idle.map(|state| {
                                state.after_stable_sample(
                                    self.config,
                                    end - u64::from(self.config.sample_ticks.get()),
                                )
                            });
                        }
                    }
                }
                Ok(None) | Err(Error::Pending) => self.status.windows_interrupted += 1,
                Err(error) if unavailable(&error) => {
                    self.status.unavailable_probes += 1;
                    return Ok(None);
                }
                Err(error) => return Err(error),
            }
        }
        if decision.is_none()
            && !mandatory
            && self.window.is_none()
            && tick.is_multiple_of(u64::from(self.config.period_ticks.get()))
            // An explicit diagnostic has priority over background sampling.
            && !pool.active_probe.is_some_and(|id| id.covers(tick))
            && !pool.dirty.iter().any(|&dirty| dirty)
        {
            // Moving the only component on a worker cannot lower the current
            // longest component time. With one local CPU no second active lane
            // can accept work. Neither case needs a timing request to a child.
            if tick
                .checked_add(u64::from(self.config.sample_ticks.get()))
                .is_none()
                || pool.local_lanes < 2
                || pool.frame.workers.iter().all(|w| w.components.len() <= 1)
            {
                self.status.windows_ineligible += 1;
                return Ok(None);
            }
            let early = self
                .idle
                .as_ref()
                .filter(|idle| tick < idle.next_sample_tick());
            if early.is_some_and(|idle| idle.unchanged(pool)) {
                self.status.windows_deferred += 1;
                return Ok(None);
            }
            match measured("placement_open_start", "placement_open_end", || {
                pool.start_resident_load_sample(self.config.sample_ticks, ADVISORY_TIMEOUT)
            }) {
                Ok(window) => {
                    self.window = Some((tick + u64::from(self.config.sample_ticks.get()), window));
                    self.status.windows_started += 1;
                    self.status.windows_woken += u64::from(early.is_some());
                    self.idle = IdleCadence::at_start(self.idle.take(), self.config, pool);
                }
                Err(error) if unavailable(&error) => self.status.unavailable_probes += 1,
                Err(error) => return Err(error),
            }
        }
        Ok(decision)
    }
}
