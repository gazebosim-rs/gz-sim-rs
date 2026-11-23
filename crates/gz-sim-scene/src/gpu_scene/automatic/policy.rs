// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{AutoPlacementConfig, ExecutionPlacement as Owner, PlacementReason};
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SamplingPhase {
    Waiting,
    Baseline,
    Alternative,
    Confirmation,
    Manual,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Evidence {
    /// Lower quartile of bracketed baseline minus the upper quartile of the alternative.
    pub conservative_saving_ns: u64,
    /// Conservative allowance for switching, first-step uploads and slower trial ticks.
    pub probe_and_switch_cost_ns: u64,
    /// Number of advancing ticks required to pay back this allowance.
    pub break_even_ticks: Option<u64>,
}
#[derive(Clone, Copy, Default)]
struct Window {
    low: u64,
    median: u64,
    high: u64,
}
impl Window {
    fn sample(values: &mut [u64]) -> Self {
        values.sort_unstable();
        Self {
            low: values[(values.len() - 1) / 4],
            median: values[(values.len() - 1) / 2],
            high: values[3 * (values.len() - 1) / 4],
        }
    }
    fn stable(self) -> bool {
        self.median > 0 && self.high - self.low <= self.median / 3
    }
}
pub(super) struct Decision {
    pub target: Owner,
    pub reason: PlacementReason,
    pub evidence: Evidence,
}
pub(super) struct Policy {
    pub phase: SamplingPhase,
    pub incumbent: Owner,
    eligible: u64,
    config: AutoPlacementConfig,
    values: Vec<u64>,
    base: Window,
    other: Window,
    cold: bool,
    debit: u64,
}
pub(super) fn opposite(o: Owner) -> Owner {
    match o {
        Owner::Cpu => Owner::Gpu,
        Owner::Gpu => Owner::Cpu,
    }
}
pub(super) fn ns(d: Duration) -> u64 {
    d.as_nanos().min(u64::MAX as u128) as u64
}
impl Policy {
    pub fn new(config: AutoPlacementConfig, owner: Owner, tick: u64) -> Self {
        Self {
            phase: SamplingPhase::Waiting,
            incumbent: owner,
            eligible: tick.saturating_add(config.min_residency_ticks),
            config,
            values: Vec::with_capacity(config.window_ticks as usize),
            base: Window::default(),
            other: Window::default(),
            cold: false,
            debit: 0,
        }
    }
    pub fn rearm(&mut self, tick: u64, owner: Owner) {
        self.phase = SamplingPhase::Waiting;
        self.incumbent = owner;
        self.eligible = tick.saturating_add(self.config.min_residency_ticks);
        self.cold = false;
        self.debit = 0;
        self.values.clear();
    }
    pub fn management_cost(&mut self, cost: Duration) {
        self.debit = self.debit.saturating_add(ns(cost));
    }
    pub fn sample(&mut self, tick: u64, elapsed: Duration) -> Option<Decision> {
        use SamplingPhase::*;
        if self.phase == Manual {
            return None;
        }
        if self.phase == Waiting {
            if tick < self.eligible {
                return None;
            }
            self.phase = Baseline;
            self.debit = 0;
        }
        let cost = ns(elapsed);
        if self.cold {
            // Charge the entire first post-switch step to the transition. This
            // bounds upload overhead without comparing different advancing ticks.
            self.debit = self.debit.saturating_add(cost);
            self.cold = false;
            return None;
        }
        self.values.push(cost);
        if self.values.len() != self.config.window_ticks as usize {
            return None;
        }
        let stats = Window::sample(&mut self.values);
        let mut evidence = Evidence::default();
        let mut target = self.incumbent;
        let reason = match self.phase {
            Baseline if stats.stable() => {
                self.base = stats;
                self.phase = Alternative;
                self.cold = true;
                target = opposite(target);
                PlacementReason::Probe
            }
            Baseline => {
                self.hold(tick);
                PlacementReason::UnstableWindow
            }
            Alternative => {
                self.other = stats;
                // A slow probe consumes real wall time. Include the positive
                // excess over the measured baseline, including occasional spikes.
                for v in &self.values {
                    self.debit = self
                        .debit
                        .saturating_add(v.saturating_sub(self.base.median));
                }
                self.phase = Confirmation;
                self.cold = true;
                PlacementReason::ConfirmBaseline
            }
            Confirmation => {
                let baseline = self.base.low.min(stats.low);
                let gain = baseline.saturating_sub(self.other.high);
                // Budget one additional full upload/handover to the winner, at
                // the worst total charged by the preceding exploratory round.
                let debit = self.debit.saturating_mul(2);
                let payback = (gain > 0).then(|| debit.div_ceil(gain));
                evidence = Evidence {
                    conservative_saving_ns: gain,
                    probe_and_switch_cost_ns: debit,
                    break_even_ticks: payback,
                };
                let stable = stats.stable()
                    && self.other.stable()
                    && self.base.median.abs_diff(stats.median)
                        <= self.base.median.min(stats.median) / 4;
                self.hold(tick);
                if !stable {
                    PlacementReason::UnstableWindow
                } else if gain == 0
                    || (gain as u128) * 100
                        < (baseline as u128) * (self.config.hysteresis_percent as u128)
                {
                    PlacementReason::InsufficientGain
                } else if payback.is_some_and(|h| h <= self.config.min_residency_ticks) {
                    target = opposite(target);
                    self.incumbent = target;
                    PlacementReason::Commit
                } else {
                    PlacementReason::InsufficientResidence
                }
            }
            Waiting | Manual => unreachable!("only completed sampling windows reach a decision"),
        };
        self.values.clear();
        Some(Decision {
            target,
            reason,
            evidence,
        })
    }
    fn hold(&mut self, tick: u64) {
        self.phase = SamplingPhase::Waiting;
        self.eligible = tick.saturating_add(self.config.reprobe_ticks);
    }
}

#[cfg(test)]
#[path = "../../../tests/private/automatic_policy.rs"]
mod tests;
