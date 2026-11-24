// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::time::Duration;

/// Status of an established opposed-pad grasp after the last advancing tick.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RecoveryState {
    #[default]
    Held,
    /// Pressure or bilateral velocity sensing is unavailable.
    Uncertain,
    /// A measured pad load has fallen below the contact threshold.
    ConfirmingLoss,
    /// A loss lasted through the debounce interval. Reapply bounded squeeze.
    Reclosing,
    /// Both pads are loaded again; wait for their relative motion to settle.
    Settling,
    /// Hand off to a set-down controller. Latched for this grasp.
    Lower,
}

/// Tactile thresholds and motor limits for two opposed pads. Loads, squeeze
/// and slew are per finger; intervals are elapsed simulation time.
#[derive(Clone, Copy, Debug)]
pub struct ContactRecoveryConfig {
    pub min_load_n: f64,
    pub loss_dwell: Duration,
    pub settle_speed_m_s: f64,
    pub settle_dwell: Duration,
    pub timeout: Duration,
    pub squeeze_rate_n_s: f64,
    pub squeeze_limit_n: f64,
}

impl Default for ContactRecoveryConfig {
    fn default() -> Self {
        Self {
            min_load_n: 0.05,
            loss_dwell: Duration::from_millis(1),
            settle_speed_m_s: 0.005,
            settle_dwell: Duration::from_millis(3),
            timeout: Duration::from_millis(24),
            squeeze_rate_n_s: 100.,
            squeeze_limit_n: 5.2,
        }
    }
}

/// Recover an established two-sided tactile grasp. Call once per committed public tick.
/// The returned command is for the next actuator interval.
#[derive(Debug)]
pub struct ContactRecovery {
    config: ContactRecoveryConfig,
    state: RecoveryState,
    force_n: f64,
    episode: Duration,
    low_pressure: Duration,
    quiet: Duration,
    confirmed: bool,
    travel_m: Option<f64>,
}

impl Default for ContactRecovery {
    fn default() -> Self {
        Self::arm(ContactRecoveryConfig::default(), 4.2).expect("valid default recovery settings")
    }
}

impl ContactRecovery {
    /// Arm at the currently applied squeeze. Reject ranges that make a confirmed
    /// loss or a settled recovery impossible before the handoff deadline.
    pub fn arm(config: ContactRecoveryConfig, force_n: f64) -> Option<Self> {
        let positive = [
            config.min_load_n,
            config.squeeze_rate_n_s,
            config.squeeze_limit_n,
        ];
        if !positive.iter().all(|n| n.is_finite() && *n > 0.)
            || !config.settle_speed_m_s.is_finite()
            || config.settle_speed_m_s < 0.
            || !(0. ..=config.squeeze_limit_n).contains(&force_n)
            || config.loss_dwell.is_zero()
            || config.settle_dwell.is_zero()
            || config.timeout < config.loss_dwell.saturating_add(config.settle_dwell)
        {
            return None;
        }
        Some(Self {
            config,
            state: RecoveryState::Held,
            force_n,
            episode: Duration::ZERO,
            low_pressure: Duration::ZERO,
            quiet: Duration::ZERO,
            confirmed: false,
            travel_m: Some(0.),
        })
    }

    pub fn squeeze_n(&self) -> f64 {
        self.force_n
    }

    /// Signed travel since the start of uninterrupted bilateral sensing, in m.
    /// A missing interval invalidates the integral. A returning sample establishes
    /// a fresh zero because the motion within the blind interval is unknown.
    pub fn tactile_travel_m(&self) -> Option<f64> {
        self.travel_m
    }

    /// Supply each pad's average normal load in N and signed endpoint tangential
    /// velocity in m/s. An unavailable pressure packet is distinct from measured
    /// zero force. A fresh supervisor starts the next independent grasp.
    pub fn observe(
        &mut self,
        dt: Duration,
        normal_n: Option<[f64; 2]>,
        slip_speed_m_s: Option<f64>,
    ) -> RecoveryState {
        use RecoveryState::*;
        if dt.is_zero() || self.state == Lower {
            return self.state;
        }
        let pressure = normal_n.filter(|pair| pair.iter().all(|n| n.is_finite() && *n >= 0.));
        let low = pressure.is_some_and(|pair| pair.iter().any(|&n| n < self.config.min_load_n));
        let speed = slip_speed_m_s.filter(|v| v.is_finite() && pressure.is_some() && !low);
        self.episode = self.episode.saturating_add(dt);
        self.low_pressure = if low {
            self.low_pressure.saturating_add(dt)
        } else {
            Duration::ZERO
        };
        self.confirmed |= self.low_pressure >= self.config.loss_dwell;
        if let Some(v) = speed {
            self.travel_m = Some(self.travel_m.map_or(0., |s| s + v * dt.as_secs_f64()));
            if self.confirmed {
                self.quiet = if v.abs() <= self.config.settle_speed_m_s {
                    self.quiet.saturating_add(dt)
                } else {
                    Duration::ZERO
                };
            }
            if !self.confirmed || self.quiet >= self.config.settle_dwell {
                self.episode = Duration::ZERO;
                self.quiet = Duration::ZERO;
                self.confirmed = false;
                self.state = Held;
                return self.state;
            }
            self.state = Settling;
        } else {
            self.travel_m = None;
            self.quiet = Duration::ZERO;
            self.state = if !low {
                Uncertain
            } else if self.confirmed {
                Reclosing
            } else {
                ConfirmingLoss
            };
        }
        if self.episode >= self.config.timeout {
            self.state = Lower;
        } else if self.state == Reclosing {
            self.force_n = (self.force_n + self.config.squeeze_rate_n_s * dt.as_secs_f64())
                .min(self.config.squeeze_limit_n);
        }
        self.state
    }
}

#[cfg(test)]
#[path = "../../tests/private/contact_recovery.rs"]
mod tests;
