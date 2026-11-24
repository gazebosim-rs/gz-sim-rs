// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::time::Duration;

/// Slip recovery settings for an established opposed-pad grasp.
/// Force and slew limits apply to each jaw individually.
#[derive(Clone, Copy, Debug)]
pub struct SlipCatchConfig {
    pub trigger_drop_m: f64,
    pub trigger_speed_m_s: f64,
    pub settled_speed_m_s: f64,
    pub confirm_time: Duration,
    pub settle_time: Duration,
    pub force_rate_n_s: f64,
    pub max_force_n: f64,
}

impl Default for SlipCatchConfig {
    fn default() -> Self {
        Self {
            trigger_drop_m: 0.0001,
            trigger_speed_m_s: 0.005,
            settled_speed_m_s: 0.0001,
            confirm_time: Duration::from_millis(1),
            settle_time: Duration::from_millis(4),
            force_rate_n_s: 120.,
            max_force_n: 4.2,
        }
    }
}

/// Status after the most recent committed sensor interval.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlipCatchState {
    Monitoring,
    Tightening,
    /// Relative speed stayed in the settling band for the configured period.
    Caught,
    /// The squeeze is at its limit and downward sliding is still measured.
    AtLimit,
    Released,
    /// An invalid observation latched the last valid force until explicit release or rearm.
    SensorFault,
}

/// Detects a parcel slipping relative to moving jaws along a known load axis.
/// Supply parcel center-of-mass height minus jaw height, projected against the
/// load, in metres. Call on the sensor clock using elapsed simulation time.
/// The returned force is a command for the following actuator interval.
#[derive(Debug)]
pub struct SlipCatch {
    config: SlipCatchConfig,
    state: SlipCatchState,
    baseline_m: f64,
    last_height_m: f64,
    elapsed: Duration,
    force_n: f64,
}

impl SlipCatch {
    /// Start a fresh grasp at an observed relative height and the current squeeze.
    pub fn arm(config: SlipCatchConfig, initial_force_n: f64, height_m: f64) -> Option<Self> {
        if !height_m.is_finite()
            || !config.max_force_n.is_finite()
            || config.max_force_n <= 0.
            || !(0. ..=config.max_force_n).contains(&initial_force_n)
            || !config.force_rate_n_s.is_finite()
            || config.force_rate_n_s <= 0.
            || !config.trigger_drop_m.is_finite()
            || config.trigger_drop_m <= 0.
            || !config.trigger_speed_m_s.is_finite()
            || !config.settled_speed_m_s.is_finite()
            || config.settled_speed_m_s < 0.
            || config.trigger_speed_m_s <= config.settled_speed_m_s
            || config.confirm_time.is_zero()
            || config.settle_time.is_zero()
        {
            return None;
        }
        Some(Self {
            config,
            state: SlipCatchState::Monitoring,
            baseline_m: height_m,
            last_height_m: height_m,
            elapsed: Duration::ZERO,
            force_n: initial_force_n,
        })
    }

    pub fn force_n(&self) -> f64 {
        self.force_n
    }

    pub fn state(&self) -> SlipCatchState {
        self.state
    }

    /// Stop retention and request zero squeeze. Arm a new controller to regrasp.
    pub fn release(&mut self) {
        self.force_n = 0.;
        self.state = SlipCatchState::Released;
    }

    /// Read the committed relative height. Pausing leaves all state untouched.
    pub fn observe(&mut self, dt: Duration, height_m: f64) -> SlipCatchState {
        use SlipCatchState::*;
        if dt.is_zero() || matches!(self.state, Released | SensorFault) {
            return self.state;
        }
        let speed = (self.last_height_m - height_m) / dt.as_secs_f64();
        let drop = self.baseline_m - height_m;
        if !speed.is_finite() || !drop.is_finite() {
            self.state = SensorFault;
            return self.state;
        }
        self.last_height_m = height_m;
        if matches!(self.state, Monitoring | Caught) {
            if drop < self.config.trigger_drop_m || speed < self.config.trigger_speed_m_s {
                self.elapsed = Duration::ZERO;
                return self.state;
            }
            self.elapsed = self.elapsed.saturating_add(dt);
            if self.elapsed < self.config.confirm_time {
                return self.state;
            }
            self.elapsed = Duration::ZERO;
        }

        self.state = Tightening;
        if speed.abs() <= self.config.settled_speed_m_s {
            self.elapsed = self.elapsed.saturating_add(dt);
            if self.elapsed >= self.config.settle_time {
                self.state = Caught;
                self.baseline_m = height_m;
                self.elapsed = Duration::ZERO;
            }
        } else {
            self.elapsed = Duration::ZERO;
            if speed > 0. {
                self.force_n = (self.force_n + self.config.force_rate_n_s * dt.as_secs_f64())
                    .min(self.config.max_force_n);
                if self.force_n == self.config.max_force_n {
                    self.state = AtLimit;
                }
            }
        }
        self.state
    }
}

#[cfg(test)]
#[path = "../../tests/private/slip_catch.rs"]
mod tests;
