// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Feedback for an established friction grasp with two opposed tactile pads.
use std::time::Duration;

/// Reading at a pad. Slip is the local tangential speed between the two contacting materials.
#[derive(Clone, Copy, Debug, Default)]
pub struct PadReading {
    pub normal_force_n: f64,
    pub slip_speed_m_s: f64,
}

/// Requested jaw/lift behavior; the caller owns the actuators and support sensing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GraspResponse {
    #[default]
    Hold,
    Tighten,
    SetDown,
}

/// Debounces brief contact changes and allows one pressure increase before a set-down.
/// Construct a new guard when the jaws close on a new load. Only feed observations
/// while retention is required; a deliberate release is a different operation.
#[derive(Clone, Copy, Debug, Default)]
pub struct GraspGuard {
    response: GraspResponse,
    bad_time: Duration,
    settling: Duration,
}

impl GraspGuard {
    /// Feed the reading from a committed physics interval. Invalid tactile measurements
    /// are treated as loss of contact. A zero-duration update never advances a timer.
    pub fn observe(&mut self, dt: Duration, pads: [PadReading; 2]) -> GraspResponse {
        if self.response == GraspResponse::SetDown || dt.is_zero() {
            return self.response;
        }
        let usable = dt.saturating_sub(self.settling);
        self.settling = self.settling.saturating_sub(dt);
        let slipping = pads.iter().any(|p| {
            !p.normal_force_n.is_finite()
                || p.normal_force_n < 0.05
                || !p.slip_speed_m_s.is_finite()
                || p.slip_speed_m_s.abs() > 0.018
        });
        self.bad_time = if slipping {
            self.bad_time.saturating_add(usable)
        } else {
            Duration::ZERO
        };
        if self.bad_time >= Duration::from_millis(4) {
            self.bad_time = Duration::ZERO;
            if self.response == GraspResponse::Hold {
                self.response = GraspResponse::Tighten;
                self.settling = Duration::from_millis(45);
            } else {
                self.response = GraspResponse::SetDown;
            }
        }
        self.response
    }
}

/// Authorizes opening the jaws after the selected receiving surface has borne
/// at least 80% of the known payload weight for 20 continuous simulated ms.
/// Start a fresh check when changing the receiving support or payload.
#[derive(Debug, Default)]
pub struct SupportedRelease {
    loaded: Duration,
}

impl SupportedRelease {
    /// Qualify a placement on the designated receiver. Pass its signed upward
    /// impulse over this committed interval and the result of the footprint
    /// or clearance check. Require 80–150% of the known weight throughout the
    /// confirmation window. Missing data or an impact clears the window.
    /// A paused simulation leaves the previous decision intact.
    pub fn observe_checked(
        &mut self,
        dt: Duration,
        upward_impulse_ns: Option<f64>,
        weight_n: f64,
        permits_opening: bool,
    ) -> bool {
        let upper_ns = 1.5 * (weight_n * dt.as_secs_f64());
        let qualified = upward_impulse_ns
            .filter(|impulse| permits_opening && *impulse <= upper_ns)
            .unwrap_or(0.);
        self.observe(dt, qualified, weight_n)
    }

    /// Supply the impulse on the parcel along world up from the selected
    /// support in this committed interval. Side contact contributes no lift.
    pub fn observe(&mut self, dt: Duration, upward_impulse_ns: f64, weight_n: f64) -> bool {
        if !dt.is_zero() {
            if weight_n.is_finite()
                && weight_n > 0.
                && upward_impulse_ns.is_finite()
                && upward_impulse_ns > 0.
                && upward_impulse_ns >= 0.8 * weight_n * dt.as_secs_f64()
            {
                self.loaded = self.loaded.saturating_add(dt);
            } else {
                self.loaded = Duration::ZERO;
            }
        }
        self.loaded >= Duration::from_millis(20)
    }
}

/// Command for a robot retaining a grasp above a designated receiving surface.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RetentionCommand {
    #[default]
    Hold,
    Tighten,
    Lower,
    Open,
}

/// Couples slip detection to a supported release. Create a fresh supervisor for
/// each grasp. Supply only the upward impulse from the intended receiving surface.
#[derive(Debug, Default)]
pub struct GraspRetention {
    guard: GraspGuard,
    release: SupportedRelease,
    command: RetentionCommand,
}

impl GraspRetention {
    /// Decide the next actuator command from a committed interval. Opening is
    /// latched only after a set-down request and sustained support. A paused
    /// observation preserves both the command and its timers.
    pub fn observe(
        &mut self,
        dt: Duration,
        pads: [PadReading; 2],
        support_upward_impulse_ns: f64,
        weight_n: f64,
    ) -> RetentionCommand {
        if dt.is_zero() || self.command == RetentionCommand::Open {
            return self.command;
        }
        self.command = match self.guard.observe(dt, pads) {
            GraspResponse::Hold => RetentionCommand::Hold,
            GraspResponse::Tighten => RetentionCommand::Tighten,
            GraspResponse::SetDown => {
                // The transition tick was integrated under the previous command.
                if self.command == RetentionCommand::Lower
                    && self
                        .release
                        .observe(dt, support_upward_impulse_ns, weight_n)
                {
                    RetentionCommand::Open
                } else {
                    RetentionCommand::Lower
                }
            }
        };
        self.command
    }
}

mod contact_recovery;
mod contact_slip;
mod deck_margin;
mod recovery_placement;
mod setdown;
mod slip_catch;
mod slip_lift_interlock;
pub use contact_recovery::{ContactRecovery, ContactRecoveryConfig, RecoveryState};
pub use contact_slip::bilateral_slip_velocity;
pub use deck_margin::FiniteDeckMargin;
pub use recovery_placement::{PlacementDrive, PlacementPhase, PlacementReading, RecoveryPlacement};
pub use setdown::{SetdownCommand, SupportedSetdown};
pub use slip_catch::{SlipCatch, SlipCatchConfig, SlipCatchState};
pub use slip_lift_interlock::SlipLiftInterlock;

#[cfg(test)]
#[path = "../tests/private/grasp_guard.rs"]
mod tests;
