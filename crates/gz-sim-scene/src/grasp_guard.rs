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

#[cfg(test)]
#[path = "../tests/private/grasp_guard.rs"]
mod tests;
