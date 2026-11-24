// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use crate::grasp_guard::SlipCatchState;
use std::time::Duration;

/// Reduces the load on a grasp by capping upward lift effort after the jaws
/// have reported sustained slip at their squeeze limit. Construct one per lift.
#[derive(Debug)]
pub struct SlipLiftInterlock {
    confirmation: Duration,
    time_at_limit: Duration,
    tripped: bool,
}

impl SlipLiftInterlock {
    pub fn new(confirmation: Duration) -> Self {
        Self {
            confirmation,
            time_at_limit: Duration::ZERO,
            tripped: false,
        }
    }

    /// Returns the effort for the *next* interval, in newtons along the upward
    /// lift axis. Supply elapsed simulation time for the observed guard state.
    /// Set `drive_at_limit` only if the applied jaw command in that interval
    /// reached its cap. A slower sensor may have requested more than the drive
    /// has applied so far. The cap persists for this lift once tripped.
    /// A typical cap is the weight of the loaded carriage. The caller can still
    /// command a lower effort, for example when setting the parcel down.
    pub fn command(
        &mut self,
        state: SlipCatchState,
        drive_at_limit: bool,
        elapsed: Duration,
        requested_n: f64,
        unloaded_n: f64,
    ) -> f64 {
        if !elapsed.is_zero() && !self.tripped {
            if state == SlipCatchState::AtLimit && drive_at_limit {
                self.time_at_limit = self.time_at_limit.saturating_add(elapsed);
                self.tripped = self.time_at_limit >= self.confirmation;
            } else {
                self.time_at_limit = Duration::ZERO;
            }
        }
        if self.tripped {
            requested_n.min(unloaded_n)
        } else {
            requested_n
        }
    }
}

#[cfg(test)]
#[path = "../../tests/private/slip_lift_interlock.rs"]
mod tests;
