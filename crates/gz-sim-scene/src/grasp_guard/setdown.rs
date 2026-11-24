// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

/// Output of a load-confirmed placement, sampled once per advancing millisecond.
/// Lift offsets have positive Z upwards. The opening clock
/// runs from 0 to 350 ms and is suitable for a smooth, bounded jaw trajectory.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SetdownCommand {
    pub lift_offset_m: f64,
    pub opening_ms: u16,
    pub advancing_opening: bool,
    pub search_exhausted: bool,
}

/// Transfers a held load onto an instrumented receiving surface. Call once per
/// advancing 1 ms actuator interval after the nominal lowering stroke. Supply
/// the upward impulse on the payload summed over *all* native exchanges in the
/// preceding interval. The command consumes only this observation and weight.
#[derive(Debug)]
pub struct SupportedSetdown {
    weight_n: f64,
    release: super::SupportedRelease,
    command: SetdownCommand,
}
impl SupportedSetdown {
    pub fn new(weight_n: f64) -> Option<Self> {
        (weight_n.is_finite() && weight_n > 0.).then_some(Self {
            weight_n,
            release: super::SupportedRelease::default(),
            command: SetdownCommand::default(),
        })
    }

    /// Use the signed, world-up component of the receiver's impulse on the
    /// payload over the preceding millisecond. A downward impulse supplies
    /// zero weight support; non-finite readings leave the command untouched.
    pub fn advance_vertical_reaction(
        &mut self,
        vertical_impulse_ns: f64,
    ) -> Option<SetdownCommand> {
        self.advance_with_support_check(vertical_impulse_ns, true)
    }

    /// Apply the measured vertical impulse to the lift controller, and permit
    /// the jaw confirmation window to advance only if the support check passes.
    /// Failing it clears the whole confirmation window, including during opening.
    pub fn advance_with_support_check(
        &mut self,
        vertical_impulse_ns: f64,
        permits_opening: bool,
    ) -> Option<SetdownCommand> {
        if !vertical_impulse_ns.is_finite() {
            return None;
        }
        self.advance_pressure(vertical_impulse_ns.max(0.), permits_opening)
    }

    /// An invalid observation produces no new command and leaves state intact.
    pub fn advance(&mut self, upward_impulse_ns: f64) -> Option<SetdownCommand> {
        self.advance_pressure(upward_impulse_ns, true)
    }

    fn advance_pressure(
        &mut self,
        upward_impulse_ns: f64,
        permits_opening: bool,
    ) -> Option<SetdownCommand> {
        let ratio = upward_impulse_ns / (0.001 * self.weight_n);
        if !ratio.is_finite() || ratio < 0. {
            return None;
        }
        let c = &mut self.command;
        if c.search_exhausted || c.opening_ms == 350 {
            c.advancing_opening = false;
            return Some(*c);
        }
        // Slowly remove any gap, or back away from excess preload. Limit travel
        // to four millimetres below the planned resting height.
        let previous = c.lift_offset_m;
        let speed = (0.0025 * (ratio - 1.)).clamp(-0.0025, 0.003);
        c.lift_offset_m = (previous + 0.001 * speed).clamp(-0.004, 0.);

        // An excessive preload breaks the confirmation window. A weak reading
        // pauses the jaw trajectory and requires renewed sustained weight.
        let normal = if permits_opening && ratio <= 1.5 {
            upward_impulse_ns
        } else {
            0.
        };
        c.advancing_opening =
            self.release
                .observe(std::time::Duration::from_millis(1), normal, self.weight_n);
        c.opening_ms += u16::from(c.advancing_opening);
        if c.lift_offset_m <= -0.004 && ratio < 0.2 {
            c.search_exhausted = true;
        }
        Some(*c)
    }
}

#[cfg(test)]
#[path = "../../tests/private/supported_setdown.rs"]
mod tests;
