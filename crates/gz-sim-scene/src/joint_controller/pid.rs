// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Gazebo's sampled force PID. The integral stores its weighted contribution in N·m.
use super::model::InvalidControllerInput;
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ForcePid {
    pub gains: [f64; 3],
    pub integral_limits: [f64; 2],
    pub output_limits: [f64; 2],
    pub offset: f64,
    pub history: [f64; 2],
}

impl ForcePid {
    pub fn update(
        self,
        target: f64,
        coordinate: f64,
        dt: Duration,
        effort_limit: f64,
    ) -> Result<(f64, [f64; 2]), InvalidControllerInput> {
        let dt = dt.as_secs_f64();
        let error = coordinate - target;
        let rate = (error - self.history[0]) / dt;
        let integral = self.history[1] + self.gains[1] * dt * error;
        if !rate.is_finite() || !integral.is_finite() {
            return Err(InvalidControllerInput::Command);
        }
        // gz-math disables a clamp when its bounds are reversed.
        let bounded = |value: f64, [lo, hi]: [f64; 2]| {
            if lo <= hi { value.clamp(lo, hi) } else { value }
        };
        let integral = bounded(integral, self.integral_limits);
        let cmd = self.offset - self.gains[0] * error - integral - self.gains[2] * rate;
        if !cmd.is_finite() {
            return Err(InvalidControllerInput::Command);
        }
        Ok((
            bounded(cmd, self.output_limits).clamp(-effort_limit, effort_limit),
            [error, integral],
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn force_pid_uses_derivative_kick_offset_signed_integral_and_native_disabled_bounds() {
        let mut pid = ForcePid {
            gains: [2., 0.5, 0.25],
            integral_limits: [-0.1, 0.1],
            output_limits: [-1., 1.],
            offset: 0.125,
            history: [0.; 2],
        };
        let dt = Duration::from_millis(500);
        let (force, history) = pid.update(2., 0., dt, f64::INFINITY).unwrap();
        assert_eq!((force, history), (1., [-2., -0.1]));
        pid.history = history;
        pid.integral_limits = [1., -1.];
        pid.output_limits = [1., -1.];
        assert_eq!(
            pid.update(2., 0., dt, f64::INFINITY).unwrap(),
            (4.725, [-2., -0.6])
        );
        assert!(pid.update(2., 0., Duration::ZERO, 1.).is_err());
        pid.gains[0] = f64::MAX;
        assert!(pid.update(2., 0., dt, 1.).is_err());
    }
}
