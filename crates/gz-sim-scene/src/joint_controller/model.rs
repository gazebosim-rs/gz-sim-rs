// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Pure continuous-error admission and bounded velocity lowering; no physics storage.
use super::JointControllerMode;
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub(super) enum InvalidControllerInput {
    #[error("controller time step must be positive and representable")]
    Time,
    #[error("controller command must be finite and representable")]
    Command,
    #[error("joint coordinate must be finite")]
    State,
    #[error("controller velocity bound and per-step movement must be finite and positive")]
    Bounds,
}

/// Mirrors the native explicit velocity-command ABS calculation, using the
/// continuous coordinate without wrapping. Velocity mode uses the result as its
/// motor target; position mode retains its numeric admission guards, then selects
/// the backend's damped position servo instead of actuating this ABS result.
/// This is not a physical speed guarantee. Validate before returning any bound.
pub(super) fn target_velocity(
    mode: JointControllerMode,
    target: f64,
    coordinate: f64,
    velocity_limit: f64,
    dt: Duration,
) -> Result<f64, InvalidControllerInput> {
    if !target.is_finite() {
        return Err(InvalidControllerInput::Command);
    }
    if !coordinate.is_finite() {
        return Err(InvalidControllerInput::State);
    }
    if !velocity_limit.is_finite() || velocity_limit <= 0.0 {
        return Err(InvalidControllerInput::Bounds);
    }
    let seconds = dt.as_secs_f64();
    if dt.is_zero() || !seconds.is_finite() || seconds <= 0.0 {
        return Err(InvalidControllerInput::Time);
    }
    let movement = velocity_limit * seconds;
    if !movement.is_finite() || movement <= 0.0 {
        return Err(InvalidControllerInput::Bounds);
    }
    match mode {
        JointControllerMode::Velocity => Ok(target.clamp(-velocity_limit, velocity_limit)),
        JointControllerMode::Position => {
            let delta = target - coordinate;
            if !delta.is_finite() {
                return Err(InvalidControllerInput::Command);
            }
            if delta.abs() > movement {
                Ok(velocity_limit.copysign(delta))
            } else {
                let velocity = delta / seconds;
                if !velocity.is_finite() {
                    return Err(InvalidControllerInput::Command);
                }
                Ok(velocity)
            }
        }
    }
}

#[cfg(test)]
#[path = "../../tests/private/joint_controller_model.rs"]
mod tests;
