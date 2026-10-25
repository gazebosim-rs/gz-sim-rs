// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! TPE constant-velocity motion over validated shared f64 state.

use glam::{DQuat, DVec3};
use gz_physics::BodyState;
use std::time::Duration;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum MotionError {
    #[error("kinematic state must be finite with a unit quaternion")]
    InvalidState,
    #[error("kinematic step exceeds the finite f64 range")]
    Overflow,
}

/// Centered kinematic body with finite state and a unit [x,y,z,w] quaternion.
/// Construction accepts squared quaternion norm within 1e-12 of one.
/// Static flags and collider policy are supplied by the simulator adapter.
#[derive(Clone, Debug)]
pub struct Body {
    state: BodyState,
}

impl TryFrom<BodyState> for Body {
    type Error = MotionError;

    fn try_from(state: BodyState) -> Result<Self, Self::Error> {
        let rotation = DQuat::from_array(state.orientation);
        if !state
            .position
            .iter()
            .chain(&state.velocity)
            .chain(&state.angular_velocity)
            .chain(&state.orientation)
            .all(|value| value.is_finite())
            || (rotation.length_squared() - 1.0).abs() > 1e-12
        {
            return Err(MotionError::InvalidState);
        }
        Ok(Self { state })
    }
}

impl Body {
    pub fn state(&self) -> &BodyState {
        &self.state
    }

    /// Advances at constant world-frame linear/angular velocity.
    /// Native TPE keeps the pose when both velocities are componentwise within
    /// 1e-3 of zero. Errors leave the state unchanged. Advanced rotations retain
    /// squared unit norm within 1e-14; paused/stationary poses preserve their bits.
    #[inline]
    pub fn advance(&mut self, dt: Duration) -> Result<(), MotionError> {
        let state = &self.state;
        if dt.is_zero()
            || state
                .velocity
                .iter()
                .chain(&state.angular_velocity)
                .all(|value| value.abs() <= 1e-3)
        {
            return Ok(());
        }
        let seconds = dt.as_secs_f64();
        let position =
            DVec3::from_array(state.position) + DVec3::from_array(state.velocity) * seconds;
        let angular = DVec3::from_array(state.angular_velocity) * seconds;
        let squared = angular.length_squared();
        if !position.is_finite() || !squared.is_finite() {
            return Err(MotionError::Overflow);
        }
        let delta = if squared == 0.0 {
            // glam's squared norm loses tiny angles; native TPE retains their half-angle.
            DQuat::from_xyzw(angular.x * 0.5, angular.y * 0.5, angular.z * 0.5, 1.0)
        } else {
            DQuat::from_scaled_axis(angular)
        };
        let orientation = delta * DQuat::from_array(state.orientation);
        let orientation = if (orientation.length_squared() - 1.0).abs() > 1e-14 {
            orientation.normalize()
        } else {
            orientation
        };
        self.state.position = position.to_array();
        self.state.orientation = orientation.to_array();
        Ok(())
    }
}
