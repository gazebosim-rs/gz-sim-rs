// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Contact timing for a translating sole and a flat support with constant relative normal acceleration.
use crate::PhysicsError;
use std::time::Duration;

/// Signed distance in metres, closing speed in m/s, and closing acceleration
/// in m/s². A positive closing speed means the normal gap is shrinking.
#[derive(Debug, Clone, Copy)]
pub struct NormalApproach {
    pub gap: f64,
    pub closing_speed: f64,
    pub closing_acceleration: f64,
}
impl NormalApproach {
    /// Earliest approaching touch in the closed interval, rounded to a nanosecond.
    /// A nonpositive initial gap is reported at zero. Geometry and tangential
    /// overlap must be checked by the caller.
    pub fn touchdown(self, dt: Duration) -> Result<Option<Duration>, PhysicsError> {
        let Some(seconds) = self.touchdown_seconds(dt)? else {
            return Ok(None);
        };
        let t = Duration::try_from_secs_f64(seconds).map_err(|_| PhysicsError::InvalidValue)?;
        Ok((t <= dt).then_some(t))
    }

    /// The same first approach in seconds, before rounding to the native clock.
    pub fn touchdown_seconds(self, dt: Duration) -> Result<Option<f64>, PhysicsError> {
        let Self {
            gap,
            closing_speed: c,
            closing_acceleration: a,
        } = self;
        if ![gap, c, a].into_iter().all(f64::is_finite) {
            return Err(PhysicsError::InvalidValue);
        }
        if gap <= 0. {
            return Ok(Some(0.));
        }
        let seconds = if a == 0. {
            if c <= 0. {
                return Ok(None);
            }
            gap / c
        } else {
            let disc = (2. * a).mul_add(gap, c * c);
            if !disc.is_finite() {
                return Err(PhysicsError::InvalidValue);
            }
            if disc < 0. || (c <= 0. && a < 0.) {
                return Ok(None);
            }
            let v = disc.sqrt();
            if c >= 0. {
                gap / (0.5 * c + 0.5 * v)
            } else {
                (v - c) / a
            }
        };
        if seconds > dt.as_secs_f64() {
            return Ok(None);
        }
        Ok(Some(seconds))
    }
}
