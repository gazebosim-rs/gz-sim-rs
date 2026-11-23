// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One authoritative time origin for a local or process-owned curved XPBD guide.
use super::BoundaryPath;
use gz_physics::trajectory::TranslationSpline;
use std::time::Duration;
pub struct TranslationBoundary {
    curve: TranslationSpline,
    origin: [f64; 3],
    at: Duration,
}
impl TranslationBoundary {
    /// The last native publication and its positive duration identify the complete soft interval.
    pub fn from_publication(
        curve: TranslationSpline,
        origin: [f64; 3],
        end: Duration,
        dt: Duration,
    ) -> Option<Self> {
        Some(Self {
            curve,
            origin,
            at: end.checked_sub(dt)?,
        })
    }
}
impl BoundaryPath for TranslationBoundary {
    fn center(&self, offset: Duration) -> [f64; 3] {
        let d = self.curve.sample(self.at + offset).displacement;
        std::array::from_fn(|i| self.origin[i] + d[i])
    }
    fn mean_velocity(&self, offset: Duration, dt: Duration) -> [f64; 3] {
        self.curve
            .mean_velocity(self.at + offset, dt)
            .expect("every common substep ends at or before the next knot")
    }
    fn until_knot(&self, offset: Duration) -> Option<Duration> {
        self.curve.until_knot(self.at + offset)
    }
    fn acceleration_bound_over(&self, offset: Duration, span: Duration) -> f64 {
        self.curve
            .acceleration_bound_over(self.at + offset, span)
            .unwrap_or_else(|| self.curve.acceleration_bound())
    }
    fn acceleration_bound(&self, offset: Duration) -> f64 {
        if self.until_knot(offset).is_none() {
            0.
        } else {
            self.curve.acceleration_bound()
        }
    }
}
