// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Force-limited position drive for either a hinge (rad, N·m) or a slider (m, N).
use crate::{JointState, PhysicsError};

/// Force-based spring/damper with a commanded velocity bound. External loads
/// still accelerate the joint; this actuator never overwrites its velocity.
#[derive(Clone, Copy, Debug)]
pub struct JointDrive {
    stiffness: f64,
    damping: f64,
    max_velocity: f64,
    max_effort: f64,
}
impl JointDrive {
    pub fn new(
        stiffness: f64,
        damping: f64,
        max_velocity: f64,
        max_effort: f64,
    ) -> Result<Self, PhysicsError> {
        if ![stiffness, damping, max_velocity, max_effort]
            .into_iter()
            .all(|x| x.is_finite() && x > 0.)
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Self {
            stiffness,
            damping,
            max_velocity,
            max_effort,
        })
    }
    /// Inside the speed bound this is a Hooke spring plus a viscous damper.
    /// Outside it the requested speed saturates before damping and feedforward.
    pub fn effort(
        self,
        state: JointState,
        position: f64,
        velocity: f64,
        feedforward: f64,
    ) -> Result<f64, PhysicsError> {
        if ![
            state.position,
            state.velocity,
            position,
            velocity,
            feedforward,
        ]
        .into_iter()
        .all(f64::is_finite)
        {
            return Err(PhysicsError::InvalidValue);
        }
        let speed = (self.stiffness / self.damping).mul_add(position - state.position, velocity);
        let desired = speed.clamp(-self.max_velocity, self.max_velocity);
        let force = self.damping.mul_add(desired - state.velocity, feedforward);
        if !force.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(force.clamp(-self.max_effort, self.max_effort))
    }
}
