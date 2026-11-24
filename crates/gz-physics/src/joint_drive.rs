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

/// Isotropic spherical-joint impedance. Orientations map the child joint axes
/// into the parent axes. Rates, feedforward and output torque use parent axes.
#[derive(Clone, Copy, Debug)]
pub struct SphericalDrive {
    stiffness_nm_rad: f64,
    damping_nm_s_rad: f64,
    max_torque_nm: f64,
}

impl SphericalDrive {
    /// Zero stiffness or damping disables that term. The limit bounds the
    /// magnitude of the complete actuator torque, including feedforward.
    pub fn new(
        stiffness_nm_rad: f64,
        damping_nm_s_rad: f64,
        max_torque_nm: f64,
    ) -> Result<Self, PhysicsError> {
        if ![stiffness_nm_rad, damping_nm_s_rad, max_torque_nm]
            .into_iter()
            .all(|v| v.is_finite() && v >= 0.)
            || max_torque_nm == 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Self {
            stiffness_nm_rad,
            damping_nm_s_rad,
            max_torque_nm,
        })
    }

    pub fn torque(
        &self,
        actual: glam::DQuat,
        target: glam::DQuat,
        actual_rate: glam::DVec3,
        target_rate: glam::DVec3,
        feedforward_nm: glam::DVec3,
    ) -> Result<glam::DVec3, PhysicsError> {
        if !actual.is_normalized() || !target.is_normalized() {
            return Err(PhysicsError::InvalidValue);
        }
        let mut relative = (target * actual.conjugate()).normalize();
        // q and -q represent the same orientation. At exactly half a turn
        // choose a deterministic axis, including when w is negative zero.
        let axis = relative.xyz();
        if relative.w < 0.
            || (relative.w == 0.
                && axis
                    .to_array()
                    .into_iter()
                    .find(|&x| x != 0.)
                    .is_some_and(|x| x < 0.))
        {
            relative = -relative;
        }
        let torque = self.stiffness_nm_rad * relative.to_scaled_axis()
            + self.damping_nm_s_rad * (target_rate - actual_rate)
            + feedforward_nm;
        if !torque.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        let peak = torque.abs().max_element();
        if peak == 0. {
            return Ok(torque);
        }
        // Scaling first also caps vectors whose squared magnitude would overflow.
        let scaled = torque / peak;
        let limit = self.max_torque_nm / scaled.length();
        Ok(if peak > limit { scaled * limit } else { torque })
    }
}
