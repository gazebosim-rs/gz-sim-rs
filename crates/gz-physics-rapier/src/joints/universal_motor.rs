// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

impl RapierBackend {
    /// Check an effort-limited angular velocity motor about one physical wrist axis.
    pub fn validate_universal_motor(
        &self,
        id: JointId,
        axis: usize,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.require_dynamic_joint(id)?;
        let joint = self
            .joints
            .records
            .get(&id)
            .ok_or(PhysicsError::MissingJoint)?;
        let JointMotion::Universal(settings) = joint.initial.motion else {
            return Err(PhysicsError::InvalidJoint);
        };
        let limit = settings
            .effort_limit(axis)
            .ok_or(PhysicsError::InvalidJoint)?;
        if motor.is_some_and(|m| {
            !m.target_velocity.is_finite()
                || !m.max_force.is_finite()
                || m.max_force < 0.
                || m.max_force > limit
        }) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }

    /// Replace or remove a single wrist motor without changing its other axis,
    /// passive spring torque, or position and velocity stops.
    pub fn set_universal_motor(
        &mut self,
        id: JointId,
        axis: usize,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.validate_universal_motor(id, axis, motor)?;
        let key = (id, axis);
        let Some(motor) = motor else {
            if let Some(row) = self.joints.universal_motors.remove(&key) {
                self.world.remove_impulse_joint(row);
                if !self.joints.universal_motors.contains_key(&(id, 1 - axis)) {
                    let source = self.joints.records[&id].initial;
                    self.world.bodies[self.bodies[&source.child].0].set_additional_pgs_iterations(
                        usize::from(source.additional_pgs_iterations),
                    );
                }
            }
            return Ok(());
        };
        let row = *self.joints.universal_motors.entry(key).or_insert_with(|| {
            let initial = self.joints.records[&id].initial;
            let child = self.bodies[&initial.child].0;
            // Coupled spring, stop and motor reactions share this interface.
            self.world.bodies[child].set_additional_pgs_iterations(
                usize::from(initial.additional_pgs_iterations).max(4),
            );
            let parent = initial.parent.map_or_else(
                || self.joints.anchor.expect("world joint anchor"),
                |p| self.bodies[&p].0,
            );
            let (first, second, builder, _) = initial.universal_axis_row(parent, child, axis);
            self.world.insert_impulse_joint(first, second, builder)
        });
        let native_axis = if axis == 0 {
            JointAxis::AngX
        } else {
            JointAxis::AngY
        };
        let target = if axis == 0 {
            motor.target_velocity
        } else {
            -motor.target_velocity
        };
        self.world
            .impulse_joints
            .get_mut(row, false)
            .expect("registered wrist motor")
            .data
            .set_motor_model(native_axis, MotorModel::AccelerationBased)
            .set_motor_velocity(native_axis, target, 0.)
            .set_motor_max_force(native_axis, motor.max_force);
        Ok(())
    }
}
