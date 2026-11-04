// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

impl RapierBackend {
    pub(crate) fn replace_joint_servo(
        &mut self,
        id: JointId,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        // Reuse accepted validation, atomic replacement and None/warm-start handling.
        // No fallible operation or physics step follows this validated replacement.
        self.replace_joint_motor(id, motor)?;
        if let Some(motor) = motor {
            let data = match self.joints.records[&id].handle {
                Handle::Impulse(handle) => {
                    &mut self
                        .world
                        .impulse_joints
                        .get_mut(handle, false)
                        .expect("registered joint handle")
                        .data
                }
                Handle::Reduced(handle) => {
                    let (body, link) = self
                        .world
                        .multibody_joints
                        .get_mut(handle)
                        .expect("registered articulation handle");
                    &mut body
                        .link_mut(link)
                        .expect("registered articulation link")
                        .joint
                        .data
                }
            };
            // Pinned Rapier's maintained reciprocal-zero convention makes all three
            // motor coefficients zero here, while retaining its enabled motor axis
            // and symmetric max_force * dt constraint impulse bounds.
            data.set_motor_model(JointAxis::AngX, MotorModel::AccelerationBased)
                .set_motor_velocity(JointAxis::AngX, motor.target_velocity, 0.0)
                .set_motor_max_force(JointAxis::AngX, motor.max_force);
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../../tests/private/servo.rs"]
mod tests;
