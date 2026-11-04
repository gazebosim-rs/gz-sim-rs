// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn data(world: &PhysicsWorld, handle: Handle) -> &GenericJoint {
    match handle {
        Handle::Impulse(handle) => {
            &world
                .impulse_joints
                .get(handle)
                .expect("registered joint handle")
                .data
        }
        Handle::Reduced(handle) => {
            let (body, link) = world
                .multibody_joints
                .get(handle)
                .expect("registered articulation handle");
            &body
                .link(link)
                .expect("registered articulation link")
                .joint
                .data
        }
    }
}

impl Joints {
    pub(crate) fn validate_motor(
        &self,
        id: JointId,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.records.get(&id).ok_or(PhysicsError::MissingJoint)?;
        if motor.is_some_and(|motor| {
            !motor.target_velocity.is_finite()
                || !motor.max_force.is_finite()
                || motor.max_force < 0.0
        }) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }

    pub(crate) fn validate_motor_step(
        &self,
        world: &PhysicsWorld,
        dt: f64,
    ) -> Result<(), PhysicsError> {
        for joint in self.records.values() {
            if let Some(motor) = data(world, joint.handle).motor(JointAxis::AngX)
                && (!(motor.target_vel * dt).is_finite() || !(motor.max_force * dt).is_finite())
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        Ok(())
    }
}

impl RapierBackend {
    pub(crate) fn replace_joint_motor(
        &mut self,
        id: JointId,
        motor: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.joints.validate_motor(id, motor)?;
        let handle = self.joints.records[&id].handle;
        // Joint construction already disables sleeping for dynamic endpoints.
        let data = match handle {
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
        if let Some(motor) = motor {
            data.set_motor_model(JointAxis::AngX, MotorModel::AccelerationBased)
                .set_motor_velocity(JointAxis::AngX, motor.target_velocity, 1.0)
                .set_motor_max_force(JointAxis::AngX, motor.max_force);
        } else {
            data.motor_axes.remove(JointAxesMask::ANG_X);
            // A later re-enable must not resurrect an old warm-start impulse.
            data.motors[JointAxis::AngX as usize] = JointMotor::default();
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../../tests/private/motor.rs"]
mod tests;
