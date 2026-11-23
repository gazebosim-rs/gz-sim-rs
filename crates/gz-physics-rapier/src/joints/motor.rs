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
        let joint = self.records.get(&id).ok_or(PhysicsError::MissingJoint)?;
        if matches!(joint.initial.motion, JointMotion::Universal(_)) && motor.is_some() {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "scalar motor on a universal joint",
            });
        }
        if matches!(joint.initial.motion, JointMotion::Ball) && motor.is_some() {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "scalar motor on a ball joint",
            });
        }
        if joint.initial.exact_lock && motor.is_some() {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "motor on an exact revolute lock",
            });
        }
        if motor.is_some_and(|motor| {
            !motor.target_velocity.is_finite()
                || !joint.initial.admits_effort(motor.max_force)
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
        // The kinematic witnesses for a speed stop must remain on their
        // angular chart over the public step, like other rotating boundaries.
        if self
            .speed_caps
            .values()
            .flatten()
            .flatten()
            .any(|cap| cap.limit() * dt > std::f64::consts::FRAC_PI_4)
        {
            return Err(PhysicsError::InvalidValue);
        }
        for joint in self.records.values() {
            if let Some(motor) = data(world, joint.handle).motor(joint.initial.motor_axis())
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
        let axis = self.joints.records[&id].initial.motor_axis();
        let handle = self.joints.records[&id].handle;
        let child = self.bodies[&self.joints.records[&id].initial.child].0;
        let wake = motor.is_some()
            && self
                .parked_robots
                .values()
                .any(|members| members.contains(&child));
        let data = match handle {
            Handle::Impulse(handle) => {
                &mut self
                    .world
                    .impulse_joints
                    .get_mut(handle, wake)
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
            data.set_motor_model(axis, MotorModel::AccelerationBased)
                .set_motor_velocity(axis, motor.target_velocity, 1.0)
                .set_motor_max_force(axis, motor.max_force);
        } else {
            data.motor_axes.remove(axis.into());
            // A later re-enable must not resurrect an old warm-start impulse.
            data.motors[axis as usize] = JointMotor::default();
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../../tests/private/motor.rs"]
mod tests;
