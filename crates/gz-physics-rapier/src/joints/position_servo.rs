// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

// Fixed nominal critically damped acceleration response (10rad/s natural frequency).
// This policy is not selected by probing gains and is not an articulated stability proof.
const STIFFNESS: f64 = 100.0;
const DAMPING: f64 = 20.0;

fn waypoint(servo: JointPositionServo, coordinate: f64, dt: Duration) -> Result<f64, PhysicsError> {
    let seconds = dt.as_secs_f64();
    if !servo.target_position.is_finite()
        || !coordinate.is_finite()
        || !servo.max_velocity.is_finite()
        || servo.max_velocity <= 0.0
        || !servo.max_force.is_finite()
        || servo.max_force < 0.0
        || dt.is_zero()
        || !seconds.is_finite()
        || !(servo.max_force * seconds).is_finite()
        || !(servo.max_velocity * seconds).is_finite()
    {
        return Err(PhysicsError::InvalidValue);
    }
    let (actual_erp, cfm, gain) =
        MotorModel::AccelerationBased.combine_coefficients(seconds, STIFFNESS, DAMPING);
    // Rapier may divide the upcoming dt into solver substeps. Its maintained
    // zero-dt ERP limit bounds every positive substep ERP for these positive
    // fixed coefficients, without guessing private island iteration counts.
    let erp = MotorModel::AccelerationBased
        .combine_coefficients(0.0, STIFFNESS, DAMPING)
        .0;
    let error = servo.target_position - coordinate;
    let maximum = servo.max_velocity / erp;
    if !erp.is_finite()
        || erp <= 0.0
        || !actual_erp.is_finite()
        || actual_erp <= 0.0
        || !cfm.is_finite()
        || !gain.is_finite()
        || !error.is_finite()
        || !maximum.is_finite()
        || maximum <= 0.0
    {
        return Err(PhysicsError::InvalidValue);
    }
    let mut target = coordinate + error.clamp(-maximum, maximum);
    // Addition/division rounding can put the represented waypoint just outside
    // the speed bound. Move one representable value toward the current coordinate,
    // then check the actual native bias rather than relying on real-number algebra.
    if ((target - coordinate) * erp).abs() > servo.max_velocity {
        target = if target > coordinate {
            target.next_down()
        } else {
            target.next_up()
        };
    }
    let bias = (coordinate - target) * erp;
    if !target.is_finite()
        || !bias.is_finite()
        || bias.abs() > servo.max_velocity
        || (error != 0.0 && target == coordinate)
    {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(target)
}

impl RapierBackend {
    pub(crate) fn prepare_joint_position_servo(
        &self,
        id: JointId,
        servo: Option<JointPositionServo>,
        dt: Duration,
    ) -> Result<Option<(f64, f64)>, PhysicsError> {
        let joint = self
            .joints
            .records
            .get(&id)
            .ok_or(PhysicsError::MissingJoint)?;
        let Some(servo) = servo else {
            return Ok(None);
        };
        let Handle::Reduced(handle) = joint.handle else {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "continuous position servos on impulse joints",
            });
        };
        let (body, link) = self
            .world
            .multibody_joints
            .get(handle)
            .expect("registered articulation handle");
        let coordinate = body
            .link(link)
            .expect("registered articulation link")
            .joint
            .coords()[3];
        waypoint(servo, coordinate, dt).map(|target| Some((target, servo.max_force)))
    }

    pub(crate) fn replace_joint_position_servo(
        &mut self,
        id: JointId,
        servo: Option<JointPositionServo>,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        let target = self.prepare_joint_position_servo(id, servo, dt)?;
        let Some((target, max_force)) = target else {
            return self.replace_joint_motor(id, None);
        };
        let Handle::Reduced(handle) = self.joints.records[&id].handle else {
            unreachable!("preflighted reduced position servo");
        };
        let (body, link) = self
            .world
            .multibody_joints
            .get_mut(handle)
            .expect("registered articulation handle");
        let data = &mut body
            .link_mut(link)
            .expect("registered articulation link")
            .joint
            .data;
        data.set_motor_model(JointAxis::AngX, MotorModel::AccelerationBased)
            .set_motor_position(JointAxis::AngX, target, STIFFNESS, DAMPING)
            .set_motor_max_force(JointAxis::AngX, max_force);
        Ok(())
    }
}

#[cfg(test)]
#[path = "../../tests/private/position_servo.rs"]
mod tests;
