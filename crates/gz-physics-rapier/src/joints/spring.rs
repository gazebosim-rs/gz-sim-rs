// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// A passive spring and viscous damper on a physical joint coordinate.
/// Translational gains use N/m and N·s/m. Angular gains use N·m/rad and N·m·s/rad.
#[derive(Clone, Copy, Debug)]
pub struct RobotSpring {
    pub stiffness: f64,
    pub damping: f64,
    pub reference: f64,
}

impl RobotSpring {
    pub fn effort(self, state: JointState) -> f64 {
        self.stiffness * (self.reference - state.position) - self.damping * state.velocity
    }

    fn valid_gains(self) -> bool {
        self.reference.is_finite()
            && self.stiffness.is_finite()
            && self.damping.is_finite()
            && self.stiffness >= 0.
            && self.damping >= 0.
            && (self.stiffness > 0. || self.damping > 0.)
    }

    pub(super) fn admissible_continuous(self) -> bool {
        self.valid_gains() && (self.stiffness * self.reference).is_finite()
    }

    /// Implicit spring torque k*(reference-q) - (d+h*k)*omega_next.
    /// A velocity row retains whole turns and applies torque on the physical axis.
    pub(super) fn angular_response(self, position: f64, dt: f64) -> (f64, f64) {
        let rate = if self.stiffness == 0. {
            0.
        } else {
            (self.reference - position) / (dt + self.damping / self.stiffness)
        };
        (rate, self.damping + self.stiffness * dt)
    }

    fn admissible(self, joint: RobotJoint) -> bool {
        let angular = match joint {
            RobotJoint::Screw { .. } => return self.admissible_continuous(),
            RobotJoint::Prismatic { .. } => false,
            // Native angular spring position is a wrapped angle. A nonzero
            // spring needs its full physical range on a single angular chart.
            RobotJoint::Revolute { limits, .. } => {
                if self.stiffness != 0.
                    && !limits.is_some_and(|[lo, hi]| {
                        [lo, hi].into_iter().all(|q| {
                            q.abs() < std::f64::consts::PI
                                && (q - self.reference).abs() < std::f64::consts::PI
                        })
                    })
                {
                    return false;
                }
                true
            }
            _ => return false,
        };
        self.valid_gains()
            && (!angular || self.stiffness == 0. || self.reference.abs() < std::f64::consts::PI)
    }
}

impl Construction {
    pub(super) fn insert_spring(
        self,
        world: &mut PhysicsWorld,
        parent: RigidBodyHandle,
        child: RigidBodyHandle,
    ) -> Option<ImpulseJointHandle> {
        let spring = self.spring?;
        let axis = self.motor_axis();
        let row = GenericJointBuilder::new(JointAxesMask::empty())
            .local_frame1(self.frame1)
            .local_frame2(self.frame2)
            .contacts_enabled(false)
            .motor_model(axis, MotorModel::ForceBased)
            .motor_position(axis, spring.reference, spring.stiffness, spring.damping);
        Some(world.insert_impulse_joint(parent, child, row))
    }
}

impl RapierBackend {
    /// Installs an independent passive constraint row. Active efforts and motors
    /// remain independently bounded and can be replaced without clearing it.
    pub fn add_robot_joint_with_spring(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        joint: RobotJoint,
        spring: RobotSpring,
    ) -> Result<JointId, PhysicsError> {
        if !spring.admissible(joint) {
            return Err(PhysicsError::InvalidValue);
        }
        let id = self.add_robot_joint(parent, child, anchor, joint)?;
        let record = self.joints.records.get_mut(&id).expect("new robot joint");
        record.initial.spring = Some(spring);
        let parent = parent.map_or_else(
            || self.joints.anchor.expect("world joint anchor"),
            |p| self.bodies[&p].0,
        );
        record.spring =
            record
                .initial
                .insert_spring(&mut self.world, parent, self.bodies[&child].0);
        Ok(id)
    }
}
