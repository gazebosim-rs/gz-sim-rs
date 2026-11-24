// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// A joint in a local rigid-body robot. Axis and orientation are in world coordinates
/// at construction. Scalar coordinates and both universal coordinates start at zero.
#[derive(Clone, Copy, Debug)]
pub enum RobotJoint {
    Fixed,
    Revolute {
        axis: [f64; 3],
        limits: Option<[f64; 2]>,
    },
    /// Thread pitch in metres per revolution. The coordinate and stops are in radians.
    Screw {
        axis: [f64; 3],
        pitch: f64,
        effort_limit: f64,
        limits: Option<[f64; 2]>,
    },
    Prismatic {
        axis: [f64; 3],
        limits: Option<[f64; 2]>,
    },
    Universal {
        axes: [[f64; 3]; 2],
        limits: [Option<[f64; 2]>; 2],
        effort_limits: [f64; 2],
    },
    Ball {
        orientation: [f64; 4],
    },
}

/// Relative orientation (xyzw) and angular velocity in the parent joint frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BallJointState {
    pub orientation: [f64; 4],
    pub angular_velocity: [f64; 3],
}

impl RapierBackend {
    /// Uses a single native impulse-joint graph for a mixed robot tree.
    pub fn add_robot_joint(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        joint: RobotJoint,
    ) -> Result<JointId, PhysicsError> {
        let (axis, constraint) = match joint {
            RobotJoint::Fixed => ([1., 0., 0.], AngularConstraint::Weld),
            RobotJoint::Universal {
                axes,
                limits,
                effort_limits,
            } => (
                axes[0],
                universal::configuration(axes, limits, effort_limits)?,
            ),
            RobotJoint::Ball { orientation } => {
                let rotation = Rotation::from_array(orientation);
                if !rotation.is_finite() || (rotation.length_squared() - 1.).abs() > 1e-12 {
                    return Err(PhysicsError::InvalidJoint);
                }
                ([1., 0., 0.], AngularConstraint::Ball(rotation))
            }
            RobotJoint::Screw {
                axis,
                pitch,
                limits,
                effort_limit,
            } => {
                let lead = pitch / std::f64::consts::TAU;
                if (pitch != 0. && !lead.is_normal())
                    || !(lead * lead).is_finite()
                    || effort_limit.is_nan()
                    || effort_limit < 0.
                    || limits.is_some_and(|[lo, hi]| {
                        !(lo <= 0. && hi >= 0.)
                            || [lo, hi]
                                .into_iter()
                                .any(|q| q.is_finite() && !(q * lead).is_finite())
                    })
                {
                    return Err(PhysicsError::InvalidValue);
                }
                (
                    axis,
                    AngularConstraint::Screw {
                        lead,
                        limits,
                        effort_limit,
                    },
                )
            }
            RobotJoint::Prismatic { axis, limits } => {
                if limits.is_some_and(|[lo, hi]| !(lo <= 0. && hi >= 0.)) {
                    return Err(PhysicsError::InvalidValue);
                }
                // Use Rapier's own unbounded limit sentinels on open ends.
                let limits = limits
                    .filter(|[lo, hi]| lo.is_finite() || hi.is_finite())
                    .map(|[lo, hi]| [lo.max(-f64::MAX), hi.min(f64::MAX)]);
                (axis, AngularConstraint::Prismatic(limits))
            }
            RobotJoint::Revolute { axis, limits } => {
                if limits.is_some_and(|[lo, hi]| {
                    !lo.is_finite() || !hi.is_finite() || lo > 0. || hi < 0.
                }) {
                    return Err(PhysicsError::InvalidValue);
                }
                (axis, AngularConstraint::Revolute(limits))
            }
        };
        let id =
            self.insert_configured_joint(parent, child, anchor, axis, Solver::Impulse, constraint)?;
        self.joints
            .records
            .get_mut(&id)
            .expect("inserted robot joint")
            .initial
            .robot = true;
        Ok(id)
    }

    /// Set the base constraint's natural frequency (Hz) for one world-mounted
    /// slider, before stepping. This sets the stabilization for its locked axes
    /// and any active positional stop, retaining the authored travel bounds.
    /// The native damping ratio, motors and separate speed rows are preserved.
    pub fn set_mounted_guide_frequency(
        &mut self,
        id: JointId,
        hz: f64,
    ) -> Result<(), PhysicsError> {
        if !self.joints.open {
            return Err(PhysicsError::JointConstructionClosed);
        }
        let defaults = SpringCoefficients::<f64>::joint_defaults();
        let coefficients = SpringCoefficients::new(hz, defaults.damping_ratio);
        // Native intervals are whole nanoseconds; Rapier may subdivide one
        // interval across a usize iteration budget. Its CFM squares inverse ERP.
        // Do not admit a frequency whose finite-step coefficients can overflow.
        let shortest_substep = 1e-9 / usize::MAX as f64;
        if !(hz > 0. && hz <= defaults.natural_frequency)
            || !coefficients.erp(shortest_substep).is_normal()
            || !coefficients.cfm_coeff(shortest_substep).is_finite()
        {
            return Err(PhysicsError::InvalidValue);
        }
        let joint = self
            .joints
            .records
            .get_mut(&id)
            .ok_or(PhysicsError::MissingJoint)?;
        let Handle::Impulse(h) = joint.handle else {
            return Err(PhysicsError::InvalidJoint);
        };
        if !joint.initial.robot
            || joint.initial.parent.is_some()
            || !matches!(joint.initial.motion, JointMotion::Prismatic)
        {
            return Err(PhysicsError::InvalidJoint);
        }
        self.world
            .impulse_joints
            .get_mut(h, false)
            .expect("registered slider")
            .data
            .softness
            .natural_frequency = hz;
        joint.initial.mounted_guide_hz = Some(hz);
        Ok(())
    }

    /// Set the native island's inner velocity-constraint iteration budget on a
    /// world-mounted rigid robot. Authored once, restored with the joint graph.
    pub fn set_robot_root_iterations(
        &mut self,
        root: BodyId,
        iterations: u8,
    ) -> Result<(), PhysicsError> {
        if !self.joints.open {
            return Err(PhysicsError::JointConstructionClosed);
        }
        if iterations > 128 {
            return Err(PhysicsError::InvalidValue);
        }
        let joint = self
            .joints
            .records
            .values_mut()
            .find(|j| j.initial.robot && j.initial.parent.is_none() && j.initial.child == root)
            .ok_or(PhysicsError::InvalidJoint)?;
        joint.initial.additional_pgs_iterations = iterations;
        self.world.bodies[self.bodies[&root].0]
            .set_additional_pgs_iterations(usize::from(iterations));
        Ok(())
    }

    pub fn is_robot_link(&self, body: BodyId) -> bool {
        self.joints
            .records
            .values()
            .any(|j| j.initial.robot && (j.initial.child == body || j.initial.parent == Some(body)))
    }

    /// Accumulate one step of torque in N·m, expressed in the parent joint frame.
    pub fn add_ball_effort(&mut self, id: JointId, torque: [f64; 3]) -> Result<(), PhysicsError> {
        let joint = self
            .joints
            .records
            .get(&id)
            .ok_or(PhysicsError::MissingJoint)?;
        if !matches!(joint.initial.motion, JointMotion::Ball) {
            return Err(PhysicsError::InvalidJoint);
        }
        let old = self
            .joints
            .angular_efforts
            .get(&id)
            .copied()
            .unwrap_or_default();
        let sum = old + Vector::from_array(torque);
        if !sum.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        self.joints.angular_efforts.insert(id, sum);
        Ok(())
    }

    /// Sum of spherical-joint torques queued for the next advance, in the parent joint axes.
    pub fn pending_ball_effort(&self, id: JointId) -> Option<[f64; 3]> {
        if !matches!(
            self.joints.records.get(&id)?.initial.motion,
            JointMotion::Ball
        ) {
            return None;
        }
        self.joints.angular_efforts.get(&id).map(|v| v.to_array())
    }

    pub fn ball_joint_state(&self, id: JointId) -> Option<BallJointState> {
        let joint = self.joints.records.get(&id)?.initial;
        if !matches!(joint.motion, JointMotion::Ball) {
            return None;
        }
        let child = &self.world.bodies[self.bodies[&joint.child].0];
        let parent = joint.parent.map(|p| &self.world.bodies[self.bodies[&p].0]);
        let rotation = parent.map_or(Rotation::IDENTITY, |p| *p.rotation()) * joint.frame1.rotation;
        let relative = rotation.inverse() * *child.rotation() * joint.frame2.rotation;
        let velocity =
            rotation.inverse() * (child.angvel() - parent.map_or(Vector::ZERO, |p| p.angvel()));
        Some(BallJointState {
            orientation: relative.normalize().to_array(),
            angular_velocity: velocity.to_array(),
        })
    }
}

impl Construction {
    pub(super) fn world_joint_frames(
        self,
        world: &PhysicsWorld,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) -> (Rotation, Rotation) {
        let parent = self.parent.map_or(Rotation::IDENTITY, |p| {
            *world.bodies[bodies[&p].0].rotation()
        });
        (
            parent * self.frame1.rotation,
            *world.bodies[bodies[&self.child].0].rotation() * self.frame2.rotation,
        )
    }

    pub(super) fn motor_axis(self) -> JointAxis {
        if matches!(self.motion, JointMotion::Prismatic) {
            JointAxis::LinX
        } else {
            JointAxis::AngX
        }
    }

    pub(super) fn slider(
        &self,
        world: &PhysicsWorld,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) -> JointState {
        let parent = self.parent.map(|p| &world.bodies[bodies[&p].0]);
        let child = &world.bodies[bodies[&self.child].0];
        let parent_frame = parent.map_or(self.frame1, |p| *p.position() * self.frame1);
        let point = *child.position() * self.frame2.translation;
        let axis = parent_frame.rotation * Vector::X;
        // Evaluating both twists at this same material point differentiates
        // displacement in a rotating parent frame, including its axis motion.
        JointState {
            position: (point - parent_frame.translation).dot(axis),
            velocity: (child.velocity_at_point(point)
                - parent.map_or(Vector::ZERO, |p| p.velocity_at_point(point)))
            .dot(axis),
        }
    }
}

#[cfg(test)]
#[path = "../../tests/private/mounted_guide.rs"]
mod mounted_guide_tests;
