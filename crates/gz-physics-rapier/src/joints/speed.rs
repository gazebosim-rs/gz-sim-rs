// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Clone)]
pub(super) enum AxisCap {
    Held(ImpulseJointHandle),
    Bounded {
        limit: f64,
        sides: [(RigidBodyHandle, ImpulseJointHandle); 2],
    },
}

pub(super) type SpeedCaps = [Option<AxisCap>; 2];

impl Construction {
    pub(super) fn insert_speed_caps(
        self,
        world: &mut PhysicsWorld,
        anchor: RigidBodyHandle,
        child: RigidBodyHandle,
        limits: [Option<f64>; 2],
    ) -> SpeedCaps {
        world.bodies[child].set_additional_solver_iterations(4);
        std::array::from_fn(|i| {
            let limit = limits[i]?;
            if limit == 0. {
                // A zero speed from construction is a fixed coordinate. Preserve
                // its original angle instead of re-centering it at every step.
                let lock = if matches!(self.motion, JointMotion::Screw { .. }) {
                    GenericJointBuilder::new(JointAxesMask::LOCKED_FIXED_AXES)
                        .local_frame1(self.frame1)
                        .local_frame2(self.frame2)
                        .contacts_enabled(false)
                        .build()
                } else {
                    self.universal_stop(i, [0.; 2])
                };
                return Some(AxisCap::Held(
                    world.insert_impulse_joint(anchor, child, lock),
                ));
            }
            let sides = std::array::from_fn(|j| {
                let reference = world.insert_body(RigidBodyBuilder::kinematic_velocity_based());
                let (first, second, axis) = if i == 0 {
                    (reference, child, JointAxis::AngX)
                } else {
                    (child, reference, JointAxis::AngY)
                };
                let range = if i == j {
                    [0., std::f64::consts::FRAC_PI_2]
                } else {
                    [-std::f64::consts::FRAC_PI_2, 0.]
                };
                let row = GenericJointBuilder::new(JointAxesMask::empty())
                    // Keep the unilateral velocity row hard while suppressing
                    // positional correction of the intentionally offset witness.
                    .softness(SpringCoefficients::new(0., 1.))
                    .limits(axis, range)
                    .contacts_enabled(false);
                (reference, world.insert_impulse_joint(first, second, row))
            });
            Some(AxisCap::Bounded { limit, sides })
        })
    }
}

impl Joints {
    pub(crate) fn prepare_speed_caps(
        &self,
        world: &mut PhysicsWorld,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) {
        for (id, caps) in &self.speed_caps {
            let joint = self.records[id].initial;
            let child = *world.bodies[bodies[&joint.child].0].rotation();
            for (axis, cap) in caps.iter().enumerate() {
                let Some(AxisCap::Bounded { limit, sides }) = cap else {
                    continue;
                };
                let basis = if axis == 0 {
                    joint.frame1.rotation
                } else {
                    child * joint.frame2.rotation
                };
                let direction = if axis == 0 { Vector::X } else { Vector::Y };
                for (side, &(reference, row)) in sides.iter().enumerate() {
                    let sign = if side == 0 { -1. } else { 1. };
                    // The offset keeps one sign of reaction available through
                    // all internal iterations. With zero position correction
                    // the row limits relative angular velocity alone.
                    let rb = &mut world.bodies[reference];
                    rb.set_rotation(
                        basis
                            * Rotation::from_axis_angle(
                                direction,
                                -sign * std::f64::consts::FRAC_PI_4,
                            ),
                        false,
                    );
                    rb.set_angvel(basis * direction * (sign * limit), false);
                    let child_frame = Pose::from_rotation(child.inverse() * basis);
                    let data = &mut world
                        .impulse_joints
                        .get_mut(row, false)
                        .expect("registered speed limit")
                        .data;
                    if axis == 0 {
                        data.set_local_frame2(child_frame);
                    } else {
                        data.set_local_frame1(child_frame);
                    }
                }
            }
        }
    }
}

impl AxisCap {
    pub(super) fn limit(&self) -> f64 {
        match self {
            Self::Held(_) => 0.,
            Self::Bounded { limit, .. } => *limit,
        }
    }
    pub(super) fn remove(self, world: &mut PhysicsWorld) {
        match self {
            Self::Held(handle) => {
                world.remove_impulse_joint(handle);
            }
            Self::Bounded { sides, .. } => {
                for (reference, _) in sides {
                    world.remove_body(reference);
                }
            }
        }
    }
}

impl RapierBackend {
    /// Set physical angular speed limits on the coordinates of a world-mounted
    /// screw or universal joint during construction. Limits use radians/second.
    pub fn set_world_joint_speed_limits(
        &mut self,
        id: JointId,
        limits: [Option<f64>; 2],
    ) -> Result<(), PhysicsError> {
        if !self.joints.open {
            return Err(PhysicsError::JointConstructionClosed);
        }
        let joint = self
            .joints
            .records
            .get(&id)
            .ok_or(PhysicsError::MissingJoint)?
            .initial;
        if joint.parent.is_some()
            || !matches!(joint.motion, JointMotion::Universal(_))
                && !(matches!(joint.motion, JointMotion::Screw { .. }) && limits[1].is_none())
        {
            return Err(PhysicsError::InvalidJoint);
        }
        if !limits.iter().flatten().all(|v| v.is_finite() && *v >= 0.) {
            return Err(PhysicsError::InvalidValue);
        }
        if let Some(old) = self.joints.speed_caps.remove(&id) {
            for cap in old.into_iter().flatten() {
                cap.remove(&mut self.world);
            }
        }
        if limits.iter().any(Option::is_some) {
            let caps = joint.insert_speed_caps(
                &mut self.world,
                self.joints.anchor.expect("world speed limit anchor"),
                self.bodies[&joint.child].0,
                limits,
            );
            self.joints.speed_caps.insert(id, caps);
        }
        self.release_unconstrained_body(joint.child);
        Ok(())
    }
}
