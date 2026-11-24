// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Clone, Copy)]
pub(super) struct SpeedSide {
    reference: RigidBodyHandle,
    row: ImpulseJointHandle,
    drive: Option<MultibodyJointHandle>,
}

#[derive(Clone)]
pub(super) enum AxisCap {
    Held(ImpulseJointHandle),
    Bounded { limit: f64, sides: [SpeedSide; 2] },
}

pub(super) type SpeedCaps = [Option<AxisCap>; 2];

impl Construction {
    pub(super) fn insert_speed_caps(
        self,
        world: &mut PhysicsWorld,
        parent: RigidBodyHandle,
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
                    world.insert_impulse_joint(parent, child, lock),
                ));
            }
            let sides = std::array::from_fn(|j| {
                let sign = if j == 0 { -1. } else { 1. };
                let (reference, drive, first, second, axis) = if !world.bodies[parent].is_fixed() {
                    let (bearing, orientation, velocity, opposite, native_axis) = if i == 0 {
                        (
                            parent,
                            self.frame1.rotation,
                            sign * limit,
                            child,
                            JointAxis::AngX,
                        )
                    } else {
                        // The child-side witness spins oppositely because the
                        // row measures parent velocity relative to the witness.
                        (
                            child,
                            self.frame2.rotation
                                * Rotation::from_rotation_z(std::f64::consts::FRAC_PI_2),
                            -sign * limit,
                            parent,
                            JointAxis::AngY,
                        )
                    };
                    let (reference, drive) =
                        speed_reference::moving_reference(world, bearing, orientation, velocity);
                    (reference, Some(drive), reference, opposite, native_axis)
                } else {
                    let reference = world.insert_body(RigidBodyBuilder::kinematic_velocity_based());
                    if i == 0 {
                        (reference, None, reference, child, JointAxis::AngX)
                    } else {
                        (reference, None, child, reference, JointAxis::AngY)
                    }
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
                SpeedSide {
                    reference,
                    drive,
                    row: world.insert_impulse_joint(first, second, row),
                }
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
        parked: &BTreeMap<BodyId, parked_speed::ParkedSpeed>,
    ) {
        for (id, caps) in &self.speed_caps {
            if parked.values().any(|group| group.joints.contains(id)) {
                continue;
            }
            let joint = self.records[id].initial;
            let child = *world.bodies[bodies[&joint.child].0].rotation();
            let parent = joint.parent.map_or(Rotation::IDENTITY, |p| {
                *world.bodies[bodies[&p].0].rotation()
            });
            for (axis, cap) in caps.iter().enumerate() {
                let Some(AxisCap::Bounded { limit, sides }) = cap else {
                    continue;
                };
                let basis = if axis == 0 {
                    parent * joint.frame1.rotation
                } else {
                    child * joint.frame2.rotation
                };
                let direction = if axis == 0 { Vector::X } else { Vector::Y };
                for (
                    side,
                    &SpeedSide {
                        reference,
                        row,
                        drive,
                    },
                ) in sides.iter().enumerate()
                {
                    let sign = if side == 0 { -1. } else { 1. };
                    // The offset keeps one sign of reaction available through
                    // all internal iterations. With zero position correction
                    // the row limits relative angular velocity alone.
                    let phase =
                        Rotation::from_axis_angle(direction, -sign * std::f64::consts::FRAC_PI_4);
                    let data = &mut world
                        .impulse_joints
                        .get_mut(row, false)
                        .expect("registered speed limit")
                        .data;
                    if drive.is_some() {
                        let (phase, opposite) = if axis == 0 {
                            (phase, child)
                        } else {
                            (phase.inverse(), parent)
                        };
                        data.set_local_frame1(Pose::from_rotation(
                            world.bodies[reference].rotation().inverse() * basis * phase,
                        ))
                        .set_local_frame2(Pose::from_rotation(opposite.inverse() * basis));
                    } else {
                        let rb = &mut world.bodies[reference];
                        rb.set_rotation(basis * phase, false);
                        rb.set_angvel(basis * direction * (sign * limit), false);
                        let child_frame = Pose::from_rotation(child.inverse() * basis);
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
}

impl AxisCap {
    pub(super) fn references(
        &self,
    ) -> impl Iterator<Item = (RigidBodyHandle, Option<MultibodyJointHandle>)> {
        let sides: &[SpeedSide] = match self {
            Self::Held(_) => &[],
            Self::Bounded { sides, .. } => sides,
        };
        sides.iter().map(|s| (s.reference, s.drive))
    }

    pub(super) fn drive_handles(&self) -> impl Iterator<Item = MultibodyJointHandle> {
        let sides: &[SpeedSide] = match self {
            Self::Held(_) => &[],
            Self::Bounded { sides, .. } => sides,
        };
        sides.iter().filter_map(|s| s.drive)
    }

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
                for side in sides {
                    world.remove_body(side.reference);
                }
            }
        }
    }
}

impl RapierBackend {
    /// Native calls per second for the rated torque of a finite, speed-limited screw.
    /// A free parent/child couple changes their relative speed by the sum of both
    /// projected inverse inertias times torque. Budget one speed-limit increment
    /// per existing temporal solver sweep. Constraints and external loads can change
    /// the actual acceleration.
    pub fn screw_stop_resolution_rate(&self, id: JointId) -> Option<f64> {
        let c = self.joints.records.get(&id)?.initial;
        let JointMotion::Screw { effort_limit, .. } = c.motion else {
            return None;
        };
        let Some(AxisCap::Bounded { limit, .. }) = &self.joints.speed_caps.get(&id)?[0] else {
            return None;
        };
        if !effort_limit.is_finite() || effort_limit <= 0. {
            return None;
        }
        let body = |id| &self.world.bodies[self.bodies[&id].0];
        let inverse = |rb: &RigidBody, rotation: Rotation| {
            let props = &rb.mass_properties().local_mprops;
            let axis = props.principal_inertia_local_frame.inverse() * (rotation * Vector::X);
            (axis * axis).dot(props.inv_principal_inertia)
        };
        let child = body(c.child);
        let child_inverse = inverse(child, c.frame2.rotation);
        let parent_inverse = c
            .parent
            .map_or(0., |id| inverse(body(id), c.frame1.rotation));
        let sweeps = self.world.integration_parameters.num_solver_iterations
            + child.additional_solver_iterations();
        Some((child_inverse + parent_inverse) * effort_limit / limit / (sweeps as f64))
    }

    /// Set physical angular speed limits on the coordinates of a world-mounted
    /// screw or universal joint during construction. Limits use radians/second.
    pub fn set_world_joint_speed_limits(
        &mut self,
        id: JointId,
        limits: [Option<f64>; 2],
    ) -> Result<(), PhysicsError> {
        if self
            .joints
            .records
            .get(&id)
            .is_some_and(|j| j.initial.parent.is_some())
        {
            return Err(PhysicsError::InvalidJoint);
        }
        self.set_joint_angular_speed_limits(id, limits)
    }

    /// Set or replace hard angular speed limits during robot construction.
    pub fn set_joint_angular_speed_limits(
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
        if !matches!(joint.motion, JointMotion::Universal(_))
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
                joint.parent.map_or_else(
                    || self.joints.anchor.expect("world speed limit anchor"),
                    |p| self.bodies[&p].0,
                ),
                self.bodies[&joint.child].0,
                limits,
            );
            self.joints.speed_caps.insert(id, caps);
        }
        self.release_unconstrained_body(joint.child);
        Ok(())
    }
}
