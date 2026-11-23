// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// Continuous angles in radians and rates in radians/second, first parent axis
/// followed by second child axis. Both angles are zero in the authored pose.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UniversalJointState {
    pub position: [f64; 2],
    pub velocity: [f64; 2],
}

#[derive(Clone, Copy)]
pub(super) struct UniversalSettings {
    limits: [Option<[f64; 2]>; 2],
    effort_limits: [f64; 2],
}

impl UniversalSettings {
    pub(super) fn range(self, i: usize) -> Option<[f64; 2]> {
        self.limits[i]
    }
    pub(super) fn effort_limit(self, axis: usize) -> Option<f64> {
        self.effort_limits.get(axis).copied()
    }
}

pub(super) fn configuration(
    axes: [[f64; 3]; 2],
    limits: [Option<[f64; 2]>; 2],
    effort_limits: [f64; 2],
) -> Result<AngularConstraint, PhysicsError> {
    let [Some(x), Some(y)] = axes.map(|a| Vector::from_array(a).try_normalize()) else {
        return Err(PhysicsError::InvalidJoint);
    };
    if !x.is_finite() || !y.is_finite() || x.dot(y).abs() > 1e-8 {
        return Err(PhysicsError::InvalidJoint);
    }
    if effort_limits.into_iter().any(|v| v.is_nan() || v < 0.) {
        return Err(PhysicsError::InvalidValue);
    }
    for [lo, hi] in limits.into_iter().flatten() {
        if !(lo <= 0. && hi >= 0.) {
            return Err(PhysicsError::InvalidValue);
        }
    }
    let initial = Rotation::from_rotation_arc(Vector::X, x);
    let y = initial.inverse() * y;
    let basis = initial * Rotation::from_rotation_x(y.z.atan2(y.y));
    Ok(AngularConstraint::Universal(
        basis,
        UniversalSettings {
            limits,
            effort_limits,
        },
    ))
}

/// Rapier's coupled swing row measures the unsigned angle between the two local X
/// directions in world space and uses their cross product as its angular Jacobian.
fn cone(mask: JointAxesMask, parent: Pose, child: Pose, range: [f64; 2]) -> GenericJoint {
    GenericJointBuilder::new(mask)
        .local_frame1(parent)
        .local_frame2(child)
        .coupled_axes(JointAxesMask::ANG_Y | JointAxesMask::ANG_Z)
        .limits(JointAxis::AngY, range)
        .contacts_enabled(false)
        .build()
}

fn witness(mut frame: Pose, direction: Vector) -> Pose {
    frame.rotation *= Rotation::from_rotation_arc(Vector::X, direction);
    frame
}

pub(super) type SpringRows = [Option<(RobotSpring, ImpulseJointHandle)>; 2];

impl Construction {
    pub(super) fn universal_axis_row(
        self,
        parent: RigidBodyHandle,
        child: RigidBodyHandle,
        index: usize,
    ) -> (
        RigidBodyHandle,
        RigidBodyHandle,
        GenericJointBuilder,
        JointAxis,
    ) {
        // Rapier takes each motor axis from its first body. The second
        // universal axis follows the child through rotation about the first.
        let (first, second, a, b, axis) = if index == 0 {
            (parent, child, self.frame1, self.frame2, JointAxis::AngX)
        } else {
            (child, parent, self.frame2, self.frame1, JointAxis::AngY)
        };
        let row = GenericJointBuilder::new(JointAxesMask::empty())
            .local_frame1(a)
            .local_frame2(b)
            .contacts_enabled(false);
        (first, second, row, axis)
    }
    pub(super) fn insert_universal_springs(
        self,
        world: &mut PhysicsWorld,
        parent: RigidBodyHandle,
        child: RigidBodyHandle,
        laws: [Option<RobotSpring>; 2],
    ) -> SpringRows {
        std::array::from_fn(|i| {
            let law = laws[i]?;
            let (first, second, row, axis) = self.universal_axis_row(parent, child, i);
            let row = row
                .motor_model(axis, MotorModel::ForceBased)
                .motor_velocity(axis, 0., law.damping);
            Some((law, world.insert_impulse_joint(first, second, row)))
        })
    }

    pub(super) fn universal_anchor(self) -> GenericJoint {
        cone(
            JointAxesMask::LIN_AXES,
            self.frame1,
            witness(self.frame2, Vector::Y),
            [FRAC_PI_2; 2],
        )
    }

    pub(super) fn insert_universal_stops(
        self,
        world: &mut PhysicsWorld,
        parent: RigidBodyHandle,
        child: RigidBodyHandle,
    ) -> [Option<ImpulseJointHandle>; 2] {
        let JointMotion::Universal(settings) = self.motion else {
            return [None; 2];
        };
        std::array::from_fn(|i| {
            let range = settings.limits[i]?;

            Some(world.insert_impulse_joint(parent, child, self.universal_stop(i, range)))
        })
    }

    pub(super) fn universal_stop(self, axis: usize, range: [f64; 2]) -> GenericJoint {
        let (a, b, limits) = self.stop_frames(axis, range, 0.);
        cone(JointAxesMask::empty(), a, b, limits)
    }

    /// On the constraint manifold the child Y direction depends only on q1
    /// and the parent X direction in the child frame depends only on -q2.
    /// Keep each witness perpendicular at its reference angle so the angular
    /// Jacobian stays well conditioned, including at a zero-width lock.
    pub(super) fn stop_frames(
        self,
        axis: usize,
        [lo, hi]: [f64; 2],
        angle: f64,
    ) -> (Pose, Pose, [f64; 2]) {
        let center = if hi - lo <= FRAC_PI_2 {
            (lo + hi) / 2.
        } else {
            angle
        };
        let offset = center - FRAC_PI_2;
        let (a, b) = if axis == 0 {
            (Rotation::from_rotation_x(offset) * Vector::Y, Vector::Y)
        } else {
            (Vector::X, Rotation::from_rotation_y(-offset) * Vector::X)
        };
        (
            witness(self.frame1, a),
            witness(self.frame2, b),
            [
                (FRAC_PI_2 + (lo - center)).max(-1.),
                (FRAC_PI_2 + (hi - center)).min(PI + 1.),
            ],
        )
    }

    pub(super) fn wide_universal(self) -> bool {
        matches!(self.motion, JointMotion::Universal(s) if s.limits.into_iter().flatten().any(|[lo,hi]|hi-lo > FRAC_PI_2))
    }
    pub(super) fn universal_frames(
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
}

impl Joints {
    pub(crate) fn prepare_universal_axes(
        &self,
        world: &mut PhysicsWorld,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) {
        for (id, rows) in &self.universal_springs {
            let joint = &self.records[id];
            let (parent, child) = joint.initial.universal_frames(world, bodies);
            let angles = joint.measured_angles(parent.inverse() * child);
            for (i, row) in rows.iter().enumerate() {
                if let Some((law, handle)) = row {
                    let (rate, damping) =
                        law.angular_response(angles[i], world.integration_parameters.dt);
                    let (axis, speed) = if i == 0 {
                        (JointAxis::AngX, rate)
                    } else {
                        (JointAxis::AngY, -rate)
                    };
                    world
                        .impulse_joints
                        .get_mut(*handle, false)
                        .expect("resident universal spring")
                        .data
                        .set_motor_velocity(axis, speed, damping);
                }
            }
        }
        for id in &self.wide_universal {
            let joint = &self.records[id];
            let c = joint.initial;
            let JointMotion::Universal(settings) = c.motion else {
                unreachable!()
            };
            let (parent, child) = c.universal_frames(world, bodies);
            let angles = joint.measured_angles(parent.inverse() * child);
            for (axis, angle) in angles.into_iter().enumerate() {
                let Some(range) = settings.limits[axis] else {
                    continue;
                };
                if range[1] - range[0] <= FRAC_PI_2 {
                    continue;
                }
                let (a, b, limits) = c.stop_frames(axis, range, angle);
                let row = world
                    .impulse_joints
                    .get_mut(
                        joint.universal_stops[axis].expect("registered universal stop"),
                        false,
                    )
                    .expect("resident stop");
                if self.projected_universal.contains_key(id) {
                    row.data = universal_projection::projected(a, b, limits);
                } else {
                    row.data
                        .set_local_frame1(a)
                        .set_local_frame2(b)
                        .set_limits(JointAxis::AngY, limits);
                }
            }
        }
    }
}

impl Joint {
    pub(super) fn update_universal_angles(&mut self, relative: Rotation) {
        self.universal_angles = self.measured_angles(relative);
    }

    fn measured_angles(&self, relative: Rotation) -> [f64; 2] {
        let y = relative * Vector::Y;
        let x = relative.inverse() * Vector::X;
        let wrapped = [y.z.atan2(y.y), x.z.atan2(x.x)];
        std::array::from_fn(|i| {
            let previous = self.universal_angles[i];
            previous + (wrapped[i] - previous + PI).rem_euclid(TAU) - PI
        })
    }
}

impl RapierBackend {
    /// Add independent passive torsional springs and dampers on the parent and child axes.
    pub fn add_universal_joint_with_springs(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        joint: RobotJoint,
        springs: [Option<RobotSpring>; 2],
    ) -> Result<JointId, PhysicsError> {
        if !matches!(joint, RobotJoint::Universal { .. }) {
            return Err(PhysicsError::InvalidJoint);
        }
        if springs
            .iter()
            .flatten()
            .any(|spring| !spring.admissible_continuous())
        {
            return Err(PhysicsError::InvalidValue);
        }
        let id = self.add_robot_joint(parent, child, anchor, joint)?;
        if springs.iter().any(Option::is_some) {
            let parent = parent.map_or_else(
                || self.joints.anchor.expect("world joint anchor"),
                |p| self.bodies[&p].0,
            );
            let construction = self.joints.records[&id].initial;
            let rows = construction.insert_universal_springs(
                &mut self.world,
                parent,
                self.bodies[&child].0,
                springs,
            );
            self.joints.universal_springs.insert(id, rows);
        }
        Ok(id)
    }

    pub fn universal_joint_state(&self, id: JointId) -> Option<UniversalJointState> {
        let j = self.joints.records.get(&id)?;
        let c = j.initial;
        if !matches!(c.motion, JointMotion::Universal(_)) {
            return None;
        }
        let (parent, child) = c.universal_frames(&self.world, &self.bodies);
        let omega = self.world.bodies[self.bodies[&c.child].0].angvel()
            - c.parent.map_or(Vector::ZERO, |p| {
                self.world.bodies[self.bodies[&p].0].angvel()
            });
        let a = parent * Vector::X;
        let b = child * Vector::Y;
        // Invert the two-axis Gram matrix to handle finite constraint residuals.
        let dot = a.dot(b);
        let denom = 1. - dot * dot;
        let (w1, w2) = (a.dot(omega), b.dot(omega));
        Some(UniversalJointState {
            position: j.universal_angles,
            velocity: [(w1 - dot * w2) / denom, (w2 - dot * w1) / denom],
        })
    }

    pub fn add_universal_effort(
        &mut self,
        id: JointId,
        efforts: [f64; 2],
    ) -> Result<(), PhysicsError> {
        let j = self
            .joints
            .records
            .get(&id)
            .ok_or(PhysicsError::MissingJoint)?;
        let JointMotion::Universal(settings) = j.initial.motion else {
            return Err(PhysicsError::InvalidJoint);
        };
        let old = self
            .joints
            .universal_efforts
            .get(&id)
            .copied()
            .unwrap_or_default();
        let sum = [old[0] + efforts[0], old[1] + efforts[1]];
        if (0..2).any(|i| {
            !sum[i].is_finite()
                || sum[i].abs() > settings.effort_limits[i]
                || efforts[i].abs() > settings.effort_limits[i]
        }) {
            return Err(PhysicsError::InvalidValue);
        }
        self.joints.universal_efforts.insert(id, sum);
        Ok(())
    }
}
