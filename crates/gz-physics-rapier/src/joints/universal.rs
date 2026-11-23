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

impl Construction {
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
            let (a, b, limits) = self.stop_frames(i, range, 0.);
            Some(world.insert_impulse_joint(
                parent,
                child,
                cone(JointAxesMask::empty(), a, b, limits),
            ))
        })
    }

    /// On the constraint manifold the child Y direction depends only on q1
    /// and the parent X direction in the child frame depends only on -q2.
    /// Keep each witness perpendicular at its reference angle so the angular
    /// Jacobian stays well conditioned, including at a zero-width lock.
    fn stop_frames(self, axis: usize, [lo, hi]: [f64; 2], angle: f64) -> (Pose, Pose, [f64; 2]) {
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
    pub(crate) fn prepare_universal_stops(
        &self,
        world: &mut PhysicsWorld,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) {
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
                row.data
                    .set_local_frame1(a)
                    .set_local_frame2(b)
                    .set_limits(JointAxis::AngY, limits);
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
