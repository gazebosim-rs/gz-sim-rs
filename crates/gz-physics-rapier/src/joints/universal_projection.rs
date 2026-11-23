// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// Project a point one metre along the second witness onto the first witness
/// axis. With coincident joint origins its signed distance is cos(angle).
/// This is valid over the same [0,π] cone chart as the ordinary angular row.
pub(super) fn projected(parent: Pose, mut child: Pose, limits: [f64; 2]) -> GenericJoint {
    child.translation += child.rotation * Vector::X;
    let [lo, hi] = limits;
    // Keep a bound outside the cone chart inactive even at exact antipodes.
    let range = [
        if hi > std::f64::consts::PI {
            f64::NEG_INFINITY
        } else {
            hi.cos()
        },
        if lo < 0. { f64::INFINITY } else { lo.cos() },
    ];
    // A bilateral zero projection directly enforces perpendicularity.
    if lo == std::f64::consts::FRAC_PI_2 && hi == lo {
        return GenericJointBuilder::new(JointAxesMask::LIN_X)
            .local_frame1(parent)
            .local_frame2(child)
            .contacts_enabled(false)
            .build();
    }
    GenericJointBuilder::new(JointAxesMask::empty())
        .local_frame1(parent)
        .local_frame2(child)
        .limits(JointAxis::LinX, range)
        .contacts_enabled(false)
        .build()
}

impl Joints {
    /// Rapier uses a different joint solver for endpoints belonging to a
    /// multibody. That solver has ordinary linear rows but no coupled cone.
    pub(super) fn prepare_projected_universal(&mut self, world: &mut PhysicsWorld) {
        for (&id, joint) in &self.records {
            let JointMotion::Universal(settings) = joint.initial.motion else {
                continue;
            };
            if self.projected_universal.contains_key(&id) {
                continue;
            }
            let Handle::Impulse(handle) = joint.handle else {
                unreachable!()
            };
            let native = world
                .impulse_joints
                .get(handle)
                .expect("registered universal anchor");
            let (parent, child) = (native.body1(), native.body2());
            if world.multibody_joints.rigid_body_link(parent).is_none()
                && world.multibody_joints.rigid_body_link(child).is_none()
            {
                continue;
            }
            let c = joint.initial;
            world
                .impulse_joints
                .get_mut(handle, false)
                .expect("registered anchor")
                .data = GenericJointBuilder::new(JointAxesMask::LIN_AXES)
                .local_frame1(c.frame1)
                .local_frame2(c.frame2)
                .contacts_enabled(false)
                .build();
            let mut child_y = c.frame2;
            child_y.rotation *= Rotation::from_rotation_z(std::f64::consts::FRAC_PI_2);
            let orthogonality = projected(c.frame1, child_y, [std::f64::consts::FRAC_PI_2; 2]);
            self.projected_universal
                .insert(id, world.insert_impulse_joint(parent, child, orthogonality));
            for i in 0..2 {
                let zero = self.speed_caps.get(&id).and_then(|caps| match caps[i] {
                    Some(speed::AxisCap::Held(h)) => Some((h, [0.; 2])),
                    _ => None,
                });
                for (h, range) in joint.universal_stops[i]
                    .zip(settings.range(i))
                    .into_iter()
                    .chain(zero)
                {
                    let (a, b, limits) = c.stop_frames(i, range, joint.universal_angles[i]);
                    world
                        .impulse_joints
                        .get_mut(h, false)
                        .expect("registered angular stop")
                        .data = projected(a, b, limits);
                }
            }
        }
    }
}
