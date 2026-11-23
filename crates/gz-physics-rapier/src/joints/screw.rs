// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

impl Construction {
    fn screw_frames(self, parent: Pose, child: Pose, angle: f64, lead: f64) -> (Pose, Pose) {
        // In the cylindrical guide's frame the only free motions are x and twist.
        // A linear row along x+y, applied at r=lead*z, measures exactly
        // (dx - lead*dθ)/sqrt(2) on that free subspace since x×z = -y.
        // Shift the parent attachment by the true accumulated helical travel.
        // This gives the row the position residual as well as its tangent.
        let base = parent * self.frame1;
        let radius = base.rotation * (Vector::Z * lead);
        let normal = Rotation::from_rotation_z(std::f64::consts::FRAC_PI_4);
        let mut frame1 = self.frame1;
        frame1.translation +=
            self.frame1.rotation * (Vector::X * (angle * lead) + Vector::Z * lead);
        frame1.rotation *= normal;
        let mut frame2 = self.frame2;
        frame2.translation += child.rotation.inverse() * radius;
        (frame1, frame2)
    }

    pub(super) fn screw_twist(
        self,
        world: &PhysicsWorld,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) -> f64 {
        let parent = self.parent.map_or(Rotation::IDENTITY, |p| {
            *world.bodies[bodies[&p].0].rotation()
        });
        let child = *world.bodies[bodies[&self.child].0].rotation();
        let rel = (parent * self.frame1.rotation).inverse() * child * self.frame2.rotation;
        2. * rel.x.atan2(rel.w)
    }

    pub(super) fn insert_screw_row(
        self,
        world: &mut PhysicsWorld,
        parent: RigidBodyHandle,
        child: RigidBodyHandle,
    ) -> Option<ImpulseJointHandle> {
        let JointMotion::Screw { lead, .. } = self.motion else {
            return None;
        };
        let (a, b) = self.screw_frames(
            *world.bodies[parent].position(),
            *world.bodies[child].position(),
            0.,
            lead,
        );
        let row = GenericJointBuilder::new(JointAxesMask::LIN_X)
            .local_frame1(a)
            .local_frame2(b)
            .contacts_enabled(false);
        Some(world.insert_impulse_joint(parent, child, row))
    }
}

impl Joints {
    pub(crate) fn prepare_screws(
        &self,
        world: &mut PhysicsWorld,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) {
        for (id, handle) in &self.screws {
            let joint = &self.records[id];
            let initial = joint.initial;
            let JointMotion::Screw { lead, .. } = initial.motion else {
                unreachable!()
            };
            let parent = initial
                .parent
                .map_or(Pose::IDENTITY, |p| *world.bodies[bodies[&p].0].position());
            let child = *world.bodies[bodies[&initial.child].0].position();
            let current = initial.screw_twist(world, bodies);
            let delta = (current - joint.previous_angle + std::f64::consts::PI)
                .rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            let position = joint.position + delta;
            if let Some((spring, handle)) = initial.spring.zip(joint.spring) {
                let (target_rate, damping) =
                    spring.angular_response(position, world.integration_parameters.dt);
                world
                    .impulse_joints
                    .get_mut(handle, false)
                    .expect("registered passive spring")
                    .data
                    .set_motor_velocity(JointAxis::AngX, target_rate, damping);
            }
            let (a, b) = initial.screw_frames(parent, child, position, lead);
            let row = world
                .impulse_joints
                .get_mut(*handle, false)
                .expect("registered screw row");
            row.data.set_local_frame1(a).set_local_frame2(b);
            if let Some([lower, upper]) = initial.limits {
                // Arrest both free components at a stop. A linear limit alone leaves
                // the rotor spinning until the next iterative coupling pass, which
                // stores enough error in the constraint to cause a spurious rebound.
                // Recenter the periodic native angular limit about this step's turn.
                let Handle::Impulse(guide) = joint.handle else {
                    unreachable!()
                };
                let guide = &mut world
                    .impulse_joints
                    .get_mut(guide, false)
                    .expect("registered screw guide")
                    .data;
                let half_pi = std::f64::consts::FRAC_PI_2;
                let mut frame = initial.frame2;
                frame.rotation *= Rotation::from_rotation_x(-current);
                guide.set_local_frame2(frame);
                if lower - position > -half_pi || upper - position < half_pi {
                    guide.set_limits(
                        JointAxis::AngX,
                        [
                            (lower - position).clamp(-half_pi, half_pi),
                            (upper - position).clamp(-half_pi, half_pi),
                        ],
                    );
                } else {
                    guide.limit_axes.remove(JointAxesMask::ANG_X);
                }
            }
        }
    }
}
