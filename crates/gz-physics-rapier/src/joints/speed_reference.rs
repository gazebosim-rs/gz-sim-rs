// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// A massless link with prescribed relative spin. Native impulse-joint reactions
/// on this link project through the multibody Jacobian onto its moving bearing.
pub(super) fn moving_reference(
    world: &mut PhysicsWorld,
    bearing: RigidBodyHandle,
    orientation: Rotation,
    speed: f64,
) -> (RigidBodyHandle, MultibodyJointHandle) {
    let body = &world.bodies[bearing];
    let position = body.translation();
    let rotation = *body.rotation() * orientation;
    let omega = body.angvel() + rotation * Vector::X * speed;
    let reference = world.insert_body(
        RigidBodyBuilder::dynamic()
            .pose(Pose::from_parts(position, rotation))
            .linvel(body.velocity_at_point(position))
            .angvel(omega)
            .can_sleep(false),
    );
    let axis = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES)
        .local_frame1(Pose::from_rotation(orientation))
        .contacts_enabled(false);
    let handle = world
        .multibody_joints
        .insert_kinematic(bearing, reference, axis, true)
        .expect("new private reference links cannot form a cycle");
    (reference, handle)
}
