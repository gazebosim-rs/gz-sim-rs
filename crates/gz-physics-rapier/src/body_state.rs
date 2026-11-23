// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fallible preparation before an infallible, standalone native body edit.
use super::*;

impl RapierBackend {
    /// Read the velocity committed by the most recent native solve. The relaxed
    /// generalized velocity precedes publication to individual rigid-body fields.
    pub(crate) fn generalized_link_twist(
        &self,
        handle: RigidBodyHandle,
    ) -> Option<(Vector, Vector)> {
        let joints = &self.world.multibody_joints;
        let member = joints.rigid_body_link(handle)?;
        let tree = joints.get_multibody(member.multibody)?;
        // Before the first native solve Rapier has not yet populated the
        // rectangular link Jacobians. Construction-time velocities live on
        // the bodies; these are also the values used to seed generalized motion.
        if tree.body_jacobian(member.id).ncols() != tree.ndofs() {
            if self.joints.construction_open() {
                let body = &self.world.bodies[handle];
                return Some((body.linvel(), body.angvel()));
            }
            return None;
        }
        let qdot = tree.generalized_velocity();
        let velocity = tree.body_jacobian(member.id) * qdot;
        let mut linear = Vector::new(velocity[0], velocity[1], velocity[2]);
        if tree.root().joint.kinematic {
            let root = &self.world.bodies[tree.root().rigid_body_handle()];
            if let Some(motion) = self
                .spinning_assemblies
                .get(&BodyId::from_raw(root.user_data as u64))
            {
                // Prescribed roots integrate along a chord. Publish the endpoint tangent.
                linear += motion.root_velocity(&self.world, None)
                    - Vector::new(qdot[0], qdot[1], qdot[2]);
            }
        }
        Some((linear, Vector::new(velocity[3], velocity[4], velocity[5])))
    }
}
pub(super) fn prepare(
    backend: &RapierBackend,
    id: BodyId,
    state: BodyState,
) -> Result<(RigidBodyHandle, Pose), PhysicsError> {
    let (handle, spec, _, inertia, _) = backend.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
    if backend.has_joint(id) == Some(true) {
        return Err(PhysicsError::Unsupported {
            engine: "rapier",
            capability: "jointed body-state replacement",
        });
    }
    let body = &backend.world.bodies[*handle];
    if !finite(state.position)
        || !finite(state.velocity)
        || !finite(state.angular_velocity)
        || (body.is_fixed() && (state.velocity != [0.0; 3] || state.angular_velocity != [0.0; 3]))
    {
        return Err(PhysicsError::InvalidValue);
    }
    if body.is_kinematic() {
        backend.validate_boundary_twist(state.velocity, state.angular_velocity)?;
    }
    let orientation = rotation(state.orientation)?;
    let mut translation = Vector::from_array(state.position);
    if let (Some(_), Some(inertia)) = (spec.articulation, inertia) {
        translation += orientation * Vector::from_array(inertia.center);
        if !translation.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
    }
    let pose = Pose::from_parts(translation, orientation);
    for collider in body.colliders() {
        let collider = &backend.world.colliders[*collider];
        let world_pose = pose * collider.position_wrt_parent().expect("parented collider");
        let bounds = collider.shape().compute_aabb(&world_pose);
        if !finite(bounds.mins.to_array()) || !finite(bounds.maxs.to_array()) {
            return Err(PhysicsError::InvalidValue);
        }
    }
    Ok((*handle, pose))
}

#[cfg(test)]
#[path = "../tests/private/body_state.rs"]
mod tests;
