// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fallible preparation before an infallible, standalone native body edit.
use super::*;

pub(super) fn prepare(
    backend: &RapierBackend,
    id: BodyId,
    state: BodyState,
) -> Result<(RigidBodyHandle, Pose), PhysicsError> {
    let (handle, ..) = backend.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
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
    let pose = Pose::from_parts(Vector::from(state.position), rotation(state.orientation)?);
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
