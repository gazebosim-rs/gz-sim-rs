// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

pub(super) struct Groups;

impl PhysicsHooks for Groups {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        // Retain the exclusion of contact pairs with no dynamic body.
        let dynamic =
            |body: Option<RigidBodyHandle>| body.is_some_and(|id| context.bodies[id].is_dynamic());
        if !dynamic(context.rigid_body1) && !dynamic(context.rigid_body2) {
            return None;
        }
        let a = context.colliders[context.collider1].user_data;
        let b = context.colliders[context.collider2].user_data;
        if a != 0 && a == b {
            None
        } else {
            Some(SolverFlags::COMPUTE_RIGID_IMPULSES)
        }
    }
}

pub(super) fn assign(world: &mut PhysicsWorld, body: RigidBodyHandle, group: Option<u64>) {
    for collider in world.bodies[body].colliders() {
        let collider = &mut world.colliders[*collider];
        // Zero is unassigned; all u64 IDs fit injectively in this u128 field.
        collider.user_data = group.map_or(0, |id| u128::from(id) + 1);
        collider.set_active_hooks(if group.is_some() {
            ActiveHooks::FILTER_CONTACT_PAIRS
        } else {
            ActiveHooks::empty()
        });
    }
}
