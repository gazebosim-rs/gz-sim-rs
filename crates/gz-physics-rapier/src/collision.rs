// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

pub(super) struct Groups;

/// Shape-level filters common to prediction and the native contact solver.
/// Body type is evaluated after the selected arm switches to dynamic.
pub(super) fn can_exchange_impulses(a: &Collider, b: &Collider, other_type: RigidBodyType) -> bool {
    a.is_enabled()
        && b.is_enabled()
        && !a.is_sensor()
        && !b.is_sensor()
        && a.collision_groups().test(b.collision_groups())
        && a.solver_groups().test(b.solver_groups())
        && (a.active_collision_types() | b.active_collision_types())
            .test(RigidBodyType::Dynamic, other_type)
        && !(a.user_data != 0 && a.user_data == b.user_data)
}

#[derive(Clone, Copy)]
pub(crate) struct SuppressedRows {
    pub foot: ColliderHandle,
    pub pad: ColliderHandle,
    pub second: Option<ColliderHandle>,
}
pub(super) struct SupportedGroups<'a>(
    pub(super) &'a BTreeMap<BodyId, Vec<articulation_support::SupportHold>>,
    pub(super) &'a [SuppressedRows],
    pub(super) &'a [axial_cap::Impact],
    #[cfg(feature = "landing-audit")] pub(super) Option<&'a landing_row_order::LandingRows>,
);
impl PhysicsHooks for SupportedGroups<'_> {
    fn filter_contact_pair(&self, ctx: &PairFilterContext) -> Option<SolverFlags> {
        for rows in self.1 {
            let opposed = |other| other == rows.pad || Some(other) == rows.second;
            if (ctx.collider1 == rows.foot && opposed(ctx.collider2))
                || (ctx.collider2 == rows.foot && opposed(ctx.collider1))
            {
                return None;
            }
        }
        Groups.filter_contact_pair(ctx)
    }

    fn modify_solver_contacts(&self, ctx: &mut ContactModificationContext) {
        // Suppress just the captured link–floor pair. New contacts on other
        // surfaces still generate their own geometry and solver observations.
        let held = [
            (ctx.rigid_body1, ctx.collider1, ctx.collider2),
            (ctx.rigid_body2, ctx.collider2, ctx.collider1),
        ]
        .into_iter()
        .any(|(body, link, floor)| {
            body.and_then(|h| {
                self.0
                    .get(&BodyId::from_raw(ctx.bodies[h].user_data as u64))
            })
            .is_some_and(|holds| holds.iter().any(|s| s.is_pair(link, floor)))
        });
        if held && let Some(m) = ctx.rigid_mut() {
            m.solver_contacts.clear();
        }
        for hit in self.2 {
            hit.apply(ctx);
        }
        #[cfg(feature = "landing-audit")]
        if !held && let Some(rows) = self.3 {
            rows.apply(ctx);
        }
    }
}

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
        let mut hooks = collider.active_hooks();
        hooks.set(ActiveHooks::FILTER_CONTACT_PAIRS, group.is_some());
        collider.set_active_hooks(hooks);
    }
}
