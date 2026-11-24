// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Observe load-bearing stationary supports independently of contact warm starting.
use super::*;
use rapier3d_f64::parry::query::distance;

/// Why a previously loaded contact can no longer justify the current prescribed hold.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportLossReason {
    /// The captured body or collider was retired.
    Retired,
    /// A stationary velocity boundary received a nonzero twist or a new pose.
    /// The next solve must resolve its contact, even while the shapes still touch.
    MotionStarted,
    /// The shapes have separated beyond their supported clearance, or their
    /// collision settings now prevent the pair from applying a reaction.
    Separated,
}

/// First observed loss during one prescribed interval. IDs belong to this backend.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SupportLoss {
    pub link: BodyId,
    pub support: BodyId,
    pub reason: SupportLossReason,
}

/// A prescribed hold assumes no motion of an infinite-mass support. Even
/// slow tangential and angular commands can change the resolved reaction.
pub(super) fn stationary_support(body: &RigidBody) -> bool {
    body.is_fixed()
        || (body.body_type() == RigidBodyType::KinematicVelocityBased
            && body.linvel() == Vector::ZERO
            && body.angvel() == Vector::ZERO)
}

pub(super) struct LoadedSupport {
    pub(super) link: BodyId,
    support: BodyId,
    foot: ColliderHandle,
    floor: ColliderHandle,
    foot_body: RigidBodyHandle,
    floor_body: RigidBodyHandle,
    previous_relative_pose: Pose,
    previous_skin: f64,
    initial_excess_gap: f64,
}

fn collider_pose(body: &RigidBody, collider: &Collider) -> Pose {
    body.position()
        * collider
            .position_wrt_parent()
            .expect("checked collider parent")
}

impl LoadedSupport {
    pub(super) fn capture(
        world: &PhysicsWorld,
        link: BodyId,
        support: BodyId,
        foot: ColliderHandle,
        floor: ColliderHandle,
    ) -> Self {
        let sole = &world.colliders[foot];
        let pad = &world.colliders[floor];
        let foot_body = sole.parent().expect("admitted articulation link");
        let floor_body = pad.parent().expect("admitted stationary support");
        let sole_pose = collider_pose(&world.bodies[foot_body], sole);
        let pad_pose = collider_pose(&world.bodies[floor_body], pad);
        let skin = sole.contact_skin() + pad.contact_skin();
        let initial_excess_gap = distance(&sole_pose, sole.shape(), &pad_pose, pad.shape())
            .map(|d| (d.distance - skin).max(0.))
            .unwrap_or(world.integration_parameters.prediction_distance());
        Self {
            link,
            support,
            foot,
            floor,
            foot_body,
            floor_body,
            previous_relative_pose: pad_pose.inv_mul(&sole_pose),
            previous_skin: skin,
            initial_excess_gap,
        }
    }

    fn loss(&mut self, world: &PhysicsWorld) -> Option<SupportLoss> {
        let event = |reason| {
            Some(SupportLoss {
                link: self.link,
                support: self.support,
                reason,
            })
        };
        let (Some(sole), Some(pad)) = (
            world.colliders.get(self.foot),
            world.colliders.get(self.floor),
        ) else {
            return event(SupportLossReason::Retired);
        };
        if sole.parent() != Some(self.foot_body) || pad.parent() != Some(self.floor_body) {
            return event(SupportLossReason::Retired);
        }
        let support = &world.bodies[self.floor_body];
        if !stationary_support(support) {
            return event(SupportLossReason::MotionStarted);
        }
        if !collision::can_exchange_impulses(sole, pad, support.body_type()) {
            return event(SupportLossReason::Separated);
        }
        let sole_pose = collider_pose(&world.bodies[self.foot_body], sole);
        let pad_pose = collider_pose(&world.bodies[self.floor_body], pad);
        let relative = pad_pose.inv_mul(&sole_pose);
        let skin = sole.contact_skin() + pad.contact_skin();
        if relative == self.previous_relative_pose && skin == self.previous_skin {
            return None;
        }
        self.previous_relative_pose = relative;
        self.previous_skin = skin;
        // A speculative contact can carry positive impulse at a small gap on
        // the demotion step. Preserve that observed clearance and both skins.
        let scale = 1.
            + sole_pose
                .translation
                .abs()
                .max(pad_pose.translation.abs())
                .max_element();
        let margin = self.initial_excess_gap + skin + 32. * f64::EPSILON * scale;
        let separated = match distance(&sole_pose, sole.shape(), &pad_pose, pad.shape()) {
            Ok(d) => d.distance > margin,
            Err(_) => {
                // Unsupported exact queries can still prove separation when
                // the bounding boxes have a gap wider than the contact envelope.
                let a = sole.shape().compute_aabb(&sole_pose);
                let b = pad.shape().compute_aabb(&pad_pose);
                (a.mins - b.maxs)
                    .max(b.mins - a.maxs)
                    .max(Vector::ZERO)
                    .length()
                    > margin
            }
        };
        separated.then_some(SupportLoss {
            link: self.link,
            support: self.support,
            reason: SupportLossReason::Separated,
        })
    }
}

#[derive(Default)]
pub(super) struct SupportWatch {
    /// A stopped or repositioned velocity boundary needs one new native
    /// reaction before its existing contact can justify another demotion.
    pub(super) awaiting_solve: std::collections::HashSet<RigidBodyHandle>,
    by_root: BTreeMap<BodyId, Vec<LoadedSupport>>,
    pending: BTreeMap<BodyId, SupportLoss>,
}

impl SupportWatch {
    #[cfg(feature = "landing-audit")]
    pub(super) fn is_inactive(&self) -> bool {
        self.by_root.is_empty() && self.pending.is_empty() && self.awaiting_solve.is_empty()
    }

    pub(super) fn install(&mut self, root: BodyId, mut supports: Vec<LoadedSupport>) {
        self.pending.remove(&root);
        if supports.is_empty() {
            self.by_root.remove(&root);
        } else {
            // Native graph iteration order can change after unrelated removals.
            supports.sort_by_key(|s| {
                (
                    s.link,
                    s.support,
                    s.foot.into_raw_parts(),
                    s.floor.into_raw_parts(),
                )
            });
            self.by_root.insert(root, supports);
        }
    }

    pub(super) fn update(&mut self, world: &PhysicsWorld, changed: Option<RigidBodyHandle>) {
        for (root, supports) in &mut self.by_root {
            if self.pending.contains_key(root) {
                continue;
            }
            if let Some(loss) = supports
                .iter_mut()
                .filter(|s| changed.is_none_or(|h| h == s.foot_body || h == s.floor_body))
                .find_map(|s| s.loss(world))
            {
                self.pending.insert(*root, loss);
            }
        }
    }

    pub(super) fn retire(&mut self, body: RigidBodyHandle) {
        self.awaiting_solve.remove(&body);
        self.latch_body(body, SupportLossReason::Retired);
    }

    fn latch_body(&mut self, body: RigidBodyHandle, reason: SupportLossReason) {
        for (&root, supports) in &self.by_root {
            if let Some(s) = supports.iter().find(|s| s.floor_body == body) {
                self.pending.entry(root).or_insert(SupportLoss {
                    link: s.link,
                    support: s.support,
                    reason,
                });
            }
        }
    }

    pub(super) fn replace_foot_collider(
        &mut self,
        root: BodyId,
        old: ColliderHandle,
        new: ColliderHandle,
    ) {
        if let Some(supports) = self.by_root.get_mut(&root) {
            for s in supports.iter_mut().filter(|s| s.foot == old) {
                s.foot = new;
            }
        }
    }
}

impl RapierBackend {
    /// Capture the loss of provenance before overwriting the native pose or
    /// velocity. Reissuing an identical stationary state keeps its reaction.
    pub(super) fn note_support_edit(
        &mut self,
        handle: RigidBodyHandle,
        pose: Option<Pose>,
        linear: [f64; 3],
        angular: [f64; 3],
    ) {
        let body = &self.world.bodies[handle];
        if (!self.parked_shared.is_empty()||!self.solved_shared_groups.is_empty()) && (pose.is_some_and(|p|p!=*body.position()) || linear!=body.linvel().to_array() || angular!=body.angvel().to_array()){
            self.parked_shared.retain(|v|!v.contains_body(handle));
            self.solved_shared_groups.clear();
        }
        if body.body_type() != RigidBodyType::KinematicVelocityBased {
            return;
        }
        let moved = pose.is_some_and(|p| p != *body.position());
        let starts_motion = stationary_support(body) && (linear != [0.; 3] || angular != [0.; 3]);
        if moved || starts_motion {
            // A later stop or return to the original pose cannot resurrect a
            // warm start from before this command.
            self.support_watch
                .latch_body(handle, SupportLossReason::MotionStarted);
            for hold in self.held_supports.values_mut().flatten() {
                hold.expire_moving_support(handle);
            }
        }
        if linear == [0.; 3]
            && angular == [0.; 3]
            && (moved || !stationary_support(body) || self.curves.drives(handle))
            && body.colliders().iter().any(|&c| {
                self.world
                    .narrow_phase
                    .contact_pairs_with(c)
                    .any(|p| p.has_any_active_contact())
            })
        {
            self.support_watch.awaiting_solve.insert(handle);
        }
    }

    /// Constant-time check for any prescribed assembly with a latched support loss.
    pub fn has_pending_support_loss(&self) -> bool {
        !self.support_watch.pending.is_empty()
    }
    /// A successful promotion, reset or new prescribed interval clears the event.
    pub fn pending_support_loss(&self, root: BodyId) -> Option<SupportLoss> {
        self.support_watch.pending.get(&root).copied()
    }
}
