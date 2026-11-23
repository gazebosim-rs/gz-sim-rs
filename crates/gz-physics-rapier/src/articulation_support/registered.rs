// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measured stationary support when two registered faces own the native solve.
use super::*;
struct Site {
    surface: Surface,
    id: BodyId,
    force: f64,
    point: Option<Vector>,
}
pub(crate) struct ParkedSharedSupport {
    root: BodyId,
    link: BodyId,
    foot: Surface,
    sites: [Site; 2],
    gravity: Vector,
    normal: Vector,
}
#[derive(Clone, Copy)]
pub(crate) struct ParkedReaction {
    pub link: BodyId,
    pub support: BodyId,
    pub foot: ColliderHandle,
    pub floor: ColliderHandle,
    pub force: f64,
    pub point: Option<Vector>,
}
impl ParkedSharedSupport {
    pub(crate) fn owns_root(&self, id: BodyId) -> bool {
        self.root == id
    }
    pub(crate) fn owns_foot(&self, p: RigidBodyHandle) -> bool {
        self.foot.parent == p
    }
    pub(crate) fn contains_body(&self, p: RigidBodyHandle) -> bool {
        self.owns_foot(p) || self.sites.iter().any(|s| s.surface.parent == p)
    }
    pub(crate) fn capture(
        b: &RapierBackend,
        root: RigidBodyHandle,
        foot: ColliderHandle,
        pads: [ColliderHandle; 2],
        load: &SharedFaceReceipt,
    ) -> Option<Self> {
        if !load.comoving_rest
            || load.physical_duration_seconds <= 0.
            || load.slip_start_end != Some([[0.; 2]; 2])
        {
            return None;
        }
        let w = &b.world;
        let f = Surface::capture(w, foot)?;
        let body = w.bodies.get(f.parent)?;
        if body.linvel() != Vector::ZERO || body.angvel() != Vector::ZERO {
            return None;
        }
        let site = |i: usize| {
            let s = load.loads[i];
            let surface = Surface::capture(w, pads[i])?;
            let rb = w.bodies.get(surface.parent)?;
            if b.bodies
                .get(&s.support)
                .is_none_or(|entry| entry.0 != surface.parent)
                || !support_loss::stationary_support(rb)
                || b.curves.drives(surface.parent)
                || s.tangent_impulse_on_foot != [0.; 3]
                || s.selected_normal_impulse < 0.
                || !s.selected_normal_impulse.is_finite()
            {
                return None;
            }
            let point = s.end_pressure_world.map(Vector::from);
            if s.selected_normal_impulse > 0.
                && (!s.contact_at_end || point.is_none_or(|p| !p.is_finite()))
            {
                return None;
            }
            Some(Site {
                surface,
                id: s.support,
                force: s.selected_normal_impulse / load.physical_duration_seconds,
                point,
            })
        };
        Some(Self {
            root: BodyId::from_raw(w.bodies[root].user_data as u64),
            link: load.foot,
            foot: f,
            sites: [site(0)?, site(1)?],
            gravity: w.gravity,
            normal: Vector::from(load.normal),
        })
    }
    pub(crate) fn reactions(
        &self,
        b: &RapierBackend,
        index: MultibodyIndex,
        root: BodyId,
        dt: Duration,
        checked_neighbors: bool,
    ) -> Result<Option<[ParkedReaction; 2]>, PhysicsError> {
        if self.root != root {
            return Ok(None);
        }
        let w = &b.world;
        let valid = |surface: &Surface| surface.current_pose(w) == Some(surface.pose);
        if w.gravity != self.gravity
            || w.gravity == Vector::ZERO
            || !valid(&self.foot)
            || w.bodies
                .get(self.foot.parent)
                .is_none_or(|rb| rb.linvel() != Vector::ZERO || rb.angvel() != Vector::ZERO)
            || self.normal.dot(-w.gravity.normalize()) < 1. - 1024. * f64::EPSILON
            || self.sites.iter().any(|s| {
                !valid(&s.surface)
                    || w.bodies.get(s.surface.parent).is_none_or(|rb| {
                        !support_loss::stationary_support(rb)
                            || b.curves.drives(s.surface.parent)
                            || b.support_watch.awaiting_solve.contains(&s.surface.parent)
                            || !collision::can_exchange_impulses(
                                &w.colliders[self.foot.handle],
                                &w.colliders[s.surface.handle],
                                rb.body_type(),
                            )
                    })
            })
        {
            return Err(PhysicsError::TransitionDeferred {
                reason: "registered parked support changed since the last measured reaction",
            });
        }
        let ids = self.sites.each_ref().map(|s| s.id);
        if !checked_neighbors && !b.parked_supports_still_clear(index, self.link, ids, dt)? {
            return Err(PhysicsError::TransitionDeferred {
                reason: "another contact or changed tree invalidates the registered parked support",
            });
        }
        Ok(Some(self.sites.each_ref().map(|s| ParkedReaction {
            link: self.link,
            support: s.id,
            foot: self.foot.handle,
            floor: s.surface.handle,
            force: s.force,
            point: s.point,
        })))
    }
}
