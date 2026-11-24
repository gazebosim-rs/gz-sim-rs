// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use rapier3d_f64::parry::partitioning::{Bvh, BvhBuildStrategy};

#[derive(Default)]
pub(crate) struct Registry {
    pub bodies: BTreeMap<BodyId, (RigidBodyHandle, f64)>,
    pub floors: std::sync::OnceLock<Fixed>,
}
impl Registry {
    pub(super) fn from_backend(backend: &RapierBackend) -> Self {
        let mut registry = Self::default();
        for (id, e) in &backend.bodies {
            registry.insert(*id, e.0, &e.1.collider);
        }
        registry
    }
    pub(crate) fn insert(&mut self, id: BodyId, handle: RigidBodyHandle, c: &Collider) {
        if let Some((local, shape)) = primitive(c)
            && let Some(cap) = shape.as_cylinder()
        {
            let radius =
                (*c.position() * local).translation.length() + cap.radius.hypot(cap.half_height);
            self.bodies.insert(id, (handle, radius));
        }
    }
}
pub(crate) struct Fixed {
    pub(super) supports: Index,
    pub(super) obstacles: Index,
}
impl Fixed {
    pub(super) fn new(world: &PhysicsWorld) -> Self {
        let (mut supports, mut obstacles) = (Vec::new(), Vec::new());
        for (h, c) in world.colliders.iter() {
            let body = c.parent().map(|h| &world.bodies[h]);
            if body.is_some_and(|b| !b.is_fixed()) {
                continue;
            }
            let bound = c
                .shape()
                .compute_aabb(&collider_pose(c, body))
                .loosened(c.contact_skin());
            if primitive(c).is_some_and(|(_, g)| g.as_cuboid().is_some()) {
                supports.push((h, bound));
            } else {
                obstacles.push((h, bound));
            }
        }
        Self {
            supports: Index::build(supports.into_iter()),
            obstacles: Index::build(obstacles.into_iter()),
        }
    }
}
pub(crate) struct Index {
    tree: Bvh,
    total: Option<Aabb>,
    handles: Vec<ColliderHandle>,
    unbounded: Vec<ColliderHandle>,
}
impl Index {
    fn build(bounds: impl Iterator<Item = (ColliderHandle, Aabb)>) -> Self {
        let (mut boxes, mut handles, mut unbounded) = (Vec::new(), Vec::new(), Vec::new());
        for (h, b) in bounds {
            if b.mins.is_finite() && b.maxs.is_finite() && (b.maxs - b.mins).is_finite() {
                boxes.push(b);
                handles.push(h);
            } else {
                unbounded.push(h);
            }
        }
        Self {
            total: boxes.iter().copied().reduce(|a, b| a.merged(&b)),
            tree: Bvh::from_leaves(BvhBuildStrategy::Binned, &boxes),
            handles,
            unbounded,
        }
    }
    pub(super) fn could_reach(&self, origin: Vector, radius: f64) -> bool {
        !self.unbounded.is_empty()
            || self.total.is_some_and(|b| {
                origin.cmpge(b.mins - Vector::splat(radius)).all()
                    && origin.cmple(b.maxs + Vector::splat(radius)).all()
            })
    }
    pub(super) fn swept(world: &PhysicsWorld) -> Self {
        let p = &world.integration_parameters;
        let bounds: Vec<_> = world
            .colliders
            .iter()
            .filter_map(|(h, c)| {
                let h_body = c.parent()?;
                let body = &world.bodies[h_body];
                if body.is_fixed() {
                    return None;
                }
                // A prescribed trajectory or a joint impulse has no valid isolated-body
                // speed bound. Keep these peers in every query, then apply collision masks.
                let bounds = if body.is_kinematic()
                    || body.is_ccd_enabled()
                    || world.impulse_joints_with(h_body).next().is_some()
                    || world.multibody_joints_with(h_body).next().is_some()
                {
                    Aabb::from_half_extents(Vector::ZERO, Vector::splat(f64::INFINITY))
                } else {
                    // In a single collision-detection pass Rapier assembles every pair from
                    // initial geometry before any constraint can redirect a third body.
                    // Also exclude independently incoming peers outside that native margin.
                    let pose = collider_pose(c, Some(body));
                    let (v, margin) = free_sweep(
                        body,
                        c.shape(),
                        &pose,
                        world.gravity,
                        p.max_linear_velocity(),
                        p.dt,
                    );
                    swept_aabb(
                        c.shape(),
                        &pose,
                        v,
                        margin
                            + c.contact_skin()
                            + body.soft_ccd_prediction()
                            + p.prediction_distance() / 2.,
                        p.dt,
                    )
                };
                Some((h, bounds))
            })
            .collect();
        let free = Self::build(bounds.iter().copied());
        // A moving/moving impact can throw a peer out of its free sweep. If any
        // compatible pair can exchange momentum, bound every free peer by Rapier's
        // step speed cap in all directions, including multiple transfers.
        let active = world.narrow_phase.contact_pairs().any(|pair| {
            pair.has_any_active_contact()
                && [pair.collider1, pair.collider2].iter().all(|h| {
                    world.colliders[*h]
                        .parent()
                        .is_some_and(|b| !world.bodies[b].is_fixed())
                })
        });
        let deflection = active
            || bounds.iter().any(|(a, b)| {
                free.query(b).any(|h| {
                    h != *a
                        && crate::collision::can_exchange_impulses(
                            &world.colliders[*a],
                            &world.colliders[h],
                            world.bodies[world.colliders[h].parent().unwrap()].body_type(),
                        )
                })
            });
        if !deflection {
            return free;
        }
        Self::build(bounds.into_iter().map(|(h, b)| {
            let c = &world.colliders[h];
            let body = &world.bodies[c.parent().unwrap()];
            let travel = p.max_linear_velocity().max(body.linvel().length()) * p.dt
                + c.contact_skin()
                + body.soft_ccd_prediction();
            let envelope = crate::contact_prediction::dynamic::deflected_aabb(
                body,
                c.shape(),
                &collider_pose(c, Some(body)),
                travel,
            );
            (h, b.merged(&envelope))
        }))
    }
    pub(super) fn query<'a>(&'a self, b: &'a Aabb) -> impl Iterator<Item = ColliderHandle> + 'a {
        self.tree
            .intersect_aabb(b)
            .map(|i| self.handles[i as usize])
            .chain(self.unbounded.iter().copied())
    }
}
fn collider_pose(c: &Collider, body: Option<&RigidBody>) -> Pose {
    body.map_or(*c.position(), |b| {
        *b.position() * c.position_wrt_parent().unwrap()
    })
}
