// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use rapier3d_f64::parry::partitioning::{Bvh, BvhBuildStrategy};

#[derive(Default)]
pub(crate) struct Registry {
    pub ids: BTreeSet<BodyId>,
    pub floors: std::sync::OnceLock<Index>,
}
impl Registry {
    pub(super) fn from_backend(backend: &RapierBackend) -> Self {
        Self {
            ids: backend
                .bodies
                .iter()
                .filter_map(|(id, e)| primitive(&e.1.collider)?.1.as_cylinder().map(|_| *id))
                .collect(),
            ..Self::default()
        }
    }
}
pub(crate) struct Index {
    tree: Bvh,
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
            tree: Bvh::from_leaves(BvhBuildStrategy::Binned, &boxes),
            handles,
            unbounded,
        }
    }
    pub(super) fn floors(world: &PhysicsWorld) -> Self {
        Self::build(world.colliders.iter().filter_map(|(h, c)| {
            let body = c.parent().map(|h| &world.bodies[h]);
            if body.is_some_and(|b| !b.is_fixed()) {
                return None;
            }
            let (local, shape) = primitive(c)?;
            shape.as_cuboid()?;
            let pose = collider_pose(c, body) * local;
            Some((h, shape.compute_aabb(&pose)))
        }))
    }
    pub(super) fn swept(world: &PhysicsWorld) -> Self {
        let params = &world.integration_parameters;
        Self::build(world.colliders.iter().map(|(h, c)| {
            let body = c.parent().map(|h| &world.bodies[h]);
            let pose = collider_pose(c, body);
            let (v, margin) = body
                .filter(|b| !b.is_fixed())
                .map_or((Vector::ZERO, 0.), |b| {
                    free_sweep(
                        b,
                        c.shape(),
                        &pose,
                        world.gravity,
                        params.max_linear_velocity(),
                        params.dt,
                    )
                });
            (
                h,
                swept_aabb(c.shape(), &pose, v, margin + c.contact_skin(), params.dt),
            )
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
