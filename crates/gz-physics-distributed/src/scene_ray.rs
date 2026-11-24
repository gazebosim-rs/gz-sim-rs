// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Body, BodyId, PhysicsError, SceneBackend};
use gz_physics_multifidelity::SolidRay;

pub(super) fn cast(
    backend: &SceneBackend,
    origin: [f64; 3],
    direction: [f64; 3],
    limit: f64,
    exclude: Option<BodyId>,
) -> Result<Option<f64>, PhysicsError> {
    let mut ray = SolidRay::new(origin, direction, limit)?;
    if exclude.is_some_and(|id| backend.current(id).is_none() && !backend.is_shared_ground(id)) {
        return Err(PhysicsError::MissingBody);
    }
    if let Some(plane) = backend
        .ground
        .filter(|_| exclude != Some(super::ground::handle()))
    {
        ray.include_halfspace(plane.surface.point(), plane.surface.normal())?;
    }
    // All admitted primitives fit inside a world-aligned cube with half-size
    // max(size). For the capsule, the outer radius is half the axial length
    // plus half the diameter. No collider pose or cache is needed for a miss.
    let interval = std::array::from_fn::<_, 3, _>(|i| {
        let travel = direction[i] * limit;
        let end = origin[i] + travel;
        // Cover the rounding of the finite segment, including cancellation.
        let slack = 32. * f64::EPSILON * (1. + origin[i].abs() + travel.abs());
        (origin[i].min(end) - slack, origin[i].max(end) + slack)
    });
    let excluded = exclude.map(BodyId::raw);
    let mut include = |body: &Body| {
        let radius = body.size[0].max(body.size[1]).max(body.size[2]);
        let outside = (0..3).any(|i| {
            (body.position[i] - radius).next_down() > interval[i].1
                || (body.position[i] + radius).next_up() < interval[i].0
        });
        if Some(body.id) != excluded && !outside {
            match body.primitive {
                crate::physics::Primitive::Box => {
                    ray.include_box(body.size, body.position, body.orientation)?
                }
                crate::physics::Primitive::Cylinder => ray.include_cylinder(
                    0.5 * body.size[0],
                    0.5 * body.size[2],
                    body.position,
                    body.orientation,
                )?,
                crate::physics::Primitive::Capsule => ray.include_capsule(
                    0.5 * body.size[0],
                    0.5 * body.size[2],
                    body.position,
                    body.orientation,
                )?,
                crate::physics::Primitive::Sphere => {
                    ray.include_sphere(body.size[0] * 0.5, body.position, body.orientation)?
                }
            }
        }
        Ok::<_, PhysicsError>(())
    };
    // Every body occurs once in a complete coordinator frame, even immediately
    // after migration. There is no read RPC, halo geometry, or second ray cache.
    let workers = backend
        .frame()
        .map(|f| f.workers.as_slice())
        .or_else(|| backend.pool_frame().map(|f| f.workers.as_slice()));
    if let Some(workers) = workers {
        workers
            .iter()
            .flat_map(|w| &w.snapshot.bodies)
            .try_for_each(include)?;
    } else {
        backend.loading.values().try_for_each(&mut include)?;
    }
    Ok(ray.distance())
}
