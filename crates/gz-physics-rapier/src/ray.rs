// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read-only casts against admitted collision shapes, without a query cache.
use super::*;

pub(super) fn cast(
    backend: &RapierBackend,
    origin: [f64; 3],
    direction: [f64; 3],
    max_distance: f64,
    exclude: Option<BodyId>,
) -> Result<Option<f64>, PhysicsError> {
    let origin = Vector::from(origin);
    let direction = Vector::from(direction);
    if !origin.is_finite()
        || !direction.is_finite()
        || (direction.length_squared() - 1.0).abs() > 1e-12
        || !max_distance.is_finite()
        || max_distance <= 0.0
        || !(origin + direction * max_distance).is_finite()
    {
        return Err(PhysicsError::InvalidValue);
    }
    let excluded = exclude
        .map(|id| {
            backend
                .bodies
                .get(&id)
                .map(|entry| entry.0)
                .ok_or(PhysicsError::MissingBody)
        })
        .transpose()?;
    let ray = Ray::new(origin, direction);
    let mut closest: Option<f64> = None;
    for (_, collider) in backend.world.colliders.iter() {
        if excluded.is_some_and(|body| collider.parent() == Some(body)) {
            continue;
        }
        // Collider world poses can lag a body edit until the next pipeline run.
        // Compose current body/local frames rather than mutating contact or BVH
        // state merely to answer a sensor query, including immediately after reset.
        let pose = if let Some(parent) = collider.parent() {
            backend.world.bodies[parent].position()
                * collider.position_wrt_parent().expect("parented collider")
        } else {
            *collider.position()
        };
        if !pose.translation.is_finite() || !pose.rotation.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        if let Some(distance) = collider.shape().cast_ray(&pose, &ray, max_distance, true) {
            if !distance.is_finite() || !(0.0..=max_distance).contains(&distance) {
                return Err(PhysicsError::InvalidValue);
            }
            closest = Some(closest.map_or(distance, |previous| previous.min(distance)));
        }
    }
    Ok(closest)
}

#[cfg(test)]
#[path = "../tests/private/ray.rs"]
mod tests;
