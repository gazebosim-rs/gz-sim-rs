// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read-only casts against admitted collision shapes, without a query cache.
use super::*;
use rapier3d_f64::parry::{
    query::details::clip_aabb_line,
    shape::{Capsule, Cylinder, Shape as _},
};

fn cylinder_distance(
    cylinder: &Cylinder,
    pose: &Pose,
    ray: &Ray,
    max_distance: f64,
) -> Result<Option<f64>, PhysicsError> {
    let local = ray.inverse_transform_by(pose);
    if !local.origin.is_finite() || !local.dir.is_finite() {
        return Err(PhysicsError::InvalidValue);
    }
    let Some((near, far)) = clip_aabb_line(&cylinder.compute_local_aabb(), local.origin, local.dir)
    else {
        return Ok(None);
    };
    let first = near.0.max(0.0);
    let last = far.0;
    if first > last || first > max_distance {
        return Ok(None);
    }
    let origin = local.point_at(first);
    let span = last - first;
    if !origin.is_finite() || !span.is_finite() || span < 0.0 {
        return Err(PhysicsError::InvalidValue);
    }
    // Within the finite Y slab, this capsule is exactly the cylinder. Clip
    // before its analytic query so neither spherical end can supply a hit.
    // Rebase near the shape to avoid cancellation from a distant ray origin.
    let capsule = Capsule::new_y(cylinder.half_height, cylinder.radius);
    let axis = capsule.segment.b - capsule.segment.a;
    let offset = origin - capsule.segment.a;
    let radial_direction = local.dir.cross(axis);
    let radial_origin = offset.cross(axis);
    let axis_squared = axis.length_squared();
    let radius_squared = cylinder.radius * cylinder.radius;
    let radial_step = radial_direction.length_squared();
    let radial_offset = radial_origin.length_squared();
    let projected_radius_squared = radius_squared * axis_squared;
    // A finite admitted collider can be too thin for this query's projected
    // arithmetic. Refuse that query instead of treating underflow as tangency
    // or parallel motion; physical construction and stepping are unchanged.
    if ![axis_squared, radius_squared, projected_radius_squared]
        .into_iter()
        .all(|value| value.is_normal() && value > 0.0)
        || (radial_direction != Vector::ZERO && !radial_step.is_normal())
        || (radial_origin != Vector::ZERO && !radial_offset.is_normal())
    {
        return Err(PhysicsError::InvalidValue);
    }
    if ![
        axis_squared,
        radius_squared,
        offset.length_squared(),
        (origin - capsule.segment.b).length_squared(),
        local.dir.length_squared(),
        offset.dot(axis),
        axis.dot(local.dir),
        radial_step,
        radial_offset,
        radial_direction.dot(radial_origin),
        projected_radius_squared,
    ]
    .into_iter()
    .all(f64::is_finite)
    {
        return Err(PhysicsError::InvalidValue);
    }
    let clipped = Ray::new(origin, local.dir);
    let Some(hit) = capsule.cast_local_ray(&clipped, span, true) else {
        return Ok(None);
    };
    let distance = first + hit;
    if !hit.is_finite() || hit < 0.0 || hit > span || !distance.is_finite() {
        return Err(PhysicsError::InvalidValue);
    }
    // Apply the caller's bound in its original parameter space, avoiding a
    // second rounded subtraction of the inclusive maximum after rebasing.
    Ok((distance <= max_distance).then_some(distance))
}

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
        let distance = if let Some(cylinder) = collider.shape().as_cylinder() {
            cylinder_distance(cylinder, &pose, &ray, max_distance)?
        } else {
            collider.shape().cast_ray(&pose, &ray, max_distance, true)
        };
        if let Some(distance) = distance {
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

#[cfg(test)]
mod cylinder_numeric_tests {
    use super::*;

    #[test]
    fn cylinder_local_query_refuses_unusable_derived_arithmetic() {
        let ray = Ray::new(Vector::new(f64::MAX, 0.0, 0.0), -Vector::X);
        let pose = Pose::from_translation(Vector::new(-f64::MAX, 0.0, 0.0));
        assert_eq!(
            cylinder_distance(&Cylinder::new(1.5, 0.5), &pose, &ray, 20.0),
            Err(PhysicsError::InvalidValue)
        );
        // Finite supplied dimensions can still overflow the analytic kernel's
        // squared cross products. This internal query boundary must refuse them.
        let ray = Ray::new(Vector::new(3e100, 0.0, 0.0), -Vector::X);
        assert_eq!(
            cylinder_distance(&Cylinder::new(1e100, 1e100), &Pose::IDENTITY, &ray, 4e100),
            Err(PhysicsError::InvalidValue)
        );
        let ray = Ray::new(Vector::Y * 3.0, -Vector::Y * f64::MAX);
        assert_eq!(
            cylinder_distance(&Cylinder::new(1.5, 0.5), &Pose::IDENTITY, &ray, 20.0),
            Err(PhysicsError::InvalidValue)
        );
        for ray in [
            Ray::new(Vector::Y * 3.0, Vector::new(1e-200, -1.0, 0.0)),
            Ray::new(Vector::new(1e-200, 0.0, 0.0), Vector::Y),
        ] {
            assert_eq!(
                cylinder_distance(&Cylinder::new(1.5, 0.5), &Pose::IDENTITY, &ray, 20.0),
                Err(PhysicsError::InvalidValue)
            );
        }
    }
}
