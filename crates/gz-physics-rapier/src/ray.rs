// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read-only casts against admitted collision shapes, without a query cache.
use super::*;

/// Closest solid hit on a finite world-space ray, using the solver's f64
/// Parry primitive geometry. Both endpoints are included.
pub struct SolidRay {
    ray: Ray,
    limit: f64,
    closest: Option<f64>,
}

impl SolidRay {
    pub fn new(origin: [f64; 3], direction: [f64; 3], limit: f64) -> Result<Self, PhysicsError> {
        let origin = Vector::from(origin);
        let direction = Vector::from(direction);
        if !origin.is_finite()
            || !direction.is_finite()
            || (direction.length_squared() - 1.0).abs() > 1e-12
            || !limit.is_finite()
            || limit <= 0.0
            || !(origin + direction * limit).is_finite()
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Self {
            ray: Ray::new(origin, direction),
            limit,
            closest: None,
        })
    }

    /// Accumulate a centered box from one committed pose and full side lengths.
    /// The geometry lives on the stack and needs no collider or solver insertion.
    pub fn include_box(
        &mut self,
        size: [f64; 3],
        position: [f64; 3],
        orientation: [f64; 4],
    ) -> Result<(), PhysicsError> {
        let half_extents = Vector::from(size) * 0.5;
        if !half_extents.is_finite() || half_extents.min_element() <= 0.0 {
            return Err(PhysicsError::InvalidValue);
        }
        let pose = Pose::from_parts(Vector::from(position), rotation(orientation)?);
        self.include(
            &pose,
            &rapier3d_f64::parry::shape::Cuboid::new(half_extents),
        )
    }

    /// Include a solid finite sphere at its current world center.
    pub fn include_sphere(
        &mut self,
        radius: f64,
        center: [f64; 3],
        orientation: [f64; 4],
    ) -> Result<(), PhysicsError> {
        if !radius.is_finite() || radius <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        self.include(
            &Pose::from_parts(Vector::from(center), rotation(orientation)?),
            &rapier3d_f64::parry::shape::Ball::new(radius),
        )
    }

    /// Include a finite Z-axis cylinder with planar end caps.
    pub fn include_cylinder(
        &mut self,
        radius: f64,
        half_height: f64,
        center: [f64; 3],
        orientation: [f64; 4],
    ) -> Result<(), PhysicsError> {
        if !radius.is_finite() || radius <= 0. || !half_height.is_finite() || half_height <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let pose = Pose::from_parts(Vector::from(center), rotation(orientation)?)
            * super::cylinder_frame();
        self.include(
            &pose,
            &rapier3d_f64::parry::shape::Cylinder::new(half_height, radius),
        )
    }

    /// Include a solid capsule whose centerline is aligned with its local Z axis.
    pub fn include_capsule(
        &mut self,
        radius: f64,
        half_length: f64,
        center: [f64; 3],
        orientation: [f64; 4],
    ) -> Result<(), PhysicsError> {
        if !radius.is_finite() || radius <= 0. || !half_length.is_finite() || half_length <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        self.include(
            &Pose::from_parts(Vector::from(center), rotation(orientation)?),
            &rapier3d_f64::parry::shape::Capsule::new_z(half_length, radius),
        )
    }

    /// Accumulate a solid horizontal half-space below its world-space elevation.
    pub fn include_horizontal_ground(&mut self, height: f64) -> Result<(), PhysicsError> {
        self.include(
            &Pose::translation(0., 0., height),
            &rapier3d_f64::parry::shape::HalfSpace::new(Vector::Z),
        )
    }

    /// Accumulate an immutable halfspace at a world-space point with a unit outward normal.
    pub fn include_halfspace(
        &mut self,
        point: [f64; 3],
        normal: [f64; 3],
    ) -> Result<(), PhysicsError> {
        let n = Vector::from(normal);
        if !n.is_finite() || (n.length_squared() - 1.).abs() > 1e-12 {
            return Err(PhysicsError::InvalidValue);
        }
        self.include(
            &Pose::from_translation(Vector::from(point)),
            &rapier3d_f64::parry::shape::HalfSpace::new(n),
        )
    }

    pub fn distance(&self) -> Option<f64> {
        self.closest
    }

    fn include(
        &mut self,
        pose: &Pose,
        shape: &dyn rapier3d_f64::parry::shape::Shape,
    ) -> Result<(), PhysicsError> {
        if !pose.translation.is_finite() || !pose.rotation.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        let distance = if let Some(cylinder) = shape.as_cylinder() {
            finite_cylinder_hit(
                &self.ray.inverse_transform_by(pose),
                cylinder.radius,
                cylinder.half_height,
                self.limit,
            )?
        } else {
            shape.cast_ray(pose, &self.ray, self.limit, true)
        };
        if let Some(distance) = distance {
            if !distance.is_finite() || !(0.0..=self.limit).contains(&distance) {
                return Err(PhysicsError::InvalidValue);
            }
            self.closest = Some(
                self.closest
                    .map_or(distance, |previous| previous.min(distance)),
            );
        }
        Ok(())
    }
}

// Intersect the axial slab and the radial disk in the native Y-axis frame.
// Expressing the chord in terms of perpendicular distance avoids subtracting
// two almost equal squared distances for long or nearly tangent sensor rays.
fn finite_cylinder_hit(
    ray: &Ray,
    radius: f64,
    half_height: f64,
    limit: f64,
) -> Result<Option<f64>, PhysicsError> {
    let (o, d) = (ray.origin, ray.dir);
    if !o.is_finite() || !d.is_finite() {
        return Err(PhysicsError::InvalidValue);
    }
    let (mut enter, mut leave) = (0_f64, limit);
    if d.y == 0. {
        if o.y.abs() > half_height {
            return Ok(None);
        }
    } else {
        let low = (-half_height - o.y) / d.y;
        let high = (half_height - o.y) / d.y;
        enter = enter.max(low.min(high));
        leave = leave.min(low.max(high));
    }
    let speed = d.x.hypot(d.z);
    if speed == 0. {
        if o.x.hypot(o.z) > radius {
            return Ok(None);
        }
    } else {
        let (ux, uz) = (d.x / speed, d.z / speed);
        let foot = -o.x.mul_add(ux, o.z * uz);
        let perpendicular = o.x.mul_add(uz, -o.z * ux).abs();
        if perpendicular > radius {
            return Ok(None);
        }
        let ratio = perpendicular / radius;
        let half_chord = radius * ((1. - ratio) * (1. + ratio)).sqrt();
        enter = enter.max((foot - half_chord) / speed);
        leave = leave.min((foot + half_chord) / speed);
    }
    Ok((enter <= leave).then_some(enter))
}

pub(super) fn cast(
    backend: &RapierBackend,
    origin: [f64; 3],
    direction: [f64; 3],
    max_distance: f64,
    exclude: Option<BodyId>,
) -> Result<Option<f64>, PhysicsError> {
    let mut ray = SolidRay::new(origin, direction, max_distance)?;
    let excluded = exclude
        .map(|id| {
            backend
                .bodies
                .get(&id)
                .map(|entry| entry.0)
                .ok_or(PhysicsError::MissingBody)
        })
        .transpose()?;
    for (_, collider) in backend.world.colliders.iter() {
        if excluded.is_some_and(|body| collider.parent() == Some(body)) {
            continue;
        }
        // Collider world poses can lag a body edit until the next pipeline run.
        // Compose current body/local frames rather than mutating contact or BVH
        // state merely to answer a sensor query, including immediately after reset.
        let pose = if let Some(parent) = collider.parent() {
            let body = &backend.world.bodies[parent];
            Pose::from_parts(backend.current_translation(parent, body), *body.rotation())
                * collider.position_wrt_parent().expect("parented collider")
        } else {
            *collider.position()
        };
        ray.include(&pose, collider.shape())?;
    }
    Ok(ray.distance())
}

#[cfg(test)]
#[path = "../tests/private/ray.rs"]
mod tests;
