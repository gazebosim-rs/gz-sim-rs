// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Conservative free-flight contact forecast for pairs with a curved member.
use super::{Error, Primitive, SweptBody, Tier, length, outer_radius};
use gz_physics::Shape;
use gz_physics_multifidelity::{PartitionBounds, partition_prediction_distance};

/// A shape enclosure at the initial attitude, valid throughout the public tick.
struct Hull {
    shape: Shape,
    orientation: [f64; 4],
}

impl Hull {
    fn new(sweep: &SweptBody, dt: f64) -> Result<Self, Error> {
        let body = &sweep.body;
        let shape = body
            .shape()
            .map_err(|_| Error::Invalid("invalid curved primitive"))?;
        let radius = outer_radius(body);
        let shape = if body.primitive == Primitive::Sphere {
            shape
        } else if body.tier == Tier::Dynamic
            && (body.angular != [0.; 3] || sweep.force.is_some_and(|f| f.has_torque))
        {
            // Gyroscopic forces and torques can change a dynamic body's rotation axis.
            Shape::Sphere { radius }
        } else {
            // A prescribed twist moves every surface point by at most r * angle.
            inflate(shape, radius * (length(body.angular) * dt).min(2.))
        };
        Ok(Self {
            shape,
            orientation: body.orientation,
        })
    }

    fn bounds(&self, at: [f64; 3], padding: f64) -> Result<PartitionBounds, Error> {
        PartitionBounds::new(inflate(self.shape, padding), at, self.orientation)
            .map_err(|_| Error::Invalid("invalid curved forecast enclosure"))
    }
}

/// Enclose the Minkowski sum of a centered primitive and a ball of radius p.
fn inflate(shape: Shape, p: f64) -> Shape {
    match shape {
        Shape::Box { half_extents } => Shape::Box {
            half_extents: half_extents.map(|h| h + p),
        },
        Shape::Sphere { radius } => Shape::Sphere { radius: radius + p },
        Shape::Cylinder {
            radius,
            half_height,
        } => Shape::Cylinder {
            radius: radius + p,
            half_height: half_height + p,
        },
        Shape::Capsule {
            radius,
            half_length,
        } => Shape::Capsule {
            radius: radius + p,
            half_length,
        },
    }
}

struct Sweep {
    origin: [f64; 3],
    travel: [f64; 3],
    distance: f64,
    padding: f64,
    first: PartitionBounds,
    second: Hull,
}

impl Sweep {
    fn overlaps(&self, time: f64, half_interval: f64) -> Result<bool, Error> {
        let midpoint = std::array::from_fn(|i| self.travel[i].mul_add(time, self.origin[i]));
        let bounds = self
            .second
            .bounds(midpoint, self.padding + self.distance * half_interval)?;
        Ok(self.first.predicts_overlap(&bounds))
    }

    /// At the midpoint, inflate the moving hull by its maximum displacement to
    /// either endpoint. A separating support interval excludes the entire interval.
    /// Subdivide ambiguous intervals; an unresolved leaf always selects refinement.
    fn candidate(&self, mid: f64, half: f64, depth: u8) -> Result<bool, Error> {
        if !self.overlaps(mid, half)? {
            return Ok(false);
        }
        if self.distance == 0. || depth == 0 || self.overlaps(mid, 0.)? {
            return Ok(true);
        }
        let half = half * 0.5;
        Ok(self.candidate(mid - half, half, depth - 1)?
            || self.candidate(mid + half, half, depth - 1)?)
    }
}

pub(super) fn curved_candidate(a: &SweptBody, b: &SweptBody, dt: f64) -> Result<bool, Error> {
    let relative = |a: [f64; 3], b: [f64; 3]| std::array::from_fn(|i| b[i] - a[i]);
    let origin = relative(a.body.position, b.body.position);
    let travel = relative(a.body.linear, b.body.linear).map(|v| v * dt);
    let distance = length(travel);
    let dv = |s: &SweptBody| s.force.map_or([0.; 3], |f| f.delta_velocity);
    let radii = outer_radius(&a.body) + outer_radius(&b.body);
    let scale = a
        .body
        .position
        .into_iter()
        .chain(b.body.position)
        .map(f64::abs)
        .fold(1.0_f64, f64::max)
        .max(distance)
        .max(radii);
    // The displacement from differential kicks is bounded by |delta v| * dt.
    // A common acceleration cancels before computing the norm.
    // Native portable rotations admit a 1e-12 squared-norm tolerance.
    let field_travel = length(relative(dv(a), dv(b))) * dt;
    let padding =
        field_travel + 256. * f64::EPSILON * scale + 8e-12 * (radii + distance + field_travel);
    if !origin.into_iter().all(f64::is_finite) || !distance.is_finite() || !padding.is_finite() {
        return Err(Error::Invalid("invalid relative curved motion"));
    }
    let tangent = if distance == 0. {
        [0.; 3]
    } else {
        travel.map(|x| x / distance)
    };
    let along = -origin.iter().zip(tangent).map(|(&x, t)| x * t).sum::<f64>();
    let closest = along.clamp(0., distance);
    let gap = length(std::array::from_fn(|i| {
        tangent[i].mul_add(closest, origin[i])
    }));
    if gap > radii + padding + partition_prediction_distance() {
        return Ok(false);
    }
    if a.body.primitive == Primitive::Sphere && b.body.primitive == Primitive::Sphere {
        return Ok(true);
    }
    let sweep = Sweep {
        origin,
        travel,
        distance,
        padding,
        first: Hull::new(a, dt)?.bounds([0.; 3], 0.)?,
        second: Hull::new(b, dt)?,
    };
    sweep.candidate(0.5, 0.5, 6)
}
