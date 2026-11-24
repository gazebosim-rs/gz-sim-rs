// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{may_clamp, outer_radius};
use crate::{
    coordinator::Error,
    physics::{Body, ForceMotion, GroundPlane, Primitive, Tier},
};
use gz_physics_multifidelity::{partition_linear_speed_limit, partition_prediction_distance};

/// Bound the minimum clearance of one dynamic body from the shared halfspace.
/// The caller supplies the velocity after all ordered center-of-mass impulses.
pub(super) fn needs_refinement(
    body: &Body,
    force: Option<ForceMotion>,
    plane: GroundPlane,
    dt: f64,
) -> Result<bool, Error> {
    if body.tier != Tier::Dynamic {
        return Ok(false);
    }
    let point = plane.point();
    let normal = plane.normal();
    let project = |v: [f64; 3]| v[0] * normal[0] + v[1] * normal[1] + v[2] * normal[2];
    let change = force.map_or([0.; 3], |f| f.delta_velocity);
    let inward = if may_clamp(body, change) {
        partition_linear_speed_limit() * dt
    } else {
        // Bound the two signed contributions independently. A held acceleration
        // can reverse the normal velocity before the end of the tick.
        -dt * (project(body.linear).min(0.) + project(change).min(0.))
    };
    let radius = outer_radius(body);
    let distance = project(std::array::from_fn(|i| body.position[i] - point[i]));
    let scale = (0..3).fold(1. + radius, |s, i| {
        s + normal[i].abs()
            * (body.position[i].abs()
                + point[i].abs()
                + dt * (body.linear[i].abs() + change[i].abs()))
    });
    // Portable orientations may differ from a unit quaternion by up to 1e-12
    // in squared norm. Allow the native pose to renormalize at a barrier.
    let padding =
        partition_prediction_distance() + inward + 8e-12 * radius + 128. * f64::EPSILON * scale;
    if !distance.is_finite() || !padding.is_finite() {
        return Ok(true);
    }
    if distance - radius > padding {
        return Ok(false);
    }
    // A sphere has the same support in every orientation. For spinning bodies
    // and applied torques the circumsphere also bounds the gyroscopic response.
    if body.primitive == Primitive::Sphere
        || body.angular != [0.; 3]
        || force.is_some_and(|f| f.has_torque)
    {
        return Ok(true);
    }
    // At constant attitude use the actual primitive's support against the plane
    // shifted outward by the prediction distance and maximum inward travel.
    let shifted = std::array::from_fn(|i| point[i] + normal[i] * padding);
    let bounds = body
        .bounds()
        .map_err(|_| Error::Invalid("invalid adaptive ground geometry"))?;
    Ok(!bounds.above_plane(shifted, normal))
}
