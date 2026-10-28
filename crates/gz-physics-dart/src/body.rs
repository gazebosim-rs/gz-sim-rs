// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Validation and mass properties, computed before any native call.
//!
//! The packaged DART asserts on NaN and aborts, and ODE's hash space asserts on
//! coordinates beyond its integer grid. Admission therefore bounds every input
//! so that DART's products of up to five such factors (I·ω², m·c·ω², F/m,
//! τ/λ, m·g, a hinge's r×(m·r·ω²)) stay below 1e300; the boundary tests in
//! tests/bounds.rs establish these limits against the real library.

use crate::ffi::bridge::{BodyConfig, Twist};
use gz_physics::{BodySpec, Inertia, PhysicsError, RotationSpec, Shape};
use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};

/// Largest admitted magnitude of mass, inertia, length, velocity, force, torque
/// and gravity.
pub(crate) const MAGNITUDE_LIMIT: f64 = 1e60;

/// Smallest admitted mass, principal moment and shape dimension, so that their
/// reciprocals also stay within about MAGNITUDE_LIMIT.
pub(crate) const MAGNITUDE_FLOOR: f64 = 1e-60;

/// Largest admitted bound on a dynamic body's contact response, in 1/kg:
/// 1/m + sum_i (sum_{j!=i} e_j^2)/lambda_i over its principal axes, where e_j is
/// the shape's extent from the COM along axis j. Every diagonal entry of DART's
/// contact matrix at a point of the shape is below it. Also bounds m|c|^2/lambda_min
/// (c: COM offset), which DART's origin-based inertia needs. The packaged DART
/// aborts when contact entries reach a few 1e7 (`isSymmetric` with its 1e-6
/// absolute tolerance, or a zero LDLT pivot); tests/fuzz.rs found aborts at 1e7
/// and 1e8 and none at 1e5, at least 100x below the smallest abort (docs/DART.md).
pub(crate) const CONDITIONING_LIMIT: f64 = 1e5;

/// Largest admitted ratio of the largest to the smallest principal moment, from
/// the input inertia before any rotation. A symmetric eigen-solve is accurate to
/// about eps*lambda_max, so within this ratio lambda_min carries a relative error
/// below 1e8 * 2.2e-16 = 2e-8; beyond it the rotated tensor's lambda_min is noise
/// and a ratio near 1e60 passed as a well-conditioned body (docs/DART.md).
pub(crate) const ANISOTROPY_LIMIT: f64 = 1e8;

/// Largest summed inverse effective mass of the two bodies at any contact point,
/// in 1/kg. Two bodies within CONDITIONING_LIMIT reach at most 4e5 at points on
/// their own shapes; deeper penetration is refused by this bound instead.
pub(crate) const CONTACT_LIMIT: f64 = 4.0 * CONDITIONING_LIMIT;

/// Largest admitted world coordinate of any collision bound, in meters. ODE's
/// hash space grids coordinates by cells of at least 2^-3 m in 32-bit integers,
/// and aborts beyond 2^32 cells (measured), so 5e8 m < 2^29 m is safe.
pub(crate) const POSITION_LIMIT: f64 = 5e8;

fn finite(values: [f64; 3]) -> bool {
    values.into_iter().all(f64::is_finite)
}

fn positive(value: f64) -> bool {
    value.is_normal() && value > 0.0
}

/// A strictly positive magnitude within [MAGNITUDE_FLOOR, MAGNITUDE_LIMIT].
pub(crate) fn bounded(value: f64) -> bool {
    (MAGNITUDE_FLOOR..=MAGNITUDE_LIMIT).contains(&value)
}

/// A vector whose Euclidean length is at most MAGNITUDE_LIMIT; nonfinite fails.
pub(crate) fn small(values: [f64; 3]) -> bool {
    values.into_iter().map(|v| v * v).sum::<f64>().sqrt() <= MAGNITUDE_LIMIT
}

/// Every coordinate of a point lies within POSITION_LIMIT.
pub(crate) fn inside(point: [f64; 3]) -> bool {
    point.into_iter().all(|v| v.abs() <= POSITION_LIMIT)
}

/// A unit quaternion within the tolerance Rapier also accepts.
pub(crate) fn rotation(value: [f64; 4]) -> Result<UnitQuaternion<f64>, PhysicsError> {
    let [x, y, z, w] = value;
    let q = Quaternion::new(w, x, y, z);
    if !value.into_iter().all(f64::is_finite) || (q.norm_squared() - 1.0).abs() > 1e-12 {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(UnitQuaternion::new_unchecked(q))
}

/// Solid-body moments about the center of mass on body axes, Z along the shape axis.
fn shape_moment(shape: Shape, mass: f64) -> Matrix3<f64> {
    let diagonal = match shape {
        Shape::Sphere { radius } => [0.4 * mass * radius * radius; 3],
        Shape::Box {
            half_extents: [x, y, z],
        } => [
            mass * (y * y + z * z) / 3.0,
            mass * (x * x + z * z) / 3.0,
            mass * (x * x + y * y) / 3.0,
        ],
        Shape::Cylinder {
            radius,
            half_height,
        } => {
            let radial = mass * (3.0 * radius * radius + 4.0 * half_height * half_height) / 12.0;
            [radial, radial, 0.5 * mass * radius * radius]
        }
        Shape::Capsule {
            radius,
            half_length,
        } => {
            // Mass splits by volume between the cylinder and the two hemispheres.
            let length = 2.0 * half_length;
            let cylinder = mass * length / (length + 4.0 * radius / 3.0);
            let caps = mass - cylinder;
            let r2 = radius * radius;
            let radial = cylinder * (3.0 * r2 + length * length) / 12.0
                + caps * (0.4 * r2 + 3.0 * length * radius / 8.0 + half_length * half_length);
            [radial, radial, (0.5 * cylinder + 0.4 * caps) * r2]
        }
    };
    Matrix3::from_diagonal(&Vector3::from(diagonal))
}

/// Shape code and DART dimensions, plus the body-axis AABB half extents.
fn geometry(
    shape: Shape,
    rotation: &Matrix3<f64>,
) -> Result<(u8, [f64; 3], [f64; 3]), PhysicsError> {
    let (code, dimensions, values) = match shape {
        Shape::Sphere { radius } => (0, [radius, 0.0, 0.0], vec![radius]),
        Shape::Box {
            half_extents: [x, y, z],
        } => (1, [2.0 * x, 2.0 * y, 2.0 * z], vec![x, y, z]),
        Shape::Cylinder {
            radius,
            half_height,
        } => (
            2,
            [radius, 2.0 * half_height, 0.0],
            vec![radius, half_height],
        ),
        Shape::Capsule {
            radius,
            half_length,
        } => (
            3,
            [radius, 2.0 * half_length, 0.0],
            vec![radius, half_length],
        ),
    };
    if !values.iter().all(|&v| bounded(v)) {
        return Err(PhysicsError::InvalidValue);
    }
    let extent = |axis: usize| -> f64 {
        let row = rotation.row(axis);
        let along = row[2].abs();
        match shape {
            Shape::Sphere { radius } => radius,
            Shape::Box { half_extents } => (0..3).map(|j| row[j].abs() * half_extents[j]).sum(),
            Shape::Cylinder {
                radius,
                half_height,
            } => along * half_height + radius * (1.0 - along * along).max(0.0).sqrt(),
            Shape::Capsule {
                radius,
                half_length,
            } => along * half_length + radius,
        }
    };
    Ok((code, dimensions, [extent(0), extent(1), extent(2)]))
}

/// Distance from the body origin to the farthest point of a centered shape,
/// from DART dimensions (box full sizes; cylinder/capsule radius and length).
pub(crate) fn reach(shape: u8, dimensions: [f64; 3]) -> f64 {
    let [a, b, c] = dimensions;
    match shape {
        0 => a,
        1 => 0.5 * (a * a + b * b + c * c).sqrt(),
        2 => a.hypot(0.5 * b),
        _ => a + 0.5 * b,
    }
}

fn conditioned(value: f64) -> bool {
    value <= CONDITIONING_LIMIT
}

/// Rejects input principal moments that are out of range, too anisotropic to
/// rotate accurately, or that violate the triangle inequality.
fn check_principal(moment: &Matrix3<f64>) -> Result<(), PhysicsError> {
    if !moment.iter().all(|v| v.is_finite()) {
        return Err(PhysicsError::InvalidValue);
    }
    let principal = moment.symmetric_eigenvalues();
    let sum: f64 = principal.iter().sum();
    if !principal.iter().all(|&v| bounded(v))
        || principal.max() > ANISOTROPY_LIMIT * principal.min()
        || !sum.is_finite()
        || principal.iter().any(|&v| v > sum - v + sum * 1e-12)
    {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(())
}

/// Half-width of a centered Z-axis shape along the body-frame unit vector `u`.
fn support(shape: u8, dimensions: [f64; 3], u: &Vector3<f64>) -> f64 {
    let [a, b, c] = dimensions;
    let along = u.z.abs();
    let across = (1.0 - along * along).max(0.0).sqrt();
    match shape {
        0 => a,
        1 => 0.5 * (u.x.abs() * a + u.y.abs() * b + u.z.abs() * c),
        2 => along * 0.5 * b + a * across,
        _ => along * 0.5 * b + a,
    }
}

/// The body-frame principal moments (ascending) and axes (columns) of `moment`.
pub(crate) fn principal(moment: &Matrix3<f64>) -> (Vector3<f64>, Matrix3<f64>) {
    let eigen = moment.symmetric_eigen();
    (eigen.eigenvalues, eigen.eigenvectors)
}

/// The contact-response bound 1/m + sum_i (sum_{j!=i} e_j^2)/lambda_i, which is
/// at least every diagonal entry of DART's contact matrix at a shape point.
fn response(
    shape: u8,
    dimensions: [f64; 3],
    center: [f64; 3],
    mass: f64,
    moment: &Matrix3<f64>,
) -> f64 {
    let (moments, axes) = principal(moment);
    let extent = |j: usize| -> f64 {
        let u = axes.column(j).into_owned();
        Vector3::from(center).dot(&u).abs() + support(shape, dimensions, &u)
    };
    let e = [extent(0), extent(1), extent(2)].map(|v| v * v);
    let total: f64 = e.iter().sum();
    1.0 / mass + (0..3).map(|i| (total - e[i]) / moments[i]).sum::<f64>()
}

fn norm(values: [f64; 3]) -> f64 {
    Vector3::from(values).norm()
}

/// Whether a world revolute about the unit `axis` through `anchor` keeps the
/// body's contact response bounded: rho_a^2 / I_axis within the conditioning
/// limit, where rho_a bounds the distance from the anchor to any shape point and
/// I_axis is the body's moment about the hinge line.
pub(crate) fn hinge_conditioned(config: &BodyConfig, anchor: [f64; 3], axis: [f64; 3]) -> bool {
    let [x, y, z, w] = config.orientation;
    let rotation = UnitQuaternion::new_unchecked(Quaternion::new(w, x, y, z)).to_rotation_matrix();
    let [xx, yy, zz, xy, xz, yz] = config.moment;
    let moment = Matrix3::new(xx, xy, xz, xy, yy, yz, xz, yz, zz);
    let axis = Vector3::from(axis);
    let origin = Vector3::from(config.position);
    let arm = origin + rotation * Vector3::from(config.center) - Vector3::from(anchor);
    let perpendicular = arm - axis * arm.dot(&axis);
    let about_axis = axis.dot(&(rotation.matrix() * moment * rotation.matrix().transpose() * axis))
        + config.mass * perpendicular.norm_squared();
    let farthest = (origin - Vector3::from(anchor)).norm() + reach(config.shape, config.dimensions);
    // DART forms the hinge inertia from origin-based spatial inertia, so m times
    // the squared lever must not swamp I_axis either.
    let arm2 = farthest * farthest;
    conditioned(arm2 / about_axis) && conditioned(config.mass * arm2 / about_axis)
}

/// Validates one link completely and returns its native construction record.
pub(crate) fn config(
    spec: BodySpec,
    rotational: RotationSpec,
    inertia: Option<Inertia>,
) -> Result<BodyConfig, PhysicsError> {
    let fixed = spec.mass.is_none();
    if !finite(spec.position)
        || !small(spec.velocity)
        || !small(rotational.angular_velocity)
        || spec.mass.is_some_and(|m| !bounded(m))
        || (fixed && (spec.velocity != [0.0; 3] || rotational.angular_velocity != [0.0; 3]))
    {
        return Err(PhysicsError::InvalidValue);
    }
    let orientation = rotation(rotational.orientation)?.to_rotation_matrix();
    let (shape, dimensions, extents) = geometry(spec.shape, orientation.matrix())?;
    for (center, extent) in spec.position.into_iter().zip(extents) {
        let (low, high) = (center - extent, center + extent);
        if !inside([low, high, 0.0]) || !positive(high - low) {
            return Err(PhysicsError::InvalidValue);
        }
    }
    let mass = spec.mass.unwrap_or(1.0);
    let (center, moment) = match inertia {
        None => {
            let moment = shape_moment(spec.shape, mass);
            check_principal(&moment)?;
            ([0.0; 3], moment)
        }
        Some(value) => {
            if !small(value.center) || !value.tensor.iter().map(|v| v * v).sum::<f64>().is_finite()
            {
                return Err(PhysicsError::InvalidValue);
            }
            let [xx, yy, zz, xy, xz, yz] = value.tensor;
            let tensor = Matrix3::new(xx, xy, xz, xy, yy, yz, xz, yz, zz);
            // Before rotating: the anisotropy cap keeps the rotated moments exact
            // to about 2e-8 relative.
            check_principal(&tensor)?;
            let frame = rotation(value.orientation)?.to_rotation_matrix();
            (
                value.center,
                frame.matrix() * tensor * frame.matrix().transpose(),
            )
        }
    };
    let smallest = principal(&moment).0.min();
    // A dynamic body's contact response and its free-joint origin velocity stay
    // within the bounds DART can solve and the step guard accepts. DART stores
    // inertia about the body origin; recovering the COM moments from I + m|c|^2
    // terms loses them unless m|c|^2/lambda_min is bounded too.
    if !fixed
        && (smallest <= 0.0
            || !conditioned(response(shape, dimensions, center, mass, &moment))
            || !conditioned(mass * norm(center).powi(2) / smallest)
            || norm(spec.velocity) + norm(rotational.angular_velocity) * norm(center)
                > MAGNITUDE_LIMIT)
    {
        return Err(PhysicsError::InvalidValue);
    }
    let (moments, axes) = principal(&moment);
    Ok(BodyConfig {
        principal: moments.into(),
        axes: std::array::from_fn(|k| axes[k]),
        shape,
        dimensions,
        position: spec.position,
        orientation: rotational.orientation,
        mobile: !fixed,
        mass,
        center,
        moment: [
            moment[(0, 0)],
            moment[(1, 1)],
            moment[(2, 2)],
            moment[(0, 1)],
            moment[(0, 2)],
            moment[(1, 2)],
        ],
        velocity: Twist {
            linear: spec.velocity,
            angular: rotational.angular_velocity,
        },
    })
}

#[cfg(test)]
#[path = "../tests/private/body.rs"]
mod tests;
