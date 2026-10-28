// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Construction checks and the lowering of shared body values to MuJoCo inputs.
//!
//! The checks match the Rapier adapter's admission rules so both engines reject the
//! same values with [`PhysicsError::InvalidValue`] before any native call.

use crate::native::{BodyJoint, BodyModel, Geometry, Inertial};
use gz_physics::{BodySpec, Inertia, PhysicsError, RotationSpec, Shape};

/// MuJoCo's compiler `mjEPS` (src/user/user_util.h).
const MUJOCO_EPS: f64 = 1e-14;
/// MuJoCo's `mjMINVAL`: the least mass and principal inertia of a moving body.
const MUJOCO_MINVAL: f64 = 1e-15;

pub(crate) fn finite(values: [f64; 3]) -> bool {
    values.into_iter().all(f64::is_finite)
}

fn positive(value: f64) -> bool {
    value.is_normal() && value > 0.0
}

/// Unit vector, rejecting lengths whose reciprocal is not finite and positive.
pub(crate) fn normalized(v: [f64; 3]) -> Option<[f64; 3]> {
    let reciprocal = 1.0 / v.into_iter().map(|c| c * c).sum::<f64>().sqrt();
    (reciprocal.is_finite() && reciprocal > 0.0).then(|| v.map(|c| c * reciprocal))
}

pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Rotates `v` by the unit quaternion [x,y,z,w].
pub(crate) fn quaternion_rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let axis = [q[0], q[1], q[2]];
    let t = cross(axis, v).map(|c| 2.0 * c);
    let u = cross(axis, t);
    [0, 1, 2].map(|i| v[i] + q[3] * t[i] + u[i])
}

pub(crate) fn quaternion_rotate_inverse(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    quaternion_rotate([-q[0], -q[1], -q[2], q[3]], v)
}

fn rotation(q: [f64; 4]) -> Result<[f64; 4], PhysicsError> {
    let norm2: f64 = q.into_iter().map(|c| c * c).sum();
    if !q.into_iter().all(f64::is_finite) || (norm2 - 1.0).abs() > 1e-12 {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(q)
}

/// Body-axis tensor xx,yy,zz,xy,xz,yz after rotating the inertial frame into the body.
fn rotated_tensor(inertia: Inertia) -> [f64; 6] {
    let [xx, yy, zz, xy, xz, yz] = inertia.tensor;
    let tensor = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]];
    let columns = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .map(|e| quaternion_rotate(inertia.orientation, e));
    // R T R^T with R's columns being the rotated basis vectors.
    let entry = |a: usize, b: usize| -> f64 {
        (0..3)
            .flat_map(|i| (0..3).map(move |j| (i, j)))
            .map(|(i, j)| columns[i][a] * tensor[i][j] * columns[j][b])
            .sum()
    };
    [
        entry(0, 0),
        entry(1, 1),
        entry(2, 2),
        entry(0, 1),
        entry(0, 2),
        entry(1, 2),
    ]
}

/// Principal moments of a symmetric tensor xx,yy,zz,xy,xz,yz and the rotation
/// whose columns are the principal axes, as a MuJoCo [w,x,y,z] quaternion. Cyclic
/// Jacobi rotation stays accurate for repeated moments; diagonal input is exact.
pub(crate) fn principal_axes(t: [f64; 6]) -> ([f64; 3], [f64; 4]) {
    let [xx, yy, zz, xy, xz, yz] = t;
    let mut a = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]];
    let mut axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..32 {
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let tangent = theta.signum() / (theta.abs() + theta.hypot(1.0));
            let cosine = 1.0 / tangent.hypot(1.0);
            let sine = tangent * cosine;
            // a <- J^T a J and axes <- axes J for the plane rotation J.
            for row in a.iter_mut().chain(axes.iter_mut()) {
                let (rp, rq) = (row[p], row[q]);
                row[p] = cosine * rp - sine * rq;
                row[q] = sine * rp + cosine * rq;
            }
            let (row_p, row_q) = (a[p], a[q]);
            a[p] = [0, 1, 2].map(|c| cosine * row_p[c] - sine * row_q[c]);
            a[q] = [0, 1, 2].map(|c| sine * row_p[c] + cosine * row_q[c]);
        }
    }
    ([a[0][0], a[1][1], a[2][2]], matrix_quaternion(axes))
}

/// [w,x,y,z] quaternion of a proper rotation matrix `m[row][column]`.
pub(crate) fn matrix_quaternion(m: [[f64; 3]; 3]) -> [f64; 4] {
    let trace = m[0][0] + m[1][1] + m[2][2];
    if trace > 0.0 {
        let s = 2.0 * (trace + 1.0).sqrt();
        [
            0.25 * s,
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
        ]
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = 2.0 * (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt();
        [
            (m[2][1] - m[1][2]) / s,
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
        ]
    } else if m[1][1] > m[2][2] {
        let s = 2.0 * (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt();
        [
            (m[0][2] - m[2][0]) / s,
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
        ]
    } else {
        let s = 2.0 * (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt();
        [
            (m[1][0] - m[0][1]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
        ]
    }
}

/// MuJoCo's compile-time checks on a moving body's diagonal inertial, evaluated
/// exactly as user_objects.cc (strict A + B < C) and CheckBodyMassInertia
/// (mjMINVAL) do, on the same doubles this crate passes to the compiler.
pub(crate) fn mujoco_accepts(mass: f64, inertia: [f64; 3]) -> bool {
    let [a, b, c] = inertia;
    let triangle_violated = a + b < c || a + c < b || b + c < a;
    !triangle_violated && mass >= MUJOCO_MINVAL && inertia.into_iter().all(|v| v >= MUJOCO_MINVAL)
}

/// Raises a moment that exceeds the sum of the other two to that sum. Callers have
/// already limited such excess to rounding (the shared 1e-12 relative slack, or a
/// solid shape's exact moments), and MuJoCo's strict check then passes exactly
/// regardless of how rotation or diagonalization rounded.
pub(crate) fn balance(mut moments: [f64; 3]) -> [f64; 3] {
    for (i, j, k) in [(0, 1, 2), (1, 0, 2), (2, 0, 1)] {
        if moments[j] + moments[k] < moments[i] {
            moments[i] = moments[j] + moments[k];
        }
    }
    moments
}

/// The inertial lowered for a moving body: explicit inertia rotated into the
/// body frame and diagonalized, or the solid shape's moments about its center.
fn inertial(spec: BodySpec, inertia: Option<Inertia>, mass: f64) -> Inertial {
    match inertia {
        Some(inertia) => {
            let (principal, quaternion) = principal_axes(rotated_tensor(inertia));
            Inertial {
                mass,
                center: inertia.center,
                principal: balance(principal),
                quaternion,
            }
        }
        None => Inertial {
            mass,
            center: [0.0; 3],
            principal: balance(shape_moments(spec.shape, mass)),
            quaternion: [1.0, 0.0, 0.0, 0.0],
        },
    }
}

/// Fixed bodies carry this inertial so that MuJoCo never derives one from geoms;
/// it has no dynamic effect and passes every compiler check.
const FIXED_INERTIAL: Inertial = Inertial {
    mass: 1.0,
    center: [0.0; 3],
    principal: [1.0; 3],
    quaternion: [1.0, 0.0, 0.0, 0.0],
};

/// Bound on an inertial center's body-frame coordinates, in meters. With MuJoCo's
/// velocity limit it keeps every free-joint conversion finite.
pub(crate) const MAX_CENTER: f64 = 1e100;

fn check_moments(moments: [f64; 3]) -> Result<(), PhysicsError> {
    if !moments.into_iter().all(|v| positive(1.0 / v)) {
        return Err(PhysicsError::InvalidValue);
    }
    let sum: f64 = moments.into_iter().sum();
    if !sum.is_finite() || moments.into_iter().any(|v| v > sum - v + sum * 1e-12) {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(())
}

/// Solid-body principal moments for `mass`, as MuJoCo computes geom inertia.
fn shape_moments(shape: Shape, mass: f64) -> [f64; 3] {
    let cylinder = |radius: f64, half: f64| {
        let side = mass * (3.0 * radius * radius + 4.0 * half * half) / 12.0;
        [side, side, mass * radius * radius / 2.0]
    };
    match shape {
        Shape::Sphere { radius } => [0.4 * mass * radius * radius; 3],
        Shape::Box {
            half_extents: [x, y, z],
        } => [y * y + z * z, x * x + z * z, x * x + y * y].map(|v| mass * v / 3.0),
        Shape::Cylinder {
            radius,
            half_height,
        } => cylinder(radius, half_height),
        Shape::Capsule {
            radius,
            half_length,
        } => {
            // A cylinder plus two hemispheres shifted by half the cylinder length.
            let length = 2.0 * half_length;
            let cylinder_mass = mass * length / (length + 4.0 * radius / 3.0);
            let cap_mass = mass - cylinder_mass;
            let radial = cylinder_mass * (3.0 * radius * radius + length * length) / 12.0
                + cap_mass
                    * (0.4 * radius * radius
                        + 3.0 * length * radius / 8.0
                        + half_length * half_length);
            [
                radial,
                radial,
                (0.5 * cylinder_mass + 0.4 * cap_mass) * radius * radius,
            ]
        }
    }
}

fn dimensions_valid(shape: Shape) -> bool {
    let dimensions: &[f64] = match &shape {
        Shape::Sphere { radius } => &[*radius],
        Shape::Box { half_extents } => half_extents,
        Shape::Cylinder {
            radius,
            half_height,
        } => &[*radius, *half_height],
        Shape::Capsule {
            radius,
            half_length,
        } => &[*radius, *half_length],
    };
    dimensions.iter().all(|&v| v.is_finite() && v > 0.0)
}

/// Solid volume, from which MuJoCo derives geom density for a requested mass.
fn volume(shape: Shape) -> f64 {
    use std::f64::consts::PI;
    match shape {
        Shape::Sphere { radius } => 4.0 / 3.0 * PI * radius.powi(3),
        Shape::Box {
            half_extents: [x, y, z],
        } => 8.0 * x * y * z,
        Shape::Cylinder {
            radius,
            half_height,
        } => 2.0 * PI * radius * radius * half_height,
        Shape::Capsule {
            radius,
            half_length,
        } => PI * radius * radius * (2.0 * half_length + 4.0 / 3.0 * radius),
    }
}

/// MuJoCo geometry and a bounding radius about the body origin.
fn geometry(shape: Shape) -> (Geometry, f64) {
    match shape {
        Shape::Sphere { radius } => (Geometry::Sphere(radius), radius),
        Shape::Box { half_extents } => (
            Geometry::Box(half_extents),
            half_extents.into_iter().map(|v| v * v).sum::<f64>().sqrt(),
        ),
        Shape::Cylinder {
            radius,
            half_height,
        } => (
            Geometry::Cylinder(radius, half_height),
            radius.hypot(half_height),
        ),
        Shape::Capsule {
            radius,
            half_length,
        } => (Geometry::Capsule(radius, half_length), radius + half_length),
    }
}

/// Admission checks for one body, in the Rapier adapter's order, then MuJoCo's
/// limits. Returns the inertial passed to MuJoCo.
pub(crate) fn spawn(
    spec: BodySpec,
    rotational: RotationSpec,
    inertia: Option<Inertia>,
) -> Result<Inertial, PhysicsError> {
    if !finite(spec.position)
        || !finite(spec.velocity)
        || spec.mass.is_some_and(|v| !v.is_finite() || v <= 0.0)
        || (spec.mass.is_none() && spec.velocity != [0.0; 3])
        || !finite(rotational.angular_velocity)
        || (spec.mass.is_none() && rotational.angular_velocity != [0.0; 3])
    {
        return Err(PhysicsError::InvalidValue);
    }
    rotation(rotational.orientation)?;
    let mass = spec.mass.unwrap_or(1.0);
    if !dimensions_valid(spec.shape) {
        return Err(PhysicsError::InvalidValue);
    }
    let (_, bound) = geometry(spec.shape);
    let extents = spec.position.map(|p| (p + bound) - (p - bound));
    if !positive(1.0 / mass)
        || !positive(mass / volume(spec.shape))
        // Shape inertia stands in for MuJoCo's geom inertia, which MuJoCo does not
        // derive for volumes at most its mjEPS.
        || (spec.mass.is_some() && inertia.is_none() && volume(spec.shape) <= MUJOCO_EPS)
        || !shape_moments(spec.shape, mass)
            .into_iter()
            .all(|v| positive(1.0 / v))
        || !extents.into_iter().all(positive)
    {
        return Err(PhysicsError::InvalidValue);
    }
    if let Some(inertia) = inertia {
        if !finite(inertia.center)
            || !inertia
                .tensor
                .into_iter()
                .map(|v| v * v)
                .sum::<f64>()
                .is_finite()
        {
            return Err(PhysicsError::InvalidValue);
        }
        rotation(inertia.orientation)?;
        check_moments(principal_axes(rotated_tensor(inertia)).0)?;
    }
    let Some(mass) = spec.mass else {
        return Ok(FIXED_INERTIAL);
    };
    // Moving bodies are lowered with exactly these values, so MuJoCo's compiler
    // accepts the body if and only if this does. A free body's position is its
    // qpos, which MuJoCo resets beyond mjMAXVAL.
    let lowered = inertial(spec, inertia, mass);
    if !mujoco_accepts(lowered.mass, lowered.principal)
        || !crate::motion::representable(&spec.position)
        || lowered.center.into_iter().any(|c| c.abs() > MAX_CENTER)
    {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(lowered)
}

/// Lowers an admitted body with the inertial `spawn` returned for it.
pub(crate) fn body_model(
    spec: BodySpec,
    rotational: RotationSpec,
    inertial: Inertial,
    joint: BodyJoint,
) -> BodyModel {
    let [x, y, z, w] = rotational.orientation;
    let (geometry, _) = geometry(spec.shape);
    BodyModel {
        position: spec.position,
        quaternion: [w, x, y, z],
        joint,
        geometry,
        inertial,
    }
}

#[cfg(test)]
#[path = "../tests/private/validate.rs"]
mod tests;
