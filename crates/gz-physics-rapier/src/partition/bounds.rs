// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Conservative box insertion before either domain can own a shared contact.
use super::*;
use rapier3d_f64::parry::{
    query,
    shape::{Ball, Capsule as NativeCapsule, Cuboid, Cylinder, Shape as ParryShape},
};

/// Current world bounds, box axes and half-extents used by native and portable admission.
pub struct PartitionBoxBounds {
    aabb: Aabb,
    center: Vector,
    rotation: Rotation,
    axes: [Vector; 3],
    half: Vector,
}

impl PartitionBoxBounds {
    pub fn new(
        size: [f64; 3],
        position: [f64; 3],
        orientation: [f64; 4],
    ) -> Result<Self, PhysicsError> {
        let half = size.map(|side| side * 0.5);
        if !half.into_iter().all(|v| v > 0. && v.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        let cube = rapier3d_f64::parry::shape::Cuboid::new(Vector::from(half));
        let pose = Pose::from_parts(Vector::from(position), rotation(orientation)?);
        Self::from_cube(&cube, &pose)
    }

    pub(crate) fn from_collider(shape: &Collider, pose: &Pose) -> Result<Self, PhysicsError> {
        let cube = shape
            .shape()
            .as_cuboid()
            .ok_or(PhysicsError::InvalidValue)?;
        Self::from_cube(cube, pose)
    }

    fn from_cube(
        cube: &rapier3d_f64::parry::shape::Cuboid,
        pose: &Pose,
    ) -> Result<Self, PhysicsError> {
        let aabb = cube.aabb(pose);
        if !pose.translation.is_finite() || !aabb.mins.is_finite() || !aabb.maxs.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Self {
            aabb,
            center: pose.translation,
            rotation: pose.rotation,
            axes: [Vector::X, Vector::Y, Vector::Z].map(|a| pose.rotation * a),
            half: cube.half_extents,
        })
    }

    pub fn minimum_z(&self) -> f64 {
        self.aabb.mins.z
    }

    fn projected_radius(&self, axis: Vector) -> f64 {
        self.half.x * self.axes[0].dot(axis).abs()
            + self.half.y * self.axes[1].dot(axis).abs()
            + self.half.z * self.axes[2].dot(axis).abs()
    }

    /// Only a verified separating projection may refine overlapping world AABBs.
    /// Face normals and edge cross-products cover the separating axes of two boxes.
    /// Comparisons near roundoff retain the more conservative broad-phase answer.
    pub fn predicts_overlap(&self, other: &Self) -> bool {
        let padding = partition_prediction_distance();
        if !self.aabb.loosened(padding).intersects(&other.aabb) {
            return false;
        }
        let delta = other.center - self.center;
        let scale = 1.
            + self.center.abs().max_element()
            + other.center.abs().max_element()
            + self.half.max_element()
            + other.half.max_element();
        let threshold = padding + 128. * f64::EPSILON * scale;
        let separates = |axis: Vector| {
            let gap =
                delta.dot(axis).abs() - self.projected_radius(axis) - other.projected_radius(axis);
            let length = axis.length();
            length >= 1e-6 && gap.is_finite() && gap > length * threshold
        };
        !self.axes.into_iter().chain(other.axes).any(separates)
            && !self
                .axes
                .iter()
                .any(|&a| other.axes.iter().any(|&b| separates(a.cross(b))))
    }
}

/// Contact prediction bounds of a centered distributed primitive.
pub struct PartitionBounds(PrimitiveBounds);
enum PrimitiveBounds {
    Box(PartitionBoxBounds),
    Sphere {
        center: Vector,
        radius: f64,
        aabb: Aabb,
    },
    Axial(AxialBounds),
}

enum AxialShape {
    Capsule(NativeCapsule),
    Cylinder(Cylinder),
}
struct AxialBounds {
    pose: Pose,
    shape: AxialShape,
    axis: Vector,
    half_length: f64,
    radius: f64,
    aabb: Aabb,
}
impl AxialShape {
    fn native(&self) -> &dyn ParryShape {
        match self {
            Self::Capsule(c) => c,
            Self::Cylinder(c) => c,
        }
    }
}
impl AxialBounds {
    // pose is the collider world pose. The two native shapes have different local axes.
    fn new(pose: Pose, shape: AxialShape) -> Result<Self, PhysicsError> {
        let (radius, half_length, local_axis) = match &shape {
            AxialShape::Capsule(c) => (c.radius, c.segment.b.z, Vector::Z),
            AxialShape::Cylinder(c) => (c.radius, c.half_height, Vector::Y),
        };
        if !radius.is_finite() || radius <= 0. || !half_length.is_finite() || half_length <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let aabb = shape.native().compute_aabb(&pose);
        if !aabb.mins.is_finite() || !aabb.maxs.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Self {
            pose,
            shape,
            axis: pose.rotation * local_axis,
            half_length,
            radius,
            aabb,
        })
    }
    fn support_radius(&self, axis: Vector) -> f64 {
        let radial = match self.shape {
            AxialShape::Capsule(_) => self.radius,
            AxialShape::Cylinder(_) => self.axis.cross(axis).length() * self.radius,
        };
        radial + self.axis.dot(axis).abs() * self.half_length
    }

    // A native closest-point query supplies a candidate axis. Only the actual
    // support intervals can certify separation, including at degenerate witnesses.
    fn overlaps(&self, other: &PrimitiveBounds) -> bool {
        let prediction = partition_prediction_distance();
        let test = |pose: &Pose, shape: &dyn ParryShape| {
            let Ok(query::ClosestPoints::WithinMargin(a, b)) =
                query::closest_points(&self.pose, self.shape.native(), pose, shape, f64::MAX)
            else {
                return true;
            };
            let Some(n) = (a - b).try_normalize() else {
                return true;
            };
            let support = match other {
                PrimitiveBounds::Box(b) => b.projected_radius(n),
                PrimitiveBounds::Sphere { radius, .. } => *radius,
                PrimitiveBounds::Axial(c) => c.support_radius(n),
            };
            let (x, y) = (self.pose.translation, pose.translation);
            let margin = 128.
                * f64::EPSILON
                * (1.
                    + x.abs().max_element()
                    + y.abs().max_element()
                    + support
                    + self.half_length
                    + self.radius);
            let gap = (x - y).dot(n).abs() - self.support_radius(n) - support;
            !gap.is_finite() || gap <= prediction + margin
        };
        match other {
            PrimitiveBounds::Box(b) => test(
                &Pose::from_parts(b.center, b.rotation),
                &Cuboid::new(b.half),
            ),
            PrimitiveBounds::Sphere { center, radius, .. } => {
                test(&Pose::from_translation(*center), &Ball::new(*radius))
            }
            PrimitiveBounds::Axial(c) => test(&c.pose, c.shape.native()),
        }
    }
}

impl PartitionBounds {
    pub fn new(
        shape: Shape,
        position: [f64; 3],
        orientation: [f64; 4],
    ) -> Result<Self, PhysicsError> {
        match shape {
            Shape::Box { half_extents } => Ok(Self(PrimitiveBounds::Box(PartitionBoxBounds::new(
                half_extents.map(|h| 2. * h),
                position,
                orientation,
            )?))),
            Shape::Sphere { radius } => {
                rotation(orientation)?;
                Self::sphere(Vector::from(position), radius)
            }
            Shape::Capsule {
                radius,
                half_length,
            } => Ok(Self(PrimitiveBounds::Axial(AxialBounds::new(
                Pose::from_parts(Vector::from(position), rotation(orientation)?),
                AxialShape::Capsule(NativeCapsule::new_z(half_length, radius)),
            )?))),
            Shape::Cylinder {
                radius,
                half_height,
            } if radius.is_finite()
                && radius > 0.
                && half_height.is_finite()
                && half_height > 0. =>
            {
                Ok(Self(PrimitiveBounds::Axial(AxialBounds::new(
                    Pose::from_parts(Vector::from(position), rotation(orientation)?)
                        * cylinder_frame(),
                    AxialShape::Cylinder(Cylinder::new(half_height, radius)),
                )?)))
            }
            Shape::Cylinder { .. } => Err(PhysicsError::InvalidValue),
        }
    }
    fn sphere(center: Vector, radius: f64) -> Result<Self, PhysicsError> {
        if !center.is_finite() || !radius.is_finite() || radius <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let aabb = Aabb::from_half_extents(center, Vector::splat(radius));
        if !aabb.mins.is_finite() || !aabb.maxs.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Self(PrimitiveBounds::Sphere {
            center,
            radius,
            aabb,
        }))
    }
    pub(crate) fn from_collider(c: &Collider, pose: &Pose) -> Result<Self, PhysicsError> {
        if let Some(ball) = c.shape().as_ball() {
            Self::sphere(pose.translation, ball.radius)
        } else if let Some(capsule) = c.shape().as_capsule() {
            let h = capsule.segment.b.z;
            if capsule.segment.b != Vector::Z * h || capsule.segment.a != Vector::NEG_Z * h {
                return Err(PhysicsError::InvalidValue);
            }
            Ok(Self(PrimitiveBounds::Axial(AxialBounds::new(
                *pose,
                AxialShape::Capsule(*capsule),
            )?)))
        } else if let Some(cylinder) = c.shape().as_cylinder() {
            Ok(Self(PrimitiveBounds::Axial(AxialBounds::new(
                *pose,
                AxialShape::Cylinder(*cylinder),
            )?)))
        } else {
            Ok(Self(PrimitiveBounds::Box(
                PartitionBoxBounds::from_collider(c, pose)?,
            )))
        }
    }
    fn aabb(&self) -> Aabb {
        match &self.0 {
            PrimitiveBounds::Box(b) => b.aabb,
            PrimitiveBounds::Sphere { aabb, .. } => *aabb,
            PrimitiveBounds::Axial(c) => c.aabb,
        }
    }
    pub fn minimum_z(&self) -> f64 {
        self.aabb().mins.z
    }
    /// Conservative separating support projection against a shared halfspace.
    pub fn above_plane(&self, point: [f64; 3], normal: [f64; 3]) -> bool {
        let p = Vector::from(point);
        let n = Vector::from(normal);
        if !p.is_finite() || !n.is_finite() || (n.length_squared() - 1.).abs() > 8. * f64::EPSILON {
            return false;
        }
        if normal == [0., 0., 1.] {
            return self.minimum_z() >= point[2];
        }
        let (center, radius) = match &self.0 {
            PrimitiveBounds::Box(b) => (b.center, b.projected_radius(n)),
            PrimitiveBounds::Sphere { center, radius, .. } => (*center, *radius),
            PrimitiveBounds::Axial(c) => (c.pose.translation, c.support_radius(n)),
        };
        let gap = (center - p).dot(n) - radius;
        let error = 32. * f64::EPSILON * (1. + (center.abs() + p.abs()).dot(n.abs()) + radius);
        gap.is_finite() && gap >= error
    }
    pub(super) fn padded_aabb(&self, padding: f64) -> Aabb {
        self.aabb().loosened(padding)
    }
    pub fn predicts_overlap(&self, other: &Self) -> bool {
        use PrimitiveBounds::*;
        match (&self.0, &other.0) {
            (Box(a), Box(b)) => a.predicts_overlap(b),
            (Axial(c), other_shape) | (other_shape, Axial(c)) => {
                self.aabb()
                    .loosened(partition_prediction_distance())
                    .intersects(&other.aabb())
                    && c.overlaps(other_shape)
            }
            (
                Sphere {
                    center: a,
                    radius: r,
                    ..
                },
                Sphere {
                    center: b,
                    radius: s,
                    ..
                },
            ) => {
                let margin = partition_prediction_distance()
                    + 128.
                        * f64::EPSILON
                        * (1. + a.abs().max_element() + b.abs().max_element() + r + s);
                let delta = *a - *b;
                delta.x.hypot(delta.y).hypot(delta.z) <= r + s + margin
            }
            (Sphere { center, radius, .. }, Box(b)) | (Box(b), Sphere { center, radius, .. }) => {
                let delta = *center - b.center;
                let local = Vector::from(b.axes.map(|axis| delta.dot(axis)));
                let outside = (local.abs() - b.half).max(Vector::ZERO);
                let margin = partition_prediction_distance()
                    + 128.
                        * f64::EPSILON
                        * (1.
                            + center.abs().max_element()
                            + b.center.abs().max_element()
                            + radius
                            + b.half.max_element());
                outside.x.hypot(outside.y).hypot(outside.z) <= radius + margin
            }
        }
    }
}
