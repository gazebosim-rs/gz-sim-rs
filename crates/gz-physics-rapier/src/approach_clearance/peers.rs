// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Conservatively refine an overlapping pair of whole-assembly envelopes.
use super::*;
use rapier3d_f64::parry::bounding_volume::Aabb;

pub(crate) struct PeerEnvelope {
    union: Aabb,
    shapes: Vec<ShapeEnvelope>,
}
struct ShapeEnvelope {
    world: Aabb,
    pose: Pose,
    half: Option<Vector>,
    reach: f64,
}
impl PeerEnvelope {
    pub(super) fn new(
        union: Aabb,
        sweeps: &[(&Collider, ApproachSweep)],
        bounds: Vec<Aabb>,
    ) -> Self {
        Self {
            union,
            shapes: sweeps
                .iter()
                .zip(bounds)
                .map(|((c, s), world)| ShapeEnvelope {
                    world,
                    pose: s.pose,
                    half: c.shape().as_cuboid().map(|v| v.half_extents),
                    reach: s.margin + c.contact_skin() + s.velocity.length() * s.dt.as_secs_f64(),
                })
                .collect(),
        }
    }
    pub(crate) fn separate(&self, other: &Self) -> bool {
        !self.union.intersects(&other.union)
            || self
                .shapes
                .iter()
                .all(|a| other.shapes.iter().all(|b| a.separate(b)))
    }
}
impl ShapeEnvelope {
    fn separate(&self, b: &Self) -> bool {
        if !self.world.intersects(&b.world) {
            return true;
        }
        let (Some(a_half), Some(b_half)) = (self.half, b.half) else {
            return false;
        };
        let axes = |p: Pose| [Vector::X, Vector::Y, Vector::Z].map(|n| p.rotation * n);
        let a_axes = axes(self.pose);
        let b_axes = axes(b.pose);
        let delta = b.pose.translation - self.pose.translation;
        let reach = self.reach + b.reach;
        // A fixed world axis must separate the inflated starting boxes. Every
        // point stays inside its shape's certified path-length envelope even
        // if it rotates or accelerates. Skipping a near-parallel cross axis
        // can only turn an uncertain result into an ordinary native handoff.
        let scale = 1.
            + self.pose.translation.abs().max_element()
            + b.pose.translation.abs().max_element()
            + a_half.element_sum()
            + b_half.element_sum()
            + reach;
        let tolerance = 256. * f64::EPSILON * scale;
        let separates = |axis: Vector| {
            let length = axis.length();
            if length <= 1e-10 {
                return false;
            }
            let n = axis / length;
            let radius = |basis: [Vector; 3], half: Vector| {
                basis
                    .into_iter()
                    .zip(half.to_array())
                    .map(|(e, h)| n.dot(e).abs() * h)
                    .sum::<f64>()
            };
            delta.dot(n).abs() > radius(a_axes, a_half) + radius(b_axes, b_half) + reach + tolerance
        };
        a_axes
            .into_iter()
            .chain(b_axes)
            .chain(a_axes.into_iter().flat_map(|a| b_axes.map(|b| a.cross(b))))
            .any(separates)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rapier3d_f64::parry::shape::Shape;

    fn slender(center: Vector, reach: f64, rotation: Rotation) -> ShapeEnvelope {
        let pose = Pose::from_parts(center, rotation);
        let half = Vector::new(0.75, 0.04, 0.03);
        let shape = rapier3d_f64::parry::shape::Cuboid::new(half);
        ShapeEnvelope {
            world: shape.compute_aabb(&pose).loosened(reach),
            pose,
            half: Some(half),
            reach,
        }
    }

    #[test]
    fn rotated_boxes_must_clear_both_trajectories_and_the_roundoff_guard() {
        let rot = Rotation::from_scaled_axis(Vector::Z * 0.73);
        let n = rot * Vector::Y;
        for center in [Vector::ZERO, Vector::new(1000., -500., 950.)] {
            let a = slender(center, 0.01, rot);
            let outside = slender(center + n * 0.120_001, 0.03, rot);
            let touch = slender(center + n * 0.12, 0.03, rot);
            let inside = slender(center + n * 0.118, 0.03, rot);
            assert!(a.world.intersects(&outside.world));
            assert!(a.separate(&outside) && outside.separate(&a));
            assert!(!a.separate(&touch) && !a.separate(&inside));
            let mut unknown = outside;
            unknown.half = None;
            assert!(!a.separate(&unknown));
        }
    }

    #[test]
    fn every_shape_must_clear_even_if_the_roots_are_apart() {
        let rot = Rotation::from_scaled_axis(Vector::Z * 0.81);
        let y = rot * Vector::Y;
        let make = |heights: [f64; 2]| {
            let shapes: Vec<_> = heights.map(|h| slender(y * h, 0.001, rot)).into();
            PeerEnvelope {
                union: shapes[0].world.merged(&shapes[1].world),
                shapes,
            }
        };
        let a = make([0., 0.25]);
        assert!(a.separate(&make([0.12, 0.41])));
        assert!(!a.separate(&make([0.12, 0.30])));
    }
}
