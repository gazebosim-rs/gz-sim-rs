// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent separating-axis test and fresh Parry queries for the box-train controls.
use gz_physics_distributed::physics::{Body, Primitive};
use rapier3d_f64::{
    parry::{
        query::{self, ContactManifold, NonlinearRigidMotion, ShapeCastStatus, Unsupported},
        shape::Cuboid,
    },
    prelude::{Pose, Rotation, Vector},
};
use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct PairGeometry {
    /// Largest signed interval gap over the fifteen unit separating axes. Positive
    /// values certify separation but can underestimate the Euclidean distance.
    pub sat_gap_m: f64,
    /// Number of clipped manifold points in a fresh zero-prediction geometry
    /// query. Rapier can filter these further before solving the velocity constraints.
    pub manifold_points_at_zero_prediction: usize,
    /// Relative normal speed at Parry's single-contact witness. Positive is opening.
    pub witness_normal_speed_m_s: Option<f64>,
    /// Converged first hit from separated shapes, extrapolating the two initial
    /// twists. Excludes forces, solver impulses and any changes to either twist.
    pub constant_twist_toi_s: Option<f64>,
    /// Parry's termination status when the cast returned a candidate.
    pub constant_twist_cast_status: Option<&'static str>,
}

fn axis_gap(poses: &[Pose; 2], boxes: &[Cuboid; 2]) -> f64 {
    let [axes_a, axes_b] = poses.map(|p| [Vector::X, Vector::Y, Vector::Z].map(|a| p.rotation * a));
    let edge_normals = axes_a.into_iter().flat_map(|a| axes_b.map(|b| a.cross(b)));
    let offset = poses[1].translation - poses[0].translation;
    let radius = |axes: [Vector; 3], half_extents: Vector, n: Vector| {
        axes.into_iter()
            .zip(half_extents.to_array())
            .map(|(axis, half)| axis.dot(n).abs() * half)
            .sum::<f64>()
    };
    axes_a
        .into_iter()
        .chain(axes_b)
        .chain(edge_normals)
        .filter_map(Vector::try_normalize)
        .map(|n| {
            offset.dot(n).abs()
                - radius(axes_a, boxes[0].half_extents, n)
                - radius(axes_b, boxes[1].half_extents, n)
        })
        .fold(f64::NEG_INFINITY, f64::max)
}

/// Inspect two committed train boxes and their constant-twist motions over a
/// finite, nonnegative horizon. A contact witness can exist at a positive gap.
pub fn inspect_pair(a: &Body, b: &Body, horizon_s: f64) -> Result<PairGeometry, Unsupported> {
    if a.primitive != Primitive::Box
        || b.primitive != Primitive::Box
        || !horizon_s.is_finite()
        || horizon_s < 0.
    {
        return Err(Unsupported);
    }
    let bodies = [a, b];
    let poses = bodies.map(|b| {
        Pose::from_parts(
            Vector::from(b.position),
            Rotation::from_array(b.orientation),
        )
    });
    let boxes = bodies.map(|b| Cuboid::new(Vector::from(b.size) * 0.5));
    let sat_gap_m = axis_gap(&poses, &boxes);
    let mut manifold = ContactManifold::<(), ()>::default();
    query::details::contact_manifold_cuboid_cuboid(
        &poses[0].inv_mul(&poses[1]),
        &boxes[0],
        &boxes[1],
        0.,
        &mut manifold,
    );

    let velocity_at = |body: &Body, point: Vector| {
        Vector::from(body.linear)
            + Vector::from(body.angular).cross(point - Vector::from(body.position))
    };
    let witness_normal_speed_m_s =
        query::contact(&poses[0], &boxes[0], &poses[1], &boxes[1], f64::MAX)?
            .map(|p| (velocity_at(b, p.point2) - velocity_at(a, p.point1)).dot(p.normal1));
    let cast = if sat_gap_m <= 0. || horizon_s == 0. {
        None
    } else if bodies.iter().all(|b| b.angular == [0.; 3]) {
        query::cast_shapes(
            &poses[0],
            Vector::from(a.linear),
            &boxes[0],
            &poses[1],
            Vector::from(b.linear),
            &boxes[1],
            query::ShapeCastOptions {
                max_time_of_impact: horizon_s,
                ..Default::default()
            },
        )?
    } else {
        let [ma, mb] = std::array::from_fn(|i| {
            NonlinearRigidMotion::new(
                poses[i],
                Vector::ZERO,
                Vector::from(bodies[i].linear),
                Vector::from(bodies[i].angular),
            )
        });
        query::cast_shapes_nonlinear(&ma, &boxes[0], &mb, &boxes[1], 0., horizon_s, true)?
    };
    let constant_twist_cast_status = cast.as_ref().map(|hit| match hit.status {
        ShapeCastStatus::Converged => "converged",
        ShapeCastStatus::OutOfIterations => "out_of_iterations",
        ShapeCastStatus::Failed => "failed",
        ShapeCastStatus::PenetratingOrWithinTargetDist => "penetrating_or_within_target_distance",
    });
    let constant_twist_toi_s = cast
        .filter(|hit| hit.status == ShapeCastStatus::Converged)
        .map(|hit| hit.time_of_impact);
    Ok(PairGeometry {
        sat_gap_m,
        manifold_points_at_zero_prediction: manifold.points.len(),
        witness_normal_speed_m_s,
        constant_twist_toi_s,
        constant_twist_cast_status,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use gz_physics_distributed::physics::fixture;

    #[test]
    fn straight_approach_and_existing_overlap_have_distinct_observations() {
        let mut a = fixture()[0].promoted();
        a.position = [0.; 3];
        a.size = [1.; 3];
        a.linear = [0.; 3];
        let mut b = a.clone();
        b.position = [1.25, 0., 0.];
        b.linear = [-4., 0., 0.];

        for (first, second) in [(&a, &b), (&b, &a)] {
            let q = inspect_pair(first, second, 0.1).unwrap();
            assert_eq!(q.sat_gap_m, 0.25);
            assert_eq!(q.manifold_points_at_zero_prediction, 0);
            assert_eq!(q.witness_normal_speed_m_s, Some(-4.));
            assert!((q.constant_twist_toi_s.unwrap() - 0.0625).abs() < 1e-6);
            assert!(
                inspect_pair(first, second, 0.05)
                    .unwrap()
                    .constant_twist_toi_s
                    .is_none()
            );
        }
        b.position[0] = 0.875;
        b.linear[0] = 2.;
        let q = inspect_pair(&a, &b, 0.1).unwrap();
        assert_eq!(q.sat_gap_m, -0.125);
        assert!(q.manifold_points_at_zero_prediction >= 1);
        assert_eq!(q.witness_normal_speed_m_s, Some(2.));
        assert_eq!(q.constant_twist_toi_s, None);
    }

    #[test]
    fn spinning_bar_reaches_the_stationary_box_by_angular_motion_alone() {
        let mut a = fixture()[0].promoted();
        a.position = [0.; 3];
        a.linear = [0.; 3];
        a.size = [2., 0.1, 0.1];
        a.angular = [0., 0., 1.];
        let mut b = a.clone();
        b.size = [0.1; 3];
        b.position = [0.55, 0.13, 0.];
        let tilt = 0.05_f64;
        b.orientation = Rotation::from_rotation_x(tilt).to_array();
        b.angular = [0.; 3];

        // The leading corner reaches local y = 0.05 in the rotating bar.
        let min_y = 0.13 - 0.05 * (tilt.cos() + tilt.sin());
        let max_x = 0.6;
        let expected = min_y.atan2(max_x) - (0.05 / max_x.hypot(min_y)).asin();
        let q = inspect_pair(&a, &b, 0.5).unwrap();
        assert_eq!(q.constant_twist_cast_status, Some("converged"));
        assert!((q.constant_twist_toi_s.unwrap() - expected).abs() < 1e-6);
        assert!((-0.6..=-0.5).contains(&q.witness_normal_speed_m_s.unwrap()));
        a.angular = [0.; 3];
        let still = inspect_pair(&a, &b, 0.5).unwrap();
        assert_eq!(still.constant_twist_toi_s, None);
        assert_eq!(still.witness_normal_speed_m_s, Some(0.));
        assert!(inspect_pair(&a, &b, -0.3).is_err());
        b.primitive = Primitive::Sphere;
        assert!(inspect_pair(&a, &b, 0.3).is_err());
    }

    #[test]
    fn edge_axes_detect_separation_that_all_six_face_axes_miss() {
        let mut a = fixture()[0].promoted();
        a.position = [0.; 3];
        a.size = [2., 0.04, 0.04];
        let mut b = a.clone();
        let r = Rotation::from_rotation_z(0.7)
            * Rotation::from_rotation_y(0.6)
            * Rotation::from_rotation_x(0.35);
        b.orientation = r.to_array();
        let n = Vector::X.cross(r * Vector::X).normalize();
        b.position = (n * 0.2).to_array();
        for face in [
            Vector::X,
            Vector::Y,
            Vector::Z,
            r * Vector::X,
            r * Vector::Y,
            r * Vector::Z,
        ] {
            let half = Vector::from(a.size) * 0.5;
            assert!(
                Vector::from(b.position).dot(face).abs()
                    < half.dot(face.abs()) + half.dot((r.inverse() * face).abs())
            );
        }
        let q = inspect_pair(&a, &b, 0.01).unwrap();
        let half = Vector::from(a.size) * 0.5;
        let expected = 0.2 - half.dot(n.abs()) - half.dot((r.inverse() * n).abs());
        assert!((q.sat_gap_m - expected).abs() < 1e-12);
        assert!(q.sat_gap_m > 0.);
        assert_eq!(q.manifold_points_at_zero_prediction, 0);
        assert_eq!(q.constant_twist_toi_s, None);
    }
}
