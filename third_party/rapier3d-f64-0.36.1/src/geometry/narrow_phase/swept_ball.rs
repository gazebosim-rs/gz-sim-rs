//! Cold speculative contact of a translating CCD ball with a stationary box.
use crate::dynamics::RigidBody;
use crate::geometry::{Collider, ContactManifold};
use crate::math::{Real, Vector};
use parry::query::{PointQuery, ShapeCastOptions, ShapeCastStatus, cast_shapes};

/// Orient the speculative constraint along the contact normal at first touch.
/// The closest point at the beginning of the tick can be on a different normal
/// near a box edge. Its tangent plane cuts through otherwise clear trajectories.
pub(super) fn update(
    manifold: &mut ContactManifold,
    colliders: [&Collider; 2],
    bodies: [Option<&RigidBody>; 2],
    ball_index: usize,
    dt: Real,
) -> bool {
    let [point] = manifold.points.as_mut_slice() else {
        return false;
    };
    if point.dist <= 0.0
        || point.data.impulse != 0.0
        || point.data.warmstart_impulse != 0.0
        || point
            .data
            .warmstart_tangent_impulse
            .iter()
            .any(|v| *v != 0.0)
        || point.data.warmstart_twist_impulse != 0.0
        || point.data.warmstart_tangent_world != Vector::ZERO
    {
        return false;
    }
    let ball_co = colliders[ball_index];
    let box_co = colliders[1 - ball_index];
    let ball = ball_co.shape().as_ball().expect("qualified CCD ball");
    let cuboid = box_co.shape().as_cuboid().expect("qualified fixed cuboid");
    let body = bodies[ball_index].expect("qualified free flight");
    let center = ball_co.pos.translation;
    let delta = body.linvel() * dt;
    let box_space = box_co.pos.inverse_transform_point(center);
    let closest = box_space.clamp(-cuboid.half_extents, cuboid.half_extents);
    let separation = box_space - closest;
    let toward_ball = separation.normalize_or_zero();
    let local_delta = box_co.pos.rotation.inverse() * delta;
    // A convex supporting plane separates the entire sweep until the center can reach it.
    let approach = -toward_ball.dot(local_delta);
    if approach <= 0.0 {
        return false;
    }
    if separation.length() - ball.radius > approach {
        manifold.clear();
        return false;
    }

    let options = ShapeCastOptions {
        max_time_of_impact: 1.0,
        compute_impact_geometry_on_penetration: false,
        ..Default::default()
    };
    let Ok(hit) = cast_shapes(
        &box_co.pos,
        Vector::ZERO,
        cuboid,
        &ball_co.pos,
        delta,
        ball,
        options,
    ) else {
        return false;
    };
    let Some(hit) = hit else {
        // The chord misses the rounded edge. Suppress the cold speculative row for this step.
        manifold.clear();
        return false;
    };
    if hit.status != ShapeCastStatus::Converged {
        return false;
    }
    let at_touch = box_space + local_delta * hit.time_of_impact;
    let (projection, feature) = cuboid.project_local_point_and_get_feature(at_touch);
    let Some(local_normal) = (at_touch - projection.point).try_normalize() else {
        return false;
    };
    let world_normal = box_co.pos.rotation * local_normal;
    let ball_normal = ball_co.pos.rotation.inverse() * -world_normal;
    let ball_point = ball_normal * ball.radius;
    point.dist = (box_space - projection.point).dot(local_normal) - ball.radius;
    if ball_index == 0 {
        point.local_p1 = ball_point;
        point.local_p2 = projection.point;
        point.fid2 = feature.into();
        manifold.local_n1 = ball_normal;
        manifold.local_n2 = local_normal;
    } else {
        point.local_p1 = projection.point;
        point.local_p2 = ball_point;
        point.fid1 = feature.into();
        manifold.local_n1 = local_normal;
        manifold.local_n2 = ball_normal;
    }
    true
}
