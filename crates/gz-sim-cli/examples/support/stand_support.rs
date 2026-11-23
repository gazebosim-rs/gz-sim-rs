// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec2;

/// Signed minimum distance to the edges of the convex hull of loaded sole points.
/// Positive means the projected mass center is inside a two-dimensional support.
pub fn margin(mut points: Vec<DVec2>, projected_com: DVec2) -> Option<f64> {
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    points.dedup();
    if points.len() < 3 {
        return None;
    }
    let mut hull = Vec::<DVec2>::with_capacity(points.len() + 1);
    for point in points.iter().copied() {
        push(&mut hull, 0, point);
    }
    let boundary = hull.len() - 1;
    for point in points.iter().rev().skip(1).copied() {
        push(&mut hull, boundary, point);
    }
    if hull.len() < 4 {
        return None;
    }
    hull.windows(2)
        .map(|e| (e[1] - e[0]).normalize().perp_dot(projected_com - e[0]))
        .reduce(f64::min)
}
fn push(hull: &mut Vec<DVec2>, start: usize, point: DVec2) {
    while hull.len() - start >= 2 {
        let i = hull.len() - 1;
        if (hull[i] - hull[i - 1]).perp_dot(point - hull[i]) > 0. {
            break;
        }
        hull.pop();
    }
    hull.push(point);
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sampled_support_excludes_unloaded_corners_and_handles_degenerate_contacts() {
        let triangle = vec![DVec2::ZERO, DVec2::X, DVec2::Y, DVec2::X];
        assert_eq!(margin(triangle.clone(), DVec2::splat(0.1)), Some(0.1));
        assert!(margin(triangle, DVec2::new(0.8, 0.8)).unwrap() < 0.);
        assert_eq!(
            margin(vec![DVec2::ZERO, DVec2::X, DVec2::X * 2.], DVec2::ZERO),
            None
        );
    }
}
