// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::primitive_replay_view::View;
use rapier3d_f64::math::Vector;
use serde_json::json;

fn rod_rotation(dir: Vector) -> [f64; 4] {
    let dir = dir.normalize();
    if dir.x > -1. + 1e-8 {
        let w = ((1. + dir.x) * 0.5).sqrt();
        return [0., -dir.z / (2. * w), dir.y / (2. * w), w];
    }
    let transverse = dir.y.hypot(dir.z);
    if transverse == 0. {
        return [0., 0., 1., 0.];
    }
    // atan2 retains the transverse component even when x rounds to -1.
    let (s, w) = (0.5 * transverse.atan2(dir.x)).sin_cos();
    [0., -dir.z / transverse * s, dir.y / transverse * s, w]
}

pub(super) fn rod(
    v: &mut View,
    id: u64,
    name: &str,
    start: Vector,
    end: Vector,
    length: f64,
    color: [f64; 4],
) {
    let displacement = end - start;
    let rotation = rod_rotation(displacement);
    v.add(
        id,
        name,
        ((start + end) * 0.5).to_array(),
        rotation,
        json!({"kind":"box","size":[length,0.06,0.06]}),
        color,
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use rapier3d_f64::math::Rotation;

    #[test]
    fn rod_orientation_is_unit_and_points_at_the_joint_near_a_half_turn() {
        for direction in [
            Vector::X,
            -Vector::X,
            Vector::new(-1., 1e-9, -4e-10),
            Vector::new(-1., -2e-7, 3e-8),
        ] {
            let q = Rotation::from_array(rod_rotation(direction));
            assert!((q.length() - 1.).abs() < 1e-14);
            assert!((q * Vector::X - direction.normalize()).length() < 1e-14);
        }
    }
}
