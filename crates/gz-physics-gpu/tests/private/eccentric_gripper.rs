// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[test]
fn offset_rotation_matches_host_trigonometry_across_turns_and_quadrants() {
    let c = constants();
    for i in -4096..=4096 {
        let a = i as f64 * 0.0187;
        let p = [0.006, -0.0038];
        let [x, z] = rotated_offset(a, p, &c);
        let (s, cos) = a.sin_cos();
        assert!(
            (x - (cos * p[0] - s * p[1])).abs() < 3e-16
                && (z - (s * p[0] + cos * p[1])).abs() < 3e-16
        );
    }
}
