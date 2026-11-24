// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[test]
fn finite_pad_faces_corners_and_interior_have_correct_outward_normals() {
    let h = [0.01, 0.03];
    let r = 0.02;
    for (p, gap, n) in [
        ([0.04, 0.], 0.01, [1., 0.]),
        ([-0.04, 0.], 0.01, [-1., 0.]),
        ([0., 0.05], 0., [0., 1.]),
        ([0., -0.05], 0., [0., -1.]),
        ([0.013, 0.034], -0.015, [0.6, 0.8]),
        ([0., 0.], -0.03, [1., 0.]),
        ([0., -0.029], -0.021, [0., -1.]),
    ] {
        let (got, normal) = box_gap(p, h, r, 0., 1.);
        assert!(
            (got - gap).abs() < 1e-12
                && (normal[0] - n[0]).abs() < 1e-12
                && (normal[1] - n[1]).abs() < 1e-12
        );
    }
}
