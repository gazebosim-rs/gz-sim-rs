// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::grasp_guard::SupportedSetdown;

#[test]
fn tangential_loads_and_moving_wrench_origins_give_the_same_top_face_intersection() {
    let guard =
        FiniteDeckMargin::new([0.1, 0.09, 0.06], 0.002, DQuat::IDENTITY.to_array()).unwrap();
    let j = DVec3::new(0.004, -0.008, 0.02);
    // Construct the moment independently by applying the impulse at a chosen
    // top-face contact. Torsion about the vertical axis does not move the intersection.
    for (x, y, d, expected) in [
        (0.08, -0.08, [0.003, -0.003, 0.], true),
        (0.099, 0., [0.; 3], false),
        (-0.099, 0., [0.; 3], false),
        (0., 0.089, [0.; 3], false),
        (0., -0.089, [0.; 3], false),
        (0., 0.087, [0., -0.005, 0.], false), // the pressure was inside at the start only
        (0., 0.091, [0., 0.005, 0.], false),  // the pressure is inside at the end only
        (0.097, 0., [0., 0., 0.01], false),   // shearing load crosses the translated top plane
    ] {
        let k = DVec3::new(x, y, 0.06).cross(j) + DVec3::Z * 0.0002;
        assert_eq!(guard.accepts(j.to_array(), k.to_array(), d), expected);
    }
    for (half, inset) in [
        ([0.1, 0., 0.06], 0.002),
        ([0.1, 0.09, f64::INFINITY], 0.002),
        ([0.1, 0.09, 0.06], 0.),
        ([0.1, 0.09, 0.06], 0.09),
        ([0.1, 0.09, 0.06], f64::NAN),
    ] {
        assert!(FiniteDeckMargin::new(half, inset, DQuat::IDENTITY.to_array()).is_none());
    }
    for (impulse, moment, motion) in [
        ([0.; 3], [0.; 3], [0.; 3]),
        ([0., 0., -1.], [0.; 3], [0.; 3]),
        ([f64::NAN, 0., 1.], [0.; 3], [0.; 3]),
        ([0., 0., 1.], [0., f64::INFINITY, 0.], [0.; 3]),
        ([0., 0., 1.], [0.; 3], [f64::NAN, 0., 0.]),
    ] {
        assert!(!guard.accepts(impulse, moment, motion));
    }
}

#[test]
fn the_edge_veto_holds_the_grip_rearms_the_wait_and_keeps_load_feedback_available() {
    let mut control = SupportedSetdown::new(2.).unwrap();
    let mut guarded = SupportedSetdown::new(2.).unwrap();
    for tick in 0..65 {
        let inside = tick >= 30 && tick != 51;
        let up = if tick == 0 { 1e-17 } else { 0.0018 };
        let a = control.advance(up).unwrap();
        let g = guarded.advance_with_support_check(up, inside).unwrap();
        assert_eq!(g.lift_offset_m, a.lift_offset_m);
        if tick < 49 {
            assert_eq!(g.opening_ms, 0);
        }
        if tick == 49 {
            assert_eq!(g.opening_ms, 1);
        }
        if tick >= 51 {
            assert_eq!(g.opening_ms, 2);
            assert!(!g.advancing_opening);
        }
    }
    let mut missing = SupportedSetdown::new(2.).unwrap();
    for _ in 0..2000 {
        let g = missing.advance_with_support_check(1e-17, true).unwrap();
        assert_eq!(g.opening_ms, 0);
    }
    assert!(
        missing
            .advance_with_support_check(0., false)
            .unwrap()
            .search_exhausted
    );
}

#[test]
fn a_yawed_collision_uses_its_own_axes_and_declines_tilt_and_invalid_frames() {
    let q = DQuat::from_rotation_z(0.8);
    let guard = FiniteDeckMargin::new([0.16, 0.04, 0.03], 0.002, q.to_array()).unwrap();
    let j = DVec3::new(0.008, 0.004, 0.05);
    for (y, ok) in [(0.03, true), (0.039, false)] {
        let point = DVec3::new(0.11, y, 0.03);
        assert_eq!(
            guard.accepts(
                (q * j).to_array(),
                (q * (point.cross(j))).to_array(),
                [0.; 3]
            ),
            ok
        );
    }
    for rotation in [
        [0.; 4],
        [0., 0., 0., f64::NAN],
        DQuat::from_rotation_y(0.03).to_array(),
    ] {
        assert!(FiniteDeckMargin::new([0.1; 3], 0.002, rotation).is_none());
    }
}
