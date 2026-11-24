// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn shape() -> Shape {
    Shape {
        half: DVec3::new(0.024, 0.012, 0.020),
        offset: DVec3::new(0.005, 0., -0.002),
        inertia: DVec3::new(1.45e-5, 2.6e-5, 1.92e-5),
        top: 0.014,
        g: 9.81,
    }
}
fn state(height: f64, speed: f64) -> FreeBoxState {
    FreeBoxState {
        mass_center: [0., 0., height],
        center_velocity: [0., 0., speed],
        orientation: DQuat::IDENTITY.to_array(),
        momentum: [0.; 3],
    }
}
#[test]
fn corner_certification_rejects_a_face_a_preexisting_overlap_and_a_turning_point() {
    let sh = shape();
    let face = sh.top + sh.half.z + sh.offset.z;
    assert!(sh.interval(state(face + 0.0001, -1.), DT, false).is_err());
    assert!(sh.interval(state(face - 0.0001, 1.), DT, false).is_err());
    assert!(sh.interval(state(face, 0.), DT, false).is_err());
    assert!(sh.interval(state(face, 1.), DT, false).unwrap().1.is_none());
    assert!(
        sh.interval(state(face + 0.000251, -1.), DT, false)
            .unwrap()
            .1
            .is_none()
    );
    let gap = DT + 0.5 * 9.81 * DT.powi(2) + 1e-9;
    assert!(
        sh.interval(state(face + gap, -1.), DT, false)
            .unwrap()
            .1
            .is_none()
    );
}

#[test]
fn edge_active_set_reduces_unloaded_rows_without_attractive_impulses() {
    for (b, expected) in [
        ([0.1, 1.], [0., 0.5]),
        ([1., 0.1], [0.5, 0.]),
        ([1., 1.], [1. / 3.8; 2]),
    ] {
        let j = edge::pair_impulses([2.; 2], 1.8, b, 0., 1.);
        for i in 0..2 {
            assert!((j[i] - expected[i]).abs() < 1e-12);
            let residual = 2. * j[i] + 1.8 * j[1 - i] - b[i];
            assert!(j[i] >= 0. && residual >= -1e-12 && (j[i] * residual).abs() < 1e-12);
        }
    }
    let s = state(0.031, -1.);
    let sh = shape();
    assert!(!edge::well_conditioned(s, [DVec3::X; 2], sh.inertia, 0.08));
    assert!(edge::well_conditioned(
        s,
        [DVec3::X * 0.024, DVec3::Y * 0.012],
        sh.inertia,
        0.08
    ));
    let face = sh.top + sh.half.z + sh.offset.z;
    assert!(sh.interval(state(face + 0.0001, -1.), DT, true).is_err());
}
