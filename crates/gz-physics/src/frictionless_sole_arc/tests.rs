// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DQuat;
fn fixture(tilt: f64, rate: f64, inertia: f64) -> (FrictionlessSoleArc, EdgeSupportState) {
    let box_ = FrictionlessSoleArc {
        mass: 2.,
        inertia_body: DMat3::from_diagonal(DVec3::new(inertia, 0.055, 0.060)),
        corners: std::array::from_fn(|k| {
            DVec3::new(
                if k & 2 == 0 { -0.18 } else { 0.18 },
                if k & 1 == 0 { -0.1 } else { 0.1 },
                -0.13,
            )
        }),
        normal: DVec3::Z,
        plane_offset: 0.27,
        gravity: DVec3::new(-0.1, 0.2, -9.81),
    };
    let orientation = DQuat::from_rotation_z(0.24) * DQuat::from_rotation_x(tilt);
    let w = orientation * DVec3::X * rate;
    let r = orientation * box_.corners[0];
    (
        box_,
        EdgeSupportState {
            center: DVec3::new(0.08, -0.04, box_.plane_offset - r.z),
            velocity: DVec3::new(0.13, -0.07, -w.cross(r).z),
            angular_velocity: w,
            orientation,
        },
    )
}
#[test]
fn departing_and_arriving_vertices_swap_in_the_same_public_step() {
    let (b, s) = fixture(3e-4, -2., 0.05);
    let full = b.advance(s, [0, 2], 0.0005).unwrap().unwrap();
    let (hit, second) = full.transfer.unwrap();
    let t = hit.seconds;
    assert!(t > 0.00014 && t < 0.00016);
    assert!(hit.numerical_bracket[0] <= t && t <= hit.numerical_bracket[1]);
    assert_eq!(second.vertices, [1, 3]);
    let before = hit.before.angular_velocity.length();
    let after = hit.after.angular_velocity.length();
    let mass_radius_squared = b.mass * 0.1_f64.powi(2);
    let i = b.inertia_body.x_axis.x;
    assert!((after / before - (i - mass_radius_squared) / (i + mass_radius_squared)).abs() < 2e-13);
    assert!(hit.dissipated_energy > 0.);
    for k in [0, 2] {
        assert!(hit.pressure.normal[k] > 0.);
    }
    for k in [1, 3] {
        assert_eq!(hit.pressure.normal[k], 0.);
    }
    let first = b.advance(s, [2, 0], 0.0001).unwrap().unwrap();
    assert!(first.transfer.is_none());
    let rem = b.advance(first.end(), [2, 0], 0.0004).unwrap().unwrap();
    assert!((rem.transfer.unwrap().0.seconds + 0.0001 - t).abs() < 3e-15);
    assert!(full.end().center.distance(rem.end().center) < 1e-14);
    assert!(
        full.end()
            .angular_velocity
            .distance(rem.end().angular_velocity)
            < 1e-12
    );
}
#[test]
fn rejects_full_face_arrest_tension_multiple_events_and_invalid_sites() {
    let (b, s) = fixture(0.0003, -2., 0.012);
    assert!(b.advance(s, [0, 2], 0.0005).unwrap().is_none());
    let (mut b, s) = fixture(0.0003, -2., 0.05);
    assert!(b.advance(s, [0, 2], 1.).unwrap().is_none());
    assert!(b.advance(s, [0, 3], 0.0005).is_err());
    assert!(b.advance(s, [0, 2], f64::NAN).is_err());
    b.gravity.z = 9.81;
    assert!(b.advance(s, [0, 2], 0.0005).unwrap().is_none());
}
