// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{JointState, joint_drive::JointDrive};
#[test]
fn spring_damper_feedforward_and_both_saturations() {
    let drive = JointDrive::new(30., 3., 2., 7.).unwrap();
    let mut s = JointState {
        position: 0.,
        velocity: 0.,
    };
    assert_eq!(drive.effort(s, 0.0625, 0.25, -0.5).unwrap(), 2.125);
    for sign in [-1., 1.] {
        assert_eq!(drive.effort(s, sign * 2., 0., 0.).unwrap(), sign * 6.);
        s.velocity = -sign * 4.;
        assert_eq!(drive.effort(s, sign * 2., 0., 0.).unwrap(), sign * 7.);
        s.velocity = 0.;
    }
}
#[test]
fn invalid_limits_and_invalid_runtime_inputs_are_rejected() {
    for i in 0..4 {
        let mut v = [1.; 4];
        v[i] = 0.;
        assert!(JointDrive::new(v[0], v[1], v[2], v[3]).is_err());
        v[i] = f64::INFINITY;
        assert!(JointDrive::new(v[0], v[1], v[2], v[3]).is_err());
    }
    let d = JointDrive::new(1., 1., 1., 1.).unwrap();
    for i in 0..5 {
        let mut v = [0.; 5];
        v[i] = f64::NAN;
        assert!(
            d.effort(
                JointState {
                    position: v[0],
                    velocity: v[1]
                },
                v[2],
                v[3],
                v[4]
            )
            .is_err()
        );
    }
    let d = JointDrive::new(1., f64::MAX, 1., 1.).unwrap();
    assert!(
        d.effort(
            JointState {
                position: 0.,
                velocity: f64::MAX
            },
            0.,
            0.,
            0.
        )
        .is_err()
    );
}

#[test]
fn spherical_drive_uses_parent_axes_shortest_rotation_and_one_vector_limit() {
    use glam::{DQuat, DVec3};
    use gz_physics::joint_drive::SphericalDrive;
    let d = SphericalDrive::new(2., 3., 10.).unwrap();
    let a = DQuat::from_rotation_y(0.8);
    let goal = DQuat::from_rotation_z(0.2) * a;
    let run = |a, g| d.torque(a, g, DVec3::X, DVec3::Y, DVec3::Z).unwrap();
    assert!(run(a, goal).abs_diff_eq(DVec3::new(-3., 3., 1.4), 1e-12));
    assert_eq!(run(a, goal), run(-a, goal));
    assert_eq!(run(a, goal), run(a, -goal));
    let half = DQuat::from_xyzw(0., -1., 0., 0.);
    assert_eq!(run(DQuat::IDENTITY, half), run(DQuat::IDENTITY, -half));
    assert!(run(DQuat::IDENTITY, half).y > 0.);
    let cap = SphericalDrive::new(0., 0., 5.).unwrap();
    for f in [
        DVec3::ZERO,
        DVec3::new(3., 4., 0.),
        DVec3::new(3e300, 4e300, 0.),
    ] {
        let out = cap.torque(a, a, DVec3::ZERO, DVec3::ZERO, f).unwrap();
        assert_eq!(
            out,
            if f == DVec3::ZERO {
                f
            } else {
                DVec3::new(3., 4., 0.)
            }
        );
    }
    for (k, c, m) in [(-1., 1., 1.), (1., f64::NAN, 1.), (1., 1., 0.)] {
        assert!(SphericalDrive::new(k, c, m).is_err());
    }
    let zero = DQuat::from_xyzw(0., 0., 0., 0.);
    for (actual, target) in [(zero, a), (a, zero)] {
        assert!(
            d.torque(actual, target, DVec3::ZERO, DVec3::ZERO, DVec3::ZERO)
                .is_err()
        );
    }
    assert!(
        d.torque(a, a, DVec3::NAN, DVec3::ZERO, DVec3::ZERO)
            .is_err()
    );
}
