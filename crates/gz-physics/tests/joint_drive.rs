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
