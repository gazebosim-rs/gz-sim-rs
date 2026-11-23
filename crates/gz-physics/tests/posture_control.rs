// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::posture::{
    MassCenterFeedback, MassCenterSum, PostureAxisLaw, PostureFeedback, PostureSetpoint,
};
use gz_physics::{BodyState, JointState};

#[test]
fn a_rotated_payload_and_a_moving_wrist_use_mass_weighted_world_com_and_com_velocity() {
    let body = |position, velocity, orientation: DQuat| BodyState {
        position,
        orientation: orientation.to_array(),
        velocity,
        angular_velocity: [0., 0., 8.],
    };
    let mut sum = MassCenterSum::default();
    sum.add(
        3.,
        [0.2, 0., 0.],
        body(
            [1., 2., 0.],
            [1., 0., -1.],
            DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
        ),
    );
    sum.add(
        1.,
        [0., 0., 0.4],
        body([1., -1., 1.], [0., 2., 1.], DQuat::IDENTITY),
    );
    let (position, velocity) = sum.finish(4.);
    assert!(DVec3::from(position).distance(DVec3::new(1., 1.4, 0.35)) < 1e-14);
    assert_eq!(velocity, [0.75, 0.5, -0.5]);
}

#[test]
fn balance_projects_both_axes_and_limits_torque_independently_of_travel() {
    let feedback = MassCenterFeedback {
        reference: [1., 1., 1.],
        gains: [2., 0.5],
        max_correction: [0.25, 0.5],
    };
    let correction = feedback.correction([1.5, 1.125, 8.], [0.25, 0.125, -20.], [0.125, 0.]);
    assert_eq!(correction, [0.25, -0.3125]);
    assert_eq!(
        feedback.correction([2., -1., 5.], [0.; 3], [1., -2.]),
        [0., 0.]
    );
    let mut axis = PostureAxisLaw {
        joint_id: 7,
        reference_rad: 0.125,
        stiffness: 8.,
        damping: 2.,
        max_effort_nm: 2.,
        limits: [-0.375, 0.5],
        feedback: PostureFeedback::X,
    };
    let state = JointState {
        position: 0.25,
        velocity: 0.125,
    };
    assert_eq!(axis.command(state, correction, None), [0.375, 0.75]);
    axis.feedback = PostureFeedback::NegativeY;
    assert_eq!(axis.command(state, correction, None), [-0.1875, -2.]);
    axis.feedback = PostureFeedback::None;
    let mut command = PostureSetpoint {
        joint_id: 7,
        target_rad: 2.,
        feedforward_nm: -1.,
        direct_effort_nm: None,
    };
    assert_eq!(axis.command(state, correction, Some(&command)), [0.5, 0.75]);
    command.target_rad = -3.;
    command.direct_effort_nm = Some(6.);
    assert_eq!(
        axis.command(state, correction, Some(&command)),
        [-0.375, 2.]
    );
}
