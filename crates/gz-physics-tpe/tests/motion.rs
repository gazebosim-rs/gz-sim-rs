// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
// Original Model.BasicAPI motion assertion: gz-physics/tpe/lib/src/Model_TEST.cc.
use gz_physics::BodyState;
use gz_physics_tpe::{Body, MotionError};
use serde::Deserialize;
use std::time::Duration;

fn advance(state: &BodyState, dt: Duration) -> Result<BodyState, MotionError> {
    let mut body = Body::try_from(*state)?;
    body.advance(dt)?;
    Ok(*body.state())
}

#[derive(Deserialize)]
struct Case {
    name: String,
    dt_nanos: u64,
    position: [f64; 3],
    orientation: [f64; 4],
    velocity: [f64; 3],
    angular_velocity: [f64; 3],
    expected_position: [f64; 3],
    expected_orientation: [f64; 4],
}

fn state() -> BodyState {
    BodyState {
        position: [1.0, 2.0, 3.0],
        orientation: [0.0, 0.0, 0.0, 1.0],
        velocity: [0.1, -0.2, 0.3],
        angular_velocity: [0.3, -0.4, 0.5],
    }
}

#[test]
fn pinned_native_model_motion_and_original_assertion_match() {
    let cases: Vec<Case> =
        serde_json::from_str(include_str!("fixtures/model_motion.json")).unwrap();
    assert_eq!(cases.len(), 12);
    for case in cases {
        let input = BodyState {
            position: case.position,
            orientation: case.orientation,
            velocity: case.velocity,
            angular_velocity: case.angular_velocity,
        };
        let actual = advance(&input, Duration::from_nanos(case.dt_nanos)).unwrap();
        // Native Pose3 == permits 1e-3 per component; this sample uses the stronger 1e-6.
        let tolerance = if case.name == "original_model_motion" {
            1e-6
        } else {
            1e-12
        };
        for (actual, expected) in actual.position.into_iter().chain(actual.orientation).zip(
            case.expected_position
                .into_iter()
                .chain(case.expected_orientation),
        ) {
            assert!(
                (actual - expected).abs() <= tolerance,
                "{}: {actual} != {expected}",
                case.name
            );
        }
        assert_eq!(actual.velocity, input.velocity);
        assert_eq!(actual.angular_velocity, input.angular_velocity);
        if case.name == "tiny_angular_with_translation" {
            assert_eq!(actual.orientation[0], case.expected_orientation[0]);
            assert_eq!(actual.orientation[0], 5e-201);
        }
    }
}

#[test]
fn zero_duration_and_native_stationary_threshold_preserve_pose_bits() {
    let mut input = state();
    input.position[0] = -0.0;
    let zero = advance(&input, Duration::ZERO).unwrap();
    assert_eq!(
        zero.position.map(f64::to_bits),
        input.position.map(f64::to_bits)
    );
    assert_eq!(
        zero.orientation.map(f64::to_bits),
        input.orientation.map(f64::to_bits)
    );
    input.velocity = [1e-3, -1e-3, -0.0];
    input.angular_velocity = [-1e-3, 1e-3, 0.0];
    let stopped = advance(&input, Duration::MAX).unwrap();
    assert_eq!(stopped, input);
    assert_eq!(stopped.position[0].to_bits(), (-0.0f64).to_bits());
}

#[test]
fn tiny_angles_survive_glam_squared_norm_underflow() {
    for angular in [1e-200, f64::MIN_POSITIVE] {
        let mut input = state();
        input.angular_velocity = [angular, 0.0, 0.0];
        input.orientation = [0.0, 0.0, 0.0, 1.0];
        let lost = glam::DQuat::from_scaled_axis(glam::DVec3::new(angular, 0.0, 0.0));
        assert_eq!(lost.x, 0.0);
        let result = advance(&input, Duration::from_secs(1)).unwrap();
        assert_eq!(result.orientation[0], angular * 0.5);
        assert!(result.orientation[0] > 0.0);
        assert_eq!(result.orientation[3], 1.0);
    }
}

#[test]
fn translation_only_keeps_orientation_and_updates_world_position() {
    let mut input = state();
    input.angular_velocity = [0.0; 3];
    let result = advance(&input, Duration::from_secs(2)).unwrap();
    assert_eq!(result.position, [1.2, 1.6, 3.6]);
    assert_eq!(result.orientation, input.orientation);
}

#[test]
fn repeated_world_rotation_keeps_unit_orientation_and_constant_velocities() {
    let mut body = Body::try_from(state()).unwrap();
    for _ in 0..10000 {
        body.advance(Duration::from_millis(1)).unwrap();
    }
    let current = *body.state();
    assert!((glam::DQuat::from_array(current.orientation).length_squared() - 1.0).abs() < 1e-14);
    for (actual, expected) in current.position.into_iter().zip([2.0, 0.0, 6.0]) {
        assert!((actual - expected).abs() < 1e-11);
    }
    assert_eq!(current.velocity, state().velocity);
    assert_eq!(current.angular_velocity, state().angular_velocity);
}

#[test]
fn nonfinite_fields_and_nonunit_quaternions_are_rejected_even_when_paused() {
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for field in 0..4 {
            let mut input = state();
            match field {
                0 => input.position[1] = invalid,
                1 => input.velocity[1] = invalid,
                2 => input.angular_velocity[1] = invalid,
                _ => input.orientation[1] = invalid,
            }
            for dt in [Duration::ZERO, Duration::from_secs(1)] {
                assert_eq!(advance(&input, dt), Err(MotionError::InvalidState));
            }
        }
    }
    for quaternion in [[0.0; 4], [0.0, 0.0, 0.0, 2.0], [f64::MAX, 0.0, 0.0, 1.0]] {
        let mut input = state();
        input.orientation = quaternion;
        assert_eq!(
            advance(&input, Duration::ZERO),
            Err(MotionError::InvalidState)
        );
    }
    assert!(
        MotionError::InvalidState
            .to_string()
            .contains("unit quaternion")
    );
}

#[test]
fn unrepresentable_translation_rotation_and_time_products_fail_explicitly() {
    for (position, velocity, angular, dt) in [
        (
            [f64::MAX, 0.0, 0.0],
            [f64::MAX, 0.0, 0.0],
            [0.0; 3],
            Duration::from_secs(1),
        ),
        ([0.0; 3], [f64::MAX, 0.0, 0.0], [0.0; 3], Duration::MAX),
        (
            [0.0; 3],
            [0.0; 3],
            [f64::MAX, 0.0, 0.0],
            Duration::from_secs(1),
        ),
        (
            [0.0; 3],
            [0.0; 3],
            [1e200, 0.0, 0.0],
            Duration::from_secs(1),
        ),
    ] {
        let input = BodyState {
            position,
            velocity,
            angular_velocity: angular,
            ..state()
        };
        assert_eq!(advance(&input, dt), Err(MotionError::Overflow));
    }
    assert!(MotionError::Overflow.to_string().contains("finite f64"));
}

#[test]
fn accepted_near_unit_input_is_renormalized_before_rotation_error_accumulates() {
    let mut input = state();
    input.orientation[3] = (1.0f64 + 5e-13).sqrt();
    let result = advance(&input, Duration::from_millis(1)).unwrap();
    assert!((glam::DQuat::from_array(result.orientation).length_squared() - 1.0).abs() < 1e-14);
    assert_eq!(result.velocity, input.velocity);
    assert_eq!(result.angular_velocity, input.angular_velocity);
}

#[test]
fn failed_step_preserves_validated_state_and_allows_retry() {
    let input = BodyState {
        position: [0.0; 3],
        velocity: [f64::MAX, 0.0, 0.0],
        angular_velocity: [0.0; 3],
        ..state()
    };
    let mut body = Body::try_from(input).unwrap();
    assert_eq!(body.advance(Duration::MAX), Err(MotionError::Overflow));
    assert_eq!(*body.state(), input);
    body.advance(Duration::ZERO).unwrap();
    assert_eq!(*body.state(), input);
    body.advance(Duration::from_nanos(1)).unwrap();
    assert!(body.state().position[0].is_finite());
    assert!(body.state().position[0] > 0.0);
    assert_eq!(body.state().velocity, input.velocity);
}
