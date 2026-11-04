// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::f64::consts::{PI, TAU};

#[test]
fn position_uses_continuous_coordinates_and_signed_step_bounds() {
    let position = JointControllerMode::Position;
    let second = Duration::from_secs(1);
    assert_eq!(target_velocity(position, 0.25, 0.0, 2.0, second), Ok(0.25));
    assert_eq!(
        target_velocity(position, -0.25, 0.0, 2.0, second),
        Ok(-0.25)
    );
    assert_eq!(target_velocity(position, 2.0, 0.0, 2.0, second), Ok(2.0));
    assert_eq!(target_velocity(position, -3.0, 0.0, 2.0, second), Ok(-2.0));
    assert_eq!(target_velocity(position, 3.0, 0.0, 2.0, second), Ok(2.0));
    // Crossing pi follows the full continuous error, not the shorter wrapped arc.
    assert_eq!(target_velocity(position, PI, -PI, 1.0, second), Ok(1.0));
    assert_eq!(target_velocity(position, -PI, PI, 1.0, second), Ok(-1.0));
    assert_eq!(
        target_velocity(position, 4.0 * TAU, TAU, 1.0, second),
        Ok(1.0)
    );
    assert_eq!(
        target_velocity(position, TAU, 4.0 * TAU, 1.0, second),
        Ok(-1.0)
    );
    assert_eq!(target_velocity(position, TAU, TAU, 1.0, second), Ok(0.0));
}

#[test]
fn small_steps_distinguish_reaching_the_target_from_saturation() {
    let mode = JointControllerMode::Position;
    let dt = Duration::from_millis(125);
    assert_eq!(target_velocity(mode, 1.0625, 1.0, 1.0, dt), Ok(0.5));
    assert_eq!(target_velocity(mode, 0.9375, 1.0, 1.0, dt), Ok(-0.5));
    assert_eq!(target_velocity(mode, 1.125, 1.0, 1.0, dt), Ok(1.0));
    assert_eq!(target_velocity(mode, 1.25, 1.0, 1.0, dt), Ok(1.0));
    assert_eq!(target_velocity(mode, 0.75, 1.0, 1.0, dt), Ok(-1.0));
}

#[test]
fn velocity_saturates_finite_commands_without_position_subtraction() {
    let mode = JointControllerMode::Velocity;
    let dt = Duration::from_millis(1);
    for (command, expected) in [
        (0.0, 0.0),
        (0.5, 0.5),
        (-0.5, -0.5),
        (2.0, 2.0),
        (f64::MAX, 2.0),
        (-f64::MAX, -2.0),
    ] {
        assert_eq!(
            target_velocity(mode, command, -f64::MAX, 2.0, dt),
            Ok(expected)
        );
    }
}

#[test]
fn invalid_domains_are_typed_and_do_not_change_following_valid_results() {
    let dt = Duration::from_secs(1);
    for mode in [JointControllerMode::Position, JointControllerMode::Velocity] {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                target_velocity(mode, value, 0.0, 1.0, dt),
                Err(InvalidControllerInput::Command)
            );
            assert_eq!(
                target_velocity(mode, 0.0, value, 1.0, dt),
                Err(InvalidControllerInput::State)
            );
        }
        for limit in [0.0, -0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                target_velocity(mode, 0.0, 0.0, limit, dt),
                Err(InvalidControllerInput::Bounds)
            );
        }
        assert_eq!(
            target_velocity(mode, 0.0, 0.0, 1.0, Duration::ZERO),
            Err(InvalidControllerInput::Time)
        );
        assert_eq!(
            target_velocity(mode, 0.0, 0.0, f64::MAX, Duration::from_secs(2)),
            Err(InvalidControllerInput::Bounds)
        );
        assert_eq!(
            target_velocity(mode, 0.0, 0.0, f64::from_bits(1), Duration::from_nanos(1)),
            Err(InvalidControllerInput::Bounds)
        );
        // A positive representable subnormal movement is admitted without a
        // blanket small-value exclusion or arbitrary coordinate cutoff.
        assert_eq!(
            target_velocity(mode, f64::from_bits(1), 0.0, f64::from_bits(1), dt),
            Ok(f64::from_bits(1))
        );
        assert_eq!(target_velocity(mode, 0.0, 0.0, 1.0, Duration::MAX), Ok(0.0));
        assert_eq!(target_velocity(mode, 0.5, 0.0, 1.0, dt), Ok(0.5));
    }
    assert_eq!(
        target_velocity(JointControllerMode::Position, f64::MAX, -f64::MAX, 1.0, dt),
        Err(InvalidControllerInput::Command)
    );
}
