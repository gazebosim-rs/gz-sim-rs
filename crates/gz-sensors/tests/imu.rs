// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, EulerRot};
use gz_sensors::{ImuInput, InvalidImuInput, imu};

fn rest() -> ImuInput {
    ImuInput {
        world_orientation: DQuat::IDENTITY.to_array(),
        reference_orientation: DQuat::IDENTITY.to_array(),
        acceleration_sensor: [0.0; 3],
        angular_velocity_sensor: [0.0; 3],
        gravity_world: [0.0, 0.0, -9.81],
    }
}

// Measurement assertions from gz-sensors test/integration/imu.cc SensorReadings.
// Copyright (C) 2019 Open Source Robotics Foundation. Apache-2.0; see LICENSE.
#[test]
#[allow(clippy::approx_constant)]
fn upstream_rest_and_upside_down_measurements() {
    let input = ImuInput {
        reference_orientation: DQuat::from_rotation_z(1.57).to_array(),
        gravity_world: [0.0, 0.0, -4.0],
        ..rest()
    };
    let reading = imu(input).unwrap();
    assert_eq!(reading.angular_velocity, [0.0; 3]);
    assert_eq!(reading.specific_force, [0.0, 0.0, 4.0]);
    assert!(
        DQuat::from_array(reading.orientation).abs_diff_eq(DQuat::from_rotation_z(-1.57), 1e-6)
    );

    let reading = imu(ImuInput {
        world_orientation: DQuat::from_rotation_y(3.14).to_array(),
        acceleration_sensor: [0.0, 0.0, 3.0],
        angular_velocity_sensor: [1.0, 2.0, 3.0],
        ..input
    })
    .unwrap();
    assert_eq!(reading.angular_velocity, [1.0, 2.0, 3.0]);
    for ((actual, expected), tolerance) in reading
        .specific_force
        .into_iter()
        .zip([0.0, 0.0, -1.0])
        .zip([1e-2, 1e-6, 1e-5])
    {
        assert!((actual - expected).abs() < tolerance);
    }
    assert!(
        DQuat::from_array(reading.orientation)
            .abs_diff_eq(DQuat::from_euler(EulerRot::ZYX, -1.57, 3.14, 0.0), 1e-6)
    );
}

#[test]
fn stationary_free_fall_and_rotated_sensor_have_distinct_specific_force() {
    assert_eq!(imu(rest()).unwrap().specific_force, [0.0, 0.0, 9.81]);
    assert_eq!(
        imu(ImuInput {
            acceleration_sensor: [0.0, 0.0, -9.81],
            ..rest()
        })
        .unwrap()
        .specific_force,
        [0.0; 3]
    );
    let input = ImuInput {
        world_orientation: DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2).to_array(),
        ..rest()
    };
    let reading = imu(input).unwrap();
    assert!((reading.specific_force[1] - 9.81).abs() < 1e-12);
    assert!(reading.specific_force[2].abs() < 1e-12);
}

#[test]
fn invalid_samples_fail_without_nan_readings() {
    for input in [
        ImuInput {
            world_orientation: [0.0; 4],
            ..rest()
        },
        ImuInput {
            world_orientation: [f64::NAN; 4],
            ..rest()
        },
        ImuInput {
            reference_orientation: [0.0; 4],
            ..rest()
        },
        ImuInput {
            acceleration_sensor: [f64::INFINITY; 3],
            ..rest()
        },
        ImuInput {
            angular_velocity_sensor: [f64::NAN; 3],
            ..rest()
        },
        ImuInput {
            gravity_world: [f64::NEG_INFINITY; 3],
            ..rest()
        },
        ImuInput {
            acceleration_sensor: [f64::MAX; 3],
            gravity_world: [-f64::MAX; 3],
            ..rest()
        },
    ] {
        assert_eq!(imu(input), Err(InvalidImuInput));
    }
    assert!(InvalidImuInput.to_string().contains("unit quaternions"));
}
