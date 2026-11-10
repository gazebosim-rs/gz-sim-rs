// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn sealed_frames_keep_custom_bits_and_native_named_table_axes() {
    let initial = DQuat::from_xyzw(-0.0, 0.0, -0.0, -1.0);
    assert_eq!(
        reference(Frame::Custom, initial)
            .to_array()
            .map(f64::to_bits),
        initial.to_array().map(f64::to_bits)
    );
    let vectors = [glam::DVec3::X, glam::DVec3::Y, glam::DVec3::Z];
    for (frame, axes) in [
        (Frame::Enu, [glam::DVec3::X, glam::DVec3::Y, glam::DVec3::Z]),
        (
            Frame::Ned,
            [glam::DVec3::Y, glam::DVec3::X, -glam::DVec3::Z],
        ),
        (
            Frame::Nwu,
            [glam::DVec3::Y, -glam::DVec3::X, glam::DVec3::Z],
        ),
    ] {
        let selected = reference(frame, initial);
        assert_eq!(
            selected.to_array().map(f64::to_bits),
            reference(frame, DQuat::IDENTITY)
                .to_array()
                .map(f64::to_bits)
        );
        assert!((selected.length_squared() - 1.0).abs() < 1e-12);
        for (vector, expected) in vectors.into_iter().zip(axes) {
            assert!((selected * vector - expected).length() < 1e-9);
        }
        // Do not silently canonicalize the sign of the measured world pose.
        let world = DQuat::from_rotation_x(0.4);
        let positive = gz_sensors::imu(gz_sensors::ImuInput {
            world_orientation: world.to_array(),
            reference_orientation: selected.to_array(),
            acceleration_sensor: [0.0; 3],
            angular_velocity_sensor: [0.0; 3],
            gravity_world: [0.0; 3],
        })
        .unwrap();
        let negative = gz_sensors::imu(gz_sensors::ImuInput {
            world_orientation: (-world).to_array(),
            reference_orientation: selected.to_array(),
            acceleration_sensor: [0.0; 3],
            angular_velocity_sensor: [0.0; 3],
            gravity_world: [0.0; 3],
        })
        .unwrap();
        for (a, b) in positive.orientation.into_iter().zip(negative.orientation) {
            assert!((a + b).abs() < 1e-9);
        }
    }
}

#[test]
fn validation_is_named_only_and_preserves_neighboring_refusals() {
    for (name, frame) in [
        ("CUSTOM", Frame::Custom),
        ("ENU", Frame::Enu),
        ("NED", Frame::Ned),
        ("NWU", Frame::Nwu),
    ] {
        let mut config = sdformat::Imu {
            localization: name.into(),
            ..Default::default()
        };
        assert_eq!(validate(&config, false, "sensor").unwrap(), frame);
        assert_eq!(
            validate(&config, true, "sensor").is_ok(),
            frame == Frame::Custom
        );
        config.custom_rpy = [-0.0, 0.0, -0.0];
        assert_eq!(validate(&config, false, "sensor").unwrap(), frame);
        config.custom_rpy = [0.0, 0.0, 0.1];
        assert!(validate(&config, false, "sensor").is_err());
        config.custom_rpy = [0.0; 3];
        config.custom_rpy_parent_frame = "world".into();
        assert!(validate(&config, false, "sensor").is_err());
    }
    for name in ["", "ned", "NONE", "UNKNOWN"] {
        let config = sdformat::Imu {
            localization: name.into(),
            ..Default::default()
        };
        assert!(
            matches!(validate(&config, false, "sensor"), Err(LoadError::Unsupported { entity, feature: "IMU orientation configuration" }) if entity == "sensor")
        );
    }
}
