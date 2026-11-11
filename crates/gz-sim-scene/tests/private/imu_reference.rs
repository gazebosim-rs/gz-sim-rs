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
        if name == "CUSTOM" {
            assert_eq!(
                validate(&config, false, "sensor").unwrap(),
                Frame::CustomWorld([0.0f64.to_bits(); 3])
            );
            assert!(validate(&config, true, "sensor").is_err());
        } else {
            assert!(validate(&config, false, "sensor").is_err());
        }
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

#[test]
fn custom_world_payload_keeps_angle_bits_and_ignores_initial_reference() {
    let initial = DQuat::from_xyzw(-0.0, 0.0, -0.0, -1.0);
    for angles in [
        [-0.0, 0.0, -0.0],
        [0.3, -0.4, 0.5],
        [std::f64::consts::PI, 0.0, -std::f64::consts::PI],
        [0.0, std::f64::consts::FRAC_PI_2, 0.0],
    ] {
        let config = sdformat::Imu {
            custom_rpy: angles,
            custom_rpy_parent_frame: "world".into(),
            ..Default::default()
        };
        let frame = validate(&config, false, "sensor").unwrap();
        assert_eq!(frame, Frame::CustomWorld(angles.map(f64::to_bits)));
        let selected = reference(frame, initial);
        assert_eq!(
            selected.to_array().map(f64::to_bits),
            reference(frame, DQuat::from_rotation_x(0.7))
                .to_array()
                .map(f64::to_bits)
        );
        // Independent axis-angle construction checks geometry, not native bits.
        let expected = DQuat::from_rotation_z(angles[2])
            * DQuat::from_rotation_y(angles[1])
            * DQuat::from_rotation_x(angles[0]);
        for (actual, expected) in selected.to_array().into_iter().zip(expected.to_array()) {
            assert!((actual - expected).abs() < 1e-9);
        }
        assert!((selected.length_squared() - 1.0).abs() < 1e-12);
        assert_eq!(
            reference(Frame::Custom, initial)
                .to_array()
                .map(f64::to_bits),
            initial.to_array().map(f64::to_bits)
        );
    }
}

#[test]
fn custom_world_rejects_unrepresentable_angles_geodesy_and_other_parents() {
    let mut config = sdformat::Imu {
        custom_rpy: [0.3, -0.4, 0.5],
        custom_rpy_parent_frame: "world".into(),
        ..Default::default()
    };
    assert!(validate(&config, true, "sensor").is_err());
    for value in [
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::from_bits(1),
    ] {
        config.custom_rpy[1] = value;
        assert!(matches!(
            validate(&config, false, "sensor"),
            Err(LoadError::Unsupported {
                feature: "IMU orientation configuration",
                ..
            })
        ));
    }
    config.custom_rpy = [0.3, -0.4, 0.5];
    for parent in ["", "body", "World", "default", "world "] {
        config.custom_rpy_parent_frame = parent.into();
        assert!(validate(&config, false, "sensor").is_err());
    }
    config.custom_rpy_parent_frame = "world".into();
    for name in ["ENU", "NED", "NWU", "UNKNOWN"] {
        config.localization = name.into();
        assert!(validate(&config, false, "sensor").is_err());
    }
    config.localization = "CUSTOM".into();
    assert!(validate(&config, false, "sensor").is_ok());
}
