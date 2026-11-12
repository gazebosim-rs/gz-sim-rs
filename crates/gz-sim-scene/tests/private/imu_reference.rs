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
        assert_eq!(validate(&config, None, "sensor").unwrap(), frame);
        assert_eq!(
            validate(&config, Some(&geodesy()), "sensor").is_ok(),
            frame == Frame::Custom
        );
        config.custom_rpy = [-0.0, 0.0, -0.0];
        assert_eq!(validate(&config, None, "sensor").unwrap(), frame);
        config.custom_rpy = [0.0, 0.0, 0.1];
        assert!(validate(&config, None, "sensor").is_err());
        config.custom_rpy = [0.0; 3];
        config.custom_rpy_parent_frame = "world".into();
        if name == "CUSTOM" {
            assert_eq!(
                validate(&config, None, "sensor").unwrap(),
                Frame::CustomWorld([0.0f64.to_bits(); 3])
            );
            assert!(validate(&config, Some(&geodesy()), "sensor").is_err());
        } else {
            assert!(validate(&config, None, "sensor").is_err());
        }
    }
    for name in ["", "ned", "NONE", "UNKNOWN"] {
        let config = sdformat::Imu {
            localization: name.into(),
            ..Default::default()
        };
        assert!(
            matches!(validate(&config, None, "sensor"), Err(LoadError::Unsupported { entity, feature: "IMU orientation configuration" }) if entity == "sensor")
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
        let frame = validate(&config, None, "sensor").unwrap();
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
    assert!(validate(&config, Some(&geodesy()), "sensor").is_err());
    for value in [
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::from_bits(1),
    ] {
        config.custom_rpy[1] = value;
        assert!(matches!(
            validate(&config, None, "sensor"),
            Err(LoadError::Unsupported {
                feature: "IMU orientation configuration",
                ..
            })
        ));
    }
    config.custom_rpy = [0.3, -0.4, 0.5];
    for parent in ["", "body", "World", "default", "world "] {
        config.custom_rpy_parent_frame = parent.into();
        assert!(validate(&config, None, "sensor").is_err());
    }
    config.custom_rpy_parent_frame = "world".into();
    for name in ["ENU", "NED", "NWU", "UNKNOWN"] {
        config.localization = name.into();
        assert!(validate(&config, None, "sensor").is_err());
    }
    config.localization = "CUSTOM".into();
    assert!(validate(&config, None, "sensor").is_ok());
}

fn spherical(heading: f64) -> sdformat::SphericalCoordinates {
    sdformat::SphericalCoordinates {
        surface: sdformat::SphericalSurface::EarthWgs84,
        orientation: sdformat::WorldFrameOrientation::EastNorthUp,
        latitude_radians: 0.0,
        longitude_radians: 0.0,
        elevation: 0.0,
        heading_radians: heading,
        source: sdformat::Source {
            uri: "m48-private".into(),
            line: 1,
            column: 1,
        },
    }
}
fn geodesy() -> sdformat::SphericalCoordinates {
    sdformat::SphericalCoordinates {
        latitude_radians: 1.0,
        ..spherical(0.0)
    }
}
fn heading_config(name: &str) -> sdformat::Imu {
    sdformat::Imu {
        localization: name.into(),
        custom_rpy: if name == "CUSTOM" {
            [0.3, -0.4, 0.5]
        } else {
            [0.0; 3]
        },
        custom_rpy_parent_frame: if name == "CUSTOM" {
            "world".into()
        } else {
            String::new()
        },
        ..Default::default()
    }
}

#[test]
fn heading_payloads_preserve_signed_zero_and_all_old_reference_bits() {
    let initial = DQuat::from_xyzw(-0.0, 0.0, -0.0, -1.0);
    for name in ["ENU", "NED", "NWU", "CUSTOM"] {
        let config = heading_config(name);
        let absent = validate(&config, None, "sensor").unwrap();
        for heading in [0.0_f64, -0.0] {
            let frame = validate(&config, Some(&spherical(heading)), "sensor").unwrap();
            let bits = match frame {
                Frame::EnuHeading(bits) | Frame::NedHeading(bits) | Frame::NwuHeading(bits) => bits,
                Frame::CustomWorldHeading(angles, bits) => {
                    assert_eq!(angles, config.custom_rpy.map(f64::to_bits));
                    bits
                }
                _ => panic!("explicit spherical metadata must keep a heading payload"),
            };
            assert_eq!(bits, heading.to_bits());
            if heading.to_bits() == 0 {
                assert_eq!(
                    reference(frame, initial).to_array().map(f64::to_bits),
                    reference(absent, initial).to_array().map(f64::to_bits)
                );
            }
        }
    }
    for world in [
        spherical(-0.0),
        geodesy(),
        sdformat::SphericalCoordinates {
            surface: sdformat::SphericalSurface::MoonScs,
            heading_radians: f64::NAN,
            ..spherical(0.0)
        },
    ] {
        let config = sdformat::Imu::default();
        assert_eq!(
            validate(&config, Some(&world), "sensor").unwrap(),
            Frame::Custom
        );
        assert_eq!(
            reference(Frame::Custom, initial)
                .to_array()
                .map(f64::to_bits),
            initial.to_array().map(f64::to_bits)
        );
    }
}

#[test]
fn heading_reference_is_left_native_product_and_preserves_quaternion_sign() {
    for name in ["ENU", "NED", "NWU", "CUSTOM"] {
        let config = heading_config(name);
        let table = match name {
            "ENU" => DQuat::IDENTITY,
            "NED" => {
                DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2)
                    * DQuat::from_rotation_x(std::f64::consts::PI)
            }
            "NWU" => DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
            _ => {
                DQuat::from_rotation_z(0.5)
                    * DQuat::from_rotation_y(-0.4)
                    * DQuat::from_rotation_x(0.3)
            }
        };
        for heading in [
            0.0,
            -0.0,
            std::f64::consts::FRAC_PI_4,
            -std::f64::consts::FRAC_PI_3,
            std::f64::consts::PI,
            -std::f64::consts::PI,
        ] {
            let selected = validate(&config, Some(&spherical(heading)), "sensor").unwrap();
            let q = reference(selected, DQuat::from_rotation_x(0.7));
            let expected = (DQuat::from_rotation_z(heading) * table).to_array();
            for (actual, expected) in q.to_array().into_iter().zip(expected) {
                assert!((actual - expected).abs() < 1e-9);
            }
            assert_eq!(
                q.to_array().map(f64::to_bits),
                reference(selected, DQuat::IDENTITY)
                    .to_array()
                    .map(f64::to_bits)
            );
            let input = gz_sensors::ImuInput {
                world_orientation: DQuat::from_rotation_y(0.4).to_array(),
                reference_orientation: q.to_array(),
                acceleration_sensor: [1.0, -2.0, 3.0],
                angular_velocity_sensor: [-0.2, 0.3, 0.4],
                gravity_world: [0.0; 3],
            };
            let positive = gz_sensors::imu(input).unwrap();
            let negative = gz_sensors::imu(gz_sensors::ImuInput {
                world_orientation: input.world_orientation.map(|v| -v),
                ..input
            })
            .unwrap();
            for (a, b) in positive.orientation.into_iter().zip(negative.orientation) {
                assert!((a + b).abs() < 1e-9);
            }
            assert_eq!(positive.angular_velocity, negative.angular_velocity);
            assert_eq!(positive.specific_force, negative.specific_force);
        }
    }
}

#[test]
fn heading_validation_refuses_every_nonheading_dto_field_before_runtime_state() {
    for name in ["ENU", "NED", "NWU", "CUSTOM"] {
        let config = heading_config(name);
        for heading in [
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::from_bits(1),
            -f64::from_bits(1),
            f64::from_bits(std::f64::consts::PI.to_bits() + 1),
            -f64::from_bits(std::f64::consts::PI.to_bits() + 1),
        ] {
            assert!(
                matches!(validate(&config, Some(&spherical(heading)), "sensor"), Err(LoadError::Unsupported { entity, feature: "IMU orientation configuration" }) if entity == "sensor")
            );
        }
        for field in 0..3 {
            for value in [1.0, f64::NAN, f64::INFINITY] {
                let mut world = spherical(0.0);
                match field {
                    0 => world.latitude_radians = value,
                    1 => world.longitude_radians = value,
                    _ => world.elevation = value,
                }
                assert!(validate(&config, Some(&world), "sensor").is_err());
            }
        }
        assert!(
            validate(
                &config,
                Some(&sdformat::SphericalCoordinates {
                    surface: sdformat::SphericalSurface::MoonScs,
                    ..spherical(0.0)
                }),
                "sensor"
            )
            .is_err()
        );
        let mut zeros = spherical(-0.0);
        zeros.latitude_radians = -0.0;
        zeros.longitude_radians = -0.0;
        zeros.elevation = -0.0;
        assert!(validate(&config, Some(&zeros), "sensor").is_ok());
    }
}
