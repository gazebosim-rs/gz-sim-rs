// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::collections::BTreeSet;
use std::time::Duration;

// The shared audited schema retains Suppressed; this forced capture has none.
#[allow(dead_code)]
mod native {
    // Root attaches this test and copies the byte-exact literal only after its
    // independent fixed-grammar DATA review. include! avoids rustfmt traversal.
    include!("../fixtures/m43_native_imu_reference_frames.rs");
}

fn close<const N: usize>(actual: [f64; N], expected: [u64; N], label: &str) {
    for (actual, expected) in actual.into_iter().zip(expected.map(f64::from_bits)) {
        assert!(actual.is_finite() && expected.is_finite());
        assert!(
            (actual - expected).abs() < 1e-9,
            "{label}: {actual} vs {expected}"
        );
    }
}

fn noise(config: native::NativeNoise) -> sdformat::Noise {
    let p = config.parameters.map(f64::from_bits);
    assert!(p.into_iter().all(f64::is_finite));
    // The captured campaign uses only deterministic zero-variance means. This
    // control cannot establish equivalence between the native and Rust RNGs.
    assert_eq!(&p[1..], &[0.0; 6]);
    sdformat::Noise {
        kind: match config.kind {
            "none" => sdformat::NoiseKind::None,
            "gaussian" => sdformat::NoiseKind::Gaussian,
            other => panic!("unreviewed native noise kind {other}"),
        },
        mean: p[0],
        stddev: p[1],
        bias_mean: p[2],
        bias_stddev: p[3],
        dynamic_bias_stddev: p[4],
        dynamic_bias_correlation_time: p[5],
        precision: p[6],
    }
}

#[test]
fn actual_native_named_references_match_all_component_readings_at_unchanged_bound() {
    let mut modes = BTreeSet::new();
    let (mut configurations, mut calls, mut observed, mut suppressed, mut disabled) =
        (0, 0, 0, 0, 0);
    for &(label, config, rows) in native::CASES {
        configurations += 1;
        assert!(modes.insert(config.mode));
        assert_eq!(label, config.mode);
        assert_eq!(config.instance, 0);
        assert_eq!(
            config.topic,
            format!("/m43/native/imu_reference/{}", config.mode)
        );
        assert_eq!(config.sensor_name, "imu");
        assert_eq!(config.frame_id, "m43/native/imu");
        assert_eq!(config.frequency, 0);
        assert_eq!(
            config.orientation_enabled_actual,
            config.orientation_enabled
        );
        assert_eq!(
            config.orientation_enabled,
            !config.mode.contains("-disabled")
        );
        assert_eq!(
            config.reference_tag_present,
            config.localization != "CUSTOM" || config.mode == "custom-explicit-motion"
        );
        assert_eq!(config.world_frame_call, config.reference_tag_present);
        assert_eq!(
            config.expected_warning,
            if config.mode == "custom-explicit-motion" {
                "CUSTOM_EMPTY_PARENT"
            } else {
                "NONE"
            }
        );
        assert_eq!(
            config.console_verbosity,
            if config.mode == "custom-explicit-motion" {
                2
            } else {
                1
            }
        );
        assert!(!config.sdf_xml.is_empty());
        assert!(
            std::str::from_utf8(config.sdf_xml)
                .unwrap()
                .starts_with("<sdf ")
        );
        assert!(
            config
                .initial_body_rpy
                .map(f64::from_bits)
                .into_iter()
                .all(f64::is_finite)
        );
        assert!(
            config
                .sensor_rpy
                .map(f64::from_bits)
                .into_iter()
                .all(f64::is_finite)
        );
        let mut imu = sdformat::Imu {
            localization: config.localization.into(),
            custom_rpy: config.custom_rpy.map(f64::from_bits),
            custom_rpy_parent_frame: config.custom_parent.into(),
            enable_orientation: config.orientation_enabled,
            ..Default::default()
        };
        for i in 0..3 {
            imu.linear_acceleration[i] = noise(config.noise[i]);
            imu.angular_velocity[i] = noise(config.noise[i + 3]);
        }
        let frame = validate(&imu, false, config.sensor_name).unwrap();
        // Native records this as the initial WORLD sensor quaternion. Do not
        // add a MODEL/parent composition, reverse-engineer it from RPY, or use
        // the recorded reference_after in place of the implementation under test.
        assert_eq!(
            config.reference_before,
            config.initial_sensor_world_quaternion
        );
        let initial = DQuat::from_array(config.initial_sensor_world_quaternion.map(f64::from_bits));
        let selected = reference(frame, initial);
        if frame == Frame::Custom {
            assert_eq!(
                selected.to_array().map(f64::to_bits),
                config.initial_sensor_world_quaternion
            );
            assert_eq!(config.reference_after, config.reference_before);
        } else {
            close(selected.to_array(), config.reference_after, label);
        }
        let mut channel_noise = crate::noise::ImuNoise::new(&imu, [43; 32], 7).unwrap();
        assert_eq!(rows.len(), 6);
        for (index, row) in rows.iter().enumerate() {
            calls += 1;
            assert_eq!(row.index as usize, index);
            assert_eq!(
                row.input_row as usize,
                if config.mode.contains("-sign") {
                    [0, 0, 2, 2, 5, 5][index]
                } else {
                    index
                }
            );
            assert_eq!(
                row.negative_world_quaternion,
                config.mode.contains("-sign") && index % 2 == 1
            );
            assert_eq!(
                row.time_ns,
                [
                    1_000_000, 2_000_000, 5_000_000, 10_000_000, 20_000_000, 30_000_000
                ][index]
            );
            assert!(row.active && row.force && row.has_connections && row.updated);
            assert_eq!((row.next_before_ns, row.next_after_ns), (0, 0));
            // These are forced rate-zero native component observations, not
            // Scene activation/demand/paused scheduling or physics history.
            let pose = row.world_pose.map(f64::from_bits);
            assert!(pose.into_iter().all(f64::is_finite));
            for world_vector in [
                row.acceleration_world,
                row.angular_velocity_world,
                row.gravity_world,
            ] {
                assert!(
                    world_vector
                        .map(f64::from_bits)
                        .into_iter()
                        .all(f64::is_finite)
                );
            }
            let getter = row.orientation_getter.map(f64::from_bits);
            assert!(getter.into_iter().all(f64::is_finite));
            let mut reading = gz_sensors::imu(gz_sensors::ImuInput {
                world_orientation: [pose[3], pose[4], pose[5], pose[6]],
                reference_orientation: selected.to_array(),
                acceleration_sensor: row.acceleration_sensor_setter.map(f64::from_bits),
                angular_velocity_sensor: row.angular_velocity_sensor_setter.map(f64::from_bits),
                gravity_world: row.gravity_world.map(f64::from_bits),
            })
            .unwrap();
            if let Some(noise) = &mut channel_noise {
                noise
                    .apply(&mut reading, Duration::from_nanos(row.time_ns))
                    .unwrap();
            }
            match row.publication {
                native::Publication::Suppressed => {
                    suppressed += 1;
                    assert!(
                        !row.updated,
                        "suppressed component call cannot report an update"
                    );
                }
                native::Publication::Observed(output) => {
                    observed += 1;
                    assert_eq!(output.sequence, index.to_string());
                    assert_eq!(output.entity_name, config.sensor_name);
                    assert!((1..=8192).contains(&output.header_wire.len()));
                    assert!((1..=8192).contains(&output.wire.len()));
                    close(reading.angular_velocity, output.angular_velocity, label);
                    close(reading.specific_force, output.linear_acceleration, label);
                    if config.orientation_enabled {
                        let orientation = output
                            .orientation
                            .expect("enabled native orientation field");
                        assert_eq!(orientation, row.orientation_getter);
                        close(reading.orientation, orientation, label);
                    } else {
                        disabled += 1;
                        assert!(output.orientation.is_none());
                        // The native disabled getter holds its old value; Rust
                        // intentionally still computes orientation. No equality
                        // between that getter and this computed value is implied.
                        assert!(reading.orientation.into_iter().all(f64::is_finite));
                    }
                    assert_eq!(output.orientation_covariance.map(f32::from_bits), [0.0; 9]);
                    for (actual, covariance) in [
                        (
                            reading.angular_velocity_covariance,
                            output.angular_velocity_covariance,
                        ),
                        (
                            reading.linear_acceleration_covariance,
                            output.linear_acceleration_covariance,
                        ),
                    ] {
                        let covariance = covariance.map(f32::from_bits);
                        for (i, value) in covariance.into_iter().enumerate() {
                            assert!(value.is_finite());
                            if ![0, 4, 8].contains(&i) {
                                assert_eq!(value, 0.0);
                            }
                        }
                        assert_eq!(
                            actual,
                            [
                                covariance[0] as f64,
                                covariance[4] as f64,
                                covariance[8] as f64
                            ]
                        );
                    }
                }
            }
        }
    }
    assert_eq!(
        (
            modes.len(),
            configurations,
            calls,
            observed,
            suppressed,
            disabled
        ),
        (17, 17, 102, 102, 0, 24)
    );
}
