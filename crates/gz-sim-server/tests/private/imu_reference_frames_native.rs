// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Captured-value encoder replay, separate from native/Rust numeric comparison.
use super::imu_packet;
use gz_sensors::ImuReading;
use gz_sim_proto::{
    msgs::{Header, Imu},
    prost::Message,
};
use gz_sim_scene::{ImuPublication, ImuSample};
use std::{collections::BTreeSet, time::Duration};

// The shared audited schema retains Suppressed; this forced capture has none.
#[allow(dead_code)]
mod native {
    include!("../../../gz-sim-scene/tests/fixtures/m43_native_imu_reference_frames.rs");
}

#[test]
fn actual_native_imu_reference_outputs_keep_all_102_packets_and_headers_byte_exact() {
    let expected_modes = [
        "custom-motion",
        "enu-motion",
        "ned-motion",
        "nwu-motion",
        "custom-sign",
        "enu-sign",
        "ned-sign",
        "nwu-sign",
        "custom-disabled",
        "enu-disabled",
        "ned-disabled",
        "nwu-disabled",
        "custom-mean",
        "enu-mean",
        "ned-mean",
        "nwu-mean",
        "custom-explicit-motion",
    ];
    let mut modes = BTreeSet::new();
    let mut topics = BTreeSet::new();
    let mut calls = 0;
    let mut observed = 0;
    let mut suppressed = 0;
    let mut enabled_packets = 0;
    let mut disabled_packets = 0;
    assert_eq!(native::CASES.len(), expected_modes.len());
    for (case_index, (label, config, inputs)) in native::CASES.iter().enumerate() {
        assert_eq!(*label, config.mode);
        assert_eq!(config.mode, expected_modes[case_index]);
        assert!(modes.insert(config.mode));
        assert!(topics.insert(config.topic), "fresh native subscriber topic");
        assert_eq!(config.instance, 0);
        assert_eq!(
            config.topic,
            format!("/m43/native/imu_reference/{}", config.mode)
        );
        assert_eq!(config.sensor_name, "imu");
        assert_eq!(config.frame_id, "m43/native/imu");
        assert_eq!(config.frequency, 0);
        let localization = config.mode.split('-').next().unwrap().to_ascii_uppercase();
        assert_eq!(config.localization, localization);
        let explicit = config.mode == "custom-explicit-motion";
        assert_eq!(
            config.reference_tag_present,
            localization != "CUSTOM" || explicit
        );
        assert_eq!(config.world_frame_call, config.reference_tag_present);
        assert_eq!(
            config.orientation_enabled,
            !config.mode.contains("-disabled")
        );
        assert_eq!(
            config.orientation_enabled_actual,
            config.orientation_enabled
        );
        assert_eq!(
            config.expected_warning,
            if explicit {
                "CUSTOM_EMPTY_PARENT"
            } else {
                "NONE"
            }
        );
        assert_eq!(config.console_verbosity, if explicit { 2 } else { 1 });
        assert_eq!(config.custom_parent, "");
        assert_eq!(config.custom_rpy, [0; 3]);
        assert!(!std::str::from_utf8(config.sdf_xml).unwrap().is_empty());
        assert!(
            config
                .initial_body_rpy
                .into_iter()
                .chain(config.sensor_rpy)
                .chain(config.initial_sensor_world_quaternion)
                .chain(config.reference_before)
                .chain(config.reference_after)
                .map(f64::from_bits)
                .all(f64::is_finite)
        );
        assert_eq!(
            config.reference_before,
            config.initial_sensor_world_quaternion
        );
        if localization == "CUSTOM" {
            assert_eq!(config.reference_after, config.reference_before);
        }
        for noise in config.noise {
            assert_eq!(
                noise.kind,
                if config.mode.contains("-mean") {
                    "gaussian"
                } else {
                    "none"
                }
            );
            assert!(
                noise
                    .parameters
                    .into_iter()
                    .map(f64::from_bits)
                    .all(f64::is_finite)
            );
            assert!(noise.parameters[1..].iter().all(|bits| *bits == 0));
        }
        let metadata = ImuPublication {
            enable_orientation: config.orientation_enabled,
            entity_name: config.sensor_name.into(),
            frame_id: config.frame_id.into(),
            topic: config.topic.into(),
        };
        assert_eq!(inputs.len(), 6);
        for (index, input) in inputs.iter().enumerate() {
            calls += 1;
            assert_eq!(input.index, index as u32);
            assert_eq!(input.time_ns, [1, 2, 5, 10, 20, 30][index] * 1_000_000);
            let sign_mode = config.mode.contains("-sign");
            assert_eq!(
                input.input_row,
                if sign_mode {
                    [0, 0, 2, 2, 5, 5][index]
                } else {
                    index as u32
                }
            );
            assert_eq!(input.negative_world_quaternion, sign_mode && index % 2 == 1);
            assert!(input.active && input.force && input.has_connections && input.updated);
            assert_eq!((input.next_before_ns, input.next_after_ns), (0, 0));
            assert!(
                input
                    .world_pose
                    .into_iter()
                    .chain(input.acceleration_world)
                    .chain(input.angular_velocity_world)
                    .chain(input.gravity_world)
                    .chain(input.acceleration_sensor_setter)
                    .chain(input.angular_velocity_sensor_setter)
                    .chain(input.orientation_getter)
                    .map(f64::from_bits)
                    .all(f64::is_finite)
            );
            let output = match input.publication {
                native::Publication::Suppressed => {
                    suppressed += 1;
                    // No reading is invented; the final zero-suppression and
                    // exact observation-count assertions reject this case.
                    continue;
                }
                native::Publication::Observed(output) => output,
            };
            observed += 1;
            let sequence: u64 = output.sequence.parse().unwrap();
            assert_eq!(output.sequence, sequence.to_string());
            assert_eq!(sequence, index as u64);
            assert_eq!(output.entity_name, config.sensor_name);
            assert_eq!(output.orientation.is_some(), config.orientation_enabled);
            let orientation = match output.orientation {
                Some(bits) => {
                    enabled_packets += 1;
                    assert_eq!(bits, input.orientation_getter);
                    bits
                }
                None => {
                    disabled_packets += 1;
                    // The required typed slot uses a captured getter only. It
                    // is not a wire orientation or a Rust scalar expectation.
                    input.orientation_getter
                }
            };
            let covariance = |bits: [u32; 9]| {
                assert!(
                    bits.into_iter()
                        .map(f32::from_bits)
                        .all(|value| value.is_finite() && value == 0.0)
                );
                [bits[0], bits[4], bits[8]].map(|value| f64::from(f32::from_bits(value)))
            };
            let reading = ImuReading {
                orientation: orientation.map(f64::from_bits),
                specific_force: output.linear_acceleration.map(f64::from_bits),
                angular_velocity: output.angular_velocity.map(f64::from_bits),
                angular_velocity_covariance: covariance(output.angular_velocity_covariance),
                linear_acceleration_covariance: covariance(output.linear_acceleration_covariance),
            };
            covariance(output.orientation_covariance);
            assert!(
                reading
                    .orientation
                    .into_iter()
                    .chain(reading.specific_force)
                    .chain(reading.angular_velocity)
                    .all(f64::is_finite)
            );
            let sample = ImuSample {
                time: Duration::from_nanos(input.time_ns),
                // Epoch is local bookkeeping; the native message has no epoch.
                epoch: 43 + case_index as u64,
                reading: Ok(reading),
            };
            let message = imu_packet(&metadata, &sample, sequence)
                .expect("actual finite native reading is encodable");
            assert_eq!(message.entity_name, output.entity_name);
            assert_eq!(message.orientation.is_some(), config.orientation_enabled);
            if let Some(bits) = output.orientation {
                let actual = message.orientation.as_ref().unwrap();
                assert_eq!(
                    [actual.x, actual.y, actual.z, actual.w].map(f64::to_bits),
                    bits
                );
            }
            let angular = message.angular_velocity.as_ref().unwrap();
            assert_eq!(
                [angular.x, angular.y, angular.z].map(f64::to_bits),
                output.angular_velocity
            );
            let acceleration = message.linear_acceleration.as_ref().unwrap();
            assert_eq!(
                [acceleration.x, acceleration.y, acceleration.z].map(f64::to_bits),
                output.linear_acceleration
            );
            for (actual, expected) in [
                (
                    message.orientation_covariance.as_ref(),
                    output.orientation_covariance,
                ),
                (
                    message.angular_velocity_covariance.as_ref(),
                    output.angular_velocity_covariance,
                ),
                (
                    message.linear_acceleration_covariance.as_ref(),
                    output.linear_acceleration_covariance,
                ),
            ] {
                assert_eq!(
                    actual
                        .unwrap()
                        .data
                        .iter()
                        .map(|value| value.to_bits())
                        .collect::<Vec<_>>(),
                    expected
                );
            }
            assert_eq!(
                message.encode_to_vec().as_slice(),
                output.wire,
                "{label} call {index}"
            );
            let header = message.header.as_ref().unwrap();
            assert_eq!(
                header.encode_to_vec().as_slice(),
                output.header_wire,
                "{label} header {index}"
            );
            assert_eq!(Header::decode(output.header_wire).unwrap(), *header);
            assert_eq!(Imu::decode(output.wire).unwrap(), message);
        }
    }
    assert_eq!(modes, expected_modes.into_iter().collect());
    assert_eq!(
        (topics.len(), calls, observed, suppressed),
        (17, 102, 102, 0)
    );
    assert_eq!((enabled_packets, disabled_packets), (78, 24));
    // These exact captured-value encoder comparisons neither evaluate the WORLD
    // handoff nor assert native/Rust numeric or random-stream byte equivalence.
}
