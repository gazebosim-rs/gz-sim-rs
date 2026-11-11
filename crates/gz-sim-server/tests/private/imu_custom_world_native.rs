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
    include!("../../../gz-sim-scene/tests/fixtures/m45_native_imu_custom_world.rs");
}

#[test]
fn actual_native_custom_world_outputs_keep_all_84_packets_and_headers_byte_exact() {
    let expected_modes = [
        "world-zero-explicit",
        "world-zero-default",
        "world-signed-zero",
        "world-roll",
        "world-pitch",
        "world-yaw-default",
        "world-mixed",
        "world-mixed-sign-default",
        "world-yaw-pi",
        "world-yaw-negative-pi",
        "world-pitch-half-pi-default",
        "world-boundary-mixed-sign",
        "world-disabled-mixed",
        "world-mean-mixed-default",
    ];
    let custom_rpy: [[f64; 3]; 14] = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [-0.0, 0.0, -0.0],
        [0.4, 0.0, 0.0],
        [0.0, -0.3, 0.0],
        [0.0, 0.0, 0.6],
        [0.35, -0.25, 0.7],
        [0.35, -0.25, 0.7],
        [0.0, 0.0, std::f64::consts::PI],
        [0.0, 0.0, -std::f64::consts::PI],
        [0.0, std::f64::consts::FRAC_PI_2, 0.0],
        [std::f64::consts::PI, 0.2, -std::f64::consts::PI],
        [0.35, -0.25, 0.7],
        [0.35, -0.25, 0.7],
    ];
    let accelerations: [[f64; 3]; 6] = [
        [0.7, -1.1, 2.3],
        [-0.4, 0.8, -1.2],
        [1.5, 0.25, -0.75],
        [-1.3, 2.1, 0.6],
        [0.05, -0.2, 1.7],
        [2.2, -1.6, 0.9],
    ];
    let angular_velocities: [[f64; 3]; 6] = [
        [0.3, -0.5, 0.8],
        [-0.2, 0.4, -0.6],
        [0.7, 0.1, -0.3],
        [-0.8, 0.6, 0.2],
        [0.15, -0.25, 0.35],
        [0.4, -0.7, 0.9],
    ];
    let mut modes = BTreeSet::new();
    let mut topics = BTreeSet::new();
    let mut calls = 0;
    let mut observed = 0;
    let mut suppressed = 0;
    let mut enabled_packets = 0;
    let mut disabled_packets = 0;
    let mut default_localizations = 0;
    let mut signed_modes = 0;
    assert_eq!(native::CASES.len(), expected_modes.len());
    for (case_index, (label, config, inputs)) in native::CASES.iter().enumerate() {
        assert_eq!(*label, config.mode);
        assert_eq!(config.mode, expected_modes[case_index]);
        assert!(modes.insert(config.mode));
        assert!(topics.insert(config.topic), "fresh native subscriber topic");
        assert_eq!(config.instance, 0);
        assert_eq!(
            config.topic,
            format!("/m45/native/imu_custom_world/{}", config.mode)
        );
        assert_eq!(config.sensor_name, "imu");
        assert_eq!(config.frame_id, "m45/native/imu");
        assert_eq!(config.frequency, 0);
        assert_eq!(config.localization, "CUSTOM");
        let default_localization = config.mode.contains("default");
        default_localizations += usize::from(default_localization);
        assert_eq!(config.localization_tag_present, !default_localization);
        assert!(config.reference_tag_present && config.world_frame_call);
        assert_eq!(config.world_frame, "ENU");
        assert_eq!(config.world_heading, 0.0_f64.to_bits());
        assert_eq!(
            config.orientation_enabled,
            config.mode != "world-disabled-mixed"
        );
        assert_eq!(
            config.orientation_enabled_actual,
            config.orientation_enabled
        );
        assert_eq!(config.expected_warning, "NONE");
        assert_eq!(config.console_verbosity, 1);
        assert_eq!(config.custom_parent, "world");
        assert_eq!(config.custom_rpy, custom_rpy[case_index].map(f64::to_bits));
        assert_eq!(config.parsed_custom_rpy, config.custom_rpy);
        // Presence describes the captured XML, not a new runtime DTO field.
        let xml = std::str::from_utf8(config.sdf_xml).unwrap();
        assert_eq!(xml.matches("<orientation_reference_frame>").count(), 1);
        assert_eq!(xml.matches("<custom_rpy parent_frame='world'>").count(), 1);
        assert_eq!(
            xml.matches("<localization>CUSTOM</localization>").count(),
            usize::from(config.localization_tag_present)
        );
        assert_eq!(
            config.initial_body_rpy,
            [0.2_f64, -0.3, 0.6].map(f64::to_bits)
        );
        assert_eq!(config.sensor_rpy, [-0.4_f64, 0.1, 0.25].map(f64::to_bits));
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
        assert_ne!(config.reference_after, config.reference_before);
        for (axis, noise) in config.noise.into_iter().enumerate() {
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
            let means = [0.1_f64, -0.2, 0.3, -0.04, 0.05, -0.06];
            let mean = if config.mode.contains("-mean") {
                means[axis]
            } else {
                0.0
            };
            assert_eq!(noise.parameters[0], mean.to_bits());
        }
        let metadata = ImuPublication {
            enable_orientation: config.orientation_enabled,
            entity_name: config.sensor_name.into(),
            frame_id: config.frame_id.into(),
            topic: config.topic.into(),
        };
        let sign_mode = matches!(
            config.mode,
            "world-signed-zero"
                | "world-mixed-sign-default"
                | "world-yaw-negative-pi"
                | "world-boundary-mixed-sign"
        );
        signed_modes += usize::from(sign_mode);
        let mut last_positive_world: Option<[u64; 4]> = None;
        let mut disabled_getter = None;
        assert_eq!(inputs.len(), 6);
        for (index, input) in inputs.iter().enumerate() {
            calls += 1;
            assert_eq!(input.index, index as u32);
            assert_eq!(input.time_ns, [1, 2, 5, 10, 20, 30][index] * 1_000_000);
            assert_eq!(
                input.input_row,
                if sign_mode {
                    [0, 0, 2, 2, 5, 5][index]
                } else {
                    index as u32
                }
            );
            assert_eq!(input.negative_world_quaternion, sign_mode && index % 2 == 1);
            let world_rotation: [u64; 4] = input.world_pose[3..].try_into().unwrap();
            if input.negative_world_quaternion {
                assert_eq!(
                    world_rotation,
                    last_positive_world
                        .unwrap()
                        .map(|bits| bits ^ (1_u64 << 63))
                );
            } else {
                last_positive_world = Some(world_rotation);
            }
            let input_row = input.input_row as usize;
            assert_eq!(
                &input.world_pose[..3],
                &[2.0_f64, -1.0, 3.0].map(f64::to_bits)
            );
            assert_eq!(
                input.acceleration_world,
                accelerations[input_row].map(f64::to_bits)
            );
            assert_eq!(
                input.angular_velocity_world,
                angular_velocities[input_row].map(f64::to_bits)
            );
            assert_eq!(input.gravity_world, [0.0_f64, 0.0, -9.81].map(f64::to_bits));
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
                    if let Some(previous) = disabled_getter {
                        assert_eq!(input.orientation_getter, previous);
                    } else {
                        disabled_getter = Some(input.orientation_getter);
                    }
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
                epoch: 45 + case_index as u64,
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
    assert_eq!((topics.len(), calls, observed, suppressed), (14, 84, 84, 0));
    assert_eq!((enabled_packets, disabled_packets), (78, 6));
    assert_eq!((default_localizations, signed_modes), (5, 4));
    // These exact captured-value encoder comparisons neither evaluate the WORLD
    // handoff nor assert native/Rust numeric or random-stream byte equivalence.
}
