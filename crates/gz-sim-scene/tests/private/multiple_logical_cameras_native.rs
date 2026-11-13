// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Direct component input/value comparison, separate from loaded-world lifecycle.
//! Empty model maps are SensorFactory API inputs, not a loaded ownerless topology.
use gz_sensors::logical_camera::{
    LogicalCameraConfig, LogicalCameraModel, LogicalCameraPose, logical_camera, valid_pose,
};
use std::collections::{BTreeMap, BTreeSet};

mod native {
    include!("../fixtures/m53_native_multiple_logical_cameras.rs");
}

fn pose(bits: &[u64; 7]) -> LogicalCameraPose {
    let values = (*bits).map(f64::from_bits);
    let result = LogicalCameraPose {
        translation: [values[0], values[1], values[2]],
        rotation: [values[3], values[4], values[5], values[6]],
    };
    assert!(valid_pose(result));
    result
}

fn assert_native_pose(actual: LogicalCameraPose, expected: &[u64; 7], context: &str) {
    for (axis, (value, bits)) in actual
        .translation
        .into_iter()
        .chain(actual.rotation)
        .zip(expected.iter().copied())
        .enumerate()
    {
        let native = f64::from_bits(bits);
        assert!(
            value.is_finite() && native.is_finite() && (value - native).abs() <= 1e-12,
            "{context} component {axis}: {value:?} != {native:?}"
        );
    }
}

#[test]
fn actual_native_component_pair_and_eight_match_all_40_numeric_pairs_and_inputs() {
    let (mut configuration_count, mut input_count, mut pair_count, mut wire_count) = (0, 0, 0, 0);
    assert_eq!(native::CASES.len(), 2);
    for &(mode, configs, rows, summary) in native::CASES {
        let expected_count = match mode {
            "pair" => 2,
            "eight" => 8,
            _ => panic!("unexpected native component mode"),
        };
        assert_eq!(configs.len(), expected_count);
        assert_eq!(summary.configuration_count as usize, configs.len());
        assert_eq!(summary.direct_update_calls as usize, 4 * configs.len());
        assert_eq!(summary.input_rows as usize, rows.len());
        assert_eq!(summary.observed_pairs as usize, rows.len());
        assert_eq!(summary.canonical_packets as usize, 2 * rows.len());
        assert!(!summary.original_system_executed);
        assert!(!summary.native_physics_executed);
        assert!(!summary.native_lifecycle_equivalence_claimed);
        assert!(!summary.old_native_modes_replayed);
        let mut identities = BTreeSet::new();
        let mut endpoints = BTreeSet::new();
        for (index, config) in configs.iter().enumerate() {
            configuration_count += 1;
            assert_eq!(config.mode, mode);
            assert_eq!(config.sensor as usize, index);
            assert!(identities.insert(config.native_id));
            assert_eq!(config.name, format!("camera{index}"));
            assert_eq!(config.topic, format!("/m53/native/{mode}/camera{index}"));
            assert_eq!(config.frustum_topic, format!("{}/frustum", config.topic));
            assert_eq!(config.frame_id, format!("m53/native/camera{index}"));
            assert!(endpoints.insert(config.topic));
            assert!(endpoints.insert(config.frustum_topic));
            // This is recorded getter metadata; the native calls bypass scheduling.
            assert_eq!(
                f64::from_bits(config.rate),
                if index.is_multiple_of(2) { 100.0 } else { 50.0 }
            );
            assert_eq!(config.console_verbosity, 2);
            assert!(valid_pose(pose(&config.sensor_local_pose)));
            let xml = std::str::from_utf8(config.sdf_xml).unwrap();
            assert!(xml.contains(
                format!("<sensor name='{}' type='logical_camera'>", config.name).as_str()
            ));
            assert!(xml.contains(format!("<topic>{}</topic>", config.topic).as_str()));
            assert!(xml.contains(format!("<frame_id>{}</frame_id>", config.frame_id).as_str()));
            assert!(xml.contains("<logical_camera>") && !xml.contains("<noise"));
            let values = config.config.map(f64::from_bits);
            LogicalCameraConfig {
                near: values[0],
                far: values[1],
                horizontal_fov: values[2],
                aspect_ratio: values[3],
            }
            .validate()
            .unwrap();
        }
        assert_eq!(endpoints.len(), 2 * configs.len());
        let mut keys = BTreeSet::new();
        let mut views: BTreeMap<u32, BTreeSet<Vec<String>>> = BTreeMap::new();
        let mut empty_pairs = 0;
        for (ordinal, input) in rows.iter().enumerate() {
            input_count += 1;
            assert_eq!(input.sensor as usize, ordinal % configs.len());
            assert_eq!(input.row as usize, ordinal / configs.len());
            assert_eq!(input.time_ns, 1_000_000 + 10_000_000 * u64::from(input.row));
            assert!(keys.insert((input.sensor, input.row, input.time_ns)));
            let config = &configs[input.sensor as usize];
            let values = config.config.map(f64::from_bits);
            let parameters = LogicalCameraConfig {
                near: values[0],
                far: values[1],
                horizontal_fov: values[2],
                aspect_ratio: values[3],
            };
            let models = input
                .models
                .iter()
                .map(|model| LogicalCameraModel {
                    name: model.name.to_owned(),
                    pose: pose(&model.pose),
                })
                .collect::<Vec<_>>();
            assert!(models.len() <= 128);
            assert!(models.windows(2).all(|pair| pair[0].name < pair[1].name));
            let reading = logical_camera(parameters, pose(&input.world_pose), &models).unwrap();
            let captured = input.observed;
            assert_eq!(captured.sequence, u64::from(input.row));
            let context = format!("{mode} sensor {} row {}", input.sensor, input.row);
            assert_native_pose(reading.pose, &captured.camera_pose, &context);
            assert_eq!(
                reading
                    .models
                    .iter()
                    .map(|model| model.name.as_str())
                    .collect::<Vec<_>>(),
                captured
                    .models
                    .iter()
                    .map(|model| model.name)
                    .collect::<Vec<_>>()
            );
            for (actual, expected) in reading.models.iter().zip(captured.models) {
                assert_native_pose(
                    actual.pose,
                    &expected.pose,
                    &format!("{context} model {}", actual.name),
                );
            }
            if input.row == 0 {
                assert!(
                    reading
                        .models
                        .iter()
                        .any(|model| model.name == format!("rig{}", input.sensor))
                );
            }
            if input.row == 2 {
                assert!(
                    models.is_empty() && reading.models.is_empty() && captured.models.is_empty()
                );
                empty_pairs += 1;
            }
            views.entry(input.row).or_default().insert(
                reading
                    .models
                    .iter()
                    .map(|model| model.name.clone())
                    .collect(),
            );
            // Preserve actual paired-byte evidence here; Server independently
            // exercises the encoder and compares both full packets byte-for-byte.
            for (header, wire) in [
                (captured.image_header_wire, captured.image_wire),
                (captured.frustum_header_wire, captured.frustum_wire),
            ] {
                assert!(!header.is_empty() && !wire.is_empty());
                assert!(wire.windows(header.len()).any(|part| part == header));
                assert!(
                    header
                        .windows(config.frame_id.len())
                        .any(|part| part == config.frame_id.as_bytes())
                );
                wire_count += 1;
            }
            pair_count += 1;
        }
        assert_eq!(keys.len(), 4 * configs.len());
        assert_eq!(empty_pairs, configs.len());
        assert!(views.get(&0).unwrap().len() > 1);
    }
    assert_eq!(
        (configuration_count, input_count, pair_count, wire_count),
        (10, 40, 40, 80)
    );
}
