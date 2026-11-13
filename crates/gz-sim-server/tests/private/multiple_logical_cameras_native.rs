// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Captured-value encoding only: all paired outputs pass through unchanged packets.
//! Explicit WORLD poses/model maps are a component-input ledger, not an output
//! estimator or a loaded ECM/topology/System/demand/schedule/lifecycle proof.
use super::packets;
use gz_sensors::logical_camera::{
    LogicalCameraConfig, LogicalCameraModel, LogicalCameraPose, LogicalCameraReading,
};
use gz_sim_proto::{
    msgs::{Header, LogicalCameraImage, LogicalCameraSensor, Pose},
    prost::Message,
};
use gz_sim_scene::{LogicalCameraPublication, LogicalCameraSample};
use std::{collections::BTreeSet, time::Duration};

mod native {
    include!("../../../gz-sim-scene/tests/fixtures/m53_native_multiple_logical_cameras.rs");
}

fn recorded_pose(bits: [u64; 7]) -> LogicalCameraPose {
    let values = bits.map(f64::from_bits);
    assert!(values.into_iter().all(f64::is_finite));
    assert!((values[3..].iter().map(|value| value * value).sum::<f64>() - 1.0).abs() <= 1e-12);
    LogicalCameraPose {
        translation: [values[0], values[1], values[2]],
        rotation: [values[3], values[4], values[5], values[6]],
    }
}
fn assert_recorded_pose(actual: &Pose, bits: [u64; 7]) {
    assert!(actual.header.is_none() && actual.name.is_empty() && actual.id == 0);
    let position = actual
        .position
        .as_ref()
        .expect("captured complete position");
    let rotation = actual
        .orientation
        .as_ref()
        .expect("captured complete orientation");
    assert!(position.header.is_none() && rotation.header.is_none());
    assert_eq!(
        [
            position.x, position.y, position.z, rotation.x, rotation.y, rotation.z, rotation.w
        ]
        .map(f64::to_bits),
        bits
    );
}
fn model_cards(models: &[native::NativeModel]) -> Vec<(&str, [u64; 7])> {
    models
        .iter()
        .map(|model| (model.name, model.pose))
        .collect()
}

#[test]
fn actual_native_multiple_logical_cameras_keep_all_80_messages_and_headers_byte_exact() {
    let expected = [("pair", 2_usize, 8_usize), ("eight", 8, 32)];
    let mut modes = BTreeSet::new();
    let mut endpoints = BTreeSet::new();
    let mut configurations = 0;
    let mut input_rows = 0;
    let mut paired_outputs = 0;
    let mut canonical_messages = 0;
    let mut direct_calls = 0;
    let mut empty_maps = 0;
    assert_eq!(native::CASES.len(), expected.len());
    for ((label, configs, inputs, summary), (mode, count, rows)) in
        native::CASES.iter().zip(expected)
    {
        assert_eq!(*label, mode);
        assert!(modes.insert(*label));
        assert_eq!(configs.len(), count);
        assert_eq!(inputs.len(), rows);
        assert_eq!(summary.configuration_count as usize, count);
        assert_eq!(summary.direct_update_calls as usize, rows);
        assert_eq!(summary.input_rows as usize, rows);
        assert_eq!(summary.observed_pairs as usize, rows);
        assert_eq!(summary.canonical_packets as usize, 2 * rows);
        assert!(!summary.original_system_executed && !summary.native_physics_executed);
        assert!(
            !summary.native_lifecycle_equivalence_claimed && !summary.old_native_modes_replayed
        );
        configurations += count;
        direct_calls += summary.direct_update_calls as usize;
        let mut metadata = Vec::new();
        let mut native_ids = BTreeSet::new();
        let mut frusta = BTreeSet::new();
        for (index, config) in configs.iter().enumerate() {
            assert_eq!(config.mode, mode);
            assert_eq!(config.sensor as usize, index);
            // Native factory IDs belong only to their own component process;
            // they are not Rust public IDs or shared across these two modes.
            assert!(native_ids.insert(config.native_id));
            assert_eq!(config.name, format!("camera{index}"));
            assert_eq!(config.topic, format!("/m53/native/{mode}/camera{index}"));
            assert_eq!(config.frustum_topic, format!("{}/frustum", config.topic));
            assert_eq!(config.frame_id, format!("m53/native/camera{index}"));
            assert!(endpoints.insert(config.topic) && endpoints.insert(config.frustum_topic));
            assert_eq!(config.console_verbosity, 2);
            let rate = f64::from_bits(config.rate);
            assert_eq!(rate, if index.is_multiple_of(2) { 100.0 } else { 50.0 });
            assert!(frusta.insert(config.config));
            assert_eq!(
                config.config,
                [
                    0.5 + 0.05 * index as f64,
                    8.0 + index as f64,
                    1.2 + 0.02 * index as f64,
                    1.0 + 0.1 * index as f64
                ]
                .map(f64::to_bits)
            );
            let [near, far, horizontal_fov, aspect_ratio] = config.config.map(f64::from_bits);
            let clip = LogicalCameraConfig {
                near,
                far,
                horizontal_fov,
                aspect_ratio,
            };
            assert!(clip.validate().is_ok());
            recorded_pose(config.sensor_local_pose);
            assert_eq!(
                config.sensor_local_pose[..3],
                [-1.0, 0.0, 0.0].map(f64::to_bits)
            );
            let xml = std::str::from_utf8(config.sdf_xml).unwrap();
            assert!(config.sdf_xml.len() <= 65536);
            assert_eq!(
                xml.matches(
                    format!("<sensor name='{}' type='logical_camera'>", config.name).as_str()
                )
                .count(),
                1
            );
            assert_eq!(xml.matches("<logical_camera>").count(), 1);
            assert!(xml.contains("<pose>-1 0 0 0 0 "));
            assert!(xml.contains(&format!("<topic>{}</topic>", config.topic)));
            assert!(xml.contains(&format!("<frame_id>{}</frame_id>", config.frame_id)));
            assert!(xml.contains(&format!("<update_rate>{rate}</update_rate>")));
            for field in ["near", "far", "horizontal_fov", "aspect_ratio"] {
                assert_eq!(xml.matches(format!("<{field}>").as_str()).count(), 1);
                assert_eq!(xml.matches(format!("</{field}>").as_str()).count(), 1);
            }
            assert!(!xml.contains("<noise") && !xml.contains("<plugin"));
            metadata.push(LogicalCameraPublication {
                entity_name: format!("rig::body::{}", config.name),
                frame_id: config.frame_id.into(),
                topic: config.topic.into(),
                frustum_topic: config.frustum_topic.into(),
                config: clip,
            });
        }
        assert_eq!(native_ids.len(), count);
        assert_eq!(frusta.len(), count);
        let mut sequences = vec![0_u64; count];
        let mut owner_inputs = vec![0_usize; count];
        let mut case_pairs = 0;
        for (ordinal, input) in inputs.iter().enumerate() {
            let index = input.sensor as usize;
            assert_eq!(index, ordinal % count);
            assert_eq!(input.row as usize, ordinal / count);
            assert!(input.row < 4);
            assert_eq!(input.time_ns, 1_000_000 + 10_000_000 * u64::from(input.row));
            recorded_pose(input.world_pose);
            owner_inputs[index] += 1;
            input_rows += 1;
            let row_start = ordinal - index;
            assert_eq!(
                model_cards(input.models),
                model_cards(inputs[row_start].models)
            );
            let mut model_names = BTreeSet::new();
            for model in input.models {
                assert!(model_names.insert(model.name));
                assert!(!model.name.is_empty() && !model.name.contains("::"));
                recorded_pose(model.pose);
            }
            assert!(
                input
                    .models
                    .windows(2)
                    .all(|pair| pair[0].name < pair[1].name)
            );
            let output = input.observed;
            assert_eq!(output.sequence, sequences[index]);
            assert_eq!(output.sequence, u64::from(input.row));
            sequences[index] += 1;
            assert_eq!(output.camera_pose, input.world_pose);
            if input.row == 2 {
                // This explicit empty component map does not describe a loaded
                // world without its camera-owner model. No Scene claim follows.
                assert!(input.models.is_empty() && output.models.is_empty());
                empty_maps += 1;
            } else {
                let mut expected_names = (0..count).map(|i| format!("rig{i}")).collect::<Vec<_>>();
                expected_names.extend(["behind".into(), "target".into()]);
                expected_names.sort();
                assert_eq!(
                    input.models.iter().map(|m| m.name).collect::<Vec<_>>(),
                    expected_names
                        .iter()
                        .map(String::as_str)
                        .collect::<Vec<_>>()
                );
            }
            if input.row == 0 {
                assert!(
                    output
                        .models
                        .iter()
                        .any(|model| model.name == format!("rig{index}"))
                );
            }
            assert!(
                output
                    .models
                    .windows(2)
                    .all(|pair| pair[0].name < pair[1].name)
            );
            assert!(
                output
                    .models
                    .iter()
                    .all(|model| model_names.contains(model.name))
            );
            // Copy ACTUAL output poses. Do not transform the input map to form
            // these expected encoder values or recompute frustum visibility.
            let sample = LogicalCameraSample {
                time: Duration::from_nanos(input.time_ns),
                epoch: 0,
                reading: Ok(LogicalCameraReading {
                    pose: recorded_pose(output.camera_pose),
                    models: output
                        .models
                        .iter()
                        .map(|model| LogicalCameraModel {
                            name: model.name.into(),
                            pose: recorded_pose(model.pose),
                        })
                        .collect(),
                }),
            };
            let (image, frustum) = packets(&metadata[index], &sample, output.sequence)
                .expect("actual finite captured paired reading is encodable");
            let image_header = image.header.as_ref().expect("canonical image header");
            let frustum_header = frustum.header.as_ref().expect("canonical frustum header");
            assert_eq!(
                image_header.encode_to_vec(),
                output.image_header_wire,
                "{mode}/{index}/{} image header",
                input.row
            );
            assert_eq!(
                frustum_header.encode_to_vec(),
                output.frustum_header_wire,
                "{mode}/{index}/{} frustum header",
                input.row
            );
            assert_eq!(
                image.encode_to_vec(),
                output.image_wire,
                "{mode}/{index}/{} image wire",
                input.row
            );
            assert_eq!(
                frustum.encode_to_vec(),
                output.frustum_wire,
                "{mode}/{index}/{} frustum wire",
                input.row
            );
            assert_eq!(
                Header::decode(output.image_header_wire).unwrap(),
                *image_header
            );
            assert_eq!(
                Header::decode(output.frustum_header_wire).unwrap(),
                *frustum_header
            );
            assert_eq!(image_header.stamp, frustum_header.stamp);
            assert_eq!(image_header.data.len(), 2);
            assert_eq!(frustum_header.data.len(), 1);
            assert_eq!(image_header.data[0], frustum_header.data[0]);
            assert_eq!(image_header.data[0].key, "frame_id");
            assert_eq!(image_header.data[0].value, [configs[index].frame_id]);
            assert_eq!(image_header.data[1].key, "seq");
            assert_eq!(image_header.data[1].value, [output.sequence.to_string()]);
            let decoded_image = LogicalCameraImage::decode(output.image_wire).unwrap();
            let decoded_frustum = LogicalCameraSensor::decode(output.frustum_wire).unwrap();
            assert_eq!(decoded_image, image);
            assert_eq!(decoded_frustum, frustum);
            assert_recorded_pose(decoded_image.pose.as_ref().unwrap(), output.camera_pose);
            assert_eq!(decoded_image.model.len(), output.models.len());
            for (model, actual) in decoded_image.model.iter().zip(output.models) {
                assert_eq!(model.name, actual.name);
                assert_recorded_pose(model.pose.as_ref().unwrap(), actual.pose);
            }
            assert_eq!(
                [
                    decoded_frustum.near_clip,
                    decoded_frustum.far_clip,
                    decoded_frustum.horizontal_fov,
                    decoded_frustum.aspect_ratio
                ]
                .map(f64::to_bits),
                configs[index].config
            );
            case_pairs += 1;
            paired_outputs += 1;
            canonical_messages += 2;
        }
        assert_eq!(case_pairs, rows);
        assert!(owner_inputs.iter().all(|count| *count == 4));
        assert!(sequences.iter().all(|sequence| *sequence == 4));
    }
    assert_eq!((modes.len(), endpoints.len(), configurations), (2, 20, 10));
    assert_eq!(
        (direct_calls, input_rows, paired_outputs, canonical_messages),
        (40, 40, 40, 80)
    );
    assert_eq!(empty_maps, 10);
}
