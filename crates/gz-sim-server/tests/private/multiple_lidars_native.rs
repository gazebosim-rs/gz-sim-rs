// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Captured scan values through the unchanged canonical packet path only.
//! LOCAL rays and supplied RayResults are a component-input ledger, not a
//! raycaster or native System/Physics/demand/schedule/lifecycle/RNG proof.
use super::packet;
use gz_sensors::lidar::PlanarLidarConfig;
use gz_sim_proto::{
    msgs::{Header, LaserScan, Pose, Time},
    prost::Message,
};
use gz_sim_scene::{LidarPublication, LidarSample, WorldPose};
use std::{collections::BTreeSet, time::Duration};

mod native {
    include!("../../../gz-sim-scene/tests/fixtures/m54_native_multiple_lidars.rs");
}

fn assert_finite_bits<const N: usize>(bits: [u64; N]) {
    assert!(bits.into_iter().map(f64::from_bits).all(f64::is_finite));
}

fn recorded_pose(bits: [u64; 7]) -> WorldPose {
    assert_finite_bits(bits);
    let values = bits.map(f64::from_bits);
    WorldPose {
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

fn range(bits: u64) -> f64 {
    let value = f64::from_bits(bits);
    assert!(value.is_finite() || bits == f64::INFINITY.to_bits());
    value
}

#[test]
fn actual_native_multiple_lidars_keep_all_30_scans_and_headers_byte_exact() {
    let expected = [
        (
            "pair", 2_usize, 8_usize, 6_usize, 24_usize, 13_usize, 11_usize,
        ),
        ("eight", 8, 1024, 24, 3072, 1706, 1366),
    ];
    let mut modes = BTreeSet::new();
    let mut endpoints = BTreeSet::new();
    let mut configurations = 0;
    let mut generated_rays = 0;
    let mut direct_calls = 0;
    let mut input_rows = 0;
    let mut scans = 0;
    let mut result_elements = 0;
    let mut finite_elements = 0;
    let mut miss_elements = 0;
    assert_eq!(native::CASES.len(), expected.len());
    for ((label, configs, inputs, summary), (mode, count, rays, rows, elements, finite, misses)) in
        native::CASES.iter().zip(expected)
    {
        assert_eq!(*label, mode);
        assert!(modes.insert(*label));
        assert_eq!(configs.len(), count);
        assert_eq!(inputs.len(), rows);
        assert_eq!(summary.configuration_count as usize, count);
        assert_eq!(summary.generated_ray_count as usize, rays);
        assert_eq!(summary.direct_update_calls as usize, rows);
        assert_eq!(summary.input_rows as usize, rows);
        assert_eq!(summary.observed_scans as usize, rows);
        assert_eq!(summary.canonical_packets as usize, rows);
        assert_eq!(summary.ray_result_elements as usize, elements);
        assert_eq!(summary.range_elements as usize, elements);
        assert_eq!(summary.finite_range_elements as usize, finite);
        assert_eq!(summary.positive_infinite_range_elements as usize, misses);
        assert!(!summary.original_system_executed && !summary.native_physics_executed);
        assert!(
            !summary.native_schedule_or_demand_equivalence_claimed
                && !summary.native_lifecycle_equivalence_claimed
                && !summary.global_rng_equivalence_claimed
                && !summary.pointcloud_subscribers_created
                && !summary.old_native_modes_replayed
        );
        configurations += count;
        direct_calls += summary.direct_update_calls as usize;
        let mut metadata = Vec::new();
        let mut native_ids = BTreeSet::new();
        let mut case_rays = 0;
        for (index, config) in configs.iter().enumerate() {
            assert_eq!(config.mode, mode);
            assert_eq!(config.sensor as usize, index);
            // These IDs belong to one native component process, not Rust IDs.
            assert!(native_ids.insert(config.native_id));
            assert_eq!(config.name, format!("laser{index}"));
            let namespace = config.topic.strip_prefix("/m54/native/").unwrap();
            let (packet_tag, suffix) = namespace.split_once('/').unwrap();
            assert!(!packet_tag.is_empty() && packet_tag.len() <= 128);
            assert!(
                packet_tag
                    .bytes()
                    .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-' || byte == b'_')
            );
            assert_eq!(suffix, format!("{mode}/laser{index}"));
            assert!(endpoints.insert(config.topic));
            assert_eq!(config.frame_id, format!("m54/native/laser{index}"));
            assert_eq!(config.console_verbosity, 2);
            let rate = f64::from_bits(config.rate);
            assert_eq!(rate, if index.is_multiple_of(2) { 100.0 } else { 50.0 });
            let beams = if mode == "pair" { [3, 5][index] } else { 128 };
            assert_eq!(config.horizontal_count as usize, beams);
            assert_eq!(config.vertical_count, 1);
            assert_finite_bits(config.angles);
            assert_finite_bits(config.range);
            assert_eq!(config.angles[2..], [0.0_f64.to_bits(); 2]);
            assert_eq!(config.range, [0.0_f64.to_bits(), 10.0_f64.to_bits()]);
            recorded_pose(config.initial_pose);
            assert_eq!(config.initial_pose[..3], [0.5, 0.0, 0.0].map(f64::to_bits));
            assert_eq!(config.rays.len(), beams);
            for (beam, ray) in config.rays.iter().enumerate() {
                assert_eq!(ray.beam as usize, beam);
                // Native GenerateRays endpoints stay LOCAL. No WORLD hit or
                // direction is reconstructed for this encoding-only control.
                assert_finite_bits(ray.start);
                assert_finite_bits(ray.end);
            }
            case_rays += config.rays.len();
            let xml = std::str::from_utf8(config.sdf_xml).unwrap();
            assert!(config.sdf_xml.len() <= 65536);
            assert_eq!(
                xml.matches(format!("<sensor name='{}' type='lidar'>", config.name).as_str())
                    .count(),
                1
            );
            assert_eq!(xml.matches("<ray>").count(), 1);
            assert!(xml.contains("<pose>0.5 0 0 0 0 "));
            assert!(xml.contains(&format!("<topic>{}</topic>", config.topic)));
            assert!(xml.contains(&format!("<frame_id>{}</frame_id>", config.frame_id)));
            assert!(xml.contains(&format!("<update_rate>{rate}</update_rate>")));
            assert!(xml.contains(&format!("<horizontal><samples>{beams}</samples>")));
            assert!(xml.contains("<vertical><samples>1</samples>"));
            assert!(xml.contains("<range><min>0</min><max>10</max>"));
            assert!(!xml.contains("<noise") && !xml.contains("<plugin"));
            let [angle_min, angle_max, _, _] = config.angles.map(f64::from_bits);
            let [range_min, range_max] = config.range.map(f64::from_bits);
            let clip = PlanarLidarConfig {
                samples: beams,
                angle_min,
                angle_max,
                range_min,
                range_max,
            };
            // Existing publication metadata relation only; captured output
            // ranges and poses below never come from a geometric estimator.
            metadata.push(LidarPublication {
                vertical: Default::default(),
                entity_name: format!("rig::body::{}", config.name),
                frame_id: config.frame_id.into(),
                topic: config.topic.into(),
                config: clip,
                angle_step: (angle_max - angle_min) / (beams - 1) as f64,
            });
        }
        assert_eq!(native_ids.len(), count);
        assert_eq!(case_rays, rays);
        generated_rays += case_rays;
        let mut owner_inputs = vec![0_usize; count];
        let mut sequences = vec![0_u64; count];
        let mut case_finite = 0;
        let mut case_misses = 0;
        let mut case_elements = 0;
        for (ordinal, input) in inputs.iter().enumerate() {
            let index = input.sensor as usize;
            assert_eq!(index, ordinal % count);
            assert_eq!(input.row as usize, ordinal / count);
            assert!(input.row < 3);
            assert_eq!(input.time_ns, 1_000_000 + 10_000_000 * u64::from(input.row));
            assert_eq!(input.getter_pose, input.pose_input);
            recorded_pose(input.pose_input);
            owner_inputs[index] += 1;
            input_rows += 1;
            let output = input.observed;
            let beams = metadata[index].config.samples;
            assert_eq!(output.sequence, sequences[index]);
            assert_eq!(output.sequence, u64::from(input.row));
            sequences[index] += 1;
            assert_eq!(output.world_pose, input.pose_input);
            assert_eq!(output.getter_pose_after_update, input.getter_pose);
            assert_eq!(input.ray_results.len(), beams);
            assert_eq!(input.ranges_before_update.len(), beams);
            assert_eq!(output.ranges_after_update.len(), beams);
            assert_eq!(output.ranges.len(), beams);
            assert_eq!(output.intensities.len(), beams);
            assert_eq!(input.ranges_before_update, output.ranges_after_update);
            assert_eq!(output.ranges_after_update, output.ranges);
            for (beam, result) in input.ray_results.iter().enumerate() {
                assert_eq!(result.beam as usize, beam);
                let fraction = range(result.fraction);
                assert!(fraction == f64::INFINITY || (0.0..=1.0).contains(&fraction));
                assert_finite_bits(result.point);
                assert_finite_bits(result.normal);
                assert_eq!(result.intensity, 0.0_f64.to_bits());
                assert_eq!(result.raw_range, output.ranges[beam]);
                let captured = range(result.raw_range);
                assert!(captured == f64::INFINITY || (0.0..=10.0).contains(&captured));
                assert_eq!(fraction == f64::INFINITY, captured == f64::INFINITY);
                assert_eq!(output.intensities[beam], 0.0_f64.to_bits());
                if captured == f64::INFINITY {
                    case_misses += 1;
                } else {
                    case_finite += 1;
                }
                case_elements += 1;
            }
            if input.row == 1 {
                // The all-miss row still has a complete actual scan packet.
                assert!(
                    output
                        .ranges
                        .iter()
                        .all(|bits| *bits == f64::INFINITY.to_bits())
                );
            } else if input.row == 2 {
                assert!(
                    output
                        .ranges
                        .iter()
                        .all(|bits| f64::from_bits(*bits).is_finite())
                );
            }
            let sample = LidarSample {
                time: Duration::from_nanos(input.time_ns),
                epoch: 0,
                world_pose: recorded_pose(output.world_pose),
                ranges: output.ranges.iter().copied().map(range).collect(),
            };
            let message = packet(&metadata[index], &sample, output.sequence)
                .expect("actual captured finite-or-positive-infinite scan is encodable");
            let header = message.header.as_ref().expect("canonical scan header");
            assert_eq!(
                header.encode_to_vec(),
                output.header_wire,
                "{mode}/{index}/{} Header",
                input.row
            );
            assert_eq!(
                message.encode_to_vec(),
                output.wire,
                "{mode}/{index}/{} LaserScan",
                input.row
            );
            assert_eq!(Header::decode(output.header_wire).unwrap(), *header);
            assert_eq!(
                header.stamp,
                Some(Time {
                    sec: 0,
                    nsec: input.time_ns as i32
                })
            );
            assert_eq!(header.data.len(), 2);
            assert_eq!(header.data[0].key, "frame_id");
            assert_eq!(header.data[0].value, [configs[index].frame_id]);
            assert_eq!(header.data[1].key, "seq");
            assert_eq!(header.data[1].value, [output.sequence.to_string()]);
            let decoded = LaserScan::decode(output.wire).unwrap();
            assert_eq!(decoded, message);
            assert_eq!(decoded.frame, configs[index].frame_id);
            assert_recorded_pose(decoded.world_pose.as_ref().unwrap(), output.world_pose);
            assert_eq!(
                [decoded.angle_min, decoded.angle_max].map(f64::to_bits),
                configs[index].angles[..2]
            );
            assert_eq!(
                decoded.angle_step.to_bits(),
                metadata[index].angle_step.to_bits()
            );
            assert_eq!(
                [decoded.range_min, decoded.range_max].map(f64::to_bits),
                configs[index].range
            );
            assert_eq!(decoded.count as usize, beams);
            assert_eq!(
                [decoded.vertical_angle_min, decoded.vertical_angle_max].map(f64::to_bits),
                configs[index].angles[2..]
            );
            assert_eq!(decoded.vertical_angle_step.to_bits(), 0.0_f64.to_bits());
            assert_eq!(decoded.vertical_count, configs[index].vertical_count);
            assert_eq!(
                decoded
                    .ranges
                    .iter()
                    .copied()
                    .map(f64::to_bits)
                    .collect::<Vec<_>>(),
                output.ranges
            );
            assert_eq!(
                decoded
                    .intensities
                    .iter()
                    .copied()
                    .map(f64::to_bits)
                    .collect::<Vec<_>>(),
                output.intensities
            );
            scans += 1;
        }
        assert!(owner_inputs.iter().all(|count| *count == 3));
        assert!(sequences.iter().all(|sequence| *sequence == 3));
        assert_eq!(
            (case_elements, case_finite, case_misses),
            (elements, finite, misses)
        );
        result_elements += case_elements;
        finite_elements += case_finite;
        miss_elements += case_misses;
    }
    assert_eq!((modes.len(), endpoints.len(), configurations), (2, 10, 10));
    assert_eq!(
        (generated_rays, direct_calls, input_rows, scans),
        (1032, 30, 30, 30)
    );
    assert_eq!(
        (result_elements, finite_elements, miss_elements),
        (3096, 1719, 1377)
    );
    // These30 standalone Header byte copies are embedded in the30 scans;
    // there are30 actual transport messages, not60.
}
