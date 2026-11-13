// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Direct component LOCAL-ray and supplied-range replay, separate from Rapier.
//! Every positive-infinite range is a present native miss in a received scan.
use gz_sensors::lidar::{PlanarLidar, PlanarLidarConfig};
use std::collections::BTreeSet;

mod native {
    include!("../fixtures/m54_native_multiple_lidars.rs");
}

fn finite<const N: usize>(bits: [u64; N]) -> [f64; N] {
    let values = bits.map(f64::from_bits);
    assert!(values.iter().all(|value| value.is_finite()));
    values
}

fn pose(bits: [u64; 7]) {
    let values = finite(bits);
    let norm = values[3..].iter().map(|value| value * value).sum::<f64>();
    assert!((norm - 1.0).abs() <= 1e-12);
}

#[test]
fn actual_native_component_pair_and_eight_match_all_local_rays_and_3096_present_ranges() {
    let (mut configurations, mut generated_rays, mut inputs, mut scans, mut ranges) =
        (0, 0, 0, 0, 0);
    let (mut finite_ranges, mut misses) = (0, 0);
    assert_eq!(native::CASES.len(), 2);
    for &(mode, configs, rows, summary) in native::CASES {
        let owners = match mode {
            "pair" => 2,
            "eight" => 8,
            _ => panic!("unexpected component mode"),
        };
        assert_eq!(configs.len(), owners);
        assert_eq!(rows.len(), 3 * owners);
        assert_eq!(summary.configuration_count as usize, owners);
        assert_eq!(summary.direct_update_calls as usize, rows.len());
        assert_eq!(summary.input_rows as usize, rows.len());
        assert_eq!(summary.observed_scans as usize, rows.len());
        assert_eq!(summary.canonical_packets as usize, rows.len());
        assert!(!summary.original_system_executed);
        assert!(!summary.native_physics_executed);
        assert!(!summary.native_schedule_or_demand_equivalence_claimed);
        assert!(!summary.native_lifecycle_equivalence_claimed);
        assert!(!summary.global_rng_equivalence_claimed);
        assert!(!summary.pointcloud_subscribers_created);
        assert!(!summary.old_native_modes_replayed);
        let mut ids = BTreeSet::new();
        let mut topics = BTreeSet::new();
        let mut models = Vec::new();
        let mut mode_generated = 0;
        for (owner, config) in configs.iter().enumerate() {
            configurations += 1;
            assert_eq!(config.mode, mode);
            assert_eq!(config.sensor as usize, owner);
            assert!(ids.insert(config.native_id));
            assert!(topics.insert(config.topic));
            assert_eq!(config.name, format!("laser{owner}"));
            // The exclusive native packet name is a genuine topic namespace.
            assert!(config.topic.starts_with("/m54/native/"));
            assert!(
                config
                    .topic
                    .ends_with(format!("/{mode}/laser{owner}").as_str())
            );
            assert_eq!(config.frame_id, format!("m54/native/laser{owner}"));
            assert_eq!(config.console_verbosity, 2);
            assert_eq!(
                f64::from_bits(config.rate),
                if owner.is_multiple_of(2) { 100.0 } else { 50.0 }
            );
            assert_eq!(config.vertical_count, 1);
            let beams = if mode == "pair" { [3, 5][owner] } else { 128 };
            assert_eq!(config.horizontal_count as usize, beams);
            assert_eq!(config.rays.len(), beams);
            let angles = finite(config.angles);
            assert_eq!(
                angles,
                [
                    -std::f64::consts::FRAC_PI_4,
                    std::f64::consts::FRAC_PI_4,
                    0.0,
                    0.0
                ]
            );
            let limits = finite(config.range);
            assert_eq!(limits, [0.0, 10.0]);
            pose(config.initial_pose);
            let xml = std::str::from_utf8(config.sdf_xml).unwrap();
            assert!(xml.contains(format!("<sensor name='{}' type='lidar'>", config.name).as_str()));
            assert!(xml.contains(format!("<topic>{}</topic>", config.topic).as_str()));
            assert!(xml.contains(format!("<frame_id>{}</frame_id>", config.frame_id).as_str()));
            assert!(!xml.contains("<noise"));
            let parameters = PlanarLidarConfig {
                samples: beams,
                angle_min: angles[0],
                angle_max: angles[1],
                range_min: limits[0],
                range_max: limits[1],
            };
            let model = PlanarLidar::new(parameters).unwrap();
            assert_eq!(*model.config(), parameters);
            // Retained angle-step precision is distinct from LOCAL endpoints.
            assert!(
                (model.angle_step() - (angles[1] - angles[0]) / (beams - 1) as f64).abs() <= 1e-15
            );
            assert_eq!(model.directions().len(), beams);
            for (beam, (ray, direction)) in config.rays.iter().zip(model.directions()).enumerate() {
                generated_rays += 1;
                mode_generated += 1;
                assert_eq!(ray.beam as usize, beam);
                let start = finite(ray.start);
                let end = finite(ray.end);
                for ((start, end), direction) in start.into_iter().zip(end).zip(direction) {
                    // Native computes its final azimuth arithmetically; Rust
                    // pins its final beam to angle_max. Keep the old bound.
                    assert!((start - direction * limits[0]).abs() <= 1e-12);
                    assert!((end - direction * limits[1]).abs() <= 1e-12);
                }
                let norm = direction.iter().map(|value| value * value).sum::<f64>();
                assert!((norm - 1.0).abs() <= 1e-12);
            }
            models.push(model);
        }
        assert_eq!(summary.generated_ray_count as usize, mode_generated);
        let mut keys = BTreeSet::new();
        let (mut mode_ranges, mut mode_finite, mut mode_misses) = (0, 0, 0);
        for (ordinal, input) in rows.iter().enumerate() {
            inputs += 1;
            let owner = input.sensor as usize;
            assert_eq!(owner, ordinal % owners);
            assert_eq!(input.row as usize, ordinal / owners);
            assert_eq!(input.time_ns, 1_000_000 + 10_000_000 * u64::from(input.row));
            assert!(keys.insert((input.sensor, input.row, input.time_ns)));
            let config = &configs[owner];
            let model = &models[owner];
            let captured = input.observed;
            assert_eq!(captured.sequence, u64::from(input.row));
            pose(input.pose_input);
            pose(input.getter_pose);
            pose(captured.world_pose);
            pose(captured.getter_pose_after_update);
            // These are actual setter/getter/scan cards, not a pose estimator.
            assert_eq!(input.pose_input, input.getter_pose);
            assert_eq!(input.getter_pose, captured.world_pose);
            assert_eq!(input.getter_pose, captured.getter_pose_after_update);
            let beams = model.directions().len();
            assert_eq!(input.ray_results.len(), beams);
            assert_eq!(input.ranges_before_update.len(), beams);
            assert_eq!(captured.ranges_after_update.len(), beams);
            assert_eq!(captured.ranges.len(), beams);
            assert_eq!(captured.intensities.len(), beams);
            for (beam, result) in input.ray_results.iter().enumerate() {
                mode_ranges += 1;
                ranges += 1;
                assert_eq!(result.beam as usize, beam);
                finite(result.point);
                assert_eq!(finite(result.normal), [0.0, 0.0, 1.0]);
                assert_eq!(f64::from_bits(result.intensity), 0.0);
                assert_eq!(result.intensity, captured.intensities[beam]);
                assert_eq!(result.raw_range, input.ranges_before_update[beam]);
                assert_eq!(result.raw_range, captured.ranges_after_update[beam]);
                assert_eq!(result.raw_range, captured.ranges[beam]);
                let supplied = f64::from_bits(result.raw_range);
                let fraction = f64::from_bits(result.fraction);
                let expected = f64::from_bits(captured.ranges[beam]);
                let actual = if supplied == f64::INFINITY {
                    assert_eq!(result.raw_range, f64::INFINITY.to_bits());
                    assert_eq!(result.fraction, f64::INFINITY.to_bits());
                    assert_eq!(expected.to_bits(), f64::INFINITY.to_bits());
                    mode_misses += 1;
                    misses += 1;
                    // None is this scalar API's explicit no-hit argument; the
                    // captured scan and its positive-infinite range are present.
                    model.range(None).unwrap()
                } else {
                    assert!(supplied.is_finite() && (0.0..=10.0).contains(&supplied));
                    assert!(fraction.is_finite() && (0.0..=1.0).contains(&fraction));
                    assert_eq!(fraction, supplied / 10.0);
                    mode_finite += 1;
                    finite_ranges += 1;
                    model.range(Some(supplied)).unwrap()
                };
                if expected.is_finite() {
                    assert!(actual.is_finite() && (actual - expected).abs() <= 1e-12);
                } else {
                    assert_eq!(actual.to_bits(), f64::INFINITY.to_bits());
                }
            }
            if input.row == 1 {
                assert!(
                    captured
                        .ranges
                        .iter()
                        .all(|bits| *bits == f64::INFINITY.to_bits())
                );
            }
            // Exact encoding belongs to the separate Server proof; preserve
            // the actual Header/full-scan evidence without reencoding it here.
            assert!(!captured.header_wire.is_empty() && !captured.wire.is_empty());
            assert!(
                captured
                    .wire
                    .windows(captured.header_wire.len())
                    .any(|part| part == captured.header_wire)
            );
            assert!(
                captured
                    .header_wire
                    .windows(config.frame_id.len())
                    .any(|part| part == config.frame_id.as_bytes())
            );
            scans += 1;
        }
        assert_eq!(keys.len(), 3 * owners);
        assert_eq!(summary.ray_result_elements as usize, mode_ranges);
        assert_eq!(summary.range_elements as usize, mode_ranges);
        assert_eq!(summary.finite_range_elements as usize, mode_finite);
        assert_eq!(
            summary.positive_infinite_range_elements as usize,
            mode_misses
        );
        assert_eq!(mode_finite + mode_misses, 3 * mode_generated);
    }
    assert_eq!(
        (configurations, generated_rays, inputs, scans, ranges),
        (10, 1032, 30, 30, 3096)
    );
    assert_eq!((finite_ranges, misses), (1719, 1377));
}
