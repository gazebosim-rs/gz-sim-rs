// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Captured native final-frame values passed through the current canonical encoder.
//! Native observations and literal fixture were independently audited before insertion.
use super::*;
use gz_sim_proto::prost::Message;

#[test]
fn actual_rotated_native_values_encode_all_2574_canonical_packets() {
    mod reference {
        include!("../../../gz-sim-scene/tests/fixtures/m46_native_odometry_rotation_offsets.rs");
    }
    use gz_sim_scene::GroundTruthOdometryOffsets as Offsets;
    let mut inputs = 0;
    let mut acquisitions = 0;
    let mut suppressed = 0;
    let mut canonical_packets = 0;
    assert_eq!(reference::CASES.len(), 32);
    for &(mode, config, ticks) in reference::CASES {
        let dimensions = match config.dimensions {
            2 => Dimensions::Two,
            3 => Dimensions::Three,
            other => panic!("invalid actual native dimensions: {mode}/{other}"),
        };
        assert!(matches!(config.frequency, 0 | 50));
        assert_eq!(
            config.period_ns,
            if config.frequency == 50 {
                20_000_000
            } else {
                0
            }
        );
        if !config.requested_local_present {
            assert_eq!(config.requested_local, [0; 3]);
        }
        if !config.requested_global_present {
            assert_eq!(config.requested_global, [0; 3]);
        }
        let offsets = Offsets::from_translations(
            config.requested_local.map(f64::from_bits),
            config.requested_global.map(f64::from_bits),
        )
        .unwrap()
        .with_rotations(
            config.requested_local_rpy.map(f64::from_bits),
            config.requested_global_rpy.map(f64::from_bits),
        )
        .unwrap();
        for (present, requested) in [
            (
                config.requested_local_rpy_present,
                config.requested_local_rpy,
            ),
            (
                config.requested_global_rpy_present,
                config.requested_global_rpy,
            ),
        ] {
            if !present {
                assert_eq!(requested, [0; 3]);
            }
        }
        assert_eq!(
            offsets.local_rotation_rpy().map(f64::to_bits),
            config.effective_local_rpy
        );
        assert_eq!(
            offsets.global_rotation_rpy().map(f64::to_bits),
            config.effective_global_rpy
        );
        assert_eq!(
            offsets.local_translation().map(f64::to_bits),
            config.effective_local
        );
        assert_eq!(
            offsets.global_translation().map(f64::to_bits),
            config.effective_global
        );
        let metadata = Metadata {
            dimensions,
            offsets,
            gaussian_noise: Default::default(),
            model_name: "rig".into(),
            odom_frame: "m46/odom".into(),
            robot_base_frame: "m46/base".into(),
            odom_topic: "/m46/odometry".into(),
            covariance_topic: "/m46/odometry_covariance".into(),
            tf_topic: "/m46/transform".into(),
        };
        // Native generations have their own time/history. Rust revision and
        // epoch are bookkeeping, not invented native message-header fields.
        let mut revision = 0;
        for &(time_ns, _, _, _, expected) in ticks {
            inputs += 1;
            let reference::Publication::Observed((pose_bits, linear_bits, angular_bits, wire)) =
                expected
            else {
                suppressed += 1;
                continue;
            };
            revision += 1;
            let pose = pose_bits.map(f64::from_bits);
            let sample = Sample {
                time: Duration::from_nanos(time_ns),
                epoch: 7,
                revision,
                pose: gz_sim_scene::WorldPose {
                    translation: [pose[0], pose[1], pose[2]],
                    rotation: [pose[3], pose[4], pose[5], pose[6]],
                },
                linear: linear_bits.map(f64::from_bits),
                angular: angular_bits.map(f64::from_bits),
            };
            let (odometry, covariance, transform) =
                packets(&metadata, 1, &sample).unwrap_or_else(|| {
                    panic!("actual rotated native values rejected: {mode}/{time_ns}")
                });
            // Output bits are already in the final odometry frame. Neither
            // MODEL->WORLD nor the offsets are applied again by the encoder.
            let encoded = [
                odometry.encode_to_vec(),
                covariance.encode_to_vec(),
                transform.encode_to_vec(),
            ];
            for (channel, (actual, expected)) in encoded.iter().zip(wire).enumerate() {
                assert_eq!(
                    actual.as_slice(),
                    expected,
                    "actual rotated native canonical bytes: {mode}/{time_ns}/channel{channel}"
                );
                canonical_packets += 1;
            }
            // Full-wire equality above includes each original header. Also
            // compare the independently decoded standalone header bytes.
            let expected_odometry = Odometry::decode(wire[0]).unwrap();
            let expected_covariance = OdometryWithCovariance::decode(wire[1]).unwrap();
            let expected_transform = PoseV::decode(wire[2]).unwrap();
            assert_eq!(
                odometry.header.as_ref().unwrap().encode_to_vec(),
                expected_odometry.header.unwrap().encode_to_vec()
            );
            assert_eq!(
                covariance.header.as_ref().unwrap().encode_to_vec(),
                expected_covariance.header.unwrap().encode_to_vec()
            );
            assert_eq!(
                transform.pose[0].header.as_ref().unwrap().encode_to_vec(),
                expected_transform.pose[0]
                    .header
                    .as_ref()
                    .unwrap()
                    .encode_to_vec()
            );
            acquisitions += 1;
        }
    }
    assert_eq!(
        (inputs, acquisitions, suppressed, canonical_packets),
        (1020, 858, 162, 2574)
    );
}
