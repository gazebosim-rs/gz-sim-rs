// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Original-system fixture replay, attached after independent native and literal audits.
// The initial numeric acceptance target is exact bits, as in M39. Matching
// scalar expressions do not establish cross-language libm portability: any
// actual mismatch must be inspected before a separate contract decision.
use super::*;
use std::time::Duration;

fn metadata() -> GroundTruthOdometryPublication {
    GroundTruthOdometryPublication {
        dimensions: GroundTruthOdometryDimensions::Two,
        offsets: GroundTruthOdometryOffsets::default(),
        model_name: "rig".into(),
        odom_frame: "m46/odom".into(),
        robot_base_frame: "m46/base".into(),
        odom_topic: "/m46/odometry".into(),
        covariance_topic: "/m46/odometry_covariance".into(),
        tf_topic: "/m46/transform".into(),
    }
}

#[test]
fn actual_rotation_offset_native_inputs_match_scalar_and_dispatcher_bits() {
    mod reference {
        include!("../fixtures/m46_native_odometry_rotation_offsets.rs");
    }
    use reference::Publication;
    let cases = [
        "omitted_rotation_compat",
        "explicit_rotation_zero",
        "local_roll",
        "local_pitch",
        "local_yaw",
        "global_roll",
        "global_pitch",
        "global_yaw",
        "mixed_noncommuting",
        "mixed_translated",
        "boundary_sign",
        "rate",
        "micro",
        "pause_equal",
        "reload/0",
        "reload/1",
    ];
    let expected_modes: Vec<String> = ["two", "three"]
        .into_iter()
        .flat_map(|dimension| cases.map(|case| format!("{dimension}_{case}")))
        .collect();
    let modes: Vec<String> = reference::CASES
        .iter()
        .map(|(mode, _, _)| mode.to_string())
        .collect();
    assert_eq!(modes, expected_modes);
    let mut inputs = 0;
    let mut observed = 0;
    let mut suppressed = 0;
    let bits = |sample: GroundTruthOdometrySample| {
        (
            [
                sample.pose.translation[0].to_bits(),
                sample.pose.translation[1].to_bits(),
                sample.pose.translation[2].to_bits(),
                sample.pose.rotation[0].to_bits(),
                sample.pose.rotation[1].to_bits(),
                sample.pose.rotation[2].to_bits(),
                sample.pose.rotation[3].to_bits(),
            ],
            sample.linear.map(f64::to_bits),
            sample.angular.map(f64::to_bits),
        )
    };
    for (case_index, &(mode, configuration, ticks)) in reference::CASES.iter().enumerate() {
        let epoch = 7 + case_index as u64;
        let dimensions = match configuration.dimensions {
            2 => GroundTruthOdometryDimensions::Two,
            3 => GroundTruthOdometryDimensions::Three,
            _ => panic!("invalid native dimension"),
        };
        assert_eq!(
            mode.starts_with("two_"),
            dimensions == GroundTruthOdometryDimensions::Two
        );
        assert!(matches!(configuration.frequency, 0 | 50));
        assert_eq!(
            configuration.period_ns,
            if configuration.frequency == 50 {
                20_000_000
            } else {
                0
            }
        );
        for (present, requested) in [
            (
                configuration.requested_local_present,
                configuration.requested_local,
            ),
            (
                configuration.requested_global_present,
                configuration.requested_global,
            ),
            (
                configuration.requested_local_rpy_present,
                configuration.requested_local_rpy,
            ),
            (
                configuration.requested_global_rpy_present,
                configuration.requested_global_rpy,
            ),
        ] {
            if !present {
                assert_eq!(requested, [0; 3]);
            }
        }
        let requested = GroundTruthOdometryOffsets::from_translations(
            configuration.requested_local.map(f64::from_bits),
            configuration.requested_global.map(f64::from_bits),
        )
        .unwrap()
        .with_rotations(
            configuration.requested_local_rpy.map(f64::from_bits),
            configuration.requested_global_rpy.map(f64::from_bits),
        )
        .unwrap();
        let effective = GroundTruthOdometryOffsets::from_translations(
            configuration.effective_local.map(f64::from_bits),
            configuration.effective_global.map(f64::from_bits),
        )
        .unwrap()
        .with_rotations(
            configuration.effective_local_rpy.map(f64::from_bits),
            configuration.effective_global_rpy.map(f64::from_bits),
        )
        .unwrap();
        assert_eq!(requested, effective, "legacy zero normalization: {mode}");
        assert_eq!(
            effective.local_translation().map(f64::to_bits),
            configuration.effective_local
        );
        assert_eq!(
            effective.global_translation().map(f64::to_bits),
            configuration.effective_global
        );
        assert_eq!(
            effective.local_rotation_rpy().map(f64::to_bits),
            configuration.effective_local_rpy
        );
        assert_eq!(
            effective.global_rotation_rpy().map(f64::to_bits),
            configuration.effective_global_rpy
        );
        let mut publication = metadata();
        publication.dimensions = dimensions;
        publication.offsets = effective;
        publication.odom_frame = "m46/odom".into();
        publication.robot_base_frame = "m46/base".into();
        publication.odom_topic = "/m46/odometry".into();
        publication.covariance_topic = "/m46/odometry_covariance".into();
        publication.tf_topic = "/m46/transform".into();
        let mut world = World::new();
        world.insert_resource(StepFault::default());
        let entity = world.spawn((EntityKind::Model, publication.clone())).id();
        let owner = gz_sim_core::assign_id(&mut world, entity).unwrap();
        let mut scalar = Publisher::new(
            owner,
            publication,
            Duration::from_nanos(configuration.period_ns),
        );
        world.entity_mut(entity).insert(scalar.clone());
        let mut iteration = 0;
        for &(time_ns, dt_ns, paused, input_bits, expected) in ticks {
            inputs += 1;
            let input = input_bits.map(f64::from_bits);
            let model_pose = WorldPose {
                translation: [input[0], input[1], input[2]],
                rotation: [input[3], input[4], input[5], input[6]],
            };
            // The oracle records a MODEL Pose component. Native Util::worldPose
            // left-composes its identity parent's Pose before UpdateOdometry.
            // Both paths below receive WORLD input, as production requires.
            let pose = native_pose_mul(native_zero_offset(), model_pose);
            assert!(
                valid_pose(pose),
                "invalid actual reference input: {mode}/{time_ns}"
            );
            let sim_time = Duration::from_nanos(time_ns);
            let dt = Duration::from_nanos(dt_ns);
            let info = UpdateInfo {
                start_time: sim_time.checked_sub(dt).unwrap(),
                sim_time,
                dt,
                iteration,
                epoch,
                paused,
            };
            if !paused {
                iteration += 1;
            }
            // Publisher::advance's private contract is unpaused input. In parallel,
            // exercise the real production dispatcher on EVERY row, including both
            // paused calls: an accidental paused advance is not hidden by this test.
            let actual = if paused {
                None
            } else {
                scalar.advance(pose, info).unwrap()
            };
            let previous = world.get::<Publisher>(entity).unwrap().revision;
            let held = world.get::<GroundTruthOdometrySample>(entity).copied();
            world.entity_mut(entity).insert(pose);
            world.insert_resource(info);
            sample(&mut world);
            assert!(
                world.resource::<StepFault>().error().is_none(),
                "dispatch fault: {mode}/{time_ns}"
            );
            let state = world.get::<Publisher>(entity).unwrap();
            assert_eq!(
                state.previous_time, scalar.previous_time,
                "history clock: {mode}/{time_ns}"
            );
            assert_eq!(
                state.previous_pose.translation.map(f64::to_bits),
                scalar.previous_pose.translation.map(f64::to_bits)
            );
            assert_eq!(
                state.previous_pose.rotation.map(f64::to_bits),
                scalar.previous_pose.rotation.map(f64::to_bits)
            );
            assert_eq!(
                (state.last_publication, state.revision),
                (scalar.last_publication, scalar.revision)
            );
            for (dispatch, direct) in state.means.iter().zip(&scalar.means) {
                assert_eq!(dispatch.count, direct.count);
                assert_eq!(
                    dispatch.values.map(f64::to_bits),
                    direct.values.map(f64::to_bits)
                );
            }
            match (&state.spatial_means, &scalar.spatial_means) {
                (Some(dispatch), Some(direct)) => {
                    assert_eq!(dimensions, GroundTruthOdometryDimensions::Three);
                    for (dispatch, direct) in dispatch.iter().zip(direct) {
                        assert_eq!(dispatch.count, direct.count);
                        assert_eq!(
                            dispatch.values.map(f64::to_bits),
                            direct.values.map(f64::to_bits)
                        );
                    }
                }
                (None, None) => assert_eq!(dimensions, GroundTruthOdometryDimensions::Two),
                _ => panic!("direct/dispatch history shape differs: {mode}"),
            }
            let dispatched = (state.revision != previous)
                .then(|| *world.get::<GroundTruthOdometrySample>(entity).unwrap());
            assert_eq!(
                dispatched.map(bits),
                actual.map(bits),
                "direct/dispatch: {mode}/{time_ns}"
            );
            match expected {
                Publication::Suppressed => {
                    suppressed += 1;
                    assert!(
                        actual.is_none(),
                        "native suppressed publication: {mode}/{time_ns}"
                    );
                    assert_eq!(state.revision, previous);
                    assert_eq!(
                        world
                            .get::<GroundTruthOdometrySample>(entity)
                            .copied()
                            .map(bits),
                        held.map(bits)
                    );
                    assert_eq!(
                        world
                            .get::<GroundTruthOdometrySample>(entity)
                            .map(|s| (s.time, s.epoch, s.revision)),
                        held.map(|s| (s.time, s.epoch, s.revision))
                    );
                }
                Publication::Observed((pose_bits, linear_bits, angular_bits, _wire_bytes)) => {
                    observed += 1;
                    let actual = actual
                        .unwrap_or_else(|| panic!("missing native-time sample: {mode}/{time_ns}"));
                    assert_eq!((actual.time, actual.epoch), (sim_time, epoch));
                    assert_eq!(
                        bits(actual),
                        (pose_bits, linear_bits, angular_bits),
                        "actual native numeric bits: {mode}/{time_ns}"
                    );
                }
            }
        }
    }
    assert_eq!((inputs, observed, suppressed), (1020, 858, 162));
}
