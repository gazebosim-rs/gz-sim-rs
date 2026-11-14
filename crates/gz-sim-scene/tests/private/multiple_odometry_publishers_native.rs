// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay captured WORLD getbacks through the existing owner-local sampler.
//! The real force/joint tests and Server's full packet checks remain separate.
use super::*;
use std::collections::{BTreeMap, BTreeSet};

mod native {
    include!("../fixtures/m56_native_multiple_odometry_publishers.rs");
}

fn pose(words: [u64; 7]) -> WorldPose {
    let values = words.map(f64::from_bits);
    let pose = WorldPose {
        translation: [values[0], values[1], values[2]],
        rotation: [values[3], values[4], values[5], values[6]],
    };
    assert!(valid_pose(pose));
    pose
}

fn validate_entity(entity: &native::NativeEntityInput, configured: &native::NativeEntityInput) {
    assert_eq!(entity.native_entity, configured.native_entity);
    assert_ne!(entity.native_entity, 0);
    assert_ne!(entity.native_entity, entity.world_entity);
    assert_eq!(entity.world_entity, configured.world_entity);
    assert_eq!(entity.parent_component, entity.world_entity);
    assert_eq!(entity.parent_graph, entity.world_entity);
    assert_eq!(entity.input_local_pose_bits, entity.getter_local_pose_bits);
    assert_eq!(
        entity.world_local_pose_bits,
        configured.world_local_pose_bits
    );
    for words in [
        entity.input_local_pose_bits,
        entity.getter_local_pose_bits,
        entity.world_local_pose_bits,
        entity.getter_world_pose_bits,
    ] {
        pose(words);
    }
}

fn configuration(
    config: &native::NativeConfiguration,
    mode: &str,
    owner: usize,
) -> GroundTruthOdometryPublication {
    assert_eq!(
        (config.schema_version, config.mode, config.owner as usize),
        (1, mode, owner)
    );
    assert_eq!(config.world_name, "reference_world");
    assert_eq!(config.name, format!("rig{owner}"));
    assert_eq!(config.console_verbosity, 2);
    validate_entity(&config.entity, &config.entity);
    let text = std::str::from_utf8(config.sdf_xml).unwrap();
    let document = roxmltree::Document::parse(text).unwrap();
    let plugin = document.root_element();
    assert_eq!(plugin.tag_name().name(), "plugin");
    assert_eq!(
        plugin.attribute("filename"),
        Some("direct-original-object-not-loader-input")
    );
    assert_eq!(
        plugin.attribute("name"),
        Some("gz::sim::systems::OdometryPublisher")
    );
    assert_eq!(plugin.attributes().len(), 2);
    let mut settings = BTreeMap::new();
    for child in plugin.children().filter(|node| node.is_element()) {
        assert_eq!(child.attributes().len(), 0);
        assert!(!child.children().any(|node| node.is_element()));
        assert!(
            settings
                .insert(child.tag_name().name(), child.text().unwrap().trim())
                .is_none()
        );
    }
    assert_eq!(settings.len(), 12);
    assert_eq!(
        settings["dimensions"].parse::<u32>().unwrap(),
        config.dimensions
    );
    assert_eq!(
        settings["odom_publish_frequency"]
            .parse::<f64>()
            .unwrap()
            .to_bits(),
        config.frequency_bits
    );
    assert_eq!(
        settings["gaussian_noise"].parse::<f64>().unwrap().to_bits(),
        config.gaussian_noise_bits
    );
    assert_eq!(config.gaussian_noise_bits, 0);
    assert_eq!(settings["odom_frame"], config.odom_frame);
    assert_eq!(settings["robot_base_frame"], config.robot_base_frame);
    for (key, topic) in ["odom_topic", "odom_covariance_topic", "tf_topic"]
        .into_iter()
        .zip(config.topics)
    {
        assert_eq!(settings[key], topic);
        assert!(
            topic.starts_with("/m56/native/")
                && topic.contains(format!("/{mode}/owner{owner}/").as_str())
        );
    }
    let vector = |key: &str| -> [f64; 3] {
        let values = settings[key]
            .split_whitespace()
            .map(|value| value.parse::<f64>().unwrap())
            .collect::<Vec<_>>();
        assert!(values.iter().all(|value| value.is_finite()));
        values.try_into().unwrap()
    };
    let offsets = GroundTruthOdometryOffsets::from_translations(
        vector("xyz_offset"),
        vector("xyz_global_offset"),
    )
    .unwrap()
    .with_rotations(vector("rpy_offset"), vector("rpy_global_offset"))
    .unwrap();
    assert_eq!(
        offsets.local_translation().map(f64::to_bits),
        pose(config.offset_pose_bits).translation.map(f64::to_bits)
    );
    assert_eq!(
        offsets.global_translation().map(f64::to_bits),
        pose(config.global_offset_pose_bits)
            .translation
            .map(f64::to_bits)
    );
    GroundTruthOdometryPublication {
        dimensions: match config.dimensions {
            2 => GroundTruthOdometryDimensions::Two,
            3 => GroundTruthOdometryDimensions::Three,
            _ => panic!("unreviewed native dimension"),
        },
        offsets,
        gaussian_noise: GroundTruthOdometryNoise::from_stddev(f64::from_bits(
            config.gaussian_noise_bits,
        ))
        .unwrap(),
        model_name: config.name.into(),
        odom_frame: config.odom_frame.into(),
        robot_base_frame: config.robot_base_frame.into(),
        odom_topic: config.topics[0].into(),
        covariance_topic: config.topics[1].into(),
        tf_topic: config.topics[2].into(),
    }
}

#[derive(Default)]
struct Fields {
    headers: usize,
    header_values: usize,
    wire_bytes: usize,
    poses: usize,
    covariance_words: usize,
}

fn header(value: Option<&native::NativeHeader>, time_ns: i64, fields: &mut Fields) {
    if let Some(value) = value {
        fields.headers += 1;
        assert!(value.wire.len() <= 4096);
        fields.wire_bytes += value.wire.len();
        if let Some((seconds, nanos)) = value.stamp {
            assert!((0..1_000_000_000).contains(&nanos));
            assert_eq!(
                i128::from(seconds) * 1_000_000_000 + i128::from(nanos),
                i128::from(time_ns)
            );
        }
        let mut keys = BTreeSet::new();
        for item in value.data {
            assert!(!item.key.is_empty() && keys.insert(item.key));
            assert!(item.key.len() <= 4096);
            for bytes in item.values {
                assert!(bytes.len() <= 4096);
                fields.header_values += 1;
            }
        }
    }
}

fn frame_header(value: Option<&native::NativeHeader>, config: &native::NativeConfiguration) {
    let value = value.expect("captured channel frame header");
    assert!(value.stamp.is_some());
    assert_eq!(value.data.len(), 2);
    for (item, key, frame) in [
        (
            &value.data[0],
            b"frame_id".as_slice(),
            config.odom_frame.as_bytes(),
        ),
        (
            &value.data[1],
            b"child_frame_id".as_slice(),
            config.robot_base_frame.as_bytes(),
        ),
    ] {
        assert_eq!(item.key, key);
        assert_eq!(item.values, [frame]);
    }
}

fn vector(value: &native::NativeVector3, time: i64, fields: &mut Fields) -> [u64; 3] {
    header(value.header, time, fields);
    assert!(
        value
            .bits
            .map(f64::from_bits)
            .iter()
            .all(|value| value.is_finite())
    );
    assert!(value.wire.len() <= 65536);
    fields.wire_bytes += value.wire.len();
    value.bits
}

fn captured_pose(value: &native::NativePose, time: i64, fields: &mut Fields) -> [u64; 7] {
    header(value.header, time, fields);
    assert!(value.name.is_empty());
    // This is the protobuf's u32 optional ID, not the captured u64 ECM owner.
    assert_eq!(value.id, 0);
    let p = vector(value.position, time, fields);
    let q = value.orientation;
    header(q.header, time, fields);
    assert!(q.wire.len() <= 65536 && value.wire.len() <= 65536);
    fields.wire_bytes += q.wire.len() + value.wire.len();
    let words = [p[0], p[1], p[2], q.bits[0], q.bits[1], q.bits[2], q.bits[3]];
    pose(words);
    fields.poses += 1;
    words
}

fn twist(value: &native::NativeTwist, time: i64, fields: &mut Fields) -> ([u64; 3], [u64; 3]) {
    header(value.header, time, fields);
    assert!(value.wire.len() <= 65536);
    fields.wire_bytes += value.wire.len();
    (
        vector(value.linear, time, fields),
        vector(value.angular, time, fields),
    )
}

fn covariance(value: &native::NativeFloatV, time: i64, fields: &mut Fields) {
    header(value.header, time, fields);
    // The actual reference configuration requested sigma0. Keep all36 f32 words.
    assert_eq!(value.f32_bits, [0; 36]);
    assert!(value.wire.len() <= 65536);
    fields.covariance_words += value.f32_bits.len();
    fields.wire_bytes += value.wire.len();
}

type SampleBits = ([u64; 7], [u64; 3], [u64; 3], Duration, u64, u64);
fn sample_bits(value: GroundTruthOdometrySample) -> SampleBits {
    (
        [
            value.pose.translation[0].to_bits(),
            value.pose.translation[1].to_bits(),
            value.pose.translation[2].to_bits(),
            value.pose.rotation[0].to_bits(),
            value.pose.rotation[1].to_bits(),
            value.pose.rotation[2].to_bits(),
            value.pose.rotation[3].to_bits(),
        ],
        value.linear.map(f64::to_bits),
        value.angular.map(f64::to_bits),
        value.time,
        value.epoch,
        value.revision,
    )
}

fn row_order(case: &native::NativeCase) {
    let mut seen = BTreeMap::from([
        ("configuration", vec![false; case.configurations.len()]),
        ("input", vec![false; case.inputs.len()]),
        ("observation", vec![false; case.observations.len()]),
        ("fence", vec![false; case.fences.len()]),
        ("summary", vec![false]),
    ]);
    for row in case.raw_order {
        let slot = &mut seen.get_mut(row.kind).expect("closed raw row kind")
            [usize::try_from(row.index).unwrap()];
        assert!(!*slot, "duplicate raw row locator");
        *slot = true;
    }
    assert!(seen.values().flatten().all(|seen| *seen));
    assert_eq!(case.raw_order.len(), case.summary.counts.raw_rows as usize);
}

#[test]
fn actual_native_multiple_odometry_inputs_match_all_90_samples_and_270_numeric_packets() {
    let (pair, eight) = native::native_cases();
    let (mut total_inputs, mut total_samples, mut total_packets, mut total_configs) = (0, 0, 0, 0);
    let mut fields = Fields::default();
    for (case, owners) in [(pair, 2), (eight, 8)] {
        assert_eq!(case.mode, if owners == 2 { "pair" } else { "eight" });
        let summary = case.summary;
        let counts = summary.counts;
        assert_eq!(summary.mode, case.mode);
        assert!(summary.original_object_linked_system_scope);
        assert!(
            !summary.original_server_or_plugin_loader_executed
                && !summary.native_physics_executed
                && !summary.native_demand_reset_removal_or_lifecycle_equivalence_claimed
                && !summary.global_rng_equivalence_claimed
                && !summary.old_native_modes_replayed
        );
        assert_eq!(case.configurations.len(), owners);
        assert_eq!(counts.configuration_count as usize, owners);
        assert_eq!(counts.global_call_groups, 13);
        assert_eq!(counts.input_rows as usize, case.inputs.len());
        assert_eq!(counts.fence_rows as usize, case.fences.len());
        assert_eq!(counts.input_rows, counts.direct_pre_update_calls);
        assert_eq!(counts.input_rows, counts.direct_post_update_calls);
        assert_eq!(
            counts.public_interface_invocations,
            counts.direct_pre_update_calls + counts.direct_post_update_calls
        );
        assert_eq!(counts.canonical_packets as usize, case.observations.len());
        assert_eq!(
            counts.canonical_packets,
            3 * counts.observed_channel_triples
        );
        assert_eq!(counts.pose_vector_records, counts.observed_channel_triples);
        row_order(case);
        let mut world = World::new();
        world.insert_resource(StepFault::default());
        let mut entities = Vec::new();
        let mut identities = BTreeSet::new();
        let mut topics = BTreeSet::new();
        for (owner, config) in case.configurations.iter().enumerate() {
            let publication = configuration(config, case.mode, owner);
            assert!(identities.insert(config.entity.native_entity));
            for topic in config.topics {
                assert!(topics.insert(topic));
            }
            let entity = world
                .spawn((
                    EntityKind::Model,
                    publication.clone(),
                    pose(config.entity.getter_world_pose_bits),
                ))
                .id();
            let id = gz_sim_core::assign_id(&mut world, entity).unwrap();
            world.entity_mut(entity).insert(Publisher::new(
                id,
                publication,
                period(f64::from_bits(config.frequency_bits)).unwrap(),
            ));
            entities.push(entity);
            total_configs += 1;
        }
        let mut callbacks = vec![0u32; owners];
        let (mut mode_samples, mut mode_packets, mut mode_poses, mut mode_words) = (0, 0, 0, 0);
        for call in 0..u64::from(counts.global_call_groups) {
            let inputs = case
                .inputs
                .iter()
                .filter(|row| row.call_index == call)
                .collect::<Vec<_>>();
            assert_eq!(inputs.len(), owners);
            let first = inputs[0];
            let info = UpdateInfo {
                start_time: Duration::from_nanos(
                    u64::try_from(first.time_ns.checked_sub(first.dt_ns).unwrap()).unwrap(),
                ),
                sim_time: Duration::from_nanos(u64::try_from(first.time_ns).unwrap()),
                dt: Duration::from_nanos(u64::try_from(first.dt_ns).unwrap()),
                iteration: first.iterations,
                epoch: 56,
                paused: first.paused,
            };
            let before = entities
                .iter()
                .map(|entity| {
                    world
                        .get::<GroundTruthOdometrySample>(*entity)
                        .copied()
                        .map(sample_bits)
                })
                .collect::<Vec<_>>();
            // Publish all captured inputs before this single family dispatch.
            for (owner, input) in inputs.into_iter().enumerate() {
                assert_eq!((input.mode, input.owner as usize), (case.mode, owner));
                assert_eq!(
                    (
                        input.time_ns,
                        input.real_time_ns,
                        input.dt_ns,
                        input.iterations,
                        input.paused
                    ),
                    (
                        first.time_ns,
                        first.real_time_ns,
                        first.dt_ns,
                        first.iterations,
                        first.paused
                    )
                );
                assert_eq!(input.real_time_ns, input.time_ns);
                validate_entity(&input.entity, &case.configurations[owner].entity);
                world
                    .entity_mut(entities[owner])
                    .insert(pose(input.entity.getter_world_pose_bits));
                total_inputs += 1;
            }
            world.insert_resource(info);
            sample(&mut world);
            assert!(world.resource::<StepFault>().error().is_none());
            for (owner, entity) in entities.iter().enumerate() {
                let output = case
                    .observations
                    .iter()
                    .filter(|row| row.call_index == call && row.owner as usize == owner)
                    .collect::<Vec<_>>();
                let fences = case
                    .fences
                    .iter()
                    .filter(|row| row.call_index == call && row.owner as usize == owner)
                    .collect::<Vec<_>>();
                assert_eq!(fences.len(), 1);
                let fence = fences[0];
                assert_eq!((fence.mode, fence.time_ns), (case.mode, first.time_ns));
                assert_eq!(fence.callback_count as usize, output.len());
                let callback_start = callbacks[owner];
                callbacks[owner] += fence.callback_count;
                assert_eq!(fence.cumulative_callbacks, callbacks[owner]);
                if output.is_empty() {
                    assert_eq!(
                        world
                            .get::<GroundTruthOdometrySample>(*entity)
                            .copied()
                            .map(sample_bits),
                        before[owner]
                    );
                    continue;
                }
                assert_eq!(output.len(), 3);
                let actual = *world
                    .get::<GroundTruthOdometrySample>(*entity)
                    .expect("captured publication missing");
                assert_eq!((actual.time, actual.epoch), (info.sim_time, info.epoch));
                let actual_bits = sample_bits(actual);
                let mut channels = BTreeSet::new();
                for (arrival, observation) in output.into_iter().enumerate() {
                    assert_eq!(
                        (observation.mode, observation.time_ns),
                        (case.mode, first.time_ns)
                    );
                    assert_eq!(
                        observation.arrival_ordinal,
                        u64::from(callback_start) + arrival as u64
                    );
                    assert!(channels.insert(observation.channel));
                    let poses_before = fields.poses;
                    let words_before = fields.covariance_words;
                    let (expected_pose, expected_twist, wire) = match observation.message {
                        native::NativeMessage::Odometry {
                            header: h,
                            pose: p,
                            twist: t,
                            wire,
                        } => {
                            assert_eq!(observation.channel, "odometry");
                            frame_header(*h, &case.configurations[owner]);
                            header(*h, observation.time_ns, &mut fields);
                            (
                                captured_pose(p, observation.time_ns, &mut fields),
                                Some(twist(t, observation.time_ns, &mut fields)),
                                *wire,
                            )
                        }
                        native::NativeMessage::OdometryWithCovariance {
                            header: h,
                            pose: p,
                            pose_covariance,
                            twist: t,
                            twist_covariance,
                            pose_with_covariance_wire,
                            twist_with_covariance_wire,
                            wire,
                        } => {
                            assert_eq!(observation.channel, "odometry_with_covariance");
                            frame_header(*h, &case.configurations[owner]);
                            header(*h, observation.time_ns, &mut fields);
                            covariance(pose_covariance, observation.time_ns, &mut fields);
                            covariance(twist_covariance, observation.time_ns, &mut fields);
                            assert!(
                                pose_with_covariance_wire.len() <= 65536
                                    && twist_with_covariance_wire.len() <= 65536
                            );
                            fields.wire_bytes +=
                                pose_with_covariance_wire.len() + twist_with_covariance_wire.len();
                            (
                                captured_pose(p, observation.time_ns, &mut fields),
                                Some(twist(t, observation.time_ns, &mut fields)),
                                *wire,
                            )
                        }
                        native::NativeMessage::PoseVector {
                            header: h,
                            poses,
                            wire,
                        } => {
                            assert_eq!(observation.channel, "pose_vector");
                            assert!(h.is_none());
                            header(*h, observation.time_ns, &mut fields);
                            assert_eq!(poses.len(), 1);
                            frame_header(poses[0].header, &case.configurations[owner]);
                            (
                                captured_pose(&poses[0], observation.time_ns, &mut fields),
                                None,
                                *wire,
                            )
                        }
                    };
                    assert_eq!(
                        actual_bits.0, expected_pose,
                        "native pose bits: {} owner{owner} call{call}",
                        case.mode
                    );
                    if let Some((linear, angular)) = expected_twist {
                        assert_eq!(
                            (actual_bits.1, actual_bits.2),
                            (linear, angular),
                            "native velocity bits: {} owner{owner} call{call}",
                            case.mode
                        );
                    }
                    assert!(!wire.is_empty() && wire.len() <= 65536);
                    fields.wire_bytes += wire.len();
                    mode_poses += fields.poses - poses_before;
                    mode_words += fields.covariance_words - words_before;
                    mode_packets += 1;
                }
                assert_eq!(
                    channels,
                    BTreeSet::from(["odometry", "odometry_with_covariance", "pose_vector"])
                );
                mode_samples += 1;
            }
        }
        assert_eq!(
            (mode_samples, mode_packets, mode_poses, mode_words),
            (
                counts.observed_channel_triples as usize,
                counts.canonical_packets as usize,
                counts.captured_pose_bodies as usize,
                counts.covariance_f32_words as usize
            )
        );
        total_samples += mode_samples;
        total_packets += mode_packets;
    }
    assert_eq!(
        (total_configs, total_inputs, total_samples, total_packets),
        (10, 130, 90, 270)
    );
    assert_eq!((fields.poses, fields.covariance_words), (270, 6480));
    assert!(fields.headers > 0 && fields.header_values > 0 && fields.wire_bytes > 0);
}
