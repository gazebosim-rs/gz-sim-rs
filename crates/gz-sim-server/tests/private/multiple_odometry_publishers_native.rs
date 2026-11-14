// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::msgs::{Quaternion, Vector3d};
use gz_sim_proto::prost::Message;
use gz_sim_scene::{GroundTruthOdometryNoise, GroundTruthOdometryOffsets, WorldPose};
use std::time::Duration;

mod native {
    include!("../../../gz-sim-scene/tests/fixtures/m56_native_multiple_odometry_publishers.rs");
}

fn finite_pose(words: [u64; 7]) -> WorldPose {
    let values = words.map(f64::from_bits);
    assert!(values.iter().all(|value| value.is_finite()));
    assert!((values[3..].iter().map(|value| value * value).sum::<f64>() - 1.0).abs() <= 1e-12);
    WorldPose {
        translation: values[..3].try_into().unwrap(),
        rotation: values[3..].try_into().unwrap(),
    }
}

fn header(actual: Option<&Header>, captured: Option<&native::NativeHeader>) {
    match (actual, captured) {
        (None, None) => {}
        (Some(actual), Some(captured)) => {
            assert!(captured.wire.len() <= 4096);
            assert_eq!(
                actual.stamp.as_ref().map(|time| (time.sec, time.nsec)),
                captured.stamp
            );
            assert_eq!(actual.data.len(), captured.data.len());
            assert!(captured.data.len() <= 16);
            for (actual, captured) in actual.data.iter().zip(captured.data) {
                assert_eq!(actual.key.as_bytes(), captured.key);
                assert!(captured.key.len() <= 256);
                assert_eq!(actual.value.len(), captured.values.len());
                assert!(captured.values.len() <= 8);
                for (actual, captured) in actual.value.iter().zip(captured.values) {
                    assert_eq!(actual.as_bytes(), *captured);
                    assert!(captured.len() <= 1024);
                }
            }
            assert_eq!(actual.encode_to_vec(), captured.wire);
            assert_eq!(Header::decode(captured.wire).unwrap(), *actual);
        }
        _ => panic!("each actual optional Header presence is retained independently"),
    }
}

fn vector(actual: &Vector3d, captured: &native::NativeVector3) {
    header(actual.header.as_ref(), captured.header);
    assert_eq!(
        [actual.x, actual.y, actual.z].map(f64::to_bits),
        captured.bits
    );
    assert!(
        captured
            .bits
            .into_iter()
            .all(|word| f64::from_bits(word).is_finite())
    );
    assert!(captured.wire.len() <= 65536);
    assert_eq!(actual.encode_to_vec(), captured.wire);
    assert_eq!(Vector3d::decode(captured.wire).unwrap(), *actual);
}

fn rotation(actual: &Quaternion, captured: &native::NativeQuaternion) {
    header(actual.header.as_ref(), captured.header);
    assert_eq!(
        [actual.x, actual.y, actual.z, actual.w].map(f64::to_bits),
        captured.bits
    );
    assert!(
        captured
            .bits
            .into_iter()
            .all(|word| f64::from_bits(word).is_finite())
    );
    assert!(captured.wire.len() <= 65536);
    assert_eq!(actual.encode_to_vec(), captured.wire);
    assert_eq!(Quaternion::decode(captured.wire).unwrap(), *actual);
}

fn pose(actual: &Pose, captured: &native::NativePose) {
    header(actual.header.as_ref(), captured.header);
    assert_eq!(actual.name.as_bytes(), captured.name);
    assert_eq!(actual.id, captured.id); // Both protobuf and audited literal IDs are u32.
    assert!(captured.name.is_empty() && captured.id == 0);
    vector(actual.position.as_ref().unwrap(), captured.position);
    rotation(actual.orientation.as_ref().unwrap(), captured.orientation);
    assert!(captured.wire.len() <= 65536);
    assert_eq!(actual.encode_to_vec(), captured.wire);
    assert_eq!(Pose::decode(captured.wire).unwrap(), *actual);
}

fn twist(actual: &Twist, captured: &native::NativeTwist) {
    header(actual.header.as_ref(), captured.header);
    vector(actual.linear.as_ref().unwrap(), captured.linear);
    vector(actual.angular.as_ref().unwrap(), captured.angular);
    assert!(captured.wire.len() <= 65536);
    assert_eq!(actual.encode_to_vec(), captured.wire);
    assert_eq!(Twist::decode(captured.wire).unwrap(), *actual);
}

fn covariance(actual: &FloatV, captured: &native::NativeFloatV) {
    header(actual.header.as_ref(), captured.header);
    assert_eq!(actual.data.len(), 36);
    let words = actual
        .data
        .iter()
        .map(|value| value.to_bits())
        .collect::<Vec<_>>();
    assert_eq!(words.as_slice(), captured.f32_bits.as_slice());
    assert!(
        captured
            .f32_bits
            .into_iter()
            .all(|word| f32::from_bits(word).is_finite())
    );
    assert!(captured.wire.len() <= 65536);
    assert_eq!(actual.encode_to_vec(), captured.wire);
    assert_eq!(FloatV::decode(captured.wire).unwrap(), *actual);
}

fn entity_input(value: &native::NativeEntityInput, configuration: &native::NativeConfiguration) {
    assert_eq!(value.native_entity, configuration.entity.native_entity);
    assert!(
        value.native_entity != 0
            && value.world_entity != 0
            && value.native_entity != value.world_entity
    );
    assert_eq!(value.world_entity, configuration.entity.world_entity);
    assert_eq!(value.parent_component, value.world_entity);
    assert_eq!(value.parent_graph, value.parent_component);
    assert_eq!(value.input_local_pose_bits, value.getter_local_pose_bits);
    assert_eq!(
        value.world_local_pose_bits,
        configuration.entity.world_local_pose_bits
    );
    for words in [
        value.input_local_pose_bits,
        value.getter_local_pose_bits,
        value.world_local_pose_bits,
        value.getter_world_pose_bits,
    ] {
        let actual = finite_pose(words);
        assert!(
            actual
                .translation
                .into_iter()
                .chain(actual.rotation)
                .all(f64::is_finite)
        );
    }
    // These are actual public getters and parent-component provenance. They are
    // not an independently reconstructed WORLD transform or a Physics trajectory.
    assert_ne!(
        value.world_local_pose_bits,
        [0, 0, 0, 0, 0, 0, 1.0_f64.to_bits()]
    );
}

fn xml_value<'a>(xml: &'a str, name: &str) -> &'a str {
    let open = format!("<{name}>");
    let close = format!("</{name}>");
    assert_eq!(xml.matches(&open).count(), 1);
    assert_eq!(xml.matches(&close).count(), 1);
    let start = xml.find(&open).unwrap() + open.len();
    let end = xml[start..].find(&close).unwrap() + start;
    xml[start..end].trim()
}

fn xml_vector(xml: &str, name: &str) -> [f64; 3] {
    let values = xml_value(xml, name)
        .split_whitespace()
        .map(|value| value.parse::<f64>().unwrap())
        .collect::<Vec<_>>();
    assert_eq!(values.len(), 3);
    assert!(values.iter().all(|value| value.is_finite()));
    values.try_into().unwrap()
}

fn metadata(configuration: &native::NativeConfiguration) -> Metadata {
    assert_eq!(configuration.schema_version, 1);
    assert_eq!(configuration.console_verbosity, 2);
    assert!(configuration.sdf_xml.len() <= 262144);
    let xml = std::str::from_utf8(configuration.sdf_xml).unwrap();
    assert!(xml.starts_with("<plugin ") && xml.ends_with("</plugin>\n"));
    assert!(xml.contains("gz::sim::systems::OdometryPublisher"));
    assert!(xml.contains("direct-original-object-not-loader-input"));
    assert_eq!(
        xml_value(xml, "dimensions").parse::<u32>().unwrap(),
        configuration.dimensions
    );
    assert_eq!(
        xml_value(xml, "odom_publish_frequency")
            .parse::<f64>()
            .unwrap()
            .to_bits(),
        configuration.frequency_bits
    );
    assert_eq!(
        xml_value(xml, "gaussian_noise")
            .parse::<f64>()
            .unwrap()
            .to_bits(),
        configuration.gaussian_noise_bits
    );
    assert_eq!(xml_value(xml, "odom_frame"), configuration.odom_frame);
    assert_eq!(
        xml_value(xml, "robot_base_frame"),
        configuration.robot_base_frame
    );
    for (name, topic) in ["odom_topic", "odom_covariance_topic", "tf_topic"]
        .into_iter()
        .zip(configuration.topics)
    {
        assert_eq!(xml_value(xml, name), topic);
    }
    let local = xml_vector(xml, "xyz_offset");
    let global = xml_vector(xml, "xyz_global_offset");
    assert_eq!(local.map(f64::to_bits), configuration.offset_pose_bits[..3]);
    assert_eq!(
        global.map(f64::to_bits),
        configuration.global_offset_pose_bits[..3]
    );
    let local_rotation = xml_vector(xml, "rpy_offset");
    let global_rotation = xml_vector(xml, "rpy_global_offset");
    let offsets = GroundTruthOdometryOffsets::from_translations(local, global)
        .unwrap()
        .with_rotations(local_rotation, global_rotation)
        .unwrap();
    assert_eq!(
        offsets.local_translation().map(f64::to_bits),
        local.map(f64::to_bits)
    );
    assert_eq!(
        offsets.global_translation().map(f64::to_bits),
        global.map(f64::to_bits)
    );
    assert_eq!(
        offsets.local_rotation_rpy().map(f64::to_bits),
        local_rotation.map(f64::to_bits)
    );
    assert_eq!(
        offsets.global_rotation_rpy().map(f64::to_bits),
        global_rotation.map(f64::to_bits)
    );
    // Offset quaternions are retained input/getter provenance; packet() consumes
    // the already acquired output pose, so no offset/output oracle is computed.
    finite_pose(configuration.offset_pose_bits);
    finite_pose(configuration.global_offset_pose_bits);
    let noise =
        GroundTruthOdometryNoise::from_stddev(f64::from_bits(configuration.gaussian_noise_bits))
            .unwrap();
    assert_eq!(noise.stddev().to_bits(), configuration.gaussian_noise_bits);
    assert_eq!(noise.stddev(), 0.0); // This fresh deterministic campaign is not the old noise proof.
    Metadata {
        dimensions: match configuration.dimensions {
            2 => Dimensions::Two,
            3 => Dimensions::Three,
            _ => panic!("closed dimensions"),
        },
        offsets,
        gaussian_noise: noise,
        model_name: configuration.name.to_owned(),
        odom_frame: configuration.odom_frame.to_owned(),
        robot_base_frame: configuration.robot_base_frame.to_owned(),
        odom_topic: configuration.topics[0].to_owned(),
        covariance_topic: configuration.topics[1].to_owned(),
        tf_topic: configuration.topics[2].to_owned(),
    }
}

fn sample(input: &native::NativeInput, captured: &native::NativeMessage) -> Sample {
    let native::NativeMessage::Odometry {
        header: captured_header,
        pose: captured_pose,
        twist: captured_twist,
        ..
    } = captured
    else {
        panic!("captured odometry, not geometric reconstruction, supplies packet input")
    };
    let stamp = captured_header.unwrap().stamp.unwrap();
    assert_eq!(
        i128::from(stamp.0) * 1_000_000_000 + i128::from(stamp.1),
        i128::from(input.time_ns)
    );
    Sample {
        time: Duration::from_nanos(u64::try_from(input.time_ns).unwrap()),
        // The packet API's local cache preconditions are not native fields or
        // private-clock equivalence. Owner IDs below are likewise Rust-only.
        epoch: 0,
        revision: input.call_index + 1,
        pose: WorldPose {
            translation: captured_pose.position.bits.map(f64::from_bits),
            rotation: captured_pose.orientation.bits.map(f64::from_bits),
        },
        linear: captured_twist.linear.bits.map(f64::from_bits),
        angular: captured_twist.angular.bits.map(f64::from_bits),
    }
}

fn row(order: &[native::NativeRowRef], cursor: &mut usize, kind: &str, index: usize) {
    let captured = order.get(*cursor).unwrap();
    assert_eq!(captured.kind, kind);
    assert_eq!(usize::try_from(captured.index).unwrap(), index);
    *cursor += 1;
}

#[test]
fn actual_native_odometry_multiplicity_preserves_all_270_typed_packets_and_complete_input_order() {
    let (pair, eight) = native::native_cases();
    assert_eq!((pair.mode, eight.mode), ("pair", "eight"));
    let mut totals = [0_usize; 13];
    for case in [pair, eight] {
        let count = match case.mode {
            "pair" => 2,
            "eight" => 8,
            _ => panic!("closed native modes"),
        };
        assert_eq!(case.summary.mode, case.mode);
        assert!(case.summary.original_object_linked_system_scope);
        assert!(!case.summary.original_server_or_plugin_loader_executed);
        assert!(!case.summary.native_physics_executed);
        assert!(
            !case
                .summary
                .native_demand_reset_removal_or_lifecycle_equivalence_claimed
        );
        assert!(!case.summary.global_rng_equivalence_claimed);
        assert!(!case.summary.old_native_modes_replayed);
        assert_eq!(case.configurations.len(), count);
        assert_eq!(case.inputs.len(), 13 * count);
        assert_eq!(case.fences.len(), case.inputs.len());
        let mut endpoints = BTreeSet::new();
        let mut identities = BTreeSet::new();
        let mut publications = Vec::new();
        let mut raw_cursor = 0;
        for (owner, configuration) in case.configurations.iter().enumerate() {
            row(case.raw_order, &mut raw_cursor, "configuration", owner);
            assert_eq!(configuration.mode, case.mode);
            assert_eq!(usize::try_from(configuration.owner).unwrap(), owner);
            assert_eq!(configuration.name, format!("rig{owner}"));
            assert_eq!(configuration.world_name, "reference_world");
            assert!(identities.insert(configuration.entity.native_entity));
            assert_eq!(
                configuration.entity.world_entity,
                case.configurations[0].entity.world_entity
            );
            entity_input(&configuration.entity, configuration);
            assert_eq!(configuration.odom_frame, format!("rig{owner}/odom"));
            assert_eq!(
                configuration.robot_base_frame,
                format!("rig{owner}/base_footprint")
            );
            assert_eq!(
                configuration.dimensions,
                if owner.is_multiple_of(2) { 2 } else { 3 }
            );
            assert_eq!(
                f64::from_bits(configuration.frequency_bits),
                if owner.is_multiple_of(2) { 100.0 } else { 50.0 }
            );
            for (topic, suffix) in
                configuration
                    .topics
                    .into_iter()
                    .zip(["odom", "covariance", "pose"])
            {
                assert!(topic.starts_with("/m56/native/"));
                assert!(topic.ends_with(&format!("/{}/owner{owner}/{suffix}", case.mode)));
                assert!(endpoints.insert(topic));
            }
            let rust_owner = 1_000_000 + u64::try_from(owner).unwrap();
            assert_ne!(rust_owner, configuration.entity.native_entity);
            let publication = metadata(configuration);
            assert!(valid(&publication, rust_owner));
            publications.push(publication);
        }
        assert_eq!(endpoints.len(), 3 * count);
        let mut callbacks = vec![0_u64; count];
        let mut observed_index = 0;
        let mut triples = 0;
        let mut pose_records = 0;
        let mut covariance_words = 0;
        let mut local_inputs = vec![BTreeSet::new(); count];
        for call in 0..13_usize {
            // Every owner's complete input precedes any callback for this call.
            for (owner, local_history) in local_inputs.iter_mut().enumerate() {
                let index = call * count + owner;
                row(case.raw_order, &mut raw_cursor, "input", index);
                let input = &case.inputs[index];
                assert_eq!(input.mode, case.mode);
                assert_eq!(usize::try_from(input.owner).unwrap(), owner);
                assert_eq!(input.call_index, u64::try_from(call).unwrap());
                assert_eq!(input.iterations, input.call_index);
                assert_eq!(input.time_ns, i64::try_from(call).unwrap() * 10_000_000);
                assert_eq!(input.real_time_ns, input.time_ns);
                assert_eq!(input.dt_ns, if call == 0 { 0 } else { 10_000_000 });
                assert!(!input.paused);
                entity_input(&input.entity, &case.configurations[owner]);
                if call == 0 {
                    assert_eq!(
                        input.entity.input_local_pose_bits,
                        case.configurations[owner].entity.input_local_pose_bits
                    );
                    assert_eq!(
                        input.entity.getter_world_pose_bits,
                        case.configurations[owner].entity.getter_world_pose_bits
                    );
                }
                assert!(local_history.insert(input.entity.input_local_pose_bits));
            }
            for (owner, cumulative) in callbacks.iter_mut().enumerate() {
                let index = call * count + owner;
                let input = &case.inputs[index];
                let observed = case
                    .observations
                    .iter()
                    .filter(|value| {
                        value.owner == input.owner && value.call_index == input.call_index
                    })
                    .collect::<Vec<_>>();
                let fence = &case.fences[index];
                assert_eq!(
                    (fence.mode, fence.owner, fence.call_index, fence.time_ns),
                    (case.mode, input.owner, input.call_index, input.time_ns)
                );
                assert_eq!(
                    usize::try_from(fence.callback_count).unwrap(),
                    observed.len()
                );
                let recorded_due = call > 0 && (owner.is_multiple_of(2) || call.is_multiple_of(2));
                assert_eq!(observed.len(), if recorded_due { 3 } else { 0 });
                // No row on a bounded fence is only no matching packet, never an
                // assertion about an internal Update/history/RNG event.
                if let Some(odometry) = observed.iter().find(|value| value.channel == "odometry") {
                    let acquired = sample(input, odometry.message);
                    let rust_owner = 1_000_000 + u64::try_from(owner).unwrap();
                    let (actual_odom, actual_cov, actual_tf) =
                        packets(&publications[owner], rust_owner, &acquired).unwrap();
                    let mut channels = BTreeSet::new();
                    for captured in observed {
                        row(
                            case.raw_order,
                            &mut raw_cursor,
                            "observation",
                            observed_index,
                        );
                        assert!(std::ptr::eq(captured, &case.observations[observed_index]));
                        observed_index += 1;
                        assert_eq!(
                            (
                                captured.mode,
                                captured.owner,
                                captured.call_index,
                                captured.time_ns
                            ),
                            (case.mode, input.owner, input.call_index, input.time_ns)
                        );
                        assert_eq!(captured.arrival_ordinal, *cumulative);
                        *cumulative += 1;
                        assert!(channels.insert(captured.channel));
                        match captured.message {
                            native::NativeMessage::Odometry {
                                header: expected_header,
                                pose: expected_pose,
                                twist: expected_twist,
                                wire,
                            } => {
                                assert_eq!(captured.channel, "odometry");
                                header(actual_odom.header.as_ref(), *expected_header);
                                pose(actual_odom.pose.as_ref().unwrap(), expected_pose);
                                twist(actual_odom.twist.as_ref().unwrap(), expected_twist);
                                assert_eq!(actual_odom.encode_to_vec(), *wire);
                                assert_eq!(Odometry::decode(*wire).unwrap(), actual_odom);
                            }
                            native::NativeMessage::OdometryWithCovariance {
                                header: expected_header,
                                pose: expected_pose,
                                pose_covariance,
                                twist: expected_twist,
                                twist_covariance,
                                pose_with_covariance_wire,
                                twist_with_covariance_wire,
                                wire,
                            } => {
                                assert_eq!(captured.channel, "odometry_with_covariance");
                                header(actual_cov.header.as_ref(), *expected_header);
                                let pose_wrapper =
                                    actual_cov.pose_with_covariance.as_ref().unwrap();
                                let twist_wrapper =
                                    actual_cov.twist_with_covariance.as_ref().unwrap();
                                pose(pose_wrapper.pose.as_ref().unwrap(), expected_pose);
                                covariance(
                                    pose_wrapper.covariance.as_ref().unwrap(),
                                    pose_covariance,
                                );
                                twist(twist_wrapper.twist.as_ref().unwrap(), expected_twist);
                                covariance(
                                    twist_wrapper.covariance.as_ref().unwrap(),
                                    twist_covariance,
                                );
                                assert_eq!(
                                    pose_wrapper.encode_to_vec(),
                                    *pose_with_covariance_wire
                                );
                                assert_eq!(
                                    twist_wrapper.encode_to_vec(),
                                    *twist_with_covariance_wire
                                );
                                assert_eq!(
                                    PoseWithCovariance::decode(*pose_with_covariance_wire).unwrap(),
                                    *pose_wrapper
                                );
                                assert_eq!(
                                    TwistWithCovariance::decode(*twist_with_covariance_wire)
                                        .unwrap(),
                                    *twist_wrapper
                                );
                                covariance_words += pose_covariance.f32_bits.len()
                                    + twist_covariance.f32_bits.len();
                                assert_eq!(actual_cov.encode_to_vec(), *wire);
                                assert_eq!(
                                    OdometryWithCovariance::decode(*wire).unwrap(),
                                    actual_cov
                                );
                            }
                            native::NativeMessage::PoseVector {
                                header: expected_header,
                                poses,
                                wire,
                            } => {
                                assert_eq!(captured.channel, "pose_vector");
                                header(actual_tf.header.as_ref(), *expected_header);
                                assert_eq!(actual_tf.pose.len(), poses.len());
                                assert_eq!(poses.len(), 1);
                                for (actual, captured) in actual_tf.pose.iter().zip(*poses) {
                                    pose(actual, captured);
                                    pose_records += 1;
                                }
                                assert_eq!(actual_tf.encode_to_vec(), *wire);
                                assert_eq!(PoseV::decode(*wire).unwrap(), actual_tf);
                            }
                        }
                    }
                    assert_eq!(
                        channels,
                        BTreeSet::from(["odometry", "odometry_with_covariance", "pose_vector"])
                    );
                    triples += 1;
                } else {
                    assert!(observed.is_empty());
                }
                row(case.raw_order, &mut raw_cursor, "fence", index);
                assert_eq!(
                    fence.cumulative_callbacks,
                    u32::try_from(*cumulative).unwrap()
                );
            }
        }
        row(case.raw_order, &mut raw_cursor, "summary", 0);
        assert_eq!(raw_cursor, case.raw_order.len());
        assert!(local_inputs.iter().all(|values| values.len() == 13));
        assert_eq!(observed_index, case.observations.len());
        assert_eq!(case.observations.len(), 3 * triples);
        assert_eq!(pose_records, triples);
        assert_eq!(covariance_words, 72 * triples);
        let actual = [
            count,
            13,
            case.inputs.len(),
            case.inputs.len(),
            case.inputs.len(),
            2 * case.inputs.len(),
            case.fences.len(),
            triples,
            case.observations.len(),
            pose_records,
            3 * triples,
            covariance_words,
            case.raw_order.len(),
        ];
        let counts = case.summary.counts;
        let captured = [
            counts.configuration_count,
            counts.global_call_groups,
            counts.input_rows,
            counts.direct_pre_update_calls,
            counts.direct_post_update_calls,
            counts.public_interface_invocations,
            counts.fence_rows,
            counts.observed_channel_triples,
            counts.canonical_packets,
            counts.pose_vector_records,
            counts.captured_pose_bodies,
            counts.covariance_f32_words,
            counts.raw_rows,
        ];
        assert_eq!(
            actual,
            captured.map(|value| usize::try_from(value).unwrap())
        );
        for (total, value) in totals.iter_mut().zip(actual) {
            *total += value;
        }
    }
    assert_eq!(
        totals,
        [10, 26, 130, 130, 130, 260, 130, 90, 270, 90, 270, 6480, 542]
    );
}
