// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::prost::Message;
use gz_sim_scene::{PoseFramePublication, PoseFrameValue, WorldPose};

mod native {
    include!("../../../gz-sim-scene/tests/fixtures/m55_native_multiple_pose_publishers.rs");
}

fn finite_pose(bits: [u64; 7]) -> WorldPose {
    let values = bits.map(f64::from_bits);
    assert!(values.iter().all(|value| value.is_finite()));
    assert!((values[3..].iter().map(|value| value * value).sum::<f64>() - 1.0).abs() <= 1e-12);
    WorldPose {
        translation: values[..3].try_into().unwrap(),
        rotation: values[3..].try_into().unwrap(),
    }
}

fn pose_bits(value: &Pose) -> [u64; 7] {
    let position = value.position.as_ref().unwrap();
    let rotation = value.orientation.as_ref().unwrap();
    assert!(position.header.is_none() && rotation.header.is_none());
    [
        position.x, position.y, position.z, rotation.x, rotation.y, rotation.z, rotation.w,
    ]
    .map(f64::to_bits)
}

// The captured Pose_V contains only ordered, length-delimited field2 Pose bodies.
// Extract slices from the original bytes, without sorting or reencoding a vector.
fn original_pose_bodies(wire: &[u8]) -> Vec<&[u8]> {
    assert!(!wire.is_empty() && wire.len() <= 65536);
    let mut cursor = 0;
    let mut result = Vec::new();
    while cursor < wire.len() {
        assert_eq!(wire[cursor], 0x12);
        cursor += 1;
        let mut length = 0_u64;
        let mut ended = false;
        for shift in (0..70).step_by(7) {
            let byte = *wire.get(cursor).unwrap();
            cursor += 1;
            assert!(shift < 63 || byte <= 1);
            length |= u64::from(byte & 0x7f) << shift;
            if byte & 0x80 == 0 {
                ended = true;
                break;
            }
        }
        assert!(ended);
        let end = cursor
            .checked_add(usize::try_from(length).unwrap())
            .unwrap();
        assert!(cursor < end && end <= wire.len());
        result.push(&wire[cursor..end]);
        cursor = end;
    }
    assert_eq!(cursor, wire.len());
    result
}

#[test]
fn actual_native_pose_multiplicity_preserves_all_65_vectors_and_95_keyed_pose_header_bytes() {
    let mut total_configurations = 0;
    let mut total_inputs = 0;
    let mut total_vectors = 0;
    let mut total_records = 0;
    assert_eq!(native::CASES.len(), 2);
    for &(mode, configurations, inputs, summary) in native::CASES {
        let count = match mode {
            "pair" => 2,
            "eight" => 8,
            _ => panic!("closed original-object modes"),
        };
        assert_eq!(configurations.len(), count);
        assert_eq!(inputs.len(), 4 * count);
        assert_eq!(summary.configuration_count as usize, count);
        assert_eq!(summary.direct_post_update_calls as usize, inputs.len());
        assert_eq!(summary.input_rows as usize, inputs.len());
        assert_eq!(summary.observed_vectors, summary.canonical_packets);
        assert!(summary.original_object_linked_system_scope);
        assert!(!summary.original_server_or_plugin_loader_executed);
        assert!(!summary.native_physics_executed);
        assert!(!summary.native_demand_or_lifecycle_equivalence_claimed);
        assert!(!summary.global_rng_equivalence_claimed);
        assert!(!summary.old_native_modes_replayed);
        let mut endpoints = BTreeSet::new();
        let mut native_entities = BTreeSet::new();
        for (owner, config) in configurations.iter().enumerate() {
            assert_eq!(config.mode, mode);
            assert_eq!(config.owner as usize, owner);
            assert!(config.world_entity != 0 && config.native_entity != config.world_entity);
            assert_eq!(config.world_name, "default");
            assert_eq!(config.world_pose, [0, 0, 0, 0, 0, 0, 1.0_f64.to_bits()]);
            let _world_configuration_pose = finite_pose(config.world_pose);
            assert_eq!(config.name, format!("rig{owner}"));
            assert!(config.static_publisher && config.use_pose_vector_msg);
            assert_eq!(config.console_verbosity, 2);
            assert_eq!(
                (config.parent_link_name, config.child_link_name),
                ("base", "tip")
            );
            assert_eq!(config.joint_type, 6);
            assert_eq!(config.static_topic, format!("{}_static", config.topic));
            assert!(config.topic.starts_with("/m55/native/"));
            assert!(config.topic.ends_with(&format!("/{mode}/pose{owner}")));
            assert!(endpoints.insert(config.topic) && endpoints.insert(config.static_topic));
            let dynamic_rate = f64::from_bits(config.dynamic_rate);
            let static_rate = f64::from_bits(config.static_rate);
            assert_eq!(
                dynamic_rate,
                if owner.is_multiple_of(2) { 100.0 } else { 50.0 }
            );
            assert_eq!(
                static_rate,
                if owner.is_multiple_of(2) { 25.0 } else { 100.0 }
            );
            let xml = std::str::from_utf8(config.sdf_xml).unwrap();
            assert!(xml.starts_with("<plugin ") && xml.ends_with("</plugin>\n"));
            for token in [
                "gz::sim::systems::PosePublisher",
                "direct-original-object-not-a-loader-input",
                "<publish_link_pose>true</publish_link_pose>",
                "<publish_model_pose>true</publish_model_pose>",
                "<publish_nested_model_pose>false</publish_nested_model_pose>",
                "<publish_sensor_pose>false</publish_sensor_pose>",
                "<publish_visual_pose>false</publish_visual_pose>",
                "<publish_collision_pose>false</publish_collision_pose>",
                "<use_pose_vector_msg>true</use_pose_vector_msg>",
                "<static_publisher>true</static_publisher>",
            ] {
                assert!(xml.contains(token));
            }
            assert!(xml.contains(&format!("<topic>{}</topic>", config.topic)));
            assert!(xml.contains(&format!(
                "<update_frequency>{dynamic_rate}</update_frequency>"
            )));
            assert!(xml.contains(&format!(
                "<static_update_frequency>{static_rate}</static_update_frequency>"
            )));
            assert_eq!(config.entities.len(), 4);
            for (role, entity) in config.entities.iter().enumerate() {
                assert!(entity.native_entity != 0 && native_entities.insert(entity.native_entity));
                let _initial_configuration_pose = finite_pose(entity.initial_local_pose);
                assert_eq!(
                    entity.parent_entity,
                    if role == 0 {
                        config.world_entity
                    } else {
                        config.native_entity
                    }
                );
                match role {
                    0 => {
                        assert_eq!(entity.native_entity, config.native_entity);
                        assert_eq!(
                            (entity.kind, entity.name, entity.declared_channel),
                            ("model", config.name, "static")
                        );
                        assert!(entity.selected && !entity.canonical_link);
                    }
                    1 => {
                        assert_eq!(entity.native_entity, config.model_canonical_link);
                        assert_eq!(
                            (entity.kind, entity.name, entity.declared_channel),
                            ("link", "base", "static")
                        );
                        assert!(entity.selected && entity.canonical_link);
                    }
                    2 => {
                        assert_eq!(
                            (entity.kind, entity.name, entity.declared_channel),
                            ("link", "tip", "dynamic")
                        );
                        assert!(entity.selected && !entity.canonical_link);
                    }
                    3 => {
                        assert_eq!(
                            (entity.kind, entity.name, entity.declared_channel),
                            ("joint", "hinge", "unselected")
                        );
                        assert!(!entity.selected && !entity.canonical_link);
                    }
                    _ => unreachable!(),
                }
            }
        }
        assert_eq!(endpoints.len(), 2 * count);
        let mut calls = BTreeSet::new();
        let mut vectors = 0;
        let mut records = 0;
        for (ordinal, input) in inputs.iter().enumerate() {
            let owner = input.owner as usize;
            assert_eq!(
                (input.call_index as usize, owner),
                (ordinal / count, ordinal % count)
            );
            assert!(calls.insert((input.owner, input.call_index)));
            let times = [0, 10_000_000, 20_000_000, 40_000_000];
            let call = input.call_index as usize;
            assert_eq!(input.time_ns, times[call]);
            assert_eq!(
                input.dt_ns,
                if call == 0 {
                    0
                } else {
                    times[call] - times[call - 1]
                }
            );
            assert!(!input.paused);
            let config = &configurations[owner];
            assert_eq!(input.local_poses.len(), 3);
            for (role, local) in input.local_poses.iter().enumerate() {
                assert_eq!(local.native_entity, config.entities[role].native_entity);
                assert_eq!(local.input_pose, local.getter_pose);
                let _actual_getback_pose = finite_pose(local.getter_pose);
                if call == 0 {
                    assert_eq!(local.input_pose, config.entities[role].initial_local_pose);
                }
            }
            assert!((1..=2).contains(&input.observed.len()));
            let mut channels = BTreeSet::new();
            for observed in input.observed {
                assert!(channels.insert(observed.channel));
                let fixed = match observed.channel {
                    "dynamic" => false,
                    "static" => true,
                    _ => panic!("closed original channel"),
                };
                assert_eq!(observed.records.len(), if fixed { 2 } else { 1 });
                let actual_native = PoseV::decode(observed.wire).unwrap();
                assert!(actual_native.header.is_none());
                let original_bodies = original_pose_bodies(observed.wire);
                assert_eq!(original_bodies.len(), observed.records.len());
                assert_eq!(actual_native.pose.len(), observed.records.len());
                let mut names = BTreeSet::new();
                let mut lexical: Vec<_> = observed.records.iter().collect();
                lexical.sort_by_key(|record| record.child_frame_id);
                // Test-local IDs bind encoder ownership only. They are never native or wire IDs.
                let rust_owner = 1000 + 10 * u64::from(input.owner);
                let identities: Vec<_> = lexical
                    .iter()
                    .map(|record| {
                        let role = config
                            .entities
                            .iter()
                            .position(|entity| {
                                entity.selected
                                    && if entity.kind == "model" {
                                        record.child_frame_id == config.name
                                    } else {
                                        record.child_frame_id
                                            == format!("{}::{}", config.name, entity.name)
                                    }
                            })
                            .unwrap();
                        rust_owner + role as u64
                    })
                    .collect();
                let metadata = PosePublication {
                    model_name: config.name.into(),
                    topic: if fixed {
                        config.static_topic
                    } else {
                        config.topic
                    }
                    .into(),
                    static_publication: None,
                    poses: lexical
                        .iter()
                        .zip(&identities)
                        .map(|(record, &identity)| PoseFramePublication {
                            identity,
                            frame_id: record.frame_id.into(),
                            child_frame_id: record.child_frame_id.into(),
                        })
                        .collect(),
                };
                let sample = PoseSample {
                    time: Duration::from_nanos(input.time_ns),
                    epoch: 0,
                    revision: u64::from(input.call_index) + 1,
                    poses: lexical
                        .iter()
                        .zip(&identities)
                        .map(|(record, &identity)| PoseFrameValue {
                            identity,
                            pose: finite_pose(record.pose),
                        })
                        .collect(),
                };
                let encoded = packet(&metadata, rust_owner, &sample).unwrap();
                assert!(encoded.header.is_none());
                assert!(
                    encoded
                        .pose
                        .windows(2)
                        .all(|pair| pair[0].name < pair[1].name)
                );
                // Full Rust lexical packet is separate from the original unordered vector.
                let rust_wire = encoded.encode_to_vec();
                let decoded_rust = PoseV::decode(rust_wire.as_slice()).unwrap();
                assert_eq!(decoded_rust.pose.len(), observed.records.len());
                for ((record, native_pose), original_body) in observed
                    .records
                    .iter()
                    .zip(&actual_native.pose)
                    .zip(original_bodies)
                {
                    assert!(names.insert(record.child_frame_id));
                    assert_eq!(record.name, record.child_frame_id);
                    assert_eq!(record.id, 0);
                    assert_eq!(
                        record.frame_id,
                        if record.child_frame_id == config.name {
                            config.world_name
                        } else {
                            config.name
                        }
                    );
                    let declared = config
                        .entities
                        .iter()
                        .find(|entity| {
                            if entity.kind == "model" {
                                record.child_frame_id == entity.name
                            } else {
                                record.child_frame_id == format!("{}::{}", config.name, entity.name)
                            }
                        })
                        .unwrap();
                    assert!(declared.selected);
                    assert_eq!(declared.declared_channel, observed.channel);
                    assert_eq!(original_body, record.pose_wire);
                    assert_eq!(native_pose.name, record.name);
                    assert_eq!(u64::from(native_pose.id), record.id);
                    assert_eq!(pose_bits(native_pose), record.pose);
                    let actual = encoded
                        .pose
                        .iter()
                        .find(|pose| pose.name == record.child_frame_id)
                        .unwrap();
                    let decoded = decoded_rust
                        .pose
                        .iter()
                        .find(|pose| pose.name == record.child_frame_id)
                        .unwrap();
                    assert_eq!(pose_bits(actual), record.pose);
                    assert_eq!(pose_bits(decoded), record.pose);
                    assert_eq!(actual.id, 0);
                    assert_eq!(actual.encode_to_vec(), record.pose_wire);
                    let header = actual.header.as_ref().unwrap();
                    assert_eq!(header.encode_to_vec(), record.header_wire);
                    assert_eq!(
                        native_pose.header.as_ref().unwrap().encode_to_vec(),
                        record.header_wire
                    );
                    let native_header = Header::decode(record.header_wire).unwrap();
                    let stamp = native_header.stamp.as_ref().unwrap();
                    assert_eq!(stamp.sec as u64, input.time_ns / 1_000_000_000);
                    assert_eq!(stamp.nsec as u64, input.time_ns % 1_000_000_000);
                    assert_eq!(native_header.data.len(), 2);
                    assert_eq!(native_header.data[0].key, "frame_id");
                    assert_eq!(native_header.data[0].value, [record.frame_id]);
                    assert_eq!(native_header.data[1].key, "child_frame_id");
                    assert_eq!(native_header.data[1].value, [record.child_frame_id]);
                    records += 1;
                }
                assert_eq!(names.len(), observed.records.len());
                vectors += 1;
            }
        }
        assert_eq!(calls.len(), inputs.len());
        assert_eq!(vectors, summary.observed_vectors as usize);
        assert_eq!(records, summary.pose_records as usize);
        assert_eq!(
            (vectors, records),
            if mode == "pair" { (13, 19) } else { (52, 76) }
        );
        total_configurations += configurations.len();
        total_inputs += inputs.len();
        total_vectors += vectors;
        total_records += records;
    }
    assert_eq!(
        (
            total_configurations,
            total_inputs,
            total_vectors,
            total_records
        ),
        (10, 40, 65, 95)
    );
}
