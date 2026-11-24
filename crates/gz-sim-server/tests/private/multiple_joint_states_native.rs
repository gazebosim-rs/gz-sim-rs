// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// HELD artifact-only Source. Adoption and actual fixture insertion require Root release.
use super::*;
use gz_sim_proto::msgs::{Quaternion, Vector3d};
use gz_sim_proto::prost::Message;
use gz_sim_scene::{JointStateJointPublication, JointStateValue, WorldPose};
use std::time::Duration;

mod native {
    include!("../../../gz-sim-scene/tests/fixtures/m57_native_multiple_joint_state_publishers.rs");
}

fn pose_words(value: WorldPose) -> [u64; 7] {
    value
        .translation
        .into_iter()
        .chain(value.rotation)
        .map(f64::to_bits)
        .collect::<Vec<_>>()
        .try_into()
        .unwrap()
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

// Raw native wire remains untouched. This bounded field view checks exact nested
// slices and absence; it never sorts, normalizes or reencodes native bytes.
struct Field<'a> {
    number: u32,
    kind: u8,
    payload: &'a [u8],
}
fn varint(bytes: &[u8], offset: &mut usize) -> u64 {
    let mut result = 0_u64;
    for shift in (0..70).step_by(7) {
        let byte = *bytes.get(*offset).unwrap();
        *offset += 1;
        assert!(shift < 63 || byte <= 1);
        result |= u64::from(byte & 127) << shift;
        if byte & 128 == 0 {
            return result;
        }
    }
    panic!("bounded protobuf varint");
}
fn fields<'a>(bytes: &'a [u8], allowed: &[u32]) -> Vec<Field<'a>> {
    assert!(bytes.len() <= 65536);
    let mut offset = 0;
    let mut result = Vec::new();
    while offset < bytes.len() {
        let key = varint(bytes, &mut offset);
        let number = u32::try_from(key >> 3).unwrap();
        let kind = u8::try_from(key & 7).unwrap();
        assert!(number != 0 && allowed.contains(&number));
        let start;
        let end;
        match kind {
            0 => {
                start = offset;
                varint(bytes, &mut offset);
                end = offset;
            }
            1 => {
                start = offset;
                end = offset.checked_add(8).unwrap();
                offset = end;
            }
            2 => {
                let length = usize::try_from(varint(bytes, &mut offset)).unwrap();
                start = offset;
                end = offset.checked_add(length).unwrap();
                offset = end;
            }
            5 => {
                start = offset;
                end = offset.checked_add(4).unwrap();
                offset = end;
            }
            _ => panic!("closed native wire kinds"),
        }
        assert!(end <= bytes.len() && result.len() < 4096);
        result.push(Field {
            number,
            kind,
            payload: &bytes[start..end],
        });
    }
    result
}
fn nested(rows: &[Field<'_>], number: u32, expected: &[&[u8]]) {
    let actual = rows
        .iter()
        .filter(|row| row.number == number)
        .map(|row| {
            assert_eq!(row.kind, 2);
            row.payload
        })
        .collect::<Vec<_>>();
    assert_eq!(actual, expected);
}
fn optional_wire<'a, T>(value: Option<&'a T>, wire: impl Fn(&'a T) -> &'a [u8]) -> Vec<&'a [u8]> {
    value.map(wire).into_iter().collect()
}
fn header(value: Option<&Header>, captured: Option<&native::NativeHeader>) {
    match (value, captured) {
        (None, None) => {}
        (Some(value), Some(captured)) => {
            assert!(captured.wire.len() <= 4096);
            assert_eq!(
                value.stamp.as_ref().map(|stamp| (stamp.sec, stamp.nsec)),
                captured.stamp.map(|stamp| (stamp.sec, stamp.nsec))
            );
            if let Some(stamp) = captured.stamp {
                assert!((0..1_000_000_000).contains(&stamp.nsec));
            }
            assert_eq!(value.data.len(), captured.data.len());
            assert!(captured.data.len() <= 16);
            for (entry, expected) in value.data.iter().zip(captured.data) {
                assert_eq!(entry.key, expected.key);
                assert!(expected.key.len() <= 256 && expected.values.len() <= 8);
                assert!(expected.values.iter().all(|value| value.len() <= 1024));
                assert_eq!(
                    entry.value.iter().map(String::as_str).collect::<Vec<_>>(),
                    expected.values
                );
            }
            // Native present-empty Header is legal, and remains Some with b"".
            let rows = fields(captured.wire, &[1, 2]);
            assert_eq!(
                rows.iter().filter(|row| row.number == 1).count(),
                usize::from(captured.stamp.is_some())
            );
            assert_eq!(
                rows.iter().filter(|row| row.number == 2).count(),
                captured.data.len()
            );
        }
        _ => panic!("actual native optional Header presence"),
    }
}
fn native_vector(captured: &native::NativeVector3) -> Vector3d {
    let value = Vector3d::decode(captured.wire).unwrap();
    header(value.header.as_ref(), captured.header);
    assert_eq!([value.x, value.y, value.z].map(f64::to_bits), captured.xyz);
    assert!(
        captured
            .xyz
            .map(f64::from_bits)
            .iter()
            .all(|value| value.is_finite())
    );
    nested(
        &fields(captured.wire, &[1, 2, 3, 4]),
        1,
        &optional_wire(captured.header, |value| value.wire),
    );
    value
}
fn native_rotation(captured: &native::NativeQuaternion) -> Quaternion {
    let value = Quaternion::decode(captured.wire).unwrap();
    header(value.header.as_ref(), captured.header);
    assert_eq!(
        [value.x, value.y, value.z, value.w].map(f64::to_bits),
        captured.xyzw
    );
    assert!(
        captured
            .xyzw
            .map(f64::from_bits)
            .iter()
            .all(|value| value.is_finite())
    );
    nested(
        &fields(captured.wire, &[1, 2, 3, 4, 5]),
        1,
        &optional_wire(captured.header, |value| value.wire),
    );
    value
}
fn native_pose(captured: &native::NativePose) -> Pose {
    let value = Pose::decode(captured.wire).unwrap();
    header(value.header.as_ref(), captured.header);
    assert_eq!(value.name, captured.name);
    assert_eq!(value.id, captured.id);
    assert_eq!(value.position.is_some(), captured.position.is_some());
    assert_eq!(value.orientation.is_some(), captured.orientation.is_some());
    if let Some(expected) = captured.position {
        assert_eq!(value.position.as_ref().unwrap(), &native_vector(expected));
    }
    if let Some(expected) = captured.orientation {
        assert_eq!(
            value.orientation.as_ref().unwrap(),
            &native_rotation(expected)
        );
    }
    let rows = fields(captured.wire, &[1, 2, 3, 4, 5]);
    nested(
        &rows,
        1,
        &optional_wire(captured.header, |value| value.wire),
    );
    nested(
        &rows,
        4,
        &optional_wire(captured.position, |value| value.wire),
    );
    nested(
        &rows,
        5,
        &optional_wire(captured.orientation, |value| value.wire),
    );
    value
}
fn native_axis(captured: &native::NativeAxis) -> Axis {
    let value = Axis::decode(captured.wire).unwrap();
    header(value.header.as_ref(), captured.header);
    assert_eq!(value.xyz.is_some(), captured.xyz.is_some());
    if let Some(expected) = captured.xyz {
        assert_eq!(value.xyz.as_ref().unwrap(), &native_vector(expected));
    }
    for (actual, expected) in [
        value.limit_lower,
        value.limit_upper,
        value.damping,
        value.position,
        value.velocity,
    ]
    .into_iter()
    .zip([
        captured.lower,
        captured.upper,
        captured.damping,
        captured.position,
        captured.velocity,
    ]) {
        assert_eq!(actual.to_bits(), expected);
    }
    let lower = f64::from_bits(captured.lower);
    let upper = f64::from_bits(captured.upper);
    assert!(
        !lower.is_nan()
            && lower != f64::INFINITY
            && !upper.is_nan()
            && upper != f64::NEG_INFINITY
            && lower <= upper
    );
    assert!(
        [captured.damping, captured.position, captured.velocity]
            .map(f64::from_bits)
            .iter()
            .all(|value| value.is_finite())
    );
    let rows = fields(captured.wire, &[1, 2, 3, 4, 7, 10, 11]);
    nested(
        &rows,
        1,
        &optional_wire(captured.header, |value| value.wire),
    );
    nested(&rows, 2, &optional_wire(captured.xyz, |value| value.wire));
    assert_eq!(value.force.to_bits(), 0.0_f64.to_bits()); // No force field or measurement.
    value
}
fn native_joint(captured: &native::NativeJointMessage) -> Joint {
    let value = Joint::decode(captured.wire).unwrap();
    header(value.header.as_ref(), captured.header);
    assert_eq!(value.name, captured.name);
    assert_eq!(value.id, captured.id);
    assert_eq!(value.r#type, captured.kind);
    assert_eq!(value.parent, captured.parent);
    assert_eq!(value.parent_id, captured.parent_id);
    assert_eq!(value.child, captured.child);
    assert_eq!(value.child_id, captured.child_id);
    assert_eq!(value.pose.is_some(), captured.pose.is_some());
    assert_eq!(value.axis1.is_some(), captured.axis1.is_some());
    assert_eq!(value.axis2.is_some(), captured.axis2.is_some());
    if let Some(expected) = captured.pose {
        assert_eq!(value.pose.as_ref().unwrap(), &native_pose(expected));
    }
    if let Some(expected) = captured.axis1 {
        assert_eq!(value.axis1.as_ref().unwrap(), &native_axis(expected));
    }
    if let Some(expected) = captured.axis2 {
        assert_eq!(value.axis2.as_ref().unwrap(), &native_axis(expected));
    }
    let rows = fields(captured.wire, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);
    nested(
        &rows,
        1,
        &optional_wire(captured.header, |value| value.wire),
    );
    nested(&rows, 9, &optional_wire(captured.pose, |value| value.wire));
    nested(
        &rows,
        10,
        &optional_wire(captured.axis1, |value| value.wire),
    );
    nested(
        &rows,
        11,
        &optional_wire(captured.axis2, |value| value.wire),
    );
    value
}
fn native_model(captured: &native::NativeModelMessage) -> Model {
    let value = Model::decode(captured.wire).unwrap();
    header(value.header.as_ref(), captured.header);
    assert_eq!(value.name, captured.name);
    assert_eq!(value.id, captured.id);
    assert_eq!(value.is_static, captured.is_static);
    assert_eq!(value.deleted, captured.deleted);
    assert_eq!(value.self_collide, captured.self_collide);
    assert_eq!(value.pose.is_some(), captured.pose.is_some());
    if let Some(expected) = captured.pose {
        assert_eq!(value.pose.as_ref().unwrap(), &native_pose(expected));
    }
    assert_eq!(value.joint.len(), captured.joints.len());
    for (actual, expected) in value.joint.iter().zip(captured.joints) {
        assert_eq!(actual, &native_joint(expected));
    }
    let rows = fields(captured.wire, &[1, 2, 3, 5, 6]);
    nested(
        &rows,
        1,
        &optional_wire(captured.header, |value| value.wire),
    );
    nested(&rows, 5, &optional_wire(captured.pose, |value| value.wire));
    nested(
        &rows,
        6,
        &captured
            .joints
            .iter()
            .map(|value| value.wire)
            .collect::<Vec<_>>(),
    );
    value
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
fn configuration(value: &native::NativeConfiguration) -> JointStatePublication {
    assert!(
        value.owner < 8
            && value.world_entity != 0
            && value.model_entity != 0
            && value.world_entity != value.model_entity
    );
    assert!(u32::try_from(value.model_entity).is_ok());
    assert!(!value.world_name.is_empty() && !value.model_name.is_empty());
    assert_eq!(
        value.full_name,
        format!("{}::{}", value.world_name, value.model_name)
    );
    assert!(value.resolved_topic.starts_with('/') && !value.resolved_topic.contains(' '));
    assert!(value.requested_topic.is_empty() || value.requested_topic == value.resolved_topic);
    if value.requested_topic.is_empty() {
        assert_eq!(
            value.resolved_topic,
            format!(
                "/world/{}/model/{}/joint_state",
                value.world_name, value.model_name
            )
        );
    }
    let rate = f64::from_bits(value.rate);
    assert!(rate.is_finite() && rate >= 0.0);
    assert_eq!(
        xml_value(value.xml, "update_rate")
            .parse::<f64>()
            .unwrap()
            .to_bits(),
        value.rate
    );
    assert!(value.xml.contains("gz::sim::systems::JointStatePublisher"));
    if value.requested_topic.is_empty() {
        assert!(!value.xml.contains("<topic>"));
    } else {
        assert_eq!(xml_value(value.xml, "topic"), value.requested_topic);
    }
    assert!(value.requested_names.iter().all(|name| !name.is_empty()));
    assert_eq!(
        value.xml.matches("<joint_name>").count(),
        value.requested_names.len()
    );
    for name in value.requested_names {
        assert!(
            value
                .xml
                .contains(&format!("<joint_name>{name}</joint_name>"))
        );
    }
    assert_eq!(pose_words(finite_pose(value.model_pose)), value.model_pose);
    assert_eq!(value.joints.len(), 2);
    assert!(
        value
            .joints
            .windows(2)
            .all(|pair| pair[0].entity < pair[1].entity)
    );
    let mut joints = Vec::new();
    for joint in value.joints {
        assert!(
            joint.entity != 0
                && joint.entity != value.model_entity
                && joint.parent_entity == value.model_entity
        );
        assert!(u32::try_from(joint.entity).is_ok());
        assert!(
            !joint.name.is_empty() && !joint.parent_link.is_empty() && !joint.child_link.is_empty()
        );
        let pose = finite_pose(joint.local_pose);
        let axis = joint.axis.map(f64::from_bits);
        assert!(axis.iter().all(|value| value.is_finite()));
        assert!(axis.iter().any(|value| *value != 0.0));
        let lower = f64::from_bits(joint.lower);
        let upper = f64::from_bits(joint.upper);
        assert!(
            !lower.is_nan()
                && lower != f64::INFINITY
                && !upper.is_nan()
                && upper != f64::NEG_INFINITY
                && lower <= upper
        );
        let damping = f64::from_bits(joint.damping);
        assert!(damping.is_finite() && damping >= 0.0);
        assert_eq!(
            joint.selected,
            value.requested_names.is_empty() || value.requested_names.contains(&joint.name)
        );
        if joint.selected {
            joints.push(JointStateJointPublication {
                kind: gz_sim_scene::JointStateKind::Revolute,
                id: joint.entity,
                name: joint.name.into(),
                parent: joint.parent_link.into(),
                child: joint.child_link.into(),
                pose: sdformat::Transform {
                    translation: pose.translation,
                    rotation: pose.rotation,
                },
                axis,
                lower,
                upper,
                damping,
            });
        }
    }
    assert!(!joints.is_empty());
    JointStatePublication {
        model_name: value.model_name.into(),
        topic: value.resolved_topic.into(),
        joints,
    }
}
fn input(value: &native::NativeInput, config: &native::NativeConfiguration) {
    assert_eq!(value.owner, config.owner);
    assert!(value.call_index < 10);
    // Complete captured UpdateInfo input ledger, not a private-clock or solver oracle.
    let schedule = [
        (0, 0, 0, true),
        (1_000_000, 1_000_000, 1_000_000, false),
        (10_000_000, 9_000_000, 10_000_000, false),
        (10_000_000, 0, 10_000_000, true),
        (11_000_000, 1_000_000, 11_000_000, false),
        (20_000_000, 9_000_000, 20_000_000, false),
        (20_000_000, 0, 20_000_000, true),
        (15_000_000, 0, 20_000_000, false),
        (30_000_000, 15_000_000, 35_000_000, false),
        (30_000_000, 0, 35_000_000, true),
    ];
    assert_eq!(
        (value.sim_ns, value.dt_ns, value.real_ns, value.paused),
        schedule[value.call_index]
    );
    assert_eq!(value.iterations, u64::try_from(value.call_index).unwrap());
    assert_eq!(value.model_pose, config.model_pose);
    assert_eq!(value.model_pose_getback, value.model_pose);
    assert_eq!(value.joints.len(), config.joints.len());
    for (joint, expected) in value.joints.iter().zip(config.joints) {
        assert_eq!(joint.entity, expected.entity);
        assert_eq!(joint.parent_entity, expected.parent_entity);
        assert_eq!(joint.local_pose, expected.local_pose);
        assert_eq!(joint.local_pose_getback, joint.local_pose);
        assert_eq!(joint.position.len(), 1);
        assert_eq!(joint.velocity.len(), 1);
        assert_eq!(joint.position_getback, joint.position);
        assert_eq!(joint.velocity_getback, joint.velocity);
        assert!(joint.force.is_empty() && joint.force_getback.is_empty());
        assert!(
            joint
                .position
                .iter()
                .chain(joint.velocity)
                .all(|word| f64::from_bits(*word).is_finite())
        );
    }
}
fn semantic_pose(actual: &Pose, captured: &native::NativePose) {
    // Encoding presence and raw bytes are checked on native data separately.
    let position = actual.position.as_ref().unwrap();
    let expected = captured.position.unwrap();
    assert_eq!(
        [position.x, position.y, position.z].map(f64::to_bits),
        expected.xyz
    );
    let rotation = actual.orientation.as_ref().unwrap();
    let expected = captured.orientation.unwrap();
    assert_eq!(
        [rotation.x, rotation.y, rotation.z, rotation.w].map(f64::to_bits),
        expected.xyzw
    );
    assert_eq!(actual.name, captured.name);
    assert_eq!(actual.id, captured.id);
}

#[test]
fn actual_native_joint_state_multiplicity_preserves_all_66_models_and_108_joint_records() {
    let mut totals = [0_usize; 9];
    for case in [&native::NATIVE_PAIR, &native::NATIVE_EIGHT] {
        let (configs, inputs, observations, fences, summary) = *case;
        assert!(configs.len() == 2 || configs.len() == 8);
        assert_eq!(summary.owners, configs.len());
        assert_eq!(summary.configurations, configs.len());
        assert_eq!(summary.global_call_groups, 10);
        assert_eq!(summary.input_rows, inputs.len());
        assert_eq!(summary.direct_post_update_calls, inputs.len());
        assert_eq!(summary.fence_rows, fences.len());
        assert_eq!(summary.observed_models, observations.len());
        assert!(
            observations
                .windows(2)
                .all(|pair| (pair[0].call_index, pair[0].owner)
                    < (pair[1].call_index, pair[1].owner))
        );
        assert_eq!(
            summary.observed_joints,
            observations
                .iter()
                .map(|row| row.message.joints.len())
                .sum::<usize>()
        );
        assert_eq!(
            summary.raw_rows,
            configs.len() + inputs.len() + observations.len() + fences.len() + 1
        );
        let counts = [
            summary.owners,
            summary.configurations,
            summary.global_call_groups,
            summary.input_rows,
            summary.direct_post_update_calls,
            summary.fence_rows,
            summary.observed_models,
            summary.observed_joints,
            summary.raw_rows,
        ];
        for (total, count) in totals.iter_mut().zip(counts) {
            *total += count;
        }
        let metadata = configs
            .iter()
            .enumerate()
            .map(|(owner, config)| {
                assert_eq!(config.owner, owner);
                configuration(config)
            })
            .collect::<Vec<_>>();
        let mut seen_models = BTreeSet::new();
        let mut seen_joints = BTreeSet::new();
        for config in configs {
            assert!(seen_models.insert(config.model_entity));
            for joint in config.joints {
                assert!(seen_joints.insert(joint.entity));
            }
        }
        assert!(seen_models.is_disjoint(&seen_joints));
        assert_eq!(inputs.len(), configs.len() * 10);
        assert_eq!(fences.len(), inputs.len());
        let mut previous_arrival = vec![0_u64; configs.len()];
        for (index, value) in inputs.iter().enumerate() {
            assert_eq!(value.call_index, index / configs.len());
            assert_eq!(value.owner, index % configs.len());
            let config = &configs[value.owner];
            input(value, config);
            let fence = &fences[index];
            assert_eq!(fence.owner, value.owner);
            assert_eq!(fence.call_index, value.call_index);
            assert_eq!(fence.initial_lazy_publication, value.call_index == 0);
            if !fence.initial_lazy_publication {
                assert!(fence.topic_info_ok);
                assert_eq!(fence.publishers_before, 1);
                assert_eq!(fence.subscribers_before, 1);
            } else {
                assert!(fence.publishers_before <= 1 && fence.subscribers_before <= 1);
            }
            assert_eq!(fence.arrivals_before, previous_arrival[value.owner]);
            assert!(fence.expected_outputs <= 1);
            assert_eq!(
                fence.arrivals_after,
                fence.arrivals_before + u64::from(fence.expected_outputs)
            );
            let matching = observations
                .iter()
                .filter(|row| row.owner == value.owner && row.call_index == value.call_index)
                .collect::<Vec<_>>();
            assert_eq!(
                matching.len(),
                usize::try_from(fence.expected_outputs).unwrap()
            );
            previous_arrival[value.owner] = fence.arrivals_after;
            for observed in matching {
                assert_eq!(observed.arrival_ordinal, fence.arrivals_before);
                let captured = &observed.message;
                let decoded = native_model(captured);
                assert_eq!(u64::from(captured.id), config.model_entity);
                assert_eq!(captured.name, config.model_name);
                let stamp = captured.header.unwrap().stamp.unwrap();
                assert_eq!(
                    i128::from(stamp.sec) * 1_000_000_000 + i128::from(stamp.nsec),
                    i128::from(value.sim_ns)
                );
                assert!(!captured.is_static && !captured.deleted && !captured.self_collide);
                assert_eq!(captured.joints.len(), metadata[value.owner].joints.len());
                let captured_pose = captured.pose.unwrap();
                let p = captured_pose.position.unwrap().xyz;
                let q = captured_pose.orientation.unwrap().xyzw;
                let sample_pose = finite_pose([p[0], p[1], p[2], q[0], q[1], q[2], q[3]]);
                let mut values = Vec::new();
                for (joint, meta) in captured.joints.iter().zip(&metadata[value.owner].joints) {
                    assert_eq!(u64::from(joint.id), meta.id);
                    assert_eq!(joint.name, meta.name);
                    assert_eq!(joint.parent, meta.parent);
                    assert_eq!(joint.child, meta.child);
                    let source = value
                        .joints
                        .iter()
                        .find(|row| row.entity == meta.id)
                        .unwrap();
                    let axis = joint.axis1.unwrap();
                    assert_eq!(axis.position, source.position[0]);
                    assert_eq!(axis.velocity, source.velocity[0]);
                    assert!(joint.axis2.is_none());
                    values.push(JointStateValue {
                        id: u64::from(joint.id),
                        position: f64::from_bits(axis.position),
                        velocity: f64::from_bits(axis.velocity),
                    });
                }
                let sample = JointStateSample {
                    time: Duration::from_nanos(u64::try_from(value.sim_ns).unwrap()),
                    // Epoch/revision are Rust-only encoder inputs, not captured native state.
                    epoch: 0,
                    revision: u64::try_from(value.call_index).unwrap() + 1,
                    pose: sample_pose,
                    joints: values,
                };
                let rust = packet(&metadata[value.owner], u64::from(captured.id), &sample).unwrap();
                assert_eq!(rust.id, decoded.id);
                assert_eq!(rust.name, decoded.name);
                assert_eq!(
                    rust.header
                        .as_ref()
                        .unwrap()
                        .stamp
                        .as_ref()
                        .map(|time| (time.sec, time.nsec)),
                    Some((stamp.sec, stamp.nsec))
                );
                semantic_pose(rust.pose.as_ref().unwrap(), captured_pose);
                assert_eq!(rust.joint.len(), captured.joints.len());
                for (actual, expected) in rust.joint.iter().zip(captured.joints) {
                    assert_eq!(actual.id, expected.id);
                    assert_eq!(actual.name, expected.name);
                    assert_eq!(actual.parent, expected.parent);
                    assert_eq!(actual.child, expected.child);
                    semantic_pose(actual.pose.as_ref().unwrap(), expected.pose.unwrap());
                    let axis = actual.axis1.as_ref().unwrap();
                    let expected = expected.axis1.unwrap();
                    assert_eq!(
                        [
                            axis.limit_lower,
                            axis.limit_upper,
                            axis.damping,
                            axis.position,
                            axis.velocity
                        ]
                        .map(f64::to_bits),
                        [
                            expected.lower,
                            expected.upper,
                            expected.damping,
                            expected.position,
                            expected.velocity
                        ]
                    );
                    let vector = axis.xyz.as_ref().unwrap();
                    assert_eq!(
                        [vector.x, vector.y, vector.z].map(f64::to_bits),
                        expected.xyz.unwrap().xyz
                    );
                    assert!(actual.axis2.is_none());
                    assert_eq!(axis.force.to_bits(), 0.0_f64.to_bits());
                }
                // Native wire was decoded and its exact nested slices checked above.
                // Rust packet semantics are compared here; full Rust canonical bytes
                // and real assigned IDs remain the independent maintained live peer proof.
            }
        }
        assert_eq!(
            previous_arrival.iter().sum::<u64>(),
            u64::try_from(observations.len()).unwrap()
        );
    }
    assert_eq!(totals, [10, 10, 20, 100, 100, 100, 66, 108, 278]);
}
