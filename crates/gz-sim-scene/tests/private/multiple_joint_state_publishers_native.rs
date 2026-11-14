// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Controlled native q/v inputs at the unchanged publication boundary.
//! The graph, backend handles and fixed poses come from the genuine loader.
//! This does not claim Physics gathering, solver motion or native lifecycle parity.
use super::*;
use std::collections::BTreeSet;

mod native {
    include!("../fixtures/m57_native_multiple_joint_state_publishers.rs");
}

fn pose_bits(value: WorldPose) -> [u64; 7] {
    [
        value.translation[0].to_bits(),
        value.translation[1].to_bits(),
        value.translation[2].to_bits(),
        value.rotation[0].to_bits(),
        value.rotation[1].to_bits(),
        value.rotation[2].to_bits(),
        value.rotation[3].to_bits(),
    ]
}

fn captured_pose(value: &native::NativePose) -> [u64; 7] {
    let p = value.position.expect("captured position presence").xyz;
    let q = value
        .orientation
        .expect("captured orientation presence")
        .xyzw;
    assert!(
        p.into_iter()
            .chain(q)
            .all(|v| f64::from_bits(v).is_finite())
    );
    [p[0], p[1], p[2], q[0], q[1], q[2], q[3]]
}

#[derive(Default)]
struct Occupancy {
    models: usize,
    joints: usize,
    poses: usize,
    axes: usize,
    vectors: usize,
    quaternions: usize,
    wire_bytes: usize,
}

fn wire(bytes: &[u8], fields: &mut Occupancy) {
    assert!(bytes.len() <= 65536);
    fields.wire_bytes += bytes.len();
}

fn header(value: Option<&native::NativeHeader>, fields: &mut Occupancy) {
    if let Some(value) = value {
        assert!(value.wire.len() <= 4096);
        wire(value.wire, fields);
        if let Some(stamp) = value.stamp {
            assert!((0..1_000_000_000).contains(&stamp.nsec));
            assert!(stamp.sec >= 0);
        }
        assert!(value.data.len() <= 16);
        for datum in value.data {
            assert!(datum.key.len() <= 1024 && datum.values.len() <= 16);
            assert!(datum.values.iter().all(|value| value.len() <= 1024));
        }
    }
}

fn vector(value: &native::NativeVector3, fields: &mut Occupancy) {
    fields.vectors += 1;
    assert!(
        value
            .xyz
            .map(f64::from_bits)
            .iter()
            .all(|value| value.is_finite())
    );
    header(value.header, fields);
    wire(value.wire, fields);
}

fn pose_fields(value: &native::NativePose, fields: &mut Occupancy) {
    fields.poses += 1;
    assert_eq!((value.name, value.id), ("", 0));
    header(value.header, fields);
    wire(value.wire, fields);
    vector(value.position.unwrap(), fields);
    let orientation = value.orientation.unwrap();
    fields.quaternions += 1;
    assert!(
        orientation
            .xyzw
            .map(f64::from_bits)
            .iter()
            .all(|value| value.is_finite())
    );
    header(orientation.header, fields);
    wire(orientation.wire, fields);
    let pose = captured_pose(value).map(f64::from_bits);
    assert!((pose[3..].iter().map(|value| value * value).sum::<f64>() - 1.0).abs() < 1e-12);
}

fn model_fields(value: &native::NativeModelMessage, time_ns: i64, fields: &mut Occupancy) {
    fields.models += 1;
    assert!(!value.is_static && !value.deleted && !value.self_collide);
    assert!(!value.wire.is_empty());
    let stamp = value.header.unwrap().stamp.unwrap();
    assert_eq!(
        stamp.sec.checked_mul(1_000_000_000).unwrap() + i64::from(stamp.nsec),
        time_ns
    );
    header(value.header, fields);
    wire(value.wire, fields);
    pose_fields(value.pose.unwrap(), fields);
    for joint in value.joints {
        fields.joints += 1;
        assert_eq!((joint.kind, joint.parent_id, joint.child_id), (0, 0, 0));
        assert!(matches!(joint.parent, "world" | "base"));
        assert!(matches!(joint.child, "base" | "tip"));
        header(joint.header, fields);
        wire(joint.wire, fields);
        pose_fields(joint.pose.unwrap(), fields);
        let axis = joint.axis1.unwrap();
        fields.axes += 1;
        assert!(joint.axis2.is_none());
        header(axis.header, fields);
        wire(axis.wire, fields);
        vector(axis.xyz.unwrap(), fields);
        let lower = f64::from_bits(axis.lower);
        let upper = f64::from_bits(axis.upper);
        assert!(lower.is_finite() || lower == f64::NEG_INFINITY);
        assert!(upper.is_finite() || upper == f64::INFINITY);
        assert!(lower <= upper);
        assert!(
            [axis.damping, axis.position, axis.velocity]
                .map(f64::from_bits)
                .iter()
                .all(|value| value.is_finite())
        );
    }
}

fn configuration_fields(config: &native::NativeConfiguration) {
    let document = roxmltree::Document::parse(config.xml).unwrap();
    let plugin = document.root_element();
    assert_eq!(plugin.tag_name().name(), "plugin");
    assert_eq!(
        plugin.attribute("name"),
        Some("gz::sim::systems::JointStatePublisher")
    );
    assert_eq!(
        plugin.attribute("filename"),
        Some("gz-sim-joint-state-publisher-system")
    );
    let rates = plugin
        .children()
        .filter(|node| node.has_tag_name("update_rate"))
        .collect::<Vec<_>>();
    assert_eq!(rates.len(), 1);
    assert_eq!(
        rates[0]
            .text()
            .unwrap()
            .trim()
            .parse::<f64>()
            .unwrap()
            .to_bits(),
        config.rate
    );
    assert!(
        [0.0f64, 50.0, 100.0]
            .map(f64::to_bits)
            .contains(&config.rate)
    );
    let topics = plugin
        .children()
        .filter(|node| node.has_tag_name("topic"))
        .collect::<Vec<_>>();
    if config.requested_topic.is_empty() {
        assert!(topics.is_empty());
        assert_eq!(
            config.resolved_topic,
            format!(
                "/world/{}/model/{}/joint_state",
                config.world_name, config.model_name
            )
        );
    } else {
        assert_eq!(topics.len(), 1);
        assert_eq!(topics[0].text().unwrap().trim(), config.requested_topic);
        assert_eq!(config.resolved_topic, config.requested_topic);
    }
    let names = plugin
        .children()
        .filter(|node| node.has_tag_name("joint_name"))
        .map(|node| node.text().unwrap().trim())
        .collect::<Vec<_>>();
    assert_eq!(names.as_slice(), config.requested_names);
    for joint in config.joints {
        assert_eq!(
            joint.selected,
            names.is_empty() || names.contains(&joint.name)
        );
        // These are immutable captured component inputs, including metadata
        // outside the genuine loader's narrower physical domain.
        let captured_limits = match joint.name {
            "base_hinge" => (-2.0f64, 2.0f64, 0.125f64),
            "tip_hinge" => (f64::NEG_INFINITY, f64::INFINITY, 0.0f64),
            _ => panic!("unexpected captured joint"),
        };
        assert_eq!(
            (joint.lower, joint.upper, joint.damping),
            (
                captured_limits.0.to_bits(),
                captured_limits.1.to_bits(),
                captured_limits.2.to_bits()
            )
        );
    }
}

fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|node| node.get::<Name>().is_some_and(|value| value.0 == name))
        .expect("genuine loaded entity")
        .id()
}

fn source(configurations: &[native::NativeConfiguration]) -> String {
    let original = include_str!("../fixtures/m18_joint_state.sdf");
    let begin = original.find("  <model ").unwrap();
    let end = original.rfind("</model>").unwrap() + "</model>".len();
    let mut models = String::new();
    for (owner, config) in configurations.iter().enumerate() {
        assert_eq!(config.owner, owner);
        assert_eq!(config.model_name, format!("arm{owner}"));
        assert_eq!(
            config.full_name,
            format!("{}::{}", config.world_name, config.model_name)
        );
        assert_eq!(config.joints.len(), 2);
        let mut model = original[begin..end]
            .replace("name=\"arm\"", &format!("name=\"{}\"", config.model_name))
            .replace(
                "<pose>0 0 1 0 0 0</pose>",
                &format!("<pose>{} {} 1 0 0 0</pose>", 1 + owner, 4 * owner),
            );
        let plugin_begin = model.find("<plugin ").unwrap();
        let plugin_end = model.rfind("</plugin>").unwrap() + "</plugin>".len();
        model.replace_range(plugin_begin..plugin_end, config.xml);
        // Genuine loaded graphs keep the existing supported unbounded,
        // zero-dynamics joint domain. Captured component metadata is checked
        // separately against native records, never installed into Physics.
        models.push_str(&model);
    }
    let world = configurations[0].world_name;
    assert!(
        configurations
            .iter()
            .all(|config| config.world_name == world)
    );
    format!(
        "<sdf version='1.12'><world name='{world}'><gravity>0 0 0</gravity><physics name='step' type='ignored'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}

#[derive(Debug, PartialEq)]
struct Reading {
    time: Duration,
    epoch: u64,
    revision: u64,
    pose: [u64; 7],
    joints: Vec<(u64, u64, u64)>,
}

fn reading(sim: &Simulation, entity: Entity) -> Option<Reading> {
    sim.world()
        .get::<JointStateSample>(entity)
        .map(|value| Reading {
            time: value.time,
            epoch: value.epoch,
            revision: value.revision,
            pose: pose_bits(value.pose),
            joints: value
                .joints
                .iter()
                .map(|joint| (joint.id, joint.position.to_bits(), joint.velocity.to_bits()))
                .collect(),
        })
}

#[test]
fn captured_pair_and_eight_joint_inputs_match_loaded_owner_samples_and_all_native_models() {
    let mut totals = [0usize; 4];
    let mut fields = Occupancy::default();
    for (case, owners) in [
        (native::NATIVE_PAIR, 2usize),
        (native::NATIVE_EIGHT, 8usize),
    ] {
        let (configs, inputs, observations, fences, summary) = case;
        assert_eq!(configs.len(), owners);
        assert_eq!(
            (
                summary.owners,
                summary.configurations,
                summary.global_call_groups
            ),
            (owners, owners, 10)
        );
        assert_eq!(
            (
                summary.input_rows,
                summary.direct_post_update_calls,
                summary.fence_rows
            ),
            (owners * 10, owners * 10, owners * 10)
        );
        assert_eq!(inputs.len(), summary.input_rows);
        assert_eq!(fences.len(), summary.fence_rows);
        assert_eq!(observations.len(), summary.observed_models);
        assert_eq!(
            summary.raw_rows,
            configs.len() + inputs.len() + observations.len() + fences.len() + 1
        );
        assert_eq!(
            observations
                .iter()
                .map(|row| row.message.joints.len())
                .sum::<usize>(),
            summary.observed_joints
        );
        let xml = source(configs);
        let axis = "<axis><xyz>0 0 1</xyz></axis>";
        assert_eq!(xml.matches(axis).count(), owners * 2);
        // Preserve the closed physical domain with separate typed negatives:
        // finite bounds fail limits, and otherwise-unbounded damping fails
        // dynamics. Neither rejected graph supplies the numeric sampler proof.
        let finite_limits = xml.replacen(
            axis,
            "<axis><xyz>0 0 1</xyz><limit><lower>-2</lower><upper>2</upper></limit></axis>",
            1,
        );
        let nonzero_damping = xml.replacen(
            axis,
            "<axis><xyz>0 0 1</xyz><dynamics><damping>0.125</damping></dynamics></axis>",
            1,
        );
        for (candidate, expected_feature) in [
            (finite_limits.as_str(), "joint limits"),
            (nonzero_damping.as_str(), "joint dynamics"),
        ] {
            match load_world_with_sdf_step(
                candidate,
                "m57-native-physical-domain-refusal.sdf",
                configs[0].world_name,
                &sdformat::NoResources,
                PhysicsEngine::Rapier,
            ) {
                Err(LoadError::Unsupported { entity, feature }) => {
                    assert_eq!(entity, format!("{}::base_hinge", configs[0].full_name));
                    assert_eq!(feature, expected_feature);
                }
                _ => panic!("unsupported physical metadata must remain refused"),
            }
        }
        let mut sim = load_world_with_sdf_step(
            &xml,
            "m57-native-inputs.sdf",
            configs[0].world_name,
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let entities = configs
            .iter()
            .map(|config| named(&sim, config.full_name))
            .collect::<Vec<_>>();
        let mut native_ids = BTreeSet::new();
        let mut topics = BTreeSet::new();
        for (owner, config) in configs.iter().enumerate() {
            configuration_fields(config);
            assert!(config.world_entity > 0 && config.model_entity > 0);
            assert!(native_ids.insert(config.model_entity));
            assert!(topics.insert(config.resolved_topic));
            let entity = entities[owner];
            assert_eq!(
                pose_bits(sim.world().get::<LocalPose>(entity).unwrap().0),
                config.model_pose
            );
            let publication = sim.world().get::<JointStatePublication>(entity).unwrap();
            assert_eq!(publication.model_name, config.model_name);
            assert_eq!(publication.topic, config.resolved_topic);
            assert_eq!(
                publication.joints.len(),
                config.joints.iter().filter(|joint| joint.selected).count()
            );
            for joint in config.joints {
                assert!(joint.entity > 0 && native_ids.insert(joint.entity));
                assert_eq!(joint.parent_entity, config.model_entity);
                let node = named(&sim, &format!("{}::{}", config.full_name, joint.name));
                assert_eq!(
                    sim.world().get::<EntityKind>(node),
                    Some(&EntityKind::Joint)
                );
                assert_eq!(sim.world().get::<ChildOf>(node).unwrap().parent(), entity);
                assert!(sim.world().get::<JointHandle>(node).is_some());
                let loaded = &sim.world().get::<JointDefinition>(node).unwrap().0;
                assert_eq!(loaded.kind, sdformat::JointKind::Revolute);
                assert_eq!(
                    (
                        loaded.name.as_str(),
                        loaded.parent.as_str(),
                        loaded.child.as_str()
                    ),
                    (joint.name, joint.parent_link, joint.child_link)
                );
                assert_eq!(loaded.pose.relative_to, loaded.child);
                let construction_pose: WorldPose = loaded.pose.transform.into();
                assert_eq!(pose_bits(construction_pose), joint.local_pose);
                let axis = loaded.axis.as_ref().unwrap();
                assert_eq!(axis.xyz.map(f64::to_bits), joint.axis);
                assert!(axis.expressed_in.is_empty() && loaded.axis2.is_none());
                assert_eq!(
                    (
                        axis.lower.to_bits(),
                        axis.upper.to_bits(),
                        axis.dynamics.damping.to_bits()
                    ),
                    (
                        f64::NEG_INFINITY.to_bits(),
                        f64::INFINITY.to_bits(),
                        0.0f64.to_bits()
                    )
                );
                if joint.selected {
                    let metadata = publication
                        .joints
                        .iter()
                        .find(|value| value.name == joint.name)
                        .unwrap();
                    assert_eq!(
                        (metadata.parent.as_str(), metadata.child.as_str()),
                        (joint.parent_link, joint.child_link)
                    );
                    assert_eq!(metadata.axis.map(f64::to_bits), joint.axis);
                    assert_eq!(
                        (
                            metadata.lower.to_bits(),
                            metadata.upper.to_bits(),
                            metadata.damping.to_bits()
                        ),
                        (
                            f64::NEG_INFINITY.to_bits(),
                            f64::INFINITY.to_bits(),
                            0.0f64.to_bits()
                        )
                    );
                    let metadata_pose: WorldPose = metadata.pose.into();
                    assert_eq!(pose_bits(metadata_pose), joint.local_pose);
                    assert_eq!(
                        metadata.id,
                        sim.world().get::<EntityId>(node).unwrap().get()
                    );
                }
            }
        }
        let mut arrivals = vec![0u64; owners];
        for call in 0..summary.global_call_groups {
            let group = inputs
                .iter()
                .filter(|input| input.call_index == call)
                .collect::<Vec<_>>();
            assert_eq!(group.len(), owners);
            let first = group[0];
            let info = UpdateInfo {
                start_time: Duration::from_nanos(
                    u64::try_from(first.sim_ns.checked_sub(first.dt_ns).unwrap()).unwrap(),
                ),
                sim_time: Duration::from_nanos(u64::try_from(first.sim_ns).unwrap()),
                dt: Duration::from_nanos(u64::try_from(first.dt_ns).unwrap()),
                iteration: first.iterations,
                epoch: 57,
                paused: first.paused,
            };
            let before = entities
                .iter()
                .map(|entity| reading(&sim, *entity))
                .collect::<Vec<_>>();
            // Install EVERY owner's genuine captured public sample inputs before
            // ONE unchanged family sampler call. Never write a pose or backend.
            for (owner, input) in group.iter().enumerate() {
                let config = &configs[owner];
                assert_eq!(input.owner, owner);
                assert_eq!(
                    (
                        input.sim_ns,
                        input.dt_ns,
                        input.real_ns,
                        input.iterations,
                        input.paused
                    ),
                    (
                        first.sim_ns,
                        first.dt_ns,
                        first.real_ns,
                        first.iterations,
                        first.paused
                    )
                );
                assert_eq!(input.model_pose, input.model_pose_getback);
                assert_eq!(input.model_pose, config.model_pose);
                assert_eq!(
                    pose_bits(sim.world().get::<LocalPose>(entities[owner]).unwrap().0),
                    config.model_pose
                );
                assert_eq!(input.joints.len(), config.joints.len());
                for (joint, definition) in input.joints.iter().zip(config.joints) {
                    assert_eq!(
                        (joint.entity, joint.parent_entity),
                        (definition.entity, config.model_entity)
                    );
                    assert_eq!(joint.local_pose, joint.local_pose_getback);
                    assert_eq!(joint.local_pose, definition.local_pose);
                    assert_eq!(joint.position, joint.position_getback);
                    assert_eq!(joint.velocity, joint.velocity_getback);
                    assert_eq!((joint.position.len(), joint.velocity.len()), (1, 1));
                    assert!(joint.force.is_empty() && joint.force_getback.is_empty());
                    let position = f64::from_bits(joint.position[0]);
                    let velocity = f64::from_bits(joint.velocity[0]);
                    assert!(position.is_finite() && velocity.is_finite());
                    let entity = named(&sim, &format!("{}::{}", config.full_name, definition.name));
                    sim.world_mut().entity_mut(entity).insert(JointSample {
                        state: gz_physics::JointState { position, velocity },
                        time: info.sim_time,
                        epoch: info.epoch,
                    });
                }
            }
            sim.world_mut().insert_resource(info);
            sample(sim.world_mut());
            assert!(sim.world().resource::<StepFault>().error().is_none());
            for (owner, entity) in entities.iter().enumerate() {
                let matching = fences
                    .iter()
                    .filter(|row| row.call_index == call && row.owner == owner)
                    .collect::<Vec<_>>();
                assert_eq!(matching.len(), 1);
                let fence = matching[0];
                let output = observations
                    .iter()
                    .filter(|row| row.call_index == call && row.owner == owner)
                    .collect::<Vec<_>>();
                assert_eq!(output.len(), fence.expected_outputs as usize);
                assert_eq!(fence.arrivals_before, arrivals[owner]);
                arrivals[owner] += output.len() as u64;
                assert_eq!(fence.arrivals_after, arrivals[owner]);
                assert_eq!(fence.initial_lazy_publication, call == 0);
                if call != 0 {
                    assert!(fence.topic_info_ok);
                    assert_eq!((fence.publishers_before, fence.subscribers_before), (1, 1));
                }
                let actual = reading(&sim, *entity);
                if output.is_empty() {
                    assert_eq!(actual, before[owner]);
                    continue;
                }
                assert_eq!(output.len(), 1);
                let observation = output[0];
                assert_eq!(observation.arrival_ordinal, fence.arrivals_before);
                let value = actual.unwrap();
                assert_eq!(
                    (value.time, value.epoch, value.revision),
                    (info.sim_time, info.epoch, arrivals[owner])
                );
                let message = &observation.message;
                model_fields(message, first.sim_ns, &mut fields);
                let config = &configs[owner];
                assert_eq!(
                    (message.name, u64::from(message.id)),
                    (config.model_name, config.model_entity)
                );
                assert_eq!(value.pose, captured_pose(message.pose.unwrap()));
                assert_eq!(value.pose, config.model_pose);
                assert_eq!(value.joints.len(), message.joints.len());
                let publication = sim.world().get::<JointStatePublication>(*entity).unwrap();
                for joint in message.joints {
                    let definition = config
                        .joints
                        .iter()
                        .find(|row| row.name == joint.name)
                        .unwrap();
                    assert!(definition.selected);
                    assert_eq!(u64::from(joint.id), definition.entity);
                    let metadata = publication
                        .joints
                        .iter()
                        .find(|row| row.name == joint.name)
                        .unwrap();
                    let sample = value
                        .joints
                        .iter()
                        .find(|row| row.0 == metadata.id)
                        .unwrap();
                    let axis = joint.axis1.unwrap();
                    assert!(joint.axis2.is_none());
                    assert_eq!((sample.1, sample.2), (axis.position, axis.velocity));
                    assert_eq!(
                        (axis.lower, axis.upper, axis.damping),
                        (definition.lower, definition.upper, definition.damping)
                    );
                    assert_eq!(axis.xyz.unwrap().xyz, definition.axis);
                    assert_eq!(
                        (joint.parent, joint.child),
                        (definition.parent_link, definition.child_link)
                    );
                    assert_eq!(captured_pose(joint.pose.unwrap()), definition.local_pose);
                }
            }
        }
        totals[0] += configs.len();
        totals[1] += inputs.len();
        totals[2] += observations.len();
        totals[3] += summary.observed_joints;
    }
    assert_eq!(totals, [10, 100, 66, 108]);
    assert_eq!(
        (fields.models, fields.joints, fields.poses, fields.axes),
        (66, 108, 174, 108)
    );
    assert_eq!((fields.vectors, fields.quaternions), (282, 174));
    assert!(fields.wire_bytes > 0);
}
