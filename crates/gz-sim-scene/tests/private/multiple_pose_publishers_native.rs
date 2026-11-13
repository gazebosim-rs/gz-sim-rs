// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Controlled LOCAL input replay through the unchanged loaded PosePublisher sampler.
//! This is separate from the real force/joint-motor tests and from native wire order.
use super::*;
use gz_sim_core::entity_by_id;
use std::collections::{BTreeMap, BTreeSet};

macro_rules! native_fixture {
    () => {
        include!("../fixtures/m55_native_multiple_pose_publishers.rs");
    };
}
mod native {
    native_fixture!();
}

fn pose(bits: [u64; 7]) -> WorldPose {
    let values = bits.map(f64::from_bits);
    assert!(values.iter().all(|value| value.is_finite()));
    assert!((values[3..].iter().map(|value| value * value).sum::<f64>() - 1.0).abs() <= 1e-12);
    WorldPose {
        translation: [values[0], values[1], values[2]],
        rotation: [values[3], values[4], values[5], values[6]],
    }
}
fn bits(pose: WorldPose) -> [u64; 7] {
    [
        pose.translation[0],
        pose.translation[1],
        pose.translation[2],
        pose.rotation[0],
        pose.rotation[1],
        pose.rotation[2],
        pose.rotation[3],
    ]
    .map(f64::to_bits)
}
fn named(sim: &Simulation, name: &str) -> Entity {
    let entities = sim
        .world()
        .iter_entities()
        .filter(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .map(|entity| entity.id())
        .collect::<Vec<_>>();
    assert_eq!(entities.len(), 1);
    entities[0]
}
fn public_id(sim: &Simulation, entity: Entity) -> u64 {
    let id = sim.world().get::<EntityId>(entity).unwrap().get();
    assert_ne!(id, 0);
    assert_eq!(entity_by_id(sim.world(), id), Some(entity));
    id
}
fn validate_configuration(config: &native::NativeConfiguration, mode: &str, owner: usize) {
    assert_eq!(config.mode, mode);
    assert_eq!(config.owner as usize, owner);
    assert_ne!(config.native_entity, 0);
    assert_ne!(config.world_entity, 0);
    assert_ne!(config.native_entity, config.world_entity);
    assert_eq!(config.world_name, "default");
    assert_eq!(config.name, format!("rig{owner}"));
    assert_eq!(
        bits(pose(config.world_pose)),
        [0, 0, 0, 0, 0, 0, 1.0_f64.to_bits()]
    );
    assert!(config.topic.starts_with("/m55/native/"));
    assert!(
        config
            .topic
            .ends_with(format!("/{mode}/pose{owner}").as_str())
    );
    assert!(
        config
            .topic
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"_/.-".contains(&byte))
    );
    assert_eq!(config.static_topic, format!("{}_static", config.topic));
    assert_eq!(
        f64::from_bits(config.dynamic_rate),
        if owner.is_multiple_of(2) { 100.0 } else { 50.0 }
    );
    assert_eq!(
        f64::from_bits(config.static_rate),
        if owner.is_multiple_of(2) { 25.0 } else { 100.0 }
    );
    assert!(config.static_publisher && config.use_pose_vector_msg);
    assert_eq!(config.console_verbosity, 2);
    assert_eq!(config.joint_type, 6); // Pinned public sdf::JointType::REVOLUTE input.
    assert_eq!(
        (config.parent_link_name, config.child_link_name),
        ("base", "tip")
    );
    assert_eq!(config.entities.len(), 4);
    assert_eq!(config.entities[0].native_entity, config.native_entity);
    assert_eq!(
        config.model_canonical_link,
        config.entities[1].native_entity
    );
    let xml = std::str::from_utf8(config.sdf_xml).unwrap();
    let document = roxmltree::Document::parse(xml).unwrap();
    let plugin = document.root_element();
    assert_eq!(plugin.tag_name().name(), "plugin");
    assert_eq!(
        plugin.attribute("filename"),
        Some("direct-original-object-not-a-loader-input")
    );
    assert_eq!(
        plugin.attribute("name"),
        Some("gz::sim::systems::PosePublisher")
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
    assert_eq!(settings.len(), 11);
    for key in [
        "publish_link_pose",
        "publish_model_pose",
        "use_pose_vector_msg",
        "static_publisher",
    ] {
        assert!(matches!(settings[key], "true" | "1"));
    }
    for key in [
        "publish_nested_model_pose",
        "publish_visual_pose",
        "publish_collision_pose",
        "publish_sensor_pose",
    ] {
        assert!(matches!(settings[key], "false" | "0"));
    }
    assert_eq!(settings["topic"], config.topic);
    assert_eq!(
        settings["update_frequency"]
            .parse::<f64>()
            .unwrap()
            .to_bits(),
        config.dynamic_rate
    );
    assert_eq!(
        settings["static_update_frequency"]
            .parse::<f64>()
            .unwrap()
            .to_bits(),
        config.static_rate
    );
}
fn load_replay(configs: &[native::NativeConfiguration]) -> Simulation {
    // The captured XML's filename is a nonloader provenance label. Build an
    // ordinary admitted rig with the existing loader; never forge a blueprint.
    const RIG: &str = include_str!("../fixtures/m31_pose_static_publisher.sdf");
    let start = RIG.find("  <model name=\"rig\"").unwrap();
    let end = RIG.rfind("</model>").unwrap() + "</model>".len();
    let template = &RIG[start..end];
    let base_joint = template.find("    <joint name=\"base_hinge\"").unwrap();
    let tip_joint = template.find("    <joint name=\"tip_hinge\"").unwrap();
    let template = format!("{}{}", &template[..base_joint], &template[tip_joint..])
        .replace("name=\"tip_hinge\"", "name=\"hinge\"");
    let plugin_start = template.find("    <plugin ").unwrap();
    let plugin_end = template.find("</plugin>").unwrap() + "</plugin>".len();
    let models = configs.iter().map(|config| {
        let plugin = format!("<plugin filename='gz-sim-pose-publisher-system' name='gz::sim::systems::PosePublisher'><publish_link_pose>true</publish_link_pose><publish_model_pose>true</publish_model_pose><publish_nested_model_pose>false</publish_nested_model_pose><publish_visual_pose>false</publish_visual_pose><publish_collision_pose>false</publish_collision_pose><publish_sensor_pose>false</publish_sensor_pose><use_pose_vector_msg>true</use_pose_vector_msg><static_publisher>true</static_publisher><topic>{}</topic><update_frequency>{}</update_frequency><static_update_frequency>{}</static_update_frequency></plugin>", config.topic, f64::from_bits(config.dynamic_rate), f64::from_bits(config.static_rate));
        format!("{}{}{}", &template[..plugin_start], plugin, &template[plugin_end..])
            .replace("name=\"rig\"", &format!("name=\"{}\"", config.name))
    }).collect::<String>();
    let xml = format!("{}{}{}", &RIG[..start], models, &RIG[end..]);
    load_world_with_sdf_step(
        &xml,
        "m55-observed-local-inputs.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn current_sample(sim: &Simulation, owner: Entity, fixed: bool) -> Option<&PoseSample> {
    if fixed {
        sim.world()
            .get::<StaticPoseSample>(owner)
            .map(|sample| &sample.0)
    } else {
        sim.world().get::<PoseSample>(owner)
    }
}
type SampleLedger = (Duration, u64, u64, Vec<(u64, [u64; 7])>);
fn ledger(sample: &PoseSample) -> SampleLedger {
    (
        sample.time,
        sample.epoch,
        sample.revision,
        sample
            .poses
            .iter()
            .map(|frame| (frame.identity, bits(frame.pose)))
            .collect(),
    )
}

#[test]
fn actual_native_pose_multiplicity_matches_all_65_local_vectors_and_95_keyed_poses() {
    assert_eq!(native::CASES.len(), 2);
    let (mut configurations, mut input_rows, mut vectors, mut records) = (0, 0, 0, 0);
    let (mut header_bytes, mut pose_bytes, mut vector_bytes) = (0, 0, 0);
    for &(mode, configs, rows, summary) in native::CASES {
        let count = match mode {
            "pair" => 2,
            "eight" => 8,
            _ => panic!("unreviewed native mode"),
        };
        assert_eq!(configs.len(), count);
        assert_eq!(rows.len(), 4 * count);
        assert_eq!(summary.configuration_count as usize, count);
        assert_eq!(summary.direct_post_update_calls as usize, rows.len());
        assert_eq!(summary.input_rows as usize, rows.len());
        assert_eq!(
            summary.observed_vectors,
            if mode == "pair" { 13 } else { 52 }
        );
        assert_eq!(summary.canonical_packets, summary.observed_vectors);
        assert_eq!(summary.pose_records, if mode == "pair" { 19 } else { 76 });
        assert!(summary.original_object_linked_system_scope);
        assert!(!summary.original_server_or_plugin_loader_executed);
        assert!(!summary.native_physics_executed);
        assert!(!summary.native_demand_or_lifecycle_equivalence_claimed);
        assert!(!summary.global_rng_equivalence_claimed);
        assert!(!summary.old_native_modes_replayed);
        let mut native_ids = BTreeSet::new();
        let mut topics = BTreeSet::new();
        for (owner, config) in configs.iter().enumerate() {
            validate_configuration(config, mode, owner);
            configurations += 1;
            assert!(topics.insert(config.topic) && topics.insert(config.static_topic));
            for (role, entity) in config.entities.iter().enumerate() {
                assert_ne!(entity.native_entity, 0);
                assert!(native_ids.insert(entity.native_entity));
                assert_eq!(
                    entity.parent_entity,
                    if role == 0 {
                        config.world_entity
                    } else {
                        config.native_entity
                    }
                );
                assert_eq!(entity.kind, ["model", "link", "link", "joint"][role]);
                assert_eq!(entity.name, [config.name, "base", "tip", "hinge"][role]);
                assert_eq!(entity.selected, role != 3);
                assert_eq!(
                    entity.declared_channel,
                    ["static", "static", "dynamic", "unselected"][role]
                );
                assert_eq!(entity.canonical_link, role == 1);
                pose(entity.initial_local_pose);
            }
            assert_eq!(
                config.entities[3].initial_local_pose,
                [0, 0, 0, 0, 0, 0, 1.0_f64.to_bits()]
            );
        }
        assert!(!native_ids.contains(&configs[0].world_entity));
        assert!(
            configs
                .iter()
                .all(|config| config.world_entity == configs[0].world_entity)
        );
        let mut sim = load_replay(configs);
        let world = named(&sim, "default");
        assert_eq!(
            sim.world().get::<EntityKind>(world),
            Some(&EntityKind::World)
        );
        // A public WORLD LOCAL config/getter check, not an input to reading().
        assert_eq!(
            bits(sim.world().get::<LocalPose>(world).unwrap().0),
            configs[0].world_pose
        );
        let mut native_to_loaded = BTreeMap::from([(configs[0].world_entity, world)]);
        let mut owners = Vec::new();
        for config in configs {
            let owner = named(&sim, &format!("default::{}", config.name));
            owners.push(owner);
            let metadata = sim.world().get::<PosePublication>(owner).unwrap();
            assert_eq!(
                (metadata.model_name.as_str(), metadata.topic.as_str()),
                (config.name, config.topic)
            );
            assert_eq!(
                metadata.static_publication.as_ref().unwrap().topic,
                config.static_topic
            );
            for entity in config.entities {
                let name = if entity.kind == "model" {
                    format!("default::{}", entity.name)
                } else {
                    format!("default::{}::{}", config.name, entity.name)
                };
                let loaded = named(&sim, &name);
                assert!(
                    native_to_loaded
                        .insert(entity.native_entity, loaded)
                        .is_none()
                );
                assert_eq!(
                    sim.world().get::<EntityKind>(loaded),
                    Some(&match entity.kind {
                        "model" => EntityKind::Model,
                        "link" => EntityKind::Link,
                        "joint" => EntityKind::Joint,
                        _ => panic!("invalid configured role"),
                    })
                );
                let parent = sim.world().get::<ChildOf>(loaded).unwrap().parent();
                assert_eq!(parent, native_to_loaded[&entity.parent_entity]);
                let id = public_id(&sim, loaded);
                let dynamic = metadata.poses.iter().any(|frame| frame.identity == id);
                let fixed = metadata
                    .static_publication
                    .as_ref()
                    .unwrap()
                    .poses
                    .iter()
                    .any(|frame| frame.identity == id);
                assert_eq!(
                    (dynamic, fixed),
                    match entity.declared_channel {
                        "dynamic" => (true, false),
                        "static" => (false, true),
                        "unselected" => (false, false),
                        _ => panic!("unknown channel"),
                    }
                );
                if entity.kind == "joint" {
                    let joint = &sim.world().get::<JointDefinition>(loaded).unwrap().0;
                    assert_eq!(joint.kind, sdformat::JointKind::Revolute);
                    assert_eq!(
                        (joint.parent.as_str(), joint.child.as_str()),
                        (config.parent_link_name, config.child_link_name)
                    );
                }
            }
        }
        let mut mode_vectors = 0;
        let mut mode_records = 0;
        for (call_index, group) in rows.chunks_exact(count).enumerate() {
            let before = owners
                .iter()
                .map(|&owner| {
                    [false, true].map(|fixed| current_sample(&sim, owner, fixed).map(ledger))
                })
                .collect::<Vec<_>>();
            // The production sampler is global. Apply ALL disjoint owners' LOCAL
            // inputs before ONE invocation, avoiding extra equal-time revisions.
            for (owner, input) in group.iter().enumerate() {
                input_rows += 1;
                assert_eq!(input.owner as usize, owner);
                assert_eq!(input.call_index as usize, call_index);
                assert_eq!(
                    input.time_ns,
                    [0, 10_000_000, 20_000_000, 40_000_000][call_index]
                );
                assert_eq!(
                    input.dt_ns,
                    [0, 10_000_000, 10_000_000, 20_000_000][call_index]
                );
                assert!(!input.paused);
                assert_eq!(input.local_poses.len(), 3);
                for (role, local) in input.local_poses.iter().enumerate() {
                    assert_eq!(
                        local.native_entity,
                        configs[owner].entities[role].native_entity
                    );
                    assert_eq!(local.input_pose, local.getter_pose);
                    if call_index == 0 {
                        assert_eq!(
                            local.getter_pose,
                            configs[owner].entities[role].initial_local_pose
                        );
                    }
                    let loaded = native_to_loaded[&local.native_entity];
                    sim.world_mut().get_mut::<LocalPose>(loaded).unwrap().0 =
                        pose(local.input_pose);
                    assert_eq!(
                        bits(sim.world().get::<LocalPose>(loaded).unwrap().0),
                        local.getter_pose
                    );
                }
            }
            let input = &group[0];
            *sim.world_mut().resource_mut::<UpdateInfo>() = UpdateInfo {
                start_time: Duration::from_nanos(input.time_ns - input.dt_ns),
                sim_time: Duration::from_nanos(input.time_ns),
                dt: Duration::from_nanos(input.dt_ns),
                iteration: call_index as u64,
                epoch: 0,
                paused: input.paused,
            };
            // Source binds native info.iterations=call; no separate iteration is captured.
            sample(sim.world_mut());
            for (owner, input) in group.iter().enumerate() {
                let metadata = sim.world().get::<PosePublication>(owners[owner]).unwrap();
                let mut channels = BTreeSet::new();
                assert!((1..=2).contains(&input.observed.len()));
                for output in input.observed {
                    vectors += 1;
                    mode_vectors += 1;
                    assert!(channels.insert(output.channel));
                    let fixed = match output.channel {
                        "dynamic" => false,
                        "static" => true,
                        _ => panic!("unexpected captured channel"),
                    };
                    let actual = current_sample(&sim, owners[owner], fixed).unwrap();
                    assert_eq!(
                        (actual.time, actual.epoch),
                        (Duration::from_nanos(input.time_ns), 0)
                    );
                    let prior = &before[owner][usize::from(fixed)];
                    assert_eq!(actual.revision, prior.as_ref().map_or(1, |old| old.2 + 1));
                    let frames = if fixed {
                        &metadata.static_publication.as_ref().unwrap().poses
                    } else {
                        &metadata.poses
                    };
                    assert_eq!(output.records.len(), frames.len());
                    assert_eq!(actual.poses.len(), frames.len());
                    let mut names = BTreeSet::new();
                    for record in output.records {
                        records += 1;
                        mode_records += 1;
                        assert!(names.insert(record.child_frame_id));
                        assert_eq!(record.name, record.child_frame_id);
                        assert_eq!(record.id, 0);
                        let frame = frames
                            .iter()
                            .find(|frame| frame.child_frame_id == record.child_frame_id)
                            .unwrap();
                        assert_eq!(frame.frame_id, record.frame_id);
                        let value = actual
                            .poses
                            .iter()
                            .find(|value| value.identity == frame.identity)
                            .unwrap();
                        let expected = pose(record.pose);
                        for (rust, native) in value
                            .pose
                            .translation
                            .into_iter()
                            .chain(value.pose.rotation)
                            .zip(expected.translation.into_iter().chain(expected.rotation))
                        {
                            assert!(
                                rust.is_finite()
                                    && native.is_finite()
                                    && (rust - native).abs() < 1e-12,
                                "{mode} owner{owner} call{call_index} {}",
                                record.child_frame_id
                            );
                        }
                        assert!(!record.header_wire.is_empty() && !record.pose_wire.is_empty());
                        header_bytes += record.header_wire.len();
                        pose_bytes += record.pose_wire.len();
                    }
                    // Exact Header/Pose/full-vector contents and native ordering
                    // belong to literal DATA and the separate Server consumer.
                    assert!(!output.wire.is_empty());
                    vector_bytes += output.wire.len();
                }
                for (fixed, channel) in [(false, "dynamic"), (true, "static")] {
                    if !channels.contains(channel) {
                        assert_eq!(
                            current_sample(&sim, owners[owner], fixed).map(ledger),
                            before[owner][usize::from(fixed)]
                        );
                    }
                }
            }
        }
        assert_eq!(mode_vectors, summary.observed_vectors);
        assert_eq!(mode_records, summary.pose_records);
        assert!(sim.world().resource::<StepFault>().error().is_none());
        // No Physics step or Simulation clock was advanced by this direct replay.
        assert_eq!((sim.elapsed(), sim.epoch()), (Duration::ZERO, 0));
    }
    assert_eq!(
        (configurations, input_rows, vectors, records),
        (10, 40, 65, 95)
    );
    assert!(header_bytes > 0 && pose_bytes > header_bytes && vector_bytes > pose_bytes);
}
