// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::msgs::{WorldControl, WorldReset};
use gz_sim_scene::{PhysicsEngine, remove_model};
use zenoh::Wait;

const XML: &str = include_str!("../../../gz-sim-scene/tests/fixtures/m18_joint_state.sdf");
fn xml(count: usize) -> String {
    let start = XML.find("  <model ").unwrap();
    let end = XML.rfind("</model>").unwrap() + "</model>".len();
    let models = (0..count)
        .map(|owner| {
            XML[start..end]
                .replace("name=\"arm\"", &format!("name=\"rig{owner}\""))
                .replace(
                    "<pose>0 0 1 0 0 0</pose>",
                    &format!("<pose>0 {} 1 0 0 0</pose>", 4 * owner),
                )
                .replace("/m18/joint_state", &format!("/m57/joint{owner}"))
                .replace(
                    "<update_rate>100</update_rate>",
                    if owner.is_multiple_of(2) {
                        "<update_rate>100</update_rate>"
                    } else {
                        "<update_rate>50</update_rate>"
                    },
                )
        })
        .collect::<String>();
    format!("{}{}{}", &XML[..start], models, &XML[end..])
}
fn server_text(text: &str) -> Server {
    let sim = gz_sim_scene::load_world_with_sdf_step(
        text,
        "multiple-joint-state-private.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let config = zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}},transport:{shared_memory:{enabled:false}}}"#).unwrap();
    Server::with_zenoh_config(
        sim,
        "default",
        ServerOptions {
            partition: format!("m57-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        config,
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap()
}
fn server(count: usize) -> Server {
    server_text(&xml(count))
}
fn owner(server: &Server, index: usize) -> Entity {
    server
        .simulation
        .world()
        .iter_entities()
        .find(|node| {
            node.get::<Name>()
                .is_some_and(|name| name.0 == format!("default::rig{index}"))
        })
        .unwrap()
        .id()
}
#[derive(Debug, PartialEq)]
struct StreamSnapshot {
    entity: Entity,
    identity: u64,
    joints: Vec<(u64, Entity)>,
    metadata: JointStatePublication,
    endpoint: String,
    output: usize,
    strong: usize,
    last: Option<(u64, u64)>,
}
#[derive(Debug, PartialEq)]
struct BoundarySnapshot {
    epoch: u64,
    time: Duration,
    streams: Vec<StreamSnapshot>,
    queued: u64,
    invalid: u64,
    failures: u64,
}
fn boundary(server: &Server) -> BoundarySnapshot {
    BoundarySnapshot {
        epoch: server.simulation.epoch(),
        time: server.simulation.elapsed(),
        streams: server
            .joint_states
            .iter()
            .map(|stream| StreamSnapshot {
                entity: stream.entity,
                identity: stream.identity,
                joints: stream.joints.clone(),
                metadata: stream.metadata.clone(),
                endpoint: stream.endpoint.clone(),
                output: Arc::as_ptr(&stream.publisher) as usize,
                strong: Arc::strong_count(&stream.publisher),
                last: stream.last_sample,
            })
            .collect(),
        queued: server.counters.joint_states_queued,
        invalid: server.counters.invalid_joint_state_samples,
        failures: server.counters.joint_state_failures,
    }
}
#[test]
fn zero_one_pair_eight_and_legacy_manual_singleton_keep_identity_and_prepared_prefix() {
    for count in [0, 1, 2, 8] {
        let active = server(count);
        let entries = prepare_reset(&active, &BTreeSet::new()).unwrap();
        assert_eq!(active.joint_states.len(), count);
        assert_eq!(entries.len(), count);
        for stream in &active.joint_states {
            let row = &entries[&stream.endpoint];
            assert_eq!(row.0, stream.identity);
            assert_eq!(row.1, stream.metadata);
            assert!(Arc::ptr_eq(&row.2, &stream.publisher));
            assert_eq!(
                row.3,
                active
                    .simulation
                    .world()
                    .get::<Name>(stream.entity)
                    .unwrap()
                    .0
            );
            assert!(row.2.matches_endpoint(&stream.endpoint));
        }
    }
    let active = server(1);
    let mut metadata = active.joint_states[0].metadata.clone();
    let mut manual = Simulation::new(Duration::from_millis(1)).unwrap();
    let entity = manual.world_mut().spawn(EntityKind::Model).id();
    gz_sim_core::assign_id(manual.world_mut(), entity).unwrap();
    for joint in &mut metadata.joints {
        let child = manual
            .world_mut()
            .spawn((EntityKind::Joint, bevy_ecs::prelude::ChildOf(entity)))
            .id();
        joint.id = gz_sim_core::assign_id(manual.world_mut(), child).unwrap();
    }
    manual
        .world_mut()
        .entity_mut(entity)
        .insert(metadata.clone());
    let output = streams(
        &manual,
        &active.world_io,
        &active.network_config,
        &BTreeSet::new(),
        None,
    )
    .unwrap();
    assert_eq!(output.len(), 1);
    assert_eq!(output[0].metadata, metadata);
    assert!(
        !manual.world().entity(entity).contains::<Name>(),
        "legacy manual binding does not invent a loaded name"
    );
    let second = manual.world_mut().spawn((EntityKind::Model, metadata)).id();
    gz_sim_core::assign_id(manual.world_mut(), second).unwrap();
    assert!(matches!(
        streams(
            &manual,
            &active.world_io,
            &active.network_config,
            &BTreeSet::new(),
            None
        ),
        Err(Error::JointStateIdentity)
    ));
    let empty = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(
        streams(
            &empty,
            &active.world_io,
            &active.network_config,
            &BTreeSet::new(),
            None
        )
        .unwrap()
        .is_empty()
    );
}
#[test]
fn late_eighth_owner_identity_metadata_keys_and_ninth_refuse_before_closed_io() {
    for case in 0..10 {
        let mut active = server(8);
        let last = owner(&active, 7);
        let mut occupied = BTreeSet::new();
        match case {
            0 => {
                active
                    .simulation
                    .world_mut()
                    .entity_mut(last)
                    .remove::<EntityId>();
            }
            1 => {
                active
                    .simulation
                    .world_mut()
                    .entity_mut(last)
                    .insert(EntityKind::Link);
            }
            2 => {
                active
                    .simulation
                    .world_mut()
                    .entity_mut(last)
                    .insert(Name("foreign::rig7".into()));
            }
            3 => {
                active
                    .simulation
                    .world_mut()
                    .get_mut::<JointStatePublication>(last)
                    .unwrap()
                    .model_name = "wrong".into();
            }
            4 => {
                active
                    .simulation
                    .world_mut()
                    .get_mut::<JointStatePublication>(last)
                    .unwrap()
                    .joints[1]
                    .damping = f64::NAN;
            }
            5 => {
                let mut metadata = active
                    .simulation
                    .world_mut()
                    .get_mut::<JointStatePublication>(last)
                    .unwrap();
                metadata.joints[1].id = metadata.joints[0].id;
            }
            6 => {
                active
                    .simulation
                    .world_mut()
                    .get_mut::<JointStatePublication>(last)
                    .unwrap()
                    .topic = "/m57/joint0".into();
            }
            7 => {
                active
                    .network_config
                    .add_topic_remap("/m57/joint7", "/m57/joint0")
                    .unwrap();
            }
            8 => {
                occupied.insert(active.joint_states[7].endpoint.clone());
            }
            _ => {
                let metadata = active
                    .simulation
                    .world()
                    .get::<JointStatePublication>(last)
                    .unwrap()
                    .clone();
                active.simulation.world_mut().spawn(metadata);
            }
        }
        active.world_io.session.close().wait().unwrap();
        let before = boundary(&active);
        assert!(
            matches!(
                streams(
                    &active.simulation,
                    &active.world_io,
                    &active.network_config,
                    &occupied,
                    None
                ),
                Err(Error::JointStateIdentity)
            ),
            "case {case}"
        );
        assert_eq!(boundary(&active), before);
    }
    assert!(
        gz_sim_scene::load_world_with_sdf_step(
            &xml(9),
            "ninth.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
}
#[test]
fn all_prepared_fields_distinct_output_swaps_and_duplicates_refuse_before_arc_reuse() {
    for case in 0..8 {
        let active = server(8);
        let mut prepared = prepare_reset(&active, &BTreeSet::new()).unwrap();
        let keys = prepared.keys().cloned().collect::<Vec<_>>();
        match case {
            0 => {
                prepared.remove(&keys[7]);
            }
            1 => {
                prepared.get_mut(&keys[7]).unwrap().0 += 1;
            }
            2 => {
                prepared
                    .get_mut(&keys[7])
                    .unwrap()
                    .1
                    .model_name
                    .push_str("-changed");
            }
            3 => {
                prepared.get_mut(&keys[7]).unwrap().3 = "other::rig7".into();
            }
            4 => {
                let first = Arc::clone(&prepared[&keys[0]].2);
                let last = Arc::clone(&prepared[&keys[7]].2);
                assert!(!Arc::ptr_eq(&first, &last));
                prepared.get_mut(&keys[0]).unwrap().2 = last;
                prepared.get_mut(&keys[7]).unwrap().2 = first;
            }
            5 => {
                prepared.get_mut(&keys[7]).unwrap().2 = Arc::clone(&prepared[&keys[0]].2);
            }
            6 => {
                let value = prepared[&keys[7]].clone();
                prepared.insert("@/unexpected@/topic".into(), value);
            }
            _ => {
                let value = prepared.remove(&keys[7]).unwrap();
                prepared.insert(
                    active.network_config.topic_key("/m57/wrong").unwrap(),
                    value,
                );
            }
        }
        active.world_io.session.close().wait().unwrap();
        let before = boundary(&active);
        assert!(
            matches!(
                streams(
                    &active.simulation,
                    &active.world_io,
                    &active.network_config,
                    &BTreeSet::new(),
                    Some(&prepared)
                ),
                Err(Error::ResetJointStates)
            ),
            "case {case}"
        );
        assert_eq!(boundary(&active), before);
    }
}
#[test]
fn retained_outputs_bind_old_endpoints_and_full_reset_restores_removed_originals_after_remap() {
    for case in 0..7 {
        let mut active = server(2);
        match case {
            0 | 1 => {
                let first = Arc::clone(&active.joint_states[0].publisher);
                let last = Arc::clone(&active.joint_states[1].publisher);
                assert!(!Arc::ptr_eq(&first, &last));
                active.joint_states[1].publisher = first;
                if case == 1 {
                    active.joint_states[0].publisher = last;
                }
            }
            2 => {
                active.joint_states[1]
                    .metadata
                    .model_name
                    .push_str("-changed");
            }
            3 => {
                active.joint_states[1].identity = active.joint_states[0].identity;
            }
            4 => {
                active.joint_states[1].entity = active.joint_states[0].entity;
            }
            5 => {
                active.joint_states[1].joints[0].1 = active.joint_states[0].joints[0].1;
            }
            _ => {
                active.joint_states[1].endpoint = active.joint_states[0].endpoint.clone();
            }
        }
        active.world_io.session.close().wait().unwrap();
        let before = boundary(&active);
        assert!(
            matches!(
                prepare_reset(&active, &BTreeSet::new()),
                Err(Error::ResetJointStates)
            ),
            "case {case}"
        );
        assert_eq!(boundary(&active), before);
    }
    let mut active = server(8);
    let original = active
        .joint_states
        .iter()
        .map(|stream| (stream.identity, stream.metadata.clone(), stream.entity))
        .collect::<Vec<_>>();
    let removed = original[7].0;
    remove_model(&mut active.simulation, removed).unwrap();
    active.retire_joint_states();
    assert_eq!(active.joint_states.len(), 7);
    for index in 0..8 {
        active
            .network_config
            .add_topic_remap(
                &format!("/m57/joint{index}"),
                &format!("/chosen/joint{index}"),
            )
            .unwrap();
    }
    let prepared = prepare_reset(&active, &BTreeSet::new()).unwrap();
    assert_eq!(prepared.len(), 8);
    for (identity, metadata, _) in &original {
        let key = active.network_config.topic_key(&metadata.topic).unwrap();
        assert_eq!((&prepared[&key].0, &prepared[&key].1), (identity, metadata));
        assert!(prepared[&key].2.matches_endpoint(&key));
    }
    drop(prepared);
    assert!(active.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    active.update().unwrap();
    assert_eq!(active.simulation.epoch(), 1);
    assert_eq!(active.joint_states.len(), 8);
    for (identity, metadata, old_entity) in original {
        let stream = active
            .joint_states
            .iter()
            .find(|stream| stream.identity == identity)
            .unwrap();
        assert_ne!(stream.entity, old_entity);
        assert_eq!(stream.metadata, metadata);
        assert!(stream.last_sample.is_none());
        assert!(stream.endpoint.contains("/chosen/joint"));
    }
}
#[test]
fn original_endpoint_reservations_survive_owner_removal_and_peer_collisions_are_atomic() {
    let mut active = server(2);
    let removed = active.joint_states[1].identity;
    remove_model(&mut active.simulation, removed).unwrap();
    active.retire_joint_states();
    active
        .network_config
        .add_topic_remap("/m57/joint0", "/m57/joint1")
        .unwrap();
    active.world_io.session.close().wait().unwrap();
    let before = boundary(&active);
    assert!(matches!(
        streams(
            &active.simulation,
            &active.world_io,
            &active.network_config,
            &BTreeSet::new(),
            None
        ),
        Err(Error::JointStateIdentity)
    ));
    assert!(matches!(
        prepare_reset(&active, &BTreeSet::new()),
        Err(Error::JointStateIdentity)
    ));
    assert_eq!(boundary(&active), before);
    for target in [
        "/world/default/clock",
        "/world/default/control",
        "/world/default/remove",
        "/m57/imu",
    ] {
        let text = xml(2).replacen("<link name=\"base\">", "<link name=\"base\"><sensor name='imu' type='imu'><topic>/m57/imu</topic><update_rate>100</update_rate></sensor>", 1);
        let mut peer = server_text(&text);
        peer.network_config
            .add_topic_remap("/m57/joint1", target)
            .unwrap();
        peer.world_io.session.close().wait().unwrap();
        let before = boundary(&peer);
        assert!(matches!(
            streams(
                &peer.simulation,
                &peer.world_io,
                &peer.network_config,
                &BTreeSet::new(),
                None
            ),
            Err(Error::JointStateIdentity)
        ));
        assert_eq!(boundary(&peer), before);
    }
}
type JointValueBits = (u64, u64, u64);
#[derive(Debug, PartialEq)]
struct CachedSample {
    time: Duration,
    epoch: u64,
    revision: u64,
    translation: [u64; 3],
    rotation: [u64; 4],
    joints: Vec<JointValueBits>,
}
fn cached(active: &Server) -> Vec<Option<CachedSample>> {
    active
        .joint_states
        .iter()
        .map(|stream| {
            active
                .simulation
                .world()
                .get::<JointStateSample>(stream.entity)
                .map(|sample| CachedSample {
                    time: sample.time,
                    epoch: sample.epoch,
                    revision: sample.revision,
                    translation: sample.pose.translation.map(f64::to_bits),
                    rotation: sample.pose.rotation.map(f64::to_bits),
                    joints: sample
                        .joints
                        .iter()
                        .map(|joint| (joint.id, joint.position.to_bits(), joint.velocity.to_bits()))
                        .collect(),
                })
        })
        .collect()
}
#[test]
fn selected_and_unselected_joint_retirement_preserve_global_terminal_cache_and_lease_boundaries() {
    for selected in [true, false] {
        let text = if selected {
            xml(2)
        } else {
            xml(2).replacen(
                "<update_rate>100</update_rate>",
                "<update_rate>100</update_rate><joint_name>tip_hinge</joint_name>",
                1,
            )
        };
        let mut active = server_text(&text);
        active.simulation.step_exact(20).unwrap();
        active.publish_joint_states();
        let first = owner(&active, 0);
        let stream = active
            .joint_states
            .iter()
            .find(|stream| stream.entity == first)
            .unwrap();
        let identity = stream.identity;
        let weak = Arc::downgrade(&stream.publisher);
        let victim = if selected {
            stream.joints[0].1
        } else {
            active
                .simulation
                .world()
                .iter_entities()
                .find(|node| {
                    node.get::<Name>()
                        .is_some_and(|name| name.0 == "default::rig0::base_hinge")
                })
                .unwrap()
                .id()
        };
        if !selected {
            assert!(stream.joints.iter().all(|(_, entity)| *entity != victim));
        }
        let before = cached(&active);
        let queued = active.counters.joint_states_queued;
        // A surviving incomplete binding triggers the unchanged global preflight,
        // including while the publication clock would suppress the next sample.
        assert_eq!(
            active
                .simulation
                .world()
                .get::<bevy_ecs::prelude::ChildOf>(victim)
                .unwrap()
                .parent(),
            first
        );
        active
            .simulation
            .world_mut()
            .entity_mut(victim)
            .remove::<bevy_ecs::prelude::ChildOf>();
        assert!(
            active
                .simulation
                .world()
                .get::<bevy_ecs::prelude::ChildOf>(victim)
                .is_none()
        );
        active.publish_joint_states();
        assert_eq!(active.counters.joint_states_queued, queued);
        assert_eq!(
            active.simulation.step_exact(1).unwrap_err(),
            gz_sim_core::StepError::Faulted
        );
        assert_eq!(
            active.simulation.fault().unwrap().to_string(),
            "loaded joint bindings are incomplete or belong to a different backend"
        );
        assert_eq!(cached(&active), before);
        active.publish_joint_states();
        assert_eq!(active.counters.joint_states_queued, queued);
        active.retire_joint_states();
        if selected {
            assert!(weak.upgrade().is_none());
            assert_eq!(active.joint_states.len(), 1);
            assert!(
                active
                    .joint_states
                    .iter()
                    .all(|stream| stream.identity != identity)
            );
        } else {
            assert!(weak.upgrade().is_some());
            assert_eq!(active.joint_states.len(), 2);
        }
        let boundary = boundary(&active);
        assert!(!active.apply_local_control(WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        }));
        assert!(matches!(
            active.update(),
            Err(Error::Step(gz_sim_core::StepError::Faulted))
        ));
        assert_eq!(self::boundary(&active), boundary);
    }

    // Retain both original full-despawn cases without inventing a next-step fault.
    for selected in [true, false] {
        let text = if selected {
            xml(2)
        } else {
            xml(2).replacen(
                "<update_rate>100</update_rate>",
                "<update_rate>100</update_rate><joint_name>tip_hinge</joint_name>",
                1,
            )
        };
        let mut active = server_text(&text);
        active.simulation.step_exact(20).unwrap();
        active.publish_joint_states();
        let first = owner(&active, 0);
        let stream = active
            .joint_states
            .iter()
            .find(|stream| stream.entity == first)
            .unwrap();
        let identity = stream.identity;
        let weak = Arc::downgrade(&stream.publisher);
        let victim = if selected {
            stream.joints[0].1
        } else {
            active
                .simulation
                .world()
                .iter_entities()
                .find(|node| {
                    node.get::<Name>()
                        .is_some_and(|name| name.0 == "default::rig0::base_hinge")
                })
                .unwrap()
                .id()
        };
        if !selected {
            assert!(stream.joints.iter().all(|(_, entity)| *entity != victim));
        }
        let before = cached(&active);
        assert!(before.iter().all(Option::is_some));
        let queued = active.counters.joint_states_queued;
        assert!(active.simulation.fault().is_none());
        assert!(active.simulation.world_mut().despawn(victim));
        active.publish_joint_states();
        assert_eq!(active.counters.joint_states_queued, queued);
        assert_eq!(cached(&active), before);
        // Full entity removal is a distinct stream-retirement boundary. It does
        // not establish an invalid surviving binding or a global terminal fault.
        active.retire_joint_states();
        if selected {
            assert!(weak.upgrade().is_none());
            assert_eq!(active.joint_states.len(), 1);
            assert!(
                active
                    .joint_states
                    .iter()
                    .all(|stream| stream.identity != identity)
            );
        } else {
            assert!(weak.upgrade().is_some());
            assert_eq!(active.joint_states.len(), 2);
            assert!(
                active
                    .joint_states
                    .iter()
                    .any(|stream| stream.identity == identity)
            );
        }
        assert!(active.simulation.fault().is_none());
    }
}
#[test]
fn selected_joint_parent_and_kind_are_live_admission_requirements_but_reset_uses_original_snapshot()
{
    for case in 0..3 {
        let mut active = server(2);
        let child = active.joint_states[1].joints[0].1;
        match case {
            0 => {
                active
                    .simulation
                    .world_mut()
                    .entity_mut(child)
                    .remove::<bevy_ecs::prelude::ChildOf>();
            }
            1 => {
                active
                    .simulation
                    .world_mut()
                    .entity_mut(child)
                    .insert(EntityKind::Link);
            }
            _ => {
                let other = owner(&active, 0);
                active
                    .simulation
                    .world_mut()
                    .entity_mut(child)
                    .insert(bevy_ecs::prelude::ChildOf(other));
            }
        }
        active.world_io.session.close().wait().unwrap();
        let before = boundary(&active);
        assert!(matches!(
            streams(
                &active.simulation,
                &active.world_io,
                &active.network_config,
                &BTreeSet::new(),
                None
            ),
            Err(Error::JointStateIdentity)
        ));
        assert_eq!(boundary(&active), before);
    }
    let mut active = server(2);
    let (identity, child) = active.joint_states[1].joints[0];
    assert!(active.simulation.world_mut().despawn(child));
    let prepared = prepare_reset(&active, &BTreeSet::new()).unwrap();
    assert_eq!(prepared.len(), 2);
    drop(prepared);
    assert!(active.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    active.update().unwrap();
    assert_ne!(
        entity_by_id(active.simulation.world(), identity).unwrap(),
        child
    );
    assert_eq!(active.joint_states.len(), 2);
}
#[test]
fn selected_metadata_128_boundary_and_129_refusal_preserve_unrestricted_limits_and_full_packet_fields()
 {
    let active = server(1);
    let mut metadata = active.joint_states[0].metadata.clone();
    let prototype = metadata.joints[0].clone();
    metadata.joints = (0..128)
        .map(|index| {
            let mut joint = prototype.clone();
            joint.id = 1000 + index;
            joint.name = format!("joint{index}");
            joint
        })
        .collect();
    let sample = JointStateSample {
        second_axes: Vec::new(),
        time: Duration::from_millis(20),
        epoch: 0,
        revision: 1,
        pose: gz_sim_scene::WorldPose {
            translation: [1.0, 2.0, 3.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        },
        joints: metadata
            .joints
            .iter()
            .map(|joint| gz_sim_scene::JointStateValue {
                id: joint.id,
                position: 0.25,
                velocity: -0.5,
            })
            .collect(),
    };
    let message = packet(&metadata, active.joint_states[0].identity, &sample).unwrap();
    assert_eq!(message.joint.len(), 128);
    for (joint, expected) in message.joint.iter().zip(&metadata.joints) {
        assert_eq!(u64::from(joint.id), expected.id);
        assert_eq!(joint.name, expected.name);
        assert!(joint.axis2.is_none());
        let axis = joint.axis1.as_ref().unwrap();
        assert_eq!(axis.limit_lower, f64::NEG_INFINITY);
        assert_eq!(axis.limit_upper, f64::INFINITY);
        assert_eq!((axis.position, axis.velocity), (0.25, -0.5));
    }
    let mut extra = prototype;
    extra.id = 1128;
    extra.name = "joint128".into();
    metadata.joints.push(extra);
    assert!(!valid_metadata(&metadata));
    assert!(packet(&metadata, active.joint_states[0].identity, &sample).is_none());
}
