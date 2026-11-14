// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::msgs::{WorldControl, WorldReset};
use gz_sim_scene::{PhysicsEngine, remove_model};
use zenoh::Wait;

fn xml(count: usize) -> String {
    let models = (0..count).map(|index| format!("<model name='rig{index}'><pose>0 {} 3 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='shape'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision></link><plugin filename='gz-sim-odometry-publisher-system' name='gz::sim::systems::OdometryPublisher'><dimensions>{}</dimensions><odom_publish_frequency>{}</odom_publish_frequency><odom_topic>/m56/odom{index}</odom_topic><odom_covariance_topic>/m56/cov{index}</odom_covariance_topic><tf_topic>/m56/tf{index}</tf_topic><odom_frame>m56/odom{index}</odom_frame><robot_base_frame>m56/base{index}</robot_base_frame></plugin></model>", index * 4, if index.is_multiple_of(2) { 2 } else { 3 }, if index.is_multiple_of(2) { 100 } else { 50 })).collect::<String>();
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ignored'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn server(count: usize) -> Server {
    server_text(&xml(count))
}
fn server_text(text: &str) -> Server {
    let simulation = gz_sim_scene::load_world_with_sdf_step(
        text,
        "multiple-odometry-private.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let config = zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}},transport:{shared_memory:{enabled:false}}}"#).unwrap();
    Server::with_zenoh_config(
        simulation,
        "default",
        ServerOptions {
            partition: format!("m56-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        config,
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap()
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
type BoundarySnapshot = (u64, Duration, Vec<(String, u64, Channel, usize)>);

fn boundary(server: &Server) -> BoundarySnapshot {
    (
        server.simulation.epoch(),
        server.simulation.elapsed(),
        server
            .ground_truth_odometries
            .iter()
            .map(|stream| {
                (
                    stream.endpoint.clone(),
                    stream.identity,
                    stream.channel,
                    Arc::strong_count(&stream.publisher),
                )
            })
            .collect(),
    )
}
#[test]
fn zero_one_pair_and_eight_bind_all_three_channels_and_preserve_manual_singleton() {
    for count in [0, 1, 2, 8] {
        let active = server(count);
        let prepared = prepare_reset(&active, &BTreeSet::new()).unwrap();
        assert_eq!(active.ground_truth_odometries.len(), 3 * count);
        assert_eq!(prepared.len(), 3 * count);
        for stream in &active.ground_truth_odometries {
            let row = &prepared[&stream.endpoint];
            assert_eq!(row.0, stream.identity);
            assert_eq!(row.1, stream.metadata);
            assert_eq!(row.2, stream.channel);
            assert!(Arc::ptr_eq(&row.3, &stream.publisher));
            assert_eq!(
                row.4,
                active
                    .simulation
                    .world()
                    .get::<Name>(stream.entity)
                    .unwrap()
                    .0
            );
        }
    }
    let active = server(1);
    let metadata = active.ground_truth_odometries[0].metadata.clone();
    let mut manual = Simulation::new(Duration::from_millis(1)).unwrap();
    let entity = manual
        .world_mut()
        .spawn((
            EntityKind::Model,
            Name("default::rig0".into()),
            metadata.clone(),
        ))
        .id();
    gz_sim_core::assign_id(manual.world_mut(), entity).unwrap();
    assert_eq!(
        streams(
            &manual,
            &active.world_io,
            &active.network_config,
            &BTreeSet::new(),
            None
        )
        .unwrap()
        .len(),
        3
    );
    let second = manual
        .world_mut()
        .spawn((
            EntityKind::Model,
            Name("default::rig1".into()),
            Metadata {
                model_name: "rig1".into(),
                odom_topic: "/manual/odom1".into(),
                covariance_topic: "/manual/cov1".into(),
                tf_topic: "/manual/tf1".into(),
                ..metadata
            },
        ))
        .id();
    gz_sim_core::assign_id(manual.world_mut(), second).unwrap();
    active.world_io.session.close().wait().unwrap();
    assert!(matches!(
        streams(
            &manual,
            &active.world_io,
            &active.network_config,
            &BTreeSet::new(),
            None
        ),
        Err(Error::GroundTruthOdometryIdentity)
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
fn late_owner_original_metadata_and_all_twenty_four_endpoints_refuse_before_closed_io() {
    for case in 0..12 {
        let mut active = server(8);
        let last = owner(&active, 7);
        let original = active
            .simulation
            .world()
            .get::<Metadata>(last)
            .unwrap()
            .clone();
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
                    .insert(Name("other::rig7".into()));
            }
            3 => {
                active
                    .simulation
                    .world_mut()
                    .get_mut::<Metadata>(last)
                    .unwrap()
                    .odom_frame
                    .push_str("-changed");
            }
            4 => {
                active
                    .simulation
                    .world_mut()
                    .get_mut::<Metadata>(last)
                    .unwrap()
                    .covariance_topic = original.odom_topic.clone();
            }
            5 => {
                active
                    .network_config
                    .add_topic_remap(&original.tf_topic, "/m56/odom0")
                    .unwrap();
            }
            6 => {
                occupied.insert(
                    active
                        .network_config
                        .topic_key(&original.covariance_topic)
                        .unwrap(),
                );
            }
            7 => {
                active
                    .simulation
                    .world_mut()
                    .get_mut::<Metadata>(last)
                    .unwrap()
                    .tf_topic = "bad topic".into();
            }
            8 => {
                active
                    .simulation
                    .world_mut()
                    .get_mut::<Metadata>(last)
                    .unwrap()
                    .gaussian_noise =
                    gz_sim_scene::GroundTruthOdometryNoise::from_stddev(0.01).unwrap();
            }
            9 => {
                let cloned = active
                    .simulation
                    .world()
                    .get::<Metadata>(last)
                    .unwrap()
                    .clone();
                active.simulation.world_mut().spawn(cloned);
            }
            10 => {
                active
                    .network_config
                    .add_topic_remap(&original.odom_topic, &original.covariance_topic)
                    .unwrap();
            }
            _ => {
                active
                    .simulation
                    .world_mut()
                    .get_mut::<Metadata>(last)
                    .unwrap()
                    .model_name = "other".into();
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
                Err(Error::GroundTruthOdometryIdentity)
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
fn every_prepared_prefix_name_channel_and_output_binding_is_validated_before_reuse() {
    for case in 0..11 {
        let mut active = server(8);
        let mut prepared = prepare_reset(&active, &BTreeSet::new()).unwrap();
        let key = prepared.keys().next_back().unwrap().clone();
        match case {
            0 => {
                prepared.remove(&key);
            }
            1 => {
                prepared.get_mut(&key).unwrap().0 += 1;
            }
            2 => {
                prepared
                    .get_mut(&key)
                    .unwrap()
                    .1
                    .robot_base_frame
                    .push_str("-bad");
            }
            3 => {
                prepared.get_mut(&key).unwrap().2 = Channel::Covariance;
            }
            4 => {
                prepared.get_mut(&key).unwrap().4 = "wrong::rig7".into();
            }
            5 => {
                let row = prepared.values().next().unwrap().clone();
                prepared.insert("extra".into(), row);
            }
            6 => {
                let replacement = prepared
                    .iter()
                    .find(|(_, row)| row.2 != prepared[&key].2)
                    .unwrap()
                    .1
                    .3
                    .clone();
                prepared.get_mut(&key).unwrap().3 = replacement;
            }
            7 => {
                let replacement = prepared
                    .iter()
                    .find(|(other, row)| *other != &key && row.2 == prepared[&key].2)
                    .unwrap()
                    .1
                    .3
                    .clone();
                prepared.get_mut(&key).unwrap().3 = replacement;
            }
            8 => {
                prepared.get_mut(&key).unwrap().1.offsets =
                    gz_sim_scene::GroundTruthOdometryOffsets::from_translations(
                        [0.25, 0.0, 0.0],
                        [0.0; 3],
                    )
                    .unwrap();
            }
            9 => {
                // Swap DISTINCT same-channel leases, leaving every key, owner,
                // metadata, channel, name and pointer cardinality unchanged.
                let other = prepared
                    .iter()
                    .find(|(_, row)| row.0 != prepared[&key].0 && row.2 == prepared[&key].2)
                    .unwrap()
                    .0
                    .clone();
                let left = Arc::clone(&prepared[&key].3);
                let right = Arc::clone(&prepared[&other].3);
                assert!(!Arc::ptr_eq(&left, &right));
                prepared.get_mut(&key).unwrap().3 = right;
                prepared.get_mut(&other).unwrap().3 = left;
            }
            _ => {
                // Retained streams bind OLD actual endpoints independently of
                // newly remapped reset keys; a swapped live lease also refuses.
                let channel = active.ground_truth_odometries[0].channel;
                let index = active
                    .ground_truth_odometries
                    .iter()
                    .position(|stream| {
                        stream.identity != active.ground_truth_odometries[0].identity
                            && stream.channel == channel
                    })
                    .unwrap();
                let left = Arc::clone(&active.ground_truth_odometries[0].publisher);
                let right = Arc::clone(&active.ground_truth_odometries[index].publisher);
                assert!(!Arc::ptr_eq(&left, &right));
                active.ground_truth_odometries[0].publisher = right;
                active.ground_truth_odometries[index].publisher = left;
            }
        }
        active.world_io.session.close().wait().unwrap();
        let before = boundary(&active);
        if case == 10 {
            assert!(matches!(
                prepare_reset(&active, &BTreeSet::new()),
                Err(Error::GroundTruthOdometryIdentity)
            ));
        } else {
            assert!(
                matches!(
                    streams(
                        &active.simulation,
                        &active.world_io,
                        &active.network_config,
                        &BTreeSet::new(),
                        Some(&prepared)
                    ),
                    Err(Error::GroundTruthOdometryIdentity)
                ),
                "case {case}"
            );
        }
        assert_eq!(boundary(&active), before);
    }
}
#[test]
fn original_owner_restoration_remapped_channels_and_peer_leases_survive_loaded_reset() {
    let mut active = server(2);
    let removed_entity = owner(&active, 1);
    let removed = active
        .simulation
        .world()
        .get::<EntityId>(removed_entity)
        .unwrap()
        .get();
    let peer = owner(&active, 0);
    let peer_id = active
        .simulation
        .world()
        .get::<EntityId>(peer)
        .unwrap()
        .get();
    let peer_outputs = active
        .ground_truth_odometries
        .iter()
        .filter(|stream| stream.identity == peer_id)
        .map(|stream| (stream.channel, Arc::clone(&stream.publisher)))
        .collect::<Vec<_>>();
    remove_model(&mut active.simulation, removed).unwrap();
    active.retire_ground_truth_odometry();
    assert_eq!(active.ground_truth_odometries.len(), 3);
    let prepared = prepare_reset(&active, &BTreeSet::new()).unwrap();
    assert_eq!(prepared.len(), 6);
    for (channel, output) in &peer_outputs {
        assert!(
            prepared
                .values()
                .any(|row| row.0 == peer_id && row.2 == *channel && Arc::ptr_eq(output, &row.3))
        );
    }
    for index in 0..2 {
        for channel in ["odom", "cov", "tf"] {
            active
                .network_config
                .add_topic_remap(
                    &format!("/m56/{channel}{index}"),
                    &format!("/chosen/{channel}{index}"),
                )
                .unwrap();
        }
    }
    let remapped = prepare_reset(&active, &BTreeSet::new()).unwrap();
    assert_eq!(remapped.len(), 6);
    for index in 0..2 {
        for channel in ["odom", "cov", "tf"] {
            assert!(
                remapped.contains_key(
                    &active
                        .network_config
                        .topic_key(&format!("/chosen/{channel}{index}"))
                        .unwrap()
                )
            );
        }
    }
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
    assert_eq!(active.ground_truth_odometries.len(), 6);
    assert_ne!(
        entity_by_id(active.simulation.world(), removed).unwrap(),
        removed_entity
    );
    assert!(
        active
            .ground_truth_odometries
            .iter()
            .all(|stream| stream.last_sample.is_none() && stream.live(&active.simulation))
    );
    let mut reservations = BTreeSet::new();
    reservations.insert(active.network_config.topic_key("/chosen/tf1").unwrap());
    active.world_io.session.close().wait().unwrap();
    let before = boundary(&active);
    assert!(matches!(
        prepare_reset(&active, &reservations),
        Err(Error::GroundTruthOdometryIdentity)
    ));
    assert_eq!(boundary(&active), before);
}
#[test]
fn live_and_removed_original_peer_roles_remain_reserved_before_any_output_lookup() {
    let peer = "<model name='sensor_peer'><pose>0 20 3 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='shape'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision><sensor name='imu' type='imu'><topic>/m56/peer_imu</topic><frame_id>sensor_peer/body/imu</frame_id><update_rate>100</update_rate><imu/></sensor></link></model>";
    let text = xml(2).replace("</world>", &format!("{peer}</world>"));
    for removed in [false, true] {
        let mut active = server_text(&text);
        if removed {
            let id = active
                .simulation
                .world()
                .iter_entities()
                .find(|node| {
                    node.get::<Name>()
                        .is_some_and(|name| name.0 == "default::sensor_peer")
                })
                .unwrap()
                .get::<EntityId>()
                .unwrap()
                .get();
            remove_model(&mut active.simulation, id).unwrap();
        }
        active.world_io.session.close().wait().unwrap();
        let options = active.network_config.clone();
        for channel in ["odom", "cov", "tf"] {
            for reverse in [false, true] {
                active.network_config = options.clone();
                let own = format!("/m56/{channel}1");
                let (from, to) = if reverse {
                    ("/m56/peer_imu", own.as_str())
                } else {
                    (own.as_str(), "/m56/peer_imu")
                };
                active.network_config.add_topic_remap(from, to).unwrap();
                let before = boundary(&active);
                assert!(matches!(
                    prepare_reset(&active, &BTreeSet::new()),
                    Err(Error::GroundTruthOdometryIdentity)
                ));
                assert_eq!(boundary(&active), before);
            }
        }
    }
    // An absent original odometry owner still owns all three restored keys.
    let mut active = server(2);
    let id = active
        .simulation
        .world()
        .get::<EntityId>(owner(&active, 1))
        .unwrap()
        .get();
    remove_model(&mut active.simulation, id).unwrap();
    active.retire_ground_truth_odometry();
    active
        .simulation
        .world_mut()
        .spawn(gz_sim_scene::ImuPublication {
            enable_orientation: true,
            entity_name: "manual_imu".into(),
            frame_id: "manual/imu".into(),
            topic: "/m56/tf1".into(),
        });
    active.world_io.session.close().wait().unwrap();
    let before = boundary(&active);
    assert!(matches!(
        prepare_reset(&active, &BTreeSet::new()),
        Err(Error::GroundTruthOdometryIdentity)
    ));
    assert_eq!(boundary(&active), before);
}
#[test]
fn one_bad_owner_keeps_global_terminal_fault_and_loaded_reset_refusal() {
    let mut active = server(2);
    active.simulation.step_exact(40).unwrap();
    active.publish_ground_truth_odometry();
    let failed = owner(&active, 0);
    assert!(active.simulation.world().get::<Sample>(failed).is_some());
    active
        .simulation
        .world_mut()
        .get_mut::<Metadata>(failed)
        .unwrap()
        .odom_frame
        .push_str("-tampered");
    assert_eq!(
        active.simulation.step_exact(1).unwrap_err(),
        gz_sim_core::StepError::Faulted
    );
    assert!(active.simulation.fault().is_some());
    assert!(active.simulation.world().get::<Sample>(failed).is_none());
    // No peer rollback or callback iteration order is promised by this global fault.
    let before = boundary(&active);
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
    assert_eq!(boundary(&active), before);
}
