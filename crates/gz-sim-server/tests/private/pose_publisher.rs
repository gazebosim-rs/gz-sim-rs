// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::{
    msgs::{WorldControl, WorldReset},
    prost::Name as _,
};
use gz_sim_scene::{
    PhysicsEngine, PoseFramePublication, PoseFrameValue, WorldPose, load_world_with_sdf_step,
    remove_model,
};
const XML: &str = include_str!("../../../gz-sim-scene/tests/fixtures/m27_pose_publisher.sdf");
fn metadata() -> PosePublication {
    PosePublication {
        static_publication: None,
        model_name: "rig".into(),
        topic: "/m27/pose".into(),
        poses: vec![
            PoseFramePublication {
                identity: 7,
                frame_id: "default".into(),
                child_frame_id: "rig".into(),
            },
            PoseFramePublication {
                identity: 11,
                frame_id: "rig".into(),
                child_frame_id: "rig::body".into(),
            },
        ],
    }
}
fn sample() -> PoseSample {
    PoseSample {
        time: Duration::new(2, 123),
        epoch: 4,
        revision: 1,
        poses: vec![
            PoseFrameValue {
                identity: 7,
                pose: WorldPose {
                    translation: [1.0, 2.0, 3.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                },
            },
            PoseFrameValue {
                identity: 11,
                pose: WorldPose {
                    translation: [0.8, 0.0, 0.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                },
            },
        ],
    }
}
#[test]
fn pose_publisher_wire_is_local_frame_vector_with_per_pose_stamps_and_no_wire_ids_or_outer_header()
{
    let frame = packet(&metadata(), 7, &sample()).unwrap();
    assert_eq!(PoseV::full_name(), "gz.msgs.Pose_V");
    assert!(frame.header.is_none());
    for (index, pose) in frame.pose.iter().enumerate() {
        let meta = &metadata().poses[index];
        assert_eq!(
            (pose.name.as_str(), pose.id),
            (meta.child_frame_id.as_str(), 0)
        );
        let header = pose.header.as_ref().unwrap();
        assert_eq!(
            header.stamp,
            Some(gz_sim_proto::msgs::Time { sec: 2, nsec: 123 })
        );
        assert_eq!(
            header.data,
            vec![
                Map {
                    key: "frame_id".into(),
                    value: vec![meta.frame_id.clone()]
                },
                Map {
                    key: "child_frame_id".into(),
                    value: vec![meta.child_frame_id.clone()]
                }
            ]
        );
        assert_eq!(
            pose.position.as_ref().unwrap().x,
            sample().poses[index].pose.translation[0]
        );
    }
    assert_eq!(
        PoseV::decode(frame.encode_to_vec().as_slice()).unwrap(),
        frame
    );
    // Public IDs are internal ownership, not the viewer's uint32 wire IDs.
    let mut m = metadata();
    let mut s = sample();
    m.poses[1].identity = u64::MAX;
    s.poses[1].identity = u64::MAX;
    assert!(packet(&m, 7, &s).is_some());
}
#[test]
fn pose_publisher_invalid_metadata_or_one_bad_transform_cannot_encode_a_partial_or_stale_vector() {
    for case in 0..16 {
        let mut m = metadata();
        let mut s = sample();
        let mut owner = 7;
        match case {
            0 => owner = 0,
            1 => m.model_name.clear(),
            2 => m.topic.clear(),
            3 => m.poses.clear(),
            4 => m.poses[1].identity = 0,
            5 => m.poses[1].identity = 7,
            6 => m.poses.swap(0, 1),
            7 => m.poses[1].frame_id = "other".into(),
            8 => m.poses[1].child_frame_id = "rig::nested::body".into(),
            9 => s.poses.clear(),
            10 => s.poses[1].identity = 99,
            11 => s.poses[1].pose.translation[0] = f64::NAN,
            12 => s.poses[1].pose.rotation = [0.0; 4],
            13 => s.time = Duration::MAX,
            14 => m.poses[0].frame_id.clear(),
            _ => m.topic = "/bad topic".into(),
        }
        assert!(packet(&m, owner, &s).is_none(), "case {case}");
    }
}
fn server(xml: &str) -> Server {
    let sim = load_world_with_sdf_step(
        xml,
        "server-m27.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let mut config = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(key, value).unwrap();
    }
    Server::with_zenoh_config(
        sim,
        "default",
        ServerOptions {
            partition: format!("m27-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        config,
        NonZeroUsize::new(4).unwrap(),
    )
    .unwrap()
}
fn receive(
    subscriber: &zenoh::pubsub::Subscriber<
        zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>,
    >,
) -> PoseV {
    let sample = subscriber
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .expect("actualPoseV");
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.Pose_V"
    );
    PoseV::decode(sample.payload().to_bytes().as_ref()).unwrap()
}
fn boundary(server: &Server) -> impl PartialEq + std::fmt::Debug + use<> {
    let bodies: Vec<_> = server
        .simulation
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<gz_sim_scene::BodyHandle>())
        .map(|h| {
            (
                h.id(),
                server
                    .simulation
                    .world()
                    .resource::<gz_physics::PhysicsBackend>()
                    .state(h.id()),
            )
        })
        .collect();
    (
        server.simulation.elapsed(),
        server.simulation.epoch(),
        bodies,
        server
            .pose_publishers
            .iter()
            .map(|s| {
                (
                    s.entity,
                    s.identity,
                    s.frames.clone(),
                    Arc::as_ptr(&s.publisher),
                    s.last_sample,
                )
            })
            .collect::<Vec<_>>(),
        format!("{:?}", server.counters()),
    )
}
#[test]
fn pose_publisher_real_wire_tracks_motion_skips_pause_and_restores_fresh_reset_handles_without_new_lease()
 {
    let mut server = server(XML);
    let stream = &server.pose_publishers[0];
    let old_entity = stream.entity;
    let id = stream.identity;
    let original = Arc::clone(&stream.publisher);
    let subscriber = server
        .world_io
        .session
        .declare_subscriber(stream.endpoint.clone())
        .wait()
        .unwrap();
    server.set_paused(true);
    server.update().unwrap();
    assert_eq!(server.counters().pose_publications_queued, 0);
    assert!(subscriber.try_recv().unwrap().is_none());
    let joint = server
        .simulation
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<gz_sim_scene::Name>()
                .is_some_and(|n| n.0 == "default::rig::tip_hinge")
        })
        .unwrap();
    let handle = joint.get::<gz_sim_scene::JointHandle>().unwrap().id();
    server
        .simulation
        .world_mut()
        .resource_mut::<gz_physics::PhysicsBackend>()
        .set_joint_velocity_motor(
            handle,
            Some(gz_physics::JointVelocityMotor {
                target_velocity: 0.5,
                max_force: 0.5,
            }),
        )
        .unwrap();
    server.set_paused(false);
    for _ in 0..10 {
        server.update().unwrap();
    }
    let moving = receive(&subscriber);
    assert_eq!(moving.pose.len(), 3);
    assert_eq!(
        moving.pose[0]
            .header
            .as_ref()
            .unwrap()
            .stamp
            .as_ref()
            .unwrap()
            .nsec,
        10_000_000
    );
    let config = &server.pose_publishers[0].metadata;
    let sample = server
        .simulation
        .world()
        .get::<PoseSample>(old_entity)
        .unwrap();
    assert_eq!(packet(config, id, sample).unwrap(), moving);
    assert!(
        server
            .simulation
            .world()
            .resource::<gz_physics::PhysicsBackend>()
            .joint_state(handle)
            .unwrap()
            .velocity
            .abs()
            > 0.0
    );
    let queued = server.counters().pose_publications_queued;
    server.set_paused(true);
    server.update().unwrap();
    server.update().unwrap();
    server.publish_pose_publishers();
    assert_eq!(server.counters().pose_publications_queued, queued);
    assert!(subscriber.try_recv().unwrap().is_none());
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert_eq!(server.update().unwrap().epoch, 1);
    assert_ne!(server.pose_publishers[0].entity, old_entity);
    assert_eq!(server.pose_publishers[0].identity, id);
    assert!(Arc::ptr_eq(&original, &server.pose_publishers[0].publisher));
    server.update().unwrap();
    assert!(subscriber.try_recv().unwrap().is_none());
    server.set_paused(false);
    for _ in 0..10 {
        server.update().unwrap();
    }
    let reset = receive(&subscriber);
    assert_eq!(
        reset.pose[0]
            .header
            .as_ref()
            .unwrap()
            .stamp
            .as_ref()
            .unwrap()
            .nsec,
        10_000_000
    );
    let weak = Arc::downgrade(&original);
    drop(original);
    remove_model(&mut server.simulation, id).unwrap();
    server.update().unwrap();
    assert!(server.pose_publishers.is_empty() && server.pose_publishers.capacity() == 0);
    assert!(weak.upgrade().is_none());
    server.shutdown();
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}
#[test]
fn pose_publisher_preflight_is_atomic_for_reserved_keys_identity_drift_and_lost_reset_mapping() {
    for case in 0..9 {
        let mut server = server(XML);
        let occupied = BTreeSet::new();
        let mut prepared = prepare_reset(&server, &occupied).unwrap();
        let owner = server.pose_publishers[0].entity;
        match case {
            0 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(owner)
                    .remove::<EntityId>();
            }
            1 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(owner)
                    .insert(EntityKind::Visual);
            }
            2 => {
                let link = server.pose_publishers[0].frames.last().unwrap().1;
                server
                    .simulation
                    .world_mut()
                    .entity_mut(link)
                    .remove::<bevy_ecs::prelude::ChildOf>();
            }
            3 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<PosePublication>(owner)
                    .unwrap()
                    .poses[0]
                    .frame_id = "wrongworld".into();
            }
            4 => {
                prepared.clear();
            }
            5 => {
                let row = prepared.values().next().unwrap().clone();
                prepared.insert("@/extra@/pose".into(), row);
            }
            6 => {
                prepared.values_mut().next().unwrap().0 += 1;
            }
            7 => {
                prepared.values_mut().next().unwrap().1.topic = "/changed".into();
            }
            _ => {
                let identity = server.pose_publishers[0].frames.last().unwrap().0;
                let entity = entity_by_id(server.simulation.world(), identity).unwrap();
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .remove::<EntityId>();
            }
        }
        let before = boundary(&server);
        let result = streams(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &occupied,
            Some(&prepared),
        );
        assert!(
            matches!(
                result,
                Err(Error::PosePublisherIdentity | Error::ResetPosePublishers)
            ),
            "case{case}"
        );
        assert_eq!(before, boundary(&server));
    }
    let server = server(XML);
    let key = server.pose_publishers[0].endpoint.clone();
    let occupied = BTreeSet::from([key]);
    let before = boundary(&server);
    assert!(matches!(
        streams(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &occupied,
            None
        ),
        Err(Error::PosePublisherIdentity)
    ));
    assert!(matches!(
        prepare_reset(&server, &occupied),
        Err(Error::PosePublisherIdentity)
    ));
    assert_eq!(boundary(&server), before);
}
#[test]
fn pose_publisher_invalid_frames_are_counted_once_and_bounded_or_closed_io_does_not_mutate_physics()
{
    let mut server = server(XML);
    server.simulation.step_exact(10).unwrap();
    let entity = server.pose_publishers[0].entity;
    let original = server
        .simulation
        .world()
        .get::<PoseSample>(entity)
        .unwrap()
        .clone();
    for case in 0..3 {
        let mut sample = original.clone();
        sample.revision = 100 + case;
        match case {
            0 => sample.epoch += 1,
            1 => sample.time += Duration::from_secs(1),
            _ => sample.poses[0].pose.translation[0] = f64::NAN,
        }
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(sample);
        server.publish_pose_publishers();
        server.publish_pose_publishers();
    }
    assert_eq!(server.counters().invalid_pose_publication_samples, 3);
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(original);
    server.world_io.max_message_bytes = 1;
    server.publish_pose_publishers();
    assert_eq!(server.counters().pose_publisher_failures, 1);
    server.world_io.session.close().wait().unwrap();
    let before = boundary(&server);
    assert!(matches!(
        streams(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &BTreeSet::new(),
            None
        ),
        Err(Error::WorldNetwork(_))
    ));
    assert_eq!(boundary(&server), before);
}
#[test]
fn pose_publisher_absence_and_no_subscriber_preserve_native_acquisition_and_reused_identity_cannot_retain_stream()
 {
    let mut active = server(XML);
    active.set_paused(false);
    for _ in 0..10 {
        active.update().unwrap();
    }
    assert_eq!(active.counters().pose_publications_queued, 1);
    let entity = active.pose_publishers[0].entity;
    assert_eq!(
        active
            .simulation
            .world()
            .get::<PoseSample>(entity)
            .unwrap()
            .revision,
        1
    );
    let weak = Arc::downgrade(&active.pose_publishers[0].publisher);
    let link = active.pose_publishers[0].frames.last().unwrap().1;
    active
        .simulation
        .world_mut()
        .entity_mut(link)
        .remove::<EntityId>();
    active.retire_pose_publishers();
    assert!(active.pose_publishers.is_empty());
    assert!(weak.upgrade().is_none());
    let start = XML.find("    <plugin ").unwrap();
    let end = start + XML[start..].find("</plugin>").unwrap() + 9;
    let mut absent = server(&format!("{}{}", &XML[..start], &XML[end..]));
    absent.update().unwrap();
    assert!(absent.pose_publishers.is_empty() && absent.pose_publishers.capacity() == 0);
    assert_eq!(absent.counters().pose_publications_queued, 0);
}
#[test]
fn pose_publisher_metadata_and_replaced_entity_refusals_keep_all_ownership_and_no_io_side_effects()
{
    for case in 0..4 {
        let mut server = server(XML);
        let owner = server.pose_publishers[0].entity;
        match case {
            0 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<gz_sim_scene::Name>(owner)
                    .unwrap()
                    .0 = "broken_scope".into()
            }
            1 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<PosePublication>(owner)
                    .unwrap()
                    .model_name = "other".into()
            }
            2 => {
                let frame = server.pose_publishers[0].frames.last().unwrap().1;
                server
                    .simulation
                    .world_mut()
                    .get_mut::<gz_sim_scene::Name>(frame)
                    .unwrap()
                    .0 = "default::rig::forged".into();
            }
            _ => {
                let extra = server.simulation.world_mut().spawn(metadata()).id();
                assert_ne!(extra, owner);
            }
        }
        let before = boundary(&server);
        assert!(matches!(
            streams(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &BTreeSet::new(),
                None
            ),
            Err(Error::PosePublisherIdentity)
        ));
        assert_eq!(before, boundary(&server));
    }
    // A new Bevy entity with the same reserved public ID cannot inherit a live
    // stream's captured entity handle, even when its metadata looks identical.
    let mut server = server(XML);
    let (identity, old) = *server.pose_publishers[0].frames.last().unwrap();
    let owner = server.pose_publishers[0].entity;
    let reserved = gz_sim_core::reserve_id(server.simulation.world(), old).unwrap();
    let name = server
        .simulation
        .world()
        .get::<gz_sim_scene::Name>(old)
        .unwrap()
        .0
        .clone();
    server.simulation.world_mut().despawn(old);
    let replacement = server
        .simulation
        .world_mut()
        .spawn((
            EntityKind::Link,
            gz_sim_scene::Name(name),
            bevy_ecs::prelude::ChildOf(owner),
        ))
        .id();
    gz_sim_core::restore_id(server.simulation.world_mut(), replacement, &reserved).unwrap();
    assert_eq!(
        entity_by_id(server.simulation.world(), identity),
        Some(replacement)
    );
    let weak = Arc::downgrade(&server.pose_publishers[0].publisher);
    server.retire_pose_publishers();
    assert!(server.pose_publishers.is_empty());
    assert!(weak.upgrade().is_none());
}

const SPLIT_XML: &str =
    include_str!("../../../gz-sim-scene/tests/fixtures/m31_pose_static_publisher.sdf");
#[test]
fn split_pose_real_topics_have_independent_credit_free_revisions_reset_and_weak_retirement() {
    let mut server = server(SPLIT_XML);
    assert_eq!(server.pose_publishers.len(), 2);
    assert!(server.pose_publishers[0].static_channel);
    let fixed = server
        .world_io
        .session
        .declare_subscriber(server.pose_publishers[0].endpoint.clone())
        .wait()
        .unwrap();
    let dynamic = server
        .world_io
        .session
        .declare_subscriber(server.pose_publishers[1].endpoint.clone())
        .wait()
        .unwrap();
    let leases: Vec<_> = server
        .pose_publishers
        .iter()
        .map(|s| Arc::clone(&s.publisher))
        .collect();
    let old = server.pose_publishers[0].entity;
    let id = server.pose_publishers[0].identity;
    server.set_paused(false);
    for tick in 1..=100 {
        server.update().unwrap();
        if tick % 10 == 0 {
            let packet = receive(&dynamic);
            assert!(packet.header.is_none());
            assert_eq!(packet.pose.len(), 1);
            assert_eq!(packet.pose[0].name, "rig::tip");
            assert_eq!(packet.pose[0].id, 0);
            assert_eq!(
                packet.pose[0]
                    .header
                    .as_ref()
                    .unwrap()
                    .stamp
                    .as_ref()
                    .unwrap()
                    .nsec,
                tick * 1_000_000
            );
        }
        if tick % 40 == 0 {
            let packet = receive(&fixed);
            assert_eq!(
                packet
                    .pose
                    .iter()
                    .map(|p| p.name.as_str())
                    .collect::<Vec<_>>(),
                ["rig", "rig::base"]
            );
            assert!(packet.header.is_none());
            for pose in packet.pose {
                let header = pose.header.unwrap();
                assert_eq!(header.stamp.unwrap().nsec, tick * 1_000_000);
                assert_eq!(header.data.len(), 2);
                assert_eq!(header.data[0].key, "frame_id");
                assert_eq!(header.data[1].key, "child_frame_id");
                assert_eq!(pose.id, 0);
            }
        }
    }
    assert_eq!(server.counters().pose_publications_queued, 12);
    server.publish_pose_publishers();
    server.set_paused(true);
    server.update().unwrap();
    assert_eq!(server.counters().pose_publications_queued, 12);
    assert!(dynamic.try_recv().unwrap().is_none() && fixed.try_recv().unwrap().is_none());
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.update().unwrap();
    assert_ne!(server.pose_publishers[0].entity, old);
    for (stream, lease) in server.pose_publishers.iter().zip(&leases) {
        assert!(Arc::ptr_eq(&stream.publisher, lease));
    }
    server.set_paused(false);
    for _ in 0..40 {
        server.update().unwrap();
    }
    for expected in [10, 20, 30, 40] {
        assert_eq!(
            receive(&dynamic).pose[0]
                .header
                .as_ref()
                .unwrap()
                .stamp
                .as_ref()
                .unwrap()
                .nsec,
            expected * 1_000_000
        );
    }
    assert_eq!(
        receive(&fixed).pose[0]
            .header
            .as_ref()
            .unwrap()
            .stamp
            .as_ref()
            .unwrap()
            .nsec,
        40_000_000
    );
    let weak: Vec<_> = leases.iter().map(Arc::downgrade).collect();
    drop(leases);
    remove_model(&mut server.simulation, id).unwrap();
    server.update().unwrap();
    assert!(server.pose_publishers.is_empty());
    assert!(weak.iter().all(|w| w.upgrade().is_none()));
}
#[test]
fn split_pose_preflight_covers_both_endpoints_and_full_static_identity_without_io() {
    for case in 0..8 {
        let mut server = server(SPLIT_XML);
        let mut prepared = prepare_reset(&server, &BTreeSet::new()).unwrap();
        assert_eq!(prepared.len(), 2);
        let entity = server.pose_publishers[0].entity;
        let mut occupied = BTreeSet::new();
        match case {
            0 => {
                occupied.insert(server.pose_publishers[0].endpoint.clone());
            }
            1 => {
                occupied.insert(server.pose_publishers[1].endpoint.clone());
            }
            2 => {
                let key = prepared.keys().next().unwrap().clone();
                prepared.remove(&key);
            }
            3 => {
                prepared.values_mut().next().unwrap().0 += 1;
            }
            4 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<PosePublication>(entity)
                    .unwrap()
                    .static_publication
                    .as_mut()
                    .unwrap()
                    .topic = "/changed".into();
            }
            5 => {
                let m = &mut server
                    .simulation
                    .world_mut()
                    .get_mut::<PosePublication>(entity)
                    .unwrap();
                m.static_publication.as_mut().unwrap().poses[0].identity = m.poses[0].identity;
            }
            6 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<PosePublication>(entity)
                    .unwrap()
                    .static_publication
                    .as_mut()
                    .unwrap()
                    .poses
                    .swap(0, 1);
            }
            _ => {
                let id = server
                    .simulation
                    .world()
                    .get::<PosePublication>(entity)
                    .unwrap()
                    .static_publication
                    .as_ref()
                    .unwrap()
                    .poses[1]
                    .identity;
                let link = entity_by_id(server.simulation.world(), id).unwrap();
                server
                    .simulation
                    .world_mut()
                    .entity_mut(link)
                    .remove::<EntityId>();
            }
        }
        let before = boundary(&server);
        assert!(matches!(
            streams(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &occupied,
                Some(&prepared)
            ),
            Err(Error::PosePublisherIdentity | Error::ResetPosePublishers)
        ));
        assert_eq!(boundary(&server), before);
    }
}
#[test]
fn split_jointless_declares_empty_dynamic_endpoint_but_never_encodes_an_empty_vector() {
    let start = SPLIT_XML.find("    <joint ").unwrap();
    let end = SPLIT_XML.find("    <plugin ").unwrap();
    let mut server = server(&format!("{}{}", &SPLIT_XML[..start], &SPLIT_XML[end..]));
    assert_eq!(server.pose_publishers.len(), 2);
    let dynamic = server
        .world_io
        .session
        .declare_subscriber(server.pose_publishers[1].endpoint.clone())
        .wait()
        .unwrap();
    server.set_paused(false);
    for _ in 0..100 {
        server.update().unwrap();
    }
    assert_eq!(server.counters().pose_publications_queued, 2);
    assert!(dynamic.try_recv().unwrap().is_none());
    assert_eq!(server.counters().invalid_pose_publication_samples, 0);
}
