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

fn m55_xml(count: usize) -> String {
    let start = SPLIT_XML.find("  <model name=\"rig\"").unwrap();
    let end = SPLIT_XML.rfind("</model>").unwrap() + "</model>".len();
    let template = &SPLIT_XML[start..end];
    let models = (0..count)
        .map(|index| {
            template
                .replace("name=\"rig\"", &format!("name=\"rig{index}\""))
                .replace("/m31/pose", &format!("/m55/pose{index}"))
                .replace(
                    "<pose>1 2 3 0 0 0.6</pose>",
                    &format!("<pose>1 {} 3 0 0 0.6</pose>", 2 + 4 * index),
                )
        })
        .collect::<String>();
    format!("{}{}{}", &SPLIT_XML[..start], models, &SPLIT_XML[end..])
}

fn m55_owner(server: &Server, index: usize) -> Entity {
    let name = format!("default::rig{index}");
    server
        .simulation
        .world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap()
        .id()
}

#[test]
fn multiple_pose_late_owner_and_immutable_metadata_refuse_before_closed_session_io() {
    for case in 0..10 {
        let count = if case == 9 { 7 } else { 8 };
        let mut server = server(&m55_xml(count));
        assert_eq!(server.pose_publishers.len(), 2 * count);
        let owner = m55_owner(&server, count - 1);
        let mut prepared = prepare_reset(&server, &BTreeSet::new()).unwrap();
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
                server
                    .simulation
                    .world_mut()
                    .get_mut::<Name>(owner)
                    .unwrap()
                    .0 = "default::other".into();
            }
            3 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<PosePublication>(owner)
                    .unwrap()
                    .model_name = "other".into();
            }
            4 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<PosePublication>(owner)
                    .unwrap()
                    .topic = "/m55/unique_changed_dynamic".into();
            }
            5 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<PosePublication>(owner)
                    .unwrap()
                    .static_publication
                    .as_mut()
                    .unwrap()
                    .topic = "/m55/unique_changed_static".into();
            }
            6 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<PosePublication>(owner)
                    .unwrap()
                    .static_publication
                    .as_mut()
                    .unwrap()
                    .poses[0]
                    .frame_id = "nonempty_wrong_world".into();
            }
            7 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<PosePublication>(owner)
                    .unwrap()
                    .static_publication
                    .as_mut()
                    .unwrap()
                    .poses[0]
                    .child_frame_id = "rig7::forged".into();
            }
            8 => {
                prepared.values_mut().last().unwrap().3.push_str("_forged");
            }
            _ => {
                // Preserve the original invalid-metadata-without-ID negative.
                let extra = server.simulation.world_mut().spawn(metadata()).id();
                assert!(server.simulation.world().get::<EntityId>(extra).is_none());
            }
        }
        server.world_io.session.close().wait().unwrap();
        let before = boundary(&server);
        for prepared in [None, Some(&prepared)] {
            let result = streams(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &BTreeSet::new(),
                prepared,
            );
            if case == 8 && prepared.is_none() {
                // A healthy live set reaches IO when no prepared map is used.
                assert!(matches!(result, Err(Error::WorldNetwork(_))));
            } else {
                assert!(
                    matches!(
                        result,
                        Err(Error::PosePublisherIdentity | Error::ResetPosePublishers)
                    ),
                    "case {case}"
                );
            }
            assert_eq!(boundary(&server), before);
        }
    }
}

#[test]
fn sixteen_pose_endpoints_and_every_prepared_identity_binding_are_atomic() {
    for case in 0..10 {
        let mut server = server(&m55_xml(8));
        let mut prepared = prepare_reset(&server, &BTreeSet::new()).unwrap();
        assert_eq!(prepared.len(), 16);
        assert_eq!(
            prepared
                .values()
                .map(|entry| entry.0)
                .collect::<BTreeSet<_>>()
                .len(),
            8
        );
        let last = prepared.keys().next_back().unwrap().clone();
        let mut occupied = BTreeSet::new();
        match case {
            0 => {
                prepared.remove(&last);
            }
            1 => {
                let row = prepared.values().next().unwrap().clone();
                prepared.insert("@/m55/extra@/pose".into(), row);
            }
            2 => {
                prepared.get_mut(&last).unwrap().0 += 1;
            }
            3 => {
                prepared.get_mut(&last).unwrap().1.topic = "/m55/changed".into();
            }
            4 => {
                prepared.get_mut(&last).unwrap().3.clear();
            }
            5 => {
                occupied.insert(last);
            }
            6 => {
                let owner = m55_owner(&server, 7);
                let first = server
                    .simulation
                    .world()
                    .get::<PosePublication>(m55_owner(&server, 0))
                    .unwrap()
                    .topic
                    .clone();
                server
                    .simulation
                    .world_mut()
                    .get_mut::<PosePublication>(owner)
                    .unwrap()
                    .static_publication
                    .as_mut()
                    .unwrap()
                    .topic = first;
            }
            7 => {
                let clone = server
                    .simulation
                    .world()
                    .get::<PosePublication>(m55_owner(&server, 7))
                    .unwrap()
                    .clone();
                server.simulation.world_mut().spawn(clone);
            }
            8 => {
                // Distinct same-schema outputs can be swapped without creating
                // duplicate Arcs; identity/metadata/cardinality alone cannot bind them.
                let keys = prepared
                    .iter()
                    .filter(|(key, row)| {
                        key.as_str() == server.network_config.topic_key(&row.1.topic).unwrap()
                    })
                    .map(|(key, _)| key.clone())
                    .take(2)
                    .collect::<Vec<_>>();
                assert_eq!(keys.len(), 2);
                assert_ne!(prepared[&keys[0]].0, prepared[&keys[1]].0);
                let left = Arc::clone(&prepared[&keys[0]].2);
                let right = Arc::clone(&prepared[&keys[1]].2);
                prepared.get_mut(&keys[0]).unwrap().2 = right;
                prepared.get_mut(&keys[1]).unwrap().2 = left;
            }
            _ => {
                let other = prepared.keys().find(|key| *key != &last).unwrap().clone();
                let duplicate = Arc::clone(&prepared[&other].2);
                prepared.get_mut(&last).unwrap().2 = duplicate;
            }
        }
        server.world_io.session.close().wait().unwrap();
        let before = boundary(&server);
        assert!(
            matches!(
                streams(
                    &server.simulation,
                    &server.world_io,
                    &server.network_config,
                    &occupied,
                    Some(&prepared)
                ),
                Err(Error::PosePublisherIdentity | Error::ResetPosePublishers)
            ),
            "case {case}"
        );
        assert_eq!(boundary(&server), before);
    }
    // Retained CPU outputs must still match their OLD endpoints when a valid
    // remap changes NEW prepared keys. Reject both permutations and duplicates
    // before lookup/reuse even with a closed session.
    for swapped in [false, true] {
        let mut server = server(&m55_xml(2));
        let other = server
            .pose_publishers
            .iter()
            .position(|stream| {
                stream.identity != server.pose_publishers[0].identity
                    && stream.static_channel == server.pose_publishers[0].static_channel
            })
            .unwrap();
        let first = Arc::clone(&server.pose_publishers[0].publisher);
        let second = Arc::clone(&server.pose_publishers[other].publisher);
        server.pose_publishers[0].publisher = second;
        if swapped {
            server.pose_publishers[other].publisher = first;
        }
        server.world_io.session.close().wait().unwrap();
        let before = boundary(&server);
        assert!(matches!(
            prepare_reset(&server, &BTreeSet::new()),
            Err(Error::ResetPosePublishers)
        ));
        assert_eq!(boundary(&server), before);
    }
    let mut server = server(&m55_xml(2));
    let first = server
        .simulation
        .world()
        .get::<PosePublication>(m55_owner(&server, 0))
        .unwrap()
        .topic
        .clone();
    let second = server
        .simulation
        .world()
        .get::<PosePublication>(m55_owner(&server, 1))
        .unwrap()
        .static_publication
        .as_ref()
        .unwrap()
        .topic
        .clone();
    server
        .network_config
        .add_topic_remap(&second, &first)
        .unwrap();
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
        Err(Error::PosePublisherIdentity)
    ));
    assert!(matches!(
        prepare_reset(&server, &BTreeSet::new()),
        Err(Error::PosePublisherIdentity)
    ));
    assert_eq!(boundary(&server), before);
}

#[test]
fn pose_prepared_prefix_restores_removed_owners_and_consistent_absence() {
    let mut server = server(&m55_xml(2));
    let prepared = prepare_reset(&server, &BTreeSet::new()).unwrap();
    let original = prepared
        .iter()
        .map(|(key, value)| {
            (
                key.clone(),
                value.0,
                value.1.clone(),
                Arc::clone(&value.2),
                value.3.clone(),
            )
        })
        .collect::<Vec<_>>();
    assert_eq!(original.len(), 4);
    let removed = server
        .simulation
        .world()
        .get::<EntityId>(m55_owner(&server, 1))
        .unwrap()
        .get();
    remove_model(&mut server.simulation, removed).unwrap();
    server.retire_pose_publishers();
    assert_eq!(server.pose_publishers.len(), 2);
    let reset = prepare_reset(&server, &BTreeSet::new()).unwrap();
    assert_eq!(reset.len(), 4);
    for (key, id, metadata, output, name) in &original {
        let entry = &reset[key];
        assert_eq!((entry.0, &entry.1, &entry.3), (*id, metadata, name));
        if *id != removed {
            assert!(Arc::ptr_eq(&entry.2, output));
        }
    }
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.update().unwrap();
    assert_eq!(server.pose_publishers.len(), 4);
    assert_eq!(server.simulation.epoch(), 1);
    for stream in &server.pose_publishers {
        let expected = reset.get(&stream.endpoint).unwrap();
        assert_eq!(stream.identity, expected.0);
        assert_eq!(stream.metadata, expected.1);
        assert!(stream.last_sample.is_none());
    }
    let absent = self::server(&m55_xml(0));
    assert!(absent.pose_publishers.is_empty());
    assert!(prepare_reset(&absent, &BTreeSet::new()).unwrap().is_empty());
    absent.world_io.session.close().wait().unwrap();
    assert!(
        streams(
            &absent.simulation,
            &absent.world_io,
            &absent.network_config,
            &BTreeSet::new(),
            Some(&Prepared::new())
        )
        .unwrap()
        .is_empty()
    );

    // Retire one selected ECS link as in the retained single-owner replaced-link
    // control. This proves cached output/lease refusal, not a new live Physics
    // link-removal API. The independent loaded peer remains owned and intact.
    let mut linked = self::server(&m55_xml(2));
    linked.simulation.step_exact(40).unwrap();
    linked.publish_pose_publishers();
    let victim = m55_owner(&linked, 0);
    let peer = m55_owner(&linked, 1);
    let identity = linked
        .simulation
        .world()
        .get::<EntityId>(victim)
        .unwrap()
        .get();
    let selected = linked
        .simulation
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::rig0::tip")
        })
        .unwrap()
        .id();
    let selected_id = linked
        .simulation
        .world()
        .get::<EntityId>(selected)
        .unwrap()
        .get();
    let leases = linked
        .pose_publishers
        .iter()
        .filter(|stream| stream.identity == identity)
        .map(|stream| Arc::downgrade(&stream.publisher))
        .collect::<Vec<_>>();
    assert_eq!(leases.len(), 2);
    let queued = linked.counters().pose_publications_queued;
    let peer_sample = format!(
        "{:?}/{:?}",
        linked.simulation.world().get::<PoseSample>(peer),
        linked.simulation.world().get::<StaticPoseSample>(peer)
    );
    assert!(linked.simulation.world_mut().despawn(selected));
    linked.publish_pose_publishers();
    assert_eq!(linked.counters().pose_publications_queued, queued);
    linked.retire_pose_publishers();
    assert_eq!(linked.pose_publishers.len(), 2);
    assert!(
        linked
            .pose_publishers
            .iter()
            .all(|stream| stream.entity == peer)
    );
    assert!(leases.iter().all(|lease| lease.upgrade().is_none()));
    assert_eq!(
        format!(
            "{:?}/{:?}",
            linked.simulation.world().get::<PoseSample>(peer),
            linked.simulation.world().get::<StaticPoseSample>(peer)
        ),
        peer_sample
    );
    let restored = prepare_reset(&linked, &BTreeSet::new()).unwrap();
    assert_eq!(restored.len(), 4);
    drop(restored);
    assert!(linked.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    linked.update().unwrap();
    assert_eq!(linked.pose_publishers.len(), 4);
    let restored_link = entity_by_id(linked.simulation.world(), selected_id).unwrap();
    assert_ne!(restored_link, selected);
    assert!(
        linked
            .simulation
            .world()
            .get::<PoseSample>(m55_owner(&linked, 0))
            .is_none()
    );
    linked.simulation.step_exact(40).unwrap();
    // Advancing with a lost selected frame keeps the unchanged GLOBAL fault
    // contract; it does not promise per-owner fault isolation or rollback.
    let failed_owner = m55_owner(&linked, 0);
    assert!(
        linked
            .simulation
            .world()
            .get::<PoseSample>(failed_owner)
            .is_some()
    );
    assert!(
        linked
            .simulation
            .world()
            .get::<StaticPoseSample>(failed_owner)
            .is_some()
    );
    // Fence both owners' existing revisions before observing failure-only enqueue.
    linked.publish_pose_publishers();
    let capture_cached = |server: &Server| {
        let bits = |sample: &PoseSample| {
            (
                sample.time,
                sample.epoch,
                sample.revision,
                sample
                    .poses
                    .iter()
                    .map(|frame| {
                        (
                            frame.identity,
                            frame.pose.translation.map(f64::to_bits),
                            frame.pose.rotation.map(f64::to_bits),
                        )
                    })
                    .collect::<Vec<_>>(),
            )
        };
        (
            server
                .simulation
                .world()
                .get::<PoseSample>(failed_owner)
                .map(bits),
            server
                .simulation
                .world()
                .get::<StaticPoseSample>(failed_owner)
                .map(|sample| bits(&sample.0)),
        )
    };
    let cached = capture_cached(&linked);
    let queued = linked.counters().pose_publications_queued;
    assert!(linked.simulation.world_mut().despawn(restored_link));
    assert_eq!(
        linked.simulation.step_exact(10).unwrap_err(),
        gz_sim_core::StepError::Faulted
    );
    assert_eq!(
        linked.simulation.fault().unwrap().to_string(),
        "loaded joint bindings are incomplete or belong to a different backend"
    );
    // Joint preflight faults before the pose sampler. Global terminal faults
    // do not run later samplers, clear earlier caches, or roll peers back.
    assert_eq!(capture_cached(&linked), cached);
    linked.publish_pose_publishers();
    assert_eq!(linked.counters().pose_publications_queued, queued);
    let before = boundary(&linked);
    assert!(!linked.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(
        linked.update(),
        Err(Error::Step(gz_sim_core::StepError::Faulted))
    ));
    assert_eq!(boundary(&linked), before);
}

#[test]
fn pose_maximum_selected_records_and_live_or_removed_peer_reservations_hold_before_io() {
    let mut models = String::new();
    for owner in 0..8 {
        let links = (0..127).map(|link| format!("<link name='body{link}'><pose>{} 0 0 0 0 0</pose><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='shape'><geometry><box><size>0.05 0.05 0.05</size></box></geometry></collision></link>", link as f64 * 0.2)).collect::<String>();
        models.push_str(&format!("<model name='rig{owner}' canonical_link='body0'><pose>0 {} 3 0 0 0</pose>{links}<plugin filename='gz-sim-pose-publisher-system' name='gz::sim::systems::PosePublisher'><use_pose_vector_msg>true</use_pose_vector_msg><publish_model_pose>true</publish_model_pose><static_publisher>true</static_publisher><topic>/m55/max{owner}</topic></plugin></model>", owner * 100));
    }
    let source = format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='step' type='ignored'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    );
    let maximum = server(&source);
    assert_eq!(maximum.pose_publishers.len(), 16);
    let prepared = prepare_reset(&maximum, &BTreeSet::new()).unwrap();
    assert_eq!(prepared.len(), 16);
    let metadata = gz_sim_scene::initial_pose_publications(&maximum.simulation)
        .unwrap()
        .map(|(_, publication)| publication.clone())
        .collect::<Vec<_>>();
    assert_eq!(metadata.len(), 8);
    assert!(metadata.iter().all(|publication| publication.poses.len()
        + publication.static_publication.as_ref().unwrap().poses.len()
        == 128));
    maximum.world_io.session.close().wait().unwrap();
    let before = boundary(&maximum);
    let restored = streams(
        &maximum.simulation,
        &maximum.world_io,
        &maximum.network_config,
        &BTreeSet::new(),
        Some(&prepared),
    )
    .unwrap();
    assert_eq!(restored.len(), 16);
    assert_eq!(boundary(&maximum), before);
    drop(restored);

    let peer = "<model name='imu_owner'><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='shape'><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision><sensor name='imu' type='imu'><topic>/m55/imu</topic><update_rate>100</update_rate><imu/></sensor></link></model>";
    for remove in [false, true] {
        let source = m55_xml(2).replace("</world>", &format!("{peer}</world>"));
        let mut server = server(&source);
        if remove {
            let id = server
                .simulation
                .world()
                .iter_entities()
                .find(|entity| {
                    entity
                        .get::<Name>()
                        .is_some_and(|name| name.0 == "default::imu_owner")
                })
                .unwrap()
                .get::<EntityId>()
                .unwrap()
                .get();
            remove_model(&mut server.simulation, id).unwrap();
        }
        server
            .network_config
            .add_topic_remap("/m55/pose1_static", "/m55/imu")
            .unwrap();
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
            Err(Error::PosePublisherIdentity)
        ));
        assert!(matches!(
            prepare_reset(&server, &BTreeSet::new()),
            Err(Error::PosePublisherIdentity)
        ));
        assert_eq!(boundary(&server), before);
    }
}
