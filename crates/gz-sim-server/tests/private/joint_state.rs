// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::{
    msgs::{WorldControl, WorldReset},
    prost::Name,
};
use gz_sim_scene::{
    JointStateJointPublication, JointStateValue, PhysicsEngine, WorldPose,
    load_world_with_sdf_step, queue_joint_effort, remove_model,
};
const XML: &str = include_str!("../../../gz-sim-scene/tests/fixtures/m18_joint_state.sdf");
fn metadata() -> JointStatePublication {
    JointStatePublication {
        model_name: "arm".into(),
        topic: "/m18/joint_state".into(),
        joints: vec![JointStateJointPublication {
            id: 11,
            name: "hinge".into(),
            parent: "base".into(),
            child: "tip".into(),
            pose: sdformat::Transform {
                translation: [0.3, -0.2, 0.1],
                ..Default::default()
            },
            axis: [0.0, 0.0, 1.0],
            lower: f64::NEG_INFINITY,
            upper: f64::INFINITY,
            damping: 0.2,
        }],
    }
}
fn sample() -> JointStateSample {
    JointStateSample {
        time: Duration::new(2, 123),
        epoch: 4,
        revision: 9,
        pose: WorldPose {
            translation: [1.0, 2.0, 3.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        },
        joints: vec![JointStateValue {
            id: 11,
            position: -0.4,
            velocity: 0.8,
        }],
    }
}
#[test]
fn joint_state_native_model_fields_and_unrestricted_limits_roundtrip_without_fabricated_force() {
    let message = packet(&metadata(), 7, &sample()).expect("canonical supported packet");
    assert_eq!(Model::full_name(), "gz.msgs.Model");
    assert_eq!((message.name.as_str(), message.id), ("arm", 7));
    let header = message.header.as_ref().unwrap();
    assert_eq!(
        header.stamp,
        Some(gz_sim_proto::msgs::Time { sec: 2, nsec: 123 })
    );
    assert!(
        header.data.is_empty(),
        "native Model has no invented frame/seq"
    );
    let pose = message.pose.as_ref().unwrap();
    assert_eq!(pose.position.as_ref().unwrap().x, 1.0);
    assert!(pose.header.is_none() && pose.name.is_empty() && pose.id == 0);
    let joint = &message.joint[0];
    assert_eq!(
        (
            joint.name.as_str(),
            joint.id,
            joint.parent.as_str(),
            joint.child.as_str()
        ),
        ("hinge", 11, "base", "tip")
    );
    assert_eq!(
        joint.pose.as_ref().unwrap().position.as_ref().unwrap().x,
        0.3
    );
    assert!(joint.header.is_none() && joint.axis2.is_none());
    assert_eq!((joint.parent_id, joint.child_id), (0, 0));
    let axis = joint.axis1.as_ref().unwrap();
    assert_eq!(
        (axis.limit_lower, axis.limit_upper),
        (f64::NEG_INFINITY, f64::INFINITY)
    );
    assert_eq!(
        (axis.position, axis.velocity, axis.damping, axis.force),
        (-0.4, 0.8, 0.2, 0.0)
    );
    assert_eq!(axis.xyz.as_ref().unwrap().z, 1.0);
    assert!(axis.header.is_none() && axis.xyz_expressed_in.is_empty());
    assert_eq!(
        Model::decode(message.encode_to_vec().as_slice()).unwrap(),
        message
    );
}
#[test]
fn joint_state_invalid_wire_identity_metadata_measurements_and_stamp_never_encode() {
    for id in [0, u64::from(u32::MAX) + 1, u64::MAX] {
        assert!(packet(&metadata(), id, &sample()).is_none());
    }
    assert!(packet(&metadata(), u64::from(u32::MAX), &sample()).is_some());
    for case in 0..18 {
        let mut m = metadata();
        match case {
            0 => m.model_name.clear(),
            1 => m.topic.clear(),
            2 => m.joints.clear(),
            3 => m.joints[0].id = 0,
            4 => m.joints[0].id = u64::from(u32::MAX) + 1,
            5 => m.joints[0].name.clear(),
            6 => m.joints[0].parent.clear(),
            7 => m.joints[0].child.clear(),
            8 => m.joints[0].pose.translation[0] = f64::NAN,
            9 => m.joints[0].pose.rotation = [0.0; 4],
            10 => m.joints[0].axis = [0.0; 3],
            11 => m.joints[0].axis[0] = f64::INFINITY,
            12 => m.joints[0].lower = f64::NAN,
            13 => m.joints[0].lower = f64::INFINITY,
            14 => m.joints[0].upper = f64::NEG_INFINITY,
            15 => {
                m.joints[0].lower = 2.0;
                m.joints[0].upper = 1.0;
            }
            16 => m.joints[0].damping = -0.1,
            _ => m.joints[0].damping = f64::INFINITY,
        }
        assert!(packet(&m, 7, &sample()).is_none(), "metadata case {case}");
    }
    let mut duplicate = metadata();
    duplicate.joints.push(duplicate.joints[0].clone());
    assert!(!valid_metadata(&duplicate));
    duplicate.joints[1].id = 12;
    assert!(
        !valid_metadata(&duplicate),
        "different IDs cannot claim the same joint name"
    );
    duplicate.joints[1].name = "second".into();
    duplicate.joints.swap(0, 1);
    assert!(
        !valid_metadata(&duplicate),
        "wire order is ascending public ID"
    );
    let mut too_many = metadata();
    too_many.joints = vec![too_many.joints[0].clone(); 129];
    assert!(!valid_metadata(&too_many));
    for case in 0..7 {
        let mut s = sample();
        match case {
            0 => s.joints.clear(),
            1 => s.joints[0].id = 12,
            2 => s.joints[0].position = f64::NAN,
            3 => s.joints[0].velocity = f64::INFINITY,
            4 => s.pose.translation[0] = f64::INFINITY,
            5 => s.pose.rotation = [0.0; 4],
            _ => s.time = Duration::MAX,
        }
        assert!(packet(&metadata(), 7, &s).is_none(), "sample case {case}");
    }
}
fn server(xml: &str) -> Server {
    let simulation = load_world_with_sdf_step(
        xml,
        "server-m18.sdf",
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
        simulation,
        "default",
        ServerOptions {
            partition: format!("m18-{}", uuid::Uuid::new_v4()),
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
) -> Model {
    let sample = subscriber
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .expect("actual Model");
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.Model"
    );
    Model::decode(sample.payload().to_bytes().as_ref()).unwrap()
}
fn boundary(server: &Server) -> impl PartialEq + std::fmt::Debug + use<> {
    let bodies: Vec<_> = server
        .simulation
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<gz_sim_scene::BodyHandle>())
        .map(|handle| {
            (
                handle.id(),
                server
                    .simulation
                    .world()
                    .resource::<gz_physics::PhysicsBackend>()
                    .state(handle.id()),
            )
        })
        .collect();
    let streams: Vec<_> = server
        .joint_states
        .iter()
        .map(|s| {
            (
                s.entity,
                s.identity,
                s.joints.clone(),
                Arc::as_ptr(&s.publisher),
                s.last_sample,
            )
        })
        .collect();
    (
        server.simulation.elapsed(),
        server.simulation.epoch(),
        bodies,
        streams,
        format!("{:?}", server.counters()),
    )
}
#[test]
fn joint_state_real_wire_motion_positive_rate_equal_time_pause_loaded_reset_and_retirement() {
    let mut server = server(XML);
    let key = server.joint_states[0].endpoint.clone();
    // Same owned Session gives a completed local subscription, without adding a
    // production matching listener or changing native always-publication behavior.
    let subscriber = server
        .world_io
        .session
        .declare_subscriber(key)
        .wait()
        .unwrap();
    server.set_paused(true);
    server.update().unwrap();
    let initial = receive(&subscriber);
    assert_eq!(
        initial
            .header
            .as_ref()
            .unwrap()
            .stamp
            .as_ref()
            .unwrap()
            .nsec,
        0
    );
    assert_eq!(initial.joint.len(), 2);
    server.update().unwrap();
    assert_eq!(
        receive(&subscriber),
        initial,
        "equal-time paused ticks publish a fresh revision"
    );
    let joint_id = server.joint_states[0].metadata.joints[1].id;
    let before_refusal = boundary(&server);
    assert!(matches!(
        queue_joint_effort(&mut server.simulation, joint_id, 0.5),
        Err(gz_sim_scene::InputError::Physics(
            gz_physics::PhysicsError::Unsupported {
                engine: "rapier",
                capability: "moving-parent joint effort commands",
            }
        ))
    ));
    assert_eq!(boundary(&server), before_refusal);
    let joint_entity = entity_by_id(server.simulation.world(), joint_id).unwrap();
    let handle = server
        .simulation
        .world()
        .get::<gz_sim_scene::JointHandle>(joint_entity)
        .unwrap()
        .id();
    let before_motor = server
        .simulation
        .world()
        .resource::<gz_physics::PhysicsBackend>()
        .joint_state(handle)
        .unwrap();
    // The maintained velocity motor supports the moving-parent joint; configuring
    // it must not substitute fabricated positions or velocities for solver motion.
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
    assert_eq!(
        server
            .simulation
            .world()
            .resource::<gz_physics::PhysicsBackend>()
            .joint_state(handle),
        Some(before_motor)
    );
    server.set_paused(false);
    for _ in 0..9 {
        server.update().unwrap();
    }
    assert!(subscriber.try_recv().unwrap().is_none());
    server.update().unwrap();
    let moving = receive(&subscriber);
    assert_eq!(
        moving.header.as_ref().unwrap().stamp.as_ref().unwrap().nsec,
        10_000_000
    );
    let entity = server.joint_states[0].entity;
    let sample = server
        .simulation
        .world()
        .get::<JointStateSample>(entity)
        .unwrap();
    assert_eq!(
        packet(
            &server.joint_states[0].metadata,
            server.joint_states[0].identity,
            sample
        )
        .unwrap(),
        moving
    );
    assert!(
        moving
            .joint
            .iter()
            .any(|joint| joint.axis1.as_ref().unwrap().velocity.abs() > 0.0)
    );
    let tip = moving
        .joint
        .iter()
        .find(|joint| u64::from(joint.id) == joint_id)
        .unwrap();
    let axis = tip.axis1.as_ref().unwrap();
    assert!(
        axis.position.abs() > 0.0 && axis.velocity.abs() > 0.0,
        "the selected moving-parent tip must have actual relative motion"
    );
    server.set_paused(true);
    server.update().unwrap();
    let paused = receive(&subscriber);
    assert_eq!(paused, moving);
    let original = Arc::clone(&server.joint_states[0].publisher);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert_eq!(server.update().unwrap().epoch, 1);
    assert!(Arc::ptr_eq(&original, &server.joint_states[0].publisher));
    assert!(
        subscriber.try_recv().unwrap().is_none(),
        "reset transaction has not run PostStep"
    );
    server.update().unwrap();
    let reset = receive(&subscriber);
    assert_eq!(
        reset.header.as_ref().unwrap().stamp.as_ref().unwrap().nsec,
        0
    );
    for joint in &reset.joint {
        assert_eq!(
            (
                joint.axis1.as_ref().unwrap().position,
                joint.axis1.as_ref().unwrap().velocity
            ),
            (0.0, 0.0)
        );
    }
    let identity = server.joint_states[0].identity;
    let weak = Arc::downgrade(&original);
    drop(original);
    remove_model(&mut server.simulation, identity).unwrap();
    server.update().unwrap();
    assert!(server.joint_states.is_empty() && server.joint_states.capacity() == 0);
    assert!(weak.upgrade().is_none());
    assert!(subscriber.try_recv().unwrap().is_none());
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert_eq!(server.update().unwrap().epoch, 2);
    server.update().unwrap();
    receive(&subscriber);
    assert_eq!(server.joint_states[0].identity, identity);
    assert_eq!(server.counters().joint_state_failures, 0);
    assert_eq!(server.counters().invalid_joint_state_samples, 0);
    server.shutdown();
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}
#[test]
fn joint_state_no_subscriber_still_acquires_and_absent_family_owns_no_stream() {
    let mut active = server(XML);
    active.set_paused(true);
    active.update().unwrap();
    active.update().unwrap();
    assert_eq!(active.counters().joint_states_queued, 2);
    assert_eq!(
        active
            .simulation
            .world()
            .get::<JointStateSample>(active.joint_states[0].entity)
            .unwrap()
            .revision,
        2
    );
    let start = XML.find("    <plugin ").unwrap();
    let end = XML[start..].find("</plugin>").unwrap() + start + "</plugin>".len();
    let mut absent = server(&format!("{}{}", &XML[..start], &XML[end..]));
    for _ in 0..3 {
        absent.update().unwrap();
    }
    assert!(absent.joint_states.is_empty() && absent.joint_states.capacity() == 0);
    assert_eq!(absent.counters().joint_states_queued, 0);
}
#[test]
fn joint_state_identity_prepared_reset_and_endpoint_refusals_leave_native_boundary_unchanged() {
    for case in 0..10 {
        let mut server = server(XML);
        let entity = server.joint_states[0].entity;
        let mut occupied = BTreeSet::new();
        let mut prepared = prepare_reset(&server, &occupied).unwrap();
        match case {
            0 => {
                occupied.insert(server.joint_states[0].endpoint.clone());
            }
            1 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .remove::<EntityId>();
            }
            2 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(EntityKind::Visual);
            }
            3 => {
                server.simulation.world_mut().spawn(metadata());
            }
            4 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<JointStatePublication>(entity)
                    .unwrap()
                    .joints[0]
                    .id = u64::MAX;
            }
            5 => {
                prepared.clear();
            }
            6 => {
                let value = prepared.values().next().unwrap().clone();
                prepared.insert("@/extra@/stale".into(), value);
            }
            7 => {
                server
                    .simulation
                    .world_mut()
                    .get_mut::<JointStatePublication>(entity)
                    .unwrap()
                    .model_name = "drift".into();
            }
            8 => {
                let joint = server.joint_states[0].joints[0].1;
                server
                    .simulation
                    .world_mut()
                    .entity_mut(joint)
                    .remove::<bevy_ecs::prelude::ChildOf>();
            }
            _ => {
                let entry = prepared.values_mut().next().unwrap();
                entry.0 = if entry.0 == 1 { 2 } else { 1 };
                assert!(wire_id(entry.0).is_some());
                assert_eq!(entry.1, server.joint_states[0].metadata);
                assert!(Arc::ptr_eq(&entry.2, &server.joint_states[0].publisher));
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
        if [5, 6, 7, 9].contains(&case) {
            assert!(
                matches!(result, Err(Error::ResetJointStates)),
                "case {case}"
            );
        } else {
            assert!(
                matches!(result, Err(Error::JointStateIdentity)),
                "case {case}"
            );
        }
        assert_eq!(boundary(&server), before);
    }
}
#[test]
fn joint_state_invalid_stale_future_and_bounded_failures_are_observable_once_per_revision() {
    let mut server = server(XML);
    let entity = server.joint_states[0].entity;
    server.simulation.step_exact(10).unwrap();
    let original = server
        .simulation
        .world()
        .get::<JointStateSample>(entity)
        .unwrap()
        .clone();
    for case in 0..3 {
        let mut sample = original.clone();
        sample.revision = 100 + case;
        match case {
            0 => sample.epoch += 1,
            1 => sample.time += Duration::from_secs(1),
            _ => sample.joints[0].velocity = f64::NAN,
        }
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(sample);
        server.publish_joint_states();
        server.publish_joint_states();
    }
    assert_eq!(server.counters().invalid_joint_state_samples, 3);
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(original);
    server.world_io.max_message_bytes = 1;
    server.publish_joint_states();
    assert_eq!(server.counters().joint_state_failures, 1);
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
fn joint_state_removed_or_reused_joint_identity_cannot_publish_or_retain_old_stream() {
    let mut server = server(XML);
    let (identity, entity) = server.joint_states[0].joints[0];
    let owner = server.joint_states[0].entity;
    let weak = Arc::downgrade(&server.joint_states[0].publisher);
    let reserved = gz_sim_core::reserve_id(server.simulation.world(), entity).unwrap();
    server.simulation.world_mut().despawn(entity);
    let replacement = server
        .simulation
        .world_mut()
        .spawn((EntityKind::Joint, bevy_ecs::prelude::ChildOf(owner)))
        .id();
    assert_eq!(
        gz_sim_core::restore_id(server.simulation.world_mut(), replacement, &reserved).unwrap(),
        identity
    );
    server.publish_joint_states();
    assert_eq!(server.counters().joint_states_queued, 0);
    server.retire_joint_states();
    assert!(server.joint_states.is_empty());
    assert!(weak.upgrade().is_none());
}

fn family_key(server: &Server, family: PublicationFamily) -> String {
    match family {
        PublicationFamily::Altimeter => server.altimeters[0].endpoint.clone(),
        PublicationFamily::AirPressure => server.air_pressures[0].endpoint.clone(),
        PublicationFamily::Magnetometer => server.magnetometers[0].endpoint.clone(),
        PublicationFamily::JointState => server.joint_states[0].endpoint.clone(),
        PublicationFamily::AirSpeed => server.air_speeds[0].endpoint.clone(),
        PublicationFamily::PosePublisher => server
            .network_config
            .topic_key(&options::as_valid_topic(
                &server.pose_publishers[0].metadata.topic,
            ))
            .unwrap(),
        PublicationFamily::LogicalCamera => server.logical_cameras[0].image_endpoint.clone(),
    }
}
fn topic_mut(server: &mut Server, family: PublicationFamily, value: String) {
    match family {
        PublicationFamily::Altimeter => {
            let e = server
                .simulation
                .world()
                .iter_entities()
                .find(|e| e.contains::<gz_sim_scene::AltimeterPublication>())
                .unwrap()
                .id();
            server
                .simulation
                .world_mut()
                .get_mut::<gz_sim_scene::AltimeterPublication>(e)
                .unwrap()
                .topic = value;
        }
        PublicationFamily::AirPressure => {
            let e = server
                .simulation
                .world()
                .iter_entities()
                .find(|e| e.contains::<gz_sim_scene::AirPressurePublication>())
                .unwrap()
                .id();
            server
                .simulation
                .world_mut()
                .get_mut::<gz_sim_scene::AirPressurePublication>(e)
                .unwrap()
                .topic = value;
        }
        PublicationFamily::Magnetometer => {
            let e = server
                .simulation
                .world()
                .iter_entities()
                .find(|e| e.contains::<gz_sim_scene::MagnetometerPublication>())
                .unwrap()
                .id();
            server
                .simulation
                .world_mut()
                .get_mut::<gz_sim_scene::MagnetometerPublication>(e)
                .unwrap()
                .topic = value;
        }
        PublicationFamily::JointState => {
            let e = server.joint_states[0].entity;
            server
                .simulation
                .world_mut()
                .get_mut::<JointStatePublication>(e)
                .unwrap()
                .topic = value;
        }
        PublicationFamily::AirSpeed => {
            let entity = server
                .simulation
                .world()
                .iter_entities()
                .find(|e| e.contains::<gz_sim_scene::AirSpeedPublication>())
                .unwrap()
                .id();
            server
                .simulation
                .world_mut()
                .get_mut::<gz_sim_scene::AirSpeedPublication>(entity)
                .unwrap()
                .topic = value;
        }
        PublicationFamily::PosePublisher => {
            let entity = server
                .simulation
                .world()
                .iter_entities()
                .find(|e| e.contains::<gz_sim_scene::PosePublication>())
                .unwrap()
                .id();
            server
                .simulation
                .world_mut()
                .get_mut::<gz_sim_scene::PosePublication>(entity)
                .unwrap()
                .topic = value;
        }
        PublicationFamily::LogicalCamera => {
            let entity = server
                .simulation
                .world()
                .iter_entities()
                .find(|e| e.contains::<gz_sim_scene::LogicalCameraPublication>())
                .unwrap()
                .id();
            let mut metadata = server
                .simulation
                .world_mut()
                .get_mut::<gz_sim_scene::LogicalCameraPublication>(entity)
                .unwrap();
            metadata.frustum_topic = format!("{value}/frustum");
            metadata.topic = value;
        }
    }
}
fn assert_family_refuses(
    server: &Server,
    family: PublicationFamily,
    occupied: &BTreeSet<String>,
    options: &ServerOptions,
) {
    match family {
        PublicationFamily::Altimeter => assert!(matches!(
            altimeter::streams(
                &server.simulation,
                &server.world_io,
                options,
                occupied,
                None
            ),
            Err(Error::AltimeterIdentity)
        )),
        PublicationFamily::AirPressure => assert!(matches!(
            air_pressure::streams(
                &server.simulation,
                &server.world_io,
                options,
                occupied,
                None
            ),
            Err(Error::AirPressureIdentity)
        )),
        PublicationFamily::Magnetometer => assert!(matches!(
            magnetometer::streams(
                &server.simulation,
                &server.world_io,
                options,
                occupied,
                None
            ),
            Err(Error::MagnetometerIdentity)
        )),
        PublicationFamily::JointState => assert!(matches!(
            streams(
                &server.simulation,
                &server.world_io,
                options,
                occupied,
                None
            ),
            Err(Error::JointStateIdentity)
        )),
        PublicationFamily::AirSpeed => assert!(matches!(
            air_speed::streams(
                &server.simulation,
                &server.world_io,
                options,
                occupied,
                None
            ),
            Err(Error::AirSpeedIdentity)
        )),
        PublicationFamily::PosePublisher => assert!(matches!(
            pose_publisher::streams(
                &server.simulation,
                &server.world_io,
                options,
                occupied,
                None
            ),
            Err(Error::PosePublisherIdentity)
        )),
        PublicationFamily::LogicalCamera => assert!(matches!(
            logical_camera::streams(
                &server.simulation,
                &server.world_io,
                options,
                occupied,
                None
            ),
            Err(Error::LogicalCameraIdentity)
        )),
    }
}
fn all_family_xml() -> String {
    let mut xml = XML.to_owned();
    for (source, owner) in [
        (
            include_str!("../../../gz-sim-scene/tests/fixtures/m13_altimeter.sdf"),
            "alt_rig",
        ),
        (
            include_str!("../../../gz-sim-scene/tests/fixtures/m15_air_pressure.sdf"),
            "pressure_rig",
        ),
        (
            include_str!("../../../gz-sim-scene/tests/fixtures/m17_magnetometer.sdf"),
            "mag_rig",
        ),
    ] {
        let start = source.find("  <model name='rig'>").unwrap();
        let end = source.rfind("</model>").unwrap() + "</model>".len();
        let model = source[start..end].replace("name='rig'", &format!("name='{owner}'"));
        xml = xml.replace("</world>", &format!("{model}</world>"));
    }
    for (source, quote, owner) in [
        (
            include_str!("../../../gz-sim-scene/tests/fixtures/m26_air_speed.sdf"),
            "'",
            "speed_rig",
        ),
        (
            include_str!("../../../gz-sim-scene/tests/fixtures/m27_pose_publisher.sdf"),
            "\"",
            "pose_rig",
        ),
    ] {
        let start = source
            .find(&format!("<model name={quote}rig{quote}"))
            .unwrap();
        let end = source.rfind("</model>").unwrap() + "</model>".len();
        let model = source[start..end].replace(
            &format!("name={quote}rig{quote}"),
            &format!("name={quote}{owner}{quote}"),
        );
        xml = xml.replace("</world>", &format!("{model}</world>"));
    }
    let source = include_str!("../../../gz-sim-scene/tests/fixtures/m30_logical_camera.sdf");
    let start = source.find("  <model name='rig'>").unwrap();
    let end = start + source[start..].find("</model>").unwrap() + "</model>".len();
    let model = source[start..end].replace("name='rig'", "name='camera_rig'");
    xml = xml.replace("</world>", &format!("<plugin filename='gz-sim-logical-camera-system' name='gz::sim::systems::LogicalCamera'/>{model}</world>"));
    xml
}
#[test]
fn all_typed_publications_reserve_live_and_removed_initial_peers_in_both_directions_before_io() {
    let families = [
        PublicationFamily::Altimeter,
        PublicationFamily::AirPressure,
        PublicationFamily::Magnetometer,
        PublicationFamily::JointState,
        PublicationFamily::AirSpeed,
        PublicationFamily::PosePublisher,
        PublicationFamily::LogicalCamera,
    ];
    let topics = [
        "/m13/altimeter",
        "/m15/air_pressure",
        "/m17/magnetometer",
        "/m18/joint_state",
        "/m26/air_speed",
        "/m27/pose",
        "/m30/logical_camera",
    ];
    let xml = all_family_xml();
    let mut live = server(&xml);
    for (index, &own) in families.iter().enumerate() {
        let occupied =
            typed_reserved_keys(&live.simulation, &live.world_io, &live.network_config, own)
                .unwrap();
        for (other, &peer) in families.iter().enumerate() {
            if own == peer {
                continue;
            }
            assert!(occupied.contains(&family_key(&live, peer)));
            topic_mut(&mut live, own, topics[other].into());
            let before = boundary(&live);
            assert_family_refuses(&live, own, &occupied, &live.network_config);
            assert_eq!(boundary(&live), before);
            topic_mut(&mut live, own, topics[index].into());
            let mut remapped = live.network_config.clone();
            remapped
                .add_topic_remap(topics[index], topics[other])
                .unwrap();
            let occupied =
                typed_reserved_keys(&live.simulation, &live.world_io, &remapped, own).unwrap();
            assert_family_refuses(&live, own, &occupied, &remapped);
        }
        for reserved in [
            "/clock",
            "/world/default/clock",
            "/world/default/pose/info",
            "/world/default/control",
            "/world/default/remove",
            "/world/default/remove/blocking",
        ] {
            assert!(occupied.contains(&live.network_config.topic_key(reserved).unwrap()));
            topic_mut(&mut live, own, reserved.into());
            assert_family_refuses(&live, own, &occupied, &live.network_config);
        }
        topic_mut(&mut live, own, topics[index].into());
    }
    live.shutdown();
    for (other, &peer) in families.iter().enumerate() {
        let mut server = server(&xml);
        let owner = [
            "default::alt_rig",
            "default::pressure_rig",
            "default::mag_rig",
            "default::arm",
            "default::speed_rig",
            "default::pose_rig",
            "default::camera_rig",
        ][other];
        let id = server
            .simulation
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<gz_sim_scene::Name>()
                    .is_some_and(|name| name.0 == owner)
            })
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        let peer_key = family_key(&server, peer);
        remove_model(&mut server.simulation, id).unwrap();
        server.retire_altimeters();
        server.retire_air_pressures();
        server.retire_magnetometers();
        server.retire_joint_states();
        server.retire_air_speeds();
        server.retire_pose_publishers();
        server.retire_logical_cameras();
        for (index, &own) in families.iter().enumerate() {
            if own == peer {
                continue;
            }
            let occupied = typed_reserved_keys(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                own,
            )
            .unwrap();
            assert!(
                occupied.contains(&peer_key),
                "initial blueprint preserves removed endpoint"
            );
            topic_mut(&mut server, own, topics[other].into());
            let before = boundary(&server);
            assert_family_refuses(&server, own, &occupied, &server.network_config);
            assert_eq!(boundary(&server), before);
            topic_mut(&mut server, own, topics[index].into());
        }
    }
}

#[test]
fn new_families_cannot_steal_removed_initial_peer_topics_before_reset_changes_or_io() {
    for (removed_name, survivor, stolen_topic) in [
        (
            "default::pose_rig",
            PublicationFamily::AirSpeed,
            "/m27/pose",
        ),
        (
            "default::speed_rig",
            PublicationFamily::PosePublisher,
            "/m26/air_speed",
        ),
    ] {
        let mut active = server(&all_family_xml());
        let removed = active
            .simulation
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<gz_sim_scene::Name>()
                    .is_some_and(|n| n.0 == removed_name)
            })
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        remove_model(&mut active.simulation, removed).unwrap();
        active.retire_air_speeds();
        active.retire_pose_publishers();
        topic_mut(&mut active, survivor, stolen_topic.into());
        let before = boundary(&active);
        let speed_leases: Vec<_> = active
            .air_speeds
            .iter()
            .map(|s| Arc::as_ptr(&s.publisher))
            .collect();
        let pose_leases: Vec<_> = active
            .pose_publishers
            .iter()
            .map(|s| Arc::as_ptr(&s.publisher))
            .collect();
        let generation = active
            .world_io
            .epoch
            .load(std::sync::atomic::Ordering::Acquire);
        assert!(matches!(
            active.reset_scene(),
            Err(Error::AirSpeedIdentity | Error::PosePublisherIdentity)
        ));
        assert_eq!(
            boundary(&active),
            before,
            "collision must precede the reset transaction"
        );
        assert_eq!(
            active
                .world_io
                .epoch
                .load(std::sync::atomic::Ordering::Acquire),
            generation
        );
        assert_eq!(
            active
                .air_speeds
                .iter()
                .map(|s| Arc::as_ptr(&s.publisher))
                .collect::<Vec<_>>(),
            speed_leases
        );
        assert_eq!(
            active
                .pose_publishers
                .iter()
                .map(|s| Arc::as_ptr(&s.publisher))
                .collect::<Vec<_>>(),
            pose_leases
        );
        assert!(!active.reset_failed && !active.pending_reset);
        active.shutdown();
    }
}

#[test]
fn secondary_pose_and_camera_keys_remain_reserved_live_removed_and_before_reset_io() {
    let xml = all_family_xml().replace(
        "<topic>/m27/pose</topic><update_frequency>100</update_frequency>",
        "<topic>/m27/pose</topic><update_frequency>100</update_frequency><static_publisher>true</static_publisher><static_update_frequency>25</static_update_frequency>",
    );
    let families = [
        (PublicationFamily::Altimeter, "/m13/altimeter"),
        (PublicationFamily::AirPressure, "/m15/air_pressure"),
        (PublicationFamily::Magnetometer, "/m17/magnetometer"),
        (PublicationFamily::JointState, "/m18/joint_state"),
        (PublicationFamily::AirSpeed, "/m26/air_speed"),
        (PublicationFamily::PosePublisher, "/m27/pose"),
        (PublicationFamily::LogicalCamera, "/m30/logical_camera"),
    ];
    for (peer, raw_key, owner_name) in [
        (
            PublicationFamily::PosePublisher,
            "/m27/pose_static",
            "default::pose_rig",
        ),
        (
            PublicationFamily::LogicalCamera,
            "/m30/logical_camera/frustum",
            "default::camera_rig",
        ),
    ] {
        for removed in [false, true] {
            let mut active = server(&xml);
            let secondary = active.network_config.topic_key(raw_key).unwrap();
            if removed {
                let owner = active
                    .simulation
                    .world()
                    .iter_entities()
                    .find(|e| {
                        e.get::<gz_sim_scene::Name>()
                            .is_some_and(|n| n.0 == owner_name)
                    })
                    .unwrap()
                    .get::<EntityId>()
                    .unwrap()
                    .get();
                remove_model(&mut active.simulation, owner).unwrap();
                active.retire_pose_publishers();
                active.retire_logical_cameras();
            }
            for (own, original) in families {
                if own == peer {
                    continue;
                }
                let occupied = typed_reserved_keys(
                    &active.simulation,
                    &active.world_io,
                    &active.network_config,
                    own,
                )
                .unwrap();
                assert!(occupied.contains(&secondary));
                let mut remapped = active.network_config.clone();
                remapped.add_topic_remap(original, raw_key).unwrap();
                let remapped_occupied =
                    typed_reserved_keys(&active.simulation, &active.world_io, &remapped, own)
                        .unwrap();
                let before = boundary(&active);
                assert_family_refuses(&active, own, &remapped_occupied, &remapped);
                assert_eq!(boundary(&active), before);
                topic_mut(&mut active, own, raw_key.into());
                let before = boundary(&active);
                let generation = active
                    .world_io
                    .epoch
                    .load(std::sync::atomic::Ordering::Acquire);
                let pose_leases: Vec<_> = active
                    .pose_publishers
                    .iter()
                    .map(|s| Arc::as_ptr(&s.publisher))
                    .collect();
                let camera_leases: Vec<_> = active
                    .logical_cameras
                    .iter()
                    .map(|s| (Arc::as_ptr(&s.image), Arc::as_ptr(&s.frustum)))
                    .collect();
                assert_family_refuses(&active, own, &occupied, &active.network_config);
                assert!(matches!(
                    active.reset_scene(),
                    Err(Error::AirSpeedIdentity
                        | Error::PosePublisherIdentity
                        | Error::LogicalCameraIdentity)
                ));
                assert_eq!(boundary(&active), before);
                assert_eq!(
                    active
                        .world_io
                        .epoch
                        .load(std::sync::atomic::Ordering::Acquire),
                    generation
                );
                assert_eq!(
                    active
                        .pose_publishers
                        .iter()
                        .map(|s| Arc::as_ptr(&s.publisher))
                        .collect::<Vec<_>>(),
                    pose_leases
                );
                assert_eq!(
                    active
                        .logical_cameras
                        .iter()
                        .map(|s| (Arc::as_ptr(&s.image), Arc::as_ptr(&s.frustum)))
                        .collect::<Vec<_>>(),
                    camera_leases
                );
                assert!(!active.reset_failed && !active.pending_reset);
                topic_mut(&mut active, own, original.into());
            }
        }
    }
}
#[test]
fn paired_channels_cannot_alias_each_other_after_remapping_before_world_io() {
    let xml = all_family_xml().replace(
        "<topic>/m27/pose</topic><update_frequency>100</update_frequency>",
        "<topic>/m27/pose</topic><update_frequency>100</update_frequency><static_publisher>true</static_publisher>",
    );
    let mut active = server(&xml);
    for (from, to) in [
        ("/m27/pose_static", "/m27/pose"),
        ("/m30/logical_camera/frustum", "/m30/logical_camera"),
    ] {
        let mut options = active.network_config.clone();
        options.add_topic_remap(from, to).unwrap();
        let before = boundary(&active);
        assert!(matches!(
            validate_air_speed_and_pose_topics(&active.simulation, &active.world_io, &options),
            Err(Error::PosePublisherIdentity | Error::LogicalCameraIdentity)
        ));
        assert_eq!(boundary(&active), before);
    }
    // Independent remapping uses the raw suffix, rather than appending a suffix
    // to the already-mapped dynamic endpoint.
    active
        .network_config
        .add_topic_remap("/m27/pose", "/chosen/dynamic")
        .unwrap();
    active
        .network_config
        .add_topic_remap("/m27/pose_static", "/chosen/static")
        .unwrap();
    let prepared = pose_publisher::prepare_reset(
        &active,
        &pose_publisher_reserved_keys(&active.simulation, &active.world_io, &active.network_config)
            .unwrap(),
    )
    .unwrap();
    assert_eq!(prepared.len(), 2);
    assert!(prepared.contains_key(&active.network_config.topic_key("/chosen/dynamic").unwrap()));
    assert!(prepared.contains_key(&active.network_config.topic_key("/chosen/static").unwrap()));
}
