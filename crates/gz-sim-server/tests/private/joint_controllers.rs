// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::prelude::{Commands, Query};
use gz_sim_core::Phase;
use gz_sim_proto::{
    msgs::{Double, WorldReset},
    prost::Name as _,
};
use gz_sim_scene::{
    JointControllerMode, JointSample, Name, PhysicsEngine, load_world_with_sdf_step, remove_model,
};
use std::time::Instant;

const XML: &str = include_str!("../../../../examples/m9_joint_controllers.sdf");
const POSITION: &str = "/model/arm/joint/base_hinge/0/cmd_pos";
const VELOCITY: &str = "/model/arm/joint/tip_hinge/cmd_vel";

fn config(endpoint: Option<&str>) -> zenoh::Config {
    let mut config = zenoh::Config::default();
    let connect = endpoint.map_or_else(|| "[]".into(), |value| format!("[\"{value}\"]"));
    for (key, value) in [
        ("mode", "\"peer\""),
        (
            "listen/endpoints",
            if endpoint.is_some() {
                "[]"
            } else {
                "[\"tcp/127.0.0.1:0\"]"
            },
        ),
        ("connect/endpoints", &connect),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(key, value).unwrap();
    }
    config
}
fn loaded(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m9-server.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn build(xml: &str) -> (Server, zenoh::Session) {
    let observer = zenoh::open(config(None)).wait().unwrap();
    let locators: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|value| value.to_string())
        .collect();
    assert_eq!(locators.len(), 1);
    assert_ne!(locators[0], "tcp/127.0.0.1:0");
    let server = Server::with_zenoh_config(
        loaded(xml),
        "default",
        ServerOptions {
            capacity: 8192,
            partition: format!("joint-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        config(Some(&locators[0])),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    (server, observer)
}
fn wait(mut condition: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !condition() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
}
fn stream(server: &Server, mode: JointControllerMode) -> &Stream {
    server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .iter()
        .find(|stream| stream.metadata.mode == mode)
        .unwrap()
}
fn state(server: &Server, mode: JointControllerMode) -> (f64, f64) {
    let sample = server
        .simulation
        .world()
        .get::<JointSample>(stream(server, mode).entity)
        .unwrap();
    assert_eq!(
        (sample.epoch, sample.time),
        (server.simulation.epoch(), server.simulation.elapsed())
    );
    (sample.state.position, sample.state.velocity)
}
fn reset(server: &mut Server) {
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.prepare_io().unwrap();
    let info = server.tick().unwrap();
    assert_eq!(info.sim_time, Duration::ZERO);
    assert!(info.dt.is_zero());
}

#[test]
fn canonical_doubles_move_both_joints_without_output_interest_and_preserve_pause_reset_ids() {
    let (mut server, observer) = build(XML);
    assert_eq!(server.joint_controllers.as_ref().unwrap().streams.len(), 2);
    let position = observer
        .declare_publisher(server.options().topic_key(POSITION).unwrap())
        .wait()
        .unwrap();
    let velocity = observer
        .declare_publisher(server.options().topic_key(VELOCITY).unwrap())
        .wait()
        .unwrap();
    wait(|| {
        position.matching_status().wait().unwrap().matching()
            && velocity.matching_status().wait().unwrap().matching()
    });
    let identities: Vec<_> = server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .iter()
        .map(|stream| (stream.identity, stream.metadata.clone()))
        .collect();
    server.set_paused(true);
    for (publisher, value) in [(&position, 0.75), (&velocity, 0.5)] {
        publisher
            .put(
                Double {
                    data: value,
                    ..Default::default()
                }
                .encode_to_vec(),
            )
            .attachment(Double::full_name())
            .wait()
            .unwrap();
    }
    wait(|| {
        assert!(server.update().unwrap().dt.is_zero());
        server.counters().joint_controller_commands_applied == 2
    });
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
    assert_eq!(state(&server, JointControllerMode::Position), (0.0, 0.0));
    assert_eq!(state(&server, JointControllerMode::Velocity), (0.0, 0.0));
    server.set_paused(false);
    for tick in 1..=1000 {
        let info = server.update().unwrap();
        assert_eq!(info.sim_time, Duration::from_millis(tick));
    }
    let (q, qdot) = state(&server, JointControllerMode::Position);
    assert!(
        q.is_finite() && qdot.is_finite() && (q - 0.75).abs() < 0.1,
        "position {q}, velocity {qdot}"
    );
    let (q, qdot) = state(&server, JointControllerMode::Velocity);
    assert!(
        q > 0.1 && qdot > 0.1 && qdot.is_finite(),
        "position {q}, velocity {qdot}"
    );
    assert_eq!(server.counters().joint_controller_messages_received, 2);
    assert_eq!(server.counters().joint_controller_messages_rejected, 0);
    assert_eq!(
        (
            server.counters().odometries_queued,
            server.counters().imus_queued,
            server.counters().lidars_queued
        ),
        (0, 0, 0)
    );
    let leases: Vec<_> = server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .iter()
        .map(|stream| Arc::downgrade(&stream._lease))
        .collect();
    reset(&mut server);
    assert_eq!(server.simulation.epoch(), 1);
    assert!(leases.iter().all(|lease| lease.upgrade().is_none()));
    let restored: Vec<_> = server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .iter()
        .map(|stream| (stream.identity, stream.metadata.clone()))
        .collect();
    // Entity traversal order is not a stream identity contract.
    assert_eq!(
        identities.into_iter().collect::<Metadata>(),
        restored.into_iter().collect::<Metadata>()
    );
    assert_eq!(state(&server, JointControllerMode::Position), (0.0, 0.0));
    assert_eq!(state(&server, JointControllerMode::Velocity), (0.0, 0.0));
    server.set_paused(false);
    for _ in 0..100 {
        server.update().unwrap();
    }
    for mode in [JointControllerMode::Position, JointControllerMode::Velocity] {
        let (q, qdot) = state(&server, mode);
        assert!(q.abs() < 1e-12 && qdot.abs() < 1e-12);
    }
    server.shutdown();
    assert!(server.joint_controllers.is_none());
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}

#[test]
fn malformed_and_old_epoch_packets_preserve_the_last_target_and_each_stream_budget() {
    let (mut server, observer) = build(XML);
    let publisher = observer
        .declare_publisher(server.options().topic_key(VELOCITY).unwrap())
        .wait()
        .unwrap();
    wait(|| publisher.matching_status().wait().unwrap().matching());
    server.set_paused(true);
    publisher
        .put(
            Double {
                data: 0.4,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .attachment(Double::full_name())
        .wait()
        .unwrap();
    wait(|| {
        server.update().unwrap();
        server.counters().joint_controller_commands_applied == 1
    });
    for (bytes, kind) in [
        (vec![255], Some(Double::full_name())),
        (vec![], None),
        (vec![], Some("gz.msgs.Twist".into())),
        (
            Double {
                data: f64::NAN,
                ..Default::default()
            }
            .encode_to_vec(),
            Some(Double::full_name()),
        ),
        (
            Double {
                data: f64::INFINITY,
                ..Default::default()
            }
            .encode_to_vec(),
            Some(Double::full_name()),
        ),
    ] {
        let before = server.counters().joint_controller_messages_received;
        let publication = publisher.put(bytes);
        if let Some(kind) = kind {
            publication.attachment(kind).wait().unwrap();
        } else {
            publication.wait().unwrap();
        }
        wait(|| {
            assert!(server.update().unwrap().dt.is_zero());
            server.counters().joint_controller_messages_received == before + 1
        });
    }
    let mut inputs = Vec::new();
    for stream in &mut server.joint_controllers.as_mut().unwrap().streams {
        let (input, receiver) = mpsc::sync_channel(4);
        stream.receiver = receiver;
        for _ in 0..3 {
            input
                .try_send((server.simulation.epoch() + 1, Some(-4.0)))
                .unwrap();
        }
        inputs.push(input);
    }
    let received = server.counters().joint_controller_messages_received;
    server.poll_joint_controllers().unwrap();
    assert_eq!(
        server.counters().joint_controller_messages_received - received,
        4
    );
    server.poll_joint_controllers().unwrap();
    assert_eq!(
        server.counters().joint_controller_messages_received - received,
        6
    );
    assert_eq!(server.counters().joint_controller_commands_applied, 1);
    assert_eq!(server.counters().joint_controller_messages_rejected, 11);
    // CPU admission also defends against an invalid internal scalar handoff.
    for input in &inputs {
        input
            .try_send((server.simulation.epoch(), Some(f64::NAN)))
            .unwrap();
    }
    server.poll_joint_controllers().unwrap();
    assert_eq!(server.counters().joint_controller_messages_rejected, 13);
    assert_eq!(server.counters().joint_controller_commands_applied, 1);
    server.set_paused(false);
    for _ in 0..300 {
        server.update().unwrap();
    }
    let (q, qdot) = state(&server, JointControllerMode::Velocity);
    assert!(
        q > 0.03 && qdot > 0.1,
        "last valid target lost: {q}, {qdot}"
    );
    server.set_paused(true);
    // Valid old-generation handoffs queued before reset cannot enter new receivers.
    for input in &inputs {
        input
            .try_send((server.simulation.epoch(), Some(3.0)))
            .unwrap();
    }
    let applied = server.counters().joint_controller_commands_applied;
    reset(&mut server);
    assert!(
        inputs
            .iter()
            .all(|input| input.try_send((0, Some(3.0))).is_err())
    );
    server.set_paused(false);
    for _ in 0..50 {
        server.update().unwrap();
    }
    assert_eq!(server.counters().joint_controller_commands_applied, applied);
    for mode in [JointControllerMode::Position, JointControllerMode::Velocity] {
        let (q, qdot) = state(&server, mode);
        assert!(q.abs() < 1e-12 && qdot.abs() < 1e-12);
    }
}

#[test]
fn typed_runtime_owner_drift_is_terminal_before_physics_and_releases_all_input_leases() {
    for mutation in 0..6 {
        let (mut server, observer) = build(XML);
        server.set_paused(true);
        let controller = stream(&server, JointControllerMode::Position);
        let entity = controller.entity;
        let metadata = controller.metadata.clone();
        let other = stream(&server, JointControllerMode::Velocity).entity;
        let leases: Vec<_> = server
            .joint_controllers
            .as_ref()
            .unwrap()
            .streams
            .iter()
            .map(|stream| Arc::downgrade(&stream._lease))
            .collect();
        let publisher = observer
            .declare_publisher(server.options().topic_key(POSITION).unwrap())
            .wait()
            .unwrap();
        wait(|| publisher.matching_status().wait().unwrap().matching());
        match mutation {
            0 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(EntityKind::Model);
            }
            1 => {
                let mut changed = metadata;
                changed.topic.push_str("/drift");
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(changed);
            }
            2 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .remove::<JointControllerConfiguration>();
            }
            3 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .remove::<EntityId>();
            }
            4 => {
                let handle = *server.simulation.world().get::<JointHandle>(other).unwrap();
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(handle);
            }
            _ => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .remove::<JointHandle>();
            }
        }
        assert!(matches!(server.tick(), Err(Error::JointControllerIdentity)));
        assert_eq!(server.simulation.elapsed(), Duration::ZERO);
        assert!(
            server.joint_controllers.is_none()
                && leases.iter().all(|lease| lease.upgrade().is_none())
        );
        assert!(matches!(server.update(), Err(Error::ResetStreams)));
        assert!(!server.apply_local_control(WorldControl::default()));
        wait(|| !publisher.matching_status().wait().unwrap().matching());
    }
}

#[test]
fn initial_metadata_endpoint_collisions_duplicate_owners_and_reset_drift_fail_closed() {
    let (mut server, _observer) = build(XML);
    let expected = initial(&server.simulation).unwrap();
    let entity = stream(&server, JointControllerMode::Position).entity;
    let original = stream(&server, JointControllerMode::Position)
        .metadata
        .clone();
    for mutation in 0..7 {
        let mut changed = original.clone();
        match mutation {
            0 => changed.model_name.clear(),
            1 => changed.joint_name.clear(),
            2 => changed.topic = "relative".into(),
            3 => changed.topic = "/bad topic".into(),
            4 => changed.topic = VELOCITY.into(),
            5 => changed.topic = "/clock".into(),
            _ => changed.mode = JointControllerMode::Velocity,
        }
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(changed);
        assert!(
            prepare(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &expected
            )
            .is_err()
        );
    }
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(original.clone());
    let extra = server.simulation.world_mut().spawn(original).id();
    assert!(matches!(
        prepare(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &expected
        ),
        Err(Error::JointControllerIdentity)
    ));
    server.simulation.world_mut().despawn(extra);
    server.simulation.add_systems(
        Phase::Reset,
        |values: Query<(Entity, &JointControllerConfiguration)>, mut commands: Commands| {
            for (entity, configuration) in &values {
                let mut changed = configuration.clone();
                changed.topic.push_str("/reset-drift");
                commands.entity(entity).insert(changed);
            }
        },
    );
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(
        server.prepare_io(),
        Err(Error::ResetJointControllers)
    ));
    assert!(server.joint_controllers.is_none());
    assert!(matches!(server.update(), Err(Error::ResetStreams)));
}

#[test]
fn absent_controller_system_has_no_input_state_and_model_removal_retires_both_streams() {
    let (mut absent, _) = build(include_str!("../../../../examples/mvp_primitives.sdf"));
    assert!(absent.joint_controllers.is_none());
    absent.set_paused(true);
    for _ in 0..3 {
        absent.update().unwrap();
    }
    assert_eq!(
        (
            absent.counters().joint_controller_messages_received,
            absent.counters().joint_controller_commands_applied
        ),
        (0, 0)
    );
    let (mut server, observer) = build(XML);
    let publisher = observer
        .declare_publisher(server.options().topic_key(POSITION).unwrap())
        .wait()
        .unwrap();
    wait(|| publisher.matching_status().wait().unwrap().matching());
    let model = server
        .simulation
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::arm")
                && entity.get::<EntityKind>() == Some(&EntityKind::Model)
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    server.set_paused(true);
    remove_model(&mut server.simulation, model).unwrap();
    server.update().unwrap();
    assert!(server.joint_controllers.is_none());
    assert_eq!(server.counters().joint_controller_commands_applied, 0);
    wait(|| !publisher.matching_status().wait().unwrap().matching());
}

#[test]
fn scene_physics_refusal_and_disconnected_ingress_are_terminal_not_bad_command_rejections() {
    for disconnected in [false, true] {
        let (mut server, _observer) = build(XML);
        let owner = stream(&server, JointControllerMode::Position).entity;
        let (input, receiver) = mpsc::sync_channel(1);
        let active = server
            .joint_controllers
            .as_mut()
            .unwrap()
            .streams
            .iter_mut()
            .find(|stream| stream.entity == owner)
            .unwrap();
        active.receiver = receiver;
        if disconnected {
            drop(input);
        } else {
            input
                .try_send((server.simulation.epoch(), Some(0.5)))
                .unwrap();
            let model = server
                .simulation
                .world()
                .iter_entities()
                .find(|entity| {
                    entity
                        .get::<Name>()
                        .is_some_and(|name| name.0 == "default::arm")
                })
                .unwrap()
                .id();
            server
                .simulation
                .world_mut()
                .entity_mut(model)
                .insert(EntityKind::Link);
        }
        // The stream itself remains typed correctly. Scene checks its deeper native owner.
        let error = server.poll_joint_controllers().unwrap_err();
        if disconnected {
            assert!(matches!(error, Error::WorldClosed));
        } else {
            assert!(matches!(error, Error::JointControllerIdentity));
        }
        assert_eq!(server.counters().joint_controller_messages_rejected, 0);
        assert_eq!(server.counters().joint_controller_commands_applied, 0);
        assert!(server.joint_controllers.is_none());
        assert_eq!(server.simulation.elapsed(), Duration::ZERO);
        assert!(matches!(server.tick(), Err(Error::ResetStreams)));
    }
}

#[test]
fn pending_reset_removal_retires_only_its_stream_and_keeps_the_other_declared() {
    let (mut server, observer) = build(XML);
    let entity = stream(&server, JointControllerMode::Position).entity;
    let weak = Arc::downgrade(&stream(&server, JointControllerMode::Position)._lease);
    let retired = observer
        .declare_publisher(server.options().topic_key(POSITION).unwrap())
        .wait()
        .unwrap();
    let retained = observer
        .declare_publisher(server.options().topic_key(VELOCITY).unwrap())
        .wait()
        .unwrap();
    wait(|| {
        retired.matching_status().wait().unwrap().matching()
            && retained.matching_status().wait().unwrap().matching()
    });
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(PendingResetRemoval);
    server.reconcile_joint_controllers().unwrap();
    assert!(weak.upgrade().is_none());
    assert_eq!(server.joint_controllers.as_ref().unwrap().streams.len(), 1);
    wait(|| !retired.matching_status().wait().unwrap().matching());
    assert!(retained.matching_status().wait().unwrap().matching());
    assert!(!server.reset_failed);
}
