// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::{
    msgs::{Twist, WorldControl, WorldReset},
    prost::{Message, Name as ProtoName},
};
use gz_sim_scene::{PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step};
use std::time::Instant;
use zenoh::Wait;
const XML: &str = include_str!("../fixtures/m79_velocity_control.sdf");
fn network() -> zenoh::Config {
    zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}"#).unwrap()
}
fn simulation(engine: PhysicsEngine) -> Simulation {
    load_world_with_sdf_step(
        XML,
        "m79-server.sdf",
        "default",
        &sdformat::NoResources,
        engine,
    )
    .unwrap()
}
fn server(engine: PhysicsEngine) -> Server {
    Server::with_zenoh_config(
        simulation(engine),
        "default",
        ServerOptions {
            partition: format!("m79-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        network(),
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn position(server: &Server, name: &str) -> [f64; 3] {
    let entity = entity_by_id(server.simulation.world(), id(&server.simulation, name)).unwrap();
    server
        .simulation
        .world()
        .get::<WorldPose>(entity)
        .unwrap()
        .translation
}
fn velocity(server: &Server) -> [f64; 3] {
    let entity = entity_by_id(
        server.simulation.world(),
        id(&server.simulation, "default::rig::body"),
    )
    .unwrap();
    server
        .simulation
        .world()
        .get::<Velocity>(entity)
        .unwrap()
        .linear
}
fn pause(server: &mut Server, paused: bool) {
    assert!(server.apply_local_control(WorldControl {
        pause: paused,
        ..Default::default()
    }));
}
fn put(
    server: &mut Server,
    publisher: &zenoh::pubsub::Publisher<'_>,
    bytes: Vec<u8>,
    attachment: Option<&str>,
) {
    let before = server.counters.velocity_control_messages_received;
    let send = publisher.put(bytes);
    if let Some(attachment) = attachment {
        send.attachment(attachment).wait().unwrap();
    } else {
        send.wait().unwrap();
    }
    let deadline = Instant::now() + Duration::from_secs(5);
    while server.counters.velocity_control_messages_received == before {
        assert!(Instant::now() < deadline);
        server.update().unwrap();
        std::thread::yield_now();
    }
}
fn twist(linear: [f64; 3], angular: [f64; 3]) -> Vec<u8> {
    Twist {
        linear: Some(vector3(linear)),
        angular: Some(vector3(angular)),
        ..Default::default()
    }
    .encode_to_vec()
}
#[test]
fn paused_pending_six_axis_command_uses_previous_target_on_first_resumed_step_and_persists() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut server = server(engine);
        pause(&mut server, true);
        let key = server.network_config.topic_key("/m79/cmd").unwrap();
        let publisher = server
            .world_io
            .session
            .declare_publisher(key)
            .wait()
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(5);
        while !publisher.matching_status().wait().unwrap().matching() {
            assert!(Instant::now() < deadline);
            std::thread::yield_now();
        }
        let before = position(&server, "default::rig::body");
        put(
            &mut server,
            &publisher,
            twist([1.0, -2.0, 3.0], [0.0; 3]),
            Some(&Twist::full_name()),
        );
        assert_eq!(position(&server, "default::rig::body"), before);
        assert_eq!(server.counters.velocity_control_commands_pending, 1);
        pause(&mut server, false);
        server.update().unwrap();
        assert!((velocity(&server)[0] - 0.25).abs() < 1e-12);
        server.update().unwrap();
        assert_eq!(velocity(&server), [1.0, -2.0, 3.0]);
        server.update().unwrap();
        assert_eq!(velocity(&server), [1.0, -2.0, 3.0]);
        pause(&mut server, true);
        put(
            &mut server,
            &publisher,
            Twist::default().encode_to_vec(),
            Some(&Twist::full_name()),
        );
        pause(&mut server, false);
        server.update().unwrap();
        assert_eq!(velocity(&server), [1.0, -2.0, 3.0]);
        server.update().unwrap();
        assert_eq!(velocity(&server), [0.0; 3]);
        server.shutdown();
    }
}
#[test]
fn invalid_ingress_preserves_target_and_loaded_reset_retires_old_callback_generation() {
    let mut server = server(PhysicsEngine::Rapier);
    pause(&mut server, true);
    let old = Arc::clone(&server.velocity_control.as_ref().unwrap().lease);
    let old_id = old.identity();
    let model = id(&server.simulation, "default::rig");
    let body = id(&server.simulation, "default::rig::body");
    let publisher = server
        .world_io
        .session
        .declare_publisher(server.network_config.topic_key("/m79/cmd").unwrap())
        .wait()
        .unwrap();
    for (bytes, attachment) in [
        (vec![255], Some(Twist::full_name())),
        (
            twist([f64::INFINITY, 0.0, 0.0], [0.0; 3]),
            Some(Twist::full_name()),
        ),
        (twist([9.0; 3], [0.0; 3]), Some("gz.msgs.Double".into())),
        (twist([9.0; 3], [0.0; 3]), None),
    ] {
        put(&mut server, &publisher, bytes, attachment.as_deref());
    }
    assert_eq!(server.counters.velocity_control_messages_rejected, 4);
    assert_eq!(server.counters.velocity_control_commands_pending, 0);
    // A retained old-generation envelope cannot target this epoch even when
    // the public IDs and numerical command are otherwise valid.
    let (stale, receiver) = mpsc::sync_channel(1);
    let live_receiver = std::mem::replace(
        &mut server.velocity_control.as_mut().unwrap().receiver,
        receiver,
    );
    stale.send((u64::MAX, Some(([7.0; 3], [0.0; 3])))).unwrap();
    server.update().unwrap();
    server.velocity_control.as_mut().unwrap().receiver = live_receiver;
    assert_eq!(server.counters.velocity_control_messages_rejected, 5);
    assert_eq!(server.counters.velocity_control_commands_pending, 0);
    put(
        &mut server,
        &publisher,
        twist([9.0; 3], [0.0; 3]),
        Some(&Twist::full_name()),
    );
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.update().unwrap();
    assert_eq!(id(&server.simulation, "default::rig"), model);
    assert_eq!(id(&server.simulation, "default::rig::body"), body);
    assert!(!old.matches(
        old_id,
        &server.network_config.topic_key("/m79/cmd").unwrap()
    ));
    let new = server.velocity_control.as_ref().unwrap();
    assert_ne!(new.lease_id, old_id);
    pause(&mut server, false);
    server.update().unwrap();
    assert!((velocity(&server)[0] - 0.25).abs() < 1e-12);
    assert_eq!(&velocity(&server)[1..], &[0.0, 0.0]);
    pause(&mut server, true);
    put(
        &mut server,
        &publisher,
        twist([8.0; 3], [0.0; 3]),
        Some(&Twist::full_name()),
    );
    let before_time_only = server.velocity_control.as_ref().unwrap().lease_id;
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            time_only: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.update().unwrap();
    assert_ne!(
        server.velocity_control.as_ref().unwrap().lease_id,
        before_time_only
    );
    pause(&mut server, false);
    server.update().unwrap();
    assert!((velocity(&server)[0] - 0.25).abs() < 1e-12);
    assert_eq!(&velocity(&server)[1..], &[0.0, 0.0]);
    server.shutdown();
}
#[test]
fn removal_retires_only_configured_owner_and_reset_restores_its_original_subscription() {
    let mut server = server(PhysicsEngine::Rapier);
    let original = id(&server.simulation, "default::rig");
    let survivor = id(&server.simulation, "default::survivor");
    let old = Arc::clone(&server.velocity_control.as_ref().unwrap().lease);
    let old_id = old.identity();
    let key = server.network_config.topic_key("/m79/cmd").unwrap();
    gz_sim_scene::remove_model(&mut server.simulation, original).unwrap();
    server.update().unwrap();
    assert!(server.velocity_control.is_none());
    assert!(!old.matches(old_id, &key));
    assert!(entity_by_id(server.simulation.world(), survivor).is_some());
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.update().unwrap();
    assert_eq!(id(&server.simulation, "default::rig"), original);
    assert!(server.velocity_control.is_some());
    assert_ne!(server.velocity_control.as_ref().unwrap().lease_id, old_id);
    server.shutdown();
    assert!(server.velocity_control.is_none());
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}
#[test]
fn every_reserved_endpoint_alias_refuses_before_network_and_metadata_drift_cannot_rebind() {
    for topic in [
        "/clock",
        "/world/default/clock",
        "/world/default/control",
        "/world/default/pose/info",
        "/world/default/remove",
        "/world/default/remove/blocking",
    ] {
        let options = ServerOptions {
            topic_remaps: std::collections::HashMap::from([("/m79/cmd".into(), topic.into())]),
            ..Default::default()
        };
        assert!(matches!(
            before_network(&simulation(PhysicsEngine::Rapier), "default", &options),
            Err(Error::VelocityControlIdentity)
        ));
        // A real held local listener makes this otherwise valid network config
        // fail if admission incorrectly reaches Session creation.
        let occupied = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let mut bad = network();
        bad.insert_json5(
            "listen/endpoints",
            &format!("[\"tcp/{}\"]", occupied.local_addr().unwrap()),
        )
        .unwrap();
        assert!(matches!(
            Server::with_zenoh_config(
                simulation(PhysicsEngine::Rapier),
                "default",
                options,
                bad,
                NonZeroUsize::new(1).unwrap()
            ),
            Err(Error::VelocityControlIdentity)
        ));
    }
    let camera = "<sensor name='camera' type='camera'><topic>/m79/image</topic><camera><horizontal_fov>1</horizontal_fov><image><width>64</width><height>64</height><format>R8G8B8</format></image><clip><near>0.1</near><far>20</far></clip></camera></sensor>";
    for (camera, topics) in [
        (camera.to_owned(), vec!["/m79/image", "/m79/camera_info"]),
        (
            camera.replace("<topic>/m79/image</topic>", ""),
            vec![
                "/world/default/model/rig/link/body/sensor/camera/image",
                "/world/default/model/rig/link/body/sensor/camera/camera_info",
            ],
        ),
        (
            camera.replace(
                "<camera>",
                "<camera><camera_info_topic>/custom/info</camera_info_topic>",
            ),
            vec!["/m79/image", "/custom/info"],
        ),
    ] {
        let xml = XML.replacen("</link>", &format!("{camera}</link>"), 1);
        let sim = load_world_with_sdf_step(
            &xml,
            "camera-collision.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        for topic in topics {
            for namespace in ["", "fleet"] {
                let options = ServerOptions {
                    namespace: namespace.into(),
                    partition: "actual-m79-partition".into(),
                    topic_remaps: std::collections::HashMap::from([(
                        "/m79/cmd".into(),
                        topic.into(),
                    )]),
                    ..Default::default()
                };
                assert!(matches!(
                    before_network(&sim, "default", &options),
                    Err(Error::VelocityControlIdentity)
                ));
                let options = ServerOptions {
                    topic_remaps: std::collections::HashMap::from([
                        ("/m79/cmd".into(), "/shared".into()),
                        (topic.into(), "/shared".into()),
                    ]),
                    ..options
                };
                assert!(matches!(
                    before_network(&sim, "default", &options),
                    Err(Error::VelocityControlIdentity)
                ));
            }
        }
    }
    let mut server = server(PhysicsEngine::Rapier);
    let entity = server.velocity_control.as_ref().unwrap().entity;
    let mut changed = server
        .simulation
        .world()
        .get::<VelocityControlConfiguration>(entity)
        .unwrap()
        .clone();
    changed.topic = "/changed".into();
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(changed);
    assert!(matches!(
        server.update(),
        Err(Error::VelocityControlIdentity)
    ));
    assert!(server.velocity_control.is_none());
    server.shutdown();
}
#[test]
fn absent_system_has_no_velocity_owner_or_reserved_input() {
    let xml=XML.replace("<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'><topic>/m79/cmd</topic><initial_linear>0.25 0 0</initial_linear><initial_angular>0 0 0</initial_angular></plugin>","");
    let sim = load_world_with_sdf_step(
        &xml,
        "absent.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    assert!(original(&sim).unwrap().is_none());
    assert!(
        reserved_keys(&sim, &ServerOptions::default())
            .unwrap()
            .is_empty()
    );
    let mut server = Server::with_zenoh_config(
        sim,
        "default",
        ServerOptions {
            partition: format!("m79-absent-{}", uuid::Uuid::new_v4()),
            ..ServerOptions::default()
        },
        network(),
        NonZeroUsize::new(1).unwrap(),
    )
    .unwrap();
    assert!(server.velocity_control.is_none());
    server.update().unwrap();
    assert_eq!(server.counters.velocity_control_messages_received, 0);
    server.shutdown();
}
