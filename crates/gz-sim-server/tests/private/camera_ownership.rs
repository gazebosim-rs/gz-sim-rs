// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::net::TcpListener;

fn simulation(configured: bool, camera: bool) -> Simulation {
    let source = if camera {
        include_str!("../../../gz-sim-scene/tests/fixtures/camera-binding.sdf").to_owned()
    } else {
        "<sdf version='1.12'><world name='default'/></sdf>".into()
    };
    let source = if configured {
        source.replace("name=\"default\">", "name=\"default\"><plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>").replace("name='default'/>", "name='default'><plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/></world>")
    } else {
        source
    };
    gz_sim_scene::load_world_with_sdf_step(
        &source,
        "camera-ownership.sdf",
        "default",
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::Tpe,
    )
    .unwrap()
}
fn options() -> ServerOptions {
    ServerOptions {
        partition: format!("camera-boundary-{}", uuid::Uuid::new_v4()),
        namespace: "ns".into(),
        ..Default::default()
    }
}
fn config(endpoint: &str) -> zenoh::Config {
    let mut cfg = zenoh::Config::default();
    for (key, value) in [
        ("listen/endpoints", format!("[\"{endpoint}\"]")),
        ("scouting/multicast/enabled", "false".into()),
        ("scouting/gossip/enabled", "false".into()),
    ] {
        cfg.insert_json5(key, &value).unwrap();
    }
    cfg
}
fn camera_id(sim: &Simulation) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.contains::<gz_sim_scene::CameraSensor>())
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

#[test]
fn configured_camera_requires_an_external_owner_before_network_advertising() {
    let occupied = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = format!("tcp/{}", occupied.local_addr().unwrap());
    let result = Server::with_zenoh_config(
        simulation(true, true),
        "default",
        options(),
        config(&address),
        NonZeroUsize::new(1).unwrap(),
    );
    assert!(matches!(result, Err(Error::CameraOutputRequired)));
    assert!(matches!(
        Server::new(
            simulation(true, true),
            "default",
            options(),
            NonZeroUsize::new(1).unwrap()
        ),
        Err(Error::CameraOutputRequired)
    ));
}

#[test]
fn empty_configured_and_manual_metadata_worlds_preserve_normal_server() {
    for (configured, camera) in [(true, false), (false, true), (false, false)] {
        let options = options();
        let partition = options.partition.clone();
        let mut server = Server::with_zenoh_config(
            simulation(configured, camera),
            "default",
            options,
            config("tcp/127.0.0.1:0"),
            NonZeroUsize::new(1).unwrap(),
        )
        .unwrap();
        assert!(!server.is_shutdown());
        assert_eq!(server.options().partition, partition);
        assert_eq!(server.options().namespace, "ns");
        assert_eq!(
            server.options().topic_key("relative").unwrap(),
            format!("@/{partition}@/ns/relative")
        );
        server.shutdown();
        assert!(server.is_shutdown());
        server.shutdown();
        assert!(server.is_shutdown());
    }
}

#[test]
fn external_camera_boundary_validates_the_actual_one_camera_identity() {
    let sim = simulation(true, true);
    let id = camera_id(&sim);
    let mut server = Server::with_external_camera_output(
        sim,
        "default",
        options(),
        config("tcp/127.0.0.1:0"),
        NonZeroUsize::new(1).unwrap(),
        id,
    )
    .unwrap();
    // This tests the explicit trust boundary; actual external IO is verified by
    // the camera application's constructor/lifecycle tests.
    server.shutdown();
    for (configured, camera) in [(false, true), (true, false)] {
        assert!(matches!(
            Server::with_external_camera_output(
                simulation(configured, camera),
                "default",
                options(),
                config("tcp/127.0.0.1:0"),
                NonZeroUsize::new(1).unwrap(),
                id
            ),
            Err(Error::CameraIdentity)
        ));
    }
    assert!(matches!(
        Server::with_external_camera_output(
            simulation(true, true),
            "default",
            options(),
            config("tcp/127.0.0.1:0"),
            NonZeroUsize::new(1).unwrap(),
            u64::MAX
        ),
        Err(Error::CameraIdentity)
    ));
}

#[test]
fn preparation_observation_tracks_only_unprepared_reset_not_terminal_admission() {
    let mut server = Server::with_zenoh_config(
        simulation(false, false),
        "default",
        options(),
        config("tcp/127.0.0.1:0"),
        NonZeroUsize::new(1).unwrap(),
    )
    .unwrap();
    assert!(!server.io_preparation_required());
    assert!(server.apply_control(WorldControl {
        pause: true,
        reset: Some(gz_sim_proto::msgs::WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(server.io_preparation_required());
    assert_eq!(server.simulation().epoch(), 0);
    server.prepare_io().unwrap();
    assert!(!server.io_preparation_required());
    assert_eq!(server.simulation().epoch(), 1);
    assert_eq!(server.tick().unwrap().epoch, 1);
    server.shutdown();
    assert!(server.is_shutdown());
    assert!(!server.io_preparation_required());
    assert!(matches!(server.prepare_io(), Err(Error::WorldClosed)));
}
