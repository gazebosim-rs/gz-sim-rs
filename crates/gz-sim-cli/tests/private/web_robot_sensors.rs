// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::prost::{Message as _, Name as _};
use gz_sim_scene::{ImuSample, LidarPublication, LidarSample};
use std::collections::BTreeSet;
use zenoh::Wait;

fn entity(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<gz_sim_scene::Name>()
                .is_some_and(|value| value.0 == name)
        })
        .unwrap()
        .id()
}

fn fixture() -> (Fixture, zenoh::Session, String) {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/m33_robot_sensors.sdf")
        .canonicalize()
        .unwrap();
    let uri = url::Url::from_file_path(path).unwrap().to_string();
    let xml = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../examples/m33_robot_sensors.sdf"
    ));
    let sdf = sdformat::load_str(xml, &uri, &sdformat::NoResources).unwrap();
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        xml,
        &uri,
        "default",
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::Rapier,
    )
    .unwrap();
    sim.set_paused(true);
    let observer = zenoh::open(config(None)).wait().unwrap();
    let endpoints: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|value| value.to_string())
        .collect();
    assert_eq!(endpoints.len(), 1);
    let endpoint = endpoints[0]
        .strip_prefix("tcp/")
        .unwrap()
        .parse::<SocketAddr>()
        .unwrap();
    assert_eq!(endpoint.ip(), Ipv4Addr::LOCALHOST);
    assert_ne!(endpoint.port(), 0);
    let options = gz_sim_server::ServerOptions {
        partition: format!(
            "m33-web-{}-{}",
            std::process::id(),
            ID.fetch_add(1, Ordering::Relaxed)
        ),
        ..Default::default()
    };
    let command_key = options.topic_key("/model/robot/cmd_vel").unwrap();
    let mut server = Server::with_zenoh_config(
        sim,
        "default",
        options,
        config(Some(&endpoints[0])),
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    // This socket control uses explicit local acquisition. The Server/C++
    // coexistence controls separately exercise canonical subscriber demand.
    server.set_interest_policy(gz_sim_server::InterestPolicy::Always);
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let address = listener.local_addr().unwrap();
    let bridge =
        Bridge::with_options(server.simulation(), "default", listener, Some(&sdf)).unwrap();
    (
        Fixture {
            server,
            bridge,
            address,
            sdf,
        },
        observer,
        command_key,
    )
}

fn config(endpoint: Option<&str>) -> zenoh::Config {
    let mut config = zenoh::Config::default();
    let connect = endpoint.map_or_else(|| "[]".to_string(), |value| format!("[\"{value}\"]"));
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

fn command(fixture: &mut Fixture, observer: &zenoh::Session, key: &str, linear: f64, angular: f64) {
    use gz_sim_proto::msgs::{Twist, Vector3d};
    let publisher = observer.declare_publisher(key).wait().unwrap();
    wait("M33 command subscriber did not match", || {
        publisher.matching_status().wait().unwrap().matching()
    });
    assert!(fixture.server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    let before = seeded_values(fixture.server.simulation());
    let elapsed = fixture.server.simulation().elapsed();
    let count = fixture.server.counters().diff_drive_commands_applied;
    publisher
        .put(
            Twist {
                linear: Some(Vector3d {
                    x: linear,
                    y: 0.0,
                    z: 0.0,
                    ..Default::default()
                }),
                angular: Some(Vector3d {
                    x: 0.0,
                    y: 0.0,
                    z: angular,
                    ..Default::default()
                }),
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .attachment(Twist::full_name())
        .wait()
        .unwrap();
    wait("M33 canonical Twist was not applied", || {
        fixture.server.tick().unwrap();
        fixture.server.counters().diff_drive_commands_applied == count + 1
    });
    assert_eq!(fixture.server.simulation().elapsed(), elapsed);
    assert_eq!(seeded_values(fixture.server.simulation()), before);
    assert!(fixture.server.apply_local_control(WorldControl::default()));
}

fn ready(token: u64) -> Message {
    Message::Text(
        json!({"version":1,"kind":"publication_ready","token":token})
            .to_string()
            .into(),
    )
}

fn barrier(socket: &mut WebSocket<TcpStream>, tag: u8) {
    socket.send(Message::Ping(vec![tag].into())).unwrap();
    assert_eq!(socket.read().unwrap(), Message::Pong(vec![tag].into()));
}

fn flight(socket: &mut WebSocket<TcpStream>, token: u64) -> (Value, Value, Value) {
    let scene = read(socket);
    let catalog = read(socket);
    let scan = read(socket);
    assert_eq!(scene["kind"], "scene");
    assert_eq!(catalog["kind"], "lifecycle_catalog");
    assert_eq!(scan["kind"], "scan");
    assert_eq!(catalog["scene_sequence"], scene["sequence"]);
    for frame in [&catalog, &scan] {
        assert_eq!(frame["world_id"], scene["world_id"]);
        assert_eq!(frame["epoch"], scene["epoch"]);
    }
    assert_eq!(
        read(socket),
        json!({"version":1,"kind":"publication_credit","token":token})
    );
    (scene, catalog, scan)
}

fn assert_catalog(scene: &Value, catalog: &Value) {
    let visuals = scene["visuals"].as_array().unwrap();
    assert_eq!(visuals.len(), 7);
    assert_eq!(
        visuals
            .iter()
            .map(|v| v["name"].as_str().unwrap())
            .collect::<BTreeSet<_>>(),
        BTreeSet::from([
            "default::ground::body::ground",
            "default::obj_landmark::body::obj_mesh",
            "default::robot::base_link::obj_cargo",
            "default::robot::base_link::chassis",
            "default::robot::left_wheel::wheel",
            "default::robot::right_wheel::wheel",
            "default::m33_target::body::box",
        ])
    );
    assert_eq!(
        visuals
            .iter()
            .filter(|v| v["shape"]["kind"] == "mesh")
            .count(),
        2
    );
    let models = catalog["models"].as_array().unwrap();
    assert_eq!(models.len(), 4);
    assert_eq!(
        models
            .iter()
            .map(|m| m["name"].as_str().unwrap())
            .collect::<BTreeSet<_>>(),
        BTreeSet::from([
            "default::ground",
            "default::robot",
            "default::obj_landmark",
            "default::m33_target"
        ])
    );
    let owned: Vec<_> = models
        .iter()
        .flat_map(|m| m["visual_ids"].as_array().unwrap())
        .map(|id| id.as_u64().unwrap())
        .collect();
    assert_eq!(owned.len(), 7);
    assert_eq!(
        owned.into_iter().collect::<BTreeSet<_>>(),
        visuals.iter().map(|v| v["id"].as_u64().unwrap()).collect()
    );
    let robot = models
        .iter()
        .find(|m| m["name"] == "default::robot")
        .unwrap();
    assert_eq!(robot["visual_ids"].as_array().unwrap().len(), 4);
    assert_eq!(robot["can_remove"], false);
    assert_eq!(robot["can_set_state"], false);
    assert!(
        catalog["templates"]
            .as_array()
            .unwrap()
            .iter()
            .all(|t| t["label"] != "robot")
    );
}

fn publish(fixture: &mut Fixture) {
    fixture.bridge.published = Instant::now() - Duration::from_millis(35);
    fixture
        .bridge
        .publish(fixture.server.simulation(), "default", Vec::new())
        .unwrap();
}

fn assert_scan(sim: &Simulation, frame: &Value) {
    let sensor = entity(sim, "default::robot::base_link::lidar");
    let sample = sim.world().get::<LidarSample>(sensor).unwrap();
    let publication = sim.world().get::<LidarPublication>(sensor).unwrap();
    let scan = &frame["scan"];
    assert_eq!(frame["epoch"], sim.epoch());
    assert_eq!(
        frame["sim_time_ns"],
        u64::try_from(sim.elapsed().as_nanos()).unwrap()
    );
    assert_eq!(
        scan["sensor_id"],
        sim.world().get::<EntityId>(sensor).unwrap().get()
    );
    assert_eq!(
        scan["acquired_ns"],
        u64::try_from(sample.time.as_nanos()).unwrap()
    );
    assert_eq!(
        scan["sensor_position"],
        json!(sample.world_pose.translation)
    );
    assert_eq!(
        scan["sensor_orientation"],
        json!(sample.world_pose.rotation)
    );
    assert_eq!(scan["angle_min"], publication.config.angle_min);
    assert_eq!(scan["angle_max"], publication.config.angle_max);
    assert_eq!(scan["angle_step"], publication.angle_step);
    assert_eq!(scan["range_min"].as_f64(), Some(0.0));
    assert_eq!(scan["range_max"].as_f64(), Some(10.0));
    assert_eq!(sample.ranges.len(), 3);
    assert_eq!(scan["ranges"].as_array().unwrap().len(), 3);
    for (value, range) in scan["ranges"]
        .as_array()
        .unwrap()
        .iter()
        .zip(&sample.ranges)
    {
        if *range == f64::INFINITY {
            assert!(value.is_null());
        } else {
            assert!(range.is_finite() && (0.0..=10.0).contains(range));
            assert_eq!(value.as_f64().unwrap(), *range);
        }
    }
}

fn seeded_values(sim: &Simulation) -> impl PartialEq + std::fmt::Debug + use<> {
    let imu = sim
        .world()
        .get::<ImuSample>(entity(sim, "default::robot::base_link::imu"))
        .unwrap();
    let lidar = sim
        .world()
        .get::<LidarSample>(entity(sim, "default::robot::base_link::lidar"))
        .unwrap();
    let reading = imu.reading.as_ref().unwrap();
    assert_eq!(imu.epoch, sim.epoch());
    assert_eq!(lidar.epoch, sim.epoch());
    (
        reading.orientation,
        reading.specific_force,
        reading.angular_velocity,
        reading.angular_velocity_covariance,
        reading.linear_acceleration_covariance,
        lidar.ranges.clone(),
    )
}

#[test]
fn combined_robot_scan_and_imu_keep_advancing_under_credit_then_reset_reconnect_replays_seed() {
    let (mut fixture, observer, command_key) = fixture();
    let counts = fixture.server.simulation().scheduled_system_counts();
    let mut socket = fixture.connect("/?scan=1&lifecycle=1&lifecycle_credit=1");
    let (initial, catalog, initial_scan) = flight(&mut socket, 1);
    assert_catalog(&initial, &catalog);
    assert!(initial_scan["scan"].is_null());
    assert!(fixture.bridge.scan_interested());
    assert!(fixture.server.apply_local_control(WorldControl::default()));
    fixture.server.tick().unwrap();
    let first = seeded_values(fixture.server.simulation());
    let first_scan: Value =
        serde_json::from_str(&scan::encode(fixture.server.simulation()).unwrap()).unwrap();
    assert_scan(fixture.server.simulation(), &first_scan);
    assert!(first_scan["scan"]["ranges"][1].as_f64().is_some());
    for _ in 0..10 {
        fixture.server.tick().unwrap();
    }
    let sim = fixture.server.simulation();
    let imu = sim
        .world()
        .get::<ImuSample>(entity(sim, "default::robot::base_link::imu"))
        .unwrap();
    let lidar = sim
        .world()
        .get::<LidarSample>(entity(sim, "default::robot::base_link::lidar"))
        .unwrap();
    assert!(imu.time > lidar.time);
    assert_eq!(lidar.time, Duration::from_millis(1));
    let held: Value = serde_json::from_str(&scan::encode(sim).unwrap()).unwrap();
    assert_eq!(held["scan"], first_scan["scan"]);
    let before = *sim
        .world()
        .get::<WorldPose>(entity(sim, "default::robot::base_link"))
        .unwrap();
    command(&mut fixture, &observer, &command_key, 0.4, 0.0);
    for _ in 0..15 {
        for _ in 0..20 {
            fixture.server.tick().unwrap();
        }
        publish(&mut fixture);
        barrier(&mut socket, 101);
    }
    let forward = *fixture
        .server
        .simulation()
        .world()
        .get::<WorldPose>(entity(
            fixture.server.simulation(),
            "default::robot::base_link",
        ))
        .unwrap();
    assert_ne!(forward.translation, before.translation);
    command(&mut fixture, &observer, &command_key, 0.0, 0.8);
    for _ in 0..15 {
        for _ in 0..20 {
            fixture.server.tick().unwrap();
        }
        publish(&mut fixture);
        barrier(&mut socket, 102);
    }
    let sim = fixture.server.simulation();
    let turned = sim
        .world()
        .get::<WorldPose>(entity(sim, "default::robot::base_link"))
        .unwrap();
    assert_ne!(turned.rotation, forward.rotation);
    assert_eq!(sim.elapsed(), Duration::from_millis(611));
    assert_eq!(sim.scheduled_system_counts(), counts);
    let imu = sim
        .world()
        .get::<ImuSample>(entity(sim, "default::robot::base_link::imu"))
        .unwrap();
    let lidar = sim
        .world()
        .get::<LidarSample>(entity(sim, "default::robot::base_link::lidar"))
        .unwrap();
    assert!(imu.time > Duration::from_millis(500) && lidar.time > Duration::from_millis(500));
    assert!(imu.reading.is_ok());
    assert_ne!(
        json!(lidar.world_pose.translation),
        first_scan["scan"]["sensor_position"]
    );
    let latest: Value = serde_json::from_str(&fixture.bridge.latest.lock().unwrap().1).unwrap();
    socket.send(ready(1)).unwrap();
    let (current, current_catalog, current_scan) = flight(&mut socket, 2);
    assert_eq!(current, latest);
    assert_ne!(current["visuals"], initial["visuals"]);
    assert_eq!(current_catalog["models"], catalog["models"]);
    assert_scan(fixture.server.simulation(), &current_scan);
    barrier(&mut socket, 103);

    socket
        .send(Message::Text(
            json!({"version":1,"kind":"control","request_id":1,"action":"reset"})
                .to_string()
                .into(),
        ))
        .unwrap();
    let mut controls = Vec::new();
    wait("M33 reset did not reach the shared queue", || {
        controls.extend(fixture.bridge.controls(&mut fixture.server));
        !controls.is_empty()
    });
    fixture.server.prepare_io().unwrap();
    fixture.server.tick().unwrap();
    fixture
        .bridge
        .apply_lifecycle(&mut fixture.server, "default")
        .unwrap();
    fixture
        .bridge
        .publish(fixture.server.simulation(), "default", controls)
        .unwrap();
    let ack = read(&mut socket);
    assert_eq!(ack["kind"], "ack");
    assert_eq!(ack["request_id"], 1);
    assert_eq!(ack["ok"], true);
    assert_eq!(ack["epoch"], 1);
    assert_eq!(ack["seconds"], 0);
    assert_eq!(ack["nanoseconds"], 0);
    assert_eq!(ack["paused"], true);
    barrier(&mut socket, 104);
    socket.send(ready(2)).unwrap();
    let (restored, restored_catalog, restored_scan) = flight(&mut socket, 3);
    assert_eq!(restored["visuals"], initial["visuals"]);
    assert_eq!(restored_catalog["models"], catalog["models"]);
    assert!(restored_scan["scan"].is_null());
    socket.close(None).unwrap();
    drop(socket);
    wait("M33 socket and scan lease did not retire", || {
        fixture.bridge.connected.load(Ordering::Acquire) == 0 && !fixture.bridge.scan_interested()
    });
    let mut reconnected = fixture.connect("/?scan=1&lifecycle=1&lifecycle_credit=1");
    let (fresh, fresh_catalog, fresh_scan) = flight(&mut reconnected, 1);
    assert_eq!(fresh["epoch"], 1);
    assert_eq!(fresh["seconds"], 0);
    assert_eq!(fresh["nanoseconds"], 0);
    assert_eq!(fresh["paused"], true);
    assert_eq!(fresh["visuals"], initial["visuals"]);
    assert_eq!(fresh_catalog["models"], catalog["models"]);
    assert!(fresh_scan["scan"].is_null());
    assert_eq!(
        fixture.server.simulation().scheduled_system_counts(),
        counts
    );
    assert!(fixture.server.apply_local_control(WorldControl::default()));
    fixture.server.tick().unwrap();
    assert_eq!(seeded_values(fixture.server.simulation()), first);
    publish(&mut fixture);
    barrier(&mut reconnected, 105);
    reconnected.send(ready(1)).unwrap();
    let (_, _, replay_scan) = flight(&mut reconnected, 2);
    assert_scan(fixture.server.simulation(), &replay_scan);
    assert_eq!(replay_scan["scan"], first_scan["scan"]);
    barrier(&mut reconnected, 106);
}
