// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn ready(token: u64) -> Message {
    Message::Text(
        json!({"version":1,"kind":"publication_ready","token":token})
            .to_string()
            .into(),
    )
}

fn flight(socket: &mut WebSocket<TcpStream>, expected_token: u64) -> (Value, Value) {
    let scene = read(socket);
    let catalog = read(socket);
    let scan = read(socket);
    let credit = read(socket);
    assert_eq!(scene["kind"], "scene");
    assert_eq!(catalog["kind"], "lifecycle_catalog");
    assert_eq!(catalog["scene_sequence"], scene["sequence"]);
    assert_eq!(catalog["world_id"], scene["world_id"]);
    assert_eq!(catalog["epoch"], scene["epoch"]);
    assert_eq!(scan["kind"], "scan");
    assert_eq!(scan["epoch"], scene["epoch"]);
    assert_eq!(
        credit,
        json!({"version":1,"kind":"publication_credit","token":expected_token})
    );
    (scene, catalog)
}

fn barrier(socket: &mut WebSocket<TcpStream>, tag: u8) {
    socket.send(Message::Ping(vec![tag].into())).unwrap();
    assert_eq!(socket.read().unwrap(), Message::Pong(vec![tag].into()));
}

#[test]
fn normal_publications_wait_for_credit_while_paused_and_advancing_without_stalling_physics() {
    for advancing in [false, true] {
        let mut fixture = Fixture::new(true);
        let mut socket = fixture.connect("/?scan=1&lifecycle=1&lifecycle_credit=1");
        let (initial, _) = flight(&mut socket, 1);
        if advancing {
            assert!(fixture.server.apply_local_control(WorldControl::default()));
        }
        for _ in 0..90 {
            fixture.server.tick().unwrap();
            fixture.bridge.published = Instant::now() - Duration::from_millis(35);
            fixture
                .bridge
                .publish(fixture.server.simulation(), "default", Vec::new())
                .unwrap();
            // A real round trip makes the worker observe every changed Latest;
            // publishing faster than the socket worker cannot hide a flood.
            barrier(&mut socket, 80);
        }
        // The application has not granted another publication across 90
        // updates, while the worker remains responsive to transport traffic.
        barrier(&mut socket, 81);
        barrier(&mut socket, 82);
        let latest: Value = serde_json::from_str(&fixture.bridge.latest.lock().unwrap().1).unwrap();
        assert!(latest["sequence"].as_u64().unwrap() > initial["sequence"].as_u64().unwrap());
        assert_eq!(latest["paused"], !advancing);
        if advancing {
            assert!(latest["nanoseconds"].as_u64().unwrap() > 0);
        } else {
            assert_eq!(latest["nanoseconds"], 0);
        }
        socket.send(ready(1)).unwrap();
        let (current, _) = flight(&mut socket, 2);
        assert_eq!(current, latest);
        barrier(&mut socket, 83);
        socket.send(ready(2)).unwrap();
        barrier(&mut socket, 84);
    }
}

#[test]
fn exact_committed_reply_bypasses_old_credit_and_ready_is_valid_while_command_is_pending() {
    let mut fixture = Fixture::new(true);
    let mut socket = fixture.connect("/?scan=1&lifecycle=1&lifecycle_credit=1");
    let (initial, catalog) = flight(&mut socket, 1);
    let initial_scene = Arc::clone(&fixture.bridge.latest.lock().unwrap().1);
    assert_eq!(
        serde_json::from_str::<Value>(&initial_scene).unwrap(),
        initial
    );
    let model = catalog["models"]
        .as_array()
        .unwrap()
        .iter()
        .find(|model| model["name"] == "default::box")
        .unwrap()["model_id"]
        .as_u64()
        .unwrap();
    let state = model_state(fixture.server.simulation(), model).unwrap();
    socket
        .send(Message::Text(
            target(1, 1, 0, "get_state", model).to_string().into(),
        ))
        .unwrap();
    let Queued::Lifecycle(pending) = fixture
        .bridge
        .commands
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
    else {
        panic!("expected lifecycle request");
    };
    // Ready must be accepted with the reply receiver still pending. It must
    // neither acknowledge this edit nor occupy the shared world-command queue.
    socket.send(ready(1)).unwrap();
    barrier(&mut socket, 85);
    assert!(fixture.bridge.commands.try_recv().is_err());
    let ack = json!({"version":1,"kind":"lifecycle_ack","request_id":1,"action":"get_state","world_id":1,"epoch":0,"scene_sequence":initial["sequence"],"seconds":0,"nanoseconds":0,"paused":true,"ok":true,"error":null,"model_id":model,"state":{"translation":state.translation,"rotation":state.rotation,"linear_velocity":state.linear_velocity,"angular_velocity":state.angular_velocity}});
    // This transport-only reply uses an admitted request and a real captured
    // frame/state; it does not claim another CPU command or native tick ran.
    assert!(pending.reply.send(lifecycle::Committed {
        scene: initial_scene,
        scan: Some(Arc::new(json!({"version":1,"kind":"scan","world_id":1,"epoch":0,"sim_time_ns":0,"scan":null}).to_string())),
        catalog: Arc::new(catalog.to_string()),
        ack: ack.to_string(),
    }).is_ok());
    assert_eq!(read(&mut socket), initial);
    assert_eq!(read(&mut socket)["kind"], "scan");
    assert_eq!(read(&mut socket), catalog);
    assert_eq!(read(&mut socket), ack);
    barrier(&mut socket, 86);
    // Ready did not consume world request ID 2; a subsequent request keeps the
    // same normal admission and receiver ownership as a non-credit peer.
    socket
        .send(Message::Text(
            target(2, 1, 0, "get_state", model).to_string().into(),
        ))
        .unwrap();
    let Queued::Lifecycle(next) = fixture
        .bridge
        .commands
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
    else {
        panic!("expected next lifecycle request");
    };
    assert_eq!(next.request.request_id, 2);
}

#[test]
fn credit_query_and_ready_refusals_do_not_admit_a_default_mutation_path() {
    let mut fixture = Fixture::new(true);
    for path in [
        "/?lifecycle_credit=1",
        "/?lifecycle=1&lifecycle_credit=0",
        "/?lifecycle=1&lifecycle_credit=1&lifecycle_credit=1",
        "/?lifecycle=1&lifecycle_credit",
    ] {
        assert!(client(fixture.address, path).is_err(), "{path}");
    }
    let unavailable = Fixture::new(false);
    assert!(client(unavailable.address, "/?lifecycle=1&lifecycle_credit=1").is_err());
    for (path, message) in [
        ("/?lifecycle=1", ready(1)),
        ("/?scan=1&lifecycle=1&lifecycle_credit=1", ready(2)),
    ] {
        let mut socket = fixture.connect(path);
        if path.contains("lifecycle_credit") {
            flight(&mut socket, 1);
        } else {
            fixture.bootstrap(&mut socket);
        }
        socket.send(message).unwrap();
        wait("invalid credit did not retire the peer", || {
            fixture.bridge.connected.load(Ordering::Acquire) == 0
        });
        assert!(fixture.bridge.commands.try_recv().is_err());
        drop(socket);
    }
}

#[test]
fn committed_frame_is_delivered_once_without_consuming_older_normal_credit() {
    let mut fixture = Fixture::new(true);
    let mut socket = fixture.connect("/?scan=1&lifecycle=1&lifecycle_credit=1");
    let (initial, catalog) = flight(&mut socket, 1);
    let model = catalog["models"]
        .as_array()
        .unwrap()
        .iter()
        .find(|model| model["name"] == "default::box")
        .unwrap()["model_id"]
        .as_u64()
        .unwrap();
    let state = model_state(fixture.server.simulation(), model).unwrap();
    socket
        .send(Message::Text(
            target(1, 1, 0, "get_state", model).to_string().into(),
        ))
        .unwrap();
    let Queued::Lifecycle(pending) = fixture
        .bridge
        .commands
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
    else {
        panic!("expected lifecycle request");
    };
    assert!(fixture.server.apply_local_control(WorldControl::default()));
    fixture.server.tick().unwrap();
    fixture.bridge.published = Instant::now() - Duration::from_millis(35);
    fixture
        .bridge
        .publish(fixture.server.simulation(), "default", Vec::new())
        .unwrap();
    let latest: Value = serde_json::from_str(&fixture.bridge.latest.lock().unwrap().1).unwrap();
    barrier(&mut socket, 87);
    let ack = json!({"version":1,"kind":"lifecycle_ack","request_id":1,"action":"get_state","world_id":1,"epoch":0,"scene_sequence":initial["sequence"],"seconds":0,"nanoseconds":0,"paused":true,"ok":true,"error":null,"model_id":model,"state":{"translation":state.translation,"rotation":state.rotation,"linear_velocity":state.linear_velocity,"angular_velocity":state.angular_velocity}});
    // The fabricated reply is only the delayed transport owner, matching the
    // existing committed-frame regression. Native evolution above is real.
    assert!(pending.reply.send(lifecycle::Committed {
        scene: Arc::new(initial.to_string()),
        scan: Some(Arc::new(json!({"version":1,"kind":"scan","world_id":1,"epoch":0,"sim_time_ns":0,"scan":null}).to_string())),
        catalog: Arc::new(catalog.to_string()),
        ack: ack.to_string(),
    }).is_ok());
    assert_eq!(read(&mut socket), initial);
    assert_eq!(read(&mut socket)["kind"], "scan");
    assert_eq!(read(&mut socket), catalog);
    assert_eq!(read(&mut socket), ack);
    // Neither ACK replay nor a newer normal publication is allowed before Ready1.
    barrier(&mut socket, 88);
    socket.send(ready(1)).unwrap();
    assert_eq!(flight(&mut socket, 2).0, latest);
    barrier(&mut socket, 89);
}

#[test]
fn same_socket_reset_ack_bypasses_credit_and_old_ready_releases_restored_scene() {
    let mut fixture = Fixture::new(true);
    let mut socket = fixture.connect("/?scan=1&lifecycle=1&lifecycle_credit=1");
    let (initial, _) = flight(&mut socket, 1);
    socket
        .send(Message::Text(
            json!({"version":1,"kind":"control","request_id":1,"action":"reset"})
                .to_string()
                .into(),
        ))
        .unwrap();
    let mut controls = Vec::new();
    wait("reset did not reach the shared queue", || {
        controls.extend(fixture.bridge.controls(&mut fixture.server));
        !controls.is_empty()
    });
    fixture.server.prepare_io().unwrap();
    fixture.server.tick().unwrap();
    fixture
        .bridge
        .apply_lifecycle(&mut fixture.server, "default")
        .unwrap();
    fixture.bridge.published = Instant::now() - Duration::from_millis(35);
    fixture
        .bridge
        .publish(fixture.server.simulation(), "default", controls)
        .unwrap();
    let ack = read(&mut socket);
    assert_eq!(ack["kind"], "ack");
    assert_eq!(ack["request_id"], 1);
    assert_eq!(ack["ok"], true);
    assert_eq!(ack["paused"], true);
    assert_eq!(ack["epoch"], 1);
    assert_eq!(ack["seconds"], 0);
    assert_eq!(ack["nanoseconds"], 0);
    barrier(&mut socket, 90);
    socket.send(ready(1)).unwrap();
    let (restored, _) = flight(&mut socket, 2);
    assert_eq!(restored["epoch"], 1);
    assert_eq!(restored["visuals"], initial["visuals"]);
    barrier(&mut socket, 91);
    socket.send(ready(2)).unwrap();
    barrier(&mut socket, 92);
}

#[test]
fn a_real_64_event_consumer_queue_survives_delayed_polling_with_native_world_updates() {
    for advancing in [false, true] {
        let mut fixture = Fixture::new(true);
        let socket = fixture.connect("/?scan=1&lifecycle=1&lifecycle_credit=1");
        let (events, inbox) = mpsc::sync_channel(64);
        // This socket reader has the same finite event budget as the browser.
        // It keeps receiving while the application deliberately does not poll.
        let reader = thread::spawn(move || {
            let mut socket = socket;
            socket
                .get_ref()
                .set_read_timeout(Some(Duration::from_millis(2)))
                .unwrap();
            let deadline = Instant::now() + Duration::from_millis(1250);
            while Instant::now() < deadline {
                match socket.read() {
                    Ok(Message::Text(text)) => {
                        let value: Value = serde_json::from_str(&text).unwrap();
                        events
                            .try_send(value)
                            .expect("publication flooded the real 64-event queue");
                    }
                    Ok(Message::Ping(_) | Message::Pong(_)) => {}
                    Err(tungstenite::Error::Io(error))
                        if matches!(
                            error.kind(),
                            io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                        ) => {}
                    other => panic!(
                        "credited socket failed during delayed application polling: {other:?}"
                    ),
                }
            }
            socket
        });
        if advancing {
            assert!(fixture.server.apply_local_control(WorldControl::default()));
        }
        for _ in 0..35 {
            fixture.server.tick().unwrap();
            fixture.bridge.published = Instant::now() - Duration::from_millis(35);
            fixture
                .bridge
                .publish(fixture.server.simulation(), "default", Vec::new())
                .unwrap();
            thread::sleep(Duration::from_millis(34));
        }
        let mut socket = reader.join().unwrap();
        socket
            .get_ref()
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let buffered: Vec<_> = inbox.try_iter().collect();
        assert_eq!(buffered.len(), 4);
        assert_eq!(buffered[0]["kind"], "scene");
        assert_eq!(buffered[1]["scene_sequence"], buffered[0]["sequence"]);
        assert_eq!(buffered[2]["kind"], "scan");
        assert_eq!(
            buffered[3],
            json!({"version":1,"kind":"publication_credit","token":1})
        );
        let latest: Value = serde_json::from_str(&fixture.bridge.latest.lock().unwrap().1).unwrap();
        socket.send(ready(1)).unwrap();
        let (current, _) = flight(&mut socket, 2);
        assert_eq!(current, latest);
        assert_eq!(current["paused"], !advancing);
        assert_eq!(current["nanoseconds"].as_u64().unwrap() > 0, advancing);
        barrier(&mut socket, 93);
    }
}

fn m19_fixture() -> Fixture {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/m19_robot.sdf")
        .canonicalize()
        .unwrap();
    let uri = url::Url::from_file_path(path).unwrap().to_string();
    let xml = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../examples/m19_robot.sdf"
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
    let robot = m19_entity(&sim, "default::robot");
    let robot_id = sim.world().get::<EntityId>(robot).unwrap().get();
    gz_sim_scene::set_diff_drive_command(&mut sim, robot_id, 0.4, 0.8).unwrap();
    sim.set_paused(true);
    let mut config = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(key, value).unwrap();
    }
    let mut server = Server::with_zenoh_config(
        sim,
        "default",
        gz_sim_server::ServerOptions {
            partition: format!(
                "m20-web-{}-{}",
                std::process::id(),
                ID.fetch_add(1, Ordering::Relaxed)
            ),
            ..Default::default()
        },
        config,
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    // Exercise maintained local acquisition, not a new Zenoh sensor observer.
    // Canonical wire values retain their separately accepted family proofs.
    server.set_interest_policy(gz_sim_server::InterestPolicy::Always);
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let address = listener.local_addr().unwrap();
    let bridge =
        Bridge::with_options(server.simulation(), "default", listener, Some(&sdf)).unwrap();
    Fixture {
        server,
        bridge,
        address,
        sdf,
    }
}

fn m19_entity(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
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

fn m19_metadata(sim: &Simulation) -> impl PartialEq + std::fmt::Debug + use<> {
    let world = sim.world();
    let mut identities: Vec<_> = world
        .iter_entities()
        .filter_map(|entity| {
            Some((
                entity.get::<EntityId>()?.get(),
                entity.get::<gz_sim_scene::Name>()?.0.clone(),
            ))
        })
        .collect();
    identities.sort_unstable();
    (
        identities,
        world
            .get::<gz_sim_scene::AltimeterPublication>(m19_entity(
                sim,
                "default::robot::base_link::altimeter",
            ))
            .unwrap()
            .clone(),
        world
            .get::<gz_sim_scene::AirPressurePublication>(m19_entity(
                sim,
                "default::robot::base_link::air_pressure",
            ))
            .unwrap()
            .clone(),
        world
            .get::<gz_sim_scene::MagnetometerPublication>(m19_entity(
                sim,
                "default::robot::base_link::magnetometer",
            ))
            .unwrap()
            .clone(),
        world
            .get::<gz_sim_scene::JointStatePublication>(m19_entity(sim, "default::robot"))
            .unwrap()
            .clone(),
        world
            .get::<gz_sim_scene::DiffDriveConfiguration>(m19_entity(sim, "default::robot"))
            .unwrap()
            .clone(),
    )
}

#[test]
fn credited_m19_catalog_keeps_every_visual_and_refuses_robot_edits_atomically() {
    use std::collections::BTreeSet;
    let mut fixture = m19_fixture();
    let metadata = m19_metadata(fixture.server.simulation());
    let counts = fixture.server.simulation().scheduled_system_counts();
    let mut socket = fixture.connect("/?scan=1&lifecycle=1&lifecycle_credit=1");
    let (initial, catalog) = flight(&mut socket, 1);
    let visuals = initial["visuals"].as_array().unwrap();
    assert_eq!(visuals.len(), 6);
    assert_eq!(
        visuals
            .iter()
            .map(|visual| visual["name"].as_str().unwrap())
            .collect::<BTreeSet<_>>(),
        BTreeSet::from([
            "default::ground::body::ground",
            "default::robot::base_link::obj_cargo",
            "default::robot::base_link::chassis",
            "default::robot::left_wheel::wheel",
            "default::robot::right_wheel::wheel",
            "default::obj_landmark::body::obj_mesh",
        ])
    );
    assert_eq!(
        visuals
            .iter()
            .filter(|visual| visual["shape"]["kind"] == "mesh")
            .count(),
        2
    );
    let models = catalog["models"].as_array().unwrap();
    assert_eq!(models.len(), 3);
    assert_eq!(
        models
            .iter()
            .map(|model| model["name"].as_str().unwrap())
            .collect::<BTreeSet<_>>(),
        BTreeSet::from(["default::ground", "default::robot", "default::obj_landmark"])
    );
    let owned: Vec<_> = models
        .iter()
        .flat_map(|model| model["visual_ids"].as_array().unwrap())
        .map(|id| id.as_u64().unwrap())
        .collect();
    assert_eq!(owned.len(), 6);
    assert_eq!(
        owned.into_iter().collect::<BTreeSet<_>>(),
        visuals
            .iter()
            .map(|visual| visual["id"].as_u64().unwrap())
            .collect()
    );
    let robot = models
        .iter()
        .find(|model| model["name"] == "default::robot")
        .unwrap();
    assert_eq!(robot["visual_ids"].as_array().unwrap().len(), 4);
    assert_eq!(robot["can_remove"], false);
    assert_eq!(robot["can_set_state"], false);
    assert!(
        catalog["templates"]
            .as_array()
            .unwrap()
            .iter()
            .all(|template| template["label"] != "robot")
    );
    let robot = robot["model_id"].as_u64().unwrap();
    let world = initial["world_id"].as_u64().unwrap();
    for (index, action) in ["remove", "set_state"].into_iter().enumerate() {
        let before = native(fixture.server.simulation());
        let next_id = next_entity_id(fixture.server.simulation().world()).unwrap();
        let mut message = target(index as u64 + 1, world, 0, action, robot);
        if action == "set_state" {
            message["translation"] = json!([3, 0, 1]);
            message["rotation"] = json!([0, 0, 0, 1]);
            message["linear_velocity"] = json!([0, 0, 0]);
            message["angular_velocity"] = json!([0, 0, 0]);
        }
        socket
            .send(Message::Text(message.to_string().into()))
            .unwrap();
        wait("robot edit did not reach the lifecycle queue", || {
            assert!(fixture.bridge.controls(&mut fixture.server).is_empty());
            !fixture
                .bridge
                .lifecycle
                .as_ref()
                .unwrap()
                .pending
                .is_empty()
        });
        fixture.server.prepare_io().unwrap();
        fixture.server.tick().unwrap();
        fixture
            .bridge
            .apply_lifecycle(&mut fixture.server, "default")
            .unwrap();
        fixture
            .bridge
            .publish(fixture.server.simulation(), "default", Vec::new())
            .unwrap();
        assert_eq!(read(&mut socket)["kind"], "scene");
        assert_eq!(read(&mut socket)["kind"], "scan");
        assert_eq!(read(&mut socket)["models"], catalog["models"]);
        let ack = read(&mut socket);
        assert_eq!(ack["kind"], "lifecycle_ack");
        assert_eq!(ack["request_id"], index as u64 + 1);
        assert_eq!(ack["action"], action);
        assert_eq!(ack["ok"], false);
        assert!(ack["state"].is_null());
        assert_eq!(native(fixture.server.simulation()), before);
        assert_eq!(fixture.server.simulation().elapsed(), Duration::ZERO);
        assert_eq!(fixture.server.simulation().epoch(), 0);
        assert_eq!(
            next_entity_id(fixture.server.simulation().world()).unwrap(),
            next_id
        );
        assert_eq!(m19_metadata(fixture.server.simulation()), metadata);
        assert_eq!(
            fixture.server.simulation().scheduled_system_counts(),
            counts
        );
        // A refused edit reply does not release the outstanding normal flight.
        barrier(&mut socket, 94);
    }
    // A committed reply already marks its scene as sent. Ready releases credit,
    // but does not require retransmission of that same frame: publish a fresh
    // paused frame first and prove it remains blocked by the held credit.
    let committed = Arc::clone(&fixture.bridge.latest.lock().unwrap().1);
    fixture.bridge.published = Instant::now() - Duration::from_millis(35);
    fixture
        .bridge
        .publish(fixture.server.simulation(), "default", Vec::new())
        .unwrap();
    let latest = Arc::clone(&fixture.bridge.latest.lock().unwrap().1);
    assert!(!Arc::ptr_eq(&committed, &latest));
    let latest: Value = serde_json::from_str(&latest).unwrap();
    barrier(&mut socket, 99);
    socket.send(ready(1)).unwrap();
    let (current, _) = flight(&mut socket, 2);
    assert_eq!(current, latest);
    assert_eq!(current["visuals"], initial["visuals"]);
    barrier(&mut socket, 95);
}

#[test]
fn credited_m19_delay_keeps_systems_advancing_and_reset_reconnect_restores_all_identities() {
    let mut fixture = m19_fixture();
    let metadata = m19_metadata(fixture.server.simulation());
    let counts = fixture.server.simulation().scheduled_system_counts();
    let mut socket = fixture.connect("/?scan=1&lifecycle=1&lifecycle_credit=1");
    let (initial, catalog) = flight(&mut socket, 1);
    assert!(fixture.server.apply_local_control(WorldControl::default()));
    for _ in 0..20 {
        for _ in 0..20 {
            fixture.server.tick().unwrap();
        }
        fixture.bridge.published = Instant::now() - Duration::from_millis(35);
        fixture
            .bridge
            .publish(fixture.server.simulation(), "default", Vec::new())
            .unwrap();
        barrier(&mut socket, 96);
    }
    let sim = fixture.server.simulation();
    assert_eq!(sim.elapsed(), Duration::from_millis(400));
    assert_eq!(m19_metadata(sim), metadata);
    assert_eq!(sim.scheduled_system_counts(), counts);
    let robot = m19_entity(sim, "default::robot");
    let joints = sim
        .world()
        .get::<gz_sim_scene::JointStateSample>(robot)
        .unwrap();
    assert_eq!(joints.epoch, 0);
    assert!(joints.time > Duration::ZERO && joints.time <= sim.elapsed());
    assert!(joints.joints.iter().any(|joint| joint.velocity.abs() > 0.0));
    let drive = sim
        .world()
        .get::<gz_sim_scene::DiffDriveSample>(robot)
        .unwrap();
    assert_eq!(drive.epoch, 0);
    assert!(drive.time > Duration::ZERO && drive.pose.iter().any(|value| value.abs() > 0.0));
    let altimeter = sim
        .world()
        .get::<gz_sim_scene::AltimeterSample>(m19_entity(
            sim,
            "default::robot::base_link::altimeter",
        ))
        .unwrap();
    assert_eq!(altimeter.epoch, 0);
    assert!(altimeter.time > Duration::ZERO && altimeter.reading.is_ok());
    let pressure = sim
        .world()
        .get::<gz_sim_scene::AirPressureSample>(m19_entity(
            sim,
            "default::robot::base_link::air_pressure",
        ))
        .unwrap();
    assert_eq!(pressure.epoch, 0);
    assert!(pressure.time > Duration::ZERO && pressure.reading.is_ok());
    let magnetic = sim
        .world()
        .get::<gz_sim_scene::MagnetometerSample>(m19_entity(
            sim,
            "default::robot::base_link::magnetometer",
        ))
        .unwrap();
    assert_eq!(magnetic.epoch, 0);
    assert!(magnetic.time > Duration::ZERO && magnetic.reading.is_ok());
    let latest: Value = serde_json::from_str(&fixture.bridge.latest.lock().unwrap().1).unwrap();
    socket.send(ready(1)).unwrap();
    assert_eq!(flight(&mut socket, 2).0, latest);
    assert_ne!(latest["visuals"], initial["visuals"]);

    socket
        .send(Message::Text(
            json!({"version":1,"kind":"control","request_id":1,"action":"reset"})
                .to_string()
                .into(),
        ))
        .unwrap();
    let mut controls = Vec::new();
    wait("coexistence reset did not reach the shared queue", || {
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
    barrier(&mut socket, 97);
    socket.send(ready(2)).unwrap();
    let (restored, restored_catalog) = flight(&mut socket, 3);
    assert_eq!(restored["epoch"], 1);
    assert_eq!(restored["visuals"], initial["visuals"]);
    assert_eq!(restored_catalog["models"], catalog["models"]);
    assert_eq!(m19_metadata(fixture.server.simulation()), metadata);
    assert_eq!(
        fixture.server.simulation().scheduled_system_counts(),
        counts
    );
    socket.close(None).unwrap();
    drop(socket);
    wait("credited coexistence peer did not retire", || {
        fixture.bridge.connected.load(Ordering::Acquire) == 0
    });
    let mut reconnected = fixture.connect("/?scan=1&lifecycle=1&lifecycle_credit=1");
    let (fresh, fresh_catalog) = flight(&mut reconnected, 1);
    assert_eq!(fresh["epoch"], 1);
    assert_eq!(fresh["seconds"], 0);
    assert_eq!(fresh["nanoseconds"], 0);
    assert_eq!(fresh["paused"], true);
    assert_eq!(fresh["visuals"], initial["visuals"]);
    assert_eq!(fresh_catalog["models"], catalog["models"]);
    barrier(&mut reconnected, 98);
}
