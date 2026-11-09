// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Actual local socket/native-scene lifecycle controls, qualified with all of M10.
use super::*;
use gz_sim_core::{EntityId, next_entity_id};
use gz_sim_scene::{BodyHandle, WorldPose, model_state};
use serde_json::{Value, json};
use std::{
    io::Read,
    net::{SocketAddr, TcpStream},
    num::NonZeroUsize,
};
use tungstenite::{Message, WebSocket, client::IntoClientRequest};

const XML: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../examples/m10_world_lifecycle.sdf"
));
static ID: AtomicU64 = AtomicU64::new(0);

struct Fixture {
    server: Server,
    bridge: Bridge,
    address: SocketAddr,
    sdf: sdformat::Sdf,
}
impl Fixture {
    fn new(enabled: bool) -> Self {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../examples/m10_world_lifecycle.sdf");
        let uri = url::Url::from_file_path(path.canonicalize().unwrap())
            .unwrap()
            .to_string();
        let sdf = sdformat::load_str(XML, &uri, &sdformat::NoResources).unwrap();
        let mut sim = gz_sim_scene::load_world_with_sdf_step(
            XML,
            &uri,
            "default",
            &sdformat::NoResources,
            gz_sim_scene::PhysicsEngine::Rapier,
        )
        .unwrap();
        sim.set_paused(true);
        let mut config = zenoh::Config::default();
        for (path, value) in [
            ("mode", "\"peer\""),
            ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
            ("scouting/multicast/enabled", "false"),
            ("scouting/gossip/enabled", "false"),
        ] {
            config.insert_json5(path, value).unwrap();
        }
        let server = Server::with_zenoh_config(
            sim,
            "default",
            gz_sim_server::ServerOptions {
                partition: format!(
                    "m10-web-{}-{}",
                    std::process::id(),
                    ID.fetch_add(1, Ordering::Relaxed)
                ),
                ..Default::default()
            },
            config,
            NonZeroUsize::new(8).unwrap(),
        )
        .unwrap();
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let address = listener.local_addr().unwrap();
        let bridge = Bridge::with_options(
            server.simulation(),
            "default",
            listener,
            enabled.then_some(&sdf),
        )
        .unwrap();
        Self {
            server,
            bridge,
            address,
            sdf,
        }
    }
    fn connect(&mut self, path: &str) -> WebSocket<TcpStream> {
        let socket = client(self.address, path).unwrap();
        wait("socket did not become active", || {
            self.bridge.connected.load(Ordering::Acquire) != 0
        });
        self.bridge
            .publish(self.server.simulation(), "default", Vec::new())
            .unwrap();
        socket
    }
    fn bootstrap(&mut self, socket: &mut WebSocket<TcpStream>) -> (Value, Value) {
        let scene = read(socket);
        let catalog = read(socket);
        assert_eq!(scene["kind"], "scene");
        assert_eq!(catalog["kind"], "lifecycle_catalog");
        assert_eq!(scene["sequence"], catalog["scene_sequence"]);
        assert_eq!(scene["epoch"], catalog["epoch"]);
        (scene, catalog)
    }
    fn command(
        &mut self,
        socket: &mut WebSocket<TcpStream>,
        request: Value,
    ) -> (Value, Value, Value) {
        socket
            .send(Message::Text(request.to_string().into()))
            .unwrap();
        let mut controls = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            controls.extend(self.bridge.controls(&mut self.server));
            if !controls.is_empty()
                || self
                    .bridge
                    .lifecycle
                    .as_ref()
                    .is_some_and(|state| !state.pending.is_empty())
            {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "request did not reach the shared CPU queue"
            );
            thread::sleep(Duration::from_millis(2));
        }
        self.server.prepare_io().unwrap();
        self.server.tick().unwrap();
        self.bridge
            .apply_lifecycle(&mut self.server, "default")
            .unwrap();
        self.bridge
            .publish(self.server.simulation(), "default", controls)
            .unwrap();
        let (mut scene, mut catalog, mut ack) = (None, None, None);
        while scene.is_none() || catalog.is_none() || ack.is_none() {
            let value = read(socket);
            match value["kind"].as_str().unwrap() {
                "scene" => scene = Some(value),
                "lifecycle_catalog" => catalog = Some(value),
                "ack" | "lifecycle_ack" => {
                    assert_eq!(value["request_id"], request["request_id"]);
                    ack = Some(value);
                }
                other => panic!("unexpected lifecycle frame {other}"),
            }
        }
        let (scene, catalog, ack) = (scene.unwrap(), catalog.unwrap(), ack.unwrap());
        assert_eq!(scene["sequence"], catalog["scene_sequence"]);
        for key in ["epoch", "seconds", "nanoseconds", "paused"] {
            assert_eq!(scene[key], ack[key]);
        }
        if ack["kind"] == "lifecycle_ack" {
            assert_eq!(scene["sequence"], ack["scene_sequence"]);
        }
        (scene, catalog, ack)
    }
}
fn client(address: SocketAddr, path: &str) -> Result<WebSocket<TcpStream>, tungstenite::Error> {
    let stream = TcpStream::connect_timeout(&address, Duration::from_secs(2))?;
    stream.set_read_timeout(Some(Duration::from_secs(2)))?;
    stream.set_write_timeout(Some(Duration::from_secs(2)))?;
    match tungstenite::client(
        format!("ws://{address}{path}").into_client_request()?,
        stream,
    ) {
        Ok((socket, _)) => Ok(socket),
        Err(tungstenite::HandshakeError::Failure(error)) => Err(error),
        Err(tungstenite::HandshakeError::Interrupted(_)) => panic!("blocking handshake suspended"),
    }
}
fn read(socket: &mut WebSocket<TcpStream>) -> Value {
    loop {
        match socket.read().unwrap() {
            Message::Text(text) => return serde_json::from_str(&text).unwrap(),
            Message::Ping(_) | Message::Pong(_) => {}
            other => panic!("unexpected frame {other:?}"),
        }
    }
}
fn wait(message: &str, mut ready: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(2);
    while !ready() {
        assert!(Instant::now() < deadline, "{message}");
        thread::sleep(Duration::from_millis(2));
    }
}
fn request(id: u64, world: u64, epoch: u64, action: &str) -> Value {
    json!({"version":1,"kind":"lifecycle","request_id":id,"world_id":world,"epoch":epoch,"action":action})
}
fn template(catalog: &Value, label: &str) -> u64 {
    catalog["templates"]
        .as_array()
        .unwrap()
        .iter()
        .find(|template| template["label"] == label)
        .unwrap()["template_id"]
        .as_u64()
        .unwrap()
}
fn spawn(id: u64, world: u64, epoch: u64, template: u64, translation: [f64; 3]) -> Value {
    let mut value = request(id, world, epoch, "spawn");
    value["template_id"] = json!(template);
    value["translation"] = json!(translation);
    value["rotation"] = json!([0, 0, 0, 1]);
    value
}
fn target(id: u64, world: u64, epoch: u64, action: &str, model: u64) -> Value {
    let mut value = request(id, world, epoch, action);
    value["model_id"] = json!(model);
    value
}
fn native(sim: &Simulation) -> Vec<(u64, gz_physics::BodyId, gz_physics::BodyState)> {
    let mut values: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| {
            entity.get::<BodyHandle>().map(|body| {
                (
                    entity.get::<EntityId>().unwrap().get(),
                    body.id(),
                    sim.world()
                        .resource::<gz_physics::PhysicsBackend>()
                        .state(body.id())
                        .unwrap(),
                )
            })
        })
        .collect();
    values.sort_by_key(|(id, _, _)| *id);
    values
}

#[test]
fn real_socket_spawn_state_remove_mesh_alias_reset_and_reconnect_keep_current_identities() {
    let mut fixture = Fixture::new(true);
    let mut socket = fixture.connect("/?lifecycle=1");
    let (initial, catalog) = fixture.bootstrap(&mut socket);
    let world = initial["world_id"].as_u64().unwrap();
    let original_native = native(fixture.server.simulation());
    let original_mesh_hit = fixture
        .server
        .simulation()
        .world()
        .resource::<gz_physics::PhysicsBackend>()
        .cast_ray([0.1, -0.2, 10.0], [0.0, 0.0, -1.0], 7.0, None)
        .unwrap()
        .unwrap();
    let mesh_definition = fixture
        .server
        .simulation()
        .world()
        .iter_entities()
        .find_map(
            |entity| match entity.get::<gz_sim_scene::VisualGeometry>()?.0.as_ref() {
                sdformat::Geometry::Mesh(mesh) => Some(mesh.clone()),
                _ => None,
            },
        )
        .unwrap();
    let first = next_entity_id(fixture.server.simulation().world()).unwrap();
    let (scene, catalog, ack) = fixture.command(
        &mut socket,
        spawn(1, world, 0, template(&catalog, "box"), [5.0, 1.0, 4.0]),
    );
    let added = ack["model_id"].as_u64().unwrap();
    assert_eq!(added, first);
    assert_eq!(ack["ok"], true);
    assert!(ack["state"].is_null());
    assert_eq!(
        scene["visuals"].as_array().unwrap().len(),
        initial["visuals"].as_array().unwrap().len() + 1
    );
    assert_eq!(
        &native(fixture.server.simulation())[..original_native.len()],
        &original_native
    );
    assert_eq!(fixture.server.simulation().elapsed(), Duration::ZERO);
    let mut edit = target(2, world, 0, "set_state", added);
    edit["translation"] = json!([6, 2, 5]);
    edit["rotation"] = json!([0, 0, 1, 0]);
    edit["linear_velocity"] = json!([0.25, 0, 0]);
    edit["angular_velocity"] = json!([0, 0, 0.5]);
    let (edited, _, ack) = fixture.command(&mut socket, edit);
    assert_eq!(ack["ok"], true);
    assert_eq!(ack["state"]["translation"], json!([6.0, 2.0, 5.0]));
    let body_state = model_state(fixture.server.simulation(), added).unwrap();
    assert_eq!(body_state.translation, [6.0, 2.0, 5.0]);
    assert_eq!(body_state.linear_velocity, [0.25, 0.0, 0.0]);
    assert_eq!(
        edited["visuals"]
            .as_array()
            .unwrap()
            .iter()
            .find(|visual| visual["id"] == added + 3)
            .unwrap()["translation"],
        json!([6.0, 2.0, 5.0])
    );
    let (_, _, ack) = fixture.command(&mut socket, target(3, world, 0, "get_state", added));
    assert_eq!(ack["state"]["angular_velocity"], json!([0.0, 0.0, 0.5]));
    let before = native(fixture.server.simulation());
    let high_water = next_entity_id(fixture.server.simulation().world()).unwrap();
    let (_, _, refused) = fixture.command(&mut socket, target(4, world, 99, "remove", added));
    assert_eq!(refused["ok"], false);
    assert!(refused["model_id"].is_null());
    assert!(refused["state"].is_null());
    assert_eq!(native(fixture.server.simulation()), before);
    assert_eq!(
        next_entity_id(fixture.server.simulation().world()).unwrap(),
        high_water
    );
    let mesh_model = catalog["models"]
        .as_array()
        .unwrap()
        .iter()
        .find(|model| model["name"] == "default::local_mesh")
        .unwrap()["model_id"]
        .as_u64()
        .unwrap();
    let original_mesh = initial["visuals"]
        .as_array()
        .unwrap()
        .iter()
        .find(|visual| visual["shape"]["kind"] == "mesh")
        .unwrap();
    let (_, catalog, removed) =
        fixture.command(&mut socket, target(5, world, 0, "remove", mesh_model));
    assert_eq!(removed["ok"], true);
    let (mesh_scene, _, mesh_ack) = fixture.command(
        &mut socket,
        spawn(
            6,
            world,
            0,
            template(&catalog, "local_mesh"),
            [0.0, 4.0, 5.0],
        ),
    );
    assert_eq!(mesh_ack["ok"], true);
    let mesh_id = mesh_ack["model_id"].as_u64().unwrap();
    let cloned_mesh = mesh_scene["visuals"]
        .as_array()
        .unwrap()
        .iter()
        .find(|visual| visual["id"] == mesh_id + 3)
        .unwrap();
    assert_eq!(cloned_mesh["shape"], original_mesh["shape"]);
    let cloned_hit = fixture
        .server
        .simulation()
        .world()
        .resource::<gz_physics::PhysicsBackend>()
        .cast_ray([0.1, 3.8, 11.0], [0.0, 0.0, -1.0], 7.0, None)
        .unwrap()
        .unwrap();
    assert!((cloned_hit - original_mesh_hit).abs() < 1e-12);
    let uri = url::Url::parse(cloned_mesh["shape"]["uri"].as_str().unwrap()).unwrap();
    let mut resource = TcpStream::connect(("127.0.0.1", uri.port().unwrap())).unwrap();
    resource
        .set_read_timeout(Some(Duration::from_secs(2)))
        .unwrap();
    use std::io::Write;
    write!(
        resource,
        "GET {} HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n",
        uri.path()
    )
    .unwrap();
    let mut response = Vec::new();
    resource.read_to_end(&mut response).unwrap();
    assert!(response.starts_with(b"HTTP/1.1 200"));
    let body = response
        .windows(4)
        .position(|bytes| bytes == b"\r\n\r\n")
        .unwrap()
        + 4;
    assert_eq!(
        &response[body..],
        include_bytes!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/assets/m2/pyramid.gltf"
        ))
    );
    let (without_mesh, _, removed) =
        fixture.command(&mut socket, target(7, world, 0, "remove", mesh_id));
    assert_eq!(removed["ok"], true);
    assert!(
        !without_mesh["visuals"]
            .as_array()
            .unwrap()
            .iter()
            .any(|visual| visual["id"] == mesh_id + 3)
    );
    assert!(
        fixture
            .bridge
            .assets
            .as_ref()
            .unwrap()
            .uri(mesh_id + 3, &mesh_definition)
            .is_err()
    );
    assert_eq!(
        fixture
            .server
            .simulation()
            .world()
            .resource::<gz_physics::PhysicsBackend>()
            .cast_ray([0.1, 3.8, 11.0], [0.0, 0.0, -1.0], 7.0, None)
            .unwrap(),
        None
    );
    let (_, _, removed) = fixture.command(&mut socket, target(8, world, 0, "remove", added));
    assert_eq!(removed["ok"], true);
    let (_, _, sphere) = fixture.command(
        &mut socket,
        spawn(9, world, 0, template(&catalog, "sphere"), [-5.0, 0.0, 4.0]),
    );
    assert_eq!(sphere["ok"], true);
    assert!(sphere["model_id"].as_u64().unwrap() > mesh_id);
    let (_, _, live_mesh) = fixture.command(
        &mut socket,
        spawn(
            10,
            world,
            0,
            template(&catalog, "local_mesh"),
            [0.0, -4.0, 6.0],
        ),
    );
    assert_eq!(live_mesh["ok"], true);
    let reset_alias = live_mesh["model_id"].as_u64().unwrap() + 3;
    assert!(
        fixture
            .bridge
            .assets
            .as_ref()
            .unwrap()
            .uri(reset_alias, &mesh_definition)
            .is_ok()
    );
    let (reset, restored, ack) = fixture.command(
        &mut socket,
        json!({"version":1,"kind":"control","request_id":11,"action":"reset"}),
    );
    assert_eq!(ack["ok"], true);
    assert_eq!(reset["epoch"], 1);
    assert_eq!(reset["visuals"], initial["visuals"]);
    assert_eq!(restored["models"].as_array().unwrap().len(), 4);
    assert!(
        fixture
            .bridge
            .assets
            .as_ref()
            .unwrap()
            .uri(reset_alias, &mesh_definition)
            .is_err()
    );
    assert_eq!(
        fixture
            .bridge
            .assets
            .as_ref()
            .unwrap()
            .uri(original_mesh["id"].as_u64().unwrap(), &mesh_definition)
            .unwrap(),
        original_mesh["shape"]["uri"].as_str().unwrap()
    );
    drop(socket);
    wait("disconnected peer retained ownership", || {
        fixture.bridge.connected.load(Ordering::Acquire) == 0
    });
    let mut socket = fixture.connect("/?lifecycle=1");
    let (current, catalog) = fixture.bootstrap(&mut socket);
    assert_eq!(current["epoch"], 1);
    assert_eq!(current["visuals"], initial["visuals"]);
    let (_, _, stale) = fixture.command(
        &mut socket,
        spawn(1, world, 0, template(&catalog, "box"), [0.0, 0.0, 8.0]),
    );
    assert_eq!(stale["ok"], false);
}

#[test]
fn lifecycle_capability_and_shared_pending_queue_have_no_default_mutation_path() {
    let mut off = Fixture::new(false);
    assert!(off.bridge.lifecycle.is_none());
    assert!(client(off.address, "/?lifecycle=1").is_err());
    let mut socket = off.connect("/");
    assert_eq!(read(&mut socket)["kind"], "scene");
    let before = native(off.server.simulation());
    socket
        .send(Message::Text(
            spawn(1, 1, 0, 1, [0.0, 0.0, 5.0]).to_string().into(),
        ))
        .unwrap();
    wait("unrequested lifecycle did not close socket", || {
        off.bridge.connected.load(Ordering::Acquire) == 0
    });
    assert!(matches!(
        off.bridge.commands.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));
    assert_eq!(native(off.server.simulation()), before);
    let mut on = Fixture::new(true);
    let mut plain = on.connect("/");
    assert_eq!(read(&mut plain)["kind"], "scene");
    plain.send(Message::Ping(vec![7, 3].into())).unwrap();
    assert_eq!(plain.read().unwrap(), Message::Pong(vec![7, 3].into()));
    plain
        .send(Message::Text(
            spawn(1, 1, 0, 1, [0.0, 0.0, 5.0]).to_string().into(),
        ))
        .unwrap();
    wait(
        "enabled server admitted an unrequested lifecycle capability",
        || on.bridge.connected.load(Ordering::Acquire) == 0,
    );
    assert!(matches!(
        on.bridge.commands.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));
    for path in [
        "/?lifecycle=0",
        "/?lifecycle=1&lifecycle=1",
        "/?scan=1&scan=1&lifecycle=1",
    ] {
        assert!(client(on.address, path).is_err());
    }
    let mut socket = on.connect("/?lifecycle=1");
    let (scene, catalog) = on.bootstrap(&mut socket);
    let world = scene["world_id"].as_u64().unwrap();
    let (_, _, ack) = on.command(
        &mut socket,
        spawn(1, world, 0, template(&catalog, "box"), [8.0, 0.0, 3.0]),
    );
    assert_eq!(ack["ok"], true);
    let (_, _, ack) = on.command(
        &mut socket,
        json!({"version":1,"kind":"control","request_id":2,"action":"pause"}),
    );
    assert_eq!(ack["ok"], true);
    socket
        .send(Message::Text(
            target(2, world, 0, "get_state", 2).to_string().into(),
        ))
        .unwrap();
    wait("shared request ID duplicate remained live", || {
        on.bridge.connected.load(Ordering::Acquire) == 0
    });
    assert!(matches!(
        on.bridge.commands.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));
}

#[test]
fn abandoned_lifecycle_requests_share_the_existing_eight_slot_ingress_bound() {
    let mut fixture = Fixture::new(true);
    let before = native(fixture.server.simulation());
    for _ in 0..8 {
        let mut socket = client(fixture.address, "/?lifecycle=1").unwrap();
        wait("socket did not connect", || {
            fixture.bridge.connected.load(Ordering::Acquire) != 0
        });
        socket
            .send(Message::Text(
                spawn(1, 1, 0, 1, [8.0, 0.0, 4.0]).to_string().into(),
            ))
            .unwrap();
        drop(socket);
        wait("abandoned lifecycle peer stayed live", || {
            fixture.bridge.connected.load(Ordering::Acquire) == 0
        });
    }
    let mut ninth = client(fixture.address, "/?lifecycle=1").unwrap();
    wait("overflow socket did not connect", || {
        fixture.bridge.connected.load(Ordering::Acquire) != 0
    });
    ninth
        .send(Message::Text(
            spawn(1, 1, 0, 1, [8.0, 0.0, 4.0]).to_string().into(),
        ))
        .unwrap();
    wait("full shared queue did not close overflow peer", || {
        fixture.bridge.connected.load(Ordering::Acquire) == 0
    });
    let queued: Vec<_> = fixture.bridge.commands.try_iter().collect();
    assert_eq!(queued.len(), 8);
    assert!(
        queued
            .iter()
            .all(|queued| !queued.alive().load(Ordering::Acquire))
    );
    drop(queued);
    assert!(fixture.bridge.controls(&mut fixture.server).is_empty());
    assert_eq!(native(fixture.server.simulation()), before);
}

#[test]
fn committed_reply_retains_its_exact_scene_even_after_latest_advances() {
    let mut fixture = Fixture::new(true);
    let (sender, receiver) = mpsc::sync_channel(8);
    fixture.bridge.commands = receiver;
    let (reply, result) = mpsc::sync_channel(1);
    let request =
        lifecycle::Request::decode(&spawn(1, 1, 0, 2, [8.0, 0.0, 4.0]).to_string(), 0).unwrap();
    assert!(
        sender
            .send(Queued::Lifecycle(lifecycle::Pending {
                trace: None,
                request,
                alive: Arc::new(AtomicBool::new(true)),
                reply
            }))
            .is_ok()
    );
    let controls = fixture.bridge.controls(&mut fixture.server);
    assert!(controls.is_empty());
    fixture.server.tick().unwrap();
    fixture
        .bridge
        .apply_lifecycle(&mut fixture.server, "default")
        .unwrap();
    fixture
        .bridge
        .publish(fixture.server.simulation(), "default", controls)
        .unwrap();
    let committed = result.recv_timeout(Duration::from_secs(2)).unwrap();
    let captured: Value = serde_json::from_str(&committed.scene).unwrap();
    assert!(fixture.server.apply_local_control(WorldControl::default()));
    fixture.server.tick().unwrap();
    fixture.bridge.published = Instant::now() - Duration::from_millis(35);
    // A real connected-generation marker permits the normal publication path;
    // the retained reply owner is deliberately delayed without changing frames.
    fixture.bridge.connected.store(1, Ordering::Release);
    fixture
        .bridge
        .publish(fixture.server.simulation(), "default", Vec::new())
        .unwrap();
    let latest = fixture.bridge.latest.lock().unwrap();
    let newer: Value = serde_json::from_str(&latest.1).unwrap();
    assert!(newer["sequence"].as_u64().unwrap() > captured["sequence"].as_u64().unwrap());
    assert!(!Arc::ptr_eq(&latest.1, &committed.scene));
    let ack: Value = serde_json::from_str(&committed.ack).unwrap();
    let catalog: Value = serde_json::from_str(&committed.catalog).unwrap();
    assert_eq!(ack["scene_sequence"], captured["sequence"]);
    assert_eq!(catalog["scene_sequence"], captured["sequence"]);
    for key in ["epoch", "seconds", "nanoseconds", "paused"] {
        assert_eq!(ack[key], captured[key]);
    }
}

#[test]
fn full_scene_and_sequence_limits_refuse_before_native_or_identity_mutation() {
    let mut fixture = Fixture::new(true);
    let native_template = gz_sim_scene::prepare_model_template(
        fixture.server.simulation(),
        &fixture.sdf,
        &fixture.sdf.worlds[0].models[1],
    )
    .unwrap();
    for index in 0..124 {
        fixture
            .server
            .spawn_model_template(
                &native_template,
                WorldPose {
                    translation: [100.0 + index as f64 * 2.0, 0.0, 3.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                },
            )
            .unwrap();
    }
    let mut socket = fixture.connect("/?lifecycle=1");
    let (scene, catalog) = fixture.bootstrap(&mut socket);
    assert_eq!(scene["visuals"].as_array().unwrap().len(), 128);
    let before = native(fixture.server.simulation());
    let high_water = next_entity_id(fixture.server.simulation().world()).unwrap();
    let (_, _, ack) = fixture.command(
        &mut socket,
        spawn(1, 1, 0, template(&catalog, "box"), [0.0, 0.0, 8.0]),
    );
    assert_eq!(ack["ok"], false);
    assert!(ack["model_id"].is_null());
    assert_eq!(native(fixture.server.simulation()), before);
    assert_eq!(
        next_entity_id(fixture.server.simulation().world()).unwrap(),
        high_water
    );
    fixture.bridge.sequence = u64::MAX;
    let (reply, _outcome) = mpsc::sync_channel(1);
    fixture
        .bridge
        .lifecycle
        .as_mut()
        .unwrap()
        .pending
        .push(lifecycle::Pending {
            trace: None,
            request: lifecycle::Request::decode(&target(2, 1, 0, "remove", 2).to_string(), 1)
                .unwrap(),
            alive: Arc::new(AtomicBool::new(true)),
            reply,
        });
    assert!(
        fixture
            .bridge
            .apply_lifecycle(&mut fixture.server, "default")
            .is_err()
    );
    assert_eq!(native(fixture.server.simulation()), before);
    assert_eq!(
        next_entity_id(fixture.server.simulation().world()).unwrap(),
        high_water
    );
}

#[test]
fn actual_socket_holds_newer_latest_until_the_exact_delayed_reply_frame_and_ack() {
    let mut fixture = Fixture::new(true);
    let mut socket = fixture.connect("/?lifecycle=1");
    let (initial, catalog) = fixture.bootstrap(&mut socket);
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
        panic!("lifecycle request expected");
    };
    assert!(fixture.server.apply_local_control(WorldControl::default()));
    fixture.server.tick().unwrap();
    fixture.bridge.published = Instant::now() - Duration::from_millis(35);
    fixture
        .bridge
        .publish(fixture.server.simulation(), "default", Vec::new())
        .unwrap();
    let newer: Value = serde_json::from_str(&fixture.bridge.latest.lock().unwrap().1).unwrap();
    assert!(newer["sequence"].as_u64().unwrap() > initial["sequence"].as_u64().unwrap());
    // Two round trips force a complete socket polling iteration after Latest
    // changed, without sleeps or a production-only pause hook. A missing freeze
    // leaks the newer Scene before one of these Pongs and fails this assertion.
    for tag in [31, 32] {
        socket.send(Message::Ping(vec![tag].into())).unwrap();
        assert_eq!(socket.read().unwrap(), Message::Pong(vec![tag].into()));
    }
    // This is a transport test: retain a real socket request's reply owner and
    // supply a frame captured before Latest advanced. It does not claim this
    // fabricated CPU reply executed an extra native command or physics tick.
    let ack = json!({"version":1,"kind":"lifecycle_ack","request_id":1,"action":"get_state","world_id":initial["world_id"],"epoch":initial["epoch"],"scene_sequence":initial["sequence"],"seconds":initial["seconds"],"nanoseconds":initial["nanoseconds"],"paused":initial["paused"],"ok":true,"error":null,"model_id":model,"state":{"translation":state.translation,"rotation":state.rotation,"linear_velocity":state.linear_velocity,"angular_velocity":state.angular_velocity}});
    assert!(
        pending
            .reply
            .send(lifecycle::Committed {
                scene: Arc::new(initial.to_string()),
                scan: None,
                catalog: Arc::new(catalog.to_string()),
                ack: ack.to_string()
            })
            .is_ok()
    );
    assert_eq!(read(&mut socket), initial);
    assert_eq!(read(&mut socket), catalog);
    assert_eq!(read(&mut socket), ack);
    let current = read(&mut socket);
    assert_eq!(current, newer);
    assert_eq!(read(&mut socket)["scene_sequence"], newer["sequence"]);
}

#[test]
fn semantic_refusals_keep_actual_full_scene_native_state_and_allocator_atomic() {
    let mut fixture = Fixture::new(true);
    let mut socket = fixture.connect("/?lifecycle=1");
    let (initial, catalog) = fixture.bootstrap(&mut socket);
    let world = initial["world_id"].as_u64().unwrap();
    let model = |name| {
        catalog["models"]
            .as_array()
            .unwrap()
            .iter()
            .find(|model| model["name"] == name)
            .unwrap()["model_id"]
            .as_u64()
            .unwrap()
    };
    let edit = |id, rotation, translation, linear_velocity| {
        let mut value = target(1, world, 0, "set_state", id);
        value["rotation"] = rotation;
        value["translation"] = translation;
        value["linear_velocity"] = linear_velocity;
        value["angular_velocity"] = json!([0, 0, 0]);
        value
    };
    let mut cases = [
        target(1, world, 0, "get_state", 999999),
        target(
            1,
            world,
            0,
            "remove",
            initial["visuals"][0]["id"].as_u64().unwrap(),
        ),
        spawn(1, world, 0, 999999, [0.0, 0.0, 6.0]),
        edit(
            model("default::box"),
            json!([0, 0, 0, 0]),
            json!([1, 2, 3]),
            json!([0, 0, 0]),
        ),
        edit(
            model("default::box"),
            json!([0, 0, 0, 1]),
            json!([f64::MAX, 0, 0]),
            json!([0, 0, 0]),
        ),
        edit(
            model("default::ground"),
            json!([0, 0, 0, 1]),
            json!([1, 2, 3]),
            json!([1, 0, 0]),
        ),
        target(1, world + 1, 0, "remove", model("default::box")),
        target(1, world, 1, "remove", model("default::box")),
    ];
    let native_before = native(fixture.server.simulation());
    let next = next_entity_id(fixture.server.simulation().world()).unwrap();
    let clock = (
        fixture.server.simulation().epoch(),
        fixture.server.simulation().elapsed(),
        fixture.server.simulation().is_paused(),
    );
    for (index, value) in cases.iter_mut().enumerate() {
        value["request_id"] = json!(index + 1);
        let (scene, _, ack) = fixture.command(&mut socket, value.clone());
        assert_eq!(ack["ok"], false);
        assert!(ack["model_id"].is_null());
        assert!(ack["state"].is_null());
        assert_eq!(scene["visuals"], initial["visuals"]);
        assert_eq!(native(fixture.server.simulation()), native_before);
        assert_eq!(
            next_entity_id(fixture.server.simulation().world()).unwrap(),
            next
        );
        assert_eq!(
            (
                fixture.server.simulation().epoch(),
                fixture.server.simulation().elapsed(),
                fixture.server.simulation().is_paused()
            ),
            clock
        );
    }
}

#[test]
fn combined_scan_and_lifecycle_deliver_the_same_committed_frame_without_inventing_a_sample() {
    let mut fixture = Fixture::new(true);
    let mut socket = fixture.connect("/?scan=1&lifecycle=1");
    let initial = read(&mut socket);
    let catalog = read(&mut socket);
    let empty_scan = read(&mut socket);
    assert_eq!(initial["kind"], "scene");
    assert_eq!(empty_scan["kind"], "scan");
    assert!(empty_scan["scan"].is_null());
    assert_eq!(catalog["kind"], "lifecycle_catalog");
    assert_eq!(catalog["scene_sequence"], initial["sequence"]);
    let model = catalog["models"]
        .as_array()
        .unwrap()
        .iter()
        .find(|model| model["name"] == "default::box")
        .unwrap()["model_id"]
        .as_u64()
        .unwrap();
    socket
        .send(Message::Text(
            target(
                1,
                initial["world_id"].as_u64().unwrap(),
                0,
                "get_state",
                model,
            )
            .to_string()
            .into(),
        ))
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        assert!(fixture.bridge.controls(&mut fixture.server).is_empty());
        if !fixture
            .bridge
            .lifecycle
            .as_ref()
            .unwrap()
            .pending
            .is_empty()
        {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "combined opt-in request did not reach CPU"
        );
        thread::sleep(Duration::from_millis(2));
    }
    fixture.server.tick().unwrap();
    fixture
        .bridge
        .apply_lifecycle(&mut fixture.server, "default")
        .unwrap();
    fixture
        .bridge
        .publish(fixture.server.simulation(), "default", Vec::new())
        .unwrap();
    let scene = read(&mut socket);
    let scan = read(&mut socket);
    let catalog = read(&mut socket);
    let ack = read(&mut socket);
    assert_eq!(scene["kind"], "scene");
    assert_eq!(scan["kind"], "scan");
    assert_eq!(catalog["kind"], "lifecycle_catalog");
    assert_eq!(ack["kind"], "lifecycle_ack");
    assert_eq!(ack["ok"], true);
    assert_eq!(ack["model_id"], model);
    assert_eq!(scene["sequence"], catalog["scene_sequence"]);
    assert_eq!(scene["sequence"], ack["scene_sequence"]);
    assert_eq!(scene["epoch"], scan["epoch"]);
    assert_eq!(scene["epoch"], ack["epoch"]);
    assert_eq!(scan["sim_time_ns"], 0);
    assert!(scan["scan"].is_null());
    assert_eq!(scene["visuals"], initial["visuals"]);
}

#[test]
fn declared_mesh_alias_refusals_leave_captured_resource_native_state_and_ids_untouched() {
    let fixture = Fixture::new(true);
    let (source, mesh) = fixture
        .server
        .simulation()
        .world()
        .iter_entities()
        .find_map(
            |entity| match entity.get::<gz_sim_scene::VisualGeometry>()?.0.as_ref() {
                sdformat::Geometry::Mesh(mesh) => {
                    Some((entity.get::<EntityId>().unwrap().get(), mesh.clone()))
                }
                _ => None,
            },
        )
        .unwrap();
    let assets = fixture.bridge.assets.as_ref().unwrap();
    let uri = assets.uri(source, &mesh).unwrap();
    let native_before = native(fixture.server.simulation());
    let next = next_entity_id(fixture.server.simulation().world()).unwrap();
    let mut other = mesh.clone();
    other.scale[0] *= 2.0;
    for id in [0, source, u64::MAX] {
        assert!(assets.prepare_alias(source, id, &mesh).is_err());
    }
    assert!(assets.prepare_alias(u64::MAX, next + 3, &mesh).is_err());
    assert!(assets.prepare_alias(source, next + 3, &other).is_err());
    assert_eq!(assets.uri(source, &mesh).unwrap(), uri);
    assert!(assets.uri(next + 3, &mesh).is_err());
    assert_eq!(native(fixture.server.simulation()), native_before);
    assert_eq!(
        next_entity_id(fixture.server.simulation().world()).unwrap(),
        next
    );
    assert_eq!(fixture.server.simulation().elapsed(), Duration::ZERO);
}

#[test]
fn disconnect_after_cpu_handoff_and_lost_lifecycle_reply_ownership_never_mutate_the_world() {
    let mut fixture = Fixture::new(true);
    let mut socket = fixture.connect("/?lifecycle=1");
    let (initial, catalog) = fixture.bootstrap(&mut socket);
    let before = native(fixture.server.simulation());
    let next = next_entity_id(fixture.server.simulation().world()).unwrap();
    socket
        .send(Message::Text(
            spawn(1, 1, 0, template(&catalog, "box"), [8.0, 0.0, 4.0])
                .to_string()
                .into(),
        ))
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        assert!(fixture.bridge.controls(&mut fixture.server).is_empty());
        if !fixture
            .bridge
            .lifecycle
            .as_ref()
            .unwrap()
            .pending
            .is_empty()
        {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "request never reached CPU ownership"
        );
        thread::sleep(Duration::from_millis(2));
    }
    drop(socket);
    wait("disconnected pending peer retained its alive flag", || {
        fixture.bridge.connected.load(Ordering::Acquire) == 0
    });
    fixture
        .bridge
        .apply_lifecycle(&mut fixture.server, "default")
        .unwrap();
    assert!(
        fixture
            .bridge
            .lifecycle
            .as_ref()
            .unwrap()
            .pending
            .is_empty()
    );
    assert!(!fixture.bridge.lifecycle.as_ref().unwrap().has_replies());
    assert_eq!(native(fixture.server.simulation()), before);
    assert_eq!(
        next_entity_id(fixture.server.simulation().world()).unwrap(),
        next
    );
    let mut socket = fixture.connect("/?lifecycle=1");
    let (current, _) = fixture.bootstrap(&mut socket);
    assert_eq!(current["visuals"], initial["visuals"]);
    socket
        .send(Message::Text(
            target(1, 1, 0, "get_state", 2).to_string().into(),
        ))
        .unwrap();
    let Queued::Lifecycle(pending) = fixture
        .bridge
        .commands
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
    else {
        panic!("lifecycle request expected");
    };
    drop(pending);
    wait("abandoned lifecycle CPU reply did not retire peer", || {
        fixture.bridge.connected.load(Ordering::Acquire) == 0
    });
    assert_eq!(native(fixture.server.simulation()), before);
    assert_eq!(
        next_entity_id(fixture.server.simulation().world()).unwrap(),
        next
    );
}

#[path = "web_publication_credit.rs"]
mod publication_credit_tests;

#[path = "web_recording.rs"]
mod recording_tests;

#[path = "web_robot_sensors.rs"]
mod robot_sensors_tests;
