// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::{EntityKind, VisualGeometry, VisualMaterial, WorldPose};
use serde_json::{Value, json};

fn simulation() -> Simulation {
    gz_sim_scene::load_world(
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/mvp_primitives.sdf"
        )),
        "mvp.sdf",
        "default",
        Duration::from_millis(5),
        &sdformat::NoResources,
    )
    .unwrap()
}
fn frame(sim: &Simulation, sequence: u64) -> Value {
    serde_json::from_str(
        &snapshot::encode(&snapshot::extract(sim, "default", sequence).unwrap()).unwrap(),
    )
    .unwrap()
}

#[test]
fn snapshots_follow_real_rapier_motion_and_atomic_reset_with_stable_visuals() {
    let mut sim = simulation();
    let initial = frame(&sim, 0);
    let visuals = initial["visuals"].as_array().unwrap();
    assert_eq!(visuals.len(), 21);
    assert_eq!(
        visuals
            .iter()
            .filter(|v| v["shape"]["kind"] == "sphere")
            .count(),
        10
    );
    assert_eq!(
        visuals
            .iter()
            .filter(|v| v["shape"]["kind"] == "box")
            .count(),
        11
    );
    let ids: Vec<_> = visuals.iter().map(|v| v["id"].clone()).collect();
    sim.step_exact(20).unwrap();
    let moving = frame(&sim, 1);
    assert_eq!(moving["nanoseconds"], 100_000_000);
    assert!(
        moving["visuals"]
            .as_array()
            .unwrap()
            .iter()
            .zip(visuals)
            .filter(|(a, b)| a["translation"] != b["translation"])
            .count()
            >= 20
    );
    gz_sim_scene::reset_loaded_world(&mut sim, |_| ()).unwrap();
    let reset = frame(&sim, 2);
    assert_eq!(reset["epoch"], 1);
    assert_eq!(reset["seconds"], 0);
    assert_eq!(reset["nanoseconds"], 0);
    assert_eq!(reset["visuals"], initial["visuals"]);
    assert_eq!(
        reset["visuals"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v["id"].clone())
            .collect::<Vec<_>>(),
        ids
    );
}

#[test]
fn malformed_or_reordered_commands_never_become_world_controls() {
    for action in ["pause", "resume", "reset"] {
        let command = Control::decode(
            &json!({"version":1,"kind":"control","request_id":1,"action":action}).to_string(),
            0,
        )
        .unwrap();
        let physical = command.world_control();
        assert_eq!(physical.pause, action != "resume");
        assert_eq!(physical.reset.is_some(), action == "reset");
    }
    for value in [
        json!({}),
        json!({"version":2,"kind":"control","request_id":1,"action":"pause"}),
        json!({"version":1,"kind":"scene","request_id":1,"action":"pause"}),
        json!({"version":1,"kind":"control","request_id":0,"action":"pause"}),
        json!({"version":1,"kind":"control","request_id":1,"action":"step"}),
        json!({"version":1,"kind":"control","request_id":1,"action":"pause","extra":true}),
    ] {
        assert!(Control::decode(&value.to_string(), 0).is_err());
    }
    assert!(Control::decode("{", 0).is_err());
    assert!(
        Control::decode(
            r#"{"version":1,"kind":"control","request_id":2,"action":"pause"}"#,
            2
        )
        .is_err()
    );
}

#[test]
fn invalid_snapshot_metadata_is_rejected_without_changing_physics() {
    let mut sim = simulation();
    let visual = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<EntityKind>() == Some(&EntityKind::Visual))
        .unwrap()
        .id();
    let original = *sim.world().get::<WorldPose>(visual).unwrap();
    for value in [f64::NAN, f64::INFINITY, f64::MAX] {
        sim.world_mut()
            .get_mut::<WorldPose>(visual)
            .unwrap()
            .translation[0] = value;
        assert!(snapshot::extract(&sim, "default", 0).is_err());
    }
    *sim.world_mut().get_mut::<WorldPose>(visual).unwrap() = original;
    sim.world_mut()
        .get_mut::<WorldPose>(visual)
        .unwrap()
        .rotation = [0.0; 4];
    assert!(snapshot::extract(&sim, "default", 0).is_err());
    *sim.world_mut().get_mut::<WorldPose>(visual).unwrap() = original;
    let geometry = sim.world().get::<VisualGeometry>(visual).unwrap().0.clone();
    for invalid in [
        sdformat::Geometry::Empty,
        sdformat::Geometry::Sphere { radius: 0.0 },
        sdformat::Geometry::Box {
            size: [1.0, -1.0, 1.0],
        },
    ] {
        sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 = Arc::new(invalid);
        assert!(snapshot::extract(&sim, "default", 0).is_err());
    }
    sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 = geometry;
    sim.world_mut()
        .get_mut::<VisualMaterial>(visual)
        .unwrap()
        .0
        .diffuse[3] = 0.5;
    assert!(snapshot::extract(&sim, "default", 0).is_err());
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert!(snapshot::extract(&sim, &"x".repeat(257), 0).is_err());
}

#[test]
fn real_socket_acknowledges_committed_pause_reset_and_reconnect_snapshot() {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let address = listener.local_addr().unwrap();
    let options = gz_sim_server::ServerOptions {
        partition: format!("gzweb-test-{}-{}", std::process::id(), address.port()),
        ..Default::default()
    };
    let mut server = Server::with_zenoh_config(
        simulation(),
        "default",
        options,
        zenoh::Config::default(),
        std::num::NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    let mut bridge = Bridge::with_listener(server.simulation(), "default", listener).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let done = stop.clone();
    let reconnect = Arc::new(AtomicBool::new(false));
    let reconnect_client = reconnect.clone();
    let (ready, wait) = mpsc::sync_channel(1);
    let client = thread::spawn(move || {
        let (mut ws, _) = tungstenite::connect(format!("ws://{address}/")).unwrap();
        if let tungstenite::stream::MaybeTlsStream::Plain(tcp) = ws.get_mut() {
            tcp.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
        }
        let read = |ws: &mut tungstenite::WebSocket<_>, kind: &str| -> Value {
            loop {
                if let tungstenite::Message::Text(text) = ws.read().unwrap() {
                    let v: Value = serde_json::from_str(&text).unwrap();
                    if v["kind"] == kind {
                        return v;
                    }
                }
            }
        };
        assert_eq!(
            read(&mut ws, "scene")["visuals"].as_array().unwrap().len(),
            21
        );
        for (id, action) in [(1, "pause"), (2, "reset"), (3, "resume")] {
            ws.send(tungstenite::Message::Text(
                json!({"version":1,"kind":"control","request_id":id,"action":action})
                    .to_string()
                    .into(),
            ))
            .unwrap();
            let ack = read(&mut ws, "ack");
            assert_eq!(ack["request_id"], id);
            assert_eq!(ack["ok"], true);
            assert_eq!(ack["paused"], action != "resume");
            if action == "reset" {
                assert_eq!(ack["epoch"], 1);
                assert_eq!(ack["seconds"], 0);
                assert_eq!(ack["nanoseconds"], 0);
            }
        }
        ws.close(None).unwrap();
        drop(ws);
        reconnect_client.store(true, Ordering::Release);
        let expected = wait.recv_timeout(Duration::from_secs(3)).unwrap();
        let (mut fresh, _) = tungstenite::connect(format!("ws://{address}/")).unwrap();
        let first = read(&mut fresh, "scene");
        assert_eq!(first["epoch"], 1);
        let actual = first["seconds"].as_u64().unwrap() * 1_000_000_000
            + first["nanoseconds"].as_u64().unwrap();
        assert!(
            actual >= expected,
            "first reconnect scene must be captured after disconnected motion"
        );
        done.store(true, Ordering::Release);
    });
    let deadline = Instant::now() + Duration::from_secs(8);
    let mut advanced_disconnected = false;
    while !stop.load(Ordering::Acquire) && Instant::now() < deadline {
        if !advanced_disconnected
            && reconnect.load(Ordering::Acquire)
            && bridge.connected.load(Ordering::Acquire) == 0
        {
            for _ in 0..100 {
                server.prepare_io().unwrap();
                server.tick().unwrap();
            }
            ready
                .send(u64::try_from(server.simulation().elapsed().as_nanos()).unwrap())
                .unwrap();
            advanced_disconnected = true;
        }
        let controls = bridge.controls(&mut server);
        server.prepare_io().unwrap();
        server.tick().unwrap();
        bridge
            .publish(server.simulation(), "default", controls)
            .unwrap();
        thread::sleep(Duration::from_millis(2));
    }
    assert!(
        stop.load(Ordering::Acquire),
        "bounded real client did not complete"
    );
    client.join().unwrap();
    drop(bridge);
    server.shutdown();
}

#[path = "web_limits.rs"]
mod limits;

#[path = "web_admission.rs"]
mod admission;

#[test]
fn competing_controls_never_acknowledge_an_uncommitted_requested_state() {
    let mut server = Server::with_zenoh_config(
        simulation(),
        "default",
        gz_sim_server::ServerOptions {
            partition: format!("gzweb-control-race-{}", std::process::id()),
            ..Default::default()
        },
        zenoh::Config::default(),
        std::num::NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let mut bridge = Bridge::with_listener(server.simulation(), "default", listener).unwrap();
    let (commands, received) = mpsc::sync_channel(8);
    bridge.commands = received;
    let submit = |id: u64, action| {
        let (reply, ack) = mpsc::sync_channel(1);
        assert!(
            commands
                .send(Queued::Control(Pending {
                    admission: Admission::Boundary,
                    trace: None,
                    control: Control::decode(
                        &json!({"version":1,"kind":"control",
                "request_id":id,"action":action})
                        .to_string(),
                        id - 1,
                    )
                    .unwrap(),
                    alive: Arc::new(AtomicBool::new(true)),
                    reply,
                }))
                .is_ok()
        );
        ack
    };
    let reset = WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    };
    assert!(server.apply_local_control(reset));
    let ack = submit(1, "resume");
    let controls = bridge.controls(&mut server);
    assert!(server.io_preparation_required());
    server.prepare_io().unwrap();
    // Neither pending nor prepared reconstruction can be overwritten.
    assert!(!server.apply_local_control(WorldControl::default()));
    server.tick().unwrap();
    bridge
        .publish(server.simulation(), "default", controls)
        .unwrap();
    let refused: Value = serde_json::from_str(&ack.recv().unwrap()).unwrap();
    assert_eq!(refused["ok"], false);
    assert_eq!(refused["paused"], true);
    assert_eq!(refused["epoch"], 1);
    assert!(refused["error"].as_str().is_some());

    let ack = submit(2, "pause");
    let controls = bridge.controls(&mut server);
    // Another admitted input changes the state before the committing tick.
    assert!(server.apply_local_control(WorldControl::default()));
    server.prepare_io().unwrap();
    server.tick().unwrap();
    bridge
        .publish(server.simulation(), "default", controls)
        .unwrap();
    let superseded: Value = serde_json::from_str(&ack.recv().unwrap()).unwrap();
    assert_eq!(superseded["request_id"], 2);
    assert_eq!(superseded["ok"], false);
    assert_eq!(superseded["paused"], false);
    assert_eq!(superseded["nanoseconds"], 5_000_000);
    assert!(superseded["error"].as_str().is_some());
    drop(bridge);
    server.shutdown();
    assert!(!server.apply_local_control(WorldControl::default()));
}

#[test]
fn admitted_recovery_pause_survives_disconnect_and_owns_one_boundary() {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let mut server = Server::with_zenoh_config(
        simulation(), "default", gz_sim_server::ServerOptions {
            partition: format!("gzweb-pause-{}", listener.local_addr().unwrap().port()),
            ..Default::default()
        }, zenoh::Config::default(), std::num::NonZeroUsize::new(8).unwrap(),
    ).unwrap();
    let mut bridge = Bridge::with_listener(server.simulation(), "default", listener).unwrap();
    let (sender, commands) = mpsc::sync_channel(8);
    bridge.commands = commands;
    let send = |action, admission, alive| {
        let (reply, receiver) = mpsc::sync_channel(1);
        assert!(sender.send(Queued::Control(Pending {
            admission, trace: None, control: Control {
                version: 1, kind: "control".into(), request_id: 1, action,
            }, alive: Arc::new(AtomicBool::new(alive)), reply,
        })).is_ok());
        receiver
    };
    drop(send(Action::Pause, Admission::RecoveryPause, false));
    let replacement = send(Action::Resume, Admission::AppliedResume, true);
    let controls = bridge.controls(&mut server);
    assert_eq!(controls.len(), 1);
    assert!(server.simulation().is_paused());
    assert!(matches!(replacement.try_recv(), Err(mpsc::TryRecvError::Empty)));
    bridge.publish(server.simulation(), "default", controls).unwrap();
    let mut controls = bridge.controls(&mut server);
    assert_eq!(controls.len(), 1);
    bridge.publish_applied_resume(server.simulation(), "default", &mut controls).unwrap();
    let ack: Value = serde_json::from_str(&replacement.try_recv().unwrap()).unwrap();
    assert!(controls.is_empty());
    assert_eq!(ack["ok"], true);
    assert_eq!(ack["paused"], false);
    assert_eq!(ack["nanoseconds"], 0);
    assert_eq!(server.simulation().elapsed(), Duration::ZERO);
}

#[test]
fn paused_uncredited_viewer_gets_one_scene_until_visible_state_changes() {
    use tungstenite::{Message, WebSocket, stream::MaybeTlsStream};
    let mut sim = simulation();
    sim.set_paused(true);
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let address = listener.local_addr().unwrap();
    let mut bridge = Bridge::with_listener(&sim, "default", listener).unwrap();
    let (mut client, _) = tungstenite::connect(format!("ws://{address}/")).unwrap();
    let MaybeTlsStream::Plain(stream) = client.get_mut() else {
        unreachable!()
    };
    stream
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    while bridge.connected.load(Ordering::Acquire) == 0 {
        assert!(Instant::now() < deadline);
        thread::sleep(Duration::from_millis(2));
    }
    let read_scene = |ws: &mut WebSocket<_>| -> Value {
        serde_json::from_str(ws.read().unwrap().to_text().unwrap()).unwrap()
    };
    bridge.publish(&sim, "default", vec![]).unwrap();
    let initial = read_scene(&mut client);
    for _ in 0..100 {
        bridge.published = Instant::now() - Duration::from_millis(40);
        bridge.publish(&sim, "default", vec![]).unwrap();
    }
    client.send(Message::Ping(vec![1].into())).unwrap();
    assert_eq!(client.read().unwrap(), Message::Pong(vec![1].into()));
    bridge.last_network_publication = Instant::now() - Duration::from_secs(2);
    bridge.published = Instant::now() - Duration::from_millis(40);
    bridge.publish(&sim, "default", vec![]).unwrap();
    let heartbeat = read_scene(&mut client);
    assert_eq!(initial["visuals"], heartbeat["visuals"]);
    assert_eq!(initial["nanoseconds"], heartbeat["nanoseconds"]);
    let visual = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<EntityKind>() == Some(&EntityKind::Visual))
        .unwrap()
        .id();
    sim.world_mut()
        .get_mut::<WorldPose>(visual)
        .unwrap()
        .translation[0] += 0.25;
    bridge.published = Instant::now() - Duration::from_millis(40);
    bridge.publish(&sim, "default", vec![]).unwrap();
    let changed = read_scene(&mut client);
    assert_eq!(changed["nanoseconds"], initial["nanoseconds"]);
    assert_ne!(changed["visuals"], initial["visuals"]);
    assert!(changed["sequence"].as_u64() > initial["sequence"].as_u64());
}
