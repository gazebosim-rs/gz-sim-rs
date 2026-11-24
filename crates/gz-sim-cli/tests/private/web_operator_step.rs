// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Checked steps use actual TCP, the ordinary server pause/step control and its committed scene.
use super::*;
use serde_json::{Value, json};
use std::net::{SocketAddr, TcpStream};
use tungstenite::{Message, WebSocket};

fn connect(addr: SocketAddr, query: &str) -> WebSocket<TcpStream> {
    let until = Instant::now() + Duration::from_secs(5);
    loop {
        let s = TcpStream::connect_timeout(&addr, Duration::from_secs(2)).unwrap();
        s.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
        if let Ok((peer, _)) = tungstenite::client(format!("ws://{addr}/?{query}"), s) {
            return peer;
        }
        assert!(Instant::now() < until, "viewer handshake exceeded deadline");
        thread::sleep(Duration::from_millis(5));
    }
}
fn read(ws: &mut WebSocket<TcpStream>, kind: &str) -> Value {
    let until = Instant::now() + Duration::from_secs(5);
    while Instant::now() < until {
        if let Message::Text(text) = ws.read().unwrap() {
            let value: Value = serde_json::from_str(&text).unwrap();
            if value["kind"] == kind {
                return value;
            }
        }
    }
    panic!("missing {kind}");
}
fn step(ws: &mut WebSocket<TcpStream>, id: u64, epoch: u64, tick: u64) {
    ws.send(Message::Text(
        json!({"version":1,"kind":"operator_step","request_id":id,"epoch":epoch,"tick":tick})
            .to_string()
            .into(),
    ))
    .unwrap();
}
fn check_ack(ws: &mut WebSocket<TcpStream>, id: u64, ok: bool, epoch: u64, tick: u64) {
    let ack = read(ws, "ack");
    let elapsed = Duration::from_millis(tick);
    assert_eq!(ack["request_id"], id);
    assert_eq!(ack["epoch"], epoch);
    assert_eq!(ack["paused"], true);
    assert_eq!(ack["ok"], ok);
    assert_eq!(ack["seconds"], elapsed.as_secs());
    assert_eq!(ack["nanoseconds"], elapsed.subsec_nanos());
}
fn with_live(step_duration: Duration, lifecycle: bool, client: impl FnOnce(SocketAddr)) {
    with_boundary_control(step_duration, lifecycle, |_| {}, client);
}

fn with_boundary_control(
    step_duration: Duration,
    lifecycle: bool,
    mut after_step: impl FnMut(&mut Server) + Send + 'static,
    client: impl FnOnce(SocketAddr),
) {
    const XML: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../examples/m10_world_lifecycle.sdf"
    ));
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let addr = listener.local_addr().unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let terminate = stop.clone();
    let (ready, prepared) = mpsc::sync_channel(1);
    let owner = thread::spawn(move || {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../examples/m10_world_lifecycle.sdf")
            .canonicalize()
            .unwrap();
        let uri = url::Url::from_file_path(path).unwrap().to_string();
        let sdf = sdformat::load_str(XML, &uri, &sdformat::NoResources).unwrap();
        let mut sim =
            gz_sim_scene::load_world(XML, &uri, "default", step_duration, &sdformat::NoResources)
                .unwrap();
        sim.set_paused(true);
        let mut config = zenoh::Config::default();
        for (key, value) in [
            ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
            ("scouting/multicast/enabled", "false"),
            ("scouting/gossip/enabled", "false"),
        ] {
            config.insert_json5(key, value).unwrap();
        }
        let options = gz_sim_server::ServerOptions {
            partition: format!("operator-step-{}-{}", std::process::id(), addr.port()),
            ..Default::default()
        };
        let mut server = Server::with_zenoh_config(
            sim,
            "default",
            options,
            config,
            std::num::NonZeroUsize::new(4).unwrap(),
        )
        .unwrap();
        let mut bridge = Bridge::with_options(
            server.simulation(),
            "default",
            listener,
            lifecycle.then_some(&sdf),
        )
        .unwrap();
        ready.send(()).unwrap();
        let deadline = Instant::now() + Duration::from_secs(15);
        while !terminate.load(Ordering::Acquire) && Instant::now() < deadline {
            let controls = bridge.controls(&mut server);
            server.update().unwrap();
            if controls.iter().any(|(request, admitted, _)| {
                *admitted && matches!(request.admission, Admission::CheckedStep { .. })
            }) {
                after_step(&mut server);
            }
            bridge
                .publish(server.simulation(), "default", controls)
                .unwrap();
            thread::sleep(Duration::from_millis(1));
        }
        drop(bridge);
        server.shutdown();
    });
    prepared.recv_timeout(Duration::from_secs(10)).unwrap();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| client(addr)));
    stop.store(true, Ordering::Release);
    owner.join().unwrap();
    if let Err(err) = result {
        std::panic::resume_unwind(err);
    }
}

#[test]
fn checked_step_commits_once_and_reconnect_replay_is_refused_at_current_cut() {
    with_live(Duration::from_millis(1), false, |addr| {
        let mut ws = connect(addr, "operator=1");
        let initial = read(&mut ws, "scene");
        assert_eq!(initial["nanoseconds"], 0);
        assert_eq!(initial["paused"], true);
        step(&mut ws, 1, 0, 0);
        check_ack(&mut ws, 1, true, 0, 1);
        ws.close(None).unwrap();
        drop(ws);
        let mut fresh = connect(addr, "operator=1");
        let scene = read(&mut fresh, "scene");
        assert_eq!(scene["nanoseconds"], 1_000_000);
        assert_eq!(scene["paused"], true);
        step(&mut fresh, 1, 0, 0);
        check_ack(&mut fresh, 1, false, 0, 1);
        step(&mut fresh, 2, 1, 1);
        check_ack(&mut fresh, 2, false, 0, 1);
        step(&mut fresh, 3, 0, 1);
        check_ack(&mut fresh, 3, true, 0, 2);
    });
}

#[test]
fn checked_step_refuses_a_world_with_a_different_interval() {
    with_live(Duration::from_millis(5), false, |addr| {
        let mut ws = connect(addr, "operator=1");
        read(&mut ws, "scene");
        step(&mut ws, 1, 0, 0);
        check_ack(&mut ws, 1, false, 0, 0);
    });
}

#[test]
fn plain_and_lifecycle_connections_cannot_admit_a_step() {
    for (lifecycle, query) in [(false, ""), (true, "operator=1&lifecycle=1")] {
        with_live(Duration::from_millis(1), lifecycle, |addr| {
            let mut ws = connect(addr, query);
            read(&mut ws, "scene");
            if lifecycle {
                read(&mut ws, "lifecycle_catalog");
            }
            step(&mut ws, 1, 0, 0);
            assert!(matches!(ws.read(), Err(_) | Ok(Message::Close(_))));
            let mut fresh = connect(addr, query);
            let first = read(&mut fresh, "scene");
            assert_eq!(first["nanoseconds"], 0);
            assert_eq!(first["paused"], true);
        });
    }
}

#[test]
fn terminal_cut_refuses_a_negotiated_step_and_remains_servable() {
    let scene=json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"seconds":0,"nanoseconds":0,"sequence":1,"paused":true,"visuals":[]}).to_string();
    let mut parked = parked::ParkedFence::bind(0, &scene, 0, 0, true, "eligible").unwrap();
    let addr = SocketAddr::from((Ipv4Addr::LOCALHOST, parked.local_port().unwrap()));
    let stop = Arc::new(AtomicBool::new(false));
    let terminate = stop.clone();
    let owner = thread::spawn(move || parked.wait(|| terminate.load(Ordering::Acquire)).unwrap());
    let mut ws = connect(addr, "operator=1");
    read(&mut ws, "operator_fence");
    read(&mut ws, "scene");
    step(&mut ws, 1, 0, 0);
    check_ack(&mut ws, 1, false, 0, 0);
    stop.store(true, Ordering::Release);
    assert!(!owner.join().unwrap());
}

#[test]
fn replay_refuses_operator_negotiation_at_the_real_handshake() {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let addr = listener.local_addr().unwrap();
    let owner = thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        socket::replay_handshake(stream, &AtomicBool::new(false)).is_none()
    });
    let s = TcpStream::connect(addr).unwrap();
    s.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
    assert!(
        matches!(tungstenite::client(format!("ws://{addr}/?lifecycle=1&lifecycle_credit=1&replay=1&operator=1"),s),Err(tungstenite::HandshakeError::Failure(tungstenite::Error::Http(r))) if r.status().as_u16()==400)
    );
    assert!(owner.join().unwrap());
}

#[test]
fn intervening_resume_reports_that_the_checked_interval_already_committed() {
    // Server::tick applies external WorldControl messages after physics and before
    // Bridge::publish. Inject the same control at that boundary deterministically.
    with_boundary_control(
        Duration::from_millis(1),
        false,
        |server| {
            assert_eq!(server.simulation().elapsed(), Duration::from_millis(1));
            assert!(server.simulation().is_paused());
            assert!(server.apply_local_control(WorldControl::default()));
        },
        |addr| {
            let mut ws = connect(addr, "operator=1");
            read(&mut ws, "scene");
            step(&mut ws, 1, 0, 0);
            let ack = read(&mut ws, "ack");
            assert_eq!(ack["request_id"], 1);
            assert_eq!(ack["ok"], false);
            assert_eq!(ack["epoch"], 0);
            assert_eq!(ack["seconds"], 0);
            assert_eq!(ack["nanoseconds"], 1_000_000);
            assert_eq!(ack["paused"], false);
            assert_eq!(
                ack["error"],
                "Requested interval committed, but another control changed the run mode."
            );
        },
    );
}
