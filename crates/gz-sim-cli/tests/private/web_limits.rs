// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Include as a child module of web::tests; no connection uses the live viewer port.
use super::*;
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use tungstenite::{Message, WebSocket, client::IntoClientRequest};

#[test]
fn ping_and_abandoned_cpu_reply_preserve_transport_lifecycle() {
    let sim = simulation();
    let (mut bridge, address) = limits_bridge(&sim);
    let mut client = limits_client(address, "/", None).unwrap();
    limits_wait("client was not admitted", || {
        bridge.connected.load(Ordering::Acquire) != 0
    });
    bridge.publish(&sim, "default", Vec::new()).unwrap();
    limits_scene(&mut client);
    client.send(Message::Ping(vec![1, 2, 3].into())).unwrap();
    assert_eq!(client.read().unwrap(), Message::Pong(vec![1, 2, 3].into()));
    client
        .send(Message::Text(
            json!({"version":1,"kind":"control","request_id":1,"action":"resume"})
                .to_string()
                .into(),
        ))
        .unwrap();
    let mut pending = None;
    limits_wait("control did not reach CPU ingress", || {
        pending = bridge.commands.try_recv().ok();
        pending.is_some()
    });
    // Losing the CPU reply owner is terminal, rather than a fabricated ACK.
    drop(pending);
    limits_closed(&mut client);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    limits_wait("abandoned reply did not retire peer", || {
        bridge.connected.load(Ordering::Acquire) == 0
    });
    limits_restored(&mut bridge, &sim, address);
}

#[test]
fn stale_network_controls_cannot_exceed_the_bounded_ingress_queue() {
    let sim = simulation();
    let (mut bridge, address) = limits_bridge(&sim);
    for _ in 0..8 {
        let mut client = limits_client(address, "/", None).unwrap();
        limits_wait("client was not admitted", || {
            bridge.connected.load(Ordering::Acquire) != 0
        });
        client
            .send(Message::Text(
                json!({"version":1,"kind":"control","request_id":1,"action":"resume"})
                    .to_string()
                    .into(),
            ))
            .unwrap();
        drop(client);
        limits_wait("closed client retained its connection", || {
            bridge.connected.load(Ordering::Acquire) == 0
        });
    }
    let mut overflow = limits_client(address, "/", None).unwrap();
    limits_wait("overflow client was not admitted", || {
        bridge.connected.load(Ordering::Acquire) != 0
    });
    overflow
        .send(Message::Text(
            json!({"version":1,"kind":"control","request_id":1,"action":"resume"})
                .to_string()
                .into(),
        ))
        .unwrap();
    limits_closed(&mut overflow);
    limits_wait("overflow retained its connection", || {
        bridge.connected.load(Ordering::Acquire) == 0
    });
    let queued: Vec<_> = bridge.commands.try_iter().collect();
    assert_eq!(queued.len(), 8);
    assert!(
        queued
            .iter()
            .all(|pending| !pending.alive().load(Ordering::Acquire))
    );
    assert_eq!(sim.elapsed(), Duration::ZERO);
    drop(queued);
    limits_restored(&mut bridge, &sim, address);
}

#[test]
fn stopped_network_worker_fails_publication_and_releases_listener() {
    let sim = simulation();
    let (mut bridge, address) = limits_bridge(&sim);
    bridge.stop.store(true, Ordering::Release);
    limits_wait("worker did not stop", || {
        bridge.worker.as_ref().unwrap().is_finished()
    });
    let error = bridge.publish(&sim, "default", Vec::new()).unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::BrokenPipe);
    assert!(error.to_string().contains("worker stopped"));
    assert_eq!(sim.elapsed(), Duration::ZERO);
    drop(bridge);
    let listener = TcpListener::bind(address).unwrap();
    drop(listener);
}

fn limits_bridge(sim: &Simulation) -> (Bridge, SocketAddr) {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let address = listener.local_addr().unwrap();
    (
        Bridge::with_listener(sim, "default", listener).unwrap(),
        address,
    )
}

fn limits_wait(message: &str, mut ready: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(2);
    while !ready() {
        assert!(Instant::now() < deadline, "{message}");
        thread::sleep(Duration::from_millis(2));
    }
}

fn limits_client(
    address: SocketAddr,
    path: &str,
    origin: Option<&str>,
) -> Result<WebSocket<TcpStream>, tungstenite::Error> {
    let stream = TcpStream::connect_timeout(&address, Duration::from_secs(2))?;
    stream.set_read_timeout(Some(Duration::from_millis(500)))?;
    stream.set_write_timeout(Some(Duration::from_millis(500)))?;
    let mut request = format!("ws://{address}{path}").into_client_request()?;
    if let Some(origin) = origin {
        request
            .headers_mut()
            .insert("Origin", origin.parse().unwrap());
    }
    match tungstenite::client(request, stream) {
        Ok((socket, _)) => Ok(socket),
        Err(tungstenite::HandshakeError::Failure(error)) => Err(error),
        Err(tungstenite::HandshakeError::Interrupted(_)) => {
            panic!("a blocking test handshake unexpectedly suspended")
        }
    }
}

fn limits_closed(socket: &mut WebSocket<TcpStream>) {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        match socket.read() {
            Ok(Message::Close(_))
            | Err(tungstenite::Error::ConnectionClosed | tungstenite::Error::AlreadyClosed)
            | Err(tungstenite::Error::Protocol(
                tungstenite::error::ProtocolError::ResetWithoutClosingHandshake,
            )) => return,
            Err(tungstenite::Error::Io(error))
                if matches!(
                    error.kind(),
                    io::ErrorKind::ConnectionReset
                        | io::ErrorKind::ConnectionAborted
                        | io::ErrorKind::UnexpectedEof
                        | io::ErrorKind::BrokenPipe
                ) =>
            {
                return;
            }
            Err(tungstenite::Error::Io(error))
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                assert!(
                    Instant::now() < deadline,
                    "rejected client remained connected"
                );
            }
            other => panic!("expected an actual closed transport, got {other:?}"),
        }
    }
}

fn limits_scene(socket: &mut WebSocket<TcpStream>) -> Value {
    match socket.read().unwrap() {
        Message::Text(text) => {
            let scene: Value = serde_json::from_str(&text).unwrap();
            assert_eq!(scene["kind"], "scene");
            scene
        }
        other => panic!("expected the first complete scene, got {other:?}"),
    }
}

fn limits_restored(bridge: &mut Bridge, sim: &Simulation, address: SocketAddr) {
    let mut client = limits_client(address, "/", Some("http://127.0.0.1:8081")).unwrap();
    limits_wait("replacement client was not admitted", || {
        bridge.connected.load(Ordering::Acquire) != 0
    });
    bridge.publish(sim, "default", Vec::new()).unwrap();
    let scene = limits_scene(&mut client);
    assert_eq!(scene["visuals"], frame(sim, 0)["visuals"]);
    assert_eq!(scene["epoch"], sim.epoch());
    assert_eq!(scene["seconds"], sim.elapsed().as_secs());
    assert_eq!(scene["nanoseconds"], sim.elapsed().subsec_nanos());
    drop(client);
    limits_wait("replacement client did not retire", || {
        bridge.connected.load(Ordering::Acquire) == 0
    });
}

#[test]
fn foreign_origins_and_paths_receive_real_403_then_allow_a_fresh_client() {
    let sim = simulation();
    let before = frame(&sim, 0);
    let (mut bridge, address) = limits_bridge(&sim);
    for (path, origin) in [
        ("/", "http://example.invalid:8080"),
        ("/other", "http://127.0.0.1:8080"),
        ("/", "http://127.0.0.1:8082"),
    ] {
        match limits_client(address, path, Some(origin)) {
            Err(tungstenite::Error::Http(response)) => {
                assert_eq!(response.status(), tungstenite::http::StatusCode::FORBIDDEN);
            }
            other => panic!("expected HTTP 403, got {other:?}"),
        }
        assert_eq!(bridge.connected.load(Ordering::Acquire), 0);
        assert!(matches!(
            bridge.commands.try_recv(),
            Err(mpsc::TryRecvError::Empty)
        ));
        assert_eq!(frame(&sim, 0), before);
        limits_restored(&mut bridge, &sim, address);
    }
}

#[test]
fn malformed_binary_and_oversized_controls_close_without_queueing_commands() {
    let sim = simulation();
    let before = frame(&sim, 0);
    let (mut bridge, address) = limits_bridge(&sim);
    for message in [
        Message::Text("{".into()),
        Message::Binary(vec![1, 2, 3].into()),
        Message::Text("x".repeat(4097).into()),
    ] {
        let mut client = limits_client(address, "/", None).unwrap();
        limits_wait("test client did not finish its handshake", || {
            bridge.connected.load(Ordering::Acquire) != 0
        });
        client.send(message).unwrap();
        limits_closed(&mut client);
        limits_wait("rejected client did not retire", || {
            bridge.connected.load(Ordering::Acquire) == 0
        });
        assert!(matches!(
            bridge.commands.try_recv(),
            Err(mpsc::TryRecvError::Empty)
        ));
        assert_eq!(frame(&sim, 0), before);
        limits_restored(&mut bridge, &sim, address);
    }
}

#[test]
fn duplicate_inflight_control_closes_and_its_dead_pending_resume_has_no_effect() {
    let sim = simulation();
    let (mut bridge, address) = limits_bridge(&sim);
    let mut server = Server::with_zenoh_config(
        sim,
        "default",
        gz_sim_server::ServerOptions {
            partition: format!("gzweb-dead-{}-{}", std::process::id(), address.port()),
            ..Default::default()
        },
        zenoh::Config::default(),
        std::num::NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    let before = frame(server.simulation(), 0);
    let mut client = limits_client(address, "/", None).unwrap();
    let command =
        json!({"version":1,"kind":"control","request_id":1,"action":"resume"}).to_string();
    client.send(Message::Text(command.clone().into())).unwrap();
    // This is the actual network-produced Pending, not a fabricated decoded input.
    let pending = bridge
        .commands
        .recv_timeout(Duration::from_secs(2))
        .unwrap();
    assert!(pending.alive().load(Ordering::Acquire));
    let Queued::Control(control_pending) = &pending else {
        panic!("control expected");
    };
    assert_eq!(control_pending.control.request_id, 1);
    assert!(matches!(control_pending.control.action, Action::Resume));
    client.send(Message::Text(command.into())).unwrap();
    limits_closed(&mut client);
    limits_wait("duplicate client did not mark its command dead", || {
        !pending.alive().load(Ordering::Acquire) && bridge.connected.load(Ordering::Acquire) == 0
    });
    // Put that same retained object back at the application handoff. Only the
    // test-owned channel changes; its control/alive/reply fields are untouched.
    let (sender, receiver) = mpsc::sync_channel(1);
    assert!(sender.send(pending).is_ok());
    bridge.commands = receiver;
    assert!(bridge.controls(&mut server).is_empty());
    assert_eq!(frame(server.simulation(), 0), before);
    assert!(server.simulation().is_paused());
    assert_eq!(server.pending_steps(), 0);
    assert!(!server.io_preparation_required());
    limits_restored(&mut bridge, server.simulation(), address);
    server.shutdown();
}

#[test]
fn second_peer_is_refused_without_displacing_the_original_or_stale_reconnect() {
    let sim = simulation();
    let before = frame(&sim, 0);
    let (mut bridge, address) = limits_bridge(&sim);
    let mut first = limits_client(address, "/", None).unwrap();
    limits_wait("first client was not admitted", || {
        bridge.connected.load(Ordering::Acquire) != 0
    });
    let generation = bridge.connected.load(Ordering::Acquire);
    match limits_client(address, "/", None) {
        Err(tungstenite::Error::Io(error))
            if matches!(
                error.kind(),
                io::ErrorKind::ConnectionReset
                    | io::ErrorKind::ConnectionAborted
                    | io::ErrorKind::UnexpectedEof
                    | io::ErrorKind::BrokenPipe
            ) => {}
        Err(tungstenite::Error::Protocol(
            tungstenite::error::ProtocolError::HandshakeIncomplete,
        )) => {}
        other => panic!("second peer was not explicitly refused: {other:?}"),
    }
    assert_eq!(bridge.connected.load(Ordering::Acquire), generation);
    bridge.publish(&sim, "default", Vec::new()).unwrap();
    assert_eq!(limits_scene(&mut first)["visuals"], before["visuals"]);
    assert_eq!(frame(&sim, 0), before);
    drop(first);
    limits_wait("original peer did not retire", || {
        bridge.connected.load(Ordering::Acquire) == 0
    });
    limits_restored(&mut bridge, &sim, address);
}

#[test]
fn incomplete_handshake_times_out_then_drop_joins_while_another_client_stays_open() {
    let sim = simulation();
    let (mut bridge, address) = limits_bridge(&sim);
    let partial = b"GET / HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\n";
    let mut slow = TcpStream::connect_timeout(&address, Duration::from_secs(2)).unwrap();
    slow.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
    slow.set_write_timeout(Some(Duration::from_secs(2)))
        .unwrap();
    slow.write_all(partial).unwrap();
    let mut byte = [0; 1];
    match slow.read(&mut byte) {
        Ok(0) => {}
        Err(error)
            if matches!(
                error.kind(),
                io::ErrorKind::ConnectionReset | io::ErrorKind::ConnectionAborted
            ) => {}
        other => panic!("incomplete handshake did not close within the deadline: {other:?}"),
    }
    assert_eq!(bridge.connected.load(Ordering::Acquire), 0);
    assert!(matches!(
        bridge.commands.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));
    limits_restored(&mut bridge, &sim, address);
    let mut still_open = TcpStream::connect_timeout(&address, Duration::from_secs(2)).unwrap();
    still_open
        .set_write_timeout(Some(Duration::from_secs(2)))
        .unwrap();
    still_open.write_all(partial).unwrap();
    // Give the listener a scheduling opportunity; the client remains open while
    // Drop must signal/join even if the handshake is currently interrupted.
    thread::sleep(Duration::from_millis(20));
    let (finished, result) = mpsc::sync_channel(1);
    let owner = thread::spawn(move || {
        drop(bridge);
        finished.send(()).unwrap();
    });
    result
        .recv_timeout(Duration::from_secs(2))
        .expect("Bridge::drop did not join its worker");
    owner.join().unwrap();
    drop(still_open);
}
