// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::{Value, json};
use std::sync::{Arc, atomic::AtomicBool};
use tungstenite::client::IntoClientRequest;

fn frame() -> String {
    json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":3,
        "seconds":1,"nanoseconds":150_000_000,"sequence":17,"paused":false,"visuals":[]})
    .to_string()
}

struct Server {
    port: u16,
    stop: Arc<AtomicBool>,
    done: Option<thread::JoinHandle<io::Result<bool>>>,
}

impl Server {
    fn start(eligible: bool) -> Self {
        let mut server = ParkedFence::bind(
            0,
            &frame(),
            3,
            1150,
            eligible,
            "worker transport unavailable",
        )
        .unwrap();
        let port = server.local_port().unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let terminate = Arc::clone(&stop);
        let done = thread::spawn(move || {
            let admitted = server.wait(|| terminate.load(Ordering::Acquire))?;
            if admitted {
                assert!(server.wait(|| true).is_err());
            }
            Ok(admitted)
        });
        Self {
            port,
            stop,
            done: Some(done),
        }
    }

    fn connect(&self, query: &str) -> WebSocket<TcpStream> {
        let s = TcpStream::connect((Ipv4Addr::LOCALHOST, self.port)).unwrap();
        s.set_read_timeout(Some(Duration::from_secs(4))).unwrap();
        tungstenite::client(format!("ws://127.0.0.1:{}/?{query}", self.port), s)
            .unwrap()
            .0
    }

    fn operator(&self) -> (WebSocket<TcpStream>, Value) {
        let mut socket = self.connect("scan=1&recovery=1&operator=1");
        let status = read(&mut socket);
        assert_eq!(status["kind"], "operator_fence");
        assert_eq!(status["version"], 1);
        assert_eq!(status["epoch"], 3);
        assert_eq!(status["tick"], 1150);
        assert_eq!(status["token"].as_str().unwrap().len(), 32);
        assert_eq!(
            read(&mut socket),
            serde_json::from_str::<Value>(&frame()).unwrap()
        );
        (socket, status)
    }

    fn finish(mut self, expected: bool) {
        if !expected {
            self.stop.store(true, Ordering::Release);
        }
        assert_eq!(self.done.take().unwrap().join().unwrap().unwrap(), expected);
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(server) = self.done.take() {
            let _ = server.join();
        }
    }
}

fn read(socket: &mut WebSocket<TcpStream>) -> Value {
    serde_json::from_str(socket.read().unwrap().to_text().unwrap()).unwrap()
}
fn request(status: &Value, id: u64) -> Value {
    json!({"version":1,"kind":"recover_workers","request_id":id,"token":status["token"],"epoch":status["epoch"],"tick":status["tick"]})
}
fn post(socket: &mut WebSocket<TcpStream>, request: Value) -> Value {
    socket
        .send(Message::Text(request.to_string().into()))
        .unwrap();
    read(socket)
}

#[test]
fn admission_checks_the_published_clock_even_above_one_second() {
    assert!(ParkedFence::bind(0, &frame(), 3, 1150, true, "verified").is_ok());
    assert!(ParkedFence::bind(0, &frame(), 4, 1150, true, "verified").is_err());
    assert!(ParkedFence::bind(0, &frame(), 3, 1151, true, "verified").is_err());
    assert!(ParkedFence::bind(0, &frame(), 3, 1150, true, "").is_err());
    assert!(ParkedFence::bind(0, "{}", 3, 1150, true, "verified").is_err());
}

#[test]
fn stale_and_replayed_operator_commands_cannot_admit() {
    let first = Server::start(true);
    let (_peer, first_status) = first.operator();
    first.finish(false);
    let second = Server::start(true);
    let (mut peer, second_status) = second.operator();
    assert_ne!(first_status["token"], second_status["token"]);
    let stale = post(&mut peer, request(&first_status, 1));
    assert_eq!(stale["kind"], "operator_reply");
    assert_eq!(stale["accepted"], false);
    assert_eq!(
        post(&mut peer, request(&second_status, 1))["accepted"],
        false
    );
    let mut wrong_cut = request(&second_status, 2);
    wrong_cut["tick"] = json!(1151);
    assert_eq!(post(&mut peer, wrong_cut)["accepted"], false);
    let reply = post(&mut peer, request(&second_status, 3));
    assert_eq!(
        reply,
        json!({"version":1,"kind":"operator_reply","request_id":3,"accepted":true,"error":null})
    );
    second.finish(true);
}

#[test]
fn parked_legacy_controls_preserve_clock_and_cannot_start_workers() {
    let server = Server::start(true);
    let mut legacy = server.connect("recovery=1");
    assert_eq!(read(&mut legacy)["kind"], "scene");
    for (id, action) in ["resume", "reset", "pause"].iter().enumerate() {
        let reply = post(
            &mut legacy,
            json!({"version":1,"kind":"control","request_id":id+1,"action":action}),
        );
        assert_eq!(reply["ok"], false);
        assert_eq!(reply["paused"], false);
        assert_eq!(reply["seconds"], 1);
        assert_eq!(reply["nanoseconds"], 150_000_000);
    }
    let (mut operator, status) = server.operator();
    assert_eq!(post(&mut operator, request(&status, 1))["accepted"], true);
    server.finish(true);
}

#[test]
fn inspection_only_remains_available_across_multiple_operator_connections() {
    let server = Server::start(false);
    let (mut a, s1) = server.operator();
    let (mut b, s2) = server.operator();
    assert_eq!(s1, s2);
    assert_eq!(s1["eligible"], false);
    for peer in [&mut a, &mut b] {
        let denied = post(peer, request(&s1, 1));
        assert_eq!(denied["accepted"], false);
        assert_eq!(denied["error"], "This checkpoint requires inspection");
    }
    server.finish(false);
}

#[test]
fn local_origin_negotiation_and_strict_schema_precede_any_admission() {
    let server = Server::start(true);
    for (path, origin) in [
        ("/?operator=1", "https://elsewhere.invalid"),
        ("/bad?operator=1", "http://localhost:8082"),
        ("/?operator=0", "http://localhost:8082"),
    ] {
        let mut req = format!("ws://127.0.0.1:{}{path}", server.port)
            .into_client_request()
            .unwrap();
        req.headers_mut().insert("origin", origin.parse().unwrap());
        let stream = TcpStream::connect((Ipv4Addr::LOCALHOST, server.port)).unwrap();
        assert!(tungstenite::client(req, stream).is_err());
    }
    let (mut unknown, s) = server.operator();
    let mut extra = request(&s, 1);
    extra["session"] = json!("/unrelated");
    unknown
        .send(Message::Text(extra.to_string().into()))
        .unwrap();
    assert!(unknown.read().is_err());
    let mut plain = server.connect("");
    assert_eq!(read(&mut plain)["kind"], "scene");
    plain
        .send(Message::Text(request(&s, 2).to_string().into()))
        .unwrap();
    assert!(plain.read().is_err());
    server.finish(false);
}

#[test]
fn slow_or_disconnected_viewer_cannot_prevent_shutdown() {
    let server = Server::start(true);
    let _incomplete = TcpStream::connect((Ipv4Addr::LOCALHOST, server.port)).unwrap();
    thread::sleep(Duration::from_millis(20));
    let begin = Instant::now();
    server.finish(false);
    assert!(begin.elapsed() < Duration::from_secs(2));
}

#[test]
fn contested_requests_admit_only_once() {
    let server = Server::start(true);
    let (a, s) = server.operator();
    let (b, again) = server.operator();
    assert_eq!(s, again);
    let start = Arc::new(std::sync::Barrier::new(2));
    let peers = [a, b].map(|mut peer| {
        let begin = start.clone();
        let message = request(&s, 1).to_string();
        thread::spawn(move || {
            begin.wait();
            peer.send(Message::Text(message.into())).is_ok()
                && peer
                    .read()
                    .ok()
                    .and_then(|m| serde_json::from_str::<Value>(m.to_text().ok()?).ok())
                    .is_some_and(|v| v["kind"] == "operator_reply" && v["accepted"] == true)
        })
    });
    assert_eq!(
        peers
            .into_iter()
            .map(|p| p.join().unwrap() as usize)
            .sum::<usize>(),
        1
    );
    server.finish(true);
}

#[test]
fn negotiated_heartbeat_preserves_the_same_conditional_offer() {
    let server = Server::start(true);
    let (mut a, status) = server.operator();
    assert_eq!(read(&mut a), status);
    server.finish(false);
}
