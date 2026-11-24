// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::web::recording::writer::Recorder;
use serde_json::{Value, json};
use std::sync::Arc;

struct Fixture {
    stop: Arc<AtomicBool>,
    worker: Option<thread::JoinHandle<io::Result<()>>>,
    port: u16,
    _directory: tempfile::TempDir,
}
impl Fixture {
    fn new() -> Self {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("capture");
        let mut recorder = Recorder::start(&path, None).unwrap();
        for sequence in 0..3 {
            let scene=json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"seconds":0,
                "nanoseconds":0,"sequence":sequence,"paused":true,"visuals":[]}).to_string();
            let catalog=json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,"templates":[],"models":[]}).to_string();
            recorder.record(scene, catalog, vec![]).unwrap();
        }
        recorder.finish().unwrap();
        let mut player = Player::open(&path, Instant::now()).unwrap();
        let listener = TcpListener::bind((std::net::Ipv4Addr::LOCALHOST, 0)).unwrap();
        let port = listener.local_addr().unwrap().port();
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        let worker = thread::spawn(move || serve(listener, &worker_stop, &mut player));
        Self {
            stop,
            worker: Some(worker),
            port,
            _directory: directory,
        }
    }
    fn connect(&self) -> WebSocket<TcpStream> {
        let stream = TcpStream::connect((std::net::Ipv4Addr::LOCALHOST, self.port)).unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        stream
            .set_write_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        tungstenite::client(
            format!(
                "ws://127.0.0.1:{}/?replay=1&lifecycle=1&lifecycle_credit=1",
                self.port
            ),
            stream,
        )
        .unwrap()
        .0
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        self.worker.take().unwrap().join().unwrap().unwrap();
    }
}
fn read(socket: &mut WebSocket<TcpStream>) -> Value {
    let Message::Text(text) = socket.read().unwrap() else {
        panic!("expected replay text tuple")
    };
    serde_json::from_str(&text).unwrap()
}
fn tuple(socket: &mut WebSocket<TcpStream>, last: &str) -> [Value; 4] {
    let values = [read(socket), read(socket), read(socket), read(socket)];
    for (value, kind) in values
        .iter()
        .zip(["scene", "lifecycle_catalog", "replay_status", last])
    {
        assert_eq!(value["kind"], kind);
    }
    assert_eq!(values[0]["sequence"], values[1]["scene_sequence"]);
    assert_eq!(values[0]["sequence"], values[2]["scene_sequence"]);
    assert_eq!(values[0]["epoch"], values[2]["epoch"]);
    assert_eq!(values[0]["seconds"], values[2]["recorded_seconds"]);
    values
}
fn send(socket: &mut WebSocket<TcpStream>, value: Value) {
    socket
        .send(Message::Text(value.to_string().into()))
        .unwrap();
}

#[test]
fn priority_reset_does_not_steal_old_credit_or_advance_restarted_cursor() {
    let fixture = Fixture::new();
    let mut socket = fixture.connect();
    let initial = tuple(&mut socket, "publication_credit");
    assert_eq!(initial[2]["frame"], 1);
    let old = initial[3]["token"].as_u64().unwrap();
    send(
        &mut socket,
        json!({"version":1,"kind":"control","request_id":1,"action":"reset"}),
    );
    let reset = tuple(&mut socket, "ack");
    assert_eq!(reset[3]["ok"], true);
    assert_eq!(reset[0]["epoch"], 1);
    send(
        &mut socket,
        json!({"version":1,"kind":"publication_ready","token":old}),
    );
    let restarted = tuple(&mut socket, "publication_credit");
    assert_eq!(restarted[2]["frame"], 1);
    assert!(restarted[3]["token"].as_u64().unwrap() > old);
    send(
        &mut socket,
        json!({"version":1,"kind":"control","request_id":2,"action":"resume"}),
    );
    let playing = tuple(&mut socket, "ack");
    assert_eq!(playing[3]["ok"], true);
    assert_eq!(playing[2]["frame"], 1);
    // Hold the restarted tuple's Ready, then pause. A stale old-generation
    // Ready must not have admitted even a same-time successor in the interim.
    send(
        &mut socket,
        json!({"version":1,"kind":"control","request_id":3,"action":"pause"}),
    );
    let paused = tuple(&mut socket, "ack");
    assert_eq!(paused[2]["frame"], 1);
    assert_eq!(paused[0]["paused"], true);
    socket.close(None).unwrap();
    drop(socket);
}

#[test]
fn explicit_readonly_refusal_and_bad_credit_leave_reconnect_available() {
    let fixture = Fixture::new();
    let mut socket = fixture.connect();
    let _ = tuple(&mut socket, "publication_credit");
    send(
        &mut socket,
        json!({"version":1,"kind":"lifecycle","request_id":1,"world_id":1,"epoch":0,"action":"get_state","model_id":2}),
    );
    let refused = tuple(&mut socket, "lifecycle_ack");
    assert_eq!(refused[3]["ok"], false);
    assert_eq!(refused[2]["frame"], 1);
    send(
        &mut socket,
        json!({"version":1,"kind":"publication_ready","token":999}),
    );
    assert!(socket.read().is_err());
    drop(socket);
    let mut reconnected = fixture.connect();
    let current = tuple(&mut reconnected, "publication_credit");
    assert_eq!(current[2]["frame"], 1);
    assert_eq!(current[0]["paused"], true);
    reconnected.close(None).unwrap();
    drop(reconnected);
}

#[test]
fn playback_session_returns_close_code_and_reconnects_with_fresh_credit_at_the_paused_cursor() {
    use tungstenite::protocol::{CloseFrame, frame::coding::CloseCode};

    let fixture = Fixture::new();
    let mut first = fixture.connect();
    let initial = tuple(&mut first, "publication_credit");
    send(
        &mut first,
        json!({"version":1,"kind":"control","request_id":1,"action":"resume"}),
    );
    let playing = tuple(&mut first, "ack");
    assert_eq!(playing[2]["playing"], true);
    send(
        &mut first,
        json!({"version":1,"kind":"publication_ready","token":initial[3]["token"]}),
    );
    let second = tuple(&mut first, "publication_credit");
    assert_eq!(second[2]["frame"], 2);
    send(
        &mut first,
        json!({"version":1,"kind":"control","request_id":2,"action":"pause"}),
    );
    let paused = tuple(&mut first, "ack");
    assert_eq!(paused[2]["frame"], 2);
    assert_eq!(paused[2]["playing"], false);
    // Ping/pong is independent of the unreturned normal presentation token.
    first.send(Message::Ping(vec![3, 1, 4].into())).unwrap();
    assert_eq!(first.read().unwrap(), Message::Pong(vec![3, 1, 4].into()));
    let close = CloseFrame {
        code: CloseCode::Away,
        reason: "closing local playback".into(),
    };
    first.close(Some(close.clone())).unwrap();
    assert_eq!(first.read().unwrap(), Message::Close(Some(close)));
    drop(first);

    let mut next = fixture.connect();
    let current = tuple(&mut next, "publication_credit");
    assert_eq!(current[2]["frame"], 2);
    assert_eq!(current[2]["playing"], false);
    assert_eq!(current[3]["token"], 1);
    assert!(current[0]["sequence"].as_u64() > paused[0]["sequence"].as_u64());
    // Restart is the supported seek to origin. Its priority reply bypasses
    // held credit and the new connection can start its request ids at one.
    send(
        &mut next,
        json!({"version":1,"kind":"control","request_id":1,"action":"reset"}),
    );
    let reset = tuple(&mut next, "ack");
    assert_eq!(reset[2]["frame"], 1);
    assert_eq!(reset[2]["epoch"], 1);
    assert_eq!(reset[3]["paused"], true);
    send(
        &mut next,
        json!({"version":1,"kind":"publication_ready","token":1}),
    );
    let restored = tuple(&mut next, "publication_credit");
    assert_eq!(restored[2]["frame"], 1);
    assert_eq!(restored[3]["token"], 2);

    send(
        &mut next,
        json!({"version":1,"kind":"control","request_id":2,"action":"seek","frame":3}),
    );
    assert!(next.read().is_err());
    drop(next);
    let mut verified = fixture.connect();
    let snapshot = tuple(&mut verified, "publication_credit");
    assert_eq!(
        (snapshot[2]["frame"].as_u64(), snapshot[2]["epoch"].as_u64()),
        (Some(1), Some(1))
    );
}

#[test]
fn playback_session_keeps_the_active_peer_when_a_second_viewer_connects() {
    let fixture = Fixture::new();
    let mut active = fixture.connect();
    let initial = tuple(&mut active, "publication_credit");
    let stream = TcpStream::connect((std::net::Ipv4Addr::LOCALHOST, fixture.port)).unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(2)))
        .unwrap();
    assert!(
        tungstenite::client(
            format!(
                "ws://127.0.0.1:{}/?replay=1&lifecycle=1&lifecycle_credit=1",
                fixture.port
            ),
            stream
        )
        .is_err()
    );
    send(
        &mut active,
        json!({"version":1,"kind":"lifecycle","request_id":1,"world_id":1,"epoch":0,"action":"remove","model_id":2}),
    );
    let denied = tuple(&mut active, "lifecycle_ack");
    assert_eq!(denied[3]["ok"], false);
    assert_eq!(denied[3]["error"], "recorded playback is read-only");
    send(
        &mut active,
        json!({"version":1,"kind":"publication_ready","token":initial[3]["token"]}),
    );
    assert_eq!(tuple(&mut active, "publication_credit")[2]["frame"], 1);
    send(
        &mut active,
        json!({"version":1,"kind":"lifecycle","request_id":1,"world_id":1,"epoch":0,"action":"remove","model_id":2}),
    );
    assert!(active.read().is_err());
}
