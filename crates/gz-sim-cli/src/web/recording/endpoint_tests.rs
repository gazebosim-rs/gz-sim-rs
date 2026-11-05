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
