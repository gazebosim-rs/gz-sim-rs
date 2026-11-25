// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::{TIMEOUT, run_operator, start_server};
use serde_json::Value;
use std::{
    net::{Ipv4Addr, TcpListener, TcpStream},
    path::{Path, PathBuf},
    thread,
};
use tungstenite::{Message, WebSocket};

fn json(frame: &Message) -> Value {
    serde_json::from_str(frame.to_text().unwrap()).unwrap()
}

fn connect(port: u16) -> TcpStream {
    let stream = TcpStream::connect((Ipv4Addr::LOCALHOST, port)).unwrap();
    stream.set_read_timeout(Some(TIMEOUT)).unwrap();
    stream.set_write_timeout(Some(TIMEOUT)).unwrap();
    stream
}

fn read_kind(socket: &mut WebSocket<TcpStream>, kind: &str) -> Message {
    loop {
        let frame = socket.read().unwrap();
        if frame.is_text() && json(&frame)["kind"] == kind {
            return frame;
        }
    }
}

#[derive(Clone, Copy)]
enum Cut {
    BeforeDelivery,
    AfterCommit,
}

/// The operator sees the real server's initial state. Cut either the outgoing
/// step or its committed reply, so both unknown outcomes are reproducible.
fn interrupt(port: u16, executable: &Path, cut: Cut) {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let proxy_port = listener.local_addr().unwrap().port();
    let proxy = thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        stream.set_read_timeout(Some(TIMEOUT)).unwrap();
        stream.set_write_timeout(Some(TIMEOUT)).unwrap();
        let mut client = tungstenite::accept(stream).unwrap();
        let (mut server, _) =
            tungstenite::client(format!("ws://127.0.0.1:{port}/?operator=1"), connect(port))
                .unwrap();
        client.send(read_kind(&mut server, "scene")).unwrap();
        let step = read_kind(&mut client, "operator_step");
        let request = json(&step);
        assert_eq!(request["epoch"], 0);
        assert_eq!(request["tick"], 0);
        if matches!(cut, Cut::AfterCommit) {
            server.send(step).unwrap();
            let mut receipt = json(&read_kind(&mut server, "ack"));
            assert_eq!(receipt["request_id"], request["request_id"]);
            assert_eq!(receipt["ok"], true);
            assert_eq!(receipt["nanoseconds"], 1_000_000);
            // A stale receipt must not turn an unknown outcome into a success.
            receipt["request_id"] = Value::from(2);
            client
                .send(Message::Text(receipt.to_string().into()))
                .unwrap();
        }
        // Drop the TCP connection without returning a WebSocket receipt.
    });
    let result = run_operator(executable, proxy_port, &["step", "0", "0"]);
    assert!(!result.status.success());
    assert!(result.stdout.is_empty());
    let message = String::from_utf8(result.stderr).unwrap();
    assert!(message.contains("Command outcome unknown"), "{message}");
    assert!(
        message.contains(&format!("--port {proxy_port} status")),
        "{message}"
    );
    proxy.join().unwrap();
}

#[test]
fn installed_rotor_reconciles_a_step_lost_before_delivery_or_after_commit() {
    let (world, server, client) = match std::env::var_os("GZ_TEST_LOCAL_CPU_INSTALL") {
        Some(path) => {
            let root = PathBuf::from(path);
            (
                root.join("share/gz-sim/upstream_joint_force.sdf"),
                root.join("bin/gz-sim-rs"),
                root.join("bin/gz-sim-control"),
            )
        }
        None => (
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../examples/worlds/upstream_joint_force.sdf"),
            PathBuf::from(env!("CARGO_BIN_EXE_gz-sim-rs")),
            PathBuf::from(env!("CARGO_BIN_EXE_gz-sim-control")),
        ),
    };
    let (_network, mut process, port) = start_server(&world, &server);
    let command = |args: &[&str], accepted: bool| {
        let result = run_operator(&client, port, args);
        assert_eq!(result.status.success(), accepted, "{args:?}: {result:?}");
        serde_json::from_slice::<Value>(&result.stdout).unwrap()
    };
    let poses = |epoch: u64, tick: u64| {
        let status = command(&["status"], true);
        let scene = &status["scene"];
        assert_eq!(scene["paused"], true);
        assert_eq!(scene["epoch"], epoch);
        assert_eq!(scene["seconds"], 0);
        assert_eq!(scene["nanoseconds"], tick * 1_000_000);
        assert_eq!(scene["visuals"].as_array().unwrap().len(), 3);
        scene["visuals"].clone()
    };
    let initial = poses(0, 0);
    interrupt(port, &client, Cut::BeforeDelivery);
    assert_eq!(poses(0, 0), initial);
    interrupt(port, &client, Cut::AfterCommit);
    let first = poses(0, 1);
    assert_ne!(first, initial);
    assert_eq!(command(&["step", "0", "0"], false)["ok"], false);
    assert_eq!(poses(0, 1), first);
    assert_eq!(command(&["step", "0", "1"], true)["ok"], true);
    let second = poses(0, 2);

    assert_eq!(command(&["reset"], true)["epoch"], 1);
    assert_eq!(poses(1, 0), initial);
    assert_eq!(command(&["step", "0", "0"], false)["ok"], false);
    assert_eq!(poses(1, 0), initial);
    for (tick, expected) in [(0, first), (1, second)] {
        assert_eq!(command(&["step", "1", &tick.to_string()], true)["ok"], true);
        assert_eq!(poses(1, tick + 1), expected);
    }
    let pid = rustix::process::Pid::from_child(process.0.as_ref().unwrap());
    rustix::process::kill_process(pid, rustix::process::Signal::TERM).unwrap();
    assert!(process.wait(TIMEOUT).status.success());
}
