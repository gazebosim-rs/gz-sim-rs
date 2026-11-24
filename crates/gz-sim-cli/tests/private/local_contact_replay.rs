// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Network, Process, candidate_port};
use gz_sim_scene::{Name, PhysicsEngine, WorldPose, load_world_with_sdf_step};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    net::{Ipv4Addr, TcpStream},
    path::Path,
    process::{Command, Stdio},
    time::Duration,
};
use tungstenite::{Message, WebSocket};

const WORLD: &str = include_str!("../../../../examples/multifidelity_finite_primary.sdf");
const TIMEOUT: Duration = Duration::from_secs(10);

fn record(path: &Path, ticks: u64) {
    let source = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/multifidelity_finite_primary.sdf");
    for _ in 0..4 {
        let port = candidate_port();
        let mut network = Network::new();
        let mut cmd = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
        cmd.arg(&source)
            .args([
                "--server",
                "--run",
                "--iterations",
                &ticks.to_string(),
                "--physics-engine",
                "multifidelity",
                "--web-viewer",
                &port.to_string(),
                "--web-lifecycle",
                "--record-scene",
            ])
            .arg(path);
        let result = Process(Some(network.spawn_server(&mut cmd))).wait(TIMEOUT);
        if !result.status.success()
            && String::from_utf8_lossy(&result.stderr).contains("Address already in use")
        {
            assert!(!path.exists());
            continue;
        }
        assert!(
            result.status.success(),
            "{}",
            String::from_utf8_lossy(&result.stderr)
        );
        assert!(result.stdout.starts_with(
            format!("simulation_ns={} iterations={ticks}\n", ticks * 500_000).as_bytes()
        ));
        return;
    }
    panic!("ephemeral recording ports remained occupied");
}

fn player(path: &Path) -> (Process, WebSocket<TcpStream>) {
    for _ in 0..4 {
        let port = candidate_port();
        let child = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
            .arg(path)
            .args(["--server", "--replay", "--web-viewer", &port.to_string()])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let mut child = Process(Some(child));
        let line = super::listen_line(&mut child);
        if line.contains("Address already in use") {
            assert!(!child.wait(TIMEOUT).status.success());
            continue;
        }
        assert!(
            line.contains(&format!("read-only playback: ws://127.0.0.1:{port}/")),
            "{line}"
        );
        let stream = TcpStream::connect((Ipv4Addr::LOCALHOST, port)).unwrap();
        stream.set_read_timeout(Some(TIMEOUT)).unwrap();
        stream.set_write_timeout(Some(TIMEOUT)).unwrap();
        let (socket, _) = tungstenite::client(
            format!("ws://127.0.0.1:{port}/?replay=1&lifecycle=1&lifecycle_credit=1"),
            stream,
        )
        .unwrap();
        return (child, socket);
    }
    panic!("ephemeral replay ports remained occupied");
}

fn read_tuple(socket: &mut WebSocket<TcpStream>, last: &str) -> [Value; 4] {
    let tuple: [Value; 4] = std::array::from_fn(|_| {
        let Message::Text(msg) = socket.read().unwrap() else {
            panic!("expected text publication")
        };
        serde_json::from_str(&msg).unwrap()
    });
    for (msg, kind) in tuple
        .iter()
        .zip(["scene", "lifecycle_catalog", "replay_status", last])
    {
        assert_eq!(msg["kind"], kind);
    }
    assert_eq!(tuple[0]["sequence"], tuple[1]["scene_sequence"]);
    assert_eq!(tuple[0]["sequence"], tuple[2]["scene_sequence"]);
    tuple
}

#[test]
fn recorded_contact_poses_survive_real_cli_replay_and_rewind() {
    let dir = tempfile::tempdir().unwrap();
    let archive = dir.path().join("rocker");
    record(&archive, 1);
    let mut reference = load_world_with_sdf_step(
        WORLD,
        "rocker.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let mut poses = Vec::new();
    for tick in 0..=1 {
        if tick != 0 {
            reference.step_exact(1).unwrap();
        }
        let values: BTreeMap<_, _> = reference
            .world_mut()
            .query::<(&Name, &WorldPose)>()
            .iter(reference.world())
            .map(|(name, pose)| (name.0.clone(), *pose))
            .collect();
        poses.push(values);
    }
    let (mut process, mut socket) = player(&archive);
    let mut frame = read_tuple(&mut socket, "publication_credit");
    let origin = frame[0]["visuals"].clone();
    let mut last_ns = 0;
    for ordinal in 1..=2 {
        let ns = frame[2]["recorded_nanoseconds"].as_u64().unwrap();
        assert!(ns.is_multiple_of(500_000) && ns <= 500_000);
        assert_eq!(frame[0]["nanoseconds"], ns);
        assert_eq!(frame[2]["frame"], ordinal);
        if ordinal > 1 {
            assert!(ns > last_ns);
        } else {
            assert_eq!(ns, 0);
        }
        last_ns = ns;
        let visuals = frame[0]["visuals"].as_array().unwrap();
        assert_eq!(visuals.len(), 3);
        for visual in visuals {
            let pose = &poses[(ns / 500_000) as usize][visual["name"].as_str().unwrap()];
            assert_eq!(visual["translation"], json!(pose.translation));
            assert_eq!(visual["rotation"], json!(pose.rotation));
        }
        if frame[2]["finished"] == true {
            break;
        }
        let token = frame[3]["token"].as_u64().unwrap();
        if ordinal == 1 {
            socket
                .send(Message::text(
                    json!({"version":1,"kind":"control","request_id":1,"action":"resume"})
                        .to_string(),
                ))
                .unwrap();
            assert_eq!(read_tuple(&mut socket, "ack")[3]["ok"], true);
        }
        socket
            .send(Message::text(
                json!({"version":1,"kind":"publication_ready","token":token}).to_string(),
            ))
            .unwrap();
        frame = read_tuple(&mut socket, "publication_credit");
    }
    assert_eq!(last_ns, 500_000);
    assert_eq!(frame[2]["finished"], true);
    socket
        .send(Message::text(
            json!({"version":1,"kind":"control","request_id":2,"action":"reset"}).to_string(),
        ))
        .unwrap();
    let rewind = read_tuple(&mut socket, "ack");
    assert_eq!(rewind[3]["ok"], true);
    assert_eq!(rewind[2]["frame"], 1);
    assert_eq!(rewind[0]["visuals"], origin);
    socket.close(None).unwrap();
    let pid = rustix::process::Pid::from_child(process.0.as_ref().unwrap());
    rustix::process::kill_process(pid, rustix::process::Signal::TERM).unwrap();
    assert!(process.wait(TIMEOUT).status.success());
}
