// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn next(socket: &mut WebSocket<TcpStream>, kind: &str) -> Value {
    let deadline = std::time::Instant::now() + TIMEOUT;
    while std::time::Instant::now() < deadline {
        let Message::Text(text) = socket.read().unwrap() else {
            panic!("expected scene or command response")
        };
        let msg: Value = serde_json::from_str(&text).unwrap();
        if msg["kind"] == kind {
            return msg;
        }
    }
    panic!("missing {kind}")
}

fn capture(path: &Path) -> Vec<(u64, bool)> {
    let world =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/worlds/local_robot_replay.sdf");
    let mut network = Network::new();
    for _ in 0..4 {
        let port = super::candidate_port();
        let mut executable = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
        executable
            .arg(&world)
            .args([
                "--server",
                "--physics-engine",
                "multifidelity",
                "--iterations",
                "180",
                "--web-viewer",
                &port.to_string(),
                "--web-lifecycle",
                "--record-scene",
            ])
            .arg(path);
        let mut process = Process(Some(network.spawn_server(&mut executable)));
        let line = super::super::listen_line(&mut process);
        if line.contains("Address already in use") {
            assert!(!process.wait(TIMEOUT).status.success());
            continue;
        }
        assert!(line.contains("WebSocket"), "{line}");
        let stream = TcpStream::connect((Ipv4Addr::LOCALHOST, port)).unwrap();
        stream.set_read_timeout(Some(TIMEOUT)).unwrap();
        stream.set_write_timeout(Some(TIMEOUT)).unwrap();
        let (mut socket, _) =
            tungstenite::client(format!("ws://127.0.0.1:{port}/?lifecycle=1"), stream).unwrap();
        let catalog = next(&mut socket, "lifecycle_catalog");
        let mut outcomes = Vec::new();
        for (id, (name, editable)) in [("default::gripper", false), ("default::parcel", true)]
            .into_iter()
            .enumerate()
        {
            let model = catalog["models"]
                .as_array()
                .unwrap()
                .iter()
                .find(|m| m["name"] == name)
                .unwrap();
            assert_eq!(model["can_set_state"], editable);
            let model_id = model["model_id"].as_u64().unwrap();
            let request = json!({"version":1,"kind":"lifecycle","world_id":catalog["world_id"],"epoch":0,"request_id":id+1,
                "action":"set_state","model_id":model_id,"translation":[0.0,0.46,0.675],"rotation":[0,0,0,1],
                "linear_velocity":[0.0,0.0,-0.02],"angular_velocity":[0,0,0]});
            socket.send(Message::text(request.to_string())).unwrap();
            let ack = next(&mut socket, "lifecycle_ack");
            assert_eq!(ack["ok"], editable);
            assert_eq!(ack["request_id"], id + 1);
            assert_eq!(ack["nanoseconds"], 0);
            if editable {
                assert_eq!(ack["state"]["translation"], request["translation"]);
                assert_eq!(ack["state"]["linear_velocity"], request["linear_velocity"]);
            } else {
                assert!(ack["state"].is_null() && ack["model_id"].is_null());
            }
            outcomes.push((model_id, editable));
        }
        socket
            .send(Message::text(
                json!({"version":1,"kind":"control","action":"resume","request_id":3}).to_string(),
            ))
            .unwrap();
        assert_eq!(next(&mut socket, "ack")["ok"], true);
        let result = process.wait(TIMEOUT);
        assert!(
            result.status.success(),
            "{}",
            String::from_utf8_lossy(&result.stderr)
        );
        assert!(
            result
                .stdout
                .starts_with(b"simulation_ns=180000000 iterations=180")
        );
        return outcomes;
    }
    panic!("ephemeral recording ports remained occupied")
}

#[test]
fn a_refused_robot_edit_and_an_accepted_parcel_launch_replay_at_their_exact_cuts() {
    let out = tempfile::tempdir().unwrap();
    let archive = std::env::var_os("GZ_RECORDED_PARCEL_EDIT")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| out.path().join("parcel"));
    let expected = capture(&archive);
    let (mut child, mut socket) = player(&archive);
    let mut frame = read_tuple(&mut socket, "publication_credit");
    let mut edits = Vec::new();
    let mut saw_first_solve = false;
    let parcel = |scene: &Value| {
        scene["visuals"]
            .as_array()
            .unwrap()
            .iter()
            .find(|v| v["name"] == "default::parcel::box::box")
            .unwrap()["translation"]
            .clone()
    };
    let initial = parcel(&frame[0]);
    let gripper: Vec<_> = frame[0]["visuals"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|v| {
            v["name"]
                .as_str()
                .unwrap()
                .starts_with("default::gripper::")
        })
        .cloned()
        .collect();
    assert!(!gripper.is_empty());
    socket
        .send(Message::text(
            json!({"version":1,"kind":"control","request_id":1,"action":"resume"}).to_string(),
        ))
        .unwrap();
    assert_eq!(read_tuple(&mut socket, "ack")[3]["ok"], true);
    loop {
        for change in frame[2]["outcomes"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|v| v["action"] == "set_state")
        {
            edits.push((
                change["model_id"].as_u64().unwrap(),
                change["ok"].as_bool().unwrap(),
            ));
            assert_eq!(frame[2]["recorded_nanoseconds"], 0);
            let pose = parcel(&frame[0]);
            if change["ok"] == true {
                assert_eq!(pose, json!([0.0, 0.46, 0.675]));
            } else {
                assert_eq!(pose, initial);
            }
            for visual in &gripper {
                assert!(frame[0]["visuals"].as_array().unwrap().contains(visual));
            }
        }
        if frame[2]["recorded_nanoseconds"] == 1_000_000 {
            let dt = 0.001;
            let expected_z = 0.675 - 0.02 * dt - 0.5 * 9.81 * dt * dt;
            assert!((parcel(&frame[0])[2].as_f64().unwrap() - expected_z).abs() < 2e-6);
            saw_first_solve = true;
        }
        if frame[2]["finished"] == true {
            break;
        }
        socket
            .send(Message::text(
                json!({"version":1,"kind":"publication_ready","token":frame[3]["token"]})
                    .to_string(),
            ))
            .unwrap();
        frame = read_tuple(&mut socket, "publication_credit");
    }
    assert!(saw_first_solve);
    assert_eq!(edits, expected);
    assert_eq!(frame[0]["nanoseconds"], 180_000_000);
    let landing = parcel(&frame[0]);
    assert!(landing[0].as_f64().unwrap().abs() < 0.001);
    assert!((landing[1].as_f64().unwrap() - 0.46).abs() < 0.001);
    assert!(
        (landing[2].as_f64().unwrap() - 0.615).abs() < 0.002,
        "{landing}"
    );
    socket.close(None).unwrap();
    let pid = rustix::process::Pid::from_child(child.0.as_ref().unwrap());
    rustix::process::kill_process(pid, rustix::process::Signal::TERM).unwrap();
    assert!(child.wait(TIMEOUT).status.success());
}
