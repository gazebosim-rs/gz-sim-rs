// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Network, Process, candidate_port, listen_line};
use gz_sim_scene::{EntityKind, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    path::Path,
    process::{Command, Stdio},
    time::{Duration, Instant},
};

const SDF: &str = "examples/worlds/local_robot_serial_transfer.sdf";
const TIMEOUT: Duration = Duration::from_secs(15);

fn start(path: &Path) -> (Network, Process, u16) {
    for _ in 0..4 {
        let port = candidate_port();
        let mut network = Network::new();
        let mut cmd = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
        cmd.arg(path).args([
            "--server",
            "--physics-engine",
            "multifidelity",
            "--web-viewer",
            &port.to_string(),
        ]);
        let mut process = Process(Some(network.spawn_server(&mut cmd)));
        let mut line = listen_line(&mut process);
        if line.starts_with("gzweb-rs local glTF resources:") {
            line = listen_line(&mut process);
        }
        if line.contains("Address already in use") {
            assert!(!process.wait(TIMEOUT).status.success());
            continue;
        }
        assert!(
            line.contains(&format!("WebSocket: ws://127.0.0.1:{port}/")),
            "{line}"
        );
        return (network, process, port);
    }
    panic!("ephemeral viewer ports remained occupied");
}

fn operator(port: u16, arguments: &[&str], passed: bool) -> Value {
    let child = Command::new(env!("CARGO_BIN_EXE_gz-sim-control"))
        .args(["--port", &port.to_string()])
        .args(arguments)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let out = Process(Some(child)).wait(TIMEOUT);
    assert_eq!(
        out.status.success(),
        passed,
        "{arguments:?}: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    if out.stdout.is_empty() {
        assert_eq!(arguments[0], "recover");
        assert!(String::from_utf8_lossy(&out.stderr).contains("no parked operator fence"));
        return Value::Null;
    }
    serde_json::from_slice(&out.stdout).unwrap()
}

#[test]
fn fresh_operators_step_the_loaded_serial_robot_once_per_authorized_cut() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(SDF);
    let xml = std::fs::read_to_string(&path).unwrap();
    let mut reference = load_world_with_sdf_step(
        &xml,
        SDF,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let (_network, mut process, port) = start(&path);
    let mut first = None;
    for tick in 0..=3 {
        if tick > 0 {
            let result = operator(port, &["step", "0", &(tick - 1).to_string()], true);
            assert_eq!(result["ok"], true);
            assert_eq!(result["nanoseconds"], tick * 1_000_000);
            reference.step_exact(1).unwrap();
        }
        let status = operator(port, &["status"], true);
        assert!(status["operator_fence"].is_null() && status["operator_restored"].is_null());
        let scene = &status["scene"];
        assert_eq!(scene["paused"], true);
        assert_eq!(scene["nanoseconds"], tick * 1_000_000);
        let poses: BTreeMap<_, _> = reference
            .world_mut()
            .query::<(&Name, &WorldPose, &EntityKind)>()
            .iter(reference.world())
            .filter(|(_, _, kind)| **kind == EntityKind::Visual)
            .map(|(name, pose, _)| (name.0.clone(), *pose))
            .collect();
        let visuals = scene["visuals"].as_array().unwrap();
        assert_eq!(visuals.len(), poses.len());
        for v in visuals {
            let pose = &poses[v["name"].as_str().unwrap()];
            assert_eq!(v["translation"], json!(pose.translation));
            assert_eq!(v["rotation"], json!(pose.rotation));
        }
        if tick == 0 {
            first = Some(visuals.clone())
        } else if tick == 3 {
            assert_ne!(first.as_ref().unwrap(), visuals);
        }
    }
    for stale in [["step", "0", "2"], ["step", "1", "3"]] {
        let denied = operator(port, &stale, false);
        assert_eq!(
            (denied["ok"].as_bool(), denied["nanoseconds"].as_u64()),
            (Some(false), Some(3_000_000))
        );
    }
    operator(port, &["recover", "0", "3", "no-fence"], false);
    assert_eq!(operator(port, &["play"], true)["paused"], false);
    let deadline = Instant::now() + TIMEOUT;
    loop {
        let status = operator(port, &["status"], true);
        assert_eq!(status["scene"]["paused"], false);
        if status["scene"]["nanoseconds"].as_u64().unwrap() > 3_000_000 {
            break;
        }
        assert!(Instant::now() < deadline);
    }
    let paused = operator(port, &["pause"], true);
    assert_eq!(paused["paused"], true);
    let cut = operator(port, &["status"], true);
    assert_eq!(cut["scene"]["seconds"], paused["seconds"]);
    assert_eq!(cut["scene"]["nanoseconds"], paused["nanoseconds"]);
    assert_eq!(operator(port, &["play"], true)["paused"], false);
    let reset = operator(port, &["reset"], true);
    assert_eq!(reset["epoch"], 1);
    assert_eq!(reset["paused"], true);
    assert_eq!(reset["seconds"], 0);
    assert_eq!(reset["nanoseconds"], 0);
    let origin = operator(port, &["status"], true);
    assert_eq!(
        origin["scene"]["visuals"].as_array().unwrap(),
        first.as_ref().unwrap()
    );
    assert_eq!(operator(port, &["step", "0", "0"], false)["epoch"], 1);
    let next = operator(port, &["step", "1", "0"], true);
    assert_eq!(next["ok"], true);
    assert_eq!(next["nanoseconds"], 1_000_000);
    let pid = rustix::process::Pid::from_child(process.0.as_ref().unwrap());
    rustix::process::kill_process(pid, rustix::process::Signal::TERM).unwrap();
    assert!(process.wait(TIMEOUT).status.success());
}

/// Exercise the public force topic and world service in the installed-style server.
/// Read poses from the same socket as gz-web, including the ground mesh resource.
#[test]
fn upstream_motor_on_a_free_base_responds_to_transport_across_reset() {
    use gz_sim_proto::{
        msgs::{Double, WorldControl, WorldReset},
        prost::Message,
    };
    use std::net::{Ipv4Addr, TcpStream};
    use zenoh::Wait;
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/worlds/upstream_joint_force.sdf");
    let (network, mut process, port) = start(&path);
    let session = network.session();
    let publisher = session
        .declare_publisher(network.key("/model/joint_force_test/joint/j1/cmd_force"))
        .wait()
        .unwrap();
    let scene = || {
        let stream = TcpStream::connect((Ipv4Addr::LOCALHOST, port)).unwrap();
        stream.set_read_timeout(Some(TIMEOUT)).unwrap();
        let (mut socket, _) =
            tungstenite::client(format!("ws://127.0.0.1:{port}/"), stream).unwrap();
        let message = socket.read().unwrap().into_text().unwrap();
        socket.close(None).unwrap();
        serde_json::from_str::<Value>(&message).unwrap()
    };
    let wait_for = |epoch: u64, tick: u64| {
        let deadline = Instant::now() + TIMEOUT;
        loop {
            let s = scene();
            if s["epoch"] == epoch && s["nanoseconds"] == tick * 1_000_000 && s["paused"] == true {
                return s;
            }
            assert!(
                Instant::now() < deadline,
                "unreached clock {epoch}:{tick}, got {s}"
            );
        }
    };
    let rotor_angle = |s: &Value| {
        let visuals = s["visuals"].as_array().unwrap();
        assert_eq!(visuals.len(), 3);
        let ground = visuals
            .iter()
            .find(|v| v["name"].as_str().unwrap().contains("::ground_plane::"))
            .unwrap();
        assert!(
            ground["shape"]["uri"]
                .as_str()
                .unwrap()
                .ends_with("/plane.glb")
        );
        let rotor = visuals
            .iter()
            .find(|v| v["name"].as_str().unwrap().ends_with("::rotor::visual"))
            .unwrap();
        assert_eq!(rotor["shape"]["kind"], "box");
        let q = &rotor["rotation"];
        2.0 * q[2].as_f64().unwrap().atan2(q[3].as_f64().unwrap())
    };
    for (epoch, torque) in [(0, 0.012_f64), (1, -0.012)] {
        if epoch != 0 {
            network.control(
                &session,
                WorldControl {
                    pause: true,
                    reset: Some(WorldReset {
                        all: true,
                        ..Default::default()
                    }),
                    ..Default::default()
                }
                .encode_to_vec(),
            );
        }
        assert!(rotor_angle(&wait_for(epoch, 0)).abs() < 1e-9);
        let changes = publisher.matching_listener().wait().unwrap();
        let until = Instant::now() + TIMEOUT;
        while !publisher.matching_status().wait().unwrap().matching() {
            changes
                .recv_timeout(until.saturating_duration_since(Instant::now()))
                .unwrap()
                .expect("motor subscriber");
        }
        publisher
            .put(
                Double {
                    data: torque,
                    ..Default::default()
                }
                .encode_to_vec(),
            )
            .attachment("gz.msgs.Double")
            .wait()
            .unwrap();
        network.control(
            &session,
            WorldControl {
                pause: true,
                multi_step: 100,
                ..Default::default()
            }
            .encode_to_vec(),
        );
        let angle = rotor_angle(&wait_for(epoch, 100));
        // 0.5 * (0.012 / 0.00012) * 0.1^2 radians, with a 1 ms discrete step.
        assert!(
            (angle - 0.5 * torque.signum()).abs() < 0.015,
            "epoch {epoch}, angle {angle}"
        );
    }
    let pid = rustix::process::Pid::from_child(process.0.as_ref().unwrap());
    rustix::process::kill_process(pid, rustix::process::Signal::TERM).unwrap();
    assert!(process.wait(TIMEOUT).status.success());
}
