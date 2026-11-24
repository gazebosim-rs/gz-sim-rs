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
        let line = listen_line(&mut process);
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
    let child = Command::new(env!("CARGO_BIN_EXE_gz-sim-convoy-control"))
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
    let pid = rustix::process::Pid::from_child(process.0.as_ref().unwrap());
    rustix::process::kill_process(pid, rustix::process::Signal::TERM).unwrap();
    assert!(process.wait(TIMEOUT).status.success());
}
