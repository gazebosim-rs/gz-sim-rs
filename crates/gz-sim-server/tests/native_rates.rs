// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_msgs::{
    msgs::{Boolean, Imu, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_core::{EntityId, UpdateInfo};
use gz_sim_scene::{ImuPublication, PhysicsEngine, imu_rate, load_world_with_engine};
use gz_sim_server::Server;
use gz_transport::{Config, Node};
use sdformat::NoResources;
use std::{
    io::{BufRead, BufReader, Read, Write},
    net::UdpSocket,
    num::NonZeroUsize,
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::{Arc, mpsc},
    thread,
    time::{Duration, Instant},
};

// Keep the port reserved while transport binds its reusable discovery socket.
// A bare temporary socket dropped before startup leaves a port-stealing race.
fn discovery_socket() -> UdpSocket {
    let socket = UdpSocket::bind("0.0.0.0:0").unwrap();
    socket2::SockRef::from(&socket)
        .set_reuse_address(true)
        .unwrap();
    #[cfg(target_os = "linux")]
    socket2::SockRef::from(&socket)
        .set_multicast_all_v4(false)
        .unwrap();
    socket
}

struct Peer {
    child: Child,
    lines: mpsc::Receiver<String>,
    stderr: Option<thread::JoinHandle<String>>,
    stdout: Option<thread::JoinHandle<()>>,
    log_dir: PathBuf,
    commands: std::fs::File,
}
impl Drop for Peer {
    fn drop(&mut self) {
        let status = match self.child.try_wait() {
            Ok(Some(status)) => Some(status),
            _ => {
                let _ = self.child.kill();
                self.child.wait().ok()
            }
        };
        let _ = std::fs::write(
            self.log_dir.join("exit.json"),
            serde_json::to_vec(
                &serde_json::json!({"status": status.map(|value| value.to_string())}),
            )
            .unwrap(),
        );
        if let Some(reader) = self.stdout.take() {
            let _ = reader.join();
        }
        if let Some(reader) = self.stderr.take()
            && let Ok(text) = reader.join()
            && !text.is_empty()
        {
            eprintln!("native rate stderr: {text}");
        }
        if thread::panicking() {
            eprintln!("Native rate client logs: {}", self.log_dir.display());
        }
    }
}
impl Peer {
    fn send(&mut self, command: &str) {
        writeln!(self.commands, "{command}").unwrap();
        writeln!(self.child.stdin.as_mut().unwrap(), "{command}").unwrap();
    }
    fn line(&mut self, server: &mut Server) -> String {
        let deadline = Instant::now() + Duration::from_secs(8);
        loop {
            match self.lines.try_recv() {
                Ok(line) => return line,
                Err(mpsc::TryRecvError::Empty) => {}
                Err(error) => panic!(
                    "native rate client closed: {error}, {:?}",
                    self.child.try_wait()
                ),
            }
            assert!(Instant::now() < deadline, "native rate client timed out");
            server.tick().unwrap();
            thread::sleep(Duration::from_millis(1));
        }
    }
    fn arm(&mut self, server: &mut Server) {
        self.send("arm");
        assert_eq!(self.line(server), "ARMED");
    }
    fn samples(&mut self, server: &mut Server, count: usize) -> Vec<Imu> {
        let deadline = Instant::now() + Duration::from_secs(8);
        loop {
            self.send("snapshot");
            let line = self.line(server);
            let wires: Vec<String> = serde_json::from_str(&line).unwrap();
            assert!(wires.len() <= count, "unexpected extra native IMU frame");
            if wires.len() == count {
                return wires.iter().map(|wire| decode(wire)).collect();
            }
            assert!(Instant::now() < deadline, "native IMU frames missing");
        }
    }
    fn rate(&mut self, server: &mut Server, id: u64, value: f64, expected: f64) {
        let counters = server.counters();
        let before =
            counters.imu_rates_applied + counters.imu_rates_ignored + counters.imu_rates_rejected;
        self.send(&format!("rate {value}"));
        assert_eq!(self.line(server), "SENT");
        // Test-only completion barrier: the native outputless send has no reply.
        let deadline = Instant::now() + Duration::from_secs(8);
        loop {
            let c = server.counters();
            if c.imu_rates_applied + c.imu_rates_ignored + c.imu_rates_rejected > before {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "native rate request not consumed"
            );
            server.tick().unwrap();
            thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(imu_rate(server.simulation(), id).unwrap().rate_hz, expected);
    }
}

fn decode(wire: &str) -> Imu {
    assert_eq!(wire.len() % 2, 0);
    let bytes: Vec<_> = wire
        .as_bytes()
        .chunks_exact(2)
        .map(|pair| u8::from_str_radix(std::str::from_utf8(pair).unwrap(), 16).unwrap())
        .collect();
    Imu::decode(bytes.as_slice()).unwrap()
}

fn control(server: &mut Server, node: &Arc<Node>, message: WorldControl) {
    let epoch = server.simulation().world().resource::<UpdateInfo>().epoch;
    let resetting = message.reset.is_some();
    let client = Arc::clone(node);
    let request = thread::spawn(move || {
        client.request(
            "/world/default/control",
            "gz.msgs.WorldControl",
            "gz.msgs.Boolean",
            message.encode_to_vec(),
            Duration::from_secs(7),
        )
    });
    let deadline = Instant::now() + Duration::from_secs(8);
    while !request.is_finished()
        || server.pending_steps() > 0
        || (resetting && server.simulation().world().resource::<UpdateInfo>().epoch == epoch)
    {
        assert!(Instant::now() < deadline, "rate test control timed out");
        server.tick().unwrap();
        // Pacing stays outside simulation work; it does not establish delivery.
        thread::sleep(Duration::from_millis(1));
    }
    let (bytes, success) = request.join().unwrap().unwrap();
    assert!(success && Boolean::decode(bytes.as_slice()).unwrap().data);
    assert!(server.simulation().is_paused());
}
fn step(server: &mut Server, node: &Arc<Node>, count: u32) {
    let before = server.simulation().elapsed();
    control(
        server,
        node,
        WorldControl {
            pause: true,
            multi_step: count,
            ..Default::default()
        },
    );
    assert_eq!(
        server.simulation().elapsed(),
        before + Duration::from_millis(u64::from(count))
    );
}
fn reset(server: &mut Server, node: &Arc<Node>, all: bool) {
    control(
        server,
        node,
        WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all,
                time_only: !all,
                ..Default::default()
            }),
            ..Default::default()
        },
    );
    assert_eq!(server.simulation().elapsed(), Duration::ZERO);
}
fn check_frames(frames: &[Imu], times: &serde_json::Value, sequence: usize, engine: PhysicsEngine) {
    let times: Vec<_> = times
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_u64().unwrap())
        .collect();
    assert_eq!(frames.len(), times.len());
    for (index, (frame, stamp)) in frames.iter().zip(times).enumerate() {
        let header = frame.header.as_ref().unwrap();
        let time = header.stamp.as_ref().unwrap();
        assert_eq!(time.sec as u64 * 1_000_000_000 + time.nsec as u64, stamp);
        assert_eq!(
            header
                .data
                .iter()
                .find(|item| item.key == "seq")
                .unwrap()
                .value,
            [(sequence + index).to_string()]
        );
        let rotation = frame.orientation.as_ref().unwrap();
        assert_eq!(
            [rotation.x, rotation.y, rotation.z, rotation.w],
            [0.0, 0.0, 0.0, 1.0]
        );
        let angular = frame.angular_velocity.as_ref().unwrap();
        assert_eq!([angular.x, angular.y, angular.z], [0.0; 3]);
        let linear = frame.linear_acceleration.as_ref().unwrap();
        let expected = if engine == PhysicsEngine::Tpe {
            9.81
        } else {
            0.0
        };
        assert!(
            linear.x.abs() < 1e-9 && linear.y.abs() < 1e-9 && (linear.z - expected).abs() < 1e-9
        );
        for covariance in [
            &frame.orientation_covariance,
            &frame.angular_velocity_covariance,
            &frame.linear_acceleration_covariance,
        ] {
            assert_eq!(covariance.as_ref().unwrap().data, [0.0; 9]);
        }
    }
}

#[test]
fn actual_cpp_outputless_rate_controls_preserve_phase_and_both_reset_cadences() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let build = Command::new("python3")
        .arg(root.join("scripts/native-peer.py"))
        .args(["--client", "rate"])
        .output()
        .unwrap();
    assert!(
        build.status.success(),
        "{}",
        String::from_utf8_lossy(&build.stderr)
    );
    let reference: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/runtime_imu_rate_reset.json")).unwrap();
    for (engine, delayed, fail) in [
        (PhysicsEngine::Tpe, false, None),
        (PhysicsEngine::Rapier, false, None),
        (PhysicsEngine::Dart, false, None),
        (PhysicsEngine::Mujoco, false, None),
        (PhysicsEngine::Tpe, true, None),
        (PhysicsEngine::Tpe, false, Some("exit")),
        (PhysicsEngine::Tpe, false, Some("panic")),
    ] {
        let message = discovery_socket();
        let service = discovery_socket();
        let config = Config {
            partition: format!(
                "native-rate-{}-{engine:?}-{delayed}-{}",
                std::process::id(),
                fail.unwrap_or("ok")
            ),
            message_port: message.local_addr().unwrap().port(),
            service_port: service.local_addr().unwrap().port(),
            ..Config::default()
        };
        let simulation = load_world_with_engine(
            include_str!("../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
            "rate.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            engine,
        )
        .unwrap();
        let id = simulation
            .world()
            .iter_entities()
            .find(|e| e.contains::<ImuPublication>())
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        let mut server = Server::new(
            simulation,
            "default",
            config.clone(),
            NonZeroUsize::new(4).unwrap(),
        )
        .unwrap();
        let prefix = std::env::var_os("GZ_REFERENCE_PREFIX")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| root.join("artifacts/native-reference/install"));
        let mut child = Command::new(root.join("artifacts/native-rate-peer/rate-client"))
            .args(delayed.then_some("--delay-first"))
            .env("GZ_PARTITION", &config.partition)
            .env("GZ_IP", config.interface.to_string())
            .env("GZ_TRANSPORT_IMPLEMENTATION", "zeromq")
            .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
            .env("GZ_DISCOVERY_MULTICAST_IP", config.multicast.to_string())
            .env("GZ_DISCOVERY_MSG_PORT", config.message_port.to_string())
            .env("GZ_DISCOVERY_SRV_PORT", config.service_port.to_string())
            .env_remove("GZ_TRANSPORT_USERNAME")
            .env_remove("GZ_TRANSPORT_PASSWORD")
            .env_remove("GZ_RELAY")
            .env("LD_LIBRARY_PATH", prefix.join("lib"))
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let run = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let log_dir = root.join(format!(
            "artifacts/native-rate-peer/runs/{}-{engine:?}-{run}",
            std::process::id()
        ));
        std::fs::create_dir_all(&log_dir).unwrap();
        let stdout = child.stdout.take().unwrap();
        let (send, lines) = mpsc::channel();
        let output = log_dir.join("stdout");
        let stdout = thread::spawn(move || {
            let mut reader = BufReader::new(stdout);
            let mut file = std::fs::File::create(output).unwrap();
            loop {
                let mut line = String::new();
                if reader.read_line(&mut line).unwrap() == 0 {
                    break;
                }
                file.write_all(line.as_bytes()).unwrap();
                if send
                    .send(line.trim_end_matches(['\n', '\r']).to_owned())
                    .is_err()
                {
                    break;
                }
            }
        });
        let mut stderr = child.stderr.take().unwrap();
        let errors = log_dir.join("stderr");
        let stderr = thread::spawn(move || {
            let mut bytes = Vec::new();
            stderr.read_to_end(&mut bytes).unwrap();
            std::fs::write(errors, &bytes).unwrap();
            String::from_utf8_lossy(&bytes).into_owned()
        });
        let commands = std::fs::File::create(log_dir.join("commands")).unwrap();
        let mut peer = Peer {
            child,
            lines,
            stderr: Some(stderr),
            stdout: Some(stdout),
            log_dir,
            commands,
        };
        assert_eq!(peer.line(&mut server), "READY");
        if let Some(failure) = fail {
            let logs = peer.log_dir.clone();
            if failure == "exit" {
                peer.send("invalid-command");
                assert_eq!(peer.child.wait().unwrap().code(), Some(7));
                drop(peer);
            } else {
                peer.send("hang");
                assert_eq!(peer.line(&mut server), "HANGING");
                // Rust-only test unwind; no foreign fault is caught.
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || {
                    let _peer = peer;
                    panic!("intentional rate-client cleanup control");
                }));
                assert!(result.is_err());
            }
            assert!(
                std::fs::read_to_string(logs.join("stdout"))
                    .unwrap()
                    .starts_with("READY\n")
            );
            let stderr = std::fs::read_to_string(logs.join("stderr")).unwrap();
            let command = std::fs::read_to_string(logs.join("commands")).unwrap();
            let exit: serde_json::Value =
                serde_json::from_slice(&std::fs::read(logs.join("exit.json")).unwrap()).unwrap();
            let status = exit["status"].as_str().unwrap();
            if failure == "exit" {
                assert_eq!(stderr, "invalid rate test command\n");
                assert_eq!(command, "invalid-command\n");
                assert!(status.contains('7'));
            } else {
                assert_eq!(stderr, "intentional rate test hang\n");
                assert_eq!(command, "hang\n");
                assert!(status.contains("SIGKILL"));
                assert!(
                    std::fs::read_to_string(logs.join("stdout"))
                        .unwrap()
                        .contains("HANGING\n")
                );
            }
            continue;
        }
        let node = Arc::new(Node::new(config).unwrap());
        // Prove actual IMU delivery before resetting the still-live publication lease.
        let mut connected = false;
        for _ in 0..100 {
            peer.arm(&mut server);
            step(&mut server, &node, 1);
            peer.send("snapshot");
            let wires: Vec<String> = serde_json::from_str(&peer.line(&mut server)).unwrap();
            let latest = wires
                .iter()
                .map(|wire| {
                    let frame = decode(wire);
                    let stamp = frame.header.unwrap().stamp.unwrap();
                    stamp.sec as u64 * 1_000_000_000 + stamp.nsec as u64
                })
                .max();
            if latest == Some(server.simulation().elapsed().as_nanos() as u64) {
                connected = true;
                break;
            }
        }
        assert!(connected, "native IMU connection not established");
        for case in reference["observations"].as_array().unwrap() {
            reset(&mut server, &node, true);
            peer.arm(&mut server);
            step(&mut server, &node, 10);
            check_frames(
                &peer.samples(&mut server, 10),
                &case["phases"][0],
                0,
                engine,
            );
            peer.rate(&mut server, id, 200.0, 200.0);
            peer.arm(&mut server);
            step(&mut server, &node, 20);
            check_frames(
                &peer.samples(&mut server, 4),
                &case["phases"][1],
                10,
                engine,
            );
            peer.rate(&mut server, id, 0.0, 200.0);
            reset(&mut server, &node, case["mode"] == "all");
            assert_eq!(imu_rate(server.simulation(), id).unwrap().rate_hz, 1000.0);
            peer.arm(&mut server);
            // Native's chunk30 contains one reset iteration, then29 advancing steps.
            step(&mut server, &node, 29);
            check_frames(
                &peer.samples(&mut server, 29),
                &case["phases"][2],
                0,
                engine,
            );
            peer.rate(&mut server, id, 500.0, 500.0);
            peer.arm(&mut server);
            step(&mut server, &node, 10);
            check_frames(
                &peer.samples(&mut server, 5),
                &case["phases"][3],
                29,
                engine,
            );
        }
        assert_eq!(server.counters().imu_rates_applied, 4);
        assert_eq!(server.counters().imu_rates_ignored, 2);
        assert_eq!(server.counters().imu_rates_rejected, 0);
        peer.send("quit");
        assert_eq!(peer.line(&mut server), "BYE");
        assert!(peer.child.wait().unwrap().success());
        let stderr = peer.stderr.take().unwrap().join().unwrap();
        assert!(stderr.is_empty(), "{stderr}");
    }
}
