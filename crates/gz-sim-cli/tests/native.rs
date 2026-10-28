// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::{
    net::UdpSocket,
    path::Path,
    process::{Child, Command, Output, Stdio},
    thread,
    time::{Duration, Instant},
};

struct Process(Option<Child>);

impl Process {
    fn wait(&mut self, timeout: Duration) -> Output {
        let deadline = Instant::now() + timeout;
        let child = self.0.as_mut().unwrap();
        while child.try_wait().unwrap().is_none() {
            assert!(Instant::now() < deadline, "child {} timed out", child.id());
            thread::sleep(Duration::from_millis(10));
        }
        self.0.take().unwrap().wait_with_output().unwrap()
    }
}

impl Drop for Process {
    fn drop(&mut self) {
        if let Some(mut child) = self.0.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

#[test]
fn cpp_peer_steps_the_paused_executable_and_signals_shutdown_cleanly() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let build = Command::new("python3")
        .arg(root.join("scripts/native-peer.py"))
        .output()
        .unwrap();
    assert!(build.status.success(), "{build:?}");
    let prefix = std::env::var_os("GZ_REFERENCE_PREFIX")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| root.join("artifacts/native-reference/install"));
    for (engine, signal) in [("tpe", "-INT"), ("rapier", "-TERM")] {
        let message = UdpSocket::bind("127.0.0.1:0").unwrap();
        let service = UdpSocket::bind("127.0.0.1:0").unwrap();
        let environment = [
            (
                "GZ_PARTITION",
                format!("cli-native-{}-{engine}", std::process::id()),
            ),
            ("GZ_IP", "127.0.0.1".to_owned()),
            ("GZ_TRANSPORT_IMPLEMENTATION", "zeromq".to_owned()),
            ("GZ_TRANSPORT_TOPIC_STATISTICS", "0".to_owned()),
            (
                "GZ_DISCOVERY_MSG_PORT",
                message.local_addr().unwrap().port().to_string(),
            ),
            (
                "GZ_DISCOVERY_SRV_PORT",
                service.local_addr().unwrap().port().to_string(),
            ),
        ];
        drop((message, service));
        let configure = |command: &mut Command| {
            command
                .envs(environment.iter().map(|(key, value)| (key, value)))
                .env_remove("GZ_TRANSPORT_USERNAME")
                .env_remove("GZ_TRANSPORT_PASSWORD")
                .env_remove("GZ_RELAY")
                .env_remove("SDF_PATH")
                .env_remove("GZ_SIM_RESOURCE_PATH")
                .stdout(Stdio::piped())
                .stderr(Stdio::piped());
        };
        let mut command = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
        command
            .arg(root.join("crates/gz-sim-scene/tests/fixtures/fall.sdf"))
            .args(["--server", "--physics-engine", engine]);
        configure(&mut command);
        let mut server = Process(Some(command.spawn().unwrap()));
        let mut command = Command::new(root.join("artifacts/native-peer/clock-client"));
        command
            .arg("--root-clock")
            .env("LD_LIBRARY_PATH", prefix.join("lib"));
        configure(&mut command);
        let mut peer = Process(Some(command.spawn().unwrap()));
        let observed = peer.wait(Duration::from_secs(45));
        assert!(observed.status.success(), "{observed:?}");
        assert_eq!(
            observed.stdout,
            b"PASS native root clock pairs=3 final=2000000\n"
        );
        assert!(observed.stderr.is_empty(), "{observed:?}");
        let child = server.0.as_mut().unwrap();
        assert!(
            child.try_wait().unwrap().is_none(),
            "unlimited server exited"
        );
        assert!(
            Command::new("kill")
                .arg(signal)
                .arg(child.id().to_string())
                .status()
                .unwrap()
                .success()
        );
        let stopped = server.wait(Duration::from_secs(5));
        assert!(stopped.status.success(), "{stopped:?}");
        assert!(
            String::from_utf8_lossy(&stopped.stdout)
                .starts_with("simulation_ns=2000000 iterations=2\n")
        );
        assert!(stopped.stderr.is_empty(), "{stopped:?}");
    }
}

#[test]
fn finite_budget_counts_advancing_steps_across_pause_and_reset() {
    use gz_msgs::{
        Clock,
        msgs::{Boolean, WorldControl, WorldReset},
        prost::Message,
    };
    use gz_transport::{Config, Node};
    for engine in ["tpe", "rapier"] {
        let message = UdpSocket::bind("127.0.0.1:0").unwrap();
        let service = UdpSocket::bind("127.0.0.1:0").unwrap();
        let config = Config {
            partition: format!("cli-budget-{}-{engine}", std::process::id()),
            message_port: message.local_addr().unwrap().port(),
            service_port: service.local_addr().unwrap().port(),
            ..Config::default()
        };
        drop((message, service));
        let node = Node::new(config.clone()).unwrap();
        let clock = node
            .subscribe("/world/default/clock", "gz.msgs.Clock")
            .unwrap();
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let mut server = Process(Some(
            Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
                .arg(root.join("crates/gz-sim-scene/tests/fixtures/fall.sdf"))
                .args(["--server", "--iterations", "3", "--physics-engine", engine])
                .env("GZ_PARTITION", &config.partition)
                .env("GZ_IP", config.interface.to_string())
                .env("GZ_DISCOVERY_MSG_PORT", config.message_port.to_string())
                .env("GZ_DISCOVERY_SRV_PORT", config.service_port.to_string())
                .env("GZ_TRANSPORT_IMPLEMENTATION", "zeromq")
                .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
                .env_remove("GZ_TRANSPORT_USERNAME")
                .env_remove("GZ_TRANSPORT_PASSWORD")
                .env_remove("GZ_RELAY")
                .env_remove("SDF_PATH")
                .env_remove("GZ_SIM_RESOURCE_PATH")
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .unwrap(),
        ));
        let wait = |nanoseconds| {
            let deadline = Instant::now() + Duration::from_secs(8);
            loop {
                let bytes = clock
                    .recv_timeout(deadline.saturating_duration_since(Instant::now()))
                    .unwrap();
                let observed = Clock::decode(bytes.data.as_slice()).unwrap().sim.unwrap();
                assert_eq!(observed.sec, 0);
                if observed.nsec == nanoseconds {
                    break;
                }
                assert!(Instant::now() < deadline);
            }
        };
        let control = |request: WorldControl| {
            let (bytes, result) = node
                .request(
                    "/world/default/control",
                    "gz.msgs.WorldControl",
                    "gz.msgs.Boolean",
                    request.encode_to_vec(),
                    Duration::from_secs(5),
                )
                .unwrap();
            assert!(result && Boolean::decode(bytes.as_slice()).unwrap().data);
        };
        wait(0);
        control(WorldControl {
            pause: true,
            multi_step: 2,
            ..Default::default()
        });
        wait(2_000_000);
        control(WorldControl {
            pause: true,
            reset: Some(WorldReset {
                time_only: true,
                ..Default::default()
            }),
            ..Default::default()
        });
        wait(0);
        assert!(server.0.as_mut().unwrap().try_wait().unwrap().is_none());
        control(WorldControl {
            pause: true,
            multi_step: 2,
            ..Default::default()
        });
        let stopped = server.wait(Duration::from_secs(5));
        assert!(stopped.status.success(), "{stopped:?}");
        assert!(
            String::from_utf8_lossy(&stopped.stdout)
                .starts_with("simulation_ns=1000000 iterations=3\n"),
            "{stopped:?}"
        );
        assert!(stopped.stderr.is_empty(), "{stopped:?}");
    }
}

#[test]
fn second_termination_signal_exits_when_final_output_is_blocked() {
    use gz_msgs::{Clock, prost::Message};
    use gz_transport::{Config, Node};
    use std::{
        io::{BufRead, BufReader},
        sync::mpsc,
    };
    let directory = tempfile::TempDir::new().unwrap();
    let world = directory.path().join("large.sdf");
    let links = (0..5000).map(|index| format!("<link name='link{index}'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link>")).collect::<String>();
    std::fs::write(&world, format!("<sdf version='1.12'><world name='default'><model name='large'><static>true</static>{links}</model></world></sdf>")).unwrap();
    let message = UdpSocket::bind("127.0.0.1:0").unwrap();
    let service = UdpSocket::bind("127.0.0.1:0").unwrap();
    let config = Config {
        partition: format!("cli-output-{}", std::process::id()),
        message_port: message.local_addr().unwrap().port(),
        service_port: service.local_addr().unwrap().port(),
        ..Config::default()
    };
    drop((message, service));
    let node = Node::new(config.clone()).unwrap();
    let clock = node
        .subscribe("/world/default/clock", "gz.msgs.Clock")
        .unwrap();
    let mut server = Process(Some(
        Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
            .arg(&world)
            .args(["--server", "--physics-engine", "tpe"])
            .env("GZ_PARTITION", &config.partition)
            .env("GZ_IP", config.interface.to_string())
            .env("GZ_DISCOVERY_MSG_PORT", config.message_port.to_string())
            .env("GZ_DISCOVERY_SRV_PORT", config.service_port.to_string())
            .env("GZ_TRANSPORT_IMPLEMENTATION", "zeromq")
            .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
            .env_remove("GZ_TRANSPORT_USERNAME")
            .env_remove("GZ_TRANSPORT_PASSWORD")
            .env_remove("GZ_RELAY")
            .env_remove("SDF_PATH")
            .env_remove("GZ_SIM_RESOURCE_PATH")
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap(),
    ));
    let bytes = clock.recv_timeout(Duration::from_secs(20)).unwrap();
    let time = Clock::decode(bytes.data.as_slice()).unwrap().sim.unwrap();
    assert_eq!((time.sec, time.nsec), (0, 0));
    let child = server.0.as_mut().unwrap();
    assert!(
        Command::new("kill")
            .args(["-TERM", &child.id().to_string()])
            .status()
            .unwrap()
            .success()
    );
    let stdout = child.stdout.take().unwrap();
    let (send, receive) = mpsc::sync_channel(1);
    let reader = thread::spawn(move || {
        let mut input = BufReader::new(stdout);
        let mut line = String::new();
        let result = input.read_line(&mut line);
        let _ = send.send((result, line, input));
    });
    let (read, header, _unread_output) = receive.recv_timeout(Duration::from_secs(10)).unwrap();
    read.unwrap();
    reader.join().unwrap();
    assert_eq!(header, "simulation_ns=0 iterations=0\n");
    let child = server.0.as_mut().unwrap();
    assert!(
        child.try_wait().unwrap().is_none(),
        "large final output must fill the unread pipe"
    );
    assert!(
        Command::new("kill")
            .args(["-TERM", &child.id().to_string()])
            .status()
            .unwrap()
            .success()
    );
    let stopped = server.wait(Duration::from_secs(5));
    assert_eq!(
        stopped.status.code(),
        Some(128 + signal_hook::consts::SIGTERM)
    );
    assert!(stopped.stderr.is_empty(), "{stopped:?}");
}
