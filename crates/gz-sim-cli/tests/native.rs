// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::{
    path::Path,
    process::{Child, Command, Output},
    thread,
    time::{Duration, Instant},
};

mod support;
#[path = "../../../tests/support/zenoh_peer.rs"]
mod zenoh_peer;
use support::{Network, Samples};

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

// ZEN-01 (Root): direct root-clock election must meet this retained C++ contract.
#[test]
fn cpp_peer_steps_the_paused_executable_and_signals_shutdown_cleanly() {
    use gz_sim_proto::{msgs::Clock, prost::Message};
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let native = zenoh_peer::NativePeer::build(&root, "clock");
    for (engine, signal) in [
        ("tpe", "-INT"),
        ("rapier", "-TERM"),
        ("dart", "-INT"),
        ("mujoco", "-TERM"),
    ] {
        let mut network = Network::new();
        let mut command = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
        command
            .arg(root.join("crates/gz-sim-scene/tests/fixtures/fall.sdf"))
            .args(["--server", "--physics-engine", engine]);
        command
            .env_remove("SDF_PATH")
            .env_remove("GZ_SIM_RESOURCE_PATH");
        let mut server = Process(Some(network.spawn_server(&mut command)));
        // Fence actual listener/publication readiness before native startup. This
        // never advances the paused world or replaces the native paired-clock checks.
        let readiness = network.session();
        let clock = Samples::new(&readiness, network.key("/world/default/clock"));
        let sample = clock.recv_timeout(Duration::from_secs(8));
        let time = Clock::decode(sample.payload().to_bytes().as_ref())
            .unwrap()
            .sim
            .unwrap();
        assert_eq!((time.sec, time.nsec), (0, 0));
        drop((clock, readiness));
        let mut command = Command::new(&native.binary);
        command.arg("--root-clock");
        network.configure_native(&mut command, &native);
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
    use gz_sim_proto::{
        msgs::{Clock, WorldControl, WorldReset},
        prost::Message,
    };
    for engine in ["tpe", "rapier", "dart", "mujoco"] {
        let mut network = Network::new();
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let mut command = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
        command
            .arg(root.join("crates/gz-sim-scene/tests/fixtures/fall.sdf"))
            .args(["--server", "--iterations", "3", "--physics-engine", engine])
            .env_remove("SDF_PATH")
            .env_remove("GZ_SIM_RESOURCE_PATH");
        let mut server = Process(Some(network.spawn_server(&mut command)));
        let session = network.session();
        let clock = Samples::new(&session, network.key("/world/default/clock"));
        let wait = |nanoseconds| {
            let deadline = Instant::now() + Duration::from_secs(8);
            loop {
                let bytes = clock.recv_timeout(deadline.saturating_duration_since(Instant::now()));
                let observed = Clock::decode(bytes.payload().to_bytes().as_ref())
                    .unwrap()
                    .sim
                    .unwrap();
                assert_eq!(observed.sec, 0);
                if observed.nsec == nanoseconds {
                    break;
                }
                assert!(Instant::now() < deadline);
            }
        };
        let control = |request: WorldControl| network.control(&session, request.encode_to_vec());
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
    use gz_sim_proto::{msgs::Clock, prost::Message};
    use std::{
        io::{BufRead, BufReader},
        sync::mpsc,
    };
    let directory = tempfile::TempDir::new().unwrap();
    let world = directory.path().join("large.sdf");
    let links = (0..5000).map(|index| format!("<link name='link{index}'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link>")).collect::<String>();
    std::fs::write(&world, format!("<sdf version='1.12'><world name='default'><model name='large'><static>true</static>{links}</model></world></sdf>")).unwrap();
    let mut network = Network::new();
    let mut command = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
    command
        .arg(&world)
        .args(["--server", "--physics-engine", "tpe"])
        .env_remove("SDF_PATH")
        .env_remove("GZ_SIM_RESOURCE_PATH");
    let mut server = Process(Some(network.spawn_server(&mut command)));
    let session = network.session();
    let clock = Samples::new(&session, network.key("/world/default/clock"));
    let bytes = clock.recv_timeout(Duration::from_secs(20));
    let time = Clock::decode(bytes.payload().to_bytes().as_ref())
        .unwrap()
        .sim
        .unwrap();
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
