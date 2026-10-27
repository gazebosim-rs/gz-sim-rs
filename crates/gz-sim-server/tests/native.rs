// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{Name, PhysicsEngine, WorldPose, load_world_with_engine};
use gz_sim_server::Server;
use gz_transport::Config;
use sdformat::NoResources;
use std::{
    net::UdpSocket,
    num::NonZeroUsize,
    path::Path,
    process::{Child, Command, Stdio},
    thread,
    time::{Duration, Instant},
};

struct Peer(Child);
impl Drop for Peer {
    fn drop(&mut self) {
        if self.0.try_wait().unwrap().is_none() {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
}

fn run_native_root_clock_cases() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let build = Command::new("python3")
        .arg(root.join("scripts/native-peer.py"))
        .output()
        .unwrap();
    assert!(build.status.success(), "{build:?}");
    let prefix = std::env::var_os("GZ_REFERENCE_PREFIX")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| root.join("artifacts/native-reference/install"));
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let message = UdpSocket::bind("127.0.0.1:0").unwrap();
        let service = UdpSocket::bind("127.0.0.1:0").unwrap();
        let config = Config {
            partition: format!("native-root-{}-{engine:?}", std::process::id()),
            message_port: message.local_addr().unwrap().port(),
            service_port: service.local_addr().unwrap().port(),
            ..Config::default()
        };
        drop((message, service));
        let simulation = load_world_with_engine(
            include_str!("../../gz-sim-scene/tests/fixtures/fall.sdf"),
            "fall.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            engine,
        )
        .unwrap();
        let mut server = Server::new(
            simulation,
            "default",
            config.clone(),
            NonZeroUsize::new(4).unwrap(),
        )
        .unwrap();
        assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
        let mut peer = Peer(
            Command::new(root.join("artifacts/native-peer/clock-client"))
                .arg("--root-clock")
                .env("GZ_PARTITION", &config.partition)
                .env("GZ_IP", config.interface.to_string())
                .env("GZ_TRANSPORT_IMPLEMENTATION", "zeromq")
                .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
                .env("GZ_DISCOVERY_MULTICAST_IP", config.multicast.to_string())
                .env_remove("GZ_TRANSPORT_USERNAME")
                .env_remove("GZ_TRANSPORT_PASSWORD")
                .env_remove("GZ_RELAY")
                .env("GZ_DISCOVERY_MSG_PORT", config.message_port.to_string())
                .env("GZ_DISCOVERY_SRV_PORT", config.service_port.to_string())
                .env("LD_LIBRARY_PATH", prefix.join("lib"))
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .unwrap(),
        );
        let deadline = Instant::now() + Duration::from_secs(45);
        let status = loop {
            if let Some(status) = peer.0.try_wait().unwrap() {
                break status;
            }
            assert!(
                Instant::now() < deadline,
                "native root clock client timed out"
            );
            server.tick().unwrap();
            // Pacing is outside stepping; matching messages establish readiness.
            thread::sleep(Duration::from_millis(1));
        };
        use std::io::Read;
        let mut stdout = String::new();
        let mut stderr = String::new();
        peer.0
            .stdout
            .take()
            .unwrap()
            .read_to_string(&mut stdout)
            .unwrap();
        peer.0
            .stderr
            .take()
            .unwrap()
            .read_to_string(&mut stderr)
            .unwrap();
        assert!(
            status.success(),
            "status={status} stdout={stdout} stderr={stderr}"
        );
        assert_eq!(stdout, "PASS native root clock pairs=3 final=2000000\n");
        assert!(stderr.is_empty(), "{stderr}");
        assert_eq!(server.simulation().elapsed(), Duration::from_millis(2));
        assert!(server.simulation().is_paused());
        assert!(server.counters().root_clocks_queued >= 3);
        assert_eq!(server.counters().root_clock_failures, 0);
        assert_eq!(server.counters().clock_failures, 0);
    }
}

#[test]
fn cpp_client_controls_a_real_sdf_world_and_observes_exact_clock_time() {
    run_native_world(false, None, false, false, false, PhysicsEngine::Rapier);
    run_native_world(true, None, false, false, false, PhysicsEngine::Rapier);
    run_native_world(
        true,
        Some("robot ~imu//@:= sensor"),
        false,
        false,
        false,
        PhysicsEngine::Rapier,
    );
    run_native_world(false, None, true, false, false, PhysicsEngine::Rapier);
    run_native_world(true, None, false, true, false, PhysicsEngine::Rapier);
    run_native_world(true, None, false, false, true, PhysicsEngine::Rapier);
    run_native_world(true, None, false, false, true, PhysicsEngine::Tpe);
    run_native_schedule_world(PhysicsEngine::Rapier, false);
    run_native_schedule_world(PhysicsEngine::Tpe, false);
    run_native_schedule_world(PhysicsEngine::Rapier, true);
    run_native_schedule_world(PhysicsEngine::Tpe, true);
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for noisy in [false, true] {
            run_native_schedule_case(engine, noisy, false);
        }
    }
    run_native_root_clock_cases();
}

fn run_native_world(
    with_imu: bool,
    topic: Option<&str>,
    remove: bool,
    shared_imu: bool,
    reset: bool,
    engine: PhysicsEngine,
) {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let build = Command::new("python3")
        .arg(root.join("scripts/native-peer.py"))
        .output()
        .unwrap();
    assert!(
        build.status.success(),
        "{}",
        String::from_utf8_lossy(&build.stderr)
    );
    let message = UdpSocket::bind("127.0.0.1:0").unwrap();
    let service = UdpSocket::bind("127.0.0.1:0").unwrap();
    let config = Config {
        partition: format!("native-sim-{}", std::process::id()),
        message_port: message.local_addr().unwrap().port(),
        service_port: service.local_addr().unwrap().port(),
        namespace: if shared_imu {
            "/robot".into()
        } else {
            String::new()
        },
        ..Config::default()
    };
    drop((message, service));
    let mut xml = include_str!("../../gz-sim-scene/tests/fixtures/fall.sdf").to_owned();
    if let Some(topic) = topic {
        xml = xml.replace(
            "<always_on>true</always_on>",
            &format!("<topic>{topic}</topic><frame_id>sensor_frame</frame_id><gz_frame_id>ignored</gz_frame_id>"),
        );
    }
    if shared_imu {
        xml = xml
            .replace("<always_on>true</always_on>", "<always_on>true</always_on><topic>shared</topic><frame_id>first</frame_id>")
            .replace("</sensor>", "</sensor><sensor name='imu2' type='imu'><always_on>true</always_on><update_rate>1000</update_rate><topic>/robot/shared</topic><frame_id>second</frame_id></sensor>");
    }
    let simulation = load_world_with_engine(
        &xml,
        "fall.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        engine,
    )
    .unwrap();
    let mut server = Server::new(
        simulation,
        "default",
        config.clone(),
        NonZeroUsize::new(4).unwrap(),
    )
    .unwrap();
    if !with_imu {
        // Preserve the original unobserved-acquisition counter workload.
        server.set_interest_policy(gz_sim_server::InterestPolicy::Always);
    }
    let prefix = std::env::var_os("GZ_REFERENCE_PREFIX")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| root.join("artifacts/native-reference/install"));
    let mut command = Command::new(root.join("artifacts/native-peer/clock-client"));
    if reset {
        command.arg(if engine == PhysicsEngine::Tpe {
            "--tpe-reset"
        } else {
            "--reset"
        });
    } else if shared_imu {
        command.arg("--shared");
    } else if with_imu {
        command.arg("--imu");
        if let Some(topic) = topic {
            command.arg(topic).arg("sensor_frame");
        }
    } else if remove {
        command.arg("--remove");
    }
    let mut peer = Peer(
        command
            .env("GZ_PARTITION", &config.partition)
            .env("GZ_IP", config.interface.to_string())
            .env("GZ_TRANSPORT_IMPLEMENTATION", "zeromq")
            .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
            .env("GZ_DISCOVERY_MULTICAST_IP", config.multicast.to_string())
            .env_remove("GZ_TRANSPORT_USERNAME")
            .env_remove("GZ_TRANSPORT_PASSWORD")
            .env_remove("GZ_RELAY")
            .env("GZ_DISCOVERY_MSG_PORT", config.message_port.to_string())
            .env("GZ_DISCOVERY_SRV_PORT", config.service_port.to_string())
            .env("LD_LIBRARY_PATH", prefix.join("lib"))
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap(),
    );
    let deadline = Instant::now() + Duration::from_secs(20);
    let status = loop {
        if let Some(status) = peer.0.try_wait().unwrap() {
            break status;
        }
        assert!(Instant::now() < deadline, "native client timed out");
        server.tick().unwrap();
        // Host pacing is outside the physics step and does not establish readiness.
        thread::sleep(Duration::from_millis(1));
    };
    use std::io::Read;
    let mut stdout = String::new();
    let mut stderr = String::new();
    peer.0
        .stdout
        .take()
        .unwrap()
        .read_to_string(&mut stdout)
        .unwrap();
    peer.0
        .stderr
        .take()
        .unwrap()
        .read_to_string(&mut stderr)
        .unwrap();
    assert!(
        status.success(),
        "status={status} stdout={stdout} stderr={stderr}"
    );
    assert!(stdout.starts_with("PASS native clock/control"));
    assert!(stdout.contains("poses=4"));
    assert!(server.counters().poses_queued > 0);
    assert_eq!(server.counters().pose_failures, 0);
    assert_eq!(server.counters().invalid_pose_frames, 0);
    assert!(stderr.is_empty(), "{stderr}");
    let final_steps: u64 = stdout
        .split_whitespace()
        .find_map(|word| word.strip_prefix("final="))
        .unwrap()
        .parse()
        .unwrap();
    if reset {
        assert_eq!(final_steps, 1);
    } else if with_imu {
        assert!(final_steps >= 4);
    } else {
        assert_eq!(final_steps, 3);
    }
    assert_eq!(
        server.simulation().elapsed(),
        Duration::from_millis(final_steps)
    );
    assert!(server.simulation().is_paused());
    let body = server.simulation().world().iter_entities().find(|entity| {
        entity
            .get::<Name>()
            .is_some_and(|name| name.0 == "default::ball::body")
    });
    if remove {
        assert!(body.is_none());
        assert!(stdout.contains("removed=1"));
        assert_eq!(server.counters().removals_queued, 3);
        assert_eq!(server.counters().removals_failed, 2);
        assert_eq!(server.counters().removals_completed, 1);
    } else if engine == PhysicsEngine::Tpe {
        assert_eq!(
            body.unwrap().get::<WorldPose>().unwrap().translation[2],
            5.0
        );
    } else {
        assert!(body.unwrap().get::<WorldPose>().unwrap().translation[2] < 5.0);
    }
    let warmup: u64 = stdout
        .split_whitespace()
        .find_map(|word| word.strip_prefix("baseline="))
        .unwrap()
        .parse()
        .unwrap();
    let reconnect: u64 = stdout
        .split_whitespace()
        .find_map(|word| word.strip_prefix("reconnect="))
        .unwrap()
        .parse()
        .unwrap();
    if reset {
        assert!(stdout.contains("reset=1"));
        assert!((1..=100).contains(&reconnect));
        assert_eq!(
            server
                .simulation()
                .world()
                .resource::<gz_sim_core::UpdateInfo>()
                .epoch,
            2
        );
        assert_eq!(server.counters().controls_accepted, warmup + reconnect + 4);
        assert_eq!(server.counters().removals_queued, 3);
        assert_eq!(server.counters().removals_failed, 2);
        assert_eq!(server.counters().removals_completed, 1);
    } else {
        assert_eq!(server.counters().controls_accepted, final_steps - 2);
    }
    assert_eq!(server.counters().controls_rejected, 1);
    assert_eq!(
        server.counters().imus_queued,
        if reset {
            warmup + reconnect + 4
        } else {
            final_steps * if shared_imu { 2 } else { 1 }
        }
    );
    if shared_imu {
        assert!(stdout.contains("shared=1"));
    }
    assert_eq!(server.counters().imu_failures, 0);
    server.shutdown();
}

fn run_native_schedule_world(engine: PhysicsEngine, noisy: bool) {
    run_native_schedule_case(engine, noisy, true);
}

fn run_native_schedule_case(engine: PhysicsEngine, noisy: bool, orientation: bool) {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let build = Command::new("python3")
        .arg(root.join("scripts/native-peer.py"))
        .output()
        .unwrap();
    assert!(
        build.status.success(),
        "{}",
        String::from_utf8_lossy(&build.stderr)
    );
    let message = UdpSocket::bind("127.0.0.1:0").unwrap();
    let service = UdpSocket::bind("127.0.0.1:0").unwrap();
    let config = Config {
        partition: format!("native-schedule-{}", std::process::id()),
        message_port: message.local_addr().unwrap().port(),
        service_port: service.local_addr().unwrap().port(),
        ..Config::default()
    };
    drop((message, service));
    let mut xml = include_str!("../../gz-sim-scene/tests/fixtures/fall.sdf").replace(
        "<update_rate>1000</update_rate>",
        "<update_rate>10</update_rate>",
    );
    if noisy {
        xml = xml.replace("</sensor>", "<imu><angular_velocity><x><noise type='gaussian'><mean>0.25</mean></noise></x><y><noise type='gaussian_quantized'><mean>-2.25</mean><precision>0.5</precision></noise></y><z><noise type='gaussian'><stddev>0.1</stddev></noise></z></angular_velocity></imu></sensor>");
    }
    if !orientation {
        xml = if noisy {
            xml.replace(
                "</imu>",
                "<enable_orientation>false</enable_orientation></imu>",
            )
        } else {
            xml.replace(
                "</sensor>",
                "<imu><enable_orientation>false</enable_orientation></imu></sensor>",
            )
        };
    }
    let simulation = load_world_with_engine(
        &xml,
        "schedule.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        engine,
    )
    .unwrap();
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
    let mut peer = Peer(
        Command::new(root.join("artifacts/native-peer/clock-client"))
            .arg(match (engine, noisy, orientation) {
                (PhysicsEngine::Tpe, true, false) => "--tpe-disabled-noisy-schedule",
                (PhysicsEngine::Rapier, true, false) => "--disabled-noisy-schedule",
                (PhysicsEngine::Tpe, false, false) => "--tpe-disabled-schedule",
                (PhysicsEngine::Rapier, false, false) => "--disabled-schedule",
                (PhysicsEngine::Tpe, true, true) => "--tpe-noisy-schedule",
                (PhysicsEngine::Rapier, true, true) => "--noisy-schedule",
                (PhysicsEngine::Tpe, false, true) => "--tpe-schedule",
                (PhysicsEngine::Rapier, false, true) => "--schedule",
            })
            .env("GZ_PARTITION", &config.partition)
            .env("GZ_IP", config.interface.to_string())
            .env("GZ_TRANSPORT_IMPLEMENTATION", "zeromq")
            .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
            .env("GZ_DISCOVERY_MULTICAST_IP", config.multicast.to_string())
            .env_remove("GZ_TRANSPORT_USERNAME")
            .env_remove("GZ_TRANSPORT_PASSWORD")
            .env_remove("GZ_RELAY")
            .env("GZ_DISCOVERY_MSG_PORT", config.message_port.to_string())
            .env("GZ_DISCOVERY_SRV_PORT", config.service_port.to_string())
            .env("LD_LIBRARY_PATH", prefix.join("lib"))
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap(),
    );
    let deadline = Instant::now() + Duration::from_secs(20);
    let status = loop {
        if let Some(status) = peer.0.try_wait().unwrap() {
            break status;
        }
        assert!(
            Instant::now() < deadline,
            "native schedule client timed out"
        );
        server.tick().unwrap();
        thread::sleep(Duration::from_millis(1));
    };
    use std::io::Read;
    let mut stdout = String::new();
    let mut stderr = String::new();
    peer.0
        .stdout
        .take()
        .unwrap()
        .read_to_string(&mut stdout)
        .unwrap();
    peer.0
        .stderr
        .take()
        .unwrap()
        .read_to_string(&mut stderr)
        .unwrap();
    assert!(
        status.success(),
        "{engine:?} status={status} stdout={stdout} stderr={stderr}"
    );
    assert!(stderr.is_empty(), "{stderr}");
    assert!(stdout.starts_with(if noisy {
        "PASS native noisy schedule"
    } else {
        "PASS native schedule"
    }));
    let warmup: u64 = stdout
        .split_whitespace()
        .find_map(|word| word.strip_prefix("warmup="))
        .unwrap()
        .parse()
        .unwrap();
    assert!((1..=20).contains(&warmup));
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(1));
    assert!(server.simulation().is_paused());
    assert_eq!(
        server
            .simulation()
            .world()
            .resource::<gz_sim_core::UpdateInfo>()
            .epoch,
        2
    );
    assert_eq!(server.counters().controls_accepted, warmup + 6);
    assert_eq!(server.counters().controls_rejected, 0);
    assert_eq!(server.counters().imus_queued, warmup + 4);
    assert_eq!(server.counters().imu_failures, 0);
    assert_eq!(server.counters().invalid_imu_samples, 0);
    server.shutdown();
}
