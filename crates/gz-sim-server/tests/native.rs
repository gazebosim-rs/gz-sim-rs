// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{Name, PhysicsEngine, WorldPose, load_world_with_engine};
use gz_sim_server::Server;
use gz_sim_server::ServerOptions;
use sdformat::NoResources;
use std::{
    net::{SocketAddr, TcpListener},
    num::NonZeroUsize,
    path::Path,
    process::{Child, Command, Stdio},
    sync::OnceLock,
    thread,
    time::{Duration, Instant},
};

use zenoh::Wait;

#[path = "../../../tests/support/zenoh_peer.rs"]
mod zenoh_peer;

struct Network {
    directory: tempfile::TempDir,
    clients: zenoh::Config,
    router: zenoh::Session,
    address: SocketAddr,
}
impl Network {
    fn new() -> Self {
        Self::from_reservation(TcpListener::bind("127.0.0.1:0").unwrap())
    }
    fn from_reservation(reservation: TcpListener) -> Self {
        let address = reservation.local_addr().unwrap();
        let endpoint = format!("tcp/{address}");
        let scouting =
            serde_json::json!({"multicast":{"enabled":false},"gossip":{"enabled":false}});
        let router = serde_json::json!({"mode":"router","listen":{"endpoints":[endpoint]},"scouting":scouting});
        let client = serde_json::json!({
            "mode":"client","listen":{"endpoints":[]},
            "connect":{"endpoints":[endpoint],"timeout_ms":2000},"scouting":scouting,
            "transport":{"shared_memory":{"enabled":false}}
        });
        let clients = zenoh::Config::from_json5(&client.to_string()).unwrap();
        let directory = tempfile::tempdir().unwrap();
        std::fs::write(
            directory.path().join("client.json"),
            serde_json::to_vec(&client).unwrap(),
        )
        .unwrap();
        // Zenoh cannot inherit this socket. A stolen port fails the bind;
        // neither side falls back to multicast or a default endpoint.
        drop(reservation);
        let router = zenoh::open(zenoh::Config::from_json5(&router.to_string()).unwrap())
            .wait()
            .unwrap();
        Self {
            directory,
            clients,
            router,
            address,
        }
    }
    fn server(&self, simulation: gz_sim_core::Simulation, config: &ServerOptions) -> Server {
        Server::with_zenoh_config(
            simulation,
            "default",
            config.clone(),
            self.clients.clone(),
            NonZeroUsize::new(4).unwrap(),
        )
        .unwrap()
    }
    fn configure_native<'a>(
        &self,
        command: &'a mut Command,
        config: &ServerOptions,
    ) -> &'a mut Command {
        native_peer().configure_runtime(command);
        command
            .env("GZ_PARTITION", &config.partition)
            .env("GZ_IP", "127.0.0.1")
            .env("GZ_TRANSPORT_IMPLEMENTATION", "zenoh")
            .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
            .env("ZENOH_CONFIG", self.directory.path().join("client.json"))
            .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
            .env_remove("GZ_DISCOVERY_MSG_PORT")
            .env_remove("GZ_DISCOVERY_SRV_PORT")
            .env_remove("GZ_DISCOVERY_MULTICAST_IP")
            .env_remove("GZ_TRANSPORT_USERNAME")
            .env_remove("GZ_TRANSPORT_PASSWORD")
            .env_remove("GZ_RELAY")
            .env_remove("LD_PRELOAD")
            .env_remove("LD_AUDIT")
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
    }
}

fn native_peer() -> &'static zenoh_peer::NativePeer {
    static PEER: OnceLock<zenoh_peer::NativePeer> = OnceLock::new();
    PEER.get_or_init(|| {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        zenoh_peer::NativePeer::build(&root, "clock")
    })
}

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
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Tpe,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        let network = Network::new();
        let config = ServerOptions {
            partition: format!("native-root-{}-{engine:?}", std::process::id()),
            ..ServerOptions::default()
        };
        let simulation = load_world_with_engine(
            include_str!("../../gz-sim-scene/tests/fixtures/fall.sdf"),
            "fall.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            engine,
        )
        .unwrap();
        let mut server = network.server(simulation, &config);
        assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
        let mut command = Command::new(&native_peer().binary);
        command.arg("--root-clock");
        let mut peer = Peer(
            network
                .configure_native(&mut command, &config)
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
            server.update().unwrap();
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
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        run_native_world(false, None, false, false, false, engine);
        run_native_world(true, None, false, false, false, engine);
        run_native_world(
            true,
            Some("robot ~imu//@:= sensor"),
            false,
            false,
            false,
            engine,
        );
        run_native_world(false, None, true, false, false, engine);
        run_native_world(true, None, false, true, false, engine);
        run_native_world(true, None, false, false, true, engine);
    }
    run_native_world(true, None, false, false, true, PhysicsEngine::Tpe);
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Tpe,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        for noisy in [false, true] {
            run_native_schedule_world(engine, noisy);
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
    let network = Network::new();
    let config = ServerOptions {
        partition: format!("native-sim-{}", std::process::id()),
        namespace: if shared_imu {
            "/robot".into()
        } else {
            String::new()
        },
        ..ServerOptions::default()
    };
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
    let mut server = network.server(simulation, &config);
    if !with_imu {
        // Preserve the original unobserved-acquisition counter workload.
        server.set_interest_policy(gz_sim_server::InterestPolicy::Always);
    }

    let mut command = Command::new(&native_peer().binary);
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
        network
            .configure_native(&mut command, &config)
            .spawn()
            .unwrap(),
    );
    let deadline = Instant::now() + Duration::from_secs(20);
    let status = loop {
        if let Some(status) = peer.0.try_wait().unwrap() {
            break status;
        }
        assert!(Instant::now() < deadline, "native client timed out");
        server.update().unwrap();
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
    let network = Network::new();
    let config = ServerOptions {
        partition: format!("native-schedule-{}", std::process::id()),
        ..ServerOptions::default()
    };
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
    let mut server = network.server(simulation, &config);

    let mut command = Command::new(&native_peer().binary);
    command.arg(match (engine == PhysicsEngine::Tpe, noisy, orientation) {
        (true, true, false) => "--tpe-disabled-noisy-schedule",
        (false, true, false) => "--disabled-noisy-schedule",
        (true, false, false) => "--tpe-disabled-schedule",
        (false, false, false) => "--disabled-schedule",
        (true, true, true) => "--tpe-noisy-schedule",
        (false, true, true) => "--noisy-schedule",
        (true, false, true) => "--tpe-schedule",
        (false, false, true) => "--schedule",
    });
    let mut peer = Peer(
        network
            .configure_native(&mut command, &config)
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
        server.update().unwrap();
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

// D28 retires reusable UDP discovery sockets. TCP listener ownership remains
// explicit: reserve, hand off without fallback, reject theft, then release.
#[test]
fn tcp_reservation_handoff_keeps_the_listener_exclusive_until_shutdown() {
    fn listener(address: SocketAddr) -> std::io::Result<TcpListener> {
        let socket = socket2::Socket::new(socket2::Domain::IPV4, socket2::Type::STREAM, None)?;
        // Permit normal TCP TIME_WAIT after close, without SO_REUSEPORT.
        socket.set_reuse_address(true)?;
        socket.bind(&address.into())?;
        socket.listen(128)?;
        Ok(socket.into())
    }
    let reservation = listener("127.0.0.1:0".parse().unwrap()).unwrap();
    let address = reservation.local_addr().unwrap();
    assert_eq!(
        listener(address).unwrap_err().kind(),
        std::io::ErrorKind::AddrInUse
    );
    let network = Network::from_reservation(reservation);
    assert_eq!(network.address, address);
    let config = ServerOptions {
        partition: format!("reserved-tcp-{}", std::process::id()),
        ..ServerOptions::default()
    };
    let simulation = gz_sim_core::Simulation::new(Duration::from_millis(1)).unwrap();
    let mut server = network.server(simulation, &config);
    server.update().unwrap();
    assert_eq!(
        listener(address).unwrap_err().kind(),
        std::io::ErrorKind::AddrInUse
    );
    server.shutdown();
    drop(server);
    assert_eq!(
        listener(address).unwrap_err().kind(),
        std::io::ErrorKind::AddrInUse
    );
    network.router.close().wait().unwrap();
    drop(network);
    let replacement = listener(address).unwrap();
    assert_eq!(replacement.local_addr().unwrap(), address);
}

fn run_native_light_poses(xml: &str, case: &str, engine: PhysicsEngine) {
    use gz_sim_core::{EntityId, entity_by_id};
    use gz_sim_proto::prost::Message;
    use gz_sim_scene::{EntityKind, LocalName};
    use std::collections::BTreeMap;
    use std::io::Read;
    let network = Network::new();
    let config = ServerOptions {
        partition: format!("native-light-{}-{engine:?}-{case}", std::process::id()),
        ..ServerOptions::default()
    };
    let simulation = load_world_with_engine(
        xml,
        "native-light-poses.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        engine,
    )
    .unwrap();
    let owned_ids = |simulation: &gz_sim_core::Simulation| {
        simulation
            .world()
            .iter_entities()
            .filter(|entity| {
                matches!(
                    entity.get::<EntityKind>(),
                    Some(EntityKind::Light | EntityKind::LightVisual)
                )
            })
            .map(|entity| {
                let id = entity.get::<EntityId>().unwrap().get();
                assert_eq!(entity_by_id(simulation.world(), id), Some(entity.id()));
                (
                    entity.get::<LocalName>().unwrap().0.clone(),
                    u32::try_from(id).unwrap(),
                )
            })
            .collect::<BTreeMap<_, _>>()
    };
    let expected = owned_ids(&simulation);
    assert_eq!(
        expected.len(),
        match case {
            "twelve" => 24,
            "empty" => 0,
            _ => 2,
        }
    );
    let output = native_peer()
        .binary
        .parent()
        .unwrap()
        .join("light-pose-peer");
    std::fs::create_dir_all(&output).unwrap();
    let output = tempfile::Builder::new()
        .prefix(&format!("{case}-{engine:?}-"))
        .tempdir_in(output)
        .unwrap()
        .keep();
    std::fs::write(
        output.join("expected-ids.json"),
        serde_json::to_vec_pretty(&expected).unwrap(),
    )
    .unwrap();
    let mut server = network.server(simulation, &config);
    let mut command = Command::new(&native_peer().binary);
    command.args(["--light-poses", case]).arg(&output);
    let mut peer = Peer(
        network
            .configure_native(&mut command, &config)
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
            "native Light pose client timed out: {}",
            output.display()
        );
        server.update().unwrap();
        thread::sleep(Duration::from_millis(1));
    };
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
    std::fs::write(output.join("stdout"), &stdout).unwrap();
    std::fs::write(output.join("stderr"), &stderr).unwrap();
    assert!(
        status.success(),
        "status={status} stdout={stdout} stderr={stderr} output={}",
        output.display()
    );
    assert!(stderr.is_empty(), "{stderr}");
    let summary = format!(
        "PASS native light poses case={case} rows={} first=1000000 reset=0",
        expected.len()
    );
    let mut received = BTreeMap::new();
    for line in stdout.lines() {
        if line == summary {
            continue;
        }
        let words: Vec<_> = line.split_whitespace().collect();
        assert_eq!(words.len(), 3, "{line}");
        assert_eq!(words[0], "ID");
        assert!(
            received
                .insert(words[1].to_owned(), words[2].parse::<u32>().unwrap())
                .is_none()
        );
    }
    assert_eq!(stdout.lines().filter(|line| *line == summary).count(), 1);
    assert_eq!(received, expected);
    for (phase, nanos) in [("initial", 0), ("step", 1_000_000), ("reset", 0)] {
        let bytes = std::fs::read(output.join(format!("{phase}.pose_v.pb"))).unwrap();
        let frame = gz_sim_proto::msgs::PoseV::decode(bytes.as_slice()).unwrap();
        let stamp = frame.header.unwrap().stamp.unwrap();
        assert_eq!((stamp.sec, stamp.nsec), (0, nanos));
        assert_eq!(frame.pose.len(), expected.len());
        assert_eq!(
            frame
                .pose
                .into_iter()
                .map(|pose| (pose.name, pose.id))
                .collect::<BTreeMap<_, _>>(),
            expected
        );
    }
    assert_eq!(owned_ids(server.simulation()), expected);
    assert_eq!(server.simulation().elapsed(), Duration::ZERO);
    assert!(server.simulation().is_paused());
    assert!(server.counters().poses_queued >= 3);
    assert_eq!(server.counters().pose_failures, 0);
    assert_eq!(server.counters().invalid_pose_frames, 0);
    println!(
        "verified Light pose peer {case} {engine:?}: {}",
        output.display()
    );
}

#[test]
fn cpp_client_observes_light_pose_groups_owned_ids_and_loaded_reset() {
    let xml = include_str!("../../gz-sim-scene/tests/fixtures/native_light_visuals.sdf");
    for engine in [
        PhysicsEngine::Tpe,
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        run_native_light_poses(xml, "twelve", engine);
    }
    run_native_light_poses(
        "<sdf version='1.12'><world name='default'/></sdf>",
        "empty",
        PhysicsEngine::Tpe,
    );
    run_native_light_poses(
        "<sdf version='1.12'><world name='default'><light name='sun' type='directional'><light_on>false</light_on></light></world></sdf>",
        "sun-off",
        PhysicsEngine::Tpe,
    );
    run_native_light_poses(
        "<sdf version='1.12'><world name='default'><light name='sun' type='directional'/></world></sdf>",
        "sun-default",
        PhysicsEngine::Tpe,
    );
}

// Concrete B03 wire slice; native delivery and native physics expectations are separate.
#[test]
fn cpp_peer_controls_loaded_b03_and_receives_complete_pose_groups_through_reset() {
    use bevy_ecs::hierarchy::ChildOf;
    use gz_sim_core::{EntityId, Simulation, entity_by_id};
    use gz_sim_proto::{msgs, prost::Message};
    use gz_sim_scene::{EntityKind, LocalName, LocalPose, load_world_with_sdf_step};
    use std::collections::{BTreeMap, BTreeSet};
    use std::io::Write;

    const XML: &str = include_str!("../../gz-sim-scene/tests/fixtures/b03_double_pendulum.sdf");
    const FRESH: &[u8] =
        include_bytes!("../../gz-sim-scene/tests/fixtures/b03_world_fresh_native.json");
    const PHASES: [(&str, u64, u64); 6] = [
        ("initial", 0, 0),
        ("step1", 0, 1),
        ("step10", 0, 10),
        ("step100", 0, 100),
        ("reset", 1, 0),
        ("reset_step1", 1, 1),
    ];
    type Row = (u32, u32, u32, String, String, [f64; 7]);
    fn rows(sim: &Simulation) -> BTreeMap<u32, Row> {
        sim.world()
            .iter_entities()
            .filter_map(|entity| {
                let group = match entity.get::<EntityKind>()? {
                    EntityKind::Model => 0,
                    EntityKind::Link => 1,
                    EntityKind::Visual | EntityKind::LightVisual => 2,
                    EntityKind::Light => 3,
                    _ => return None,
                };
                let id = entity.get::<EntityId>().unwrap().get();
                assert_eq!(entity_by_id(sim.world(), id), Some(entity.id()));
                let parent = sim
                    .world()
                    .get_entity(entity.get::<ChildOf>().unwrap().parent())
                    .unwrap();
                let parent_id = if parent.get::<EntityKind>() == Some(&EntityKind::World) {
                    0
                } else {
                    u32::try_from(parent.get::<EntityId>().unwrap().get()).unwrap()
                };
                let pose = entity.get::<LocalPose>().unwrap().0;
                let values: [f64; 7] = pose
                    .translation
                    .into_iter()
                    .chain(pose.rotation)
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap();
                assert!(values.iter().all(|v| v.is_finite()));
                let id = u32::try_from(id).unwrap();
                Some((
                    id,
                    (
                        group,
                        id,
                        parent_id,
                        entity.get::<LocalName>().unwrap().0.clone(),
                        entity.get::<Name>().unwrap().0.clone(),
                        values,
                    ),
                ))
            })
            .collect()
    }
    fn checksum(bytes: &[u8]) -> String {
        let mut hash = Command::new("sha256sum")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        hash.stdin.take().unwrap().write_all(bytes).unwrap();
        let output = hash.wait_with_output().unwrap();
        assert!(output.status.success() && output.stderr.is_empty());
        String::from_utf8(output.stdout)
            .unwrap()
            .split_whitespace()
            .next()
            .unwrap()
            .to_owned()
    }
    assert_eq!(
        checksum(FRESH),
        "22c9bf024638fdfac70c366596ecfd359049fe2f53db5486d221168b5402b4ed"
    );
    assert_eq!(
        checksum(XML.as_bytes()),
        "24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa"
    );
    let native: serde_json::Value = serde_json::from_slice(FRESH).unwrap();
    assert_eq!(
        native["xml_sha256"],
        "24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa"
    );
    assert_eq!(native["body_fields"].as_array().unwrap().len(), 22);
    let simulation =
        load_world_with_sdf_step(XML, "b03.sdf", "default", &NoResources, PhysicsEngine::Dart)
            .unwrap();
    assert!(simulation.is_paused());
    let initial = rows(&simulation);
    let mut expected_names: BTreeSet<String> = [
        "default::ground_plane",
        "default::ground_plane::link",
        "default::ground_plane::link::visual",
        "default::sun",
        "default::sun::sunVisual",
    ]
    .into_iter()
    .map(str::to_owned)
    .collect();
    for model in ["double_pendulum_with_base0", "double_pendulum_with_base1"] {
        let prefix = format!("default::{model}");
        expected_names.insert(prefix.clone());
        for (link, visuals) in [
            ("base", &["vis_plate_on_ground", "vis_pole"][..]),
            (
                "upper_link",
                &["vis_upper_joint", "vis_lower_joint", "vis_cylinder"][..],
            ),
            ("lower_link", &["vis_lower_joint", "vis_cylinder"][..]),
        ] {
            expected_names.insert(format!("{prefix}::{link}"));
            for visual in visuals {
                expected_names.insert(format!("{prefix}::{link}::{visual}"));
            }
        }
    }
    assert_eq!(
        initial
            .values()
            .map(|r| r.4.clone())
            .collect::<BTreeSet<_>>(),
        expected_names
    );
    assert_eq!(initial.len(), 27);
    for (group, count) in [(0, 3), (1, 7), (2, 16), (3, 1)] {
        assert_eq!(initial.values().filter(|r| r.0 == group).count(), count);
    }
    let output = native_peer()
        .binary
        .parent()
        .unwrap()
        .join("b03-world-peer");
    std::fs::create_dir_all(&output).unwrap();
    let output = tempfile::Builder::new()
        .prefix("loaded-")
        .tempdir_in(output)
        .unwrap()
        .keep();
    std::fs::write(output.join("input.sdf"), XML).unwrap();
    std::fs::write(output.join("native-fixture.json"), FRESH).unwrap();
    let mut manifest = std::fs::File::create(output.join("entities.txt")).unwrap();
    for (group, id, parent, local, name, _) in initial.values() {
        writeln!(manifest, "{group} {id} {parent} {local:?} {name:?}").unwrap();
    }
    let mut targets = std::fs::File::create(output.join("native-body-world.txt")).unwrap();
    let frames = native["frames"].as_array().unwrap();
    assert_eq!(
        frames
            .iter()
            .map(|f| f["tick"].as_u64().unwrap())
            .collect::<Vec<_>>(),
        [1, 10, 100]
    );
    for frame in frames {
        let tick = frame["tick"].as_u64().unwrap();
        assert_eq!(frame["time_ns"].as_u64(), Some(tick * 1_000_000));
        let bodies = frame["bodies"].as_array().unwrap();
        assert_eq!(bodies.len(), 7);
        assert_eq!(
            bodies
                .iter()
                .map(|b| b["name"].as_str().unwrap().to_owned())
                .collect::<BTreeSet<_>>(),
            initial
                .values()
                .filter(|r| r.0 == 1)
                .map(|r| r.4.clone())
                .collect::<BTreeSet<_>>()
        );
        for body in bodies {
            write!(targets, "{tick} {:?}", body["name"].as_str().unwrap()).unwrap();
            let values = body["values"].as_array().unwrap();
            assert_eq!(values.len(), 22);
            for value in values.iter().take(7) {
                let value = value.as_f64().unwrap();
                assert!(value.is_finite());
                write!(targets, " {value:.17e}").unwrap();
            }
            writeln!(targets).unwrap();
        }
    }
    drop(manifest);
    drop(targets);
    let network = Network::new();
    let config = ServerOptions {
        partition: format!("b03-native-{}-{}", std::process::id(), uuid::Uuid::new_v4()),
        ..ServerOptions::default()
    };
    let mut server = network.server(simulation, &config);
    // This existing publication setting changes wall-time throttling, not SDF dt or physics.
    server.set_pose_publication_period(Duration::ZERO);
    let mut command = Command::new(&native_peer().binary);
    command.arg("--b03-world").arg(&output);
    // Retain actual child writes even if an update, assertion or deadline panics.
    // File-backed streams also avoid pipe backpressure hiding a native failure.
    let stdout = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output.join("stdout"))
        .unwrap();
    let stderr = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output.join("stderr"))
        .unwrap();
    println!("B03 native peer evidence: {}", output.display());
    let mut peer = Peer(
        network
            .configure_native(&mut command, &config)
            .stdout(Stdio::from(stdout))
            .stderr(Stdio::from(stderr))
            .spawn()
            .unwrap(),
    );
    let mut snapshots = BTreeMap::from([((0, 0), initial.clone())]);
    let mut advanced = BTreeSet::new();
    let deadline = Instant::now() + Duration::from_secs(45);
    let status = loop {
        if let Some(status) = peer.0.try_wait().unwrap() {
            break status;
        }
        assert!(
            Instant::now() < deadline,
            "native B03 peer timed out: {}",
            output.display()
        );
        let info = server.update().unwrap();
        assert!(info.epoch <= 1);
        assert!(info.sim_time <= Duration::from_millis(if info.epoch == 0 { 100 } else { 1 }));
        if info.dt != Duration::ZERO {
            assert_eq!(info.dt, Duration::from_millis(1));
            assert!(advanced.insert((info.epoch, info.sim_time.as_millis() as u64)));
        }
        let current = rows(server.simulation());
        assert_eq!(
            current
                .iter()
                .map(|(id, r)| (*id, (r.0, r.1, r.2, r.3.clone(), r.4.clone())))
                .collect::<BTreeMap<_, _>>(),
            initial
                .iter()
                .map(|(id, r)| (*id, (r.0, r.1, r.2, r.3.clone(), r.4.clone())))
                .collect::<BTreeMap<_, _>>()
        );
        if [0, 1, 10, 100].contains(&(info.sim_time.as_millis() as u64)) {
            snapshots
                .entry((info.epoch, info.sim_time.as_millis() as u64))
                .or_insert(current);
        }
        thread::sleep(Duration::from_millis(1));
    };
    let stdout = std::fs::read_to_string(output.join("stdout")).unwrap();
    let stderr = std::fs::read_to_string(output.join("stderr")).unwrap();
    assert!(
        status.success(),
        "{status}: {stdout} {stderr}; {}",
        output.display()
    );
    assert!(stderr.is_empty(), "{stderr}");
    assert_eq!(
        stdout,
        "PASS native B03 groups=3,7,16,1 poses=27 phases=6 controls=5 reset=0 final=1000000\n"
    );
    for (phase, epoch, tick) in PHASES {
        let poses = msgs::PoseV::decode(
            std::fs::read(output.join(format!("{phase}.pose_v.pb")))
                .unwrap()
                .as_slice(),
        )
        .unwrap();
        let clock = msgs::Clock::decode(
            std::fs::read(output.join(format!("{phase}.clock.pb")))
                .unwrap()
                .as_slice(),
        )
        .unwrap();
        let stamp = poses.header.as_ref().unwrap().stamp.as_ref().unwrap();
        assert_eq!(
            (stamp.sec, stamp.nsec),
            (0, i32::try_from(tick * 1_000_000).unwrap())
        );
        let sim = clock.sim.unwrap();
        assert_eq!((sim.sec, sim.nsec), (stamp.sec, stamp.nsec));
        if phase == "reset" {
            let real = clock.real.unwrap();
            assert_eq!((real.sec, real.nsec), (0, 0));
        }
        assert_eq!(poses.pose.len(), 27);
        let expected = &snapshots[&(epoch, tick)];
        let mut delivered = BTreeSet::new();
        for pose in poses.pose {
            assert!(delivered.insert(pose.id));
            let row = &expected[&pose.id];
            assert_eq!(pose.name, row.3);
            let p = pose.position.unwrap();
            let q = pose.orientation.unwrap();
            for (actual, target) in [p.x, p.y, p.z, q.x, q.y, q.z, q.w].into_iter().zip(row.5) {
                assert!(actual.is_finite() && (actual - target).abs() <= 1e-12);
            }
        }
        assert_eq!(delivered, initial.keys().copied().collect());
    }
    assert_eq!(
        advanced,
        (1..=100).map(|t| (0, t)).chain([(1, 1)]).collect()
    );
    assert_eq!(rows(server.simulation()), snapshots[&(1, 1)]);
    assert_eq!(server.simulation().epoch(), 1);
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(1));
    assert!(server.simulation().is_paused());
    assert_eq!(server.pending_steps(), 0);
    let counters = server.counters();
    assert_eq!(counters.controls_accepted, 5);
    assert_eq!(counters.controls_rejected, 0);
    assert_eq!(counters.response_failures, 0);
    assert_eq!(counters.clock_failures, 0);
    assert_eq!(counters.pose_failures, 0);
    assert_eq!(counters.invalid_pose_frames, 0);
    assert_eq!(counters.imus_queued, 0);
    server.shutdown();
    assert!(matches!(
        server.update(),
        Err(gz_sim_server::Error::WorldClosed)
    ));
    println!(
        "verified loaded B03 native clock/Pose_V/control/reset peer: {}",
        output.display()
    );
}
