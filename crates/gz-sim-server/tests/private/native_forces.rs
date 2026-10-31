// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::Server;
use gz_sim_core::EntityId;
use gz_sim_scene::{JointForceCommand, JointSample, Name, PhysicsEngine, load_world_with_sdf_step};
use gz_transport::Config;
use sdformat::NoResources;
use std::{
    io::{BufRead, BufReader, Read, Write},
    net::UdpSocket,
    num::NonZeroUsize,
    path::Path,
    process::{Child, Command, Stdio},
    sync::mpsc,
    thread,
    time::{Duration, Instant},
};
const XML: &str = include_str!("../fixtures/joint_force.sdf");
const PLUGIN: &str = "<plugin filename='gz-sim-apply-joint-force-system' name='gz::sim::systems::ApplyJointForce'><joint_name>hinge</joint_name></plugin>";

fn socket() -> UdpSocket {
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
    stdout: Option<thread::JoinHandle<()>>,
    stderr: Option<thread::JoinHandle<String>>,
}
impl Peer {
    fn start(binary: &str, config: &Config, topic: &str) -> Self {
        let prefix = std::env::var_os("GZ_REFERENCE_PREFIX")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| {
                Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../artifacts/native-reference/install")
            });
        let mut child = Command::new(binary)
            .arg(topic)
            .env("GZ_PARTITION", &config.partition)
            .env("GZ_DISCOVERY_MSG_PORT", config.message_port.to_string())
            .env("GZ_DISCOVERY_SRV_PORT", config.service_port.to_string())
            .env("GZ_IP", "127.0.0.1")
            // Native SDK dependencies are transitive; executable RUNPATH is not.
            .env("LD_LIBRARY_PATH", prefix.join("lib"))
            .env_remove("GZ_TRANSPORT_TOPIC_STATISTICS")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let out = child.stdout.take().unwrap();
        let mut err = child.stderr.take().unwrap();
        let (tx, lines) = mpsc::sync_channel(16);
        let stdout = thread::spawn(move || {
            for line in BufReader::new(out).lines() {
                if tx.try_send(line.unwrap()).is_err() {
                    break;
                }
            }
        });
        let stderr = thread::spawn(move || {
            let mut s = String::new();
            err.read_to_string(&mut s).unwrap();
            s
        });
        Self {
            child,
            lines,
            stdout: Some(stdout),
            stderr: Some(stderr),
        }
    }
    fn line(&self, expected: &str) {
        assert_eq!(
            self.lines.recv_timeout(Duration::from_secs(8)).unwrap(),
            expected
        );
    }
    fn send(&mut self, text: &str) {
        writeln!(self.child.stdin.as_mut().unwrap(), "{text}").unwrap();
        self.line("SENT");
    }
    fn finish(&mut self) {
        writeln!(self.child.stdin.as_mut().unwrap(), "quit").unwrap();
        self.line("DONE");
        assert!(self.child.wait().unwrap().success());
    }
}
impl Drop for Peer {
    fn drop(&mut self) {
        if self.child.try_wait().unwrap().is_none() {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
        if let Some(t) = self.stdout.take() {
            t.join().unwrap();
        }
        if let Some(t) = self.stderr.take() {
            let text = t.join().unwrap();
            if thread::panicking() {
                eprint!("{text}");
            } else {
                assert!(text.is_empty(), "{text}");
            }
        }
    }
}
fn received(server: &mut Server, count: u64) {
    let deadline = Instant::now() + Duration::from_secs(3);
    while server.counters().force_messages_received < count {
        server.tick().unwrap();
        assert!(Instant::now() < deadline, "force timeout");
        thread::sleep(Duration::from_millis(1));
    }
}
fn native_client() -> &'static str {
    static CLIENT: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    CLIENT
        .get_or_init(|| {
            let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
            let build = Command::new("python3")
                .arg(root.join("scripts/native-peer.py"))
                .args(["--client", "force"])
                .output()
                .unwrap();
            assert!(
                build.status.success(),
                "{}",
                String::from_utf8_lossy(&build.stderr)
            );
            let binary = String::from_utf8(build.stdout).unwrap();
            binary.trim().to_owned()
        })
        .as_str()
}

#[test]
fn real_cpp_publisher_controls_loaded_joint_force_on_all_dynamic_engines() {
    let binary = native_client();
    for (index, engine) in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ]
    .into_iter()
    .enumerate()
    {
        let msg = socket();
        let srv = socket();
        let config = Config {
            partition: format!("force-cpp-{}-{index}", std::process::id()),
            message_port: msg.local_addr().unwrap().port(),
            service_port: srv.local_addr().unwrap().port(),
            ..Config::default()
        };
        let source = XML.replace("</model>", &format!("{PLUGIN}</model>"));
        let sim = load_world_with_sdf_step(&source, "force.sdf", "default", &NoResources, engine)
            .unwrap();
        let joint = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::rotor::hinge")
            })
            .unwrap()
            .id();
        let id = sim.world().get::<EntityId>(joint).unwrap().get();
        let mut server = Server::new(
            sim,
            "default",
            config.clone(),
            NonZeroUsize::new(2).unwrap(),
        )
        .unwrap();
        let mut peer = Peer::start(binary, &config, "/model/rotor/joint/hinge/cmd_force");
        peer.line("READY");
        for _ in 0..10 {
            server.tick().unwrap();
        }
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            0.0
        );
        peer.send("0.001");
        received(&mut server, 1);
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            0.001
        );
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointSample>(joint)
                .unwrap()
                .state
                .velocity,
            0.0
        );
        server.set_paused(false);
        for _ in 0..10 {
            server.tick().unwrap();
        }
        server.set_paused(true);
        let velocity = server
            .simulation()
            .world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .velocity;
        assert!((velocity - 0.000625).abs() < 1e-12, "{engine:?}:{velocity}");
        peer.send("nan");
        received(&mut server, 2);
        assert_eq!(server.counters().force_messages_rejected, 1);
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            0.001
        );
        peer.send("default");
        received(&mut server, 3);
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            0.0
        );
        assert_eq!(server.counters().joint_forces_applied, 2);
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<EntityId>(joint)
                .unwrap()
                .get(),
            id
        );
        peer.send("0.002");
        received(&mut server, 4);
        peer.finish();
        server.reset_scene().unwrap();
        let restored = gz_sim_core::entity_by_id(server.simulation().world(), id).unwrap();
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(restored)
                .unwrap()
                .torque(),
            0.0
        );
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointSample>(restored)
                .unwrap()
                .state
                .velocity,
            0.0
        );
        server.set_paused(false);
        for _ in 0..3 {
            server.tick().unwrap();
        }
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointSample>(restored)
                .unwrap()
                .state
                .velocity,
            0.0
        );
        server.set_paused(true);
        let model = server
            .simulation()
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::rotor"))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        gz_sim_scene::remove_model(&mut server.simulation, model).unwrap();
        server.tick().unwrap();
        assert!(gz_sim_core::entity_by_id(server.simulation().world(), id).is_none());
        server.reset_scene().unwrap();
        let restored = gz_sim_core::entity_by_id(server.simulation().world(), id).unwrap();
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(restored)
                .unwrap()
                .torque(),
            0.0
        );
        server.shutdown();
    }
}

#[test]
fn unread_native_force_queue_is_discarded_when_subscriptions_restart() {
    let message = socket();
    let service = socket();
    let config = Config {
        capacity: 1,
        partition: format!("force-unread-{}", std::process::id()),
        message_port: message.local_addr().unwrap().port(),
        service_port: service.local_addr().unwrap().port(),
        ..Config::default()
    };
    let source = XML.replace("</model>", &format!("{PLUGIN}</model>"));
    let mut sim = load_world_with_sdf_step(
        &source,
        "unread.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let definitions: Vec<_> = gz_sim_scene::initial_joint_force_configurations(&sim)
        .unwrap()
        .map(|(id, cfg)| (id, cfg.clone()))
        .collect();
    let mut forces = crate::forces::Forces::prepare(&config, definitions).unwrap();
    forces.bind(&sim).unwrap();
    let mut peer = Peer::start(
        native_client(),
        &config,
        "/model/rotor/joint/hinge/cmd_force",
    );
    peer.line("READY");
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let mut stats = gz_transport::Statistics::default();
        forces.add_statistics(&mut stats);
        if stats.queue_drops >= 2 {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "native frames did not fill old bounded receiver: {stats:?}"
        );
        // PUB connection discovery precedes socket readiness. Retry until
        // observable overflow proves that the old receiver contains wire data.
        peer.send("1.0");
        thread::sleep(Duration::from_millis(10));
    }
    peer.finish();
    sim.reset().unwrap();
    forces.restart(&sim).unwrap();
    let mut counters = crate::Counters::default();
    forces
        .poll(&mut sim, NonZeroUsize::new(4).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(counters.force_messages_received, 0);
    sim.step_exact(2).unwrap();
    let joint = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<JointForceCommand>())
        .unwrap();
    assert_eq!(joint.get::<JointForceCommand>().unwrap().torque(), 0.0);
    assert_eq!(joint.get::<JointSample>().unwrap().state.velocity, 0.0);
    forces.shutdown();
}

// Copyright (C) 2019 Open Source Robotics Foundation
// Assertion port of ApplyJointForceTestFixture.JointVelocityCommand.
#[test]
fn original_ground_world_accepts_cpp_joint_force_with_original_sample_bounds() {
    let source = include_str!("../fixtures/apply_joint_force_original.sdf");
    let msg = socket();
    let srv = socket();
    let config = Config {
        partition: format!("original-force-cpp-{}", std::process::id()),
        message_port: msg.local_addr().unwrap().port(),
        service_port: srv.local_addr().unwrap().port(),
        ..Config::default()
    };
    let sim = load_world_with_sdf_step(
        source,
        "apply_joint_force.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Dart,
    )
    .unwrap();
    let joint = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|name| name.0 == "default::joint_force_test::j1")
        })
        .unwrap()
        .id();
    let mut server = Server::new(
        sim,
        "default",
        config.clone(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    // The tick-driven Rust server does not advance when constructed.
    assert_eq!(
        server
            .simulation()
            .world()
            .resource::<gz_sim_core::UpdateInfo>()
            .iteration,
        0
    );
    assert_eq!(server.pending_steps(), 0);
    server.set_paused(false);
    let mut samples = Vec::new();
    for _ in 0..10 {
        if let Some(command) = server.simulation().world().get::<JointForceCommand>(joint) {
            samples.push(command.torque());
        }
        server.tick().unwrap();
        thread::sleep(Duration::from_millis(1));
    }
    assert_eq!(samples.len(), 10);
    for force in &samples {
        assert!(force.abs() <= 1e-4);
    }
    samples.clear();
    let mut peer = Peer::start(
        native_client(),
        &config,
        "/model/joint_force_test/joint/j1/cmd_force",
    );
    peer.line("READY");
    peer.send("0.001");
    for _ in 0..1000 {
        if let Some(command) = server.simulation().world().get::<JointForceCommand>(joint) {
            samples.push(command.torque());
        }
        server.tick().unwrap();
        thread::sleep(Duration::from_millis(1));
        if samples
            .last()
            .is_some_and(|force| (force - 0.001).abs() < 1e-6)
        {
            break;
        }
    }
    let actual = *samples.last().unwrap();
    assert!(actual.to_bits().abs_diff(0.001_f64.to_bits()) <= 4);
    peer.finish();
}
