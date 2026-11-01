// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::Server;
use crate::ServerOptions;
use gz_sim_core::EntityId;
use gz_sim_scene::{JointForceCommand, JointSample, Name, PhysicsEngine, load_world_with_sdf_step};
use sdformat::NoResources;
use std::{
    io::{BufRead, BufReader, Read, Write},
    net::TcpListener,
    num::NonZeroUsize,
    path::Path,
    process::{Child, Command, Stdio},
    sync::mpsc,
    thread,
    time::{Duration, Instant},
};
#[path = "../../../../tests/support/zenoh_peer.rs"]
mod zenoh_peer;

const XML: &str = include_str!("../fixtures/joint_force.sdf");
const PLUGIN: &str = "<plugin filename='gz-sim-apply-joint-force-system' name='gz::sim::systems::ApplyJointForce'><joint_name>hinge</joint_name></plugin>";

// Process-local configuration prevents concurrent tests from mutating global
// Zenoh settings. A held loopback port is released only for this child's bind.
fn isolate(name: &str) -> bool {
    const CHILD: &str = "GZ_FORCE_ZENOH_CASE";
    if std::env::var(CHILD).as_deref() == Ok(name) {
        return false;
    }
    let directory = tempfile::tempdir().unwrap();
    let reservation = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = format!("tcp/127.0.0.1:{}", reservation.local_addr().unwrap().port());
    let rust = directory.path().join("rust.json");
    let native = directory.path().join("native.json");
    let scouting = serde_json::json!({"multicast":{"enabled":false},"gossip":{"enabled":false}});
    std::fs::write(&rust, serde_json::to_vec(&serde_json::json!({"mode":"peer","listen":{"endpoints":[address]},"scouting":scouting})).unwrap()).unwrap();
    std::fs::write(&native, serde_json::to_vec(&serde_json::json!({"mode":"peer","listen":{"endpoints":["tcp/127.0.0.1:0"]},"connect":{"endpoints":[address],"timeout_ms":2000},"scouting":scouting,"transport":{"shared_memory":{"enabled":false}}})).unwrap()).unwrap();
    let mut child = Command::new(std::env::current_exe().unwrap());
    let module = module_path!().split_once("::").unwrap().1;
    child
        .args([
            "--exact",
            &format!("{module}::{name}"),
            "--nocapture",
            "--test-threads=1",
        ])
        .env(CHILD, name)
        .env("ZENOH_CONFIG", rust)
        .env("GZ_FORCE_ZENOH_NATIVE_CONFIG", native)
        .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
        .env_remove("LD_PRELOAD")
        .env_remove("LD_AUDIT");
    drop(reservation);
    let result = child.output().unwrap();
    assert!(
        result.status.success(),
        "{}{}",
        String::from_utf8_lossy(&result.stdout),
        String::from_utf8_lossy(&result.stderr)
    );
    assert!(String::from_utf8_lossy(&result.stdout).contains("1 passed"));
    true
}
fn force_owner(config: &ServerOptions) -> crate::world_io::WorldIo {
    let prefix = format!("@/native-force-owner-{}@", std::process::id());
    crate::world_io::WorldIo::new(
        format!("{prefix}/clock"),
        format!("{prefix}/control"),
        format!("{prefix}/poses"),
        config.capacity,
        config.max_message_bytes,
    )
    .unwrap()
}

struct Peer {
    child: Child,
    lines: mpsc::Receiver<String>,
    stdout: Option<thread::JoinHandle<()>>,
    stderr: Option<thread::JoinHandle<String>>,
}
impl Peer {
    fn start(native: &zenoh_peer::NativePeer, config: &ServerOptions, topic: &str) -> Self {
        let mut command = Command::new(&native.binary);
        command
            .arg(topic)
            .env("GZ_PARTITION", &config.partition)
            .env("GZ_TRANSPORT_IMPLEMENTATION", "zenoh")
            .env(
                "ZENOH_CONFIG",
                std::env::var_os("GZ_FORCE_ZENOH_NATIVE_CONFIG").unwrap(),
            )
            .env_remove("GZ_RELAY")
            .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
            .env_remove("LD_PRELOAD")
            .env_remove("LD_AUDIT")
            .env_remove("GZ_TRANSPORT_TOPIC_STATISTICS")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        native.configure_runtime(&mut command);
        let mut child = command.spawn().unwrap();
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
        server.update().unwrap();
        assert!(Instant::now() < deadline, "force timeout");
        thread::sleep(Duration::from_millis(1));
    }
}
fn native_client() -> &'static zenoh_peer::NativePeer {
    static CLIENT: std::sync::OnceLock<zenoh_peer::NativePeer> = std::sync::OnceLock::new();
    CLIENT.get_or_init(|| {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        zenoh_peer::NativePeer::build(&root, "force")
    })
}

#[test]
fn real_cpp_publisher_controls_loaded_joint_force_on_all_dynamic_engines() {
    if isolate("real_cpp_publisher_controls_loaded_joint_force_on_all_dynamic_engines") {
        return;
    }
    let binary = native_client();
    for (index, engine) in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ]
    .into_iter()
    .enumerate()
    {
        let config = ServerOptions {
            partition: format!("force-cpp-{}-{index}", std::process::id()),
            ..ServerOptions::default()
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
            server.update().unwrap();
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
            server.update().unwrap();
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
            server.update().unwrap();
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
        server.update().unwrap();
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
    if isolate("unread_native_force_queue_is_discarded_when_subscriptions_restart") {
        return;
    }
    let config = ServerOptions {
        capacity: 1,
        partition: format!("force-unread-{}", std::process::id()),
        ..ServerOptions::default()
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
    let io = force_owner(&config);
    let (mut forces, setup) =
        crate::forces::Forces::prepare(&config, definitions, &io.ingress_drops).unwrap();
    forces.bind(&sim).unwrap();
    io.install_forces(setup).unwrap();
    let mut peer = Peer::start(
        native_client(),
        &config,
        "/model/rotor/joint/hinge/cmd_force",
    );
    peer.line("READY");
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let queue_drops = io.ingress_drops.load(std::sync::atomic::Ordering::Relaxed);
        if queue_drops >= 2 {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "native frames did not fill old bounded receiver: {queue_drops} drops"
        );
        // Native MS discovery precedes delivery readiness. Observable overflow
        // proves that actual wire samples reached the old bounded receiver.
        peer.send("1.0");
        thread::sleep(Duration::from_millis(10));
    }
    peer.finish();
    sim.reset().unwrap();
    forces.clear();
    let (replacement, setup) =
        crate::forces::Forces::restart(&config, &sim, &io.ingress_drops).unwrap();
    forces = replacement;
    io.install_forces(setup).unwrap();
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
    if isolate("original_ground_world_accepts_cpp_joint_force_with_original_sample_bounds") {
        return;
    }
    let source = include_str!("../fixtures/apply_joint_force_original.sdf");
    let config = ServerOptions {
        partition: format!("original-force-cpp-{}", std::process::id()),
        ..ServerOptions::default()
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
        server.update().unwrap();
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
        server.update().unwrap();
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
