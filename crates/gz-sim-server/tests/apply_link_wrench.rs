// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_proto::{
    msgs::{Entity, EntityWrench, Vector3d, WorldControl, WorldReset, Wrench, entity::Type},
    prost::{Message, Name as ProtoName},
};
use gz_sim_scene::{Name, PhysicsEngine, Velocity, load_world_with_sdf_step};
use gz_sim_server::{Server, ServerOptions};
use std::{
    io::{BufRead, BufReader, Read, Write},
    num::NonZeroUsize,
    path::Path,
    process::{Child, Command, Stdio},
    sync::{Arc, Mutex, OnceLock, mpsc},
    time::{Duration, Instant},
};
use zenoh::Wait;
#[path = "../../../tests/support/zenoh_peer.rs"]
mod zenoh_peer;
const XML: &str = include_str!("../../../examples/m35_apply_link_wrench.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m35.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn motion(sim: &Simulation) -> ([f64; 3], [f64; 3]) {
    let v = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::rig::body"))
        .unwrap()
        .get::<Velocity>()
        .unwrap();
    (v.linear, v.angular)
}
fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1e-12, "{actual} != {expected}");
}
fn vector(x: f64, y: f64, z: f64) -> Vector3d {
    Vector3d {
        x,
        y,
        z,
        ..Default::default()
    }
}
fn target(id: u64) -> Entity {
    Entity {
        id,
        r#type: Type::Link as i32,
        ..Default::default()
    }
}
fn wrench(id: u64, force: f64, torque: f64) -> EntityWrench {
    EntityWrench {
        entity: Some(target(id)),
        wrench: Some(Wrench {
            force: Some(vector(force, 0.0, 0.0)),
            torque: Some(vector(0.0, 0.0, torque)),
            ..Default::default()
        }),
        ..Default::default()
    }
}
fn wait(server: &mut Server, mut ready: impl FnMut(&Server) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !ready(server) {
        assert!(
            Instant::now() < deadline,
            "canonical wrench admission deadline"
        );
        server.update().unwrap();
        std::thread::yield_now();
    }
}
fn step(server: &mut Server, n: u32) {
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        multi_step: n,
        ..Default::default()
    }));
    for _ in 0..n {
        server.update().unwrap();
    }
}
fn reset(server: &mut Server) {
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.update().unwrap();
    assert_eq!(server.simulation().elapsed(), Duration::ZERO);
}
struct Network {
    observer: zenoh::Session,
    clients: zenoh::Config,
    dir: tempfile::TempDir,
}
impl Network {
    fn new() -> Self {
        let mut cfg = zenoh::Config::default();
        for (k, v) in [
            ("mode", "\"peer\""),
            ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
            ("connect/endpoints", "[]"),
            ("scouting/multicast/enabled", "false"),
            ("scouting/gossip/enabled", "false"),
        ] {
            cfg.insert_json5(k, v).unwrap();
        }
        let observer = zenoh::open(cfg).wait().unwrap();
        let endpoints: Vec<_> = observer
            .info()
            .locators()
            .wait()
            .into_iter()
            .map(|x| x.to_string())
            .collect();
        assert_eq!(endpoints.len(), 1);
        assert_ne!(endpoints[0], "tcp/127.0.0.1:0");
        let config = serde_json::json!({"mode":"client","listen":{"endpoints":[]},"connect":{"endpoints":endpoints,"timeout_ms":2000},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},"transport":{"shared_memory":{"enabled":false}}});
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("client.json"), config.to_string()).unwrap();
        Self {
            observer,
            clients: zenoh::Config::from_json5(&config.to_string()).unwrap(),
            dir,
        }
    }
    fn options(&self) -> ServerOptions {
        ServerOptions {
            partition: format!("m35-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        }
    }
    fn server(
        &self,
        sim: Simulation,
        options: ServerOptions,
    ) -> Result<Server, gz_sim_server::Error> {
        Server::with_zenoh_config(
            sim,
            "default",
            options,
            self.clients.clone(),
            NonZeroUsize::new(8).unwrap(),
        )
    }
}
#[test]
fn real_canonical_inputs_preserve_pause_add_clear_validation_reset_and_shutdown() {
    let network = Network::new();
    let options = network.options();
    let sim = load(XML);
    let body = id(&sim, "default::rig::body");
    let keys = [
        "/world/default/wrench",
        "/world/default/wrench/persistent",
        "/world/default/wrench/clear",
    ]
    .map(|s| options.topic_key(s).unwrap());
    let once = network
        .observer
        .declare_publisher(keys[0].clone())
        .wait()
        .unwrap();
    let persistent = network
        .observer
        .declare_publisher(keys[1].clone())
        .wait()
        .unwrap();
    let clear = network
        .observer
        .declare_publisher(keys[2].clone())
        .wait()
        .unwrap();
    let mut server = network.server(sim, options).unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    wait(&mut server, |_| {
        once.matching_status().wait().unwrap().matching()
            && persistent.matching_status().wait().unwrap().matching()
            && clear.matching_status().wait().unwrap().matching()
    });
    once.put(wrench(body, 4.0, 2.0).encode_to_vec())
        .attachment(EntityWrench::full_name())
        .wait()
        .unwrap();
    wait(&mut server, |s| s.counters().link_wrenches_queued == 1);
    assert_eq!(motion(server.simulation()), ([0.0; 3], [0.0; 3]));
    assert_eq!(server.simulation().elapsed(), Duration::ZERO);
    step(&mut server, 1);
    close(motion(server.simulation()).0[0], 0.002);
    close(motion(server.simulation()).1[2], 0.002);
    for value in [wrench(body, 1.0, 2.0), wrench(body, 3.0, 4.0)] {
        persistent
            .put(value.encode_to_vec())
            .attachment(EntityWrench::full_name())
            .wait()
            .unwrap();
    }
    wait(&mut server, |s| s.counters().link_wrenches_queued == 3);
    step(&mut server, 2);
    close(motion(server.simulation()).0[0], 0.006);
    close(motion(server.simulation()).1[2], 0.014);
    clear
        .put(target(body).encode_to_vec())
        .attachment(Entity::full_name())
        .wait()
        .unwrap();
    wait(&mut server, |s| s.counters().link_wrench_clears_queued == 1);
    server.update().unwrap();
    step(&mut server, 2);
    close(motion(server.simulation()).0[0], 0.006);
    let received = server.counters().wrench_messages_received;
    let rejected = server.counters().wrench_messages_rejected;
    let mut nonzero = wrench(body, 1.0, 0.0);
    nonzero.wrench.as_mut().unwrap().force_offset = Some(vector(0.1, 0.0, 0.0));
    let mut nonfinite = wrench(body, 1.0, 0.0);
    nonfinite.wrench.as_mut().unwrap().force = Some(vector(f64::NAN, 0.0, 0.0));
    let mut model = wrench(id(server.simulation(), "default::rig"), 1.0, 0.0);
    model.entity.as_mut().unwrap().r#type = Type::Model as i32;
    let missing_entity = EntityWrench {
        wrench: Some(Wrench::default()),
        ..Default::default()
    };
    let missing_wrench = EntityWrench {
        entity: Some(target(body)),
        ..Default::default()
    };
    let unknown = wrench(u64::MAX, 1.0, 0.0);
    for value in [
        nonzero,
        nonfinite,
        model,
        missing_entity,
        missing_wrench,
        unknown,
    ] {
        once.put(value.encode_to_vec())
            .attachment(EntityWrench::full_name())
            .wait()
            .unwrap();
    }
    once.put(wrench(body, 1.0, 0.0).encode_to_vec())
        .attachment("gz.msgs.Double")
        .wait()
        .unwrap();
    once.put(vec![0xff])
        .attachment(EntityWrench::full_name())
        .wait()
        .unwrap();
    wait(&mut server, |s| {
        s.counters().wrench_messages_received == received + 8
    });
    assert_eq!(server.counters().wrench_messages_rejected, rejected + 8);
    assert_eq!(server.counters().link_wrenches_queued, 3);
    // ID takes precedence over a conflicting name. Name-only admission is exact.
    let mut by_id = wrench(body, 2.0, 0.0);
    by_id.entity.as_mut().unwrap().name = "missing::body".into();
    once.put(by_id.encode_to_vec())
        .attachment(EntityWrench::full_name())
        .wait()
        .unwrap();
    let mut by_name = wrench(0, 2.0, 0.0);
    by_name.entity.as_mut().unwrap().name = "default::rig::body".into();
    persistent
        .put(by_name.encode_to_vec())
        .attachment(EntityWrench::full_name())
        .wait()
        .unwrap();
    wait(&mut server, |s| s.counters().link_wrenches_queued == 5);
    reset(&mut server);
    assert_eq!(server.simulation().epoch(), 1);
    step(&mut server, 1);
    assert_eq!(motion(server.simulation()), ([0.0; 3], [0.0; 3]));
    wait(&mut server, |_| {
        once.matching_status().wait().unwrap().matching()
    });
    once.put(
        EntityWrench {
            entity: Some(target(body)),
            wrench: Some(Wrench::default()),
            ..Default::default()
        }
        .encode_to_vec(),
    )
    .attachment(EntityWrench::full_name())
    .wait()
    .unwrap();
    wait(&mut server, |s| s.counters().link_wrenches_queued == 6);
    persistent
        .put(wrench(body, 100.0, 3.0).encode_to_vec())
        .attachment(EntityWrench::full_name())
        .wait()
        .unwrap();
    wait(&mut server, |s| s.counters().link_wrenches_queued == 7);
    let before = motion(server.simulation());
    let mut sim = server.into_simulation();
    sim.step_exact(1).unwrap();
    assert_eq!(motion(&sim), before);
}
#[test]
fn all_three_keys_alias_atomically_and_absent_system_does_not_declare_inputs() {
    let network = Network::new();
    let raw = [
        "/world/default/wrench",
        "/world/default/wrench/persistent",
        "/world/default/wrench/clear",
    ];
    for i in 0..3 {
        for j in 0..3 {
            if i == j {
                continue;
            }
            let mut options = network.options();
            options.add_topic_remap(raw[i], raw[j]).unwrap();
            assert!(network.server(load(XML), options).is_err());
        }
    }
    for from in raw {
        for reserved in ["/clock", "/world/default/clock", "/world/default/control"] {
            let mut options = network.options();
            options.add_topic_remap(from, reserved).unwrap();
            assert!(network.server(load(XML), options).is_err());
        }
    }
    let xml=XML.replace("  <plugin name=\"gz::sim::systems::ApplyLinkWrench\" filename=\"gz-sim-apply-link-wrench-system\"/>\n","");
    let options = network.options();
    let key = options.topic_key(raw[0]).unwrap();
    let mut server = network.server(load(&xml), options).unwrap();
    let publisher = network.observer.declare_publisher(key).wait().unwrap();
    server.update().unwrap();
    assert!(!publisher.matching_status().wait().unwrap().matching());
    assert_eq!(server.counters().wrench_messages_received, 0);
    server.shutdown();
}
fn peer() -> &'static zenoh_peer::NativePeer {
    static PEER: OnceLock<zenoh_peer::NativePeer> = OnceLock::new();
    PEER.get_or_init(|| {
        zenoh_peer::NativePeer::build(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."),
            "apply_link_wrench",
        )
    })
}
const PEER_CHANNEL_CAP: usize = 1024 * 1024;
#[derive(Default)]
struct Captured {
    bytes: Vec<u8>,
    error: Option<String>,
    exceeded: bool,
}
fn capture<R: Read + Send + 'static>(
    stream: R,
    state: Arc<Mutex<Captured>>,
    lines: Option<mpsc::SyncSender<String>>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut reader = BufReader::new(stream).take((PEER_CHANNEL_CAP + 1) as u64);
        loop {
            let mut line = Vec::new();
            match reader.read_until(b'\n', &mut line) {
                Ok(0) => return,
                Ok(_) => {
                    let mut saved = state.lock().unwrap();
                    let remaining = PEER_CHANNEL_CAP - saved.bytes.len();
                    saved
                        .bytes
                        .extend_from_slice(&line[..line.len().min(remaining)]);
                    if line.len() > remaining {
                        saved.exceeded = true;
                        return;
                    }
                    drop(saved);
                    if let Some(sender) = &lines {
                        if line.last() == Some(&b'\n') {
                            line.pop();
                        }
                        let text = match String::from_utf8(line) {
                            Ok(text) => text,
                            Err(error) => {
                                state.lock().unwrap().error = Some(error.to_string());
                                return;
                            }
                        };
                        if let Err(error) = sender.try_send(text) {
                            state.lock().unwrap().error =
                                Some(format!("bounded phase channel: {error}"));
                            return;
                        }
                    }
                }
                Err(error) => {
                    state.lock().unwrap().error = Some(error.to_string());
                    return;
                }
            }
        }
    })
}
struct Peer {
    child: Child,
    lines: mpsc::Receiver<String>,
    readers: Vec<std::thread::JoinHandle<()>>,
    stdout: Arc<Mutex<Captured>>,
    stderr: Arc<Mutex<Captured>>,
    status: Option<std::process::ExitStatus>,
    retired: bool,
    retired_clean: bool,
    reported: bool,
}
impl Peer {
    fn diagnostics(&self) -> String {
        let channel = |name: &str, state: &Arc<Mutex<Captured>>| {
            let saved = state.lock().unwrap();
            let bytes = match std::str::from_utf8(&saved.bytes) {
                Ok(text) => format!("{text:?}"),
                Err(_) => format!("{:?}", saved.bytes),
            };
            format!(
                "{name}: captured_bytes={} exceeded={} read_error={:?} contents={bytes}",
                saved.bytes.len(),
                saved.exceeded,
                saved.error
            )
        };
        format!(
            "actual_wait={:?}; {}\n{}",
            self.status,
            channel("stdout", &self.stdout),
            channel("stderr", &self.stderr)
        )
    }
    fn retire(&mut self, force: bool) -> String {
        if self.retired {
            return self.diagnostics();
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut cleanup = Vec::new();
        if force && self.status.is_none() {
            match self.child.try_wait() {
                Ok(Some(status)) => self.status = Some(status),
                Ok(None) => {
                    if let Err(error) = self.child.kill() {
                        cleanup.push(format!("owned kill: {error}"));
                    }
                }
                Err(error) => {
                    cleanup.push(format!("owned wait: {error}"));
                    if let Err(error) = self.child.kill() {
                        cleanup.push(format!("owned kill: {error}"));
                    }
                }
            }
        }
        while self.status.is_none() && Instant::now() < deadline {
            match self.child.try_wait() {
                Ok(Some(status)) => self.status = Some(status),
                Ok(None) => std::thread::sleep(Duration::from_millis(1)),
                Err(error) => {
                    cleanup.push(format!("owned wait: {error}"));
                    break;
                }
            }
        }
        if self.status.is_some() {
            while self.readers.iter().any(|reader| !reader.is_finished())
                && Instant::now() < deadline
            {
                std::thread::sleep(Duration::from_millis(1));
            }
            let readers = std::mem::take(&mut self.readers);
            for reader in readers {
                if reader.is_finished() {
                    if reader.join().is_err() {
                        cleanup.push("channel reader panicked".into());
                    }
                } else {
                    cleanup.push("channel reader completion UNKNOWN at cleanup deadline".into());
                }
            }
        } else {
            cleanup.push("actual native wait UNKNOWN at cleanup deadline".into());
        }
        self.retired_clean = self.status.is_some() && cleanup.is_empty();
        self.retired = true;
        format!("{}; cleanup={cleanup:?}", self.diagnostics())
    }
    fn fail(&mut self, message: impl std::fmt::Display) -> ! {
        let details = self.retire(true);
        self.reported = true;
        panic!("native wrench peer: {message}; {details}");
    }
    fn line(&mut self, server: &mut Server) -> String {
        let deadline = Instant::now() + Duration::from_secs(8);
        loop {
            let broken = [&self.stdout, &self.stderr].into_iter().any(|state| {
                let saved = state.lock().unwrap();
                saved.exceeded || saved.error.is_some()
            });
            if broken {
                self.fail("bounded channel capture failed");
            }
            match self.lines.try_recv() {
                Ok(value) => return value,
                Err(mpsc::TryRecvError::Empty) => {}
                Err(error) => self.fail(format!("phase channel closed: {error}")),
            }
            if Instant::now() >= deadline {
                self.fail("unchanged eight-second phase deadline");
            }
            if let Err(error) = server.update() {
                self.fail(format!("Server update: {error}"));
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    fn next(&mut self) {
        let result = self
            .child
            .stdin
            .as_mut()
            .ok_or_else(|| std::io::Error::other("missing owned stdin"))
            .and_then(|input| {
                writeln!(input, "next")?;
                input.flush()
            });
        if let Err(error) = result {
            self.fail(format!("phase input: {error}"));
        }
    }
    fn finish(&mut self) -> (std::process::ExitStatus, String) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while self.status.is_none() {
            match self.child.try_wait() {
                Ok(Some(status)) => self.status = Some(status),
                Ok(None) => {}
                Err(error) => self.fail(format!("exit wait: {error}")),
            }
            if Instant::now() >= deadline {
                self.fail("unchanged five-second exit deadline");
            }
            if self.status.is_none() {
                std::thread::yield_now();
            }
        }
        let details = self.retire(false);
        let failed = [&self.stdout, &self.stderr].into_iter().any(|state| {
            let saved = state.lock().unwrap();
            saved.exceeded || saved.error.is_some()
        });
        if failed || !self.retired_clean {
            self.reported = true;
            panic!("native wrench channel capture failed: {details}");
        }
        let status = self.status.unwrap();
        let stderr = String::from_utf8(self.stderr.lock().unwrap().bytes.clone())
            .unwrap_or_else(|error| format!("non-UTF8 native stderr: {error}"));
        (status, stderr)
    }
}
impl Drop for Peer {
    fn drop(&mut self) {
        if !self.retired || std::thread::panicking() && !self.reported {
            eprintln!("native wrench retained diagnostics: {}", self.retire(true));
        }
    }
}
#[test]
fn actual_cpp_entity_wrench_and_clear_messages_drive_the_rust_world() {
    let network = Network::new();
    let options = network.options();
    let sim = load(XML);
    let body = id(&sim, "default::rig::body");
    let mut server = network.server(sim, options.clone()).unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    let mut command = Command::new(&peer().binary);
    peer().configure_runtime(&mut command);
    command
        .arg(body.to_string())
        .env("GZ_PARTITION", &options.partition)
        .env("GZ_IP", "127.0.0.1")
        .env("ZENOH_CONFIG", network.dir.path().join("client.json"))
        .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
        .env_remove("GZ_DISCOVERY_MSG_PORT")
        .env_remove("GZ_DISCOVERY_SRV_PORT")
        .env_remove("GZ_DISCOVERY_MULTICAST_IP")
        .env_remove("GZ_TRANSPORT_USERNAME")
        .env_remove("GZ_TRANSPORT_PASSWORD")
        .env_remove("GZ_RELAY")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = command.spawn().unwrap();
    let output = child.stdout.take().unwrap();
    let errors = child.stderr.take().unwrap();
    let (send, lines) = mpsc::sync_channel(16);
    let stdout = Arc::new(Mutex::new(Captured::default()));
    let stderr = Arc::new(Mutex::new(Captured::default()));
    let readers = vec![
        capture(output, Arc::clone(&stdout), Some(send)),
        capture(errors, Arc::clone(&stderr), None),
    ];
    let mut peer = Peer {
        child,
        lines,
        readers,
        stdout,
        stderr,
        status: None,
        retired: false,
        retired_clean: false,
        reported: false,
    };
    assert_eq!(peer.line(&mut server), "READY wrench inputs");
    peer.next();
    assert_eq!(peer.line(&mut server), "SENT once");
    wait(&mut server, |s| s.counters().link_wrenches_queued == 1);
    step(&mut server, 1);
    close(motion(server.simulation()).0[0], 0.002);
    close(motion(server.simulation()).1[2], 0.002);
    peer.next();
    assert_eq!(peer.line(&mut server), "SENT persistent");
    wait(&mut server, |s| s.counters().link_wrenches_queued == 3);
    step(&mut server, 2);
    close(motion(server.simulation()).0[0], 0.006);
    close(motion(server.simulation()).1[2], 0.014);
    peer.next();
    assert_eq!(peer.line(&mut server), "SENT clear");
    wait(&mut server, |s| s.counters().link_wrench_clears_queued == 1);
    server.update().unwrap();
    step(&mut server, 2);
    close(motion(server.simulation()).0[0], 0.006);
    close(motion(server.simulation()).1[2], 0.014);
    peer.next();
    assert_eq!(peer.line(&mut server), "PASS canonical ApplyLinkWrench");
    let (status, stderr) = peer.finish();
    assert!(
        status.success(),
        "{status}: {stderr}; {}",
        peer.diagnostics()
    );
    assert!(stderr.is_empty(), "{stderr}");
    assert_eq!(
        (
            server.counters().wrench_messages_received,
            server.counters().wrench_messages_rejected
        ),
        (4, 0)
    );
    server.shutdown();
}

#[test]
fn inactive_configured_server_refuses_template_reactivation_before_any_mutation() {
    let mut sim = load(XML);
    let source = sdformat::load_str(XML, "m35.sdf", &sdformat::NoResources).unwrap();
    let template =
        gz_sim_scene::prepare_model_template(&sim, &source, &source.worlds[0].models[0]).unwrap();
    for name in ["default::rig", "default::other"] {
        let identity = id(&sim, name);
        gz_sim_scene::remove_model(&mut sim, identity).unwrap();
    }
    assert!(!gz_sim_scene::apply_link_wrench_active(&sim));
    let network = Network::new();
    let options = network.options();
    let once = network
        .observer
        .declare_publisher(options.topic_key("/world/default/wrench").unwrap())
        .wait()
        .unwrap();
    let mut server = network.server(sim, options).unwrap();
    let before: std::collections::BTreeSet<_> = server
        .simulation()
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(EntityId::get))
        .collect();
    let epoch = server.simulation().epoch();
    let time = server.simulation().elapsed();
    assert!(matches!(
        server.spawn_model_template(
            &template,
            gz_sim_scene::WorldPose {
                translation: [0.0, 6.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0]
            }
        ),
        Err(gz_sim_server::Error::ApplyLinkWrenchReactivation)
    ));
    let after: std::collections::BTreeSet<_> = server
        .simulation()
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(EntityId::get))
        .collect();
    assert_eq!(after, before);
    assert_eq!(
        (server.simulation().epoch(), server.simulation().elapsed()),
        (epoch, time)
    );
    assert!(!once.matching_status().wait().unwrap().matching());
    assert_eq!(server.counters().link_wrenches_queued, 0);
    reset(&mut server);
    assert!(gz_sim_scene::apply_link_wrench_active(server.simulation()));
    assert_eq!(
        id(server.simulation(), "default::rig::body"),
        id(&load(XML), "default::rig::body")
    );
    server.shutdown();
}
