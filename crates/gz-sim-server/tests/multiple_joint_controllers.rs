// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical transport and actual Rapier motion; original System trajectories are separate.
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_proto::{
    msgs::{
        Boolean, Double, Entity as EntityRequest, WorldControl, WorldReset,
        entity::Type as EntityType,
    },
    prost::Message,
};
use gz_sim_scene::{
    EntityKind, JointControllerConfiguration, JointControllerMode, JointHandle, JointSample, Name,
    PhysicsEngine, WorldPose, load_world_with_sdf_step,
};
use gz_sim_server::{Error, Server, ServerOptions};
use std::{
    collections::BTreeMap,
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
const XML: &str = include_str!("../../../examples/m58_multiple_joint_controllers.sdf");
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
            partition: format!("m58-{}", uuid::Uuid::new_v4()),
            capacity: 8192,
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
fn peer() -> &'static zenoh_peer::NativePeer {
    static PEER: OnceLock<zenoh_peer::NativePeer> = OnceLock::new();
    PEER.get_or_init(|| {
        zenoh_peer::NativePeer::build(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."),
            "multiple_joint_controllers",
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
        panic!("native multiple JointController peer: {message}; {details}");
    }
    fn line(&mut self) -> String {
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
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    fn command(&mut self, command: &str) {
        let result = self
            .child
            .stdin
            .as_mut()
            .ok_or_else(|| std::io::Error::other("missing owned stdin"))
            .and_then(|input| {
                writeln!(input, "{command}")?;
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
            panic!("native multiple JointController channel capture failed: {details}");
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
            eprintln!(
                "native multiple JointController retained diagnostics: {}",
                self.retire(true)
            );
        }
    }
}

fn decode_hex(value: &str) -> Vec<u8> {
    assert!(value.len() <= 131072 && value.len().is_multiple_of(2));
    value
        .as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let digit = |c: u8| match c {
                b'0'..=b'9' => c - b'0',
                b'a'..=b'f' => c - b'a' + 10,
                _ => panic!("noncanonical packet hex"),
            };
            digit(pair[0]) * 16 + digit(pair[1])
        })
        .collect()
}
fn start_peer(network: &Network, options: &ServerOptions, count: usize) -> Peer {
    let mut command = Command::new(&peer().binary);
    command.arg(count.to_string());
    peer().configure_runtime(&mut command);
    command
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
    let out = child.stdout.take().unwrap();
    let err = child.stderr.take().unwrap();
    let (send, lines) = mpsc::sync_channel(16);
    let stdout = Arc::new(Mutex::new(Captured::default()));
    let stderr = Arc::new(Mutex::new(Captured::default()));
    let readers = vec![
        capture(out, Arc::clone(&stdout), Some(send)),
        capture(err, Arc::clone(&stderr), None),
    ];
    Peer {
        child,
        lines,
        readers,
        stdout,
        stderr,
        status: None,
        retired: false,
        retired_clean: false,
        reported: false,
    }
}

fn loaded() -> Simulation {
    load_world_with_sdf_step(
        XML,
        "m58-server.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
#[derive(Debug, Clone, PartialEq, Eq)]
struct Reading {
    identity: u64,
    configuration: JointControllerConfiguration,
    time: Duration,
    epoch: u64,
    q: u64,
    v: u64,
    child_pose: [u64; 7],
}
fn readings(server: &Server) -> BTreeMap<String, Reading> {
    let sim = server.simulation();
    sim.world()
        .iter_entities()
        .filter_map(|node| {
            let configuration = node.get::<JointControllerConfiguration>()?;
            let sample = node.get::<JointSample>().unwrap();
            let joint = node.get::<JointHandle>().unwrap();
            let child = entity_by_id(sim.world(), joint.child()).unwrap();
            let pose = sim.world().get::<WorldPose>(child).unwrap();
            assert_eq!((sample.time, sample.epoch), (sim.elapsed(), sim.epoch()));
            Some((
                configuration.topic.clone(),
                Reading {
                    identity: node.get::<EntityId>().unwrap().get(),
                    configuration: configuration.clone(),
                    time: sample.time,
                    epoch: sample.epoch,
                    q: sample.state.position.to_bits(),
                    v: sample.state.velocity.to_bits(),
                    child_pose: [
                        pose.translation[0].to_bits(),
                        pose.translation[1].to_bits(),
                        pose.translation[2].to_bits(),
                        pose.rotation[0].to_bits(),
                        pose.rotation[1].to_bits(),
                        pose.rotation[2].to_bits(),
                        pose.rotation[3].to_bits(),
                    ],
                },
            ))
        })
        .collect()
}
fn identities(server: &Server) -> BTreeMap<String, (u64, JointControllerConfiguration)> {
    readings(server)
        .into_iter()
        .map(|(key, value)| (key, (value.identity, value.configuration)))
        .collect()
}
fn wait_applied(server: &mut Server, target: u64) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        assert!(
            Instant::now() < deadline,
            "actual command application deadline"
        );
        assert!(server.update().unwrap().dt.is_zero());
        let applied = server.counters().joint_controller_commands_applied;
        assert!(applied <= target);
        if applied == target {
            break;
        }
        std::thread::yield_now();
    }
}
fn send(peer: &mut Peer, slot: usize, value: f64) {
    assert!(slot < 4 && value.is_finite());
    let word = format!("{:016x}", value.to_bits());
    peer.command(&format!("send {slot} {word}"));
    let line = peer.line();
    let fields: Vec<_> = line.split_whitespace().collect();
    assert_eq!(fields.len(), 4, "{line}");
    assert_eq!(fields[..3], ["SENT", &slot.to_string(), &word]);
    let wire = if fields[3] == "-" {
        Vec::new()
    } else {
        decode_hex(fields[3])
    };
    assert_eq!(
        wire,
        Double {
            data: value,
            ..Default::default()
        }
        .encode_to_vec()
    );
    assert_eq!(
        Double::decode(wire.as_slice()).unwrap().data.to_bits(),
        value.to_bits()
    );
}
fn matched(peer: &mut Peer, owner: usize) {
    peer.command(&format!("matched {owner}"));
    assert_eq!(peer.line(), format!("MATCHED {owner}"));
}
fn finish(mut peer: Peer) {
    peer.command("finish");
    assert_eq!(peer.line(), "PASS canonical multiple joint controllers");
    let (status, stderr) = peer.finish();
    assert!(status.success(), "actual native wait {status}; {stderr}");
    assert!(stderr.is_empty(), "{stderr}");
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
    server.prepare_io().unwrap();
    let info = server.tick().unwrap();
    assert!(info.dt.is_zero() && info.sim_time.is_zero());
}
fn motion(server: &Server, left: f64, right: f64) {
    let rows = readings(server);
    assert_eq!(rows.len(), 4);
    for row in rows.values() {
        let target = if row.configuration.model_name == "arm_left" {
            left
        } else {
            assert_eq!(row.configuration.model_name, "arm_right");
            right
        };
        let q = f64::from_bits(row.q);
        let v = f64::from_bits(row.v);
        assert!(q.is_finite() && v.is_finite());
        if row.configuration.mode == JointControllerMode::Position {
            assert!(
                (q - target).abs() < 0.1,
                "{}: q={q}, target={target}, v={v}",
                row.configuration.topic
            );
        } else {
            assert!(
                q * target > 0.03 && v * target > 0.03,
                "{}: q={q}, v={v}, target={target}",
                row.configuration.topic
            );
        }
    }
}

#[test]
fn four_canonical_native_doubles_drive_independent_loaded_chains_and_reset_without_output_interest()
{
    let network = Network::new();
    let options = network.options();
    let mut server = network.server(loaded(), options.clone()).unwrap();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    let original = identities(&server);
    assert_eq!(original.len(), 4);
    let mut peer = start_peer(&network, &options, 2);
    assert_eq!(peer.line(), "READY multiple-joint-controllers 2");
    let before = readings(&server);
    for (slot, value) in [0.6, 0.6, -0.5, -0.5].into_iter().enumerate() {
        send(&mut peer, slot, value);
    }
    wait_applied(&mut server, 4);
    assert_eq!(readings(&server), before);
    assert_eq!(server.simulation().elapsed(), Duration::ZERO);
    assert_eq!(server.counters().joint_controller_messages_received, 4);
    assert_eq!(server.counters().joint_controller_messages_rejected, 0);
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    for tick in 1..=1000 {
        assert_eq!(
            server.update().unwrap().sim_time,
            Duration::from_millis(tick)
        );
    }
    motion(&server, 0.6, -0.5);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    let held = readings(&server);
    for _ in 0..3 {
        assert!(server.update().unwrap().dt.is_zero());
        assert_eq!(readings(&server), held);
    }
    reset(&mut server);
    assert_eq!(server.simulation().epoch(), 1);
    assert_eq!(identities(&server), original);
    for row in readings(&server).values() {
        assert_eq!((f64::from_bits(row.q), f64::from_bits(row.v)), (0.0, 0.0));
    }
    matched(&mut peer, 0);
    matched(&mut peer, 1);
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    for _ in 0..100 {
        server.update().unwrap();
    }
    assert_eq!(server.counters().joint_controller_commands_applied, 4);
    for row in readings(&server).values() {
        assert!(f64::from_bits(row.q).abs() < 1e-12 && f64::from_bits(row.v).abs() < 1e-12);
    }
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    for (slot, value) in [-0.4, -0.4, 0.7, 0.7].into_iter().enumerate() {
        send(&mut peer, slot, value);
    }
    wait_applied(&mut server, 8);
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    for _ in 0..1000 {
        server.update().unwrap();
    }
    motion(&server, -0.4, 0.7);
    assert_eq!(server.counters().joint_controller_messages_received, 8);
    assert_eq!(server.counters().joint_controller_messages_rejected, 0);
    assert_eq!(
        (
            server.counters().odometries_queued,
            server.counters().imus_queued,
            server.counters().lidars_queued
        ),
        (0, 0, 0)
    );
    finish(peer);
}

fn remove(server: &mut Server, network: &Network, options: &ServerOptions, model: u64) {
    let before = readings(server);
    assert!(matches!(
        server.remove_editable_model(model),
        Err(Error::WorldEdit(
            gz_sim_scene::WorldEditError::UnsupportedModel
        ))
    ));
    assert_eq!(readings(server), before);
    let time = server.simulation().elapsed();
    let epoch = server.simulation().epoch();
    let old = server.counters();
    let deadline = Instant::now() + Duration::from_secs(5);
    let querier = network
        .observer
        .declare_querier(options.topic_key("/world/default/remove/blocking").unwrap())
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    while !querier.matching_status().wait().unwrap().matching() {
        assert!(Instant::now() < deadline);
        assert!(server.update().unwrap().dt.is_zero());
        std::thread::yield_now();
    }
    let replies = querier
        .get()
        .payload(
            EntityRequest {
                id: model,
                r#type: EntityType::Model as i32,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .wait()
        .unwrap();
    loop {
        assert!(Instant::now() < deadline, "actual blocking removal reply");
        assert!(server.update().unwrap().dt.is_zero());
        assert_eq!(
            (server.simulation().elapsed(), server.simulation().epoch()),
            (time, epoch)
        );
        if let Some(reply) = replies.recv_timeout(Duration::from_millis(20)).unwrap() {
            let response = reply.result().unwrap();
            assert!(response.payload().len() <= 64);
            assert_eq!(
                response.payload().to_bytes().as_ref(),
                Boolean {
                    data: true,
                    ..Default::default()
                }
                .encode_to_vec()
            );
            break;
        }
    }
    let new = server.counters();
    assert_eq!(
        (
            new.removals_queued,
            new.removals_completed,
            new.removals_failed,
            new.removals_rejected
        ),
        (
            old.removals_queued + 1,
            old.removals_completed + 1,
            old.removals_failed,
            old.removals_rejected
        )
    );
    assert!(entity_by_id(server.simulation().world(), model).is_none());
}

#[test]
fn removing_one_loaded_chain_retires_only_its_typed_inputs_and_reset_restores_both() {
    let network = Network::new();
    let options = network.options();
    let mut server = network.server(loaded(), options.clone()).unwrap();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    let original = identities(&server);
    let model = server
        .simulation()
        .world()
        .iter_entities()
        .find(|node| {
            node.get::<EntityKind>() == Some(&EntityKind::Model)
                && node
                    .get::<Name>()
                    .is_some_and(|name| name.0 == "default::arm_left")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let mut peer = start_peer(&network, &options, 2);
    assert_eq!(peer.line(), "READY multiple-joint-controllers 2");
    remove(&mut server, &network, &options, model);
    peer.command("retired 0");
    assert_eq!(peer.line(), "RETIRED 0");
    matched(&mut peer, 1);
    assert_eq!(readings(&server).len(), 2);
    for row in readings(&server).values() {
        assert_eq!(row.configuration.model_name, "arm_right");
    }
    send(&mut peer, 2, 0.5);
    send(&mut peer, 3, 0.5);
    wait_applied(&mut server, 2);
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    for _ in 0..1000 {
        server.update().unwrap();
    }
    for row in readings(&server).values() {
        let q = f64::from_bits(row.q);
        let v = f64::from_bits(row.v);
        if row.configuration.mode == JointControllerMode::Position {
            assert!((q - 0.5).abs() < 0.1);
        } else {
            assert!(q > 0.1 && v > 0.1);
        }
    }
    reset(&mut server);
    assert_eq!(server.simulation().epoch(), 1);
    matched(&mut peer, 0);
    matched(&mut peer, 1);
    assert_eq!(identities(&server), original);
    assert_eq!(readings(&server).len(), 4);
    assert_eq!(server.counters().joint_controller_commands_applied, 2);
    finish(peer);
}
