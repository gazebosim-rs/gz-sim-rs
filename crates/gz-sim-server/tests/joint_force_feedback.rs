// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Actual canonical Double commands and complete FIFO Model feedback.
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_proto::{
    msgs::{
        Axis, Boolean, Entity as EntityRequest, Header, Joint, Model, Pose, Quaternion, Time,
        Vector3d, WorldControl, WorldReset, entity::Type as EntityType,
    },
    prost::Message,
};
use gz_sim_scene::{
    JointForceCommand, JointSample, JointStatePublication, JointStateSample, Name, PhysicsEngine,
    load_world_with_sdf_step,
};
use gz_sim_server::{Counters, Error, Server, ServerOptions};
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
const XML: &str = include_str!("../../../examples/m42_joint_force_feedback.sdf");
const FORCE: &str = "<plugin filename=\"gz-sim-apply-joint-force-system\" name=\"gz::sim::systems::ApplyJointForce\"><joint_name>hinge</joint_name></plugin>";
fn load() -> Simulation {
    load_world_with_sdf_step(
        XML,
        "m42.sdf",
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
fn sample(server: &Server) -> Option<JointStateSample> {
    server
        .simulation()
        .world()
        .iter_entities()
        .find_map(|e| e.get::<JointStateSample>())
        .cloned()
}
fn motion(server: &Server) -> (f64, f64) {
    let e = entity_by_id(
        server.simulation().world(),
        id(server.simulation(), "default::rotor::hinge"),
    )
    .unwrap();
    let sample = server.simulation().world().get::<JointSample>(e).unwrap();
    assert_eq!(
        (sample.time, sample.epoch),
        (server.simulation().elapsed(), server.simulation().epoch())
    );
    (sample.state.position, sample.state.velocity)
}
fn reference_motion(sim: &Simulation, joint: u64) -> (f64, f64) {
    let entity = entity_by_id(sim.world(), joint).unwrap();
    let sample = sim.world().get::<JointSample>(entity).unwrap();
    assert_eq!((sample.time, sample.epoch), (sim.elapsed(), sim.epoch()));
    (sample.state.position, sample.state.velocity)
}
fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1e-12, "{actual} != {expected}");
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
            partition: format!("m42-{}", uuid::Uuid::new_v4()),
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
            "joint_force_feedback",
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
        panic!("native joint-force feedback peer: {message}; {details}");
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
            panic!("native joint-force feedback channel capture failed: {details}");
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
                "native joint-force feedback retained diagnostics: {}",
                self.retire(true)
            );
        }
    }
}

fn decode_hex(value: &str) -> Vec<u8> {
    assert!(value.len() <= 16384 && value.len().is_multiple_of(2));
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
fn vector(v: [f64; 3]) -> Vector3d {
    Vector3d {
        x: v[0],
        y: v[1],
        z: v[2],
        ..Default::default()
    }
}
fn pose(position: [f64; 3], q: [f64; 4]) -> Pose {
    Pose {
        position: Some(vector(position)),
        orientation: Some(Quaternion {
            x: q[0],
            y: q[1],
            z: q[2],
            w: q[3],
            ..Default::default()
        }),
        ..Default::default()
    }
}
fn check_packet(server: &Server, actual: &[u8], s: &JointStateSample) {
    let model_id = id(server.simulation(), "default::rotor");
    let model = entity_by_id(server.simulation().world(), model_id).unwrap();
    let metadata = server
        .simulation()
        .world()
        .get::<JointStatePublication>(model)
        .unwrap();
    assert_eq!(
        (metadata.model_name.as_str(), metadata.topic.as_str()),
        ("rotor", "/m42/joint_state")
    );
    assert_eq!(metadata.joints.len(), 1);
    assert_eq!(s.joints.len(), metadata.joints.len());
    assert!(metadata.joints.windows(2).all(|p| p[0].id < p[1].id));
    let joints = metadata
        .joints
        .iter()
        .zip(&s.joints)
        .map(|(m, v)| {
            assert_eq!(m.id, v.id);
            assert_eq!(
                (m.name.as_str(), m.parent.as_str(), m.child.as_str()),
                ("hinge", "world", "body")
            );
            assert_eq!(m.axis, [0.0, 1.0, 0.0]);
            assert_eq!(
                (m.lower, m.upper, m.damping),
                (f64::NEG_INFINITY, f64::INFINITY, 0.0)
            );
            assert!(v.position.is_finite() && v.velocity.is_finite());
            Joint {
                id: m.id.try_into().unwrap(),
                name: m.name.clone(),
                parent: m.parent.clone(),
                child: m.child.clone(),
                pose: Some(pose(m.pose.translation, m.pose.rotation)),
                axis1: Some(Axis {
                    xyz: Some(vector(m.axis)),
                    limit_lower: m.lower,
                    limit_upper: m.upper,
                    damping: m.damping,
                    position: v.position,
                    velocity: v.velocity,
                    ..Default::default()
                }),
                ..Default::default()
            }
        })
        .collect();
    let expected = Model {
        header: Some(Header {
            stamp: Some(Time {
                sec: s.time.as_secs().try_into().unwrap(),
                nsec: s.time.subsec_nanos().try_into().unwrap(),
            }),
            data: vec![],
        }),
        id: model_id.try_into().unwrap(),
        name: "rotor".into(),
        pose: Some(pose(s.pose.translation, s.pose.rotation)),
        joint: joints,
        ..Default::default()
    };
    // Whole-message equality covers every unset/default Model, Joint, Axis, Pose
    // and Vector/Quaternion field. Byte equality also refuses unmodeled fields.
    assert_eq!(Model::decode(actual).unwrap(), expected);
    assert_eq!(actual, expected.encode_to_vec());
    assert_eq!(
        expected.joint[0].axis1.as_ref().unwrap().force,
        0.0,
        "not a measured effort or command echo"
    );
}
fn received_packet(peer: &mut Peer) -> Vec<u8> {
    let line = peer.line();
    let hex = line
        .strip_prefix("PACKET ")
        .unwrap_or_else(|| panic!("{line}"));
    decode_hex(hex)
}
fn drain(peer: &mut Peer, server: &Server, expected: Option<&JointStateSample>) {
    let count = usize::from(expected.is_some());
    peer.command(&format!("drain {count}"));
    if let Some(expected) = expected {
        check_packet(server, &received_packet(peer), expected);
    }
    assert_eq!(peer.line(), format!("DRAINED {count}"));
    assert!(peer.lines.try_recv().is_err(), "unexpected phase output");
}
fn update(server: &mut Server, peer: &mut Peer, observing: bool) {
    let before = sample(server).map(|s| (s.epoch, s.revision));
    let queued = server.counters().joint_states_queued;
    if let Err(error) = server.update() {
        peer.fail(format!("Server update: {error}"));
    }
    let after = sample(server);
    let changed = after.as_ref().map(|s| (s.epoch, s.revision)) != before;
    assert_eq!(
        server.counters().joint_states_queued - queued,
        u64::from(changed)
    );
    if changed {
        let s = after.as_ref().unwrap();
        assert_eq!(
            (s.time, s.epoch),
            (server.simulation().elapsed(), server.simulation().epoch())
        );
        assert_eq!((s.joints[0].position, s.joints[0].velocity), motion(server));
        assert_eq!(s.revision, before.map_or(1, |(_, r)| r + 1));
    }
    if observing {
        drain(peer, server, if changed { after.as_ref() } else { None });
    }
}
fn advance(server: &mut Server, peer: &mut Peer, count: u32, observing: bool) {
    for _ in 0..count {
        let t = server.simulation().elapsed();
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            multi_step: 1,
            ..Default::default()
        }));
        update(server, peer, observing);
        assert_eq!(server.simulation().elapsed(), t + Duration::from_millis(1));
    }
}
fn admission(server: &mut Server, peer: &mut Peer, command: &str, effort: f64, observing: bool) {
    let before = server.counters();
    let boundary = (
        server.simulation().elapsed(),
        server.simulation().epoch(),
        motion(server),
    );
    peer.command(command);
    assert_eq!(peer.line(), format!("SENT {command}"));
    let deadline = Instant::now() + Duration::from_secs(5);
    for attempt in 0..64 {
        let counters = server.counters();
        let received = counters.force_messages_received - before.force_messages_received;
        let applied = counters.joint_forces_applied - before.joint_forces_applied;
        assert!(received <= 1 && applied <= 1);
        assert_eq!(
            counters.force_messages_rejected,
            before.force_messages_rejected
        );
        assert_eq!(
            (
                server.simulation().elapsed(),
                server.simulation().epoch(),
                motion(server)
            ),
            boundary
        );
        if (received, applied) == (1, 1) {
            let joint = entity_by_id(
                server.simulation().world(),
                id(server.simulation(), "default::rotor::hinge"),
            )
            .unwrap();
            assert_eq!(
                server
                    .simulation()
                    .world()
                    .get::<JointForceCommand>(joint)
                    .unwrap()
                    .torque(),
                effort
            );
            return;
        }
        if attempt == 63 || Instant::now() >= deadline {
            peer.fail("actual received+queued force admission deadline");
        }
        update(server, peer, observing);
        std::thread::sleep(Duration::from_millis(1));
    }
    unreachable!()
}
fn remove_through_canonical_service(
    server: &mut Server,
    network: &Network,
    options: &ServerOptions,
    model: u64,
) {
    let time = server.simulation().elapsed();
    let epoch = server.simulation().epoch();
    let pending = server.pending_steps();
    let before_counters = format!("{:?}", server.counters());
    let before_sample = format!("{:?}", sample(server));
    let identities = |sim: &Simulation| {
        sim.world()
            .iter_entities()
            .map(|entity| {
                (
                    entity.id(),
                    entity.get::<EntityId>().map(|id| id.get()),
                    entity
                        .get::<gz_sim_scene::WorldPose>()
                        .map(|pose| (pose.translation, pose.rotation)),
                )
            })
            .collect::<Vec<_>>()
    };
    let before_entities = identities(server.simulation());
    assert!(matches!(
        server.remove_editable_model(model),
        Err(Error::WorldEdit(
            gz_sim_scene::WorldEditError::UnsupportedModel
        ))
    ));
    assert_eq!(identities(server.simulation()), before_entities);
    assert_eq!(format!("{:?}", sample(server)), before_sample);
    assert_eq!(format!("{:?}", server.counters()), before_counters);
    assert_eq!(
        (
            server.simulation().elapsed(),
            server.simulation().epoch(),
            server.pending_steps()
        ),
        (time, epoch, pending)
    );
    assert_eq!(pending, 0);
    assert!(server.simulation().is_paused());

    // This existing service admits sensor/joint-bearing top-level models. The
    // narrower direct world-edit API above must continue refusing them.
    let counters = server.counters();
    let deadline = Instant::now() + Duration::from_secs(5);
    let key = options.topic_key("/world/default/remove/blocking").unwrap();
    let querier = network
        .observer
        .declare_querier(key)
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let mut matched = false;
    for _ in 0..256 {
        assert!(
            Instant::now() < deadline,
            "removal Queryable matching deadline"
        );
        if querier.matching_status().wait().unwrap().matching() {
            matched = true;
            break;
        }
        server.update().unwrap();
        assert_eq!(
            (server.simulation().elapsed(), server.simulation().epoch()),
            (time, epoch)
        );
        // Host pacing is not a readiness proof: actual matching is required.
        std::thread::sleep(Duration::from_millis(1));
    }
    assert!(matched, "actual removal Queryable did not match");
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
    let mut actual_reply = None;
    for _ in 0..256 {
        assert!(
            Instant::now() < deadline,
            "actual blocking removal reply deadline"
        );
        server.update().unwrap();
        assert_eq!(
            (
                server.simulation().elapsed(),
                server.simulation().epoch(),
                server.pending_steps()
            ),
            (time, epoch, 0)
        );
        if let Some(reply) = replies.recv_timeout(Duration::from_millis(20)).unwrap() {
            assert!(Instant::now() < deadline, "late blocking removal reply");
            actual_reply = Some(reply);
            break;
        }
    }
    let reply = actual_reply.expect("one actual blocking removal reply required");
    let response = reply.result().expect("canonical removal service error");
    assert!(response.payload().len() <= 64);
    let bytes = response.payload().to_bytes();
    assert_eq!(
        bytes.as_ref(),
        Boolean {
            data: true,
            ..Default::default()
        }
        .encode_to_vec()
    );
    assert!(Boolean::decode(bytes.as_ref()).unwrap().data);
    let after = server.counters();
    assert_eq!(after.removals_queued, counters.removals_queued + 1);
    assert_eq!(after.removals_completed, counters.removals_completed + 1);
    assert_eq!(after.removals_failed, counters.removals_failed);
    assert_eq!(after.removals_rejected, counters.removals_rejected);
    assert_eq!(after.response_failures, counters.response_failures);
    assert!(entity_by_id(server.simulation().world(), model).is_none());
}

fn reset_loaded(server: &mut Server) {
    let epoch = server.simulation().epoch();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.update().unwrap();
    assert_eq!(
        (server.simulation().elapsed(), server.simulation().epoch()),
        (Duration::ZERO, epoch + 1)
    );
    assert!(
        sample(server).is_none(),
        "reset transaction has no PostStep acquisition"
    );
    assert_eq!(motion(server), (0.0, 0.0));
    let joint = entity_by_id(
        server.simulation().world(),
        id(server.simulation(), "default::rotor::hinge"),
    )
    .unwrap();
    assert_eq!(
        server
            .simulation()
            .world()
            .get::<JointForceCommand>(joint)
            .unwrap()
            .torque(),
        0.0
    );
}
fn start_peer(network: &Network, options: &ServerOptions) -> Peer {
    let mut command = Command::new(&peer().binary);
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
    let output = child.stdout.take().unwrap();
    let errors = child.stderr.take().unwrap();
    let (send, lines) = mpsc::sync_channel(16);
    let stdout = Arc::new(Mutex::new(Captured::default()));
    let stderr = Arc::new(Mutex::new(Captured::default()));
    let readers = vec![
        capture(output, Arc::clone(&stdout), Some(send)),
        capture(errors, Arc::clone(&stderr), None),
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
#[test]
fn disconnected_world_hinge_still_acquires_equal_time_and_due_samples_without_force_echo() {
    let network = Network::new();
    let mut server = network.server(load(), network.options()).unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    for revision in 1..=3 {
        server.update().unwrap();
        let s = sample(&server).unwrap();
        assert_eq!((s.time, s.epoch, s.revision), (Duration::ZERO, 0, revision));
        assert_eq!((s.joints[0].position, s.joints[0].velocity), (0.0, 0.0));
    }
    let held = sample(&server).unwrap();
    for millisecond in 1..=10 {
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            multi_step: 1,
            ..Default::default()
        }));
        server.update().unwrap();
        let s = sample(&server).unwrap();
        assert_eq!(s.revision, held.revision + u64::from(millisecond == 10));
    }
    assert_eq!(server.counters().joint_states_queued, 4);
    assert_eq!(server.counters().force_messages_received, 0);
    assert_eq!(server.counters().joint_forces_applied, 0);
    assert_eq!(server.counters().joint_state_failures, 0);
    server.shutdown();
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}
fn begin_prime(peer: &mut Peer, time: Duration) {
    peer.command(&format!("prime {}", time.as_nanos()));
    assert_eq!(peer.line(), format!("PRIMING {}", time.as_nanos()));
}
fn finish_prime(peer: &mut Peer, server: &Server, time: Duration, first: Option<String>) {
    let bytes = match first {
        Some(line) => decode_hex(
            line.strip_prefix("PACKET ")
                .unwrap_or_else(|| panic!("{line}")),
        ),
        None => received_packet(peer),
    };
    let s = sample(server).unwrap();
    assert_eq!(s.time, time);
    check_packet(server, &bytes, &s);
    assert_eq!(peer.line(), format!("PRIMED {}", time.as_nanos()));
}
fn prime_current(peer: &mut Peer, server: &mut Server) {
    let t = server.simulation().elapsed();
    begin_prime(peer, t);
    let deadline = Instant::now() + Duration::from_secs(5);
    // Priming may miss early matching-time publications. Only its one actual
    // Model is credited; none of this interval is claimed as exact FIFO history.
    for attempt in 0..256 {
        update(server, peer, false);
        assert_eq!(server.simulation().elapsed(), t);
        match peer.lines.recv_timeout(Duration::from_millis(20)) {
            Ok(line) => {
                finish_prime(peer, server, t, Some(line));
                return;
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(error) => peer.fail(format!("priming channel: {error}")),
        }
        if attempt == 255 || Instant::now() >= deadline {
            peer.fail("actual paused Model priming deadline");
        }
    }
    unreachable!()
}
fn cutoff_and_arm(peer: &mut Peer, server: &mut Server) {
    let t = server.simulation().elapsed();
    assert!(t.as_nanos().is_multiple_of(10_000_000));
    let due = t + Duration::from_millis(10);
    begin_prime(peer, due);
    // Exactly one later due Model; no equal-time updates after reaching it.
    // This is an explicit zero-effort warmup, not a Clock/network barrier.
    advance(server, peer, 10, false);
    finish_prime(peer, server, due, None);
    peer.command("arm");
    assert_eq!(peer.line(), "ARMED");
    drain(peer, server, None);
}
fn drop_feedback(peer: &mut Peer) {
    peer.command("drop");
    assert_eq!(peer.line(), "DROPPED");
}
fn watch_feedback(peer: &mut Peer) {
    peer.command("watch");
    assert_eq!(peer.line(), "WATCHING");
}
#[test]
fn canonical_force_and_complete_fifo_model_feedback_cover_pulses_coast_reset_and_retirement() {
    let network = Network::new();
    assert!(!network.observer.is_closed());
    let options = network.options();
    let sim = load();
    let joint = id(&sim, "default::rotor::hinge");
    let model = id(&sim, "default::rotor");
    let mut server = network.server(sim, options.clone()).unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    let mut peer = start_peer(&network, &options);
    assert_eq!(peer.line(), "READY joint-force-feedback");
    prime_current(&mut peer, &mut server);
    assert_eq!(motion(&server), (0.0, 0.0));
    cutoff_and_arm(&mut peer, &mut server);
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(10));
    assert_eq!(motion(&server), (0.0, 0.0));
    let initial = sample(&server).unwrap();
    update(&mut server, &mut peer, true);
    update(&mut server, &mut peer, true);
    let duplicates = sample(&server).unwrap();
    assert_eq!(
        (duplicates.time, duplicates.revision),
        (initial.time, initial.revision + 2)
    );
    assert_eq!(duplicates.joints, initial.joints);
    let fixture: serde_json::Value = serde_json::from_str(include_str!(
        "../../gz-physics-dart/tests/fixtures/joint_effort.json"
    ))
    .unwrap();
    let rows = fixture["observations"].as_array().unwrap();
    assert_eq!(rows.len(), 6);
    assert_eq!(XML.matches(FORCE).count(), 1);
    let mut direct = load_world_with_sdf_step(
        &XML.replace(FORCE, ""),
        "m42-direct-reference.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let direct_joint = id(&direct, "default::rotor::hinge");
    // Match the explicit Server zero-effort warmup; no DART position formula.
    direct.step_exact(10).unwrap();
    assert_eq!(direct.elapsed(), server.simulation().elapsed());
    assert_eq!(reference_motion(&direct, direct_joint), (0.0, 0.0));
    let commands = ["positive", "zero", "minus-three", "zero", "two", "zero"];
    for (command, row) in commands.into_iter().zip(rows) {
        let effort = row["effort"].as_f64().unwrap();
        admission(&mut server, &mut peer, command, effort, true);
        gz_sim_scene::queue_joint_effort(&mut direct, direct_joint, effort).unwrap();
        advance(&mut server, &mut peer, 1, true);
        direct.step_exact(1).unwrap();
        let actual = motion(&server);
        assert_eq!(actual, reference_motion(&direct, direct_joint));
        assert_eq!(server.simulation().elapsed(), direct.elapsed());
        close(actual.1, row["dq"].as_f64().unwrap());
    }
    advance(&mut server, &mut peer, 4, true);
    assert_eq!(sample(&server).unwrap().time, Duration::from_millis(20));
    admission(&mut server, &mut peer, "positive", 1.5, true);
    advance(&mut server, &mut peer, 10, true);
    let driven = motion(&server);
    assert!(driven.0 > 0.0 && driven.1 > 0.0);
    admission(&mut server, &mut peer, "zero", 0.0, true);
    advance(&mut server, &mut peer, 10, true);
    let coast = motion(&server);
    assert!(coast.0 > driven.0);
    close(coast.1, driven.1);
    admission(&mut server, &mut peer, "negative", -1.5, true);
    advance(&mut server, &mut peer, 20, true);
    assert!(motion(&server).1 < 0.0);
    admission(&mut server, &mut peer, "zero", 0.0, true);
    let before_disconnect = sample(&server).unwrap();
    let queued = server.counters().joint_states_queued;
    drop_feedback(&mut peer);
    advance(&mut server, &mut peer, 10, false);
    let disconnected = sample(&server).unwrap();
    assert_eq!(
        (disconnected.time, disconnected.revision),
        (Duration::from_millis(70), before_disconnect.revision + 1)
    );
    assert_eq!(
        server.counters().joint_states_queued,
        queued + 1,
        "no subscriber demand gate"
    );
    watch_feedback(&mut peer);
    prime_current(&mut peer, &mut server);
    cutoff_and_arm(&mut peer, &mut server);
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(80));
    update(&mut server, &mut peer, true);
    update(&mut server, &mut peer, true);
    // A real received/queued command exists before reset, but no advancing tick
    // applies it. Reset reconstructs zero command and measured history.
    admission(&mut server, &mut peer, "positive", 1.5, true);
    let old_epoch = server.simulation().epoch();
    drop_feedback(&mut peer);
    reset_loaded(&mut server);
    assert_eq!(
        (
            id(server.simulation(), "default::rotor"),
            id(server.simulation(), "default::rotor::hinge")
        ),
        (model, joint)
    );
    assert_eq!(server.simulation().epoch(), old_epoch + 1);
    watch_feedback(&mut peer);
    prime_current(&mut peer, &mut server);
    let reset = sample(&server).unwrap();
    assert_eq!((reset.time, reset.epoch), (Duration::ZERO, old_epoch + 1));
    assert_eq!(reset.joints[0].position, 0.0);
    assert_eq!(reset.joints[0].velocity, 0.0);
    cutoff_and_arm(&mut peer, &mut server);
    assert_eq!(
        motion(&server),
        (0.0, 0.0),
        "old admitted command was discarded"
    );
    update(&mut server, &mut peer, true);
    admission(&mut server, &mut peer, "negative", -1.5, true);
    advance(&mut server, &mut peer, 10, true);
    assert!(motion(&server).1 < 0.0);
    admission(&mut server, &mut peer, "zero", 0.0, true);
    drop_feedback(&mut peer);
    remove_through_canonical_service(&mut server, &network, &options, model);
    assert!(entity_by_id(server.simulation().world(), joint).is_none());
    assert!(sample(&server).is_none());
    server.update().unwrap();
    peer.command("retired");
    assert_eq!(peer.line(), "RETIRED");
    reset_loaded(&mut server);
    assert_eq!(id(server.simulation(), "default::rotor::hinge"), joint);
    watch_feedback(&mut peer);
    prime_current(&mut peer, &mut server);
    assert_eq!(motion(&server), (0.0, 0.0));
    cutoff_and_arm(&mut peer, &mut server);
    assert_eq!(motion(&server), (0.0, 0.0));
    update(&mut server, &mut peer, true);
    drop_feedback(&mut peer);
    peer.command("finish");
    assert_eq!(peer.line(), "PASS canonical joint force feedback");
    let (status, stderr) = peer.finish();
    assert!(
        status.success(),
        "{status}: {stderr}; {}",
        peer.diagnostics()
    );
    assert!(stderr.is_empty(), "{stderr}");
    let counters: Counters = server.counters();
    assert_eq!(counters.force_messages_rejected, 0);
    assert_eq!(counters.joint_state_failures, 0);
    assert_eq!(counters.invalid_joint_state_samples, 0);
    server.shutdown();
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}
