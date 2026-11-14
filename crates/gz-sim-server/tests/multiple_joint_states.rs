// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent loaded JointStatePublisher ownership and canonical Rust Model transport.
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_proto::{
    msgs::{
        Axis, Boolean, Entity as EntityRequest, Header, Joint, Model, Pose, Quaternion, Time,
        Vector3d, WorldControl, WorldReset, entity::Type as EntityType,
    },
    prost::Message,
};
use gz_sim_scene::{
    BodyHandle, JointHandle, JointSample, JointStatePublication, JointStateSample, JointStateValue,
    LocalPose, Name, PendingResetRemoval, PhysicsEngine, WorldPose, load_world_with_sdf_step,
    queue_link_world_wrench,
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
            partition: format!("m57-{}", uuid::Uuid::new_v4()),
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
            "multiple_joint_states",
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
        panic!("native multiple JointStatePublisher peer: {message}; {details}");
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
            panic!("native multiple JointStatePublisher channel capture failed: {details}");
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
                "native multiple JointStatePublisher retained diagnostics: {}",
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
    let before_sample = format!("{:?}", readings(server));
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
    assert_eq!(format!("{:?}", readings(server)), before_sample);
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

const ARTICULATED: &str = include_str!("../../gz-sim-scene/tests/fixtures/m18_joint_state.sdf");
fn xml(count: usize) -> String {
    let start = ARTICULATED.find("  <model ").unwrap();
    let end = ARTICULATED.rfind("</model>").unwrap() + "</model>".len();
    let models = (0..count).map(|owner| ARTICULATED[start..end]
        .replace("name=\"arm\"", &format!("name=\"rig{owner}\""))
        .replace("<pose>0 0 1 0 0 0</pose>", &format!("<pose>0 {} 1 0 0 0</pose>", 4 * owner))
        .replace("/m18/joint_state", &format!("/m57/joint{owner}"))
        .replace("<update_rate>100</update_rate>", if owner.is_multiple_of(2) {
            "<update_rate>100</update_rate>"
        } else { "<update_rate>50</update_rate><joint_name>tip_hinge</joint_name><joint_name>tip_hinge</joint_name>" }))
        .collect::<String>();
    format!("{}{}{}", &ARTICULATED[..start], models, &ARTICULATED[end..])
}
fn load(text: &str) -> Simulation {
    load_world_with_sdf_step(
        text,
        "multiple-joint-states.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|node| node.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
#[derive(Clone, Debug)]
struct Reading {
    identity: u64,
    metadata: JointStatePublication,
    time: Duration,
    epoch: u64,
    revision: u64,
    pose: WorldPose,
    joints: Vec<JointStateValue>,
}
impl PartialEq for Reading {
    fn eq(&self, other: &Self) -> bool {
        self.identity == other.identity
            && self.metadata == other.metadata
            && self.time == other.time
            && self.epoch == other.epoch
            && self.revision == other.revision
            && self.pose.translation.map(f64::to_bits) == other.pose.translation.map(f64::to_bits)
            && self.pose.rotation.map(f64::to_bits) == other.pose.rotation.map(f64::to_bits)
            && self.joints == other.joints
    }
}
fn readings(server: &Server) -> BTreeMap<usize, Reading> {
    server
        .simulation()
        .world()
        .iter_entities()
        .filter_map(|node| {
            if node.contains::<PendingResetRemoval>() {
                return None;
            }
            let metadata = node.get::<JointStatePublication>()?;
            let sample = node.get::<JointStateSample>()?;
            let owner = metadata
                .model_name
                .strip_prefix("rig")
                .unwrap()
                .parse()
                .unwrap();
            Some((
                owner,
                Reading {
                    identity: node.get::<EntityId>().unwrap().get(),
                    metadata: metadata.clone(),
                    time: sample.time,
                    epoch: sample.epoch,
                    revision: sample.revision,
                    pose: sample.pose,
                    joints: sample.joints.clone(),
                },
            ))
        })
        .collect()
}
fn pose(p: [f64; 3], q: [f64; 4]) -> Pose {
    Pose {
        position: Some(Vector3d {
            x: p[0],
            y: p[1],
            z: p[2],
            ..Default::default()
        }),
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
fn canonical(reading: &Reading) -> Model {
    assert_eq!(reading.metadata.joints.len(), reading.joints.len());
    Model {
        header: Some(Header {
            stamp: Some(Time {
                sec: reading.time.as_secs() as i64,
                nsec: reading.time.subsec_nanos() as i32,
            }),
            data: Vec::new(),
        }),
        id: u32::try_from(reading.identity).unwrap(),
        name: reading.metadata.model_name.clone(),
        pose: Some(pose(reading.pose.translation, reading.pose.rotation)),
        joint: reading
            .metadata
            .joints
            .iter()
            .zip(&reading.joints)
            .map(|(joint, value)| {
                assert_eq!(joint.id, value.id);
                Joint {
                    id: u32::try_from(joint.id).unwrap(),
                    name: joint.name.clone(),
                    parent: joint.parent.clone(),
                    child: joint.child.clone(),
                    pose: Some(pose(joint.pose.translation, joint.pose.rotation)),
                    axis1: Some(Axis {
                        xyz: Some(Vector3d {
                            x: joint.axis[0],
                            y: joint.axis[1],
                            z: joint.axis[2],
                            ..Default::default()
                        }),
                        limit_lower: joint.lower,
                        limit_upper: joint.upper,
                        damping: joint.damping,
                        position: value.position,
                        velocity: value.velocity,
                        ..Default::default()
                    }),
                    ..Default::default()
                }
            })
            .collect(),
        ..Default::default()
    }
}
fn receive(peer: &mut Peer, slot: usize, reading: &Reading) {
    let line = peer.line();
    let prefix = format!("PACKET {slot} ");
    let bytes = decode_hex(
        line.strip_prefix(&prefix)
            .expect("actual same-owner packet"),
    );
    let expected = canonical(reading);
    assert_eq!(
        bytes,
        expected.encode_to_vec(),
        "complete canonical Rust Model for owner{slot}"
    );
    let actual = Model::decode(bytes.as_slice()).unwrap();
    assert_eq!(actual, expected);
    assert_eq!(
        actual.header.as_ref().unwrap().encode_to_vec(),
        expected.header.as_ref().unwrap().encode_to_vec()
    );
    assert_eq!(
        actual.joint.len(),
        if slot.is_multiple_of(2) { 2 } else { 1 }
    );
    assert_eq!(actual.name, format!("rig{slot}"));
    assert!(actual.id != 0 && actual.joint.windows(2).all(|pair| pair[0].id < pair[1].id));
    for (joint, expected) in actual.joint.iter().zip(&reading.metadata.joints) {
        assert_eq!(u64::from(joint.id), expected.id);
        assert_eq!(
            (&joint.name, &joint.parent, &joint.child),
            (&expected.name, &expected.parent, &expected.child)
        );
        assert!(joint.header.is_none() && joint.axis2.is_none());
        let axis = joint.axis1.as_ref().unwrap();
        assert!(axis.header.is_none() && axis.xyz.as_ref().unwrap().header.is_none());
        assert_eq!(
            axis.force, 0.0,
            "unavailable measurement remains omitted/default"
        );
        assert_eq!(
            (axis.limit_lower, axis.limit_upper),
            (f64::NEG_INFINITY, f64::INFINITY)
        );
    }
}
fn geometry(server: &Server, reading: &Reading) {
    assert_eq!(reading.time, server.simulation().elapsed());
    let world = server.simulation().world();
    let owner = entity_by_id(world, reading.identity).unwrap();
    let local = world.get::<LocalPose>(owner).unwrap().0;
    assert_eq!(
        reading.pose.translation.map(f64::to_bits),
        local.translation.map(f64::to_bits)
    );
    assert_eq!(
        reading.pose.rotation.map(f64::to_bits),
        local.rotation.map(f64::to_bits)
    );
    for value in &reading.joints {
        let joint = world
            .get::<JointSample>(entity_by_id(world, value.id).unwrap())
            .unwrap();
        assert_eq!((joint.time, joint.epoch), (reading.time, reading.epoch));
        assert_eq!(
            [value.position, value.velocity].map(f64::to_bits),
            [joint.state.position, joint.state.velocity].map(f64::to_bits)
        );
    }
}
fn step(server: &mut Server, peer: &mut Peer, recording: &[bool]) {
    let before = readings(server);
    let counters = server.counters();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        multi_step: 1,
        ..Default::default()
    }));
    server.update().unwrap();
    let after = readings(server);
    let mut acquired = 0;
    for (&slot, reading) in &after {
        let fresh = before
            .get(&slot)
            .is_none_or(|old| (old.epoch, old.revision) != (reading.epoch, reading.revision));
        if fresh {
            acquired += 1;
            geometry(server, reading);
        } else {
            assert_eq!(reading, &before[&slot]);
        }
        if recording[slot] {
            peer.command(&format!("drain {slot} {}", usize::from(fresh)));
            if fresh {
                receive(peer, slot, reading);
            }
            assert_eq!(
                peer.line(),
                format!("DRAINED {slot} {}", usize::from(fresh))
            );
        }
    }
    assert_eq!(
        server.counters().joint_states_queued - counters.joint_states_queued,
        acquired
    );
    assert_eq!(
        server.counters().invalid_joint_state_samples,
        counters.invalid_joint_state_samples
    );
}
fn cutoff(server: &mut Server, peer: &mut Peer, slots: &[usize], recording: &mut [bool]) {
    let now = server.simulation().elapsed().as_millis() as u64;
    let target = (now / 20 + 1) * 20;
    for &slot in slots {
        peer.command(&format!("prime {slot} {}", target * 1_000_000));
        assert_eq!(peer.line(), format!("PRIMING {slot}"));
    }
    for _ in now..target {
        step(server, peer, recording);
    }
    for &slot in slots {
        peer.command(&format!("cutoff {slot}"));
        let values = readings(server);
        let reading = &values[&slot];
        assert_eq!(reading.time, Duration::from_millis(target));
        receive(peer, slot, reading);
        assert_eq!(peer.line(), format!("PRIMED {slot} {}", target * 1_000_000));
        peer.command(&format!("arm {slot}"));
        assert_eq!(peer.line(), format!("ARMED {slot}"));
        recording[slot] = true;
    }
}
fn drop_channel(peer: &mut Peer, slot: usize, recording: &mut [bool]) {
    peer.command(&format!("drop {slot}"));
    assert_eq!(peer.line(), format!("DROPPED {slot}"));
    recording[slot] = false;
}
#[test]
fn actual_cpp_pair_and_eight_validate_independent_filtered_model_packets_motion_reset_and_retirement()
 {
    for count in [2, 8] {
        let network = Network::new();
        let options = network.options();
        let text = xml(count);
        let mut sim = load(&text);
        let owners = (0..count)
            .map(|index| id(&sim, &format!("default::rig{index}")))
            .collect::<Vec<_>>();
        for index in 0..count {
            let tip = id(&sim, &format!("default::rig{index}::tip"));
            let force = [2.0, 3.0, 0.0];
            let torque = [0.0, 0.0, 0.5 + index as f64];
            assert!(
                matches!(queue_link_world_wrench(&mut sim, tip, force, torque), Err(gz_sim_scene::InputError::NotFreeLink(id)) if id == tip)
            );
            let body = sim
                .world()
                .get::<BodyHandle>(entity_by_id(sim.world(), tip).unwrap())
                .unwrap()
                .id();
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .add_world_wrench(body, force, torque)
                .unwrap();
            let joint = id(&sim, &format!("default::rig{index}::tip_hinge"));
            let handle = sim
                .world()
                .get::<JointHandle>(entity_by_id(sim.world(), joint).unwrap())
                .unwrap()
                .id();
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .set_joint_velocity_motor(
                    handle,
                    Some(JointVelocityMotor {
                        target_velocity: 0.5,
                        max_force: 0.5,
                    }),
                )
                .unwrap();
        }
        let mut server = network.server(sim, options.clone()).unwrap();
        let mut peer = start_peer(&network, &options, count);
        assert_eq!(peer.line(), format!("READY multiple-joint-states {count}"));
        let all = (0..count).collect::<Vec<_>>();
        let mut recording = vec![false; count];
        cutoff(&mut server, &mut peer, &all, &mut recording);
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording);
        }
        let moved = readings(&server);
        assert_eq!(moved.len(), count);
        for reading in moved.values() {
            let tip = reading
                .metadata
                .joints
                .iter()
                .position(|joint| joint.name == "tip_hinge")
                .unwrap();
            assert!(
                reading.joints[tip].position.abs() > 0.0
                    && reading.joints[tip].velocity.abs() > 0.0
            );
        }
        // Positive sub-period elapsed suppresses even paused calls. Equal time
        // at an actual publication is separately covered by the retained tests.
        step(&mut server, &mut peer, &recording);
        let held = readings(&server);
        let queued = server.counters().joint_states_queued;
        for _ in 0..3 {
            assert!(server.update().unwrap().dt.is_zero());
            assert_eq!(readings(&server), held);
        }
        assert_eq!(server.counters().joint_states_queued, queued);
        drop_channel(&mut peer, 0, &mut recording);
        let without_listener = readings(&server)[&0].time;
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording);
        }
        assert!(readings(&server)[&0].time > without_listener);
        peer.command("watch 0");
        assert_eq!(peer.line(), "WATCHING 0");
        cutoff(&mut server, &mut peer, &[0], &mut recording);
        for &slot in &all {
            drop_channel(&mut peer, slot, &mut recording);
        }
        let old_handles = owners
            .iter()
            .map(|id| entity_by_id(server.simulation().world(), *id).unwrap())
            .collect::<Vec<_>>();
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
            (server.simulation().epoch(), server.simulation().elapsed()),
            (1, Duration::ZERO)
        );
        assert!(readings(&server).is_empty());
        for (index, identity) in owners.iter().enumerate() {
            assert_eq!(
                id(server.simulation(), &format!("default::rig{index}")),
                *identity
            );
            assert_ne!(
                entity_by_id(server.simulation().world(), *identity).unwrap(),
                old_handles[index]
            );
            peer.command(&format!("watch {index}"));
            assert_eq!(peer.line(), format!("WATCHING {index}"));
        }
        cutoff(&mut server, &mut peer, &all, &mut recording);
        let mut fresh = network.server(load(&text), network.options()).unwrap();
        for _ in 0..20 {
            assert!(fresh.apply_local_control(WorldControl {
                pause: true,
                multi_step: 1,
                ..Default::default()
            }));
            fresh.update().unwrap();
        }
        let restored = readings(&server);
        let initial = readings(&fresh);
        for &slot in &all {
            assert_eq!(restored[&slot].time, initial[&slot].time);
            assert_eq!(restored[&slot].revision, initial[&slot].revision);
            assert_eq!(
                restored[&slot].pose.translation.map(f64::to_bits),
                initial[&slot].pose.translation.map(f64::to_bits)
            );
            assert_eq!(
                restored[&slot].pose.rotation.map(f64::to_bits),
                initial[&slot].pose.rotation.map(f64::to_bits)
            );
            assert_eq!(restored[&slot].joints, initial[&slot].joints);
        }
        fresh.shutdown();
        for &slot in &all {
            drop_channel(&mut peer, slot, &mut recording);
        }
        remove_through_canonical_service(&mut server, &network, &options, owners[0]);
        peer.command("retired 0");
        assert_eq!(peer.line(), "RETIRED 0");
        for &slot in all.iter().skip(1) {
            peer.command(&format!("watch {slot}"));
            assert_eq!(peer.line(), format!("WATCHING {slot}"));
        }
        let remaining = all.iter().copied().skip(1).collect::<Vec<_>>();
        cutoff(&mut server, &mut peer, &remaining, &mut recording);
        let before = readings(&server)[&1].time;
        for _ in 0..20 {
            step(&mut server, &mut peer, &recording);
        }
        assert!(readings(&server)[&1].time > before);
        for &slot in &remaining {
            drop_channel(&mut peer, slot, &mut recording);
        }
        for (slot, identity) in owners.iter().enumerate().skip(1) {
            remove_through_canonical_service(&mut server, &network, &options, *identity);
            peer.command(&format!("retired {slot}"));
            assert_eq!(peer.line(), format!("RETIRED {slot}"));
        }
        assert!(readings(&server).is_empty());
        let queued = server.counters().joint_states_queued;
        for _ in 0..2 {
            step(&mut server, &mut peer, &recording);
        }
        assert_eq!(server.counters().joint_states_queued, queued);
        assert_eq!(server.counters().joint_state_failures, 0);
        assert_eq!(server.counters().invalid_joint_state_samples, 0);
        server.shutdown();
        assert!(matches!(server.update(), Err(Error::WorldClosed)));
        peer.command("finish");
        assert_eq!(peer.line(), "PASS canonical multiple joint states");
        let (status, stderr) = peer.finish();
        assert!(status.success(), "{status:?}: {stderr}");
        assert!(stderr.is_empty(), "{stderr}");
    }
}
#[test]
fn duplicate_remapped_reserved_endpoints_ninth_owner_and_global_terminal_reset_refuse() {
    let network = Network::new();
    for target in [
        "/m57/joint0",
        "/world/default/clock",
        "/world/default/control",
    ] {
        let mut options = network.options();
        options.add_topic_remap("/m57/joint1", target).unwrap();
        assert!(matches!(
            network.server(load(&xml(2)), options),
            Err(Error::JointStateIdentity)
        ));
    }
    assert!(
        gz_sim_scene::load_world_with_sdf_step(
            &xml(9),
            "ninth-joint-state.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
    let mut server = network.server(load(&xml(2)), network.options()).unwrap();
    for _ in 0..20 {
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            multi_step: 1,
            ..Default::default()
        }));
        server.update().unwrap();
    }
    let before = readings(&server);
    let clock = (server.simulation().epoch(), server.simulation().elapsed());
    server
        .simulation()
        .world()
        .resource::<gz_sim_core::StepFault>()
        .report(std::io::Error::other(
            "multiple JointStatePublisher terminal reset fence",
        ));
    assert!(!server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(
        server.update(),
        Err(Error::Step(gz_sim_core::StepError::Faulted))
    ));
    assert_eq!(readings(&server), before);
    assert_eq!(
        (server.simulation().epoch(), server.simulation().elapsed()),
        clock
    );
}
