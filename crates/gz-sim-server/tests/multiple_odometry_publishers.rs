// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Loaded independent odometry owners and three canonical transport channels.
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_proto::{
    msgs::{
        Boolean, Entity as EntityRequest, FloatV, Header, Odometry, OdometryWithCovariance, Pose,
        PoseV, PoseWithCovariance, Quaternion, Time, Twist, TwistWithCovariance, Vector3d,
        WorldControl, WorldReset, entity::Type as EntityType, header::Map,
    },
    prost::Message,
};
use gz_sim_scene::{
    GroundTruthOdometryPublication as Metadata, GroundTruthOdometrySample as Sample, Name,
    PhysicsEngine, load_world_seeded, queue_link_world_wrench,
};
use gz_sim_server::{Error, InterestPolicy, Server, ServerOptions};
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
            partition: format!("m56-{}", uuid::Uuid::new_v4()),
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
            "multiple_odometry_publishers",
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
        panic!("native multiple odometry peer: {message}; {details}");
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
            panic!("native multiple odometry channel capture failed: {details}");
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
                "native multiple odometry retained diagnostics: {}",
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

fn xml(count: usize) -> String {
    let models = (0..count).map(|index| format!("<model name='rig{index}'><pose>1 {} 3 0 0 0.4</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='shape'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision></link><plugin filename='gz-sim-odometry-publisher-system' name='gz::sim::systems::OdometryPublisher'><dimensions>{}</dimensions><odom_publish_frequency>{}</odom_publish_frequency><xyz_offset>0.1 0.2 0</xyz_offset><odom_topic>/m56/odom{index}</odom_topic><odom_covariance_topic>/m56/cov{index}</odom_covariance_topic><tf_topic>/m56/tf{index}</tf_topic><odom_frame>m56/odom{index}</odom_frame><robot_base_frame>m56/base{index}</robot_base_frame></plugin></model>", index * 4, if index.is_multiple_of(2) { 2 } else { 3 }, if index.is_multiple_of(2) { 100 } else { 50 })).collect::<String>();
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ignored'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn load(text: &str) -> Simulation {
    load_world_seeded(
        text,
        "multiple-odometry.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        [56; 32],
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
#[derive(Clone, Debug, Eq, PartialEq)]
struct Snapshot {
    time: Duration,
    epoch: u64,
    revision: u64,
    pose: [u64; 7],
    linear: [u64; 3],
    angular: [u64; 3],
}
fn snapshot(sample: &Sample) -> Snapshot {
    Snapshot {
        time: sample.time,
        epoch: sample.epoch,
        revision: sample.revision,
        pose: [
            sample.pose.translation[0].to_bits(),
            sample.pose.translation[1].to_bits(),
            sample.pose.translation[2].to_bits(),
            sample.pose.rotation[0].to_bits(),
            sample.pose.rotation[1].to_bits(),
            sample.pose.rotation[2].to_bits(),
            sample.pose.rotation[3].to_bits(),
        ],
        linear: sample.linear.map(f64::to_bits),
        angular: sample.angular.map(f64::to_bits),
    }
}
#[derive(Clone, Debug)]
struct Reading {
    metadata: Metadata,
    sample: Sample,
}
impl PartialEq for Reading {
    fn eq(&self, other: &Self) -> bool {
        self.metadata == other.metadata && snapshot(&self.sample) == snapshot(&other.sample)
    }
}
fn readings(server: &Server) -> BTreeMap<usize, Reading> {
    server
        .simulation()
        .world()
        .iter_entities()
        .filter_map(|node| {
            let metadata = node.get::<Metadata>()?;
            let index = metadata.model_name.strip_prefix("rig")?.parse().unwrap();
            Some((
                index,
                Reading {
                    metadata: metadata.clone(),
                    sample: *node.get::<Sample>()?,
                },
            ))
        })
        .collect()
}
fn vector([x, y, z]: [f64; 3]) -> Vector3d {
    Vector3d {
        header: None,
        x,
        y,
        z,
    }
}
fn expected(reading: &Reading, channel: usize) -> Vec<u8> {
    let m = &reading.metadata;
    let s = &reading.sample;
    let header = Header {
        stamp: Some(Time {
            sec: s.time.as_secs().try_into().unwrap(),
            nsec: s.time.subsec_nanos() as i32,
        }),
        data: vec![
            Map {
                key: "frame_id".into(),
                value: vec![m.odom_frame.clone()],
            },
            Map {
                key: "child_frame_id".into(),
                value: vec![m.robot_base_frame.clone()],
            },
        ],
    };
    let q = s.pose.rotation;
    let pose = Pose {
        header: None,
        name: String::new(),
        id: 0,
        position: Some(vector(s.pose.translation)),
        orientation: Some(Quaternion {
            header: None,
            x: q[0],
            y: q[1],
            z: q[2],
            w: q[3],
        }),
    };
    let twist = Twist {
        linear: Some(vector(s.linear)),
        angular: Some(vector(s.angular)),
        ..Default::default()
    };
    let variance = m.gaussian_noise.variance();
    let covariance = FloatV {
        data: (0..36)
            .map(|index| if index % 7 == 0 { variance } else { 0.0 })
            .collect(),
        ..Default::default()
    };
    match channel {
        0 => Odometry {
            header: Some(header),
            pose: Some(pose),
            twist: Some(twist),
        }
        .encode_to_vec(),
        1 => OdometryWithCovariance {
            header: Some(header),
            pose_with_covariance: Some(PoseWithCovariance {
                pose: Some(pose),
                covariance: Some(covariance.clone()),
            }),
            twist_with_covariance: Some(TwistWithCovariance {
                twist: Some(twist),
                covariance: Some(covariance),
            }),
        }
        .encode_to_vec(),
        2 => PoseV {
            header: None,
            pose: vec![Pose {
                header: Some(header),
                ..pose
            }],
        }
        .encode_to_vec(),
        _ => unreachable!(),
    }
}
fn receive(peer: &mut Peer, slot: usize, reading: &Reading) {
    let line = peer.line();
    let wire = decode_hex(
        line.strip_prefix(&format!("PACKET {slot} "))
            .expect("exact owner/channel packet prefix"),
    );
    assert_eq!(
        wire,
        expected(reading, slot % 3),
        "full typed canonical wire at slot {slot}"
    );
    // Independently decode the peer's actual bytes; complete expected-message
    // equality includes wrapper presence, all scalar bits and both covariance arrays.
    match slot % 3 {
        0 => {
            let frame = Odometry::decode(wire.as_slice()).unwrap();
            assert_eq!(frame.encode_to_vec(), wire);
            assert_eq!(frame.header.unwrap().data.len(), 2);
        }
        1 => {
            let frame = OdometryWithCovariance::decode(wire.as_slice()).unwrap();
            assert_eq!(frame.encode_to_vec(), wire);
            assert_eq!(
                frame
                    .pose_with_covariance
                    .unwrap()
                    .covariance
                    .unwrap()
                    .data
                    .len(),
                36
            );
            assert_eq!(
                frame
                    .twist_with_covariance
                    .unwrap()
                    .covariance
                    .unwrap()
                    .data
                    .len(),
                36
            );
        }
        _ => {
            let frame = PoseV::decode(wire.as_slice()).unwrap();
            assert_eq!(frame.encode_to_vec(), wire);
            assert!(frame.header.is_none());
            assert_eq!(frame.pose.len(), 1);
            assert!(frame.pose[0].header.is_some());
            assert_eq!(frame.pose[0].id, 0);
        }
    }
}
fn step(server: &mut Server, peer: &mut Peer, recording: &[bool]) {
    let before = readings(server);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        multi_step: 1,
        ..Default::default()
    }));
    server.update().unwrap();
    let after = readings(server);
    for (&owner, reading) in &after {
        let changed = before.get(&owner).is_none_or(|old| {
            (old.sample.epoch, old.sample.revision)
                != (reading.sample.epoch, reading.sample.revision)
        });
        if !changed {
            assert_eq!(Some(reading), before.get(&owner));
        } else {
            assert_eq!(reading.sample.time, server.simulation().elapsed());
            let period = if owner.is_multiple_of(2) { 10 } else { 20 };
            assert!(reading.sample.time.as_millis().is_multiple_of(period));
            if let Some(old) = before.get(&owner) {
                assert_eq!(reading.sample.revision, old.sample.revision + 1);
            }
        }
        for channel in 0..3 {
            let slot = owner * 3 + channel;
            if recording[slot] {
                peer.command(&format!("drain {slot} {}", usize::from(changed)));
                if changed {
                    receive(peer, slot, reading);
                }
                assert_eq!(
                    peer.line(),
                    format!("DRAINED {slot} {}", usize::from(changed))
                );
            }
        }
    }
}
fn cutoff(server: &mut Server, peer: &mut Peer, slots: &[usize], recording: &mut [bool]) {
    let now: u64 = server
        .simulation()
        .elapsed()
        .as_millis()
        .try_into()
        .unwrap();
    let target = (now / 20 + 1) * 20;
    for &slot in slots {
        peer.command(&format!("prime {slot} {}", target * 1_000_000));
        assert_eq!(peer.line(), format!("PRIMING {slot}"));
    }
    for _ in now..target {
        step(server, peer, recording);
    }
    let at = readings(server);
    for &slot in slots {
        assert_eq!(at[&(slot / 3)].sample.time, Duration::from_millis(target));
        peer.command(&format!("cutoff {slot}"));
        receive(peer, slot, &at[&(slot / 3)]);
        assert_eq!(peer.line(), format!("PRIMED {slot} {}", target * 1_000_000));
        peer.command(&format!("arm {slot}"));
        assert_eq!(peer.line(), format!("ARMED {slot}"));
        recording[slot] = true;
    }
}
#[test]
fn real_cpp_pair_and_eight_preserve_all_three_exact_channels_demand_reset_and_retirement() {
    for count in [2, 8] {
        let network = Network::new();
        let options = network.options();
        let mut sim = load(&xml(count));
        let owners = (0..count)
            .map(|index| id(&sim, &format!("default::rig{index}")))
            .collect::<Vec<_>>();
        for index in 0..count {
            let body = id(&sim, &format!("default::rig{index}::body"));
            // This is the existing admitted single-link free-body input domain.
            queue_link_world_wrench(
                &mut sim,
                body,
                [2.0 + index as f64, 1.0, 0.5],
                [0.1, 0.2, 0.3 + index as f64],
            )
            .unwrap();
        }
        let mut server = network.server(sim, options.clone()).unwrap();
        server.set_interest_policy(InterestPolicy::Always);
        let mut peer = start_peer(&network, &options, count);
        assert_eq!(
            peer.line(),
            format!("READY multiple-odometry-publishers {count}")
        );
        let mut recording = vec![false; count * 3];
        let slots = (0..count * 3).collect::<Vec<_>>();
        cutoff(&mut server, &mut peer, &slots, &mut recording);
        for _ in 0..60 {
            step(&mut server, &mut peer, &recording);
        }
        for (index, identity) in owners.iter().enumerate() {
            let model = entity_by_id(server.simulation().world(), *identity).unwrap();
            let actual = server
                .simulation()
                .world()
                .get::<gz_sim_scene::WorldPose>(model)
                .unwrap();
            assert!(
                actual.translation[0] > 1.0,
                "real free-body force must translate owner {index}"
            );
        }
        let pause = readings(&server);
        let time = server.simulation().elapsed();
        for _ in 0..3 {
            server.update().unwrap();
        }
        assert_eq!(server.simulation().elapsed(), time);
        assert_eq!(readings(&server), pause);
        // Drop just one channel: acquisition/history and the other two channels continue.
        peer.command("drop 1");
        assert_eq!(peer.line(), "DROPPED 1");
        recording[1] = false;
        let revision = readings(&server)[&0].sample.revision;
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording);
        }
        assert!(readings(&server)[&0].sample.revision > revision);
        peer.command("watch 1");
        assert_eq!(peer.line(), "WATCHING 1");
        cutoff(&mut server, &mut peer, &[1], &mut recording);
        for (slot, active) in recording.iter_mut().enumerate() {
            peer.command(&format!("drop {slot}"));
            assert_eq!(peer.line(), format!("DROPPED {slot}"));
            *active = false;
        }
        let no_interest = readings(&server)[&0].sample.revision;
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording);
        }
        assert!(readings(&server)[&0].sample.revision > no_interest);
        // Loaded reset restores IDs/history while subscriber leases are absent.
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        }));
        server.update().unwrap();
        assert_eq!(server.simulation().epoch(), 1);
        assert!(readings(&server).is_empty());
        for (index, identity) in owners.iter().enumerate() {
            assert_eq!(
                id(server.simulation(), &format!("default::rig{index}")),
                *identity
            );
        }
        for slot in 0..count * 3 {
            peer.command(&format!("watch {slot}"));
            assert_eq!(peer.line(), format!("WATCHING {slot}"));
        }
        cutoff(&mut server, &mut peer, &slots, &mut recording);
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording);
        }
        let removed = count - 1;
        for channel in 0..3 {
            let slot = removed * 3 + channel;
            peer.command(&format!("drop {slot}"));
            assert_eq!(peer.line(), format!("DROPPED {slot}"));
            recording[slot] = false;
        }
        remove_through_canonical_service(&mut server, &network, &options, owners[removed]);
        for channel in 0..3 {
            let slot = removed * 3 + channel;
            peer.command(&format!("retired {slot}"));
            assert_eq!(peer.line(), format!("RETIRED {slot}"));
        }
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording);
        }
        for owner in (0..removed).rev() {
            for channel in 0..3 {
                let slot = owner * 3 + channel;
                peer.command(&format!("drop {slot}"));
                assert_eq!(peer.line(), format!("DROPPED {slot}"));
                recording[slot] = false;
            }
            remove_through_canonical_service(&mut server, &network, &options, owners[owner]);
            for channel in 0..3 {
                let slot = owner * 3 + channel;
                peer.command(&format!("retired {slot}"));
                assert_eq!(peer.line(), format!("RETIRED {slot}"));
            }
        }
        assert!(readings(&server).is_empty());
        peer.command("finish");
        assert_eq!(peer.line(), "PASS canonical multiple odometry publishers");
        let (status, stderr) = peer.finish();
        assert!(status.success(), "{status}: {stderr}");
        assert!(stderr.is_empty());
    }
}
#[test]
fn remapped_triplet_aliases_ninth_owner_and_original_peer_reservations_fail_atomically() {
    let network = Network::new();
    for from in ["/m56/odom1", "/m56/cov1", "/m56/tf1"] {
        let mut options = network.options();
        options.add_topic_remap(from, "/m56/odom0").unwrap();
        assert!(matches!(
            network.server(load(&xml(2)), options),
            Err(Error::GroundTruthOdometryIdentity)
        ));
    }
    let text = xml(2);
    let malformed = text.replace(
        "<robot_base_frame>m56/base1</robot_base_frame>",
        "<robot_base_frame>bad frame</robot_base_frame>",
    );
    assert!(
        gz_sim_scene::load_world_seeded(
            &malformed,
            "invalid-frame.sdf",
            "default",
            Some(Duration::from_millis(1)),
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
            [56; 32]
        )
        .is_err()
    );
    assert!(
        gz_sim_scene::load_world_seeded(
            &xml(9),
            "ninth.sdf",
            "default",
            Some(Duration::from_millis(1)),
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
            [56; 32]
        )
        .is_err()
    );
    let empty = network.server(load(&xml(0)), network.options()).unwrap();
    assert_eq!(empty.counters().ground_truth_odometry_queued, 0);
}
