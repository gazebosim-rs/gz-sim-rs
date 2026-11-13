// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent loaded owners, listener-independent acquisition and lexical canonical Pose_V.
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_proto::{
    msgs::{
        Boolean, Entity as EntityRequest, Header, Pose, PoseV, Quaternion, Time, Vector3d,
        WorldControl, WorldReset, entity::Type as EntityType, header::Map,
    },
    prost::Message,
};
use gz_sim_scene::{
    BodyHandle, JointHandle, LocalPose, Name, PendingResetRemoval, PhysicsEngine, PosePublication,
    PoseSample, StaticPoseSample, WorldPose, load_world_with_sdf_step, queue_link_world_wrench,
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
            partition: format!("m55-{}", uuid::Uuid::new_v4()),
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
            "multiple_pose_publishers",
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
        panic!("native multiple PosePublisher peer: {message}; {details}");
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
            panic!("native multiple PosePublisher channel capture failed: {details}");
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
                "native multiple PosePublisher retained diagnostics: {}",
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

const ARTICULATED: &str =
    include_str!("../../gz-sim-scene/tests/fixtures/m31_pose_static_publisher.sdf");
fn xml(count: usize) -> String {
    let start = ARTICULATED.find("  <model ").unwrap();
    let end = ARTICULATED.rfind("</model>").unwrap() + "</model>".len();
    let models = (0..count)
        .map(|owner| {
            ARTICULATED[start..end]
                .replace("name=\"rig\"", &format!("name=\"rig{owner}\""))
                .replace("/m31/pose", &format!("/m55/pose{owner}"))
                .replace(
                    "<pose>1 2 3 0 0 0.6</pose>",
                    &format!("<pose>1 {} 3 0 0 0.6</pose>", 2 + 4 * owner),
                )
                .replace(
                    "<update_frequency>100</update_frequency>",
                    if owner.is_multiple_of(2) {
                        "<update_frequency>100</update_frequency>"
                    } else {
                        "<update_frequency>50</update_frequency>"
                    },
                )
                .replace(
                    "<static_update_frequency>25</static_update_frequency>",
                    if owner.is_multiple_of(2) {
                        "<static_update_frequency>25</static_update_frequency>"
                    } else {
                        "<static_update_frequency>100</static_update_frequency>"
                    },
                )
        })
        .collect::<String>();
    format!("{}{}{}", &ARTICULATED[..start], models, &ARTICULATED[end..])
}
fn load(text: &str) -> Simulation {
    load_world_with_sdf_step(
        text,
        "multiple-pose-publishers.sdf",
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
type Frame = (u64, [f64; 3], [f64; 4]);
#[derive(Clone, Debug, PartialEq)]
struct Reading {
    time: Duration,
    epoch: u64,
    revision: u64,
    metadata: PosePublication,
    poses: Vec<Frame>,
}
fn readings(server: &Server) -> BTreeMap<usize, Reading> {
    let mut rows = BTreeMap::new();
    for entity in server.simulation().world().iter_entities() {
        if entity.contains::<PendingResetRemoval>() {
            continue;
        }
        let Some(metadata) = entity.get::<PosePublication>() else {
            continue;
        };
        let owner: usize = metadata
            .model_name
            .strip_prefix("rig")
            .unwrap()
            .parse()
            .unwrap();
        for fixed in [false, true] {
            let sample = if fixed {
                entity.get::<StaticPoseSample>().map(|s| &s.0)
            } else {
                entity.get::<PoseSample>()
            };
            if let Some(sample) = sample {
                assert!(!sample.poses.is_empty());
                assert!(
                    rows.insert(
                        2 * owner + usize::from(fixed),
                        Reading {
                            time: sample.time,
                            epoch: sample.epoch,
                            revision: sample.revision,
                            metadata: metadata.clone(),
                            poses: sample
                                .poses
                                .iter()
                                .map(|p| (p.identity, p.pose.translation, p.pose.rotation))
                                .collect()
                        }
                    )
                    .is_none()
                );
            }
        }
    }
    rows
}
fn canonical(slot: usize, r: &Reading) -> PoseV {
    let frames = if slot.is_multiple_of(2) {
        &r.metadata.poses
    } else {
        &r.metadata.static_publication.as_ref().unwrap().poses
    };
    assert_eq!(frames.len(), r.poses.len());
    PoseV {
        header: None,
        pose: frames
            .iter()
            .zip(&r.poses)
            .map(|(frame, (identity, p, q))| {
                assert_eq!(*identity, frame.identity);
                Pose {
                    header: Some(Header {
                        stamp: Some(Time {
                            sec: r.time.as_secs() as i64,
                            nsec: r.time.subsec_nanos() as i32,
                        }),
                        data: vec![
                            Map {
                                key: "frame_id".into(),
                                value: vec![frame.frame_id.clone()],
                            },
                            Map {
                                key: "child_frame_id".into(),
                                value: vec![frame.child_frame_id.clone()],
                            },
                        ],
                    }),
                    name: frame.child_frame_id.clone(),
                    id: 0,
                    position: Some(Vector3d {
                        header: None,
                        x: p[0],
                        y: p[1],
                        z: p[2],
                    }),
                    orientation: Some(Quaternion {
                        header: None,
                        x: q[0],
                        y: q[1],
                        z: q[2],
                        w: q[3],
                    }),
                }
            })
            .collect(),
    }
}
fn receive(peer: &mut Peer, slot: usize, reading: &Reading) {
    let line = peer.line();
    let prefix = format!("PACKET {slot} ");
    let bytes = decode_hex(
        line.strip_prefix(&prefix)
            .expect("same owned channel actual packet"),
    );
    let expected = canonical(slot, reading);
    assert_eq!(
        bytes,
        expected.encode_to_vec(),
        "full actual lexical Pose_V slot {slot}"
    );
    let decoded = PoseV::decode(bytes.as_slice()).unwrap();
    assert!(decoded.header.is_none());
    assert_eq!(decoded.pose.len(), expected.pose.len());
    for (actual, expected) in decoded.pose.iter().zip(&expected.pose) {
        assert_eq!(
            actual.header.as_ref().unwrap().encode_to_vec(),
            expected.header.as_ref().unwrap().encode_to_vec()
        );
        assert_eq!((&actual.name, actual.id), (&expected.name, 0));
        let (p, q) = (
            actual.position.as_ref().unwrap(),
            actual.orientation.as_ref().unwrap(),
        );
        let (ep, eq) = (
            expected.position.as_ref().unwrap(),
            expected.orientation.as_ref().unwrap(),
        );
        assert_eq!(
            [p.x, p.y, p.z].map(f64::to_bits),
            [ep.x, ep.y, ep.z].map(f64::to_bits)
        );
        assert_eq!(
            [q.x, q.y, q.z, q.w].map(f64::to_bits),
            [eq.x, eq.y, eq.z, eq.w].map(f64::to_bits)
        );
    }
}
fn geometry(server: &Server, r: &Reading) {
    assert_eq!(r.time, server.simulation().elapsed());
    for (identity, p, q) in &r.poses {
        let local = server
            .simulation()
            .world()
            .get::<LocalPose>(entity_by_id(server.simulation().world(), *identity).unwrap())
            .unwrap()
            .0;
        assert_eq!(p.map(f64::to_bits), local.translation.map(f64::to_bits));
        assert_eq!(q.map(f64::to_bits), local.rotation.map(f64::to_bits));
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
    let mut acquired_count = 0;
    for (&slot, r) in &after {
        let acquired = before
            .get(&slot)
            .is_none_or(|old| (old.epoch, old.time, old.revision) != (r.epoch, r.time, r.revision));
        if acquired {
            acquired_count += 1;
            geometry(server, r);
        } else {
            assert_eq!(r, &before[&slot]);
        }
        if recording[slot] {
            peer.command(&format!("drain {slot} {}", usize::from(acquired)));
            if acquired {
                receive(peer, slot, r);
            }
            assert_eq!(
                peer.line(),
                format!("DRAINED {slot} {}", usize::from(acquired))
            );
        }
    }
    assert_eq!(
        server.counters().pose_publications_queued - counters.pose_publications_queued,
        acquired_count
    );
    assert_eq!(
        server.counters().invalid_pose_publication_samples,
        counters.invalid_pose_publication_samples
    );
}
fn cutoff(server: &mut Server, peer: &mut Peer, slots: &[usize], recording: &mut [bool]) {
    let now = server.simulation().elapsed().as_millis() as u64;
    let target = (now / 40 + 1) * 40;
    for &slot in slots {
        peer.command(&format!("prime {slot} {}", target * 1_000_000));
        assert_eq!(peer.line(), format!("PRIMING {slot}"));
    }
    for _ in now..target {
        step(server, peer, recording);
    }
    for &slot in slots {
        peer.command(&format!("cutoff {slot}"));
        let all = readings(server);
        let r = &all[&slot];
        assert_eq!(r.time, Duration::from_millis(target));
        receive(peer, slot, r);
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
fn real_cpp_pair_and_eight_keep_independent_split_pose_packets_motion_no_listener_reset_and_retirement()
 {
    for count in [2, 8] {
        let network = Network::new();
        let options = network.options();
        let text = xml(count);
        let mut sim = load(&text);
        let owners = (0..count)
            .map(|i| id(&sim, &format!("default::rig{i}")))
            .collect::<Vec<_>>();
        let tips = (0..count)
            .map(|i| id(&sim, &format!("default::rig{i}::tip")))
            .collect::<Vec<_>>();
        let initial = tips
            .iter()
            .map(|id| {
                *sim.world()
                    .get::<WorldPose>(entity_by_id(sim.world(), *id).unwrap())
                    .unwrap()
            })
            .collect::<Vec<_>>();
        for (index, tip) in tips.iter().enumerate() {
            // The queued Scene API intentionally admits only free single-link models.
            assert!(matches!(
                queue_link_world_wrench(
                    &mut sim,
                    *tip,
                    [2.0, 3.0, 0.0],
                    [0.0, 0.0, 0.5 + index as f64],
                ),
                Err(gz_sim_scene::InputError::NotFreeLink(id)) if id == *tip
            ));
            // This articulated-motion witness uses the existing public physics API,
            // not an expansion of queued Scene input admission.
            let body = sim
                .world()
                .get::<BodyHandle>(entity_by_id(sim.world(), *tip).unwrap())
                .unwrap()
                .id();
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .add_world_wrench(body, [2.0, 3.0, 0.0], [0.0, 0.0, 0.5 + index as f64])
                .unwrap();
            let joint = entity_by_id(
                sim.world(),
                id(&sim, &format!("default::rig{index}::tip_hinge")),
            )
            .unwrap();
            let handle = sim.world().get::<JointHandle>(joint).unwrap().id();
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
        assert_eq!(
            peer.line(),
            format!("READY multiple-pose-publishers {count}")
        );
        let all = (0..2 * count).collect::<Vec<_>>();
        let mut recording = vec![false; 2 * count];
        cutoff(&mut server, &mut peer, &all, &mut recording);
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording);
        }
        assert_eq!(readings(&server).len(), 2 * count);
        for (index, before) in initial.iter().enumerate() {
            let moved = server
                .simulation()
                .world()
                .get::<WorldPose>(entity_by_id(server.simulation().world(), tips[index]).unwrap())
                .unwrap();
            assert_ne!(moved.translation, before.translation);
            assert_ne!(moved.rotation, before.rotation);
        }
        let held = readings(&server);
        let queued = server.counters().pose_publications_queued;
        for _ in 0..3 {
            assert!(server.update().unwrap().dt.is_zero());
            assert_eq!(readings(&server), held);
        }
        assert_eq!(server.counters().pose_publications_queued, queued);
        // Neither channel's listener controls this family's acquisition clocks.
        for slot in [0, 1] {
            drop_channel(&mut peer, slot, &mut recording);
        }
        let before = readings(&server);
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording);
        }
        for slot in [0, 1] {
            assert!(readings(&server)[&slot].time > before[&slot].time);
            peer.command(&format!("watch {slot}"));
            assert_eq!(peer.line(), format!("WATCHING {slot}"));
        }
        cutoff(&mut server, &mut peer, &[0, 1], &mut recording);
        for (slot, active) in recording.iter_mut().enumerate() {
            peer.command(&format!("drop {slot}"));
            assert_eq!(peer.line(), format!("DROPPED {slot}"));
            *active = false;
        }
        let old_handles = owners
            .iter()
            .map(|id| entity_by_id(server.simulation().world(), *id).unwrap())
            .collect::<Vec<_>>();
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
            (server.simulation().epoch(), server.simulation().elapsed()),
            (epoch + 1, Duration::ZERO)
        );
        assert!(readings(&server).is_empty());
        for (index, owner) in owners.iter().enumerate() {
            assert_eq!(
                id(server.simulation(), &format!("default::rig{index}")),
                *owner
            );
            assert_ne!(
                entity_by_id(server.simulation().world(), *owner).unwrap(),
                old_handles[index]
            );
        }
        for &slot in &all {
            peer.command(&format!("watch {slot}"));
            assert_eq!(peer.line(), format!("WATCHING {slot}"));
        }
        cutoff(&mut server, &mut peer, &all, &mut recording);
        let reset = readings(&server);
        let mut fresh = network.server(load(&text), network.options()).unwrap();
        for _ in 0..40 {
            assert!(fresh.apply_local_control(WorldControl {
                pause: true,
                multi_step: 1,
                ..Default::default()
            }));
            fresh.update().unwrap();
        }
        let expected = readings(&fresh);
        for &slot in &all {
            assert_eq!(
                (
                    reset[&slot].time,
                    reset[&slot].revision,
                    &reset[&slot].poses
                ),
                (
                    expected[&slot].time,
                    expected[&slot].revision,
                    &expected[&slot].poses
                )
            );
        }
        fresh.shutdown();
        for slot in [0, 1] {
            drop_channel(&mut peer, slot, &mut recording);
        }
        remove_through_canonical_service(&mut server, &network, &options, owners[0]);
        for slot in [0, 1] {
            peer.command(&format!("retired {slot}"));
            assert_eq!(peer.line(), format!("RETIRED {slot}"));
        }
        let before = readings(&server)[&2].time;
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording);
        }
        assert!(readings(&server)[&2].time > before);
        assert!(entity_by_id(server.simulation().world(), owners[0]).is_none());
        for (index, owner) in owners.iter().enumerate().skip(1) {
            for slot in [2 * index, 2 * index + 1] {
                drop_channel(&mut peer, slot, &mut recording);
            }
            remove_through_canonical_service(&mut server, &network, &options, *owner);
            for slot in [2 * index, 2 * index + 1] {
                peer.command(&format!("retired {slot}"));
                assert_eq!(peer.line(), format!("RETIRED {slot}"));
            }
        }
        assert!(readings(&server).is_empty());
        let queued = server.counters().pose_publications_queued;
        for _ in 0..2 {
            step(&mut server, &mut peer, &recording);
        }
        assert_eq!(server.counters().pose_publications_queued, queued);
        server.shutdown();
        assert!(matches!(server.update(), Err(Error::WorldClosed)));
        peer.command("finish");
        assert_eq!(peer.line(), "PASS canonical multiple pose publishers");
        let (status, stderr) = peer.finish();
        assert!(status.success(), "{status:?}: {stderr}");
        assert!(stderr.is_empty(), "{stderr}");
    }
}
#[test]
fn cross_channel_aliases_ninth_owner_and_terminal_reset_refuse_without_mutation() {
    let network = Network::new();
    let text = xml(2);
    for (from, to) in [
        ("/m55/pose1", "/m55/pose0_static"),
        ("/m55/pose1_static", "/m55/pose0"),
        ("/m55/pose1", "/m55/pose0"),
        ("/m55/pose1_static", "/world/default/clock"),
    ] {
        let mut options = network.options();
        options.add_topic_remap(from, to).unwrap();
        assert!(matches!(
            network.server(load(&text), options),
            Err(Error::PosePublisherIdentity)
        ));
    }
    assert!(matches!(
        load_world_with_sdf_step(
            &xml(9),
            "ninth-pose.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        ),
        Err(gz_sim_scene::LoadError::Unsupported { .. })
    ));
    let mut server = network.server(load(&text), network.options()).unwrap();
    for _ in 0..40 {
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            multi_step: 1,
            ..Default::default()
        }));
        server.update().unwrap();
    }
    let before = readings(&server);
    let clock = (server.simulation().epoch(), server.simulation().elapsed());
    let queued = server.counters().pose_publications_queued;
    server
        .simulation()
        .world()
        .resource::<gz_sim_core::StepFault>()
        .report(std::io::Error::other(
            "multiple PosePublisher terminal reset fence",
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
    assert_eq!(server.counters().pose_publications_queued, queued);
    server.shutdown();
}
