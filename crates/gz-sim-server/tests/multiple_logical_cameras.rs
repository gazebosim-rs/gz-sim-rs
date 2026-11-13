// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Actual paired canonical channels and independent loaded logical-camera owners.
use gz_sensors::logical_camera::{
    LogicalCameraModel, LogicalCameraPose, LogicalCameraReading, logical_camera,
};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_proto::{
    msgs::{
        Boolean, Entity as EntityRequest, Header, LogicalCameraImage, LogicalCameraSensor, Pose,
        Quaternion, Time, Vector3d, WorldControl, WorldReset, entity::Type as EntityType,
        header::Map, logical_camera_image,
    },
    prost::Message,
};
use gz_sim_scene::{
    EntityKind, LogicalCameraPublication, LogicalCameraSample, Name, PhysicsEngine, WorldPose,
    load_world_seeded, queue_link_world_wrench,
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
            partition: format!("m53-{}", uuid::Uuid::new_v4()),
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
            "multiple_logical_cameras",
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
        panic!("native multiple logical-camera peer: {message}; {details}");
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
            panic!("native multiple logical-camera channel capture failed: {details}");
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
                "native multiple logical-camera retained diagnostics: {}",
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

const PERIODS: [u64; 8] = [10, 20, 40, 10, 20, 40, 10, 20];
fn xml(count: usize) -> String {
    let mut text = String::from(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='step' type='ode'><max_step_size>0.001</max_step_size></physics>",
    );
    for index in 0..count {
        let rate = 1000 / PERIODS[index % 8];
        let y = 3 * index;
        let yaw = if index.is_multiple_of(2) { 0.0 } else { 0.3 };
        let near = 0.5 + 0.02 * index as f64;
        let far = 10 + index;
        let hfov = 1.2 + 0.02 * index as f64;
        let aspect = 1.0 + 0.1 * index as f64;
        text.push_str(&format!("<model name='rig{index}'><pose>0 {y} 3 0 0 0</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='shape'><geometry><box><size>.2 .2 .2</size></box></geometry></collision><sensor name='camera' type='logical_camera'><pose>-1 0 0 0 0 {yaw}</pose><update_rate>{rate}</update_rate><topic>/m53/camera{index}</topic><frame_id>rig{index}/body/camera</frame_id><logical_camera><near>{near}</near><far>{far}</far><horizontal_fov>{hfov}</horizontal_fov><aspect_ratio>{aspect}</aspect_ratio></logical_camera></sensor></link></model>"));
    }
    text.push_str("<model name='target'><static>true</static><pose>4 0 3 .1 -.2 .5</pose><link name='body'><collision name='shape'><geometry><box><size>.1 .1 .1</size></box></geometry></collision></link></model><model name='behind'><static>true</static><pose>-4 0 3 0 0 .3</pose><link name='body'><collision name='shape'><geometry><box><size>.1 .1 .1</size></box></geometry></collision></link></model></world></sdf>");
    text
}
fn load(text: &str) -> Simulation {
    load_world_seeded(
        text,
        "m53-server.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        [53; 32],
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
#[derive(Clone, Debug, PartialEq)]
struct Reading {
    time: Duration,
    epoch: u64,
    metadata: LogicalCameraPublication,
    value: LogicalCameraReading,
}
fn readings(server: &Server) -> BTreeMap<usize, Reading> {
    server
        .simulation()
        .world()
        .iter_entities()
        .filter_map(|e| {
            let metadata = e.get::<LogicalCameraPublication>()?;
            let index = metadata.topic.strip_prefix("/m53/camera")?.parse().unwrap();
            let s = e.get::<LogicalCameraSample>()?;
            Some((
                index,
                Reading {
                    time: s.time,
                    epoch: s.epoch,
                    metadata: metadata.clone(),
                    value: s.reading.as_ref().unwrap().clone(),
                },
            ))
        })
        .collect()
}
fn canonical_pose(p: LogicalCameraPose) -> Pose {
    Pose {
        header: None,
        name: String::new(),
        id: 0,
        position: Some(Vector3d {
            header: None,
            x: p.translation[0],
            y: p.translation[1],
            z: p.translation[2],
        }),
        orientation: Some(Quaternion {
            header: None,
            x: p.rotation[0],
            y: p.rotation[1],
            z: p.rotation[2],
            w: p.rotation[3],
        }),
    }
}
fn canonical(r: &Reading, sequence: u64) -> (LogicalCameraImage, LogicalCameraSensor) {
    let frame = Map {
        key: "frame_id".into(),
        value: vec![r.metadata.frame_id.clone()],
    };
    let stamp = Some(Time {
        sec: r.time.as_secs().try_into().unwrap(),
        nsec: r.time.subsec_nanos() as i32,
    });
    (
        LogicalCameraImage {
            header: Some(Header {
                stamp,
                data: vec![
                    frame.clone(),
                    Map {
                        key: "seq".into(),
                        value: vec![sequence.to_string()],
                    },
                ],
            }),
            pose: Some(canonical_pose(r.value.pose)),
            model: r
                .value
                .models
                .iter()
                .map(|m| logical_camera_image::Model {
                    name: m.name.clone(),
                    pose: Some(canonical_pose(m.pose)),
                })
                .collect(),
        },
        LogicalCameraSensor {
            header: Some(Header {
                stamp,
                data: vec![frame],
            }),
            near_clip: r.metadata.config.near,
            far_clip: r.metadata.config.far,
            horizontal_fov: r.metadata.config.horizontal_fov,
            aspect_ratio: r.metadata.config.aspect_ratio,
        },
    )
}
fn receive(peer: &mut Peer, slot: usize, reading: &Reading) -> Option<u64> {
    let line = peer.line();
    let prefix = format!("PACKET {slot} ");
    let bytes = decode_hex(
        line.strip_prefix(&prefix)
            .unwrap_or_else(|| panic!("{line}")),
    );
    if slot.is_multiple_of(2) {
        let message = LogicalCameraImage::decode(bytes.as_slice()).unwrap();
        let sequence = message.header.as_ref().unwrap().data[1].value[0]
            .parse()
            .unwrap();
        let expected = canonical(reading, sequence).0;
        assert_eq!(message, expected);
        assert_eq!(bytes, expected.encode_to_vec());
        assert_eq!(
            message.header.unwrap().encode_to_vec(),
            expected.header.unwrap().encode_to_vec()
        );
        Some(sequence)
    } else {
        let message = LogicalCameraSensor::decode(bytes.as_slice()).unwrap();
        let expected = canonical(reading, 0).1;
        assert_eq!(message, expected);
        assert_eq!(bytes, expected.encode_to_vec());
        assert_eq!(
            message.header.unwrap().encode_to_vec(),
            expected.header.unwrap().encode_to_vec()
        );
        None
    }
}
fn actual_geometry(server: &Server, r: &Reading) {
    // Only fresh samples are compared to current poses. Held samples are instead
    // compared in full against their own preceding acquisition below.
    assert_eq!(r.time, server.simulation().elapsed());
    let world = server.simulation().world();
    let owner = world
        .iter_entities()
        .find(|e| e.get::<LogicalCameraPublication>() == Some(&r.metadata))
        .unwrap();
    let pose = *owner.get::<WorldPose>().unwrap();
    let camera = LogicalCameraPose {
        translation: pose.translation,
        rotation: pose.rotation,
    };
    let models: Vec<_> = world
        .iter_entities()
        .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Model))
        .map(|e| {
            let name = e
                .get::<Name>()
                .unwrap()
                .0
                .strip_prefix("default::")
                .unwrap()
                .to_owned();
            let p = e.get::<WorldPose>().unwrap();
            LogicalCameraModel {
                name,
                pose: LogicalCameraPose {
                    translation: p.translation,
                    rotation: p.rotation,
                },
            }
        })
        .collect();
    assert_eq!(
        r.value,
        logical_camera(r.metadata.config, camera, &models).unwrap()
    );
}
fn step(server: &mut Server, peer: &mut Peer, recording: &[bool], sequences: &mut [u64]) {
    let before = readings(server);
    let counters = server.counters();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        multi_step: 1,
        ..Default::default()
    }));
    server.update().unwrap();
    let after = readings(server);
    let mut changed = 0;
    for (&index, r) in &after {
        let acquired = before
            .get(&index)
            .is_none_or(|old| (old.epoch, old.time) != (r.epoch, r.time));
        if acquired {
            changed += 1;
            actual_geometry(server, r);
        } else {
            assert_eq!(r, &before[&index]);
        }
        for slot in [2 * index, 2 * index + 1] {
            if recording[slot] {
                peer.command(&format!("drain {slot} {}", usize::from(acquired)));
                if acquired && let Some(seq) = receive(peer, slot, r) {
                    assert_eq!(seq, sequences[slot]);
                    sequences[slot] += 1;
                }
                assert_eq!(
                    peer.line(),
                    format!("DRAINED {slot} {}", usize::from(acquired))
                );
            }
        }
    }
    assert_eq!(
        server.counters().logical_camera_images_queued - counters.logical_camera_images_queued,
        changed
    );
    assert_eq!(
        server.counters().logical_camera_frustums_queued - counters.logical_camera_frustums_queued,
        changed
    );
}
fn cutoff(
    server: &mut Server,
    peer: &mut Peer,
    slots: &[usize],
    recording: &mut [bool],
    sequences: &mut [u64],
) {
    let now = server.simulation().elapsed().as_millis() as u64;
    let target = (now / 40 + 1) * 40;
    for &slot in slots {
        peer.command(&format!("prime {slot} {}", target * 1_000_000));
        assert_eq!(peer.line(), format!("PRIMING {slot}"));
    }
    for _ in now..target {
        step(server, peer, recording, sequences);
    }
    for &slot in slots {
        peer.command(&format!("cutoff {slot}"));
        let values = readings(server);
        let r = &values[&(slot / 2)];
        assert_eq!(r.time, Duration::from_millis(target));
        if let Some(seq) = receive(peer, slot, r) {
            sequences[slot] = seq + 1;
        }
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
fn real_cpp_pair_and_eight_keep_independent_paired_packets_or_demand_resets_and_retirement() {
    for count in [2, 8] {
        let network = Network::new();
        let options = network.options();
        let text = xml(count);
        let mut sim = load(&text);
        let initial = (0..count)
            .map(|i| {
                *sim.world()
                    .get::<WorldPose>(
                        entity_by_id(sim.world(), id(&sim, &format!("default::rig{i}::body")))
                            .unwrap(),
                    )
                    .unwrap()
            })
            .collect::<Vec<_>>();
        for i in 0..count {
            let body = id(&sim, &format!("default::rig{i}::body"));
            queue_link_world_wrench(&mut sim, body, [8.0, -4.0, 2.0], [2.0, -3.0, 4.0]).unwrap();
        }
        let mut server = network.server(sim, options.clone()).unwrap();
        let identities = (0..count)
            .map(|i| {
                id(
                    server.simulation(),
                    &format!("default::rig{i}::body::camera"),
                )
            })
            .collect::<Vec<_>>();
        let mut peer = start_peer(&network, &options, count);
        assert_eq!(
            peer.line(),
            format!("READY multiple-logical-cameras {count}")
        );
        let mut recording = vec![false; 2 * count];
        let mut sequences = vec![0; 2 * count];
        let all = (0..2 * count).collect::<Vec<_>>();
        cutoff(&mut server, &mut peer, &all, &mut recording, &mut sequences);
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording, &mut sequences);
        }
        assert_eq!(readings(&server).len(), count);
        for (i, before) in initial.iter().enumerate() {
            let p = server
                .simulation()
                .world()
                .get::<WorldPose>(
                    entity_by_id(
                        server.simulation().world(),
                        id(server.simulation(), &format!("default::rig{i}::body")),
                    )
                    .unwrap(),
                )
                .unwrap();
            assert_ne!(
                p.translation.map(f64::to_bits),
                before.translation.map(f64::to_bits)
            );
            assert_ne!(
                p.rotation.map(f64::to_bits),
                before.rotation.map(f64::to_bits)
            );
        }
        let held = readings(&server);
        let counters = server.counters();
        for _ in 0..3 {
            assert!(server.update().unwrap().dt.is_zero());
            assert_eq!(readings(&server), held);
        }
        assert_eq!(
            server.counters().logical_camera_images_queued,
            counters.logical_camera_images_queued
        );
        assert_eq!(
            server.counters().logical_camera_frustums_queued,
            counters.logical_camera_frustums_queued
        );
        // Frustum-only demand still drives the same owner's image production.
        drop_channel(&mut peer, 0, &mut recording);
        let before = readings(&server)[&0].time;
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording, &mut sequences);
        }
        assert!(readings(&server)[&0].time > before);
        peer.command("watch 0");
        assert_eq!(peer.line(), "WATCHING 0");
        cutoff(&mut server, &mut peer, &[0], &mut recording, &mut sequences);
        // Image-only demand has the symmetric effect, with no frustum sequence.
        drop_channel(&mut peer, 1, &mut recording);
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording, &mut sequences);
        }
        drop_channel(&mut peer, 0, &mut recording);
        let deadline = Instant::now() + Duration::from_secs(5);
        while readings(&server).contains_key(&0) {
            assert!(Instant::now() < deadline);
            server.update().unwrap();
            std::thread::yield_now();
        }
        let before = readings(&server)[&1].time;
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording, &mut sequences);
            assert!(!readings(&server).contains_key(&0));
        }
        assert!(readings(&server)[&1].time > before);
        for slot in [0, 1] {
            peer.command(&format!("watch {slot}"));
            assert_eq!(peer.line(), format!("WATCHING {slot}"));
        }
        cutoff(
            &mut server,
            &mut peer,
            &[0, 1],
            &mut recording,
            &mut sequences,
        );
        for slot in &all {
            drop_channel(&mut peer, *slot, &mut recording);
        }
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
        for (i, identity) in identities.iter().enumerate() {
            assert_eq!(
                id(
                    server.simulation(),
                    &format!("default::rig{i}::body::camera")
                ),
                *identity
            );
        }
        for slot in &all {
            peer.command(&format!("watch {slot}"));
            assert_eq!(peer.line(), format!("WATCHING {slot}"));
        }
        cutoff(&mut server, &mut peer, &all, &mut recording, &mut sequences);
        let reset = readings(&server);
        let mut fresh = network.server(load(&text), network.options()).unwrap();
        fresh.set_interest_policy(InterestPolicy::Always);
        for _ in 0..40 {
            assert!(fresh.apply_local_control(WorldControl {
                pause: true,
                multi_step: 1,
                ..Default::default()
            }));
            fresh.update().unwrap();
        }
        let expected = readings(&fresh);
        for i in 0..count {
            assert_eq!(reset[&i].time, expected[&i].time);
            assert_eq!(reset[&i].value, expected[&i].value);
        }
        fresh.shutdown();
        // Removing a model input changes subsequent fresh images, not a cached frame.
        let target = id(server.simulation(), "default::target");
        remove_through_canonical_service(&mut server, &network, &options, target);
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording, &mut sequences);
        }
        assert!(
            readings(&server)
                .values()
                .all(|r| r.value.models.iter().all(|m| m.name != "target"))
        );
        for slot in [0, 1] {
            drop_channel(&mut peer, slot, &mut recording);
        }
        let model = id(server.simulation(), "default::rig0");
        remove_through_canonical_service(&mut server, &network, &options, model);
        for slot in [0, 1] {
            peer.command(&format!("retired {slot}"));
            assert_eq!(peer.line(), format!("RETIRED {slot}"));
        }
        let before = readings(&server)[&1].time;
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording, &mut sequences);
        }
        assert!(readings(&server)[&1].time > before);
        assert!(entity_by_id(server.simulation().world(), identities[0]).is_none());
        for i in 1..count {
            for slot in [2 * i, 2 * i + 1] {
                drop_channel(&mut peer, slot, &mut recording);
            }
            let model = id(server.simulation(), &format!("default::rig{i}"));
            remove_through_canonical_service(&mut server, &network, &options, model);
            for slot in [2 * i, 2 * i + 1] {
                peer.command(&format!("retired {slot}"));
                assert_eq!(peer.line(), format!("RETIRED {slot}"));
            }
        }
        assert!(readings(&server).is_empty());
        let counters = server.counters();
        for _ in 0..2 {
            assert!(server.apply_local_control(WorldControl {
                pause: true,
                multi_step: 1,
                ..Default::default()
            }));
            server.update().unwrap();
        }
        assert_eq!(
            server.counters().logical_camera_images_queued,
            counters.logical_camera_images_queued
        );
        assert_eq!(
            server.counters().logical_camera_frustums_queued,
            counters.logical_camera_frustums_queued
        );
        server.shutdown();
        assert!(matches!(server.update(), Err(Error::WorldClosed)));
        peer.command("finish");
        assert_eq!(peer.line(), "PASS canonical multiple logical cameras");
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
        ("/m53/camera1", "/m53/camera0/frustum"),
        ("/m53/camera1/frustum", "/m53/camera0"),
        ("/m53/camera1", "/m53/camera0"),
        ("/m53/camera1/frustum", "/world/default/clock"),
    ] {
        let mut options = network.options();
        options.add_topic_remap(from, to).unwrap();
        assert!(matches!(
            network.server(load(&text), options),
            Err(Error::LogicalCameraIdentity)
        ));
    }
    assert!(matches!(
        load_world_seeded(
            &xml(9),
            "ninth.sdf",
            "default",
            Some(Duration::from_millis(1)),
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
            [53; 32]
        ),
        Err(gz_sim_scene::LoadError::Unsupported { .. })
    ));
    let mut server = network.server(load(&text), network.options()).unwrap();
    server.set_interest_policy(InterestPolicy::Always);
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
    let counters = server.counters();
    server
        .simulation()
        .world()
        .resource::<gz_sim_core::StepFault>()
        .report(std::io::Error::other(
            "multiple logical-camera terminal reset fence",
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
    assert_eq!(
        server.counters().logical_camera_images_queued,
        counters.logical_camera_images_queued
    );
    assert_eq!(
        server.counters().logical_camera_frustums_queued,
        counters.logical_camera_frustums_queued
    );
    server.shutdown();
}
