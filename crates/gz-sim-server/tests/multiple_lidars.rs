// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent loaded planar CPU lidars: actual per-topic canonical and ownership proof.
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_proto::{
    msgs::{
        Boolean, Entity as EntityRequest, Header, LaserScan, Pose, Quaternion, Time, Vector3d,
        WorldControl, WorldReset, entity::Type as EntityType, header::Map,
    },
    prost::Message,
};
use gz_sim_scene::{
    LidarPublication, LidarSample, Name, PhysicsEngine, load_world_seeded, queue_link_world_wrench,
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
            partition: format!("m54-{}", uuid::Uuid::new_v4()),
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
            "multiple_lidars",
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
        panic!("native multiple lidar peer: {message}; {details}");
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
            panic!("native multiple lidar channel capture failed: {details}");
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
                "native multiple lidar retained diagnostics: {}",
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

const PERIODS: [u64; 8] = [10, 20, 40, 10, 20, 40, 10, 20];
fn xml(count: usize, noise: u8) -> String {
    let mut text = String::from(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='step' type='ode'><max_step_size>0.001</max_step_size></physics>",
    );
    for (index, period) in PERIODS.iter().cycle().take(count).enumerate() {
        let rate = 1000 / *period;
        let noise_xml = match noise {
            0 => String::new(),
            1 => "<noise><type>gaussian</type><mean>.125</mean><stddev>0</stddev></noise>".into(),
            2 => "<noise><type>gaussian</type><mean>.125</mean><stddev>.02</stddev></noise>".into(),
            _ => unreachable!("closed noise mode"),
        };
        let min = -std::f64::consts::FRAC_PI_4;
        let max = std::f64::consts::FRAC_PI_2;
        let yaw = if index.is_multiple_of(2) { 0.1 } else { 0.2 };
        text.push_str(&format!("<model name='rig{index}'><pose>0 {} 3 0 0 0</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='shape'><geometry><box><size>.2 .2 .2</size></box></geometry></collision><sensor name='lidar' type='lidar'><pose>.5 0 .5 .1 -.15 {yaw}</pose><update_rate>{rate}</update_rate><topic>/m54/scan{index}</topic><frame_id>rig{index}/body/lidar</frame_id><lidar><scan><horizontal><samples>5</samples><resolution>1</resolution><min_angle>{min}</min_angle><max_angle>{max}</max_angle></horizontal></scan><range><min>0</min><max>10</max><resolution>0</resolution></range>{noise_xml}</lidar></sensor></link></model>", index * 30));
    }
    // Peer bodies are 30 m apart, beyond the admitted 10 m rays. The wall is an
    // independent real collider; its front plane is x=4, not an injected range.
    text.push_str("<model name='wall'><static>true</static><pose>5 100 3 0 0 0</pose><link name='body'><collision name='shape'><geometry><box><size>2 600 100</size></box></geometry></collision></link></model></world></sdf>");
    text
}
fn load(text: &str) -> Simulation {
    load_world_seeded(
        text,
        "m54-server.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        [54; 32],
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
    translation: [f64; 3],
    rotation: [f64; 4],
    ranges: Vec<f64>,
    metadata: LidarPublication,
}
fn readings(server: &Server) -> BTreeMap<usize, Reading> {
    server
        .simulation()
        .world()
        .iter_entities()
        .filter_map(|e| {
            let metadata = e.get::<LidarPublication>()?;
            let index = metadata.topic.strip_prefix("/m54/scan")?.parse().unwrap();
            let s = e.get::<LidarSample>()?;
            Some((
                index,
                Reading {
                    time: s.time,
                    epoch: s.epoch,
                    translation: s.world_pose.translation,
                    rotation: s.world_pose.rotation,
                    ranges: s.ranges.clone(),
                    metadata: metadata.clone(),
                },
            ))
        })
        .collect()
}
fn canonical(reading: &Reading, sequence: u64) -> LaserScan {
    let m = &reading.metadata;
    let c = m.config;
    let p = reading;
    LaserScan {
        header: Some(Header {
            stamp: Some(Time {
                sec: reading.time.as_secs().try_into().unwrap(),
                nsec: reading.time.subsec_nanos() as i32,
            }),
            data: vec![
                Map {
                    key: "frame_id".into(),
                    value: vec![m.frame_id.clone()],
                },
                Map {
                    key: "seq".into(),
                    value: vec![sequence.to_string()],
                },
            ],
        }),
        frame: m.frame_id.clone(),
        world_pose: Some(Pose {
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
            ..Default::default()
        }),
        angle_min: c.angle_min,
        angle_max: c.angle_max,
        angle_step: m.angle_step,
        range_min: c.range_min,
        range_max: c.range_max,
        count: c.samples.try_into().unwrap(),
        vertical_angle_min: 0.0,
        vertical_angle_max: 0.0,
        vertical_angle_step: 0.0,
        vertical_count: 1,
        ranges: reading.ranges.clone(),
        intensities: vec![0.0; c.samples],
    }
}
fn receive(peer: &mut Peer, index: usize, reading: &Reading) -> u64 {
    let line = peer.line();
    let prefix = format!("PACKET {index} ");
    let bytes = decode_hex(
        line.strip_prefix(&prefix)
            .unwrap_or_else(|| panic!("{line}")),
    );
    let message = LaserScan::decode(bytes.as_slice()).unwrap();
    let header = message.header.as_ref().unwrap();
    assert_eq!(header.data.len(), 2);
    let sequence = header.data[1].value[0].parse().unwrap();
    let expected = canonical(reading, sequence);
    assert_eq!(message, expected);
    assert_eq!(bytes, expected.encode_to_vec());
    assert_eq!(
        message
            .ranges
            .iter()
            .map(|v| v.to_bits())
            .collect::<Vec<_>>(),
        reading
            .ranges
            .iter()
            .map(|v| v.to_bits())
            .collect::<Vec<_>>()
    );
    assert_eq!(
        header.encode_to_vec(),
        expected.header.unwrap().encode_to_vec()
    );
    assert!(
        message
            .intensities
            .iter()
            .all(|v| v.to_bits() == 0.0f64.to_bits())
    );
    sequence
}
fn step(server: &mut Server, peer: &mut Peer, recording: &[bool], sequences: &mut [u64]) {
    let before = readings(server);
    let counter = server.counters().lidars_queued;
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        multi_step: 1,
        ..Default::default()
    }));
    server.update().unwrap();
    let after = readings(server);
    let mut changed = 0;
    for (&index, reading) in &after {
        let acquired = before
            .get(&index)
            .is_none_or(|old| (old.epoch, old.time) != (reading.epoch, reading.time));
        changed += u64::from(acquired);
        if !acquired {
            assert_eq!(Some(reading), before.get(&index));
        }
        if recording[index] {
            peer.command(&format!("drain {index} {}", usize::from(acquired)));
            if acquired {
                assert_eq!(receive(peer, index, reading), sequences[index]);
                sequences[index] += 1;
            }
            assert_eq!(
                peer.line(),
                format!("DRAINED {index} {}", usize::from(acquired))
            );
        }
    }
    assert_eq!(server.counters().lidars_queued - counter, changed);
}
fn cutoff(
    server: &mut Server,
    peer: &mut Peer,
    indices: &[usize],
    recording: &mut [bool],
    sequences: &mut [u64],
) {
    let now = server.simulation().elapsed().as_millis() as u64;
    let target = (now / 40 + 1) * 40;
    for &index in indices {
        peer.command(&format!("prime {index} {}", target * 1_000_000));
        assert_eq!(peer.line(), format!("PRIMING {index}"));
    }
    for _ in now..target {
        step(server, peer, recording, sequences);
    }
    let at = readings(server);
    for &index in indices {
        assert_eq!(at[&index].time, Duration::from_millis(target));
        peer.command(&format!("cutoff {index}"));
        sequences[index] = receive(peer, index, &at[&index]) + 1;
        assert_eq!(
            peer.line(),
            format!("PRIMED {index} {}", target * 1_000_000)
        );
        peer.command(&format!("arm {index}"));
        assert_eq!(peer.line(), format!("ARMED {index}"));
        recording[index] = true;
    }
}
fn rotate([x, y, z, w]: [f64; 4], [a, b, c]: [f64; 3]) -> [f64; 3] {
    let t = [
        2.0 * (y * c - z * b),
        2.0 * (z * a - x * c),
        2.0 * (x * b - y * a),
    ];
    [
        a + w * t[0] + y * t[2] - z * t[1],
        b + w * t[1] + z * t[0] - x * t[2],
        c + w * t[2] + x * t[1] - y * t[0],
    ]
}
fn fresh_geometry(server: &Server, noise: u8, expected: usize, wall: bool) {
    let mut seen = 0;
    let mut misses = 0;
    for reading in readings(server).values() {
        if reading.time != server.simulation().elapsed() {
            continue;
        }
        seen += 1;
        let c = reading.metadata.config;
        for (beam, &observed) in reading.ranges.iter().enumerate() {
            let angle = if beam + 1 == c.samples {
                c.angle_max
            } else {
                c.angle_min + beam as f64 * reading.metadata.angle_step
            };
            let (sin, cos) = angle.sin_cos();
            let direction = rotate(reading.rotation, [cos, sin, 0.0]);
            let distance = (4.0 - reading.translation[0]) / direction[0];
            let y = reading.translation[1] + distance * direction[1];
            let z = reading.translation[2] + distance * direction[2];
            let hit = wall
                && (c.range_min..=c.range_max).contains(&distance)
                && direction[0] > 0.0
                && (-200.0..=400.0).contains(&y)
                && (-47.0..=53.0).contains(&z);
            if !hit {
                assert_eq!(observed, f64::INFINITY);
                misses += 1;
            } else if noise != 2 {
                let mean = if noise == 1 { 0.125 } else { 0.0 };
                assert!(
                    (observed - (distance + mean).clamp(c.range_min, c.range_max)).abs() < 1e-10
                );
            } else {
                assert!(observed.is_finite() && (c.range_min..=c.range_max).contains(&observed));
            }
        }
    }
    assert_eq!(seen, expected);
    assert!(misses > 0, "positive infinity is a present acquired miss");
}

#[test]
fn real_cpp_pair_and_eight_keep_independent_scans_motion_noise_demand_reset_and_retirement() {
    for (count, noise) in [(2, 0), (2, 1), (2, 2), (8, 0)] {
        let network = Network::new();
        let options = network.options();
        let text = xml(count, noise);
        let mut simulation = load(&text);
        let initial: Vec<_> = (0..count)
            .map(|i| {
                *simulation
                    .world()
                    .get::<gz_sim_scene::WorldPose>(
                        entity_by_id(
                            simulation.world(),
                            id(&simulation, &format!("default::rig{i}::body")),
                        )
                        .unwrap(),
                    )
                    .unwrap()
            })
            .collect();
        for index in 0..count {
            let body = id(&simulation, &format!("default::rig{index}::body"));
            queue_link_world_wrench(&mut simulation, body, [8.0, -4.0, 2.0], [2.0, -3.0, 4.0])
                .unwrap();
        }
        let mut server = network.server(simulation, options.clone()).unwrap();
        let identities: Vec<_> = (0..count)
            .map(|i| {
                id(
                    server.simulation(),
                    &format!("default::rig{i}::body::lidar"),
                )
            })
            .collect();
        let mut peer = start_peer(&network, &options, count);
        assert_eq!(peer.line(), format!("READY multiple-lidars {count}"));
        let mut recording = vec![false; count];
        let mut sequences = vec![0; count];
        cutoff(
            &mut server,
            &mut peer,
            &(0..count).collect::<Vec<_>>(),
            &mut recording,
            &mut sequences,
        );
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording, &mut sequences);
        }
        fresh_geometry(&server, noise, count, true);
        for (index, old) in initial.iter().enumerate() {
            let body = entity_by_id(
                server.simulation().world(),
                id(server.simulation(), &format!("default::rig{index}::body")),
            )
            .unwrap();
            let pose = server
                .simulation()
                .world()
                .get::<gz_sim_scene::WorldPose>(body)
                .unwrap();
            assert_ne!(
                pose.translation.map(f64::to_bits),
                old.translation.map(f64::to_bits)
            );
            assert_ne!(
                pose.rotation.map(f64::to_bits),
                old.rotation.map(f64::to_bits)
            );
        }
        let held = readings(&server);
        let queued = server.counters().lidars_queued;
        for _ in 0..3 {
            assert!(server.update().unwrap().dt.is_zero());
            assert_eq!(readings(&server), held);
        }
        assert_eq!(server.counters().lidars_queued, queued);
        peer.command("drop 0");
        assert_eq!(peer.line(), "DROPPED 0");
        recording[0] = false;
        let deadline = Instant::now() + Duration::from_secs(5);
        while readings(&server).contains_key(&0) {
            assert!(Instant::now() < deadline);
            server.update().unwrap();
            std::thread::yield_now();
        }
        let peer_time = readings(&server)[&1].time;
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording, &mut sequences);
            assert!(!readings(&server).contains_key(&0));
        }
        assert!(readings(&server)[&1].time > peer_time);
        peer.command("watch 0");
        assert_eq!(peer.line(), "WATCHING 0");
        cutoff(&mut server, &mut peer, &[0], &mut recording, &mut sequences);
        let wall = id(server.simulation(), "default::wall");
        remove_through_canonical_service(&mut server, &network, &options, wall);
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording, &mut sequences);
        }
        fresh_geometry(&server, noise, count, false);
        for (index, active) in recording.iter_mut().enumerate() {
            peer.command(&format!("drop {index}"));
            assert_eq!(peer.line(), format!("DROPPED {index}"));
            *active = false;
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
        for (index, identity) in identities.iter().enumerate() {
            assert_eq!(
                id(
                    server.simulation(),
                    &format!("default::rig{index}::body::lidar")
                ),
                *identity
            );
            peer.command(&format!("watch {index}"));
            assert_eq!(peer.line(), format!("WATCHING {index}"));
        }
        cutoff(
            &mut server,
            &mut peer,
            &(0..count).collect::<Vec<_>>(),
            &mut recording,
            &mut sequences,
        );
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
        for (index, actual) in &reset {
            let other = &expected[index];
            assert_eq!(actual.time, other.time);
            assert_eq!(
                actual.translation.map(f64::to_bits),
                other.translation.map(f64::to_bits)
            );
            assert_eq!(
                actual.rotation.map(f64::to_bits),
                other.rotation.map(f64::to_bits)
            );
            assert_eq!(
                actual
                    .ranges
                    .iter()
                    .map(|r| r.to_bits())
                    .collect::<Vec<_>>(),
                other.ranges.iter().map(|r| r.to_bits()).collect::<Vec<_>>()
            );
        }
        fresh.shutdown();
        peer.command("drop 0");
        assert_eq!(peer.line(), "DROPPED 0");
        recording[0] = false;
        let owner = id(server.simulation(), "default::rig0");
        remove_through_canonical_service(&mut server, &network, &options, owner);
        peer.command("retired 0");
        assert_eq!(peer.line(), "RETIRED 0");
        let peer_time = readings(&server)[&1].time;
        for _ in 0..40 {
            step(&mut server, &mut peer, &recording, &mut sequences);
        }
        assert!(readings(&server)[&1].time > peer_time);
        assert!(entity_by_id(server.simulation().world(), identities[0]).is_none());
        for index in 1..count {
            peer.command(&format!("drop {index}"));
            assert_eq!(peer.line(), format!("DROPPED {index}"));
        }
        server.shutdown();
        assert!(matches!(server.update(), Err(Error::WorldClosed)));
        for index in 1..count {
            peer.command(&format!("retired {index}"));
            assert_eq!(peer.line(), format!("RETIRED {index}"));
        }
        peer.command("finish");
        assert_eq!(peer.line(), "PASS canonical multiple lidars");
        let (status, stderr) = peer.finish();
        assert!(status.success(), "{status:?}: {stderr}");
        assert!(stderr.is_empty(), "{stderr}");
    }
}

#[test]
fn resolved_aliases_ninth_owner_and_terminal_reset_preserve_loaded_lidar_state() {
    let network = Network::new();
    let text = xml(2, 0);
    let mut options = network.options();
    options.add_topic_remap("/m54/scan1", "/m54/scan0").unwrap();
    assert!(matches!(
        network.server(load(&text), options),
        Err(Error::LidarIdentity)
    ));
    assert!(matches!(
        load_world_seeded(
            &xml(9, 0),
            "ninth.sdf",
            "default",
            Some(Duration::from_millis(1)),
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
            [54; 32]
        ),
        Err(gz_sim_scene::LoadError::Unsupported {
            feature: "multiple CPU lidars",
            ..
        })
    ));
    let mut server = network.server(load(&text), network.options()).unwrap();
    server.set_interest_policy(InterestPolicy::Always);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        multi_step: 40,
        ..Default::default()
    }));
    for _ in 0..40 {
        server.update().unwrap();
    }
    let before = readings(&server);
    let clock = (server.simulation().epoch(), server.simulation().elapsed());
    let queued = server.counters().lidars_queued;
    server
        .simulation()
        .world()
        .resource::<gz_sim_core::StepFault>()
        .report(std::io::Error::other("multiple lidar terminal reset fence"));
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
    assert_eq!(server.counters().lidars_queued, queued);
    server.shutdown();
}
