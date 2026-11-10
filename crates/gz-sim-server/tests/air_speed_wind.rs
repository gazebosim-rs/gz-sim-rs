// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Real canonical wind-relative AirSpeed, with unchanged demand and lifecycle.
use gz_sensors::air_speed::AirSpeedReading;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_proto::{
    msgs::{
        AirSpeed, Boolean, Entity as EntityRequest, Header, Time, WorldControl, WorldReset,
        entity::Type as EntityType, header::Map,
    },
    prost::Message,
};
use gz_sim_scene::{
    AirSpeedSample, AirSpeedSampleError, Name, PhysicsEngine, load_world_with_sdf_step,
    queue_link_world_velocity,
};
use gz_sim_server::{Error, InterestPolicy, Server, ServerOptions};
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
const XML: &str = include_str!("../../../examples/m41_air_speed_wind.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m41-server.sdf",
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
type Reading = (Duration, u64, AirSpeedReading);
fn sample(server: &Server) -> Option<Reading> {
    server
        .simulation()
        .world()
        .iter_entities()
        .find_map(|e| e.get::<AirSpeedSample>())
        .map(|s| (s.time, s.epoch, *s.reading.as_ref().unwrap()))
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
            partition: format!("m41-{}", uuid::Uuid::new_v4()),
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
            "air_speed_wind",
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
        panic!("native AirSpeed wind peer: {message}; {details}");
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
            panic!("native AirSpeed wind channel capture failed: {details}");
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
                "native AirSpeed wind retained diagnostics: {}",
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
fn packet_line(peer: &mut Peer) -> Vec<u8> {
    let line = peer.line();
    decode_hex(
        line.strip_prefix("PACKET ")
            .unwrap_or_else(|| panic!("{line}")),
    )
}
fn packet_time(bytes: &[u8]) -> Duration {
    let m = AirSpeed::decode(bytes).unwrap();
    let t = m.header.unwrap().stamp.unwrap();
    assert!(t.sec >= 0 && (0..1_000_000_000).contains(&t.nsec));
    Duration::new(t.sec.try_into().unwrap(), t.nsec.try_into().unwrap())
}
fn check_packet(bytes: &[u8], s: Reading, expected_sequence: Option<u64>) -> u64 {
    let m = AirSpeed::decode(bytes).unwrap();
    let h = m.header.as_ref().unwrap();
    assert_eq!(h.data.len(), 2);
    assert_eq!((&*h.data[1].key, h.data[1].value.len()), ("seq", 1));
    let actual: u64 = h.data[1].value[0].parse().unwrap();
    assert_eq!(h.data[1].value[0], actual.to_string());
    if let Some(expected) = expected_sequence {
        assert_eq!(actual, expected);
    }
    let expected = AirSpeed {
        header: Some(Header {
            stamp: Some(Time {
                sec: s.0.as_secs().try_into().unwrap(),
                nsec: s.0.subsec_nanos().try_into().unwrap(),
            }),
            data: vec![
                Map {
                    key: "frame_id".into(),
                    value: vec!["rig/body/air_speed".into()],
                },
                Map {
                    key: "seq".into(),
                    value: vec![actual.to_string()],
                },
            ],
        }),
        diff_pressure: s.2.diff_pressure,
        temperature: s.2.temperature,
    };
    assert_eq!(m.diff_pressure.to_bits(), s.2.diff_pressure.to_bits());
    assert_eq!(m.temperature.to_bits(), s.2.temperature.to_bits());
    assert_eq!(m, expected);
    assert_eq!(bytes, expected.encode_to_vec());
    actual
}
fn update(server: &mut Server, peer: &mut Peer, sequence: Option<&mut u64>) -> Option<Reading> {
    let before = sample(server).map(|s| (s.0, s.1));
    let count = server.counters().air_speeds_queued;
    if let Err(error) = server.update() {
        peer.fail(format!("Server update: {error}"));
    }
    let after = sample(server);
    let acquired = after.is_some_and(|s| before != Some((s.0, s.1)));
    assert_eq!(
        server.counters().air_speeds_queued - count,
        u64::from(acquired)
    );
    if acquired {
        let s = after.unwrap();
        assert_eq!(
            (s.0, s.1),
            (server.simulation().elapsed(), server.simulation().epoch())
        );
    }
    if let Some(sequence) = sequence {
        peer.command(&format!("drain {}", usize::from(acquired)));
        if acquired {
            check_packet(&packet_line(peer), after.unwrap(), Some(*sequence));
            *sequence += 1;
        }
        assert_eq!(peer.line(), format!("DRAINED {}", usize::from(acquired)));
        assert!(peer.lines.try_recv().is_err(), "unexpected phase output");
    }
    after
}
fn advance(server: &mut Server, peer: &mut Peer, steps: u32, mut sequence: Option<&mut u64>) {
    for _ in 0..steps {
        let t = server.simulation().elapsed();
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            multi_step: 1,
            ..Default::default()
        }));
        update(server, peer, sequence.as_deref_mut());
        assert_eq!(server.simulation().elapsed(), t + Duration::from_millis(1));
    }
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
fn prime_and_arm(peer: &mut Peer, server: &mut Server) -> u64 {
    peer.command("prime any");
    assert_eq!(peer.line(), "PRIMING any");
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut history = Vec::new();
    let mut first = None;
    for attempt in 0..256 {
        advance(server, peer, 1, None);
        if let Some(s) = sample(server)
            && history.last().is_none_or(|old: &Reading| old.0 != s.0)
        {
            history.push(s);
        }
        match peer.lines.recv_timeout(Duration::from_millis(20)) {
            Ok(line) => {
                first = Some(decode_hex(
                    line.strip_prefix("PACKET ")
                        .unwrap_or_else(|| panic!("{line}")),
                ));
                break;
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(error) => peer.fail(format!("first acquisition channel: {error}")),
        }
        if attempt == 255 || Instant::now() >= deadline {
            peer.fail("actual first positive AirSpeed receipt deadline");
        }
    }
    let first = first.unwrap();
    let at = packet_time(&first);
    let actual = *history
        .iter()
        .find(|s| s.0 == at)
        .expect("actual sample at first demand time");
    check_packet(&first, actual, None);
    assert_eq!(peer.line(), format!("PRIMED {}", at.as_nanos()));
    // Discovery/first delivery is not a barrier. A later unique same-topic due
    // frame is an explicit cutoff before exact FIFO recording begins.
    let now = server.simulation().elapsed().as_nanos();
    let due = (now / 10_000_000 + 1) * 10_000_000;
    peer.command(&format!("prime {due}"));
    assert_eq!(peer.line(), format!("PRIMING {due}"));
    advance(
        server,
        peer,
        ((due - now) / 1_000_000).try_into().unwrap(),
        None,
    );
    let current = sample(server).unwrap();
    assert_eq!(current.0.as_nanos(), due);
    let sequence = check_packet(&packet_line(peer), current, None);
    assert_eq!(peer.line(), format!("PRIMED {due}"));
    peer.command("arm");
    assert_eq!(peer.line(), "ARMED");
    peer.command("drain 0");
    assert_eq!(peer.line(), "DRAINED 0");
    sequence + 1
}
fn drop_feedback(peer: &mut Peer) {
    peer.command("drop");
    assert_eq!(peer.line(), "DROPPED");
}
fn watch_feedback(peer: &mut Peer) {
    peer.command("watch");
    assert_eq!(peer.line(), "WATCHING");
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
    let e = server.simulation().epoch();
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
        (Duration::ZERO, e + 1)
    );
    assert!(sample(server).is_none());
}
#[test]
fn real_cpp_wind_relative_air_speed_packets_survive_rotation_demand_pause_reset_and_removal() {
    for noisy in [false, true] {
        let xml = if noisy {
            XML.replace("<air_speed/>","<air_speed><pressure><noise type='gaussian'><mean>0.1</mean><stddev>0</stddev></noise></pressure></air_speed>")
        } else {
            XML.into()
        };
        let network = Network::new();
        assert!(!network.observer.is_closed());
        let options = network.options();
        let mut sim = load(&xml);
        let sensor = id(&sim, "default::rig::body::air_speed");
        let body = id(&sim, "default::rig::body");
        let owner = id(&sim, "default::rig");
        queue_link_world_velocity(&mut sim, body, [3.0, -1.0, 0.5], [0.4, -0.2, 2.0]).unwrap();
        let mut server = network.server(sim, options.clone()).unwrap();
        assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
        // With no consumer, no new sample or sensor deadline is consumed.
        for _ in 0..10 {
            assert!(server.apply_local_control(WorldControl {
                pause: true,
                multi_step: 1,
                ..Default::default()
            }));
            server.update().unwrap();
            assert!(sample(&server).is_none());
        }
        assert_eq!(server.counters().air_speeds_queued, 0);
        let mut peer = start_peer(&network, &options);
        assert_eq!(peer.line(), "READY air-speed-wind");
        let mut seq = prime_and_arm(&mut peer, &mut server);
        let first = sample(&server).unwrap();
        let mut pressures = vec![first.2.diff_pressure.to_bits()];
        for _ in 0..3 {
            advance(&mut server, &mut peer, 10, Some(&mut seq));
            pressures.push(sample(&server).unwrap().2.diff_pressure.to_bits());
        }
        assert!(
            pressures.windows(2).any(|p| p[0] != p[1]),
            "rotating offset produces changing valid pressure"
        );
        let held = sample(&server);
        for _ in 0..3 {
            update(&mut server, &mut peer, Some(&mut seq));
            assert_eq!(sample(&server), held);
        }
        drop_feedback(&mut peer);
        let deadline = Instant::now() + Duration::from_secs(5);
        while sample(&server).is_some() {
            server.update().unwrap();
            assert!(Instant::now() < deadline, "actual demand retirement");
            std::thread::sleep(Duration::from_millis(1));
        }
        let count = server.counters().air_speeds_queued;
        advance(&mut server, &mut peer, 20, None);
        assert!(sample(&server).is_none());
        assert_eq!(server.counters().air_speeds_queued, count);
        watch_feedback(&mut peer);
        seq = prime_and_arm(&mut peer, &mut server);
        advance(&mut server, &mut peer, 10, Some(&mut seq));
        drop_feedback(&mut peer);
        reset_loaded(&mut server);
        assert_eq!(
            id(server.simulation(), "default::rig::body::air_speed"),
            sensor
        );
        let fresh = entity_by_id(server.simulation().world(), sensor).unwrap();
        assert!(
            server
                .simulation()
                .world()
                .get::<AirSpeedSample>(fresh)
                .is_none()
        );
        watch_feedback(&mut peer);
        seq = prime_and_arm(&mut peer, &mut server);
        let reset = sample(&server).unwrap();
        assert_eq!(reset.1, 1);
        advance(&mut server, &mut peer, 10, Some(&mut seq));
        assert_eq!(
            sample(&server).unwrap().2,
            reset.2,
            "reset clears motion and preserves immutable wind/noise"
        );
        drop_feedback(&mut peer);
        remove_through_canonical_service(&mut server, &network, &options, owner);
        assert!(entity_by_id(server.simulation().world(), sensor).is_none());
        server.update().unwrap();
        peer.command("retired");
        assert_eq!(peer.line(), "RETIRED");
        reset_loaded(&mut server);
        assert_eq!(
            id(server.simulation(), "default::rig::body::air_speed"),
            sensor
        );
        watch_feedback(&mut peer);
        seq = prime_and_arm(&mut peer, &mut server);
        assert_eq!(sample(&server).unwrap().2, reset.2);
        advance(&mut server, &mut peer, 10, Some(&mut seq));
        drop_feedback(&mut peer);
        peer.command("finish");
        assert_eq!(peer.line(), "PASS canonical AirSpeed wind");
        let (status, stderr) = peer.finish();
        assert!(
            status.success(),
            "{status}: {stderr}; {}",
            peer.diagnostics()
        );
        assert!(stderr.is_empty(), "{stderr}");
        assert_eq!(server.counters().air_speed_failures, 0);
        assert_eq!(server.counters().invalid_air_speed_samples, 0);
        server.shutdown();
        assert!(matches!(server.update(), Err(Error::WorldClosed)));
    }
}
#[test]
fn finite_wind_overflow_is_a_held_ideal_sample_error_without_canonical_output_or_clock_fault() {
    let xml = XML.replace(
        "<linear_velocity>1 -2 0.5</linear_velocity>",
        "<linear_velocity>1e308 0 0</linear_velocity>",
    );
    let network = Network::new();
    let mut server = network.server(load(&xml), network.options()).unwrap();
    server.set_interest_policy(InterestPolicy::Always);
    for tick in 1..=10 {
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            multi_step: 1,
            ..Default::default()
        }));
        server.update().unwrap();
        let s = server
            .simulation()
            .world()
            .iter_entities()
            .find_map(|e| e.get::<AirSpeedSample>())
            .unwrap();
        assert!(matches!(s.reading, Err(AirSpeedSampleError::Ideal(_))));
        assert_eq!(
            s.time,
            Duration::from_millis(if tick < 10 { 1 } else { 10 })
        );
        assert_eq!(
            server.counters().invalid_air_speed_samples,
            if tick < 10 { 1 } else { 2 }
        );
        assert_eq!(server.counters().air_speeds_queued, 0);
        assert!(server.simulation().fault().is_none());
    }
    reset_loaded(&mut server);
    assert!(server.simulation().fault().is_none());
    assert_eq!(server.simulation().epoch(), 1);
    server.shutdown();
}
