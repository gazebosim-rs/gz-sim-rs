// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Real XYZ wrench motion and complete noiseless 3D canonical feedback.
use gz_sim_core::{EntityId, Simulation};
use gz_sim_proto::{
    msgs::{Odometry, OdometryWithCovariance, PoseV, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_scene::{
    GroundTruthOdometryDimensions, GroundTruthOdometryPublication, GroundTruthOdometrySample, Name,
    PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
};
use gz_sim_server::{Counters, Error, Server, ServerOptions};
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
const XML: &str = include_str!("../../../examples/m38_odometry_3d.sdf");
fn load() -> Simulation {
    load_world_with_sdf_step(
        XML,
        "m38.sdf",
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
fn sample(server: &Server) -> Option<GroundTruthOdometrySample> {
    server
        .simulation()
        .world()
        .iter_entities()
        .find_map(|e| e.get::<GroundTruthOdometrySample>())
        .copied()
}
fn motion(server: &Server) -> ([f64; 3], [f64; 3]) {
    let e = gz_sim_core::entity_by_id(
        server.simulation().world(),
        id(server.simulation(), "default::rig::body"),
    )
    .unwrap();
    let velocity = server.simulation().world().get::<Velocity>(e).unwrap();
    (velocity.linear, velocity.angular)
}
fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1e-12, "{actual} != {expected}");
}
fn close_vector(actual: [f64; 3], expected: [f64; 3]) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        close(actual, expected);
    }
}
fn dimensions(server: &Server) -> GroundTruthOdometryDimensions {
    server
        .simulation()
        .world()
        .iter_entities()
        .find_map(|e| e.get::<GroundTruthOdometryPublication>())
        .unwrap()
        .dimensions
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
            partition: format!("m38-{}", uuid::Uuid::new_v4()),
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
            "odometry_3d",
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
        panic!("native 3D odometry peer: {message}; {details}");
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
            panic!("native 3D odometry channel capture failed: {details}");
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
                "native 3D odometry retained diagnostics: {}",
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
fn check_packets(packets: &BTreeMap<String, Vec<u8>>, expected: GroundTruthOdometrySample) {
    let odom = Odometry::decode(packets["odometry"].as_slice()).unwrap();
    let covariance = OdometryWithCovariance::decode(packets["covariance"].as_slice()).unwrap();
    let transform = PoseV::decode(packets["transform"].as_slice()).unwrap();
    let header = odom.header.as_ref().unwrap();
    let stamp = header.stamp.as_ref().unwrap();
    let sec: i64 = expected.time.as_secs().try_into().unwrap();
    let nsec: i32 = expected.time.subsec_nanos().try_into().unwrap();
    assert_eq!((stamp.sec, stamp.nsec), (sec, nsec));
    assert_eq!(header.data.len(), 2);
    assert_eq!(
        (&*header.data[0].key, header.data[0].value.as_slice()),
        ("frame_id", ["m38/odom".to_owned()].as_slice())
    );
    assert_eq!(
        (&*header.data[1].key, header.data[1].value.as_slice()),
        ("child_frame_id", ["m38/base".to_owned()].as_slice())
    );
    let pose = odom.pose.as_ref().unwrap();
    assert!(pose.header.is_none() && pose.name.is_empty() && pose.id == 0);
    let position = pose.position.as_ref().unwrap();
    let rotation = pose.orientation.as_ref().unwrap();
    assert_eq!(
        [position.x, position.y, position.z],
        expected.pose.translation
    );
    assert_eq!(
        [rotation.x, rotation.y, rotation.z, rotation.w],
        expected.pose.rotation
    );
    assert!(position.z.is_finite());
    let twist = odom.twist.as_ref().unwrap();
    assert!(twist.header.is_none());
    let linear = twist.linear.as_ref().unwrap();
    let angular = twist.angular.as_ref().unwrap();
    assert_eq!([linear.x, linear.y, linear.z], expected.linear);
    assert_eq!([angular.x, angular.y, angular.z], expected.angular);
    assert!(
        expected
            .pose
            .translation
            .into_iter()
            .chain(expected.pose.rotation)
            .chain(expected.linear)
            .chain(expected.angular)
            .all(f64::is_finite)
    );
    assert_eq!(covariance.header, odom.header);
    let pose_cov = covariance.pose_with_covariance.unwrap();
    let twist_cov = covariance.twist_with_covariance.unwrap();
    assert_eq!(pose_cov.pose, odom.pose);
    assert_eq!(twist_cov.twist, odom.twist);
    let zeros = gz_sim_proto::msgs::FloatV {
        data: vec![0.0; 36],
        ..Default::default()
    };
    assert_eq!(pose_cov.covariance.unwrap(), zeros);
    assert_eq!(twist_cov.covariance.unwrap(), zeros);
    assert!(transform.header.is_none());
    assert_eq!(transform.pose.len(), 1);
    let mut tf_pose = transform.pose.into_iter().next().unwrap();
    assert_eq!(tf_pose.header.take(), odom.header);
    assert_eq!(Some(tf_pose), odom.pose);
}
fn drain(peer: &mut Peer, server: &mut Server, expected: Option<GroundTruthOdometrySample>) {
    let count = usize::from(expected.is_some());
    peer.command(&format!("drain {count}"));
    let mut packets = BTreeMap::new();
    for _ in 0..3 * count {
        let line = peer.line(server);
        let parts: Vec<_> = line.split(' ').collect();
        assert_eq!(parts.len(), 3, "{line}");
        assert_eq!(parts[0], "PACKET");
        assert!(["odometry", "covariance", "transform"].contains(&parts[1]));
        assert!(
            packets
                .insert(parts[1].to_owned(), decode_hex(parts[2]))
                .is_none(),
            "duplicate channel"
        );
    }
    assert_eq!(peer.line(server), format!("DRAINED {count}"));
    assert_eq!(packets.len(), 3 * count);
    if let Some(expected) = expected {
        check_packets(&packets, expected);
    }
    assert!(peer.lines.try_recv().is_err(), "unexpected phase remainder");
}
fn advance(server: &mut Server, peer: &mut Peer, steps: u32, observing: bool) {
    for _ in 0..steps {
        let before_time = server.simulation().elapsed();
        let before = sample(server).map(|s| (s.epoch, s.revision));
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            multi_step: 1,
            ..Default::default()
        }));
        server.update().unwrap();
        assert_eq!(
            server.simulation().elapsed(),
            before_time + Duration::from_millis(1)
        );
        let after = sample(server);
        let acquired = after.map(|s| (s.epoch, s.revision)) != before;
        let due = server
            .simulation()
            .elapsed()
            .as_nanos()
            .is_multiple_of(20_000_000);
        assert_eq!(
            acquired, due,
            "exact 50 Hz acquisition, including while disconnected"
        );
        if acquired {
            let current = after.unwrap();
            assert_eq!(
                current.revision,
                before.map_or(1, |(_, revision)| revision + 1)
            );
            assert_eq!(current.time, server.simulation().elapsed());
            assert_eq!(current.epoch, server.simulation().epoch());
            let model = gz_sim_core::entity_by_id(
                server.simulation().world(),
                id(server.simulation(), "default::rig"),
            )
            .unwrap();
            let actual = server.simulation().world().get::<WorldPose>(model).unwrap();
            assert_eq!(current.pose.translation, actual.translation);
            assert_eq!(current.pose.rotation, actual.rotation);
            if observing {
                drain(peer, server, Some(current));
            }
        }
    }
}
fn admission(server: &mut Server, peer: &mut Peer, command: &str, queued: u64, clears: u64) {
    let before = server.counters();
    let clock = (server.simulation().epoch(), server.simulation().elapsed());
    let velocity = motion(server);
    peer.command(command);
    assert_eq!(peer.line(server), format!("SENT {command}"));
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let c = server.counters();
        let received = c.wrench_messages_received - before.wrench_messages_received;
        let admitted = c.link_wrenches_queued - before.link_wrenches_queued;
        let cleared = c.link_wrench_clears_queued - before.link_wrench_clears_queued;
        assert!(received <= queued + clears && admitted <= queued && cleared <= clears);
        assert_eq!(c.wrench_messages_rejected, before.wrench_messages_rejected);
        assert_eq!(
            (server.simulation().epoch(), server.simulation().elapsed()),
            clock
        );
        assert_eq!(motion(server), velocity);
        if (received, admitted, cleared) == (queued + clears, queued, clears) {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "actual semantic admission deadline: {command}"
        );
        server.update().unwrap();
        std::thread::yield_now();
    }
}
fn drop_outputs(server: &mut Server, peer: &mut Peer) {
    peer.command("drop");
    assert_eq!(peer.line(server), "DROPPED");
}
fn watch_outputs(server: &mut Server, peer: &mut Peer) {
    peer.command("watch");
    assert_eq!(peer.line(server), "WATCHING");
    drain(peer, server, None);
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
        (server.simulation().epoch(), server.simulation().elapsed()),
        (epoch + 1, Duration::ZERO)
    );
    assert!(sample(server).is_none());
    assert_eq!(dimensions(server), GroundTruthOdometryDimensions::Three);
    assert_eq!(motion(server), ([0.0; 3], [0.0; 3]));
}
fn start_peer(network: &Network, options: &ServerOptions, body: u64) -> Peer {
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
fn actual_cpp_xyz_motion_and_three_odometry_streams_survive_loaded_reset_and_retirement() {
    let network = Network::new();
    assert!(!network.observer.is_closed());
    let options = network.options();
    let sim = load();
    let body = id(&sim, "default::rig::body");
    let model = id(&sim, "default::rig");
    let mut server = network.server(sim, options.clone()).unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    let mut peer = start_peer(&network, &options, body);
    assert_eq!(peer.line(&mut server), "READY three-dimensional");
    drain(&mut peer, &mut server, None);
    advance(&mut server, &mut peer, 20, true);
    let initial = sample(&server).unwrap();
    assert_eq!(dimensions(&server), GroundTruthOdometryDimensions::Three);
    assert_eq!(initial.pose.translation, [2.0, -1.0, 3.0]);
    assert_eq!(motion(&server), ([0.0; 3], [0.0; 3]));

    admission(&mut server, &mut peer, "once-force", 1, 0);
    let paused_sample = sample(&server).unwrap();
    for _ in 0..3 {
        server.update().unwrap();
    }
    assert_eq!(
        (
            sample(&server).unwrap().time,
            sample(&server).unwrap().revision
        ),
        (paused_sample.time, paused_sample.revision)
    );
    advance(&mut server, &mut peer, 1, true);
    close_vector(motion(&server).0, [0.002, 0.001, 0.003]);
    advance(&mut server, &mut peer, 19, true);
    close_vector(motion(&server).0, [0.002, 0.001, 0.003]);
    admission(&mut server, &mut peer, "persistent-force", 2, 0);
    advance(&mut server, &mut peer, 20, true);
    close_vector(motion(&server).0, [0.042, 0.021, 0.063]);
    admission(&mut server, &mut peer, "once-force", 1, 0);
    admission(&mut server, &mut peer, "clear", 0, 1);
    server.update().unwrap();
    advance(&mut server, &mut peer, 1, true);
    close_vector(motion(&server).0, [0.044, 0.022, 0.066]);
    advance(&mut server, &mut peer, 19, true);
    close_vector(motion(&server).0, [0.044, 0.022, 0.066]);

    let last_wire = sample(&server).unwrap().time;
    drop_outputs(&mut server, &mut peer);
    advance(&mut server, &mut peer, 20, false);
    let disconnected = sample(&server).unwrap();
    assert!(disconnected.time > last_wire);
    assert_eq!(disconnected.time, Duration::from_millis(100));
    // The same WorldIO worker queues Clock before GT in each frame and waits
    // each put. Its next NONDUE 101 ms Clock follows every 100 ms GT job.
    // Observe it on the same C++ Node/session before resubscribing. This fences
    // this queue boundary, not arbitrary cross-publisher callback delivery.
    advance(&mut server, &mut peer, 1, false);
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(101));
    peer.command("clock-101");
    assert_eq!(peer.line(&mut server), "CLOCK 101000000");
    watch_outputs(&mut server, &mut peer);
    advance(&mut server, &mut peer, 19, true);
    assert!(sample(&server).unwrap().time > disconnected.time);
    close_vector(motion(&server).0, [0.044, 0.022, 0.066]);
    assert!(sample(&server).unwrap().pose.translation[2] > initial.pose.translation[2]);
    assert!(
        sample(&server)
            .unwrap()
            .linear
            .into_iter()
            .all(|v| v != 0.0)
    );
    // Body-frame filtered finite differences are not instantaneous world Velocity.
    assert_ne!(sample(&server).unwrap().linear, motion(&server).0);
    assert_eq!(motion(&server).1, [0.0; 3]);

    // Commands are genuinely admitted before reset; neither may reach its epoch.
    admission(&mut server, &mut peer, "once-force", 1, 0);
    admission(&mut server, &mut peer, "persistent-force", 2, 0);
    drop_outputs(&mut server, &mut peer);
    reset_loaded(&mut server);
    assert_eq!(
        (
            id(server.simulation(), "default::rig::body"),
            id(server.simulation(), "default::rig")
        ),
        (body, model)
    );
    watch_outputs(&mut server, &mut peer);
    advance(&mut server, &mut peer, 20, true);
    let replay = sample(&server).unwrap();
    assert_eq!(
        (
            replay.time,
            replay.revision,
            replay.pose.translation,
            replay.pose.rotation,
            replay.linear,
            replay.angular
        ),
        (
            initial.time,
            initial.revision,
            initial.pose.translation,
            initial.pose.rotation,
            initial.linear,
            initial.angular
        )
    );
    assert_eq!(replay.epoch, initial.epoch + 1);
    assert_eq!(motion(&server), ([0.0; 3], [0.0; 3]));

    admission(&mut server, &mut peer, "once-torque", 1, 0);
    advance(&mut server, &mut peer, 1, true);
    close_vector(motion(&server).1, [0.002, 0.004, 0.006]);
    advance(&mut server, &mut peer, 19, true);
    close_vector(motion(&server).1, [0.002, 0.004, 0.006]);
    admission(&mut server, &mut peer, "persistent-torque", 2, 0);
    advance(&mut server, &mut peer, 20, true);
    close_vector(motion(&server).1, [0.122, 0.064, 0.126]);
    admission(&mut server, &mut peer, "clear", 0, 1);
    server.update().unwrap();
    advance(&mut server, &mut peer, 20, true);
    close_vector(motion(&server).1, [0.122, 0.064, 0.126]);
    assert_eq!(motion(&server).0, [0.0; 3]);
    assert!(
        sample(&server)
            .unwrap()
            .angular
            .into_iter()
            .all(|v| v != 0.0)
    );
    assert_ne!(sample(&server).unwrap().angular, motion(&server).1);

    admission(&mut server, &mut peer, "once-torque", 1, 0);
    admission(&mut server, &mut peer, "persistent-torque", 2, 0);
    drop_outputs(&mut server, &mut peer);
    server.remove_editable_model(model).unwrap();
    assert!(gz_sim_core::entity_by_id(server.simulation().world(), body).is_none());
    assert!(sample(&server).is_none());
    assert!(!gz_sim_scene::apply_link_wrench_active(server.simulation()));
    peer.command("retired");
    assert_eq!(peer.line(&mut server), "RETIRED");
    reset_loaded(&mut server);
    assert_eq!(id(server.simulation(), "default::rig::body"), body);
    watch_outputs(&mut server, &mut peer);
    advance(&mut server, &mut peer, 20, true);
    assert_eq!(motion(&server), ([0.0; 3], [0.0; 3]));
    drop_outputs(&mut server, &mut peer);
    peer.command("finish");
    assert_eq!(peer.line(&mut server), "PASS three-dimensional odometry");
    let (status, stderr) = peer.finish();
    assert!(
        status.success(),
        "{status}: {stderr}; {}",
        peer.diagnostics()
    );
    assert!(stderr.is_empty(), "{stderr}");
    let counters: Counters = server.counters();
    assert_eq!(counters.wrench_messages_rejected, 0);
    assert_eq!(counters.ground_truth_odometry_failures, 0);
    assert_eq!(counters.ground_truth_odometry_invalid_samples, 0);
    server.shutdown();
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}

#[test]
fn three_dimensional_topics_remain_atomic_at_initial_collision_and_loaded_reset() {
    let network = Network::new();
    for from in ["/m38/odometry_covariance", "/m38/transform"] {
        let mut options = network.options();
        options
            .topic_remaps
            .insert(from.into(), "/m38/odometry".into());
        assert!(matches!(
            network.server(load(), options),
            Err(Error::GroundTruthOdometryIdentity)
        ));
    }
    let mut server = network.server(load(), network.options()).unwrap();
    assert_eq!(dimensions(&server), GroundTruthOdometryDimensions::Three);
    for _ in 0..20 {
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            multi_step: 1,
            ..Default::default()
        }));
        server.update().unwrap();
    }
    let first = sample(&server).unwrap();
    assert_eq!(first.pose.translation, [2.0, -1.0, 3.0]);
    assert_eq!(first.linear, [0.0; 3]);
    assert_eq!(first.angular, [0.0; 3]);
    assert_eq!(first.time, Duration::from_millis(20));
    reset_loaded(&mut server);
    for _ in 0..20 {
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            multi_step: 1,
            ..Default::default()
        }));
        server.update().unwrap();
    }
    let replay = sample(&server).unwrap();
    assert_eq!(replay.epoch, first.epoch + 1);
    assert_eq!(
        (
            replay.time,
            replay.revision,
            replay.pose.translation,
            replay.pose.rotation,
            replay.linear,
            replay.angular
        ),
        (
            first.time,
            first.revision,
            first.pose.translation,
            first.pose.rotation,
            first.linear,
            first.angular
        )
    );
    server.shutdown();
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}
