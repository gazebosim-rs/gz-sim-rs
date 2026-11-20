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
use zenoh::{
    Wait,
    bytes::{Encoding, ZBytes},
    handlers::{FifoChannel, FifoChannelHandler, IntoHandler},
};
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
const ADMISSION_BOUND: Duration = Duration::from_secs(5);
const POLL_BACKOFF: Duration = Duration::from_millis(1);
const PRESENCE_QUERY_BOUND: Duration = Duration::from_millis(100);

fn poll_delay(now: Instant, deadline: Instant) -> Option<Duration> {
    deadline
        .checked_duration_since(now)
        .filter(|remaining| !remaining.is_zero())
        .map(|remaining| remaining.min(POLL_BACKOFF))
}
fn wait_until(
    server: &mut Server,
    phase: &'static str,
    deadline: Instant,
    mut ready: impl FnMut(&Server) -> bool,
) {
    loop {
        assert!(
            Instant::now() < deadline,
            "{phase}: canonical wrench admission deadline; counters={:?}; ingress={:?}; elapsed={:?}; epoch={}",
            server.counters(),
            server.ingress_statistics(),
            server.simulation().elapsed(),
            server.simulation().epoch()
        );
        if ready(server) {
            assert!(
                Instant::now() < deadline,
                "{phase}: readiness completed after its original deadline"
            );
            return;
        }
        if let Err(error) = server.update() {
            panic!(
                "{phase}: canonical wrench update failed: {error}; counters={:?}; ingress={:?}; elapsed={:?}; epoch={}",
                server.counters(),
                server.ingress_statistics(),
                server.simulation().elapsed(),
                server.simulation().epoch()
            );
        }
        if let Some(delay) = poll_delay(Instant::now(), deadline) {
            std::thread::sleep(delay);
        }
    }
}
fn wait(server: &mut Server, phase: &'static str, ready: impl FnMut(&Server) -> bool) {
    wait_until(server, phase, Instant::now() + ADMISSION_BOUND, ready);
}
fn put_once(
    publisher: &zenoh::pubsub::Publisher<'_>,
    payload: Vec<u8>,
    attachment: &str,
    phase: &'static str,
    deadline: Instant,
) {
    // These fixed canonical inputs fit comfortably in one default 65535-byte
    // transport batch. The finite Block queue wait is not a receiver ACK.
    assert!(payload.len() + attachment.len() + publisher.key_expr().as_str().len() <= 1024);
    assert!(
        Instant::now() < deadline,
        "{phase}: publication started after its original deadline"
    );
    publisher
        .put(payload)
        .attachment(attachment)
        .wait()
        .unwrap();
    assert!(
        Instant::now() < deadline,
        "{phase}: publication exceeded the original admission deadline"
    );
}
#[derive(Clone, Debug, PartialEq, Eq)]
struct InputPresence {
    process: String,
    node: uuid::Uuid,
}
fn channel_type(index: usize) -> String {
    if index == 2 {
        Entity::full_name()
    } else {
        EntityWrench::full_name()
    }
}
fn presence_selector(key: &str, index: usize) -> String {
    let (partition, topic) = key.strip_prefix('@').unwrap().split_once('@').unwrap();
    assert!(!partition.is_empty() && topic.starts_with('/'));
    format!(
        "@gz/{}/*/*/*/MS/%/%/%/{}/{}/%/%",
        partition.replace('/', "%"),
        topic.replace('/', "%"),
        channel_type(index)
    )
}
fn parse_presence(key: &str, index: usize, token: &str) -> Option<InputPresence> {
    let (partition, topic) = key.strip_prefix('@')?.split_once('@')?;
    if partition.is_empty() || !topic.starts_with('/') {
        return None;
    }
    let parts: Vec<_> = token.split('/').collect();
    if parts.len() != 13
        || parts[0] != "@gz"
        || parts[1] != partition.replace('/', "%")
        || parts[5] != "MS"
        || parts[6..9] != ["%", "%", "%"]
        || parts[9] != topic.replace('/', "%")
        || parts[10] != channel_type(index)
        || parts[11..13] != ["%", "%"]
        || parts[3] != parts[4]
        || parts[2].is_empty()
        || parts[2].len() > 32
        || !parts[2].bytes().all(|byte| byte.is_ascii_hexdigit())
    {
        return None;
    }
    let node = uuid::Uuid::parse_str(parts[3]).ok()?;
    if node.is_nil() || node.hyphenated().to_string() != parts[3] {
        return None;
    }
    Some(InputPresence {
        process: parts[2].to_owned(),
        node,
    })
}
fn presence_generation(
    keys: &[String; 3],
    tokens: &[String; 3],
    previous: Option<&[InputPresence; 3]>,
) -> Option<[InputPresence; 3]> {
    let [Some(once), Some(persistent), Some(clear)] =
        std::array::from_fn(|index| parse_presence(&keys[index], index, &tokens[index]))
    else {
        return None;
    };
    let current = [once, persistent, clear];
    if current
        .iter()
        .any(|entry| entry.process != current[0].process)
        || current
            .iter()
            .map(|entry| entry.node)
            .collect::<std::collections::BTreeSet<_>>()
            .len()
            != 3
    {
        return None;
    }
    if let Some(previous) = previous
        && (current[0].process != previous[0].process
            || current
                .iter()
                .any(|entry| previous.iter().any(|old| old.node == entry.node)))
    {
        return None;
    }
    Some(current)
}
fn presence_reply<T>(
    replies: &FifoChannelHandler<T>,
    deadline: Instant,
    observer_open: impl FnOnce() -> bool,
) -> Option<T> {
    match replies.recv_deadline(deadline) {
        Ok(Some(reply)) => Some(reply),
        Ok(None) => panic!("presence query absolute deadline"),
        Err(error) => {
            // DeclareFinal drops the query's sole FIFO sender. A drained,
            // disconnected query is complete only while its observer is open.
            assert!(replies.is_disconnected() && replies.is_empty(), "{error}");
            assert!(observer_open(), "presence observer closed");
            None
        }
    }
}
fn discard_timed_out_presence(
    tokens: &mut Vec<Vec<String>>,
    channel: &mut Vec<String>,
    encoding: &Encoding,
    payload: &ZBytes,
) {
    // Pinned Zenoh emits exactly this error for the bounded query timeout.
    // Nothing from any channel in this incomplete attempt may qualify.
    assert!(
        encoding == &Encoding::ZENOH_STRING
            && payload.len() == 7
            && payload.to_bytes().as_ref() == b"Timeout",
        "unexpected presence query error"
    );
    tokens.clear();
    channel.clear();
}
fn wait_generation(
    server: &mut Server,
    observer: &zenoh::Session,
    keys: &[String; 3],
    previous: Option<&[InputPresence; 3]>,
    deadline: Instant,
) -> [InputPresence; 3] {
    loop {
        let mut tokens = Vec::new();
        let mut complete = true;
        for (index, key) in keys.iter().enumerate() {
            let remaining = deadline
                .checked_duration_since(Instant::now())
                .expect("fresh wrench generation deadline");
            assert!(!remaining.is_zero());
            let replies = observer
                .liveliness()
                .get(presence_selector(key, index))
                .timeout(remaining.min(PRESENCE_QUERY_BOUND))
                .wait()
                .unwrap();
            let mut channel = Vec::new();
            let mut timed_out = false;
            while let Some(reply) = presence_reply(&replies, deadline, || !observer.is_closed()) {
                match reply.into_result() {
                    Ok(sample) => {
                        assert!(!timed_out, "presence sample after terminal timeout");
                        let token = sample.key_expr().as_str().to_owned();
                        assert!(
                            parse_presence(key, index, &token).is_some(),
                            "malformed wrench input presence"
                        );
                        channel.push(token);
                        assert!(channel.len() <= 2, "ambiguous wrench input generations");
                    }
                    Err(error) => {
                        assert!(!timed_out, "duplicate presence query timeout");
                        discard_timed_out_presence(
                            &mut tokens,
                            &mut channel,
                            error.encoding(),
                            error.payload(),
                        );
                        timed_out = true;
                        complete = false;
                    }
                }
            }
            if timed_out {
                break;
            }
            tokens.push(channel);
        }
        if complete && tokens.len() == 3 && tokens.iter().all(|channel| channel.len() == 1) {
            let selected = std::array::from_fn(|index| tokens[index][0].clone());
            if let Some(generation) = presence_generation(keys, &selected, previous) {
                assert!(
                    Instant::now() < deadline,
                    "fresh wrench generation arrived after deadline"
                );
                return generation;
            }
        }
        assert!(
            Instant::now() < deadline,
            "fresh three-channel wrench generation missing"
        );
        server.update().unwrap();
        if let Some(delay) = poll_delay(Instant::now(), deadline) {
            std::thread::sleep(delay);
        }
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
        // Explicit finite upper bound for Block publishing; negative values mean
        // unlimited upstream and are deliberately not used by this test fixture.
        cfg.insert_json5(
            "transport/link/tx/queue/congestion_control/block/wait_before_close",
            &ADMISSION_BOUND.as_micros().to_string(),
        )
        .unwrap();
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
        .congestion_control(zenoh::qos::CongestionControl::Block)
        .wait()
        .unwrap();
    let persistent = network
        .observer
        .declare_publisher(keys[1].clone())
        .congestion_control(zenoh::qos::CongestionControl::Block)
        .wait()
        .unwrap();
    let clear = network
        .observer
        .declare_publisher(keys[2].clone())
        .congestion_control(zenoh::qos::CongestionControl::Block)
        .wait()
        .unwrap();
    let mut server = network.server(sim, options).unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    let readiness_deadline = Instant::now() + ADMISSION_BOUND;
    wait_until(
        &mut server,
        "rust initial all-three input matching",
        readiness_deadline,
        |_| {
            once.matching_status().wait().unwrap().matching()
                && persistent.matching_status().wait().unwrap().matching()
                && clear.matching_status().wait().unwrap().matching()
        },
    );
    let original_generation = wait_generation(
        &mut server,
        &network.observer,
        &keys,
        None,
        readiness_deadline,
    );
    let deadline = Instant::now() + ADMISSION_BOUND;
    put_once(
        &once,
        wrench(body, 4.0, 2.0).encode_to_vec(),
        &EntityWrench::full_name(),
        "rust once admission queued=1",
        deadline,
    );
    wait_until(&mut server, "rust once admission queued=1", deadline, |s| {
        s.counters().link_wrenches_queued == 1
    });
    assert_eq!(motion(server.simulation()), ([0.0; 3], [0.0; 3]));
    assert_eq!(server.simulation().elapsed(), Duration::ZERO);
    step(&mut server, 1);
    close(motion(server.simulation()).0[0], 0.002);
    close(motion(server.simulation()).1[2], 0.002);
    let deadline = Instant::now() + ADMISSION_BOUND;
    for value in [wrench(body, 1.0, 2.0), wrench(body, 3.0, 4.0)] {
        put_once(
            &persistent,
            value.encode_to_vec(),
            &EntityWrench::full_name(),
            "rust persistent admission queued=3",
            deadline,
        );
    }
    wait_until(
        &mut server,
        "rust persistent admission queued=3",
        deadline,
        |s| s.counters().link_wrenches_queued == 3,
    );
    step(&mut server, 2);
    close(motion(server.simulation()).0[0], 0.006);
    close(motion(server.simulation()).1[2], 0.014);
    let deadline = Instant::now() + ADMISSION_BOUND;
    put_once(
        &clear,
        target(body).encode_to_vec(),
        &Entity::full_name(),
        "rust clear admission clears=1",
        deadline,
    );
    wait_until(
        &mut server,
        "rust clear admission clears=1",
        deadline,
        |s| s.counters().link_wrench_clears_queued == 1,
    );
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
    let deadline = Instant::now() + ADMISSION_BOUND;
    for value in [
        nonzero,
        nonfinite,
        model,
        missing_entity,
        missing_wrench,
        unknown,
    ] {
        put_once(
            &once,
            value.encode_to_vec(),
            &EntityWrench::full_name(),
            "rust invalid batch received=baseline+8",
            deadline,
        );
    }
    put_once(
        &once,
        wrench(body, 1.0, 0.0).encode_to_vec(),
        "gz.msgs.Double",
        "rust invalid batch received=baseline+8",
        deadline,
    );
    put_once(
        &once,
        vec![0xff],
        &EntityWrench::full_name(),
        "rust invalid batch received=baseline+8",
        deadline,
    );
    wait_until(
        &mut server,
        "rust invalid batch received=baseline+8",
        deadline,
        |s| s.counters().wrench_messages_received == received + 8,
    );
    assert_eq!(server.counters().wrench_messages_rejected, rejected + 8);
    assert_eq!(server.counters().link_wrenches_queued, 3);
    // ID takes precedence over a conflicting name. Name-only admission is exact.
    let mut by_id = wrench(body, 2.0, 0.0);
    by_id.entity.as_mut().unwrap().name = "missing::body".into();
    let deadline = Instant::now() + ADMISSION_BOUND;
    put_once(
        &once,
        by_id.encode_to_vec(),
        &EntityWrench::full_name(),
        "rust ID/name admission queued=5",
        deadline,
    );
    let mut by_name = wrench(0, 2.0, 0.0);
    by_name.entity.as_mut().unwrap().name = "default::rig::body".into();
    put_once(
        &persistent,
        by_name.encode_to_vec(),
        &EntityWrench::full_name(),
        "rust ID/name admission queued=5",
        deadline,
    );
    wait_until(
        &mut server,
        "rust ID/name admission queued=5",
        deadline,
        |s| s.counters().link_wrenches_queued == 5,
    );
    reset(&mut server);
    assert_eq!(server.simulation().epoch(), 1);
    step(&mut server, 1);
    assert_eq!(motion(server.simulation()), ([0.0; 3], [0.0; 3]));
    // Reset replaces all three callback generations. Require matching for
    // each publisher before sending into the fresh owners. This strengthens
    // readiness coverage; it does not identify the earlier failed wait's cause.
    let readiness_deadline = Instant::now() + ADMISSION_BOUND;
    wait_until(
        &mut server,
        "rust reset all-three input matching",
        readiness_deadline,
        |_| {
            once.matching_status().wait().unwrap().matching()
                && persistent.matching_status().wait().unwrap().matching()
                && clear.matching_status().wait().unwrap().matching()
        },
    );
    let _fresh_generation = wait_generation(
        &mut server,
        &network.observer,
        &keys,
        Some(&original_generation),
        readiness_deadline,
    );
    let deadline = Instant::now() + ADMISSION_BOUND;
    put_once(
        &once,
        EntityWrench {
            entity: Some(target(body)),
            wrench: Some(Wrench::default()),
            ..Default::default()
        }
        .encode_to_vec(),
        &EntityWrench::full_name(),
        "rust post-reset once admission queued=6",
        deadline,
    );
    wait_until(
        &mut server,
        "rust post-reset once admission queued=6",
        deadline,
        |s| s.counters().link_wrenches_queued == 6,
    );
    let deadline = Instant::now() + ADMISSION_BOUND;
    put_once(
        &persistent,
        wrench(body, 100.0, 3.0).encode_to_vec(),
        &EntityWrench::full_name(),
        "rust post-reset persistent admission queued=7",
        deadline,
    );
    wait_until(
        &mut server,
        "rust post-reset persistent admission queued=7",
        deadline,
        |s| s.counters().link_wrenches_queued == 7,
    );
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
    wait(&mut server, "cpp once admission queued=1", |s| {
        s.counters().link_wrenches_queued == 1
    });
    step(&mut server, 1);
    close(motion(server.simulation()).0[0], 0.002);
    close(motion(server.simulation()).1[2], 0.002);
    peer.next();
    assert_eq!(peer.line(&mut server), "SENT persistent");
    wait(&mut server, "cpp persistent admission queued=3", |s| {
        s.counters().link_wrenches_queued == 3
    });
    step(&mut server, 2);
    close(motion(server.simulation()).0[0], 0.006);
    close(motion(server.simulation()).1[2], 0.014);
    peer.next();
    assert_eq!(peer.line(&mut server), "SENT clear");
    wait(&mut server, "cpp clear admission clears=1", |s| {
        s.counters().link_wrench_clears_queued == 1
    });
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

#[test]
fn wrench_presence_parser_closes_key_type_process_and_uuid_shape() {
    let key = "@m35-test@/world/default/wrench";
    let node = "00000000-0000-4000-8000-000000000001";
    let valid = format!(
        "@gz/m35-test/aabb/{node}/{node}/MS/%/%/%/%world%default%wrench/gz.msgs.EntityWrench/%/%"
    );
    assert!(parse_presence(key, 0, &valid).is_some());
    for invalid in [
        valid.replace("m35-test", "different"),
        valid.replace("/MS/", "/MP/"),
        valid.replace("EntityWrench", "Entity"),
        valid.replace("/aabb/", "/not-a-zid/"),
        valid.replacen(node, "00000000-0000-4000-8000-000000000002", 1),
        format!("{valid}/extra"),
        valid.replace(node, "00000000-0000-0000-0000-000000000000"),
    ] {
        assert!(parse_presence(key, 0, &invalid).is_none(), "{invalid}");
    }
    assert!(parse_presence(key, 2, &valid).is_none());
    assert!(parse_presence("@m35-test@/different", 0, &valid).is_none());
}
#[test]
fn wrench_presence_generation_requires_three_fresh_nodes_and_same_process() {
    let keys = ["wrench", "wrench/persistent", "wrench/clear"]
        .map(|suffix| format!("@m35-test@/world/default/{suffix}"));
    let make = |first: u128| {
        std::array::from_fn(|index| {
            let node = uuid::Uuid::from_u128(first + index as u128)
                .hyphenated()
                .to_string();
            let topic = keys[index]
                .split_once('@')
                .unwrap()
                .1
                .split_once('@')
                .unwrap()
                .1
                .replace('/', "%");
            format!(
                "@gz/m35-test/aabb/{node}/{node}/MS/%/%/%/{topic}/{}/%/%",
                channel_type(index)
            )
        })
    };
    let original = make(1);
    let previous = presence_generation(&keys, &original, None).unwrap();
    let current = make(10);
    assert!(presence_generation(&keys, &current, Some(&previous)).is_some());
    assert!(presence_generation(&keys, &original, Some(&previous)).is_none());
    for (index, old_token) in original.iter().enumerate() {
        let mut stale = current.clone();
        stale[index] = old_token.clone();
        assert!(presence_generation(&keys, &stale, Some(&previous)).is_none());
        let mut missing = current.clone();
        missing[index] = String::new();
        assert!(presence_generation(&keys, &missing, Some(&previous)).is_none());
    }
    let mut other_process = current.clone();
    other_process[1] = other_process[1].replace("/aabb/", "/ccdd/");
    assert!(presence_generation(&keys, &other_process, Some(&previous)).is_none());
    let different_process = current
        .clone()
        .map(|token| token.replace("/aabb/", "/ccdd/"));
    assert!(presence_generation(&keys, &different_process, Some(&previous)).is_none());
    let mut duplicate = current.clone();
    let old_node = uuid::Uuid::from_u128(11).hyphenated().to_string();
    let reused_node = uuid::Uuid::from_u128(10).hyphenated().to_string();
    duplicate[1] = duplicate[1].replace(&old_node, &reused_node);
    assert!(presence_generation(&keys, &duplicate, Some(&previous)).is_none());
}
#[test]
fn wrench_poll_backoff_preserves_deadline_and_never_sleeps_past_it() {
    let now = Instant::now();
    assert_eq!(poll_delay(now, now + ADMISSION_BOUND), Some(POLL_BACKOFF));
    assert_eq!(
        poll_delay(now, now + Duration::from_micros(200)),
        Some(Duration::from_micros(200))
    );
    assert_eq!(poll_delay(now, now), None);
    assert_eq!(poll_delay(now + Duration::from_nanos(1), now), None);
}

#[test]
fn wrench_presence_fifo_drains_queued_replies_then_accepts_open_observer_eof() {
    let (callback, replies): (
        zenoh::handlers::Callback<String>,
        FifoChannelHandler<String>,
    ) = FifoChannel::new(2).into_handler();
    callback.call("actual queued token".to_owned());
    drop(callback);
    let deadline = Instant::now() + PRESENCE_QUERY_BOUND;
    assert_eq!(
        presence_reply(&replies, deadline, || panic!("not EOF yet")),
        Some("actual queued token".to_owned())
    );
    assert!(replies.is_disconnected() && replies.is_empty());
    assert_eq!(presence_reply(&replies, deadline, || true), None);

    let (callback, empty): (
        zenoh::handlers::Callback<String>,
        FifoChannelHandler<String>,
    ) = FifoChannel::new(2).into_handler();
    drop(callback);
    assert_eq!(presence_reply(&empty, deadline, || true), None);
}

#[test]
fn wrench_presence_fifo_deadline_is_not_query_completion() {
    let (_callback, replies): (
        zenoh::handlers::Callback<String>,
        FifoChannelHandler<String>,
    ) = FifoChannel::new(2).into_handler();
    assert!(!replies.is_disconnected());
    let failure = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        presence_reply(&replies, Instant::now(), || true)
    }))
    .expect_err("an empty live FIFO must expire, not complete");
    let message = failure
        .downcast_ref::<&str>()
        .copied()
        .or_else(|| failure.downcast_ref::<String>().map(String::as_str));
    assert_eq!(message, Some("presence query absolute deadline"));
}

#[test]
fn wrench_presence_fifo_eof_checks_current_observer_state() {
    let (callback, replies): (
        zenoh::handlers::Callback<String>,
        FifoChannelHandler<String>,
    ) = FifoChannel::new(2).into_handler();
    drop(callback);
    let observed = std::cell::Cell::new(false);
    let failure = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        presence_reply(&replies, Instant::now() + PRESENCE_QUERY_BOUND, || {
            observed.set(true);
            false
        })
    }));
    let failure = failure.expect_err("observer closure cannot qualify as query completion");
    let message = failure
        .downcast_ref::<&str>()
        .copied()
        .or_else(|| failure.downcast_ref::<String>().map(String::as_str));
    assert_eq!(message, Some("presence observer closed"));
    assert!(observed.get());
}

#[test]
fn wrench_presence_timeout_discards_all_partial_tokens_and_rejects_other_errors() {
    let mut tokens = vec![vec!["earlier completed key".to_owned()]];
    let mut channel = vec!["partial current key".to_owned()];
    discard_timed_out_presence(
        &mut tokens,
        &mut channel,
        &Encoding::ZENOH_STRING,
        &ZBytes::from("Timeout"),
    );
    assert!(tokens.is_empty() && channel.is_empty());
    for (encoding, payload) in [
        (Encoding::ZENOH_BYTES, "Timeout"),
        (Encoding::ZENOH_STRING, "timeout"),
        (Encoding::ZENOH_STRING, "Timeout "),
        (Encoding::ZENOH_STRING, "unexpected"),
    ] {
        let mut tokens = vec![vec!["preserved diagnostic key".to_owned()]];
        let mut channel = vec!["preserved diagnostic partial".to_owned()];
        let before = (tokens.clone(), channel.clone());
        let failure = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            discard_timed_out_presence(
                &mut tokens,
                &mut channel,
                &encoding,
                &ZBytes::from(payload),
            );
        }));
        assert!(failure.is_err());
        assert_eq!((tokens, channel), before);
    }
}
