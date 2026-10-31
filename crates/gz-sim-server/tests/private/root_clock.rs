// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::prost::Name;
use std::sync::{atomic::AtomicBool, mpsc};
use zenoh::{
    Wait,
    liveliness::LivelinessToken,
    pubsub::{Publisher, Subscriber},
    sample::Sample,
};

struct RootNetwork {
    observer: zenoh::Session,
    clients: zenoh::Config,
    _router: zenoh::Session,
}
impl RootNetwork {
    fn new() -> Self {
        let reservation = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("tcp/127.0.0.1:{}", reservation.local_addr().unwrap().port());
        let mut router = zenoh::Config::default();
        router.insert_json5("mode", "\"router\"").unwrap();
        router
            .insert_json5(
                "listen/endpoints",
                &serde_json::to_string(&[&endpoint]).unwrap(),
            )
            .unwrap();
        router
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        router
            .insert_json5("scouting/gossip/enabled", "false")
            .unwrap();
        // A stolen port fails this bind; the test never falls back to discovery.
        drop(reservation);
        let router = zenoh::open(router).wait().unwrap();
        let mut clients = zenoh::Config::default();
        clients.insert_json5("mode", "\"client\"").unwrap();
        clients.insert_json5("listen/endpoints", "[]").unwrap();
        clients
            .insert_json5(
                "connect/endpoints",
                &serde_json::to_string(&[&endpoint]).unwrap(),
            )
            .unwrap();
        clients
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        clients
            .insert_json5("scouting/gossip/enabled", "false")
            .unwrap();
        let observer = zenoh::open(clients.clone()).wait().unwrap();
        Self {
            observer,
            clients,
            _router: router,
        }
    }
    fn server(&self, simulation: Simulation, name: &str, config: &ServerOptions) -> Server {
        Server::with_zenoh_config(
            simulation,
            name,
            config.clone(),
            self.clients.clone(),
            NonZeroUsize::new(2).unwrap(),
        )
        .unwrap()
    }
}

struct ClockSamples {
    _subscriber: Subscriber<()>,
    packets: mpsc::Receiver<Sample>,
    overflow: Arc<AtomicBool>,
}
impl ClockSamples {
    fn recv_timeout(&self, timeout: Duration) -> Result<Vec<u8>, mpsc::RecvTimeoutError> {
        assert!(!self.overflow.load(Ordering::Relaxed));
        let packet = self.packets.recv_timeout(timeout)?;
        assert!(!self.overflow.load(Ordering::Relaxed));
        assert_eq!(
            packet.attachment().unwrap().to_bytes().as_ref(),
            Clock::full_name().as_bytes()
        );
        Ok(packet.payload().to_bytes().into_owned())
    }
}
fn clock_samples(observer: &zenoh::Session, config: &ServerOptions, topic: &str) -> ClockSamples {
    let key = crate::options::qualified_name(
        &config.partition,
        &config.namespace,
        config.topic_remap(topic).unwrap_or(topic),
    )
    .unwrap();
    let (sender, packets) = mpsc::sync_channel(8);
    let overflow = Arc::new(AtomicBool::new(false));
    let failed = Arc::clone(&overflow);
    let subscriber = observer
        .declare_subscriber(key)
        .callback(move |sample| {
            if sender.try_send(sample).is_err() {
                failed.store(true, Ordering::Relaxed);
            }
        })
        .wait()
        .unwrap();
    ClockSamples {
        _subscriber: subscriber,
        packets,
        overflow,
    }
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
struct PublisherRecord {
    process: String,
    node: String,
    message_type: String,
}
fn publisher_records(
    observer: &zenoh::Session,
    config: &ServerOptions,
    topic: &str,
    timeout: Duration,
) -> Vec<PublisherRecord> {
    let key = crate::options::qualified_name(
        &config.partition,
        &config.namespace,
        config.topic_remap(topic).unwrap_or(topic),
    )
    .unwrap();
    let (partition, topic) = key.strip_prefix('@').unwrap().split_once('@').unwrap();
    let partition = partition.replace('/', "%");
    let topic = topic.replace('/', "%");
    let selector = format!("@gz/{partition}/*/*/*/MP/%/%/%/{topic}/*/%/%");
    let deadline = Instant::now() + timeout;
    let replies = observer
        .liveliness()
        .get(selector)
        .timeout(timeout)
        .wait()
        .unwrap();
    let mut records = Vec::new();
    loop {
        let reply = match replies.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
            Ok(Some(reply)) => reply,
            Ok(None) => panic!("publisher snapshot did not finish before its deadline"),
            Err(error) => {
                assert!(replies.is_disconnected(), "{error}");
                assert!(!observer.is_closed());
                break;
            }
        };
        let sample = reply.into_result().unwrap();
        let fields: Vec<_> = sample.key_expr().as_str().split('/').collect();
        assert_eq!(fields.len(), 13);
        assert_eq!(&fields[..2], &["@gz", partition.as_str()]);
        uuid::Uuid::parse_str(fields[2]).unwrap();
        uuid::Uuid::parse_str(fields[3]).unwrap();
        assert_eq!(fields[3], fields[4]);
        assert_eq!(&fields[5..10], &["MP", "%", "%", "%", topic.as_str()]);
        assert_eq!(&fields[11..], &["%", "%"]);
        records.push(PublisherRecord {
            process: fields[2].into(),
            node: fields[3].into(),
            message_type: fields[10].into(),
        });
    }
    records.sort();
    records
}
fn publisher_types(
    observer: &zenoh::Session,
    config: &ServerOptions,
    topic: &str,
    expected: &[&str],
) -> Vec<String> {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let mut types: Vec<_> = publisher_records(
            observer,
            config,
            topic,
            deadline.saturating_duration_since(Instant::now()),
        )
        .into_iter()
        .map(|record| record.message_type)
        .collect();
        types.sort();
        if types
            .iter()
            .map(String::as_str)
            .eq(expected.iter().copied())
        {
            return types;
        }
        assert!(
            Instant::now() < deadline,
            "publisher types {types:?}, expected {expected:?}"
        );
        thread::sleep(Duration::from_millis(1));
    }
}
fn foreign_publisher(
    network: &RootNetwork,
    config: &ServerOptions,
    topic: &str,
    message_type: &str,
) -> (Publisher<'static>, LivelinessToken, zenoh::Session) {
    let key = crate::options::qualified_name(
        &config.partition,
        &config.namespace,
        config.topic_remap(topic).unwrap_or(topic),
    )
    .unwrap();
    let process = uuid::Uuid::new_v4().to_string();
    let node = uuid::Uuid::new_v4().to_string();
    let owner = zenoh::open(network.clients.clone()).wait().unwrap();
    let publisher = owner.declare_publisher(key.clone()).wait().unwrap();
    let token = owner
        .liveliness()
        .declare_token(
            crate::world_io::presence(&key, &process, &node, "MP", message_type).unwrap(),
        )
        .wait()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while !publisher_records(
        &network.observer,
        config,
        topic,
        deadline.saturating_duration_since(Instant::now()),
    )
    .iter()
    .any(|record| {
        record.process == process && record.node == node && record.message_type == message_type
    }) {
        assert!(
            Instant::now() < deadline,
            "foreign publisher identity missing"
        );
        thread::sleep(Duration::from_millis(1));
    }
    (publisher, token, owner)
}
fn wait_matching(publisher: &Publisher<'_>) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !publisher.matching_status().wait().unwrap().matching() {
        assert!(
            Instant::now() < deadline,
            "clock subscription did not match"
        );
        thread::sleep(Duration::from_millis(1));
    }
}
fn running_root_server(
    network: &RootNetwork,
    config: &ServerOptions,
    name: &str,
    step: Duration,
) -> Server {
    let mut server = network.server(Simulation::new(step).unwrap(), name, config);
    assert!(server.apply_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    server
}
fn matching_clocks(server: &mut Server, root: &ClockSamples, world: &ClockSamples) -> Clock {
    wait_matching(&server.world_io.clock);
    wait_matching(&server.root_clock.as_ref().unwrap().publisher);
    server.prepare_io().unwrap();
    let info = server.tick().unwrap();
    let world = world.recv_timeout(Duration::from_secs(5)).unwrap();
    let root = root.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(root, world);
    let clock = Clock::decode(root.as_slice()).unwrap();
    assert_eq!(
        clock.sim,
        Some(time(info.sim_time.as_nanos() as i128).unwrap())
    );
    clock
}

#[test]
fn same_process_root_reuses_complete_clock_bytes_and_survives_reset() {
    let network = RootNetwork::new();
    let configuration = config();
    let observer = &network.observer;
    let root = clock_samples(observer, &configuration, "/clock");
    let world = clock_samples(observer, &configuration, "/world/test/clock");
    // A loaded scene installs the reset handler exercised below.
    let simulation = gz_sim_scene::load_world(
        "<sdf version='1.12'><world name='test'/></sdf>",
        "root-clock.sdf",
        "test",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let mut server = network.server(simulation, "test", &configuration);
    assert!(server.apply_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    assert_eq!(
        publisher_types(observer, &configuration, "/clock", &["gz.msgs.Clock"]),
        ["gz.msgs.Clock"]
    );
    let initial_owner =
        publisher_records(observer, &configuration, "/clock", Duration::from_secs(5));
    assert_eq!(initial_owner.len(), 1);
    for expected in [1_000_000, 2_000_000] {
        assert_eq!(
            matching_clocks(&mut server, &root, &world)
                .sim
                .unwrap()
                .nsec,
            expected
        );
    }
    assert!(server.apply_control(WorldControl {
        reset: Some(gz_sim_proto::msgs::WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert_eq!(
        matching_clocks(&mut server, &root, &world)
            .sim
            .unwrap()
            .nsec,
        0
    );
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    assert_eq!(
        publisher_types(observer, &configuration, "/clock", &["gz.msgs.Clock"]),
        ["gz.msgs.Clock"]
    );
    assert_eq!(
        publisher_records(observer, &configuration, "/clock", Duration::from_secs(5)),
        initial_owner
    );
    assert_eq!(server.counters().clocks_queued, 3);
    assert_eq!(server.counters().root_clocks_queued, 3);
    assert_eq!(server.counters().root_clock_failures, 0);
}

#[test]
fn any_known_publisher_type_blocks_root_until_the_next_prepared_run() {
    let network = RootNetwork::new();
    for type_name in ["gz.msgs.Clock", "gz.msgs.StringMsg"] {
        let configuration = config();
        let foreign = foreign_publisher(&network, &configuration, "/clock", type_name);
        let observer = &network.observer;
        let root = clock_samples(observer, &configuration, "/clock");
        let world = clock_samples(observer, &configuration, "/world/test/clock");
        let mut server =
            running_root_server(&network, &configuration, "test", Duration::from_millis(1));
        assert!(!server.prepare_run(Duration::from_secs(5)).unwrap());
        wait_matching(&server.world_io.clock);
        for expected in [1_000_000, 2_000_000] {
            server.tick().unwrap();
            let packet = world.recv_timeout(Duration::from_secs(5)).unwrap();
            assert_eq!(
                Clock::decode(packet.as_slice()).unwrap().sim.unwrap().nsec,
                expected
            );
        }
        assert_eq!(
            publisher_types(observer, &configuration, "/clock", &[type_name]),
            [type_name]
        );
        assert!(matches!(
            root.recv_timeout(Duration::from_millis(5)),
            Err(mpsc::RecvTimeoutError::Timeout)
        ));
        assert_eq!(server.counters().root_clocks_queued, 0);
        drop(foreign);
        assert!(publisher_types(observer, &configuration, "/clock", &[]).is_empty());
        assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
        assert_eq!(
            matching_clocks(&mut server, &root, &world)
                .sim
                .unwrap()
                .nsec,
            3_000_000
        );
        assert_eq!(server.counters().clocks_queued, 3);
        assert_eq!(server.counters().root_clocks_queued, 1);
    }
}

#[test]
fn late_competitors_and_a_second_world_leave_the_existing_root_owner_intact() {
    let network = RootNetwork::new();
    let configuration = config();
    let observer = &network.observer;
    let root = clock_samples(observer, &configuration, "/clock");
    let world = clock_samples(observer, &configuration, "/world/test/clock");
    let other = clock_samples(observer, &configuration, "/world/other/clock");
    let mut first = running_root_server(&network, &configuration, "test", Duration::from_millis(1));
    assert!(first.prepare_run(Duration::from_secs(5)).unwrap());
    matching_clocks(&mut first, &root, &world);
    let competitor = foreign_publisher(&network, &configuration, "/clock", "gz.msgs.Clock");
    assert!(first.prepare_run(Duration::from_secs(5)).unwrap());
    let mut second =
        running_root_server(&network, &configuration, "other", Duration::from_millis(2));
    assert!(!second.prepare_run(Duration::from_secs(5)).unwrap());
    wait_matching(&second.world_io.clock);
    second.tick().unwrap();
    let packet = other.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(
        Clock::decode(packet.as_slice()).unwrap().sim.unwrap().nsec,
        2_000_000
    );
    assert!(matches!(
        root.recv_timeout(Duration::from_millis(5)),
        Err(mpsc::RecvTimeoutError::Timeout)
    ));
    matching_clocks(&mut first, &root, &world);
    assert_eq!(
        publisher_types(
            observer,
            &configuration,
            "/clock",
            &["gz.msgs.Clock", "gz.msgs.Clock"]
        )
        .len(),
        2
    );
    drop(competitor);
    assert!(!second.prepare_run(Duration::from_secs(5)).unwrap());
    assert_eq!(first.counters().root_clocks_queued, 2);
    assert_eq!(second.counters().clocks_queued, 1);
    assert_eq!(second.counters().root_clocks_queued, 0);
}

#[test]
fn invalid_run_deadlines_do_not_admit_root_or_advance_simulation() {
    let network = RootNetwork::new();
    let configuration = config();
    let observer = &network.observer;
    let mut server =
        running_root_server(&network, &configuration, "test", Duration::from_millis(1));
    for (timeout, expected) in [
        (Duration::ZERO, SetupError::Timeout),
        (Duration::MAX, SetupError::InvalidConfig),
    ] {
        assert!(matches!(
            server.prepare_run(timeout),
            Err(Error::Setup(actual)) if actual == expected
        ));
        assert!(publisher_types(observer, &configuration, "/clock", &[]).is_empty());
        assert_eq!(server.counters().clocks_queued, 0);
        assert_eq!(server.counters().root_clocks_queued, 0);
    }
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    assert_eq!(server.tick().unwrap().sim_time, Duration::from_millis(1));
}

#[test]
fn world_interest_also_attempts_root_and_packet_limits_report_each_endpoint() {
    let network = RootNetwork::new();
    let configuration = config();
    let observer = &network.observer;
    let world = clock_samples(observer, &configuration, "/world/test/clock");
    let mut server =
        running_root_server(&network, &configuration, "test", Duration::from_millis(1));
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    wait_matching(&server.world_io.clock);
    server.tick().unwrap();
    world.recv_timeout(Duration::from_secs(5)).unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    assert_eq!(server.counters().clocks_queued, 1);
    assert_eq!(server.counters().root_clocks_queued, 1);

    let mut idle = running_root_server(&network, &config(), "idle", Duration::from_millis(1));
    assert!(idle.prepare_run(Duration::from_secs(5)).unwrap());
    idle.tick().unwrap();
    assert_eq!(idle.counters().root_clocks_queued, 0);

    let configuration = ServerOptions {
        max_message_bytes: 1,
        ..config()
    };
    let observer = &network.observer;
    let _root = clock_samples(observer, &configuration, "/clock");
    let mut server =
        running_root_server(&network, &configuration, "test", Duration::from_millis(1));
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    wait_matching(&server.root_clock.as_ref().unwrap().publisher);
    server.tick().unwrap();
    assert_eq!(server.counters().clocks_queued, 0);
    assert_eq!(server.counters().clock_failures, 1);
    assert_eq!(server.counters().root_clocks_queued, 0);
    assert_eq!(server.counters().root_clock_failures, 1);
}

#[test]
fn stopped_workers_report_admission_and_root_snapshot_failures() {
    let network = RootNetwork::new();
    let configuration = config();
    let observer = &network.observer;
    let _root = clock_samples(observer, &configuration, "/clock");
    let mut admitted =
        running_root_server(&network, &configuration, "test", Duration::from_millis(1));
    assert!(admitted.prepare_run(Duration::from_secs(5)).unwrap());
    admitted.shutdown();
    assert!(admitted.world_io.session.is_closed());
    assert!(matches!(
        admitted.prepare_run(Duration::from_secs(5)),
        Err(Error::WorldNetwork(_))
    ));
    assert!(admitted.tick().is_err());
    assert_eq!(admitted.counters().root_clocks_queued, 0);
    assert_eq!(admitted.counters().root_clock_failures, 1);

    let configuration = config();
    let _foreign = foreign_publisher(&network, &configuration, "/clock", "gz.msgs.Clock");
    let mut blocked =
        running_root_server(&network, &configuration, "test", Duration::from_millis(1));
    blocked.shutdown();
    assert!(blocked.world_io.session.is_closed());
    assert!(matches!(
        blocked.prepare_run(Duration::from_secs(5)),
        Err(Error::WorldNetwork(_))
    ));
}

#[test]
fn construction_does_not_claim_root_before_a_competitor_can_advertise() {
    let network = RootNetwork::new();
    let configuration = config();
    let observer = &network.observer;
    let root = clock_samples(observer, &configuration, "/clock");
    let world = clock_samples(observer, &configuration, "/world/test/clock");
    let mut server =
        running_root_server(&network, &configuration, "test", Duration::from_millis(1));
    assert!(publisher_types(observer, &configuration, "/clock", &[]).is_empty());
    let competitor = foreign_publisher(&network, &configuration, "/clock", "gz.msgs.StringMsg");
    assert!(!server.prepare_run(Duration::from_secs(5)).unwrap());
    wait_matching(&server.world_io.clock);
    server.tick().unwrap();
    world.recv_timeout(Duration::from_secs(5)).unwrap();
    assert!(matches!(
        root.recv_timeout(Duration::from_millis(5)),
        Err(mpsc::RecvTimeoutError::Timeout)
    ));
    drop(competitor);
    assert!(publisher_types(observer, &configuration, "/clock", &[]).is_empty());
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    assert_eq!(
        matching_clocks(&mut server, &root, &world)
            .sim
            .unwrap()
            .nsec,
        2_000_000
    );
}

#[test]
fn root_clock_namespace_and_relative_remap_preserve_metadata_scope() {
    let network = RootNetwork::new();
    for remap in [None, Some("time")] {
        let mut configuration = ServerOptions {
            namespace: "/robot".into(),
            ..config()
        };
        if let Some(remap) = remap {
            configuration.add_topic_remap("/clock", remap).unwrap();
        }
        let observer = &network.observer;
        let root = clock_samples(observer, &configuration, "/clock");
        let world = clock_samples(observer, &configuration, "/world/test/clock");
        let wrong_namespace = ServerOptions {
            namespace: "/other".into(),
            ..configuration.clone()
        };
        let outside = remap.map(|_| clock_samples(observer, &wrong_namespace, "/clock"));
        let topic = configuration.topic_remap("/clock").unwrap_or("/clock");
        assert_eq!(
            crate::options::qualified_name(
                &configuration.partition,
                &configuration.namespace,
                topic
            )
            .unwrap(),
            format!(
                "@/{}@{}",
                configuration.partition,
                if remap.is_some() {
                    "/robot/time"
                } else {
                    "/clock"
                }
            )
        );
        let mut server =
            running_root_server(&network, &configuration, "test", Duration::from_millis(1));
        assert!(publisher_types(observer, &configuration, "/clock", &[]).is_empty());
        assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
        assert_eq!(
            matching_clocks(&mut server, &root, &world)
                .sim
                .unwrap()
                .nsec,
            1_000_000
        );
        assert_eq!(
            publisher_types(observer, &configuration, "/clock", &["gz.msgs.Clock"]),
            ["gz.msgs.Clock"]
        );
        if let Some(outside) = outside {
            assert!(matches!(
                outside.recv_timeout(Duration::from_millis(5)),
                Err(mpsc::RecvTimeoutError::Timeout)
            ));
            assert!(publisher_types(observer, &wrong_namespace, "/clock", &[]).is_empty());
        }
        assert_eq!(server.counters().clocks_queued, 1);
        assert_eq!(server.counters().root_clocks_queued, 1);
        assert_eq!(server.counters().root_clock_failures, 0);
    }
}
