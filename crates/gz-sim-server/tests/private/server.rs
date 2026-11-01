// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
    thread,
};
use zenoh::{
    pubsub::Subscriber,
    sample::{Locality, Sample},
};

static ID: AtomicU64 = AtomicU64::new(0);

fn config() -> ServerOptions {
    ServerOptions {
        partition: format!(
            "sim-{}-{}",
            std::process::id(),
            ID.fetch_add(1, Ordering::Relaxed)
        ),
        ..ServerOptions::default()
    }
}

// Concrete test clients; production ownership stays with Server and WorldIo.
pub(super) fn isolated_zenoh_config() -> zenoh::Config {
    let mut config = zenoh::Config::default();
    for (path, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(path, value).unwrap();
    }
    config
}

pub(super) fn isolated_server(
    simulation: Simulation,
    name: &str,
    config: ServerOptions,
    budget: NonZeroUsize,
) -> Result<Server, Error> {
    Server::with_zenoh_config(simulation, name, config, isolated_zenoh_config(), budget)
}

pub(super) struct SampleReceiver {
    _subscriber: Subscriber<()>,
    incoming: mpsc::Receiver<Sample>,
    overflow: Arc<AtomicBool>,
    key: String,
    message_type: String,
}
impl SampleReceiver {
    pub(super) fn recv_timeout(&self, timeout: Duration) -> Result<Sample, mpsc::RecvTimeoutError> {
        assert!(
            !self.overflow.load(Ordering::Acquire),
            "sample observation overflowed"
        );
        let received = self.incoming.recv_timeout(timeout);
        assert!(
            !self.overflow.load(Ordering::Acquire),
            "sample observation overflowed"
        );
        let sample = received?;
        assert_eq!(sample.key_expr().as_str(), self.key);
        assert_eq!(
            sample.attachment().unwrap().to_bytes().as_ref(),
            self.message_type.as_bytes()
        );
        Ok(sample)
    }
}

pub(super) fn sample_channel(
    session: &zenoh::Session,
    key: String,
    message_type: &str,
    capacity: usize,
) -> SampleReceiver {
    let (sender, incoming) = mpsc::sync_channel(capacity);
    let overflow = Arc::new(AtomicBool::new(false));
    let failed = Arc::clone(&overflow);
    let subscriber = session
        .declare_subscriber(key.clone())
        // D28 preserves direct local delivery through actual Session locality,
        // rather than inventing the discarded SDK MessageInfo boolean.
        .allowed_origin(Locality::SessionLocal)
        .callback(move |sample| {
            if sender.try_send(sample).is_err() {
                failed.store(true, Ordering::Release);
            }
        })
        .wait()
        .unwrap();
    SampleReceiver {
        _subscriber: subscriber,
        incoming,
        overflow,
        key,
        message_type: message_type.into(),
    }
}
pub(super) fn samples(server: &Server, topic: &str, message_type: &str) -> SampleReceiver {
    let config = &server.network_config;
    let key = crate::options::qualified_name(
        &config.partition,
        &config.namespace,
        config.topic_remap(topic).unwrap_or(topic),
    )
    .unwrap();
    let samples = sample_channel(&server.world_io.session, key, message_type, config.capacity);
    wait_imu_interest(server, topic, true);
    samples
}

// Zenoh dispatches matching changes asynchronously. Establish test-client
// readiness outside stepping before asserting the next committed sample.
pub(super) fn wait_imu_interest(server: &Server, topic: &str, expected: bool) {
    let config = &server.network_config;
    let key = crate::options::qualified_name(
        &config.partition,
        &config.namespace,
        config.topic_remap(topic).unwrap_or(topic),
    )
    .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    for stream in server.imus.iter().filter(|stream| stream.endpoint == key) {
        while stream.publisher.matching().unwrap() != expected {
            assert!(
                Instant::now() < deadline,
                "IMU interest did not become {expected}: {topic}"
            );
            thread::sleep(Duration::from_millis(1));
        }
    }
}

#[test]
fn tcp_listener_reservation_tracks_actual_session_lifecycle() {
    use std::process::{Child, Command, Stdio};
    const CHILD_ADDRESS: &str = "GZ_SIM_TCP_LIFECYCLE_ADDRESS";
    const HOST_NAMESPACE: &str = "GZ_SIM_TCP_LIFECYCLE_HOST_NAMESPACE";
    if std::env::var_os(CHILD_ADDRESS).is_none() {
        // Keep this exact address occupied on the host for the child's entire
        // lifecycle. Its successful binds must occur in a different netns.
        let host = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let address = host.local_addr().unwrap();
        struct OwnedChild(Child);
        impl Drop for OwnedChild {
            fn drop(&mut self) {
                if matches!(self.0.try_wait(), Ok(None)) {
                    let _ = self.0.kill();
                }
                let _ = self.0.wait();
            }
        }
        let mut child = OwnedChild(
            Command::new("unshare")
                .args([
                    "-Urn",
                    "--",
                    "sh",
                    "-c",
                    "ip link set dev lo up && exec \"$@\"",
                    "tcp-lifecycle",
                ])
                .arg(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "tests::tcp_listener_reservation_tracks_actual_session_lifecycle",
                    "--nocapture",
                ])
                .env(CHILD_ADDRESS, address.to_string())
                .env(
                    HOST_NAMESPACE,
                    std::fs::read_link("/proc/self/ns/net").unwrap(),
                )
                .stdin(Stdio::null())
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .unwrap(),
        );
        let deadline = Instant::now() + Duration::from_secs(20);
        let status = loop {
            if let Some(status) = child.0.try_wait().unwrap() {
                break status;
            }
            assert!(
                Instant::now() < deadline,
                "isolated TCP lifecycle child stalled"
            );
            thread::sleep(Duration::from_millis(5));
        };
        let mut stdout = String::new();
        let mut stderr = String::new();
        std::io::Read::read_to_string(&mut child.0.stdout.take().unwrap(), &mut stdout).unwrap();
        std::io::Read::read_to_string(&mut child.0.stderr.take().unwrap(), &mut stderr).unwrap();
        assert!(status.success(), "{status}: {stdout} {stderr}");
        assert!(
            stdout.contains(
                "test tests::tcp_listener_reservation_tracks_actual_session_lifecycle ... ok"
            ) && stdout.contains("test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured;"),
            "isolated lifecycle test did not execute: {stdout}"
        );
        assert_eq!(host.local_addr().unwrap(), address);
        assert_eq!(
            std::net::TcpListener::bind(address).unwrap_err().kind(),
            std::io::ErrorKind::AddrInUse
        );
        return;
    }
    let host_namespace = std::env::var_os(HOST_NAMESPACE).unwrap();
    assert_ne!(
        std::fs::read_link("/proc/self/ns/net").unwrap(),
        std::path::PathBuf::from(host_namespace),
        "TCP lifecycle did not enter a private network namespace"
    );
    // D28 replaces the discarded SDK's reusable UDP discovery-port fixture.
    // A stolen port at the explicit handoff fails setup; there is no retry.
    let address = std::env::var(CHILD_ADDRESS).unwrap();
    let reserved = std::net::TcpListener::bind(address.as_str()).unwrap();
    let address = reserved.local_addr().unwrap();
    assert_eq!(
        std::net::TcpListener::bind(address).unwrap_err().kind(),
        std::io::ErrorKind::AddrInUse
    );
    let mut config = isolated_zenoh_config();
    config
        .insert_json5(
            "listen/endpoints",
            &serde_json::to_string(&[format!("tcp/{address}")]).unwrap(),
        )
        .unwrap();
    drop(reserved);
    let session = zenoh::open(config).wait().unwrap();
    assert_eq!(
        std::net::TcpListener::bind(address).unwrap_err().kind(),
        std::io::ErrorKind::AddrInUse
    );
    session.close().wait().unwrap();
    let released = std::net::TcpListener::bind(address).unwrap();
    assert_eq!(released.local_addr().unwrap(), address);
}

fn server(config: ServerOptions, step: Duration) -> Server {
    let mut server = isolated_server(
        Simulation::new(step).unwrap(),
        "test",
        config,
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    server.set_pose_publication_period(Duration::ZERO);
    server.set_interest_policy(InterestPolicy::Always);
    server
}

#[test]
fn active_time_excludes_pauses_and_repeated_state_changes() {
    let start = Instant::now();
    let mut timer = ActiveTime::default();
    timer.set_running(false, start);
    assert_eq!(
        timer.elapsed(start + Duration::from_secs(10)),
        Duration::ZERO
    );
    timer.set_running(true, start + Duration::from_secs(10));
    timer.set_running(true, start + Duration::from_secs(11));
    assert_eq!(
        timer.elapsed(start + Duration::from_secs(12)),
        Duration::from_secs(2)
    );
    timer.set_running(false, start + Duration::from_secs(13));
    timer.set_running(false, start + Duration::from_secs(14));
    assert_eq!(
        timer.elapsed(start + Duration::from_secs(20)),
        Duration::from_secs(3)
    );
    timer.set_running(true, start + Duration::from_secs(20));
    assert_eq!(
        timer.elapsed(start + Duration::from_secs(22)),
        Duration::from_secs(5)
    );
}

#[test]
fn clock_fields_retain_distinct_times_and_signed_epoch_boundaries() {
    let message = clock(
        Duration::from_nanos(1_000_000_002),
        Duration::from_nanos(7),
        UNIX_EPOCH + Duration::from_nanos(9),
    )
    .unwrap();
    assert_eq!(message.sim, Some(Time { sec: 1, nsec: 2 }));
    assert_eq!(message.real, Some(Time { sec: 0, nsec: 7 }));
    assert_eq!(message.system, Some(Time { sec: 0, nsec: 9 }));
    assert!(message.header.is_none());
    let negative = clock(
        Duration::ZERO,
        Duration::ZERO,
        UNIX_EPOCH - Duration::from_nanos(1_000_000_002),
    )
    .unwrap();
    assert_eq!(negative.system, Some(Time { sec: -1, nsec: -2 }));
    assert_eq!(
        time(i128::from(i64::MAX) * 1_000_000_000 + 999_999_999)
            .unwrap()
            .sec,
        i64::MAX
    );
    assert!(matches!(
        time((i128::from(i64::MAX) + 1) * 1_000_000_000),
        Err(Error::TimeRange)
    ));
    for (sim, real) in [
        (Duration::MAX, Duration::ZERO),
        (Duration::ZERO, Duration::MAX),
    ] {
        assert!(matches!(
            clock(sim, real, UNIX_EPOCH),
            Err(Error::TimeRange)
        ));
    }
    assert!(Error::TimeRange.to_string().contains("seconds"));
}

#[test]
fn native_pause_and_step_precedence_preserve_pending_counts() {
    let mut server = server(config(), Duration::from_millis(1));
    assert_eq!(server.update().unwrap().sim_time, Duration::ZERO);
    assert!(server.apply_control(WorldControl {
        pause: true,
        step: true,
        multi_step: 3,
        ..Default::default()
    }));
    assert_eq!(server.pending_steps(), 3);
    for step in 1..=3 {
        assert_eq!(
            server.update().unwrap().sim_time,
            Duration::from_millis(step)
        );
    }
    assert!(server.simulation().is_paused());
    assert_eq!(server.pending_steps(), 0);
    assert_eq!(server.update().unwrap().sim_time, Duration::from_millis(3));
    assert!(server.apply_control(WorldControl {
        pause: false,
        multi_step: 17,
        ..Default::default()
    }));
    server.update().unwrap();
    assert_eq!(server.pending_steps(), 0);
    assert!(!server.simulation().is_paused());
    assert!(server.apply_control(WorldControl {
        pause: true,
        step: true,
        ..Default::default()
    }));
    server.update().unwrap();
    assert!(server.simulation().is_paused());
    assert!(server.apply_control(WorldControl {
        pause: true,
        multi_step: 2,
        ..Default::default()
    }));
    assert!(server.apply_control(WorldControl {
        pause: true,
        multi_step: 3,
        ..Default::default()
    }));
    assert_eq!(server.pending_steps(), 5);
    assert!(server.apply_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    assert_eq!(server.update().unwrap().sim_time, Duration::from_millis(5));
    assert_eq!(server.pending_steps(), 5);
    assert!(server.apply_control(WorldControl::default()));
    for _ in 0..5 {
        server.update().unwrap();
    }
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(10));
    assert!(server.simulation().is_paused());
    assert!(server.counters().clocks_queued > 0);
    let counters = server.counters();
    assert_eq!(
        (
            counters.clock_failures,
            counters.pose_failures,
            counters.imu_failures,
            counters.root_clock_failures
        ),
        (0, 0, 0, 0)
    );
    server.shutdown();
}

#[test]
fn unsupported_and_overflowing_controls_leave_state_unchanged() {
    let mut server = server(config(), Duration::from_millis(1));
    for request in [
        WorldControl {
            seed: 1,
            ..Default::default()
        },
        WorldControl {
            run_to_sim_time: Some(Time::default()),
            ..Default::default()
        },
        WorldControl {
            reset: Some(gz_sim_proto::msgs::WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        },
        WorldControl {
            reset: Some(gz_sim_proto::msgs::WorldReset {
                time_only: true,
                ..Default::default()
            }),
            ..Default::default()
        },
    ] {
        assert!(!server.apply_control(request));
    }
    assert!(server.simulation().is_paused());
    assert_eq!(server.pending_steps(), 0);
    assert!(server.apply_control(WorldControl {
        pause: true,
        reset: Some(Default::default()),
        ..Default::default()
    }));
    server.pending_steps = u64::MAX;
    assert!(!server.apply_control(WorldControl {
        pause: true,
        step: true,
        ..Default::default()
    }));
    assert_eq!(server.pending_steps(), u64::MAX);
    let mut huge = self::server(config(), Duration::MAX);
    assert!(!huge.apply_control(WorldControl {
        pause: true,
        multi_step: 2,
        ..Default::default()
    }));
    assert!(!huge.apply_control(WorldControl {
        pause: true,
        step: true,
        ..Default::default()
    }));
    huge.simulation.set_paused(false);
    assert!(matches!(huge.update(), Err(Error::TimeRange)));
    assert_eq!(huge.simulation().elapsed(), Duration::ZERO);
    huge.simulation.step_exact(1).unwrap();
    assert!(matches!(
        huge.update(),
        Err(Error::Step(StepError::Overflow))
    ));
    assert!(
        Error::Step(StepError::Overflow)
            .to_string()
            .contains("overflow")
    );
    assert!(Error::WorldClosed.to_string().contains("closed"));
}

#[test]
fn setup_rejects_invalid_names_configuration_and_unrepresentable_time() {
    // These sanitize to an empty or invalid topic, which native Gazebo also rejects.
    for name in ["", "/", "//", "@~", ":="] {
        assert!(
            isolated_server(
                Simulation::new(Duration::from_millis(1)).unwrap(),
                name,
                config(),
                NonZeroUsize::new(1).unwrap()
            )
            .is_err()
        );
    }
    let invalid = ServerOptions {
        capacity: 0,
        ..config()
    };
    assert!(matches!(
        isolated_server(
            Simulation::new(Duration::from_millis(1)).unwrap(),
            "test",
            invalid,
            NonZeroUsize::new(1).unwrap()
        ),
        Err(Error::Setup(SetupError::InvalidConfig))
    ));

    // D28 retires the SDK shared endpoint quota: clock held its first lease and
    // control registration exhausted capacity1. Capacity now bounds input work.
    let mut one = server(
        ServerOptions {
            capacity: 1,
            ..config()
        },
        Duration::from_millis(1),
    );
    one.command_budget = NonZeroUsize::new(1).unwrap();
    let clocks = samples(&one, "/world/test/clock", "gz.msgs.Clock");
    let first = WorldControl {
        pause: true,
        multi_step: 1,
        ..Default::default()
    }
    .encode_to_vec();
    let accepted = request(&one, first.clone());
    let pending = one
        .world_io
        .controls
        .recv_timeout(Duration::from_secs(5))
        .unwrap();
    assert_eq!(pending.payload, first);
    let rejected = request(
        &one,
        WorldControl {
            pause: true,
            multi_step: 9,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    assert!(rejected.join().unwrap().is_err());
    assert_eq!(one.ingress_statistics().queue_drops, 1);
    assert_eq!(one.pending_steps(), 0);
    assert_eq!(one.counters().controls_accepted, 0);
    one.respond(pending);
    let (bytes, success) = accepted.join().unwrap().unwrap();
    assert!(success && Boolean::decode(bytes.as_slice()).unwrap().data);
    assert_eq!(one.pending_steps(), 1);
    let info = one.update().unwrap();
    assert_eq!(info.sim_time, Duration::from_millis(1));
    assert_eq!(one.pending_steps(), 0);
    assert!(one.simulation().is_paused());
    let frame = clocks.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(
        Clock::decode(frame.payload().to_bytes().as_ref())
            .unwrap()
            .sim,
        Some(Time {
            sec: 0,
            nsec: 1_000_000
        })
    );
    assert_eq!(
        (
            one.counters().controls_accepted,
            one.counters().controls_rejected,
            one.counters().clocks_queued,
            one.counters().clock_failures
        ),
        (1, 1, 1, 0)
    );
    assert!(matches!(
        one.world_io.controls.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));
    one.shutdown();
    let mut huge = Simulation::new(Duration::MAX).unwrap();
    huge.step_exact(1).unwrap();
    assert!(matches!(
        isolated_server(huge, "test", config(), NonZeroUsize::new(1).unwrap()),
        Err(Error::TimeRange)
    ));
}

fn request(server: &Server, bytes: Vec<u8>) -> thread::JoinHandle<Result<(Vec<u8>, bool), String>> {
    let session = server.world_io.session.clone();
    let config = &server.network_config;
    let topic = config
        .topic_remap("/world/test/control")
        .unwrap_or("/world/test/control");
    let key = crate::options::qualified_name(&config.partition, &config.namespace, topic).unwrap();
    thread::spawn(move || {
        let replies = session
            .get(key)
            .payload(bytes)
            .timeout(Duration::from_secs(5))
            .wait()
            .map_err(|error| error.to_string())?;
        let reply = replies
            .recv_timeout(Duration::from_secs(5))
            .map_err(|error| error.to_string())?
            .ok_or("control timed out")?;
        let sample = reply.result().map_err(|error| format!("{error:?}"))?;
        Ok((sample.payload().to_bytes().into_owned(), true))
    })
}

#[test]
fn world_names_are_sanitized_like_native_topics() {
    for (name, topic) in [
        ("bad world", "/world/bad_world/clock"),
        ("a@b~c", "/world/abc/clock"),
        ("nested/world", "/world/nested/world/clock"),
    ] {
        let configuration = config();
        let mut server = isolated_server(
            Simulation::new(Duration::from_millis(1)).unwrap(),
            name,
            configuration,
            NonZeroUsize::new(1).unwrap(),
        )
        .unwrap();
        let key =
            crate::options::qualified_name(&server.partition, &server.namespace, topic).unwrap();
        let clock = server
            .world_io
            .session
            .declare_subscriber(key)
            .wait()
            .unwrap();
        assert!(
            server.prepare_run(Duration::from_secs(5)).unwrap(),
            "{name}"
        );
        server.update().unwrap();
        let sample = clock.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
        assert!(
            Clock::decode(sample.payload().to_bytes().as_ref())
                .unwrap()
                .sim
                .is_some()
        );
    }
}

#[test]
fn real_transport_commands_report_acceptance_and_exact_step_completion() {
    let config = config();
    let mut server = server(config.clone(), Duration::from_millis(1));
    let subscription = server
        .world_io
        .session
        .declare_subscriber(server.world_io.clock.key_expr().clone().into_owned())
        .wait()
        .unwrap();
    let mut first = None;
    let deadline = Instant::now() + Duration::from_secs(5);
    while first.is_none() && Instant::now() < deadline {
        server.update().unwrap();
        if let Ok(Some(message)) = subscription.recv_timeout(Duration::from_millis(2)) {
            first = Some(Clock::decode(message.payload().to_bytes().as_ref()).unwrap());
        }
    }
    assert_eq!(first.unwrap().sim.unwrap(), Time::default());
    for (bytes, accepted) in [
        (vec![0x80], false),
        (
            WorldControl {
                seed: 1,
                ..Default::default()
            }
            .encode_to_vec(),
            false,
        ),
        (
            WorldControl {
                pause: true,
                multi_step: 3,
                ..Default::default()
            }
            .encode_to_vec(),
            true,
        ),
    ] {
        let result = request(&server, bytes);
        let deadline = Instant::now() + Duration::from_secs(5);
        while !result.is_finished() && Instant::now() < deadline {
            server.update().unwrap();
            thread::yield_now();
        }
        let (bytes, success) = result.join().unwrap().unwrap();
        assert!(success);
        assert_eq!(Boolean::decode(bytes.as_slice()).unwrap().data, accepted);
    }
    while server.pending_steps() > 0 {
        server.update().unwrap();
    }
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(3));
    let deadline = Instant::now() + Duration::from_secs(5);
    let final_clock = loop {
        assert!(Instant::now() < deadline);
        server.update().unwrap();
        if let Ok(Some(message)) = subscription.recv_timeout(Duration::from_millis(2)) {
            let clock = Clock::decode(message.payload().to_bytes().as_ref()).unwrap();
            if clock
                .sim
                .as_ref()
                .is_some_and(|time| time.nsec == 3_000_000)
            {
                break clock;
            }
        }
    };
    assert!(final_clock.real.unwrap().nsec > 0);
    assert!(final_clock.system.unwrap().sec > 1_000_000_000);
    assert_eq!(server.counters().controls_rejected, 2);
    assert_eq!(server.counters().controls_accepted, 1);
    server.shutdown();
    let stopped_info = *server.simulation.world().resource::<UpdateInfo>();
    let stopped_pending = server.pending_steps();
    let stopped_counters = format!("{:?}", server.counters());
    for _ in 0..3 {
        assert!(matches!(server.update(), Err(Error::WorldClosed)));
        assert_eq!(
            *server.simulation.world().resource::<UpdateInfo>(),
            stopped_info
        );
        assert_eq!(server.simulation.elapsed(), stopped_info.sim_time);
        assert_eq!(server.pending_steps(), stopped_pending);
        assert_eq!(format!("{:?}", server.counters()), stopped_counters);
    }
}

#[test]
fn shutdown_after_receipt_reports_reply_failure_without_waiting() {
    let config = config();
    let mut server = server(config.clone(), Duration::from_millis(1));
    let result = request(&server, WorldControl::default().encode_to_vec());
    let received = server
        .world_io
        .controls
        .recv_timeout(Duration::from_secs(5))
        .unwrap();
    server.world_io.session.close().wait().unwrap();
    server.respond(received);
    assert_eq!(server.counters().response_failures, 1);
    assert_eq!(server.counters().controls_accepted, 1);
    server.shutdown();
    assert!(result.join().unwrap().is_err());
}

fn imu_server(config: ServerOptions) -> Server {
    let simulation = gz_sim_scene::load_world(
        include_str!("../../../gz-sim-scene/tests/fixtures/fall.sdf"),
        "fall.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let mut server =
        isolated_server(simulation, "default", config, NonZeroUsize::new(2).unwrap()).unwrap();
    server.set_pose_publication_period(Duration::ZERO);
    server.set_interest_policy(InterestPolicy::Always);
    server
}

#[test]
fn shutdown_releases_imu_leases_and_storage_without_dropping_simulation() {
    let mut server = imu_server(config());
    server.simulation.set_paused(false);
    server.update().unwrap();
    let leases: Vec<_> = server
        .imus
        .iter()
        .map(|stream| Arc::downgrade(&stream.publisher))
        .collect();
    assert!(!leases.is_empty());
    assert!(server.imus.capacity() > 0);
    assert!(leases.iter().all(|lease| lease.upgrade().is_some()));
    let elapsed = server.simulation.elapsed();
    let entities = server.simulation.world().entities().len();
    let counters = server.counters();

    for _ in 0..2 {
        server.shutdown();
        assert!(server.imus.is_empty());
        assert_eq!(server.imus.capacity(), 0);
        assert!(leases.iter().all(|lease| lease.upgrade().is_none()));
        assert!(server.world_io.session.is_closed());
        assert_eq!(server.simulation.elapsed(), elapsed);
        assert_eq!(server.simulation.world().entities().len(), entities);
        assert_eq!(server.counters().imus_queued, counters.imus_queued);
    }
}

#[test]
fn imu_message_matches_native_metadata_presence_covariance_and_sequence() {
    let metadata = ImuPublication {
        enable_orientation: true,
        entity_name: "robot::body::imu".into(),
        frame_id: "robot::body::imu".into(),
        topic: "/imu".into(),
    };
    let mut sample = ImuSample {
        time: Duration::from_millis(1500),
        epoch: 0,
        reading: Ok(gz_sensors::ImuReading {
            orientation: [0.0, 0.0, 0.0, 1.0],
            specific_force: [0.0, 0.0, 9.81],
            angular_velocity: [1.0, 2.0, 3.0],
            angular_velocity_covariance: [0.0; 3],
            linear_acceleration_covariance: [0.0; 3],
        }),
    };
    let packet = imu_packet(&metadata, &sample, u64::MAX).unwrap();
    assert_eq!(packet.entity_name, "robot::body::imu");
    let header = packet.header.unwrap();
    assert_eq!(
        header.stamp.unwrap(),
        Time {
            sec: 1,
            nsec: 500_000_000
        }
    );
    assert_eq!(header.data.len(), 2);
    assert_eq!(
        (&header.data[0].key[..], &header.data[0].value[..]),
        ("frame_id", &["robot::body::imu".to_owned()][..])
    );
    assert_eq!(
        (&header.data[1].key[..], &header.data[1].value[..]),
        ("seq", &[u64::MAX.to_string()][..])
    );
    for covariance in [
        packet.orientation_covariance,
        packet.angular_velocity_covariance,
        packet.linear_acceleration_covariance,
    ] {
        let covariance = covariance.unwrap();
        assert!(covariance.header.is_none());
        assert_eq!(covariance.data, vec![0.0f32; 9]);
    }
    assert_eq!(packet.orientation.unwrap().w, 1.0);
    assert_eq!(packet.angular_velocity.unwrap().y, 2.0);
    assert_eq!(packet.linear_acceleration.unwrap().z, 9.81);
    sample.time = Duration::MAX;
    assert!(imu_packet(&metadata, &sample, 0).is_none());
    sample.reading = Err(gz_sensors::InvalidImuInput);
    assert!(imu_packet(&metadata, &sample, 0).is_none());
}

#[test]
fn imu_publication_tracks_new_samples_epochs_and_rejected_payloads() {
    let mut server = imu_server(ServerOptions {
        max_message_bytes: 1,
        ..config()
    });
    server.update().unwrap();
    assert_eq!(
        server.counters().imus_queued + server.counters().imu_failures,
        0
    );
    assert!(server.apply_control(WorldControl {
        pause: true,
        multi_step: 3,
        ..Default::default()
    }));
    for _ in 0..3 {
        server.update().unwrap();
    }
    assert_eq!(server.counters().imu_failures, 3);
    assert_eq!(server.imus[0].sequence, 3);
    server.update().unwrap();
    assert_eq!(server.counters().imu_failures, 3);
    for _ in 0..2 {
        server.simulation.reset().unwrap();
        server.publish_imus();
        server.simulation.step_exact(1).unwrap();
        server.publish_imus();
    }
    assert_eq!(server.imus[0].sequence, 5);
    server.imus[0].sequence = u64::MAX;
    server.simulation.step_exact(1).unwrap();
    server.publish_imus();
    assert_eq!(server.imus[0].sequence, 0);
    let entity = server.imus[0].entity;
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(ImuSample {
            time: Duration::ZERO,
            epoch: 99,
            reading: Err(gz_sensors::InvalidImuInput),
        });
    server.publish_imus();
    server.publish_imus();
    assert_eq!(server.counters().invalid_imu_samples, 1);
    assert_eq!(server.imus[0].sequence, 0);
}

#[test]
fn valid_imu_samples_are_queued_and_invalid_topics_fail_setup() {
    let mut server = imu_server(config());
    assert!(server.apply_control(WorldControl {
        pause: true,
        step: true,
        ..Default::default()
    }));
    server.update().unwrap();
    assert_eq!(server.counters().imus_queued, 1);
    assert_eq!(server.counters().imu_failures, 0);
    let mut simulation = Simulation::new(Duration::from_millis(1)).unwrap();
    simulation.world_mut().spawn(ImuPublication {
        enable_orientation: true,
        entity_name: "test".into(),
        frame_id: "test".into(),
        topic: "~@:=//".into(),
    });
    assert!(matches!(
        isolated_server(simulation, "test", config(), NonZeroUsize::new(2).unwrap()),
        Err(Error::Setup(SetupError::InvalidName))
    ));
}

#[test]
fn pose_publication_reports_invalid_frames_and_bounded_transport_failure() {
    use gz_sim_scene::{EntityKind, LocalName};
    let mut malformed = Simulation::new(Duration::from_millis(1)).unwrap();
    malformed.world_mut().spawn(EntityKind::Model);
    assert!(matches!(
        isolated_server(malformed, "test", config(), NonZeroUsize::new(2).unwrap()),
        Err(Error::PoseData)
    ));
    let mut valid = imu_server(config());
    valid.update().unwrap();
    assert_eq!(valid.counters().poses_queued, 1);
    assert_eq!(valid.counters().pose_failures, 0);
    let entity = valid
        .simulation
        .world()
        .iter_entities()
        .find(|e| e.get::<EntityKind>() == Some(&EntityKind::Model))
        .unwrap()
        .id();
    valid
        .simulation
        .world_mut()
        .entity_mut(entity)
        .remove::<LocalName>();
    valid.update().unwrap();
    assert_eq!(valid.counters().invalid_pose_frames, 1);
    assert_eq!(valid.counters().poses_queued, 1);
    let mut bounded = imu_server(ServerOptions {
        max_message_bytes: 1,
        ..config()
    });
    bounded.update().unwrap();
    assert_eq!(bounded.counters().pose_failures, 1);
    assert_eq!(bounded.counters().poses_queued, 0);
}

fn public_id(server: &Server, name: &str) -> u64 {
    server
        .simulation()
        .world()
        .iter_entities()
        .find(|e| e.get::<gz_sim_scene::Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<gz_sim_core::EntityId>()
        .unwrap()
        .get()
}

fn removal_request(
    server: &Server,
    config: ServerOptions,
    blocking: bool,
    bytes: Vec<u8>,
) -> thread::JoinHandle<Result<(Vec<u8>, bool), String>> {
    let session = server.world_io.session.clone();
    let topic = if blocking {
        "/world/default/remove/blocking"
    } else {
        "/world/default/remove"
    };
    let key = crate::options::qualified_name(
        &config.partition,
        &config.namespace,
        config.topic_remap(topic).unwrap_or(topic),
    )
    .unwrap();
    thread::spawn(move || {
        let replies = session
            .get(key)
            .payload(bytes)
            .timeout(Duration::from_secs(5))
            .wait()
            .map_err(|error| error.to_string())?;
        let reply = replies
            .recv_timeout(Duration::from_secs(5))
            .map_err(|error| error.to_string())?
            .ok_or("removal reply timed out")?;
        let sample = reply.into_result().map_err(|error| format!("{error:?}"))?;
        Ok((sample.payload().to_bytes().into_owned(), true))
    })
}

fn poll_removals(server: &mut Server, count: u64) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while server.counters.removals_queued + server.counters.removals_rejected < count {
        assert!(Instant::now() < deadline);
        server
            .removals
            .poll(
                &server.world_io,
                server.command_budget,
                &mut server.counters,
            )
            .unwrap();
        thread::yield_now();
    }
}

fn removal_result(request: thread::JoinHandle<Result<(Vec<u8>, bool), String>>) -> bool {
    let (bytes, success) = request.join().unwrap().unwrap();
    assert!(success);
    Boolean::decode(bytes.as_slice()).unwrap().data
}

#[test]
fn queued_removal_preserves_final_publication_and_retires_sensor_streams() {
    use gz_sim_proto::msgs::{Entity, entity::Type};
    let config = config();
    let mut server = imu_server(config.clone());
    server.simulation.set_paused(false);
    server.update().unwrap();
    let ball = public_id(&server, "default::ball");
    let request = removal_request(
        &server,
        config,
        false,
        Entity {
            name: "default::ball".into(),
            r#type: Type::Model as i32,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    poll_removals(&mut server, 1);
    assert!(removal_result(request));
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_some());
    let samples = server.counters.imus_queued;
    let poses = server.counters.poses_queued;
    server.update().unwrap();
    assert_eq!(server.simulation.elapsed(), Duration::from_millis(2));
    assert_eq!(server.counters.imus_queued, samples + 1);
    assert_eq!(server.counters.poses_queued, poses + 1);
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_none());
    assert!(server.imus.is_empty());
    assert_eq!(server.counters.removals_completed, 1);
    server.update().unwrap();
    assert_eq!(server.counters.imus_queued, samples + 1);
    assert_eq!(
        poses::packet(server.simulation.world(), server.simulation.elapsed())
            .unwrap()
            .pose
            .len(),
        2
    );
}

#[test]
fn blocking_removal_reports_execution_and_duplicates_resolve_before_deletion() {
    use gz_sim_proto::msgs::{Entity, entity::Type};
    let config = config();
    let mut server = imu_server(config.clone());
    let ball = public_id(&server, "default::ball");
    let first = removal_request(
        &server,
        config.clone(),
        true,
        Entity {
            id: ball,
            name: "wrong".into(),
            r#type: Type::Link as i32,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    let second = removal_request(
        &server,
        config,
        true,
        Entity {
            name: "ball".into(),
            r#type: Type::Model as i32,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    poll_removals(&mut server, 2);
    assert!(!first.is_finished());
    assert!(!second.is_finished());
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_some());
    server.update().unwrap();
    assert!(removal_result(first));
    assert!(removal_result(second));
    assert_eq!(server.counters.removals_completed, 2);
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
    assert!(server.simulation.is_paused());
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_none());
}

#[test]
fn malformed_and_invalid_removal_requests_preserve_acceptance_semantics() {
    use gz_sim_proto::msgs::{Entity, entity::Type};
    let config = config();
    let mut server = imu_server(config.clone());
    let body = public_id(&server, "default::ball::body");
    let mut count = 0;
    for blocking in [false, true] {
        for bytes in [
            vec![0x80],
            Entity::default().encode_to_vec(),
            Entity {
                id: u64::MAX,
                ..Default::default()
            }
            .encode_to_vec(),
            Entity {
                id: body,
                ..Default::default()
            }
            .encode_to_vec(),
            Entity {
                name: "ball".into(),
                ..Default::default()
            }
            .encode_to_vec(),
            Entity {
                name: "ball".into(),
                r#type: Type::Link as i32,
                ..Default::default()
            }
            .encode_to_vec(),
            Entity {
                name: "missing".into(),
                r#type: Type::Model as i32,
                ..Default::default()
            }
            .encode_to_vec(),
        ] {
            let malformed = bytes == [0x80];
            let request = removal_request(&server, config.clone(), blocking, bytes);
            count += 1;
            poll_removals(&mut server, count);
            if !blocking || malformed {
                assert_eq!(removal_result(request), !malformed);
                server.update().unwrap();
            } else {
                assert!(!request.is_finished());
                server.update().unwrap();
                assert!(!removal_result(request));
            }
            assert!(gz_sim_core::entity_by_id(server.simulation.world(), body).is_some());
        }
    }
    assert_eq!(server.counters.removals_rejected, 2);
    assert_eq!(server.counters.removals_failed, 12);
    assert_eq!(server.counters.removals_completed, 0);
}

#[test]
fn removal_polling_is_bounded_and_shutdown_does_not_undo_accepted_execution() {
    use gz_sim_proto::msgs::Entity;
    let config = config();
    let mut server = imu_server(config.clone());
    server.command_budget = NonZeroUsize::new(1).unwrap();
    let ball = public_id(&server, "default::ball");
    let request = removal_request(
        &server,
        config,
        true,
        Entity {
            id: ball,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    poll_removals(&mut server, 1);
    for _ in 0..3 {
        server
            .removals
            .poll(
                &server.world_io,
                server.command_budget,
                &mut server.counters,
            )
            .unwrap();
    }
    assert_eq!(server.counters.removals_queued, 1);
    server.world_io.session.close().wait().unwrap();
    server.removals.execute(
        &server.world_io,
        &mut server.simulation,
        &mut server.counters,
    );
    assert_eq!(server.counters.removals_completed, 1);
    assert_eq!(server.counters.response_failures, 1);
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_none());
    server.shutdown();
    assert!(request.join().unwrap().is_err());
    assert!(
        server
            .removals
            .poll(
                &server.world_io,
                server.command_budget,
                &mut server.counters
            )
            .is_err()
    );
}

#[test]
fn each_removal_batch_is_bounded_when_both_services_have_work() {
    use gz_sim_proto::msgs::Entity;
    let config = config();
    let mut server = imu_server(config.clone());
    server.command_budget = NonZeroUsize::new(1).unwrap();
    let requests: Vec<_> = [false, true, false, true]
        .into_iter()
        .map(|blocking| {
            removal_request(
                &server,
                config.clone(),
                blocking,
                Entity {
                    id: u64::MAX,
                    ..Default::default()
                }
                .encode_to_vec(),
            )
        })
        .collect();
    for count in 1..=4 {
        poll_removals(&mut server, count);
        assert_eq!(server.counters.removals_queued, count);
        // Repeated ingress cannot exceed the one-command pending-batch limit.
        for _ in 0..3 {
            server
                .removals
                .poll(
                    &server.world_io,
                    server.command_budget,
                    &mut server.counters,
                )
                .unwrap();
        }
        assert_eq!(server.counters.removals_queued, count);
        server.removals.execute(
            &server.world_io,
            &mut server.simulation,
            &mut server.counters,
        );
        assert_eq!(server.counters.removals_failed, count);
    }
    for (request, blocking) in requests.into_iter().zip([false, true, false, true]) {
        assert_eq!(removal_result(request), !blocking);
    }
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
}

#[test]
fn client_timeout_does_not_cancel_an_accepted_removal() {
    use gz_sim_proto::msgs::Entity;
    let config = config();
    let mut server = imu_server(config.clone());
    let ball = public_id(&server, "default::ball");
    let session = server.world_io.session.clone();
    let topic = "/world/default/remove/blocking";
    let key = crate::options::qualified_name(
        &config.partition,
        &config.namespace,
        config.topic_remap(topic).unwrap_or(topic),
    )
    .unwrap();
    let request = thread::spawn(move || {
        let replies = session
            .get(key)
            .payload(
                Entity {
                    id: ball,
                    ..Default::default()
                }
                .encode_to_vec(),
            )
            .timeout(Duration::from_secs(5))
            .wait()
            .unwrap();
        // Keep the original two-second application receive window. The query
        // stays bounded by the normal five-second IO limit after the caller leaves.
        assert!(matches!(
            replies.recv_timeout(Duration::from_secs(2)),
            Ok(None)
        ));
    });
    poll_removals(&mut server, 1);
    request.join().unwrap();
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_some());
    server.update().unwrap();
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_none());
    assert_eq!(server.counters.removals_completed, 1);
    assert!(server.imus.is_empty());
}

#[test]
fn shared_imu_topic_preserves_each_source_and_retires_only_the_last_publisher() {
    use gz_sim_proto::msgs::Imu;
    use std::collections::BTreeSet;
    let config = ServerOptions {
        namespace: "/robot".into(),
        ..config()
    };
    let xml = include_str!("../../../gz-sim-scene/tests/fixtures/fall.sdf")
        .replace("<always_on>true</always_on>", "<always_on>true</always_on><topic>shared</topic><frame_id>first</frame_id>")
        .replace("</sensor>", "</sensor><sensor name='imu2' type='imu'><always_on>true</always_on><update_rate>1000</update_rate><topic>/robot/shared</topic><frame_id>second</frame_id></sensor>");
    let sim = gz_sim_scene::load_world(
        &xml,
        "shared.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let mut server = isolated_server(
        sim,
        "default",
        config.clone(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    assert_eq!(server.imus.len(), 2);
    assert!(Arc::ptr_eq(
        &server.imus[0].publisher,
        &server.imus[1].publisher
    ));
    let endpoint = Arc::downgrade(&server.imus[0].publisher);
    let subscriber = samples(&server, "/robot/shared", "gz.msgs.IMU");
    server.simulation.set_paused(false);
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut sources = BTreeSet::new();
    while sources.len() < 2 {
        assert!(Instant::now() < deadline);
        server.update().unwrap();
        if let Ok(message) = subscriber.recv_timeout(Duration::from_millis(2)) {
            let imu = Imu::decode(message.payload().to_bytes().as_ref()).unwrap();
            let frame = &imu.header.as_ref().unwrap().data[0].value[0];
            assert!(matches!(
                (imu.entity_name.as_str(), frame.as_str()),
                ("ball::body::imu", "first") | ("ball::body::imu2", "second")
            ));
            sources.insert(imu.entity_name);
        }
    }
    assert!(server.imus.iter().all(|stream| stream.sequence > 0));
    let first = server
        .imus
        .iter()
        .find(|stream| stream.metadata.frame_id == "first")
        .unwrap()
        .entity;
    let removed_at = server.simulation.elapsed();
    server.simulation.world_mut().entity_mut(first).despawn();
    server.update().unwrap();
    assert_eq!(server.imus.len(), 1);
    assert_eq!(endpoint.strong_count(), 1);
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        assert!(Instant::now() < deadline);
        server.update().unwrap();
        if let Ok(message) = subscriber.recv_timeout(Duration::from_millis(2)) {
            let imu = Imu::decode(message.payload().to_bytes().as_ref()).unwrap();
            let stamp = imu.header.as_ref().unwrap().stamp.as_ref().unwrap();
            let time = Duration::new(
                stamp.sec.try_into().unwrap(),
                stamp.nsec.try_into().unwrap(),
            );
            if time > removed_at {
                assert_eq!(imu.entity_name, "ball::body::imu2");
                assert_eq!(imu.header.as_ref().unwrap().data[0].value, ["second"]);
                break;
            }
        }
    }
    let ball = public_id(&server, "default::ball");
    assert!(server.imus.capacity() > 0);
    gz_sim_scene::remove_model(&mut server.simulation, ball).unwrap();
    server.update().unwrap();
    assert!(server.imus.is_empty());
    assert_eq!(server.imus.capacity(), 0);
    assert!(endpoint.upgrade().is_none());
    assert_eq!(server.counters.imu_failures, 0);
}

#[path = "reset.rs"]
mod reset_tests;

#[test]
fn native_default_pose_period_does_not_pace_clock_imu_or_reset_the_wall_gate() {
    let mut default = isolated_server(
        Simulation::new(Duration::from_millis(1)).unwrap(),
        "test",
        config(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    assert_eq!(
        default.pose_publication_period(),
        Duration::from_nanos(16_666_667)
    );
    default.shutdown();

    let mut server = imu_server(config());
    // A long explicit period makes the integration boundary independent of host scheduling.
    server.set_pose_publication_period(Duration::from_secs(86_400));
    let poses = samples(&server, "/world/default/pose/info", "gz.msgs.Pose_V");
    let clocks = samples(&server, "/world/default/clock", "gz.msgs.Clock");
    let imus = samples(&server, &server.imus[0].metadata.topic, "gz.msgs.IMU");
    assert!(server.apply_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    for step in 1..=3 {
        assert_eq!(
            server.update().unwrap().sim_time,
            Duration::from_millis(step)
        );
        let message = clocks.recv_timeout(Duration::from_secs(5)).unwrap();
        assert_eq!(
            Clock::decode(message.payload().to_bytes().as_ref())
                .unwrap()
                .sim
                .unwrap()
                .nsec,
            step as i32 * 1_000_000
        );
        let message = imus.recv_timeout(Duration::from_secs(5)).unwrap();
        assert_eq!(
            gz_sim_proto::msgs::Imu::decode(message.payload().to_bytes().as_ref())
                .unwrap()
                .header
                .unwrap()
                .stamp
                .unwrap()
                .nsec,
            step as i32 * 1_000_000
        );
    }
    let message = poses.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(
        gz_sim_proto::msgs::PoseV::decode(message.payload().to_bytes().as_ref())
            .unwrap()
            .header
            .unwrap()
            .stamp
            .unwrap()
            .nsec,
        1_000_000
    );
    assert!(matches!(
        poses.recv_timeout(Duration::from_millis(1)),
        Err(mpsc::RecvTimeoutError::Timeout)
    ));
    assert_eq!(server.counters().poses_queued, 1);
    assert_eq!(server.counters().poses_throttled, 2);
    assert_eq!(server.counters().pose_failures, 0);
    assert!(server.apply_control(WorldControl {
        pause: true,
        reset: Some(gz_sim_proto::msgs::WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert_eq!(server.update().unwrap().sim_time, Duration::ZERO);
    assert_eq!(server.counters().poses_queued, 1);
    assert_eq!(server.counters().poses_throttled, 3);
    server.set_pose_publication_period(Duration::ZERO);
    server.update().unwrap();
    let message = poses.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(
        gz_sim_proto::msgs::PoseV::decode(message.payload().to_bytes().as_ref())
            .unwrap()
            .header
            .unwrap()
            .stamp
            .unwrap()
            .nsec,
        0
    );
    assert_eq!(server.counters().poses_queued, 2);
    assert_eq!(server.pose_publication_period(), Duration::ZERO);
}

#[test]
fn failed_pose_enqueue_consumes_its_wall_publication_slot() {
    let mut config = config();
    config.max_message_bytes = 1;
    let mut server = imu_server(config);
    server.set_pose_publication_period(Duration::from_secs(86_400));
    server.update().unwrap();
    server.update().unwrap();
    assert_eq!(server.counters().pose_failures, 1);
    assert_eq!(server.counters().poses_throttled, 1);
    assert_eq!(server.counters().poses_queued, 0);
}

#[test]
fn invalid_pose_packet_does_not_consume_the_next_publication_slot() {
    use gz_sim_scene::{EntityKind, LocalName};
    let mut server = imu_server(config());
    server.set_pose_publication_period(Duration::from_secs(86_400));
    let entity = server
        .simulation
        .world()
        .iter_entities()
        .find(|e| e.get::<EntityKind>() == Some(&EntityKind::Model))
        .unwrap()
        .id();
    let name = server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .take::<LocalName>()
        .unwrap();
    server.update().unwrap();
    assert_eq!(server.counters().invalid_pose_frames, 1);
    assert_eq!(server.counters().poses_queued, 0);
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(name);
    server.update().unwrap();
    assert_eq!(server.counters().poses_queued, 1);
    assert_eq!(server.counters().poses_throttled, 0);
}

#[test]
fn noisy_covariance_uses_native_float_diagonals_and_zero_cross_terms() {
    let metadata = ImuPublication {
        enable_orientation: true,
        entity_name: "robot::imu".into(),
        frame_id: "imu".into(),
        topic: "/imu".into(),
    };
    let mut reading = gz_sensors::imu(gz_sensors::ImuInput {
        world_orientation: [0.0, 0.0, 0.0, 1.0],
        reference_orientation: [0.0, 0.0, 0.0, 1.0],
        acceleration_sensor: [0.0; 3],
        angular_velocity_sensor: [0.0; 3],
        gravity_world: [0.0, 0.0, -9.81],
    })
    .unwrap();
    reading.angular_velocity_covariance = [0.01, 0.04, 0.09];
    reading.linear_acceleration_covariance = [0.16, 0.25, 0.36];
    let sample = ImuSample {
        time: Duration::from_millis(1),
        epoch: 0,
        reading: Ok(reading),
    };
    let packet = imu_packet(&metadata, &sample, 0).unwrap();
    assert_eq!(
        packet.angular_velocity_covariance.unwrap().data,
        [0.01f32, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.09]
    );
    assert_eq!(
        packet.linear_acceleration_covariance.unwrap().data,
        [0.16f32, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.36]
    );
    assert_eq!(packet.orientation_covariance.unwrap().data, [0.0; 9]);
    assert_eq!(packet.linear_acceleration.unwrap().z, 9.81);
}

#[path = "interest.rs"]
mod interest_tests;

#[test]
fn disabled_orientation_omits_only_the_quaternion_field() {
    let mut metadata = ImuPublication {
        enable_orientation: true,
        entity_name: "robot::imu".into(),
        frame_id: "imu_frame".into(),
        topic: "/imu".into(),
    };
    let sample = ImuSample {
        time: Duration::from_nanos(1_234_567_890),
        epoch: 7,
        reading: gz_sensors::imu(gz_sensors::ImuInput {
            world_orientation: [0.0, 0.0, 0.0, 1.0],
            reference_orientation: [0.0, 0.0, 0.0, 1.0],
            acceleration_sensor: [1.0, 2.0, 3.0],
            angular_velocity_sensor: [4.0, 5.0, 6.0],
            gravity_world: [0.0, 0.0, -9.81],
        }),
    };
    let mut expected = imu_packet(&metadata, &sample, 42).unwrap();
    assert!(expected.orientation.take().is_some());
    metadata.enable_orientation = false;
    let actual = imu_packet(&metadata, &sample, 42).unwrap();
    assert!(actual.orientation.is_none());
    assert_eq!(
        actual.orientation_covariance.as_ref().unwrap().data,
        [0.0; 9]
    );
    assert_eq!(actual, expected);
}

#[path = "runtime_rates.rs"]
mod runtime_rate_tests;

#[path = "activation.rs"]
mod activation_tests;

#[path = "root_clock.rs"]
mod root_clock_tests;

#[test]
fn finishing_joins_workers_and_returns_the_final_simulation() {
    let config = config();
    let mut server = server(config, Duration::from_millis(1));
    server.simulation.set_paused(false);
    server.update().unwrap();
    let simulation = server.into_simulation();
    assert_eq!(simulation.elapsed(), Duration::from_millis(1));
    assert!(!simulation.is_paused());
    simulation.validate_steps(1).unwrap();
}

#[path = "native_forces.rs"]
mod native_force_tests;

#[test]
fn control_response_obeys_exact_size_limit() {
    for (limit, succeeds) in [(2, true), (1, false)] {
        let mut settings = config();
        settings.max_message_bytes = limit;
        let mut server = server(settings, Duration::from_millis(1));
        let response = request(&server, WorldControl::default().encode_to_vec());
        let deadline = Instant::now() + Duration::from_secs(5);
        while !response.is_finished() {
            assert!(Instant::now() < deadline, "control reply did not finish");
            server.update().unwrap();
            thread::yield_now();
        }
        let result = response.join().unwrap();
        assert_eq!(server.counters().controls_accepted, 1);
        assert_eq!(server.counters().controls_rejected, 0);
        assert_eq!(server.counters().response_failures, u64::from(!succeeds));
        if succeeds {
            let (bytes, success) = result.unwrap();
            assert!(success);
            assert_eq!(bytes.len(), 2);
            assert!(Boolean::decode(bytes.as_slice()).unwrap().data);
        } else {
            assert!(result.is_err(), "oversized Boolean must not be delivered");
        }
        server.shutdown();
    }
}

#[test]
fn remaps_cannot_mix_world_publisher_message_types() {
    let mut settings = config();
    settings
        .add_topic_remap("/world/test/pose/info", "/world/test/clock")
        .unwrap();
    assert!(matches!(
        isolated_server(Simulation::new(Duration::from_millis(1)).unwrap(), "test", settings,
                    NonZeroUsize::new(2).unwrap()),
        Err(Error::WorldNetwork(message)) if message.contains("publisher keys collide")
    ));
    for topic in ["/world/default/clock", "/world/default/pose/info"] {
        let mut settings = config();
        settings.add_topic_remap("/p2/imu", topic).unwrap();
        let simulation = gz_sim_scene::load_world_with_sdf_step(
            include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
            "headless_imu.sdf",
            "default",
            &sdformat::NoResources,
            gz_sim_scene::PhysicsEngine::Tpe,
        )
        .unwrap();
        assert!(matches!(
            isolated_server(simulation, "default", settings, NonZeroUsize::new(2).unwrap()),
            Err(Error::WorldNetwork(message)) if message.contains("IMU key collides")
        ));
    }
}

#[path = "terminal_admission.rs"]
mod terminal_admission;
