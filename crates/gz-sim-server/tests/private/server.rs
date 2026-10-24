// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{
    net::UdpSocket,
    sync::atomic::{AtomicU64, Ordering},
    thread,
};

static ID: AtomicU64 = AtomicU64::new(0);

fn config() -> Config {
    let message = UdpSocket::bind("127.0.0.1:0").unwrap();
    let service = UdpSocket::bind("127.0.0.1:0").unwrap();
    Config {
        partition: format!(
            "sim-{}-{}",
            std::process::id(),
            ID.fetch_add(1, Ordering::Relaxed)
        ),
        message_port: message.local_addr().unwrap().port(),
        service_port: service.local_addr().unwrap().port(),
        ..Config::default()
    }
}

fn server(config: Config, step: Duration) -> Server {
    Server::new(
        Simulation::new(step).unwrap(),
        "test",
        config,
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap()
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
    assert_eq!(server.tick().unwrap().sim_time, Duration::ZERO);
    assert!(server.apply_control(WorldControl {
        pause: true,
        step: true,
        multi_step: 3,
        ..Default::default()
    }));
    assert_eq!(server.pending_steps(), 3);
    for step in 1..=3 {
        assert_eq!(server.tick().unwrap().sim_time, Duration::from_millis(step));
    }
    assert!(server.simulation().is_paused());
    assert_eq!(server.pending_steps(), 0);
    assert_eq!(server.tick().unwrap().sim_time, Duration::from_millis(3));
    assert!(server.apply_control(WorldControl {
        pause: false,
        multi_step: 17,
        ..Default::default()
    }));
    server.tick().unwrap();
    assert_eq!(server.pending_steps(), 0);
    assert!(!server.simulation().is_paused());
    assert!(server.apply_control(WorldControl {
        pause: true,
        step: true,
        ..Default::default()
    }));
    server.tick().unwrap();
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
    assert_eq!(server.tick().unwrap().sim_time, Duration::from_millis(5));
    assert_eq!(server.pending_steps(), 5);
    assert!(server.apply_control(WorldControl::default()));
    for _ in 0..5 {
        server.tick().unwrap();
    }
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(10));
    assert!(server.simulation().is_paused());
    assert!(server.counters().clocks_queued > 0);
    assert_eq!(server.transport_statistics().send_failures, 0);
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
            reset: Some(gz_msgs::msgs::WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        },
        WorldControl {
            reset: Some(gz_msgs::msgs::WorldReset {
                time_only: true,
                ..Default::default()
            }),
            ..Default::default()
        },
        WorldControl {
            reset: Some(gz_msgs::msgs::WorldReset {
                model_only: true,
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
    assert!(matches!(huge.tick(), Err(Error::TimeRange)));
    assert_eq!(huge.simulation().elapsed(), Duration::ZERO);
    huge.simulation.step_exact(1).unwrap();
    assert!(matches!(huge.tick(), Err(Error::Step(StepError::Overflow))));
    assert!(
        Error::Step(StepError::Overflow)
            .to_string()
            .contains("overflow")
    );
    assert!(
        Error::Transport(TransportError::Shutdown)
            .to_string()
            .contains("Shutdown")
    );
}

#[test]
fn setup_rejects_invalid_names_configuration_and_unrepresentable_time() {
    for name in ["", "bad/world", "bad world"] {
        assert!(
            Server::new(
                Simulation::new(Duration::from_millis(1)).unwrap(),
                name,
                config(),
                NonZeroUsize::new(1).unwrap()
            )
            .is_err()
        );
    }
    for config in [
        Config {
            capacity: 0,
            ..config()
        },
        Config {
            capacity: 1,
            ..config()
        },
    ] {
        assert!(
            Server::new(
                Simulation::new(Duration::from_millis(1)).unwrap(),
                "test",
                config,
                NonZeroUsize::new(1).unwrap()
            )
            .is_err()
        );
    }
    let mut huge = Simulation::new(Duration::MAX).unwrap();
    huge.step_exact(1).unwrap();
    assert!(matches!(
        Server::new(huge, "test", config(), NonZeroUsize::new(1).unwrap()),
        Err(Error::TimeRange)
    ));
}

fn request(
    client: Node,
    bytes: Vec<u8>,
) -> thread::JoinHandle<Result<(Vec<u8>, bool), TransportError>> {
    thread::spawn(move || {
        client.request(
            "/world/test/control",
            "gz.msgs.WorldControl",
            "gz.msgs.Boolean",
            bytes,
            Duration::from_secs(5),
        )
    })
}

#[test]
fn real_transport_commands_report_acceptance_and_exact_step_completion() {
    let config = config();
    let mut server = server(config.clone(), Duration::from_millis(1));
    let client = Node::new(config.clone()).unwrap();
    let subscription = client
        .subscribe("/world/test/clock", "gz.msgs.Clock")
        .unwrap();
    let mut first = None;
    let deadline = Instant::now() + Duration::from_secs(5);
    while first.is_none() && Instant::now() < deadline {
        server.tick().unwrap();
        if let Ok(message) = subscription.recv_timeout(Duration::from_millis(2)) {
            first = Some(Clock::decode(message.data.as_slice()).unwrap());
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
        let result = request(Node::new(config.clone()).unwrap(), bytes);
        let deadline = Instant::now() + Duration::from_secs(5);
        while !result.is_finished() && Instant::now() < deadline {
            server.tick().unwrap();
            thread::yield_now();
        }
        let (bytes, success) = result.join().unwrap().unwrap();
        assert!(success);
        assert_eq!(Boolean::decode(bytes.as_slice()).unwrap().data, accepted);
    }
    while server.pending_steps() > 0 {
        server.tick().unwrap();
    }
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(3));
    let deadline = Instant::now() + Duration::from_secs(5);
    let final_clock = loop {
        assert!(Instant::now() < deadline);
        server.tick().unwrap();
        if let Ok(message) = subscription.recv_timeout(Duration::from_millis(2)) {
            let clock = Clock::decode(message.data.as_slice()).unwrap();
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
    assert!(matches!(
        server.tick(),
        Err(Error::Transport(TransportError::Shutdown))
    ));
    assert!(server.counters().clock_failures > 0);
}

#[test]
fn shutdown_after_receipt_reports_reply_failure_without_waiting() {
    let config = config();
    let mut server = server(config.clone(), Duration::from_millis(1));
    let result = request(
        Node::new(config).unwrap(),
        WorldControl::default().encode_to_vec(),
    );
    let received = server
        .controls
        .recv_timeout(Duration::from_secs(5))
        .unwrap();
    server.node.cancel();
    server.respond(received);
    assert_eq!(server.counters().response_failures, 1);
    assert_eq!(server.counters().controls_accepted, 1);
    server.shutdown();
    assert!(result.join().unwrap().is_err());
}

fn imu_server(config: Config) -> Server {
    let simulation = gz_sim_scene::load_world(
        include_str!("../../../gz-sim-scene/tests/fixtures/fall.sdf"),
        "fall.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    Server::new(simulation, "default", config, NonZeroUsize::new(2).unwrap()).unwrap()
}

#[test]
fn imu_message_matches_native_metadata_presence_covariance_and_sequence() {
    let metadata = ImuPublication {
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
    let mut server = imu_server(Config {
        max_message_bytes: 1,
        ..config()
    });
    server.tick().unwrap();
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
        server.tick().unwrap();
    }
    assert_eq!(server.counters().imu_failures, 3);
    assert_eq!(server.imus[0].sequence, 3);
    server.tick().unwrap();
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
    server.tick().unwrap();
    assert_eq!(server.counters().imus_queued, 1);
    assert_eq!(server.counters().imu_failures, 0);
    let mut simulation = Simulation::new(Duration::from_millis(1)).unwrap();
    simulation.world_mut().spawn(ImuPublication {
        entity_name: "test".into(),
        frame_id: "test".into(),
        topic: "~@:=//".into(),
    });
    assert!(matches!(
        Server::new(simulation, "test", config(), NonZeroUsize::new(2).unwrap()),
        Err(Error::Transport(TransportError::InvalidName))
    ));
}

#[test]
fn pose_publication_reports_invalid_frames_and_bounded_transport_failure() {
    use gz_sim_scene::{EntityKind, LocalName};
    let mut malformed = Simulation::new(Duration::from_millis(1)).unwrap();
    malformed.world_mut().spawn(EntityKind::Model);
    assert!(matches!(
        Server::new(malformed, "test", config(), NonZeroUsize::new(2).unwrap()),
        Err(Error::PoseData)
    ));
    let mut valid = imu_server(config());
    valid.tick().unwrap();
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
    valid.tick().unwrap();
    assert_eq!(valid.counters().invalid_pose_frames, 1);
    assert_eq!(valid.counters().poses_queued, 1);
    let mut bounded = imu_server(Config {
        max_message_bytes: 1,
        ..config()
    });
    bounded.tick().unwrap();
    assert_eq!(bounded.counters().pose_failures, 1);
    assert_eq!(bounded.counters().poses_queued, 0);
}
