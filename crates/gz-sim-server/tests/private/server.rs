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
    let mut server = Server::new(
        Simulation::new(step).unwrap(),
        "test",
        config,
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    server.set_pose_publication_period(Duration::ZERO);
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
    let mut server =
        Server::new(simulation, "default", config, NonZeroUsize::new(2).unwrap()).unwrap();
    server.set_pose_publication_period(Duration::ZERO);
    server
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
    config: Config,
    blocking: bool,
    bytes: Vec<u8>,
) -> thread::JoinHandle<Result<(Vec<u8>, bool), TransportError>> {
    thread::spawn(move || {
        Node::new(config).unwrap().request(
            if blocking {
                "/world/default/remove/blocking"
            } else {
                "/world/default/remove"
            },
            "gz.msgs.Entity",
            "gz.msgs.Boolean",
            bytes,
            Duration::from_secs(5),
        )
    })
}

fn poll_removals(server: &mut Server, count: u64) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while server.counters.removals_queued + server.counters.removals_rejected < count {
        assert!(Instant::now() < deadline);
        server
            .removals
            .poll(server.command_budget, &mut server.counters)
            .unwrap();
        thread::yield_now();
    }
}

fn removal_result(request: thread::JoinHandle<Result<(Vec<u8>, bool), TransportError>>) -> bool {
    let (bytes, success) = request.join().unwrap().unwrap();
    assert!(success);
    Boolean::decode(bytes.as_slice()).unwrap().data
}

#[test]
fn queued_removal_preserves_final_publication_and_retires_sensor_streams() {
    use gz_msgs::msgs::{Entity, entity::Type};
    let config = config();
    let mut server = imu_server(config.clone());
    server.simulation.set_paused(false);
    server.tick().unwrap();
    let ball = public_id(&server, "default::ball");
    let request = removal_request(
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
    server.tick().unwrap();
    assert_eq!(server.simulation.elapsed(), Duration::from_millis(2));
    assert_eq!(server.counters.imus_queued, samples + 1);
    assert_eq!(server.counters.poses_queued, poses + 1);
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_none());
    assert!(server.imus.is_empty());
    assert_eq!(server.counters.removals_completed, 1);
    server.tick().unwrap();
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
    use gz_msgs::msgs::{Entity, entity::Type};
    let config = config();
    let mut server = imu_server(config.clone());
    let ball = public_id(&server, "default::ball");
    let first = removal_request(
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
    server.tick().unwrap();
    assert!(removal_result(first));
    assert!(removal_result(second));
    assert_eq!(server.counters.removals_completed, 2);
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
    assert!(server.simulation.is_paused());
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_none());
}

#[test]
fn malformed_and_invalid_removal_requests_preserve_acceptance_semantics() {
    use gz_msgs::msgs::{Entity, entity::Type};
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
            let request = removal_request(config.clone(), blocking, bytes);
            count += 1;
            poll_removals(&mut server, count);
            if !blocking || malformed {
                assert_eq!(removal_result(request), !malformed);
                server.tick().unwrap();
            } else {
                assert!(!request.is_finished());
                server.tick().unwrap();
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
    use gz_msgs::msgs::Entity;
    let config = config();
    let mut server = imu_server(config.clone());
    server.command_budget = NonZeroUsize::new(1).unwrap();
    let ball = public_id(&server, "default::ball");
    let request = removal_request(
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
            .poll(server.command_budget, &mut server.counters)
            .unwrap();
    }
    assert_eq!(server.counters.removals_queued, 1);
    server.node.cancel();
    server
        .removals
        .execute(&mut server.simulation, &mut server.counters);
    assert_eq!(server.counters.removals_completed, 1);
    assert_eq!(server.counters.response_failures, 1);
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_none());
    server.shutdown();
    assert!(request.join().unwrap().is_err());
    assert!(
        server
            .removals
            .poll(server.command_budget, &mut server.counters)
            .is_err()
    );
}

#[test]
fn each_removal_batch_is_bounded_when_both_services_have_work() {
    use gz_msgs::msgs::Entity;
    let config = config();
    let mut server = imu_server(config.clone());
    server.command_budget = NonZeroUsize::new(1).unwrap();
    let requests: Vec<_> = [false, true, false, true]
        .into_iter()
        .map(|blocking| {
            removal_request(
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
                .poll(server.command_budget, &mut server.counters)
                .unwrap();
        }
        assert_eq!(server.counters.removals_queued, count);
        server
            .removals
            .execute(&mut server.simulation, &mut server.counters);
        assert_eq!(server.counters.removals_failed, count);
    }
    for (request, blocking) in requests.into_iter().zip([false, true, false, true]) {
        assert_eq!(removal_result(request), !blocking);
    }
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
}

#[test]
fn client_timeout_does_not_cancel_an_accepted_removal() {
    use gz_msgs::msgs::Entity;
    let config = config();
    let mut server = imu_server(config.clone());
    let ball = public_id(&server, "default::ball");
    let request = thread::spawn(move || {
        Node::new(config).unwrap().request(
            "/world/default/remove/blocking",
            "gz.msgs.Entity",
            "gz.msgs.Boolean",
            Entity {
                id: ball,
                ..Default::default()
            }
            .encode_to_vec(),
            Duration::from_secs(2),
        )
    });
    poll_removals(&mut server, 1);
    assert!(matches!(
        request.join().unwrap(),
        Err(TransportError::Timeout)
    ));
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_some());
    server.tick().unwrap();
    assert!(gz_sim_core::entity_by_id(server.simulation.world(), ball).is_none());
    assert_eq!(server.counters.removals_completed, 1);
    assert!(server.imus.is_empty());
}

#[test]
fn shared_imu_topic_preserves_each_source_and_retires_only_the_last_publisher() {
    use gz_msgs::msgs::Imu;
    use std::collections::BTreeSet;
    let config = Config {
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
    let mut server = Server::new(
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
    let client = Node::new(config).unwrap();
    let subscriber = client.subscribe("/robot/shared", "gz.msgs.IMU").unwrap();
    server.simulation.set_paused(false);
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut sources = BTreeSet::new();
    while sources.len() < 2 {
        assert!(Instant::now() < deadline);
        server.tick().unwrap();
        if let Ok(message) = subscriber.recv_timeout(Duration::from_millis(2)) {
            let imu = Imu::decode(message.data.as_slice()).unwrap();
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
    server.tick().unwrap();
    assert_eq!(server.imus.len(), 1);
    assert_eq!(endpoint.strong_count(), 1);
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        assert!(Instant::now() < deadline);
        server.tick().unwrap();
        if let Ok(message) = subscriber.recv_timeout(Duration::from_millis(2)) {
            let imu = Imu::decode(message.data.as_slice()).unwrap();
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
    gz_sim_scene::remove_model(&mut server.simulation, ball).unwrap();
    server.tick().unwrap();
    assert!(server.imus.is_empty());
    assert!(endpoint.upgrade().is_none());
    assert_eq!(server.counters.imu_failures, 0);
}

#[path = "reset.rs"]
mod reset_tests;

#[test]
fn native_default_pose_period_does_not_pace_clock_imu_or_reset_the_wall_gate() {
    let mut default = Server::new(
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
    let poses = server
        .node
        .subscribe("/world/default/pose/info", "gz.msgs.Pose_V")
        .unwrap();
    let clocks = server
        .node
        .subscribe("/world/default/clock", "gz.msgs.Clock")
        .unwrap();
    let imus = server
        .node
        .subscribe(&server.imus[0].metadata.topic, "gz.msgs.IMU")
        .unwrap();
    assert!(server.apply_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    for step in 1..=3 {
        assert_eq!(server.tick().unwrap().sim_time, Duration::from_millis(step));
        let message = clocks.recv_timeout(Duration::from_secs(5)).unwrap();
        assert_eq!(
            Clock::decode(message.data.as_slice())
                .unwrap()
                .sim
                .unwrap()
                .nsec,
            step as i32 * 1_000_000
        );
        let message = imus.recv_timeout(Duration::from_secs(5)).unwrap();
        assert_eq!(
            gz_msgs::msgs::Imu::decode(message.data.as_slice())
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
        gz_msgs::msgs::PoseV::decode(message.data.as_slice())
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
        Err(TransportError::Timeout)
    ));
    assert_eq!(server.counters().poses_queued, 1);
    assert_eq!(server.counters().poses_throttled, 2);
    assert_eq!(server.counters().pose_failures, 0);
    assert!(server.apply_control(WorldControl {
        pause: true,
        reset: Some(gz_msgs::msgs::WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert_eq!(server.tick().unwrap().sim_time, Duration::ZERO);
    assert_eq!(server.counters().poses_queued, 1);
    assert_eq!(server.counters().poses_throttled, 3);
    server.set_pose_publication_period(Duration::ZERO);
    server.tick().unwrap();
    let message = poses.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(
        gz_msgs::msgs::PoseV::decode(message.data.as_slice())
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
    server.tick().unwrap();
    server.tick().unwrap();
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
    server.tick().unwrap();
    assert_eq!(server.counters().invalid_pose_frames, 1);
    assert_eq!(server.counters().poses_queued, 0);
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(name);
    server.tick().unwrap();
    assert_eq!(server.counters().poses_queued, 1);
    assert_eq!(server.counters().poses_throttled, 0);
}

#[test]
fn noisy_covariance_uses_native_float_diagonals_and_zero_cross_terms() {
    let metadata = ImuPublication {
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
