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
