// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn running_root_server(config: Config, name: &str, step: Duration) -> Server {
    let mut server = Server::new(
        Simulation::new(step).unwrap(),
        name,
        config,
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    assert!(server.apply_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    server
}

fn matching_clocks(
    server: &mut Server,
    root: &gz_transport::Subscription,
    world: &gz_transport::Subscription,
) -> Clock {
    let info = server.tick().unwrap();
    let world = world.recv_timeout(Duration::from_secs(5)).unwrap();
    let root = root.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(root.data, world.data);
    let clock = Clock::decode(root.data.as_slice()).unwrap();
    assert_eq!(
        clock.sim,
        Some(time(info.sim_time.as_nanos() as i128).unwrap())
    );
    clock
}

#[test]
fn same_process_root_reuses_complete_clock_bytes_and_survives_reset() {
    let configuration = config();
    let observer = Node::new(configuration.clone()).unwrap();
    let root = observer.subscribe("/clock", "gz.msgs.Clock").unwrap();
    let world = observer
        .subscribe("/world/test/clock", "gz.msgs.Clock")
        .unwrap();
    // A loaded scene installs the reset handler exercised below.
    let simulation = gz_sim_scene::load_world(
        "<sdf version='1.12'><world name='test'/></sdf>",
        "root-clock.sdf",
        "test",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let mut server = Server::new(
        simulation,
        "test",
        configuration,
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    assert!(server.apply_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
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
        reset: Some(gz_msgs::msgs::WorldReset {
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
        observer.known_topic_info("/clock").unwrap().publishers,
        ["gz.msgs.Clock"]
    );
    assert_eq!(server.counters().clocks_queued, 3);
    assert_eq!(server.counters().root_clocks_queued, 3);
    assert_eq!(server.counters().root_clock_failures, 0);
}

#[test]
fn any_known_publisher_type_blocks_root_until_the_next_prepared_run() {
    for type_name in ["gz.msgs.Clock", "gz.msgs.StringMsg"] {
        let configuration = config();
        let owner = Node::new(configuration.clone()).unwrap();
        let foreign = owner.advertise("/clock", type_name).unwrap();
        let observer = Node::new(configuration.clone()).unwrap();
        let root = observer.subscribe("/clock", "gz.msgs.Clock").unwrap();
        let world = observer
            .subscribe("/world/test/clock", "gz.msgs.Clock")
            .unwrap();
        let mut server = running_root_server(configuration, "test", Duration::from_millis(1));
        assert!(!server.prepare_run(Duration::from_secs(5)).unwrap());
        for expected in [1_000_000, 2_000_000] {
            server.tick().unwrap();
            let packet = world.recv_timeout(Duration::from_secs(5)).unwrap();
            assert_eq!(
                Clock::decode(packet.data.as_slice())
                    .unwrap()
                    .sim
                    .unwrap()
                    .nsec,
                expected
            );
        }
        assert_eq!(
            observer.known_topic_info("/clock").unwrap().publishers,
            [type_name]
        );
        assert!(matches!(
            root.recv_timeout(Duration::from_millis(5)),
            Err(TransportError::Timeout)
        ));
        assert_eq!(server.counters().root_clocks_queued, 0);
        drop(foreign);
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
    let configuration = config();
    let observer = Node::new(configuration.clone()).unwrap();
    let root = observer.subscribe("/clock", "gz.msgs.Clock").unwrap();
    let world = observer
        .subscribe("/world/test/clock", "gz.msgs.Clock")
        .unwrap();
    let other = observer
        .subscribe("/world/other/clock", "gz.msgs.Clock")
        .unwrap();
    let mut first = running_root_server(configuration.clone(), "test", Duration::from_millis(1));
    assert!(first.prepare_run(Duration::from_secs(5)).unwrap());
    matching_clocks(&mut first, &root, &world);
    let competitor = observer.advertise("/clock", "gz.msgs.Clock").unwrap();
    assert!(first.prepare_run(Duration::from_secs(5)).unwrap());
    let mut second = running_root_server(configuration, "other", Duration::from_millis(2));
    assert!(!second.prepare_run(Duration::from_secs(5)).unwrap());
    second.tick().unwrap();
    let packet = other.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(
        Clock::decode(packet.data.as_slice())
            .unwrap()
            .sim
            .unwrap()
            .nsec,
        2_000_000
    );
    assert!(matches!(
        root.recv_timeout(Duration::from_millis(5)),
        Err(TransportError::Timeout)
    ));
    matching_clocks(&mut first, &root, &world);
    assert_eq!(
        observer
            .known_topic_info("/clock")
            .unwrap()
            .publishers
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
    let configuration = config();
    let observer = Node::new(configuration.clone()).unwrap();
    let mut server = running_root_server(configuration, "test", Duration::from_millis(1));
    for (timeout, expected) in [
        (Duration::ZERO, gz_transport::Error::Timeout),
        (Duration::MAX, gz_transport::Error::InvalidConfig),
    ] {
        assert!(matches!(
            server.prepare_run(timeout),
            Err(Error::Transport(actual)) if actual == expected
        ));
        assert!(
            observer
                .known_topic_info("/clock")
                .unwrap()
                .publishers
                .is_empty()
        );
        assert_eq!(server.counters().clocks_queued, 0);
        assert_eq!(server.counters().root_clocks_queued, 0);
    }
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    assert_eq!(server.tick().unwrap().sim_time, Duration::from_millis(1));
}

#[test]
fn world_interest_also_attempts_root_and_packet_limits_report_each_endpoint() {
    let configuration = config();
    let observer = Node::new(configuration.clone()).unwrap();
    let world = observer
        .subscribe("/world/test/clock", "gz.msgs.Clock")
        .unwrap();
    let mut server = running_root_server(configuration, "test", Duration::from_millis(1));
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    server.tick().unwrap();
    world.recv_timeout(Duration::from_secs(5)).unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    assert_eq!(server.counters().clocks_queued, 1);
    assert_eq!(server.counters().root_clocks_queued, 1);

    let mut idle = running_root_server(config(), "idle", Duration::from_millis(1));
    assert!(idle.prepare_run(Duration::from_secs(5)).unwrap());
    idle.tick().unwrap();
    assert_eq!(idle.counters().root_clocks_queued, 0);

    let configuration = Config {
        max_message_bytes: 1,
        ..config()
    };
    let observer = Node::new(configuration.clone()).unwrap();
    let _root = observer.subscribe("/clock", "gz.msgs.Clock").unwrap();
    let mut server = running_root_server(configuration, "test", Duration::from_millis(1));
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    server.tick().unwrap();
    assert_eq!(server.counters().clocks_queued, 0);
    assert_eq!(server.counters().clock_failures, 1);
    assert_eq!(server.counters().root_clocks_queued, 0);
    assert_eq!(server.counters().root_clock_failures, 1);
}

#[test]
fn stopped_workers_report_admission_and_root_snapshot_failures() {
    let configuration = config();
    let observer = Node::new(configuration.clone()).unwrap();
    let _root = observer.subscribe("/clock", "gz.msgs.Clock").unwrap();
    let mut admitted = running_root_server(configuration, "test", Duration::from_millis(1));
    assert!(admitted.prepare_run(Duration::from_secs(5)).unwrap());
    admitted.shutdown();
    assert!(matches!(
        admitted.prepare_run(Duration::from_secs(5)),
        Err(Error::Transport(TransportError::Shutdown))
    ));
    assert!(admitted.tick().is_err());
    assert_eq!(admitted.counters().root_clocks_queued, 0);
    assert_eq!(admitted.counters().root_clock_failures, 1);

    let configuration = config();
    let owner = Node::new(configuration.clone()).unwrap();
    let _foreign = owner.advertise("/clock", "gz.msgs.Clock").unwrap();
    let mut blocked = running_root_server(configuration, "test", Duration::from_millis(1));
    blocked.shutdown();
    assert!(matches!(
        blocked.prepare_run(Duration::from_secs(5)),
        Err(Error::Transport(TransportError::Shutdown))
    ));
}

#[test]
fn construction_does_not_claim_root_before_a_competitor_can_advertise() {
    let configuration = config();
    let observer = Node::new(configuration.clone()).unwrap();
    let root = observer.subscribe("/clock", "gz.msgs.Clock").unwrap();
    let world = observer
        .subscribe("/world/test/clock", "gz.msgs.Clock")
        .unwrap();
    let mut server = running_root_server(configuration, "test", Duration::from_millis(1));
    assert!(
        observer
            .known_topic_info("/clock")
            .unwrap()
            .publishers
            .is_empty()
    );
    let competitor = observer.advertise("/clock", "gz.msgs.StringMsg").unwrap();
    assert!(!server.prepare_run(Duration::from_secs(5)).unwrap());
    server.tick().unwrap();
    world.recv_timeout(Duration::from_secs(5)).unwrap();
    assert!(matches!(
        root.recv_timeout(Duration::from_millis(5)),
        Err(TransportError::Timeout)
    ));
    drop(competitor);
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    assert_eq!(
        matching_clocks(&mut server, &root, &world)
            .sim
            .unwrap()
            .nsec,
        2_000_000
    );
}
