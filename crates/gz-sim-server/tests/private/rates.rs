// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::collections::BTreeSet;

fn sensor(name: &str, topic: &str) -> ImuPublication {
    ImuPublication {
        enable_orientation: true,
        entity_name: name.into(),
        frame_id: name.into(),
        topic: topic.into(),
    }
}

#[test]
fn worker_layout_uses_the_minimum_threads_for_collisions_and_capacity() {
    for capacity in 1..=5 {
        for a in 0..=5 {
            for b in 0..=5 {
                for c in 0..=5 {
                    let mut sensors = Vec::new();
                    for (topic, count) in [("/a", a), ("/b", b), ("/c", c)] {
                        for index in 0..count {
                            sensors.push(sensor(&format!("{topic}/{index}"), topic));
                        }
                    }
                    let config = Config {
                        capacity,
                        ..Config::default()
                    };
                    let workers = plan(&config, &sensors).unwrap();
                    let minimum = a.max(b).max(c).max(sensors.len().div_ceil(capacity));
                    assert_eq!(workers.len(), minimum);
                    assert_eq!(workers.iter().map(Vec::len).sum::<usize>(), sensors.len());
                    let mut names = BTreeSet::new();
                    for worker in workers {
                        assert!(worker.len() <= capacity);
                        let endpoints: BTreeSet<_> =
                            worker.iter().map(|spec| spec.topic.as_str()).collect();
                        assert_eq!(endpoints.len(), worker.len());
                        for provider in worker {
                            assert!(names.insert(provider.name));
                        }
                    }
                    assert_eq!(names.len(), sensors.len());
                }
            }
        }
    }
}

#[test]
fn worker_layout_counts_repaired_qualified_and_remapped_collisions() {
    let mut config = Config {
        namespace: "/robot".into(),
        ..Config::default()
    };
    config
        .add_topic_remap("/other/set_rate", "/robot/shared/set_rate")
        .unwrap();
    let sensors = [
        sensor("a", "shared"),
        sensor("b", "/robot/shared"),
        sensor("c", "/other"),
    ];
    assert_eq!(plan(&config, &sensors).unwrap().len(), 3);
    let sensors = [sensor("a", "/sensor~one"), sensor("b", "/sensorone")];
    assert_eq!(plan(&Config::default(), &sensors).unwrap().len(), 2);
}

#[test]
fn invalid_pool_configuration_fails_before_worker_allocation() {
    let config = Config {
        capacity: 0,
        ..Config::default()
    };
    assert!(matches!(
        plan(&config, std::iter::empty()),
        Err(TransportError::InvalidConfig)
    ));
    let config = Config {
        partition: "invalid partition".into(),
        ..Config::default()
    };
    assert!(plan(&config, &[sensor("a", "/imu")]).is_err());
    assert!(
        plan(&Config::default(), std::iter::empty())
            .unwrap()
            .is_empty()
    );
}

#[test]
fn queued_rate_controls_from_the_old_generation_do_not_apply_after_reset() {
    use gz_msgs::msgs::{WorldControl, WorldReset};
    use std::net::UdpSocket;
    let message = UdpSocket::bind("127.0.0.1:0").unwrap();
    let service = UdpSocket::bind("127.0.0.1:0").unwrap();
    let config = Config {
        partition: format!("rate-reset-{}", std::process::id()),
        message_port: message.local_addr().unwrap().port(),
        service_port: service.local_addr().unwrap().port(),
        ..Config::default()
    };
    drop((message, service));
    let simulation = gz_sim_scene::load_world(
        include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
        "reset.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let mut server =
        crate::Server::new(simulation, "default", config, NonZeroUsize::new(2).unwrap()).unwrap();
    for all in [true, false] {
        // The worker's own outputless request completes only after queue handoff.
        let (reply, handed_off) = server.rates.workers[&0]
            .request(
                "/p2/imu/set_rate",
                "gz.msgs.Double",
                "gz.msgs.Empty",
                Double {
                    data: 200.0,
                    ..Default::default()
                }
                .encode_to_vec(),
                Duration::from_secs(5),
            )
            .unwrap();
        assert!(handed_off && reply.is_empty());
        assert!(server.apply_control(WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all,
                time_only: !all,
                ..Default::default()
            }),
            ..Default::default()
        }));
        server.tick().unwrap();
        let id = server
            .simulation
            .world()
            .get::<EntityId>(server.imus[0].entity)
            .unwrap()
            .get();
        assert_eq!(
            gz_sim_scene::imu_rate(server.simulation(), id)
                .unwrap()
                .rate_hz,
            1000.0
        );
        assert_eq!(server.counters().imu_rates_applied, 0);
    }
}

#[test]
fn invalid_imu_public_identity_is_a_setup_error() {
    let mut simulation = Simulation::new(Duration::from_millis(1)).unwrap();
    simulation.world_mut().spawn(sensor("orphan", "/orphan"));
    let result = crate::Server::new(
        simulation,
        "default",
        network_config(),
        NonZeroUsize::new(1).unwrap(),
    );
    assert!(matches!(result, Err(Error::ImuIdentity)));
}

fn network_config() -> Config {
    use std::{
        net::UdpSocket,
        sync::atomic::{AtomicUsize, Ordering},
    };
    static ID: AtomicUsize = AtomicUsize::new(0);
    let message = UdpSocket::bind("127.0.0.1:0").unwrap();
    let service = UdpSocket::bind("127.0.0.1:0").unwrap();
    Config {
        partition: format!(
            "rate-unit-{}-{}",
            std::process::id(),
            ID.fetch_add(1, Ordering::Relaxed)
        ),
        message_port: message.local_addr().unwrap().port(),
        service_port: service.local_addr().unwrap().port(),
        ..Config::default()
    }
}

#[test]
fn rate_worker_failure_during_committed_reset_is_terminal_and_retires_leases() {
    use gz_msgs::msgs::{WorldControl, WorldReset};
    let simulation = gz_sim_scene::load_world(
        include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
        "reset.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let mut server = crate::Server::new(
        simulation,
        "default",
        network_config(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    server.rates.workers[&0].cancel();
    assert!(server.apply_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(
        server.tick(),
        Err(Error::Transport(TransportError::Shutdown))
    ));
    assert_eq!(
        server
            .simulation
            .world()
            .resource::<gz_sim_core::UpdateInfo>()
            .epoch,
        1
    );
    assert!(server.rates.providers.is_empty());
    assert!(server.reset_failed);
    let id = server
        .simulation
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<ImuPublication>())
        .unwrap()
        .get::<gz_sim_core::EntityId>()
        .unwrap()
        .get();
    assert!(matches!(
        server.set_imu_active(id, false),
        Err(Error::ResetStreams)
    ));
    assert!(gz_sim_scene::imu_active(server.simulation(), id).unwrap());
    assert!(matches!(server.tick(), Err(Error::ResetStreams)));
}

#[test]
fn duplicate_sensor_names_from_a_reset_handler_fail_after_commit_without_live_services() {
    use bevy_ecs::prelude::*;
    use gz_msgs::msgs::{WorldControl, WorldReset};
    let xml = include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf").replace(
        "</sensor>", "</sensor><sensor name='other' type='imu'><topic>/other</topic><update_rate>1000</update_rate></sensor>",
    );
    let mut simulation = gz_sim_scene::load_world(
        &xml,
        "reset.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    simulation.add_systems(
        gz_sim_core::Phase::Reset,
        |mut sensors: Query<&mut ImuPublication>| {
            for mut publication in &mut sensors {
                publication.entity_name = "duplicate".into();
            }
        },
    );
    let mut server = crate::Server::new(
        simulation,
        "default",
        network_config(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    assert!(server.apply_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            time_only: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(server.tick(), Err(Error::ImuIdentity)));
    assert_eq!(
        server
            .simulation
            .world()
            .resource::<gz_sim_core::UpdateInfo>()
            .epoch,
        1
    );
    assert!(server.rates.providers.is_empty());
    assert!(server.reset_failed);
    assert!(matches!(server.tick(), Err(Error::ResetStreams)));
}

#[test]
fn full_rate_queue_is_observable_and_each_tick_respects_the_command_budget() {
    let simulation = gz_sim_scene::load_world(
        include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
        "bounded.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let config = Config {
        capacity: 6,
        ..network_config()
    };
    let mut server =
        crate::Server::new(simulation, "default", config, NonZeroUsize::new(2).unwrap()).unwrap();
    let bytes = Double {
        data: 200.0,
        ..Default::default()
    }
    .encode_to_vec();
    for _ in 0..6 {
        assert!(
            server.rates.workers[&0]
                .request(
                    "/p2/imu/set_rate",
                    "gz.msgs.Double",
                    "gz.msgs.Empty",
                    bytes.clone(),
                    Duration::from_secs(5),
                )
                .unwrap()
                .1
        );
    }
    assert!(matches!(
        server.rates.workers[&0].request(
            "/p2/imu/set_rate",
            "gz.msgs.Double",
            "gz.msgs.Empty",
            bytes,
            Duration::from_secs(5),
        ),
        Err(TransportError::QueueFull)
    ));
    assert_eq!(server.counters().imu_rates_applied, 0);
    assert_eq!(server.transport_statistics().queue_drops, 1);
    for expected in [2, 4, 6] {
        server.tick().unwrap();
        assert_eq!(server.counters().imu_rates_applied, expected);
        assert_eq!(server.simulation().elapsed(), Duration::ZERO);
    }
}
