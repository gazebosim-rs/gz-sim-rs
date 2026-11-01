// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{collections::BTreeSet, time::Duration};
use zenoh::Wait;

fn sensor(name: &str, topic: &str) -> ImuPublication {
    ImuPublication {
        enable_orientation: true,
        entity_name: name.into(),
        frame_id: name.into(),
        topic: topic.into(),
    }
}

fn zenoh_config() -> zenoh::Config {
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

fn handoff(server: &crate::Server, config: &ServerOptions, bytes: Vec<u8>) -> Result<(), String> {
    let topic = "/p2/imu/set_rate";
    let remapped = config.topic_remap(topic).unwrap_or(topic);
    let key =
        crate::options::qualified_name(&config.partition, &config.namespace, remapped).unwrap();
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let replies = server
        .world_io
        .session
        .get(key)
        .payload(bytes)
        .timeout(Duration::from_secs(5))
        .wait()
        .map_err(|error| error.to_string())?;
    let reply = replies.recv().map_err(|error| error.to_string())?;
    let sample = reply.result().map_err(|error| {
        String::from_utf8_lossy(error.payload().to_bytes().as_ref()).into_owned()
    })?;
    assert!(sample.payload().is_empty());
    assert!(
        replies.recv().is_err(),
        "one provider returns exactly one Empty reply and closes the query"
    );
    assert!(
        std::time::Instant::now() < deadline,
        "query did not finalize before the original timeout"
    );
    Ok(())
}

#[test]
fn endpoint_groups_preserve_all_sensor_providers_across_capacity_inputs() {
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
                    let config = ServerOptions {
                        capacity,
                        ..ServerOptions::default()
                    };
                    let groups = endpoint_groups(&config, &sensors).unwrap();
                    // D28 removes SDK worker/thread packing. Retain the complete
                    // old sensor/capacity matrix as real endpoint/provider data.
                    assert_eq!(
                        groups.len(),
                        [a, b, c].iter().filter(|count| **count > 0).count()
                    );
                    assert_eq!(groups.values().map(Vec::len).sum::<usize>(), sensors.len());
                    let mut names = BTreeSet::new();
                    for (endpoint, providers) in groups {
                        for provider in providers {
                            let original = sensors
                                .iter()
                                .find(|sensor| sensor.entity_name == provider.name)
                                .unwrap();
                            let topic = format!(
                                "{}/set_rate",
                                crate::options::as_valid_topic(&original.topic)
                            );
                            assert_eq!(
                                crate::options::qualified_name(
                                    &config.partition,
                                    &config.namespace,
                                    &topic
                                )
                                .unwrap(),
                                endpoint
                            );
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
fn endpoint_groups_preserve_repaired_qualified_and_remapped_collisions() {
    let mut config = ServerOptions {
        namespace: "/robot".into(),
        ..ServerOptions::default()
    };
    config
        .add_topic_remap("/other/set_rate", "/robot/shared/set_rate")
        .unwrap();
    let sensors = [
        sensor("a", "shared"),
        sensor("b", "/robot/shared"),
        sensor("c", "/other"),
    ];
    let groups = endpoint_groups(&config, &sensors).unwrap();
    assert_eq!(groups.len(), 1);
    let providers = groups.values().next().unwrap();
    assert_eq!(providers.len(), 3);
    assert_eq!(
        providers
            .iter()
            .map(|provider| provider.name.as_str())
            .collect::<Vec<_>>(),
        ["a", "b", "c"]
    );
    let sensors = [sensor("a", "/sensor~one"), sensor("b", "/sensorone")];
    let groups = endpoint_groups(&ServerOptions::default(), &sensors).unwrap();
    assert_eq!(groups.len(), 1);
    assert_eq!(groups.values().next().unwrap().len(), 2);
}

#[test]
fn invalid_rate_configuration_fails_before_ingress_allocation() {
    let config = ServerOptions {
        capacity: 0,
        ..ServerOptions::default()
    };
    assert!(matches!(
        endpoint_groups(&config, std::iter::empty()),
        Err(SetupError::InvalidConfig)
    ));
    for config in [
        ServerOptions {
            capacity: usize::MAX,
            ..ServerOptions::default()
        },
        ServerOptions {
            max_message_bytes: 0,
            ..ServerOptions::default()
        },
        ServerOptions {
            max_message_bytes: usize::MAX,
            ..ServerOptions::default()
        },
    ] {
        assert!(matches!(
            endpoint_groups(&config, std::iter::empty()),
            Err(SetupError::InvalidConfig)
        ));
    }
    let config = ServerOptions {
        partition: "invalid partition".into(),
        ..ServerOptions::default()
    };
    assert!(endpoint_groups(&config, &[sensor("a", "/imu")]).is_err());
    assert!(
        endpoint_groups(&ServerOptions::default(), std::iter::empty())
            .unwrap()
            .is_empty()
    );
}

#[test]
fn queued_rate_controls_from_the_old_generation_do_not_apply_after_reset() {
    use gz_sim_proto::msgs::{WorldControl, WorldReset};
    let config = network_config();
    let simulation = gz_sim_scene::load_world(
        include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
        "reset.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let mut server = crate::Server::with_zenoh_config(
        simulation,
        "default",
        config.clone(),
        zenoh_config(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    for all in [true, false] {
        // Successful Empty proves only IO-to-CPU queue handoff, not application.
        handoff(
            &server,
            &config,
            Double {
                data: 200.0,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .unwrap();
        assert!(server.apply_control(WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all,
                time_only: !all,
                ..Default::default()
            }),
            ..Default::default()
        }));
        server.update().unwrap();
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
    let result = crate::Server::with_zenoh_config(
        simulation,
        "default",
        network_config(),
        zenoh_config(),
        NonZeroUsize::new(1).unwrap(),
    );
    assert!(matches!(result, Err(Error::ImuIdentity)));
}

fn network_config() -> ServerOptions {
    ServerOptions {
        partition: format!("rate-unit-{}", uuid::Uuid::new_v4()),
        ..ServerOptions::default()
    }
}

#[test]
fn rate_worker_failure_during_committed_reset_is_terminal_and_retires_leases() {
    use gz_sim_proto::msgs::{WorldControl, WorldReset};
    let simulation = gz_sim_scene::load_world(
        include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
        "reset.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let mut server = crate::Server::with_zenoh_config(
        simulation,
        "default",
        network_config(),
        zenoh_config(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    server.world_io.session.close().wait().unwrap();
    assert!(server.apply_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    let failure = server.update();
    assert!(matches!(failure, Err(Error::WorldClosed)), "{failure:?}");
    assert_eq!(
        server
            .simulation
            .world()
            .resource::<gz_sim_core::UpdateInfo>()
            .epoch,
        1
    );
    assert!(server.rates.is_none());
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
    assert!(matches!(server.update(), Err(Error::ResetStreams)));
}

#[test]
fn duplicate_sensor_names_from_a_reset_handler_fail_after_commit_without_live_services() {
    use bevy_ecs::prelude::*;
    use gz_sim_proto::msgs::{WorldControl, WorldReset};
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
    let mut server = crate::Server::with_zenoh_config(
        simulation,
        "default",
        network_config(),
        zenoh_config(),
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
    assert!(matches!(server.update(), Err(Error::ImuIdentity)));
    assert_eq!(
        server
            .simulation
            .world()
            .resource::<gz_sim_core::UpdateInfo>()
            .epoch,
        1
    );
    assert!(server.rates.is_none());
    assert!(server.reset_failed);
    assert!(matches!(server.update(), Err(Error::ResetStreams)));
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
    let config = ServerOptions {
        capacity: 6,
        ..network_config()
    };
    let mut server = crate::Server::with_zenoh_config(
        simulation,
        "default",
        config.clone(),
        zenoh_config(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    let bytes = Double {
        data: 200.0,
        ..Default::default()
    }
    .encode_to_vec();
    for _ in 0..6 {
        handoff(&server, &config, bytes.clone()).unwrap();
    }
    assert!(
        handoff(&server, &config, bytes).is_err(),
        "the seventh request must fail, never report handoff"
    );
    assert_eq!(server.counters().imu_rates_applied, 0);
    assert_eq!(server.ingress_statistics().queue_drops, 1);
    for expected in [2, 4, 6] {
        server.update().unwrap();
        assert_eq!(server.counters().imu_rates_applied, expected);
        assert_eq!(server.simulation().elapsed(), Duration::ZERO);
    }
}

fn isolated_rate_cpu() -> (Simulation, ServerOptions, Rates, RateSetup, Arc<AtomicU64>) {
    let simulation = gz_sim_scene::load_world(
        include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
        "reset.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let metadata: Vec<_> = simulation
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<ImuPublication>().cloned())
        .collect();
    let config = network_config();
    let drops = Arc::new(AtomicU64::new(0));
    let (mut rates, setup) = Rates::prepare(&config, &metadata, &drops).unwrap();
    rates.bind(&simulation).unwrap();
    (simulation, config, rates, setup, drops)
}

#[test]
fn cpu_retirement_cannot_apply_queued_io_query_and_io_retirement_waits_for_finalization() {
    let (mut simulation, config, mut rates, setup, drops) = isolated_rate_cpu();
    let session = zenoh::open(zenoh_config()).wait().unwrap();
    let (sender, incoming) = mpsc::sync_channel(2);
    let mut declarations = RateDeclarations::new(&session, setup, sender).unwrap();
    let key =
        crate::options::qualified_name(&config.partition, &config.namespace, "/p2/imu/set_rate")
            .unwrap();
    let replies = session
        .get(key)
        .payload(
            Double {
                data: 200.0,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    // This value is held by the test's IO phase, never inside CPU Rates.
    let world_io::ControlWork::Rate(request) =
        incoming.recv_timeout(Duration::from_secs(5)).unwrap()
    else {
        panic!("rate request expected")
    };
    rates.clear();
    assert!(rates.providers.is_empty());
    assert!(
        !declarations.reconcile(),
        "queued Query must keep IO retirement unsettled"
    );
    assert!(!declarations.is_empty());
    request.execute().unwrap();
    assert!(
        replies.recv().is_err(),
        "retired admission must not report a successful new handoff"
    );
    assert!(declarations.reconcile());
    assert!(declarations.is_empty());
    let mut counters = Counters::default();
    rates
        .poll(
            &mut simulation,
            NonZeroUsize::new(2).unwrap(),
            &mut counters,
        )
        .unwrap();
    assert_eq!(
        counters.imu_rates_applied + counters.imu_rates_ignored + counters.imu_rates_rejected,
        0
    );
    assert_eq!(drops.load(Ordering::Relaxed), 0);
}

#[test]
fn malformed_and_oversized_queries_keep_only_bounded_scalar_markers_for_cpu_budget() {
    let (mut simulation, mut config, mut rates, _, drops) = isolated_rate_cpu();
    config.max_message_bytes = 16;
    rates.clear();
    let metadata: Vec<_> = simulation
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<ImuPublication>().cloned())
        .collect();
    let (mut rates, setup) = Rates::prepare(&config, &metadata, &drops).unwrap();
    rates.bind(&simulation).unwrap();
    let session = zenoh::open(zenoh_config()).wait().unwrap();
    let (sender, incoming) = mpsc::sync_channel(2);
    let mut declarations = RateDeclarations::new(&session, setup, sender).unwrap();
    let key =
        crate::options::qualified_name(&config.partition, &config.namespace, "/p2/imu/set_rate")
            .unwrap();
    let malformed = session
        .get(key.clone())
        .payload(vec![0x80])
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let world_io::ControlWork::Rate(request) =
        incoming.recv_timeout(Duration::from_secs(5)).unwrap()
    else {
        panic!("rate request expected")
    };
    request.execute().unwrap();
    assert!(
        malformed
            .recv()
            .unwrap()
            .result()
            .unwrap()
            .payload()
            .is_empty()
    );
    assert!(malformed.recv().is_err());
    let oversized = session
        .get(key)
        .payload(vec![0; 17])
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let world_io::ControlWork::Rate(request) =
        incoming.recv_timeout(Duration::from_secs(5)).unwrap()
    else {
        panic!("rate request expected")
    };
    assert!(
        request.query.is_none(),
        "oversized bytes must not remain in IO ingress"
    );
    request.execute().unwrap();
    assert!(oversized.recv().is_err());
    let mut counters = Counters::default();
    for expected in [1, 2] {
        rates
            .poll(
                &mut simulation,
                NonZeroUsize::new(1).unwrap(),
                &mut counters,
            )
            .unwrap();
        assert_eq!(counters.imu_rates_rejected, expected);
        assert_eq!(counters.imu_rates_applied, 0);
        assert_eq!(simulation.elapsed(), Duration::ZERO);
    }
    assert_eq!(drops.load(Ordering::Relaxed), 0);
    rates.clear();
    assert!(declarations.reconcile());
    assert!(declarations.is_empty());
}

#[test]
fn full_control_ingress_rejects_query_without_mutating_cpu_or_losing_accepted_query() {
    let (mut simulation, config, mut rates, setup, drops) = isolated_rate_cpu();
    let id = rates.providers[0].id.unwrap();
    let session = zenoh::open(zenoh_config()).wait().unwrap();
    let (sender, incoming) = mpsc::sync_channel(1);
    let mut declarations = RateDeclarations::new(&session, setup, sender.clone()).unwrap();
    let key =
        crate::options::qualified_name(&config.partition, &config.namespace, "/p2/imu/set_rate")
            .unwrap();
    let accepted = session
        .get(key.clone())
        .payload(
            Double {
                data: 200.0,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    // Observe the real first callback, then leave its Query queued while the
    // second callback encounters the exact capacity-one ingress boundary.
    let first = incoming.recv_timeout(Duration::from_secs(5)).unwrap();
    sender.try_send(first).unwrap();
    let rejected = session
        .get(key)
        .payload(
            Double {
                data: 300.0,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    assert!(
        rejected.recv().is_err(),
        "full ingress must not report handoff"
    );
    assert_eq!(drops.load(Ordering::Relaxed), 1);
    assert!(matches!(
        rates.providers[0].input.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));
    let world_io::ControlWork::Rate(first) = incoming.try_recv().unwrap() else {
        panic!("rate request expected")
    };
    first.execute().unwrap();
    assert!(
        accepted
            .recv()
            .unwrap()
            .result()
            .unwrap()
            .payload()
            .is_empty()
    );
    assert!(accepted.recv().is_err());
    assert!(matches!(
        incoming.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));
    let mut counters = Counters::default();
    rates
        .poll(
            &mut simulation,
            NonZeroUsize::new(2).unwrap(),
            &mut counters,
        )
        .unwrap();
    assert_eq!(counters.imu_rates_applied, 1);
    assert_eq!(counters.imu_rates_ignored, 0);
    assert_eq!(counters.imu_rates_rejected, 0);
    assert_eq!(
        gz_sim_scene::imu_rate(&simulation, id).unwrap().rate_hz,
        200.0
    );
    assert_eq!(simulation.elapsed(), Duration::ZERO);
    rates.clear();
    assert!(declarations.reconcile());
    assert!(declarations.is_empty());
}

#[test]
fn retired_queued_setup_declares_no_service_or_presence() {
    let (_, config, mut rates, setup, drops) = isolated_rate_cpu();
    assert_eq!(rates.providers.len(), 1);
    rates.clear();
    assert!(rates.providers.is_empty());
    assert!(
        setup
            .providers
            .iter()
            .all(|provider| !provider.active.load(Ordering::Acquire))
    );
    let session = zenoh::open(zenoh_config()).wait().unwrap();
    let (sender, incoming) = mpsc::sync_channel(1);
    let declarations = RateDeclarations::new(&session, setup, sender).unwrap();
    let replies = session
        .liveliness()
        .get(format!("@gz/%{}/**", config.partition))
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let mut presence = 0;
    while let Ok(reply) = replies.recv() {
        reply.into_result().unwrap();
        presence += 1;
    }
    assert_eq!(presence, 0, "retired setup must not advertise a service");
    assert!(declarations.is_empty());
    assert!(matches!(
        incoming.try_recv(),
        Err(mpsc::TryRecvError::Disconnected)
    ));
    assert_eq!(drops.load(Ordering::Relaxed), 0);
}

#[test]
fn closed_session_refuses_rate_setup_without_mutating_sensor_or_drop_count() {
    let (simulation, _, rates, setup, drops) = isolated_rate_cpu();
    let id = rates.providers[0].id.unwrap();
    let before = gz_sim_scene::imu_rate(&simulation, id).unwrap().rate_hz;
    let session = zenoh::open(zenoh_config()).wait().unwrap();
    session.close().wait().unwrap();
    let (sender, _) = mpsc::sync_channel(1);
    assert!(matches!(
        RateDeclarations::new(&session, setup, sender),
        Err(Error::WorldClosed)
    ));
    assert_eq!(
        gz_sim_scene::imu_rate(&simulation, id).unwrap().rate_hz,
        before
    );
    assert!(matches!(
        rates.providers[0].input.try_recv(),
        Err(mpsc::TryRecvError::Disconnected)
    ));
    assert_eq!(drops.load(Ordering::Relaxed), 0);
    assert_eq!(simulation.elapsed(), Duration::ZERO);
}

#[test]
fn queued_rate_session_close_preserves_scalar_admission_and_full_queue_accounting() {
    for full in [false, true] {
        let (mut simulation, mut config, old_rates, old_setup, drops) = isolated_rate_cpu();
        drop((old_rates, old_setup));
        config.capacity = 1;
        let sensors: Vec<_> = simulation
            .world()
            .iter_entities()
            .filter_map(|entity| entity.get::<ImuPublication>().cloned())
            .collect();
        let (mut rates, setup) = Rates::prepare(&config, &sensors, &drops).unwrap();
        rates.bind(&simulation).unwrap();
        let id = rates.providers[0].id.unwrap();
        let session = zenoh::open(zenoh_config()).wait().unwrap();
        let (sender, incoming) = mpsc::sync_channel(1);
        let mut declarations = RateDeclarations::new(&session, setup, sender).unwrap();
        let key = crate::options::qualified_name(
            &config.partition,
            &config.namespace,
            "/p2/imu/set_rate",
        )
        .unwrap();
        let query = |value| {
            session
                .get(key.clone())
                .allowed_destination(zenoh::sample::Locality::SessionLocal)
                .payload(
                    Double {
                        data: value,
                        ..Default::default()
                    }
                    .encode_to_vec(),
                )
                .timeout(Duration::from_secs(3))
                .wait()
                .unwrap()
        };
        if full {
            let accepted = query(100.0);
            let world_io::ControlWork::Rate(request) =
                incoming.recv_timeout(Duration::from_secs(2)).unwrap()
            else {
                panic!("rate request expected");
            };
            request.execute().unwrap();
            assert!(
                accepted
                    .recv()
                    .unwrap()
                    .result()
                    .unwrap()
                    .payload()
                    .is_empty()
            );
            assert!(accepted.recv().is_err());
        }
        let replies = query(200.0);
        let world_io::ControlWork::Rate(request) =
            incoming.recv_timeout(Duration::from_secs(2)).unwrap()
        else {
            panic!("rate request expected");
        };
        session.close().wait().unwrap();
        assert!(request.state.active.load(Ordering::Acquire));
        // A retained local Query accepts its reply after Session close; queue
        // admission and the already-closed requester stream are distinct outcomes.
        request.execute().unwrap();
        assert!(replies.recv().is_err());
        assert_eq!(drops.load(Ordering::Relaxed), u64::from(full));
        let mut counters = Counters::default();
        rates
            .poll(
                &mut simulation,
                NonZeroUsize::new(2).unwrap(),
                &mut counters,
            )
            .unwrap();
        assert_eq!(counters.imu_rates_applied, 1);
        assert_eq!(
            (counters.imu_rates_ignored, counters.imu_rates_rejected),
            (0, 0)
        );
        assert_eq!(
            gz_sim_scene::imu_rate(&simulation, id).unwrap().rate_hz,
            if full { 100.0 } else { 200.0 }
        );
        assert_eq!(simulation.elapsed(), Duration::ZERO);
        rates.clear();
        assert!(declarations.reconcile());
        assert!(declarations.is_empty());
    }
}

#[test]
fn absent_rate_ingress_stays_absent_across_frames_and_prepared_resets() {
    use gz_sim_proto::msgs::{WorldControl, WorldReset};
    let simulation = gz_sim_scene::load_world(
        "<sdf version='1.12'><world name='default'/></sdf>",
        "absent-rates.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let scheduled = simulation.scheduled_system_counts();
    let mut server = crate::Server::with_zenoh_config(
        simulation,
        "default",
        network_config(),
        zenoh_config(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    let metric = Arc::downgrade(&server.world_io.ingress_drops);
    assert!(server.rates.is_none());
    for paused in [true, false] {
        server.set_paused(paused);
        for _ in 0..3 {
            server.update().unwrap();
            assert!(server.rates.is_none());
            assert_eq!(server.simulation.scheduled_system_counts(), scheduled);
        }
    }
    for all in [true, false] {
        assert!(server.apply_control(WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all,
                time_only: !all,
                ..Default::default()
            }),
            ..Default::default()
        }));
        server.prepare_io().unwrap();
        assert!(server.rates.is_none());
        let prepared = server.simulation.elapsed();
        let info = server.tick().unwrap();
        assert_eq!(info.sim_time, prepared);
        assert_eq!(server.simulation.elapsed(), prepared);
        assert!(server.rates.is_none());
        assert_eq!(server.simulation.scheduled_system_counts(), scheduled);
    }
    assert!(std::ptr::eq(
        metric.as_ptr(),
        Arc::as_ptr(&server.world_io.ingress_drops)
    ));
    assert_eq!(server.ingress_statistics().queue_drops, 0);
    assert_eq!(server.counters().imu_rates_applied, 0);
    assert_eq!(server.counters().imu_rates_ignored, 0);
    assert_eq!(server.counters().imu_rates_rejected, 0);
    server.shutdown();
    assert!(server.rates.is_none());
}

#[test]
fn shared_rate_providers_retire_individually_and_loaded_reset_discards_unread_work() {
    let xml = include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf").replace(
        "</sensor>",
        "</sensor><sensor name='other' type='imu'><topic>/other</topic><update_rate>1000</update_rate></sensor>",
    );
    let simulation = gz_sim_scene::load_world(
        &xml,
        "shared-rates.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let mut config = network_config();
    config.capacity = 1;
    config
        .add_topic_remap("/other/set_rate", "/p2/imu/set_rate")
        .unwrap();
    let mut server = crate::Server::with_zenoh_config(
        simulation,
        "default",
        config.clone(),
        zenoh_config(),
        NonZeroUsize::new(1).unwrap(),
    )
    .unwrap();
    let metric = Arc::downgrade(&server.world_io.ingress_drops);
    // Observe real scalar receiver lifetime without retaining a CPU provider.
    // The retained Query tests separately exercise native ingress and receipts.
    let sensors: Vec<_> = server
        .simulation
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<ImuPublication>().cloned())
        .collect();
    let (mut rates, setup) =
        Rates::prepare(&config, &sensors, &server.world_io.ingress_drops).unwrap();
    rates.bind(&server.simulation).unwrap();
    assert_eq!(rates.providers.len(), 2);
    assert_eq!(setup.providers[0].key, setup.providers[1].key);
    let inputs: Vec<_> = setup
        .providers
        .iter()
        .zip(&rates.providers)
        .map(|(spec, provider)| {
            (
                provider.id.unwrap(),
                spec.input.clone(),
                Arc::downgrade(&spec.active),
            )
        })
        .collect();
    server.rates = rates.into_active();
    server.world_io.install_rates(setup).unwrap();
    server.set_paused(true);
    for (index, (_, input, _)) in inputs.iter().enumerate() {
        input.try_send(Some(100. * (index + 1) as f64)).unwrap();
    }
    for expected in [1, 2] {
        server.update().unwrap();
        assert_eq!(server.counters().imu_rates_applied, expected);
        assert_eq!(server.simulation.elapsed(), Duration::ZERO);
    }
    for (index, (id, _, _)) in inputs.iter().enumerate() {
        assert_eq!(
            gz_sim_scene::imu_rate(server.simulation(), *id)
                .unwrap()
                .rate_hz,
            100. * (index + 1) as f64
        );
    }
    let first = entity_by_id(server.simulation.world(), inputs[0].0).unwrap();
    server.simulation.world_mut().entity_mut(first).despawn();
    server.update().unwrap();
    assert_eq!(server.rates.as_ref().unwrap().providers.len(), 1);
    assert!(matches!(
        inputs[0].1.try_send(Some(300.)),
        Err(mpsc::TrySendError::Disconnected(_))
    ));
    assert!(
        inputs[0]
            .2
            .upgrade()
            .is_none_or(|active| !active.load(Ordering::Acquire))
    );
    inputs[1].1.try_send(Some(300.)).unwrap();
    assert!(matches!(
        inputs[1].1.try_send(Some(400.)),
        Err(mpsc::TrySendError::Full(_))
    ));
    let last = entity_by_id(server.simulation.world(), inputs[1].0).unwrap();
    server.simulation.world_mut().entity_mut(last).despawn();
    server.update().unwrap();
    assert!(server.rates.is_none());
    assert!(matches!(
        inputs[1].1.try_send(Some(500.)),
        Err(mpsc::TrySendError::Disconnected(_))
    ));
    assert!(
        inputs[1]
            .2
            .upgrade()
            .is_none_or(|active| !active.load(Ordering::Acquire))
    );
    assert_eq!(server.counters().imu_rates_applied, 2);
    server.reset_scene().unwrap();
    assert_eq!(server.rates.as_ref().unwrap().providers.len(), 2);
    for provider in &server.rates.as_ref().unwrap().providers {
        assert!(provider.active.load(Ordering::Acquire));
        assert_eq!(
            gz_sim_scene::imu_rate(server.simulation(), provider.id.unwrap())
                .unwrap()
                .rate_hz,
            1000.
        );
    }
    server.update().unwrap();
    assert_eq!(server.counters().imu_rates_applied, 2);
    assert_eq!(server.counters().imu_rates_ignored, 0);
    assert_eq!(server.counters().imu_rates_rejected, 0);
    assert_eq!(server.ingress_statistics().queue_drops, 0);
    assert!(std::ptr::eq(
        metric.as_ptr(),
        Arc::as_ptr(&server.world_io.ingress_drops)
    ));
    for (_, input, _) in &inputs {
        assert!(matches!(
            input.try_send(Some(600.)),
            Err(mpsc::TrySendError::Disconnected(_))
        ));
    }
    server.shutdown();
    assert!(server.rates.is_none());
}

#[test]
fn real_rate_queue_loss_survives_final_cpu_retirement_and_loaded_reset() {
    let simulation = gz_sim_scene::load_world(
        include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
        "rate-loss.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let config = ServerOptions {
        capacity: 1,
        ..network_config()
    };
    let mut server = crate::Server::with_zenoh_config(
        simulation,
        "default",
        config.clone(),
        zenoh_config(),
        NonZeroUsize::new(1).unwrap(),
    )
    .unwrap();
    let metric = Arc::downgrade(&server.world_io.ingress_drops);
    handoff(
        &server,
        &config,
        Double {
            data: 200.,
            ..Default::default()
        }
        .encode_to_vec(),
    )
    .unwrap();
    assert!(
        handoff(
            &server,
            &config,
            Double {
                data: 300.,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .is_err()
    );
    assert_eq!(server.ingress_statistics().queue_drops, 1);
    assert_eq!(server.counters().imu_rates_applied, 0);
    let model = server
        .simulation
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<gz_sim_scene::Name>()
                .is_some_and(|name| name.0 == "default::ball")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    gz_sim_scene::remove_model(&mut server.simulation, model).unwrap();
    server.update().unwrap();
    assert!(server.rates.is_none());
    assert_eq!(server.ingress_statistics().queue_drops, 1);
    assert_eq!(server.counters().imu_rates_applied, 0);
    server.reset_scene().unwrap();
    assert_eq!(server.rates.as_ref().unwrap().providers.len(), 1);
    let id = server.rates.as_ref().unwrap().providers[0].id.unwrap();
    assert_eq!(
        gz_sim_scene::imu_rate(server.simulation(), id)
            .unwrap()
            .rate_hz,
        1000.
    );
    server.update().unwrap();
    assert_eq!(server.counters().imu_rates_applied, 0);
    assert_eq!(server.ingress_statistics().queue_drops, 1);
    assert!(std::ptr::eq(
        metric.as_ptr(),
        Arc::as_ptr(&server.world_io.ingress_drops)
    ));
    handoff(
        &server,
        &config,
        Double {
            data: 400.,
            ..Default::default()
        }
        .encode_to_vec(),
    )
    .unwrap();
    server.update().unwrap();
    assert_eq!(server.counters().imu_rates_applied, 1);
    assert_eq!(
        gz_sim_scene::imu_rate(server.simulation(), id)
            .unwrap()
            .rate_hz,
        400.
    );
    assert_eq!(server.ingress_statistics().queue_drops, 1);
    server.shutdown();
    assert!(server.rates.is_none());
}

#[test]
fn loaded_reset_waits_for_bounded_setup_admission_without_advancing_the_scene() {
    struct Release(Arc<(std::sync::Mutex<bool>, std::sync::Condvar)>);
    impl Drop for Release {
        fn drop(&mut self) {
            let (state, wake) = &*self.0;
            *state.lock().unwrap() = true;
            wake.notify_one();
        }
    }
    std::thread::scope(|scope| {
        let barrier = Arc::new((std::sync::Mutex::new(false), std::sync::Condvar::new()));
        let release = Release(Arc::clone(&barrier));
        let (started, waiting) = mpsc::sync_channel(0);
        let (finished, completion) = mpsc::sync_channel(1);
        let worker = scope.spawn(move || {
            let simulation = gz_sim_scene::load_world(
                include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
                "reset-admission.sdf",
                "default",
                Duration::from_millis(1),
                &sdformat::NoResources,
            )
            .unwrap();
            let mut config = network_config();
            config.capacity = 1;
            let mut server = crate::Server::with_zenoh_config(
                simulation,
                "default",
                config,
                zenoh_config(),
                NonZeroUsize::new(1).unwrap(),
            )
            .unwrap();
            server.set_paused(true);
            let (entered, observed) = mpsc::sync_channel(1);
            let observer = server
                .world_io
                .session
                .declare_subscriber(server.world_io.clock.key_expr().clone())
                .allowed_origin(zenoh::sample::Locality::SessionLocal)
                .callback(move |sample| {
                    entered.try_send(sample).unwrap();
                    let (state, wake) = &*barrier;
                    let (state, timeout) = wake
                        .wait_timeout_while(
                            state.lock().unwrap(),
                            Duration::from_secs(8),
                            |released| !*released,
                        )
                        .unwrap();
                    assert!(!timeout.timed_out() && *state);
                })
                .wait()
                .unwrap();
            let id = server.rates.as_ref().unwrap().providers[0].id.unwrap();
            let entity = entity_by_id(server.simulation.world(), id).unwrap();
            server.simulation.world_mut().entity_mut(entity).despawn();
            server.update().unwrap();
            observed.recv_timeout(Duration::from_secs(2)).unwrap();
            assert!(server.imus.is_empty() && server.rates.is_none());
            server
                .world_io
                .publish_poses(gz_sim_proto::msgs::PoseV::default().encode_to_vec())
                .unwrap();
            let before = *server
                .simulation
                .world()
                .resource::<gz_sim_core::UpdateInfo>();
            let counters = format!("{:?}", server.counters);
            started.send(()).unwrap();
            let reset = server.reset_scene();
            finished.send(reset.is_ok()).unwrap();
            let reset = reset.unwrap();
            assert_eq!(reset.epoch, before.epoch + 1);
            assert_eq!(reset.sim_time, Duration::ZERO);
            assert_eq!(server.simulation.elapsed(), Duration::ZERO);
            assert_eq!(format!("{:?}", server.counters), counters);
            assert_eq!(server.imus.len(), 1);
            assert_eq!(server.rates.as_ref().unwrap().providers.len(), 1);
            let restored = server.rates.as_ref().unwrap().providers[0].id.unwrap();
            assert_eq!(
                gz_sim_scene::imu_rate(server.simulation(), restored)
                    .unwrap()
                    .rate_hz,
                1000.
            );
            assert!(!server.reset_failed);
            drop(observer);
            server.shutdown();
        });
        waiting.recv_timeout(Duration::from_secs(2)).unwrap();
        // Observe the call remaining pending while the actual publication owner
        // and capacity-one FIFO are held; this is not a readiness sleep/retry.
        assert!(matches!(
            completion.recv_timeout(Duration::from_millis(50)),
            Err(mpsc::RecvTimeoutError::Timeout)
        ));
        drop(release);
        assert!(completion.recv_timeout(Duration::from_secs(2)).unwrap());
        worker.join().unwrap();
    });
}
