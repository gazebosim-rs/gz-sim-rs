// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::EntityId;
use gz_sim_proto::msgs::{Double, WorldReset};
use gz_sim_scene::imu_rate;
use std::collections::BTreeSet;
use zenoh::Wait;

fn rate_id(server: &Server, index: usize) -> u64 {
    server
        .simulation
        .world()
        .get::<EntityId>(server.imus[index].entity)
        .unwrap()
        .get()
}
fn rate_topic(server: &Server) -> String {
    format!(
        "{}/set_rate",
        crate::options::as_valid_topic(&server.imus[0].metadata.topic)
    )
}
fn processed(server: &Server) -> u64 {
    let counters = server.counters();
    counters.imu_rates_applied + counters.imu_rates_ignored + counters.imu_rates_rejected
}
fn qualified(config: &ServerOptions, topic: &str) -> String {
    crate::options::qualified_name(
        &config.partition,
        &config.namespace,
        config.topic_remap(topic).unwrap_or(topic),
    )
    .unwrap()
}

fn remote_server(simulation: Simulation, config: ServerOptions) -> (Server, zenoh::Session) {
    let reservation = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/{}", reservation.local_addr().unwrap());
    let mut server_config = zenoh::Config::default();
    let mut client_config = zenoh::Config::default();
    for config in [&mut server_config, &mut client_config] {
        for (key, value) in [
            ("mode", "\"peer\""),
            ("scouting/multicast/enabled", "false"),
            ("scouting/gossip/enabled", "false"),
        ] {
            config.insert_json5(key, value).unwrap();
        }
    }
    server_config
        .insert_json5("listen/endpoints", &format!("[\"{endpoint}\"]"))
        .unwrap();
    server_config
        .insert_json5("connect/endpoints", "[]")
        .unwrap();
    client_config
        .insert_json5("listen/endpoints", "[]")
        .unwrap();
    client_config
        .insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]"))
        .unwrap();
    client_config
        .insert_json5("connect/timeout_ms", "2000")
        .unwrap();
    // Exclusive address handoff: any bind collision is a setup failure, never retried.
    drop(reservation);
    let server = Server::with_zenoh_config(
        simulation,
        "default",
        config,
        server_config,
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    let client = zenoh::open(client_config).wait().unwrap();
    (server, client)
}
fn imu_server(config: ServerOptions) -> (Server, zenoh::Session) {
    let simulation = gz_sim_scene::load_world(
        include_str!("../../../gz-sim-scene/tests/fixtures/fall.sdf"),
        "fall.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let (mut server, client) = remote_server(simulation, config);
    server.set_pose_publication_period(Duration::ZERO);
    server.set_interest_policy(InterestPolicy::Always);
    (server, client)
}

fn send_rate(server: &mut Server, client: &zenoh::Session, config: &ServerOptions, bytes: Vec<u8>) {
    send_to_providers(server, client, config, bytes, 1)
}
fn send_to_providers(
    server: &mut Server,
    client: &zenoh::Session,
    config: &ServerOptions,
    bytes: Vec<u8>,
    recipients: usize,
) {
    let expected = processed(server) + recipients as u64;
    let sender = client.clone();
    let key = qualified(config, &rate_topic(server));
    let deadline = Instant::now() + Duration::from_secs(5);
    let pending = thread::spawn(move || {
        // This is a distinct Session over TCP, not an SDK local provider shortcut.
        let replies = sender
            .get(key)
            .payload(bytes)
            .timeout(Duration::from_secs(5))
            .consolidation(zenoh::query::ConsolidationMode::None)
            .wait()
            .unwrap();
        let mut count = 0;
        while let Ok(reply) = replies.recv() {
            let sample = reply.result().expect("rate handoff must succeed");
            assert!(sample.payload().is_empty());
            count += 1;
        }
        count
    });
    while processed(server) < expected || !pending.is_finished() {
        assert!(
            Instant::now() < deadline,
            "rate handlers/query did not complete"
        );
        server.update().unwrap();
        thread::yield_now();
    }
    // Native Empty remains a handoff receipt even for the malformed marker;
    // the independent processed counters establish the CPU outcome.
    assert_eq!(pending.join().unwrap(), recipients);
}

fn provider_count(client: &zenoh::Session, config: &ServerOptions, topic: &str, count: usize) {
    let key = qualified(config, topic);
    let (partition, topic) = key.strip_prefix('@').unwrap().split_once('@').unwrap();
    let partition = partition.replace('/', "%");
    let topic = topic.replace('/', "%");
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let remaining = deadline
            .checked_duration_since(Instant::now())
            .expect("provider discovery timed out");
        let replies = client
            .liveliness()
            .get("@gz/**")
            .timeout(remaining)
            .wait()
            .unwrap();
        let mut identities = BTreeSet::new();
        while let Ok(reply) = replies.recv() {
            let sample = reply.result().unwrap();
            let fields: Vec<_> = sample.key_expr().as_str().split('/').collect();
            if fields.len() == 13
                && fields[1] == partition
                && fields[5] == "SS"
                && fields[9] == topic
            {
                assert_eq!(fields[10], "gz.msgs.Double&gz.msgs.Empty");
                assert_eq!(fields[3], fields[4]);
                identities.insert((fields[2].to_owned(), fields[3].to_owned()));
            }
        }
        if identities.len() == count {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "wrong provider identities: {identities:?}"
        );
        thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn paused_outputless_rate_service_records_applied_ignored_and_malformed_controls() {
    let config = config();
    let (mut server, client) = imu_server(config.clone());
    server.simulation.set_paused(true);
    let id = rate_id(&server, 0);
    for value in [200.0, 0.0, 2000.0, -5.0] {
        send_rate(
            &mut server,
            &client,
            &config,
            Double {
                data: value,
                ..Default::default()
            }
            .encode_to_vec(),
        );
        assert_eq!(imu_rate(server.simulation(), id).unwrap().rate_hz, 200.0);
        assert_eq!(server.simulation().elapsed(), Duration::ZERO);
    }
    send_rate(&mut server, &client, &config, vec![0x80]);
    assert_eq!(imu_rate(server.simulation(), id).unwrap().rate_hz, 200.0);
    let counters = server.counters();
    assert_eq!(
        (
            counters.imu_rates_applied,
            counters.imu_rates_ignored,
            counters.imu_rates_rejected
        ),
        (1, 3, 1)
    );
}

#[test]
fn rate_services_withdraw_on_removal_and_restore_initial_rate_on_both_resets() {
    let config = config();
    let (mut server, client) = imu_server(config.clone());
    let topic = rate_topic(&server);
    provider_count(&client, &config, &topic, 1);
    for all in [true, false] {
        send_rate(
            &mut server,
            &client,
            &config,
            Double {
                data: 200.0,
                ..Default::default()
            }
            .encode_to_vec(),
        );
        let id = rate_id(&server, 0);
        assert_eq!(imu_rate(server.simulation(), id).unwrap().rate_hz, 200.0);
        let ball = public_id(&server, "default::ball");
        gz_sim_scene::remove_model(&mut server.simulation, ball).unwrap();
        server.update().unwrap();
        provider_count(&client, &config, &topic, 0);
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
        provider_count(&client, &config, &topic, 1);
        assert_eq!(imu_rate(server.simulation(), id).unwrap().rate_hz, 1000.0);
        send_rate(
            &mut server,
            &client,
            &config,
            Double {
                data: 300.0,
                ..Default::default()
            }
            .encode_to_vec(),
        );
        assert_eq!(imu_rate(server.simulation(), id).unwrap().rate_hz, 300.0);
    }
    server.shutdown();
    provider_count(&client, &config, &topic, 0);
}

#[test]
fn shared_topic_keeps_two_providers_and_remote_query_changes_both_recipients() {
    let config = config();
    let xml = include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf").replace(
        "</sensor>", "</sensor><sensor name='other' type='imu'><always_on>true</always_on><topic>/p2/imu</topic><update_rate>1000</update_rate></sensor>",
    );
    let simulation = gz_sim_scene::load_world(
        &xml,
        "shared.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let (mut server, client) = remote_server(simulation, config.clone());
    provider_count(&client, &config, "/p2/imu/set_rate", 2);
    // D28 disposition: actual remote native Zenoh reaches both sensor-owned
    // queryables. The discarded SDK/local fast path selected just one.
    send_to_providers(
        &mut server,
        &client,
        &config,
        Double {
            data: 20.0,
            ..Default::default()
        }
        .encode_to_vec(),
        2,
    );
    let mut values: Vec<_> = (0..2)
        .map(|index| {
            imu_rate(server.simulation(), rate_id(&server, index))
                .unwrap()
                .rate_hz
        })
        .collect();
    values.sort_by(f64::total_cmp);
    assert_eq!(values, [20.0, 20.0]);
    assert_eq!(server.counters().imu_rates_applied, 2);
}
