// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_msgs::msgs::{Double, WorldReset};
use gz_sim_core::EntityId;
use gz_sim_scene::imu_rate;

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
        gz_transport::as_valid_topic(&server.imus[0].metadata.topic)
    )
}

fn processed(server: &Server) -> u64 {
    let counters = server.counters();
    counters.imu_rates_applied + counters.imu_rates_ignored + counters.imu_rates_rejected
}

fn send_rate(server: &mut Server, config: &Config, bytes: Vec<u8>) {
    let expected = processed(server) + 1;
    let client = Arc::new(Node::new(config.clone()).unwrap());
    let sender = Arc::clone(&client);
    let topic = rate_topic(server);
    let pending = thread::spawn(move || {
        sender.request(
            &topic,
            "gz.msgs.Double",
            "gz.msgs.Empty",
            bytes,
            Duration::from_secs(5),
        )
    });
    let deadline = Instant::now() + Duration::from_secs(5);
    while processed(server) < expected {
        assert!(Instant::now() < deadline, "rate handler did not execute");
        server.tick().unwrap();
        thread::yield_now();
    }
    // Empty is outputless: this proves handoff, not the handler's outcome.
    let (reply, handed_off) = pending.join().unwrap().unwrap();
    assert!(handed_off);
    assert!(reply.is_empty());
}

fn provider_count(client: &Node, topic: &str, count: usize) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let providers = client.service_provider_types(topic).unwrap();
        if providers.len() == count {
            assert!(
                providers
                    .iter()
                    .all(|provider| provider.request_type == "gz.msgs.Double"
                        && provider.response_type == "gz.msgs.Empty")
            );
            return;
        }
        assert!(
            Instant::now() < deadline,
            "wrong provider count: {providers:?}"
        );
        thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn paused_outputless_rate_service_records_applied_ignored_and_malformed_controls() {
    let config = config();
    let mut server = imu_server(config.clone());
    server.simulation.set_paused(true);
    let id = rate_id(&server, 0);
    for value in [200.0, 0.0, 2000.0, -5.0] {
        send_rate(
            &mut server,
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
    send_rate(&mut server, &config, vec![0x80]);
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
    let mut server = imu_server(config.clone());
    let topic = rate_topic(&server);
    let client = Node::new(config.clone()).unwrap();
    provider_count(&client, &topic, 1);
    for all in [true, false] {
        send_rate(
            &mut server,
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
        server.tick().unwrap();
        provider_count(&client, &topic, 0);
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
        provider_count(&client, &topic, 1);
        assert_eq!(imu_rate(server.simulation(), id).unwrap().rate_hz, 1000.0);
        send_rate(
            &mut server,
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
    provider_count(&client, &topic, 0);
}

#[test]
fn shared_topic_keeps_two_providers_and_changes_only_one_recipient() {
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
    let mut server = Server::new(
        simulation,
        "default",
        config.clone(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    let client = Node::new(config.clone()).unwrap();
    provider_count(&client, "/p2/imu/set_rate", 2);
    send_rate(
        &mut server,
        &config,
        Double {
            data: 20.0,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    let mut values: Vec<_> = (0..2)
        .map(|index| {
            imu_rate(server.simulation(), rate_id(&server, index))
                .unwrap()
                .rate_hz
        })
        .collect();
    values.sort_by(f64::total_cmp);
    assert_eq!(values, [20.0, 1000.0]);
    assert_eq!(server.counters().imu_rates_applied, 1);
}
