// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_proto::{
    msgs::{Imu, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_scene::{
    ImuPublication, ImuSample, Name, PhysicsEngine, imu_active, imu_rate, load_world_with_sdf_step,
    queue_link_world_velocity, queue_link_world_wrench, remove_model, reset_loaded_world,
    set_imu_interest,
};
use gz_sim_server::{InterestPolicy, Server, ServerOptions};
use sdformat::NoResources;
use std::{
    num::NonZeroUsize,
    time::{Duration, Instant},
};
use zenoh::Wait;

const XML: &str = include_str!("../../../examples/mvp_imu.sdf");
const SENSOR: &str = "default::imu_box::body::imu";

fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "mvp_imu.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}

fn id(simulation: &Simulation, name: &str) -> u64 {
    simulation
        .world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

fn sample(simulation: &Simulation) -> &ImuSample {
    let mut samples = simulation
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<ImuSample>());
    let sample = samples.next().unwrap();
    assert!(samples.next().is_none());
    sample
}

fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() < 1e-9, "{actual} != {expected}");
}

fn isolated_server(simulation: Simulation) -> Server {
    let mut network = zenoh::Config::default();
    for (path, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        network.insert_json5(path, value).unwrap();
    }
    Server::with_zenoh_config(
        simulation,
        "default",
        ServerOptions {
            partition: format!("mvp-imu-{}", uuid::Uuid::new_v4()),
            ..ServerOptions::default()
        },
        network,
        NonZeroUsize::new(4).unwrap(),
    )
    .unwrap()
}

#[test]
fn demo_imu_uses_sensor_axes_si_units_end_step_cadence_and_reset_epochs() {
    let mut simulation = load(XML);
    assert_eq!(simulation.step_duration(), Duration::from_millis(1));
    let sensor_id = id(&simulation, SENSOR);
    let body_id = id(&simulation, "default::imu_box::body");
    let metadata = simulation
        .world()
        .iter_entities()
        .find_map(|entity| entity.get::<ImuPublication>())
        .unwrap();
    assert_eq!(metadata.topic, "/mvp/imu");
    assert_eq!(metadata.frame_id, "imu_box/body/imu");
    assert_eq!(metadata.entity_name, "imu_box::body::imu");
    assert!(metadata.enable_orientation);
    let rate = imu_rate(&simulation, sensor_id).unwrap();
    assert_eq!((rate.sdf_max_hz, rate.rate_hz), (100.0, 100.0));
    assert!(imu_active(&simulation, sensor_id).unwrap());
    assert!(
        simulation
            .world()
            .iter_entities()
            .all(|entity| !entity.contains::<ImuSample>())
    );

    // Two newtons on two kilograms adds +1 m/s² in world Y. The rolled sensor
    // observes -Z, while free-fall gravity cancels from its specific force.
    queue_link_world_wrench(&mut simulation, body_id, [0.0, 2.0, 0.0], [0.0; 3]).unwrap();
    simulation.step_exact(1).unwrap();
    let first = sample(&simulation);
    assert_eq!((first.time, first.epoch), (Duration::from_millis(1), 0));
    for (actual, expected) in first
        .reading
        .as_ref()
        .unwrap()
        .specific_force
        .into_iter()
        .zip([0.0, 0.0, -1.0])
    {
        close(actual, expected);
    }
    let mut previous = first.time;
    // Angular velocity is rad/s, not degrees/s; world +Z is this sensor's +Y.
    queue_link_world_velocity(&mut simulation, body_id, [0.0; 3], [0.0, 0.0, 2.0]).unwrap();
    let mut observed_times = vec![1];
    for tick in 2..=25 {
        simulation.step_exact(1).unwrap();
        let current = sample(&simulation);
        if current.time != previous {
            assert_eq!(current.time, Duration::from_millis(tick));
            assert_eq!(current.epoch, 0);
            observed_times.push(tick);
            previous = current.time;
            let reading = current.reading.as_ref().unwrap();
            for (actual, expected) in reading.specific_force.into_iter().zip([0.0; 3]) {
                close(actual, expected);
            }
            for (actual, expected) in reading.angular_velocity.into_iter().zip([0.0, 2.0, 0.0]) {
                close(actual, expected);
            }
            let half_angle = (tick - 1) as f64 * 0.001;
            for (actual, expected) in
                reading
                    .orientation
                    .into_iter()
                    .zip([0.0, half_angle.sin(), 0.0, half_angle.cos()])
            {
                close(actual, expected);
            }
            assert_eq!(reading.angular_velocity_covariance, [0.0; 3]);
            assert_eq!(reading.linear_acceleration_covariance, [0.0; 3]);
        }
    }
    assert_eq!(observed_times, [1, 10, 20]);
    for _ in 0..3 {
        simulation.tick().unwrap();
    }
    assert_eq!(simulation.elapsed(), Duration::from_millis(25));
    assert_eq!(sample(&simulation).time, Duration::from_millis(20));
    simulation.reset().unwrap();
    assert!(
        simulation
            .world()
            .iter_entities()
            .all(|entity| !entity.contains::<ImuSample>())
    );
    simulation.step_exact(1).unwrap();
    assert_eq!(
        (sample(&simulation).time, sample(&simulation).epoch),
        (Duration::from_millis(1), 1)
    );
    for value in sample(&simulation).reading.as_ref().unwrap().specific_force {
        close(value, 0.0);
    }
    reset_loaded_world(&mut simulation, |_| ()).unwrap();
    assert_eq!(id(&simulation, SENSOR), sensor_id);
    simulation.step_exact(1).unwrap();
    assert_eq!(
        (sample(&simulation).time, sample(&simulation).epoch),
        (Duration::from_millis(1), 2)
    );
}

fn without_sensors(xml: &str) -> String {
    let mut result = xml.to_owned();
    while let Some(start) = result.find("<sensor ") {
        let end = start + result[start..].find("</sensor>").unwrap() + "</sensor>".len();
        result.replace_range(start..end, "");
    }
    result
}

#[test]
fn no_imu_has_no_sampling_callbacks_and_last_model_removal_retires_them() {
    let start = XML.find("<model name=\"imu_box\">").unwrap();
    let end = start + XML[start..].find("</model>").unwrap() + "</model>".len();
    let second = XML[start..end]
        .replacen("name=\"imu_box\"", "name=\"second\"", 1)
        .replacen("0 0 3 0 0 0", "4 0 3 0 0 0", 1);
    let xml = XML.replace("</world>", &format!("{second}</world>"));
    let mut idle = load(&without_sensors(&xml));
    let idle_counts = idle.scheduled_system_counts();
    set_imu_interest(&mut idle, []);
    idle.step_exact(25).unwrap();
    idle.reset().unwrap();
    assert_eq!(idle.scheduled_system_counts(), idle_counts);
    assert!(
        idle.world()
            .iter_entities()
            .all(|entity| !entity.contains::<ImuPublication>() && !entity.contains::<ImuSample>())
    );

    let mut live = load(&xml);
    let counts = live.scheduled_system_counts();
    assert_eq!(counts, (idle_counts.0 + 1, idle_counts.1 + 1));
    let first_model = id(&live, "default::imu_box");
    let second_model = id(&live, "default::second");
    let first_sensor = id(&live, SENSOR);
    let second_sensor = id(&live, "default::second::body::imu");
    live.step_exact(1).unwrap();
    let retired = remove_model(&mut live, first_model).unwrap();
    assert!(retired.contains(&first_sensor));
    assert_eq!(live.scheduled_system_counts(), counts);
    live.step_exact(9).unwrap();
    assert_eq!(sample(&live).time, Duration::from_millis(10));
    let retired = remove_model(&mut live, second_model).unwrap();
    assert!(retired.contains(&second_sensor));
    assert_eq!(live.scheduled_system_counts(), idle_counts);
    set_imu_interest(&mut live, []);
    live.step_exact(10).unwrap();
    live.reset().unwrap();
    assert_eq!(live.scheduled_system_counts(), idle_counts);
    assert!(
        live.world()
            .iter_entities()
            .all(|entity| !entity.contains::<ImuPublication>() && !entity.contains::<ImuSample>())
    );
    reset_loaded_world(&mut live, |_| ()).unwrap();
    assert_eq!(live.scheduled_system_counts(), counts);
    assert_eq!(id(&live, SENSOR), first_sensor);
    assert_eq!(id(&live, "default::second::body::imu"), second_sensor);
    live.step_exact(1).unwrap();
    assert_eq!(
        live.world()
            .iter_entities()
            .filter(|entity| entity.contains::<ImuSample>())
            .count(),
        2
    );
    for entity in live.world().iter_entities() {
        if let Some(sample) = entity.get::<ImuSample>() {
            assert_eq!((sample.time, sample.epoch), (Duration::from_millis(1), 2));
            assert!(sample.reading.is_ok());
        }
    }
}

#[test]
fn server_subscriber_demand_activation_pause_and_reset_preserve_sample_counts() {
    let mut server = isolated_server(load(XML));
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    let sensor_id = id(server.simulation(), SENSOR);
    assert_eq!(server.interest_policy(), InterestPolicy::Subscribers);
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    for _ in 0..25 {
        server.update().unwrap();
    }
    assert_eq!(server.counters().imus_queued, 0);
    assert!(
        server
            .simulation()
            .world()
            .iter_entities()
            .all(|entity| !entity.contains::<ImuSample>())
    );
    server.set_interest_policy(InterestPolicy::Always);
    for _ in 0..20 {
        server.update().unwrap();
    }
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(45));
    assert_eq!(server.counters().imus_queued, 3); // ticks 26, 30, 40
    assert_eq!(sample(server.simulation()).time, Duration::from_millis(40));
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    for _ in 0..3 {
        server.update().unwrap();
    }
    assert_eq!(server.counters().imus_queued, 3);
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(45));
    server.set_imu_active(sensor_id, false).unwrap();
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    for _ in 0..20 {
        server.update().unwrap();
    }
    assert!(!imu_active(server.simulation(), sensor_id).unwrap());
    assert_eq!(server.counters().imus_queued, 3);
    assert_eq!(sample(server.simulation()).time, Duration::from_millis(40));
    server.set_imu_active(sensor_id, true).unwrap();
    server.update().unwrap();
    assert_eq!(sample(server.simulation()).time, Duration::from_millis(66));
    assert_eq!(server.counters().imus_queued, 4);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.update().unwrap();
    assert_eq!(server.simulation().epoch(), 1);
    assert_eq!(server.simulation().elapsed(), Duration::ZERO);
    assert_eq!(server.counters().imus_queued, 4);
    assert!(
        server
            .simulation()
            .world()
            .iter_entities()
            .all(|entity| !entity.contains::<ImuSample>())
    );
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        multi_step: 10,
        ..Default::default()
    }));
    for _ in 0..10 {
        server.update().unwrap();
    }
    assert_eq!(server.counters().imus_queued, 6); // reset-epoch ticks 1 and 10
    assert_eq!(
        (
            sample(server.simulation()).time,
            sample(server.simulation()).epoch
        ),
        (Duration::from_millis(10), 1)
    );
    assert!(server.simulation().is_paused());
    assert_eq!(server.counters().imu_failures, 0);
    assert_eq!(server.counters().invalid_imu_samples, 0);
    server.shutdown();
    let before = server.simulation().elapsed();
    assert!(matches!(
        server.update(),
        Err(gz_sim_server::Error::WorldClosed)
    ));
    assert_eq!(server.simulation().elapsed(), before);
}

fn decode_imu(sample: zenoh::sample::Sample, key: &str) -> Imu {
    assert_eq!(sample.key_expr().as_str(), key);
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.IMU"
    );
    let message = Imu::decode(sample.payload().to_bytes().as_ref()).unwrap();
    assert_eq!(message.entity_name, "imu_box::body::imu");
    let header = message.header.as_ref().unwrap();
    assert_eq!(header.data.len(), 2);
    assert_eq!(header.data[0].key, "frame_id");
    assert_eq!(header.data[0].value, ["imu_box/body/imu"]);
    assert_eq!(header.data[1].key, "seq");
    message
}

#[test]
fn demo_publishes_canonical_imu_over_real_zenoh_with_paused_and_reset_fences() {
    // Reuse the actual owned-listener locator pattern: no dropped reservation,
    // external router, multicast discovery, or guessed TCP port.
    let mut observer_config = zenoh::Config::default();
    for (path, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        observer_config.insert_json5(path, value).unwrap();
    }
    let observer = zenoh::open(observer_config).wait().unwrap();
    let locators: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|locator| locator.to_string())
        .collect();
    assert_eq!(locators.len(), 1);
    assert!(locators[0].starts_with("tcp/127.0.0.1:"));
    assert_ne!(locators[0], "tcp/127.0.0.1:0");
    let options = ServerOptions {
        partition: format!("mvp-imu-wire-{}", uuid::Uuid::new_v4()),
        ..ServerOptions::default()
    };
    let key = options.topic_key("/mvp/imu").unwrap();
    let frames = observer
        .declare_subscriber(key.clone())
        .with(zenoh::handlers::FifoChannel::new(1024))
        .wait()
        .unwrap();
    let mut network = zenoh::Config::default();
    for (path, value) in [
        ("mode", "\"client\"".to_owned()),
        ("listen/endpoints", "[]".to_owned()),
        (
            "connect/endpoints",
            serde_json::to_string(&locators).unwrap(),
        ),
        ("connect/timeout_ms", "2000".to_owned()),
        ("scouting/multicast/enabled", "false".to_owned()),
        ("scouting/gossip/enabled", "false".to_owned()),
    ] {
        network.insert_json5(path, &value).unwrap();
    }
    let mut simulation = load(XML);
    let body_id = id(&simulation, "default::imu_box::body");
    // A known twist makes the actual startup wire check nonzero rad/s in the
    // rotated sensor frame. Upward motion keeps the bounded readiness phase
    // clear of contacts; loaded resets below restore the unchanged demo state.
    queue_link_world_velocity(&mut simulation, body_id, [0.0, 0.0, 100.0], [0.0, 0.0, 2.0])
        .unwrap();
    let mut server = Server::with_zenoh_config(
        simulation,
        "default",
        options,
        network,
        NonZeroUsize::new(4).unwrap(),
    )
    .unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    assert_eq!(server.interest_policy(), InterestPolicy::Subscribers);
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));

    // One bounded readiness phase uses actual delivery, then drains every
    // queued startup attempt while paused before the first reset epoch.
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut startup = Vec::new();
    while startup.is_empty() {
        server.update().unwrap();
        while let Some(frame) = frames.try_recv().unwrap() {
            startup.push(decode_imu(frame, &key));
        }
        assert!(
            Instant::now() < deadline,
            "IMU subscriber did not receive an actual frame"
        );
        if startup.is_empty() {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    server.update().unwrap();
    let startup_count = usize::try_from(server.counters().imus_queued).unwrap();
    while startup.len() < startup_count {
        startup.push(decode_imu(
            frames
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap(),
            &key,
        ));
    }
    assert_eq!(startup.len(), startup_count);
    for (sequence, message) in startup.iter().enumerate() {
        assert_eq!(
            message.header.as_ref().unwrap().data[1].value,
            [sequence.to_string()]
        );
        let angular = message.angular_velocity.as_ref().unwrap();
        for (actual, expected) in [angular.x, angular.y, angular.z]
            .into_iter()
            .zip([0.0, 2.0, 0.0])
        {
            close(actual, expected);
        }
        let acceleration = message.linear_acceleration.as_ref().unwrap();
        for value in [acceleration.x, acceleration.y, acceleration.z] {
            close(value, 0.0);
        }
    }
    assert!(
        frames
            .recv_timeout(Duration::from_millis(20))
            .unwrap()
            .is_none()
    );

    for epoch in 1..=2 {
        let before = server.counters().imus_queued;
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        }));
        server.update().unwrap();
        assert_eq!(server.simulation().epoch(), epoch);
        assert_eq!(server.simulation().elapsed(), Duration::ZERO);
        assert_eq!(server.counters().imus_queued, before);
        assert!(frames.try_recv().unwrap().is_none());
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            multi_step: 25,
            ..Default::default()
        }));
        let mut received_times = Vec::new();
        for tick in 1..=25 {
            server.update().unwrap();
            if [1, 10, 20].contains(&tick) {
                let message = decode_imu(
                    frames
                        .recv_timeout(Duration::from_secs(5))
                        .unwrap()
                        .unwrap(),
                    &key,
                );
                let header = message.header.as_ref().unwrap();
                assert_eq!(header.stamp.as_ref().unwrap().sec, 0);
                assert_eq!(header.stamp.as_ref().unwrap().nsec, tick * 1_000_000);
                assert_eq!(header.data[1].value, [received_times.len().to_string()]);
                let q = message.orientation.unwrap();
                for (actual, expected) in [q.x, q.y, q.z, q.w].into_iter().zip([0.0, 0.0, 0.0, 1.0])
                {
                    close(actual, expected);
                }
                let angular = message.angular_velocity.unwrap();
                let acceleration = message.linear_acceleration.unwrap();
                for value in [
                    angular.x,
                    angular.y,
                    angular.z,
                    acceleration.x,
                    acceleration.y,
                    acceleration.z,
                ] {
                    close(value, 0.0);
                }
                for covariance in [
                    message.orientation_covariance,
                    message.angular_velocity_covariance,
                    message.linear_acceleration_covariance,
                ] {
                    assert_eq!(covariance.unwrap().data, vec![0.0; 9]);
                }
                let local = sample(server.simulation());
                assert_eq!(
                    (local.time, local.epoch),
                    (Duration::from_millis(tick as u64), epoch)
                );
                received_times.push(tick);
            }
        }
        assert_eq!(received_times, [1, 10, 20]);
        assert_eq!(server.counters().imus_queued, before + 3);
        assert!(server.simulation().is_paused());
        for _ in 0..3 {
            server.update().unwrap();
        }
        assert_eq!(server.simulation().elapsed(), Duration::from_millis(25));
        assert_eq!(server.counters().imus_queued, before + 3);
        assert!(
            frames
                .recv_timeout(Duration::from_millis(20))
                .unwrap()
                .is_none()
        );
    }
    assert_eq!(server.counters().imu_failures, 0);
    assert_eq!(server.counters().invalid_imu_samples, 0);
    server.shutdown();
    drop(frames);
    observer.close().wait().unwrap();
}
