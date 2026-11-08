// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sensors::air_pressure::{
    AirPressureInput, AirPressureReading, InvalidAirPressureInput, air_pressure,
};
use gz_sim_proto::{
    msgs::{WorldControl, WorldReset},
    prost::Name,
};
use gz_sim_scene::{
    AirPressureSampleError, Name as SceneName, PhysicsEngine, load_world_with_sdf_step,
    remove_model,
};

const XML: &str = include_str!("../../../gz-sim-scene/tests/fixtures/m15_air_pressure.sdf");
fn metadata() -> AirPressurePublication {
    AirPressurePublication {
        entity_name: "rig::body::air_pressure".into(),
        frame_id: "rig/body/air_pressure".into(),
        topic: "/m15/air_pressure".into(),
    }
}
fn sample() -> AirPressureSample {
    AirPressureSample {
        time: Duration::new(2, 123),
        epoch: 4,
        reading: Ok(AirPressureReading {
            pressure: 92125.25,
            variance: 0.0,
        }),
    }
}

#[test]
fn pressure_canonical_packet_preserves_native_fields_acquisition_time_frame_and_sequence() {
    let message = packet(&metadata(), &sample(), u64::MAX).expect("valid sample");
    assert_eq!(FluidPressure::full_name(), "gz.msgs.FluidPressure");
    assert_eq!(message.pressure.to_bits(), 92125.25_f64.to_bits());
    assert_eq!(message.variance.to_bits(), 0.0_f64.to_bits());
    let header = message.header.as_ref().expect("header");
    assert_eq!(
        header.stamp,
        Some(gz_sim_proto::msgs::Time { sec: 2, nsec: 123 })
    );
    assert_eq!(
        header.data,
        vec![
            Map {
                key: "frame_id".into(),
                value: vec!["rig/body/air_pressure".into()]
            },
            Map {
                key: "seq".into(),
                value: vec![u64::MAX.to_string()]
            },
        ]
    );
    assert_eq!(
        FluidPressure::decode(message.encode_to_vec().as_slice()).expect("wire decode"),
        message
    );
}

#[test]
fn noisy_pressure_variance_is_preserved_as_double_and_each_typed_error_is_suppressed() {
    let ideal = packet(&metadata(), &sample(), 9).unwrap();
    for variance in [1.0, 0.2_f64.sqrt(), f64::MAX] {
        let noisy = AirPressureSample {
            reading: Ok(AirPressureReading {
                variance,
                ..sample().reading.unwrap()
            }),
            ..sample()
        };
        let wire = packet(&metadata(), &noisy, 9).unwrap();
        assert_eq!(wire.header, ideal.header);
        assert_eq!(wire.pressure, ideal.pressure);
        assert_eq!(wire.variance.to_bits(), variance.to_bits());
        assert_eq!(
            FluidPressure::decode(wire.encode_to_vec().as_slice()).unwrap(),
            wire
        );
    }
    for error in [
        AirPressureSampleError::Ideal(InvalidAirPressureInput),
        AirPressureSampleError::Noise(gz_sensors::InvalidNoise),
        AirPressureSampleError::NonpositiveNoisyPressure,
    ] {
        assert!(
            packet(
                &metadata(),
                &AirPressureSample {
                    reading: Err(error),
                    ..sample()
                },
                9
            )
            .is_none()
        );
    }
}

#[test]
fn noisy_pressure_real_wire_reset_reconnect_and_retirement_keep_acquisition_ownership() {
    let xml = XML.replace("<air_pressure/>", "<air_pressure><pressure><noise type='gaussian_quantized'><mean>2.25</mean><stddev>0.2</stddev><precision>0.5</precision><dynamic_bias_stddev>100</dynamic_bias_stddev><dynamic_bias_correlation_time>2</dynamic_bias_correlation_time></noise></pressure></air_pressure>");
    let (mut server, observer, key) = server(&xml);
    let subscriber = observer.declare_subscriber(key.clone()).wait().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while !server.air_pressures[0].publisher.matching().unwrap() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    let identity = server.air_pressures[0].identity;
    server.set_paused(false);
    let mut values = Vec::new();
    for tick in 1..=20 {
        server.update().unwrap();
        if let Some(sequence) = [1, 10, 20].iter().position(|&time| time == tick) {
            let received = subscriber
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap();
            assert_eq!(
                received.attachment().unwrap().to_bytes().as_ref(),
                b"gz.msgs.FluidPressure"
            );
            let message = FluidPressure::decode(received.payload().to_bytes().as_ref()).unwrap();
            let acquired = server
                .simulation
                .world()
                .get::<AirPressureSample>(server.air_pressures[0].entity)
                .unwrap();
            assert_eq!(
                message,
                packet(&metadata(), acquired, sequence as u64).unwrap()
            );
            assert_eq!(message.variance.to_bits(), 0.2_f64.sqrt().to_bits());
            assert_eq!(message.pressure / 0.5, (message.pressure / 0.5).round());
            values.push(message.pressure);
        }
    }
    server.set_paused(true);
    for _ in 0..3 {
        server.update().unwrap();
    }
    assert!(subscriber.try_recv().unwrap().is_none());
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert_eq!(server.update().unwrap().epoch, 1);
    assert_eq!(server.air_pressures[0].identity, identity);
    server.set_paused(false);
    server.update().unwrap();
    let message = FluidPressure::decode(
        subscriber
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(message.pressure, values[0]);
    assert_eq!(message.header.unwrap().data[1].value, ["0"]);
    drop(subscriber);
    let deadline = Instant::now() + Duration::from_secs(5);
    while server.air_pressures[0].publisher.matching().unwrap() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    server.update().unwrap();
    assert!(
        server
            .simulation
            .world()
            .get::<AirPressureSample>(server.air_pressures[0].entity)
            .is_none()
    );
    let subscriber = observer.declare_subscriber(key).wait().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while !server.air_pressures[0].publisher.matching().unwrap() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    for _ in 0..8 {
        server.update().unwrap();
    }
    let message = FluidPressure::decode(
        subscriber
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(message.pressure, values[1]);
    assert_eq!(message.header.unwrap().data[1].value, ["1"]);
    let model = server
        .simulation
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<SceneName>()
                .is_some_and(|name| name.0 == "default::rig")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let lease = Arc::downgrade(&server.air_pressures[0].publisher);
    remove_model(&mut server.simulation, model).unwrap();
    server.set_paused(true);
    server.update().unwrap();
    assert!(server.air_pressures.is_empty());
    assert!(lease.upgrade().is_none());
    assert!(subscriber.try_recv().unwrap().is_none());
}

#[test]
fn newly_invalid_noisy_acquisition_never_republishes_the_previous_valid_pressure() {
    let xml = XML.replace("<air_pressure/>", "<air_pressure><pressure><noise type='gaussian'><mean>-101280</mean></noise></pressure></air_pressure>");
    let (mut server, observer, key) = server(&xml);
    let subscriber = observer.declare_subscriber(key).wait().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while !server.air_pressures[0].publisher.matching().unwrap() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    server.set_paused(false);
    server.update().unwrap();
    let first = FluidPressure::decode(
        subscriber
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert!(first.pressure > 0.0);
    assert_eq!(server.counters().air_pressures_queued, 1);
    let link = server
        .simulation
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<SceneName>()
                .is_some_and(|name| name.0 == "default::rig::body")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    // Stay below Rapier's maintained 400m/s solver velocity bound. This modest
    // height change crosses the configured pressure threshold at the next 100Hz sample.
    gz_sim_scene::queue_link_world_velocity(
        &mut server.simulation,
        link,
        [0.0, 0.0, 100.0],
        [0.0; 3],
    )
    .unwrap();
    for _ in 0..9 {
        server.update().unwrap();
    }
    let sensor_z = server
        .simulation
        .world()
        .get::<gz_sim_scene::WorldPose>(server.air_pressures[0].entity)
        .unwrap()
        .translation[2];
    assert!(sensor_z > 3.5);
    let ideal = air_pressure(AirPressureInput { world_z: sensor_z }).unwrap();
    assert!(ideal.pressure > 0.0 && ideal.pressure - 101280.0 <= 0.0);
    let acquired = server
        .simulation
        .world()
        .get::<AirPressureSample>(server.air_pressures[0].entity)
        .unwrap();
    assert_eq!(acquired.time, Duration::from_millis(10));
    assert_eq!(
        acquired.reading,
        Err(AirPressureSampleError::NonpositiveNoisyPressure)
    );
    assert_eq!(server.counters().invalid_air_pressure_samples, 1);
    server.publish_air_pressures();
    assert_eq!(server.counters().invalid_air_pressure_samples, 1);
    assert_eq!(server.counters().air_pressures_queued, 1);
    assert_eq!(server.air_pressures[0].sequence, 1);
    // A synchronous same-worker declaration fences the queued output before
    // checking that no stale first sample escaped as a second publication.
    let _fence = server
        .world_io
        .air_pressure_output("@/pressure-noise-fence@/sample".into())
        .unwrap();
    assert!(subscriber.try_recv().unwrap().is_none());
}

#[test]
fn pressure_metadata_nonphysical_values_faults_and_unrepresentable_stamps_never_encode() {
    for field in 0..3 {
        let mut invalid = metadata();
        match field {
            0 => invalid.entity_name.clear(),
            1 => invalid.frame_id.clear(),
            _ => invalid.topic.clear(),
        }
        assert!(packet(&invalid, &sample(), 0).is_none());
    }
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -1.0] {
        let invalid = AirPressureSample {
            reading: Ok(AirPressureReading {
                pressure: value,
                variance: 0.0,
            }),
            ..sample()
        };
        assert!(packet(&metadata(), &invalid, 0).is_none());
    }
    for variance in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
        let invalid = AirPressureSample {
            reading: Ok(AirPressureReading {
                pressure: 101325.0,
                variance,
            }),
            ..sample()
        };
        assert!(packet(&metadata(), &invalid, 0).is_none());
    }
    assert!(
        packet(
            &metadata(),
            &AirPressureSample {
                reading: Err(AirPressureSampleError::Ideal(InvalidAirPressureInput)),
                ..sample()
            },
            0
        )
        .is_none()
    );
    assert!(
        packet(
            &metadata(),
            &AirPressureSample {
                time: Duration::MAX,
                ..sample()
            },
            0
        )
        .is_none()
    );
}

fn config(endpoint: Option<&str>) -> zenoh::Config {
    let mut config = zenoh::Config::default();
    let connect = endpoint.map_or_else(|| "[]".into(), |endpoint| format!("[\"{endpoint}\"]"));
    for (path, value) in [
        ("mode", "\"peer\""),
        (
            "listen/endpoints",
            if endpoint.is_none() {
                "[\"tcp/127.0.0.1:0\"]"
            } else {
                "[]"
            },
        ),
        ("connect/endpoints", connect.as_str()),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config
            .insert_json5(path, value)
            .expect("bounded local Zenoh config");
    }
    config
}
fn server(xml: &str) -> (Server, zenoh::Session, String) {
    let observer = zenoh::open(config(None)).wait().expect("owned observer");
    let locators: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|value| value.to_string())
        .collect();
    assert_eq!(locators.len(), 1);
    assert!(locators[0].starts_with("tcp/127.0.0.1:"));
    assert_ne!(locators[0], "tcp/127.0.0.1:0");
    let simulation = load_world_with_sdf_step(
        xml,
        "server-air_pressure.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .expect("fixture simulation");
    let options = ServerOptions {
        partition: format!("m15-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    };
    let key = options
        .topic_key("/m15/air_pressure")
        .expect("canonical key");
    let server = Server::with_zenoh_config(
        simulation,
        "default",
        options,
        config(Some(&locators[0])),
        NonZeroUsize::new(4).expect("positive budget"),
    )
    .expect("server with air_pressure");
    (server, observer, key)
}

#[test]
fn pressure_real_zenoh_motion_cadence_pause_reset_and_removal_keep_native_contract() {
    let (mut server, observer, key) = server(XML);
    let subscriber = observer
        .declare_subscriber(key)
        .wait()
        .expect("real subscriber");
    let deadline = Instant::now() + Duration::from_secs(5);
    while !server.air_pressures[0]
        .publisher
        .matching()
        .expect("interest")
    {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    let identity = server.air_pressures[0].identity;
    let link = server
        .simulation
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<SceneName>()
                .is_some_and(|name| name.0 == "default::rig::body")
        })
        .expect("link")
        .get::<EntityId>()
        .expect("link ID")
        .get();
    gz_sim_scene::queue_link_world_velocity(
        &mut server.simulation,
        link,
        [0.0, 0.0, -1.0],
        [0.0; 3],
    )
    .expect("real vertical motion");
    server.set_paused(false);
    for tick in 1..=20 {
        let info = server.update().expect("advance server");
        assert_eq!(info.sim_time, Duration::from_millis(tick));
        if let Some(sequence) = [1, 10, 20].iter().position(|&value| value == tick) {
            let received = subscriber
                .recv_timeout(Duration::from_secs(5))
                .expect("receive")
                .expect("actual acquisition");
            assert_eq!(
                received
                    .attachment()
                    .expect("type attachment")
                    .to_bytes()
                    .as_ref(),
                b"gz.msgs.FluidPressure"
            );
            let message = FluidPressure::decode(received.payload().to_bytes().as_ref())
                .expect("canonical decode");
            let expected = packet(
                &metadata(),
                &AirPressureSample {
                    epoch: info.epoch,
                    time: info.sim_time,
                    ..sample()
                },
                sequence as u64,
            )
            .expect("expected header");
            assert_eq!(message.header, expected.header);
            let expected = air_pressure(AirPressureInput {
                world_z: 3.5 - info.sim_time.as_secs_f64(),
            })
            .expect("absolute offset-point pressure");
            assert!((message.pressure - expected.pressure).abs() < 1e-9);
            assert_eq!(message.variance.to_bits(), 0.0_f64.to_bits());
            let native = server
                .simulation
                .world()
                .get::<AirPressureSample>(server.air_pressures[0].entity)
                .expect("actual acquired sample")
                .reading
                .as_ref()
                .expect("physical domain");
            assert_eq!(message.pressure.to_bits(), native.pressure.to_bits());
        }
    }
    assert_eq!(server.counters().air_pressures_queued, 3);
    server.set_paused(true);
    for _ in 0..3 {
        assert!(server.update().expect("paused poll").dt.is_zero());
    }
    assert!(subscriber.try_recv().expect("empty receive").is_none());
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    let reset = server.update().expect("loaded reset");
    assert_eq!((reset.epoch, reset.sim_time), (1, Duration::ZERO));
    assert_eq!(server.air_pressures[0].identity, identity);
    assert!(
        subscriber
            .try_recv()
            .expect("no reset-time publication")
            .is_none()
    );
    server.set_paused(false);
    server.update().expect("new epoch advance");
    let message = FluidPressure::decode(
        subscriber
            .recv_timeout(Duration::from_secs(5))
            .expect("receive")
            .expect("new epoch sample")
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .expect("decode reset sample");
    assert_eq!(
        message.header.as_ref().expect("header").data[1].value,
        ["0"]
    );
    let initial = air_pressure(AirPressureInput { world_z: 3.5 }).expect("initial sensor altitude");
    assert_eq!(message.pressure.to_bits(), initial.pressure.to_bits());
    assert_eq!(message.variance.to_bits(), 0.0_f64.to_bits());
    let model = server
        .simulation
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<SceneName>()
                .is_some_and(|name| name.0 == "default::rig")
        })
        .expect("model")
        .get::<EntityId>()
        .expect("model ID")
        .get();
    let lease = Arc::downgrade(&server.air_pressures[0].publisher);
    remove_model(&mut server.simulation, model).expect("model removal");
    server.set_paused(true);
    server.update().expect("retirement poll");
    assert!(server.air_pressures.is_empty() && server.air_pressures.capacity() == 0);
    assert!(lease.upgrade().is_none());
    assert!(subscriber.try_recv().expect("empty receive").is_none());
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert_eq!(server.update().expect("restore removed sensor").epoch, 2);
    assert_eq!(server.air_pressures[0].identity, identity);
    let deadline = Instant::now() + Duration::from_secs(5);
    while !server.air_pressures[0]
        .publisher
        .matching()
        .expect("restored interest")
    {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    server.set_paused(false);
    server.update().expect("restored acquisition");
    let message = FluidPressure::decode(
        subscriber
            .recv_timeout(Duration::from_secs(5))
            .expect("receive")
            .expect("restored sample")
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .expect("decode");
    assert_eq!(message.header.expect("header").data[1].value, ["0"]);
    assert_eq!(
        (
            server.counters().air_pressure_failures,
            server.counters().invalid_air_pressure_samples
        ),
        (0, 0)
    );
    server.shutdown();
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}

#[test]
fn pressure_disconnected_demand_and_absent_sensor_own_no_measurement_or_stream() {
    let (mut server, observer, key) = server(XML);
    server.set_paused(false);
    for _ in 0..20 {
        server.update().expect("disconnected advance");
    }
    assert!(
        server
            .simulation
            .world()
            .get::<AirPressureSample>(server.air_pressures[0].entity)
            .is_none()
    );
    assert_eq!(server.counters().air_pressures_queued, 0);
    server.set_interest_policy(InterestPolicy::Always);
    server.update().expect("explicit local acquisition");
    assert!(
        server
            .simulation
            .world()
            .get::<AirPressureSample>(server.air_pressures[0].entity)
            .is_some()
    );
    server.set_interest_policy(InterestPolicy::Subscribers);
    server.update().expect("disconnect clears sample");
    assert!(
        server
            .simulation
            .world()
            .get::<AirPressureSample>(server.air_pressures[0].entity)
            .is_none()
    );
    drop(
        observer
            .declare_subscriber(key)
            .wait()
            .expect("brief subscriber"),
    );
    let start = XML.find("    <sensor ").expect("sensor start");
    let end = start + XML[start..].find("</sensor>").expect("sensor end") + "</sensor>".len();
    let absent = format!("{}{}", &XML[..start], &XML[end..]);
    let (mut absent, _, _) = self::server(&absent);
    assert!(absent.air_pressures.is_empty() && absent.air_pressures.capacity() == 0);
    absent.set_paused(false);
    for _ in 0..3 {
        absent.update().expect("absent sensor advance");
    }
    assert_eq!(absent.counters().air_pressures_queued, 0);
}

#[test]
fn pressure_occupied_endpoints_invalid_ownership_and_reset_metadata_drift_refuse() {
    let (mut server, _, key) = server(XML);
    let entity = server.air_pressures[0].entity;
    let original = Arc::clone(&server.air_pressures[0].publisher);
    let occupied = BTreeSet::from([key]);
    assert!(matches!(
        streams(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &occupied,
            None
        ),
        Err(Error::AirPressureIdentity)
    ));
    assert_eq!(server.air_pressures.len(), 1);
    assert!(Arc::ptr_eq(&original, &server.air_pressures[0].publisher));
    let prepared = prepare_reset(&server, &BTreeSet::new()).expect("prepare original metadata");
    server
        .simulation
        .world_mut()
        .get_mut::<AirPressurePublication>(entity)
        .expect("metadata")
        .frame_id = "drift".into();
    assert!(matches!(
        streams(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &BTreeSet::new(),
            Some(&prepared)
        ),
        Err(Error::ResetAirPressures)
    ));
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(EntityKind::Visual);
    assert!(matches!(
        streams(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &BTreeSet::new(),
            None
        ),
        Err(Error::AirPressureIdentity)
    ));
    server.retire_air_pressures();
    assert!(server.air_pressures.is_empty());
}

#[test]
fn pressure_invalid_stale_future_and_bounded_publication_failures_never_become_valid_samples() {
    let (mut server, _, _) = server(XML);
    let entity = server.air_pressures[0].entity;
    server.simulation.step_exact(20).expect("local advance");
    for (epoch, time, reading) in [
        (
            0,
            Duration::from_millis(20),
            Err(AirPressureSampleError::Ideal(InvalidAirPressureInput)),
        ),
        (1, Duration::from_millis(20), sample().reading),
        (0, Duration::from_millis(21), sample().reading),
    ] {
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(AirPressureSample {
                epoch,
                time,
                reading,
            });
        server.publish_air_pressures();
        server.publish_air_pressures();
    }
    assert_eq!(server.counters().invalid_air_pressure_samples, 3);
    assert_eq!(server.air_pressures[0].sequence, 0);
    server.simulation.step_exact(10).expect("fresh sample");
    server.world_io.max_message_bytes = 1;
    server.publish_air_pressures();
    assert_eq!(server.counters().air_pressure_failures, 1);
    assert_eq!(server.air_pressures[0].sequence, 1);
    server.world_io.max_message_bytes = 16 * 1024 * 1024;
    server.simulation.step_exact(10).expect("another sample");
    server.air_pressures[0].sequence = u64::MAX;
    server.publish_air_pressures();
    assert_eq!(server.counters().air_pressures_queued, 1);
    assert_eq!(server.air_pressures[0].sequence, 0);
}

fn with_air_pressure(base: &str) -> String {
    let start = XML
        .find("  <model name='rig'>")
        .expect("air_pressure model start");
    let end = XML.rfind("</model>").expect("air_pressure model end") + "</model>".len();
    base.replace("</world>", &format!("{}\n</world>", &XML[start..end]))
}

fn assert_all_collisions(
    simulation: &mut Simulation,
    io: &world_io::WorldIo,
    config: &ServerOptions,
    topics: &[&str],
) {
    let occupied =
        air_pressure_reserved_keys(simulation, io, config).expect("complete reserved set");
    let entity = simulation
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<AirPressurePublication>())
        .expect("loaded air_pressure")
        .id();
    for topic in topics {
        let key = config.topic_key(topic).expect("actual qualified key");
        assert!(
            occupied.contains(&key),
            "missing reservation for {topic}: {key}"
        );
        simulation
            .world_mut()
            .get_mut::<AirPressurePublication>(entity)
            .expect("metadata")
            .topic = (*topic).into();
        assert!(
            matches!(
                streams(simulation, io, config, &occupied, None),
                Err(Error::AirPressureIdentity)
            ),
            "collision did not refuse {topic} before the first new declaration"
        );
    }
    simulation
        .world_mut()
        .get_mut::<AirPressurePublication>(entity)
        .expect("metadata")
        .topic = "/m15/air_pressure".into();
}

#[test]
fn pressure_every_reserved_family_and_removed_initial_configuration_blocks_air_pressure_aliases() {
    let (server, _, _) = server(XML);
    let mut options = server.network_config.clone();
    options
        .add_topic_remap("/clock", "/m15/remapped_clock")
        .expect("one root-clock remap");
    let mut sensor_world = load_world_with_sdf_step(
        XML,
        "reserved-world",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .expect("loaded sensor");
    assert_all_collisions(
        &mut sensor_world,
        &server.world_io,
        &options,
        &[
            "/world/default/clock",
            "/world/default/pose/info",
            "/world/default/control",
            "/world/default/remove",
            "/world/default/remove/blocking",
            "/clock",
        ],
    );
    let start = XML.find("    <sensor ").expect("sensor start");
    let end = start + XML[start..].find("</sensor>").expect("sensor end") + "</sensor>".len();
    let instrument = format!("{}{}{}", &XML[..start],
        "<sensor name='imu' type='imu'><topic>/m15/imu</topic><imu/></sensor>\
         <sensor name='lidar' type='lidar'><topic>/m15/scan</topic><lidar><scan><horizontal>\
         <samples>3</samples><resolution>1</resolution><min_angle>-0.5</min_angle><max_angle>0.5</max_angle>\
         </horizontal></scan><range><min>0.1</min><max>10</max><resolution>0</resolution></range></lidar></sensor>",
        &XML[end..]).replace("name='rig'", "name='instrument'");
    let force = include_str!("../fixtures/joint_force.sdf").replace("</model>",
        "<plugin filename='gz-sim-apply-joint-force-system' name='gz::sim::systems::ApplyJointForce'>\
         <joint_name>hinge</joint_name></plugin></model>");
    let fixtures = [
        (
            instrument,
            "default::instrument",
            vec!["/m15/imu", "/m15/imu/set_rate", "/m15/scan"],
        ),
        (
            include_str!("../../../../examples/m8_diff_drive.sdf").to_owned(),
            "default::robot",
            vec!["/model/robot/cmd_vel", "/model/robot/odometry"],
        ),
        (
            include_str!("../../../../examples/m9_joint_controllers.sdf").to_owned(),
            "default::arm",
            vec![
                "/model/arm/joint/base_hinge/0/cmd_pos",
                "/model/arm/joint/tip_hinge/cmd_vel",
            ],
        ),
        (
            force,
            "default::rotor",
            vec!["/model/rotor/joint/hinge/cmd_force"],
        ),
    ];
    for (base, owner, topics) in fixtures {
        let mut simulation = load_world_with_sdf_step(
            &with_air_pressure(&base),
            "reserved-families",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .expect("actual loaded family metadata");
        assert_all_collisions(&mut simulation, &server.world_io, &options, &topics);
        let model = simulation
            .world()
            .iter_entities()
            .find(|entity| {
                entity
                    .get::<SceneName>()
                    .is_some_and(|name| name.0 == owner)
            })
            .expect("family owner")
            .get::<EntityId>()
            .expect("model identity")
            .get();
        remove_model(&mut simulation, model).expect("retire live family");
        // The same exact keys remain reserved by the loaded reconstruction blueprint.
        assert_all_collisions(&mut simulation, &server.world_io, &options, &topics);
    }
    assert_eq!(server.air_pressures.len(), 1);
    assert_eq!(server.air_pressures[0].metadata, metadata());
    assert_eq!(server.counters().air_pressures_queued, 0);
}

// These owned snapshots must not borrow Server across the mutation under test.
fn native_boundary_snapshot(server: &Server) -> impl PartialEq + std::fmt::Debug + use<> {
    let world = server.simulation.world();
    let backend = world.resource::<gz_physics::PhysicsBackend>();
    let mut bodies: Vec<_> = world
        .iter_entities()
        .filter_map(|entity| entity.get::<gz_sim_scene::BodyHandle>())
        .map(|handle| {
            (
                handle.id(),
                backend.state(handle.id()).expect("actual native state"),
            )
        })
        .collect();
    bodies.sort_by_key(|(id, _)| *id);
    let mut identities: Vec<_> = world
        .iter_entities()
        .filter_map(|entity| entity.get::<EntityId>().map(|id| (id.get(), entity.id())))
        .collect();
    identities.sort_by_key(|(id, _)| *id);
    (
        server.simulation.epoch(),
        server.simulation.elapsed(),
        bodies,
        identities,
    )
}

fn refusal_boundary_snapshot(server: &Server) -> impl PartialEq + std::fmt::Debug + use<> {
    let streams: Vec<_> = server
        .air_pressures
        .iter()
        .map(|stream| {
            (
                stream.entity,
                stream.identity,
                Arc::as_ptr(&stream.publisher),
                stream.sequence,
                stream.last_sample,
            )
        })
        .collect();
    (
        native_boundary_snapshot(server),
        format!("{:?}", server.counters()),
        streams,
    )
}

#[test]
fn pressure_stream_identity_and_prepared_reset_fences_refuse_before_lease_or_scene_mutation() {
    for case in 0..4 {
        let (mut server, _observer, key) = server(XML);
        let entity = server.air_pressures[0].entity;
        let original = Arc::clone(&server.air_pressures[0].publisher);
        let mut prepared = if case >= 2 {
            Some(prepare_reset(&server, &BTreeSet::new()).expect("original reset transaction"))
        } else {
            None
        };
        match case {
            0 => {
                // A second public metadata owner must be refused before either
                // candidate can trigger a declaration, even without an assigned ID.
                server.simulation.world_mut().spawn(metadata());
            }
            1 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .remove::<EntityId>();
            }
            2 => {
                prepared
                    .as_mut()
                    .expect("prepared transaction")
                    .remove(&key)
                    .expect("configured endpoint removed from transaction");
            }
            _ => {
                let entry = prepared
                    .as_ref()
                    .expect("prepared transaction")
                    .get(&key)
                    .expect("configured endpoint")
                    .clone();
                let stale = server
                    .network_config
                    .topic_key("/m15/stale_reset_endpoint")
                    .expect("concrete stale endpoint");
                assert!(
                    prepared
                        .as_mut()
                        .expect("prepared transaction")
                        .insert(stale, entry)
                        .is_none()
                );
            }
        }
        let before = refusal_boundary_snapshot(&server);
        let result = streams(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &BTreeSet::new(),
            prepared.as_ref(),
        );
        if case < 2 {
            assert!(
                matches!(result, Err(Error::AirPressureIdentity)),
                "ownership case {case}"
            );
        } else {
            assert!(
                matches!(result, Err(Error::ResetAirPressures)),
                "reset case {case}"
            );
        }
        assert_eq!(refusal_boundary_snapshot(&server), before, "case {case}");
        assert!(Arc::ptr_eq(&original, &server.air_pressures[0].publisher));
        assert_eq!(server.air_pressures.len(), 1);
        drop(prepared);
        drop(original);
        server.shutdown();
    }
}

#[test]
fn pressure_closed_world_io_retires_air_pressure_demand_and_refuses_new_declarations_atomically() {
    let (mut server, observer, key) = server(XML);
    let subscriber = observer
        .declare_subscriber(key)
        .wait()
        .expect("real subscriber");
    let deadline = Instant::now() + Duration::from_secs(5);
    while !server.air_pressures[0]
        .publisher
        .matching()
        .expect("actual interest")
    {
        assert!(Instant::now() < deadline, "air_pressure interest deadline");
        std::thread::yield_now();
    }
    server.set_paused(false);
    server.update().expect("first real acquisition");
    let message = FluidPressure::decode(
        subscriber
            .recv_timeout(Duration::from_secs(5))
            .expect("receive")
            .expect("actual first frame")
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .expect("canonical first frame");
    assert_eq!(message.header.expect("header").data[1].value, ["0"]);
    let entity = server.air_pressures[0].entity;
    assert_eq!(
        server
            .simulation
            .world()
            .get::<AirPressureSample>(entity)
            .expect("actual sample")
            .time,
        Duration::from_millis(1)
    );
    let sequence = server.air_pressures[0].sequence;
    let lease = Arc::downgrade(&server.air_pressures[0].publisher);
    let physical_before = native_boundary_snapshot(&server);
    let fence_key = server
        .network_config
        .topic_key("/m15/closed_io_fence")
        .expect("fence topic");
    let fence = server
        .world_io
        .air_pressure_output(fence_key)
        .expect("prior queued publications completed");
    drop(fence);
    let mut expected_counters = server.counters();
    expected_counters.interest_failures += 1;
    server
        .world_io
        .session
        .close()
        .wait()
        .expect("close owned IO session");
    server.refresh_air_pressure_interest();
    assert_eq!(native_boundary_snapshot(&server), physical_before);
    assert_eq!(
        format!("{:?}", server.counters()),
        format!("{expected_counters:?}")
    );
    assert!(
        server
            .simulation
            .world()
            .get::<AirPressureSample>(entity)
            .is_none()
    );
    assert_eq!(server.air_pressures[0].sequence, sequence);
    assert!(Arc::ptr_eq(
        &lease.upgrade().expect("CPU owner retained"),
        &server.air_pressures[0].publisher
    ));

    // This crosses the next due 10ms deadline using actual Scene acquisition;
    // the closed IO error must select empty demand instead of leaving it active.
    server
        .simulation
        .step_exact(10)
        .expect("advance closed-interest Scene");
    assert_eq!(server.simulation.elapsed(), Duration::from_millis(11));
    assert!(
        server
            .simulation
            .world()
            .get::<AirPressureSample>(entity)
            .is_none()
    );
    assert_eq!(server.counters().air_pressures_queued, 1);
    assert!(subscriber.try_recv().expect("no later frame").is_none());
    let before = refusal_boundary_snapshot(&server);
    assert!(matches!(
        streams(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &BTreeSet::new(),
            None
        ),
        Err(Error::WorldNetwork(_))
    ));
    assert_eq!(refusal_boundary_snapshot(&server), before);
    assert!(Arc::ptr_eq(
        &lease.upgrade().expect("same CPU owner"),
        &server.air_pressures[0].publisher
    ));
    server.shutdown();
    assert!(lease.upgrade().is_none());
}

#[test]
fn pressure_reset_preflight_collision_and_closed_replacement_leave_scene_and_retirement_atomic() {
    let (mut server, _observer, key) = server(XML);
    let original = Arc::clone(&server.air_pressures[0].publisher);
    let lease = Arc::downgrade(&original);
    let occupied = BTreeSet::from([key]);
    let before = refusal_boundary_snapshot(&server);
    assert!(matches!(
        prepare_reset(&server, &occupied),
        Err(Error::AirPressureIdentity)
    ));
    assert_eq!(refusal_boundary_snapshot(&server), before);
    assert!(Arc::ptr_eq(&original, &server.air_pressures[0].publisher));
    drop(original);

    let model = server
        .simulation
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<SceneName>()
                .is_some_and(|name| name.0 == "default::rig")
        })
        .expect("loaded model")
        .get::<EntityId>()
        .expect("model ID")
        .get();
    let bodies: Vec<_> = server
        .simulation
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<gz_sim_scene::BodyHandle>())
        .map(gz_sim_scene::BodyHandle::id)
        .collect();
    assert!(!bodies.is_empty());
    remove_model(&mut server.simulation, model).expect("legitimate model removal");
    server.retire_air_pressures();
    assert!(server.air_pressures.is_empty() && server.air_pressures.capacity() == 0);
    assert!(lease.upgrade().is_none());
    server
        .world_io
        .session
        .close()
        .wait()
        .expect("close owned IO session");
    let before = refusal_boundary_snapshot(&server);
    // The loaded blueprint still owns the original endpoint, but there is no
    // live CPU lease to reuse. A real declaration refusal must precede reset.
    assert!(matches!(server.reset_scene(), Err(Error::WorldNetwork(_))));
    assert_eq!(refusal_boundary_snapshot(&server), before);
    assert!(server.air_pressures.is_empty() && server.air_pressures.capacity() == 0);
    assert!(lease.upgrade().is_none());
    let backend = server
        .simulation
        .world()
        .resource::<gz_physics::PhysicsBackend>();
    for body in bodies {
        assert!(backend.state(body).is_none());
    }
    server.shutdown();
}

#[test]
fn pressure_and_altimeter_live_and_removed_initial_keys_refuse_aliases_in_both_directions() {
    let altimeter = include_str!("../../../gz-sim-scene/tests/fixtures/m13_altimeter.sdf")
        .replace("name='rig'", "name='alt_rig'");
    let combined = with_air_pressure(&altimeter);
    for remove_pressure in [false, true] {
        let (mut server, _observer, pressure_key) = server(&combined);
        assert_eq!(server.air_pressures.len(), 1);
        assert_eq!(server.altimeters.len(), 1);
        let altimeter_key = server.altimeters[0].endpoint.clone();
        let pressure_entity = server.air_pressures[0].entity;
        let altimeter_entity = server
            .simulation
            .world()
            .iter_entities()
            .find(|entity| entity.contains::<gz_sim_scene::AltimeterPublication>())
            .expect("altimeter owner")
            .id();
        let pressure_lease = Arc::downgrade(&server.air_pressures[0].publisher);
        let altimeter_lease = Arc::downgrade(&server.altimeters[0].publisher);
        let pressure_reserved = air_pressure_reserved_keys(
            &server.simulation,
            &server.world_io,
            &server.network_config,
        )
        .expect("all other families");
        let altimeter_reserved =
            altimeter_reserved_keys(&server.simulation, &server.world_io, &server.network_config)
                .expect("all other families");
        assert!(pressure_reserved.contains(&altimeter_key));
        assert!(altimeter_reserved.contains(&pressure_key));
        // Remapping is applied once to both sides, before declaration.
        let mut remapped = server.network_config.clone();
        remapped
            .add_topic_remap("/m15/air_pressure", "/m13/altimeter")
            .expect("one cross-family alias");
        let before = refusal_boundary_snapshot(&server);
        let occupied = air_pressure_reserved_keys(&server.simulation, &server.world_io, &remapped)
            .expect("pressure aliases altimeter");
        assert!(matches!(
            streams(
                &server.simulation,
                &server.world_io,
                &remapped,
                &occupied,
                None
            ),
            Err(Error::AirPressureIdentity)
        ));
        let occupied = altimeter_reserved_keys(&server.simulation, &server.world_io, &remapped)
            .expect("altimeter reserves remapped pressure");
        assert!(matches!(
            altimeter::streams(
                &server.simulation,
                &server.world_io,
                &remapped,
                &occupied,
                None
            ),
            Err(Error::AltimeterIdentity)
        ));
        assert_eq!(refusal_boundary_snapshot(&server), before);
        assert!(Arc::ptr_eq(
            &pressure_lease.upgrade().expect("original pressure lease"),
            &server.air_pressures[0].publisher
        ));
        assert!(Arc::ptr_eq(
            &altimeter_lease.upgrade().expect("original altimeter lease"),
            &server.altimeters[0].publisher
        ));

        let owner_name = if remove_pressure {
            "default::rig"
        } else {
            "default::alt_rig"
        };
        let owner = server
            .simulation
            .world()
            .iter_entities()
            .find(|entity| {
                entity
                    .get::<SceneName>()
                    .is_some_and(|name| name.0 == owner_name)
            })
            .expect("selected original model")
            .get::<EntityId>()
            .expect("public model ID")
            .get();
        remove_model(&mut server.simulation, owner).expect("real family removal");
        server.retire_air_pressures();
        server.retire_altimeters();
        if remove_pressure {
            assert!(pressure_lease.upgrade().is_none());
            assert!(server.air_pressures.is_empty());
            let occupied = altimeter_reserved_keys(
                &server.simulation,
                &server.world_io,
                &server.network_config,
            )
            .expect("removed initial pressure remains reserved");
            assert!(occupied.contains(&pressure_key));
            server
                .simulation
                .world_mut()
                .get_mut::<gz_sim_scene::AltimeterPublication>(altimeter_entity)
                .expect("remaining altimeter")
                .topic = "/m15/air_pressure".into();
            let before = refusal_boundary_snapshot(&server);
            assert!(matches!(
                altimeter::streams(
                    &server.simulation,
                    &server.world_io,
                    &server.network_config,
                    &occupied,
                    None
                ),
                Err(Error::AltimeterIdentity)
            ));
            assert_eq!(refusal_boundary_snapshot(&server), before);
        } else {
            assert!(altimeter_lease.upgrade().is_none());
            assert!(server.altimeters.is_empty());
            let occupied = air_pressure_reserved_keys(
                &server.simulation,
                &server.world_io,
                &server.network_config,
            )
            .expect("removed initial altimeter remains reserved");
            assert!(occupied.contains(&altimeter_key));
            server
                .simulation
                .world_mut()
                .get_mut::<AirPressurePublication>(pressure_entity)
                .expect("remaining pressure")
                .topic = "/m13/altimeter".into();
            let before = refusal_boundary_snapshot(&server);
            assert!(matches!(
                streams(
                    &server.simulation,
                    &server.world_io,
                    &server.network_config,
                    &occupied,
                    None
                ),
                Err(Error::AirPressureIdentity)
            ));
            assert_eq!(refusal_boundary_snapshot(&server), before);
        }
        server.shutdown();
    }
}
