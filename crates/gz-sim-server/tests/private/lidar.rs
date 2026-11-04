// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sensors::lidar::PlanarLidarConfig;
use gz_sim_proto::{
    msgs::{WorldControl, WorldReset},
    prost::Name,
};
use gz_sim_scene::{
    Name as SceneName, PhysicsEngine, WorldPose, load_world_with_sdf_step, remove_model,
};

const XML: &str = r#"<sdf version='1.12'><world name='default'>
<gravity>0 0 0</gravity><physics name='step' type='ode'><max_step_size>0.001</max_step_size></physics>
<model name='rig'><static>true</static><pose>0 0 1 0 0 0</pose><link name='body'>
<collision name='body'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision>
<sensor name='lidar' type='lidar'><always_on>true</always_on><update_rate>100</update_rate>
<topic>/m7/scan</topic><frame_id>rig/body/lidar</frame_id><lidar><scan><horizontal>
<samples>3</samples><resolution>1</resolution><min_angle>-0.7853981633974483</min_angle><max_angle>0.7853981633974483</max_angle>
</horizontal></scan><range><min>0.1</min><max>10</max><resolution>0</resolution></range></lidar></sensor>
</link></model><model name='wall'><static>true</static><pose>5 0 1 0 0 0</pose><link name='body'>
<collision name='wall'><geometry><box><size>2 20 2</size></box></geometry></collision>
</link></model></world></sdf>"#;

fn metadata() -> LidarPublication {
    LidarPublication {
        entity_name: "rig::body::lidar".into(),
        frame_id: "rig/body/lidar".into(),
        topic: "/m7/scan".into(),
        config: PlanarLidarConfig {
            samples: 3,
            angle_min: -std::f64::consts::FRAC_PI_4,
            angle_max: std::f64::consts::FRAC_PI_4,
            range_min: 0.1,
            range_max: 10.0,
        },
        angle_step: std::f64::consts::FRAC_PI_4,
    }
}

fn sample() -> LidarSample {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    LidarSample {
        time: Duration::new(2, 123),
        epoch: 4,
        world_pose: WorldPose {
            translation: [1.0, 2.0, 3.0],
            rotation: [0.0, 0.0, s, s],
        },
        ranges: vec![0.1, 4.25, f64::INFINITY],
    }
}

#[test]
fn canonical_packet_preserves_acquisition_pose_frame_units_infinity_and_sequence() {
    let metadata = metadata();
    let sample = sample();
    let message = packet(&metadata, &sample, u64::MAX).unwrap();
    assert_eq!(LaserScan::full_name(), "gz.msgs.LaserScan");
    let header = message.header.as_ref().unwrap();
    assert_eq!(
        header.stamp,
        Some(gz_sim_proto::msgs::Time { sec: 2, nsec: 123 })
    );
    assert_eq!(
        header.data,
        vec![
            Map {
                key: "frame_id".into(),
                value: vec![metadata.frame_id.clone()]
            },
            Map {
                key: "seq".into(),
                value: vec![u64::MAX.to_string()]
            }
        ]
    );
    assert_eq!(message.frame, metadata.frame_id);
    let pose = message.world_pose.as_ref().unwrap();
    assert_eq!(pose.position, Some(vector3(sample.world_pose.translation)));
    assert_eq!(
        pose.orientation,
        Some(quaternion(sample.world_pose.rotation))
    );
    assert!(pose.header.is_none() && pose.name.is_empty() && pose.id == 0);
    assert_eq!(
        (message.angle_min, message.angle_max, message.angle_step),
        (
            -std::f64::consts::FRAC_PI_4,
            std::f64::consts::FRAC_PI_4,
            std::f64::consts::FRAC_PI_4
        )
    );
    assert_eq!(
        (message.range_min, message.range_max, message.count),
        (0.1, 10.0, 3)
    );
    assert_eq!(
        (
            message.vertical_angle_min,
            message.vertical_angle_max,
            message.vertical_angle_step,
            message.vertical_count
        ),
        (0.0, 0.0, 0.0, 1)
    );
    assert_eq!(message.ranges, sample.ranges);
    assert_eq!(message.intensities, [0.0; 3]);
    assert_eq!(
        LaserScan::decode(message.encode_to_vec().as_slice()).unwrap(),
        message
    );
    let mut maximum = metadata;
    maximum.config.samples = 128;
    maximum.angle_step = (maximum.config.angle_max - maximum.config.angle_min) / 127.0;
    let mut full = sample;
    full.ranges = vec![f64::INFINITY; 128];
    assert_eq!(packet(&maximum, &full, 0).unwrap().ranges.len(), 128);
}

#[test]
fn invalid_metadata_samples_and_timestamps_cannot_become_wire_packets() {
    let base = metadata();
    for field in 0..11 {
        let mut invalid = base.clone();
        match field {
            0 => invalid.config.samples = 1,
            1 => invalid.config.samples = 129,
            2 => invalid.frame_id.clear(),
            3 => invalid.entity_name.clear(),
            4 => invalid.config.angle_min = f64::NAN,
            5 => invalid.config.angle_max = invalid.config.angle_min,
            6 => invalid.config.angle_max = invalid.config.angle_min + 7.0,
            7 => invalid.config.range_min = -1.0,
            8 => invalid.config.range_max = invalid.config.range_min,
            9 => invalid.angle_step = 0.125,
            _ => {
                invalid.config.samples = 128;
                invalid.config.angle_min = 9_007_199_254_740_992.0;
                invalid.config.angle_max = 9_007_199_254_740_994.0;
                invalid.angle_step = 2.0 / 127.0;
            }
        }
        assert!(packet(&invalid, &sample(), 0).is_none(), "{invalid:?}");
    }
    for ranges in [
        vec![],
        vec![1.0; 2],
        vec![0.01, 1.0, 1.0],
        vec![1.0, 11.0, 1.0],
        vec![1.0, f64::NAN, 1.0],
        vec![1.0, f64::NEG_INFINITY, 1.0],
    ] {
        assert!(packet(&base, &LidarSample { ranges, ..sample() }, 0).is_none());
    }
    for (translation, rotation) in [
        ([f64::NAN, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        ([0.0; 3], [f64::INFINITY; 4]),
        ([0.0; 3], [0.0; 4]),
    ] {
        assert!(
            packet(
                &base,
                &LidarSample {
                    world_pose: WorldPose {
                        translation,
                        rotation
                    },
                    ..sample()
                },
                0
            )
            .is_none()
        );
    }
    assert!(
        packet(
            &base,
            &LidarSample {
                time: Duration::from_secs(u64::MAX),
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
        ("connect/endpoints", &connect),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(path, value).unwrap();
    }
    config
}

fn server(xml: &str) -> (Server, zenoh::Session, String) {
    let observer = zenoh::open(config(None)).wait().unwrap();
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
        "server-lidar.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let options = ServerOptions {
        partition: format!("lidar-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    };
    let key = options.topic_key("/m7/scan").unwrap();
    let server = Server::with_zenoh_config(
        simulation,
        "default",
        options,
        config(Some(&locators[0])),
        NonZeroUsize::new(4).unwrap(),
    )
    .unwrap();
    (server, observer, key)
}

#[test]
fn real_zenoh_scan_cadence_pause_reset_and_removal_preserve_canonical_contract() {
    let (mut server, observer, key) = server(XML);
    let subscriber = observer.declare_subscriber(key).wait().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while !server.lidars[0].publisher.matching().unwrap() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    let sensor_id = server.lidars[0].identity;
    server.set_paused(false);
    for tick in 1..=20 {
        let info = server.update().unwrap();
        assert_eq!(info.sim_time, Duration::from_millis(tick));
        if [1, 10, 20].contains(&tick) {
            let received = subscriber
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap();
            assert_eq!(
                received.attachment().unwrap().to_bytes().as_ref(),
                b"gz.msgs.LaserScan"
            );
            let message = LaserScan::decode(received.payload().to_bytes().as_ref()).unwrap();
            let sequence = [1, 10, 20].iter().position(|&value| value == tick).unwrap();
            let mut expected = sample();
            expected.time = info.sim_time;
            expected.epoch = info.epoch;
            expected.world_pose = WorldPose {
                translation: [0.0, 0.0, 1.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            };
            expected.ranges = vec![
                4.0 * std::f64::consts::SQRT_2,
                4.0,
                4.0 * std::f64::consts::SQRT_2,
            ];
            let expected = packet(&metadata(), &expected, sequence as u64).unwrap();
            assert_eq!(message.header, expected.header);
            assert_eq!(message.frame, expected.frame);
            assert_eq!(message.world_pose, expected.world_pose);
            assert_eq!((message.count, message.vertical_count), (3, 1));
            assert_eq!(
                (message.angle_min, message.angle_max, message.angle_step),
                (expected.angle_min, expected.angle_max, expected.angle_step)
            );
            assert_eq!((message.range_min, message.range_max), (0.1, 10.0));
            for (actual, expected) in message.ranges.iter().zip(&expected.ranges) {
                assert!((actual - expected).abs() < 1e-12);
            }
            assert_eq!(message.ranges.len(), 3);
            assert_eq!(message.intensities, [0.0; 3]);
        }
    }
    assert_eq!(server.counters().lidars_queued, 3);
    server.set_paused(true);
    for _ in 0..3 {
        assert!(server.update().unwrap().dt.is_zero());
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
    let reset = server.update().unwrap();
    assert_eq!((reset.epoch, reset.sim_time), (1, Duration::ZERO));
    assert_eq!(server.lidars[0].identity, sensor_id);
    assert!(subscriber.try_recv().unwrap().is_none());
    server.set_paused(false);
    let info = server.update().unwrap();
    assert_eq!((info.epoch, info.sim_time), (1, Duration::from_millis(1)));
    let message = LaserScan::decode(
        subscriber
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(message.header.as_ref().unwrap().data[1].value, ["0"]);
    assert_eq!(
        message.header.as_ref().unwrap().stamp,
        Some(gz_sim_proto::msgs::Time {
            sec: 0,
            nsec: 1_000_000
        })
    );
    let model_id = server
        .simulation
        .world()
        .iter_entities()
        .find(|e| e.get::<SceneName>().is_some_and(|n| n.0 == "default::rig"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let lease = Arc::downgrade(&server.lidars[0].publisher);
    remove_model(&mut server.simulation, model_id).unwrap();
    server.set_paused(true);
    server.update().unwrap();
    assert!(server.lidars.is_empty() && server.lidars.capacity() == 0);
    assert!(lease.upgrade().is_none());
    assert!(subscriber.try_recv().unwrap().is_none());
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    let restored = server.update().unwrap();
    assert_eq!((restored.epoch, restored.sim_time), (2, Duration::ZERO));
    assert_eq!(server.lidars[0].identity, sensor_id);
    let deadline = Instant::now() + Duration::from_secs(5);
    while !server.lidars[0].publisher.matching().unwrap() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    server.set_paused(false);
    server.update().unwrap();
    let message = LaserScan::decode(
        subscriber
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(message.header.as_ref().unwrap().data[1].value, ["0"]);
    assert_eq!(
        message.header.as_ref().unwrap().stamp,
        Some(gz_sim_proto::msgs::Time {
            sec: 0,
            nsec: 1_000_000
        })
    );
    assert_eq!(
        (
            server.counters().lidar_failures,
            server.counters().invalid_lidar_samples
        ),
        (0, 0)
    );
    server.shutdown();
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}

#[test]
fn viewer_and_zenoh_demand_are_unioned_and_absent_lidar_has_no_stream_state() {
    let (mut idle, _, _) = server("<sdf version='1.12'><world name='default'/></sdf>");
    idle.set_lidar_viewer_interest(true);
    idle.set_paused(false);
    idle.update().unwrap();
    assert!(idle.lidars.is_empty() && idle.lidars.capacity() == 0);
    assert_eq!(idle.counters().lidars_queued, 0);
    let (mut server, _, _) = server(XML);
    server.set_paused(false);
    for _ in 0..20 {
        server.update().unwrap();
    }
    assert_eq!(server.counters().lidars_queued, 0);
    assert!(
        server
            .simulation
            .world()
            .get::<LidarSample>(server.lidars[0].entity)
            .is_none()
    );
    server.set_lidar_viewer_interest(true);
    server.update().unwrap();
    assert_eq!(server.counters().lidars_queued, 1);
    server.set_lidar_viewer_interest(false);
    for _ in 0..8 {
        server.update().unwrap();
    }
    assert_eq!(server.counters().lidars_queued, 1);
    server.set_interest_policy(InterestPolicy::Always);
    server.update().unwrap();
    assert_eq!(server.counters().lidars_queued, 2);
    server.world_io.session.close().wait().unwrap();
    server.set_interest_policy(InterestPolicy::Subscribers);
    server.refresh_lidar_interest();
    assert_eq!(server.counters().interest_failures, 1);
}

#[test]
fn malformed_public_ownership_and_cross_type_endpoints_are_refused() {
    let (host, _, _) = server("<sdf version='1.12'><world name='default'/></sdf>");
    for field in 0..5 {
        let mut simulation = load_world_with_sdf_step(
            XML,
            "lidar.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let entity = simulation
            .world()
            .iter_entities()
            .find(|entity| entity.contains::<LidarPublication>())
            .unwrap()
            .id();
        match field {
            0 => {
                simulation
                    .world_mut()
                    .entity_mut(entity)
                    .remove::<EntityId>();
            }
            1 => {
                simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(EntityKind::Model);
            }
            2 => {
                simulation
                    .world_mut()
                    .get_mut::<LidarPublication>(entity)
                    .unwrap()
                    .frame_id
                    .clear();
            }
            3 => {
                simulation.world_mut().spawn((metadata(),));
            }
            _ => {
                simulation
                    .world_mut()
                    .get_mut::<LidarPublication>(entity)
                    .unwrap()
                    .topic = "/world/default/clock".into();
            }
        }
        let result = streams(&simulation, &host.world_io, &host.network_config, &[], None);
        assert!(matches!(
            result,
            Err(Error::LidarIdentity | Error::WorldNetwork(_))
        ));
    }
    let mut simulation = load_world_with_sdf_step(
        XML,
        "lidar.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let entity = simulation
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<LidarPublication>())
        .unwrap()
        .id();
    simulation
        .world_mut()
        .get_mut::<LidarPublication>(entity)
        .unwrap()
        .topic
        .clear();
    assert!(matches!(
        streams(&simulation, &host.world_io, &host.network_config, &[], None),
        Err(Error::Setup(_))
    ));
}

#[test]
fn invalid_samples_deduplicate_and_bounded_failure_still_advances_attempt_sequence() {
    let (mut server, _, _) = server(XML);
    let entity = server.lidars[0].entity;
    server.simulation.step_exact(20).unwrap();
    let mut invalid = sample();
    invalid.time = Duration::from_millis(20);
    invalid.epoch = 0;
    invalid.ranges[1] = f64::NAN;
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(invalid);
    server.publish_lidars();
    server.publish_lidars();
    assert_eq!(server.counters().invalid_lidar_samples, 1);
    for (epoch, time) in [
        (1, Duration::from_millis(20)),
        (0, Duration::from_millis(21)),
    ] {
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(LidarSample {
                epoch,
                time,
                ..sample()
            });
        server.publish_lidars();
    }
    assert_eq!(server.counters().invalid_lidar_samples, 3);
    assert_eq!(server.lidars[0].sequence, 0);
    server.simulation.step_exact(10).unwrap();
    server.world_io.max_message_bytes = 1;
    server.publish_lidars();
    assert_eq!(server.counters().lidar_failures, 1);
    assert_eq!(server.lidars[0].sequence, 1);
    server.world_io.max_message_bytes = 16 * 1024 * 1024;
    server.simulation.step_exact(10).unwrap();
    server.lidars[0].sequence = u64::MAX;
    server.publish_lidars();
    assert_eq!(server.counters().lidars_queued, 1);
    assert_eq!(server.lidars[0].sequence, 0);
    let lease = Arc::downgrade(&server.lidars[0].publisher);
    server
        .simulation
        .world_mut()
        .get_mut::<LidarPublication>(entity)
        .unwrap()
        .topic = "/changed".into();
    server.publish_lidars();
    assert_eq!(server.counters().lidars_queued, 1);
    server.retire_lidars();
    assert!(lease.upgrade().is_none() && server.lidars.capacity() == 0);
}

#[test]
fn reset_metadata_drift_is_terminal_and_drops_prepared_lidar_leases() {
    fn change(mut metadata: bevy_ecs::prelude::Query<&mut LidarPublication>) {
        for mut metadata in &mut metadata {
            metadata.frame_id = "changed".into();
        }
    }
    let (mut server, _, _) = server(XML);
    let lease = Arc::downgrade(&server.lidars[0].publisher);
    server
        .simulation
        .add_systems(gz_sim_core::Phase::Reset, change);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(server.update(), Err(Error::ResetLidars)));
    assert!(server.reset_failed);
    assert!(server.lidars.is_empty() && lease.upgrade().is_none());
    assert!(matches!(server.update(), Err(Error::ResetStreams)));
}

#[test]
fn runtime_kind_mutation_withholds_a_fresh_scan_and_retires_the_original_lease() {
    let (mut server, observer, key) = server(XML);
    let subscriber = observer.declare_subscriber(key).wait().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while !server.lidars[0].publisher.matching().unwrap() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    server.set_paused(false);
    server.update().unwrap();
    let first = subscriber
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap();
    let first = LaserScan::decode(first.payload().to_bytes().as_ref()).unwrap();
    assert_eq!(
        first.header.unwrap().stamp,
        Some(gz_sim_proto::msgs::Time {
            sec: 0,
            nsec: 1_000_000
        })
    );
    let entity = server.lidars[0].entity;
    let original = Arc::downgrade(&server.lidars[0].publisher);
    let before = server.counters().lidars_queued;
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(EntityKind::Model);
    // Produce the next real due sample before exercising the publication guard.
    // Duplicate suppression at 1 ms must not be what prevents another frame.
    server.simulation.step_exact(9).unwrap();
    assert_eq!(
        server
            .simulation
            .world()
            .get::<LidarSample>(entity)
            .unwrap()
            .time,
        Duration::from_millis(10)
    );
    server.publish_lidars();
    assert_eq!(server.counters().lidars_queued, before);
    assert!(subscriber.try_recv().unwrap().is_none());
    server.retire_lidars();
    assert!(server.lidars.is_empty() && server.lidars.capacity() == 0);
    assert!(original.upgrade().is_none());
}
