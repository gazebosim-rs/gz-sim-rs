// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn native_interest_server() -> Server {
    configured_interest_server(false)
}

fn configured_interest_server(noisy: bool) -> Server {
    let fixture = include_str!("../../../gz-sim-scene/tests/fixtures/imu_interest.sdf");
    let xml = if noisy {
        fixture.replace("</sensor>", "<imu><angular_velocity><x><noise type='gaussian'><stddev>0.1</stddev><dynamic_bias_stddev>0.7</dynamic_bias_stddev><dynamic_bias_correlation_time>2</dynamic_bias_correlation_time></noise></x></angular_velocity></imu></sensor>")
    } else {
        fixture.to_owned()
    };
    let sim = gz_sim_scene::load_world_with_engine(
        &xml,
        "activation.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::Tpe,
    )
    .unwrap();
    let mut server = Server::new(sim, "default", config(), NonZeroUsize::new(2).unwrap()).unwrap();
    assert_eq!(server.interest_policy(), InterestPolicy::Subscribers);
    assert!(server.apply_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    server
}

fn received_imu(subscription: &gz_transport::Subscription, milliseconds: i32, sequence: u64) {
    let received = subscription.recv_timeout(Duration::from_secs(5)).unwrap();
    let message = gz_msgs::msgs::Imu::decode(received.data.as_slice()).unwrap();
    let header = message.header.unwrap();
    assert_eq!(
        header.stamp.unwrap(),
        Time {
            sec: 0,
            nsec: milliseconds * 1_000_000
        }
    );
    assert_eq!(
        header
            .data
            .iter()
            .find(|entry| entry.key == "seq")
            .unwrap()
            .value,
        [sequence.to_string()]
    );
    assert_eq!(message.linear_acceleration.unwrap().z, 9.81);
    assert!(matches!(
        subscription.recv_timeout(Duration::from_millis(1)),
        Err(TransportError::Timeout)
    ));
}

#[test]
fn native_late_and_shared_interest_preserve_unobserved_sequences_and_idle_phase() {
    for shared in [false, true] {
        let mut server = native_interest_server();
        let slow = shared.then(|| {
            server
                .node
                .subscribe("/activation/slow", "gz.msgs.IMU")
                .unwrap()
        });
        let initial = if shared { 100 } else { 50 };
        for _ in 0..initial {
            server.tick().unwrap();
        }
        if let Some(slow) = &slow {
            let first = slow.recv_timeout(Duration::from_secs(5)).unwrap();
            let message = gz_msgs::msgs::Imu::decode(first.data.as_slice()).unwrap();
            let header = message.header.unwrap();
            assert_eq!(header.stamp.unwrap().nsec, 1_000_000);
            assert_eq!(
                header
                    .data
                    .iter()
                    .find(|entry| entry.key == "seq")
                    .unwrap()
                    .value,
                ["0"]
            );
            received_imu(slow, 100, 1);
            // The unconnected fast sensor also acquired twice under the native world gate.
            assert_eq!(server.counters().imus_queued, 4);
        } else {
            assert_eq!(server.counters().imus_queued, 0);
            assert!(
                server
                    .simulation()
                    .world()
                    .iter_entities()
                    .all(|e| !e.contains::<ImuSample>())
            );
        }
        let fast = server
            .node
            .subscribe("/activation/fast", "gz.msgs.IMU")
            .unwrap();
        server.tick().unwrap();
        received_imu(&fast, initial + 1, if shared { 2 } else { 0 });
        let before = server.counters().imus_queued;
        drop(slow);
        drop(fast);
        for _ in 0..10 {
            server.tick().unwrap();
        }
        assert_eq!(server.counters().imus_queued, before);
        let fast = server
            .node
            .subscribe("/activation/fast", "gz.msgs.IMU")
            .unwrap();
        server.tick().unwrap();
        received_imu(&fast, initial + 12, if shared { 3 } else { 1 });
        assert_eq!(server.counters().pose_failures, 0);
        assert_eq!(server.counters().poses_queued, 0);
        assert_eq!(server.counters().interest_failures, 0);
    }
}

#[test]
fn uninterested_poses_neither_consume_nor_reset_the_wall_slot() {
    let mut server = native_interest_server();
    server.set_pose_publication_period(Duration::from_secs(86_400));
    server.tick().unwrap();
    assert_eq!(server.counters().poses_unobserved, 1);
    let poses = server
        .node
        .subscribe("/world/default/pose/info", "gz.msgs.Pose_V")
        .unwrap();
    server.tick().unwrap();
    poses.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(server.counters().poses_queued, 1);
    drop(poses);
    server.tick().unwrap();
    assert_eq!(server.counters().poses_unobserved, 2);
    let poses = server
        .node
        .subscribe("/world/default/pose/info", "gz.msgs.Pose_V")
        .unwrap();
    server.tick().unwrap();
    assert!(matches!(
        poses.recv_timeout(Duration::from_millis(1)),
        Err(TransportError::Timeout)
    ));
    assert_eq!(server.counters().poses_queued, 1);
    assert_eq!(server.counters().poses_throttled, 1);
    assert!(server.apply_control(WorldControl {
        pause: true,
        reset: Some(gz_msgs::msgs::WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.tick().unwrap();
    assert_eq!(server.counters().poses_throttled, 2);
    assert_eq!(server.counters().poses_queued, 1);
}

#[test]
fn stopped_interest_is_observable_and_does_not_stop_physics() {
    let mut server = native_interest_server();
    server.node.shutdown();
    assert!(matches!(
        server.tick(),
        Err(Error::Transport(TransportError::Shutdown))
    ));
    assert_eq!(server.simulation().elapsed(), Duration::from_millis(1));
    assert_eq!(server.counters().interest_failures, 3);
    assert_eq!(server.counters().poses_unobserved, 0);
    assert_eq!(server.counters().imus_queued, 0);
    assert_eq!(server.counters().clock_failures, 1);
}

#[test]
fn idle_ticks_preserve_noise_stream_and_use_the_full_acquisition_gap() {
    let mut server = configured_interest_server(true);
    for _ in 0..50 {
        server.tick().unwrap();
    }
    assert_eq!(server.counters().imus_queued, 0);
    let fast = server
        .node
        .subscribe("/activation/fast", "gz.msgs.IMU")
        .unwrap();
    let identity = server
        .simulation()
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<ImuPublication>()
                .is_some_and(|p| p.topic == "/activation/fast")
        })
        .unwrap()
        .get::<gz_sim_core::EntityId>()
        .unwrap()
        .get();
    let mut random = gz_sensors::NoiseStream::new([0; 32], identity);
    let mut oracle = gz_sensors::GaussianNoise::new(
        gz_sensors::GaussianParameters {
            stddev: 0.1,
            dynamic_bias_stddev: 0.7,
            dynamic_bias_correlation_time: 2.0,
            ..Default::default()
        },
        &mut random,
    )
    .unwrap();
    server.tick().unwrap();
    let raw = fast.recv_timeout(Duration::from_secs(5)).unwrap();
    let first = gz_msgs::msgs::Imu::decode(raw.data.as_slice()).unwrap();
    assert_eq!(first.header.unwrap().stamp.unwrap().nsec, 51_000_000);
    assert_eq!(
        first.angular_velocity.unwrap().x,
        oracle.apply(0.0, 0.0, &mut random).unwrap()
    );
    drop(fast);
    let queued = server.counters().imus_queued;
    for _ in 0..10 {
        server.tick().unwrap();
    }
    assert_eq!(server.counters().imus_queued, queued);
    let fast = server
        .node
        .subscribe("/activation/fast", "gz.msgs.IMU")
        .unwrap();
    server.tick().unwrap();
    let raw = fast.recv_timeout(Duration::from_secs(5)).unwrap();
    let second = gz_msgs::msgs::Imu::decode(raw.data.as_slice()).unwrap();
    assert_eq!(second.header.unwrap().stamp.unwrap().nsec, 62_000_000);
    assert_eq!(
        second.angular_velocity.unwrap().x,
        oracle
            .apply(0.0, (11_000_000u64 as f32 / 1e9f32) as f64, &mut random)
            .unwrap()
    );
}

#[test]
fn separate_node_receives_first_imu_frame_directly_and_once_after_reset() {
    for engine in [
        gz_sim_scene::PhysicsEngine::Rapier,
        gz_sim_scene::PhysicsEngine::Tpe,
    ] {
        let settings = config();
        let sim = gz_sim_scene::load_world_with_engine(
            include_str!("../../../gz-sim-scene/tests/fixtures/imu_interest.sdf"),
            "separate-node.sdf",
            "default",
            Duration::from_millis(1),
            &sdformat::NoResources,
            engine,
        )
        .unwrap();
        let mut server = Server::new(
            sim,
            "default",
            settings.clone(),
            NonZeroUsize::new(2).unwrap(),
        )
        .unwrap();
        let mut client = Node::new(settings.clone()).unwrap();
        let subscription = client.subscribe("/activation/fast", "gz.msgs.IMU").unwrap();
        let publisher = &server
            .imus
            .iter()
            .find(|stream| stream.metadata.topic == "/activation/fast")
            .unwrap()
            .publisher;
        let deadline = Instant::now() + Duration::from_secs(5);
        while !publisher.connection_snapshot().unwrap() {
            assert!(
                Instant::now() < deadline,
                "separate Node registration did not arrive"
            );
            thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(server.simulation().elapsed(), Duration::ZERO);
        let mut first = None;
        for epoch in 0..2 {
            assert!(server.apply_control(WorldControl {
                pause: true,
                multi_step: 1,
                ..Default::default()
            }));
            server.tick().unwrap();
            assert_eq!(server.simulation().elapsed(), Duration::from_millis(1));
            let received = subscription
                .recv_with_info_timeout(Duration::from_secs(5))
                .unwrap();
            assert!(
                received.info().intra_process,
                "another Node in this process must use native direct delivery"
            );
            assert_eq!(
                received.info().partition,
                format!("/{}", settings.partition)
            );
            assert_eq!(received.info().topic, "/activation/fast");
            assert_eq!(received.info().type_name, "gz.msgs.IMU");
            let frame = gz_msgs::msgs::Imu::decode(received.data.as_slice()).unwrap();
            let header = frame.header.as_ref().unwrap();
            assert_eq!(
                header.stamp,
                Some(Time {
                    sec: 0,
                    nsec: 1_000_000
                })
            );
            assert_eq!(header.data.len(), 2);
            assert_eq!(header.data[0].key, "frame_id");
            assert_eq!(header.data[0].value, ["ball::body::fast"]);
            assert_eq!(header.data[1].key, "seq");
            assert_eq!(header.data[1].value, ["0"]);
            assert_eq!(frame.entity_name, "ball::body::fast");
            let orientation = frame.orientation.as_ref().unwrap();
            assert_eq!(
                [orientation.x, orientation.y, orientation.z, orientation.w],
                [0.0, 0.0, 0.0, 1.0]
            );
            let gyro = frame.angular_velocity.as_ref().unwrap();
            assert_eq!([gyro.x, gyro.y, gyro.z], [0.0; 3]);
            let accel = frame.linear_acceleration.as_ref().unwrap();
            assert_eq!([accel.x, accel.y], [0.0; 2]);
            assert!(
                (accel.z
                    - if engine == gz_sim_scene::PhysicsEngine::Tpe {
                        9.81
                    } else {
                        0.0
                    })
                .abs()
                    < 1e-10
            );
            for covariance in [
                &frame.orientation_covariance,
                &frame.angular_velocity_covariance,
                &frame.linear_acceleration_covariance,
            ] {
                assert!(covariance.as_ref().unwrap().header.is_none());
                assert_eq!(covariance.as_ref().unwrap().data, [0.0; 9]);
            }
            if let Some(first) = &first {
                assert_eq!(&frame, first);
            } else {
                first = Some(frame);
            }
            assert!(matches!(
                subscription.recv_timeout(Duration::from_millis(50)),
                Err(TransportError::Timeout)
            ));
            assert_eq!(server.counters().imu_failures, 0);
            assert_eq!(server.counters().invalid_imu_samples, 0);
            assert_eq!(server.counters().interest_failures, 0);
            assert_eq!(
                server.simulation().world().resource::<UpdateInfo>().epoch,
                epoch
            );
            assert!(server.apply_control(WorldControl {
                pause: true,
                reset: Some(gz_msgs::msgs::WorldReset {
                    all: true,
                    ..Default::default()
                }),
                ..Default::default()
            }));
            server.tick().unwrap();
            assert_eq!(server.simulation().elapsed(), Duration::ZERO);
        }
        assert_eq!(server.transport_statistics().routing_errors, 0);
        server.shutdown();
        client.shutdown();
    }
}
