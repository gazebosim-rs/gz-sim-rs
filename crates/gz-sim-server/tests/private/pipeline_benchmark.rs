// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_msgs::msgs::Imu;
use gz_sim_core::EntityId;
use gz_sim_scene::{
    Name, PhysicsEngine, WorldPose, load_world_with_sdf_step, queue_link_world_velocity,
};
use sdformat::NoResources;

fn check(message: &Imu, sequence: u64, metadata: &ImuPublication) -> (u64, f64) {
    let header = message.header.as_ref().unwrap();
    let stamp = header.stamp.as_ref().unwrap();
    let nanos = stamp.sec as u64 * 1_000_000_000 + stamp.nsec as u64;
    assert_eq!(nanos, (sequence + 1) * 1_000_000);
    assert_eq!(message.entity_name, metadata.entity_name);
    assert_eq!(header.data.len(), 2);
    let frame = header
        .data
        .iter()
        .find(|entry| entry.key == "frame_id")
        .unwrap();
    let seq = header.data.iter().find(|entry| entry.key == "seq").unwrap();
    assert_eq!(frame.value.len(), 1);
    assert_eq!(frame.value[0], metadata.frame_id);
    assert_eq!(seq.value.len(), 1);
    assert_eq!(seq.value[0], sequence.to_string());
    let q = message.orientation.as_ref().unwrap();
    assert_eq!([q.x, q.y, q.z, q.w], [0.0, 0.0, 0.0, 1.0]);
    let w = message.angular_velocity.as_ref().unwrap();
    assert_eq!([w.x, w.y, w.z], [0.0; 3]);
    let a = message.linear_acceleration.as_ref().unwrap();
    let expected_x = if sequence == 0 { 1000.0 } else { 0.0 };
    assert!((a.x - expected_x).abs() < 1e-9);
    assert_eq!(a.y, 0.0);
    assert!((a.z - 9.81).abs() < 1e-9);
    for covariance in [
        &message.orientation_covariance,
        &message.angular_velocity_covariance,
        &message.linear_acceleration_covariance,
    ] {
        let data = &covariance.as_ref().unwrap().data;
        assert_eq!(data.len(), 9);
        assert!(data.iter().all(|value| *value == 0.0));
    }
    (nanos, a.z)
}

#[test]
fn tpe_imu_process_local_pipeline() {
    let steps: u64 = std::env::var("GZ_PIPELINE_STEPS")
        .unwrap_or_else(|_| "64".into())
        .parse()
        .unwrap();
    assert!((1..=1000).contains(&steps));
    let mut sim = load_world_with_sdf_step(
        include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
        "headless_imu.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    match std::env::var("GZ_PIPELINE_EXECUTOR")
        .as_deref()
        .unwrap_or("default")
    {
        "default" => {}
        "parallel" => sim.set_step_executor(bevy_ecs::schedule::MultiThreadedExecutor::new()),
        "single" => sim.set_step_executor(bevy_ecs::schedule::SingleThreadedExecutor::new()),
        _ => panic!("invalid pipeline executor"),
    }
    let body = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::ball::body")
        })
        .unwrap();
    let body_entity = body.id();
    let body_id = body.get::<EntityId>().unwrap().get();
    queue_link_world_velocity(&mut sim, body_id, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
    let (entity, metadata) = sim
        .world()
        .iter_entities()
        .find_map(|entity| {
            entity
                .get::<ImuPublication>()
                .map(|m| (entity.id(), m.clone()))
        })
        .unwrap();
    assert_eq!(metadata.entity_name, "ball::body::imu");
    assert_eq!(metadata.frame_id, "ball::body::imu");
    let mut node = Node::new(Config {
        partition: format!(
            "pipeline-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ),
        interface: std::net::Ipv4Addr::LOCALHOST,
        ..Config::default()
    })
    .unwrap();
    let subscription = node.subscribe(&metadata.topic, "gz.msgs.IMU").unwrap();
    let publisher = node.advertise(&metadata.topic, "gz.msgs.IMU").unwrap();
    sim.step_exact(1).unwrap();
    let first = imu_packet(&metadata, sim.world().get::<ImuSample>(entity).unwrap(), 0).unwrap();
    publisher.publish(first.encode_to_vec()).unwrap();
    let first = subscription.recv_timeout(Duration::from_secs(5)).unwrap();
    let first_message = Imu::decode(first.data.as_slice()).unwrap();
    let warmup_force_x = first_message.linear_acceleration.as_ref().unwrap().x;
    check(&first_message, 0, &metadata);
    let start = Instant::now();
    let mut sent_bytes = 0;
    for sequence in 1..=steps {
        sim.step_exact(1).unwrap();
        let pose = sim.world().get::<WorldPose>(body_entity).unwrap();
        assert!((pose.translation[0] - (sequence + 1) as f64 * 0.001).abs() < 1e-9);
        assert_eq!([pose.translation[1], pose.translation[2]], [0.0, 5.0]);
        let packet = imu_packet(
            &metadata,
            sim.world().get::<ImuSample>(entity).unwrap(),
            sequence,
        )
        .unwrap();
        let bytes = packet.encode_to_vec();
        sent_bytes += bytes.len();
        publisher.publish(bytes).unwrap();
    }
    let mut received_bytes = 0;
    let mut time_sum = 0;
    let mut force_sum = 0.0;
    for sequence in 1..=steps {
        let received = subscription.recv_timeout(Duration::from_secs(5)).unwrap();
        received_bytes += received.data.len();
        let message = Imu::decode(received.data.as_slice()).unwrap();
        let (time, force) = check(&message, sequence, &metadata);
        time_sum += time;
        force_sum += force;
    }
    let nanos = start.elapsed().as_nanos();
    assert_eq!(received_bytes, sent_bytes);
    assert!(matches!(
        subscription.recv_timeout(Duration::from_millis(1)),
        Err(TransportError::Timeout)
    ));
    assert_eq!(sim.elapsed(), Duration::from_millis(steps + 1));
    let final_x = sim
        .world()
        .get::<WorldPose>(body_entity)
        .unwrap()
        .translation[0];
    let fixture_hex: String =
        include_bytes!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf")
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect();
    node.shutdown();
    println!(
        "PIPELINE {{\"steps\":{steps},\"nanos\":{nanos},\"logical_bytes\":{received_bytes},\"time_sum\":{time_sum},\"force_sum\":{force_sum:.17},\"final_x\":{final_x:.17},\"warmup_force_x\":{warmup_force_x:.17},\"fixture_hex\":\"{fixture_hex}\"}}"
    );
}
