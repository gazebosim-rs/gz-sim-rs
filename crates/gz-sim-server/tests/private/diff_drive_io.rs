// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::time::{Duration, Instant};

fn session() -> zenoh::Session {
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
    zenoh::open(config).wait().unwrap()
}
fn keys() -> (String, String) {
    let prefix = format!("@/drive-{}@", uuid::Uuid::new_v4());
    (format!("{prefix}/cmd"), format!("{prefix}/odom"))
}
fn setup(command_key: String, odometry_key: String, input: mpsc::SyncSender<Command>) -> Setup {
    Setup {
        command_key,
        odometry_key,
        input,
        max_bytes: 512,
        drops: Arc::new(AtomicU64::new(0)),
        epoch: Arc::new(AtomicU64::new(7)),
    }
}
fn twist(linear: f64, angular: f64) -> Twist {
    Twist {
        linear: Some(crate::vector3([linear, 0.0, 0.0])),
        angular: Some(crate::vector3([0.0, 0.0, angular])),
        ..Default::default()
    }
}
fn wait(mut predicate: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(3);
    while !predicate() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
}

#[test]
fn planar_twist_domain_rejects_every_unsupported_axis_and_nonfinite_value() {
    assert_eq!(command(Twist::default()), Some([0.0, 0.0]));
    assert_eq!(command(twist(-2.0, 0.5)), Some([-2.0, 0.5]));
    for index in 0..6 {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut values = [0.0; 6];
            values[index] = value;
            assert_eq!(
                command(Twist {
                    linear: Some(crate::vector3([values[0], values[1], values[2]])),
                    angular: Some(crate::vector3([values[3], values[4], values[5]])),
                    ..Default::default()
                }),
                None
            );
        }
    }
    for index in [1, 2, 3, 4] {
        let mut values = [0.0; 6];
        values[index] = 0.001;
        assert_eq!(
            command(Twist {
                linear: Some(crate::vector3([values[0], values[1], values[2]])),
                angular: Some(crate::vector3([values[3], values[4], values[5]])),
                ..Default::default()
            }),
            None
        );
    }
}

#[test]
fn actual_canonical_input_bounds_output_and_retired_callback_capture_share_no_cpu_owner() {
    let session = session();
    let (cmd, odom) = keys();
    let publisher = session.declare_publisher(cmd.clone()).wait().unwrap();
    let monitor = session.declare_subscriber(cmd.clone()).wait().unwrap();
    let observer = session.declare_subscriber(odom.clone()).wait().unwrap();
    let (input, receiver) = mpsc::sync_channel(1);
    let setup = setup(cmd, odom, input);
    let drops = Arc::clone(&setup.drops);
    let epoch = Arc::clone(&setup.epoch);
    let (mut registry, output) = Declarations::new(&session, setup, "process", "node").unwrap();
    wait(|| output.matching().unwrap());
    let bytes = twist(2.0, -0.5).encode_to_vec();
    publisher
        .put(bytes.clone())
        .attachment(Twist::full_name())
        .wait()
        .unwrap();
    let captured_sample = monitor
        .recv_timeout(Duration::from_secs(3))
        .unwrap()
        .unwrap();
    assert_eq!(
        receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
        (7, Some([2.0, -0.5]))
    );
    for (data, attachment) in [
        (vec![255], Twist::full_name()),
        (bytes.clone(), "gz.msgs.Double".into()),
        (vec![0; 513], Twist::full_name()),
        (twist(f64::NAN, 0.0).encode_to_vec(), Twist::full_name()),
    ] {
        publisher.put(data).attachment(attachment).wait().unwrap();
        assert_eq!(
            receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
            (7, None)
        );
    }
    publisher.put(bytes.clone()).wait().unwrap();
    assert_eq!(
        receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
        (7, None)
    );
    epoch.store(8, Ordering::Release);
    for _ in 0..2 {
        publisher
            .put(bytes.clone())
            .attachment(Twist::full_name())
            .wait()
            .unwrap();
    }
    wait(|| drops.load(Ordering::Relaxed) == 1);
    assert_eq!(
        receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
        (8, Some([2.0, -0.5]))
    );
    let frame = Odometry::default().encode_to_vec();
    assert!(registry.publish(Job::new(&output, frame.clone())).unwrap());
    let sample = observer
        .recv_timeout(Duration::from_secs(3))
        .unwrap()
        .unwrap();
    assert_eq!(sample.payload().to_bytes().as_ref(), frame);
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.Odometry"
    );
    drop(observer);
    wait(|| !output.matching().unwrap());
    let held = Arc::clone(&registry.state);
    let weak = Arc::downgrade(&output);
    let job = Job::new(&output, frame);
    drop(output);
    assert!(weak.upgrade().is_none());
    assert!(!registry.publish(job).unwrap());
    assert!(!registry.reconcile(false));
    assert!(
        registry.publisher.is_none()
            && registry.subscriber.is_none()
            && registry.presence.is_empty()
    );
    held.receive(captured_sample);
    assert!(receiver.try_recv().is_err());
    drop(held);
    wait(|| registry.reconcile(false));
    session.close().wait().unwrap();
}

#[test]
fn failed_setup_closed_transport_and_mismatched_jobs_cannot_publish_or_retain_endpoints() {
    let session = session();
    let (cmd, odom) = keys();
    for max in [0, usize::MAX] {
        let (input, _) = mpsc::sync_channel(1);
        let mut setup = setup(cmd.clone(), odom.clone(), input);
        setup.max_bytes = max;
        assert!(Declarations::new(&session, setup, "process", "node").is_err());
    }
    let (input, _) = mpsc::sync_channel(1);
    assert!(
        Declarations::new(
            &session,
            setup(cmd.clone(), cmd.clone(), input),
            "process",
            "node"
        )
        .is_err()
    );
    let (input, _) = mpsc::sync_channel(1);
    let (mut registry, output) = Declarations::new(
        &session,
        setup(cmd.clone(), odom.clone(), input),
        "process",
        "node",
    )
    .unwrap();
    let (input, _) = mpsc::sync_channel(1);
    let (second_cmd, second_odom) = keys();
    let (_other, other) = Declarations::new(
        &session,
        setup(second_cmd, second_odom, input),
        "process",
        "node",
    )
    .unwrap();
    assert!(!registry.publish(Job::new(&other, vec![])).unwrap());
    let mut bad_id = Job::new(&output, vec![]);
    bad_id.id = uuid::Uuid::nil();
    assert!(!registry.publish(bad_id).unwrap());
    session.close().wait().unwrap();
    assert!(registry.publish(Job::new(&output, vec![])).is_err());
    wait(|| registry.reconcile(true));
    assert!(output.matching().is_err());
    let (input, _) = mpsc::sync_channel(1);
    assert!(Declarations::new(&session, setup(cmd, odom, input), "process", "node").is_err());
}
