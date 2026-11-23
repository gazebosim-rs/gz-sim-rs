// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::time::{Duration, Instant};
fn session() -> zenoh::Session {
    let config = zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}"#).unwrap();
    zenoh::open(config).wait().unwrap()
}
fn setup(key: String, input: mpsc::SyncSender<Command>) -> Setup {
    Setup {
        key,
        input,
        max_bytes: 512,
        drops: Arc::new(AtomicU64::new(0)),
        epoch: Arc::new(AtomicU64::new(7)),
    }
}
fn twist(values: [f64; 6]) -> Twist {
    Twist {
        linear: Some(crate::vector3([values[0], values[1], values[2]])),
        angular: Some(crate::vector3([values[3], values[4], values[5]])),
        ..Default::default()
    }
}
fn wait(mut condition: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(3);
    while !condition() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
}
#[test]
fn six_axis_twist_admits_signed_values_and_missing_vectors_but_refuses_each_nonfinite_axis() {
    assert_eq!(command(Twist::default()), Some(([0.0; 3], [0.0; 3])));
    assert_eq!(
        command(twist([1.0, -2.0, 3.0, -4.0, 5.0, -6.0])),
        Some(([1.0, -2.0, 3.0], [-4.0, 5.0, -6.0]))
    );
    for axis in 0..6 {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut values = [0.0; 6];
            values[axis] = bad;
            assert_eq!(command(twist(values)), None);
        }
    }
}
#[test]
fn canonical_six_axis_callbacks_preserve_type_bounds_epoch_and_weak_retirement() {
    let session = session();
    let key = format!("@/m79-{}@/cmd", uuid::Uuid::new_v4());
    let publisher = session.declare_publisher(key.clone()).wait().unwrap();
    let monitor = session.declare_subscriber(key.clone()).wait().unwrap();
    let (input, receiver) = mpsc::sync_channel(1);
    let setup = setup(key.clone(), input);
    let epoch = Arc::clone(&setup.epoch);
    let drops = Arc::clone(&setup.drops);
    let (mut declaration, lease) = Declaration::new(&session, setup, "process", "node").unwrap();
    wait(|| publisher.matching_status().wait().unwrap().matching());
    let bytes = twist([1.0, -2.0, 3.0, -4.0, 5.0, -6.0]).encode_to_vec();
    publisher
        .put(bytes.clone())
        .attachment(Twist::full_name())
        .wait()
        .unwrap();
    let sample = monitor
        .recv_timeout(Duration::from_secs(3))
        .unwrap()
        .unwrap();
    assert_eq!(
        receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
        (7, Some(([1.0, -2.0, 3.0], [-4.0, 5.0, -6.0])))
    );
    for (bytes, attachment) in [
        (vec![255], Twist::full_name()),
        (vec![0; 513], Twist::full_name()),
        (bytes.clone(), "gz.msgs.Double".into()),
        (
            twist([f64::NAN, 0.0, 0.0, 0.0, 0.0, 0.0]).encode_to_vec(),
            Twist::full_name(),
        ),
    ] {
        publisher.put(bytes).attachment(attachment).wait().unwrap();
        assert_eq!(
            receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
            (7, None)
        );
    }
    publisher.put(bytes).wait().unwrap();
    assert_eq!(
        receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
        (7, None)
    );
    epoch.store(8, Ordering::Release);
    declaration.state.receive(sample.clone());
    declaration.state.receive(sample.clone());
    assert_eq!(drops.load(Ordering::Relaxed), 1);
    assert_eq!(receiver.recv_timeout(Duration::from_secs(3)).unwrap().0, 8);
    assert!(lease.matches(lease.identity(), &key));
    assert!(!lease.matches(uuid::Uuid::nil(), &key));
    assert!(!lease.matches(lease.identity(), "wrong"));
    let capture = Arc::clone(&declaration.state);
    let weak = Arc::downgrade(&lease);
    drop(lease);
    assert!(weak.upgrade().is_none());
    assert!(!declaration.reconcile(false));
    assert!(declaration.subscriber.is_none() && declaration.presence.is_none());
    capture.receive(sample);
    assert!(receiver.try_recv().is_err());
    drop(capture);
    wait(|| declaration.reconcile(false));
    session.close().wait().unwrap();
}
#[test]
fn invalid_velocity_setup_is_atomic_and_closed_session_invalidates_even_a_retained_lease() {
    let session = session();
    for (key, max) in [
        ("@/m79@/ok", 0),
        ("@/m79@/ok", usize::MAX),
        ("@/m79@/*", 512),
    ] {
        let (input, _receiver) = mpsc::sync_channel(1);
        let mut setup = setup(key.into(), input);
        setup.max_bytes = max;
        assert!(Declaration::new(&session, setup, "process", "node").is_err());
    }
    let (input, _receiver) = mpsc::sync_channel(1);
    let (mut declaration, lease) = Declaration::new(
        &session,
        setup("@/m79@/ok".into(), input),
        "process",
        "node",
    )
    .unwrap();
    session.close().wait().unwrap();
    wait(|| declaration.reconcile(true));
    assert!(!lease.matches(lease.identity(), "@/m79@/ok"));
    let (input, _receiver) = mpsc::sync_channel(1);
    assert!(
        Declaration::new(
            &session,
            setup("@/m79@/closed".into(), input),
            "process",
            "node"
        )
        .is_err()
    );
}

#[test]
fn velocity_family_keys_capacity_and_captured_retirement_preserve_other_owners() {
    let session = session();
    let prefix = format!("@/fleet-{}@", uuid::Uuid::new_v4());
    let mut family = Declarations::default();
    let mut leases = Vec::new();
    let mut receivers = Vec::new();
    for i in 0..8 {
        let (input, receiver) = mpsc::sync_channel(2);
        let key = format!("{prefix}/c{i}");
        let Installation::Ready(lease) = family
            .install(&session, setup(key, input), "process", "node")
            .unwrap()
        else {
            panic!("fresh key cannot be retiring")
        };
        leases.push(lease);
        receivers.push(receiver);
    }
    assert!(!family.is_empty());
    let key = format!("{prefix}/c0");
    let (input, _) = mpsc::sync_channel(1);
    assert!(
        matches!(family.install(&session, setup(key.clone(), input), "process", "node"),
        Err(error) if error == "velocity key already installed")
    );
    let (input, _) = mpsc::sync_channel(1);
    assert!(
        matches!(family.install(&session, setup(format!("{prefix}/ninth"), input), "process", "node"),
        Err(error) if error == "at most eight velocity inputs")
    );
    assert!(
        leases
            .iter()
            .enumerate()
            .all(|(i, lease)| lease.matches(lease.identity(), &format!("{prefix}/c{i}")))
    );
    // Keep the real callback capture after the lease dies. Reacquisition must
    // not replace this key until the capture is released, even at capacity.
    let captured = Arc::clone(&family.entries[&key].state);
    let old_id = leases[0].identity();
    drop(leases.remove(0));
    assert!(!family.reconcile(false));
    let (input, replacement) = mpsc::sync_channel(2);
    let Installation::Retiring(pending) = family
        .install(&session, setup(key.clone(), input), "process", "node")
        .unwrap()
    else {
        panic!("captured old callback must keep its key")
    };
    assert!(!captured.active.load(Ordering::Acquire));
    let other_key = format!("{prefix}/c1");
    let other = session.declare_publisher(other_key).wait().unwrap();
    wait(|| other.matching_status().wait().unwrap().matching());
    other
        .put(twist([2., 0., 0., 0., 0., 0.]).encode_to_vec())
        .attachment(Twist::full_name())
        .wait()
        .unwrap();
    assert_eq!(
        receivers[1].recv_timeout(Duration::from_secs(3)).unwrap(),
        (7, Some(([2., 0., 0.], [0.; 3])))
    );
    drop(captured);
    wait(|| {
        family.reconcile(false);
        !family.entries.contains_key(&key)
    });
    let Installation::Ready(current) = family
        .install(&session, pending, "process", "node")
        .unwrap()
    else {
        panic!("quiescent key must be reusable")
    };
    assert_ne!(current.identity(), old_id);
    let publisher = session.declare_publisher(key.clone()).wait().unwrap();
    wait(|| publisher.matching_status().wait().unwrap().matching());
    publisher
        .put(twist([-3., 0., 0., 0., 0., 0.]).encode_to_vec())
        .attachment(Twist::full_name())
        .wait()
        .unwrap();
    assert_eq!(
        replacement.recv_timeout(Duration::from_secs(3)).unwrap(),
        (7, Some(([-3., 0., 0.], [0.; 3])))
    );
    assert!(receivers[0].try_recv().is_err());
    family.begin_shutdown();
    assert!(!current.matches(current.identity(), &key));
    assert!(
        leases
            .iter()
            .all(|lease| !lease.active.load(Ordering::Acquire))
    );
    wait(|| family.reconcile(true));
    assert!(family.is_empty());
    let (input, _) = mpsc::sync_channel(1);
    let mut invalid = setup(format!("{prefix}/invalid"), input);
    invalid.max_bytes = 0;
    assert!(
        family
            .install(&session, invalid, "process", "node")
            .is_err()
    );
    assert!(family.is_empty());
    session.close().wait().unwrap();
}
