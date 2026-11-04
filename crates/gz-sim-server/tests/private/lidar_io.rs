// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::{
    msgs::{Header, LaserScan, Time},
    prost::Message,
};
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

fn publication_key() -> String {
    format!("@/lidar-{}@/scan", uuid::Uuid::new_v4())
}

fn tokens(session: &zenoh::Session) -> usize {
    session
        .liveliness()
        .get("@gz/**")
        .timeout(Duration::from_secs(1))
        .wait()
        .unwrap()
        .into_iter()
        .fold(0, |count, reply| {
            let _sample = reply.into_result().unwrap();
            count + 1
        })
}

fn wait_matching(output: &LidarOutput, expected: bool) {
    let deadline = Instant::now() + Duration::from_secs(3);
    while output.matching().unwrap() != expected {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
}

#[test]
fn actual_matching_and_last_lease_retirement_have_no_hidden_strong_owner() {
    let session = session();
    let key = publication_key();
    let subscriber = session.declare_subscriber(key.clone()).wait().unwrap();
    let (wake, retired) = mpsc::sync_channel(1);
    let mut registry = LidarDeclarations::new();
    assert!(registry.is_empty() && registry.dirty.is_none());
    registry.reconcile(false);
    let first = registry
        .get_or_create(&session, key.clone(), "process", "node", wake.clone())
        .unwrap();
    let second = registry
        .get_or_create(&session, key.clone(), "process", "node", wake)
        .unwrap();
    assert!(Arc::ptr_eq(&first, &second));
    assert_eq!(tokens(&session), 1);
    wait_matching(&first, true);
    drop(subscriber);
    wait_matching(&first, false);
    let subscriber = session.declare_subscriber(key).wait().unwrap();
    wait_matching(&first, true);
    let weak = Arc::downgrade(&first);
    drop(first);
    registry.reconcile(false);
    assert!(!registry.is_empty());
    assert!(retired.try_recv().is_err());
    drop(second);
    assert!(weak.upgrade().is_none());
    assert!(retired.try_recv().is_ok());
    assert_eq!(tokens(&session), 1, "only the IO owner drops declarations");
    registry.reconcile(false);
    assert!(registry.is_empty() && registry.dirty.is_none());
    assert_eq!(tokens(&session), 0);
    drop(subscriber);
}

#[test]
fn publishes_real_canonical_payload_and_cancels_retired_or_replaced_jobs() {
    let session = session();
    let key = publication_key();
    let subscriber = session.declare_subscriber(key.clone()).wait().unwrap();
    let (wake, _) = mpsc::sync_channel(1);
    let mut registry = LidarDeclarations::new();
    let lease = registry
        .get_or_create(&session, key.clone(), "process", "node", wake.clone())
        .unwrap();
    let message = LaserScan {
        header: Some(Header {
            stamp: Some(Time { sec: 2, nsec: 123 }),
            ..Default::default()
        }),
        frame: "rig/body/lidar".into(),
        count: 2,
        vertical_count: 1,
        ranges: vec![1.25, f64::INFINITY],
        intensities: vec![0.0; 2],
        ..Default::default()
    };
    let bytes = message.encode_to_vec();
    assert!(
        registry
            .publish(LidarJob::new(&lease, bytes.clone()))
            .unwrap()
    );
    let sample = subscriber
        .recv_timeout(Duration::from_secs(3))
        .unwrap()
        .unwrap();
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.LaserScan"
    );
    assert_eq!(sample.payload().to_bytes().as_ref(), bytes);
    assert_eq!(
        LaserScan::decode(sample.payload().to_bytes().as_ref()).unwrap(),
        message
    );
    let stale = LidarJob::new(&lease, bytes.clone());
    assert_eq!(Arc::strong_count(&lease), 1);
    drop(lease);
    assert!(!registry.publish(stale).unwrap());
    let replacement = registry
        .get_or_create(&session, key, "process", "node", wake)
        .unwrap();
    let missing = LidarJob {
        id: LidarId(uuid::Uuid::new_v4()),
        lease: Arc::downgrade(&replacement),
        bytes: bytes.clone(),
    };
    assert!(!registry.publish(missing).unwrap());
    let foreign = LidarJob {
        id: replacement.id,
        lease: Weak::new(),
        bytes: bytes.clone(),
    };
    assert!(!registry.publish(foreign).unwrap());
    assert!(subscriber.try_recv().unwrap().is_none());
    registry.reconcile(true);
    assert!(registry.is_empty());
    assert!(replacement.matching().is_err());
    assert_eq!(tokens(&session), 0);
    session.close().wait().unwrap();
    assert!(
        registry
            .get_or_create(
                &session,
                publication_key(),
                "process",
                "node",
                mpsc::sync_channel(1).0
            )
            .is_err()
    );
}

#[test]
fn a_full_retirement_wake_channel_does_not_keep_declarations_alive() {
    let session = session();
    let (wake, retired) = mpsc::sync_channel(1);
    let mut registry = LidarDeclarations::new();
    let first = registry
        .get_or_create(&session, publication_key(), "process", "node", wake.clone())
        .unwrap();
    let second = registry
        .get_or_create(&session, publication_key(), "process", "node", wake)
        .unwrap();
    drop(first);
    drop(second);
    assert!(retired.try_recv().is_ok());
    registry.reconcile(false);
    assert!(registry.is_empty() && registry.dirty.is_none());
    assert_eq!(tokens(&session), 0);
}

#[test]
fn invalid_registration_and_closed_transport_are_real_setup_or_publish_errors() {
    let session = session();
    let (wake, _) = mpsc::sync_channel(1);
    let mut registry = LidarDeclarations::new();
    assert!(
        registry
            .get_or_create(&session, "invalid".into(), "process", "node", wake.clone())
            .is_err()
    );
    assert!(registry.is_empty());
    let lease = registry
        .get_or_create(&session, publication_key(), "process", "node", wake)
        .unwrap();
    session.close().wait().unwrap();
    assert!(registry.publish(LidarJob::new(&lease, vec![8, 1])).is_err());
    registry.reconcile(true);
    assert!(registry.is_empty() && lease.matching().is_err());
}
