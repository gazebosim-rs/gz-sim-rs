// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::time::{Duration, Instant};
fn session() -> zenoh::Session {
    let mut c = zenoh::Config::default();
    for (p, v) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        c.insert_json5(p, v).unwrap();
    }
    zenoh::open(c).wait().unwrap()
}
fn key() -> String {
    format!("@/m30-{}@/camera", uuid::Uuid::new_v4())
}
fn matching(output: &LogicalCameraOutput, value: bool) {
    let end = Instant::now() + Duration::from_secs(3);
    while output.matching().unwrap() != value {
        assert!(Instant::now() < end);
        std::thread::yield_now();
    }
}
#[test]
fn channel_schema_leases_reconnect_retire_and_generation_fence() {
    let session = session();
    let mut declarations = LogicalCameraDeclarations::new();
    let (wake, _) = mpsc::sync_channel(1);
    let base = key();
    for channel in [LogicalCameraChannel::Image, LogicalCameraChannel::Frustum] {
        let key = format!("{base}/{channel:?}");
        let first = declarations
            .get_or_create(&session, key.clone(), channel, "p", "n", wake.clone())
            .unwrap();
        let alias = declarations
            .get_or_create(&session, key.clone(), channel, "p", "n", wake.clone())
            .unwrap();
        assert!(Arc::ptr_eq(&first, &alias));
        let other = if channel == LogicalCameraChannel::Image {
            LogicalCameraChannel::Frustum
        } else {
            LogicalCameraChannel::Image
        };
        assert!(
            declarations
                .get_or_create(&session, key.clone(), other, "p", "n", wake.clone())
                .is_err()
        );
        let observer = session.declare_subscriber(key.clone()).wait().unwrap();
        matching(&first, true);
        assert_eq!(Arc::strong_count(&first), 2);
        assert!(
            declarations
                .publish(LogicalCameraJob::new(&first, vec![1, 2, 3]))
                .unwrap()
        );
        let sample = observer
            .recv_timeout(Duration::from_secs(3))
            .unwrap()
            .unwrap();
        assert_eq!(sample.payload().to_bytes().as_ref(), [1, 2, 3]);
        assert_eq!(
            sample.attachment().unwrap().to_bytes().as_ref(),
            channel.schema().as_bytes()
        );
        drop(observer);
        matching(&first, false);
        let observer = session.declare_subscriber(key.clone()).wait().unwrap();
        matching(&first, true);
        let stale = LogicalCameraJob::new(&first, vec![4]);
        let weak = Arc::downgrade(&first);
        drop(alias);
        drop(first);
        assert!(weak.upgrade().is_none());
        declarations.reconcile(false);
        assert!(declarations.is_empty());
        assert!(!declarations.publish(stale).unwrap());
        let fresh = declarations
            .get_or_create(&session, key, channel, "p", "n", wake.clone())
            .unwrap();
        assert!(observer.try_recv().unwrap().is_none());
        matching(&fresh, true);
        assert!(
            declarations
                .publish(LogicalCameraJob::new(&fresh, vec![5]))
                .unwrap()
        );
        assert_eq!(
            observer
                .recv_timeout(Duration::from_secs(3))
                .unwrap()
                .unwrap()
                .payload()
                .to_bytes()
                .as_ref(),
            [5]
        );
        drop(fresh);
        declarations.reconcile(false);
    }
    assert!(declarations.is_empty() && declarations.dirty.is_none());
    session.close().wait().unwrap();
    assert!(
        declarations
            .get_or_create(&session, base, LogicalCameraChannel::Image, "p", "n", wake)
            .is_err()
    );
}
#[test]
fn session_closure_invalidates_both_cpu_leases_without_retaining_native_declarations() {
    let session = session();
    let mut d = LogicalCameraDeclarations::new();
    let (wake, _) = mpsc::sync_channel(1);
    let a = d
        .get_or_create(
            &session,
            key(),
            LogicalCameraChannel::Image,
            "p",
            "n",
            wake.clone(),
        )
        .unwrap();
    let b = d
        .get_or_create(
            &session,
            key(),
            LogicalCameraChannel::Frustum,
            "p",
            "n",
            wake,
        )
        .unwrap();
    session.close().wait().unwrap();
    d.reconcile(true);
    assert!(a.matching().is_err() && b.matching().is_err());
    assert!(d.is_empty());
    assert!(!d.publish(LogicalCameraJob::new(&a, vec![])).unwrap());
}
