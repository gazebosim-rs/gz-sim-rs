// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::{
    msgs::{Header, Pose, PoseV, Time},
    prost::Message,
};
use std::time::Duration;

fn session() -> zenoh::Session {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"peer\"").unwrap();
    config
        .insert_json5("listen/endpoints", "[\"tcp/127.0.0.1:0\"]")
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    zenoh::open(config).wait().unwrap()
}
fn key() -> String {
    format!(
        "@/pose_publisher-{}@/sensor/pose_publisher",
        uuid::Uuid::new_v4()
    )
}
fn output(
    registry: &mut PosePublisherDeclarations,
    session: &zenoh::Session,
    key: &str,
    sender: mpsc::SyncSender<PosePublisherId>,
) -> Arc<PosePublisherOutput> {
    registry
        .get_or_create(session, key.into(), "process", "node", sender)
        .unwrap()
}
fn count_tokens(session: &zenoh::Session) -> usize {
    session
        .liveliness()
        .get("@gz/**")
        .timeout(Duration::from_secs(1))
        .wait()
        .unwrap()
        .into_iter()
        .fold(0, |count, reply| {
            reply.into_result().unwrap();
            count + 1
        })
}
#[test]
fn pose_publisher_effective_topic_shares_cpu_lease_until_last_consumer_and_io_retirement() {
    let session = session();
    let mut registry = PosePublisherDeclarations::new();
    assert!(registry.is_empty() && registry.dirty.is_none());
    registry.reconcile(false);
    let (wake, retired) = mpsc::sync_channel(1);
    let key = key();
    let first = output(&mut registry, &session, &key, wake.clone());
    let second = output(&mut registry, &session, &key, wake);
    assert!(Arc::ptr_eq(&first, &second));
    assert_eq!(registry.entries.len(), 1);
    assert_eq!(count_tokens(&session), 1);
    let weak = Arc::downgrade(&first);
    drop(first);
    registry.reconcile(false);
    assert!(!registry.is_empty());
    assert!(retired.try_recv().is_err());
    drop(second);
    assert!(weak.upgrade().is_none());
    assert!(retired.try_recv().is_ok());
    assert_eq!(
        count_tokens(&session),
        1,
        "CPU drop must not drop native declarations"
    );
    registry.reconcile(false);
    assert!(registry.is_empty() && registry.dirty.is_none());
    assert_eq!(count_tokens(&session), 0);
}

#[test]
fn pose_publisher_publishes_exact_frames_and_type_attachment_without_changing_stamp_or_model_bytes()
{
    let session = session();
    let key = key();
    let samples = session.declare_subscriber(key.clone()).wait().unwrap();
    let (wake, _) = mpsc::sync_channel(1);
    let mut registry = PosePublisherDeclarations::new();
    let lease = output(&mut registry, &session, &key, wake);
    for nsec in [1_000_000, 2_000_000] {
        let frame = PoseV {
            header: None,
            pose: vec![Pose {
                header: Some(Header {
                    stamp: Some(Time { sec: 0, nsec }),
                    data: Vec::new(),
                }),
                name: "rig::body".into(),
                ..Default::default()
            }],
        };
        let bytes = frame.encode_to_vec();
        let job = PosePublisherJob::new(&lease, bytes.clone());
        assert_eq!(Arc::strong_count(&lease), 1);
        assert!(registry.publish(job).unwrap());
        let sample = samples
            .recv_timeout(Duration::from_secs(3))
            .unwrap()
            .unwrap();
        assert_eq!(sample.payload().to_bytes().as_ref(), bytes);
        assert_eq!(
            sample.attachment().unwrap().to_bytes().as_ref(),
            PoseV::full_name().as_bytes()
        );
        let received = PoseV::decode(sample.payload().to_bytes().as_ref()).unwrap();
        assert!(received.header.is_none());
        assert_eq!(
            received.pose[0]
                .header
                .as_ref()
                .unwrap()
                .stamp
                .as_ref()
                .unwrap()
                .nsec,
            nsec
        );
        assert_eq!(
            (received.pose[0].name.as_str(), received.pose[0].id),
            ("rig::body", 0)
        );
    }
}

#[test]
fn pose_publisher_queued_frames_never_keep_retired_cpu_leases_alive_or_reach_replacement_endpoints()
{
    let session = session();
    let key = key();
    let samples = session.declare_subscriber(key.clone()).wait().unwrap();
    let (wake, _) = mpsc::sync_channel(1);
    let mut registry = PosePublisherDeclarations::new();
    let lease = output(&mut registry, &session, &key, wake.clone());
    let dead = PosePublisherJob::new(&lease, vec![1]);
    let stale = PosePublisherJob::new(&lease, vec![2]);
    let weak = Arc::downgrade(&lease);
    drop(lease);
    assert!(weak.upgrade().is_none());
    assert!(!registry.publish(dead).unwrap());
    let replacement = output(&mut registry, &session, &key, wake);
    assert_ne!(replacement.id, stale.id);
    assert!(!registry.publish(stale).unwrap());
    assert!(
        samples
            .recv_timeout(Duration::from_millis(30))
            .unwrap()
            .is_none()
    );
    assert!(
        registry
            .publish(PosePublisherJob::new(&replacement, vec![3]))
            .unwrap()
    );
    assert_eq!(
        samples
            .recv_timeout(Duration::from_secs(3))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
        &[3]
    );
    assert_eq!(count_tokens(&session), 1);
}

#[test]
fn pose_publisher_full_or_disconnected_retirement_notifications_do_not_retain_native_state() {
    for disconnect in [false, true] {
        let session = session();
        let mut registry = PosePublisherDeclarations::new();
        let (wake, receiver) = mpsc::sync_channel(1);
        let first = output(&mut registry, &session, &key(), wake.clone());
        let second = output(&mut registry, &session, &key(), wake);
        if disconnect {
            drop(receiver);
        }
        drop(first);
        drop(second);
        registry.reconcile(false);
        assert!(registry.is_empty() && registry.dirty.is_none());
        assert_eq!(count_tokens(&session), 0);
    }
}

#[test]
fn pose_publisher_closed_session_refuses_setup_and_put_and_releases_declarations() {
    let session = session();
    let (wake, _) = mpsc::sync_channel(1);
    let mut registry = PosePublisherDeclarations::new();
    let lease = output(&mut registry, &session, &key(), wake.clone());
    let job = PosePublisherJob::new(&lease, Vec::new());
    session.close().wait().unwrap();
    assert!(registry.publish(job).is_err());
    registry.reconcile(true);
    assert!(registry.is_empty() && registry.dirty.is_none());
    assert!(
        registry
            .get_or_create(&session, key(), "p", "n", wake)
            .is_err()
    );
}

#[test]
fn pose_publisher_invalid_setup_and_foreign_jobs_cannot_publish_or_leave_watch_state() {
    let session = session();
    let (wake, _) = mpsc::sync_channel(1);
    let mut registry = PosePublisherDeclarations::new();
    for key in ["invalid", "@/test@/bad//key"] {
        assert!(
            registry
                .get_or_create(&session, key.into(), "p", "n", wake.clone())
                .is_err()
        );
        assert!(registry.is_empty() && registry.dirty.is_none());
    }
    let first = output(&mut registry, &session, &key(), wake.clone());
    let mut other = PosePublisherDeclarations::new();
    let foreign = output(&mut other, &session, &key(), wake);
    let mut job = PosePublisherJob::new(&foreign, Vec::new());
    assert!(
        !registry
            .publish(PosePublisherJob::new(&foreign, Vec::new()))
            .unwrap()
    );
    job.id = first.id;
    assert!(!registry.publish(job).unwrap());
}

#[test]
fn pose_publisher_stalled_io_put_does_not_hold_cpu_lease_and_following_retired_frame_cancels() {
    let session = session();
    let key = key();
    let release = Arc::new((std::sync::Mutex::new(false), std::sync::Condvar::new()));
    let callback_release = Arc::clone(&release);
    let frames = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let observed = Arc::clone(&frames);
    let (entered, started) = mpsc::sync_channel(1);
    let _subscriber = session
        .declare_subscriber(key.clone())
        .callback(move |_: zenoh::sample::Sample| {
            observed.fetch_add(1, Ordering::Relaxed);
            let _ = entered.try_send(());
            let (lock, condition) = &*callback_release;
            drop(
                condition
                    .wait_while(lock.lock().unwrap(), |released| !*released)
                    .unwrap(),
            );
        })
        .wait()
        .unwrap();
    let (wake, retired) = mpsc::sync_channel(1);
    let (ready, lease) = mpsc::sync_channel(1);
    let (jobs, queued) = mpsc::sync_channel::<[PosePublisherJob; 2]>(1);
    let io_session = session.clone();
    let io = std::thread::spawn(move || {
        let mut registry = PosePublisherDeclarations::new();
        ready
            .send(output(&mut registry, &io_session, &key, wake))
            .unwrap_or_else(|_| panic!("CPU owner disappeared"));
        let [first, second] = queued.recv().unwrap();
        assert!(registry.publish(first).unwrap());
        assert!(!registry.publish(second).unwrap());
        registry.reconcile(false);
        assert!(registry.is_empty());
    });
    let lease = lease.recv_timeout(Duration::from_secs(3)).unwrap();
    assert_eq!(Arc::strong_count(&lease), 1);
    let weak = Arc::downgrade(&lease);
    jobs.send([
        PosePublisherJob::new(&lease, vec![1]),
        PosePublisherJob::new(&lease, vec![2]),
    ])
    .unwrap_or_else(|_| panic!("IO owner disappeared"));
    started.recv_timeout(Duration::from_secs(3)).unwrap();
    let (dropped, completion) = mpsc::sync_channel(1);
    let cpu = std::thread::spawn(move || {
        drop(lease);
        dropped.send(()).unwrap();
    });
    let cpu_drop_before_io_resumed = completion.recv_timeout(Duration::from_secs(1));
    *release.0.lock().unwrap() = true;
    release.1.notify_all();
    cpu.join().unwrap();
    io.join().unwrap();
    cpu_drop_before_io_resumed.expect("CPU lease drop waited for stalled native publication");
    assert!(weak.upgrade().is_none());
    assert!(retired.try_recv().is_ok());
    assert_eq!(frames.load(Ordering::Relaxed), 1);
    assert_eq!(count_tokens(&session), 0);
}

#[test]
fn pose_publisher_zero_strong_count_before_cpu_destructor_cannot_reuse_an_expired_topic_lease() {
    let session = session();
    let key = key();
    let (wake, _) = mpsc::sync_channel(1);
    let mut registry = PosePublisherDeclarations::new();
    let lease = output(&mut registry, &session, &key, wake.clone());
    let stale = PosePublisherJob::new(&lease, Vec::new());
    // Standard Arc ownership transfer exposes the real zero-strong-count window
    // before PosePublisherOutput::drop sets dirty, without a production test hook.
    let old_cpu_value = Arc::into_inner(lease).unwrap();
    assert!(!registry.dirty.as_ref().unwrap().load(Ordering::Acquire));
    let replacement = output(&mut registry, &session, &key, wake);
    assert_ne!(replacement.id, old_cpu_value.id);
    drop(old_cpu_value);
    assert!(!registry.dirty.as_ref().unwrap().load(Ordering::Acquire));
    assert!(!registry.publish(stale).unwrap());
    assert_eq!(registry.entries.len(), 1);
    assert_eq!(count_tokens(&session), 1);
}
