// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::msgs::{Pose, PoseV};

fn isolated() -> WorldIo {
    let mut config = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(key, value).expect("local-only config");
    }
    WorldIo::new_with_config(
        "@/test@/clock".into(),
        "@/test@/control".into(),
        "@/test@/poses".into(),
        4,
        1024,
        config,
    )
    .expect("owned world IO")
}

#[test]
fn pose_publisher_io_owner_cancels_old_epoch_and_retired_lease_without_relabeling_bytes() {
    let mut io = isolated();
    for key in [
        io.clock.key_expr().as_str().to_owned(),
        io.poses.key_expr().as_str().to_owned(),
    ] {
        assert!(io.pose_publisher_output(key).is_err());
    }
    let key = "@/test@/pose_publisher".to_owned();
    let observer = io
        .session
        .declare_subscriber(key.clone())
        .wait()
        .expect("observer");
    let output = io
        .pose_publisher_output(key.clone())
        .expect("pose_publisher lease");
    let bytes = PoseV {
        header: None,
        pose: vec![Pose {
            name: "rig::body".into(),
            ..Default::default()
        }],
    }
    .encode_to_vec();
    io.epoch.store(1, Ordering::Release);
    io.incoming
        .send(WorldWork::PosePublisher(
            0,
            crate::pose_publisher_io::PosePublisherJob::new(&output, bytes.clone()),
        ))
        .expect("old queued epoch");
    let fence = io
        .pose_publisher_output("@/test@/fence".into())
        .expect("same-owner fence");
    assert!(observer.try_recv().expect("empty old epoch").is_none());
    let stale = crate::pose_publisher_io::PosePublisherJob::new(&output, bytes.clone());
    let weak = Arc::downgrade(&output);
    drop(output);
    assert!(weak.upgrade().is_none());
    io.incoming
        .send(WorldWork::PosePublisher(1, stale))
        .expect("retired queued lease");
    let replacement = io
        .pose_publisher_output(key)
        .expect("new endpoint generation");
    assert!(observer.try_recv().expect("no retired sample").is_none());
    io.publish_pose_publisher(&replacement, bytes.clone())
        .expect("current queued sample");
    let received = observer
        .recv_timeout(Duration::from_secs(3))
        .expect("receive")
        .expect("actual current publication");
    assert_eq!(received.payload().to_bytes().as_ref(), bytes);
    assert_eq!(
        received
            .attachment()
            .expect("canonical type")
            .to_bytes()
            .as_ref(),
        b"gz.msgs.Pose_V"
    );
    assert!(
        io.publish_pose_publisher(&replacement, vec![0; 1025])
            .is_err()
    );
    assert!(POSE_PUBLISHER_FAILURE_INDEX < io.publication_failures.len());
    assert!(
        io.publication_failures
            .iter()
            .all(|count| count.load(Ordering::Relaxed) == 0)
    );
    drop(fence);
    // Exercise the same bounded admission operation against a full and then
    // disconnected real channel, without waiting on a network consumer.
    let (bounded, receiver) = crossbeam_channel::bounded(1);
    let incoming = std::mem::replace(&mut io.incoming, bounded);
    io.publish_pose_publisher(&replacement, bytes.clone())
        .expect("one available slot");
    assert!(
        io.publish_pose_publisher(&replacement, bytes.clone())
            .is_err()
    );
    drop(receiver);
    assert!(
        io.publish_pose_publisher(&replacement, bytes.clone())
            .is_err()
    );
    io.incoming = incoming;
    io.session.close().wait().expect("close owned session");
    assert!(io.publish_pose_publisher(&replacement, bytes).is_err());
    assert!(io.pose_publisher_output("@/test@/closed".into()).is_err());
    io.shutdown();
}

#[test]
fn split_pose_world_epoch_cancels_both_channels_and_retiring_one_does_not_retire_the_other() {
    let io = isolated();
    let dynamic_key = "@/test@/split_pose".to_owned();
    let fixed_key = format!("{dynamic_key}_static");
    let dynamic_samples = io
        .session
        .declare_subscriber(dynamic_key.clone())
        .wait()
        .unwrap();
    let fixed_samples = io
        .session
        .declare_subscriber(fixed_key.clone())
        .wait()
        .unwrap();
    let dynamic = io.pose_publisher_output(dynamic_key).unwrap();
    let fixed = io.pose_publisher_output(fixed_key).unwrap();
    let frame = PoseV {
        header: None,
        pose: vec![Pose {
            name: "rig::base".into(),
            ..Default::default()
        }],
    }
    .encode_to_vec();
    io.epoch.store(2, Ordering::Release);
    for lease in [&dynamic, &fixed] {
        io.incoming
            .send(WorldWork::PosePublisher(
                1,
                crate::pose_publisher_io::PosePublisherJob::new(lease, frame.clone()),
            ))
            .unwrap();
    }
    let fence = io
        .pose_publisher_output("@/test@/split_fence".into())
        .unwrap();
    assert!(dynamic_samples.try_recv().unwrap().is_none());
    assert!(fixed_samples.try_recv().unwrap().is_none());
    let weak = Arc::downgrade(&dynamic);
    drop(dynamic);
    assert!(weak.upgrade().is_none());
    io.publish_pose_publisher(&fixed, frame.clone()).unwrap();
    assert_eq!(
        fixed_samples
            .recv_timeout(Duration::from_secs(3))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
        frame
    );
    drop(fence);
    io.session.close().wait().unwrap();
    assert!(
        io.pose_publisher_output("@/test@/split_closed".into())
            .is_err()
    );
}
