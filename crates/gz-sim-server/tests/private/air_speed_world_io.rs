// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::msgs::AirSpeed;

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
fn air_speed_io_owner_cancels_old_epoch_and_retired_lease_without_relabeling_bytes() {
    let mut io = isolated();
    for key in [
        io.clock.key_expr().as_str().to_owned(),
        io.poses.key_expr().as_str().to_owned(),
    ] {
        assert!(io.air_speed_output(key).is_err());
    }
    let key = "@/test@/air_speed".to_owned();
    let observer = io
        .session
        .declare_subscriber(key.clone())
        .wait()
        .expect("observer");
    let output = io.air_speed_output(key.clone()).expect("air_speed lease");
    let bytes = AirSpeed {
        diff_pressure: -61.25,
        temperature: 288.15,
        ..Default::default()
    }
    .encode_to_vec();
    io.epoch.store(1, Ordering::Release);
    io.incoming
        .send(WorldWork::AirSpeed(
            0,
            crate::air_speed_io::AirSpeedJob::new(&output, bytes.clone()),
        ))
        .expect("old queued epoch");
    let fence = io
        .air_speed_output("@/test@/fence".into())
        .expect("same-owner fence");
    assert!(observer.try_recv().expect("empty old epoch").is_none());
    let stale = crate::air_speed_io::AirSpeedJob::new(&output, bytes.clone());
    let weak = Arc::downgrade(&output);
    drop(output);
    assert!(weak.upgrade().is_none());
    io.incoming
        .send(WorldWork::AirSpeed(1, stale))
        .expect("retired queued lease");
    let replacement = io.air_speed_output(key).expect("new endpoint generation");
    assert!(observer.try_recv().expect("no retired sample").is_none());
    io.publish_air_speed(&replacement, bytes.clone())
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
        b"gz.msgs.AirSpeed"
    );
    assert!(io.publish_air_speed(&replacement, vec![0; 1025]).is_err());
    assert_eq!(io.publication_failures.len(), 12);
    assert_eq!(AIR_SPEED_FAILURE_INDEX, 10);
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
    io.publish_air_speed(&replacement, bytes.clone())
        .expect("one available slot");
    assert!(io.publish_air_speed(&replacement, bytes.clone()).is_err());
    drop(receiver);
    assert!(io.publish_air_speed(&replacement, bytes.clone()).is_err());
    io.incoming = incoming;
    io.session.close().wait().expect("close owned session");
    assert!(io.publish_air_speed(&replacement, bytes).is_err());
    assert!(io.air_speed_output("@/test@/closed".into()).is_err());
    io.shutdown();
}
