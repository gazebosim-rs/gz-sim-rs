// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::{
    msgs::{Odometry, OdometryWithCovariance, PoseV},
    prost::Message,
};

#[test]
fn world_owner_routes_three_ground_truth_types_with_disjoint_failure_slots() {
    let config = zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}},transport:{shared_memory:{enabled:false}}}"#).unwrap();
    let mut io = WorldIo::new_with_config(
        "@/m34@/clock".into(),
        "@/m34@/control".into(),
        "@/m34@/poses".into(),
        8,
        1024,
        config,
    )
    .unwrap();
    assert_eq!(io.publication_failures.len(), 17);
    let slots = [
        GROUND_TRUTH_ODOMETRY_FAILURE_INDEX,
        GROUND_TRUTH_ODOMETRY_COVARIANCE_FAILURE_INDEX,
        GROUND_TRUTH_ODOMETRY_TF_FAILURE_INDEX,
    ];
    assert_eq!(slots, [14, 15, 16]);
    assert!(
        slots
            .into_iter()
            .all(|slot| slot > LOGICAL_CAMERA_FRUSTUM_FAILURE_INDEX
                && slot < io.publication_failures.len())
    );
    for (number, channel, bytes, kind) in [
        (
            0,
            GroundTruthOdometryChannel::Odometry,
            Odometry::default().encode_to_vec(),
            "gz.msgs.Odometry",
        ),
        (
            1,
            GroundTruthOdometryChannel::Covariance,
            OdometryWithCovariance::default().encode_to_vec(),
            "gz.msgs.OdometryWithCovariance",
        ),
        (
            2,
            GroundTruthOdometryChannel::Transform,
            PoseV::default().encode_to_vec(),
            "gz.msgs.Pose_V",
        ),
    ] {
        let key = format!("@/m34@/io/{number}");
        let subscriber = io.session.declare_subscriber(key.clone()).wait().unwrap();
        let output = io
            .ground_truth_odometry_output(key.clone(), channel)
            .unwrap();
        assert!(Arc::ptr_eq(
            &output,
            &io.ground_truth_odometry_output(key, channel).unwrap()
        ));
        io.publish_ground_truth_odometry(&output, bytes.clone())
            .unwrap();
        let received = subscriber
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap();
        assert_eq!(received.payload().to_bytes().as_ref(), bytes);
        assert_eq!(
            received.attachment().unwrap().to_bytes().as_ref(),
            kind.as_bytes()
        );
        assert!(
            io.publish_ground_truth_odometry(&output, vec![0; 1025])
                .is_err()
        );
        let weak = Arc::downgrade(&output);
        drop(output);
        assert!(weak.upgrade().is_none());
    }
    assert!(
        io.publication_failures
            .iter()
            .all(|count| count.load(Ordering::Relaxed) == 0)
    );
    io.shutdown();
    assert!(
        io.ground_truth_odometry_output(
            "@/m34@/io/closed".into(),
            GroundTruthOdometryChannel::Odometry
        )
        .is_err()
    );
}

fn bounded_odometry_io() -> WorldIo {
    let config = zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}},transport:{shared_memory:{enabled:false}}}"#).unwrap();
    WorldIo::new_with_config(
        "@/m34@/clock".into(),
        "@/m34@/control".into(),
        "@/m34@/poses".into(),
        8,
        1024,
        config,
    )
    .unwrap()
}

#[test]
fn odometry_retirement_and_epoch_fences_cancel_all_three_channels_without_relabeling() {
    let io = bounded_odometry_io();
    for channel in [
        GroundTruthOdometryChannel::Odometry,
        GroundTruthOdometryChannel::Covariance,
        GroundTruthOdometryChannel::Transform,
    ] {
        for key in ["@/m34@/clock", "@/m34@/poses"] {
            assert!(
                io.ground_truth_odometry_output(key.into(), channel)
                    .is_err()
            );
        }
    }
    io.epoch.store(2, Ordering::Release);
    for (i, channel) in [
        GroundTruthOdometryChannel::Odometry,
        GroundTruthOdometryChannel::Covariance,
        GroundTruthOdometryChannel::Transform,
    ]
    .into_iter()
    .enumerate()
    {
        let key = format!("@/m34@/epoch/{i}");
        let observer = io.session.declare_subscriber(key.clone()).wait().unwrap();
        let output = io
            .ground_truth_odometry_output(key.clone(), channel)
            .unwrap();
        let other = if channel == GroundTruthOdometryChannel::Odometry {
            GroundTruthOdometryChannel::Transform
        } else {
            GroundTruthOdometryChannel::Odometry
        };
        assert!(io.ground_truth_odometry_output(key.clone(), other).is_err());
        let bytes = vec![i as u8];
        io.incoming
            .send(WorldWork::GroundTruthOdometry(
                1,
                crate::odometry_publisher_io::GroundTruthOdometryJob::new(&output, bytes.clone()),
            ))
            .unwrap();
        let fence = io
            .ground_truth_odometry_output(format!("@/m34@/fence/{i}"), channel)
            .unwrap();
        assert!(observer.try_recv().unwrap().is_none());
        let stale =
            crate::odometry_publisher_io::GroundTruthOdometryJob::new(&output, bytes.clone());
        let weak = Arc::downgrade(&output);
        drop(output);
        assert!(weak.upgrade().is_none());
        io.incoming
            .send(WorldWork::GroundTruthOdometry(2, stale))
            .unwrap();
        let replacement = io.ground_truth_odometry_output(key, channel).unwrap();
        assert!(observer.try_recv().unwrap().is_none());
        io.publish_ground_truth_odometry(&replacement, bytes.clone())
            .unwrap();
        assert_eq!(
            observer
                .recv_timeout(Duration::from_secs(3))
                .unwrap()
                .unwrap()
                .payload()
                .to_bytes()
                .as_ref(),
            bytes
        );
        drop(fence);
    }
    assert!(
        io.publication_failures
            .iter()
            .all(|count| count.load(Ordering::Relaxed) == 0)
    );
}

#[test]
fn odometry_output_admission_refuses_full_disconnected_and_closed_owners() {
    let mut io = bounded_odometry_io();
    let output = io
        .ground_truth_odometry_output(
            "@/m34@/bounded".into(),
            GroundTruthOdometryChannel::Odometry,
        )
        .unwrap();
    let (bounded, receiver) = crossbeam_channel::bounded(1);
    let incoming = std::mem::replace(&mut io.incoming, bounded);
    io.publish_ground_truth_odometry(&output, vec![1]).unwrap();
    assert!(io.publish_ground_truth_odometry(&output, vec![2]).is_err());
    drop(receiver);
    assert!(io.publish_ground_truth_odometry(&output, vec![3]).is_err());
    io.incoming = incoming;
    io.session.close().wait().unwrap();
    assert!(io.publish_ground_truth_odometry(&output, vec![4]).is_err());
    assert!(
        io.ground_truth_odometry_output(
            "@/m34@/closed".into(),
            GroundTruthOdometryChannel::Transform
        )
        .is_err()
    );
    io.shutdown();
}
