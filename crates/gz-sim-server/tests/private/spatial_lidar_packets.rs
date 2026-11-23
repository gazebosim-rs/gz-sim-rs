// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sensors::lidar::{PlanarLidarConfig, VerticalLidarConfig};
use gz_sim_proto::prost::Message;
fn metadata() -> LidarPublication {
    LidarPublication {
        entity_name: "rig::body::scan".into(),
        frame_id: "grid_frame".into(),
        topic: "/grid".into(),
        config: PlanarLidarConfig {
            samples: 3,
            angle_min: -0.3,
            angle_max: 0.3,
            range_min: 0.0,
            range_max: 10.0,
        },
        angle_step: 0.3,
        vertical: VerticalLidarConfig {
            samples: 3,
            angle_min: -0.2,
            angle_max: 0.2,
        },
    }
}
fn sample() -> LidarSample {
    LidarSample {
        time: Duration::from_millis(17),
        epoch: 3,
        world_pose: gz_sim_scene::WorldPose {
            translation: [1., 2., 3.],
            rotation: [0., 0., 0., 1.],
        },
        ranges: vec![0., 1., 2., 3., 4., 5., 6., 7., f64::INFINITY],
    }
}
#[test]
fn spatial_packet_carries_row_major_ranges_both_axes_and_acquisition_pose() {
    let m = metadata();
    let s = sample();
    let p = packet(&m, &s, 7).unwrap();
    assert_eq!((p.count, p.vertical_count), (3, 3));
    assert_eq!((p.angle_min, p.angle_max, p.angle_step), (-0.3, 0.3, 0.3));
    assert_eq!(
        (
            p.vertical_angle_min,
            p.vertical_angle_max,
            p.vertical_angle_step
        ),
        (-0.2, 0.2, 0.2)
    );
    assert_eq!(p.ranges, s.ranges);
    assert_eq!(p.intensities, vec![0.; 9]);
    assert_eq!(p.frame, "grid_frame");
    assert_eq!(
        p.world_pose.as_ref().unwrap().position,
        Some(vector3([1., 2., 3.]))
    );
    assert_eq!(LaserScan::decode(p.encode_to_vec().as_slice()).unwrap(), p);
}
#[test]
fn spatial_packet_refuses_grid_overflow_drift_and_incomplete_or_nonfinite_scans() {
    let m = metadata();
    for field in 0..7 {
        let mut bad = m.clone();
        match field {
            0 => bad.vertical.samples = 0,
            1 => bad.vertical.samples = usize::MAX,
            2 => bad.vertical.samples = 64,
            3 => bad.vertical.angle_min = f64::NAN,
            4 => bad.vertical.angle_max = bad.vertical.angle_min,
            5 => bad.vertical.angle_max = 2.0,
            _ => bad.angle_step = 0.1,
        }
        assert!(!valid_metadata(&bad));
        assert!(packet(&bad, &sample(), 0).is_none());
    }
    for ranges in [
        vec![1.; 8],
        vec![1.; 10],
        vec![f64::NAN; 9],
        vec![f64::NEG_INFINITY; 9],
        vec![11.; 9],
    ] {
        assert!(packet(&m, &LidarSample { ranges, ..sample() }, 0).is_none());
    }
    let mut max = m;
    max.config.samples = 16;
    max.angle_step = 0.6 / 15.;
    max.vertical.samples = 8;
    let p = packet(
        &max,
        &LidarSample {
            ranges: vec![f64::INFINITY; 128],
            ..sample()
        },
        0,
    )
    .unwrap();
    assert_eq!((p.count, p.vertical_count, p.ranges.len()), (16, 8, 128));
}
