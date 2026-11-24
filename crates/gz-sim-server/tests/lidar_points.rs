// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_proto::{
    msgs::{LaserScan, PointCloudPacked, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use gz_sim_server::{Server, ServerOptions};
use std::{
    num::NonZeroUsize,
    time::{Duration, Instant},
};
use zenoh::Wait;

const XML: &str = "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='rig'><static>true</static><link name='body'><collision name='self'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision><sensor name='scan' type='lidar'><update_rate>100</update_rate><topic>/cloud/scan</topic><frame_id>sensor_frame</frame_id><lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal><vertical><samples>3</samples><resolution>1</resolution><min_angle>-0.2</min_angle><max_angle>0.2</max_angle></vertical></scan><range><min>0</min><max>20</max></range></lidar></sensor></link></model><model name='wall'><static>true</static><pose>5 0 0 0 0 0</pose><link name='body'><collision name='wall'><geometry><box><size>2 100 100</size></box></geometry></collision></link></model></world></sdf>";
fn server(xml: &str, cloud: bool) -> Server {
    let sim = load_world_with_sdf_step(
        xml,
        "points-wire.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let mut config = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(key, value).unwrap();
    }
    Server::with_zenoh_config(
        sim,
        "default",
        ServerOptions {
            lidar_point_clouds: cloud,
            partition: format!("points-wire-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        config,
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap()
}
fn xyz(bytes: &[u8], i: usize) -> [f32; 3] {
    std::array::from_fn(|a| {
        f32::from_le_bytes(
            bytes[i * 18 + a * 4..i * 18 + a * 4 + 4]
                .try_into()
                .unwrap(),
        )
    })
}
#[test]
fn organized_cloud_real_wire_matches_analytic_wall_and_the_same_scan_acquisition() {
    let mut server = server(XML, true);
    let session = server.zenoh_session();
    let scan = session
        .declare_subscriber(server.options().topic_key("/cloud/scan").unwrap())
        .wait()
        .unwrap();
    let cloud = session
        .declare_subscriber(server.options().topic_key("/cloud/scan/points").unwrap())
        .wait()
        .unwrap();
    server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    });
    let end = Instant::now() + Duration::from_secs(5);
    let mut scans = std::collections::BTreeMap::new();
    let mut clouds = std::collections::BTreeMap::new();
    let stamp = |header: &Option<gz_sim_proto::msgs::Header>| {
        let t = header.as_ref().unwrap().stamp.as_ref().unwrap();
        (t.sec, t.nsec)
    };
    let (matched, cloud) = loop {
        assert!(Instant::now() < end);
        server.update().unwrap();
        while let Some(s) = scan.try_recv().unwrap() {
            assert_eq!(
                s.attachment().unwrap().to_bytes().as_ref(),
                b"gz.msgs.LaserScan"
            );
            let p = LaserScan::decode(s.payload().to_bytes().as_ref()).unwrap();
            scans.insert(stamp(&p.header), p);
        }
        while let Some(s) = cloud.try_recv().unwrap() {
            assert_eq!(
                s.attachment().unwrap().to_bytes().as_ref(),
                b"gz.msgs.PointCloudPacked"
            );
            let p = PointCloudPacked::decode(s.payload().to_bytes().as_ref()).unwrap();
            clouds.insert(stamp(&p.header), p);
        }
        if let Some(key) = clouds.keys().find(|key| scans.contains_key(*key)).copied() {
            break (scans.remove(&key).unwrap(), clouds.remove(&key).unwrap());
        }
        while scans.len() > 32 {
            scans.pop_first();
        }
        while clouds.len() > 32 {
            clouds.pop_first();
        }
        std::thread::sleep(Duration::from_millis(1));
    };
    assert_eq!(
        (cloud.width, cloud.height, cloud.point_step, cloud.row_step),
        (3, 3, 18, 54)
    );
    assert!(cloud.is_dense && !cloud.is_bigendian);
    for v in 0..3 {
        for h in 0..3 {
            let i = v * 3 + h;
            let yaw = -0.3 + h as f64 * 0.3;
            let pitch = -0.2 + v as f64 * 0.2;
            let actual = xyz(&cloud.data, i);
            let expected = [4., 4. * yaw.tan(), 4. * pitch.tan() / yaw.cos()];
            for axis in 0..3 {
                assert!((f64::from(actual[axis]) - expected[axis]).abs() < 1e-6);
            }
            assert!((matched.ranges[i] - 4. / (yaw.cos() * pitch.cos())).abs() < 1e-10);
        }
    }
    assert_eq!(
        cloud.header.as_ref().unwrap().data[0].value,
        ["sensor_frame"]
    );
    server.shutdown();
}

#[test]
fn default_scan_only_world_never_announces_cloud_and_opt_in_reset_restarts_both_streams() {
    let mut legacy = server(XML, false);
    let session = legacy.zenoh_session();
    let cloud = session
        .declare_subscriber(legacy.options().topic_key("/cloud/scan/points").unwrap())
        .wait()
        .unwrap();
    legacy.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    });
    for _ in 0..30 {
        legacy.update().unwrap();
    }
    assert_eq!(legacy.counters().lidars_queued, 0);
    assert!(cloud.try_recv().unwrap().is_none());
    legacy.shutdown();
    let mut host = server(XML, true);
    let session = host.zenoh_session();
    let cloud = session
        .declare_subscriber(host.options().topic_key("/cloud/scan/points").unwrap())
        .wait()
        .unwrap();
    host.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    });
    let end = Instant::now() + Duration::from_secs(5);
    let first = loop {
        assert!(Instant::now() < end);
        host.update().unwrap();
        if let Some(s) = cloud.try_recv().unwrap() {
            break PointCloudPacked::decode(s.payload().to_bytes().as_ref()).unwrap();
        }
        std::thread::sleep(Duration::from_millis(1));
    };
    assert_eq!(first.header.unwrap().data[1].value, ["0"]);
    host.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    });
    let info = host.update().unwrap();
    assert_eq!((info.epoch, info.sim_time), (1, Duration::ZERO));
    // Drain only completed old-epoch deliveries before the new advancing step.
    while cloud.try_recv().unwrap().is_some() {}
    host.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    });
    let reset = loop {
        assert!(Instant::now() < end);
        host.update().unwrap();
        if let Some(s) = cloud.try_recv().unwrap() {
            break PointCloudPacked::decode(s.payload().to_bytes().as_ref()).unwrap();
        }
        std::thread::sleep(Duration::from_millis(1));
    };
    assert_eq!(reset.header.unwrap().data[1].value, ["0"]);
    host.shutdown();
}
