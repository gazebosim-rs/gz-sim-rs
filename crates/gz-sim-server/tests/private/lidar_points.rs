// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sensors::lidar::{PlanarLidarConfig, VerticalLidarConfig};
use gz_sim_proto::msgs::{WorldControl, WorldReset};
use gz_sim_scene::{PhysicsEngine, WorldPose, load_world_with_sdf_step, remove_model};
use std::sync::mpsc;

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
            range_max: 20.0,
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
        time: Duration::new(2, 17),
        epoch: 3,
        world_pose: WorldPose {
            translation: [9., 8., 7.],
            rotation: [0., 0., 1., 0.],
        },
        ranges: vec![4.; 9],
    }
}
fn float(data: &[u8], point: usize, axis: usize) -> f32 {
    f32::from_le_bytes(
        data[point * 18 + axis * 4..point * 18 + axis * 4 + 4]
            .try_into()
            .unwrap(),
    )
}
fn selected() -> ServerOptions {
    ServerOptions {
        lidar_point_clouds: true,
        partition: format!("points-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    }
}
fn config() -> zenoh::Config {
    let mut c = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        c.insert_json5(key, value).unwrap();
    }
    c
}
fn xml() -> &'static str {
    "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='rig'><static>true</static><link name='body'><collision name='self'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision><sensor name='scan' type='lidar'><update_rate>100</update_rate><topic>/grid</topic><frame_id>grid_frame</frame_id><lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal><vertical><samples>3</samples><resolution>1</resolution><min_angle>-0.2</min_angle><max_angle>0.2</max_angle></vertical></scan><range><min>0</min><max>20</max></range></lidar></sensor></link></model><model name='wall'><static>true</static><pose>5 0 0 0 0 0</pose><link name='body'><collision name='wall'><geometry><box><size>2 100 100</size></box></geometry></collision></link></model></world></sdf>"
}
fn simulation(text: &str) -> Simulation {
    load_world_with_sdf_step(
        text,
        "points.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn server(options: ServerOptions) -> Server {
    Server::with_zenoh_config(
        simulation(xml()),
        "default",
        options,
        config(),
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap()
}
fn matching(output: &world_io::LidarOutput, wanted: bool) {
    let end = Instant::now() + Duration::from_secs(5);
    while output.matching().unwrap() != wanted {
        assert!(Instant::now() < end);
        std::thread::yield_now();
    }
}

#[test]
fn organized_points_have_native_unaligned_fields_sensor_coordinates_and_canonical_header() {
    let m = metadata();
    let s = sample();
    let p = points_packet(&m, &s, 19).unwrap();
    assert_eq!(
        (p.width, p.height, p.point_step, p.row_step, p.data.len()),
        (3, 3, 18, 54, 162)
    );
    assert!(!p.is_bigendian && p.is_dense);
    let descriptors: Vec<_> = p
        .field
        .iter()
        .map(|f| (f.name.as_str(), f.offset, f.datatype, f.count))
        .collect();
    assert_eq!(
        descriptors,
        [
            ("x", 0, 6, 1),
            ("y", 4, 6, 1),
            ("z", 8, 6, 1),
            ("intensity", 12, 6, 1),
            ("ring", 16, 3, 1)
        ]
    );
    assert_eq!(p.header, packet(&m, &s, 19).unwrap().header);
    for v in 0..3 {
        for h in 0..3 {
            let pitch = -0.2 + v as f64 * 0.2;
            let yaw = -0.3 + h as f64 * 0.3;
            let i = v * 3 + h;
            let expected = [
                4. * pitch.cos() * yaw.cos(),
                4. * pitch.cos() * yaw.sin(),
                4. * pitch.sin(),
            ];
            for (axis, value) in expected.into_iter().enumerate() {
                assert!((f64::from(float(&p.data, i, axis)) - value).abs() < 3e-7);
            }
            assert_eq!(float(&p.data, i, 3).to_bits(), 0);
            assert_eq!(
                u16::from_le_bytes(p.data[i * 18 + 16..i * 18 + 18].try_into().unwrap()),
                v as u16
            );
        }
    }
    let moved = LidarSample {
        world_pose: WorldPose {
            translation: [-1.; 3],
            rotation: [0., 0., 0., 1.],
        },
        ..sample()
    };
    assert_eq!(
        points_packet(&m, &moved, 19).unwrap(),
        p,
        "XYZ never becomes world coordinates"
    );
    assert_eq!(
        PointCloudPacked::decode(p.encode_to_vec().as_slice()).unwrap(),
        p
    );
}

#[test]
fn point_misses_preserve_componentwise_infinity_and_zero_nan_without_invalid_samples() {
    let m = metadata();
    let all = points_packet(
        &m,
        &LidarSample {
            ranges: vec![f64::INFINITY; 9],
            ..sample()
        },
        0,
    )
    .unwrap();
    assert!(!all.is_dense);
    for v in 0..3 {
        for h in 0..3 {
            let i = v * 3 + h;
            let pitch = -0.2 + v as f64 * 0.2;
            let yaw = -0.3 + h as f64 * 0.3;
            for (axis, d) in [
                pitch.cos() * yaw.cos(),
                pitch.cos() * yaw.sin(),
                pitch.sin(),
            ]
            .into_iter()
            .enumerate()
            {
                let actual = float(&all.data, i, axis);
                if d == 0. {
                    assert!(actual.is_nan());
                } else {
                    assert!(actual.is_infinite());
                    assert_eq!(actual.is_sign_negative(), d.is_sign_negative());
                }
            }
        }
    }
    let zero = points_packet(
        &m,
        &LidarSample {
            ranges: vec![0.; 9],
            ..sample()
        },
        0,
    )
    .unwrap();
    assert!(zero.is_dense);
    for i in 0..9 {
        for axis in 0..3 {
            assert_eq!(
                float(&zero.data, i, axis).to_bits() & 0x7fff_ffff,
                0,
                "XYZ is signed or positive zero"
            );
        }
        assert_eq!(
            float(&zero.data, i, 3).to_bits(),
            0,
            "intensity is positive zero"
        );
    }
    let mut mixed = sample();
    mixed.ranges[4] = f64::INFINITY;
    let mixed = points_packet(&m, &mixed, 0).unwrap();
    assert!(
        !mixed.is_dense
            && float(&mixed.data, 4, 0).is_infinite()
            && float(&mixed.data, 4, 1).is_nan()
    );
}

#[test]
fn point_packet_and_admission_bound_malformed_ranges_pose_time_grid_and_float_domain() {
    let m = metadata();
    for ranges in [
        vec![],
        vec![1.; 8],
        vec![1.; 10],
        vec![f64::NAN; 9],
        vec![f64::NEG_INFINITY; 9],
        vec![21.; 9],
        vec![-1.; 9],
    ] {
        assert!(points_packet(&m, &LidarSample { ranges, ..sample() }, 0).is_none());
    }
    for (translation, rotation) in [
        ([f64::NAN, 0., 0.], [0., 0., 0., 1.]),
        ([0.; 3], [0.; 4]),
        ([0.; 3], [f64::INFINITY; 4]),
    ] {
        assert!(
            points_packet(
                &m,
                &LidarSample {
                    world_pose: WorldPose {
                        translation,
                        rotation
                    },
                    ..sample()
                },
                0
            )
            .is_none()
        );
    }
    assert!(
        points_packet(
            &m,
            &LidarSample {
                time: Duration::new(i64::MAX as u64 + 1, 0),
                ..sample()
            },
            0
        )
        .is_none()
    );
    for field in 0..6 {
        let mut bad = m.clone();
        match field {
            0 => bad.vertical.samples = usize::MAX,
            1 => bad.config.samples = 129,
            2 => bad.vertical.samples = 0,
            3 => bad.angle_step = 0.1,
            4 => bad.frame_id = "x".repeat(1025),
            _ => bad.config.range_max = f64::from(f32::MAX) * 2.,
        }
        assert!(points_endpoint(&bad, &selected()).is_err());
    }
    let mut max = m;
    max.config.range_max = f64::from(f32::MAX);
    assert!(
        points_packet(
            &max,
            &LidarSample {
                ranges: vec![max.config.range_max; 9],
                ..sample()
            },
            0
        )
        .is_some()
    );
    max.config.range_max = f64::from_bits(max.config.range_max.to_bits() + 1);
    assert!(points_endpoint(&max, &selected()).is_err());
    assert!(
        points_endpoint(&max, &ServerOptions::default())
            .unwrap()
            .is_none()
    );
    assert!(
        packet(
            &max,
            &LidarSample {
                ranges: vec![max.config.range_max; 9],
                ..sample()
            },
            0
        )
        .is_some()
    );
}

#[test]
fn point_caps_and_independent_topic_remaps_are_admitted_before_any_declaration() {
    let mut m = metadata();
    m.config.samples = 16;
    m.angle_step = 0.6 / 15.;
    m.vertical.samples = 8;
    m.frame_id = "é".repeat(512);
    let mut options = selected();
    assert!(points_endpoint(&m, &options).unwrap().is_some());
    let p = points_packet(
        &m,
        &LidarSample {
            ranges: vec![20.; 128],
            ..sample()
        },
        u64::MAX,
    )
    .unwrap();
    assert!(p.encoded_len() <= 4096);
    options.max_message_bytes = 4096;
    assert!(
        points_endpoint(&m, &options).is_err(),
        "scan repeats the full frame and exceeds this global cap"
    );
    options.max_message_bytes = 8192;
    assert!(points_endpoint(&m, &options).is_ok());
    m.frame_id.push('x');
    assert!(points_endpoint(&m, &options).is_err());
    assert!(!ServerOptions::default().lidar_point_clouds);
    options.max_message_bytes = 4095;
    assert!(options.validate().is_err());
    options.lidar_point_clouds = false;
    assert!(options.validate().is_ok());
    let mut options = selected();
    options
        .topic_remaps
        .insert("/grid".into(), "/scan_elsewhere".into());
    assert_eq!(
        points_endpoint(&metadata(), &options).unwrap(),
        Some(options.topic_key("/grid/points").unwrap())
    );
    options
        .topic_remaps
        .insert("/grid/points".into(), "/cloud_elsewhere".into());
    assert_eq!(
        points_endpoint(&metadata(), &options).unwrap(),
        Some(options.topic_key("/cloud_elsewhere").unwrap())
    );
    let mut long = selected();
    long.partition = "p".repeat(1024);
    assert!(points_endpoint(&metadata(), &long).is_err());
    let overhead = long.topic_key("/grid/points").unwrap().len() - long.partition.len();
    long.partition = "p".repeat(1024 - overhead);
    assert_eq!(
        points_endpoint(&metadata(), &long).unwrap().unwrap().len(),
        1024
    );
    long.partition.push('p');
    assert!(points_endpoint(&metadata(), &long).is_err());
}

#[test]
fn typed_sixteen_declarations_retire_independently_and_reject_stale_pair_before_scan_put() {
    use crate::lidar_io::{Kind, LidarDeclarations, LidarJob};
    let session = zenoh::open(config()).wait().unwrap();
    let (wake, _) = mpsc::sync_channel(1);
    let mut declarations = LidarDeclarations::new();
    let mut leases = Vec::new();
    for i in 0..8 {
        for kind in [Kind::Scan, Kind::Points] {
            let key = format!("@/points-registry@/{}-{kind:?}", i);
            let lease = declarations
                .get_or_create_kind(&session, key.clone(), "process", "node", wake.clone(), kind)
                .unwrap();
            assert!(lease.matches(&key, kind));
            assert!(
                declarations
                    .get_or_create_kind(
                        &session,
                        key,
                        "process",
                        "node",
                        wake.clone(),
                        if kind == Kind::Scan {
                            Kind::Points
                        } else {
                            Kind::Scan
                        }
                    )
                    .is_err()
            );
            leases.push(lease);
        }
    }
    assert!(
        declarations
            .get_or_create(
                &session,
                "@/points-registry@/ninth".into(),
                "process",
                "node",
                wake.clone()
            )
            .is_err()
    );
    let scan = leases.remove(0);
    let points = leases.remove(0);
    let sub = session
        .declare_subscriber("@/points-registry@/0-Scan")
        .wait()
        .unwrap();
    matching(&scan, true);
    assert!(
        declarations
            .publish_pair(
                LidarJob::new(&scan, vec![1]),
                LidarJob::new(&points, vec![2; 4097])
            )
            .is_err()
    );
    assert!(
        !declarations
            .publish_pair(
                LidarJob::new(&points, vec![1]),
                LidarJob::new(&scan, vec![2])
            )
            .unwrap()
    );
    assert!(sub.try_recv().unwrap().is_none());
    let scan_job = LidarJob::new(&scan, vec![1]);
    let point_job = LidarJob::new(&points, vec![2]);
    let weak = Arc::downgrade(&points);
    drop(points);
    assert!(weak.upgrade().is_none());
    assert!(!declarations.publish_pair(scan_job, point_job).unwrap());
    assert!(
        sub.try_recv().unwrap().is_none(),
        "both weak identities checked before any put"
    );
    declarations.reconcile(false);
    assert!(
        declarations
            .get_or_create_kind(
                &session,
                "@/points-registry@/0-Points".into(),
                "process",
                "node",
                wake.clone(),
                Kind::Scan
            )
            .is_err(),
        "eight Scan owners still live"
    );
    let retired_scan = leases.remove(0);
    assert!(retired_scan.matches("@/points-registry@/1-Scan", Kind::Scan));
    let retired_scan_weak = Arc::downgrade(&retired_scan);
    drop(retired_scan);
    assert!(retired_scan_weak.upgrade().is_none());
    declarations.reconcile(false);
    let replacement = declarations
        .get_or_create_kind(
            &session,
            "@/points-registry@/0-Points".into(),
            "process",
            "node",
            wake,
            Kind::Scan,
        )
        .unwrap();
    assert!(!replacement.is_points());
    session.close().wait().unwrap();
    declarations.reconcile(true);
    assert!(scan.matching().is_err() && leases.iter().all(|lease| lease.matching().is_err()));
    assert!(
        !declarations
            .publish(LidarJob::new(&replacement, vec![1]))
            .unwrap()
    );
    drop(leases);
    drop(scan);
    drop(replacement);
    declarations.reconcile(false);
    assert!(declarations.is_empty());
}

#[test]
fn paired_real_wire_keeps_sample_atomic_on_invalid_data_and_preserves_scan_sequence() {
    let noisy = xml().replace(
        "</range></lidar>",
        "</range><noise><type>gaussian</type><mean>0.1</mean><stddev>0.2</stddev></noise></lidar>",
    );
    let mut host = Server::with_zenoh_config(
        simulation(&noisy),
        "default",
        selected(),
        config(),
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    let session = host.zenoh_session();
    let scan = session
        .declare_subscriber(host.lidars[0].endpoint.clone())
        .wait()
        .unwrap();
    let cloud = session
        .declare_subscriber(host.lidars[0].points.as_ref().unwrap().endpoint.clone())
        .wait()
        .unwrap();
    matching(&host.lidars[0].publisher, true);
    matching(&host.lidars[0].points.as_ref().unwrap().publisher, true);
    let output = &host.lidars[0];
    let point = &output.points.as_ref().unwrap().publisher;
    assert!(
        host.world_io
            .publish_lidar_pair(&output.publisher, vec![0; 4097], point, vec![0; 4096])
            .is_err()
    );
    assert!(
        host.world_io
            .publish_lidar_pair(&output.publisher, vec![0], point, vec![0; 4097])
            .is_err()
    );
    assert!(host.world_io.publish_lidar(point, vec![0]).is_err());
    host.set_paused(false);
    host.update().unwrap();
    let a = scan.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    let b = cloud.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    assert_eq!(
        a.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.LaserScan"
    );
    assert_eq!(
        b.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.PointCloudPacked"
    );
    let a = LaserScan::decode(a.payload().to_bytes().as_ref()).unwrap();
    let b = PointCloudPacked::decode(b.payload().to_bytes().as_ref()).unwrap();
    assert_eq!(a.header, b.header);
    for v in 0..3 {
        for h in 0..3 {
            let i = v * 3 + h;
            let pitch = -0.2 + v as f64 * 0.2;
            let yaw = -0.3 + h as f64 * 0.3;
            for (axis, d) in [
                pitch.cos() * yaw.cos(),
                pitch.cos() * yaw.sin(),
                pitch.sin(),
            ]
            .into_iter()
            .enumerate()
            {
                assert!(
                    (f64::from(float(&b.data, i, axis)) - d * a.ranges[i]).abs() < 1e-6,
                    "same noisy acquired range"
                );
            }
        }
    }
    assert_eq!(a.header.unwrap().data[1].value, ["0"]);
    let entity = host.lidars[0].entity;
    host.simulation.step_exact(9).unwrap();
    let good = host
        .simulation
        .world()
        .get::<LidarSample>(entity)
        .unwrap()
        .ranges
        .clone();
    host.simulation
        .world_mut()
        .get_mut::<LidarSample>(entity)
        .unwrap()
        .ranges[0] = f64::NAN;
    let before = (
        host.lidars[0].last_sample,
        host.lidars[0].sequence,
        host.lidars[0].points.as_ref().unwrap().sequence,
        host.counters.lidars_queued,
    );
    host.publish_lidars();
    assert_eq!(
        before,
        (
            host.lidars[0].last_sample,
            host.lidars[0].sequence,
            host.lidars[0].points.as_ref().unwrap().sequence,
            host.counters.lidars_queued
        )
    );
    assert!(scan.try_recv().unwrap().is_none() && cloud.try_recv().unwrap().is_none());
    host.simulation
        .world_mut()
        .get_mut::<LidarSample>(entity)
        .unwrap()
        .ranges = good;
    host.publish_lidars();
    let a = LaserScan::decode(
        scan.recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    let b = PointCloudPacked::decode(
        cloud
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(a.header, b.header);
    assert_eq!(a.header.unwrap().data[1].value, ["1"]);
    assert_eq!(host.counters.invalid_lidar_samples, 1);
}

#[test]
fn cloud_only_demand_default_scan_bits_and_independent_late_join_sequence_are_preserved() {
    let mut host = server(selected());
    let session = host.zenoh_session();
    host.set_paused(false);
    for _ in 0..20 {
        host.update().unwrap();
    }
    assert_eq!(host.counters.lidars_queued, 0);
    assert!(
        host.simulation
            .world()
            .get::<LidarSample>(host.lidars[0].entity)
            .is_none()
    );
    let scan = session
        .declare_subscriber(host.lidars[0].endpoint.clone())
        .wait()
        .unwrap();
    matching(&host.lidars[0].publisher, true);
    host.update().unwrap();
    let a = LaserScan::decode(
        scan.recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    let sample = host
        .simulation
        .world()
        .get::<LidarSample>(host.lidars[0].entity)
        .unwrap();
    assert_eq!(
        a.encode_to_vec(),
        packet(&host.lidars[0].metadata, sample, 0)
            .unwrap()
            .encode_to_vec()
    );
    assert_eq!(host.lidars[0].points.as_ref().unwrap().sequence, 0);
    drop(scan);
    matching(&host.lidars[0].publisher, false);
    let cloud = session
        .declare_subscriber(host.lidars[0].points.as_ref().unwrap().endpoint.clone())
        .wait()
        .unwrap();
    matching(&host.lidars[0].points.as_ref().unwrap().publisher, true);
    for _ in 0..10 {
        host.update().unwrap();
    }
    let b = PointCloudPacked::decode(
        cloud
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(b.header.unwrap().data[1].value, ["0"]);
    assert_eq!(host.lidars[0].sequence, 2);
    drop(cloud);
    matching(&host.lidars[0].points.as_ref().unwrap().publisher, false);
    let count = host.counters.lidars_queued;
    for _ in 0..20 {
        host.update().unwrap();
    }
    assert_eq!(host.counters.lidars_queued, count);
}

#[test]
fn reset_preflight_rejects_swapped_typed_outputs_then_removal_and_loaded_reset_restore_both() {
    let mut host = server(selected());
    let initial_id = host.lidars[0].identity;
    let entity = host.lidars[0].entity;
    let mut prepared = prepare_reset(&host).unwrap();
    assert_eq!(prepared.len(), 2);
    let scan_key = host.lidars[0].endpoint.clone();
    let points_key = host.lidars[0].points.as_ref().unwrap().endpoint.clone();
    let scan = prepared.remove(&scan_key).unwrap();
    let points = prepared.remove(&points_key).unwrap();
    prepared.insert(scan_key.clone(), points);
    prepared.insert(points_key.clone(), scan);
    let clock = (host.simulation.epoch(), host.simulation.elapsed());
    assert!(matches!(
        streams(
            &host.simulation,
            &host.world_io,
            &host.network_config,
            &host.imus,
            Some(&prepared)
        ),
        Err(Error::ResetLidars)
    ));
    assert_eq!(clock, (host.simulation.epoch(), host.simulation.elapsed()));
    drop(prepared);
    assert_eq!(
        host.simulation
            .world()
            .get::<EntityId>(entity)
            .unwrap()
            .get(),
        initial_id
    );
    let scan = Arc::downgrade(&host.lidars[0].publisher);
    let points = Arc::downgrade(&host.lidars[0].points.as_ref().unwrap().publisher);
    let owner = host
        .simulation
        .world()
        .iter_entities()
        .find(|e| e.get::<SceneName>().is_some_and(|n| n.0 == "default::rig"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    remove_model(&mut host.simulation, owner).unwrap();
    host.update().unwrap();
    assert!(host.lidars.is_empty() && scan.upgrade().is_none() && points.upgrade().is_none());
    assert_eq!(prepare_reset(&host).unwrap().len(), 2);
    assert!(host.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    host.update().unwrap();
    assert_eq!(host.lidars[0].identity, initial_id);
    assert_eq!(
        (
            host.lidars[0].sequence,
            host.lidars[0].points.as_ref().unwrap().sequence
        ),
        (0, 0)
    );
}

#[test]
fn cloud_reservations_cover_live_removed_original_and_legacy_drive_domains() {
    let mut host = server(selected());
    let key = host.lidars[0].points.as_ref().unwrap().endpoint.clone();
    for (lidars, legacy) in [(true, false), (true, true)] {
        assert!(
            sensor_reserved_keys_from_base(
                &host.simulation,
                &host.network_config,
                lidars,
                legacy,
                BTreeSet::new()
            )
            .unwrap()
            .contains(&key)
        );
    }
    assert!(
        diff_drive_legacy_keys(
            &host.simulation,
            &host.world_io,
            &host.network_config,
            false
        )
        .unwrap()
        .contains(&key)
    );
    let owner = host
        .simulation
        .world()
        .iter_entities()
        .find(|e| e.get::<SceneName>().is_some_and(|n| n.0 == "default::rig"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    remove_model(&mut host.simulation, owner).unwrap();
    host.update().unwrap();
    assert!(
        sensor_reserved_keys_from_base(
            &host.simulation,
            &host.network_config,
            true,
            true,
            BTreeSet::new()
        )
        .unwrap()
        .contains(&key)
    );
    assert!(
        diff_drive_legacy_keys(&host.simulation, &host.world_io, &host.network_config, true)
            .unwrap()
            .contains(&key)
    );
    let occupied = BTreeSet::from([key]);
    assert!(matches!(
        reset_plan(&host, &occupied),
        Err(Error::LidarIdentity)
    ));
    assert!(
        reset_plan(&host, &BTreeSet::new()).is_ok(),
        "repair preserves original reset reservations"
    );
    let mut options = host.network_config.clone();
    options.lidar_point_clouds = false;
    assert!(
        !sensor_reserved_keys_from_base(&host.simulation, &options, true, true, BTreeSet::new())
            .unwrap()
            .contains(&options.topic_key("/grid/points").unwrap())
    );

    let alt = "<sensor name='alt' type='altimeter'><update_rate>100</update_rate><topic>/collision</topic><frame_id>alt</frame_id><altimeter/></sensor>";
    let mixed = xml().replace(
        "</collision></link></model></world>",
        &format!("</collision>{alt}</link></model></world>"),
    );
    let mut sim = simulation(&mixed);
    let mut options = host.network_config.clone();
    options
        .topic_remaps
        .insert("/grid/points".into(), "/collision".into());
    for removed in [false, true] {
        if removed {
            let owner = sim
                .world()
                .iter_entities()
                .find(|e| e.get::<SceneName>().is_some_and(|n| n.0 == "default::rig"))
                .unwrap()
                .get::<EntityId>()
                .unwrap()
                .get();
            remove_model(&mut sim, owner).unwrap();
        }
        let reserved =
            typed_reserved_keys(&sim, &host.world_io, &options, PublicationFamily::Altimeter)
                .unwrap();
        assert!(matches!(
            crate::altimeter::streams(&sim, &host.world_io, &options, &reserved, None),
            Err(Error::AltimeterIdentity)
        ));
        let mut repaired = options.clone();
        repaired
            .topic_remaps
            .insert("/collision".into(), "/repaired-altimeter".into());
        let reserved = typed_reserved_keys(
            &sim,
            &host.world_io,
            &repaired,
            PublicationFamily::Altimeter,
        )
        .unwrap();
        assert_eq!(
            crate::altimeter::streams(&sim, &host.world_io, &repaired, &reserved, None)
                .unwrap()
                .len(),
            1
        );
    }
}

#[test]
fn eight_cloud_owners_preflight_ninth_duplicate_and_removed_original_without_partial_leases() {
    let text = xml();
    let start = text.find("<sensor ").unwrap();
    let end = text[start..].find("</sensor>").unwrap() + start + 9;
    let sensor = &text[start..end];
    let fleet = |n: usize| {
        text.replacen(
            sensor,
            &(0..n)
                .map(|i| {
                    sensor
                        .replace("name='scan'", &format!("name='scan{i}'"))
                        .replace("/grid", &format!("/grid{i}"))
                })
                .collect::<String>(),
            1,
        )
    };
    let options = selected();
    let sim = simulation(&fleet(8));
    let host = Server::with_zenoh_config(
        sim,
        "default",
        options.clone(),
        config(),
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    assert_eq!(host.lidars.len(), 8);
    assert_eq!(prepare_reset(&host).unwrap().len(), 16);
    for stream in &host.lidars {
        assert!(
            stream
                .publisher
                .matches(&stream.endpoint, lidar_io::Kind::Scan)
        );
        let point = stream.points.as_ref().unwrap();
        assert!(
            point
                .publisher
                .matches(&point.endpoint, lidar_io::Kind::Points)
        );
    }
    assert!(matches!(
        load_world_with_sdf_step(
            &fleet(9),
            "ninth.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        ),
        Err(gz_sim_scene::LoadError::Unsupported {
            feature: "multiple CPU lidars",
            ..
        })
    ));
    let mut ninth = simulation(&fleet(8));
    ninth.world_mut().spawn((metadata(),));
    assert!(matches!(
        admission_plan(&ninth, &options, || Ok(BTreeSet::new()), None),
        Err(Error::LidarIdentity)
    ));
    let duplicate = simulation(&fleet(2).replace("/grid1", "/grid0"));
    assert!(matches!(
        admission_plan(&duplicate, &options, || Ok(BTreeSet::new()), None),
        Err(Error::LidarIdentity)
    ));
    let sibling = simulation(&fleet(2).replace("/grid1", "/grid0/points"));
    assert!(matches!(
        admission_plan(&sibling, &options, || Ok(BTreeSet::new()), None),
        Err(Error::LidarIdentity)
    ));
    // A prospective later owner cannot steal an endpoint restored by loaded reset.
    let mut removed = simulation(&fleet(2));
    let owner = removed
        .world()
        .iter_entities()
        .find(|e| e.get::<SceneName>().is_some_and(|n| n.0 == "default::rig"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    remove_model(&mut removed, owner).unwrap();
    let reserved = BTreeSet::from([options.topic_key("/grid0/points").unwrap()]);
    assert!(matches!(
        originals(&removed, &options, &reserved),
        Err(Error::LidarIdentity)
    ));
    assert_eq!(
        originals(&removed, &options, &BTreeSet::new())
            .unwrap()
            .len(),
        2
    );
}

#[test]
fn cloud_reset_metadata_fault_is_terminal_and_retires_both_leases_without_repaired_output() {
    fn change(mut publications: bevy_ecs::prelude::Query<&mut LidarPublication>) {
        for mut metadata in &mut publications {
            metadata.frame_id.push_str("-drift");
        }
    }
    let mut host = server(selected());
    let scan = Arc::downgrade(&host.lidars[0].publisher);
    let points = Arc::downgrade(&host.lidars[0].points.as_ref().unwrap().publisher);
    host.simulation
        .add_systems(gz_sim_core::Phase::Reset, change);
    assert!(host.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(host.update(), Err(Error::ResetLidars)));
    assert!(scan.upgrade().is_none() && points.upgrade().is_none());
    assert!(matches!(host.update(), Err(Error::ResetStreams)));
}
