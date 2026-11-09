// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sensors::logical_camera::{
    InvalidLogicalCameraInput, LogicalCameraConfig, LogicalCameraModel, LogicalCameraPose,
    LogicalCameraReading,
};
use gz_sim_proto::prost::Name;
use gz_sim_scene::{Name as SceneName, PhysicsEngine, load_world_with_sdf_step, remove_model};
const XML: &str = include_str!("../../../gz-sim-scene/tests/fixtures/m30_logical_camera.sdf");
fn metadata() -> LogicalCameraPublication {
    LogicalCameraPublication {
        entity_name: "rig::body::logical_camera".into(),
        frame_id: "rig/body/logical_camera".into(),
        topic: "/m30/logical_camera".into(),
        frustum_topic: "/m30/logical_camera/frustum".into(),
        config: LogicalCameraConfig {
            near: 0.5,
            far: 10.0,
            horizontal_fov: std::f64::consts::FRAC_PI_2,
            aspect_ratio: 1.0,
        },
    }
}
fn sample() -> LogicalCameraSample {
    LogicalCameraSample {
        time: Duration::new(2, 123),
        epoch: 4,
        reading: Ok(LogicalCameraReading {
            pose: LogicalCameraPose {
                translation: [-1.0, 0.0, 3.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
            models: vec![LogicalCameraModel {
                name: "rig".into(),
                pose: LogicalCameraPose {
                    translation: [1.0, 0.0, 0.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        }),
    }
}
#[test]
fn pair_has_native_pose_order_fields_stamp_and_only_image_sequence() {
    let (image, frustum) = packets(&metadata(), &sample(), u64::MAX).unwrap();
    assert_eq!(
        LogicalCameraImage::full_name(),
        "gz.msgs.LogicalCameraImage"
    );
    assert_eq!(
        LogicalCameraSensor::full_name(),
        "gz.msgs.LogicalCameraSensor"
    );
    assert_eq!(
        image.header.as_ref().unwrap().stamp,
        frustum.header.as_ref().unwrap().stamp
    );
    assert_eq!(
        image.header.as_ref().unwrap().stamp,
        Some(gz_sim_proto::msgs::Time { sec: 2, nsec: 123 })
    );
    assert_eq!(
        image.header.as_ref().unwrap().data[1],
        Map {
            key: "seq".into(),
            value: vec![u64::MAX.to_string()]
        }
    );
    assert_eq!(
        frustum.header.as_ref().unwrap().data,
        vec![image.header.as_ref().unwrap().data[0].clone()]
    );
    assert_eq!(
        image.pose.as_ref().unwrap().position.as_ref().unwrap().x,
        -1.0
    );
    assert_eq!(image.model[0].name, "rig");
    assert_eq!(
        image.model[0]
            .pose
            .as_ref()
            .unwrap()
            .position
            .as_ref()
            .unwrap()
            .x,
        1.0
    );
    assert_eq!((frustum.near_clip, frustum.far_clip), (0.5, 10.0));
    assert_eq!(
        LogicalCameraImage::decode(image.encode_to_vec().as_slice()).unwrap(),
        image
    );
    assert_eq!(
        LogicalCameraSensor::decode(frustum.encode_to_vec().as_slice()).unwrap(),
        frustum
    );
    let mut empty = sample();
    empty.reading.as_mut().unwrap().models.clear();
    assert!(packets(&metadata(), &empty, 0).unwrap().0.model.is_empty());
}
#[test]
fn entire_pair_rejects_invalid_field_order_stamp_or_fault() {
    for field in 0..8 {
        let mut m = metadata();
        let mut s = sample();
        match field {
            0 => m.frustum_topic = m.topic.clone(),
            1 => m.config.near = 0.0,
            2 => m.frame_id.clear(),
            3 => s.reading = Err(InvalidLogicalCameraInput),
            4 => s.reading.as_mut().unwrap().pose.rotation = [0.0; 4],
            5 => s.reading.as_mut().unwrap().models[0].pose.translation[0] = f64::NAN,
            6 => {
                let model = s.reading.as_ref().unwrap().models[0].clone();
                s.reading.as_mut().unwrap().models.push(model);
            }
            _ => s.time = Duration::MAX,
        };
        assert!(packets(&m, &s, 0).is_none());
    }
}
fn config(connect: Option<&str>) -> zenoh::Config {
    let mut c = zenoh::Config::default();
    for (p, v) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        c.insert_json5(p, v).unwrap();
    }
    if let Some(endpoint) = connect {
        c.insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]"))
            .unwrap();
    }
    c
}
fn server_with_xml(xml: &str) -> (Server, zenoh::Session) {
    let observer = zenoh::open(config(None)).wait().unwrap();
    let locators: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|v| v.to_string())
        .collect();
    assert_eq!(locators.len(), 1);
    let sim = load_world_with_sdf_step(
        xml,
        "server-camera",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let options = ServerOptions {
        partition: format!("m30-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    };
    let s = Server::with_zenoh_config(
        sim,
        "default",
        options,
        config(Some(&locators[0])),
        NonZeroUsize::new(4).unwrap(),
    )
    .unwrap();
    (s, observer)
}
fn server() -> (Server, zenoh::Session) {
    server_with_xml(XML)
}
fn wait_interest(s: &mut Server) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        s.refresh_logical_camera_interest();
        let stream = &s.logical_cameras[0];
        if stream.image.matching().unwrap() || stream.frustum.matching().unwrap() {
            break;
        }
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
}
#[test]
fn either_channel_drives_one_acquisition_both_packets_pause_and_reconnect() {
    for image_only in [true, false] {
        let (mut s, observer) = server();
        s.set_paused(false);
        for _ in 0..12 {
            s.update().unwrap();
        }
        assert_eq!(s.counters().logical_camera_images_queued, 0);
        assert!(
            s.simulation
                .world()
                .get::<LogicalCameraSample>(s.logical_cameras[0].entity)
                .is_none()
        );
        let key = if image_only {
            s.logical_cameras[0].image_endpoint.clone()
        } else {
            s.logical_cameras[0].frustum_endpoint.clone()
        };
        let subscriber = observer.declare_subscriber(key.clone()).wait().unwrap();
        wait_interest(&mut s);
        s.update().unwrap();
        let received = subscriber
            .recv_timeout(Duration::from_secs(3))
            .unwrap()
            .unwrap();
        if image_only {
            let m = LogicalCameraImage::decode(received.payload().to_bytes().as_ref()).unwrap();
            assert_eq!(m.header.unwrap().data[1].value, ["0"]);
        } else {
            let m = LogicalCameraSensor::decode(received.payload().to_bytes().as_ref()).unwrap();
            assert_eq!(m.header.unwrap().data.len(), 1);
        }
        assert_eq!(
            (
                s.counters().logical_camera_images_queued,
                s.counters().logical_camera_frustums_queued
            ),
            (1, 1)
        );
        s.set_paused(true);
        for _ in 0..3 {
            s.update().unwrap();
        }
        assert_eq!(s.counters().logical_camera_images_queued, 1);
        drop(subscriber);
        let deadline = Instant::now() + Duration::from_secs(5);
        while s.logical_cameras[0].image.matching().unwrap()
            || s.logical_cameras[0].frustum.matching().unwrap()
        {
            assert!(Instant::now() < deadline);
            std::thread::yield_now();
        }
        s.refresh_logical_camera_interest();
        assert!(
            s.simulation
                .world()
                .get::<LogicalCameraSample>(s.logical_cameras[0].entity)
                .is_none()
        );
        let subscriber = observer.declare_subscriber(key).wait().unwrap();
        wait_interest(&mut s);
        s.set_paused(false);
        for _ in 0..10 {
            s.update().unwrap();
        }
        assert!(
            subscriber
                .recv_timeout(Duration::from_secs(3))
                .unwrap()
                .is_some()
        );
        assert_eq!(s.counters().invalid_logical_camera_samples, 0);
    }
}
#[test]
fn both_endpoint_collisions_reset_drift_identity_and_removal_are_atomic() {
    let (mut s, _) = server();
    let a = s.logical_cameras[0].image_endpoint.clone();
    let b = s.logical_cameras[0].frustum_endpoint.clone();
    let image = Arc::clone(&s.logical_cameras[0].image);
    let frustum = Arc::clone(&s.logical_cameras[0].frustum);
    for key in [a, b] {
        assert!(matches!(
            streams(
                &s.simulation,
                &s.world_io,
                &s.network_config,
                &BTreeSet::from([key]),
                None
            ),
            Err(Error::LogicalCameraIdentity)
        ));
        assert!(
            Arc::ptr_eq(&image, &s.logical_cameras[0].image)
                && Arc::ptr_eq(&frustum, &s.logical_cameras[0].frustum)
        );
    }
    let prepared = prepare_reset(&s, &BTreeSet::new()).unwrap();
    let e = s.logical_cameras[0].entity;
    s.simulation
        .world_mut()
        .get_mut::<LogicalCameraPublication>(e)
        .unwrap()
        .frustum_topic = "/drift".into();
    assert!(
        streams(
            &s.simulation,
            &s.world_io,
            &s.network_config,
            &BTreeSet::new(),
            Some(&prepared)
        )
        .is_err()
    );
    s.simulation.world_mut().entity_mut(e).insert(metadata());
    s.simulation
        .world_mut()
        .entity_mut(e)
        .insert(EntityKind::Visual);
    assert!(matches!(
        streams(
            &s.simulation,
            &s.world_io,
            &s.network_config,
            &BTreeSet::new(),
            None
        ),
        Err(Error::LogicalCameraIdentity)
    ));
    s.simulation
        .world_mut()
        .entity_mut(e)
        .insert(EntityKind::Sensor);
    let rig = s
        .simulation
        .world()
        .iter_entities()
        .find(|e| e.get::<SceneName>().is_some_and(|n| n.0 == "default::rig"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    remove_model(&mut s.simulation, rig).unwrap();
    s.retire_logical_cameras();
    assert!(s.logical_cameras.is_empty());
    drop(image);
    drop(frustum);
}
#[test]
fn invalid_due_frame_never_queues_either_channel_or_advances_sequence() {
    let (mut s, _) = server();
    s.set_interest_policy(InterestPolicy::Always);
    s.set_paused(false);
    s.update().unwrap();
    let e = s.logical_cameras[0].entity;
    let counts = (
        s.counters.logical_camera_images_queued,
        s.counters.logical_camera_frustums_queued,
    );
    s.simulation
        .world_mut()
        .get_mut::<LogicalCameraSample>(e)
        .unwrap()
        .reading = Err(InvalidLogicalCameraInput);
    s.logical_cameras[0].last_sample = None;
    s.publish_logical_cameras();
    assert_eq!(
        (
            s.counters.logical_camera_images_queued,
            s.counters.logical_camera_frustums_queued
        ),
        counts
    );
    assert_eq!(s.logical_cameras[0].sequence, 1);
    assert_eq!(s.counters.invalid_logical_camera_samples, 1);
}
#[test]
fn actual_both_channels_follow_current_sample_and_loaded_reset_restarts_sequence() {
    let (mut s, observer) = server();
    let image = observer
        .declare_subscriber(s.logical_cameras[0].image_endpoint.clone())
        .wait()
        .unwrap();
    let frustum = observer
        .declare_subscriber(s.logical_cameras[0].frustum_endpoint.clone())
        .wait()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while !s.logical_cameras[0].image.matching().unwrap()
        || !s.logical_cameras[0].frustum.matching().unwrap()
    {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    let old = s.logical_cameras[0].entity;
    let id = s.logical_cameras[0].identity;
    let link = s
        .simulation
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<SceneName>()
                .is_some_and(|n| n.0 == "default::rig::body")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    gz_sim_scene::queue_link_world_velocity(
        &mut s.simulation,
        link,
        [0.5, 0.0, 0.0],
        [0.0, 0.0, 0.5],
    )
    .unwrap();
    s.set_paused(false);
    let mut first = None;
    for tick in 1..=20 {
        s.update().unwrap();
        if let Some(seq) = [1, 10, 20].iter().position(|&t| t == tick) {
            let a = image.recv_timeout(Duration::from_secs(3)).unwrap().unwrap();
            let b = frustum
                .recv_timeout(Duration::from_secs(3))
                .unwrap()
                .unwrap();
            assert_eq!(
                a.attachment().unwrap().to_bytes().as_ref(),
                b"gz.msgs.LogicalCameraImage"
            );
            assert_eq!(
                b.attachment().unwrap().to_bytes().as_ref(),
                b"gz.msgs.LogicalCameraSensor"
            );
            let a = LogicalCameraImage::decode(a.payload().to_bytes().as_ref()).unwrap();
            let b = LogicalCameraSensor::decode(b.payload().to_bytes().as_ref()).unwrap();
            let expected = packets(
                &metadata(),
                s.simulation
                    .world()
                    .get::<LogicalCameraSample>(s.logical_cameras[0].entity)
                    .unwrap(),
                seq as u64,
            )
            .unwrap();
            assert_eq!((a.clone(), b), expected);
            if let Some(first) = &first {
                assert_ne!(first, &a.pose);
            } else {
                first = Some(a.pose.clone());
            }
        }
    }
    s.set_paused(true);
    for _ in 0..3 {
        s.update().unwrap();
    }
    assert!(image.try_recv().unwrap().is_none() && frustum.try_recv().unwrap().is_none());
    assert!(s.apply_local_control(gz_sim_proto::msgs::WorldControl {
        pause: true,
        reset: Some(gz_sim_proto::msgs::WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert_eq!(s.update().unwrap().epoch, 1);
    assert_eq!(s.logical_cameras[0].identity, id);
    assert_ne!(s.logical_cameras[0].entity, old);
    s.set_paused(false);
    s.update().unwrap();
    let a = LogicalCameraImage::decode(
        image
            .recv_timeout(Duration::from_secs(3))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    let b = LogicalCameraSensor::decode(
        frustum
            .recv_timeout(Duration::from_secs(3))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(a.header.as_ref().unwrap().data[1].value, ["0"]);
    assert_eq!(
        a.header.as_ref().unwrap().stamp.as_ref().unwrap().nsec,
        1_000_000
    );
    assert_eq!(a.pose.unwrap().position.unwrap().x, -1.0);
    assert_eq!(b.header.unwrap().data.len(), 1);
}
#[test]
fn aliases_all_owned_keys_missing_prepared_entries_and_closed_io_refuse_before_mutation() {
    let (mut s, _) = server();
    let image = Arc::clone(&s.logical_cameras[0].image);
    let frustum = Arc::clone(&s.logical_cameras[0].frustum);
    let e = s.logical_cameras[0].entity;
    for raw in [
        "/world/default/clock",
        "/world/default/pose/info",
        "/world/default/control",
        "/world/default/remove",
        "/world/default/remove/blocking",
        "/clock",
    ] {
        for channel in ["/m30/logical_camera", "/m30/logical_camera/frustum"] {
            let mut options = s.network_config.clone();
            options.add_topic_remap(channel, raw).unwrap();
            let occupied =
                logical_camera_reserved_keys(&s.simulation, &s.world_io, &options).unwrap();
            assert!(occupied.contains(&options.topic_key(raw).unwrap()));
            assert!(matches!(
                streams(&s.simulation, &s.world_io, &options, &occupied, None),
                Err(Error::LogicalCameraIdentity)
            ));
        }
    }
    let mut options = s.network_config.clone();
    options
        .add_topic_remap("/m30/logical_camera/frustum", "/m30/logical_camera")
        .unwrap();
    assert!(matches!(
        streams(&s.simulation, &s.world_io, &options, &BTreeSet::new(), None),
        Err(Error::LogicalCameraIdentity)
    ));
    let mut prepared = prepare_reset(&s, &BTreeSet::new()).unwrap();
    prepared.clear();
    assert!(matches!(
        streams(
            &s.simulation,
            &s.world_io,
            &s.network_config,
            &BTreeSet::new(),
            Some(&prepared)
        ),
        Err(Error::ResetLogicalCameras)
    ));
    s.simulation.world_mut().spawn(metadata());
    assert!(matches!(
        streams(
            &s.simulation,
            &s.world_io,
            &s.network_config,
            &BTreeSet::new(),
            None
        ),
        Err(Error::LogicalCameraIdentity)
    ));
    assert_eq!(
        s.simulation.world().get::<LogicalCameraPublication>(e),
        Some(&metadata())
    );
    assert!(
        Arc::ptr_eq(&image, &s.logical_cameras[0].image)
            && Arc::ptr_eq(&frustum, &s.logical_cameras[0].frustum)
    );
    let extra = s
        .simulation
        .world()
        .iter_entities()
        .find(|n| n.contains::<LogicalCameraPublication>() && n.id() != e)
        .unwrap()
        .id();
    s.simulation.world_mut().despawn(extra);
    s.world_io.session.close().wait().unwrap();
    assert!(matches!(
        streams(
            &s.simulation,
            &s.world_io,
            &s.network_config,
            &BTreeSet::new(),
            None
        ),
        Err(Error::WorldNetwork(_))
    ));
    s.refresh_logical_camera_interest();
    assert!(s.simulation.world().get::<LogicalCameraSample>(e).is_none());
}
#[test]
fn absent_camera_has_no_stream_capacity_sequence_or_acquisition() {
    let start = XML.find("    <sensor ").unwrap();
    let end = start + XML[start..].find("</sensor>").unwrap() + 9;
    let xml = format!("{}{}", &XML[..start], &XML[end..]);
    let (mut s, _) = server_with_xml(&xml);
    assert!(s.logical_cameras.is_empty() && s.logical_cameras.capacity() == 0);
    s.set_paused(false);
    for _ in 0..3 {
        s.update().unwrap();
    }
    assert_eq!(
        (
            s.counters.logical_camera_images_queued,
            s.counters.logical_camera_frustums_queued,
            s.counters.invalid_logical_camera_samples
        ),
        (0, 0, 0)
    );
    assert!(prepare_reset(&s, &BTreeSet::new()).unwrap().is_empty());
}
