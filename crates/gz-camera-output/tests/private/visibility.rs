// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
const XML: &str = include_str!("../../../../examples/m75_camera_visibility.sdf");
const K: [[f64; 4]; 2] = [[40.0, 48.0, 20.0, 25.0], [20.0, 24.0, 42.0, 20.0]];
fn app(sim: Simulation, settings: &CameraServerOptions) -> CameraServer {
    CameraServer::new_mixed_pair_with_server_session(
        sim,
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        settings,
    )
    .unwrap()
}
fn info(
    sub: &zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>,
    owner: usize,
) {
    let sample = sub.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.CameraInfo"
    );
    let wire = sample.payload().to_bytes();
    let v = CameraInfo::decode(wire.as_ref()).unwrap();
    assert_eq!(v.encode_to_vec(), wire.as_ref());
    let [fx, fy, cx, cy] = K[owner];
    assert_eq!(
        v.intrinsics.unwrap().k,
        [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    );
    assert_eq!(
        v.projection.unwrap().p,
        [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    );
    let h = v.header.unwrap();
    let stamp = h.stamp.unwrap();
    assert_eq!((stamp.sec, stamp.nsec), (0, 1_000_000));
    assert_eq!(h.data.len(), 1);
    assert_eq!(h.data[0].key, "frame_id");
    assert_eq!(
        h.data[0].value,
        [format!(
            "rig_{}/body/camera",
            if owner == 0 { "a" } else { "b" }
        )]
    );
}
#[test]
fn visibility_zero_masks_cannot_hide_invalid_world_data_before_external_io() {
    if isolation::child(
        "application::tests::visibility::visibility_zero_masks_cannot_hide_invalid_world_data_before_external_io",
        30,
    ) {
        return;
    }
    let settings = settings();
    for fault in 0..4 {
        let mut sim = load(
            &XML.replace(
                "<visibility_mask>5</visibility_mask>",
                "<visibility_mask>0</visibility_mask>",
            )
            .replace(
                "<visibility_mask>6</visibility_mask>",
                "<visibility_mask>0</visibility_mask>",
            ),
        );
        let visual = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::occluder_a::body::surface")
            })
            .unwrap()
            .id();
        let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
        match fault {
            0 => {
                sim.world_mut()
                    .get_mut::<gz_sim_scene::VisualProperties>(visual)
                    .unwrap()
                    .0
                    .visibility_flags = 0x1000_0000
            }
            1 => {
                sim.world_mut()
                    .get_mut::<gz_sim_scene::VisualMaterial>(visual)
                    .unwrap()
                    .0
                    .diffuse[0] = f32::NAN
            }
            2 => {
                sim.world_mut()
                    .get_mut::<gz_sim_scene::WorldPose>(visual)
                    .unwrap()
                    .translation[0] = f64::INFINITY
            }
            3 => {
                sim.world_mut()
                    .entity_mut(visual)
                    .remove::<gz_sim_scene::VisualGeometry>();
            }
            _ => unreachable!(),
        }
        assert!(prepare_mixed_pair(&sim, &options(), &settings).is_err());
        assert_eq!(
            (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
            before
        );
        let reserved = TcpListener::bind("127.0.0.1:0").unwrap();
        let mut network = config();
        network
            .insert_json5(
                "listen/endpoints",
                &format!("[\"tcp/{}\"]", reserved.local_addr().unwrap()),
            )
            .unwrap();
        assert!(
            matches!(
                CameraServer::new_mixed_pair_with_server_session(
                    sim,
                    "default",
                    options(),
                    network,
                    NonZeroUsize::new(1).unwrap(),
                    &settings
                ),
                Err(CameraServerError::Camera(CameraOutputError::Snapshot(_)))
            ),
            "hidden invalid world fault{fault} must precede external listener bind"
        );
    }
}
#[test]
fn visibility_mask_drift_retires_and_loaded_reset_restores_masks_flags_and_identity() {
    if isolation::child(
        "application::tests::visibility::visibility_mask_drift_retires_and_loaded_reset_restores_masks_flags_and_identity",
        45,
    ) {
        return;
    }
    for owner in 0..2 {
        let settings = settings();
        let mut sim = load(XML);
        sim.set_paused(false);
        let camera = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>().is_some_and(|n| {
                    n.0 == format!(
                        "default::rig_{}::body::camera",
                        if owner == 0 { "a" } else { "b" }
                    )
                })
            })
            .unwrap()
            .id();
        let trigger = Arc::new(AtomicBool::new(false));
        let event = Arc::clone(&trigger);
        sim.add_systems(
            gz_sim_core::Phase::Finalize,
            move |world: &mut bevy_ecs::world::World| {
                if event.swap(false, Ordering::AcqRel) {
                    let mut sensor = world.get_mut::<CameraSensor>(camera).unwrap();
                    let sdformat::SensorConfig::Camera(c) =
                        &mut Arc::make_mut(&mut sensor.0).config
                    else {
                        unreachable!()
                    };
                    c.visibility_mask = 0;
                }
            },
        );
        let mut app = app(sim, &settings);
        let ids = app.pair_ids.unwrap();
        let kinds = app.pair_sensor_kinds;
        let world = app.server().zenoh_session();
        let subs: Vec<_> = ["/m75/rgb/camera_info", "/m75/depth/camera_info"]
            .into_iter()
            .map(|topic| {
                world
                    .declare_subscriber(app.server().options().topic_key(topic).unwrap())
                    .wait()
                    .unwrap()
            })
            .collect();
        let until = Instant::now() + Duration::from_secs(5);
        for id in ids {
            while app.camera_demand_for(id).unwrap() != Some((false, true)) {
                assert!(Instant::now() < until);
                std::thread::yield_now();
            }
        }
        assert_eq!(
            app.tick(&settings).unwrap().sim_time,
            Duration::from_millis(1)
        );
        for row in app.capture_all().unwrap() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::InfoPublished));
        }
        for (i, sub) in subs.iter().enumerate() {
            info(sub, i);
            assert!(app.camera_worker_diagnostics(ids[i]).is_none());
        }
        trigger.store(true, Ordering::Release);
        app.tick(&settings).unwrap();
        let before = app.server().simulation().elapsed();
        assert!(matches!(
            app.capture_all(),
            Err(CameraServerError::Camera(CameraOutputError::OwnerInvalid))
        ));
        assert!(!app.has_camera_output());
        assert_eq!(app.server().simulation().elapsed(), before);
        for sub in &subs {
            assert!(sub.try_recv().unwrap().is_none());
        }
        assert!(!world.is_closed());
        control(
            &mut app,
            &world,
            WorldControl {
                pause: true,
                reset: Some(WorldReset {
                    all: true,
                    ..Default::default()
                }),
                ..Default::default()
            },
        );
        app.prepare_io(&settings).unwrap();
        assert_eq!(app.server().simulation().epoch(), 1);
        assert_eq!(app.pair_ids, Some(ids));
        assert_eq!(app.pair_sensor_kinds, kinds);
        for (i, id) in ids.iter().copied().enumerate() {
            let e = entity_by_id(app.server().simulation().world(), id).unwrap();
            assert_eq!(
                app.server()
                    .simulation()
                    .world()
                    .get::<CameraSensor>(e)
                    .unwrap()
                    .0
                    .camera()
                    .unwrap()
                    .visibility_mask,
                [5, 6][i]
            );
        }
        for (name, flag) in [("backdrop", 4), ("occluder_a", 1), ("occluder_b", 2)] {
            let visual = app
                .server()
                .simulation()
                .world()
                .iter_entities()
                .find(|e| {
                    e.get::<Name>()
                        .is_some_and(|n| n.0 == format!("default::{name}::body::surface"))
                })
                .unwrap();
            assert_eq!(
                visual
                    .get::<gz_sim_scene::VisualProperties>()
                    .unwrap()
                    .0
                    .visibility_flags,
                flag
            );
        }
        let paused = app.tick(&settings).unwrap();
        assert!(paused.paused && paused.sim_time.is_zero() && paused.dt.is_zero());
        for row in app.capture_all().unwrap() {
            assert!(row.result.unwrap().requested.is_none());
        }
        let until = Instant::now() + Duration::from_secs(5);
        for id in ids {
            while app.camera_demand_for(id).unwrap() != Some((false, true)) {
                assert!(Instant::now() < until);
                std::thread::yield_now();
            }
        }
        assert!(app.server_mut().apply_local_control(WorldControl {
            pause: true,
            step: true,
            ..Default::default()
        }));
        assert_eq!(
            app.tick(&settings).unwrap().sim_time,
            Duration::from_millis(1)
        );
        for row in app.capture_all().unwrap() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::InfoPublished));
        }
        for (i, sub) in subs.iter().enumerate() {
            info(sub, i);
            assert!(app.camera_worker_diagnostics(ids[i]).is_none());
        }
        app.shutdown();
        assert!(world.is_closed());
    }
}
