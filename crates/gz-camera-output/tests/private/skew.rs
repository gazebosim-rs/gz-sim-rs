// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
const XML: &str = include_str!("../../../../examples/m74_camera_skew.sdf");
const K: [[f64; 4]; 2] = [[52.0, 44.0, 34.0, 29.0], [36.0, 42.0, 26.0, 38.0]];
const P: [[f64; 4]; 2] = [[40.0, 48.0, 20.0, 25.0], [20.0, 24.0, 42.0, 20.0]];
const SKEW: [f64; 2] = [36.0, -48.0];
fn source(explicit_p: bool) -> String {
    if !explicit_p {
        return XML.into();
    }
    XML.replace("<s>36</s></intrinsics>","<s>36</s></intrinsics><projection><p_fx>40</p_fx><p_fy>48</p_fy><p_cx>20</p_cx><p_cy>25</p_cy><tx>0</tx><ty>0</ty></projection>")
        .replace("<s>-48</s></intrinsics>","<s>-48</s></intrinsics><projection><p_fx>20</p_fx><p_fy>24</p_fy><p_cx>42</p_cx><p_cy>20</p_cy><tx>0</tx><ty>0</ty></projection>")
}
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
    explicit_p: bool,
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
    let [pfx, pfy, pcx, pcy] = if explicit_p { P[owner] } else { K[owner] };
    // Source-native limitation: published offdiagonals remain zero, even though
    // the actual software renderer consumes the separate signed skew.
    assert_eq!(
        v.intrinsics.unwrap().k,
        [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    );
    assert_eq!(
        v.projection.unwrap().p,
        [pfx, 0.0, pcx, 0.0, 0.0, pfy, pcy, 0.0, 0.0, 0.0, 1.0, 0.0]
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
fn signed_skew_preflight_rejects_nonfinite_and_unrepresentable_k_even_with_p_override() {
    if isolation::child(
        "application::tests::skew::signed_skew_preflight_rejects_nonfinite_and_unrepresentable_k_even_with_p_override",
        30,
    ) {
        return;
    }
    let settings = settings();
    for explicit_p in [false, true] {
        for owner in 0..2 {
            for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 1e300, 1e-300] {
                let mut sim = load(&source(explicit_p));
                let entity = sim
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
                let id = sim.world().get::<EntityId>(entity).unwrap().get();
                let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
                {
                    let mut sensor = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
                    let sdformat::SensorConfig::Camera(camera) =
                        &mut Arc::make_mut(&mut sensor.0).config
                    else {
                        unreachable!()
                    };
                    camera.lens.intrinsics.skew = invalid;
                }
                assert!(prepare_mixed_pair(&sim, &options(), &settings).is_err());
                assert_eq!(
                    (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                    before
                );
                assert_eq!(entity_by_id(sim.world(), id), Some(entity));
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
                    "owner{owner} skew{invalid} explicitP{explicit_p} must fail before occupied listener bind"
                );
            }
        }
    }
}
#[test]
fn signed_skew_metadata_drift_retires_and_loaded_reset_restores_original_signed_calibrations() {
    if isolation::child(
        "application::tests::skew::signed_skew_metadata_drift_retires_and_loaded_reset_restores_original_signed_calibrations",
        45,
    ) {
        return;
    }
    for (owner, explicit_p) in [(0, false), (1, false), (0, true)] {
        let settings = settings();
        let mut sim = load(&source(explicit_p));
        sim.set_paused(false);
        let entity = sim
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
                    let mut sensor = world.get_mut::<CameraSensor>(entity).unwrap();
                    let sdformat::SensorConfig::Camera(camera) =
                        &mut Arc::make_mut(&mut sensor.0).config
                    else {
                        unreachable!()
                    };
                    camera.lens.intrinsics.skew += 1.0;
                }
            },
        );
        let mut app = app(sim, &settings);
        let ids = app.pair_ids.unwrap();
        let kinds = app.pair_sensor_kinds;
        let world = app.server().zenoh_session();
        let subs: Vec<_> = ["/m74/rgb/camera_info", "/m74/depth/camera_info"]
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
            info(sub, i, explicit_p);
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
            let sensor = app
                .server()
                .simulation()
                .world()
                .get::<CameraSensor>(e)
                .unwrap();
            let c = sensor.0.camera().unwrap();
            assert_eq!(c.lens.intrinsics.skew, SKEW[i]);
            assert_eq!(
                [
                    c.lens.intrinsics.fx,
                    c.lens.intrinsics.fy,
                    c.lens.intrinsics.cx,
                    c.lens.intrinsics.cy
                ],
                K[i]
            );
            assert_eq!(c.lens.has_projection, explicit_p);
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
            info(sub, i, explicit_p);
            assert!(app.camera_worker_diagnostics(ids[i]).is_none());
        }
        app.shutdown();
        assert!(world.is_closed());
    }
}
