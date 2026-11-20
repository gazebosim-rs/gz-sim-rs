// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Included once by the real-worker explicit_intrinsics integration target.
//! Metadata offsets do not create a new image oracle or a stereo baseline.
use super::*;
use gz_sim_scene::CameraSensor;

const XML: &str = include_str!("../../../../examples/m73_camera_projection.sdf");
const INTRINSICS: [[f64; 4]; 2] = [[52.0, 44.0, 34.0, 29.0], [36.0, 42.0, 26.0, 38.0]];
const OFFSETS: [[f64; 2]; 2] = [[-0.125, 0.25], [0.5, -0.75]];

fn source(offsets: [[f64; 2]; 2]) -> String {
    let mut source = XML.to_owned();
    for ([fx, fy, cx, cy], [tx, ty]) in K.into_iter().zip(offsets) {
        source = source.replacen(
            &format!("<p_fx>{fx}</p_fx><p_fy>{fy}</p_fy><p_cx>{cx}</p_cx><p_cy>{cy}</p_cy><tx>0</tx><ty>0</ty>"),
            &format!("<p_fx>{fx}</p_fx><p_fy>{fy}</p_fy><p_cx>{cx}</p_cx><p_cy>{cy}</p_cy><tx>{tx}</tx><ty>{ty}</ty>"),
            1,
        );
    }
    source
}

fn sub(r: &Rig, owner: usize, image: bool) -> Subscriber {
    let topic = format!(
        "/m73/{}/{}",
        if owner == 0 { "rgb" } else { "depth" },
        if image { "image" } else { "camera_info" }
    );
    r.observer
        .declare_subscriber(r.app.server().options().topic_key(&topic).unwrap())
        .wait()
        .unwrap()
}

fn info(wire: Vec<u8>, owner: usize, stamp: Duration, offsets: [f64; 2]) -> CameraInfo {
    let value = CameraInfo::decode(wire.as_slice()).unwrap();
    assert_eq!(value.encode_to_vec(), wire, "canonical complete CameraInfo");
    header(value.header.as_ref().unwrap(), owner, stamp, None);
    assert_eq!((value.width, value.height), (64, 64));
    let [fx, fy, cx, cy] = INTRINSICS[owner];
    assert_eq!(
        value.intrinsics.as_ref().unwrap().k,
        [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    );
    let [fx, fy, cx, cy] = K[owner];
    let [tx, ty] = offsets;
    assert_eq!(
        value.projection.as_ref().unwrap().p,
        [fx, 0.0, cx, tx, 0.0, fy, cy, ty, 0.0, 0.0, 1.0, 0.0]
    );
    assert_eq!(
        value.rectification_matrix,
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    );
    assert_eq!(value.distortion.as_ref().unwrap().model, 0);
    assert_eq!(value.distortion.as_ref().unwrap().k, [0.0; 5]);
    value
}

fn publish(
    r: &mut Rig,
    owner: usize,
    subscriber: &Subscriber,
    stamp: Duration,
    sequence: u64,
) -> Image {
    assert_eq!(
        r.app.pump_camera(r.ids[owner]).unwrap(),
        Some(Progress::ImagePublished)
    );
    // Unchanged full-footprint RGB and all-4096 axial-depth/+INF oracle.
    super::projection::image(
        receive(subscriber, b"gz.msgs.Image"),
        owner,
        stamp,
        sequence,
    )
}

fn metadata(r: &Rig, expected: [[f64; 2]; 2]) {
    let world = r.app.server().simulation().world();
    for (owner, id) in r.ids.iter().copied().enumerate() {
        let entity = entity_by_id(world, id).unwrap();
        let sensor = &world.get::<CameraSensor>(entity).unwrap().0;
        let camera = sensor.camera().unwrap();
        assert!(camera.lens.has_intrinsics && camera.lens.has_projection);
        assert_eq!(
            [camera.lens.projection.tx, camera.lens.projection.ty],
            expected[owner]
        );
        assert_eq!(camera.visibility_mask, u32::MAX);
        let selected = gz_render_scene::camera_calibrations(sensor, id).unwrap();
        assert_eq!(selected.projection_offsets, expected[owner]);
        assert_eq!(selected.projection.config().tx, 0.0);
        assert_eq!(selected.projection.config().ty, 0.0);
    }
}

type FirstCapture = (
    [Subscriber; 2],
    [Subscriber; 2],
    [Image; 2],
    [CameraInfo; 2],
    [Worker; 2],
);

fn first(r: &mut Rig, offsets: [[f64; 2]; 2]) -> FirstCapture {
    metadata(r, offsets);
    let images: [Subscriber; 2] = std::array::from_fn(|owner| sub(r, owner, true));
    let infos: [Subscriber; 2] = std::array::from_fn(|owner| sub(r, owner, false));
    r.matching([(true, true); 2]);
    for id in r.ids {
        assert!(r.app.camera_worker_diagnostics(id).is_none());
    }
    let stamp = r.tick();
    assert_eq!(stamp, Duration::from_millis(1));
    let rows = r.capture();
    for id in r.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    let calibration = std::array::from_fn(|owner| {
        info(
            receive(&infos[owner], b"gz.msgs.CameraInfo"),
            owner,
            stamp,
            offsets[owner],
        )
    });
    r.ready(&[0, 1]);
    let workers = [worker(r, 0), worker(r, 1)];
    assert_ne!(workers[0].pid, workers[1].pid);
    let frames = std::array::from_fn(|owner| publish(r, owner, &images[owner], stamp, 0));
    (images, infos, frames, calibration, workers)
}

#[test]
fn projection_offsets_real_workers_keep_complete_rgb_depth_bytes_and_only_change_p_translation() {
    if isolation::child(
        "projection_offsets::projection_offsets_real_workers_keep_complete_rgb_depth_bytes_and_only_change_p_translation",
        180,
    ) {
        return;
    }
    let mut baseline = Rig::with_xml(true, XML, "m76-zero-offset-control.sdf");
    let (_, _, zero_images, zero_infos, zero_workers) = first(&mut baseline, [[0.0; 2]; 2]);
    let zero_ids = baseline.ids;
    baseline.app.shutdown();
    for w in &zero_workers {
        retired(w);
    }
    baseline.observer.close().wait().unwrap();
    for offsets in [
        OFFSETS,
        [[f64::MAX, -f64::MAX], [f64::MIN_POSITIVE, -1e300]],
    ] {
        let mut r = Rig::with_xml(true, &source(offsets), "m76-offset-metadata.sdf");
        assert_eq!(r.ids, zero_ids);
        let (image_subs, info_subs, images, infos, workers) = first(&mut r, offsets);
        for (owner, image) in images.iter().enumerate() {
            assert_eq!(*image, zero_images[owner]);
            assert_eq!(image.encode_to_vec(), zero_images[owner].encode_to_vec());
            let mut expected = zero_infos[owner].clone();
            expected.projection.as_mut().unwrap().p[3] = offsets[owner][0];
            expected.projection.as_mut().unwrap().p[7] = offsets[owner][1];
            assert_eq!(infos[owner], expected);
            assert_eq!(infos[owner].encode_to_vec(), expected.encode_to_vec());
        }
        if offsets == OFFSETS {
            let mut stamps: [Vec<Duration>; 2] =
                std::array::from_fn(|_| vec![Duration::from_millis(1)]);
            while r.app.server().simulation().elapsed() < Duration::from_millis(200) {
                let stamp = r.tick();
                let rows = r.capture();
                let mut due = Vec::new();
                for (owner, sub) in info_subs.iter().enumerate() {
                    match requested(&rows, r.ids[owner]) {
                        Some(Progress::Pending) => {
                            info(
                                receive(sub, b"gz.msgs.CameraInfo"),
                                owner,
                                stamp,
                                offsets[owner],
                            );
                            due.push(owner);
                        }
                        None => {}
                        other => panic!("offset cadence {other:?}"),
                    }
                }
                r.ready(&due);
                for owner in due {
                    publish(
                        &mut r,
                        owner,
                        &image_subs[owner],
                        stamp,
                        stamps[owner].len() as u64,
                    );
                    stamps[owner].push(stamp);
                }
            }
            assert_eq!(stamps[0], [1, 100, 200].map(Duration::from_millis));
            assert_eq!(stamps[1], [1, 50, 100, 150, 200].map(Duration::from_millis));
        }
        r.pause(true);
        let paused = r.app.server().simulation().elapsed();
        for _ in 0..8 {
            assert_eq!(r.tick(), paused);
            for row in r.capture() {
                assert!(row.result.unwrap().requested.is_none());
            }
        }
        for sub in image_subs.iter().chain(&info_subs) {
            assert!(sub.try_recv().unwrap().is_none());
        }
        r.app.shutdown();
        for w in &workers {
            retired(w);
        }
        r.observer.close().wait().unwrap();
    }
}

#[test]
fn projection_offsets_camera_info_matches_complete_new_pinned_native_cpu_capture() {
    // This new input must come from the reviewed CPU-only optional mode. Its
    // absence is intentional until Root's end-of-cycle capture; no Rust-made
    // replacement or old zero-offset fixture qualifies this comparison.
    let raw = include_bytes!("../fixtures/native_camera_info_projection_offsets.bin");
    let native = CameraInfo::decode(raw.as_slice()).unwrap();
    let actual = gz_camera_output::camera_messages::camera_info_packet_with_projection_offsets(
        Duration::new(4, 5),
        "frame",
        gz_camera_output::camera_messages::IntrinsicValues {
            fx: 30.0,
            fy: 31.0,
            cx: 20.0,
            cy: 21.0,
        },
        gz_camera_output::camera_messages::IntrinsicValues {
            fx: 52.0,
            fy: 36.0,
            cx: 11.0,
            cy: 45.0,
        },
        -0.125,
        0.25,
    )
    .unwrap();
    assert_eq!(actual, native);
    assert_eq!(actual.encode_to_vec(), raw.as_slice());
    assert_eq!(native.encode_to_vec(), raw.as_slice());
    let mut expected =
        CameraInfo::decode(include_bytes!("../fixtures/native_camera_info.bin").as_slice())
            .unwrap();
    expected.projection.as_mut().unwrap().p = vec![
        52.0, 0.0, 11.0, -0.125, 0.0, 36.0, 45.0, 0.25, 0.0, 0.0, 1.0, 0.0,
    ];
    assert_eq!(native, expected);
}

#[test]
fn projection_offsets_presence_and_nonfinite_preflight_precede_all_external_io() {
    if isolation::child(
        "projection_offsets::projection_offsets_presence_and_nonfinite_preflight_precede_all_external_io",
        45,
    ) {
        return;
    }
    for has_k in [false, true] {
        for has_p in [false, true] {
            for (owner, intrinsics) in INTRINSICS.into_iter().enumerate() {
                for coordinate in 0..2 {
                    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                        let mut sim = load_world_with_sdf_step(
                            &source(OFFSETS),
                            "m76-preflight.sdf",
                            "default",
                            &sdformat::NoResources,
                            PhysicsEngine::Rapier,
                        )
                        .unwrap();
                        let wanted = format!(
                            "default::rig_{}::body::camera",
                            if owner == 0 { "a" } else { "b" }
                        );
                        let entity = sim
                            .world()
                            .iter_entities()
                            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == wanted))
                            .unwrap()
                            .id();
                        {
                            let mut metadata =
                                sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
                            let sdformat::SensorConfig::Camera(camera) =
                                &mut Arc::make_mut(&mut metadata.0).config
                            else {
                                unreachable!()
                            };
                            camera.lens.has_intrinsics = has_k;
                            camera.lens.has_projection = has_p;
                            if coordinate == 0 {
                                camera.lens.projection.tx = invalid;
                            } else {
                                camera.lens.projection.ty = invalid;
                            }
                        }
                        let sensor = &sim.world().get::<CameraSensor>(entity).unwrap().0;
                        let id = sim.world().get::<EntityId>(entity).unwrap().get();
                        let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
                        let selected = gz_render_scene::camera_calibrations(sensor, id);
                        if has_p {
                            assert!(selected.is_err());
                        } else {
                            let selected = selected.unwrap();
                            assert_eq!(selected.projection_offsets, [0.0; 2]);
                            let k = selected.intrinsics.config();
                            if has_k {
                                let [fx, fy, cx, cy] = intrinsics;
                                assert_eq!([k.fx, k.fy, k.cx, k.cy], [fx, fy, cx, cy]);
                            } else {
                                // Existing exact HFOV selector, independent of dormant P.
                                assert_eq!(k.fx, 32.0 / (std::f64::consts::FRAC_PI_4).tan());
                                assert_eq!([k.fy, k.cx, k.cy], [k.fx, 32.0, 32.0]);
                            }
                        }
                        assert_eq!(
                            (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                            before
                        );
                        if has_p {
                            let held = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
                            let mut network = config(None);
                            network
                                .insert_json5(
                                    "listen/endpoints",
                                    &format!("[\"tcp/{}\"]", held.local_addr().unwrap()),
                                )
                                .unwrap();
                            let settings = CameraServerOptions {
                                worker_path: "/nonexistent/worker".into(),
                                origin: [0.0; 3],
                                zenoh: config(None),
                            };
                            let options = ServerOptions {
                                partition: format!("m76-preflight-{}", std::process::id()),
                                ..Default::default()
                            };
                            assert!(matches!(
                                CameraServer::new_mixed_pair_with_server_session(
                                    sim,
                                    "default",
                                    options,
                                    network,
                                    NonZeroUsize::new(8).unwrap(),
                                    &settings
                                ),
                                Err(CameraServerError::Camera(
                                    gz_camera_output::CameraOutputError::Snapshot(_)
                                ))
                            ));
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn projection_offsets_info_only_presence_keeps_workers_lazy_and_complete_metadata() {
    if isolation::child(
        "projection_offsets::projection_offsets_info_only_presence_keeps_workers_lazy_and_complete_metadata",
        60,
    ) {
        return;
    }
    for has_k in [false, true] {
        for has_p in [false, true] {
            let mut source = source(OFFSETS);
            if !has_k {
                for values in INTRINSICS {
                    let [fx, fy, cx, cy] = values;
                    source = source.replace(&format!("<intrinsics><fx>{fx}</fx><fy>{fy}</fy><cx>{cx}</cx><cy>{cy}</cy><s>0</s></intrinsics>"), "");
                }
            }
            if !has_p {
                for (values, offsets) in K.into_iter().zip(OFFSETS) {
                    let [fx, fy, cx, cy] = values;
                    let [tx, ty] = offsets;
                    source = source.replace(&format!("<projection><p_fx>{fx}</p_fx><p_fy>{fy}</p_fy><p_cx>{cx}</p_cx><p_cy>{cy}</p_cy><tx>{tx}</tx><ty>{ty}</ty></projection>"), "");
                }
            }
            let mut r = Rig::with_xml(true, &source, "m76-info-presence.sdf");
            assert_eq!(r.tick(), Duration::from_millis(1));
            for row in r.capture() {
                assert_eq!(row.result.unwrap().requested, Some(Progress::NoDemand));
            }
            let subscribers: [Subscriber; 2] = std::array::from_fn(|owner| sub(&r, owner, false));
            r.matching([(false, true); 2]);
            let stamp = r.tick();
            assert_eq!(stamp, Duration::from_millis(2));
            for row in r.capture() {
                assert_eq!(row.result.unwrap().requested, Some(Progress::InfoPublished));
            }
            for (owner, subscriber) in subscribers.iter().enumerate() {
                let raw = receive(subscriber, b"gz.msgs.CameraInfo");
                let value = CameraInfo::decode(raw.as_slice()).unwrap();
                assert_eq!(value.encode_to_vec(), raw);
                header(value.header.as_ref().unwrap(), owner, stamp, None);
                assert_eq!((value.width, value.height), (64, 64));
                let selected = if has_k {
                    INTRINSICS[owner]
                } else {
                    [
                        32.0 / (std::f64::consts::FRAC_PI_4).tan(),
                        32.0 / (std::f64::consts::FRAC_PI_4).tan(),
                        32.0,
                        32.0,
                    ]
                };
                let [fx, fy, cx, cy] = selected;
                assert_eq!(
                    value.intrinsics.as_ref().unwrap().k,
                    [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
                );
                let [fx, fy, cx, cy] = if has_p { K[owner] } else { selected };
                let [tx, ty] = if has_p { OFFSETS[owner] } else { [0.0; 2] };
                assert_eq!(
                    value.projection.as_ref().unwrap().p,
                    [fx, 0.0, cx, tx, 0.0, fy, cy, ty, 0.0, 0.0, 1.0, 0.0]
                );
                assert_eq!(
                    value.rectification_matrix,
                    [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                );
                assert_eq!(value.distortion.as_ref().unwrap().model, 0);
                assert_eq!(value.distortion.as_ref().unwrap().k, [0.0; 5]);
                assert!(r.app.camera_worker_diagnostics(r.ids[owner]).is_none());
            }
            r.app.shutdown();
            r.observer.close().wait().unwrap();
        }
    }
}

#[test]
fn projection_offsets_zero_masks_keep_image_demand_and_exact_empty_frame_bytes() {
    if isolation::child(
        "projection_offsets::projection_offsets_zero_masks_keep_image_demand_and_exact_empty_frame_bytes",
        180,
    ) {
        return;
    }
    let mut baseline = None;
    for offsets in [[[0.0; 2]; 2], OFFSETS] {
        let xml =
            source(offsets).replace("<camera>", "<camera><visibility_mask>0</visibility_mask>");
        let mut r = Rig::with_xml(true, &xml, "m76-zero-mask-offset.sdf");
        let images: [Subscriber; 2] = std::array::from_fn(|owner| sub(&r, owner, true));
        let infos: [Subscriber; 2] = std::array::from_fn(|owner| sub(&r, owner, false));
        r.matching([(true, true); 2]);
        let stamp = r.tick();
        assert_eq!(stamp, Duration::from_millis(1));
        let rows = r.capture();
        for id in r.ids {
            assert_eq!(requested(&rows, id), Some(Progress::Pending));
        }
        for (owner, sub) in infos.iter().enumerate() {
            info(
                receive(sub, b"gz.msgs.CameraInfo"),
                owner,
                stamp,
                offsets[owner],
            );
        }
        r.ready(&[0, 1]);
        let workers = [worker(&r, 0), worker(&r, 1)];
        let frames: [Image; 2] = std::array::from_fn(|owner| {
            assert_eq!(
                r.app.pump_camera(r.ids[owner]).unwrap(),
                Some(Progress::ImagePublished)
            );
            let raw = receive(&images[owner], b"gz.msgs.Image");
            let frame = Image::decode(raw.as_slice()).unwrap();
            assert_eq!(frame.encode_to_vec(), raw);
            header(frame.header.as_ref().unwrap(), owner, stamp, Some(0));
            assert_eq!((frame.width, frame.height), (64, 64));
            if owner == 0 {
                assert_eq!(
                    (frame.step, frame.pixel_format_type, frame.data.len()),
                    (192, PixelFormatType::RgbInt8 as i32, 12288)
                );
                // Same pinned Bevy default clear color as the M75 zero-mask oracle.
                assert!(
                    frame
                        .data
                        .chunks_exact(3)
                        .all(|pixel| pixel == [43, 44, 47])
                );
            } else {
                assert_eq!(
                    (frame.step, frame.pixel_format_type, frame.data.len()),
                    (256, PixelFormatType::RFloat32 as i32, 16384)
                );
                assert_eq!(frame.data.chunks_exact(4).count(), 4096);
                for word in frame.data.chunks_exact(4) {
                    assert_eq!(f32::from_le_bytes(word.try_into().unwrap()), f32::INFINITY);
                }
            }
            frame
        });
        if let Some(zero) = &baseline {
            assert_eq!(&frames, zero);
        } else {
            baseline = Some(frames);
        }
        r.app.shutdown();
        for worker in &workers {
            retired(worker);
        }
        r.observer.close().wait().unwrap();
    }
}

#[test]
fn projection_offsets_tx_ty_drift_reset_and_removal_retire_buffered_frames_without_closing_world() {
    if isolation::child(
        "projection_offsets::projection_offsets_tx_ty_drift_reset_and_removal_retire_buffered_frames_without_closing_world",
        180,
    ) {
        return;
    }
    for mutation in [8, 9] {
        let mut r = Rig::with_xml(true, &source(OFFSETS), "m76-offset-lifecycle.sdf");
        let ids = r.ids;
        assert_eq!(r.tick(), Duration::from_millis(1));
        for row in r.capture() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::NoDemand));
        }
        let infos: [Subscriber; 2] = std::array::from_fn(|owner| sub(&r, owner, false));
        r.matching([(false, true); 2]);
        let stamp = r.tick();
        for row in r.capture() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::InfoPublished));
        }
        for (owner, s) in infos.iter().enumerate() {
            info(
                receive(s, b"gz.msgs.CameraInfo"),
                owner,
                stamp,
                OFFSETS[owner],
            );
            assert!(r.app.camera_worker_diagnostics(ids[owner]).is_none());
        }
        let rgb = sub(&r, 0, true);
        r.matching([(true, true), (false, true)]);
        r.advance(100);
        let rows = r.capture();
        assert_eq!(requested(&rows, ids[0]), Some(Progress::Pending));
        assert_eq!(requested(&rows, ids[1]), Some(Progress::InfoPublished));
        for (owner, s) in infos.iter().enumerate() {
            info(
                receive(s, b"gz.msgs.CameraInfo"),
                owner,
                Duration::from_millis(100),
                OFFSETS[owner],
            );
        }
        r.ready(&[0]);
        let old = worker(&r, 0);
        assert!(rgb.try_recv().unwrap().is_none());
        r.pause(true);
        r.remove.store(mutation, Ordering::Release);
        assert_eq!(r.tick(), Duration::from_millis(100));
        assert!(matches!(
            r.app.pump_camera(ids[0]),
            Err(CameraServerError::Camera(
                gz_camera_output::CameraOutputError::OwnerInvalid
            ))
        ));
        retired(&old);
        assert!(!r.app.has_camera_output());
        assert!(rgb.try_recv().unwrap().is_none());
        assert!(!r.app.server().zenoh_session().is_closed());
        r.reset();
        assert_eq!(r.ids, ids);
        metadata(&r, OFFSETS);
        r.matching([(true, true), (false, true)]);
        r.pause(false);
        let first = r.tick();
        assert_eq!(first, Duration::from_millis(1));
        let rows = r.capture();
        assert_eq!(requested(&rows, ids[0]), Some(Progress::Pending));
        assert_eq!(requested(&rows, ids[1]), Some(Progress::InfoPublished));
        for (owner, s) in infos.iter().enumerate() {
            info(
                receive(s, b"gz.msgs.CameraInfo"),
                owner,
                first,
                OFFSETS[owner],
            );
        }
        r.ready(&[0]);
        let fresh_rgb = worker(&r, 0);
        publish(&mut r, 0, &rgb, first, 0);
        let depth = sub(&r, 1, true);
        r.matching([(true, true); 2]);
        r.advance(50);
        let rows = r.capture();
        assert_eq!(requested(&rows, ids[0]), None);
        assert_eq!(requested(&rows, ids[1]), Some(Progress::Pending));
        info(
            receive(&infos[1], b"gz.msgs.CameraInfo"),
            1,
            Duration::from_millis(50),
            OFFSETS[1],
        );
        r.ready(&[1]);
        let fresh_depth = worker(&r, 1);
        publish(&mut r, 1, &depth, Duration::from_millis(50), 0);
        r.pause(true);
        r.remove.store(1, Ordering::Release);
        assert_eq!(r.tick(), Duration::from_millis(50));
        assert_eq!(r.app.pump_camera(ids[0]).unwrap(), None);
        retired(&fresh_rgb);
        assert_eq!(proc_identity(fresh_depth.pid).unwrap().0, fresh_depth.start);
        assert!(!fresh_depth.lifetime.is_retired());
        assert!(!r.app.server().zenoh_session().is_closed());
        r.remove.store(2, Ordering::Release);
        assert_eq!(r.tick(), Duration::from_millis(50));
        assert_eq!(r.app.pump_camera(ids[1]).unwrap(), None);
        retired(&fresh_depth);
        assert!(!r.app.has_camera_output());
        assert!(!r.app.server().zenoh_session().is_closed());
        r.app.shutdown();
        assert!(r.app.server().zenoh_session().is_closed());
        r.observer.close().wait().unwrap();
    }
}
