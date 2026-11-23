// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../../gz-camera-output/tests/support/isolated_camera.rs"]
mod isolation;
use gz_camera_output::{CameraServer, CameraServerOptions, Progress};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_proto::{
    msgs::{CameraInfo, Image, PixelFormatType, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_scene::{CameraSensor, PhysicsEngine, load_world_with_sdf_step};
use gz_sim_server::ServerOptions;
use std::{
    collections::BTreeSet,
    num::NonZeroUsize,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};
use zenoh::Wait;
fn network(endpoint: Option<&str>) -> zenoh::Config {
    let mut c = zenoh::Config::default();
    let connect = endpoint.map_or_else(|| "[]".into(), |e| format!("[\"{e}\"]"));
    for (k, v) in [
        ("mode", "\"peer\""),
        (
            "listen/endpoints",
            if endpoint.is_some() {
                "[]"
            } else {
                "[\"tcp/127.0.0.1:0\"]"
            },
        ),
        ("connect/endpoints", &connect),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        c.insert_json5(k, v).unwrap();
    }
    c
}
fn source(count: usize) -> String {
    let xml = include_str!("../../gz-camera-output/tests/fixtures/m5-rgb.sdf");
    let start = xml.find("<model name=\"rig\">").unwrap();
    let end = start + xml[start..].find("</model>").unwrap() + "</model>".len();
    let mut rigs = String::new();
    for i in 0..count {
        rigs += &xml[start..end]
            .replace("name=\"rig\"", &format!("name=\"rig_{i}\""))
            .replace("/m5/rgb/image", &format!("/m111/{i}/image"))
            .replace("/m5/rgb/camera_info", &format!("/m111/{i}/info"))
            .replace("rig/body/camera", &format!("rig_{i}/body/camera"))
            .replace(
                "<update_rate>10</update_rate>",
                &format!("<update_rate>{}</update_rate>", [10, 20, 25, 50][i]),
            );
    }
    xml[..start].to_owned() + &rigs + &xml[end..]
}
fn application(
    count: usize,
) -> (
    CameraServer,
    CameraServerOptions,
    zenoh::Session,
    Vec<u64>,
    Arc<AtomicU64>,
) {
    let observer = zenoh::open(network(None)).wait().unwrap();
    let locators: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|l| l.to_string())
        .collect();
    assert_eq!(locators.len(), 1);
    let mut sim = load_world_with_sdf_step(
        &source(count),
        "m111-rgb",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let trigger = Arc::new(AtomicU64::new(0));
    let remove = Arc::clone(&trigger);
    sim.add_systems(
        gz_sim_core::Phase::Finalize,
        move |world: &mut bevy_ecs::world::World| {
            let id = remove.swap(0, Ordering::AcqRel);
            if id != 0 {
                let entity = entity_by_id(world, id).unwrap();
                assert!(world.despawn(entity));
            }
        },
    );
    sim.set_paused(false);
    let mut ids: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|e| e.contains::<CameraSensor>())
        .map(|e| e.get::<EntityId>().unwrap().get())
        .collect();
    ids.sort();
    let settings = CameraServerOptions {
        worker_path: env!("CARGO_BIN_EXE_gz-render-worker").into(),
        origin: [0.; 3],
        zenoh: network(Some(&locators[0])),
    };
    let app = CameraServer::new_rgb_family_with_server_session(
        sim,
        "default",
        ServerOptions {
            partition: format!("m111-real-{}-{count}", std::process::id()),
            ..Default::default()
        },
        network(Some(&locators[0])),
        NonZeroUsize::new(8).unwrap(),
        &settings,
    )
    .unwrap();
    (app, settings, observer, ids, trigger)
}
fn matched(app: &mut CameraServer, ids: &[u64]) {
    let until = Instant::now() + Duration::from_secs(5);
    while ids
        .iter()
        .any(|id| app.camera_demand_for(*id).unwrap() != Some((true, true)))
    {
        assert!(Instant::now() < until);
        std::thread::yield_now();
    }
}
fn ready(app: &mut CameraServer, ids: &[u64]) {
    let until = Instant::now() + Duration::from_secs(30);
    let mut done = BTreeSet::new();
    while done.len() != ids.len() {
        for id in ids {
            if !done.contains(id) {
                match app.pump_camera(*id).unwrap().unwrap() {
                    Progress::FrameReady => {
                        done.insert(*id);
                    }
                    Progress::Pending
                    | Progress::Backpressure
                    | Progress::SnapshotAccepted
                    | Progress::CaptureSubmitted => {}
                    other => panic!("unexpected actual worker transition {other:?}"),
                }
            }
        }
        assert!(Instant::now() < until);
        std::thread::yield_now();
    }
    // FrameReady keeps the received frame buffered; publication is the next
    // transition and must not advance the simulation again.
    for id in ids {
        assert_eq!(
            app.pump_camera(*id).unwrap(),
            Some(Progress::ImagePublished)
        );
    }
}
fn stamp(sim: &Simulation) -> i32 {
    i32::try_from(sim.elapsed().subsec_nanos()).unwrap()
}
#[test]
fn m111_three_and_four_actual_workers_render_independent_rgb_packets_and_retire() {
    if isolation::child(
        "m111_three_and_four_actual_workers_render_independent_rgb_packets_and_retire",
        180,
    ) {
        return;
    }
    for count in [3, 4] {
        let (mut app, s, observer, ids, _) = application(count);
        let images: Vec<_> = (0..count)
            .map(|i| {
                observer
                    .declare_subscriber(
                        app.server()
                            .options()
                            .topic_key(&format!("/m111/{i}/image"))
                            .unwrap(),
                    )
                    .wait()
                    .unwrap()
            })
            .collect();
        let infos: Vec<_> = (0..count)
            .map(|i| {
                observer
                    .declare_subscriber(
                        app.server()
                            .options()
                            .topic_key(&format!("/m111/{i}/info"))
                            .unwrap(),
                    )
                    .wait()
                    .unwrap()
            })
            .collect();
        matched(&mut app, &ids);
        assert!(
            ids.iter()
                .all(|id| app.camera_worker_diagnostics(*id).is_none())
        );
        app.tick(&s).unwrap();
        for row in app.capture_all().unwrap() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::Pending));
        }
        ready(&mut app, &ids);
        let expected_stamp = stamp(app.server().simulation());
        let mut processes = BTreeSet::new();
        let mut lives = Vec::new();
        for (i, id) in ids.iter().enumerate() {
            let image_sample = images[i]
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap();
            assert_eq!(
                image_sample.attachment().unwrap().to_bytes().as_ref(),
                b"gz.msgs.Image"
            );
            let image = Image::decode(image_sample.payload().to_bytes().as_ref()).unwrap();
            assert_eq!((image.width, image.height, image.step), (64, 64, 192));
            assert_eq!(image.pixel_format_type, PixelFormatType::RgbInt8 as i32);
            assert_eq!(image.data.len(), 64 * 64 * 3);
            assert_eq!(
                &image.data[(32 * 64 + 32) * 3..(32 * 64 + 32) * 3 + 3],
                &[255, 0, 0]
            );
            assert_eq!(
                image.header.as_ref().unwrap().stamp.as_ref().unwrap().nsec,
                expected_stamp
            );
            assert!(
                image
                    .header
                    .unwrap()
                    .data
                    .iter()
                    .any(|d| d.key == "frame_id" && d.value == [format!("rig_{i}/body/camera")])
            );
            let info = CameraInfo::decode(
                infos[i]
                    .recv_timeout(Duration::from_secs(5))
                    .unwrap()
                    .unwrap()
                    .payload()
                    .to_bytes()
                    .as_ref(),
            )
            .unwrap();
            assert_eq!((info.width, info.height), (64, 64));
            assert_eq!(info.header.unwrap().stamp.unwrap().nsec, expected_stamp);
            let (pid, life) = app.camera_worker_diagnostics(*id).unwrap();
            assert!(processes.insert(pid.unwrap()));
            assert!(!life.is_retired());
            lives.push(life);
        }
        assert_eq!(processes.len(), count);
        app.shutdown();
        let until = Instant::now() + Duration::from_secs(5);
        while lives.iter().any(|l| !l.is_retired())
            || processes
                .iter()
                .any(|p| std::path::Path::new(&format!("/proc/{p}")).exists())
        {
            assert!(Instant::now() < until, "worker retirement incomplete");
            std::thread::yield_now();
        }
        assert!(!app.has_camera_output());
        drop((images, infos));
        observer.close().wait().unwrap();
    }
}
#[test]
fn m111_actual_worker_subset_removal_and_full_time_reset_cancel_generations() {
    if isolation::child(
        "m111_actual_worker_subset_removal_and_full_time_reset_cancel_generations",
        180,
    ) {
        return;
    }
    let (mut app, s, observer, ids, remove) = application(4);
    let images: Vec<_> = (0..4)
        .map(|i| {
            observer
                .declare_subscriber(
                    app.server()
                        .options()
                        .topic_key(&format!("/m111/{i}/image"))
                        .unwrap(),
                )
                .wait()
                .unwrap()
        })
        .collect();
    let infos: Vec<_> = (0..4)
        .map(|i| {
            observer
                .declare_subscriber(
                    app.server()
                        .options()
                        .topic_key(&format!("/m111/{i}/info"))
                        .unwrap(),
                )
                .wait()
                .unwrap()
        })
        .collect();
    matched(&mut app, &ids);
    app.tick(&s).unwrap();
    app.capture_all().unwrap();
    ready(&mut app, &ids);
    for i in 0..4 {
        images[i]
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap();
        infos[i]
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap();
    }
    let old: Vec<_> = ids
        .iter()
        .map(|id| app.camera_worker_diagnostics(*id).unwrap().1)
        .collect();
    remove.store(ids[2], Ordering::Release);
    app.tick(&s).unwrap();
    assert_eq!(app.capture_all().unwrap().len(), 3);
    assert!(old[2].is_retired());
    assert!(
        old.iter()
            .enumerate()
            .filter(|(i, _)| *i != 2)
            .all(|(_, l)| !l.is_retired())
    );
    for reset in [
        WorldReset {
            all: true,
            ..Default::default()
        },
        WorldReset {
            time_only: true,
            ..Default::default()
        },
    ] {
        assert!(app.server_mut().apply_local_control(WorldControl {
            pause: true,
            reset: Some(reset),
            ..Default::default()
        }));
        app.tick(&s).unwrap();
        assert_eq!(app.capture_all().unwrap().len(), 4);
        assert!(
            ids.iter()
                .all(|id| app.camera_worker_diagnostics(*id).is_none())
        );
        assert!(images.iter().all(|p| p.try_recv().unwrap().is_none()));
        assert!(app.server_mut().apply_local_control(WorldControl {
            pause: false,
            ..Default::default()
        }));
        matched(&mut app, &ids);
        app.tick(&s).unwrap();
        app.capture_all().unwrap();
        ready(&mut app, &ids);
        for i in 0..4 {
            images[i]
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap();
            infos[i]
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap();
        }
    }
    assert!(old.iter().all(|l| l.is_retired()));
    app.shutdown();
    drop((images, infos));
    observer.close().wait().unwrap();
}
