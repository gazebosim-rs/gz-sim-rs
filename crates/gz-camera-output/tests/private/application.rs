// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[path = "../support/isolated_camera.rs"]
mod isolation;
use gz_sim_proto::{
    msgs::{Boolean, CameraInfo, WorldControl, WorldReset},
    prost::Message,
};
use std::{
    net::TcpListener,
    time::{Duration, Instant},
};

const SENSORS: &str = "<plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>";
fn config() -> zenoh::Config {
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
    config
}
fn settings() -> CameraServerOptions {
    CameraServerOptions {
        worker_path: "/nonexistent/worker".into(),
        origin: [1_000_000_000., 0., 0.],
        zenoh: config(),
    }
}
fn options() -> ServerOptions {
    ServerOptions {
        partition: format!("automatic-camera-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    }
}
fn xml(configured: bool) -> String {
    let source = include_str!("../../../gz-sim-scene/tests/fixtures/camera-binding.sdf");
    if configured {
        source.replace(
            "<world name=\"default\">",
            &format!("<world name=\"default\">{SENSORS}"),
        )
    } else {
        source.into()
    }
}
fn load(source: &str) -> Simulation {
    gz_sim_scene::load_world_with_sdf_step(
        source,
        "automatic-camera.sdf",
        "default",
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::Tpe,
    )
    .unwrap()
}

// A separately connected control client exercises the public application API;
// no private Server control or simulation mutation is used by these tests.
fn application(sim: Simulation, settings: &CameraServerOptions) -> (CameraServer, Session) {
    let reserved = TcpListener::bind("127.0.0.2:0").unwrap();
    let endpoint = format!("tcp/{}", reserved.local_addr().unwrap());
    let mut server = config();
    server
        .insert_json5("listen/endpoints", &format!("[\"{endpoint}\"]"))
        .unwrap();
    drop(reserved);
    let app = CameraServer::new(
        sim,
        "default",
        options(),
        server,
        NonZeroUsize::new(1).unwrap(),
        settings,
    )
    .unwrap();
    let mut client = config();
    client.insert_json5("listen/endpoints", "[]").unwrap();
    client
        .insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]"))
        .unwrap();
    client.insert_json5("connect/timeout_ms", "2000").unwrap();
    (app, zenoh::open(client).wait().unwrap())
}

fn control(app: &mut CameraServer, client: &Session, value: WorldControl) {
    let key = app
        .server()
        .options()
        .topic_key("/world/default/control")
        .unwrap();
    let replies = client
        .get(key)
        .payload(value.encode_to_vec())
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let before = app.server().counters().controls_accepted;
    let deadline = Instant::now() + Duration::from_secs(5);
    // One request, consumed through real frames. This is not a request retry or
    // a camera-output retry; preparation/output remains the caller's next phase.
    while app.server().counters().controls_accepted == before {
        assert!(Instant::now() < deadline, "world control was not consumed");
        app.server_mut().update().unwrap();
        std::thread::yield_now();
    }
    assert_eq!(app.server().counters().controls_accepted, before + 1);
    let sample = replies
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap()
        .into_result()
        .unwrap();
    assert!(
        Boolean::decode(sample.payload().to_bytes().as_ref())
            .unwrap()
            .data
    );
}

#[test]
fn automatic_intent_is_required_and_absent_options_own_no_camera_state() {
    if isolation::child(
        "application::tests::automatic_intent_is_required_and_absent_options_own_no_camera_state",
        20,
    ) {
        return;
    }
    let mut bad = settings();
    bad.worker_path = "relative".into();
    bad.origin = [f64::NAN; 3];
    let empty = format!("<sdf version='1.12'><world name='default'>{SENSORS}</world></sdf>");
    for source in [xml(false), empty] {
        let mut sim = load(&source);
        sim.set_paused(false);
        let counts = sim.scheduled_system_counts();
        let mut app = CameraServer::new(
            sim,
            "default",
            options(),
            config(),
            NonZeroUsize::new(1).unwrap(),
            &bad,
        )
        .unwrap();
        assert!(app.runtime.is_none());
        assert_eq!(app.camera_demand().unwrap(), None);
        for _ in 0..2 {
            let frame = app.update(&bad).unwrap();
            assert!(frame.pumped.is_none() && frame.requested.is_none());
        }
        assert_eq!(app.server().simulation().scheduled_system_counts(), counts);
        app.shutdown();
        assert!(!app.has_camera_output());
    }
}

#[test]
fn loaded_topics_follow_native_preprocessing_before_namespace_and_remap() {
    let cases = [
        (
            "",
            "",
            "/world/default/model/robot/link/body/sensor/camera/image",
            "/world/default/model/robot/link/body/sensor/camera/camera_info",
        ),
        ("/frames/rgb", "", "/frames/rgb", "/frames/camera_info"),
        ("image with spaces", "", "image_with_spaces", "/camera_info"),
        ("/frames/rgb", "/custom_info", "/frames/rgb", "/custom_info"),
    ];
    for (topic, info, image_expected, info_expected) in cases {
        let source = xml(true)
            .replace(
                "<topic>/explicit-camera</topic>",
                &format!("<topic>{topic}</topic>"),
            )
            .replace(
                "<camera>",
                &format!("<camera><camera_info_topic>{info}</camera_info_topic>"),
            );
        let mut options = options();
        options.namespace = "ns".into();
        let sim = load(&source);
        let prepared = prepare_camera(&sim, &options, &settings())
            .unwrap()
            .unwrap();
        assert_eq!(
            prepared.config.image_key,
            options.topic_key(image_expected).unwrap()
        );
        assert_eq!(
            prepared.config.info_key,
            options.topic_key(info_expected).unwrap()
        );
        assert_eq!(prepared.config.frame_id, "robot::body::camera");
    }
    let mut options = options();
    options
        .add_topic_remap("/explicit-camera", "/other/image")
        .unwrap();
    options
        .add_topic_remap("/camera_info", "/calibration")
        .unwrap();
    let sim = load(&xml(true));
    let p = prepare_camera(&sim, &options, &settings())
        .unwrap()
        .unwrap();
    assert_eq!(
        p.config.image_key,
        options.topic_key("/explicit-camera").unwrap()
    );
    assert_eq!(
        p.config.info_key,
        options.topic_key("/camera_info").unwrap()
    );
    assert!(!p.config.info_key.contains("/other/"));
}

#[test]
fn automatic_info_only_output_is_cadenced_without_a_worker() {
    if isolation::child(
        "application::tests::automatic_info_only_output_is_cadenced_without_a_worker",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(true);
    let counts = sim.scheduled_system_counts();
    let settings = settings();
    let (mut app, client) = application(sim, &settings);
    let runtime = app.runtime.as_ref().unwrap();
    let key = app.server().options().topic_key("/camera_info").unwrap();
    let (send, receive) = std::sync::mpsc::sync_channel(4);
    let listener = runtime
        .session
        .declare_subscriber(key)
        .allowed_origin(zenoh::sample::Locality::SessionLocal)
        .callback(move |sample| send.try_send(sample).unwrap())
        .wait()
        .unwrap();
    assert!(app.update(&settings).unwrap().requested.is_none());
    assert!(matches!(
        receive.try_recv(),
        Err(std::sync::mpsc::TryRecvError::Empty)
    ));
    control(
        &mut app,
        &client,
        WorldControl {
            pause: false,
            ..Default::default()
        },
    );
    for ns in [1_000_000, 2_000_000] {
        assert_eq!(
            app.update(&settings).unwrap().requested,
            Some(Progress::InfoPublished)
        );
        let sample = receive.recv_timeout(Duration::from_secs(2)).unwrap();
        let info = CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
        assert_eq!(info.width, 64);
        assert_eq!(info.height, 64);
        assert_eq!(
            info.header.as_ref().unwrap().data[0].value,
            ["robot::body::camera"]
        );
        assert_eq!(info.header.unwrap().stamp.unwrap().nsec, ns);
        // The configured worker path does not exist: image startup would fail.
        // Successful info publication therefore exercises the CPU-only path.
    }
    assert_eq!(app.server().simulation().scheduled_system_counts(), counts);
    app.shutdown();
    assert!(app.runtime.is_none());
    drop(listener);
}

#[test]
fn camera_session_is_released_when_subsequent_server_construction_fails() {
    if isolation::child(
        "application::tests::camera_session_is_released_when_subsequent_server_construction_fails",
        20,
    ) {
        return;
    }
    let reserved = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = reserved.local_addr().unwrap();
    let endpoints = format!("[\"tcp/{address}\"]");
    let mut settings = settings();
    settings
        .zenoh
        .insert_json5("listen/endpoints", &endpoints)
        .unwrap();
    let mut server = config();
    server.insert_json5("listen/endpoints", &endpoints).unwrap();
    drop(reserved);
    // Camera IO binds first. The following real Server bind must fail. Without
    // actual camera setup this constructor would instead succeed.
    let result = CameraServer::new(
        load(&xml(true)),
        "default",
        options(),
        server,
        NonZeroUsize::new(1).unwrap(),
        &settings,
    );
    assert!(matches!(
        result,
        Err(CameraServerError::Server(
            gz_sim_server::Error::WorldNetwork(_)
        ))
    ));
    // Exactly one rebind, with no wait/retry, proves constructor unwind released
    // the already-prepared camera Session's listener.
    let rebound = TcpListener::bind(address).unwrap();
    assert_eq!(rebound.local_addr().unwrap(), address);
}

#[test]
fn exposed_server_shutdown_prevents_camera_work_without_an_epoch_change() {
    if isolation::child(
        "application::tests::exposed_server_shutdown_prevents_camera_work_without_an_epoch_change",
        20,
    ) {
        return;
    }
    let settings = settings();
    let mut app = CameraServer::new(
        load(&xml(true)),
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        &settings,
    )
    .unwrap();
    let camera_session = app.runtime.as_ref().unwrap().session.clone();
    let epoch = app.server().simulation().epoch();
    let elapsed = app.server().simulation().elapsed();
    assert!(!camera_session.is_closed());
    app.server_mut().shutdown();
    assert!(app.server().is_shutdown());
    assert!(app.has_camera_output()); // Not yet visited by the outer application.
    for _ in 0..2 {
        assert!(matches!(
            app.pump(),
            Err(CameraServerError::Server(gz_sim_server::Error::WorldClosed))
        ));
        assert!(matches!(
            app.update(&settings),
            Err(CameraServerError::Server(gz_sim_server::Error::WorldClosed))
        ));
        assert!(!app.has_camera_output());
        assert!(camera_session.is_closed());
        assert_eq!(app.server().simulation().epoch(), epoch);
        assert_eq!(app.server().simulation().elapsed(), elapsed);
    }
}

#[test]
fn reset_reconstruction_failure_retires_old_session_before_returning_error() {
    if isolation::child(
        "application::tests::reset_reconstruction_failure_retires_old_session_before_returning_error",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(true);
    let settings = settings();
    let (mut app, client) = application(sim, &settings);
    let old_session = app.runtime.as_ref().unwrap().session.clone();
    control(
        &mut app,
        &client,
        WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        },
    );
    let mut invalid = settings;
    invalid.origin = [f64::NAN; 3];
    assert!(matches!(
        app.prepare_io(&invalid),
        Err(CameraServerError::Camera(CameraOutputError::Snapshot(_)))
    ));
    assert_eq!(app.server().simulation().epoch(), 1);
    assert_eq!(app.server().simulation().elapsed(), Duration::ZERO);
    assert!(app.runtime.is_none());
    assert!(old_session.is_closed());
    assert_eq!(app.pump().unwrap(), None);
}

#[test]
fn committed_server_reset_error_retires_camera_io_before_return() {
    if isolation::child(
        "application::tests::committed_server_reset_error_retires_camera_io_before_return",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(true);
    sim.add_systems(
        gz_sim_core::Phase::Reset,
        |fault: bevy_ecs::prelude::Res<gz_sim_core::StepFault>| {
            fault.report(std::io::Error::other("camera application reset control"));
        },
    );
    let settings = settings();
    let (mut app, client) = application(sim, &settings);
    let old_session = app.runtime.as_ref().unwrap().session.clone();
    control(
        &mut app,
        &client,
        WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        },
    );
    assert!(matches!(
        app.prepare_io(&settings),
        Err(CameraServerError::Server(gz_sim_server::Error::Reset(
            gz_sim_scene::SceneResetError::CommittedReset
        )))
    ));
    assert_eq!(app.server().simulation().epoch(), 1);
    assert!(app.runtime.is_none());
    assert!(old_session.is_closed());
    assert_eq!(app.pump().unwrap(), None);
}

#[test]
fn preflight_refuses_invalid_keys_frames_bounds_and_multiple_cameras_before_io() {
    for replacement in [
        "<topic>@</topic>",
        "<topic></topic><gz_frame_id></gz_frame_id>",
    ] {
        let source = xml(true).replace("<topic>/explicit-camera</topic>", replacement);
        assert!(prepare_camera(&load(&source), &options(), &settings()).is_err());
    }
    let source = xml(true).replace(
        "<camera>",
        "<camera><camera_info_topic>/explicit-camera</camera_info_topic>",
    );
    assert!(matches!(
        prepare_camera(&load(&source), &options(), &settings()),
        Err(CameraServerError::Camera(
            CameraOutputError::DuplicateTopics
        ))
    ));
    let mut settings = settings();
    settings.worker_path = "relative-worker".into();
    assert!(matches!(
        prepare_camera(&load(&xml(true)), &options(), &settings),
        Err(CameraServerError::Camera(CameraOutputError::Client(_)))
    ));
    let sim = load(&xml(true));
    let mut invalid_bounds = options();
    invalid_bounds.max_message_bytes = 0;
    assert!(matches!(
        prepare_camera(&sim, &invalid_bounds, &settings),
        Err(CameraServerError::Topic(SetupError::InvalidConfig))
    ));

    let mut sim = load(&xml(true));
    let camera = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<CameraSensor>())
        .unwrap()
        .get::<CameraSensor>()
        .unwrap()
        .clone();
    sim.world_mut().spawn(camera);
    assert!(matches!(
        prepare_camera(&sim, &options(), &settings),
        Err(CameraServerError::Camera(
            CameraOutputError::MultipleCameras
        ))
    ));
}

#[test]
fn observed_removal_and_replacement_cancel_native_ownership_after_step() {
    if isolation::child(
        "application::tests::observed_removal_and_replacement_cancel_native_ownership_after_step",
        20,
    ) {
        return;
    }
    for remove in [true, false] {
        let mut sim = load(&xml(true));
        sim.set_paused(false);
        let camera = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<CameraSensor>())
            .unwrap()
            .id();
        // A real configured Bevy lifecycle system changes the Scene during the
        // committed frame. The app must validate before touching old output.
        sim.add_systems(
            gz_sim_core::Phase::Finalize,
            move |world: &mut bevy_ecs::world::World| {
                if remove {
                    world.entity_mut(camera).remove::<CameraSensor>();
                } else {
                    world
                        .entity_mut(camera)
                        .insert(Name("default::changed".into()));
                }
            },
        );
        let settings = settings();
        let mut app = CameraServer::new(
            sim,
            "default",
            options(),
            config(),
            NonZeroUsize::new(1).unwrap(),
            &settings,
        )
        .unwrap();
        let session = app.runtime.as_ref().unwrap().session.clone();
        let result = app.update(&settings);
        if remove {
            let step = result.unwrap();
            assert!(step.pumped.is_none() && step.requested.is_none());
        } else {
            assert!(matches!(
                result,
                Err(CameraServerError::Camera(CameraOutputError::OwnerInvalid))
            ));
        }
        assert_eq!(
            app.server().simulation().elapsed(),
            Duration::from_millis(1)
        );
        assert!(app.runtime.is_none());
        assert!(session.is_closed());
        assert_eq!(app.pump().unwrap(), None);
    }
}

#[test]
fn accepted_reset_refuses_old_camera_pump_before_epoch_commit_then_rebuilds() {
    if isolation::child(
        "application::tests::accepted_reset_refuses_old_camera_pump_before_epoch_commit_then_rebuilds",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(true);
    let settings = settings();
    let (mut app, client) = application(sim, &settings);
    let old_session = app.runtime.as_ref().unwrap().session.clone();
    control(
        &mut app,
        &client,
        WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        },
    );
    assert!(app.server().io_preparation_required());
    assert_eq!(app.server().simulation().epoch(), 0);
    assert!(matches!(
        app.pump(),
        Err(CameraServerError::Server(
            gz_sim_server::Error::IoPreparationRequired
        ))
    ));
    assert!(app.runtime.is_none());
    assert!(old_session.is_closed());
    assert_eq!(app.server().simulation().epoch(), 0);
    app.prepare_io(&settings).unwrap();
    assert!(!app.server().io_preparation_required());
    assert_eq!(app.server().simulation().epoch(), 1);
    assert!(app.runtime.is_some());
    assert!(!app.runtime.as_ref().unwrap().session.is_closed());
    let reset = app.update(&settings).unwrap();
    assert_eq!(reset.info.epoch, 1);
    assert_eq!(reset.info.sim_time, Duration::ZERO);
    assert!(reset.requested.is_none());
}

#[test]
fn reset_accepted_by_application_update_retires_before_same_frame_publication() {
    if isolation::child(
        "application::tests::reset_accepted_by_application_update_retires_before_same_frame_publication",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(false);
    let settings = settings();
    let (mut app, client) = application(sim, &settings);
    let old_session = app.runtime.as_ref().unwrap().session.clone();
    let key = app.server().options().topic_key("/camera_info").unwrap();
    let (send, receive) = std::sync::mpsc::sync_channel(8);
    let _listener = old_session
        .declare_subscriber(key)
        .allowed_origin(zenoh::sample::Locality::SessionLocal)
        .callback(move |sample| send.try_send(sample).unwrap())
        .wait()
        .unwrap();
    let key = app
        .server()
        .options()
        .topic_key("/world/default/control")
        .unwrap();
    let replies = client
        .get(key)
        .payload(
            WorldControl {
                pause: true,
                reset: Some(WorldReset {
                    all: true,
                    ..Default::default()
                }),
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let before = app.server().counters().controls_accepted;
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let result = app.update(&settings);
        let stamps: Vec<_> = receive
            .try_iter()
            .map(|sample| {
                CameraInfo::decode(sample.payload().to_bytes().as_ref())
                    .unwrap()
                    .header
                    .unwrap()
                    .stamp
                    .unwrap()
            })
            .collect();
        if app.server().counters().controls_accepted == before + 1 {
            let accepted_time = app.server().simulation().elapsed();
            eprintln!(
                "accepted reset: result={result:?}, epoch={}, time={accepted_time:?}, info_stamps={stamps:?}",
                app.server().simulation().epoch()
            );
            assert!(matches!(
                result,
                Err(CameraServerError::Server(
                    gz_sim_server::Error::IoPreparationRequired
                ))
            ));
            assert!(
                stamps.iter().all(
                    |stamp| Duration::new(stamp.sec as u64, stamp.nsec as u32) != accepted_time
                )
            );
            assert!(app.server().io_preparation_required());
            assert_eq!(app.server().simulation().epoch(), 0);
            assert!(app.runtime.is_none());
            assert!(old_session.is_closed());
            break;
        }
        result.unwrap();
        assert!(Instant::now() < deadline, "reset request was not consumed");
        std::thread::yield_now();
    }
    let sample = replies
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap()
        .into_result()
        .unwrap();
    assert!(
        Boolean::decode(sample.payload().to_bytes().as_ref())
            .unwrap()
            .data
    );
    app.prepare_io(&settings).unwrap();
    assert_eq!(app.server().simulation().epoch(), 1);
}

#[test]
fn own_matching_observation_preserves_cadence_and_retires_terminal_output() {
    if isolation::child(
        "application::tests::own_matching_observation_preserves_cadence_and_retires_terminal_output",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(false);
    let settings = settings();
    let mut app = CameraServer::new(
        sim,
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        &settings,
    )
    .unwrap();
    let session = app.runtime.as_ref().unwrap().session.clone();
    assert_eq!(app.camera_demand().unwrap(), Some((false, false)));
    let key = app.server().options().topic_key("/camera_info").unwrap();
    let (send, receive) = std::sync::mpsc::sync_channel(4);
    let listener = session
        .declare_subscriber(key)
        .allowed_origin(zenoh::sample::Locality::SessionLocal)
        .callback(move |sample| send.try_send(sample).unwrap())
        .wait()
        .unwrap();
    for _ in 0..2 {
        assert_eq!(app.camera_demand().unwrap(), Some((false, true)));
        assert_eq!(app.server().simulation().elapsed(), Duration::ZERO);
        assert!(receive.try_recv().is_err());
    }
    assert_eq!(
        app.update(&settings).unwrap().requested,
        Some(Progress::InfoPublished)
    );
    let sample = receive.recv_timeout(Duration::from_secs(2)).unwrap();
    let info = CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
    assert_eq!(info.header.unwrap().stamp.unwrap().nsec, 1_000_000);
    app.server_mut().shutdown();
    assert!(matches!(
        app.camera_demand(),
        Err(CameraServerError::Server(gz_sim_server::Error::WorldClosed))
    ));
    assert!(!app.has_camera_output());
    assert!(session.is_closed());
    drop(listener);
}

#[test]
fn own_matching_observation_refuses_pending_reset_and_changed_binding() {
    if isolation::child(
        "application::tests::own_matching_observation_refuses_pending_reset_and_changed_binding",
        20,
    ) {
        return;
    }
    for reset in [true, false] {
        let mut sim = load(&xml(true));
        sim.set_paused(reset);
        if !reset {
            let camera = sim
                .world()
                .iter_entities()
                .find(|e| e.contains::<CameraSensor>())
                .unwrap()
                .id();
            sim.add_systems(
                gz_sim_core::Phase::Finalize,
                move |world: &mut bevy_ecs::world::World| {
                    world
                        .entity_mut(camera)
                        .insert(Name("default::changed".into()));
                },
            );
        }
        let settings = settings();
        let (mut app, client) = application(sim, &settings);
        let session = app.runtime.as_ref().unwrap().session.clone();
        if reset {
            control(
                &mut app,
                &client,
                WorldControl {
                    pause: true,
                    reset: Some(WorldReset {
                        all: true,
                        ..Default::default()
                    }),
                    ..Default::default()
                },
            );
            assert!(matches!(
                app.camera_demand(),
                Err(CameraServerError::Server(
                    gz_sim_server::Error::IoPreparationRequired
                ))
            ));
            assert!(app.server().io_preparation_required());
        } else {
            app.server_mut().update().unwrap();
            assert!(matches!(
                app.camera_demand(),
                Err(CameraServerError::Camera(CameraOutputError::OwnerInvalid))
            ));
        }
        assert_eq!(app.server().simulation().epoch(), 0);
        assert!(!app.has_camera_output());
        assert!(session.is_closed());
    }
}

#[test]
fn into_simulation_retires_camera_io_and_preserves_a_usable_scene() {
    if isolation::child(
        "application::tests::into_simulation_retires_camera_io_and_preserves_a_usable_scene",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(false);
    let settings = settings();
    let mut app = CameraServer::new(
        sim,
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        &settings,
    )
    .unwrap();
    app.update(&settings).unwrap();
    let session = app.runtime.as_ref().unwrap().session.clone();
    let before = app.server().simulation();
    let ids: Vec<_> = before
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(EntityId::get))
        .collect();
    let metadata = before
        .world()
        .iter_entities()
        .find_map(|e| e.get::<CameraSensor>().cloned())
        .unwrap();
    let counts = before.scheduled_system_counts();
    let epoch = before.epoch();
    let elapsed = before.elapsed();
    let mut sim = app.into_simulation();
    assert!(session.is_closed());
    assert_eq!(sim.epoch(), epoch);
    assert_eq!(sim.elapsed(), elapsed);
    assert_eq!(sim.scheduled_system_counts(), counts);
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter_map(|e| e.get::<EntityId>().map(EntityId::get))
            .collect::<Vec<_>>(),
        ids
    );
    let restored = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<CameraSensor>())
        .unwrap();
    assert!(std::sync::Arc::ptr_eq(&restored.0, &metadata.0));
    assert_eq!(format!("{:?}", restored.0), format!("{:?}", metadata.0));
    sim.step_exact(1).unwrap();
    assert_eq!(sim.elapsed(), elapsed + Duration::from_millis(1));
    assert_eq!(sim.epoch(), epoch);
}

#[test]
fn physics_step_failure_retires_camera_io_and_preserves_typed_fault() {
    if isolation::child(
        "application::tests::physics_step_failure_retires_camera_io_and_preserves_typed_fault",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(false);
    sim.add_systems(
        gz_sim_core::Phase::PhysicsStep,
        |fault: bevy_ecs::prelude::Res<gz_sim_core::StepFault>| {
            fault.report(std::io::Error::other("automatic camera physics failure"));
        },
    );
    let settings = settings();
    let mut app = CameraServer::new(
        sim,
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        &settings,
    )
    .unwrap();
    let session = app.runtime.as_ref().unwrap().session.clone();
    assert!(matches!(
        app.update(&settings),
        Err(CameraServerError::Server(gz_sim_server::Error::Step(
            gz_sim_core::StepError::Faulted
        )))
    ));
    assert!(app.runtime.is_none());
    assert!(session.is_closed());
    assert!(app.server().simulation().is_faulted());
    assert_eq!(app.server().simulation().epoch(), 0);
    assert_eq!(app.server().simulation().elapsed(), Duration::ZERO);
    assert!(
        app.server()
            .simulation()
            .fault()
            .unwrap()
            .to_string()
            .contains("automatic camera physics failure")
    );
    let elapsed = app.server().simulation().elapsed();
    assert_eq!(app.pump().unwrap(), None);
    assert!(matches!(
        app.update(&settings),
        Err(CameraServerError::Server(gz_sim_server::Error::Step(
            gz_sim_core::StepError::Faulted
        )))
    ));
    assert_eq!(app.server().simulation().elapsed(), elapsed);
}

#[test]
fn preflight_rejects_broken_loaded_identity_and_hierarchy_before_admission() {
    for case in 0..12 {
        let mut sim = load(&xml(true));
        let camera = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<CameraSensor>())
            .unwrap()
            .id();
        let id = sim.world().get::<EntityId>(camera).unwrap().get();
        let link = sim.world().get::<ChildOf>(camera).unwrap().parent();
        let root = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<EntityKind>() == Some(&EntityKind::World))
            .unwrap()
            .id();
        match case {
            0 => {
                sim.world_mut().entity_mut(camera).remove::<EntityId>();
            }
            1 => {
                sim.world_mut().entity_mut(camera).remove::<EntityKind>();
            }
            2 => {
                sim.world_mut().entity_mut(camera).remove::<LocalName>();
            }
            3 => {
                sim.world_mut().entity_mut(camera).remove::<Name>();
            }
            4 => {
                sim.world_mut().entity_mut(camera).remove::<ChildOf>();
            }
            5 => {
                sim.world_mut()
                    .entity_mut(camera)
                    .insert(LocalName(String::new()));
            }
            6 => {
                sim.world_mut()
                    .entity_mut(camera)
                    .insert(LocalName("bad::scope".into()));
            }
            7 => {
                sim.world_mut().entity_mut(link).insert(EntityKind::Model);
            }
            8 => {
                sim.world_mut()
                    .entity_mut(camera)
                    .insert(Name("default::wrong".into()));
            }
            9 => {
                sim.world_mut().entity_mut(camera).insert(ChildOf(root));
            }
            10 => {
                sim.world_mut()
                    .spawn((EntityKind::World, RenderingSensorsConfiguration));
            }
            11 => {
                sim.world_mut().entity_mut(root).remove::<EntityId>();
            }
            _ => unreachable!(),
        }
        let expected = if case == 0 || case == 10 { 0 } else { id };
        assert!(
            matches!(prepare_camera(&sim, &options(), &settings()),
            Err(CameraServerError::Camera(CameraOutputError::Snapshot(SnapshotError::MissingCamera(found)))) if found == expected),
            "case {case}"
        );
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(sim.epoch(), 0);
    }
}

#[test]
fn nested_model_preflight_uses_the_full_typed_topic_and_scoped_frame() {
    let source = xml(true)
        .replace(
            "<model name=\"robot\">",
            "<model name=\"outer\"><model name=\"robot\">",
        )
        .replace("</model>", "</model></model>")
        .replace("<topic>/explicit-camera</topic>", "<topic></topic>");
    // TPE explicitly refuses nested models; use the existing nested-capable backend.
    let sim = gz_sim_scene::load_world_with_sdf_step(
        &source,
        "automatic-camera.sdf",
        "default",
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::Rapier,
    )
    .unwrap();
    let options = options();
    let prepared = prepare_camera(&sim, &options, &settings())
        .unwrap()
        .unwrap();
    let expected = "/world/default/model/outer/model/robot/link/body/sensor/camera/image";
    assert_eq!(
        prepared.config.image_key,
        options.topic_key(expected).unwrap()
    );
    assert_eq!(prepared.config.frame_id, "outer::robot::body::camera");
    assert!(prepared.binding.current(&sim));
    assert_eq!(prepared.binding.nodes.len(), 5);
    assert_eq!(sim.elapsed(), Duration::ZERO);
}

#[test]
fn committed_binding_mutations_retire_before_any_camera_output() {
    if isolation::child(
        "application::tests::committed_binding_mutations_retire_before_any_camera_output",
        20,
    ) {
        return;
    }
    for case in 0..4 {
        let mut sim = load(&xml(true));
        sim.set_paused(false);
        let camera = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<CameraSensor>())
            .unwrap()
            .id();
        let root = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<EntityKind>() == Some(&EntityKind::World))
            .unwrap()
            .id();
        sim.add_systems(
            gz_sim_core::Phase::Finalize,
            move |world: &mut bevy_ecs::world::World| match case {
                0 => {
                    world.entity_mut(camera).insert(EntityKind::Link);
                }
                1 => {
                    world
                        .entity_mut(camera)
                        .insert(LocalName("replacement".into()));
                }
                2 => {
                    world.entity_mut(camera).insert(ChildOf(root));
                }
                3 => {
                    world
                        .entity_mut(root)
                        .remove::<RenderingSensorsConfiguration>();
                }
                _ => unreachable!(),
            },
        );
        let settings = settings();
        let mut app = CameraServer::new(
            sim,
            "default",
            options(),
            config(),
            NonZeroUsize::new(1).unwrap(),
            &settings,
        )
        .unwrap();
        let session = app.runtime.as_ref().unwrap().session.clone();
        assert!(
            matches!(
                app.update(&settings),
                Err(CameraServerError::Camera(CameraOutputError::OwnerInvalid))
            ),
            "case {case}"
        );
        assert_eq!(
            app.server().simulation().elapsed(),
            Duration::from_millis(1)
        );
        assert!(app.runtime.is_none());
        assert!(session.is_closed());
        assert_eq!(app.camera_demand().unwrap(), None);
        assert_eq!(app.pump().unwrap(), None);
    }
}
