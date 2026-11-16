// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::EntityId;
use gz_sim_proto::prost::Message;
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_sdf_step};
use std::num::NonZeroUsize;
use zenoh::Wait;

fn fixture() -> (Simulation, u64) {
    let source = include_str!("../../../gz-sim-scene/tests/fixtures/camera-binding.sdf");
    let sim = load_world_with_sdf_step(
        source,
        "camera-binding.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find_map(|entity| {
            (entity.get::<Name>()?.0 == "default::robot::body::camera")
                .then(|| entity.get::<EntityId>().unwrap().get())
        })
        .unwrap();
    (sim, id)
}

fn session() -> (zenoh::Session, CameraWriterScope, NonZeroUsize) {
    session_with_max(16 * 1024 * 1024)
}

fn session_with_max(max_message_bytes: usize) -> (zenoh::Session, CameraWriterScope, NonZeroUsize) {
    let mut config = zenoh::Config::default();
    for (path, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(path, value).unwrap();
    }
    (
        zenoh::open(config).wait().unwrap(),
        CameraWriterScope::new(NonZeroUsize::new(4).unwrap()),
        NonZeroUsize::new(max_message_bytes).unwrap(),
    )
}

#[test]
fn no_demand_and_info_only_never_start_a_worker() {
    let (sim, id) = fixture();
    let systems = sim.scheduled_system_counts();
    let (session, scope, limit) = session();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/nonexistent/worker".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::NoDemand);
    assert!(output.client.is_none());
    let subscription = session.declare_subscriber("@/camera@/info").wait().unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::InfoPublished);
    let packet = subscription
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap();
    let info =
        gz_sim_proto::msgs::CameraInfo::decode(packet.payload().to_bytes().as_ref()).unwrap();
    assert_eq!((info.width, info.height), (64, 64));
    assert_eq!(info.header.unwrap().stamp.unwrap().sec, 0);
    assert!(output.client.is_none());
    assert_eq!(systems, sim.scheduled_system_counts());
}

#[test]
fn invalid_camera_or_worker_configuration_precedes_advertisement() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    assert!(matches!(
        CameraOutput::new(
            &sim,
            &session,
            &scope,
            id + 100,
            CameraOutputConfig {
                origin: [0.0; 3],
                frame_id: "frame".into(),
                image_key: "@/camera@/image".into(),
                info_key: "@/camera@/info".into(),
                worker_path: "/nonexistent/worker".into(),
                max_message_bytes: limit
            }
        ),
        Err(CameraOutputError::Snapshot(_))
    ));
    assert!(matches!(
        CameraOutput::new(
            &sim,
            &session,
            &scope,
            id,
            CameraOutputConfig {
                origin: [1_000_000_000.0, 0.0, 0.0],
                frame_id: "".into(),
                image_key: "@/camera@/image".into(),
                info_key: "@/camera@/info".into(),
                worker_path: "/nonexistent/worker".into(),
                max_message_bytes: limit
            }
        ),
        Err(CameraOutputError::InvalidFrameId)
    ));
    assert!(matches!(
        CameraOutput::new(
            &sim,
            &session,
            &scope,
            id,
            CameraOutputConfig {
                origin: [1_000_000_000.0, 0.0, 0.0],
                frame_id: "frame".into(),
                image_key: "@/camera@/image".into(),
                info_key: "@/camera@/info".into(),
                worker_path: "relative-worker".into(),
                max_message_bytes: limit
            }
        ),
        Err(CameraOutputError::Client(ClientError::InvalidWorkerPath))
    ));
    // Both topics remain available after failed construction.
    let _output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/nonexistent/worker".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
}

#[test]
fn same_entity_metadata_replacement_invalidates_inflight_acquisition() {
    let (mut sim, id) = fixture();
    let (session, scope, limit) = session();
    let _image = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/false".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    let entity = entity_by_id(sim.world(), id).unwrap();
    let mut sensor = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
    sensor.0 = std::sync::Arc::new((*sensor.0).clone());
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    output.retire();
}

#[test]
fn actual_worker_exit_is_reported_and_requires_retirement() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    let _image = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/false".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    loop {
        match output.pump(&sim) {
            Ok(Progress::Backpressure | Progress::Pending) => std::thread::yield_now(),
            Err(CameraOutputError::Client(_) | CameraOutputError::Protocol) => break,
            other => panic!("unexpected worker result: {other:?}"),
        }
        assert!(
            std::time::Instant::now() < deadline,
            "worker exit not observed"
        );
    }
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::RetireRequired)
    ));
    output.retire();
}

#[test]
fn idle_replacement_requires_old_owner_drop_before_reconstruction() {
    let (mut sim, id) = fixture();
    let (session, scope, limit) = session();
    let make = |sim: &Simulation| {
        CameraOutput::new(
            sim,
            &session,
            &scope,
            id,
            CameraOutputConfig {
                origin: [1_000_000_000.0, 0.0, 0.0],
                frame_id: "frame".into(),
                image_key: "@/camera@/image".into(),
                info_key: "@/camera@/info".into(),
                worker_path: "/bin/false".into(),
                max_message_bytes: limit,
            },
        )
    };
    let mut output = make(&sim).unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::NoDemand);
    let info = session.declare_subscriber("@/camera@/info").wait().unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::InfoPublished);
    info.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    let entity = entity_by_id(sim.world(), id).unwrap();
    let mut sensor = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
    sensor.0 = std::sync::Arc::new((*sensor.0).clone());
    assert!(matches!(
        output.pump(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    output.retire();
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    assert!(matches!(
        make(&sim),
        Err(CameraOutputError::AlreadyPublishing)
    ));
    drop(output);
    let _replacement = make(&sim).unwrap();
}

#[test]
fn same_name_and_restored_id_cannot_rebind_an_existing_idle_owner() {
    use gz_sim_scene::{EntityKind, Name, WorldPose};
    let (mut sim, id) = fixture();
    let (session, scope, limit) = session();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/false".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::NoDemand);
    let entity = entity_by_id(sim.world(), id).unwrap();
    let reservation = gz_sim_core::reserve_id(sim.world(), entity).unwrap();
    let name = sim.world().get::<Name>(entity).unwrap().0.clone();
    let pose = *sim.world().get::<WorldPose>(entity).unwrap();
    let sensor = sim.world().get::<CameraSensor>(entity).unwrap().clone();
    assert!(sim.world_mut().despawn(entity));
    let replacement = sim
        .world_mut()
        .spawn((Name(name), EntityKind::Sensor, pose, sensor))
        .id();
    assert_ne!(entity, replacement);
    assert_eq!(
        gz_sim_core::restore_id(sim.world_mut(), replacement, &reservation).unwrap(),
        id
    );
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    output.retire();
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    drop(output);
    let _replacement = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/false".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
}

#[test]
fn explicit_retirement_joins_idle_worker_before_returning() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    let _image = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/cat".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    let lifetime = output.client.as_ref().unwrap().diagnostic_lifetime();
    assert!(!lifetime.is_retired());
    // A real child that never sends a protocol greeting cannot accept work.
    assert_eq!(output.pump(&sim).unwrap(), Progress::Backpressure);
    output.retire();
    assert!(lifetime.is_retired());
    assert!(output.client.is_none());
}

#[test]
fn idle_pump_and_demand_loss_require_explicit_worker_retirement() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    let image = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/cat".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    assert_eq!(output.pump(&sim).unwrap(), Progress::NoDemand);
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    let lifetime = output.client.as_ref().unwrap().diagnostic_lifetime();
    drop(image);
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::RetireRequired)
    ));
    output.retire();
    assert!(lifetime.is_retired());
    assert_eq!(output.pump(&sim).unwrap(), Progress::NoDemand);
}

#[test]
fn exhausted_request_ids_do_not_start_worker_or_publish_info() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    let info = session.declare_subscriber("@/camera@/info").wait().unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/cat".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    output.next_id = u64::MAX - 1;
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::IdExhausted)
    ));
    output.next_id = u64::MAX;
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::IdExhausted)
    ));
    assert!(output.client.is_none());
    assert!(
        info.recv_timeout(Duration::from_millis(20))
            .unwrap()
            .is_none()
    );
}

#[test]
fn unrepresentable_acquisition_time_cannot_publish_or_start_worker() {
    use gz_sim_core::assign_id;
    use gz_sim_scene::{EntityKind, WorldPose};
    use sdformat::{Camera, Sensor, SensorConfig};
    let mut sim = Simulation::new(Duration::from_secs(i64::MAX as u64 + 1)).unwrap();
    let mut camera = Camera {
        image_width: 64,
        image_height: 64,
        horizontal_fov: std::f64::consts::FRAC_PI_2,
        ..Default::default()
    };
    camera.lens.has_intrinsics = false;
    camera.lens.has_projection = false;
    let sensor = Sensor {
        kind: "camera".into(),
        config: SensorConfig::Camera(Box::new(camera)),
        ..Default::default()
    };
    let entity = sim
        .world_mut()
        .spawn((
            EntityKind::Sensor,
            CameraSensor(std::sync::Arc::new(sensor)),
            WorldPose {
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        ))
        .id();
    let id = assign_id(sim.world_mut(), entity).unwrap();
    let (session, scope, limit) = session();
    let info = session.declare_subscriber("@/camera@/info").wait().unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [0.0; 3],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/cat".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    sim.step_exact(1).unwrap();
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::Message(CameraMessageError::TimeRange))
    ));
    assert!(output.client.is_none());
    assert!(
        info.recv_timeout(Duration::from_millis(20))
            .unwrap()
            .is_none()
    );
}

#[test]
fn stopped_transport_is_reported_before_and_during_an_acquisition() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    let image = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/cat".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    session.close().wait().unwrap();
    assert!(matches!(
        output.demand(),
        Err(CameraOutputError::SessionClosed)
    ));
    assert!(matches!(
        output.pump(&sim),
        Err(CameraOutputError::SessionClosed)
    ));
    output.retire();
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::SessionClosed)
    ));
    drop(image);
}

#[test]
fn duplicate_topics_and_oversized_info_fail_without_starting_a_worker() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    assert!(matches!(
        CameraOutput::new(
            &sim,
            &session,
            &scope,
            id,
            CameraOutputConfig {
                origin: [1_000_000_000.0, 0.0, 0.0],
                frame_id: "frame".into(),
                image_key: "@/camera@/same".into(),
                info_key: "@/camera@/same".into(),
                worker_path: "/bin/cat".into(),
                max_message_bytes: limit
            }
        ),
        Err(CameraOutputError::DuplicateTopics)
    ));
    let _replacement = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/same".into(),
            info_key: "@/camera@/different".into(),
            worker_path: "/bin/cat".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    let (session, scope, limit) = session_with_max(16);
    let info = session.declare_subscriber("@/camera@/info").wait().unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/cat".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::PublicationTooLarge { .. })
    ));
    assert!(output.client.is_none());
    assert!(
        info.recv_timeout(Duration::from_millis(20))
            .unwrap()
            .is_none()
    );
}

#[test]
fn scene_mutation_after_construction_fails_before_publication_or_worker_start() {
    let (mut sim, id) = fixture();
    let (session, scope, limit) = session();
    let info = session.declare_subscriber("@/camera@/info").wait().unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/cat".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    let visual = sim.world_mut().spawn(gz_sim_scene::EntityKind::Visual).id();
    let visual_id = gz_sim_core::assign_id(sim.world_mut(), visual).unwrap();
    assert!(
        matches!(output.request(&sim), Err(CameraOutputError::Snapshot(SnapshotError::MissingVisualData(actual))) if actual == visual_id)
    );
    assert!(output.client.is_none());
    assert!(
        info.recv_timeout(Duration::from_millis(20))
            .unwrap()
            .is_none()
    );
}

#[test]
fn isolated_supervisor_thread_exhaustion_is_reported_before_acquisition() {
    const CHILD: &str = "GZ_CAMERA_THREAD_LIMIT_CASE";
    if std::env::var_os(CHILD).is_some() {
        use rustix::process::{Resource, Rlimit, getrlimit, setrlimit};
        let (sim, id) = fixture();
        let (session, scope, limit) = session();
        let image = session
            .declare_subscriber("@/camera@/image")
            .wait()
            .unwrap();
        let mut output = CameraOutput::new(
            &sim,
            &session,
            &scope,
            id,
            CameraOutputConfig {
                origin: [1_000_000_000.0, 0.0, 0.0],
                frame_id: "frame".into(),
                image_key: "@/camera@/image".into(),
                info_key: "@/camera@/info".into(),
                worker_path: "/bin/cat".into(),
                max_message_bytes: limit,
            },
        )
        .unwrap();
        let limit = getrlimit(Resource::Nproc);
        setrlimit(
            Resource::Nproc,
            Rlimit {
                current: Some(1),
                maximum: limit.maximum,
            },
        )
        .unwrap();
        assert!(
            matches!(output.request(&sim), Err(CameraOutputError::Client(ClientError::Io(error))) if error.raw_os_error() == Some(11))
        );
        assert!(output.client.is_none());
        assert!(output.acquisition.is_none());
        assert!(
            image
                .recv_timeout(Duration::from_millis(20))
                .unwrap()
                .is_none()
        );
        return;
    }
    let result = std::process::Command::new("timeout")
        .args(["--signal=KILL", "20s", "unshare", "-Ur"])
        .arg(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "tests::isolated_supervisor_thread_exhaustion_is_reported_before_acquisition",
            "--nocapture",
        ])
        .env(CHILD, "1")
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert!(String::from_utf8_lossy(&result.stdout).contains("1 passed; 0 failed"));
}

#[test]
fn cached_local_interest_is_immediate_across_drop_and_rebind() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    let output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/cat".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    assert_eq!(output.demand().unwrap(), (false, false));
    let images = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    assert_eq!(output.demand().unwrap(), (true, false));
    let infos = session.declare_subscriber("@/camera@/info").wait().unwrap();
    assert_eq!(output.demand().unwrap(), (true, true));
    drop(images);
    assert_eq!(output.demand().unwrap(), (false, true));
    let _images = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    assert_eq!(output.demand().unwrap(), (true, true));
    drop(infos);
    assert_eq!(output.demand().unwrap(), (true, false));
    assert!(output.client.is_none());
}

#[test]
fn camera_writer_admission_is_bounded_shared_and_released_on_failure_and_drop() {
    let (sim, id) = fixture();
    let (session, _, limit) = session();
    let scope = CameraWriterScope::new(NonZeroUsize::new(1).unwrap());
    let clone = scope.clone();
    let make = |scope: &CameraWriterScope, image: &str, info: &str| {
        CameraOutput::new(
            &sim,
            &session,
            scope,
            id,
            CameraOutputConfig {
                origin: [1_000_000_000.0, 0.0, 0.0],
                frame_id: "frame".into(),
                image_key: image.into(),
                info_key: info.into(),
                worker_path: "/bin/cat".into(),
                max_message_bytes: limit,
            },
        )
    };
    for bad in [
        "/unqualified",
        "@@/info",
        "@/camera@/wild/*",
        "@/camera@/info@extra",
    ] {
        assert!(matches!(
            make(&scope, "@/camera@/image", bad),
            Err(CameraOutputError::InvalidPublicationKey)
        ));
        assert!(scope.entries.lock().unwrap().is_empty());
    }
    let mut output = make(&scope, "@/camera@/image", "@/camera@/info").unwrap();
    assert_eq!(scope.entries.lock().unwrap().len(), 2);
    assert!(matches!(
        make(&clone, "@/camera@/image", "@/camera@/other"),
        Err(CameraOutputError::AlreadyPublishing)
    ));
    assert!(matches!(
        make(&clone, "@/camera@/other-image", "@/camera@/other-info"),
        Err(CameraOutputError::WriterLimit)
    ));
    output.retire();
    assert!(matches!(
        make(&clone, "@/camera@/image", "@/camera@/info"),
        Err(CameraOutputError::AlreadyPublishing)
    ));
    drop(output);
    assert!(scope.entries.lock().unwrap().is_empty());
    let output = make(&clone, "@/camera@/image", "@/camera@/info").unwrap();
    drop(output);
    session.close().wait().unwrap();
    assert!(matches!(
        make(&scope, "@/camera@/image", "@/camera@/info"),
        Err(CameraOutputError::SessionClosed)
    ));
    assert!(scope.entries.lock().unwrap().is_empty());
}

#[test]
fn native_liveliness_keys_and_wire_attachments_withdraw_with_output() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    let tokens = session
        .liveliness()
        .declare_subscriber("@gz/**")
        .wait()
        .unwrap();
    let info = session.declare_subscriber("@/camera@/info").wait().unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: "/bin/cat".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    let mut declared = BTreeSet::new();
    for _ in 0..2 {
        let sample = tokens
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap();
        assert_eq!(sample.kind(), zenoh::sample::SampleKind::Put);
        let fields: Vec<_> = sample.key_expr().as_str().split('/').collect();
        assert_eq!(&fields[..3], ["@gz", "%camera", &session.zid().to_string()]);
        assert_eq!(fields[3], fields[4]);
        assert!(uuid::Uuid::parse_str(fields[3]).is_ok());
        assert_eq!(&fields[5..9], ["MP", "%", "%", "%"]);
        assert_eq!(&fields[11..], ["%", "%"]);
        assert!(matches!(
            (fields[9], fields[10]),
            ("%image", "gz.msgs.Image") | ("%info", "gz.msgs.CameraInfo")
        ));
        declared.insert(sample.key_expr().to_string());
    }
    assert_eq!(declared.len(), 2);
    assert_eq!(output.request(&sim).unwrap(), Progress::InfoPublished);
    let packet = info.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    assert_eq!(packet.key_expr().as_str(), "@/camera@/info");
    assert_eq!(packet.encoding().to_string(), "zenoh/bytes");
    assert_eq!(
        packet.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.CameraInfo"
    );
    let decoded =
        gz_sim_proto::msgs::CameraInfo::decode(packet.payload().to_bytes().as_ref()).unwrap();
    assert_eq!((decoded.width, decoded.height), (64, 64));
    assert!(output.client.is_none());
    drop(output);
    let mut withdrawn = BTreeSet::new();
    for _ in 0..2 {
        let sample = tokens
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap();
        assert_eq!(sample.kind(), zenoh::sample::SampleKind::Delete);
        withdrawn.insert(sample.key_expr().to_string());
    }
    assert_eq!(declared, withdrawn);
    assert!(scope.entries.lock().unwrap().is_empty());
}

#[test]
fn publication_budget_accepts_exact_bytes_and_rejects_one_byte_short_before_send() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    let info = session.declare_subscriber("@/camera@/info").wait().unwrap();
    let make = |max_message_bytes| {
        CameraOutput::new(
            &sim,
            &session,
            &scope,
            id,
            CameraOutputConfig {
                origin: [1_000_000_000.0, 0.0, 0.0],
                frame_id: "frame".into(),
                image_key: "@/camera@/image".into(),
                info_key: "@/camera@/info".into(),
                worker_path: "/bin/cat".into(),
                max_message_bytes,
            },
        )
        .unwrap()
    };
    let mut output = make(limit);
    assert_eq!(output.request(&sim).unwrap(), Progress::InfoPublished);
    let expected = info
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap()
        .payload()
        .to_bytes()
        .into_owned();
    assert!(expected.len() > 1);
    drop(output);
    let mut output = make(NonZeroUsize::new(expected.len()).unwrap());
    assert_eq!(output.request(&sim).unwrap(), Progress::InfoPublished);
    let actual = info.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    assert_eq!(actual.payload().to_bytes().as_ref(), expected);
    drop(output);
    let mut output = make(NonZeroUsize::new(expected.len() - 1).unwrap());
    assert!(
        matches!(output.request(&sim), Err(CameraOutputError::PublicationTooLarge { size, limit }) if size == expected.len() && limit + 1 == size)
    );
    assert!(output.client.is_none());
    assert!(output.acquisition.is_none());
    assert!(
        info.recv_timeout(Duration::from_millis(20))
            .unwrap()
            .is_none()
    );
}

#[test]
fn native_invalid_names_cannot_reserve_or_advertise_camera_topics() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    let tokens = session
        .liveliness()
        .declare_subscriber("@gz/**")
        .wait()
        .unwrap();
    let mut invalid = vec![
        "@/@/info".to_owned(),
        "@/camera@/".to_owned(),
        "@/camera@/bad topic".to_owned(),
        "@/camera@/bad~topic".to_owned(),
        "@/camera@/bad:=topic".to_owned(),
        "@/camera@/bad//topic".to_owned(),
        "@/camera@/bad@topic".to_owned(),
        "@/bad partition@/info".to_owned(),
        "@/bad~partition@/info".to_owned(),
        "@/bad:=partition@/info".to_owned(),
        "@/bad//partition@/info".to_owned(),
    ];
    let prefix = "@/camera@/";
    let longest = format!(
        "{prefix}{}",
        "a".repeat(usize::from(u16::MAX) - prefix.len())
    );
    assert_eq!(longest.len(), usize::from(u16::MAX));
    // Qualify the native name bound without advertising a huge token. Both
    // components and the qualified key must fit native's byte-counted domain.
    assert!(publisher_token(&longest, &session, "boundary-node", "gz.msgs.CameraInfo").is_ok());
    invalid.push(format!("{longest}a"));
    for key in invalid {
        for bad_image in [false, true] {
            let (image_key, info_key) = if bad_image {
                (key.clone(), "@/camera@/info".into())
            } else {
                ("@/camera@/image".into(), key.clone())
            };
            assert!(
                matches!(
                    CameraOutput::new(
                        &sim,
                        &session,
                        &scope,
                        id,
                        CameraOutputConfig {
                            origin: [1_000_000_000.0, 0.0, 0.0],
                            frame_id: "frame".into(),
                            image_key,
                            info_key,
                            worker_path: "/bin/cat".into(),
                            max_message_bytes: limit,
                        }
                    ),
                    Err(CameraOutputError::InvalidPublicationKey)
                ),
                "{key:?}"
            );
            assert!(scope.entries.lock().unwrap().is_empty());
            // Local liveliness is synchronous (also tested by the successful
            // declaration/withdrawal control), so this is no quiet-time guess.
            assert!(tokens.try_recv().unwrap().is_none());
        }
    }
}

#[test]
fn image_wire_budget_and_closed_publisher_preserve_typed_failure() {
    let (session, _, _) = session();
    let key = "@/camera-wire@/image";
    let incoming = session
        .declare_subscriber(key)
        .allowed_origin(zenoh::sample::Locality::SessionLocal)
        .wait()
        .unwrap();
    let publisher = session
        .declare_publisher(key)
        .congestion_control(zenoh::qos::CongestionControl::Drop)
        .wait()
        .unwrap();
    let rgba = [1, 2, 3, 255].repeat(64 * 64);
    let message = image_packet(
        &rgba,
        Duration::new(i64::MAX as u64, 999_999_999),
        "frame/boundary",
        u64::MAX,
    )
    .unwrap();
    let expected = message.encode_to_vec();
    let exact = NonZeroUsize::new(expected.len()).unwrap();
    publish(&publisher, message.clone(), exact).unwrap();
    let sample = incoming
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap();
    assert_eq!(sample.key_expr().as_str(), key);
    assert_eq!(sample.encoding().to_string(), "zenoh/bytes");
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.Image"
    );
    assert_eq!(sample.payload().to_bytes().as_ref(), expected);
    let decoded = gz_sim_proto::msgs::Image::decode(sample.payload().to_bytes().as_ref()).unwrap();
    assert_eq!((decoded.width, decoded.height, decoded.step), (64, 64, 192));
    assert_eq!(
        decoded.pixel_format_type,
        gz_sim_proto::msgs::PixelFormatType::RgbInt8 as i32
    );
    assert_eq!(decoded.data, [1, 2, 3].repeat(64 * 64));
    let header = decoded.header.unwrap();
    assert_eq!(header.data.len(), 2);
    assert_eq!(
        header.stamp.unwrap(),
        gz_sim_proto::msgs::Time {
            sec: i64::MAX,
            nsec: 999_999_999,
        }
    );
    assert_eq!(header.data[0].key, "frame_id");
    assert_eq!(header.data[0].value, ["frame/boundary"]);
    assert_eq!(header.data[1].key, "seq");
    assert_eq!(header.data[1].value, [u64::MAX.to_string()]);
    assert!(matches!(
        publish(&publisher, message.clone(), NonZeroUsize::new(expected.len() - 1).unwrap()),
        Err(CameraOutputError::PublicationTooLarge { size, limit })
            if size == expected.len() && limit + 1 == size
    ));
    assert!(incoming.try_recv().unwrap().is_none());
    session.close().wait().unwrap();
    assert!(matches!(
        publish(&publisher, message, exact),
        Err(CameraOutputError::Zenoh(_))
    ));
}

#[test]
fn noncanonical_camera_keys_fail_before_reservation_or_liveliness() {
    let (sim, id) = fixture();
    let (session, scope, limit) = session();
    let tokens = session
        .liveliness()
        .declare_subscriber("@gz/**")
        .wait()
        .unwrap();
    for key in ["@camera@/image", "@/camera@image", "@/camera@/image/"] {
        for bad_image in [false, true] {
            let (image_key, info_key) = if bad_image {
                (key.into(), "@/camera@/info".into())
            } else {
                ("@/camera@/image".into(), key.into())
            };
            assert!(
                matches!(
                    CameraOutput::new(
                        &sim,
                        &session,
                        &scope,
                        id,
                        CameraOutputConfig {
                            origin: [1_000_000_000.0, 0.0, 0.0],
                            frame_id: "frame".into(),
                            image_key,
                            info_key,
                            worker_path: "/bin/cat".into(),
                            max_message_bytes: limit,
                        },
                    ),
                    Err(CameraOutputError::InvalidPublicationKey)
                ),
                "{key}"
            );
            assert!(scope.entries.lock().unwrap().is_empty());
            assert!(tokens.try_recv().unwrap().is_none());
        }
    }
}

#[path = "loaded_frame.rs"]
mod loaded_frame;

#[test]
fn cylinder_rgb_info_and_pending_snapshot_keep_lazy_worker_and_rgb_capture_kind() {
    let sim = load_world_with_sdf_step(
        include_str!("../../../../examples/m64_rgb_cylinder.sdf"),
        "m64-output.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find_map(|row| {
            row.contains::<gz_sim_scene::CameraSensor>()
                .then(|| row.get::<EntityId>().unwrap().get())
        })
        .unwrap();
    let (session, scope, limit) = session();
    let mut output = CameraOutput::new(
        &sim,
        &session,
        &scope,
        id,
        CameraOutputConfig {
            origin: [0.0; 3],
            frame_id: "rig/body/camera".into(),
            image_key: "@/m64@/image".into(),
            info_key: "@/m64@/info".into(),
            worker_path: "/bin/cat".into(),
            max_message_bytes: limit,
        },
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::NoDemand);
    assert!(output.client.is_none());
    let info = session.declare_subscriber("@/m64@/info").wait().unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::InfoPublished);
    assert!(output.client.is_none());
    let wire = info.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    let value = gz_sim_proto::msgs::CameraInfo::decode(wire.payload().to_bytes().as_ref()).unwrap();
    assert_eq!((value.width, value.height), (64, 64));
    assert_eq!(value.header.unwrap().data[0].value, ["rig/body/camera"]);
    let image = session.declare_subscriber("@/m64@/image").wait().unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    assert!(info.recv_timeout(Duration::from_secs(5)).unwrap().is_some());
    let acquisition = output.acquisition.as_ref().unwrap();
    assert!(!acquisition.depth);
    assert_eq!(acquisition.stamp.epoch, 0);
    assert!(
        matches!(&acquisition.phase,Phase::Submit(Request::CalibratedShapeSnapshot { visuals,.. }) if visuals.len()==3)
    );
    // cat is deliberately not a renderer: no Ready means no request may enter
    // the pipe or manufacture an image. The new snapshot stays pending RGB work.
    for _ in 0..2 {
        assert_eq!(output.pump(&sim).unwrap(), Progress::Backpressure);
        assert!(matches!(
            &output.acquisition.as_ref().unwrap().phase,
            Phase::Submit(Request::CalibratedShapeSnapshot { .. })
        ));
    }
    assert!(image.try_recv().unwrap().is_none());
    assert!(info.try_recv().unwrap().is_none());
    assert_eq!(output.sequence, 0);
    output.retire();
    assert!(output.client.is_none() && output.acquisition.is_none());
}

#[test]
fn cylinder_invalid_and_depth_geometry_fail_before_publication_admission() {
    use gz_sim_scene::{CameraSensor, VisualGeometry};
    let mut sim = load_world_with_sdf_step(
        include_str!("../../../../examples/m64_rgb_cylinder.sdf"),
        "m64-invalid-output.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let camera = sim
        .world()
        .iter_entities()
        .find(|row| row.contains::<CameraSensor>())
        .unwrap()
        .id();
    let id = sim.world().get::<EntityId>(camera).unwrap().get();
    let visual = sim
        .world()
        .iter_entities()
        .find(|row| {
            row.get::<Name>()
                .is_some_and(|name| name.0 == "default::cylinder::body::red_cap")
        })
        .unwrap()
        .id();
    let (session, scope, limit) = session();
    let config = || CameraOutputConfig {
        origin: [0.0; 3],
        frame_id: "frame".into(),
        image_key: "@/m64@/image".into(),
        info_key: "@/m64@/info".into(),
        worker_path: "/nonexistent/worker".into(),
        max_message_bytes: limit,
    };
    sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
        Arc::new(sdformat::Geometry::Cylinder {
            radius: 0.0,
            length: 2.0,
        });
    assert!(matches!(
        CameraOutput::new(&sim, &session, &scope, id, config()),
        Err(CameraOutputError::Snapshot(SnapshotError::InvalidVisual(_)))
    ));
    assert!(scope.entries.lock().unwrap().is_empty());
    sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
        Arc::new(sdformat::Geometry::Cylinder {
            radius: 1.0,
            length: 2.0,
        });
    Arc::make_mut(&mut sim.world_mut().get_mut::<CameraSensor>(camera).unwrap().0).kind =
        "depth_camera".into();
    assert!(matches!(
        CameraOutput::new(&sim, &session, &scope, id, config()),
        Err(CameraOutputError::Snapshot(
            SnapshotError::UnsupportedGeometry(_)
        ))
    ));
    assert!(scope.entries.lock().unwrap().is_empty());
}

#[test]
fn shared_writer_scope_recovers_poison_without_releasing_peer_ownership() {
    let sim = load_world_with_sdf_step(
        include_str!("../../../../examples/m63_two_rgb_cameras.sdf"),
        "shared-camera-scope.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let ids = [
        "default::rig_a::body::camera",
        "default::rig_b::body::camera",
    ]
    .map(|name| {
        sim.world()
            .iter_entities()
            .find_map(|row| {
                (row.get::<Name>()?.0 == name).then(|| row.get::<EntityId>().unwrap().get())
            })
            .unwrap()
    });
    assert_ne!(ids[0], ids[1]);
    let systems = sim.scheduled_system_counts();
    let (session, _, limit) = session();
    let scope = CameraWriterScope::new(NonZeroUsize::new(2).unwrap());
    let shared = scope.clone();
    let tokens = session
        .liveliness()
        .declare_subscriber("@gz/**")
        .wait()
        .unwrap();
    let infos = session
        .declare_subscriber("@/shared@/b/info")
        .wait()
        .unwrap();
    let make = |index: usize, suffix: &str| {
        CameraOutput::new(
            &sim,
            &session,
            &shared,
            ids[index],
            CameraOutputConfig {
                origin: [0.0; 3],
                frame_id: format!("owner/{suffix}"),
                image_key: format!("@/shared@/{suffix}/image"),
                info_key: format!("@/shared@/{suffix}/info"),
                worker_path: "/bin/cat".into(),
                max_message_bytes: limit,
            },
        )
    };
    let mut a = make(0, "a").unwrap();
    // A caught fixture fault poisons only the reservation lock. The production
    // recovery paths must conserve the already-held real owner's two keys.
    assert!(
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _held = scope.entries.lock().unwrap();
            panic!("intentional shared-writer reservation poison");
        }))
        .is_err()
    );
    assert!(scope.entries.is_poisoned());
    let mut b = make(1, "b").unwrap();
    let held = || {
        scope
            .entries
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .clone()
    };
    assert_eq!(held().len(), 4);
    assert!(matches!(
        make(0, "a"),
        Err(CameraOutputError::AlreadyPublishing)
    ));
    assert!(matches!(
        make(0, "third"),
        Err(CameraOutputError::WriterLimit)
    ));
    let mut declared = BTreeSet::new();
    for _ in 0..4 {
        let sample = tokens
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap();
        assert_eq!(sample.kind(), zenoh::sample::SampleKind::Put);
        let fields: Vec<_> = sample.key_expr().as_str().split('/').collect();
        assert_eq!(&fields[..3], ["@gz", "%shared", &session.zid().to_string()]);
        assert!(matches!(
            fields[9],
            "%a%image" | "%a%info" | "%b%image" | "%b%info"
        ));
        declared.insert(sample.key_expr().to_string());
    }
    assert_eq!(declared.len(), 4);
    a.retire();
    assert_eq!(held().len(), 4);
    assert!(tokens.try_recv().unwrap().is_none());
    assert!(matches!(
        make(0, "a"),
        Err(CameraOutputError::AlreadyPublishing)
    ));
    drop(a);
    assert!(!session.is_closed());
    assert_eq!(held().len(), 2);
    for _ in 0..2 {
        let sample = tokens
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap();
        assert_eq!(sample.kind(), zenoh::sample::SampleKind::Delete);
        assert!(
            sample
                .key_expr()
                .as_str()
                .split('/')
                .nth(9)
                .unwrap()
                .starts_with("%a%")
        );
        assert!(declared.remove(sample.key_expr().as_str()));
    }
    assert_eq!(declared.len(), 2);
    assert!(tokens.try_recv().unwrap().is_none());
    assert_eq!(b.request(&sim).unwrap(), Progress::InfoPublished);
    let sample = infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    assert_eq!(sample.key_expr().as_str(), "@/shared@/b/info");
    assert_eq!(sample.encoding().to_string(), "zenoh/bytes");
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.CameraInfo"
    );
    let info =
        gz_sim_proto::msgs::CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
    assert_eq!((info.width, info.height), (64, 64));
    let header = info.header.unwrap();
    assert_eq!(
        header.stamp.unwrap(),
        gz_sim_proto::msgs::Time { sec: 0, nsec: 0 }
    );
    assert_eq!(header.data[0].value, ["owner/b"]);
    assert!(b.client.is_none() && b.acquisition.is_none());
    assert!(infos.try_recv().unwrap().is_none());
    drop(b);
    assert!(!session.is_closed());
    assert!(held().is_empty());
    for _ in 0..2 {
        let sample = tokens
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap();
        assert_eq!(sample.kind(), zenoh::sample::SampleKind::Delete);
        assert!(
            sample
                .key_expr()
                .as_str()
                .split('/')
                .nth(9)
                .unwrap()
                .starts_with("%b%")
        );
        assert!(declared.remove(sample.key_expr().as_str()));
    }
    assert!(declared.is_empty());
    assert_eq!((sim.elapsed(), sim.epoch()), (Duration::ZERO, 0));
    assert_eq!(sim.scheduled_system_counts(), systems);
}

#[test]
fn buffered_rgb_and_depth_failures_preserve_canonical_publication_state() {
    for depth in [false, true] {
        let (mut sim, id) = fixture();
        if depth {
            let entity = entity_by_id(sim.world(), id).unwrap();
            Arc::make_mut(&mut sim.world_mut().get_mut::<CameraSensor>(entity).unwrap().0).kind =
                "depth_camera".into();
        }
        let (session, scope, limit) = session();
        let incoming = session
            .declare_subscriber("@/buffered@/image")
            .allowed_origin(zenoh::sample::Locality::SessionLocal)
            .wait()
            .unwrap();
        let mut output = CameraOutput::new(
            &sim,
            &session,
            &scope,
            id,
            CameraOutputConfig {
                origin: [1_000_000_000.0, 0.0, 0.0],
                frame_id: "buffered/frame".into(),
                image_key: "@/buffered@/image".into(),
                info_key: "@/buffered@/info".into(),
                worker_path: "/bin/cat".into(),
                max_message_bytes: limit,
            },
        )
        .unwrap();
        assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
        let lifetime = output.client.as_ref().unwrap().diagnostic_lifetime();
        let stamp = Stamp {
            epoch: sim.epoch(),
            seconds: 0,
            nanoseconds: 0,
        };
        // Inject at the private completed-frame boundary, not through a fake
        // renderer reply. These negatives test publication guards; cat never
        // supplied Ready or a frame, and no worker-rendering claim is made.
        let buffered = |stamp: Stamp, data: Vec<u8>| Acquisition {
            stamp,
            capture_id: 2,
            depth,
            phase: if depth {
                Phase::PublishDepthFrame {
                    stamp,
                    metres_le: data,
                }
            } else {
                Phase::PublishFrame { stamp, rgba: data }
            },
        };
        output.acquisition = Some(buffered(stamp, vec![0]));
        let expected_error = if depth {
            CameraMessageError::InvalidDepthFrame
        } else {
            CameraMessageError::InvalidFrame
        };
        assert!(
            matches!(output.pump(&sim), Err(CameraOutputError::Message(error)) if error == expected_error)
        );
        assert_eq!(output.sequence, 0);
        assert!(output.acquisition.is_none() && !output.retiring);
        assert!(incoming.try_recv().unwrap().is_none());
        let data = if depth {
            let mut bytes = 1.25_f32.to_le_bytes().repeat(64 * 64);
            bytes[..4].copy_from_slice(&f32::INFINITY.to_le_bytes());
            bytes
        } else {
            [12_u8, 34, 56, 255].repeat(64 * 64)
        };
        if depth {
            let mut invalid = data.clone();
            invalid[4..8].copy_from_slice(&f32::NAN.to_le_bytes());
            output.acquisition = Some(buffered(stamp, invalid));
            assert!(matches!(
                output.pump(&sim),
                Err(CameraOutputError::Message(
                    CameraMessageError::InvalidDepthFrame
                ))
            ));
            assert_eq!(output.sequence, 0);
            assert!(output.acquisition.is_none());
            assert!(incoming.try_recv().unwrap().is_none());
        }
        let out_of_range = Stamp {
            seconds: i64::MAX as u64 + 1,
            ..stamp
        };
        output.acquisition = Some(buffered(out_of_range, data.clone()));
        assert!(matches!(
            output.pump(&sim),
            Err(CameraOutputError::Message(CameraMessageError::TimeRange))
        ));
        assert_eq!(output.sequence, 0);
        assert!(output.acquisition.is_none());
        assert!(incoming.try_recv().unwrap().is_none());
        let message = |sequence| {
            if depth {
                depth_image_packet(&data, Duration::ZERO, "buffered/frame", sequence).unwrap()
            } else {
                image_packet(&data, Duration::ZERO, "buffered/frame", sequence).unwrap()
            }
        };
        let size = message(0).encoded_len();
        output.max_message_bytes = NonZeroUsize::new(size - 1).unwrap();
        output.acquisition = Some(buffered(stamp, data.clone()));
        assert!(
            matches!(output.pump(&sim), Err(CameraOutputError::PublicationTooLarge { size: actual, limit }) if actual == size && limit + 1 == size)
        );
        // The existing sequence advances before publish; a failed publication
        // consumes that sequence and cannot silently retry the buffered frame.
        assert_eq!(output.sequence, 1);
        assert!(output.acquisition.is_none() && !output.retiring);
        assert!(incoming.try_recv().unwrap().is_none());
        for sequence in [1, u64::MAX] {
            output.sequence = sequence;
            let expected = message(sequence).encode_to_vec();
            output.max_message_bytes = NonZeroUsize::new(expected.len()).unwrap();
            output.acquisition = Some(buffered(stamp, data.clone()));
            assert_eq!(output.pump(&sim).unwrap(), Progress::ImagePublished);
            assert_eq!(output.sequence, sequence.wrapping_add(1));
            assert!(output.acquisition.is_none());
            let sample = incoming
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap();
            assert_eq!(sample.key_expr().as_str(), "@/buffered@/image");
            assert_eq!(sample.encoding().to_string(), "zenoh/bytes");
            assert_eq!(
                sample.attachment().unwrap().to_bytes().as_ref(),
                b"gz.msgs.Image"
            );
            assert_eq!(sample.payload().to_bytes().as_ref(), expected);
            let image =
                gz_sim_proto::msgs::Image::decode(sample.payload().to_bytes().as_ref()).unwrap();
            assert_eq!((image.width, image.height), (64, 64));
            if depth {
                assert_eq!(image.step, 256);
                assert_eq!(
                    image.pixel_format_type,
                    gz_sim_proto::msgs::PixelFormatType::RFloat32 as i32
                );
                assert_eq!(image.data, data);
            } else {
                assert_eq!(image.step, 192);
                assert_eq!(
                    image.pixel_format_type,
                    gz_sim_proto::msgs::PixelFormatType::RgbInt8 as i32
                );
                assert_eq!(image.data, [12_u8, 34, 56].repeat(64 * 64));
            }
            let header = image.header.unwrap();
            assert_eq!(
                header.stamp.unwrap(),
                gz_sim_proto::msgs::Time { sec: 0, nsec: 0 }
            );
            assert_eq!(header.data.len(), 2);
            assert_eq!(
                (header.data[0].key.as_str(), header.data[0].value.as_slice()),
                ("frame_id", ["buffered/frame".to_owned()].as_slice())
            );
            assert_eq!(
                (header.data[1].key.as_str(), header.data[1].value.as_slice()),
                ("seq", [sequence.to_string()].as_slice())
            );
            assert_eq!(output.pump(&sim).unwrap(), Progress::NoDemand);
            assert!(incoming.try_recv().unwrap().is_none());
        }
        output.retire();
        assert!(lifetime.is_retired());
        assert!(output.client.is_none() && output.acquisition.is_none());
        assert!(!session.is_closed());
        drop(output);
        assert!(scope.entries.lock().unwrap().is_empty());
        assert_eq!((sim.elapsed(), sim.epoch()), (Duration::ZERO, 0));
    }
}
