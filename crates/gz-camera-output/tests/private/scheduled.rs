// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::prost::Message;
use gz_sim_scene::{
    Name, PhysicsEngine, load_world_with_sdf_step, remove_model, reset_loaded_world,
};
use std::{num::NonZeroUsize, sync::Arc};
use zenoh::Wait;

fn source(rate: f64) -> String {
    include_str!("../../../gz-sim-scene/tests/fixtures/camera-binding.sdf").replace(
        "<topic>/explicit-camera</topic>",
        &format!("<update_rate>{rate}</update_rate><topic>/explicit-camera</topic>"),
    )
}

fn simulation(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "scheduled.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap()
}

fn setup() -> (Session, CameraWriterScope) {
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
        CameraWriterScope::new(NonZeroUsize::new(1).unwrap()),
    )
}

fn config(limit: usize) -> CameraOutputConfig {
    CameraOutputConfig {
        origin: [1_000_000_000.0, 0.0, 0.0],
        frame_id: "camera_frame".into(),
        image_key: "@/scheduled@/image".into(),
        info_key: "@/scheduled@/info".into(),
        worker_path: "/nonexistent/worker".into(),
        max_message_bytes: NonZeroUsize::new(limit).unwrap(),
    }
}

fn received_time(sample: zenoh::sample::Sample) -> Duration {
    let message =
        gz_sim_proto::msgs::CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
    assert_eq!(message.header.as_ref().unwrap().data[0].key, "frame_id");
    assert_eq!(
        message.header.as_ref().unwrap().data[0].value,
        ["camera_frame"]
    );
    let stamp = message.header.unwrap().stamp.unwrap();
    Duration::new(
        stamp.sec.try_into().unwrap(),
        stamp.nsec.try_into().unwrap(),
    )
}

fn late_interest_after_observation(image: bool) {
    let mut sim = simulation(&source(100.0));
    let (session, scope) = setup();
    let mut owner = ScheduledCameraOutput::prepare(&sim, &session, &scope, config(1024))
        .unwrap()
        .unwrap();
    sim.step_exact(1).unwrap();
    // Observe actual local routing state, then install real subscribers before
    // production consumes that snapshot. There is no timing or callback hook.
    let observed = owner.output.demand().unwrap();
    assert_eq!(observed, (false, false));
    let infos = session
        .declare_subscriber("@/scheduled@/info")
        .wait()
        .unwrap();
    let images = image.then(|| {
        session
            .declare_subscriber("@/scheduled@/image")
            .wait()
            .unwrap()
    });
    assert_eq!(owner.output.demand().unwrap(), (image, true));
    assert_eq!(
        owner.request_observed(&sim, observed).unwrap(),
        Some(Progress::NoDemand)
    );
    assert_eq!(owner.last_attempt, None);
    assert!(owner.schedule.is_due(sim.elapsed()));
    assert!(owner.output.acquisition.is_none());
    assert!(owner.output.client.is_none());
    assert!(infos.try_recv().unwrap().is_none());

    assert_eq!(
        owner.request_due(&sim).unwrap(),
        Some(if image {
            Progress::Pending
        } else {
            Progress::InfoPublished
        })
    );
    assert_eq!(owner.last_attempt, Some(Duration::from_millis(1)));
    assert_eq!(owner.output.acquisition.is_some(), image);
    assert_eq!(owner.output.client.is_some(), image);
    assert_eq!(owner.request_due(&sim).unwrap(), None);
    assert_eq!(
        received_time(infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap()),
        Duration::from_millis(1)
    );
    assert!(infos.try_recv().unwrap().is_none());
    if let Some(images) = images {
        assert!(images.try_recv().unwrap().is_none());
    }
    owner.set_active(false);
    owner.retire();
}

#[test]
fn observed_no_demand_cannot_admit_late_info() {
    late_interest_after_observation(false);
}

#[test]
fn observed_no_demand_cannot_admit_late_image() {
    late_interest_after_observation(true);
}

#[test]
fn loaded_rate_uses_committed_steps_without_repeated_paused_or_wall_time_samples() {
    let mut sim = simulation(&source(100.0));
    let systems = sim.scheduled_system_counts();
    let (session, scope) = setup();
    let infos = session
        .declare_subscriber("@/scheduled@/info")
        .wait()
        .unwrap();
    let mut owner = ScheduledCameraOutput::prepare(&sim, &session, &scope, config(1024))
        .unwrap()
        .unwrap();
    assert!(owner.is_active());
    assert_eq!(owner.request_due(&sim).unwrap(), None);
    let mut stamps = Vec::new();
    for _ in 0..50 {
        // step_exact advances a committed frame even while configured paused.
        assert!(sim.is_paused());
        sim.step_exact(1).unwrap();
        if owner.request_due(&sim).unwrap() == Some(Progress::InfoPublished) {
            stamps.push(received_time(
                infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
            ));
        }
        assert_eq!(owner.request_due(&sim).unwrap(), None);
        assert!(owner.output.client.is_none());
    }
    assert_eq!(stamps, [1, 10, 20, 30, 40, 50].map(Duration::from_millis));
    for _ in 0..3 {
        let update = sim.tick().unwrap();
        assert!(update.paused && update.dt.is_zero());
        assert_eq!(owner.request_due(&sim).unwrap(), None);
        assert_eq!(owner.pump(&sim).unwrap(), Progress::NoDemand);
    }
    assert!(infos.try_recv().unwrap().is_none());
    assert_eq!(sim.scheduled_system_counts(), systems);
}

#[test]
fn no_demand_inactive_and_skipped_time_preserve_native_phase_without_backlog() {
    let mut sim = simulation(&source(100.0));
    let (session, scope) = setup();
    let mut owner = ScheduledCameraOutput::prepare(&sim, &session, &scope, config(1024))
        .unwrap()
        .unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(owner.request_due(&sim).unwrap(), Some(Progress::NoDemand));
    assert_eq!(owner.last_attempt, None);
    let infos = session
        .declare_subscriber("@/scheduled@/info")
        .wait()
        .unwrap();
    assert_eq!(
        owner.request_due(&sim).unwrap(),
        Some(Progress::InfoPublished)
    );
    assert_eq!(
        received_time(infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap()),
        Duration::from_millis(1)
    );
    owner.set_active(false);
    assert!(!owner.is_active());
    sim.step_exact(34).unwrap();
    assert_eq!(owner.request_due(&sim).unwrap(), None);
    assert_eq!(owner.last_attempt, Some(Duration::from_millis(1)));
    assert!(owner.output.client.is_none());
    owner.set_active(true);
    assert_eq!(
        owner.request_due(&sim).unwrap(),
        Some(Progress::InfoPublished)
    );
    assert_eq!(
        received_time(infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap()),
        Duration::from_millis(35)
    );
    sim.step_exact(4).unwrap();
    assert_eq!(owner.request_due(&sim).unwrap(), None);
    sim.step_exact(1).unwrap();
    assert_eq!(
        owner.request_due(&sim).unwrap(),
        Some(Progress::InfoPublished)
    );
    assert_eq!(
        received_time(infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap()),
        Duration::from_millis(40)
    );
    assert!(infos.try_recv().unwrap().is_none());
}

#[test]
fn unlimited_rate_still_admits_only_one_sample_per_committed_time() {
    let mut sim = simulation(&source(0.0));
    let (session, scope) = setup();
    let infos = session
        .declare_subscriber("@/scheduled@/info")
        .wait()
        .unwrap();
    let mut owner = ScheduledCameraOutput::prepare(&sim, &session, &scope, config(1024))
        .unwrap()
        .unwrap();
    for millis in 1..=3 {
        sim.step_exact(1).unwrap();
        assert_eq!(
            owner.request_due(&sim).unwrap(),
            Some(Progress::InfoPublished)
        );
        assert_eq!(owner.request_due(&sim).unwrap(), None);
        assert_eq!(
            received_time(infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap()),
            Duration::from_millis(millis)
        );
    }
    assert!(infos.try_recv().unwrap().is_none());
}

#[test]
fn failed_publication_consumes_attempt_but_reports_error_without_retry() {
    let mut sim = simulation(&source(100.0));
    let (session, scope) = setup();
    let infos = session
        .declare_subscriber("@/scheduled@/info")
        .wait()
        .unwrap();
    let mut owner = ScheduledCameraOutput::prepare(&sim, &session, &scope, config(1))
        .unwrap()
        .unwrap();
    sim.step_exact(1).unwrap();
    assert!(matches!(
        owner.request_due(&sim),
        Err(CameraOutputError::PublicationTooLarge { limit: 1, .. })
    ));
    assert_eq!(owner.request_due(&sim).unwrap(), None);
    assert_eq!(owner.last_attempt, Some(Duration::from_millis(1)));
    assert!(owner.output.client.is_none());
    sim.step_exact(8).unwrap();
    assert_eq!(owner.request_due(&sim).unwrap(), None);
    sim.step_exact(1).unwrap();
    assert!(matches!(
        owner.request_due(&sim),
        Err(CameraOutputError::PublicationTooLarge { limit: 1, .. })
    ));
    assert_eq!(owner.last_attempt, Some(Duration::from_millis(10)));
    assert!(infos.try_recv().unwrap().is_none());
}

#[test]
fn busy_and_inactive_worker_require_explicit_retirement_without_spending_a_slot() {
    let mut sim = simulation(&source(100.0));
    let (session, scope) = setup();
    let images = session
        .declare_subscriber("@/scheduled@/image")
        .wait()
        .unwrap();
    let mut owner = ScheduledCameraOutput::prepare(&sim, &session, &scope, config(1024))
        .unwrap()
        .unwrap();
    sim.step_exact(1).unwrap();
    // An absent executable reports through the real asynchronous supervisor;
    // request admission still owns exactly one bounded acquisition until pump.
    assert_eq!(owner.request_due(&sim).unwrap(), Some(Progress::Pending));
    let witness = owner.output.client.as_ref().unwrap().diagnostic_lifetime();
    sim.step_exact(9).unwrap();
    assert!(matches!(
        owner.request_due(&sim),
        Err(CameraOutputError::Busy)
    ));
    assert_eq!(owner.last_attempt, Some(Duration::from_millis(1)));
    owner.set_active(false);
    assert!(owner.output.acquisition.is_none());
    assert!(matches!(
        owner.request_due(&sim),
        Err(CameraOutputError::RetireRequired)
    ));
    assert_eq!(owner.pump(&sim).unwrap(), Progress::RetireRequired);
    owner.retire();
    assert!(witness.is_retired());
    assert!(owner.output.client.is_none());
    assert_eq!(owner.request_due(&sim).unwrap(), None);
    owner.set_active(true);
    assert_eq!(owner.request_due(&sim).unwrap(), Some(Progress::Pending));
    assert_eq!(owner.last_attempt, Some(Duration::from_millis(10)));
    owner.retire();
    assert!(images.try_recv().unwrap().is_none());
}

#[test]
fn absent_multiple_and_invalid_rate_fail_before_camera_resources() {
    let xml = source(100.0);
    let start = xml.find("<sensor ").unwrap();
    let end = xml[start..].find("</sensor>").unwrap() + start + "</sensor>".len();
    let absent = simulation(&format!("{}{}", &xml[..start], &xml[end..]));
    let systems = absent.scheduled_system_counts();
    let (session, scope) = setup();
    session.close().wait().unwrap();
    let mut invalid = config(1024);
    invalid.worker_path = "relative".into();
    invalid.frame_id.clear();
    assert!(
        ScheduledCameraOutput::prepare(&absent, &session, &scope, invalid)
            .unwrap()
            .is_none()
    );
    assert_eq!(absent.scheduled_system_counts(), systems);
    assert!(scope.entries.lock().unwrap().is_empty());
    let second = xml[start..end].replace("name=\"camera\"", "name=\"second\"");
    let multiple = simulation(&format!("{}{}{}", &xml[..end], second, &xml[end..]));
    assert!(matches!(
        ScheduledCameraOutput::prepare(&multiple, &session, &scope, config(1024)),
        Err(CameraOutputError::MultipleCameras)
    ));
    for rate in [f64::NAN, f64::INFINITY, 1e-300] {
        let mut sim = simulation(&xml);
        let entity = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<CameraSensor>())
            .unwrap()
            .id();
        Arc::make_mut(&mut sim.world_mut().get_mut::<CameraSensor>(entity).unwrap().0)
            .update_rate = rate;
        assert!(matches!(
            ScheduledCameraOutput::prepare(&sim, &session, &scope, config(1024)),
            Err(CameraOutputError::Schedule(_))
        ));
    }
    assert!(scope.entries.lock().unwrap().is_empty());
}

#[test]
fn reset_removal_and_replacement_invalidate_before_inactive_or_pause_gates() {
    let mut sim = simulation(&source(100.0));
    let (session, scope) = setup();
    let infos = session
        .declare_subscriber("@/scheduled@/info")
        .wait()
        .unwrap();
    let mut owner = ScheduledCameraOutput::prepare(&sim, &session, &scope, config(1024))
        .unwrap()
        .unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        owner.request_due(&sim).unwrap(),
        Some(Progress::InfoPublished)
    );
    assert_eq!(
        received_time(infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap()),
        Duration::from_millis(1)
    );
    owner.set_active(false);
    sim.set_paused(false);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert!(matches!(
        owner.request_due(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    assert!(matches!(
        owner.pump(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    owner.retire();
    drop(owner);
    let mut owner = ScheduledCameraOutput::prepare(&sim, &session, &scope, config(1024))
        .unwrap()
        .unwrap();
    assert!(owner.is_active());
    assert!(!sim.world().resource::<UpdateInfo>().paused);
    assert!(sim.world().resource::<UpdateInfo>().dt.is_zero());
    assert_eq!(owner.request_due(&sim).unwrap(), None);
    let update = sim.tick().unwrap();
    assert!(!update.paused);
    assert_eq!(update.dt, Duration::from_millis(1));
    assert_eq!(
        owner.request_due(&sim).unwrap(),
        Some(Progress::InfoPublished)
    );
    assert_eq!(
        received_time(infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap()),
        Duration::from_millis(1)
    );
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<CameraSensor>())
        .unwrap()
        .id();
    let mut camera = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
    camera.0 = Arc::new((*camera.0).clone());
    owner.set_active(false);
    assert!(matches!(
        owner.request_due(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    drop(owner);
    let mut owner = ScheduledCameraOutput::prepare(&sim, &session, &scope, config(1024))
        .unwrap()
        .unwrap();
    let model = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::robot"))
        .unwrap();
    let id = model.get::<EntityId>().unwrap().get();
    remove_model(&mut sim, id).unwrap();
    assert!(matches!(
        owner.pump(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    drop(owner);
    assert!(scope.entries.lock().unwrap().is_empty());
    assert!(
        ScheduledCameraOutput::prepare(&sim, &session, &scope, config(1024))
            .unwrap()
            .is_none()
    );
    assert!(infos.try_recv().unwrap().is_none());
}
