// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_msgs::msgs::{Entity, WorldReset};
use gz_sim_core::{EntityId, entity_by_id};

fn rewind(reset: WorldReset, steps: u32) -> WorldControl {
    WorldControl {
        pause: true,
        reset: Some(reset),
        multi_step: steps,
        ..Default::default()
    }
}

#[test]
fn reset_restores_removed_scene_ids_and_restarts_sensor_instances() {
    let mut server = imu_server(config());
    let mut initial: Vec<_> = server
        .simulation
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(EntityId::get))
        .collect();
    initial.sort();
    let ball = public_id(&server, "default::ball");
    let old_sensor = server.imus[0].entity;
    server.simulation.set_paused(false);
    for _ in 0..3 {
        server.tick().unwrap();
    }
    assert_eq!(server.imus[0].sequence, 3);
    gz_sim_scene::remove_model(&mut server.simulation, ball).unwrap();
    server.tick().unwrap();
    assert!(server.imus.is_empty());
    assert!(server.apply_control(rewind(
        WorldReset {
            all: true,
            ..Default::default()
        },
        0
    )));
    let info = server.tick().unwrap();
    assert_eq!(
        (info.sim_time, info.iteration, info.epoch, info.paused),
        (Duration::ZERO, 0, 1, true)
    );
    let mut restored: Vec<_> = server
        .simulation
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(EntityId::get))
        .collect();
    restored.sort();
    assert_eq!(restored, initial);
    assert_eq!(server.imus.len(), 1);
    assert_ne!(server.imus[0].entity, old_sensor);
    assert_eq!(server.imus[0].sequence, 0);
    assert_eq!(server.imus[0].last_sample, None);
    assert_eq!(server.counters.imus_queued, 3);
    let body = entity_by_id(
        server.simulation.world(),
        public_id(&server, "default::ball::body"),
    )
    .unwrap();
    assert_eq!(
        server
            .simulation
            .world()
            .get::<gz_sim_scene::WorldPose>(body)
            .unwrap()
            .translation[2],
        5.0
    );
    assert!(server.apply_control(WorldControl {
        pause: true,
        step: true,
        ..Default::default()
    }));
    server.tick().unwrap();
    assert_eq!(server.imus[0].sequence, 1);
    assert_eq!(
        server.imus[0].last_sample,
        Some((1, Duration::from_millis(1)))
    );
    assert_eq!(server.counters.imus_queued, 4);
}

#[test]
fn native_reset_flags_overwrite_and_pending_iteration_accounting() {
    let mut server = imu_server(config());
    assert!(server.apply_control(rewind(
        WorldReset {
            all: true,
            ..Default::default()
        },
        3
    )));
    for (millis, epoch) in [(0, 1), (1, 1), (2, 1)] {
        let info = server.tick().unwrap();
        assert_eq!(
            (info.sim_time, info.epoch),
            (Duration::from_millis(millis), epoch)
        );
    }
    assert_eq!(server.pending_steps(), 0);
    assert!(server.simulation.is_paused());
    for cancelling in [
        WorldControl {
            pause: true,
            ..Default::default()
        },
        rewind(
            WorldReset {
                model_only: true,
                ..Default::default()
            },
            0,
        ),
        rewind(WorldReset::default(), 0),
    ] {
        assert!(server.apply_control(rewind(
            WorldReset {
                all: true,
                ..Default::default()
            },
            0
        )));
        assert!(server.apply_control(cancelling));
        assert_eq!(server.tick().unwrap().epoch, 1);
    }
    assert!(server.apply_control(rewind(
        WorldReset {
            time_only: true,
            ..Default::default()
        },
        0
    )));
    assert!(!server.apply_control(WorldControl {
        seed: 7,
        ..Default::default()
    }));
    assert_eq!(server.tick().unwrap().epoch, 2);
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
    assert!(server.apply_control(WorldControl {
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    let info = server.tick().unwrap();
    assert_eq!(
        (info.epoch, info.paused, info.sim_time),
        (3, false, Duration::ZERO)
    );
    assert!(!server.simulation.is_paused());
    assert_eq!(server.tick().unwrap().sim_time, Duration::from_millis(1));
    let mut unloaded = super::server(config(), Duration::from_millis(1));
    assert!(unloaded.apply_control(rewind(
        WorldReset {
            model_only: true,
            ..Default::default()
        },
        0
    )));
    assert_eq!(unloaded.tick().unwrap().epoch, 0);
}

#[test]
fn reset_cancels_only_admitted_removals_after_commit_and_preserves_live_endpoint() {
    let cfg = config();
    let mut server = imu_server(cfg.clone());
    let ball = public_id(&server, "default::ball");
    let endpoint = Arc::clone(&server.imus[0].publisher);
    let ordinary = removal_request(
        cfg.clone(),
        false,
        Entity {
            id: ball,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    let blocking = removal_request(
        cfg,
        true,
        Entity {
            id: ball,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    poll_removals(&mut server, 2);
    assert!(removal_result(ordinary));
    assert!(!blocking.is_finished());
    assert!(server.apply_control(rewind(
        WorldReset {
            all: true,
            ..Default::default()
        },
        0
    )));
    server.tick().unwrap();
    assert!(!removal_result(blocking));
    assert_eq!(server.counters.removals_cancelled, 2);
    assert_eq!(
        server.counters.removals_completed + server.counters.removals_failed,
        0
    );
    assert!(entity_by_id(server.simulation.world(), ball).is_some());
    assert!(Arc::ptr_eq(&endpoint, &server.imus[0].publisher));
    assert_eq!(server.counters.response_failures, 0);
}

#[test]
fn failed_reset_keeps_clock_streams_and_admitted_removals() {
    use bevy_ecs::hierarchy::ChildOf;
    let cfg = config();
    let mut server = imu_server(cfg.clone());
    server.simulation.step_exact(2).unwrap();
    let ball = public_id(&server, "default::ball");
    let ball_entity = entity_by_id(server.simulation.world(), ball).unwrap();
    let registered = server.simulation.world_mut().register_system(|| ());
    server
        .simulation
        .world_mut()
        .entity_mut(registered.entity())
        .insert(ChildOf(ball_entity));
    let request = removal_request(
        cfg,
        true,
        Entity {
            id: ball,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    poll_removals(&mut server, 1);
    assert!(server.apply_control(rewind(
        WorldReset {
            all: true,
            ..Default::default()
        },
        0
    )));
    let before = *server.simulation.world().resource::<UpdateInfo>();
    let endpoint = Arc::clone(&server.imus[0].publisher);
    assert!(matches!(
        server.tick(),
        Err(Error::Reset(
            gz_sim_scene::SceneResetError::InfrastructureOwnership
        ))
    ));
    assert_eq!(*server.simulation.world().resource::<UpdateInfo>(), before);
    assert_eq!(server.simulation.elapsed(), Duration::from_millis(2));
    assert!(server.pending_reset);
    assert_eq!(server.counters.removals_cancelled, 0);
    assert!(!request.is_finished());
    assert!(Arc::ptr_eq(&endpoint, &server.imus[0].publisher));
    server
        .simulation
        .world_mut()
        .entity_mut(registered.entity())
        .remove::<ChildOf>();
    server.tick().unwrap();
    assert!(!removal_result(request));
    assert_eq!(server.counters.removals_cancelled, 1);
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
}

#[test]
fn rewind_validation_uses_zero_clock_and_checks_full_future_range() {
    let mut server = imu_server(config());
    server.simulation.step_exact(2).unwrap();
    assert!(server.validate_reset_steps(0).is_ok());
    assert!(server.validate_reset_steps(u64::MAX).is_ok());
    let huge = super::server(config(), Duration::MAX);
    assert!(matches!(
        huge.validate_reset_steps(1),
        Err(Error::TimeRange)
    ));
    assert!(matches!(
        huge.validate_reset_steps(2),
        Err(Error::Step(StepError::Overflow))
    ));
    assert!(matches!(
        huge.validate_reset_steps(u64::MAX),
        Err(Error::Step(StepError::Overflow))
    ));
    // Only the reset iteration is requested, so even a huge step need not advance time.
    let xml = include_str!("../../../gz-sim-scene/tests/fixtures/fall.sdf").replace(
        "<update_rate>1000</update_rate>",
        "<update_rate>0</update_rate>",
    );
    let sim = gz_sim_scene::load_world(
        &xml,
        "huge.sdf",
        "default",
        Duration::MAX,
        &sdformat::NoResources,
    )
    .unwrap();
    let mut huge = Server::new(sim, "default", config(), NonZeroUsize::new(2).unwrap()).unwrap();
    assert!(huge.apply_control(rewind(
        WorldReset {
            all: true,
            ..Default::default()
        },
        1
    )));
    assert_eq!(huge.tick().unwrap().sim_time, Duration::ZERO);
    assert!(huge.simulation.is_paused());
    assert!(!huge.apply_control(rewind(
        WorldReset {
            all: true,
            ..Default::default()
        },
        2
    )));
    assert_eq!(huge.pending_steps(), 0);
    assert!(!huge.pending_reset);
}

#[test]
fn reset_handler_stream_changes_are_explicit_committed_errors() {
    use bevy_ecs::prelude::*;
    for topic in ["/unprepared", "~@:=//"] {
        let mut server = imu_server(config());
        server.simulation.add_systems(
            gz_sim_core::Phase::Reset,
            move |mut streams: Query<&mut ImuPublication>| {
                for mut stream in &mut streams {
                    stream.topic = topic.into();
                }
            },
        );
        assert!(server.apply_control(rewind(
            WorldReset {
                all: true,
                ..Default::default()
            },
            0
        )));
        let error = server.tick().unwrap_err();
        if topic == "/unprepared" {
            assert!(matches!(error, Error::ResetStreams));
        } else {
            assert!(matches!(
                error,
                Error::Transport(TransportError::InvalidName)
            ));
        }
        assert_eq!(server.simulation.world().resource::<UpdateInfo>().epoch, 1);
        assert!(!server.pending_reset);
        assert_eq!(server.simulation.elapsed(), Duration::ZERO);
        assert!(matches!(server.tick(), Err(Error::ResetStreams)));
        assert!(server.imus.is_empty());
        assert_eq!(server.simulation.world().resource::<UpdateInfo>().epoch, 1);
    }
}

#[test]
fn reset_stream_reconciliation_preserves_shared_publishers_and_retires_additions() {
    let cfg = Config {
        namespace: "/robot".into(),
        ..config()
    };
    let xml = include_str!("../../../gz-sim-scene/tests/fixtures/fall.sdf")
        .replace(
            "<always_on>true</always_on>",
            "<always_on>true</always_on><topic>shared</topic>",
        )
        .replace(
            "</sensor>",
            "</sensor><sensor name='imu2' type='imu'><topic>/robot/shared</topic></sensor>",
        );
    let sim = gz_sim_scene::load_world(
        &xml,
        "shared.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let mut server = Server::new(sim, "default", cfg, NonZeroUsize::new(2).unwrap()).unwrap();
    let endpoint = Arc::clone(&server.imus[0].publisher);
    // A pending runtime-only stream is excluded from reconstructed instances.
    server.simulation.world_mut().spawn(ImuPublication {
        topic: "/runtime".into(),
        frame_id: "runtime".into(),
        entity_name: "runtime".into(),
    });
    assert!(server.apply_control(rewind(
        WorldReset {
            all: true,
            ..Default::default()
        },
        0
    )));
    server.tick().unwrap();
    assert_eq!(server.imus.len(), 2);
    assert!(
        server
            .imus
            .iter()
            .all(|stream| Arc::ptr_eq(&endpoint, &stream.publisher))
    );
    assert!(server.imus.iter().all(|stream| stream.sequence == 0));
    assert_eq!(
        server
            .simulation
            .world()
            .iter_entities()
            .filter(|e| e.contains::<ImuPublication>())
            .count(),
        2
    );
}

#[test]
fn reported_step_failure_stops_server_publications_and_future_controls() {
    use bevy_ecs::prelude::Res;
    use gz_sim_core::StepFault;
    let mut server = imu_server(config());
    server.simulation.set_paused(false);
    server.tick().unwrap();
    let before = server.counters;
    let committed = *server.simulation.world().resource::<UpdateInfo>();
    server
        .simulation
        .add_systems(gz_sim_core::Phase::PhysicsStep, |fault: Res<StepFault>| {
            fault.report(std::io::Error::other("physics step failed"));
        });
    assert!(matches!(
        server.tick(),
        Err(Error::Step(StepError::Faulted))
    ));
    assert_eq!(
        *server.simulation.world().resource::<UpdateInfo>(),
        committed
    );
    assert_eq!(server.simulation.elapsed(), Duration::from_millis(1));
    assert!(
        server
            .simulation
            .fault()
            .unwrap()
            .to_string()
            .contains("physics step failed")
    );
    assert_eq!(
        (
            server.counters.clocks_queued,
            server.counters.clock_failures
        ),
        (before.clocks_queued, before.clock_failures)
    );
    assert_eq!(
        (server.counters.poses_queued, server.counters.pose_failures),
        (before.poses_queued, before.pose_failures)
    );
    assert_eq!(
        (server.counters.imus_queued, server.counters.imu_failures),
        (before.imus_queued, before.imu_failures)
    );
    assert_eq!(
        (
            server.counters.controls_accepted,
            server.counters.controls_rejected
        ),
        (before.controls_accepted, before.controls_rejected)
    );
    assert!(!server.apply_control(WorldControl::default()));
    assert!(matches!(
        server.tick(),
        Err(Error::Step(StepError::Faulted))
    ));
    assert_eq!(server.counters.clocks_queued, before.clocks_queued);
    server.shutdown();
}

#[test]
fn committed_reset_fault_cancels_admitted_removals_and_retires_stale_streams() {
    use bevy_ecs::prelude::Res;
    use gz_sim_core::StepFault;
    let cfg = config();
    let mut server = imu_server(cfg.clone());
    server.simulation.step_exact(2).unwrap();
    server.tick().unwrap();
    let before = server.counters;
    let endpoint = Arc::downgrade(&server.imus[0].publisher);
    let ball = public_id(&server, "default::ball");
    let request = removal_request(
        cfg,
        true,
        Entity {
            id: ball,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    poll_removals(&mut server, 1);
    server
        .simulation
        .add_systems(gz_sim_core::Phase::Reset, |fault: Res<StepFault>| {
            fault.report(std::io::Error::other("reset callback failed"));
        });
    assert!(server.apply_control(rewind(
        WorldReset {
            all: true,
            ..Default::default()
        },
        0
    )));
    assert!(matches!(
        server.tick(),
        Err(Error::Reset(gz_sim_scene::SceneResetError::CommittedReset))
    ));
    assert!(!removal_result(request));
    assert_eq!(server.counters.removals_cancelled, 1);
    assert_eq!(
        server.counters.removals_completed + server.counters.removals_failed,
        0
    );
    assert!(!server.pending_reset);
    assert!(server.imus.is_empty());
    assert!(endpoint.upgrade().is_none());
    assert!(entity_by_id(server.simulation.world(), ball).is_some());
    assert_eq!(
        (
            server.simulation.elapsed(),
            server.simulation.world().resource::<UpdateInfo>().epoch
        ),
        (Duration::ZERO, 1)
    );
    assert_eq!(
        (
            server.counters.clocks_queued,
            server.counters.poses_queued,
            server.counters.imus_queued
        ),
        (
            before.clocks_queued,
            before.poses_queued,
            before.imus_queued
        )
    );
    assert!(matches!(
        server.tick(),
        Err(Error::Step(StepError::Faulted))
    ));
    assert!(!server.apply_control(WorldControl::default()));
    server.shutdown();
}
