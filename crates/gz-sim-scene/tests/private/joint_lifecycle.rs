// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use sdformat::NoResources;

const SOURCE: &str = include_str!("../fixtures/pendulum.sdf");

fn load(xml: &str, engine: PhysicsEngine) -> Simulation {
    load_world_with_engine(
        xml,
        "joint-demand.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        engine,
    )
    .unwrap()
}

fn engines() -> [PhysicsEngine; 3] {
    [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ]
}

fn model_id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

fn without_joint() -> String {
    let begin = SOURCE.find("<joint ").unwrap();
    let end = SOURCE[begin..].find("</joint>").unwrap() + begin + "</joint>".len();
    format!("{}{}", &SOURCE[..begin], &SOURCE[end..])
}

fn two_models() -> String {
    let begin = SOURCE.find("<model ").unwrap();
    let end = SOURCE[begin..].find("</model>").unwrap() + begin + "</model>".len();
    let second = SOURCE[begin..end]
        .replace("name='robot'", "name='other'")
        .replace("<pose>0 0 0 0 0.02 0</pose>", "<pose>3 0 0 0 0.02 0</pose>");
    SOURCE.replace("</world>", &format!("{second}</world>"))
}

fn with_separate_imu_model() -> String {
    let fall = include_str!("../fixtures/fall.sdf");
    let begin = fall.find("<model name=\"ball\"").unwrap();
    let end = fall[begin..].find("</model>").unwrap() + begin + "</model>".len();
    SOURCE.replace("</world>", &format!("{}</world>", &fall[begin..end]))
}

#[test]
fn absent_joints_have_no_preflight_or_sampling_callbacks() {
    for engine in engines() {
        let mut idle = load(&without_joint(), engine);
        let live = load(SOURCE, engine);
        let counts = idle.scheduled_system_counts();
        assert_eq!(live.scheduled_system_counts(), (counts.0 + 2, counts.1 + 1));
        assert!(!idle.world().contains_resource::<Installed>());
        install(&mut idle);
        idle.step_exact(3).unwrap();
        idle.reset().unwrap();
        assert!(!idle.world().contains_resource::<Installed>());
        assert_eq!(idle.scheduled_system_counts(), counts);
    }
    let idle = load(&without_joint(), PhysicsEngine::Tpe);
    assert!(!idle.world().contains_resource::<Installed>());
}

#[test]
fn only_last_joint_consumer_retires_and_loaded_reset_restores_callbacks() {
    for engine in engines() {
        let mut sim = load(&two_models(), engine);
        let counts = sim.scheduled_system_counts();
        install(&mut sim);
        assert_eq!(sim.scheduled_system_counts(), counts);
        sim.step_exact(3).unwrap();
        let first = model_id(&sim, "default::robot");
        let second = model_id(&sim, "default::other");
        remove_model(&mut sim, first).unwrap();
        assert!(sim.world().contains_resource::<Installed>());
        assert_eq!(sim.scheduled_system_counts(), counts);
        sim.step_exact(1).unwrap();
        let sample = sim
            .world_mut()
            .query::<&JointSample>()
            .single(sim.world())
            .unwrap();
        assert_eq!(sample.time, Duration::from_millis(4));
        remove_model(&mut sim, second).unwrap();
        let retired = (counts.0 - 2, counts.1 - 1);
        assert_eq!(sim.scheduled_system_counts(), retired);
        assert!(!sim.world().contains_resource::<Installed>());
        retire_if_unused(&mut sim).unwrap();
        sim.step_exact(2).unwrap();
        sim.reset().unwrap();
        assert_eq!(sim.scheduled_system_counts(), retired);
        reset_loaded_world(&mut sim, |sim| {
            assert!(sim.world().contains_resource::<Installed>());
            assert_eq!(sim.scheduled_system_counts(), counts);
            let samples: Vec<_> = sim
                .world()
                .iter_entities()
                .filter_map(|entity| entity.get::<JointSample>())
                .collect();
            assert_eq!(samples.len(), 2);
            for sample in samples {
                assert_eq!(sample.time, Duration::ZERO);
                assert_eq!(sample.epoch, sim.world().resource::<UpdateInfo>().epoch);
            }
        })
        .unwrap();
        sim.step_exact(1).unwrap();
    }
}

#[test]
fn invalid_reset_schedule_preserves_last_joint_model_and_physics() {
    fn a() {}
    fn b() {}
    for engine in engines() {
        let mut sim = load(&two_models(), engine);
        sim.step_exact(1).unwrap();
        sim.add_systems(Phase::Reset, (a.before(b), b.before(a)));
        let first = model_id(&sim, "default::robot");
        let second = model_id(&sim, "default::other");
        remove_model(&mut sim, first).unwrap();
        assert!(matches!(
            remove_model(&mut sim, second),
            Err(RemoveError::Schedule(_))
        ));
        assert!(gz_sim_core::entity_by_id(sim.world(), second).is_some());
        assert!(sim.world().contains_resource::<Installed>());
        sim.step_exact(1).unwrap();
        let sample = sim
            .world_mut()
            .query::<&JointSample>()
            .single(sim.world())
            .unwrap();
        assert_eq!(sample.time, Duration::from_millis(2));
    }
}

#[test]
fn schedule_observers_run_before_final_validation_or_after_body_commit() {
    use bevy_ecs::schedule::ScheduleBuilt;
    #[derive(Resource)]
    struct Observe {
        model: Entity,
        body: BodyId,
        seen: Option<(bool, bool)>,
    }
    for engine in engines() {
        for initialized in [false, true] {
            let mut sim = load(SOURCE, engine);
            if initialized {
                sim.initialize_schedules().unwrap();
            }
            let id = model_id(&sim, "default::robot");
            let model = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
            let body = sim
                .world()
                .iter_entities()
                .find_map(|entity| entity.get::<BodyHandle>())
                .unwrap()
                .id();
            sim.world_mut().insert_resource(Observe {
                model,
                body,
                seen: None,
            });
            sim.world_mut().add_observer(
                |_: On<ScheduleBuilt>,
                 mut observe: ResMut<Observe>,
                 models: Query<&Name>,
                 mut backend: ResMut<PhysicsBackend>| {
                    if observe.seen.is_none() {
                        let present = models.get(observe.model).is_ok();
                        let removed = backend.remove(observe.body);
                        observe.seen = Some((present, removed));
                    }
                },
            );
            let result = remove_model(&mut sim, id);
            if initialized {
                assert!(result.is_ok());
                assert_eq!(sim.world().resource::<Observe>().seen, Some((false, false)));
                assert!(gz_sim_core::entity_by_id(sim.world(), id).is_none());
                assert!(!sim.world().contains_resource::<Installed>());
            } else {
                assert_eq!(result, Err(RemoveError::InvalidPhysics));
                assert_eq!(sim.world().resource::<Observe>().seen, Some((true, true)));
                assert_eq!(gz_sim_core::entity_by_id(sim.world(), id), Some(model));
                assert!(sim.world().contains_resource::<Installed>());
            }
        }
    }
}

#[test]
fn last_joint_removed_by_observer_reports_a_terminal_committed_teardown_fault() {
    fn a() {}
    fn b() {}
    for engine in engines() {
        let mut sim = load(&two_models(), engine);
        sim.step_exact(1).unwrap();
        let first = model_id(&sim, "default::robot");
        let joint = |name: &str| {
            sim.world()
                .iter_entities()
                .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
                .unwrap()
                .id()
        };
        let first_joint = joint("default::robot::hinge");
        let other_joint = joint("default::other::hinge");
        sim.add_systems(Phase::Reset, (a.before(b), b.before(a)));
        sim.world_mut().add_observer(
            move |event: On<Remove<JointHandle>>, mut commands: Commands| {
                if event.entity == first_joint {
                    commands.entity(other_joint).despawn();
                }
            },
        );
        assert!(matches!(
            remove_model(&mut sim, first),
            Err(RemoveError::CommittedSchedule(_))
        ));
        assert!(gz_sim_core::entity_by_id(sim.world(), first).is_none());
        assert!(sim.world().get_entity(other_joint).is_err());
        assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
    }
}

#[test]
fn jointless_removal_does_not_initialize_unneeded_reset_work() {
    fn a() {}
    fn b() {}
    for engine in engines() {
        let mut sim = load(&without_joint(), engine);
        let id = model_id(&sim, "default::robot");
        sim.add_systems(Phase::Reset, (a.before(b), b.before(a)));
        remove_model(&mut sim, id).unwrap();
        assert!(gz_sim_core::entity_by_id(sim.world(), id).is_none());
        assert!(!sim.world().contains_resource::<Installed>());
        sim.step_exact(1).unwrap();
    }
}

#[test]
fn joint_and_imu_callbacks_retire_together_and_restore_on_loaded_reset() {
    let xml = SOURCE.replace("</collision></link>",
        "</collision><sensor name='imu' type='imu'><always_on>true</always_on><update_rate>1000</update_rate></sensor></link>");
    for engine in engines() {
        let mut sim = load(&xml, engine);
        let counts = sim.scheduled_system_counts();
        sim.step_exact(2).unwrap();
        let id = model_id(&sim, "default::robot");
        remove_model(&mut sim, id).unwrap();
        assert_eq!(sim.scheduled_system_counts(), (counts.0 - 3, counts.1 - 2));
        assert!(!sim.world().contains_resource::<Installed>());
        assert!(!sim.world().contains_resource::<imu_lifecycle::Installed>());
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(sim.scheduled_system_counts(), counts);
        assert!(sim.world().contains_resource::<Installed>());
        assert!(sim.world().contains_resource::<imu_lifecycle::Installed>());
        sim.step_exact(1).unwrap();
        let joint = sim
            .world_mut()
            .query::<&JointSample>()
            .single(sim.world())
            .unwrap();
        assert_eq!(joint.time, Duration::from_millis(1));
    }
}

#[test]
fn joint_retirement_builds_recheck_last_imu_removed_by_an_observer() {
    use bevy_ecs::schedule::ScheduleBuilt;
    for engine in engines() {
        let mut sim = load(&with_separate_imu_model(), engine);
        sim.initialize_schedules().unwrap();
        let sensor = sim
            .world()
            .iter_entities()
            .find(|entity| entity.contains::<Imu>())
            .unwrap()
            .id();
        sim.world_mut()
            .add_observer(move |_: On<ScheduleBuilt>, mut commands: Commands| {
                commands.entity(sensor).try_despawn();
            });
        let id = model_id(&sim, "default::robot");
        remove_model(&mut sim, id).unwrap();
        assert!(
            !sim.world()
                .iter_entities()
                .any(|entity| entity.contains::<Imu>())
        );
        assert!(!sim.world().contains_resource::<imu_lifecycle::Installed>());
        assert!(!sim.world().contains_resource::<Installed>());
        sim.step_exact(1).unwrap();
    }
}

#[test]
fn imu_retirement_builds_recheck_last_joint_removed_by_an_observer() {
    use bevy_ecs::schedule::ScheduleBuilt;
    for engine in engines() {
        let mut sim = load(&with_separate_imu_model(), engine);
        sim.initialize_schedules().unwrap();
        let joint = sim
            .world()
            .iter_entities()
            .find(|entity| entity.contains::<JointHandle>())
            .unwrap();
        let id = joint.get::<JointHandle>().unwrap().id();
        let joint_entity = joint.id();
        sim.world_mut().add_observer(
            move |_: On<ScheduleBuilt>,
                  mut commands: Commands,
                  mut backend: ResMut<PhysicsBackend>,
                  mut done: Local<bool>| {
                if !*done {
                    assert!(backend.remove_joint(id));
                    commands.entity(joint_entity).try_despawn();
                    *done = true;
                }
            },
        );
        let model = model_id(&sim, "default::ball");
        remove_model(&mut sim, model).unwrap();
        assert!(
            !sim.world()
                .iter_entities()
                .any(|entity| entity.contains::<JointHandle>())
        );
        assert!(!sim.world().contains_resource::<Installed>());
        assert!(!sim.world().contains_resource::<imu_lifecycle::Installed>());
        sim.step_exact(1).unwrap();
    }
}

#[test]
fn adding_consumers_during_retirement_reports_a_terminal_committed_fault() {
    use bevy_ecs::schedule::ScheduleBuilt;
    for engine in engines() {
        let mut sim = load(&with_separate_imu_model(), engine);
        sim.initialize_schedules().unwrap();
        let imu = sim
            .world()
            .iter_entities()
            .find_map(|entity| entity.get::<Imu>())
            .unwrap()
            .clone();
        sim.world_mut()
            .add_observer(move |_: On<ScheduleBuilt>, mut commands: Commands| {
                commands.spawn(imu.clone());
            });
        let id = model_id(&sim, "default::ball");
        assert!(matches!(
            remove_model(&mut sim, id),
            Err(RemoveError::CommittedSchedule(_))
        ));
        assert!(gz_sim_core::entity_by_id(sim.world(), id).is_none());
        assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
    }
}

#[test]
fn loaded_reset_extra_cleanup_retires_callbacks_after_observer_removal() {
    for engine in engines() {
        let mut sim = load(SOURCE, engine);
        let counts = sim.scheduled_system_counts();
        let extra = sim.world_mut().spawn_empty().id();
        sim.world_mut().add_observer(
            move |event: On<Remove<PendingResetRemoval>>,
                  joints: Query<(&JointHandle, &ChildOf)>,
                  mut commands: Commands,
                  mut backend: ResMut<PhysicsBackend>| {
                if event.entity == extra {
                    for (handle, parent) in &joints {
                        let body = backend.joint_child(handle.id()).unwrap();
                        assert!(backend.remove(body));
                        commands.entity(parent.parent()).despawn();
                    }
                }
            },
        );
        reset_loaded_world(&mut sim, |sim| {
            assert!(sim.world().contains_resource::<Installed>());
        })
        .unwrap();
        assert!(!sim.world().contains_resource::<Installed>());
        assert_eq!(sim.scheduled_system_counts(), (counts.0 - 2, counts.1 - 1));
        sim.step_exact(1).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert!(sim.world().contains_resource::<Installed>());
        assert_eq!(sim.scheduled_system_counts(), counts);
        sim.step_exact(1).unwrap();
    }
}
