// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::entity_by_id;
use sdformat::NoResources;

const SOURCE: &str = include_str!("../fixtures/fall.sdf");
fn load(source: &str, engine: PhysicsEngine) -> Simulation {
    load_world_with_engine(
        source,
        "imu-demand.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        engine,
    )
    .unwrap()
}
fn engines() -> [PhysicsEngine; 4] {
    [
        PhysicsEngine::Tpe,
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ]
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (entity.id(), entity.get::<EntityId>().unwrap().get())
}
fn without_imu() -> String {
    let start = SOURCE.find("<sensor ").unwrap();
    let end = SOURCE[start..].find("</sensor>").unwrap() + start + "</sensor>".len();
    format!("{}{}", &SOURCE[..start], &SOURCE[end..])
}
fn twice() -> String {
    let start = SOURCE.find("<model name=\"ball\"").unwrap();
    let end = SOURCE[start..].find("</model>").unwrap() + start + "</model>".len();
    let model = SOURCE[start..end].replacen("name=\"ball\"", "name=\"other\"", 1);
    SOURCE.replace("</world>", &format!("{model}</world>"))
}

#[test]
fn absent_imus_own_no_sampling_callbacks_or_interest_state() {
    for engine in engines() {
        let mut idle = load(&without_imu(), engine);
        let live = load(SOURCE, engine);
        let counts = idle.scheduled_system_counts();
        assert_eq!(live.scheduled_system_counts(), (counts.0 + 1, counts.1 + 1));
        assert!(!idle.world().contains_resource::<Installed>());
        set_imu_interest(&mut idle, []);
        assert!(!idle.world().contains_resource::<ImuInterest>());
        idle.step_exact(3).unwrap();
        idle.reset().unwrap();
        assert!(!idle.world().contains_resource::<Installed>());
        assert!(!idle.world().contains_resource::<ImuInterest>());
        assert_eq!(idle.scheduled_system_counts(), counts);
    }
}

#[test]
fn only_last_consumer_retires_callbacks_and_loaded_reset_restores_them() {
    for engine in engines() {
        let mut sim = load(&twice(), engine);
        let (first, first_id) = named(&sim, "default::ball::body::imu");
        let (second, second_id) = named(&sim, "default::other::body::imu");
        let first_model = named(&sim, "default::ball").1;
        let second_model = named(&sim, "default::other").1;
        set_imu_interest(&mut sim, [first, second]);
        sim.step_exact(3).unwrap();
        let counts = sim.scheduled_system_counts();
        assert!(sim.world().resource::<ImuInterest>().0.capacity() >= 2);
        remove_model(&mut sim, first_model).unwrap();
        assert!(sim.world().contains_resource::<Installed>());
        assert_eq!(sim.scheduled_system_counts(), counts);
        sim.step_exact(1).unwrap();
        assert_eq!(
            sim.world().get::<ImuSample>(second).unwrap().time,
            Duration::from_millis(4)
        );
        remove_model(&mut sim, second_model).unwrap();
        assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
        assert!(!sim.world().contains_resource::<Installed>());
        assert!(!sim.world().contains_resource::<ImuInterest>());
        set_imu_interest(&mut sim, []);
        assert!(!sim.world().contains_resource::<ImuInterest>());
        sim.step_exact(2).unwrap();
        initial::reset_loaded_world(&mut sim, |sim| {
            assert!(sim.world().contains_resource::<Installed>());
            assert_eq!(sim.scheduled_system_counts(), counts);
            for id in [first_id, second_id] {
                let entity = entity_by_id(sim.world(), id).unwrap();
                assert!(sim.world().get::<ImuSample>(entity).is_none());
            }
        })
        .unwrap();
        sim.step_exact(1).unwrap();
        for id in [first_id, second_id] {
            let entity = entity_by_id(sim.world(), id).unwrap();
            let sample = sim.world().get::<ImuSample>(entity).unwrap();
            assert_eq!(sample.time, Duration::from_millis(1));
            assert_eq!(sample.epoch, 1);
        }
        for epoch in 2..=3 {
            remove_model(&mut sim, first_model).unwrap();
            remove_model(&mut sim, second_model).unwrap();
            assert!(!sim.world().contains_resource::<Installed>());
            initial::reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(sim.scheduled_system_counts(), counts);
            sim.step_exact(1).unwrap();
            for id in [first_id, second_id] {
                let entity = entity_by_id(sim.world(), id).unwrap();
                let sample = sim.world().get::<ImuSample>(entity).unwrap();
                assert_eq!(sample.epoch, epoch);
                assert_eq!(sample.time, Duration::from_millis(1));
            }
        }
    }
}

#[test]
fn inactive_unconnected_sensors_keep_continuous_history_and_demand() {
    for engine in engines() {
        let mut sim = load(SOURCE, engine);
        let (sensor, id) = named(&sim, "default::ball::body::imu");
        let body = named(&sim, "default::ball::body").0;
        let counts = sim.scheduled_system_counts();
        set_imu_active(&mut sim, id, false).unwrap();
        set_imu_interest(&mut sim, []);
        for _ in 0..5 {
            sim.step_exact(1).unwrap();
            assert_eq!(
                sim.world().get::<Imu>(sensor).unwrap().previous_velocity,
                DVec3::from_array(sim.world().get::<Velocity>(body).unwrap().linear)
            );
        }
        assert_eq!(sim.scheduled_system_counts(), counts);
        assert!(sim.world().contains_resource::<Installed>());
        assert!(sim.world().get::<ImuSample>(sensor).is_none());
        set_imu_active(&mut sim, id, true).unwrap();
        set_imu_interest(&mut sim, [sensor]);
        sim.step_exact(1).unwrap();
        assert_eq!(
            sim.world().get::<ImuSample>(sensor).unwrap().time,
            Duration::from_millis(6)
        );
    }
}

#[test]
fn invalid_reset_schedule_rejects_retirement_before_model_or_physics_removal() {
    fn a() {}
    fn b() {}
    for engine in engines() {
        let mut sim = load(&twice(), engine);
        let (sensor, _) = named(&sim, "default::ball::body::imu");
        let (model, id) = named(&sim, "default::ball");
        sim.step_exact(1).unwrap();
        sim.add_systems(Phase::Reset, (a.before(b), b.before(a)));
        let other = named(&sim, "default::other").1;
        // A remaining sensor still needs these callbacks; no retirement/build
        // is required until the final consumer is removed.
        remove_model(&mut sim, other).unwrap();
        assert!(matches!(
            remove_model(&mut sim, id),
            Err(RemoveError::Schedule(_))
        ));
        assert_eq!(entity_by_id(sim.world(), id), Some(model));
        assert!(sim.world().contains_resource::<Installed>());
        sim.step_exact(1).unwrap();
        assert_eq!(
            sim.world().get::<ImuSample>(sensor).unwrap().time,
            Duration::from_millis(2)
        );
    }
}

#[test]
fn schedule_observers_cannot_invalidate_final_removal_preflight() {
    use bevy_ecs::schedule::ScheduleBuilt;
    use gz_physics::{BodyId, PhysicsBackend};
    #[derive(Resource)]
    struct Observe {
        model: Entity,
        body: BodyId,
        seen: Option<(bool, bool)>,
    }
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        for initialized in [false, true] {
            let mut sim = load(SOURCE, engine);
            if initialized {
                sim.initialize_schedules().unwrap();
            }
            let (model, model_id) = named(&sim, "default::ball");
            let body = sim
                .world()
                .get::<BodyHandle>(named(&sim, "default::ball::body").0)
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
                        let model_present = models.get(observe.model).is_ok();
                        let removed_body = backend.remove(observe.body);
                        observe.seen = Some((model_present, removed_body));
                    }
                },
            );
            let result = remove_model(&mut sim, model_id);
            if initialized {
                assert!(result.is_ok());
                assert_eq!(sim.world().resource::<Observe>().seen, Some((false, false)));
                assert!(entity_by_id(sim.world(), model_id).is_none());
                assert!(!sim.world().contains_resource::<Installed>());
            } else {
                assert_eq!(result, Err(RemoveError::InvalidPhysics));
                assert_eq!(sim.world().resource::<Observe>().seen, Some((true, true)));
                assert_eq!(entity_by_id(sim.world(), model_id), Some(model));
                assert!(sim.world().contains_resource::<Installed>());
            }
        }
    }
}

#[test]
fn observer_removing_the_other_sensor_reports_a_committed_schedule_failure() {
    fn a() {}
    fn b() {}
    for engine in engines() {
        let mut sim = load(&twice(), engine);
        let (first_sensor, _) = named(&sim, "default::ball::body::imu");
        let (other_sensor, _) = named(&sim, "default::other::body::imu");
        let model_id = named(&sim, "default::ball").1;
        sim.step_exact(1).unwrap();
        sim.add_systems(Phase::Reset, (a.before(b), b.before(a)));
        sim.world_mut().add_observer(
            move |event: On<Remove<ImuPublication>>, mut commands: Commands| {
                if event.entity == first_sensor {
                    commands.entity(other_sensor).despawn();
                }
            },
        );
        let result = remove_model(&mut sim, model_id);
        assert!(
            matches!(result, Err(RemoveError::CommittedSchedule(_))),
            "{engine:?}: {result:?}"
        );
        assert!(entity_by_id(sim.world(), model_id).is_none());
        assert!(sim.world().get_entity(other_sensor).is_err());
        assert_eq!(sim.step_exact(1), Err(gz_sim_core::StepError::Faulted));
    }
}

#[test]
fn sensorless_model_removal_does_not_initialize_unneeded_reset_work() {
    fn a() {}
    fn b() {}
    for engine in engines() {
        let mut sim = load(&without_imu(), engine);
        let id = named(&sim, "default::ball").1;
        sim.add_systems(Phase::Reset, (a.before(b), b.before(a)));
        assert!(remove_model(&mut sim, id).is_ok());
        assert!(entity_by_id(sim.world(), id).is_none());
        assert!(!sim.world().contains_resource::<Installed>());
        assert!(!sim.world().contains_resource::<ImuInterest>());
        sim.step_exact(1).unwrap();
    }
}
