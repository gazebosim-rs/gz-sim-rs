// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use sdformat::NoResources;

const SOURCE: &str = include_str!("../fixtures/pendulum.sdf");
const CONFIGURATION: &str = "<plugin filename='gz-sim-apply-joint-force-system' name='gz::sim::systems::ApplyJointForce'><joint_name>hinge</joint_name></plugin>";

fn engines() -> [PhysicsEngine; 3] {
    [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ]
}
fn load(xml: &str, engine: PhysicsEngine) -> Simulation {
    load_world_with_engine(
        xml,
        "force-demand.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        engine,
    )
    .unwrap()
}
fn configured() -> String {
    SOURCE.replace("</model>", &format!("{CONFIGURATION}</model>"))
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap();
    (entity.id(), entity.get::<EntityId>().unwrap().get())
}

fn with_unforced_joint_model() -> String {
    let source = configured();
    let begin = SOURCE.find("<model ").unwrap();
    let end = SOURCE[begin..].find("</model>").unwrap() + begin + "</model>".len();
    let other = SOURCE[begin..end]
        .replace("name='robot'", "name='other'")
        .replace("<pose>0 0 0 0 0.02 0</pose>", "<pose>3 0 0 0 0.02 0</pose>");
    source.replace("</world>", &format!("{other}</world>"))
}

#[test]
fn no_controller_owns_no_persistent_force_callback() {
    for engine in engines() {
        let mut idle = load(SOURCE, engine);
        let live = load(&configured(), engine);
        let counts = idle.scheduled_system_counts();
        assert_eq!(live.scheduled_system_counts(), (counts.0 + 1, counts.1));
        assert!(!idle.world().contains_resource::<Installed>());
        install(&mut idle);
        idle.step_exact(2).unwrap();
        idle.reset().unwrap();
        reset_loaded_world(&mut idle, |_| ()).unwrap();
        assert!(!idle.world().contains_resource::<Installed>());
        assert_eq!(idle.scheduled_system_counts(), counts);
    }
}

#[test]
fn typed_commands_activate_once_and_loaded_reset_removes_programmatic_demand() {
    for engine in engines() {
        let mut sim = load(SOURCE, engine);
        let counts = sim.scheduled_system_counts();
        let (joint, id) = named(&sim, "default::robot::hinge");
        assert!(matches!(
            set_joint_force(&mut sim, id, f64::NAN),
            Err(InputError::InvalidValue)
        ));
        assert!(!sim.world().contains_resource::<Installed>());
        assert_eq!(sim.scheduled_system_counts(), counts);
        set_joint_force(&mut sim, id, 1.5).unwrap();
        set_joint_force(&mut sim, id, 2.0).unwrap();
        assert!(sim.world().contains_resource::<Installed>());
        assert_eq!(sim.scheduled_system_counts(), (counts.0 + 1, counts.1));
        assert_eq!(
            sim.world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            2.0
        );
        sim.step_exact(2).unwrap();
        sim.reset().unwrap();
        assert_eq!(
            sim.world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            0.0
        );
        assert!(sim.world().contains_resource::<Installed>());
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert!(!sim.world().contains_resource::<Installed>());
        assert_eq!(sim.scheduled_system_counts(), counts);
        sim.step_exact(1).unwrap();
    }
}

#[test]
fn removing_a_configured_model_retires_and_loaded_reset_reinstalls_its_controller() {
    for engine in engines() {
        let mut sim = load(&configured(), engine);
        let counts = sim.scheduled_system_counts();
        let joint = named(&sim, "default::robot::hinge").1;
        set_joint_force(&mut sim, joint, 1.0).unwrap();
        sim.step_exact(2).unwrap();
        let model = named(&sim, "default::robot").1;
        remove_model(&mut sim, model).unwrap();
        assert!(!sim.world().contains_resource::<Installed>());
        assert!(
            !sim.world()
                .contains_resource::<joint_lifecycle::Installed>()
        );
        assert_eq!(sim.scheduled_system_counts(), (counts.0 - 3, counts.1 - 1));
        retire_if_unused(&mut sim).unwrap();
        sim.step_exact(1).unwrap();
        reset_loaded_world(&mut sim, |sim| {
            assert!(sim.world().contains_resource::<Installed>());
            assert_eq!(sim.scheduled_system_counts(), counts);
            let (joint, _) = named(sim, "default::robot::hinge");
            assert_eq!(
                sim.world()
                    .get::<JointForceCommand>(joint)
                    .unwrap()
                    .torque(),
                0.0
            );
        })
        .unwrap();
        sim.step_exact(1).unwrap();
    }
}

#[test]
fn last_controller_retires_without_removing_other_loaded_joints() {
    let configured = configured();
    let begin = configured.find("<model ").unwrap();
    let end = configured[begin..].find("</model>").unwrap() + begin + "</model>".len();
    let second = configured[begin..end]
        .replace("name='robot'", "name='other'")
        .replace("<pose>0 0 0 0 0.02 0</pose>", "<pose>3 0 0 0 0.02 0</pose>");
    let third = second
        .replace("name='other'", "name='unforced'")
        .replace(CONFIGURATION, "")
        .replace("<pose>3 0 0 0 0.02 0</pose>", "<pose>6 0 0 0 0.02 0</pose>");
    let xml = configured.replace("</world>", &format!("{second}{third}</world>"));
    for engine in engines() {
        let mut sim = load(&xml, engine);
        sim.step_exact(1).unwrap();
        let counts = sim.scheduled_system_counts();
        let first = named(&sim, "default::robot").1;
        remove_model(&mut sim, first).unwrap();
        assert!(sim.world().contains_resource::<Installed>());
        assert_eq!(sim.scheduled_system_counts(), counts);
        let second = named(&sim, "default::other").1;
        remove_model(&mut sim, second).unwrap();
        assert!(!sim.world().contains_resource::<Installed>());
        assert!(
            sim.world()
                .contains_resource::<joint_lifecycle::Installed>()
        );
        assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1));
        sim.step_exact(1).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert!(sim.world().contains_resource::<Installed>());
        assert_eq!(sim.scheduled_system_counts(), counts);
    }
}

#[test]
fn controller_retirement_builds_recheck_the_last_imu() {
    use bevy_ecs::schedule::ScheduleBuilt;
    let fall = include_str!("../fixtures/fall.sdf");
    let begin = fall.find("<model name=\"ball\"").unwrap();
    let end = fall[begin..].find("</model>").unwrap() + begin + "</model>".len();
    let xml =
        with_unforced_joint_model().replace("</world>", &format!("{}</world>", &fall[begin..end]));
    for engine in engines() {
        let mut sim = load(&xml, engine);
        sim.initialize_schedules().unwrap();
        let counts = sim.scheduled_system_counts();
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
        let model = named(&sim, "default::robot").1;
        remove_model(&mut sim, model).unwrap();
        assert!(!sim.world().contains_resource::<Installed>());
        assert!(!sim.world().contains_resource::<imu_lifecycle::Installed>());
        assert!(
            sim.world()
                .contains_resource::<joint_lifecycle::Installed>()
        );
        assert_eq!(sim.scheduled_system_counts(), (counts.0 - 2, counts.1 - 1));
        sim.step_exact(1).unwrap();
    }
}

#[test]
fn raw_controller_addition_during_retirement_is_a_terminal_committed_fault() {
    use bevy_ecs::schedule::ScheduleBuilt;
    for engine in engines() {
        let mut sim = load(&with_unforced_joint_model(), engine);
        sim.initialize_schedules().unwrap();
        let joint = named(&sim, "default::other::hinge").0;
        sim.world_mut()
            .add_observer(move |_: On<ScheduleBuilt>, mut commands: Commands| {
                commands.entity(joint).insert(JointForceCommand::default());
            });
        let model = named(&sim, "default::robot").1;
        assert!(matches!(
            remove_model(&mut sim, model),
            Err(RemoveError::CommittedSchedule(_))
        ));
        assert!(gz_sim_core::entity_by_id(sim.world(), model).is_none());
        assert!(
            sim.world()
                .resource::<StepFault>()
                .error()
                .unwrap()
                .to_string()
                .contains("schedule observer added consumers")
        );
        assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
    }
}
