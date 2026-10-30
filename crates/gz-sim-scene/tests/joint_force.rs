// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::*;
use gz_physics::{JointId, PhysicsBackend, PhysicsError};
use gz_sim_core::{EntityId, Simulation, StepError, entity_by_id};
use gz_sim_scene::{
    InputError, JointForceCommand, JointHandle, JointSample, Name, PhysicsEngine,
    load_world_with_sdf_step, queue_joint_effort, remove_model, reset_loaded_world,
    set_joint_force,
};
use sdformat::NoResources;
const XML: &str = "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='rotor'><link name='body'><inertial><mass>1</mass><inertia><ixx>0.016</ixx><iyy>0.016</iyy><izz>0.016</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision></link><joint name='hinge' type='revolute'><parent>world</parent><child>body</child><axis><xyz>0 5 0</xyz></axis></joint></model></world></sdf>";
fn load(engine: PhysicsEngine) -> Simulation {
    load_world_with_sdf_step(XML, "joint-input.sdf", "default", &NoResources, engine).unwrap()
}
fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn public(sim: &Simulation, entity: Entity) -> u64 {
    sim.world().get::<EntityId>(entity).unwrap().get()
}
fn rate(sim: &Simulation, id: u64) -> f64 {
    sim.world()
        .get::<JointSample>(entity_by_id(sim.world(), id).unwrap())
        .unwrap()
        .state
        .velocity
}
fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
}

const ENGINES: [PhysicsEngine; 3] = [
    PhysicsEngine::Rapier,
    PhysicsEngine::Dart,
    PhysicsEngine::Mujoco,
];
#[test]
fn persistent_latest_force_combines_with_one_step_inputs_and_stops_at_zero() {
    for engine in ENGINES {
        let mut sim = load(engine);
        let joint = named(&sim, "default::rotor::hinge");
        let id = public(&sim, joint);
        set_joint_force(&mut sim, id, 3.0).unwrap();
        set_joint_force(&mut sim, id, 1.5).unwrap();
        assert_eq!(
            sim.world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            1.5
        );
        for _ in 0..10 {
            sim.tick().unwrap();
        }
        close(rate(&sim, id), 0.0);
        queue_joint_effort(&mut sim, id, 0.5).unwrap();
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.125);
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.21875);
        set_joint_force(&mut sim, id, 0.0).unwrap();
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.21875);
        set_joint_force(&mut sim, id, -1.5).unwrap();
        sim.step_exact(2).unwrap();
        close(rate(&sim, id), 0.03125);
    }
}
#[test]
fn rejected_replacement_preserves_latest_force_and_one_step_effort() {
    for engine in ENGINES {
        let mut sim = load(engine);
        let joint = named(&sim, "default::rotor::hinge");
        let id = public(&sim, joint);
        set_joint_force(&mut sim, id, 1.5).unwrap();
        queue_joint_effort(&mut sim, id, 2.0).unwrap();
        for value in [f64::NAN, f64::INFINITY, -f64::INFINITY] {
            assert!(matches!(
                set_joint_force(&mut sim, id, value),
                Err(InputError::InvalidValue)
            ));
        }
        if engine == PhysicsEngine::Dart {
            assert!(matches!(
                set_joint_force(&mut sim, id, 1e61),
                Err(InputError::InvalidValue)
            ));
        }
        assert!(matches!(
            set_joint_force(&mut sim, u64::MAX, 2.0),
            Err(InputError::MissingEntity(_))
        ));
        let link = public(&sim, named(&sim, "default::rotor::body"));
        assert!(matches!(
            set_joint_force(&mut sim, link, 2.0),
            Err(InputError::NotJoint(_))
        ));
        assert_eq!(
            sim.world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            1.5
        );
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.21875);
    }
}
#[test]
fn readonly_validation_does_not_change_queued_effort_or_install_a_controller() {
    for engine in ENGINES {
        let mut sim = load(engine);
        let joint = named(&sim, "default::rotor::hinge");
        let id = public(&sim, joint);
        queue_joint_effort(&mut sim, id, 1.5).unwrap();
        let handle = sim.world().get::<JointHandle>(joint).unwrap().id();
        let backend = sim.world().resource::<PhysicsBackend>();
        let limit = if engine == PhysicsEngine::Dart {
            1e60
        } else {
            f64::MAX
        };
        for torque in [limit, -limit, 0.0] {
            backend.validate_joint_effort(handle, torque).unwrap();
        }
        assert!(matches!(
            backend.validate_joint_effort(handle, f64::NAN),
            Err(PhysicsError::InvalidValue)
        ));
        assert!(matches!(
            backend.validate_joint_effort(JointId::from_raw(u64::MAX), 1.0),
            Err(PhysicsError::MissingJoint)
        ));
        assert!(sim.world().get::<JointForceCommand>(joint).is_none());
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.09375);
    }
}
#[test]
fn both_resets_clear_persistent_and_one_step_effort() {
    for engine in ENGINES {
        for loaded in [false, true] {
            let mut sim = load(engine);
            let id = public(&sim, named(&sim, "default::rotor::hinge"));
            set_joint_force(&mut sim, id, 1.5).unwrap();
            queue_joint_effort(&mut sim, id, 2.0).unwrap();
            if loaded {
                reset_loaded_world(&mut sim, |_| ()).unwrap();
            } else {
                sim.reset().unwrap();
            }
            sim.step_exact(2).unwrap();
            close(rate(&sim, id), 0.0);
            set_joint_force(&mut sim, id, 1.5).unwrap();
            sim.step_exact(1).unwrap();
            close(rate(&sim, id), 0.09375);
        }
    }
}
#[test]
fn retirement_removes_controllers_and_loaded_reset_restores_an_unforced_joint() {
    for engine in ENGINES {
        let mut sim = load(engine);
        let id = public(&sim, named(&sim, "default::rotor::hinge"));
        let model = public(&sim, named(&sim, "default::rotor"));
        set_joint_force(&mut sim, id, 1.5).unwrap();
        remove_model(&mut sim, model).unwrap();
        assert!(matches!(
            set_joint_force(&mut sim, id, 1.5),
            Err(InputError::MissingEntity(_))
        ));
        sim.step_exact(1).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(2).unwrap();
        close(rate(&sim, id), 0.0);
    }
}
#[test]
fn persistent_combination_failure_stops_the_run_before_native_stepping() {
    for engine in ENGINES {
        let mut sim = load(engine);
        let joint = named(&sim, "default::rotor::hinge");
        let id = public(&sim, joint);
        let handle = sim.world().get::<JointHandle>(joint).unwrap().id();
        let limit = if engine == PhysicsEngine::Dart {
            1e60
        } else {
            f64::MAX
        };
        queue_joint_effort(&mut sim, id, limit).unwrap();
        set_joint_force(&mut sim, id, limit).unwrap();
        assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
        assert!(matches!(
            sim.fault().unwrap().downcast_ref::<PhysicsError>(),
            Some(PhysicsError::InvalidValue)
        ));
        close(
            sim.world()
                .resource::<PhysicsBackend>()
                .joint_state(handle)
                .unwrap()
                .velocity,
            0.0,
        );
        assert!(matches!(
            set_joint_force(&mut sim, id, 0.0),
            Err(InputError::Clock(StepError::Faulted))
        ));
    }
}
#[test]
fn a_controller_without_a_joint_handle_faults_instead_of_disappearing() {
    let mut sim = load(PhysicsEngine::Rapier);
    let joint = named(&sim, "default::rotor::hinge");
    let id = public(&sim, joint);
    set_joint_force(&mut sim, id, 1.5).unwrap();
    let command = *sim.world().get::<JointForceCommand>(joint).unwrap();
    let model = named(&sim, "default::rotor");
    sim.world_mut().entity_mut(model).insert(command);
    assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
    assert!(matches!(
        sim.fault().unwrap().downcast_ref::<InputError>(),
        Some(InputError::InvalidPhysics)
    ));
}

#[test]
fn persistent_replacements_match_the_native_six_pulse_rate_fixture() {
    let fixture: serde_json::Value = serde_json::from_str(include_str!(
        "../../gz-physics-dart/tests/fixtures/joint_effort.json"
    ))
    .unwrap();
    for engine in ENGINES {
        let mut sim = load(engine);
        let id = public(&sim, named(&sim, "default::rotor::hinge"));
        for row in fixture["observations"].as_array().unwrap() {
            set_joint_force(&mut sim, id, row["effort"].as_f64().unwrap()).unwrap();
            sim.step_exact(1).unwrap();
            close(rate(&sim, id), row["dq"].as_f64().unwrap());
        }
    }
}
#[test]
fn moving_parent_force_is_rejected_without_installing_a_controller() {
    let parent = "<link name='anchor'><inertial><mass>2</mass><inertia><ixx>0.032</ixx><iyy>0.032</iyy><izz>0.032</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision></link>";
    let xml = XML
        .replace("<link name='body'>", &format!("{parent}<link name='body'>"))
        .replace("<parent>world</parent>", "<parent>anchor</parent>");
    let mut sim = load_world_with_sdf_step(
        &xml,
        "moving-force.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let joint = named(&sim, "default::rotor::hinge");
    let id = public(&sim, joint);
    assert!(matches!(
        set_joint_force(&mut sim, id, 1.0),
        Err(InputError::Physics(PhysicsError::Unsupported {
            engine: "rapier",
            ..
        }))
    ));
    assert!(sim.world().get::<JointForceCommand>(joint).is_none());
    sim.step_exact(1).unwrap();
    close(rate(&sim, id), 0.0);
}

#[test]
fn swapped_joint_handles_cannot_redirect_persistent_force_after_admission() {
    for swap_identity in [false, true] {
        let link = XML
            .split_once("<link name='body'>")
            .unwrap()
            .1
            .split_once("</link>")
            .unwrap()
            .0;
        let extra = format!(
            "<link name='second_body'><pose>2 0 0 0 0 0</pose>{link}</link><joint name='second_hinge' type='revolute'><parent>world</parent><child>second_body</child><axis><xyz>0 5 0</xyz></axis></joint>"
        );
        let xml = XML.replace("</model>", &format!("{extra}</model>"));
        let mut sim = load_world_with_sdf_step(
            &xml,
            "alias.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let first = named(&sim, "default::rotor::hinge");
        let second = named(&sim, "default::rotor::second_hinge");
        let first_id = public(&sim, first);
        let second_id = public(&sim, second);
        let first_handle = *sim.world().get::<JointHandle>(first).unwrap();
        let second_handle = *sim.world().get::<JointHandle>(second).unwrap();
        let bodies: Vec<_> = ["default::rotor::body", "default::rotor::second_body"]
            .map(|name| {
                sim.world()
                    .get::<gz_sim_scene::BodyHandle>(named(&sim, name))
                    .unwrap()
                    .id()
            })
            .into_iter()
            .collect();
        let before: Vec<_> = bodies
            .iter()
            .map(|id| sim.world().resource::<PhysicsBackend>().state(*id).unwrap())
            .collect();
        set_joint_force(&mut sim, first_id, 1.5).unwrap();
        set_joint_force(&mut sim, second_id, -0.5).unwrap();
        sim.world_mut().entity_mut(first).insert(second_handle);
        sim.world_mut().entity_mut(second).insert(first_handle);
        if swap_identity {
            let first_identity = sim
                .world_mut()
                .entity_mut(first)
                .take::<EntityId>()
                .unwrap();
            let second_identity = sim
                .world_mut()
                .entity_mut(second)
                .take::<EntityId>()
                .unwrap();
            sim.world_mut().entity_mut(first).insert(second_identity);
            sim.world_mut().entity_mut(second).insert(first_identity);
        }
        assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
        for (id, expected) in bodies.into_iter().zip(before) {
            assert_eq!(
                sim.world().resource::<PhysicsBackend>().state(id).unwrap(),
                expected
            );
        }
        assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
    }
}
