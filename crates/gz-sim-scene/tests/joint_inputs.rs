// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::*;
use gz_physics::{PhysicsBackend, PhysicsError};
use gz_sim_core::{EntityId, Simulation, StepError, entity_by_id};
use gz_sim_scene::{
    BodyHandle, EntityKind, InputError, JointHandle, JointSample, Name, PhysicsEngine, WorldPose,
    load_world_with_sdf_step, queue_joint_effort, remove_model, reset_loaded_world,
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
#[test]
fn loaded_entity_effort_combines_and_survives_pause_then_consumes_once() {
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        let mut sim = load(engine);
        let joint = named(&sim, "default::rotor::hinge");
        let id = public(&sim, joint);
        queue_joint_effort(&mut sim, id, 2.0).unwrap();
        queue_joint_effort(&mut sim, id, -0.5).unwrap();
        sim.tick().unwrap();
        sim.tick().unwrap();
        close(rate(&sim, id), 0.0);
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.09375);
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.09375);
        queue_joint_effort(&mut sim, id, -3.0).unwrap();
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), -0.09375);
    }
}
#[test]
fn native_force_pulses_reach_loaded_joint_samples_on_every_dynamic_engine() {
    let native: serde_json::Value = serde_json::from_str(include_str!(
        "../../gz-physics-dart/tests/fixtures/joint_effort.json"
    ))
    .unwrap();
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        let mut sim = load(engine);
        let id = public(&sim, named(&sim, "default::rotor::hinge"));
        for row in native["observations"].as_array().unwrap() {
            queue_joint_effort(&mut sim, id, row["effort"].as_f64().unwrap()).unwrap();
            sim.step_exact(1).unwrap();
            close(rate(&sim, id), row["dq"].as_f64().unwrap());
        }
    }
}
#[test]
fn invalid_admission_preserves_an_existing_command() {
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        let mut sim = load(engine);
        let id = public(&sim, named(&sim, "default::rotor::hinge"));
        let link = public(&sim, named(&sim, "default::rotor::body"));
        queue_joint_effort(&mut sim, id, 2.0).unwrap();
        for value in [f64::NAN, f64::INFINITY, -f64::INFINITY] {
            assert!(matches!(
                queue_joint_effort(&mut sim, id, value),
                Err(InputError::InvalidValue)
            ));
        }
        assert!(matches!(
            queue_joint_effort(&mut sim, u64::MAX, 1.0),
            Err(InputError::MissingEntity(_))
        ));
        assert!(matches!(
            queue_joint_effort(&mut sim, link, 1.0),
            Err(InputError::NotJoint(_))
        ));
        if engine == PhysicsEngine::Dart {
            assert!(matches!(
                queue_joint_effort(&mut sim, id, 1e61),
                Err(InputError::InvalidValue)
            ));
        }
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.125);
    }
}
#[test]
fn loaded_reset_and_model_retirement_discard_queued_effort() {
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        let mut sim = load(engine);
        let id = public(&sim, named(&sim, "default::rotor::hinge"));
        queue_joint_effort(&mut sim, id, 2.0).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.0);
        queue_joint_effort(&mut sim, id, 2.0).unwrap();
        sim.reset().unwrap();
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.0);
        queue_joint_effort(&mut sim, id, 2.0).unwrap();
        let model = public(&sim, named(&sim, "default::rotor"));
        remove_model(&mut sim, model).unwrap();
        assert!(matches!(
            queue_joint_effort(&mut sim, id, 1.0),
            Err(InputError::MissingEntity(_))
        ));
        sim.step_exact(1).unwrap();
    }
}
#[test]
fn invalid_local_binding_cannot_admit_new_commands() {
    type Mutation = fn(&mut Simulation, Entity);
    let mutations: &[Mutation] = &[
        |s, j| {
            s.world_mut().entity_mut(j).remove::<JointHandle>();
        },
        |s, j| {
            s.world_mut().entity_mut(j).remove::<JointSample>();
        },
        |s, j| {
            s.world_mut().entity_mut(j).remove::<WorldPose>();
        },
        |s, j| {
            let model = s.world().get::<ChildOf>(j).unwrap().parent();
            s.world_mut().entity_mut(model).remove::<EntityKind>();
        },
        |s, j| {
            s.world_mut().entity_mut(j).remove::<ChildOf>();
        },
        |s, j| {
            let h = s.world().get::<JointHandle>(j).unwrap().id();
            assert!(
                s.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .remove_joint(h)
            );
        },
        |s, _| {
            s.world_mut().remove_resource::<PhysicsBackend>();
        },
        |s, j| {
            let child = s.world().get::<JointHandle>(j).unwrap().child();
            let child = entity_by_id(s.world(), child).unwrap();
            s.world_mut().entity_mut(child).remove::<BodyHandle>();
        },
    ];
    for mutate in mutations {
        let mut sim = load(PhysicsEngine::Rapier);
        let joint = named(&sim, "default::rotor::hinge");
        let id = public(&sim, joint);
        mutate(&mut sim, joint);
        assert!(matches!(
            queue_joint_effort(&mut sim, id, 1.0),
            Err(InputError::InvalidPhysics)
        ));
    }
}
#[test]
fn a_terminal_run_rejects_further_joint_admission() {
    let mut sim = load(PhysicsEngine::Rapier);
    let joint = named(&sim, "default::rotor::hinge");
    let id = public(&sim, joint);
    sim.world_mut().entity_mut(joint).remove::<JointSample>();
    assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
    assert!(matches!(
        queue_joint_effort(&mut sim, id, 1.0),
        Err(InputError::Clock(StepError::Faulted))
    ));
}

#[test]
fn unsupported_moving_parent_is_an_admission_error_not_a_run_fault() {
    let parent = "<link name='anchor'><inertial><mass>2</mass><inertia><ixx>0.032</ixx><iyy>0.032</iyy><izz>0.032</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision></link>";
    let xml = XML
        .replace("<link name='body'>", &format!("{parent}<link name='body'>"))
        .replace("<parent>world</parent>", "<parent>anchor</parent>");
    let mut sim = load_world_with_sdf_step(
        &xml,
        "moving-parent.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let id = public(&sim, named(&sim, "default::rotor::hinge"));
    assert!(matches!(
        queue_joint_effort(&mut sim, id, 1.0),
        Err(InputError::Physics(PhysicsError::Unsupported {
            engine: "rapier",
            ..
        }))
    ));
    assert!(sim.fault().is_none());
    sim.step_exact(1).unwrap();
    close(rate(&sim, id), 0.0);
}
#[test]
fn accumulated_numeric_overflow_can_be_rejected_and_cancelled_before_stepping() {
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        let mut sim = load(engine);
        let id = public(&sim, named(&sim, "default::rotor::hinge"));
        let value = if engine == PhysicsEngine::Dart {
            1e60
        } else {
            f64::MAX
        };
        queue_joint_effort(&mut sim, id, value).unwrap();
        assert!(matches!(
            queue_joint_effort(&mut sim, id, value),
            Err(InputError::InvalidValue)
        ));
        queue_joint_effort(&mut sim, id, -value).unwrap();
        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.0);
    }
}

#[test]
fn repairing_a_local_binding_keeps_only_the_previously_admitted_effort() {
    let mut sim = load(PhysicsEngine::Rapier);
    let joint = named(&sim, "default::rotor::hinge");
    let id = public(&sim, joint);
    let sample = *sim.world().get::<JointSample>(joint).unwrap();
    queue_joint_effort(&mut sim, id, 1.5).unwrap();
    sim.world_mut().entity_mut(joint).remove::<JointSample>();
    assert!(matches!(
        queue_joint_effort(&mut sim, id, 2.0),
        Err(InputError::InvalidPhysics)
    ));
    sim.world_mut().entity_mut(joint).insert(sample);
    sim.step_exact(1).unwrap();
    close(rate(&sim, id), 0.09375);
}
#[test]
fn a_foreign_backend_with_the_same_engine_name_cannot_receive_loaded_commands() {
    let mut sim = load(PhysicsEngine::Rapier);
    let id = public(&sim, named(&sim, "default::rotor::hinge"));
    queue_joint_effort(&mut sim, id, 1.5).unwrap();
    let original = sim.world_mut().remove_resource::<PhysicsBackend>().unwrap();
    sim.world_mut().insert_resource(PhysicsBackend::from(
        gz_physics_rapier::RapierBackend::new([0.0; 3]).unwrap(),
    ));
    assert!(matches!(
        queue_joint_effort(&mut sim, id, 2.0),
        Err(InputError::InvalidPhysics)
    ));
    sim.world_mut().insert_resource(original);
    sim.step_exact(1).unwrap();
    close(rate(&sim, id), 0.09375);
}

#[test]
fn copied_joint_handle_cannot_redirect_a_second_public_identity() {
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
    queue_joint_effort(&mut sim, first_id, 1.5).unwrap();
    sim.world_mut().entity_mut(second).insert(first_handle);
    assert!(matches!(
        queue_joint_effort(&mut sim, second_id, 2.0),
        Err(InputError::InvalidPhysics)
    ));
    sim.world_mut().entity_mut(second).insert(second_handle);
    sim.step_exact(1).unwrap();
    close(rate(&sim, first_id), 0.09375);
    close(rate(&sim, second_id), 0.0);
}
