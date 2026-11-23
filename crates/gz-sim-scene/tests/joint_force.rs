// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::*;
use gz_physics::DynamicBackend;
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

const ENGINES: [PhysicsEngine; 1] = [PhysicsEngine::Rapier];
fn moving_parent_states(sim: &Simulation) -> [gz_physics::BodyState; 2] {
    ["anchor", "body"].map(|link| {
        let entity = named(sim, &format!("default::rotor::{link}"));
        let handle = sim
            .world()
            .get::<gz_sim_scene::BodyHandle>(entity)
            .unwrap()
            .id();
        sim.world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .unwrap()
    })
}
fn moving_parent_native_twin() -> (
    gz_physics_rapier::RapierBackend,
    [gz_physics::BodyId; 2],
    gz_physics::JointId,
) {
    use gz_physics::{BodySpec, Inertia, RotationSpec, Shape};
    let mut native = gz_physics_rapier::RapierBackend::new([0.; 3]).unwrap();
    let bodies = [(2., 0.032), (1., 0.016)].map(|(mass, moment)| {
        let body = native
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.2 },
                    position: [0.; 3],
                    velocity: [0.; 3],
                    mass: Some(mass),
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    tensor: [moment, moment, moment, 0., 0., 0.],
                }),
            )
            .unwrap();
        native.set_self_collision_group(body, Some(7)).unwrap();
        body
    });
    let joint = native
        .add_revolute(bodies[0], bodies[1], [0.; 3], [0., 1., 0.])
        .unwrap();
    (native, bodies, joint)
}
fn check_moving_parent_response(sim: &Simulation, angular_impulse: f64) {
    let [parent, child] = moving_parent_states(sim);
    for (state, expected_y) in [
        (&parent, -angular_impulse / 0.032),
        (&child, angular_impulse / 0.016),
    ] {
        for (axis, expected) in [0., expected_y, 0.].into_iter().enumerate() {
            close(state.angular_velocity[axis], expected);
            close(state.velocity[axis], 0.);
            close(state.position[axis], 0.);
            close(
                0.032 * parent.angular_velocity[axis] + 0.016 * child.angular_velocity[axis],
                0.,
            );
        }
    }
    let joint = named(sim, "default::rotor::hinge");
    let sample = sim.world().get::<JointSample>(joint).unwrap();
    let handle = sim.world().get::<JointHandle>(joint).unwrap().id();
    assert_eq!(
        sample.state,
        sim.world()
            .resource::<PhysicsBackend>()
            .joint_state(handle)
            .unwrap()
    );
    assert_eq!((sample.time, sample.epoch), (sim.elapsed(), sim.epoch()));
    close(
        sample.state.velocity,
        child.angular_velocity[1] - parent.angular_velocity[1],
    );
    if angular_impulse > 0. {
        // Original positive unit torque has positive measured relative power;
        // spin momenta cancel because both COMs share the stationary pivot.
        assert!(child.angular_velocity[1] - parent.angular_velocity[1] > 0.);
        assert!(
            0.5 * (0.032 * parent.angular_velocity[1].powi(2)
                + 0.016 * child.angular_velocity[1].powi(2))
                > 0.
        );
    }
    for (link, state) in ["anchor", "body"].into_iter().zip([parent, child]) {
        for name in [
            format!("default::rotor::{link}"),
            format!("default::rotor::{link}::shape"),
        ] {
            let pose = sim
                .world()
                .get::<gz_sim_scene::WorldPose>(named(sim, &name))
                .unwrap();
            for (actual, expected) in pose.translation.into_iter().zip(state.position) {
                close(actual, expected);
            }
            for (actual, expected) in pose.rotation.into_iter().zip(state.orientation) {
                close(actual, expected);
            }
        }
    }
    assert!(sim.fault().is_none());
}
fn check_moving_parent_twin(
    sim: &Simulation,
    native: &gz_physics_rapier::RapierBackend,
    bodies: [gz_physics::BodyId; 2],
    joint: gz_physics::JointId,
) {
    for (scene, body) in moving_parent_states(sim).into_iter().zip(bodies) {
        let direct = native.state(body).unwrap();
        for (actual, expected) in scene
            .position
            .into_iter()
            .chain(scene.velocity)
            .chain(scene.angular_velocity)
            .chain(scene.orientation)
            .zip(
                direct
                    .position
                    .into_iter()
                    .chain(direct.velocity)
                    .chain(direct.angular_velocity)
                    .chain(direct.orientation),
            )
        {
            close(actual, expected);
        }
    }
    let scene_joint = sim
        .world()
        .get::<JointSample>(named(sim, "default::rotor::hinge"))
        .unwrap()
        .state;
    let direct_joint = native.joint_state(joint).unwrap();
    close(scene_joint.position, direct_joint.position);
    close(scene_joint.velocity, direct_joint.velocity);
}

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
        let limit = f64::MAX;
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
        let limit = f64::MAX;
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
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/reference/joint_effort.json")).unwrap();
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
    sim.initialize_schedules().unwrap();
    let baseline = sim.scheduled_system_counts();
    let before = moving_parent_states(&sim);
    let handle = sim.world().get::<JointHandle>(joint).unwrap().id();
    sim.world()
        .resource::<PhysicsBackend>()
        .validate_joint_effort(handle, 1.0)
        .unwrap();
    assert!(sim.world().get::<JointForceCommand>(joint).is_none());
    assert_eq!(sim.scheduled_system_counts(), baseline);
    assert_eq!(moving_parent_states(&sim), before);
    // M106 admits the same unit force as an equal/opposite native couple.
    set_joint_force(&mut sim, id, 1.0).unwrap();
    assert_eq!(
        sim.world()
            .get::<JointForceCommand>(joint)
            .unwrap()
            .torque(),
        1.0
    );
    assert_eq!(sim.scheduled_system_counts(), (baseline.0 + 1, baseline.1));
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(matches!(
            set_joint_force(&mut sim, id, invalid),
            Err(InputError::InvalidValue)
        ));
    }
    assert_eq!(
        sim.world()
            .get::<JointForceCommand>(joint)
            .unwrap()
            .torque(),
        1.0
    );
    assert_eq!(sim.scheduled_system_counts(), (baseline.0 + 1, baseline.1));
    sim.set_paused(true);
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(moving_parent_states(&sim), before);
    close(rate(&sim, id), 0.0);
    sim.set_paused(false);
    let (mut native, native_bodies, native_joint) = moving_parent_native_twin();
    native.add_joint_effort(native_joint, 1.0).unwrap();
    native.step(std::time::Duration::from_millis(1)).unwrap();
    sim.step_exact(1).unwrap();
    check_moving_parent_response(&sim, 0.001);
    check_moving_parent_twin(&sim, &native, native_bodies, native_joint);
    // Persistent unit torque and a .5 one-shot sum once, without reinstalling.
    queue_joint_effort(&mut sim, id, 0.5).unwrap();
    native.add_joint_effort(native_joint, 1.5).unwrap();
    native.step(std::time::Duration::from_millis(1)).unwrap();
    sim.step_exact(1).unwrap();
    check_moving_parent_response(&sim, 0.0025);
    check_moving_parent_twin(&sim, &native, native_bodies, native_joint);
    set_joint_force(&mut sim, id, 0.0).unwrap();
    native.step(std::time::Duration::from_millis(1)).unwrap();
    sim.step_exact(1).unwrap();
    check_moving_parent_response(&sim, 0.0025);
    check_moving_parent_twin(&sim, &native, native_bodies, native_joint);
    let mut quiet = load_world_with_sdf_step(
        &xml,
        "moving-force.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    quiet.step_exact(3).unwrap();
    check_moving_parent_response(&quiet, 0.0);
    assert_ne!(moving_parent_states(&sim), moving_parent_states(&quiet));
    for loaded in [false, true] {
        set_joint_force(&mut sim, id, 1.0).unwrap();
        queue_joint_effort(&mut sim, id, 0.5).unwrap();
        if loaded {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        } else {
            sim.reset().unwrap();
        }
        sim.step_exact(1).unwrap();
        check_moving_parent_response(&sim, 0.0);
    }
    let joint = named(&sim, "default::rotor::hinge");
    assert!(sim.world().get::<JointForceCommand>(joint).is_none());
    assert_eq!(sim.scheduled_system_counts(), baseline);
    set_joint_force(&mut sim, id, 1.0).unwrap();
    queue_joint_effort(&mut sim, id, 0.5).unwrap();
    let model = public(&sim, named(&sim, "default::rotor"));
    remove_model(&mut sim, model).unwrap();
    assert!(matches!(
        set_joint_force(&mut sim, id, 1.0),
        Err(InputError::MissingEntity(_))
    ));
    sim.step_exact(1).unwrap();
    assert!(sim.fault().is_none());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(public(&sim, named(&sim, "default::rotor::hinge")), id);
    sim.step_exact(1).unwrap();
    check_moving_parent_response(&sim, 0.0);
    set_joint_force(&mut sim, id, 1.0).unwrap();
    sim.step_exact(1).unwrap();
    check_moving_parent_response(&sim, 0.001);
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
