// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::*;
use gz_physics::DynamicBackend;
use gz_physics::PhysicsBackend;
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
fn loaded_entity_effort_combines_and_survives_pause_then_consumes_once() {
    {
        let engine = PhysicsEngine::Rapier;
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
    let native: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/reference/joint_effort.json")).unwrap();
    {
        let engine = PhysicsEngine::Rapier;
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
    {
        let engine = PhysicsEngine::Rapier;
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

        sim.step_exact(1).unwrap();
        close(rate(&sim, id), 0.125);
    }
}
#[test]
fn loaded_reset_and_model_retirement_discard_queued_effort() {
    {
        let engine = PhysicsEngine::Rapier;
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
    sim.initialize_schedules().unwrap();
    let baseline = sim.scheduled_system_counts();
    let before = moving_parent_states(&sim);
    // M106 makes the original unit effort a one-shot internal couple. Queuing
    // remains distinct from demand-installing a persistent force callback.
    queue_joint_effort(&mut sim, id, 1.0).unwrap();
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(matches!(
            queue_joint_effort(&mut sim, id, invalid),
            Err(InputError::InvalidValue)
        ));
    }
    assert_eq!(sim.scheduled_system_counts(), baseline);
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<gz_sim_scene::JointForceCommand>())
    );
    sim.set_paused(true);
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(moving_parent_states(&sim), before);
    assert!(sim.fault().is_none());
    close(rate(&sim, id), 0.0);
    sim.set_paused(false);
    let (mut native, native_bodies, native_joint) = moving_parent_native_twin();
    native.add_joint_effort(native_joint, 1.0).unwrap();
    native.step(std::time::Duration::from_millis(1)).unwrap();
    sim.step_exact(1).unwrap();
    check_moving_parent_response(&sim, 0.001);
    check_moving_parent_twin(&sim, &native, native_bodies, native_joint);
    native.step(std::time::Duration::from_millis(1)).unwrap();
    sim.step_exact(1).unwrap();
    check_moving_parent_response(&sim, 0.001);
    check_moving_parent_twin(&sim, &native, native_bodies, native_joint);
    // A later equal reverse impulse cancels momentum; the first was consumed.
    queue_joint_effort(&mut sim, id, -1.0).unwrap();
    native.add_joint_effort(native_joint, -1.0).unwrap();
    native.step(std::time::Duration::from_millis(1)).unwrap();
    sim.step_exact(1).unwrap();
    check_moving_parent_response(&sim, 0.0);
    check_moving_parent_twin(&sim, &native, native_bodies, native_joint);
    assert_eq!(sim.scheduled_system_counts(), baseline);
    let mut quiet = load_world_with_sdf_step(
        &xml,
        "moving-parent.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    quiet.step_exact(2).unwrap();
    check_moving_parent_response(&quiet, 0.0);
    // Compare the positive response again before resetting, not a requested rate.
    queue_joint_effort(&mut sim, id, 1.0).unwrap();
    sim.step_exact(1).unwrap();
    check_moving_parent_response(&sim, 0.001);
    assert_ne!(moving_parent_states(&sim), moving_parent_states(&quiet));
    for loaded in [false, true] {
        queue_joint_effort(&mut sim, id, 1.0).unwrap();
        if loaded {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        } else {
            sim.reset().unwrap();
        }
        sim.step_exact(1).unwrap();
        check_moving_parent_response(&sim, 0.0);
        assert_eq!(sim.scheduled_system_counts(), baseline);
    }
    queue_joint_effort(&mut sim, id, 1.0).unwrap();
    let model = public(&sim, named(&sim, "default::rotor"));
    remove_model(&mut sim, model).unwrap();
    assert!(matches!(
        queue_joint_effort(&mut sim, id, 1.0),
        Err(InputError::MissingEntity(_))
    ));
    sim.step_exact(1).unwrap();
    assert!(sim.fault().is_none());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(public(&sim, named(&sim, "default::rotor::hinge")), id);
    sim.step_exact(1).unwrap();
    check_moving_parent_response(&sim, 0.0);
    queue_joint_effort(&mut sim, id, 1.0).unwrap();
    sim.step_exact(1).unwrap();
    check_moving_parent_response(&sim, 0.001);
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<gz_sim_scene::JointForceCommand>())
    );
}
#[test]
fn accumulated_numeric_overflow_can_be_rejected_and_cancelled_before_stepping() {
    {
        let engine = PhysicsEngine::Rapier;
        let mut sim = load(engine);
        let id = public(&sim, named(&sim, "default::rotor::hinge"));
        let value = f64::MAX;
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
