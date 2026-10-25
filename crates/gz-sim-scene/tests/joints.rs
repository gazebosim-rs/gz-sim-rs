// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use gz_physics_rapier::{BodySpec, RapierBackend, Shape};
use gz_sim_core::{EntityId, Simulation, StepError, entity_by_id};
use gz_sim_scene::{
    BodyHandle, EntityKind, InputError, JointDefinition, JointHandle, JointSample, LoadError, Name,
    PhysicsEngine, WorldPose, load_world_with_sdf_step, queue_link_world_velocity,
    queue_link_world_wrench, remove_model, reset_loaded_world,
};
use sdformat::NoResources;
const XML: &str = include_str!("fixtures/pendulum.sdf");
fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(xml, "pendulum.sdf", "default", &NoResources, engine)
}
fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
#[test]
fn sdf_world_revolute_preserves_metadata_frames_native_observations_and_loaded_lifecycle() {
    let mut sim = load(XML, PhysicsEngine::Rapier).unwrap();
    let joint = named(&sim, "default::robot::hinge");
    let child = named(&sim, "default::robot::body");
    let model = named(&sim, "default::robot");
    assert_eq!(
        sim.world().get::<EntityKind>(joint),
        Some(&EntityKind::Joint)
    );
    let joint_id = sim.world().get::<EntityId>(joint).unwrap().get();
    let model_id = sim.world().get::<EntityId>(model).unwrap().get();
    let child_id = sim.world().get::<EntityId>(child).unwrap().get();
    let handle = *sim.world().get::<JointHandle>(joint).unwrap();
    assert_eq!(handle.child(), child_id);
    assert_eq!(
        sim.world()
            .resource::<RapierBackend>()
            .joint_child(handle.id()),
        Some(sim.world().get::<BodyHandle>(child).unwrap().id())
    );
    let definition = &sim.world().get::<JointDefinition>(joint).unwrap().0;
    assert_eq!(definition.name, "hinge");
    assert_eq!(definition.parent, "world");
    assert_eq!(definition.axis.as_ref().unwrap().upper, f64::INFINITY);
    assert!(matches!(
        queue_link_world_velocity(&mut sim, child_id, [0.0; 3], [0.0; 3]),
        Err(InputError::Unsupported("jointed-body input"))
    ));
    assert!(matches!(
        queue_link_world_wrench(&mut sim, child_id, [1.0; 3], [0.0; 3]),
        Err(InputError::Unsupported("jointed-body input"))
    ));
    let rows: Vec<serde_json::Value> = serde_json::from_str(include_str!(
        "../../gz-physics-rapier/tests/fixtures/world_pendulum.json"
    ))
    .unwrap();
    for row in rows.iter().filter(|r| r["epoch"] == 0) {
        let tick = row["iteration"].as_u64().unwrap();
        if tick > (sim.elapsed().as_millis() as u64) {
            sim.step_exact(tick - (sim.elapsed().as_millis() as u64))
                .unwrap();
        }
        let sample = sim.world().get::<JointSample>(joint).unwrap();
        assert_eq!(
            sample.time.as_nanos(),
            row["time_ns"].as_u64().unwrap() as u128
        );
        assert_eq!(sample.epoch, 0);
        assert!((sample.state.position - row["q"].as_f64().unwrap()).abs() < 1e-4);
        assert!((sample.state.velocity - row["dq"].as_f64().unwrap()).abs() < 3e-4);
        let pose = sim.world().get::<WorldPose>(child).unwrap();
        assert!((pose.translation[0] - row["x"].as_f64().unwrap()).abs() < 1e-4);
        let pivot = sim.world().get::<WorldPose>(joint).unwrap();
        assert!(pivot.translation.into_iter().all(|v| v.abs() < 1e-5));
    }
    let sample = sim
        .world()
        .get::<JointSample>(joint)
        .unwrap()
        .state
        .position;
    sim.tick().unwrap();
    assert_eq!(
        sim.world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .position,
        sample
    );
    sim.world_mut()
        .get_mut::<JointDefinition>(joint)
        .unwrap()
        .0
        .name = "edited".into();
    let retired = remove_model(&mut sim, model_id).unwrap();
    assert!(retired.contains(&joint_id));
    assert!(
        sim.world()
            .resource::<RapierBackend>()
            .joint_state(handle.id())
            .is_none()
    );
    sim.step_exact(1).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = entity_by_id(sim.world(), joint_id).unwrap();
    assert_ne!(restored, joint);
    assert_eq!(
        sim.world().get::<JointDefinition>(restored).unwrap().0.name,
        "hinge"
    );
    let sample = sim.world().get::<JointSample>(restored).unwrap();
    assert_eq!(sample.state.position, 0.0);
    assert_eq!(sample.time.as_nanos(), 0);
    assert_eq!(sample.epoch, 1);
    sim.step_exact(10).unwrap();
    let expected = rows
        .iter()
        .find(|r| r["epoch"] == 1 && r["iteration"] == 10)
        .unwrap();
    assert!(
        (sim.world()
            .get::<JointSample>(restored)
            .unwrap()
            .state
            .position
            - expected["q"].as_f64().unwrap())
        .abs()
            < 1e-4
    );
}

#[test]
fn unsupported_joint_effects_and_topologies_are_explicit() {
    assert!(
        load(XML, PhysicsEngine::Tpe)
            .err()
            .expect("unsupported TPE joint")
            .to_string()
            .contains("TPE joints")
    );
    assert!(matches!(
        load(
            &XML.replace("<parent>world</parent>", "<parent>body</parent>"),
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Sdf(_))
    ));
    for (from, to, feature) in [
        (
            "type='revolute'",
            "type='prismatic'",
            "joints (unsupported type)",
        ),
        (
            "<model name='robot'>",
            "<model name='robot'><static>true</static>",
            "joint topology (expected one dynamic world-anchored link)",
        ),
        (
            "</axis>",
            "<limit><lower>-1</lower><upper>1</upper></limit></axis>",
            "joint limits",
        ),
        (
            "</axis>",
            "<dynamics><damping>0.1</damping></dynamics></axis>",
            "joint dynamics",
        ),
    ] {
        let result = load(&XML.replace(from, to), PhysicsEngine::Rapier);
        assert!(
            matches!(result,Err(LoadError::Unsupported{feature:f,..}) if f==feature),
            "expected {feature}"
        );
    }
}

#[test]
fn missing_duplicate_or_foreign_joint_bindings_fault_before_time_advances() {
    for corruption in 0..7 {
        let mut sim = load(XML, PhysicsEngine::Rapier).unwrap();
        let joint = named(&sim, "default::robot::hinge");
        let model_id = sim
            .world()
            .get::<EntityId>(named(&sim, "default::robot"))
            .unwrap()
            .get();
        let handle = *sim.world().get::<JointHandle>(joint).unwrap();
        match corruption {
            0 => {
                sim.world_mut().entity_mut(joint).remove::<JointHandle>();
            }
            1 => {
                sim.world_mut().spawn(handle);
            }
            3 => {
                sim.world_mut().remove_resource::<RapierBackend>();
            }
            4 => {
                sim.world_mut()
                    .entity_mut(joint)
                    .remove::<bevy_ecs::hierarchy::ChildOf>();
            }
            5 => {
                sim.world_mut()
                    .resource_mut::<RapierBackend>()
                    .remove_joint(handle.id());
            }
            6 => {
                sim.world_mut().entity_mut(joint).insert(EntityKind::Sensor);
            }
            _ => {
                let mut foreign = RapierBackend::new([0.0; 3]).unwrap();
                let body = foreign
                    .spawn(BodySpec {
                        shape: Shape::Sphere { radius: 0.1 },
                        position: [0.0; 3],
                        velocity: [0.0; 3],
                        mass: Some(1.0),
                    })
                    .unwrap();
                assert_eq!(
                    foreign
                        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
                        .unwrap(),
                    handle.id()
                );
                sim.world_mut().insert_resource(foreign);
            }
        }
        assert_eq!(
            remove_model(&mut sim, model_id),
            Err(gz_sim_scene::RemoveError::InvalidPhysics)
        );
        assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
        assert_eq!(sim.elapsed().as_nanos(), 0);
    }
}

#[test]
fn semantic_joint_frame_axis_and_rotated_world_variants_preserve_the_constraint() {
    let implicit = XML.replace(
        "<pose relative_to='__model__'>0 0 0 0 0 0</pose>",
        "<pose>0 0 1 0 0 0</pose>",
    );
    let expressed = XML.replace(
        "<xyz>0 1 0</xyz>",
        "<xyz expressed_in='__model__'>0 2 0</xyz>",
    );
    for xml in [implicit, expressed] {
        let mut base = load(XML, PhysicsEngine::Rapier).unwrap();
        let mut variant = load(&xml, PhysicsEngine::Rapier).unwrap();
        base.step_exact(1000).unwrap();
        variant.step_exact(1000).unwrap();
        let a = base
            .world()
            .get::<JointSample>(named(&base, "default::robot::hinge"))
            .unwrap();
        let b = variant
            .world()
            .get::<JointSample>(named(&variant, "default::robot::hinge"))
            .unwrap();
        assert!((a.state.position - b.state.position).abs() < 1e-12);
        assert!((a.state.velocity - b.state.velocity).abs() < 1e-12);
    }
    let mut base = load(XML, PhysicsEngine::Rapier).unwrap();
    let mut rotated = load(
        &XML.replace("0 0 0 0 0.02 0", "0 0 0 0 0.02 0.7"),
        PhysicsEngine::Rapier,
    )
    .unwrap();
    base.step_exact(1000).unwrap();
    rotated.step_exact(1000).unwrap();
    let a = base
        .world()
        .get::<WorldPose>(named(&base, "default::robot::body"))
        .unwrap();
    let b = rotated
        .world()
        .get::<WorldPose>(named(&rotated, "default::robot::body"))
        .unwrap();
    let expected = glam::DQuat::from_rotation_z(0.7) * glam::DVec3::from_array(a.translation);
    assert!((expected - glam::DVec3::from_array(b.translation)).length() < 1e-10);
}
