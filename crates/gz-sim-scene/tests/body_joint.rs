// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::Entity;
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, JointHandle, JointSample, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step,
    remove_model, reset_loaded_world,
};
use sdformat::NoResources;

const XML: &str = "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity>
<physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>
<model name='robot'><pose>2 -3 4 0.2 -0.3 0.4</pose>
<link name='parent'><collision name='shape'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link>
<link name='child'><pose>0 0 -1 0 0 0</pose><collision name='shape'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link>
<joint name='hinge' type='revolute'><parent>parent</parent><child>child</child>
<pose relative_to='parent'>0 0 0 0 0 0</pose><axis><xyz expressed_in='__model__'>0 1 0</xyz></axis></joint>
</model></world></sdf>";

fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap()
        .id()
}

#[test]
fn two_link_body_hinge_uses_both_endpoints_and_survives_loaded_reset() {
    let mut sim = load_world_with_sdf_step(
        XML,
        "body_joint.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let parent = named(&sim, "default::robot::parent");
    let child = named(&sim, "default::robot::child");
    let joint = named(&sim, "default::robot::hinge");
    let model = named(&sim, "default::robot");
    let model_id = sim.world().get::<EntityId>(model).unwrap().get();
    let joint_id = sim.world().get::<EntityId>(joint).unwrap().get();
    let parent_id = sim.world().get::<BodyHandle>(parent).unwrap().id();
    let child_id = sim.world().get::<BodyHandle>(child).unwrap().id();
    let handle = *sim.world().get::<JointHandle>(joint).unwrap();
    let initial_parent = *sim.world().get::<WorldPose>(parent).unwrap();
    let initial_child = *sim.world().get::<WorldPose>(child).unwrap();
    let backend = sim.world().resource::<PhysicsBackend>();
    assert_eq!(backend.is_jointed(parent_id), Some(true));
    assert_eq!(backend.is_jointed(child_id), Some(true));
    assert_eq!(backend.joint_child(handle.id()), Some(child_id));
    sim.step_exact(10).unwrap();
    assert_eq!(
        sim.world().get::<WorldPose>(parent).unwrap().translation,
        initial_parent.translation
    );
    assert_eq!(
        sim.world().get::<WorldPose>(child).unwrap().translation,
        initial_child.translation
    );
    assert!(
        sim.world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .position
            .abs()
            < 1e-12
    );
    remove_model(&mut sim, model_id).unwrap();
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .joint_state(handle.id())
            .is_none()
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = named(&sim, "default::robot::hinge");
    assert_eq!(
        sim.world().get::<EntityId>(restored).unwrap().get(),
        joint_id
    );
    assert_eq!(sim.world().get::<JointSample>(restored).unwrap().epoch, 1);
    sim.step_exact(10).unwrap();
    assert!(
        sim.world()
            .get::<JointSample>(restored)
            .unwrap()
            .state
            .position
            .abs()
            < 1e-12
    );
}

#[test]
fn moving_endpoints_sample_relative_angle_and_joint_pose_follows_child() {
    use glam::{DQuat, DVec3};
    let xml = XML.replace("0 0 -1 0 0 0", "0 0 0 0 0 0");
    let mut sim = load_world_with_sdf_step(
        &xml,
        "body_joint.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let parent = named(&sim, "default::robot::parent");
    let child = named(&sim, "default::robot::child");
    let joint = named(&sim, "default::robot::hinge");
    let start = *sim.world().get::<WorldPose>(parent).unwrap();
    let axis = DQuat::from_array(start.rotation) * DVec3::Y;
    for (entity, speed) in [(parent, 1.0), (child, 2.0)] {
        let id = sim.world().get::<BodyHandle>(entity).unwrap().id();
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_world_velocity(id, [0.3, -0.2, 0.1], (axis * speed).to_array())
            .unwrap();
    }
    sim.step_exact(100).unwrap();
    let sample = sim.world().get::<JointSample>(joint).unwrap();
    assert!((sample.state.position - 0.1).abs() < 1e-7);
    assert!((sample.state.velocity - 1.0).abs() < 1e-10);
    let parent_pose = *sim.world().get::<WorldPose>(parent).unwrap();
    let child_pose = *sim.world().get::<WorldPose>(child).unwrap();
    let joint_pose = *sim.world().get::<WorldPose>(joint).unwrap();
    assert!(
        (DVec3::from_array(parent_pose.translation)
            - DVec3::from_array(start.translation)
            - DVec3::new(0.03, -0.02, 0.01))
        .length()
            < 1e-10
    );
    assert!(
        (DVec3::from_array(parent_pose.translation) - DVec3::from_array(child_pose.translation))
            .length()
            < 1e-10
    );
    let child_rotation = DQuat::from_array(child_pose.rotation);
    assert!(
        child_rotation
            .dot(DQuat::from_array(joint_pose.rotation))
            .abs()
            > 1.0 - 1e-12
    );
    assert!(
        child_rotation
            .dot(DQuat::from_array(parent_pose.rotation))
            .abs()
            < 0.999
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let child = named(&sim, "default::robot::child");
    let joint = named(&sim, "default::robot::hinge");
    assert_eq!(
        sim.world().get::<WorldPose>(child).unwrap().translation,
        start.translation
    );
    assert_eq!(
        sim.world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .position,
        0.0
    );
    sim.step_exact(100).unwrap();
    assert!(
        sim.world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .position
            .abs()
            < 1e-12
    );
}

#[test]
fn unsupported_engines_do_not_fall_back_to_world_anchoring() {
    use gz_physics::PhysicsError;
    use gz_sim_scene::LoadError;
    for engine in [PhysicsEngine::Dart, PhysicsEngine::Mujoco] {
        let result =
            load_world_with_sdf_step(XML, "body_joint.sdf", "default", &NoResources, engine);
        assert!(matches!(
            result,
            Err(LoadError::Physics(PhysicsError::Unsupported {
                capability: "body-to-body revolute joint",
                ..
            }))
        ));
    }
    assert!(matches!(
        load_world_with_sdf_step(
            XML,
            "body_joint.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Tpe
        ),
        Err(LoadError::Unsupported {
            feature: "TPE joints",
            ..
        })
    ));
}

#[test]
fn parent_and_sampling_corruption_faults_before_removal_or_time_advances() {
    use bevy_ecs::hierarchy::ChildOf;
    use gz_sim_scene::{LocalPose, RemoveError};
    for corruption in 0..9 {
        let mut sim = load_world_with_sdf_step(
            XML,
            "body_joint.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let parent = named(&sim, "default::robot::parent");
        let child = named(&sim, "default::robot::child");
        let joint = named(&sim, "default::robot::hinge");
        let model = named(&sim, "default::robot");
        let world = named(&sim, "default");
        let model_id = sim.world().get::<EntityId>(model).unwrap().get();
        match corruption {
            0 => {
                sim.world_mut().entity_mut(parent).remove::<BodyHandle>();
            }
            1 => {
                sim.world_mut().entity_mut(parent).insert(ChildOf(world));
            }
            2 => {
                let a = sim
                    .world_mut()
                    .entity_mut(parent)
                    .take::<BodyHandle>()
                    .unwrap();
                let b = sim
                    .world_mut()
                    .entity_mut(child)
                    .take::<BodyHandle>()
                    .unwrap();
                sim.world_mut().entity_mut(parent).insert(b);
                sim.world_mut().entity_mut(child).insert(a);
            }
            3 => {
                sim.world_mut().entity_mut(parent).remove::<WorldPose>();
            }
            4 => {
                sim.world_mut().entity_mut(joint).remove::<WorldPose>();
            }
            5 => {
                sim.world_mut().entity_mut(joint).remove::<JointSample>();
            }
            6 => {
                sim.world_mut().entity_mut(joint).remove::<LocalPose>();
            }
            7 => {
                sim.world_mut().entity_mut(model).remove::<WorldPose>();
            }
            8 => {
                sim.world_mut().entity_mut(parent).despawn();
            }
            _ => unreachable!(),
        }
        assert_eq!(
            remove_model(&mut sim, model_id),
            Err(RemoveError::InvalidPhysics),
            "case {corruption}"
        );
        assert!(sim.step_exact(1).is_err(), "case {corruption}");
        assert_eq!(sim.elapsed().as_nanos(), 0, "case {corruption}");
    }
}

#[test]
fn larger_or_aliased_topologies_are_rejected_explicitly() {
    use gz_sim_scene::LoadError;
    for xml in [
        XML.replace("</model>", "<link name='extra'/></model>"),
        XML.replace("<parent>parent</parent>", "<parent>world</parent>"),
        XML.replace("<model name='robot'>", "<model name='robot'><frame name='mount' attached_to='parent'/>")
            .replace("<parent>parent</parent>", "<parent>mount</parent>"),
        XML.replace("</model>", "<joint name='other' type='revolute'><parent>world</parent><child>parent</child><axis><xyz>0 1 0</xyz></axis></joint></model>"),
    ] {
        assert!(matches!(
            load_world_with_sdf_step(&xml, "body_joint.sdf", "default", &NoResources, PhysicsEngine::Rapier),
            Err(LoadError::Unsupported { .. })
        ));
    }
}

#[test]
fn missing_endpoint_velocity_must_fault_before_advancing() {
    for endpoint in ["parent", "child"] {
        let mut sim = load_world_with_sdf_step(
            XML,
            "review.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let entity = named(&sim, &format!("default::robot::{endpoint}"));
        sim.world_mut()
            .entity_mut(entity)
            .remove::<gz_sim_scene::Velocity>();
        let model = named(&sim, "default::robot");
        let model_id = sim.world().get::<EntityId>(model).unwrap().get();
        assert_eq!(
            remove_model(&mut sim, model_id),
            Err(gz_sim_scene::RemoveError::InvalidPhysics)
        );
        assert!(
            sim.step_exact(1).is_err(),
            "missing {endpoint} Velocity was admitted"
        );
        assert_eq!(sim.elapsed().as_nanos(), 0);
    }
}

#[test]
fn canonical_child_retains_nonidentity_child_to_joint_pose() {
    use glam::{DQuat, DVec3};
    let xml = XML
        .replace(
            "<model name='robot'>",
            "<model name='robot' canonical_link='child'>",
        )
        .replace("0 0 -1 0 0 0", "0 0 0 0.5 -0.2 0.3")
        .replace(
            "<pose relative_to='parent'>0 0 0 0 0 0</pose>",
            "<pose relative_to='parent'>0 0.2 0 0 0 0</pose>",
        );
    let mut sim = load_world_with_sdf_step(
        &xml,
        "review.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let parent = named(&sim, "default::robot::parent");
    let child = named(&sim, "default::robot::child");
    let joint = named(&sim, "default::robot::hinge");
    let cp = *sim.world().get::<WorldPose>(child).unwrap();
    let jp = *sim.world().get::<WorldPose>(joint).unwrap();
    let pp = *sim.world().get::<WorldPose>(parent).unwrap();
    let cq = DQuat::from_array(cp.rotation);
    let offset_rotation = cq.conjugate() * DQuat::from_array(jp.rotation);
    let offset_translation =
        cq.conjugate() * (DVec3::from_array(jp.translation) - DVec3::from_array(cp.translation));
    assert!(offset_rotation.w.abs() < 0.99);
    assert!(offset_translation.length() > 0.1);
    let axis = DQuat::from_array(pp.rotation) * DVec3::Y;
    for (e, speed) in [(parent, 1.), (child, 2.)] {
        let id = sim.world().get::<BodyHandle>(e).unwrap().id();
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_world_velocity(id, [0.3, -0.2, 0.1], (axis * speed).to_array())
            .unwrap();
    }
    for _ in 0..100 {
        sim.step_exact(1).unwrap();
        let cp = sim.world().get::<WorldPose>(child).unwrap();
        let jp = sim.world().get::<WorldPose>(joint).unwrap();
        let cq = DQuat::from_array(cp.rotation);
        assert!(
            (DVec3::from_array(jp.translation)
                - DVec3::from_array(cp.translation)
                - cq * offset_translation)
                .length()
                < 1e-11
        );
        assert!(
            (cq * offset_rotation)
                .dot(DQuat::from_array(jp.rotation))
                .abs()
                > 1. - 1e-12
        );
    }
    assert!(
        (sim.world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .position
            - 0.1)
            .abs()
            < 1e-7
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let joint = named(&sim, "default::robot::hinge");
    let restored = *sim.world().get::<WorldPose>(joint).unwrap();
    assert!(
        (DVec3::from_array(restored.translation) - DVec3::from_array(jp.translation)).length()
            < 1e-12
    );
    assert!(
        DQuat::from_array(restored.rotation)
            .dot(DQuat::from_array(jp.rotation))
            .abs()
            > 1. - 1e-12
    );
}

#[test]
fn joint_endpoint_group_cannot_use_another_joint_as_its_model() {
    use bevy_ecs::hierarchy::ChildOf;
    let body = XML
        .split_once("<model name='robot'>")
        .unwrap()
        .1
        .split_once("</model>")
        .unwrap()
        .0;
    let xml = XML.replace(
        "</world>",
        &format!("<model name='other'>{body}</model></world>"),
    );
    for owner in ["default::other::hinge", "default::other"] {
        let mut sim = load_world_with_sdf_step(
            &xml,
            "review.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let other_joint = named(&sim, owner);
        for name in ["parent", "child", "hinge"] {
            let entity = named(&sim, &format!("default::robot::{name}"));
            sim.world_mut()
                .entity_mut(entity)
                .insert(ChildOf(other_joint));
        }
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| sim.step_exact(1)));
        assert!(
            matches!(result, Ok(Err(_))),
            "invalid owner must fault, not panic or advance"
        );
        assert_eq!(sim.elapsed().as_nanos(), 0);
    }
}
