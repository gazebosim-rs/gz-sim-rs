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
fn prepared_dart_free_root_exact_old_input_keeps_sampling_reset_and_removal() {
    let mut sim = load_world_with_sdf_step(
        XML,
        "body_joint.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Dart,
    )
    .unwrap();
    let names = ["default::robot::parent", "default::robot::child"];
    let initial = names.map(|name| *sim.world().get::<WorldPose>(named(&sim, name)).unwrap());
    let model_id = sim
        .world()
        .get::<EntityId>(named(&sim, "default::robot"))
        .unwrap()
        .get();
    let joint = named(&sim, "default::robot::hinge");
    let joint_id = sim.world().get::<EntityId>(joint).unwrap().get();
    let handle = *sim.world().get::<JointHandle>(joint).unwrap();
    let child = sim
        .world()
        .get::<BodyHandle>(named(&sim, names[1]))
        .unwrap()
        .id();
    let parent = sim
        .world()
        .get::<BodyHandle>(named(&sim, names[0]))
        .unwrap()
        .id();
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .joint_child(handle.id()),
        Some(child)
    );
    sim.step_exact(10).unwrap();
    for (name, pose) in names.into_iter().zip(initial) {
        let current = sim.world().get::<WorldPose>(named(&sim, name)).unwrap();
        for (actual, expected) in current.translation.into_iter().zip(pose.translation) {
            assert!((actual - expected).abs() < 1e-12);
        }
    }
    assert!(
        sim.world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .position
            .abs()
            < 1e-12
    );
    gz_sim_scene::queue_joint_effort(&mut sim, joint_id, 0.1).unwrap();
    sim.step_exact(1).unwrap();
    let parent_state = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(parent)
        .unwrap();
    assert!(
        parent_state
            .angular_velocity
            .into_iter()
            .any(|value| value.abs() > 1e-12)
    );
    assert!(
        parent_state
            .position
            .into_iter()
            .zip(initial[0].translation)
            .any(|(actual, expected)| (actual - expected).abs() > 1e-12)
    );
    assert!(
        sim.world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .velocity
            > 0.0
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let joint = named(&sim, "default::robot::hinge");
    assert_eq!(sim.world().get::<EntityId>(joint).unwrap().get(), joint_id);
    assert_eq!(
        sim.world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .position,
        0.0
    );
    sim.step_exact(1).unwrap();
    assert!(
        sim.world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .velocity
            .abs()
            < 1e-12
    );
    remove_model(&mut sim, model_id).unwrap();
    assert!(
        sim.world()
            .iter_entities()
            .all(|entity| entity.get::<JointHandle>().is_none())
    );
    sim.step_exact(1).unwrap();
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
    {
        let engine = PhysicsEngine::Mujoco;
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
fn unsupported_links_and_endpoint_aliases_are_rejected_explicitly() {
    use gz_sim_scene::LoadError;
    for xml in [
        XML.replace("</model>", "<link name='extra'/></model>"),
        XML.replace(
            "<model name='robot'>",
            "<model name='robot'><frame name='mount' attached_to='parent'/>",
        )
        .replace("<parent>parent</parent>", "<parent>mount</parent>"),
    ] {
        assert!(matches!(
            load_world_with_sdf_step(
                &xml,
                "body_joint.sdf",
                "default",
                &NoResources,
                PhysicsEngine::Rapier
            ),
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

#[test]
fn previously_rejected_world_root_and_forest_inputs_preserve_physics_and_reset() {
    use glam::{DQuat, DVec3};
    for (rooted, xml) in [
        (false, XML.replace("<parent>parent</parent>", "<parent>world</parent>")),
        (true, XML.replace("</model>", "<joint name='other' type='revolute'><parent>world</parent><child>parent</child><axis><xyz>0 1 0</xyz></axis></joint></model>")),
    ] {
        let mut sim = load_world_with_sdf_step(&xml, "body_joint.sdf", "default", &NoResources, PhysicsEngine::Rapier).unwrap();
        let parent = named(&sim, "default::robot::parent");
        let child = named(&sim, "default::robot::child");
        let joint = named(&sim, "default::robot::hinge");
        let initial_joint = *sim.world().get::<WorldPose>(joint).unwrap();
        let parent_pose = *sim.world().get::<WorldPose>(parent).unwrap();
        let parent_body = sim.world().get::<BodyHandle>(parent).unwrap().id();
        let child_body = sim.world().get::<BodyHandle>(child).unwrap().id();
        if rooted {
            let axis = DQuat::from_array(parent_pose.rotation) * DVec3::Y;
            let arm = DVec3::from_array(sim.world().get::<WorldPose>(child).unwrap().translation)
                - DVec3::from_array(parent_pose.translation);
            sim.world_mut().resource_mut::<PhysicsBackend>().set_world_velocity(child_body, axis.cross(arm).to_array(), axis.to_array()).unwrap();
        } else {
            assert_eq!(sim.world().resource::<PhysicsBackend>().is_jointed(parent_body), Some(false));
            sim.world_mut().resource_mut::<PhysicsBackend>().set_world_velocity(parent_body, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
        }
        sim.step_exact(100).unwrap();
        let joint_pose = sim.world().get::<WorldPose>(joint).unwrap();
        assert!((DVec3::from_array(joint_pose.translation) - DVec3::from_array(initial_joint.translation)).length() < 1e-4);
        if rooted {
            assert!(sim.world().get::<JointSample>(joint).unwrap().state.position > 0.05);
            assert!((sim.world().get::<WorldPose>(parent).unwrap().translation[0] - parent_pose.translation[0]).abs() < 1e-5);
        } else {
            assert!((sim.world().get::<WorldPose>(parent).unwrap().translation[0] - parent_pose.translation[0] - 0.1).abs() < 1e-10);
            assert!(sim.world().get::<JointSample>(joint).unwrap().state.position.abs() < 1e-12);
        }
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        let joint = named(&sim, "default::robot::hinge");
        assert_eq!(sim.world().get::<JointSample>(joint).unwrap().state.position, 0.0);
        assert_eq!(sim.world().get::<JointSample>(joint).unwrap().epoch, 1);
        sim.step_exact(10).unwrap();
        assert!(sim.world().get::<JointSample>(joint).unwrap().state.position.abs() < 1e-12);
    }
}

#[test]
fn three_link_revolutes_keep_relative_motion_contacts_and_loaded_lifecycle() {
    use glam::{DQuat, DVec3};
    let xml = XML.replace("0 0 -1 0 0 0", "0 0 0 0 0 0")
        .replace("</model>", "<link name='third'><collision name='shape'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link><joint name='second' type='revolute'><parent>child</parent><child>third</child><axis><xyz expressed_in='__model__'>0 1 0</xyz></axis></joint></model>");
    let mut sim = load_world_with_sdf_step(
        &xml,
        "articulated.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let model = named(&sim, "default::robot");
    let model_id = sim.world().get::<EntityId>(model).unwrap().get();
    let parent = named(&sim, "default::robot::parent");
    let initial = *sim.world().get::<WorldPose>(parent).unwrap();
    let axis = DQuat::from_array(initial.rotation) * DVec3::Y;
    for (name, speed) in [("parent", 1.0), ("child", 2.0), ("third", 3.0)] {
        let entity = named(&sim, &format!("default::robot::{name}"));
        let body = sim.world().get::<BodyHandle>(entity).unwrap().id();
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_world_velocity(body, [0.3, 0.0, 0.0], (axis * speed).to_array())
            .unwrap();
    }
    sim.step_exact(100).unwrap();
    let mut joints = Vec::new();
    for name in ["hinge", "second"] {
        let joint = named(&sim, &format!("default::robot::{name}"));
        let sample = sim.world().get::<JointSample>(joint).unwrap();
        assert!((sample.state.position - 0.1).abs() < 1e-7);
        assert!((sample.state.velocity - 1.0).abs() < 1e-10);
        joints.push(sim.world().get::<JointHandle>(joint).unwrap().id());
    }
    for name in ["parent", "child", "third"] {
        let entity = named(&sim, &format!("default::robot::{name}"));
        let position = sim.world().get::<WorldPose>(entity).unwrap().translation;
        assert!(
            (DVec3::from_array(position)
                - DVec3::from_array(initial.translation)
                - DVec3::new(0.03, 0.0, 0.0))
            .length()
                < 1e-10
        );
    }
    remove_model(&mut sim, model_id).unwrap();
    for joint in joints {
        assert!(
            sim.world()
                .resource::<PhysicsBackend>()
                .joint_state(joint)
                .is_none()
        );
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(100).unwrap();
    for name in ["hinge", "second"] {
        let joint = named(&sim, &format!("default::robot::{name}"));
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
}

#[test]
fn cycles_and_multiple_parent_joints_are_explicitly_rejected() {
    use gz_sim_scene::LoadError;
    for (extra, feature) in [
        (
            "<joint name='other' type='revolute'><parent>child</parent><child>parent</child><axis><xyz>0 1 0</xyz></axis></joint>",
            "closed joint loops",
        ),
        (
            "<joint name='other' type='revolute'><parent>world</parent><child>child</child><axis><xyz>0 1 0</xyz></axis></joint>",
            "multiple parent joints",
        ),
    ] {
        let xml = XML.replace("</model>", &format!("{extra}</model>"));
        assert!(
            matches!(load_world_with_sdf_step(&xml, "articulated.sdf", "default", &NoResources, PhysicsEngine::Rapier), Err(LoadError::Unsupported { feature: f, .. }) if f == feature)
        );
    }
}

#[test]
fn deep_valid_joint_graph_reaches_geometry_checks_without_recursion() {
    use gz_sim_scene::LoadError;
    let mut xml = String::from("<sdf version='1.12'><world name='default'><model name='chain'>");
    for index in 0..=20_000 {
        xml.push_str(&format!("<link name='l{index}'/>"));
        if index != 0 {
            xml.push_str(&format!("<joint name='j{index}' type='revolute'><parent>l{}</parent><child>l{index}</child><axis><xyz>0 1 0</xyz></axis></joint>", index - 1));
        }
    }
    xml.push_str("</model></world></sdf>");
    assert!(matches!(
        load_world_with_sdf_step(
            &xml,
            "deep.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "collision count (expected one)",
            ..
        })
    ));
}

#[test]
fn separate_articulated_models_keep_external_contacts_and_distinct_groups() {
    let body = XML
        .split_once("<model name='robot'>")
        .unwrap()
        .1
        .split_once("</model>")
        .unwrap()
        .0
        .replace(
            "<pose>2 -3 4 0.2 -0.3 0.4</pose>",
            "<pose>0 0 0 0 0 0</pose>",
        )
        .replace("0 0 -1 0 0 0", "0 0 0 0 0 0");
    let other = body.replacen("<pose>0 0 0 0 0 0</pose>", "<pose>0.15 0 0 0 0 0</pose>", 1);
    let xml = format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='a'>{body}</model><model name='b'>{other}</model></world></sdf>"
    );
    let mut sim = load_world_with_sdf_step(
        &xml,
        "models.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let a = named(&sim, "default::a::parent");
    let b = named(&sim, "default::b::parent");
    let model_b = named(&sim, "default::b");
    let model_b_id = sim.world().get::<EntityId>(model_b).unwrap().get();
    sim.step_exact(100).unwrap();
    assert!(sim.world().get::<WorldPose>(a).unwrap().translation[0] < -1e-4);
    assert!(sim.world().get::<WorldPose>(b).unwrap().translation[0] > 0.1501);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    remove_model(&mut sim, model_b_id).unwrap();
    let a = named(&sim, "default::a::parent");
    sim.step_exact(100).unwrap();
    assert!(sim.world().get::<WorldPose>(a).unwrap().translation[0].abs() < 1e-12);
}

#[test]
fn unconstrained_links_in_an_articulated_model_share_its_contact_group() {
    use glam::DVec3;
    let xml = XML.replace("0 0 -1 0 0 0", "0 0 0 0 0 0")
        .replace("</model>", "<link name='free'><collision name='shape'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model>");
    let mut sim = load_world_with_sdf_step(
        &xml,
        "forest.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let free = named(&sim, "default::robot::free");
    let body = sim.world().get::<BodyHandle>(free).unwrap().id();
    assert_eq!(
        sim.world().resource::<PhysicsBackend>().is_jointed(body),
        Some(false)
    );
    let initial = sim.world().get::<WorldPose>(free).unwrap().translation;
    sim.step_exact(100).unwrap();
    let actual = sim.world().get::<WorldPose>(free).unwrap().translation;
    assert!((DVec3::from_array(actual) - DVec3::from_array(initial)).length() < 1e-10);
}
