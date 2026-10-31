// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::Entity;
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    JointHandle, JointSample, Name, PhysicsEngine, load_world_with_sdf_step, queue_joint_effort,
    remove_model, reset_loaded_world,
};
use sdformat::NoResources;

// Child joint precedes its root deliberately: declaration order owns public
// identities, but DART must construct the world hinge before attaching a child.
const XML: &str = "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity>
<physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>
<model name='robot'>
<link name='parent'><pose>0 0 2 0 0 0</pose><collision name='shape'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link>
<link name='child'><pose>0 0 1 0 0 0</pose><collision name='shape'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link>
<joint name='child_hinge' type='revolute'><parent>parent</parent><child>child</child><axis><xyz>0 1 0</xyz></axis></joint>
<joint name='root_hinge' type='revolute'><parent>world</parent><child>parent</child><axis><xyz>0 1 0</xyz></axis></joint>
</model></world></sdf>";

fn named(sim: &Simulation, suffix: &str) -> Entity {
    let name = format!("default::robot{suffix}");
    sim.world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap()
        .id()
}

#[test]
fn reversed_declarations_preserve_identity_and_coupled_effort_through_reset_and_removal() {
    let mut sim = load_world_with_sdf_step(
        XML,
        "dart_chain.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Dart,
    )
    .unwrap();
    let root = named(&sim, "::root_hinge");
    let child = named(&sim, "::child_hinge");
    let child_id = sim.world().get::<EntityId>(child).unwrap().get();
    let root_id = sim.world().get::<EntityId>(root).unwrap().get();
    let backend_ids =
        [root, child].map(|entity| sim.world().get::<JointHandle>(entity).unwrap().id());
    assert!(child_id < root_id);
    for _ in 0..10 {
        queue_joint_effort(&mut sim, child_id, 1.0).unwrap();
        sim.step_exact(1).unwrap();
    }
    // Unit masses/inertias and one-metre separation give M=[[3,1],[1,1]].
    // A unit child effort therefore accelerates root/child by -0.5/+1.5.
    for (entity, velocity, position) in [(root, -0.005, -0.0000275), (child, 0.015, 0.0000825)] {
        let state = sim.world().get::<JointSample>(entity).unwrap().state;
        assert!((state.velocity - velocity).abs() < 1e-12);
        assert!((state.position - position).abs() < 1e-12);
    }

    reset_loaded_world(&mut sim, |_| ()).unwrap();
    for (index, (suffix, public_id)) in [("::root_hinge", root_id), ("::child_hinge", child_id)]
        .into_iter()
        .enumerate()
    {
        let entity = named(&sim, suffix);
        assert_eq!(
            sim.world().get::<EntityId>(entity).unwrap().get(),
            public_id
        );
        assert_eq!(
            sim.world().get::<JointHandle>(entity).unwrap().id(),
            backend_ids[index]
        );
        let sample = sim.world().get::<JointSample>(entity).unwrap();
        assert_eq!(sample.epoch, 1);
        assert_eq!(sample.state.position, 0.0);
        assert_eq!(sample.state.velocity, 0.0);
    }
    sim.step_exact(10).unwrap();
    let joints = ["::root_hinge", "::child_hinge"].map(|suffix| {
        sim.world()
            .get::<JointHandle>(named(&sim, suffix))
            .unwrap()
            .id()
    });
    let model_id = sim.world().get::<EntityId>(named(&sim, "")).unwrap().get();
    remove_model(&mut sim, model_id).unwrap();
    for joint in joints {
        assert!(
            sim.world()
                .resource::<PhysicsBackend>()
                .joint_state(joint)
                .is_none()
        );
    }
    sim.step_exact(1).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    for (index, (suffix, public_id)) in [("::root_hinge", root_id), ("::child_hinge", child_id)]
        .into_iter()
        .enumerate()
    {
        let entity = named(&sim, suffix);
        assert_eq!(
            sim.world().get::<EntityId>(entity).unwrap().get(),
            public_id
        );
        assert_eq!(
            sim.world().get::<JointHandle>(entity).unwrap().id(),
            backend_ids[index]
        );
        let sample = sim.world().get::<JointSample>(entity).unwrap();
        assert_eq!(sample.epoch, 2);
        assert_eq!(sample.state.position, 0.0);
        assert_eq!(sample.state.velocity, 0.0);
    }
    queue_joint_effort(&mut sim, child_id, 1.0).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        (sim.world()
            .get::<JointSample>(named(&sim, "::child_hinge"))
            .unwrap()
            .state
            .velocity
            - 0.0015)
            .abs()
            < 1e-12
    );
}

#[test]
fn body_and_joint_declaration_orders_do_not_change_motion_or_overlapping_pair_retirement() {
    for reverse_links in [false, true] {
        for reverse_joints in [false, true] {
            let mut xml = XML.replace("0 0 1 0 0 0", "0 0 2 0 0 0");
            for (reverse, start, end) in [
                (reverse_links, "<link name='parent'>", "</link>"),
                (reverse_joints, "<joint name='child_hinge'", "</joint>"),
            ] {
                if reverse {
                    let first = xml.find(start).unwrap();
                    let middle = first + xml[first..].find(end).unwrap() + end.len();
                    let last = middle + xml[middle..].find(end).unwrap() + end.len();
                    xml.replace_range(
                        first..last,
                        &format!("{}{}", &xml[middle..last], &xml[first..middle]),
                    );
                }
            }
            let mut sim = load_world_with_sdf_step(
                &xml,
                "overlapping_chain.sdf",
                "default",
                &NoResources,
                PhysicsEngine::Dart,
            )
            .unwrap();
            let child = named(&sim, "::child_hinge");
            let child_id = sim.world().get::<EntityId>(child).unwrap().get();
            for _ in 0..10 {
                queue_joint_effort(&mut sim, child_id, 1.0).unwrap();
                sim.step_exact(1).unwrap();
            }
            for (suffix, expected) in [("::root_hinge", -0.01), ("::child_hinge", 0.02)] {
                let sample = sim.world().get::<JointSample>(named(&sim, suffix)).unwrap();
                assert!((sample.state.velocity - expected).abs() < 1e-12);
            }
            let model_id = sim.world().get::<EntityId>(named(&sim, "")).unwrap().get();
            remove_model(&mut sim, model_id).unwrap();
            sim.step_exact(1).unwrap();
        }
    }
}

#[test]
fn independent_world_hinges_and_larger_dart_models_remain_rejected() {
    for xml in [
        XML.replace("<parent>parent</parent>", "<parent>world</parent>"),
        XML.replace("</model>", "<link name='third'/></model>"),
    ] {
        assert!(matches!(
            load_world_with_sdf_step(
                &xml,
                "unsupported_chain.sdf",
                "default",
                &NoResources,
                PhysicsEngine::Dart,
            ),
            Err(gz_sim_scene::LoadError::Unsupported { .. })
        ));
    }
}

#[test]
fn external_contact_is_rejected_during_construction() {
    let obstacle = "<model name='obstacle'><static>true</static><link name='body'><pose>0 0 2 0 0 0</pose><collision name='shape'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model>";
    let xml = XML.replace(
        "<model name='robot'>",
        &format!("{obstacle}<model name='robot'>"),
    );
    assert!(matches!(
        load_world_with_sdf_step(
            &xml,
            "contacting_chain.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Dart
        ),
        Err(gz_sim_scene::LoadError::Physics(
            gz_physics::PhysicsError::InvalidJoint
        ))
    ));
}
