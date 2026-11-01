// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Construction and reset controls from the pinned native Light observations.
use bevy_ecs::hierarchy::ChildOf;
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_scene::{
    EntityKind, LocalName, LocalPose, Name, PhysicsEngine, VisualGeometry, VisualProperties,
    WorldLight, WorldPose, load_world_with_sdf_step, reset_loaded_world,
};
use sdformat::NoResources;

const XML: &str = include_str!("fixtures/native_light_visuals.sdf");
const POSE: [f64; 7] = [
    3.44025611409198,
    -0.7556191885311716,
    4.483992419371312,
    0.09068129542746878,
    0.018271207596863924,
    0.05842711968717301,
    0.9939966485444552,
];
fn native_pose(actual: WorldPose) {
    for (a, b) in actual
        .translation
        .into_iter()
        .chain(actual.rotation)
        .zip(POSE)
    {
        assert!((a - b).abs() < 1e-12, "{actual:?}");
    }
}
fn exact(actual: WorldPose, expected: WorldPose) {
    assert_eq!(actual.translation, expected.translation);
    assert_eq!(actual.rotation, expected.rotation);
}
fn identity(actual: WorldPose) {
    assert_eq!(actual.translation, [0.; 3]);
    assert_eq!(actual.rotation, [0., 0., 0., 1.]);
}
#[test]
fn native_light_children_follow_parent_and_restore_complete_identity() {
    for engine in [
        PhysicsEngine::Tpe,
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        let mut sim = load_world_with_sdf_step(
            XML,
            "native-light-visuals.sdf",
            "default",
            &NoResources,
            engine,
        )
        .unwrap();
        let lights: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|e| e.contains::<WorldLight>())
            .map(|e| {
                (
                    e.get::<EntityId>().unwrap().get(),
                    e.get::<LocalName>().unwrap().0.clone(),
                )
            })
            .collect();
        assert_eq!(lights.len(), 12);
        let children: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::LightVisual))
            .map(|e| {
                (
                    e.get::<EntityId>().unwrap().get(),
                    e.get::<LocalName>().unwrap().0.clone(),
                )
            })
            .collect();
        assert_eq!(children.len(), 12);
        for (light_id, name) in lights {
            let light = entity_by_id(sim.world(), light_id).unwrap();
            let visual_name = format!("{name}Visual");
            let visual_id = children
                .iter()
                .find(|(_, name)| name == &visual_name)
                .unwrap()
                .0;
            let visual = entity_by_id(sim.world(), visual_id).unwrap();
            assert_ne!(light_id, visual_id);
            assert_eq!(sim.world().get::<ChildOf>(visual).unwrap().parent(), light);
            identity(sim.world().get::<LocalPose>(visual).unwrap().0);
            native_pose(*sim.world().get::<WorldPose>(visual).unwrap());
            let props = &sim.world().get::<VisualProperties>(visual).unwrap().0;
            assert!(!props.cast_shadows);
            assert_eq!(props.transparency, 0.0);
            assert!(sim.world().get::<VisualGeometry>(visual).is_none());
            let original_name = sim.world().get::<Name>(visual).unwrap().0.clone();
            sim.world_mut()
                .get_mut::<WorldPose>(light)
                .unwrap()
                .translation = [8.0, 9.0, 10.0];
            sim.step_exact(1).unwrap();
            identity(sim.world().get::<LocalPose>(visual).unwrap().0);
            exact(
                *sim.world().get::<WorldPose>(visual).unwrap(),
                *sim.world().get::<WorldPose>(light).unwrap(),
            );
            sim.world_mut().entity_mut(light).despawn();
            assert!(entity_by_id(sim.world(), visual_id).is_none());
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            let light = entity_by_id(sim.world(), light_id).unwrap();
            let visual = entity_by_id(sim.world(), visual_id).unwrap();
            assert_eq!(
                sim.world().get::<EntityKind>(visual),
                Some(&EntityKind::LightVisual)
            );
            assert_eq!(sim.world().get::<ChildOf>(visual).unwrap().parent(), light);
            assert_eq!(sim.world().get::<LocalName>(visual).unwrap().0, visual_name);
            assert_eq!(sim.world().get::<Name>(visual).unwrap().0, original_name);
            identity(sim.world().get::<LocalPose>(visual).unwrap().0);
            native_pose(*sim.world().get::<WorldPose>(visual).unwrap());
            let props = &sim.world().get::<VisualProperties>(visual).unwrap().0;
            assert!(!props.cast_shadows);
            assert_eq!(props.transparency, 0.0);
            sim.world_mut()
                .get_mut::<WorldPose>(light)
                .unwrap()
                .translation = [-4.0, 5.0, 6.0];
            sim.step_exact(1).unwrap();
            identity(sim.world().get::<LocalPose>(visual).unwrap().0);
            exact(
                *sim.world().get::<WorldPose>(visual).unwrap(),
                *sim.world().get::<WorldPose>(light).unwrap(),
            );
        }
    }
}
