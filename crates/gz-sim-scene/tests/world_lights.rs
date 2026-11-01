// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Supplemental headless construction metadata; no full native Light/B03 credit.
use bevy_ecs::hierarchy::ChildOf;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, EntityKind, LocalName, LocalPose, Name, PhysicsEngine, Velocity, WorldLight,
    WorldPose, load_world_with_sdf_step, remove_model, reset_loaded_world,
};
use sdformat::{Light, LightType, NoResources, Resource, ResourceResolver, Source, Transform};
use std::time::Duration;

const XML: &str = include_str!("fixtures/world_lights.sdf");
const B03: &str = include_str!("fixtures/b03_double_pendulum.sdf");
const ENGINES: [PhysicsEngine; 4] = [
    PhysicsEngine::Tpe,
    PhysicsEngine::Rapier,
    PhysicsEngine::Dart,
    PhysicsEngine::Mujoco,
];
fn named(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|v| v.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world().get::<EntityId>(named(sim, name)).unwrap().get()
}
fn lightless() -> String {
    let doc = roxmltree::Document::parse(XML).unwrap();
    let mut value = XML.to_string();
    for node in doc
        .descendants()
        .filter(|n| n.has_tag_name("light"))
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
    {
        value.replace_range(node.range(), "");
    }
    value
}
fn exact_pose(actual: WorldPose, expected: WorldPose) {
    assert_eq!(actual.translation, expected.translation);
    assert_eq!(actual.rotation, expected.rotation);
}
fn pose(actual: WorldPose, translation: [f64; 3], rotation: [f64; 4]) {
    for (a, b) in actual
        .translation
        .into_iter()
        .zip(translation)
        .chain(actual.rotation.into_iter().zip(rotation))
    {
        assert!((a - b).abs() < 1e-12, "{actual:?}");
    }
}
fn sun(uri: &str) -> Light {
    Light {
        source: Some(Source {
            uri: uri.into(),
            line: 9,
            column: 5,
        }),
        name: "sun".into(),
        light_type: LightType::Directional,
        pose: sdformat::Pose {
            transform: Transform {
                translation: [0., 0., 10.],
                rotation: [0., 0., 0., 1.],
            },
            relative_to: String::new(),
        },
        light_on: true,
        visualize: true,
        cast_shadows: true,
        diffuse: [0.8, 0.8, 0.8, 1.],
        specular: [0.2, 0.2, 0.2, 1.],
        attenuation_range: 1000.,
        spot_falloff: 0.,
        spot_inner_angle: 0.,
        spot_outer_angle: 0.,
        linear_attenuation_factor: 0.01,
        constant_attenuation_factor: 0.9,
        quadratic_attenuation_factor: 0.001,
        direction: [-0.5, 0.1, -0.9],
        intensity: 1.,
    }
}
#[test]
fn unchanged_b03_sun_has_complete_native_values_and_keeps_dart_chain_refusal() {
    let parsed = sdformat::load_str(B03, "b03.sdf", &NoResources).unwrap();
    assert_eq!(parsed.worlds[0].lights.as_slice(), &[sun("b03.sdf")]);
    let error =
        load_world_with_sdf_step(B03, "b03.sdf", "default", &NoResources, PhysicsEngine::Dart)
            .err()
            .unwrap();
    assert!(
        matches!(error,gz_sim_scene::LoadError::Unsupported{entity,feature} if entity=="default::double_pendulum_with_base0" && feature=="joint topology (expected one dynamic world-anchored link)")
    );
}
#[test]
fn whole_typed_metadata_world_ownership_and_semantic_poses_on_all_backends() {
    let parsed = sdformat::load_str(XML, "world-lights.sdf", &NoResources).unwrap();
    for engine in ENGINES {
        let sim =
            load_world_with_sdf_step(XML, "world-lights.sdf", "default", &NoResources, engine)
                .unwrap();
        let world = named(&sim, "default");
        assert_eq!(
            sim.world()
                .iter_entities()
                .filter(|e| e.contains::<WorldLight>())
                .count(),
            3
        );
        for expected in &parsed.worlds[0].lights {
            let entity = sim
                .world()
                .entity(named(&sim, &format!("default::{}", expected.name)));
            assert_eq!(entity.get::<WorldLight>().unwrap().0.as_ref(), expected);
            assert_eq!(entity.get::<EntityKind>(), Some(&EntityKind::Light));
            assert_eq!(entity.get::<LocalName>().unwrap().0, expected.name);
            assert_eq!(entity.get::<ChildOf>().unwrap().parent(), world);
            assert!(entity.get::<EntityId>().unwrap().get() > 0);
            assert!(!entity.contains::<BodyHandle>());
            exact_pose(
                entity.get::<LocalPose>().unwrap().0,
                *entity.get::<WorldPose>().unwrap(),
            );
        }
        pose(
            *sim.world()
                .get::<WorldPose>(named(&sim, "default::sun"))
                .unwrap(),
            [0., 0., 10.],
            [0., 0., 0., 1.],
        );
        let quarter = std::f64::consts::FRAC_1_SQRT_2;
        pose(
            *sim.world()
                .get::<WorldPose>(named(&sim, "default::point"))
                .unwrap(),
            [1., 3., 3.],
            [0., 0., quarter, quarter],
        );
        pose(
            *sim.world()
                .get::<WorldPose>(named(&sim, "default::spot"))
                .unwrap(),
            [2., 4., 4.],
            [0., 0., 0., 1.],
        );
    }
}
#[test]
fn metadata_adds_no_light_callbacks_and_does_not_change_physics() {
    for engine in ENGINES {
        let mut with =
            load_world_with_sdf_step(XML, "world-lights.sdf", "default", &NoResources, engine)
                .unwrap();
        let mut without = load_world_with_sdf_step(
            &lightless(),
            "world-lights.sdf",
            "default",
            &NoResources,
            engine,
        )
        .unwrap();
        assert_eq!(
            with.scheduled_system_counts(),
            without.scheduled_system_counts()
        );
        assert!(!without.world().iter_entities().any(
            |e| e.contains::<WorldLight>() || e.get::<EntityKind>() == Some(&EntityKind::Light)
        ));
        with.step_exact(5).unwrap();
        without.step_exact(5).unwrap();
        let a = named(&with, "default::robot::body");
        let b = named(&without, "default::robot::body");
        exact_pose(
            *with.world().get::<WorldPose>(a).unwrap(),
            *without.world().get::<WorldPose>(b).unwrap(),
        );
        let va = with.world().get::<Velocity>(a).unwrap();
        let vb = without.world().get::<Velocity>(b).unwrap();
        assert_eq!(va.linear, vb.linear);
        assert_eq!(va.angular, vb.angular);
        assert_eq!(
            with.scheduled_system_counts(),
            without.scheduled_system_counts()
        );
    }
}
#[test]
fn loaded_reset_restores_metadata_pose_name_and_removed_light_identity() {
    let parsed = sdformat::load_str(XML, "reset-light.sdf", &NoResources).unwrap();
    for engine in ENGINES {
        let mut sim =
            load_world_with_sdf_step(XML, "reset-light.sdf", "default", &NoResources, engine)
                .unwrap();
        let light_id = id(&sim, "default::sun");
        let initial_pose = *sim
            .world()
            .get::<WorldPose>(named(&sim, "default::sun"))
            .unwrap();
        for mode in 0..3 {
            let entity = entity_by_id(sim.world(), light_id).unwrap();
            if mode == 0 {
                let mut value = sim.world_mut().get_mut::<WorldLight>(entity).unwrap();
                value.0.intensity = 99.;
                value.0.source.as_mut().unwrap().uri = "changed".into();
                sim.world_mut().get_mut::<Name>(entity).unwrap().0 = "changed".into();
                sim.world_mut()
                    .get_mut::<WorldPose>(entity)
                    .unwrap()
                    .translation = [99.; 3];
            } else if mode == 1 {
                sim.world_mut().entity_mut(entity).remove::<WorldLight>();
            } else {
                sim.world_mut().entity_mut(entity).despawn();
                assert!(entity_by_id(sim.world(), light_id).is_none());
            }
            sim.step_exact(1).unwrap();
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(sim.elapsed(), Duration::ZERO);
            let entity = entity_by_id(sim.world(), light_id).unwrap();
            assert_eq!(sim.world().get::<Name>(entity).unwrap().0, "default::sun");
            assert_eq!(sim.world().get::<LocalName>(entity).unwrap().0, "sun");
            assert_eq!(
                sim.world().get::<WorldLight>(entity).unwrap().0.as_ref(),
                &parsed.worlds[0].lights[0]
            );
            exact_pose(*sim.world().get::<WorldPose>(entity).unwrap(), initial_pose);
            exact_pose(
                sim.world().get::<LocalPose>(entity).unwrap().0,
                initial_pose,
            );
        }
    }
}
#[test]
fn model_retirement_does_not_own_world_lights_and_reset_keeps_absence() {
    for engine in ENGINES {
        let mut sim =
            load_world_with_sdf_step(XML, "ownership-light.sdf", "default", &NoResources, engine)
                .unwrap();
        let lights = ["sun", "point", "spot"].map(|name| id(&sim, &format!("default::{name}")));
        let robot = id(&sim, "default::robot");
        remove_model(&mut sim, robot).unwrap();
        for light in lights {
            assert!(
                sim.world()
                    .get::<WorldLight>(entity_by_id(sim.world(), light).unwrap())
                    .is_some()
            );
        }
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(id(&sim, "default::robot"), robot);
        let mut empty = load_world_with_sdf_step(
            &lightless(),
            "absent-light.sdf",
            "default",
            &NoResources,
            engine,
        )
        .unwrap();
        let world = named(&empty, "default");
        empty
            .world_mut()
            .entity_mut(world)
            .insert(WorldLight(Box::default()));
        reset_loaded_world(&mut empty, |_| ()).unwrap();
        assert!(
            !empty
                .world()
                .iter_entities()
                .any(|e| e.contains::<WorldLight>())
        );
    }
}
struct Imported;
impl ResourceResolver for Imported {
    fn resolve(&self, uri: &str, parent: &str) -> Result<Resource, String> {
        assert_eq!((uri, parent), ("robot", "resources.sdf"));
        Ok(Resource{canonical_uri:"resources/robot.sdf".into(),xml:"<sdf version='1.12'><model name='robot'><link name='body'><pose>1 0 0 0 0 0</pose><collision name='shape'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model></sdf>".into()})
    }
}
#[test]
fn included_frame_resolution_keeps_declaring_light_source_and_reference() {
    let xml = "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><include><uri>robot</uri><name>imported</name><pose>10 0 0 0 0 0</pose></include><light name='lamp' type='point'><pose relative_to='imported::body'>0 2 0 0 0 0</pose></light></world></sdf>";
    let parsed = sdformat::load_str(xml, "resources.sdf", &Imported).unwrap();
    assert_eq!(
        parsed.worlds[0].models[0].links[0].source.uri,
        "resources/robot.sdf"
    );
    for engine in ENGINES {
        let sim =
            load_world_with_sdf_step(xml, "resources.sdf", "default", &Imported, engine).unwrap();
        let entity = named(&sim, "default::lamp");
        let light = sim.world().get::<WorldLight>(entity).unwrap();
        assert_eq!(light.0.as_ref(), &parsed.worlds[0].lights[0]);
        assert_eq!(light.0.source.as_ref().unwrap().uri, "resources.sdf");
        assert_eq!(light.0.pose.relative_to, "imported::body");
        pose(
            *sim.world().get::<WorldPose>(entity).unwrap(),
            [11., 2., 0.],
            [0., 0., 0., 1.],
        );
    }
}
#[test]
fn link_lights_and_invalid_document_poses_remain_refused() {
    let link = lightless().replace("</link>", "<light name='link_light' type='point'/></link>");
    for engine in ENGINES {
        assert!(
            matches!(load_world_with_sdf_step(&link,"link-light.sdf","default",&NoResources,engine),Err(gz_sim_scene::LoadError::Unsupported{feature,..}) if feature=="link lights")
        );
        let bad = XML.replace("relative_to=\"rig\"", "relative_to=\"missing\"");
        assert!(
            matches!(load_world_with_sdf_step(&bad,"missing-frame.sdf","default",&NoResources,engine),Err(gz_sim_scene::LoadError::Sdf(error)) if error.code==sdformat::ErrorCode::UnknownReference)
        );
    }
}

#[test]
fn reset_restores_lights_and_world_sensor_intent_together() {
    use gz_sim_scene::RenderingSensorsConfiguration;

    let xml = XML.replace(
        "<gravity>0 0 0</gravity>",
        "<gravity>0 0 0</gravity><plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>",
    );
    let parsed = sdformat::load_str(&xml, "light-sensors.sdf", &NoResources).unwrap();
    let mut sim = load_world_with_sdf_step(
        &xml,
        "light-sensors.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let world_id = id(&sim, "default");
    let light_ids: Vec<_> = parsed.worlds[0]
        .lights
        .iter()
        .map(|light| id(&sim, &format!("default::{}", light.name)))
        .collect();
    let world_entity = entity_by_id(sim.world(), world_id).unwrap();
    assert!(
        sim.world()
            .entity(world_entity)
            .contains::<RenderingSensorsConfiguration>()
    );
    sim.world_mut()
        .entity_mut(world_entity)
        .remove::<RenderingSensorsConfiguration>();
    for (index, light_id) in light_ids.iter().enumerate() {
        let entity = entity_by_id(sim.world(), *light_id).unwrap();
        if index == 0 {
            assert!(sim.world_mut().despawn(entity));
        } else {
            sim.world_mut()
                .get_mut::<WorldLight>(entity)
                .unwrap()
                .0
                .intensity = 99.;
        }
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let world_entity = entity_by_id(sim.world(), world_id).unwrap();
    assert!(
        sim.world()
            .entity(world_entity)
            .contains::<RenderingSensorsConfiguration>()
    );
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|e| e.contains::<WorldLight>())
            .count(),
        3
    );
    for (light_id, expected) in light_ids.iter().zip(&parsed.worlds[0].lights) {
        let entity = entity_by_id(sim.world(), *light_id).unwrap();
        assert_eq!(
            sim.world().get::<WorldLight>(entity).unwrap().0.as_ref(),
            expected
        );
        assert_eq!(
            sim.world().get::<ChildOf>(entity).unwrap().parent(),
            world_entity
        );
        assert_eq!(id(&sim, &format!("default::{}", expected.name)), *light_id);
    }
}
