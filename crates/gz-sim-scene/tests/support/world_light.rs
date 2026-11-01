// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::hierarchy::ChildOf;
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_scene::{
    EntityKind, InputError, Name, PhysicsEngine, Velocity, WorldLight, WorldPose,
    load_world_with_sdf_step, queue_link_world_wrench, reset_loaded_world,
};
use sdformat::{Light, LightType, NoResources, Pose, Source};
use std::{collections::BTreeMap, time::Duration};

const LIGHT: &str = "<light name='lamp' type='point'/>";
pub const ORIGINAL_STATIC_LIGHT_XML: &str = "<sdf version='1.12'><world name='default'><light name='lamp' type='point'/><model name='robot' ><static>true</static><link name='body'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model></world></sdf>";

pub fn assert_static_world_light(xml: &str, uri: &str) {
    assert_eq!(xml, ORIGINAL_STATIC_LIGHT_XML);
    assert_eq!(uri, "sdf8");
    let parsed = sdformat::load_str(xml, uri, &NoResources).unwrap();
    assert_eq!(parsed.version, "1.12");
    assert!(parsed.models.is_empty());
    assert_eq!(parsed.worlds.len(), 1);
    let expected_light = Light {
        source: Some(Source {
            uri: uri.into(),
            line: 1,
            column: u32::try_from(xml.find(LIGHT).unwrap() + 1).unwrap(),
        }),
        name: "lamp".into(),
        light_type: LightType::Point,
        pose: Pose::default(),
        light_on: true,
        visualize: true,
        cast_shadows: false,
        diffuse: [1.; 4],
        specular: [0.1, 0.1, 0.1, 1.],
        attenuation_range: 10.,
        spot_falloff: 0.,
        spot_inner_angle: 0.,
        spot_outer_angle: 0.,
        linear_attenuation_factor: 1.,
        constant_attenuation_factor: 1.,
        quadratic_attenuation_factor: 0.,
        direction: [0., 0., -1.],
        intensity: 1.,
    };
    let mut expected = sdformat::load_str(&xml.replace(LIGHT, ""), uri, &NoResources).unwrap();
    let delta = u32::try_from(LIGHT.len()).unwrap();
    let model = &mut expected.worlds[0].models[0];
    model.source.column += delta;
    model.links[0].source.column += delta;
    model.links[0].collisions[0].source.column += delta;
    expected.worlds[0].lights.push(expected_light.clone());
    assert_eq!(
        format!("{:?}", parsed.worlds),
        format!("{:?}", expected.worlds)
    );
    let world = &parsed.worlds[0];
    assert_eq!(world.name, "default");
    assert_eq!(world.source.uri, uri);
    assert_eq!(
        world.lights.as_slice(),
        std::slice::from_ref(&expected_light)
    );
    assert_eq!(world.models.len(), 1);
    let model = &world.models[0];
    assert!(model.is_static);
    assert_eq!(model.links.len(), 1);
    let link = &model.links[0];
    assert_eq!(link.inertial.mass, 1.);
    assert_eq!(link.inertial.inertia, [1., 1., 1., 0., 0., 0.]);
    assert_eq!(link.inertial.pose, Pose::default());
    assert_eq!(
        link.collisions[0].geometry,
        sdformat::Geometry::Sphere { radius: 0.5 }
    );
    assert!(link.collisions[0].density.is_none());
    for source in [
        &world.source,
        &model.source,
        &link.source,
        &link.collisions[0].source,
    ] {
        assert_eq!(source.uri, uri);
        assert_eq!(source.line, 1);
        assert!(source.column > 0);
    }
    for engine in [
        PhysicsEngine::Tpe,
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        let mut sim = load_world_with_sdf_step(xml, uri, "default", &NoResources, engine).unwrap();
        let ids = sim
            .world()
            .iter_entities()
            .filter_map(|entity| {
                Some((
                    entity.get::<Name>()?.0.clone(),
                    entity.get::<EntityId>()?.get(),
                ))
            })
            .collect::<BTreeMap<_, _>>();
        assert_eq!(
            ids.keys().map(String::as_str).collect::<Vec<_>>(),
            [
                "default",
                "default::lamp",
                "default::robot",
                "default::robot::body",
                "default::robot::body::shape"
            ]
        );
        let body = ids["default::robot::body"];
        let light = ids["default::lamp"];
        for cycle in 0..2 {
            let entity = entity_by_id(sim.world(), light).unwrap();
            assert_eq!(
                sim.world().get::<EntityKind>(entity),
                Some(&EntityKind::Light)
            );
            assert_eq!(
                sim.world().get::<WorldLight>(entity).unwrap().0.as_ref(),
                &expected_light
            );
            assert_eq!(
                sim.world().get::<ChildOf>(entity).unwrap().parent(),
                entity_by_id(sim.world(), ids["default"]).unwrap()
            );
            assert_eq!(
                sim.world()
                    .iter_entities()
                    .filter(|e| e.contains::<WorldLight>())
                    .count(),
                1
            );
            if cycle == 0 {
                sim.world_mut()
                    .get_mut::<WorldLight>(entity)
                    .unwrap()
                    .0
                    .intensity = 7.;
            } else {
                sim.world_mut().entity_mut(entity).remove::<WorldLight>();
            }
            assert!(matches!(
                queue_link_world_wrench(&mut sim, body, [1., 0., 0.], [0., 0., 1.]),
                Err(InputError::Unsupported("fixed-body input"))
            ));
            sim.step_exact(1).unwrap();
            for id in [body, light] {
                let e = entity_by_id(sim.world(), id).unwrap();
                let pose = sim.world().get::<WorldPose>(e).unwrap();
                assert_eq!(pose.translation, [0.; 3]);
                assert_eq!(pose.rotation, [0., 0., 0., 1.]);
            }
            let velocity = sim
                .world()
                .get::<Velocity>(entity_by_id(sim.world(), body).unwrap())
                .unwrap();
            assert_eq!(velocity.linear, [0.; 3]);
            assert_eq!(velocity.angular, [0.; 3]);
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(sim.elapsed(), Duration::ZERO);
            for (name, id) in &ids {
                assert_eq!(
                    sim.world()
                        .get::<Name>(entity_by_id(sim.world(), *id).unwrap())
                        .unwrap()
                        .0,
                    *name
                );
            }
        }
        assert_eq!(
            sim.world()
                .get::<WorldLight>(entity_by_id(sim.world(), light).unwrap())
                .unwrap()
                .0
                .as_ref(),
            &expected_light
        );
    }
}
