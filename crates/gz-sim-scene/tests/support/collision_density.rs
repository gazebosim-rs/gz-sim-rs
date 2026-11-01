// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_scene::{
    CollisionDensity, EntityKind, InputError, Name, PhysicsEngine, Velocity, WorldPose,
    load_world_with_sdf_step, queue_link_world_wrench, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

pub const ORIGINAL_STATIC_DENSITY_XML: &str = "<sdf version='1.12'><world name='default'><model name='robot' ><static>true</static><link name='body'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry><density>1000</density></collision></link></model></world></sdf>";

pub fn assert_static_density(xml: &str, uri: &str) {
    assert_eq!(xml, ORIGINAL_STATIC_DENSITY_XML);
    assert_eq!(uri, "sdf8");
    let parsed = sdformat::load_str(xml, uri, &NoResources).unwrap();
    let mut expected = sdformat::load_str(
        &xml.replace("<density>1000</density>", ""),
        uri,
        &NoResources,
    )
    .unwrap();
    expected.worlds[0].models[0].links[0].collisions[0].density = Some(1000.);
    assert_eq!(
        format!("{:?}", parsed.worlds),
        format!("{:?}", expected.worlds)
    );
    assert_eq!(parsed.version, "1.12");
    assert!(parsed.models.is_empty());
    let world = &parsed.worlds[0];
    assert_eq!(world.name, "default");
    assert_eq!(world.models.len(), 1);
    let model = &world.models[0];
    assert!(model.is_static);
    assert_eq!(model.links.len(), 1);
    let link = &model.links[0];
    assert_eq!(link.inertial.mass, 1.);
    assert_eq!(link.inertial.inertia, [1., 1., 1., 0., 0., 0.]);
    assert_eq!(link.inertial.pose, sdformat::Pose::default());
    assert_eq!(
        link.collisions[0].geometry,
        sdformat::Geometry::Sphere { radius: 0.5 }
    );
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
            .collect::<std::collections::BTreeMap<_, _>>();
        assert_eq!(ids.len(), 4);
        assert_eq!(
            ids.keys().map(String::as_str).collect::<Vec<_>>(),
            [
                "default",
                "default::robot",
                "default::robot::body",
                "default::robot::body::shape"
            ]
        );
        let collision = ids["default::robot::body::shape"];
        let body = ids["default::robot::body"];
        for cycle in 0..2 {
            let entity = entity_by_id(sim.world(), collision).unwrap();
            assert_eq!(
                sim.world().get::<EntityKind>(entity),
                Some(&EntityKind::Collision)
            );
            assert_eq!(
                sim.world().get::<CollisionDensity>(entity),
                Some(&CollisionDensity(1000.))
            );
            assert_eq!(
                sim.world()
                    .iter_entities()
                    .filter(|e| e.contains::<CollisionDensity>())
                    .count(),
                1
            );
            if cycle == 0 {
                sim.world_mut()
                    .entity_mut(entity)
                    .insert(CollisionDensity(7.));
            } else {
                sim.world_mut()
                    .entity_mut(entity)
                    .remove::<CollisionDensity>();
            }
            assert!(matches!(
                queue_link_world_wrench(&mut sim, body, [1., 0., 0.], [0., 0., 1.]),
                Err(InputError::Unsupported("fixed-body input"))
            ));
            sim.step_exact(1).unwrap();
            let entity = entity_by_id(sim.world(), body).unwrap();
            let pose = sim.world().get::<WorldPose>(entity).unwrap();
            assert_eq!(pose.translation, [0.; 3]);
            assert_eq!(pose.rotation, [0., 0., 0., 1.]);
            let velocity = sim.world().get::<Velocity>(entity).unwrap();
            assert_eq!(velocity.linear, [0.; 3]);
            assert_eq!(velocity.angular, [0.; 3]);
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(sim.elapsed(), Duration::ZERO);
            for (name, id) in &ids {
                let entity = entity_by_id(sim.world(), *id).unwrap();
                assert_eq!(sim.world().get::<Name>(entity).unwrap().0, *name);
            }
        }
        let entity = entity_by_id(sim.world(), collision).unwrap();
        assert_eq!(
            sim.world().get::<CollisionDensity>(entity),
            Some(&CollisionDensity(1000.))
        );
    }
}
