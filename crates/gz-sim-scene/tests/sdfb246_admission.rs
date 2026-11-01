// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Supplemental consumer qualification; no upstream original completion credit.
//! TPE retains its kinematic/wrench refusal; density metadata never recalculates inertia.
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    CollisionDensity, InputError, LoadError, Name, PhysicsEngine, Velocity, WorldPose,
    load_world_with_sdf_step, queue_link_world_wrench, remove_model, reset_loaded_world,
};
use sdformat::{ErrorCode, Inertial, NoResources, Resource, ResourceResolver};
use std::time::Duration;

const FIXTURE: &str = include_str!("fixtures/sdfb246_auto_inertia.sdf");
const ENGINES: [PhysicsEngine; 4] = [
    PhysicsEngine::Tpe,
    PhysicsEngine::Rapier,
    PhysicsEngine::Dart,
    PhysicsEngine::Mujoco,
];

fn single_model(name: &str) -> String {
    let document = roxmltree::Document::parse(FIXTURE).unwrap();
    let other = document
        .descendants()
        .find(|n| n.has_tag_name("model") && n.attribute("name") != Some(name))
        .unwrap();
    FIXTURE.replace(&FIXTURE[other.range()], "")
}
fn inertial(xml: &str) -> Inertial {
    sdformat::load_str(xml, "computed.sdf", &NoResources)
        .unwrap()
        .worlds[0]
        .models[0]
        .links[0]
        .inertial
        .clone()
}
fn manual_counterpart(xml: &str, value: &Inertial) -> String {
    let fields = ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]
        .into_iter()
        .zip(value.inertia)
        .map(|(name, v)| format!("<{name}>{v}</{name}>"))
        .collect::<String>();
    xml.replace(
        "<inertial auto='true'/>",
        &format!(
            "<inertial><mass>{}</mass><inertia>{fields}</inertia></inertial>",
            value.mass
        ),
    )
    .replace("<density>1240</density>", "")
    .replace("<density>2170</density>", "")
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn velocity(sim: &Simulation, id: u64) -> Velocity {
    *sim.world()
        .iter_entities()
        .find(|e| e.get::<EntityId>().is_some_and(|v| v.get() == id))
        .unwrap()
        .get::<Velocity>()
        .unwrap()
}
fn density(sim: &Simulation, name: &str) -> Option<f64> {
    let collision = id(sim, &format!("default::{name}::body::collision"));
    let entity = gz_sim_core::entity_by_id(sim.world(), collision).unwrap();
    sim.world()
        .get::<CollisionDensity>(entity)
        .map(|value| value.0)
}
fn exercise(
    sim: &mut Simulation,
    engine: PhysicsEngine,
    name: &str,
    value: &Inertial,
    expected_density: Option<f64>,
) {
    let body = id(sim, &format!("default::{name}::body"));
    let collision = id(sim, &format!("default::{name}::body::collision"));
    assert_eq!(density(sim, name), expected_density);
    for cycle in 0..2 {
        // Runtime metadata edits have no physical effect and reset restores input.
        let entity = gz_sim_core::entity_by_id(sim.world(), collision).unwrap();
        if cycle == 0 {
            sim.world_mut()
                .entity_mut(entity)
                .insert(CollisionDensity(99.));
            assert_eq!(density(sim, name), Some(99.));
        } else {
            sim.world_mut()
                .entity_mut(entity)
                .remove::<CollisionDensity>();
            assert_eq!(density(sim, name), None);
        }
        let result =
            queue_link_world_wrench(sim, body, [value.mass, 0., 0.], [0., 0., value.inertia[2]]);
        let expected = if engine == PhysicsEngine::Tpe {
            assert!(matches!(
                result,
                Err(InputError::Unsupported("TPE external wrench"))
            ));
            0.
        } else {
            result.unwrap();
            0.001
        };
        sim.step_exact(1).unwrap();
        let v = velocity(sim, body);
        assert!(
            (v.linear[0] - expected).abs() < 1e-12,
            "{engine:?} {name} cycle{cycle}: {:?}",
            v.linear
        );
        assert!(
            (v.angular[2] - expected).abs() < 1e-12,
            "{engine:?} {name} cycle{cycle}: {:?}",
            v.angular
        );
        assert_eq!(sim.elapsed(), Duration::from_millis(1));
        reset_loaded_world(sim, |_| ()).unwrap();
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(
            id(sim, &format!("default::{name}::body::collision")),
            collision
        );
        assert_eq!(density(sim, name), expected_density);
        assert_eq!(id(sim, &format!("default::{name}::body")), body);
        assert_eq!(velocity(sim, body).linear, [0.; 3]);
        assert_eq!(velocity(sim, body).angular, [0.; 3]);
    }
}
#[test]
fn manual_counterparts_prove_existing_lowering_and_reset_on_all_backends() {
    for name in ["box", "sphere"] {
        let xml = single_model(name);
        let expected = inertial(&xml);
        let manual = manual_counterpart(&xml, &expected);
        let parsed = inertial(&manual);
        assert_eq!(parsed.mass.to_bits(), expected.mass.to_bits());
        assert_eq!(
            parsed.inertia.map(f64::to_bits),
            expected.inertia.map(f64::to_bits)
        );
        for engine in ENGINES {
            let mut sim =
                load_world_with_sdf_step(&manual, "manual.sdf", "default", &NoResources, engine)
                    .unwrap();
            exercise(&mut sim, engine, name, &expected, None);
        }
    }
}
fn automatic_all_backends(name: &str) {
    let xml = single_model(name);
    let expected = inertial(&xml);
    if name == "box" {
        assert_eq!(expected.mass, 9920.);
    } else {
        assert_eq!(expected.mass.to_bits(), 4621306650028725639);
        assert_eq!(
            expected.inertia.map(f64::to_bits),
            [
                4585400661777136886,
                4585400661777136886,
                4585400661777136886,
                0,
                0,
                0
            ]
        );
    }
    let mut failures = Vec::new();
    for engine in ENGINES {
        match load_world_with_sdf_step(&xml, "automatic.sdf", "default", &NoResources, engine) {
            Ok(mut sim) => exercise(
                &mut sim,
                engine,
                name,
                &expected,
                Some(if name == "box" { 1240. } else { 2170. }),
            ),
            Err(error) => failures.push(format!("{engine:?}: {error}")),
        }
    }
    assert!(failures.is_empty(), "{failures:?}");
}
#[test]
fn automatic_box_uses_the_same_backend_mass_inertia_and_reset() {
    automatic_all_backends("box");
}
#[test]
fn automatic_sphere_uses_the_same_backend_mass_inertia_and_reset() {
    automatic_all_backends("sphere");
}

#[test]
fn explicit_manual_density_is_metadata_and_absent_density_stays_absent() {
    for name in ["box", "sphere"] {
        let base = single_model(name);
        let value = Inertial {
            mass: 2.,
            inertia: [0.2, 0.2, 0.2, 0., 0., 0.],
            ..Inertial::default()
        };
        let manual = manual_counterpart(&base, &value);
        for attribute in ["", " auto='false'", " auto='0'"] {
            let xml = manual
                .replace("<inertial>", &format!("<inertial{attribute}>"))
                .replace("<geometry>", "<density>1000</density><geometry>");
            assert_eq!(inertial(&xml).mass, 2.);
            assert_eq!(inertial(&xml).inertia, value.inertia);
            for engine in ENGINES {
                let mut sim = load_world_with_sdf_step(
                    &xml,
                    "manual-density.sdf",
                    "default",
                    &NoResources,
                    engine,
                )
                .unwrap();
                exercise(&mut sim, engine, name, &value, Some(1000.));
            }
        }
    }
}

#[test]
fn static_density_is_retained_without_changing_fixed_body_policy() {
    for name in ["box", "sphere"] {
        let xml = single_model(name).replace("<static>false</static>", "<static>true</static>");
        let expected = if name == "box" { 1240. } else { 2170. };
        for engine in ENGINES {
            let mut sim =
                load_world_with_sdf_step(&xml, "static.sdf", "default", &NoResources, engine)
                    .unwrap();
            let body = id(&sim, &format!("default::{name}::body"));
            let entity = gz_sim_core::entity_by_id(sim.world(), body).unwrap();
            let pose = *sim.world().get::<WorldPose>(entity).unwrap();
            assert_eq!(density(&sim, name), Some(expected));
            assert!(matches!(
                queue_link_world_wrench(&mut sim, body, [1., 0., 0.], [0., 0., 1.]),
                Err(InputError::Unsupported("fixed-body input"))
            ));
            sim.step_exact(1).unwrap();
            let current = sim.world().get::<WorldPose>(entity).unwrap();
            assert_eq!(current.translation, pose.translation);
            assert_eq!(current.rotation, pose.rotation);
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(density(&sim, name), Some(expected));
            assert_eq!(velocity(&sim, body).linear, [0.; 3]);
        }
    }
}

#[test]
fn density_has_collision_ownership_and_loaded_reset_restores_removed_models() {
    let xml=single_model("sphere").replace("</link>","<visual name='visual'><geometry><sphere><radius>0.1</radius></sphere></geometry></visual></link>");
    for engine in ENGINES {
        let mut sim =
            load_world_with_sdf_step(&xml, "ownership.sdf", "default", &NoResources, engine)
                .unwrap();
        let owned = sim
            .world()
            .iter_entities()
            .filter(|entity| entity.contains::<CollisionDensity>())
            .collect::<Vec<_>>();
        assert_eq!(owned.len(), 1);
        assert_eq!(
            owned[0].get::<Name>().unwrap().0,
            "default::sphere::body::collision"
        );
        let collision = id(&sim, "default::sphere::body::collision");
        let model = id(&sim, "default::sphere");
        let retired = remove_model(&mut sim, model).unwrap();
        assert!(retired.contains(&collision));
        assert!(gz_sim_core::entity_by_id(sim.world(), collision).is_none());
        assert!(
            !sim.world()
                .iter_entities()
                .any(|entity| entity.contains::<CollisionDensity>())
        );
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(id(&sim, "default::sphere::body::collision"), collision);
        assert_eq!(density(&sim, "sphere"), Some(2170.));
        exercise(&mut sim, engine, "sphere", &inertial(&xml), Some(2170.));
    }
}

struct Imported;
impl ResourceResolver for Imported {
    fn resolve(&self, uri: &str, parent: &str) -> Result<Resource, String> {
        assert_eq!(parent, "included-world.sdf");
        assert!(matches!(uri, "box" | "sphere"));
        let xml = single_model(uri);
        let document = roxmltree::Document::parse(&xml).unwrap();
        let model = document
            .descendants()
            .find(|n| n.has_tag_name("model"))
            .unwrap();
        Ok(Resource {
            canonical_uri: format!("resources/{uri}.sdf"),
            xml: format!("<sdf version='1.12'>{}</sdf>", &xml[model.range()]),
        })
    }
}
#[test]
fn included_density_maps_original_resource_to_the_renamed_collision_identity() {
    let xml = "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><include><uri>box</uri><name>included_box</name></include><include><uri>sphere</uri><name>included_sphere</name></include></world></sdf>";
    let parsed = sdformat::load_str(xml, "included-world.sdf", &Imported).unwrap();
    for model in &parsed.worlds[0].models {
        let original = model.name.strip_prefix("included_").unwrap();
        let source = &model.links[0].collisions[0].source;
        assert_eq!(source.uri, format!("resources/{original}.sdf"));
        assert!(source.line > 0 && source.column > 0);
        assert_eq!(
            model.links[0].inertial.mass.to_bits(),
            inertial(&single_model(original)).mass.to_bits()
        );
    }
    for engine in ENGINES {
        let mut sim =
            load_world_with_sdf_step(xml, "included-world.sdf", "default", &Imported, engine)
                .unwrap();
        assert_eq!(density(&sim, "included_box"), Some(1240.));
        assert_eq!(density(&sim, "included_sphere"), Some(2170.));
        sim.step_exact(1).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(density(&sim, "included_box"), Some(1240.));
        assert_eq!(density(&sim, "included_sphere"), Some(2170.));
    }
}

#[test]
fn producer_numeric_and_structural_refusals_precede_backend_admission() {
    let base = single_model("sphere");
    let cases = [
        (
            base.replace("<density>2170</density>", ""),
            ErrorCode::Unsupported,
        ),
        (
            base.replace("2170</density>", "0</density>"),
            ErrorCode::InvalidValue,
        ),
        (
            base.replace("2170</density>", "-1</density>"),
            ErrorCode::InvalidValue,
        ),
        (
            base.replace("2170</density>", "inf</density>"),
            ErrorCode::InvalidValue,
        ),
        (
            base.replace(
                "<collision name='collision'>",
                "<collision name='collision'><pose>0 0 0 0 0 0</pose>",
            ),
            ErrorCode::Unsupported,
        ),
        (
            base.replace(
                "<inertial auto='true'/>",
                "<inertial auto='true'><mass>1</mass></inertial>",
            ),
            ErrorCode::Unsupported,
        ),
        (
            base.replace("2170</density>", "2.2250738585072014e-308</density>")
                .replace("0.1</radius>", "0.5</radius>"),
            ErrorCode::Unsupported,
        ),
    ];
    for (xml, expected) in cases {
        for engine in ENGINES {
            let error =
                load_world_with_sdf_step(&xml, "rejected.sdf", "default", &NoResources, engine)
                    .err()
                    .unwrap();
            assert!(
                matches!(error,LoadError::Sdf(ref value) if value.code==expected && value.source.uri=="rejected.sdf"),
                "{error}"
            );
        }
    }
    let manual = manual_counterpart(&base, &Inertial::default())
        .replace("<geometry>", "<density>-1</density><geometry>");
    assert!(sdformat::load_str(&manual, "manual-negative.sdf", &NoResources).is_ok());
    for engine in ENGINES {
        assert!(matches!(
            load_world_with_sdf_step(
                &manual,
                "manual-negative.sdf",
                "default",
                &NoResources,
                engine
            ),
            Err(LoadError::Unsupported {
                feature: "collision density",
                ..
            })
        ));
    }
}

#[test]
fn parsed_small_sphere_keeps_the_existing_backend_minimum_policy() {
    let xml = single_model("sphere")
        .replace("2170</density>", "1</density>")
        .replace("0.1</radius>", "0.0001</radius>");
    let value = inertial(&xml);
    assert!(value.mass > 1e-15 && value.inertia[0] > 0. && value.inertia[0] < 1e-15);
    for engine in ENGINES {
        let result = load_world_with_sdf_step(&xml, "small.sdf", "default", &NoResources, engine);
        if matches!(engine, PhysicsEngine::Dart | PhysicsEngine::Mujoco) {
            assert!(
                matches!(
                    result,
                    Err(LoadError::Physics(gz_physics::PhysicsError::InvalidValue))
                ),
                "{engine:?}"
            );
        } else {
            result.unwrap().step_exact(1).unwrap();
        }
    }
}

#[test]
fn manual_density_also_preserves_other_existing_shape_and_plane_policies() {
    let base = single_model("sphere");
    let manual = manual_counterpart(&base, &Inertial::default());
    let sphere = "<sphere><radius>0.1</radius></sphere>";
    for geometry in [
        "<cylinder><radius>0.1</radius><length>1</length></cylinder>",
        "<capsule><radius>0.1</radius><length>1</length></capsule>",
        "<plane><normal>0 0 1</normal><size>2 2</size></plane>",
    ] {
        let plane = geometry.starts_with("<plane>");
        let mut xml = manual
            .replace(sphere, geometry)
            .replace("<geometry>", "<density>1000</density><geometry>");
        if plane {
            xml = xml.replace("<static>false</static>", "<static>true</static>");
        }
        for engine in ENGINES {
            let result =
                load_world_with_sdf_step(&xml, "manual-shape.sdf", "default", &NoResources, engine);
            if plane && engine == PhysicsEngine::Tpe {
                assert!(matches!(
                    result,
                    Err(LoadError::Unsupported {
                        feature: "TPE plane collision",
                        ..
                    })
                ));
                continue;
            }
            if plane && matches!(engine, PhysicsEngine::Rapier | PhysicsEngine::Mujoco) {
                let expected = if engine == PhysicsEngine::Rapier {
                    "rapier"
                } else {
                    "mujoco"
                };
                assert!(
                    matches!(result, Err(LoadError::Physics(gz_physics::PhysicsError::Unsupported { engine: actual, capability: "static plane collider" })) if actual == expected)
                );
                continue;
            }
            let mut sim = result.unwrap();
            assert_eq!(density(&sim, "sphere"), Some(1000.));
            if !plane {
                exercise(
                    &mut sim,
                    engine,
                    "sphere",
                    &Inertial::default(),
                    Some(1000.),
                );
            }
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(density(&sim, "sphere"), Some(1000.));
        }
    }
}

#[test]
fn automatic_geometry_uses_the_producers_existing_normalized_values() {
    for name in ["box", "sphere"] {
        let xml = single_model(name)
            .replace("<size>2 2 2</size>", "<size>0 2 2</size>")
            .replace("<radius>0.1</radius>", "<radius>0</radius>");
        let parsed = sdformat::load_str(&xml, "normalized.sdf", &NoResources).unwrap();
        let link = &parsed.worlds[0].models[0].links[0];
        let geometry = &link.collisions[0].geometry;
        if name == "box" {
            assert_eq!(*geometry, sdformat::Geometry::Box { size: [1.; 3] });
        } else {
            assert_eq!(*geometry, sdformat::Geometry::Sphere { radius: 1. });
        }
        for engine in ENGINES {
            let mut sim =
                load_world_with_sdf_step(&xml, "normalized.sdf", "default", &NoResources, engine)
                    .unwrap();
            exercise(
                &mut sim,
                engine,
                name,
                &link.inertial,
                link.collisions[0].density,
            );
        }
    }
}
