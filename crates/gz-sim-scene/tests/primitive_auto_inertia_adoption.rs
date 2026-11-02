// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Consumer checks against retained producer observations; no original-test credit.
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    CollisionDensity, InputError, Name, PhysicsEngine, Velocity, WorldPose,
    load_world_with_sdf_step, queue_link_world_wrench, remove_model, reset_loaded_world,
};
use sdformat::{Inertial, NoResources};
use serde_json::Value;
use std::time::Duration;

const NATIVE: &str = include_str!("fixtures/primitive_auto_inertia_native.json");
const ENGINES: [PhysicsEngine; 4] = [
    PhysicsEngine::Tpe,
    PhysicsEngine::Rapier,
    PhysicsEngine::Dart,
    PhysicsEngine::Mujoco,
];

fn entity(sim: &Simulation, name: &str) -> (bevy_ecs::entity::Entity, u64) {
    let found = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (found.id(), found.get::<EntityId>().unwrap().get())
}

fn native_inertial(case: &Value) -> (String, Inertial, Option<f64>) {
    let input = &case["case"];
    let xml = input["xml"].as_str().unwrap();
    let uri = input["uri"].as_str().unwrap();
    let native = &case["native"];
    for key in ["load_errors", "resolve_errors", "collision_errors"] {
        assert!(native[key].as_array().unwrap().is_empty());
    }
    let frames = native["root_values"].as_array().unwrap();
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0], frames[1]);
    let bits = frames[0].as_array().unwrap();
    assert_eq!(bits[0], 1);
    assert_eq!(bits[1], 1);
    let parsed = sdformat::load_str(xml, uri, &NoResources).unwrap();
    let link = &parsed.models[0].links[0];
    let value = link.inertial.clone();
    let density = link.collisions[0].density;
    for (actual, expected) in std::iter::once(value.mass)
        .chain(value.inertia)
        .zip(&bits[2..9])
    {
        let expected = expected.as_u64().unwrap();
        assert!(actual.is_finite());
        assert!(f64::from_bits(expected).is_finite());
        // This is the accepted producer's four-ULP arithmetic boundary.
        assert!(actual == f64::from_bits(expected) || actual.to_bits().abs_diff(expected) <= 4);
    }
    let doc = roxmltree::Document::parse(xml).unwrap();
    let density_nodes = doc
        .descendants()
        .filter(|node| node.has_tag_name("density"))
        .collect::<Vec<_>>();
    assert_eq!(density_nodes.len(), 1);
    let explicit_density = density_nodes[0]
        .text()
        .unwrap()
        .trim()
        .parse::<f64>()
        .unwrap();
    assert_eq!(density, Some(explicit_density));
    let model = doc
        .descendants()
        .find(|node| node.has_tag_name("model"))
        .unwrap();
    let world = format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity>\
         <physics name='step' type='ode'><max_step_size>0.001</max_step_size></physics>\
         {}</world></sdf>",
        &xml[model.range()]
    );
    (world, value, density)
}

#[derive(Debug, PartialEq)]
struct InitialSnapshot {
    ids: [u64; 3],
    poses: [([f64; 3], [f64; 4]); 3],
}

fn initial_snapshot(sim: &Simulation, density: Option<f64>) -> InitialSnapshot {
    let identities =
        ["default::m", "default::m::l", "default::m::l::c"].map(|name| entity(sim, name));
    let ids = identities.map(|(_, id)| id);
    let poses = identities.map(|(entity, _)| {
        let pose = sim.world().get::<WorldPose>(entity).unwrap();
        assert_eq!(pose.translation, [0.0; 3]);
        assert_eq!(pose.rotation, [0.0, 0.0, 0.0, 1.0]);
        (pose.translation, pose.rotation)
    });
    let velocity = sim.world().get::<Velocity>(identities[1].0).unwrap();
    assert_eq!(velocity.linear, [0.0; 3]);
    assert_eq!(velocity.angular, [0.0; 3]);
    assert_eq!(
        sim.world()
            .get::<CollisionDensity>(identities[2].0)
            .map(|v| v.0),
        density
    );
    assert_eq!(sim.elapsed(), Duration::ZERO);
    InitialSnapshot { ids, poses }
}

fn exercise(xml: &str, value: &Inertial, engine: PhysicsEngine, density: Option<f64>) {
    let mut sim =
        load_world_with_sdf_step(xml, "consumer.sdf", "default", &NoResources, engine).unwrap();
    let initial = initial_snapshot(&sim, density);
    let [model_id, body_id, _] = initial.ids;
    for _ in 0..2 {
        let (body, _) = entity(&sim, "default::m::l");
        let (collision, _) = entity(&sim, "default::m::l::c");
        let applied = queue_link_world_wrench(
            &mut sim,
            body_id,
            [value.mass, 0.0, 0.0],
            [0.0, 0.0, value.inertia[2]],
        );
        let expected = if engine == PhysicsEngine::Tpe {
            assert!(matches!(
                applied,
                Err(InputError::Unsupported("TPE external wrench"))
            ));
            0.0
        } else {
            applied.unwrap();
            0.001
        };
        sim.step_exact(1).unwrap();
        let velocity = sim.world().get::<Velocity>(body).unwrap();
        for (actual, expected) in velocity
            .linear
            .into_iter()
            .chain(velocity.angular)
            .zip([expected, 0.0, 0.0, 0.0, 0.0, expected])
        {
            assert!(
                (actual - expected).abs() < 1e-12,
                "{engine:?}: {actual} versus {expected}"
            );
        }
        assert_eq!(sim.elapsed(), Duration::from_millis(1));
        sim.world_mut()
            .entity_mut(collision)
            .remove::<CollisionDensity>();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(initial_snapshot(&sim, density), initial);
    }
    remove_model(&mut sim, model_id).unwrap();
    for retired in initial.ids {
        assert!(
            !sim.world()
                .iter_entities()
                .any(|e| e.get::<EntityId>().is_some_and(|v| v.get() == retired))
        );
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(initial_snapshot(&sim, density), initial);
}

#[test]
fn native_primitive_inertia_is_consumed_by_existing_backends_and_loaded_reset() {
    let fixture: Value = serde_json::from_str(NATIVE).unwrap();
    assert_eq!(
        fixture["producer_revision"],
        "a44bf8be09aff979449afddc8c75118c4ae1a227"
    );
    let cases = fixture["cases"].as_array().unwrap();
    assert_eq!(cases.len(), 2);
    assert_eq!(
        cases
            .iter()
            .map(|case| case["kind"].as_str().unwrap())
            .collect::<Vec<_>>(),
        ["cylinder", "capsule"]
    );
    for case in cases {
        let (xml, value, density) = native_inertial(case);
        let fields = ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]
            .into_iter()
            .zip(value.inertia)
            .map(|(name, v)| format!("<{name}>{v}</{name}>"))
            .collect::<String>();
        let manual = xml.replace(
            "<inertial auto='true'/>",
            &format!(
                "<inertial><mass>{}</mass><inertia>{fields}</inertia></inertial>",
                value.mass
            ),
        );
        assert_ne!(manual, xml);
        let parsed = sdformat::load_str(&manual, "manual.sdf", &NoResources).unwrap();
        let inertial = &parsed.worlds[0].models[0].links[0].inertial;
        assert_eq!(inertial.mass.to_bits(), value.mass.to_bits());
        assert_eq!(
            inertial.inertia.map(f64::to_bits),
            value.inertia.map(f64::to_bits)
        );
        for engine in ENGINES {
            exercise(&xml, &value, engine, density);
            exercise(&manual, &value, engine, density);
        }
    }
}
