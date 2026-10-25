// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_tpe::{Contacts, ModelBounds};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_engine, remove_model, reset_loaded_world};
use sdformat::NoResources;
use std::time::Duration;

fn xml(mode: u64) -> String {
    let x = match mode {
        1 => 3.0,
        2 => 3.001,
        10 => 1e308,
        _ => 1.0,
    };
    let ax = if mode == 10 { 1e308 } else { 0.0 };
    let a_static = mode == 3 || mode == 4;
    let b_static = mode == 3;
    let angle = if mode == 7 || mode == 8 || mode == 9 {
        0.7
    } else {
        0.0
    };
    let b_angle = if mode == 9 { 0.3 } else { 0.0 };
    let link_x = if mode == 9 { 0.5 } else { 0.0 };
    let shape = if mode == 8 {
        "<sphere><radius>1</radius></sphere>"
    } else {
        "<box><size>2 2 2</size></box>"
    };
    let second = if mode == 9 {
        "<link name='second'><pose>-3 1 0 0 0 0</pose><collision name='shape'><geometry><box><size>1 1 1</size></box></geometry></collision></link>"
    } else {
        ""
    };
    format!(
        "<sdf version='1.12'><world name='default'><model name='a'><static>{a_static}</static><pose>{ax} 0 0 0 0 0</pose><link name='body'><collision name='shape'><geometry><box><size>4 4 4</size></box></geometry></collision></link></model><model name='b'><static>{b_static}</static><pose>{x} 0 0 0 0 {b_angle}</pose><link name='body'><pose>{link_x} 0 0 0 0 {angle}</pose><collision name='shape'><geometry>{shape}</geometry></collision></link>{second}</model></world></sdf>"
    )
}
fn load(xml: &str) -> Simulation {
    load_world_with_engine(
        xml,
        "contacts.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap()
}
fn named(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}

#[test]
fn loaded_contacts_match_fresh_native_model_bounds_and_filtering() {
    let proof: serde_json::Value = serde_json::from_str(include_str!(
        "../../../planning/reference/tpe-contacts.json"
    ))
    .unwrap();
    for row in proof["rows"].as_array().unwrap() {
        let mode = row["mode"].as_u64().unwrap();
        let mut sim = load(&xml(mode));
        let a = named(&sim, "default::a");
        let b = named(&sim, "default::b");
        // Surface XML remains unsupported; exercise the engine's explicit mask API.
        if mode == 5 || mode == 6 {
            sim.world_mut().entity_mut(a).insert(
                ModelBounds::new([-2.0; 3], [2.0; 3], false, if mode == 5 { 1 } else { 3 })
                    .unwrap(),
            );
            sim.world_mut()
                .entity_mut(b)
                .insert(ModelBounds::new([-1.0; 3], [1.0; 3], false, 2).unwrap());
        }
        sim.step_exact(1).unwrap();
        let contacts = sim.world().resource::<Contacts>().as_slice();
        let native = row["contacts"].as_array().unwrap();
        assert_eq!(contacts.len(), native.len(), "{row}");
        for (contact, point) in contacts.iter().zip(native) {
            assert!(contact.models.contains(&a));
            assert!(contact.models.contains(&b));
            assert_ne!(contact.models[0], contact.models[1]);
            for (actual, expected) in contact.point.iter().zip(point.as_array().unwrap()) {
                assert!(
                    (actual - expected.as_f64().unwrap()).abs() < 1e-12,
                    "{row}: {contact:?}"
                );
            }
        }
    }
}

#[test]
fn loaded_reset_rebuilds_contacts_for_restored_model_identities() {
    let mut sim = load(&xml(0));
    let a = named(&sim, "default::a");
    let b = named(&sim, "default::b");
    let id = sim.world().get::<EntityId>(b).unwrap().get();
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().resource::<Contacts>().as_slice().len(), 1);
    remove_model(&mut sim, id).unwrap();
    sim.step_exact(1).unwrap();
    assert!(sim.world().resource::<Contacts>().as_slice().is_empty());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = named(&sim, "default::b");
    let restored_a = named(&sim, "default::a");
    assert_ne!(restored, b);
    assert_eq!(sim.world().get::<EntityId>(restored).unwrap().get(), id);
    let contact = sim.world().resource::<Contacts>().as_slice()[0];
    assert!(contact.models.contains(&restored));
    assert!(contact.models.contains(&restored_a));
    assert!(!contact.models.contains(&a));
    assert!(!contact.models.contains(&b));
    assert_eq!(contact.point, [1.0, 0.0, 0.0]);
}

#[test]
fn empty_static_models_have_no_bounds_or_contacts() {
    let mut sim = load(
        "<sdf version='1.12'><world name='default'><model name='empty'><static>true</static></model></world></sdf>",
    );
    sim.step_exact(1).unwrap();
    assert!(
        sim.world_mut()
            .query::<&ModelBounds>()
            .iter(sim.world())
            .next()
            .is_none()
    );
    assert!(sim.world().resource::<Contacts>().as_slice().is_empty());
}

#[test]
fn finite_source_shape_with_unrepresentable_derived_bounds_is_a_load_error() {
    let xml = xml(8).replace(
        "<radius>1</radius>",
        &format!("<radius>{}</radius>", f64::MAX),
    );
    assert!(matches!(
        load_world_with_engine(
            &xml,
            "overflow.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            PhysicsEngine::Tpe
        ),
        Err(gz_sim_scene::LoadError::TpeContact(
            gz_physics_tpe::ContactError::InvalidBounds
        ))
    ));
}
