// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::Simulation;
use gz_sim_scene::{Name, PhysicsEngine, WorldPose, load_world_with_sdf_step};
use sdformat::NoResources;

const LINKS: &str = "<link name='l0'><collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link><link name='l1'><pose>0.75 0 0 0 0 0</pose><collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link>";
fn load(models: &str) -> Simulation {
    let xml = format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    );
    load_world_with_sdf_step(
        &xml,
        "model-contacts.sdf",
        "w",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn x(sim: &Simulation, name: &str) -> f64 {
    sim.world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap()
        .get::<WorldPose>()
        .unwrap()
        .translation[0]
}
#[test]
fn native_default_top_level_model_excludes_contacts_without_joints() {
    let mut sim = load(&format!("<model name='a'>{LINKS}</model>"));
    sim.step_exact(100).unwrap();
    let reference: serde_json::Value = serde_json::from_str(include_str!(
        "../../../planning/reference/model-contacts.json"
    ))
    .unwrap();
    let native = &reference["observations"][0];
    assert_eq!(native["contacts"], 0);
    assert!((x(&sim, "w::a::l0") - native["first_x"].as_f64().unwrap()).abs() < 1e-12);
    assert!((x(&sim, "w::a::l1") - native["second_x"].as_f64().unwrap()).abs() < 1e-12);
}

#[test]
fn distinct_models_keep_the_native_positive_contact_control() {
    let models = "<model name='a'><link name='l0'><collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model><model name='b'><link name='l1'><pose>0.75 0 0 0 0 0</pose><collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model>";
    let mut sim = load(models);
    sim.step_exact(100).unwrap();
    let reference: serde_json::Value = serde_json::from_str(include_str!(
        "../../../planning/reference/model-contacts.json"
    ))
    .unwrap();
    assert!(reference["observations"][1]["contacts"].as_u64().unwrap() > 0);
    assert!(x(&sim, "w::a::l0") < -1e-7);
    assert!(x(&sim, "w::b::l1") > 0.7500001);
}

#[test]
fn independent_link_motion_and_loaded_lifecycle_preserve_contact_exclusion() {
    use gz_physics::PhysicsBackend;
    use gz_sim_core::EntityId;
    use gz_sim_scene::{BodyHandle, remove_model, reset_loaded_world};
    let mut sim = load(&format!("<model name='a'>{LINKS}</model>"));
    let id = |sim: &Simulation, name: &str| {
        sim.world()
            .iter_entities()
            .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get()
    };
    let model = id(&sim, "w::a");
    let first = id(&sim, "w::a::l0");
    let second = id(&sim, "w::a::l1");
    for (name, speed) in [("w::a::l0", 0.2), ("w::a::l1", -0.1)] {
        let body = sim
            .world()
            .iter_entities()
            .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
            .unwrap()
            .get::<BodyHandle>()
            .unwrap()
            .id();
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_world_velocity(body, [speed, 0.0, 0.0], [0.0; 3])
            .unwrap();
    }
    sim.step_exact(1000).unwrap();
    assert!((x(&sim, "w::a::l0") - 0.2).abs() < 1e-12);
    assert!((x(&sim, "w::a::l1") - 0.65).abs() < 1e-12);
    remove_model(&mut sim, model).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(id(&sim, "w::a::l0"), first);
    assert_eq!(id(&sim, "w::a::l1"), second);
    sim.step_exact(1000).unwrap();
    assert!((x(&sim, "w::a::l0")).abs() < 1e-12);
    assert!((x(&sim, "w::a::l1") - 0.75).abs() < 1e-12);
}
