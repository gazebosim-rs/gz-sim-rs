// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step, prepare_model_template,
    remove_model, reset_loaded_world, spawn_model_template,
};
use sdformat::NoResources;
const SDF: &str = include_str!("../../../examples/multifidelity_local_friction.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "friction.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn id(s: &Simulation, name: &str) -> u64 {
    #[allow(deprecated)]
    s.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn read(s: &Simulation, name: &str) -> BodyState {
    let e = entity_by_id(s.world(), id(s, name)).unwrap();
    let b = s.world().get::<BodyHandle>(e).unwrap().id();
    s.world().resource::<PhysicsBackend>().state(b).unwrap()
}
#[test]
fn material_coefficient_survives_plane_contact_and_loaded_reset() {
    let plane = SDF
        .replace(
            "<box><size>3.5 1.7 0.1</size></box>",
            "<plane><normal>0 0 1</normal><size>3.5 1.7</size></plane>",
        )
        .replace("0 0 -0.05 0 0 0", "0 0 0 0 0 0");
    for xml in [SDF, &plane] {
        let mut sim = load(xml);
        let mut results = vec![];
        for _ in 0..2 {
            sim.step_exact(100).unwrap();
            let states = [
                "default::ice::ball",
                "default::medium::ball",
                "default::grip::ball",
            ]
            .map(|n| read(&sim, n));
            for (s, mu) in states.into_iter().zip([0., 0.2, 0.6]) {
                assert!((s.velocity[0] - (2.1 - mu * 9.81 * 0.05)).abs() < 0.0001);
                assert!((s.angular_velocity[1] - mu * 9.81 * 0.05 * 25.).abs() < 0.003);
            }
            results.push(states);
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        assert_eq!(results[0], results[1]);
    }
}

#[test]
fn runtime_clone_preserves_the_lower_of_two_surface_coefficients() {
    let xml = SDF.replace("<mu>1</mu><mu2>1</mu2>", "<mu>0.07</mu><mu2>0.07</mu2>");
    let parsed = sdformat::load_str(&xml, "clone.sdf", &NoResources).unwrap();
    let mut s = load(&xml);
    let template = prepare_model_template(&s, &parsed, &parsed.worlds[0].models[0]).unwrap();
    let original = id(&s, "default::runway");
    let old_pose = *s
        .world()
        .get::<WorldPose>(entity_by_id(s.world(), original).unwrap())
        .unwrap();
    // Keep the template source available through clone publication. Removal
    // occurs before any solver step, so exactly one runway takes the load.
    let clone = spawn_model_template(&mut s, &template, old_pose).unwrap();
    assert_ne!(clone, original);
    remove_model(&mut s, original).unwrap();
    s.step_exact(100).unwrap();
    let expected = 2.1 - 0.07 * 9.81 * 0.05;
    for name in ["default::medium::ball", "default::grip::ball"] {
        assert!((read(&s, name).velocity[0] - expected).abs() < 1e-4);
    }
}

#[test]
fn a_free_sliding_articulation_uses_its_authored_sole_on_a_prescribed_pad() {
    let source = include_str!("../../../examples/multifidelity_level_face.sdf").replace(
        "<linear_velocity>0 0 -1.1</linear_velocity>",
        "<linear_velocity>1 0 -1.1</linear_velocity>",
    );
    let mut vx = vec![];
    for mu in [0., 0.3] {
        let xml=source.replace("</geometry></collision>",&format!("</geometry><surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>"));
        let mut sim = load(&xml);
        sim.step_exact(15).unwrap();
        let root = read(&sim, "default::rocker::foot");
        let upper = read(&sim, "default::rocker::upper");
        vx.push((root.velocity[0] + upper.velocity[0]) * 0.5);
    }
    assert!((vx[0] - 1.).abs() < 1e-8);
    assert!(vx[0] - vx[1] > 0.1);
}

#[test]
fn material_admission_preserves_other_surface_semantics() {
    for law in [
        "<ode><mu>0.1</mu><mu2>0.2</mu2></ode>",
        "<ode><slip1>0.2</slip1></ode>",
        "<ode><fdir1>0 1 0</fdir1></ode>",
        "<ode/><bullet/>",
        "<torsional/>",
    ] {
        let xml = SDF.replacen("<ode><mu>1</mu><mu2>1</mu2></ode>", law, 1);
        assert!(
            load_world_with_sdf_step(
                &xml,
                "material.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn omitted_sdf_surface_matches_explicit_friction_after_cloning_the_runway() {
    use gz_sim_scene::{WorldLoadOptions, WorldRuntimeOptions, load_world_with_runtime};
    let omitted = SDF.replace(
        "<surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface>",
        "",
    );
    assert_ne!(omitted, SDF);
    let run = |xml: &str, engine| {
        load_world_with_runtime(
            xml,
            "clone.sdf",
            "default",
            &NoResources,
            WorldLoadOptions {
                engine,
                step: None,
                noise_seed: [0; 32],
                distributed_worker: None,
            },
            WorldRuntimeOptions {
                sdf_default_friction: true,
                ..Default::default()
            },
        )
    };
    assert!(run(&omitted, PhysicsEngine::Rapier).is_err());
    let mut expected = load(SDF);
    let mut sim = run(&omitted, PhysicsEngine::MultiFidelity).unwrap();
    let parsed = sdformat::load_str(&omitted, "clone.sdf", &NoResources).unwrap();
    let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
    let runway = id(&sim, "default::runway");
    let pose = *sim
        .world()
        .get::<WorldPose>(entity_by_id(sim.world(), runway).unwrap())
        .unwrap();
    spawn_model_template(&mut sim, &template, pose).unwrap();
    remove_model(&mut sim, runway).unwrap();
    expected.step_exact(100).unwrap();
    sim.step_exact(100).unwrap();
    for name in [
        "default::ice::ball",
        "default::medium::ball",
        "default::grip::ball",
    ] {
        let dx = read(&sim, name).velocity[0] - read(&expected, name).velocity[0];
        assert!(dx.abs() < 1e-10, "{name}: {dx}");
    }
}
