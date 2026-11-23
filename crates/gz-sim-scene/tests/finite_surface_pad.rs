// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{
    PhysicsEngine, deformable::bodies, load_world_with_sdf_step, reset_loaded_world,
};
use sdformat::NoResources;

const GARMENT: &str = include_str!("../../../examples/folded_surface_pad_transport.sdf");

#[test]
fn unsupported_surface_pad_scene_profiles_fail_at_load() {
    let cloth = &GARMENT[GARMENT.find("<cloth ").unwrap()..GARMENT.find("</cloth>").unwrap() + 8];
    let two_cloths = GARMENT.replace(
        cloth,
        &format!("{cloth}{}", cloth.replace("swatch", "copy")),
    );
    for xml in [
        GARMENT.replace("pad_contact=\"surface\"", "pad_contact=\"unknown\""),
        GARMENT.replace("self_contact=\"true\"", "self_contact=\"false\""),
        GARMENT.replacen("coupling=\"two_way\"", "coupling=\"one_way\"", 1),
        two_cloths,
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml,
                "pad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity,
            )
            .is_err()
        );
    }
}

#[test]
fn loaded_skin_returns_to_its_seed_and_reproduces_committed_particles_on_reset() {
    let mut sim = load_world_with_sdf_step(
        GARMENT,
        "pad.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let snapshot = |s: &gz_sim_core::Simulation| {
        let (_, cloth) = bodies(s).next().unwrap();
        (
            cloth.particles().to_vec(),
            cloth.last_boundary_impulses().to_vec(),
        )
    };
    let seed = snapshot(&sim);
    sim.tick().unwrap();
    assert_eq!(snapshot(&sim), seed);
    sim.step_exact(30).unwrap();
    let moved = snapshot(&sim);
    assert_ne!(moved.0, seed.0);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(snapshot(&sim), seed);
    sim.step_exact(30).unwrap();
    assert_eq!(snapshot(&sim), moved);
}
