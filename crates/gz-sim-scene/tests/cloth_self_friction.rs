// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{
    PhysicsEngine, deformable::bodies, load_world_with_sdf_step, reset_loaded_world,
};
use sdformat::NoResources;
const SHEET: &str = include_str!("../../../examples/sliding_folded_cloth.sdf");

#[test]
fn loaded_self_friction_replays_the_supported_contact_receipts_and_particles_after_reset() {
    let mut sim = load_world_with_sdf_step(
        SHEET,
        "sliding.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let collect = |sim: &mut gz_sim_core::Simulation| {
        (0..250)
            .map(|_| {
                sim.step_exact(1).unwrap();
                let (_, s) = bodies(sim).next().unwrap();
                (s.last_self_contact_impulse(), s.particles().to_vec())
            })
            .collect::<Vec<_>>()
    };
    let first = collect(&mut sim);
    assert!(
        first
            .iter()
            .filter(|(j, _)| j.tangent > 0. && j.friction_work > 0.)
            .count()
            > 10
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let seed = bodies(&sim).next().unwrap().1;
    assert_eq!(seed.last_self_contact_impulse(), Default::default());
    assert_eq!(first, collect(&mut sim));
}
#[test]
fn invalid_loaded_coefficient_or_missing_shell_fails_at_load() {
    for xml in [
        SHEET.replace("self_friction=\"0.55\"", "self_friction=\"-0.2\""),
        SHEET.replace("self_contact=\"true\"", "self_contact=\"false\""),
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml,
                "sliding.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}
