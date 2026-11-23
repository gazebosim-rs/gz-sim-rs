// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, LoadError, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    reset_loaded_world,
};
use std::time::Duration;

const SDF: &str = include_str!("../../../examples/multifidelity_curved_shuttle.sdf");
fn load(source: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        source,
        "shuttle.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
fn body(sim: &Simulation, model: &str) -> (BodyId, BodyState) {
    let name = format!("default::{model}::body");
    let link = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    let id = link.get::<BodyHandle>().unwrap().id();
    let state = sim.world().resource::<PhysicsBackend>().state(id).unwrap();
    assert_eq!(link.get::<WorldPose>().unwrap().translation, state.position);
    assert_eq!(link.get::<Velocity>().unwrap().linear, state.velocity);
    (id, state)
}
/// Independent Hermite basis over the authoring knots, including the off-grid corners.
fn analytical(ns: u64) -> ([f64; 3], [f64; 3]) {
    let knots = [
        (0, [0.; 3], [0.; 3]),
        (205000000, [0.; 3], [0.; 3]),
        (1400250000, [0.55, 0.2, 0.25], [0.05, 0.2, 0.]),
        (2200350000, [0.45, 0.44, 0.2], [-0.3, 0., 0.]),
        (3600000000, [0.; 3], [0.; 3]),
    ];
    let n = ns % 3600000000;
    let span = knots.windows(2).find(|pair| n < pair[1].0).unwrap();
    let [(t, a, v), (end, b, w)] = [span[0], span[1]];
    let h = (end - t) as f64 * 1e-9;
    let u = (n - t) as f64 * 1e-9 / h;
    let p = std::array::from_fn(|i| {
        (1. - 3. * u * u + 2. * u * u * u) * a[i]
            + (3. * u * u - 2. * u * u * u) * b[i]
            + h * (u * (1. - u) * (1. - u) * v[i] + u * u * (u - 1.) * w[i])
    });
    let d = std::array::from_fn(|i| {
        6. * u * (1. - u) * (b[i] - a[i]) / h
            + (1. - 4. * u + 3. * u * u) * v[i]
            + (3. * u * u - 2. * u) * w[i]
    });
    (p, d)
}

#[test]
fn a_loaded_shuttle_transfers_load_and_matches_analytical_path_through_reset() {
    let mut sim = load(SDF).unwrap();
    let mut first = Vec::new();
    for epoch in 0..2 {
        if epoch != 0 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        let mut total = 0.;
        let mut supported = 0;
        for i in 1..=750 {
            let before = body(&sim, "cargo").1;
            sim.step_exact(1).unwrap();
            let ns = i * 5_000_000;
            let (p, v) = analytical(ns);
            let (_, deck) = body(&sim, "platform");
            let (_, cargo) = body(&sim, "cargo");
            for axis in 0..3 {
                assert!((deck.position[axis] - p[axis] - [0., 0., 0.1][axis]).abs() < 2e-14);
                assert!((deck.velocity[axis] - v[axis]).abs() < 3e-14);
            }
            let backend = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            let (substeps, hits) = backend.translation_interval();
            assert_eq!(substeps, if matches!(i, 281 | 441) { 11 } else { 10 });
            let normal = hits.iter().map(|c| c.normal_impulse).sum::<f64>();
            let expected = 1.2 * (cargo.velocity[2] - before.velocity[2] + 0.005 * 9.81);
            assert!(
                (normal - expected).abs() < 2e-7,
                "tick {i} normal {normal} momentum {expected}"
            );
            if i > 40 {
                assert!((cargo.position[2] - deck.position[2] - 0.2).abs() < 0.003);
                assert!(normal > 0.);
                supported += 1;
            }
            total += normal;
            let sample = (deck, cargo, normal.to_bits());
            if epoch == 0 {
                first.push(sample);
            } else {
                assert_eq!(sample, first[(i - 1) as usize]);
            }
        }
        assert_eq!(supported, 710);
        assert!((total - 1.2 * (9.81 * 3.75 + body(&sim, "cargo").1.velocity[2])).abs() < 1e-5);
        let start = sim.elapsed();
        let prev = body(&sim, "platform");
        sim.set_paused(true);
        assert!(sim.tick().unwrap().dt.is_zero());
        assert_eq!(sim.elapsed(), start);
        assert_eq!(body(&sim, "platform"), prev);
    }
}

#[test]
fn a_bad_trajectory_fails_during_sdf_admission() {
    for (a, b) in [
        ("205000000", "0"),
        ("repeat", "wrap"),
        ("0.55 0.2 0.25", "nan 0 0"),
        (
            "<trajectory ",
            "<linear_velocity>0 0 0</linear_velocity><trajectory ",
        ),
        (
            "3600000000\" displacement=\"0 0 0",
            "3600000000\" displacement=\"1 0 0",
        ),
        ("0.05 0.2 0", "700 0 0"),
    ] {
        let input = SDF.replace(a, b);
        assert!(
            matches!(load(&input), Err(LoadError::TranslationPath { .. })),
            "{b}"
        );
    }
}

#[test]
fn valid_edits_override_one_epoch_and_rejected_steps_or_edits_preserve_the_path() {
    let mut sim = load(SDF).unwrap();
    let (id, _) = body(&sim, "platform");
    sim.step_exact(62).unwrap();
    let old = body(&sim, "platform").1;
    let mut physics = sim.world_mut().resource_mut::<PhysicsBackend>();
    let (_, cargo) = physics
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .translation_interval();
    let old_impulse = cargo[0].normal_impulse;
    let backend = physics.downcast_mut::<MultiFidelityBackend>().unwrap();
    use gz_physics::DynamicBackend;
    assert!(backend.step(Duration::from_secs(10)).is_err());
    let mut invalid = old;
    invalid.velocity[0] = f64::NAN;
    assert!(backend.set_body_state(id, invalid).is_err());
    assert_eq!(backend.state(id).unwrap(), old);
    assert!(backend.translation_sample(id).is_some());
    assert_eq!(
        backend.translation_interval().1[0].normal_impulse,
        old_impulse
    );
    let mut changed = old;
    changed.velocity = [0.07, -0.02, 0.];
    backend.set_body_state(id, changed).unwrap();
    assert!(backend.translation_sample(id).is_none());
    sim.step_exact(1).unwrap();
    let after = body(&sim, "platform").1;
    assert_eq!(after.velocity, changed.velocity);
    for i in 0..3 {
        assert!((after.position[i] - old.position[i] - 0.005 * after.velocity[i]).abs() < 3e-14);
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(62).unwrap();
    assert_eq!(body(&sim, "platform").1, old);
}
