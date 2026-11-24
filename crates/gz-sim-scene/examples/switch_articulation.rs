// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! `cargo run --locked -p gz-sim-scene --example switch_articulation`
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, JointSample, Name, PhysicsEngine, articulation_root_pose,
    load_world_with_sdf_step, model_fidelity, queue_joint_effort, queue_link_world_wrench,
    set_model_fidelity,
};
use sdformat::NoResources;
use std::{error::Error, time::Duration};

fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|n| n.0 == name))
        .expect("the bundled example declares this entity")
        .get::<EntityId>()
        .unwrap()
        .get()
}

fn report(
    sim: &Simulation,
    model: u64,
    hinge: u64,
    tier: ArticulationTier,
    t_ms: u64,
) -> Result<(), Box<dyn Error>> {
    assert_eq!(id(sim, "default::robot"), model);
    assert_eq!(id(sim, "default::robot::hinge"), hinge);
    assert_eq!(sim.elapsed(), Duration::from_millis(t_ms));
    assert_eq!(model_fidelity(sim, model)?, tier);
    let root = articulation_root_pose(sim, model)?;
    let entity = entity_by_id(sim.world(), hinge).expect("the hinge retains its public ID");
    let sample = sim
        .world()
        .get::<JointSample>(entity)
        .expect("loaded hinge observation");
    assert_eq!(sample.time, sim.elapsed());
    println!(
        "t={t_ms} ms tier={tier:?} model={model} hinge={hinge} root_xyz={:?} root_xyzw={:?} q={:.8} rad qdot={:.8} rad/s",
        root.translation, root.rotation, sample.state.position, sample.state.velocity
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut sim = load_world_with_sdf_step(
        include_str!("../../../examples/multifidelity_articulation.sdf"),
        "multifidelity_articulation.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let model = id(&sim, "default::robot");
    let hinge = id(&sim, "default::robot::hinge");
    report(&sim, model, hinge, ArticulationTier::Dynamic, 0)?;
    // Give each mass in this fixture the same acceleration for one millisecond.
    // The assembly translates without exciting its hinge.
    for (name, mass) in [("default::robot::base", 2.0), ("default::robot::tool", 0.8)] {
        let link = id(&sim, name);
        queue_link_world_wrench(&mut sim, link, [mass * 100.0, 0.0, 0.0], [0.0; 3])?;
    }
    sim.step_exact(1)?;
    report(&sim, model, hinge, ArticulationTier::Dynamic, 1)?;
    set_model_fidelity(&mut sim, model, ArticulationTier::Prescribed)?;
    report(&sim, model, hinge, ArticulationTier::Prescribed, 1)?;
    let start = articulation_root_pose(&sim, model)?.translation[0];
    sim.step_exact(10)?;
    report(&sim, model, hinge, ArticulationTier::Prescribed, 11)?;
    assert!((articulation_root_pose(&sim, model)?.translation[0] - start - 0.001).abs() < 1e-9);
    set_model_fidelity(&mut sim, model, ArticulationTier::Dynamic)?;
    report(&sim, model, hinge, ArticulationTier::Dynamic, 11)?;
    queue_joint_effort(&mut sim, hinge, 4.0)?;
    sim.step_exact(10)?;
    report(&sim, model, hinge, ArticulationTier::Dynamic, 21)
}
