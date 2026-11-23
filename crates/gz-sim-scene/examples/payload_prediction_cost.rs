// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time the native prediction query against a stationary array of selected convex payloads.
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, set_model_fidelity,
};
use sdformat::NoResources;
use serde_json::json;
use std::{error::Error, hint::black_box, time::Instant};

fn main() -> Result<(), Box<dyn Error>> {
    let mut targets = String::new();
    let names: Vec<_> = (0..64).map(|i| format!("p{i}")).collect();
    for (i, name) in names.iter().enumerate() {
        // Boxes start one or more metres from either arm link.
        targets.push_str(&format!(r#"<model name="{name}"><pose>{} {} 0.17 0 0 0</pose><link name="body"><collision name="c"><geometry><box><size>0.3 0.25 0.3</size></box></geometry></collision></link></model>"#,1.4 + (i%8) as f64,1.2+(i/8) as f64));
    }
    let source = include_str!("../../../examples/multifidelity_sliding_payload.sdf")
        .replace("10 10 0.2", "30 30 0.2")
        .replace("</world>", &format!("{targets}</world>"));
    let mut sim = load_world_with_sdf_step(
        &source,
        "query.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    sim.step_exact(2000)?;
    let ids: std::collections::HashMap<_, _> = sim
        .world()
        .iter_entities()
        .filter_map(|e| Some((e.get::<Name>()?.0.clone(), e.get::<EntityId>()?.get())))
        .collect();
    set_model_fidelity(&mut sim, ids["default::arm"], ArticulationTier::Prescribed)?;
    let body = |name: &str| {
        sim.world()
            .get::<BodyHandle>(entity_by_id(sim.world(), ids[name]).unwrap())
            .unwrap()
            .id()
    };
    let root = body("default::arm::base");
    let obstacles: Vec<_> = names
        .iter()
        .map(|name| body(&format!("default::{name}::body")))
        .collect();
    let backend = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    let mut batches = Vec::new();
    for i in 0..=400 {
        let start = Instant::now();
        for &target in &obstacles {
            assert!(!black_box(backend).predicts_articulation_contact(
                root,
                black_box(target),
                sim.step_duration()
            )?);
        }
        if i > 0 {
            batches.push(start.elapsed().as_nanos() as u64);
        }
    }
    batches.sort_unstable();
    println!(
        "{}",
        json!({"targets":obstacles.len(),"batches":batches.len(),
        "median_batch_ns":batches[199], "p95_batch_ns":batches[379], "max_batch_ns":batches[399]})
    );
    Ok(())
}
