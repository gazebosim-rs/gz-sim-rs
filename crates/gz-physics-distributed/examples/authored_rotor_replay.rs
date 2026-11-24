// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record an unmodified, authored Gazebo model and its physical promotion timestamp.
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, model_fidelity,
    reset_loaded_world,
};
use serde_json::json;
use std::{error::Error, fs, path::PathBuf};
const SDF: &str = include_str!("../../../examples/multifidelity_authored_rotor.sdf");
fn id(s: &Simulation, name: &str) -> u64 {
    s.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(std::env::args_os().nth(1).ok_or("need output directory")?);
    let sdf = sdformat::load_str(SDF, "authored.sdf", &sdformat::NoResources)?;
    let mut s = load_world_with_sdf_step(
        SDF,
        "authored.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let model = id(&s, "default::rotor");
    let labels = [
        "default::rotor::base",
        "default::rotor::tip",
        "default::stop::head",
    ];
    let ids = labels.map(|n| id(&s, n));
    let mut frames = Vec::new();
    let mut out = Vec::new();
    let mut promotion = None;
    for epoch in 0..2 {
        if epoch == 1 {
            reset_loaded_world(&mut s, |_| ())?;
        }
        for tick in 0..=120 {
            if tick > 0 {
                s.step_exact(1)?;
            }
            let dynamic = model_fidelity(&s, model)? == ArticulationTier::Dynamic;
            if dynamic && promotion.is_none() {
                promotion = Some(tick);
            }
            let backend = s.world().resource::<PhysicsBackend>();
            let poses:Vec<_>=ids.iter().map(|&i|{
                let e=entity_by_id(s.world(),i).unwrap();
                let v=backend.state(s.world().get::<BodyHandle>(e).unwrap().id()).unwrap();
                json!({"position":v.position,"rotation":v.orientation,"velocity":v.velocity,"spin":v.angular_velocity})
            }).collect();
            let row = json!({"tick":tick,"dynamic":dynamic,"bodies":poses});
            if epoch == 0 {
                out.push(row);
                frames.push(gz_sim_cli::primitive_frame(&s, &sdf, "default", tick + 1)?);
            } else if out[tick as usize] != row {
                return Err(format!("reset mismatch at {tick}").into());
            }
        }
    }
    fs::create_dir(&dir)?;
    gz_sim_cli::export_archive(&dir.join("replay"), Default::default(), frames)?;
    fs::write(
        dir.join("index.json"),
        serde_json::to_vec(
            &json!({"frames":out,"reset_frames":121,"step_ns":s.step_duration().as_nanos(),"labels":labels,"promotion_tick":promotion.ok_or("no promotion")?}),
        )?,
    )?;
    Ok(())
}
