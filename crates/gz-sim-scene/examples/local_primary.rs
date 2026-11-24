// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay-source oracle for the ordinary SDF/Bevy local primary contact scene.
use gz_sim_core::Simulation;
use gz_sim_scene::{
    EntityKind, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step,
    primary_contact::last_refinement,
};
use serde_json::{Value, json};
use std::{env, error::Error, fs};
const SCENE: &str = include_str!("../../../examples/multifidelity_local_primary.sdf");
fn pose_frame(sim: &Simulation) -> Value {
    #[allow(deprecated)]
    let links: std::collections::BTreeMap<_, _> = sim
        .world()
        .iter_entities()
        .filter_map(|e| {
            if !matches!(e.get::<EntityKind>()?, EntityKind::Link | EntityKind::Model) {
                return None;
            }
            let p = e.get::<WorldPose>()?;
            Some((
                &e.get::<Name>()?.0,
                json!({"translation":p.translation,"rotation":p.rotation}),
            ))
        })
        .collect();
    json!({"step":sim.world().resource::<gz_sim_core::UpdateInfo>().iteration,"time_ns":sim.elapsed().as_nanos() as u64,"poses":links})
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let out = args
        .next()
        .ok_or("usage: local_primary OUTPUT_DIRECTORY [SDF_PATH [STEPS]]")?;
    let scene = match args.next() {
        Some(path) => fs::read_to_string(path)?,
        None => SCENE.to_owned(),
    };
    let steps = args
        .next()
        .map(|s| s.to_string_lossy().parse::<usize>())
        .transpose()?
        .unwrap_or(400);
    if args.next().is_some() {
        return Err("unexpected local_primary argument".into());
    }
    let dir = std::path::PathBuf::from(out);
    fs::create_dir_all(&dir)?;
    let mut sim = load_world_with_sdf_step(
        &scene,
        "local_primary.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut poses = vec![pose_frame(&sim)];
    let mut events = vec![];
    for _ in 0..steps {
        sim.step_exact(1)?;
        poses.push(pose_frame(&sim));
        if let Some(e) = last_refinement(&sim)
            .filter(|e| e.iteration == sim.world().resource::<gz_sim_core::UpdateInfo>().iteration)
        {
            let p = e.requested_pairs[0];
            events.push(json!({"epoch":e.epoch,"step":e.iteration,"start_time_ns":e.start_time.as_nanos() as u64,
                "end_time_ns":e.sim_time.as_nanos() as u64,"native_steps":e.interval.native_steps,"foot_link":p.foot_link,
                "support_link":p.support_link,"normal_impulse_ns":p.normal_impulse,"last_native_normal_impulse_ns":p.last_native_normal_impulse,
                "normal_impulse_on_foot_ns":p.normal_impulse_on_foot,"tangent_impulse_on_foot_ns":p.tangent_impulse_on_foot,
                "first_positive_end_ns":p.first_positive_end.map(|t|t.as_nanos() as u64),"active_at_end":p.active_at_end}));
        }
    }
    if events.is_empty() {
        return Err("authored scene did not select a primary interval".into());
    }
    fs::write(dir.join("poses.json"), serde_json::to_vec(&poses)?)?;
    fs::write(dir.join("events.json"), serde_json::to_vec_pretty(&events)?)?;
    println!("{}", serde_json::to_string_pretty(&events)?);
    Ok(())
}
