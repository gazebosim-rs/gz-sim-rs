// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record the authored finite-pair world, including the native impulse from inside each interval.
use gz_sim_scene::{
    PhysicsEngine, WorldLoadOptions, gpu_scene::resident_pair_observation, load_world_on_adapter,
    reset_loaded_world,
};
use serde_json::json;
use std::{error::Error, fs, path::PathBuf};
const SDF: &str = include_str!("../../../examples/gpu_resident_contacts.sdf");
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("need a physical Vulkan adapter name")?;
    let out = PathBuf::from(
        args.next()
            .ok_or("need a new directory for the recording")?,
    );
    let sdf = sdformat::load_str(SDF, "gpu_resident_contacts.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_on_adapter(
        SDF,
        "gpu_resident_contacts.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(&adapter),
    )?;
    let mut frames = Vec::new();
    let mut cuts = Vec::new();
    let mut impact = None;
    for epoch in 0..=1 {
        if epoch != 0 {
            reset_loaded_world(&mut sim, |_| ())?;
        }
        for barrier in 0..=80 {
            if barrier > 0 {
                sim.step_exact(1)?;
            }
            let mut pairs = Vec::new();
            for (id, name) in [(7, "impact"), (41, "clear")] {
                let r = resident_pair_observation(&sim, id)?;
                if id == 41 && r.contact_ticks() != 0 {
                    return Err("clear pair produced an impulse".into());
                }
                if let Some((t, _)) = r.contact_times() {
                    if epoch == 0 {
                        if impact.is_some() {
                            return Err("unexpected repeated impact".into());
                        }
                        impact = Some((
                            barrier,
                            sim.elapsed().as_secs_f64() - sim.step_duration().as_secs_f64() + t,
                        ));
                    }
                }
                pairs.push(json!({"name":name,"id":id,"state":r.state,"impulse_on_b":r.impulse_on_b,"contact_ticks":r.contact_ticks(),"contact_times":r.contact_times()}));
            }
            let cut = json!({"tick":barrier,"pairs":pairs});
            if epoch == 0 {
                frames.push(gz_sim_cli::primitive_frame(
                    &sim,
                    &sdf,
                    "default",
                    barrier + 1,
                )?);
                cuts.push(cut);
            } else if cuts[barrier as usize] != cut {
                return Err(format!("reset changed physical state at barrier {barrier}").into());
            }
        }
    }
    fs::create_dir(&out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
    let (barrier, seconds) = impact.ok_or("no physical impact")?;
    fs::write(
        out.join("index.json"),
        serde_json::to_vec(
            &json!({"adapter":adapter,"step_ns":sim.step_duration().as_nanos(),"first_impact_barrier":barrier,"impact_s":seconds,"reset_verified":true,"frames":cuts}),
        )?,
    )?;
    Ok(())
}
