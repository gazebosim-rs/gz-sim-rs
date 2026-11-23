// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure the closed-form loss of contact of a curved translating guide inside one public tick.
use gz_physics::PhysicsBackend;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldLoadOptions, deformable, load_world_configured,
    reset_loaded_world,
};
use serde_json::json;
use std::{
    error::Error,
    fs,
    path::PathBuf,
    time::{Duration, Instant},
};
const SDF: &str = include_str!("../../../examples/multifidelity_curved_cable_guide.sdf");
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args_os().skip(1);
    let out = PathBuf::from(
        args.next()
            .ok_or("curved_cable_guide OUTPUT_DIR PUBLIC_STEP_NS")?,
    );
    let ns: u64 = args
        .next()
        .ok_or("expected public step in ns")?
        .to_str()
        .ok_or("invalid step")?
        .parse()?;
    if !(50_000..=100_000_000).contains(&ns) || 100_000_000 % ns != 0 || args.next().is_some() {
        return Err("choose a step from 50 µs to 100 ms that divides 100 ms".into());
    }
    let mut sim = load_world_configured(
        SDF,
        "curved_cable_guide.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            step: Some(Duration::from_nanos(ns)),
            engine: PhysicsEngine::MultiFidelity,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
    )?;
    let guide = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::guide::roller")
        })
        .and_then(|e| e.get::<BodyHandle>())
        .ok_or("missing guide")?
        .id();
    let hit = (1. - 0.5f64.sqrt()) * 0.05;
    let incident = 4.8 - 96. * hit;
    let mut data = Vec::new();
    let mut first = None;
    for epoch in 0..2 {
        if epoch > 0 {
            reset_loaded_world(&mut sim, |_| ())?;
        }
        let mut dense = Vec::new();
        let mut timings = Vec::new();
        let mut impulse = 0.;
        let mut angular = 0.;
        let mut nsteps = 0;
        let mut loaded = Vec::new();
        for t in 0..=500_000_000 / ns {
            if t > 0 {
                let start = Instant::now();
                sim.step_exact(1)?;
                timings.push(start.elapsed().as_nanos() as u64);
            }
            let native = sim
                .world()
                .resource::<PhysicsBackend>()
                .state(guide)
                .unwrap();
            let soft = deformable::bodies(&sim).next().unwrap().1;
            let n = soft.last_path_substeps();
            nsteps += n as u64;
            let r = soft
                .last_boundary_impulses()
                .first()
                .copied()
                .unwrap_or_default();
            impulse += r.linear[2];
            angular += r.angular[0];
            if r.linear[2] < -1e-12 {
                loaded.push(t);
            }
            let x: Vec<_> = soft.particles().iter().map(|p| p.position).collect();
            let v: Vec<_> = soft.particles().iter().map(|p| p.velocity).collect();
            let momentum = soft
                .particles()
                .iter()
                .map(|p| p.velocity[2] / p.inverse_mass)
                .sum::<f64>();
            if (momentum + impulse).abs() > 1e-9 {
                return Err("material momentum and boundary impulse diverged".into());
            }
            dense.push(json!({"tick":t,"time_ns":t*ns,"guide":native.position,"orientation":native.orientation,"particles":x,"velocities":v,"reaction":r.linear,"moment_about_public_start":r.angular,"path_substeps":n}));
        }
        let p = deformable::bodies(&sim).next().unwrap().1.particles()[0];
        if epoch == 0 {
            first = Some(dense.clone());
        } else if first.as_ref() != Some(&dense) {
            return Err("loaded reset changed the measured trajectory".into());
        }
        timings.sort_unstable();
        let q = |v: usize| timings[(timings.len() * v).div_ceil(100) - 1];
        data.push(json!({"epoch":epoch,"step_ns":ns,"first_loaded_interval":loaded.first(),"last_loaded_interval":loaded.last(),"analytical_impact_s":hit,"analytical_departure_mps":incident,"measured_departure_mps":p.velocity[2],"final_lane_z":p.position[2],
            "analytical_final_z":0.65+incident*(0.5-hit),"normal_reaction_ns":impulse,"angular_reaction_x_nms":angular,"resolved_slices":nsteps,"p50_ns":q(50),"p95_ns":q(95)}));
    }
    fs::create_dir_all(&out)?;
    fs::write(out.join("summary.json"), serde_json::to_vec_pretty(&data)?)?;
    fs::write(out.join("dense.json"), serde_json::to_vec(&first.unwrap())?)?;
    println!("{}", serde_json::to_string(&data)?);
    Ok(())
}
