// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure a level, centered 2 kg foot landing on a translating deck in a loaded SDF.
use gz_physics::{BodyId, BodyState, DynamicBackend, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, primary_contact::last_refinement,
};
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf, time::Instant};
const LEVEL: &str = include_str!("../../../examples/multifidelity_level_face.sdf");
const SWEEPS: [usize; 5] = [1, 4, 8, 16, 32];

fn load(passes: usize) -> Result<Simulation, Box<dyn Error>> {
    Ok(load_world_with_sdf_step(
        &LEVEL.replace(
            "<relaxation_iterations>8",
            &format!("<relaxation_iterations>{passes}"),
        ),
        "level.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn binding(sim: &Simulation, name: &str) -> BodyId {
    #[allow(deprecated)]
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<BodyHandle>()
        .unwrap()
        .id()
}
fn state(b: BodyState) -> Value {
    json!({"position":b.position,"orientation":b.orientation,"velocity":b.velocity,"angular_velocity":b.angular_velocity})
}
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("usage: local_primary_level OUT")?,
    );
    fs::create_dir_all(dir.join("traces"))?;
    let mut summary = Vec::new();
    for passes in SWEEPS {
        let mut sim = load(passes)?;
        let [foot, upper, pad] = [
            "default::rocker::foot",
            "default::rocker::upper",
            "default::lift::pad",
        ]
        .map(|n| binding(&sim, n));
        let mut frames = vec![];
        for tick in 0..=30 {
            if tick > 0 {
                sim.step_exact(1)?;
            }
            let native = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            let f = native.state(foot).unwrap();
            let u = native.state(upper).unwrap();
            let p = native.state(pad).unwrap();
            let event = last_refinement(&sim).filter(|e| e.iteration == tick);
            let mut impulse = native
                .rigid_contact_observations()
                .iter()
                .find(|c| c.a == foot.min(pad) && c.b == foot.max(pad))
                .map_or(0., |c| c.normal_impulse);
            let n = event.map_or(u32::from(tick > 0), |r| r.interval.native_steps);
            if let Some(r) = event {
                let q = r.requested_pairs[0];
                impulse = q.normal_impulse;
                summary.push(json!({"relaxation":passes,"tick":tick,"elapsed_ns":sim.elapsed().as_nanos() as u64,
                    "first_positive_end_ns":q.first_positive_end.map(|t|t.as_nanos() as u64),"normal_impulse_ns":impulse,
                    "normal_impulse_error_ns":impulse-2.8,"spin_rad_s":glam::DVec3::from(f.angular_velocity).length(),
                    "normal_relative_velocity_m_s":f.velocity[2]-p.velocity[2],"last_native_normal_impulse_ns":q.last_native_normal_impulse}));
            }
            let normal = glam::DQuat::from_array(f.orientation).conjugate() * glam::DVec3::Z;
            let gap = f.position[2]
                - p.position[2]
                - 0.06
                - normal.abs().dot(glam::DVec3::new(0.18, 0.10, 0.06));
            frames.push(json!({"tick":tick,"time_us":tick*500,"native_steps":n,"solver_iterations":4,
                "relaxation_iterations":event.map(|_|passes),"selected":event.is_some(),"weak_start":false,"secondary_start":false,
                "foot":state(f),"upper":state(u),"pad":state(p),"normal_impulse_ns":impulse,"gap_m":gap}));
        }
        let trace = json!({"frames":frames,"geometry":{"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.33,0.18,0.06]}});
        fs::write(
            dir.join(format!("traces/relax{passes}.json")),
            serde_json::to_vec(&trace)?,
        )?;
    }
    let lanes=[1,8,32].map(|p|json!({"label":format!("relax{p}"),"name":format!("{p} relaxation passes"),"file":format!("traces/relax{p}.json")}));
    let manifest = json!({"title":"Authored level touchdown · unbiased relaxation passes","pair_totals_only":true,"lanes":lanes});
    fs::write(
        dir.join("comparison.json"),
        serde_json::to_vec_pretty(&manifest)?,
    )?;
    fs::write(
        dir.join("physics.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    let mut samples: Vec<Vec<u64>> = vec![Vec::new(); 5];
    // Interleave 64 independent first landings per pass count, including the full Bevy tick.
    for round in 0..65 {
        for offset in 0..5 {
            let i = (round + offset) % 5;
            let mut sim = load(SWEEPS[i])?;
            sim.step_exact(4)?;
            let start = Instant::now();
            sim.step_exact(1)?;
            let ns = start.elapsed().as_nanos() as u64;
            let event = last_refinement(&sim).ok_or("missing impact")?;
            if event.iteration != 5
                || event.requested_pairs[0].normal_impulse
                    != summary[i]["normal_impulse_ns"].as_f64().unwrap()
            {
                return Err("timed landing differs from measured SDF run".into());
            }
            if round > 0 {
                samples[i].push(ns);
            }
        }
    }
    fs::write(dir.join("cost.json"), serde_json::to_vec_pretty(&samples.iter().zip(SWEEPS).map(|(row,r)|{
        let mut sorted=row.clone();sorted.sort();json!({"relaxation":r,"first_impact_p95_ns":sorted[(sorted.len()*95).div_ceil(100)-1],"samples_ns":row})
    }).collect::<Vec<_>>())?)?;
    Ok(())
}
