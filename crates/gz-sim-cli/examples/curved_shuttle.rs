// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record a Bevy-loaded cubic shuttle and the payload carried by friction.
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step};
use serde_json::json;
use std::{error::Error, fs, path::PathBuf, time::Instant};

const SDF: &str = include_str!("../../../examples/multifidelity_curved_shuttle.sdf");
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn links(sim: &Simulation) -> Result<[BodyId; 2]> {
    let get = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .and_then(|e| e.get::<BodyHandle>())
            .map(|h| h.id())
            .ok_or("missing link")
    };
    Ok([
        get("default::platform::body")?,
        get("default::cargo::body")?,
    ])
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let dir = PathBuf::from(args.next().ok_or("curved_shuttle OUTPUT_DIR [CYCLES]")?);
    let cycles = args
        .next()
        .map(|s| s.parse::<u32>())
        .transpose()?
        .unwrap_or(2);
    if cycles == 0 || cycles > 10000 || args.next().is_some() {
        return Err("expected 1 through 10000 cycles".into());
    }
    let sdf = sdformat::load_str(SDF, "shuttle.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        SDF,
        "shuttle.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let [deck, cargo] = links(&sim)?;
    let mut frames = Vec::new();
    let mut samples = Vec::new();
    let mut dense = Vec::new();
    let mut errors = [0f64; 3];
    let mut last_vz = 0.;
    let mut sums = [0.; 2];
    let mut first_impact = None;
    let mut max_load = 0f64;
    let mut substeps = 0;
    let mut times = Vec::new();
    for tick in 0..=cycles * 720 {
        if tick > 0 {
            let start = Instant::now();
            sim.step_exact(1)?;
            times.push(start.elapsed().as_nanos() as u64);
        }
        let res = sim.world().resource::<PhysicsBackend>();
        let backend = res
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("wrong engine")?;
        let sample = backend
            .translation_sample(deck)
            .ok_or("missing prescribed motion")?;
        let base = res.state(deck).ok_or("missing deck")?;
        let load = res.state(cargo).ok_or("missing cargo")?;
        let (count, hits) = backend.translation_interval();
        let normal = hits.iter().map(|c| c.normal_impulse).sum::<f64>();
        substeps += count;
        if tick > 0 {
            let expected = 1.2 * (load.velocity[2] - last_vz + 0.005 * 9.81);
            errors[0] = errors[0].max((normal - expected).abs());
            sums[0] += normal;
            if normal > 0. {
                first_impact.get_or_insert(tick);
            }
            if tick > 41 {
                max_load = max_load.max(normal / 0.005);
                let delta = load.position[2] - base.position[2] - 0.2;
                errors[1] = errors[1].max(delta.abs());
                sums[1] += f64::from(normal > 0.);
            }
        }
        last_vz = load.velocity[2];
        for (axis, x) in sample.displacement.into_iter().enumerate() {
            errors[2] = errors[2].max((base.position[axis] - [0., 0., 0.1][axis] - x).abs());
        }
        if tick <= 1440 {
            dense.push(json!({"tick":tick,"deck":base.position,"cargo":load.position,"cargo_orientation":load.orientation}));
        }
        // Export the first two complete laps. Qualification can run longer without storing extra frames.
        if tick <= 1440 && (tick % 6 == 0 || tick == first_impact.unwrap_or(u32::MAX)) {
            for (id, state) in [(deck, base), (cargo, load)] {
                let link = sim
                    .world()
                    .iter_entities()
                    .find(|e| e.get::<BodyHandle>().is_some_and(|h| h.id() == id))
                    .unwrap();
                assert_eq!(link.get::<WorldPose>().unwrap().translation, state.position);
            }
            let frame =
                gz_sim_cli::primitive_frame(&sim, &sdf, "default", frames.len() as u64 + 1)?;
            samples.push(json!({"tick":tick,"time_ns":sim.elapsed().as_nanos(),"deck":base.position,"cargo":load.position,
                "cargo_orientation":load.orientation,"cargo_velocity":load.velocity,"velocity":sample.velocity,"acceleration":sample.acceleration,
                "normal_impulse":normal,"normal_N":normal/0.005,"native_substeps":count}));
            frames.push(frame);
        }
    }
    if errors[0] > 2e-7
        || errors[1] > 0.003
        || errors[2] > 1e-13
        || sums[1] != (cycles * 720 - 41) as f64
    {
        return Err(format!("shuttle verification failed: {errors:?}, {sums:?}").into());
    }
    times.sort_unstable();
    let summary = json!({"cycles":cycles,"public_ticks":cycles*720,"native_substeps":substeps,"initial_impact_tick":first_impact,
        "normal_impulse_sum_Ns":sums[0],"supported_ticks_after_settling":sums[1],"max_settled_load_N":max_load,
        "max_impulse_minus_momentum_Ns":errors[0],"max_vertical_tracking_error_m":errors[1],"max_deck_projection_error_m":errors[2],
        "public_step_p50_us":times[times.len()/2]/1000,"public_step_p95_us":times[(times.len()*95).div_ceil(100)-1]/1000});
    fs::create_dir_all(&dir)?;
    gz_sim_cli::export_archive(&dir.join("replay"), Default::default(), frames)?;
    fs::write(dir.join("dense.json"), serde_json::to_vec(&dense)?)?;
    fs::write(
        dir.join("index.json"),
        serde_json::to_vec(&json!({"summary":summary,"frames":samples}))?,
    )?;
    println!("{summary}");
    Ok(())
}
