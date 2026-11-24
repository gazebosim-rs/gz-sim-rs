// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time a complete, observed horizontal transfer using the ordinary SDF/Bevy route.
use gz_physics_gpu::simulation::BatchTiming;
use gz_sim_scene::{
    JointStateSample, PhysicsEngine, WorldLoadOptions, gpu_scene::parallel_gripper::carry,
    load_world_on_adapter, reset_loaded_world,
};
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path, time::Instant};
const SDF: &str = include_str!("../../../examples/gpu_carry_gripper.sdf");
fn measure(source: &str, id: &str, adapter: &str, ticks: usize) -> Result<Value, Box<dyn Error>> {
    let mut sim = load_world_on_adapter(
        source,
        "gpu_carry_gripper.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(adapter),
    )?;
    let mut baseline = Vec::new();
    let mut epochs = Vec::new();
    for epoch in 0..2 {
        if epoch > 0 {
            reset_loaded_world(&mut sim, |_| ()).map_err(|e| format!("reset: {e:?}"))?;
        }
        let model = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<gz_sim_scene::JointStatePublication>())
            .ok_or("missing publisher")?
            .id();
        let mut timings = Vec::with_capacity(ticks);
        let mut uploaded = 0u64;
        let mut read = 0u64;
        let start = Instant::now();
        for tick in 1..=ticks {
            let begin = Instant::now();
            sim.step_exact(1)?;
            let (l, r) = carry::observe(&sim, 12)?;
            let sample = sim
                .world()
                .get::<JointStateSample>(model)
                .ok_or("missing joint sample")?;
            assert_eq!(sample.time.as_millis(), tick as u128);
            assert_eq!(sample.joints.len(), 4);
            let frame = (
                l.track,
                l.track_speed,
                l.hand.position,
                l.hand.speed,
                l.hand.ball,
                l.hand.velocity,
                l.hand.spin,
                l.hand.normal_impulse,
                l.shelf_up,
                r,
            );
            if epoch == 0 {
                baseline.push(frame)
            } else {
                assert_eq!(baseline[tick - 1], frame)
            }
            let timing = sim.world().resource::<BatchTiming>().last.unwrap();
            uploaded += timing.uploaded_bytes;
            read += timing.readback_bytes;
            timings.push(begin.elapsed().as_secs_f64() * 1e6);
        }
        let duration = start.elapsed().as_secs_f64();
        timings.sort_by(f64::total_cmp);
        let (l, r) = carry::observe(&sim, 12)?;
        let percentile = |p: usize| timings[(ticks * p).div_ceil(100) - 1];
        epochs.push(json!({
            "epoch":epoch,"steps_per_sec":ticks as f64/duration,"public_p50_us":percentile(50),"public_p95_us":percentile(95),
            "public_p99_us":percentile(99),"upload_bytes":uploaded,"readback_bytes":read,"release_ms":r.map(|r|r.tick),
            "simulated_seconds_per_wall_second":ticks as f64*0.001/duration,"final_sphere":l.hand.ball
        }));
    }
    Ok(json!({"case":id,"epochs":epochs}))
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let [adapter, output, options @ ..] = args.as_slice() else {
        return Err("expected physical adapter name and output directory".into());
    };
    fs::create_dir_all(output)?;
    let mut rows = Vec::new();
    let eccentric = include_str!("../../../examples/gpu_eccentric_gripper.sdf");
    let (ticks, cases) = match options {
        [mode] if mode == "eccentric" => (
            1000,
            vec![
                ("heavy_right", eccentric.to_owned()),
                (
                    "slip",
                    eccentric.replace("<mu>0.6</mu><mu2>0.6", "<mu>0.12</mu><mu2>0.12"),
                ),
            ],
        ),
        [] => (
            2300,
            vec![
                ("80g", SDF.to_owned()),
                (
                    "200g",
                    SDF.replace("<mass>0.08</mass>", "<mass>0.2</mass>")
                        .replace("0.000015488", "0.00003872"),
                ),
                (
                    "frictionless",
                    SDF.replace("<mu>0.6</mu><mu2>0.6", "<mu>0</mu><mu2>0"),
                ),
                ("missed", SDF.replace("0.18 0 0.007", "0.34 0 0.007")),
                (
                    "wrong_floor",
                    SDF.replace("0.18 0 0.007", "0.34 0 0.007")
                        .replace("lift=\"0.014\"", "lift=\"-0.0025\""),
                ),
            ],
        ),
        _ => return Err("expected optional eccentric mode".into()),
    };
    for (name, s) in cases {
        rows.push(measure(&s, name, adapter, ticks)?);
    }
    let out = serde_json::to_vec_pretty(&rows)?;
    fs::write(Path::new(output).join("ordinary.json"), &out)?;
    println!("{}", String::from_utf8(out)?);
    Ok(())
}
