// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! End-to-end placement cost in a full loaded SDF, including every ECS transform.
#[path = "support/gpu_sdf_fixture.rs"]
mod generated;
use gz_physics_gpu::{
    ExecutionPlacement::{self, Cpu, Gpu},
    MixedBatch,
    simulation::{BatchTiming, batch_boundary, set_environment_placement},
};
use gz_sim_scene::{
    PhysicsEngine, WorldLoadOptions, gpu_scene::GpuSceneObservations, load_world_on_adapter,
    reset_loaded_world,
};
use sdformat::NoResources;
use std::{
    error::Error,
    fs::File,
    io::Write,
    time::{Duration, Instant},
};
const WARM: u64 = 30;
const MEASURE: u64 = 200;
fn quantile(v: &mut [u128], p: f64) -> u128 {
    v.sort_unstable();
    v[(p * v.len() as f64).ceil() as usize - 1]
}
fn trial(adapter: &str, count: usize, repeat: usize, out: &mut File) -> Result<(), Box<dyn Error>> {
    let xml = generated::fixture(count, 10);
    let mut sim = load_world_on_adapter(
        &xml,
        "placed.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(adapter),
    )?;
    let ids: Vec<_> = (0..count as u64).collect();
    // Vary treatment order across independent full scene resets to expose
    // contention and the cost of opening a fresh device independently from switching.
    let order: &[usize] = match repeat {
        0 => &[100, 50, 0, 25, 75],
        1 => &[25, 0, 75, 100, 50],
        _ => &[75, 25, 50, 0, 100],
    };
    for &cpu_percent in order {
        reset_loaded_world(&mut sim, |_| ())?;
        let goal_cpu = ids
            .iter()
            .filter(|i| (**i as usize * 73 % 100) < cpu_percent)
            .copied()
            .collect::<Vec<_>>();
        let at = batch_boundary(&sim);
        let begin = Instant::now();
        let switch = set_environment_placement(&mut sim, at, &goal_cpu, Cpu)?;
        let switch_ns = begin.elapsed().as_nanos();
        let mut fit = vec![];
        let mut backend = vec![];
        let mut total = Duration::ZERO;
        let mut readback = 0u64;
        for tick in 1..=WARM + MEASURE {
            let t = Instant::now();
            sim.step_exact(1)?;
            let wall = t.elapsed();
            let d = sim.world().resource::<BatchTiming>().last.as_ref().unwrap();
            let obs = sim.world().resource::<GpuSceneObservations>();
            assert_eq!(obs.tick(), tick - tick % 10);
            let b = sim.world().resource::<MixedBatch>();
            assert_eq!(b.cpu_environments(), goal_cpu.len());
            if b.cpu_environments() == count {
                assert_eq!((d.uploaded_bytes, d.readback_bytes), (0, 0));
            }
            if tick > WARM {
                assert_eq!(d.uploaded_bytes, 0);
                fit.push(wall.as_nanos());
                backend.push(d.latency.as_nanos());
                total += wall;
                readback = d.readback_bytes;
            }
            writeln!(
                out,
                "{repeat},{count},{cpu_percent},{tick},{},{},{},{},{},{},{},{},{}",
                goal_cpu.len(),
                count - goal_cpu.len(),
                if tick == 1 { switch_ns } else { 0 },
                d.uploaded_bytes,
                d.readback_bytes,
                d.cpu_solve.as_nanos(),
                d.gpu_dispatch.map_or(0, |p| p.as_nanos()),
                d.latency.as_nanos(),
                wall.as_nanos()
            )?;
        }
        let p50 = quantile(&mut fit, 0.50);
        let p99 = quantile(&mut fit, 0.99);
        let solver = quantile(&mut backend, 0.50);
        // Transfer in both directions on a warm committed frame. A second call that
        // requests the same ownership at this barrier is an idempotent no-op.
        let at = batch_boundary(&sim);
        let start = Instant::now();
        let reverse = set_environment_placement(&mut sim, at, &ids, Gpu)?;
        let reverse_ns = start.elapsed().as_nanos();
        let unchanged = set_environment_placement(&mut sim, at, &ids, ExecutionPlacement::Gpu)?;
        assert_eq!(unchanged.moved, 0);
        sim.step_exact(1)?;
        let upload = sim
            .world()
            .resource::<BatchTiming>()
            .last
            .unwrap()
            .uploaded_bytes;
        println!(
            "repeat={repeat} environments={count} cpu={} gpu={} moved={} switch_us={:.2} gpu_relocations={} reverse_us={:.2} reverse_upload_B={upload} reads_B_tick={readback} env_steps_s={:.0} p50_us={:.2} p99_us={:.2} backend_p50_us={:.2}",
            goal_cpu.len(),
            count - goal_cpu.len(),
            switch.moved,
            switch_ns as f64 / 1e3,
            reverse.reseeded_gpu_rows,
            reverse_ns as f64 / 1e3,
            MEASURE as f64 * count as f64 / total.as_secs_f64(),
            p50 as f64 / 1e3,
            p99 as f64 / 1e3,
            solver as f64 / 1e3
        );
    }
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut a = std::env::args().skip(1);
    let device = a.next().ok_or("provide physical Vulkan adapter")?;
    let path = a.next().ok_or("provide a new CSV file")?;
    if a.next().is_some() {
        return Err("expected ADAPTER OUTPUT.csv".into());
    }
    let mut out = File::create_new(path)?;
    writeln!(
        out,
        "repeat,environments,target_cpu_percent,tick,cpu,gpu,switch_ns,uploaded_bytes,readback_bytes,cpu_solve_ns,gpu_dispatch_ns,backend_ns,whole_step_ns"
    )?;
    for repeat in 0..3 {
        for count in [3, 128, 2048] {
            trial(&device, count, repeat, &mut out)?;
        }
    }
    Ok(())
}
