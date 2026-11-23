// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure the normal SDF loader and complete Bevy schedule for mixed GPU physics.
//! Run with a named physical Vulkan adapter and an output CSV, e.g.
//! cargo run --release -p gz-sim-scene --features gpu-batch --example gpu_sdf_batch -- "NVIDIA GeForce RTX 5090" samples.csv
use gz_physics_gpu::{Environment, MixedBatch, simulation::BatchTiming};
use gz_sim_scene::{
    PhysicsEngine, WorldLoadOptions, gpu_scene::GpuSceneObservations, load_world_on_adapter,
};
use sdformat::NoResources;
use std::{
    error::Error,
    fs::File,
    io::Write as _,
    path::Path,
    time::{Duration, Instant},
};

#[path = "support/gpu_sdf_fixture.rs"]
mod generated;
use generated::fixture;
const WARMUP: u64 = 25;
const SAMPLES: u64 = 200;
fn verify_sample(
    sample: &GpuSceneObservations,
    envs: &[Environment],
    tick: u64,
) -> Result<(), Box<dyn Error>> {
    if sample.tick() != tick
        || sample.sim_time() != Duration::from_millis(tick)
        || sample.environments() != envs
    {
        return Err("SDF subscription/public physics frame mismatch".into());
    }
    Ok(())
}
fn measure(adapter: &str, count: usize, every: u64, out: &mut File) -> Result<(), Box<dyn Error>> {
    let xml = fixture(count, every);
    let start = Instant::now();
    let mut sim = load_world_on_adapter(
        &xml,
        "generated_gpu_cells.sdf",
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
    let load_time = start.elapsed();
    let objects = sim.world().entities().len();
    let mut times = Vec::new();
    let mut gpu_times = Vec::new();
    let mut read_times = Vec::new();
    let mut sampled = 0;
    let mut bytes = 0;
    let mut uploads = 0;
    for tick in 1..=WARMUP + SAMPLES {
        let before = Instant::now();
        sim.step_exact(1)?;
        let wall = before.elapsed();
        let batch = sim.world().resource::<MixedBatch>();
        let timing = sim.world().resource::<BatchTiming>().last.as_ref().unwrap();
        if batch.tick() != tick || sim.elapsed() != Duration::from_millis(tick) {
            return Err("clock mismatch".into());
        }
        let sample = sim.world().get_resource::<GpuSceneObservations>();
        let due = every > 0 && tick % every == 0;
        if due {
            verify_sample(sample.unwrap(), batch.environments(), tick)?;
        }
        if every > 0 && sample.unwrap().tick() != tick - tick % every {
            return Err("unexpected sample cadence".into());
        }
        if every == 0 && sample.is_some() {
            return Err("unexpected observation resource".into());
        }
        if tick <= WARMUP {
            continue;
        }
        sampled += usize::from(due);
        times.push(wall);
        gpu_times.push(timing.latency);
        read_times.push(timing.wait_map + timing.decode);
        bytes += timing.readback_bytes;
        uploads += timing.uploaded_bytes;
        writeln!(
            out,
            "{count},{every},{tick},{},{},{},{},{},{},{},{}",
            u8::from(due),
            wall.as_nanos(),
            timing.latency.as_nanos(),
            timing.gpu_dispatch.map_or(0, |x| x.as_nanos()),
            timing.wait_map.as_nanos(),
            timing.decode.as_nanos(),
            timing.uploaded_bytes,
            timing.readback_bytes
        )?;
    }
    let total: Duration = times.iter().sum();
    let percentile = |v: &mut Vec<Duration>, p: f64| {
        v.sort_unstable();
        v[((v.len() as f64 * p).ceil() as usize) - 1].as_secs_f64() * 1000.
    };
    let p50 = percentile(&mut times, 0.50);
    let p99 = percentile(&mut times, 0.99);
    let native = percentile(&mut gpu_times, 0.5);
    let read = percentile(&mut read_times, 0.5);
    println!(
        "envs={count} sample_every={every} entities={objects} load_ms={:.1} measured={SAMPLES} samples={sampled} env_steps_s={:.0} wall_p50_ms={p50:.4} wall_p99_ms={p99:.4} backend_p50_ms={native:.4} wait_decode_p50_ms={read:.4} readback_B_tick={} resident_uploaded_B={uploads}",
        load_time.as_secs_f64() * 1000.,
        count as f64 * SAMPLES as f64 / total.as_secs_f64(),
        bytes / SAMPLES
    );
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args_os().skip(1);
    let adapter = args
        .next()
        .ok_or("supply an explicit hardware adapter")?
        .into_string()
        .map_err(|_| "adapter must be UTF-8")?;
    let path = args.next().ok_or("supply a new output CSV path")?;
    if args.next().is_some() {
        return Err("expected ADAPTER OUTPUT.csv".into());
    }
    let mut out = File::create_new(Path::new(&path))?;
    writeln!(
        out,
        "environments,sample_every,tick,sampled,wall_ns,backend_ns,dispatch_ns,wait_map_ns,decode_ns,uploaded_bytes,readback_bytes"
    )?;
    for size in [3, 128, 2048] {
        for every in [0, 1, 10] {
            measure(&adapter, size, every, &mut out)?;
        }
    }
    Ok(())
}
