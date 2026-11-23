// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Full-readback validation and timing for 2,048 independent mixed cells.
//! Run in release mode with `--adapter <Vulkan device name> [--capture <new directory>]`.
#[path = "support/observations.rs"]
mod observations;
#[path = "support/reference.rs"]
mod reference;
#[path = "support/replay.rs"]
mod replay;
#[path = "support/scalar.rs"]
mod scalar;

use gz_physics_gpu::{
    GpuBatch, Lane,
    simulation::{BatchTiming, batch_simulation},
};
use gz_sim_core::Simulation;
use observations::{Agreement, Times, fingerprint};
use reference::{DT, Reference, fixture};
use std::{
    error::Error,
    hint::black_box,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const COUNT: usize = 2048;
const TICKS: usize = 1000;
const WARMUP: usize = 100;

fn parse() -> Result<(String, Option<PathBuf>)> {
    let mut args = std::env::args().skip(1);
    let (mut adapter, mut capture) = (None, None);
    while let Some(flag) = args.next() {
        match flag.as_str() {
            "--adapter" if adapter.is_none() => {
                adapter = Some(args.next().ok_or("missing adapter name")?)
            }
            "--capture" if capture.is_none() => {
                capture = Some(PathBuf::from(
                    args.next().ok_or("missing capture directory")?,
                ))
            }
            _ => return Err("expected --adapter NAME [--capture NEW_DIRECTORY]".into()),
        }
    }
    if capture.as_deref().is_some_and(Path::exists) {
        return Err("capture destination already exists".into());
    }
    Ok((
        adapter.ok_or("use --adapter NAME to select the physical Vulkan device")?,
        capture,
    ))
}

fn batch(sim: &Simulation) -> &GpuBatch {
    sim.world().resource::<GpuBatch>()
}

fn verify(sim: &mut Simulation, initial: &[Lane], capture_path: Option<&Path>) -> Result<()> {
    let mut reference = Reference::new(initial)?;
    let mut errors = Agreement::new(initial.len());
    let mut scalar_errors = Agreement::new(initial.len());
    let mut scalar_lanes = initial.to_vec();
    let mut hashes = Vec::with_capacity(TICKS);
    let mut capture = capture_path.map(|_| replay::Capture::default());
    if let Some(c) = &mut capture {
        c.sample(0, batch(sim).lanes())?;
    }
    let init_hash = fingerprint(initial);
    sim.set_paused(true);
    sim.tick()?;
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(batch(sim).tick(), 0);
    assert_eq!(fingerprint(batch(sim).lanes()), init_hash);
    assert!(sim.world().resource::<BatchTiming>().last.is_none());
    for tick in 1..=TICKS {
        reference.step()?;
        sim.step_exact(1)?;
        let gpu = batch(sim);
        assert_eq!(sim.elapsed(), DT * tick as u32);
        assert_eq!(gpu.sim_time(), sim.elapsed());
        assert_eq!(gpu.tick(), tick as u64);
        errors.compare(tick, gpu.lanes(), &reference.lanes);
        scalar::step(&mut scalar_lanes);
        scalar_errors.compare(tick, &scalar_lanes, &reference.lanes);
        hashes.push((fingerprint(gpu.lanes()), fingerprint(&reference.lanes)));
        if tick % 20 == 0
            && let Some(c) = &mut capture
        {
            c.sample(u64::try_from(sim.elapsed().as_nanos())?, gpu.lanes())?;
        }
    }
    errors.verify(batch(sim).lanes(), "specialized GPU");
    scalar_errors.verify(&scalar_lanes, "specialized serial CPU");
    sim.reset()?;
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(fingerprint(batch(sim).lanes()), init_hash);
    assert!(sim.world().resource::<BatchTiming>().last.is_none());
    let mut reference = Reference::new(initial)?;
    for (i, (expected_gpu, expected_cpu)) in hashes.into_iter().enumerate() {
        reference.step()?;
        sim.step_exact(1)?;
        assert_eq!(
            fingerprint(batch(sim).lanes()),
            expected_gpu,
            "GPU repeat at tick {}",
            i + 1
        );
        assert_eq!(
            fingerprint(&reference.lanes),
            expected_cpu,
            "CPU repeat at tick {}",
            i + 1
        );
    }
    println!(
        "dense agreement over {} lane-ticks; independently repeated both solvers with matching per-tick full-state fingerprints",
        initial.len() * TICKS
    );
    if let (Some(c), Some(path)) = (capture, capture_path) {
        c.write(path)?;
    }
    Ok(())
}

fn performance(sim: &mut Simulation, initial: &[Lane]) -> Result<()> {
    // Separate CPU and GPU passes use identical seeds, warmup and timed ticks.
    // Consume every state on both paths, including its complete material tuple.
    let mut cpu = Reference::new(initial)?;
    let mut cpu_times = Times::default();
    let mut cpu_obs = Times::default();
    for tick in 1..=TICKS {
        let start = Instant::now();
        cpu.step()?;
        let obs = Instant::now();
        black_box(fingerprint(&cpu.lanes));
        let read = obs.elapsed();
        if tick > WARMUP {
            cpu_times.add(start.elapsed());
            cpu_obs.add(read);
        }
    }
    let mut scalar_lanes = initial.to_vec();
    let mut scalar_times = Times::default();
    let mut scalar_observe = Times::default();
    for tick in 1..=TICKS {
        let start = Instant::now();
        scalar::step(&mut scalar_lanes);
        let obs = Instant::now();
        black_box(fingerprint(&scalar_lanes));
        let observed = obs.elapsed();
        if tick > WARMUP {
            scalar_times.add(start.elapsed());
            scalar_observe.add(observed);
        }
    }
    let mut phases: [Times; 10] = Default::default();
    let mut gpu_times = Times::default();
    let (mut uploaded, mut readback) = (0, 0);
    sim.reset()?;
    let mut cold = None;
    for tick in 1..=TICKS {
        let start = Instant::now();
        sim.step_exact(1)?;
        let obs = Instant::now();
        black_box(fingerprint(batch(sim).lanes()));
        let observe = obs.elapsed();
        let end_to_end = start.elapsed();
        let m = sim
            .world()
            .resource::<BatchTiming>()
            .last
            .ok_or("missing step metrics")?;
        if tick == 1 {
            cold = Some((end_to_end, m));
        }
        if tick <= WARMUP {
            continue;
        }
        gpu_times.add(end_to_end);
        let bevy = end_to_end.saturating_sub(m.latency + observe);
        for (s, t) in phases.iter_mut().zip([
            Some(m.pack),
            Some(m.upload),
            Some(m.encode_submit),
            Some(m.wait_map),
            Some(m.decode),
            Some(m.latency),
            m.gpu_dispatch,
            m.gpu_copy,
            Some(observe),
            Some(bevy),
        ]) {
            if let Some(t) = t {
                s.add(t);
            }
        }
        uploaded += m.uploaded_bytes;
        readback += m.readback_bytes;
    }
    let samples = TICKS - WARMUP;
    let lanes = (initial.len() * samples) as f64;
    let cpu_rate = lanes / cpu_times.total();
    let gpu_rate = lanes / gpu_times.total();
    let scalar_rate = lanes / scalar_times.total();
    println!(
        "timed ticks {samples} after {WARMUP} warmup ticks, {} environments with full observations",
        initial.len()
    );
    let (cold_time, cold_metrics) = cold.expect("non-empty benchmark window");
    println!(
        "first reseed including host observation {:.1} us; pack {:.3} us; upload {:.1} us ({} B); readback {} B",
        cold_time.as_secs_f64() * 1e6,
        cold_metrics.pack.as_secs_f64() * 1e6,
        cold_metrics.upload.as_secs_f64() * 1e6,
        cold_metrics.uploaded_bytes,
        cold_metrics.readback_bytes
    );
    match batch(sim).allocator_usage() {
        Some((allocated, reserved)) => println!(
            "Vulkan allocator after run (outside timing): allocated {allocated} B, reserved {reserved} B"
        ),
        None => println!("Vulkan allocator totals unavailable"),
    }
    println!(
        "resident transfers over {samples} ticks: upload {uploaded} B, readback {readback} B ({} B/tick)",
        readback / samples as u64
    );
    for (n, s) in [
        "pack",
        "upload",
        "encode+submit",
        "wait+map",
        "decode/validate",
        "GPU host step",
        "device dispatch",
        "device copy",
        "host dense observation",
        "Bevy/clock overhead",
    ]
    .into_iter()
    .zip(&mut phases)
    {
        s.report(n);
    }
    cpu_obs.report("CPU host dense observation");
    cpu_times.report("CPU reference end-to-end");
    scalar_observe.report("serial specialized CPU dense observation");
    scalar_times.report("serial specialized CPU end-to-end");
    gpu_times.report("GPU Bevy end-to-end");
    println!(
        "environment-steps/s: production general CPU {cpu_rate:.0}, specialized serial CPU {scalar_rate:.0}, specialized GPU {gpu_rate:.0}; GPU/general CPU {:.3}, GPU/specialized CPU {:.3}; aggregate simulated seconds/wall second: general CPU {:.1}, specialized CPU {:.1}, GPU {:.1}",
        gpu_rate / cpu_rate,
        gpu_rate / scalar_rate,
        cpu_rate * DT.as_secs_f64(),
        scalar_rate * DT.as_secs_f64(),
        gpu_rate * DT.as_secs_f64()
    );
    Ok(())
}

fn main() -> Result<()> {
    let (adapter, path) = parse()?;
    let initial = fixture(COUNT);
    let mut sim = batch_simulation(&adapter, initial.clone())?;
    println!("Vulkan batch on {:?}", batch(&sim).device_info());
    verify(&mut sim, &initial, path.as_deref())?;
    performance(&mut sim, &initial)
}
