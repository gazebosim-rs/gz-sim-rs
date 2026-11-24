// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify simultaneous scalar CPU and actual GPU execution for the same 2,048
//! independent mixed environments. Usage: execution_placement ADAPTER out.csv
#[path = "support/sphere_pair/mod.rs"]
pub mod pair;
#[path = "support/sphere_plane/mod.rs"]
pub mod plane;
pub use pair::reference;
#[path = "support/mixed/qualification.rs"]
mod qualification;
#[path = "support/supported_pair/fixture.rs"]
pub mod support;
use gz_physics_gpu::{
    Environment,
    ExecutionPlacement::{self, Cpu, Gpu},
    MixedBatch, STEP,
    simulation::{BatchTiming, batch_boundary, install_mixed, set_environment_placement},
};
use gz_sim_core::Simulation;
use std::{error::Error, fs::File, io::Write, time::Instant};
fn switches(tick: usize, initial: &[Environment]) -> Option<(ExecutionPlacement, Vec<u64>)> {
    let (target, stride, offset) = match tick {
        72 => (Gpu, 2, 0),
        128 => (Gpu, 2, 1),
        225 => (Cpu, 4, 0),
        365 => (Cpu, 1, 0),
        465 => (Gpu, 3, 1),
        550 => (Cpu, 1, 0),
        _ => return None,
    };
    Some((
        target,
        initial
            .iter()
            .skip(offset)
            .step_by(stride)
            .map(|e| e.id)
            .collect(),
    ))
}
fn run(
    adapter: &str,
    mut csv: Option<&mut File>,
    repeat: Option<&[u64]>,
) -> Result<Vec<u64>, Box<dyn Error>> {
    use qualification as q;
    let mut analytical = q::fixture();
    let initial = analytical.clone();
    let all: Vec<_> = initial.iter().map(|e| e.id).collect();
    let mut ordered = q::registered(&initial);
    if repeat.is_some() {
        ordered.reverse();
    }
    let b = MixedBatch::new(adapter, ordered)?.with_initial_cpu(&all)?;
    let mut sim = Simulation::new(STEP)?;
    install_mixed(&mut sim, b)?;
    let mut dense = q::Dense::default();
    let mut native = repeat.is_none().then(|| q::Native::new(&initial));
    let mut trace = Vec::with_capacity(q::TICKS);
    for tick in 0..q::TICKS {
        let mut transition = std::time::Duration::ZERO;
        if let Some((target, ids)) = switches(tick, &initial) {
            let at = batch_boundary(&sim);
            let before = q::signature(sim.world().resource::<MixedBatch>().environments());
            let start = Instant::now();
            let changed = set_environment_placement(&mut sim, at, &ids, target)?;
            transition = start.elapsed();
            let snapshot = sim.world().resource::<MixedBatch>();
            assert_eq!(before, q::signature(snapshot.environments()));
            assert_eq!(at, batch_boundary(&sim));
            for id in &ids {
                assert_eq!(snapshot.execution_placement(*id)?, target);
            }
            if repeat.is_none() {
                println!(
                    "boundary={tick} target={target:?} moved={} relocated_gpu={} switch_us={:.2}",
                    changed.moved,
                    changed.reseeded_gpu_rows,
                    transition.as_secs_f64() * 1e6
                );
            }
        }
        let start = Instant::now();
        sim.step_exact(1)?;
        let wall = start.elapsed();
        let world = sim.world();
        let batch = world.resource::<MixedBatch>();
        let states = batch.environments();
        let metrics = world.resource::<BatchTiming>().last.as_ref().unwrap();
        if batch.cpu_environments() == q::N {
            assert_eq!((metrics.uploaded_bytes, metrics.readback_bytes), (0, 0));
            assert!(batch.groups().is_empty());
        } else {
            assert_eq!(
                metrics.readback_bytes,
                batch.active_gpu_bytes() + 32 * u64::from(batch.device_info().timestamp_queries)
            );
            assert_eq!(
                batch.groups().iter().map(|g| g.environments).sum::<usize>()
                    + batch.cpu_environments(),
                q::N
            );
        }
        let signature = q::signature(states);
        trace.push(signature);
        if let Some(expected) = repeat {
            assert_eq!(signature, expected[tick], "repetition at tick {tick}");
        } else {
            for (a, b) in analytical.iter_mut().zip(states) {
                q::cpu_step(a);
                dense.add(b, a);
            }
            native.as_mut().unwrap().step(tick + 1, states);
        }
        if let Some(out) = csv.as_mut() {
            writeln!(
                out,
                "{},{},{},{},{},{},{},{},{}",
                tick + 1,
                batch.cpu_environments(),
                q::N - batch.cpu_environments(),
                transition.as_nanos(),
                metrics.uploaded_bytes,
                metrics.readback_bytes,
                metrics.cpu_solve.as_nanos(),
                metrics.gpu_dispatch.map_or(0, |t| t.as_nanos()),
                wall.as_nanos()
            )?;
        }
    }
    if let Some(native) = native {
        dense.verify();
        native.verify();
    }
    Ok(trace)
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("supply a physical adapter")?;
    let path = args.next().ok_or("supply an unused CSV path")?;
    if args.next().is_some() {
        return Err("unexpected argument".into());
    }
    let mut out = File::create_new(path)?;
    writeln!(
        out,
        "tick,cpu_environments,gpu_environments,switch_ns,uploaded_bytes,readback_bytes,scalar_solve_ns,gpu_dispatch_ns,total_step_ns"
    )?;
    let trace = run(&adapter, Some(&mut out), None)?;
    let repeated = run(&adapter, None, Some(&trace))?;
    assert_eq!(trace, repeated);
    println!(
        "600 repeated CPU/GPU frames match after reversing admission order; final signature {:016x}",
        trace.last().unwrap()
    );
    Ok(())
}
