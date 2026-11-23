// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the cumulative cost of switching with keeping the current owner.
//! Every treatment starts at the same committed scene state after an independent reset.
#[path = "support/gpu_sdf_fixture.rs"]
mod generated;
use gz_physics_gpu::{
    Environment,
    ExecutionPlacement::{self, Cpu, Gpu},
    MixedBatch,
    simulation::{BatchTiming, batch_boundary},
};
use gz_sim_core::Simulation;
use gz_sim_scene::{
    PhysicsEngine, WorldLoadOptions,
    gpu_scene::{GpuSceneObservations, set_environment_placement},
    load_world_on_adapter, reset_loaded_world,
};
use sdformat::NoResources;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    time::Instant,
};
const WARM: u64 = 75;
const STEPS: u64 = 300;
type Result<T> = std::result::Result<T, Box<dyn Error>>;

struct Sample {
    boundary: u128,
    wall: u128,
    upload: u64,
    read: u64,
}
struct Trial {
    samples: Vec<Sample>,
    states: Vec<Environment>,
}

fn trial(
    sim: &mut Simulation,
    ids: &[u64],
    source: ExecutionPlacement,
    target: ExecutionPlacement,
) -> Result<Trial> {
    reset_loaded_world(sim, |_| ())?;
    let at = batch_boundary(sim);
    set_environment_placement(sim, at, ids, source)?;
    sim.step_exact(WARM)?;
    let mut samples = Vec::with_capacity(STEPS as usize);
    // The destination cost is charged once at the barrier. Remaining on the same
    // owner needs no call; a hypothetical automatic decision would add its own cost.
    let start = Instant::now();
    let at = batch_boundary(sim);
    if source != target {
        assert_eq!(
            set_environment_placement(sim, at, ids, target)?.moved,
            ids.len()
        );
    }
    let switch = if source == target {
        0
    } else {
        start.elapsed().as_nanos()
    };
    for n in 1..=STEPS {
        let start = Instant::now();
        sim.step_exact(1)?;
        let wall = start.elapsed().as_nanos();
        let timing = sim.world().resource::<BatchTiming>().last.unwrap();
        assert_eq!(
            sim.world().resource::<GpuSceneObservations>().tick(),
            (WARM + n) / 10 * 10
        );
        if target == Cpu {
            assert_eq!((timing.uploaded_bytes, timing.readback_bytes), (0, 0));
        }
        if target == Gpu && source == Cpu && n == 1 {
            assert!(timing.uploaded_bytes > 0);
        }
        samples.push(Sample {
            boundary: if n == 1 { switch } else { 0 },
            wall,
            upload: timing.uploaded_bytes,
            read: timing.readback_bytes,
        });
    }
    Ok(Trial {
        samples,
        states: sim.world().resource::<MixedBatch>().environments().to_vec(),
    })
}

fn validate(a: &[Environment], b: &[Environment]) -> Result<f64> {
    let mut peak = 0_f64;
    for (a, b) in a.iter().zip(b) {
        assert_eq!((a.id, a.state.profile()), (b.id, b.state.profile()));
        let av: &[f64] = a.state.values();
        let bv: &[f64] = b.state.values();
        for (&x, &y) in av.iter().zip(bv) {
            let error = (x - y).abs();
            if !error.is_finite() || error > 2e-10 {
                return Err(format!("environment {} committed final values {x}, {y}", a.id).into());
            }
            peak = peak.max(error);
        }
    }
    Ok(peak)
}
fn main() -> Result<()> {
    let mut a = std::env::args().skip(1);
    let adapter = a.next().ok_or("provide physical adapter and output CSV")?;
    let path = a.next().ok_or("provide a new CSV file")?;
    if a.next().is_some() {
        return Err("expected ADAPTER OUTPUT.csv".into());
    }
    let mut out = BufWriter::new(File::create_new(path)?);
    writeln!(
        out,
        "count,repeat,source,target,tick,owner_change_ns,wall_ns,upload_B,read_B"
    )?;
    for count in [128, 256, 512, 768, 1024, 1280, 1536, 1792, 2048] {
        let xml = generated::fixture(count, 10);
        let mut sim = load_world_on_adapter(
            &xml,
            "placement.sdf",
            "default",
            &NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::GpuBatch,
                step: None,
                noise_seed: [0; 32],
                distributed_worker: None,
            },
            Some(&adapter),
        )?;
        let ids: Vec<_> = (0..count as u64).collect();
        for repeat in 0..3 {
            for source in if repeat == 1 { [Gpu, Cpu] } else { [Cpu, Gpu] } {
                let destination = if source == Cpu { Gpu } else { Cpu };
                let mut stay = None;
                let mut migrate = None;
                for target in if repeat % 2 == 0 {
                    [source, destination]
                } else {
                    [destination, source]
                } {
                    let t = trial(&mut sim, &ids, source, target)?;
                    for (i, s) in t.samples.iter().enumerate() {
                        writeln!(
                            out,
                            "{count},{repeat},{source:?},{target:?},{},{},{},{},{}",
                            i + 1,
                            s.boundary,
                            s.wall,
                            s.upload,
                            s.read
                        )?;
                    }
                    if target == source {
                        stay = Some(t);
                    } else {
                        migrate = Some(t);
                    }
                }
                let stay = stay.unwrap();
                let migrate = migrate.unwrap();
                let error = validate(&stay.states, &migrate.states)?;
                let mut remaining = 0_i128;
                let mut last_not_cheaper = 0;
                for (i, (s, m)) in stay.samples.iter().zip(&migrate.samples).enumerate() {
                    remaining += (m.boundary + m.wall) as i128 - s.wall as i128;
                    if remaining >= 0 {
                        last_not_cheaper = i + 1;
                    }
                }
                println!(
                    "environments={count} repeat={repeat} source={source:?} target={destination:?} first_sustained_saving_tick={:?} hold_300_delta_ms={:.3} peak_final_f64_field_error={error:.3e}",
                    (last_not_cheaper < STEPS as usize).then_some(last_not_cheaper + 1),
                    remaining as f64 * 1e-6
                );
            }
        }
        out.flush()?;
    }
    Ok(())
}
