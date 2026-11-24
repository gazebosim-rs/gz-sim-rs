// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! End-to-end stationary and changing-load qualification for measured SDF placement.
#[path = "support/gpu_sdf_fixture.rs"]
mod generated;
use bevy_ecs::prelude::*;
use gz_physics_gpu::{
    Environment,
    ExecutionPlacement::{self, Cpu, Gpu},
    MixedBatch,
    simulation::BatchTiming,
};
use gz_sim_core::{Phase, Simulation};
use gz_sim_scene::{
    PhysicsEngine, Velocity, WorldLoadOptions, WorldPose,
    gpu_scene::{
        automatic::{AutomaticPlacement, PlacementEvent, PlacementReason, set_placement_workload},
        batch_boundary,
    },
    load_world_on_adapter,
};
use sdformat::NoResources;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::Path,
    time::Instant,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

const TAG: &str = "<automatic_placement window_ticks='24' min_residency_ticks='120' reprobe_ticks='1600' hysteresis_percent='6'/>";
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Load {
    Stationary,
    Churning,
    Phases,
    DeclaredAcquisition,
    UnannouncedAcquisition,
}
#[derive(Resource, Default)]
struct Collector {
    enabled: bool,
    stations: bool,
    moments: [f64; 6],
    samples: u64,
}
/// A caller's ordinary ECS output calculation under changing interest. This
/// performs a real reduction over links; it does not feed back into the solver.
fn collect(q: Query<(&WorldPose, &Velocity)>, mut c: ResMut<Collector>) {
    if !c.enabled {
        return;
    }
    let mut sum = [0.; 6];
    for (p, v) in &q {
        for (i, x) in p.translation.into_iter().chain(v.linear).enumerate() {
            sum[i] += x * x;
        }
        if c.stations {
            // Eight independent spherical-coordinate observations provide real
            // host-side acquisition work with a short, changing demand interval.
            let [x, y, z] = p.translation;
            for station in 0..8 {
                let dx = x - 3. * (station % 4) as f64 - 1.;
                let dy = y - 4. * (station / 4) as f64 + 0.7;
                let dz = z - 1.5;
                let horizontal = dx.hypot(dy);
                let range = horizontal.hypot(dz).max(f64::MIN_POSITIVE);
                sum[0] += range;
                sum[1] += dy.atan2(dx);
                sum[2] += dz.atan2(horizontal);
                sum[3] += (dx * v.linear[0] + dy * v.linear[1] + dz * v.linear[2]) / range;
            }
        }
    }
    c.moments = sum;
    c.samples += 1;
}
fn load_world(
    adapter: &str,
    n: usize,
    start: ExecutionPlacement,
    autoplace: bool,
) -> Result<Simulation> {
    let mut xml = generated::fixture(n, 10);
    if start == Cpu {
        xml = xml.replace(" profile=", " execution='cpu' profile=");
    }
    if autoplace {
        xml = xml.replace("<observe ", &format!("{TAG}<observe "));
    }
    let mut sim = load_world_on_adapter(
        &xml,
        "automatic.sdf",
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
    sim.world_mut().insert_resource(Collector::default());
    sim.add_systems(Phase::PostStep, collect);
    Ok(sim)
}
struct Trial {
    total_ns: u128,
    p99_ns: u128,
    states: Vec<Environment>,
    events: Vec<PlacementEvent>,
    observer: [f64; 6],
}
struct Writers {
    steps: BufWriter<File>,
    events: BufWriter<File>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Treatment {
    Automatic,
    FixedCpu,
    FixedGpu,
}
#[derive(Clone, Copy)]
struct Case {
    n: usize,
    start: ExecutionPlacement,
    load: Load,
    rep: u8,
    steps: u64,
}
fn trial(adapter: &str, case: Case, treatment: Treatment, out: &mut Writers) -> Result<Trial> {
    let Case {
        n,
        start,
        load,
        rep,
        steps,
    } = case;
    let automatic = treatment == Treatment::Automatic;
    let actual = match treatment {
        Treatment::Automatic => start,
        Treatment::FixedCpu => Cpu,
        Treatment::FixedGpu => Gpu,
    };
    let mut sim = load_world(adapter, n, actual, automatic)?;
    // Paused ticks prime Bevy schedules. All advancing work is measured, including
    // probes, initial upload, reversion to the baseline and the final migration.
    sim.tick()?;
    sim.world_mut().resource_mut::<Collector>().stations = matches!(
        load,
        Load::DeclaredAcquisition | Load::UnannouncedAcquisition
    );
    let mut ns = vec![];
    let mut total_ns = 0;
    let mut events = vec![];
    let mut key = 0;
    for tick in 1..=steps {
        let next = match load {
            Load::Stationary => 0,
            Load::DeclaredAcquisition | Load::UnannouncedAcquisition => (tick - 1) / 4 % 2,
            Load::Churning => (tick - 1) / 8 % 2,
            Load::Phases => (tick - 1) / 275 % 2,
        };
        let start_wall = Instant::now();
        if next != key {
            sim.world_mut().resource_mut::<Collector>().enabled = next == 1;
            if automatic && load != Load::UnannouncedAcquisition {
                let at = batch_boundary(&sim);
                set_placement_workload(&mut sim, at, next)?;
            }
            key = next;
        }
        sim.step_exact(1)?;
        let wall = start_wall.elapsed().as_nanos();
        ns.push(wall);
        total_ns += wall;
        let b = sim.world().resource::<MixedBatch>();
        let d = sim.world().resource::<BatchTiming>().last.unwrap();
        let source = if d.readback_bytes == 0 { Cpu } else { Gpu };
        if automatic {
            let a = sim.world().resource::<AutomaticPlacement>();
            if let Some(e) = a.events().back().filter(|e| e.at.tick == tick) {
                writeln!(
                    out.events,
                    "{n},{rep},{load:?},{tick},{},{:?},{:?},{},{},{}",
                    e.workload,
                    e.reason,
                    e.owner,
                    e.evidence.conservative_saving_ns,
                    e.evidence.probe_and_switch_cost_ns,
                    e.evidence.break_even_ticks.unwrap_or(0)
                )?;
                events.push(*e);
            }
        }
        writeln!(
            out.steps,
            "{n},{rep},{load:?},{treatment:?},{tick},{key},{source:?},{},{},{},{}",
            b.cpu_environments(),
            d.uploaded_bytes,
            d.readback_bytes,
            wall
        )?;
    }
    let c = sim.world().resource::<Collector>();
    if load != Load::Stationary {
        assert!(c.samples > 0 && c.moments.iter().any(|&v| v > 0.));
    }
    if matches!(load, Load::Churning | Load::DeclaredAcquisition) && automatic {
        assert_eq!(sim.world().resource::<AutomaticPlacement>().probes(), 0);
    }
    ns.sort_unstable();
    Ok(Trial {
        total_ns,
        p99_ns: ns[ns.len() * 99 / 100 - 1],
        states: sim.world().resource::<MixedBatch>().environments().to_vec(),
        events,
        observer: c.moments,
    })
}
fn check(a: &Trial, b: &Trial) -> Result<f64> {
    let mut max = 0_f64;
    for (a, b) in a.states.iter().zip(&b.states) {
        assert_eq!(a.id, b.id);
        for (&x, &y) in a.state.values().iter().zip(b.state.values()) {
            let delta = (x - y).abs();
            if !delta.is_finite() || delta > 5e-10 {
                return Err(format!("physical mismatch in ID {}: {x} vs {y}", a.id).into());
            }
            max = max.max(delta);
        }
    }
    for (x, y) in a.observer.into_iter().zip(b.observer) {
        assert!((x - y).abs() <= 2e-11 * (1. + x.abs()));
    }
    Ok(max)
}
fn main() -> Result<()> {
    let mut a = std::env::args().skip(1);
    let adapter = a.next().ok_or("provide ADAPTER NEW_OUTPUT_DIRECTORY")?;
    let dir = a.next().ok_or("provide new output directory")?;
    let mode = a.next();
    if !matches!(mode.as_deref(), None | Some("--long") | Some("--bursts")) {
        return Err("expected optional --long or --bursts".into());
    }
    if a.next().is_some() {
        return Err("unexpected argument".into());
    }
    std::fs::create_dir(&dir)?;
    let path = Path::new(&dir);
    let mut out = Writers {
        steps: BufWriter::new(File::create_new(path.join("steps.csv"))?),
        events: BufWriter::new(File::create_new(path.join("events.csv"))?),
    };
    writeln!(
        out.steps,
        "count,repeat,load,treatment,tick,workload,preceding_owner,cpu_after_barrier,upload_B,read_B,whole_step_ns"
    )?;
    writeln!(
        out.events,
        "count,repeat,load,tick,workload,reason,owner,saving_ns,probe_and_transfer_ns,break_even_ticks"
    )?;
    for rep in 0..3 {
        let fast = [
            (128, Gpu, Load::Stationary),
            (2048, Cpu, Load::Stationary),
            (3072, Cpu, Load::Stationary),
            (128, Cpu, Load::Churning),
            (2048, Cpu, Load::Churning),
            (2048, Cpu, Load::Phases),
        ];
        let prolonged = [
            (128, Gpu, Load::Stationary),
            (3072, Cpu, Load::Stationary),
            (3072, Cpu, Load::Churning),
        ];
        let bursts = [
            (128, Gpu, Load::DeclaredAcquisition),
            (128, Gpu, Load::UnannouncedAcquisition),
            (2048, Cpu, Load::DeclaredAcquisition),
            (2048, Cpu, Load::UnannouncedAcquisition),
        ];
        let (cases, steps): (&[_], u64) = match mode.as_deref() {
            Some("--bursts") => (&bursts, 480),
            Some("--long") => (&prolonged, 2400),
            _ => (&fast, 800),
        };
        for &(n, start, load) in cases {
            let mut tries = vec![];
            let order = match rep {
                0 => [
                    Treatment::Automatic,
                    Treatment::FixedCpu,
                    Treatment::FixedGpu,
                ],
                1 => [
                    Treatment::FixedGpu,
                    Treatment::Automatic,
                    Treatment::FixedCpu,
                ],
                _ => [
                    Treatment::FixedCpu,
                    Treatment::FixedGpu,
                    Treatment::Automatic,
                ],
            };
            for t in order {
                tries.push((
                    t,
                    trial(
                        &adapter,
                        Case {
                            n,
                            start,
                            load,
                            rep,
                            steps,
                        },
                        t,
                        &mut out,
                    )?,
                ));
            }
            let get = |t| &tries.iter().find(|(k, _)| *k == t).unwrap().1;
            let auto = get(Treatment::Automatic);
            let cpu = get(Treatment::FixedCpu);
            let gpu = get(Treatment::FixedGpu);
            let error = check(cpu, auto)?.max(check(gpu, auto)?);
            println!(
                "count={n} repeat={rep} load={load:?} initial={start:?} cpu_ms={:.3} gpu_ms={:.3} auto_ms={:.3} p99_us={:.1}/{:.1}/{:.1} probes={} commits={} final_field_error={error:.3e}",
                cpu.total_ns as f64 * 1e-6,
                gpu.total_ns as f64 * 1e-6,
                auto.total_ns as f64 * 1e-6,
                cpu.p99_ns as f64 * 1e-3,
                gpu.p99_ns as f64 * 1e-3,
                auto.p99_ns as f64 * 1e-3,
                auto.events
                    .iter()
                    .filter(|e| e.reason == PlacementReason::Probe)
                    .count(),
                auto.events
                    .iter()
                    .filter(|e| e.reason == PlacementReason::Commit)
                    .count()
            );
        }
        out.steps.flush()?;
        out.events.flush()?;
    }
    Ok(())
}
