// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Generate and qualify a population of independently supported two-box stacks.
#[path = "support/grid.rs"]
mod grid;
#[path = "support/stack_order.rs"]
mod stack_order;
use clap::Parser;
#[path = "support/grid_control.rs"]
mod grid_control;
use grid_control::{Control, Pairs, Supports};
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{SceneBackend, coordinator::Frame};
use gz_sim_core::Simulation;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured, reset_loaded_world};
use sdformat::NoResources;
use std::{error::Error, path::PathBuf, time::Instant};
#[derive(Parser)]
struct Args {
    #[arg(long, default_value_t = 256)]
    bodies: usize,
    #[arg(long, default_value_t = 240)]
    ticks: usize,
    /// Isolate native contact ordering by adding distant loaded or airborne boxes.
    #[arg(long)]
    probe_native_order: bool,
    /// Request the historical full JSON worker observations for a matched timing control.
    #[arg(long)]
    json_observations: bool,
    /// Write the SDF used by the numerical qualification or ordinary viewer.
    #[arg(long)]
    write_sdf: Option<PathBuf>,
    /// Omit for SDF generation alone.
    #[arg(long)]
    worker: Option<PathBuf>,
    /// Require exact agreement with an additional native world containing every stack.
    #[arg(long)]
    require_monolith: bool,
    /// Write frame-by-frame errors against the full native world.
    #[arg(long)]
    trace: Option<PathBuf>,
}
fn frame(sim: &Simulation) -> &Frame {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
        .frame()
        .unwrap()
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    if args.probe_native_order {
        return stack_order::run(args.ticks);
    }
    let sdf = grid::sdf(args.bodies);
    if let Some(file) = args.write_sdf {
        std::fs::write(file, &sdf)?;
    }
    let Some(worker) = args.worker else {
        return Ok(());
    };
    assert!(args.ticks > 0);
    let start = Instant::now();
    let mut sim = load_world_configured(
        &sdf,
        "distributed_box_grid.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(&worker),
        },
    )?;
    let load_ms = start.elapsed().as_secs_f64() * 1000.;
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<SceneBackend>()
        .unwrap()
        .set_packed_observations(!args.json_observations);
    let traffic = |sim: &Simulation| {
        sim.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .unwrap()
            .worker_traffic()
            .unwrap()
    };
    let initial_traffic = traffic(&sim);
    let seed = frame(&sim).clone();
    let h = sim.step_duration();
    let bodies = (0..args.bodies as u64)
        .map(|id| seed.body(id).unwrap().1.clone())
        .collect::<Vec<_>>();
    let mut native = Control::new(&bodies, h)?;
    // Every two-box cell is separated from other cells by 8 m and has its own
    // uninterrupted native oracle. A second, monolithic oracle also includes
    // all remote contacts in Rapier's global constraint ordering.
    let mut cells = bodies
        .chunks(2)
        .map(|pair| Control::new(pair, h))
        .collect::<Result<Vec<_>, _>>()?;
    let mut times = Vec::new();
    let mut native_times = Vec::new();
    let mut total_ground = 0;
    let mut total_pairs = 0;
    let mut peaks = [0f64; 6];
    let mut exact_frames = 0;
    let mut first = None;
    let mut trace = args
        .trace
        .map(std::fs::File::create)
        .transpose()?
        .map(std::io::BufWriter::new);
    use std::io::Write;
    if let Some(out) = &mut trace {
        writeln!(
            out,
            "tick,exact_bodies,total_bodies,position_m,velocity_m_s,angle_rad,spin_rad_s,ground_impulse_Ns,pair_impulse_Ns,pair_sets_equal"
        )?;
    }
    for tick in 1..=args.ticks {
        let clock = Instant::now();
        sim.step_exact(1)?;
        times.push(clock.elapsed().as_nanos() as f64 / 1000.);
        let clock = Instant::now();
        native.step(h)?;
        native_times.push(clock.elapsed().as_nanos() as f64 / 1000.);
        for c in &mut cells {
            c.step(h)?;
        }
        let f = frame(&sim);
        let mut errors = [0f64; 6];
        let mut exact_bodies = 0;
        for b in f.workers.iter().flat_map(|w| &w.snapshot.bodies) {
            let i = b.id as usize;
            assert!(
                grid_control::exact(b, cells[i / 2].state(i % 2)),
                "independent native stack {i} at {tick}"
            );
            let other = native.state(i);
            exact_bodies += usize::from(grid_control::exact(b, other));
            for (error, diff) in errors.iter_mut().zip(grid_control::state_error(b, other)) {
                *error = error.max(diff);
            }
        }
        let actual: Supports = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.supports)
            .map(|p| (p.body, p.normal_impulse))
            .collect();
        let pairs: Pairs = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .map(|p| ((p.a, p.b), p.normal_impulse))
            .collect();
        let (mut local_pairs, mut local_ground) = (Pairs::new(), Supports::new());
        for c in &cells {
            c.contacts(&mut local_pairs, &mut local_ground);
        }
        assert!(
            grid_control::impulses(&actual, &local_ground).0
                && grid_control::impulses(&pairs, &local_pairs).0,
            "native cell contacts at {tick}"
        );
        let (mut mono_pairs, mut mono_ground) = (Pairs::new(), Supports::new());
        native.contacts(&mut mono_pairs, &mut mono_ground);
        let (support_exact, support_error) = grid_control::impulses(&actual, &mono_ground);
        let (pair_exact, pair_error) = grid_control::impulses(&pairs, &mono_pairs);
        errors[4] = support_error;
        errors[5] = pair_error;
        let exact = exact_bodies == args.bodies && support_exact && pair_exact;
        if exact {
            exact_frames += 1;
        } else {
            first.get_or_insert(tick);
        }
        if args.require_monolith {
            assert!(exact, "native global mismatch at {tick}, {errors:?}");
        }
        if let Some(out) = &mut trace {
            writeln!(
                out,
                "{tick},{exact_bodies},{},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{}",
                args.bodies,
                errors[0],
                errors[1],
                errors[2],
                errors[3],
                errors[4],
                errors[5],
                actual.keys().eq(mono_ground.keys()) && pairs.keys().eq(mono_pairs.keys())
            )?;
        }
        for (peak, error) in peaks.iter_mut().zip(errors) {
            *peak = peak.max(error);
        }
        total_pairs += pairs.len();
        total_ground += actual.len();
    }
    println!(
        "monolithic comparison: {exact_frames}/{} exact frames, first divergence {first:?}, peak position m / speed m/s / angle rad / spin rad/s / ground Ns / pair Ns {peaks:?}",
        args.ticks
    );
    let f = frame(&sim);
    let final_contacts = f
        .workers
        .each_ref()
        .map(|w| w.snapshot.supports.len() + w.snapshot.impulses.len());
    let q = |v: &mut [f64]| {
        v.sort_by(f64::total_cmp);
        [v[v.len() / 2], v[(v.len() * 99 / 100).min(v.len() - 1)]]
    };
    let rtf = h.as_secs_f64() * args.ticks as f64 * 1e6 / times.iter().sum::<f64>();
    println!(
        "{} boxes, {} x {} ms, owner counts {:?}, final contacts {:?}, {} floor + {} box contact reports, every observation bit-exact against uninterrupted isolated stack solvers; load {:.2} ms; normal scene p50/p99 {:?} µs; native-only p50/p99 {:?} µs; distributed {:.3}× real time",
        args.bodies,
        args.ticks,
        h.as_millis(),
        f.workers.each_ref().map(|w| w.snapshot.bodies.len()),
        final_contacts,
        total_ground,
        total_pairs,
        load_ms,
        q(&mut times),
        q(&mut native_times),
        rtf
    );
    let final_traffic = traffic(&sim);
    let request_bytes: u64 = (0..2)
        .map(|i| final_traffic[i].request_bytes - initial_traffic[i].request_bytes)
        .sum();
    let reply_bytes: u64 = (0..2)
        .map(|i| final_traffic[i].reply_bytes - initial_traffic[i].reply_bytes)
        .sum();
    println!(
        "worker payloads: {} requests and {} replies (bytes), reply format {}",
        request_bytes,
        reply_bytes,
        if args.json_observations {
            "json"
        } else {
            "packed"
        }
    );
    reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(frame(&sim), &seed);
    Ok(())
}
