// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare ordinary SDF worker barriers against one continuous native world at equal simulated times.
use clap::Parser;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{
    SceneBackend,
    coordinator::Frame,
    physics::{Island, SimulationStep, Snapshot},
};
use gz_sim_core::Simulation;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured, reset_loaded_world};
use sdformat::NoResources;
use serde::Serialize;
use std::{
    error::Error,
    path::PathBuf,
    time::{Duration, Instant},
};

#[derive(Parser)]
struct Args {
    /// Trusted physics_worker executable built alongside this example.
    #[arg(long)]
    worker: PathBuf,
}
#[derive(Serialize)]
struct ResultRow {
    step_ms: u128,
    ticks: u64,
    first_common_owner_ms: u128,
    first_positive_impulse_ms: u128,
    cargo_x_m: f64,
    cargo_vx_m_s: f64,
    distributed_elapsed_ms: f64,
    native_elapsed_ms: f64,
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
    let xml = include_str!("../../../examples/distributed_boxes.sdf");
    for profile in [
        SimulationStep::Ms1,
        SimulationStep::Ms2,
        SimulationStep::Ms5,
        SimulationStep::Ms10,
    ] {
        let h = profile.duration();
        let mut sim = load_world_configured(
            xml,
            "distributed_boxes.sdf",
            "default",
            &NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::Distributed,
                step: Some(h),
                noise_seed: [0; 32],
                distributed_worker: Some(&args.worker),
            },
        )?;
        let seed = frame(&sim).clone();
        assert_ne!(seed.body(0).unwrap().0, seed.body(1).unwrap().0);
        let bodies = (0..2).map(|id| seed.body(id).unwrap().1.clone()).collect();
        let mut native = Island::restore(&Snapshot {
            step: profile,
            bodies,
            ..Default::default()
        })?;
        let ticks = (1320 / h.as_millis()) as u64;
        let mut route = None;
        let mut hit = None;
        let mut distributed_wall = Duration::ZERO;
        let mut native_wall = Duration::ZERO;
        for tick in 1..=ticks {
            let start = Instant::now();
            sim.step_exact(1)?;
            distributed_wall += start.elapsed();
            let start = Instant::now();
            native.step()?;
            native_wall += start.elapsed();
            let world = frame(&sim);
            assert_eq!(world.step(), profile);
            assert_eq!(world.tick, tick);
            let check = native.snapshot();
            for body in &check.bodies {
                assert_eq!(world.body(body.id).unwrap().1, body);
            }
            let actual: Vec<_> = world
                .workers
                .iter()
                .flat_map(|w| w.snapshot.impulses.iter())
                .collect();
            assert!(actual.iter().copied().eq(check.impulses.iter()));
            if world.body(0).unwrap().0 == world.body(1).unwrap().0 {
                route.get_or_insert(tick);
            }
            if actual.iter().any(|c| c.normal_impulse > 0.) {
                hit.get_or_insert(tick);
            }
        }
        assert!(route.unwrap() < hit.unwrap());
        assert_eq!(sim.elapsed(), Duration::from_millis(1320));
        let cargo = frame(&sim).body(1).unwrap().1;
        assert!((cargo.position[0] - 0.78).abs() < 1e-7);
        assert!((cargo.linear[0] - 1.5).abs() < 1e-7);
        let row = ResultRow {
            step_ms: h.as_millis(),
            ticks,
            first_common_owner_ms: u128::from(route.unwrap()) * h.as_millis(),
            first_positive_impulse_ms: u128::from(hit.unwrap()) * h.as_millis(),
            cargo_x_m: cargo.position[0],
            cargo_vx_m_s: cargo.linear[0],
            distributed_elapsed_ms: distributed_wall.as_secs_f64() * 1000.,
            native_elapsed_ms: native_wall.as_secs_f64() * 1000.,
        };
        reset_loaded_world(&mut sim, |_| ())?;
        assert_eq!(frame(&sim), &seed);
        assert_eq!(sim.step_duration(), h);
        println!("{}", serde_json::to_string(&row)?);
    }
    Ok(())
}
